#!/usr/bin/env python3
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
import math

class KinematicAEBFilter(Node):
    def __init__(self):
        super().__init__('kinematic_aeb_filter')
        
        # --- SOFTWARE LIMITER ---
        self.software_speed_limit = 0.10
        
        # --- KINEMATICS (Tuned for High Inertia / Slow ESC) ---
        self.max_deceleration = 0.8       # Triggers earlier because the car takes longer to stop
        self.system_latency = 0.3  
        
        self.base_safety_margin = 0.33 
        self.base_corridor_width = 0.3    
        self.base_ttc_threshold = 3.5  
        self.strict_lockout_dist = 0.3    # Extended to 40cm to account for massive braking distance
        
        # --- STATE MACHINE ---
        self.brake_state = 'IDLE'    
        self.brake_start_time = 0.0
        self.active_brake_cmd = 0.0  
        self.dynamic_pulse_duration = 0.11 # Default massive pulse
        
        self.last_cmd_vel_forward = 0.0  
        self.last_eval_time = None
        self.free_drive_duration = 0.0    
        
        self.latest_scan = None
        self.aeb_enabled = True
        self.filtered_accel_x = 0.0  
        
        teleop_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.cmd_raw_sub = self.create_subscription(TwistStamped, '/cmd_vel_raw', self.cmd_raw_callback, teleop_qos)
        self.state_sub = self.create_subscription(Bool, '/aeb/state', self.state_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data_raw', self.imu_callback, qos_profile_sensor_data)
            
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel_stamped', 10)
        self.req_decel_pub = self.create_publisher(Float32, '/required_deceleration', 10)
        self.ttc_pub = self.create_publisher(Float32, '/ttc', 10) 
        
        self.get_logger().info("AEB Online: Uninterruptible 3-Second Juggernaut Pulse Active.")

    def scan_callback(self, msg):
        self.latest_scan = msg

    def state_callback(self, msg):
        self.aeb_enabled = msg.data

    def imu_callback(self, msg):
        alpha = 0.15 
        self.filtered_accel_x = (alpha * msg.linear_acceleration.x) + ((1.0 - alpha) * self.filtered_accel_x)

    def calculate_forward_distance(self, dynamic_corridor_width):
        if not self.latest_scan:
            return float('inf')
            
        min_dist = float('inf')
        for i, r in enumerate(self.latest_scan.ranges):
            if math.isinf(r) or math.isnan(r) or r < 0.15 or r > 12.0:
                continue
                
            angle = self.latest_scan.angle_min + (i * self.latest_scan.angle_increment)
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            
            if x > 0.0 and abs(y) <= dynamic_corridor_width / 2.0:
                if r < min_dist:
                    min_dist = r
        return min_dist

    def cmd_raw_callback(self, msg):
        current_time_sec = self.get_clock().now().nanoseconds / 1e9
        
        if self.last_eval_time is None:
            dt = 0.0
        else:
            dt = current_time_sec - self.last_eval_time
        self.last_eval_time = current_time_sec
        
        filtered_msg = TwistStamped()
        filtered_msg.header = msg.header
        filtered_msg.header.stamp = self.get_clock().now().to_msg()
        filtered_msg.twist.angular = msg.twist.angular
        
        msg_time = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)
        raw_throttle = msg.twist.linear.x

        if abs(current_time_sec - msg_time) > 0.3:
            raw_throttle = 0.0

        is_user_braking = (raw_throttle < 0.0)
        is_user_coasting = (raw_throttle == 0.0)

        # 1. Bypasses: If AEB is off, or user is manually reversing
        if not self.aeb_enabled or is_user_braking:
            self.brake_state = 'IDLE'
            self.free_drive_duration += dt
            filtered_msg.twist.linear.x = raw_throttle
            self.cmd_pub.publish(filtered_msg)
            self.last_cmd_vel_forward = raw_throttle
            return

        # 2. Driver Panic Abort Fix: If user lets go, but we are actively emergency braking, KEEP BRAKING.
        # If user lets go and we are  braking, just coast.
        if is_user_coasting and self.brake_state == 'BRAKING':
            self.brake_state = 'IDLE'
            self.free_drive_duration += dt
            filtered_msg.twist.linear.x = 0.0
            self.cmd_pub.publish(filtered_msg)
            self.last_cmd_vel_forward = 0.0
            return

        # Apply Expo curve only if user is commanding forward
        if raw_throttle > 0.0:
            joy_norm = min(1.0, raw_throttle / 0.5)
            cmd_vel_forward = self.software_speed_limit * (joy_norm ** 2)
        else:
            cmd_vel_forward = 0.0 # Handled by the uninterruptible pulse below

        speed_factor = min(1.0, cmd_vel_forward / self.software_speed_limit) if cmd_vel_forward > 0 else 1.0
        
        dynamic_corridor = self.base_corridor_width - (0.1 * speed_factor)
        front_distance = self.calculate_forward_distance(dynamic_corridor)

        dynamic_margin = self.base_safety_margin + (0.3 * speed_factor)
        dynamic_ttc = self.base_ttc_threshold + (0.4 * speed_factor)

        ttc_value = front_distance / cmd_vel_forward if (cmd_vel_forward > 0 and front_distance != float('inf')) else float('inf')
        self.ttc_pub.publish(Float32(data=float(ttc_value)))

        accel_buffer = max(0.0, self.filtered_accel_x * 0.15) 
        total_margin = dynamic_margin + accel_buffer

        latency_distance = cmd_vel_forward * self.system_latency
        available_distance = max(0.0, front_distance - total_margin - latency_distance)
        
        req_decel = (cmd_vel_forward ** 2) / (2.0 * available_distance) if available_distance > 0 else float('inf')
        self.req_decel_pub.publish(Float32(data=float(req_decel)))

        trigger_lockout = (front_distance <= self.strict_lockout_dist)
        trigger_aeb = (req_decel > self.max_deceleration) or (ttc_value < dynamic_ttc) or trigger_lockout

        # ---------------------------------------------------------
        # UNINTERRUPTIBLE STATE MACHINE
        # ---------------------------------------------------------
        if not trigger_aeb and self.brake_state != 'BRAKING':
            self.free_drive_duration += dt
            self.brake_state = 'IDLE'
            filtered_msg.twist.linear.x = cmd_vel_forward
        else:
            if self.brake_state == 'IDLE':
                if self.free_drive_duration < 1.0 or self.last_cmd_vel_forward <= 0.0:
                    self.brake_state = 'HOLD'
                    filtered_msg.twist.linear.x = 0.0
                    if self.free_drive_duration < 1.0:
                        self.get_logger().warn(f"AEB BLOCKED: Sensor stutter intercepted.")
                else:
                    self.brake_state = 'BRAKING'
                    self.brake_start_time = current_time_sec
                    
                    if trigger_lockout:
                        self.active_brake_cmd = -0.3
                        self.dynamic_pulse_duration = 1.5 # Massive lockout pulse
                        self.get_logger().warn(f"EMERGENCY LOCKOUT! Dist: {front_distance:.2f}m.")
                    else:
                        accel_penalty = max(0.0, self.filtered_accel_x * 0.15)
                        speed_penalty = cmd_vel_forward * 1.5
                        brake_intensity = 0.6 if req_decel == float('inf') else (speed_penalty + accel_penalty)
                        
                        self.active_brake_cmd = -float(max(0.25, min(0.4, brake_intensity)))
                        
                        # SCALING THE MASSIVE BRAKE PULSE (1.5s to 3.0s)
                        self.dynamic_pulse_duration = 1.5 + (1.5 * speed_factor)
                        self.get_logger().warn(f"AEB ENGAGED! Force: {self.active_brake_cmd:.2f} | Pulse: {self.dynamic_pulse_duration:.2f}s")
                        
                    filtered_msg.twist.linear.x = self.active_brake_cmd 
                
            elif self.brake_state == 'BRAKING':
                elapsed = current_time_sec - self.brake_start_time
                if elapsed < self.dynamic_pulse_duration:
                    filtered_msg.twist.linear.x = self.active_brake_cmd
                else:
                    self.brake_state = 'HOLD'
                    filtered_msg.twist.linear.x = 0.0
                    
            elif self.brake_state == 'HOLD':
                filtered_msg.twist.linear.x = 0.0 

            self.free_drive_duration = 0.0
            
        self.cmd_pub.publish(filtered_msg)
        self.last_cmd_vel_forward = cmd_vel_forward

def main(args=None):
    rclpy.init(args=args)
    node = KinematicAEBFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
