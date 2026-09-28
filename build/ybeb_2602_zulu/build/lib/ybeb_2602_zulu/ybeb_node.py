#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu, MagneticField

from Rosmaster_Lib import Rosmaster

class TWIST_CMD_NODE(Node):
    def __init__(self):
        super().__init__("twist_cmd_node_charlie_2402")
        self.get_logger().info("Hardware Driver Online. Watchdog Active.")

        self.twist_subs = self.create_subscription(
            TwistStamped, 
            '/cmd_vel_stamped', 
            self.twist_callback, 
            1
        )

        # 50 Hz control loop
        timer_period = 0.02 
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.imuPublisher = self.create_publisher(Imu, "/imu/data_raw", 100)
        self.magPublisher = self.create_publisher(MagneticField, "/imu/mag", 100)
        self.volPublisher = self.create_publisher(Float32, "voltage", 100)

        self.robot = Rosmaster()
        self.robot.create_receive_threading()

        self.cmd_vel = TwistStamped()
        
        # Initialize Watchdog Timestamp
        self.last_msg_time = self.get_clock().now()

    def timer_callback(self):
        current_time = self.get_clock().now()
        
        imu = Imu()
        mag = MagneticField()
        battery = Float32()

        battery.data = float(self.robot.get_battery_voltage())
        ax, ay, az = self.robot.get_accelerometer_data()
        gx, gy, gz = self.robot.get_gyroscope_data()
        mx, my, mz = self.robot.get_magnetometer_data()

        imu.header.stamp = current_time.to_msg()
        imu.header.frame_id = 'imu_link'
        imu.linear_acceleration.x = float(ax)
        imu.linear_acceleration.y = float(ay)
        imu.linear_acceleration.z = float(az)
        imu.angular_velocity.x = float(gx)
        imu.angular_velocity.y = float(gy)
        imu.angular_velocity.z = float(gz)

        mag.header.stamp = current_time.to_msg()
        mag.header.frame_id = 'imu_link'
        mag.magnetic_field.x = float(mx * 1.0)
        mag.magnetic_field.y = float(my * 1.0)
        mag.magnetic_field.z = float(mz * 1.0)

        self.imuPublisher.publish(imu)
        self.magPublisher.publish(mag)
        self.volPublisher.publish(battery)

        # ---------------------------------------------------------
        # HARDWARE WATCHDOG: Check for Wi-Fi Loss of Signal (LOS)
        # ---------------------------------------------------------
        time_since_last_msg = (current_time - self.last_msg_time).nanoseconds / 1e9
        
        if time_since_last_msg > 0.5:
            # If no command received in 500ms, force neutral to prevent runaway
            self.cmd_vel.twist.linear.x = 0.0
            self.cmd_vel.twist.angular.z = 0.0
            
            # Prevent log spamming at 50Hz by only logging occasionally
            if int(time_since_last_msg * 50) % 50 == 0:
                self.get_logger().error("LOS DETECTED: Network timeout. Hardware watchdog engaged.")

        # Map to PWM space
        throttle_float = self.map(self.cmd_vel.twist.linear.x, -0.5, +0.5, 47.5, 132.5)
        steering_float = self.map(self.cmd_vel.twist.angular.z, -0.5, +0.5, 47.5, 132.5)
        
        # CRITICAL FIX: Cast to integer to prevent I2C driver lockups
        throttle = int(max(47, min(132, throttle_float)))
        steering = int(max(47, min(132, steering_float)))

        if time_since_last_msg <= 0.5:
            msg = 'xd: {:.3f}, PWM: {:d}, Thd: {:.3f}, PWM: {:d}'.format(
                self.cmd_vel.twist.linear.x, throttle, 
                self.cmd_vel.twist.angular.z, steering
            )
            self.get_logger().info(msg)

        # Execute hardware writes
        self.robot.set_pwm_servo(1, throttle)
        self.robot.set_pwm_servo(4, steering)

    def twist_callback(self, data):
        # Update watchdog timer on every successful packet reception
        self.last_msg_time = self.get_clock().now()
        
        self.cmd_vel.twist.linear.x = data.twist.linear.x
        self.cmd_vel.twist.angular.z = data.twist.angular.z

    def map(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    
    def kill(self):
        # Ensure motors stop on shutdown
        self.robot.set_pwm_servo(1, 90)
        self.robot.set_pwm_servo(4, 90)
        del self.robot

def main(args=None):
    rclpy.init(args=args)
    node = TWIST_CMD_NODE()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Hardware driver terminating.")
    finally:
        node.kill()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()
