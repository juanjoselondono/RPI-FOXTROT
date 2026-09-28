import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = "ybeb_2602_zulu"

    return LaunchDescription([
        # 1. Hardware Motor/IMU Driver
        Node(
            package=pkg_name,
            executable='ybeb_node.py',
            name='twist_cmd_node',
            output='screen'
        ),
        
        # 2. Hardware LiDAR Driver (Persistent ID)
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
                'serial_baudrate': 115200,
                'frame_id': 'laser_frame',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Sensitivity'
            }]
        ),
        
        # 3. Local Safety Filter (Edge Computing)
        Node(
            package=pkg_name,
            executable='aeb_ttc.py',
            name='kinematic_aeb_filter',
            output='screen'
        ),
        
        # 4. Local State Supervisor
        Node(
            package=pkg_name,
            executable='supervisor.py',
            name='mode_supervisor',
            output='screen'
        )
    ])
