from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0',
            'serial_baudrate': 115200,
            'frame_id': 'scan_link',
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }]
    )

    return LaunchDescription([
        rplidar_node
    ])
