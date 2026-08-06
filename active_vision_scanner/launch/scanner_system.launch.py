import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # -----------------------------------------------------------------------
    # Declare Launch Arguments (overridable from command line)
    # -----------------------------------------------------------------------
    esp32_ip_arg = DeclareLaunchArgument(
        'esp32_ip',
        default_value='10.225.34.175',
        description='IP address of the ESP32 Servo Controller'
    )
    
    esp32_cam_url_arg = DeclareLaunchArgument(
        'esp32_cam_url',
        default_value='http://10.225.34.222:81/stream',
        description='ESP32-CAM MJPEG Video Stream URL'
    )

    camera_height_cm_arg = DeclareLaunchArgument(
        'camera_height_cm',
        default_value='13.35',
        description='Camera height from ground in cm'
    )

    rack_height_cm_arg = DeclareLaunchArgument(
        'rack_height_cm',
        default_value='24.5',
        description='Total rack height in cm (halved for 2-shelf racks)'
    )

    rack_horizontal_cm_arg = DeclareLaunchArgument(
        'rack_horizontal_cm',
        default_value='60.0',
        description='Rack width in cm'
    )

    camera_distance_cm_arg = DeclareLaunchArgument(
        'camera_distance_cm',
        default_value='45.0',
        description='Distance from camera to rack in cm'
    )

    database_path_arg = DeclareLaunchArgument(
        'database_path',
        default_value='/home/abhinav/warehouse_bot_simulation_ws/src/warehouse_database.xlsx',
        description='Path to Excel database file'
    )

    scanned_inventory_path_arg = DeclareLaunchArgument(
        'scanned_inventory_path',
        default_value='/home/abhinav/warehouse_bot_simulation_ws/src/scanned_inventory.csv',
        description='Output path for correct inventory CSV'
    )

    mismatch_log_path_arg = DeclareLaunchArgument(
        'mismatch_log_path',
        default_value='/home/abhinav/warehouse_bot_simulation_ws/src/mismatch_log.csv',
        description='Output path for mismatch log CSV'
    )

    # -----------------------------------------------------------------------
    # Launch Configurations
    # -----------------------------------------------------------------------
    esp32_ip = LaunchConfiguration('esp32_ip')
    esp32_cam_url = LaunchConfiguration('esp32_cam_url')
    camera_height_cm = LaunchConfiguration('camera_height_cm')
    rack_height_cm = LaunchConfiguration('rack_height_cm')
    rack_horizontal_cm = LaunchConfiguration('rack_horizontal_cm')
    camera_distance_cm = LaunchConfiguration('camera_distance_cm')
    database_path = LaunchConfiguration('database_path')
    scanned_inventory_path = LaunchConfiguration('scanned_inventory_path')
    mismatch_log_path = LaunchConfiguration('mismatch_log_path')

    # -----------------------------------------------------------------------
    # Nodes to Launch
    # -----------------------------------------------------------------------
    # 1. ESP32 WiFi HTTP Bridge Node
    esp32_bridge_node = Node(
        package='active_vision_scanner',
        executable='esp32_bridge',
        name='esp32_wifi_bridge',
        output='screen',
        parameters=[{
            'esp32_ip': esp32_ip,
        }]
    )

    # 2. Active Vision Rack Scanner Node
    rack_scanner_node = Node(
        package='active_vision_scanner',
        executable='rack_scanner',
        name='rack_scanner_node',
        output='screen',
        parameters=[{
            'esp32_cam_url': esp32_cam_url,
            'camera_height_cm': camera_height_cm,
            'rack_height_cm': rack_height_cm,
            'rack_horizontal_cm': rack_horizontal_cm,
            'camera_distance_cm': camera_distance_cm,
            'database_path': database_path,
            'scanned_inventory_path': scanned_inventory_path,
            'mismatch_log_path': mismatch_log_path,
        }]
    )

    return LaunchDescription([
        esp32_ip_arg,
        esp32_cam_url_arg,
        camera_height_cm_arg,
        rack_height_cm_arg,
        rack_horizontal_cm_arg,
        camera_distance_cm_arg,
        database_path_arg,
        scanned_inventory_path_arg,
        mismatch_log_path_arg,
        esp32_bridge_node,
        rack_scanner_node,
    ])
