import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('pi_bot'))
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot_base.urdf.xacro')
    robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=true sim_mode:=false'])

    # Create a robot_state_publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config, 'use_sim_time': False}]
    )

    return LaunchDescription([
        node_robot_state_publisher
    ])