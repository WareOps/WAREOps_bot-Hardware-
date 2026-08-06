import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'pi_bot'
    pkg_share = get_package_share_directory(package_name)

    # 1. Launch robot (RSP, twist_mux, controllers)
    launch_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'launch_robot.launch.py')
        )
    )

    # 2. Launch RPLidar
    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rplidar.launch.py')
        )
    )

    # 3. Static TF: map -> odom
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        output='screen',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom']
    )

    # 4. Nav2 bringup
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'map': os.path.join(pkg_share, 'maps', 'New_map.yaml'),
            'params_file': os.path.join(pkg_share, 'config', 'param_nav2.yaml'),
        }.items()
    )

    # 5. Delayed execution of initial pose setup script
    initial_pose_cmd = TimerAction(
        period=8.0,  # Delay execution by 8 seconds to ensure Nav2 nodes are ready
        actions=[
            ExecuteProcess(
                cmd=['python3', '/home/abhinav/warehouse_bot_simulation_ws/src/initialise.py'],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        launch_robot,
        rplidar,
        map_to_odom_tf,
        nav2_bringup,
        initial_pose_cmd,
    ])
