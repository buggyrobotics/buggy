import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_buggy = get_package_share_directory('buggy')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    nav2_params = os.path.join(pkg_buggy, 'config', 'nav2_params.yaml')
    map_file = os.path.join(pkg_buggy, 'maps', 'my_map_2.yaml')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'params_file': nav2_params,
            'use_sim_time': 'false',   # <-- changed
        }.items()
    )

    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        output='screen',
        arguments=['/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped'],
        parameters=[{'use_sim_time': False}],   # <-- changed
    )

    return LaunchDescription([
        nav2,
        cmd_vel_relay,
    ])