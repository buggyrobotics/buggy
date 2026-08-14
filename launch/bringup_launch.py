import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_buggy = get_package_share_directory('buggy')

    # NOTE: this xacro must resolve to a <ros2_control> block with a REAL
    # hardware plugin (not gz_ros2_control) when a "use_sim:=false" (or
    # similar) arg is passed. Adjust the xacro args below to match however
    # your xacro is set up to switch between sim/real hardware interfaces.
    xacro_file = os.path.join(pkg_buggy, 'urdf', 'nvis_3302ard.urdf.xacro')
    controllers_yaml = os.path.join(pkg_buggy, 'config', 'controllers.yaml')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_sim:=false']),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
    )

    # Real controller_manager — this replaces what gz_ros2_control did in sim.
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            {'robot_description': robot_description, 'use_sim_time': False},
            controllers_yaml,
        ],
    )

    # Real LIDAR driver — REPLACE this block with your actual sensor's
    # driver package/executable (e.g. rplidar_ros, sick_scan_xd, ldlidar, etc).
    # It must publish sensor_msgs/LaserScan directly on /scan.
    lidar_driver = Node(
        package='your_lidar_driver_pkg',       # <-- replace
        executable='your_lidar_driver_node',   # <-- replace
        name='lidar_driver',
        output='screen',
        parameters=[{'use_sim_time': False}],
        # arguments/parameters for port, baud rate, frame_id, etc. go here
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster'],
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['diff_drive_controller'],
    )

    # controller_manager never "exits" like spawn_robot did in sim, so we
    # can't key off OnProcessExit. Instead wait for it to start, then give
    # it a moment to come up before spawning controllers.
    delayed_jsb = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[TimerAction(period=3.0, actions=[joint_state_broadcaster_spawner])],
        )
    )

    delayed_diff_drive = TimerAction(
        period=6.0,
        actions=[diff_drive_spawner],
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        lidar_driver,
        delayed_jsb,
        delayed_diff_drive,
    ])