import os
import re
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory

def is_wsl():
    try:
        with open('/proc/version', 'r') as f:
            version_info = f.read().lower()
            return 'microsoft' in version_info or 'wsl' in version_info
    except FileNotFoundError:
        return False


def generate_launch_description():
    prefix = "gnome-terminal --"
    if is_wsl():
        prefix = "xterm -e"
        print("Current system is WSL, use xterm as terminal")
    else:
        print("Current system is not WSL, use gnome-terminal as terminal")

    return LaunchDescription([
        DeclareLaunchArgument(
            name='rviz',
            default_value='false'
        ),
        DeclareLaunchArgument(
            name='target_marker',
            default_value='true',
            description='Whether to enable target marker'
        ),
        DeclareLaunchArgument(
            name='dummy',
            default_value='true',
            description='Whether to enable dummy mrt node'
        ),
        DeclareLaunchArgument(
            name='urdfFile',
            default_value=get_package_share_directory(
                'ocs2_robotic_assets') + '/resources/mobile_manipulator/viper/urdf/vx300s.urdf'
        ),
        DeclareLaunchArgument(
            name='taskFile',
            default_value=get_package_share_directory(
                'ocs2_mobile_manipulator') + '/config/viper/task.info'
        ),
        DeclareLaunchArgument(
            name='libFolder',
            default_value=get_package_share_directory(
                'ocs2_mobile_manipulator') + '/auto_generated/viper'
        ),
        DeclareLaunchArgument(
            name='debug',
            default_value='false'
        ),
        DeclareLaunchArgument(
            name='enableJoystick',
            default_value='false',
            description='Whether to enable joystick control'
        ),
        DeclareLaunchArgument(
            name='enableAutoPosition',
            default_value='false',
            description='Whether to enable automatic marker position updates'
        ),
        Node(
            package='ocs2_mobile_manipulator_ros',
            executable='mobile_manipulator_mpc',
            name='mobile_manipulator_mpc',
            prefix=prefix,
            condition=IfCondition(LaunchConfiguration("debug")),
            output='screen',
            parameters=[
                {
                    'taskFile': LaunchConfiguration('taskFile')
                },
                {
                    'urdfFile': LaunchConfiguration('urdfFile')
                },
                {
                    'libFolder': LaunchConfiguration('libFolder')
                }
            ]
        ),
        Node(
            package='ocs2_mobile_manipulator_ros',
            executable='mobile_manipulator_mpc_node',
            name='mobile_manipulator_mpc',
            condition=UnlessCondition(LaunchConfiguration("debug")),
            output='screen',
            parameters=[
                {
                    'taskFile': LaunchConfiguration('taskFile')
                },
                {
                    'urdfFile': LaunchConfiguration('urdfFile')
                },
                {
                    'libFolder': LaunchConfiguration('libFolder')
                }
            ]
        ),
        Node(
            package='ocs2_mobile_manipulator_ros',
            executable='mobile_manipulator_dummy_mrt_node',
            name='mobile_manipulator_dummy_mrt_node',
            prefix=prefix,
            output='screen',
            condition=IfCondition(LaunchConfiguration("dummy")),
            parameters=[
                {
                    'taskFile': LaunchConfiguration('taskFile')
                },
                {
                    'urdfFile': LaunchConfiguration('urdfFile')
                },
                {
                    'libFolder': LaunchConfiguration('libFolder')
                }
            ]
        ),
        Node(
            package='ocs2_mobile_manipulator_ros',
            executable='mobile_manipulator_mrt_node',
            name='mobile_manipulator_mrt_node',
            prefix=prefix,
            output='screen',
            condition=UnlessCondition(LaunchConfiguration("dummy")),
            parameters=[
                {
                    'taskFile': LaunchConfiguration('taskFile')
                },
                {
                    'urdfFile': LaunchConfiguration('urdfFile')
                },
                {
                    'libFolder': LaunchConfiguration('libFolder')
                },
                {
                    'robotStateTopic': '/vx300s/arm_controller/controller_state'
                },
                {
                    'robotCommandTopic': '/vx300s/commands/joint_group'
                }
            ]
        ),
        Node(
            package='ocs2_mobile_manipulator_ros',
            executable='mobile_manipulator_target',
            name='mobile_manipulator_target',
            condition=IfCondition(LaunchConfiguration("target_marker")),
            parameters=[
                {
                    'taskFile': LaunchConfiguration('taskFile')
                },
                {
                    'enableJoystick': LaunchConfiguration('enableJoystick')
                },
                {
                    'enableAutoPosition': LaunchConfiguration('enableAutoPosition')
                },
            ],
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'ocs2_mobile_manipulator_ros'), 'launch/include/visualize.launch.py')
            ),
            launch_arguments={
                'rviz': LaunchConfiguration('rviz')
            }.items(),
            condition=IfCondition(LaunchConfiguration("rviz"))
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            condition=IfCondition(LaunchConfiguration("dummy")),
            arguments=[LaunchConfiguration("urdfFile")],
        )
    ])
