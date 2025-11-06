import os
import launch
import launch_ros.actions
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='rviz',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='rvizconfig',
            default_value=get_package_share_directory('ocs2_mobile_manipulator_ros') + "/rviz/mobile_manipulator.rviz"
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='mobile_manipulator',
            output='screen',
            condition=IfCondition(LaunchConfiguration("rviz")),
            arguments=["-d", LaunchConfiguration("rvizconfig")]
        )
    ])
    return ld

if __name__ == '__main__':
    generate_launch_description()
