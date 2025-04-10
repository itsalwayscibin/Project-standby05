import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    passing_service = Node(
        package="pymoveit2",
        name="servo_service",
        executable="ex_servo.py",
        output='screen'
    )

    ebot_nav2 = Node(
        package="ebot_nav2",
        name="ebot_nav2",
        executable="nav2_cmd_task3b.py",
        output='screen'
    )

    return LaunchDescription([
        TimerAction(period=0.0, actions=[ebot_nav2]),  # Wait 5 seconds
        TimerAction(period=5.0, actions=[passing_service]),  # Wait 30 seconds (5 seconds after nav2)
    ])
