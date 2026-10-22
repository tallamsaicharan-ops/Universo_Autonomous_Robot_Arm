import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    moveit_share = get_package_share_directory("robot_arm_moveit_config")

    demo_launch = os.path.join(
        moveit_share, "launch", "demo.launch.py"
    )
    servo_launch = os.path.join(
        moveit_share, "launch", "servo.launch.py"
    )

    moveit_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(demo_launch)
    )

    servo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(servo_launch)
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
    )

    xbox_teleop = Node(
        package="universo_bringup",
        executable="xbox_servo.py",
        name="xbox_servo",
        output="screen",
    )

    start_servo = Node(
        package="universo_bringup",
        executable="servo_starter.py",
        name="servo_starter",
        output="screen",
    )

    return LaunchDescription([
        moveit_simulation,
        servo,
        joy_node,
        xbox_teleop,
        TimerAction(
            period=3.0,
            actions=[start_servo],
        ),
    ])
