"""View the robot arm URDF in RViz2 with joint sliders (ROS 2 Humble).

Run:
  ros2 launch robot_arm display.launch.py            # with slider GUI
  ros2 launch robot_arm display.launch.py gui:=false # without slider GUI
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_arm')
    urdf_path = os.path.join(pkg_share, 'description', 'urdf', 'robot_arm.urdf')
    rviz_path = os.path.join(pkg_share, 'description', 'rviz', 'robot_arm.rviz')

    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    gui = LaunchConfiguration('gui')

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Start joint_state_publisher_gui (sliders) instead of joint_state_publisher'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            condition=IfCondition(gui),
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            condition=UnlessCondition(gui),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_path],
        ),
    ])
