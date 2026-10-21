"""Game controller teleop: joy_node (reads the pad) + joy_teleop (moves the arm).

Run next to any simulation (or later the real arm):
  ros2 launch robot_arm teleop.launch.py                    # RViz / mock hardware
  ros2 launch robot_arm teleop.launch.py use_sim_time:=true # Gazebo
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    joystick_yaml = os.path.join(get_package_share_directory('robot_arm'), 'config', 'joystick.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='true when running with Gazebo'),

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[joystick_yaml, {'use_sim_time': use_sim_time}],
        ),

        Node(
            package='robot_arm',
            executable='joy_teleop',
            name='joy_teleop',
            output='screen',
            parameters=[joystick_yaml, {'use_sim_time': use_sim_time}],
        ),
    ])
