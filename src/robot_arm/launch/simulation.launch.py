"""One command for every simulator, with the game controller.

  ros2 launch robot_arm simulation.launch.py                        # RViz + MoveIt (mock motors)
  ros2 launch robot_arm simulation.launch.py simulator:=gazebo      # Gazebo Classic 11
  ros2 launch robot_arm simulation.launch.py simulator:=ignition    # Gazebo Ignition Fortress
  ... joystick:=false   to run without a game controller
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory('robot_arm'), 'launch')
    simulator = LaunchConfiguration('simulator')

    def include(path, condition, args=None):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(path), condition=condition,
                                        launch_arguments=(args or {}).items())

    # robot_arm_moveit_config is looked up only when simulator:=rviz is chosen
    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('robot_arm_moveit_config'), 'launch', 'demo.launch.py'])),
        condition=LaunchConfigurationEquals('simulator', 'rviz'))

    use_sim_time = PythonExpression(["'", simulator, "' != 'rviz'"])

    return LaunchDescription([
        DeclareLaunchArgument('simulator', default_value='rviz', choices=['rviz', 'gazebo', 'ignition'],
                              description='rviz (mock motors + MoveIt), gazebo (Classic 11) or ignition (Fortress)'),
        DeclareLaunchArgument('joystick', default_value='true', description='start the game controller teleop'),
        DeclareLaunchArgument('mount_height', default_value='0.4', description='Gazebo pedestal height (m)'),
        DeclareLaunchArgument('rviz', default_value='true', description='open RViz with Gazebo'),

        moveit_demo,
        include(os.path.join(launch_dir, 'gazebo.launch.py'), LaunchConfigurationEquals('simulator', 'gazebo'),
                {'mount_height': LaunchConfiguration('mount_height'), 'rviz': LaunchConfiguration('rviz')}),
        include(os.path.join(launch_dir, 'ignition.launch.py'), LaunchConfigurationEquals('simulator', 'ignition'),
                {'mount_height': LaunchConfiguration('mount_height'), 'rviz': LaunchConfiguration('rviz')}),
        include(os.path.join(launch_dir, 'teleop.launch.py'), IfCondition(LaunchConfiguration('joystick')),
                {'use_sim_time': use_sim_time}),
    ])
