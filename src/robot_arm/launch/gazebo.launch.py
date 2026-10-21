"""Gazebo Classic 11: arm on a pedestal, ros2_control + joint trajectory controller.

  ros2 launch robot_arm gazebo.launch.py
  ros2 launch robot_arm gazebo.launch.py rviz:=true mount_height:=0.4 world:=<path>
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('robot_arm')
    xacro_file = os.path.join(pkg, 'description', 'urdf', 'robot_arm_sim.urdf.xacro')
    default_world = os.path.join(pkg, 'worlds', 'arm_test.world')
    rviz_config = os.path.join(pkg, 'description', 'rviz', 'robot_arm.rviz')

    # Lets Gazebo find package://robot_arm/... meshes
    share_parent = os.path.dirname(pkg)
    model_path = os.pathsep.join(p for p in (os.environ.get('GAZEBO_MODEL_PATH', ''), share_parent) if p)

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file,
                 ' sim:=gazebo', ' mount_height:=', LaunchConfiguration('mount_height')]),
        value_type=str)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': LaunchConfiguration('world'), 'verbose': 'false'}.items())

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    spawn = Node(
        package='gazebo_ros', executable='spawn_entity.py', output='screen',
        arguments=['-topic', 'robot_description', '-entity', 'robot_arm'])

    joint_state_broadcaster = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'])

    arm_controller = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'])

    rviz = Node(
        package='rviz2', executable='rviz2', output='screen', arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world, description='Gazebo world file'),
        DeclareLaunchArgument('mount_height', default_value='0.4', description='pedestal height (m)'),
        DeclareLaunchArgument('rviz', default_value='false', description='also open RViz'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),
        gazebo,
        robot_state_publisher,
        spawn,
        # start controllers only after the arm exists in Gazebo, broadcaster first
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[joint_state_broadcaster])),
        RegisterEventHandler(OnProcessExit(target_action=joint_state_broadcaster, on_exit=[arm_controller])),
        rviz,
    ])
