"""Gazebo Ignition (Fortress, the Gazebo paired with ROS 2 Humble): arm on a pedestal + ros2_control.

  ros2 launch robot_arm ignition.launch.py
  ros2 launch robot_arm ignition.launch.py rviz:=true world:=empty.sdf
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
    default_world = os.path.join(pkg, 'worlds', 'arm_test.sdf')
    rviz_config = os.path.join(pkg, 'description', 'rviz', 'robot_arm.rviz')

    # Lets Ignition find package://robot_arm/... meshes
    share_parent = os.path.dirname(pkg)
    resource_path = os.pathsep.join(
        p for p in (os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''), share_parent) if p)

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file,
                 ' sim:=ignition', ' mount_height:=', LaunchConfiguration('mount_height')]),
        value_type=str)

    ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments=[('gz_args', ['-r ', LaunchConfiguration('world')])])

    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'])

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', 'robot_arm', '-allow_renaming', 'true'])

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
        DeclareLaunchArgument('world', default_value=default_world,
                              description='SDF world file (or a built-in one such as empty.sdf)'),
        DeclareLaunchArgument('mount_height', default_value='0.4', description='pedestal height (m)'),
        DeclareLaunchArgument('rviz', default_value='false', description='also open RViz'),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_path),
        ignition,
        clock_bridge,
        robot_state_publisher,
        spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[joint_state_broadcaster])),
        RegisterEventHandler(OnProcessExit(target_action=joint_state_broadcaster, on_exit=[arm_controller])),
        rviz,
    ])
