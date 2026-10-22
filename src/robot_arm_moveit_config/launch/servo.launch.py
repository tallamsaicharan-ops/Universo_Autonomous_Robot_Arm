import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    package_name = "robot_arm_moveit_config"
    package_share = get_package_share_directory(package_name)

    servo_config_path = os.path.join(
        package_share, "config", "servo.yaml"
    )

    with open(servo_config_path, "r") as file:
        servo_yaml = yaml.safe_load(file)["moveit_servo"]["ros__parameters"]

    moveit_config = (
        MoveItConfigsBuilder(
            "robot_arm",
            package_name=package_name,
        )
        .to_moveit_configs()
    )

    return LaunchDescription([
        Node(
            package="moveit_servo",
            executable="servo_node_main",
            name="servo_node",
            output="screen",
            parameters=[
                {"moveit_servo.move_group_name": "arm"},
                {"moveit_servo.planning_frame": "base_link"},
                {"moveit_servo.ee_frame_name": "arm_tip"},
                {"moveit_servo.robot_link_command_frame": "base_link"},
                {"moveit_servo.command_out_topic": "/arm_controller/joint_trajectory"},
                {"moveit_servo.command_in_type": "unitless"},
                {"moveit_servo.is_primary_planning_scene_monitor": False},
                {"moveit_servo.check_collisions": True},
                {"moveit_servo.incoming_command_timeout": 0.1},
                {"moveit_servo.publish_period": 0.034},
                {"moveit_servo.scale.linear": 20.0},
                {"moveit_servo.scale.rotational": 0.2},
                {"moveit_servo.scale.joint": 0.2},
                servo_yaml,
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
            ],
        )
    ])
