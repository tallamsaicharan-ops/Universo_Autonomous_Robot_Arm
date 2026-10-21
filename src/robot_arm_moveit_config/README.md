# robot_arm_moveit_config (MoveIt 2, ROS 2 Humble)

Planning group `arm` = chain `base_link` -> `arm_tip` (base_joint, shoulder_joint, elbow_joint).
Mock hardware (ros2_control GenericSystem) - simulation only.

## Install + run
```bash
sudo apt install ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-xacro ros-humble-joint-state-publisher-gui
cd ~/autonomous_arm_ws
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_arm robot_arm_moveit_config
source install/setup.bash
ros2 launch robot_arm_moveit_config demo.launch.py
```

## Key settings
- `kinematics.yaml`: KDL with `position_only_ik: true` (3 DOF can only reach a tip position, not a full pose).
- `initial_positions.yaml`: start at "home" (shoulder 0 is outside its limits).
- `joint_limits.yaml`: real actuator speeds (shoulder 0.0420, elbow 0.0355 rad/s) -> motions take 10-30 s.
  Raise `max_velocity` here only for faster simulation.
- SRDF: no link pair can self-collide inside the limits (10000-pose check) -> all pairs disabled.
  Add the rover deck / ground as planning-scene objects to stop the arm hitting them.
- Group states: `home` (both actuators retracted), `ready` (upper arm 60 deg, elbow inner angle 110 deg).
