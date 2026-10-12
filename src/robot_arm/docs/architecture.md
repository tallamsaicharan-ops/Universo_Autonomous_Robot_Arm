# Architecture notes

## Motion command path
Target pose -> TF2/base-frame transform -> MoveIt IK/planning -> trajectory validation/timing
-> ROS 2 controller/hardware interface -> CAN controller(s) -> actuators.

## Feedback path
Actuator/joint sensors -> CAN parser/interface -> measured actuator state -> joint-angle conversion
(where valid) -> ROS 2 joint states -> monitoring and fault handling.

## Linear actuator model
For a simple planar mechanism, compute the actuator pin-to-pin distance from the measured coordinates
of the fixed and moving mounts relative to the joint pivot. Do not assume that a single angle/length
formula fits the actual linkage without checking the CAD geometry. The inverse mapping may have
multiple solutions. Keep joint-angle and actuator-stroke limits separate.

## Collision safety
MoveIt collision checking depends on accurate collision geometry, planning-scene obstacles, and
correct robot transforms. Software collision checking is not a substitute for a physical E-stop,
safe limits, or supervised commissioning.
