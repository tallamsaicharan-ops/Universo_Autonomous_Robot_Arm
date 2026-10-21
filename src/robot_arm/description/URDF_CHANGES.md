# robot_arm.urdf — what changed vs. the Onshape export

CAD geometry is untouched: all 15 CAD links, meshes, mesh origins, inertials and CAD joint
transforms are copied verbatim (verified: link poses identical to the original to 1e-15 m).

| # | Change | Why |
|---|--------|-----|
| 1 | Mesh paths `package://robotic_arm_cad/meshes/` → `package://robot_arm/description/meshes/` | `robotic_arm_cad` was not a ROS 2 package, so meshes could not load |
| 2 | Added `base_link` + `base_joint` (continuous, z-up) + `base_yaw_link` | CAD has no turntable → no base joint in the export |
| 3 | Replaced `root` / `fixed_node_to_root_joint` (0.18° / 0.49° CAD alignment noise) with an exact rotation | clean REP-103 frame: x forward, y left, z up |
| 4 | `revolute_1` → `shoulder_joint`, `revolute_2` → `elbow_joint` | same origin + axis, readable names |
| 5 | Limits from actuator geometry (was Onshape placeholders: effort=1, velocity=1, wrong angles) | see table below |
| 6 | `<collision>` on every link (same mesh as visual) | MoveIt needs collision geometry |
| 7 | New `arm_tip` frame = centre of forearm extrusion end face | IK target (gripper mounting face) |

Removed: the ROS 1 launch file (`robotic_arm_cad.launch`) → replaced by `launch/display.launch.py`.

## Confirmed inputs
- Base yaw axis passes through the centre of the base block (`ra_5`): 5.0 mm behind the part origin,
  97.5 mm behind the shoulder axis. Slip ring fitted → `base_joint` is truly continuous.
- Actuator pin-to-pin (team value): 300 mm retracted, 500 mm extended, stroke 200 ± 3 mm.
  (Datasheet formula would give 305 / 505; the team chose 300 / 500.)

## Geometry (from URDF, mm)
- Shoulder axis → elbow axis: 540.0. Elbow axis → arm_tip: 430.0.
- Shoulder axis: 20.0 above base-block bottom, 92.5 in front of part origin.
- Actuator pins: base lug (−100, 75) in base part; cross plates on upper arm at 250 (shoulder actuator)
  and 170 (elbow actuator) above shoulder axis, 30.5 in front of arm axis; forearm plate 197.4 from elbow.

## Joint conventions (zero = CAD export pose)
- shoulder 0 = upper arm vertical, negative = forward. Upper-arm elevation = 90° + q_shoulder.
- elbow 0 = inner angle 125.0°. Inner angle = 125.0° − q_elbow. Forearm elevation = 35.0° + q_shoulder − q_elbow.

## Limits (rad / deg) and where they come from
| Joint | Lower | Upper |
|-------|-------|-------|
| shoulder_joint | −1.570796 / −90.0° (CAD limit; actuator 450.7 mm) | −0.022426 / −1.285° (actuator 300 mm) |
| elbow_joint | −0.149650 / −8.574° (actuator 500 mm) | 1.007196 / 57.708° (actuator 300 mm) |


Shoulder warning: max possible pin-to-pin is 452.1 mm (toggle at −98.99°), so the actuator's
built-in extend limit switch (500 mm) can NEVER trigger on the shoulder. Lever arm drops from
167 mm (vertical) to 17.5 mm (horizontal) → 0.35°/mm near vertical, 3.3°/mm near horizontal.
