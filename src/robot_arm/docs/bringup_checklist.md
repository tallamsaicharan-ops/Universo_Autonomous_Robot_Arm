# Bring-up checklist

- [ ] Confirm actuator datasheets, controller interfaces, CAN topology, and protocol.
- [ ] Measure joint pivots and actuator mounting points from CAD.
- [ ] Confirm joint zero conventions, axis directions, and safe mechanical limits.
- [ ] Implement and unit-test angle/length conversion.
- [ ] Verify URDF visually in RViz; check collision meshes and self-collision.
- [ ] Configure MoveIt planning group, IK plugin, OMPL, and controller execution.
- [ ] Validate feedback scaling, sign, offset, freshness, and fault reporting.
- [ ] Test CAN with motors disabled/unpowered where possible.
- [ ] Verify physical emergency stop/drive disable independently.
- [ ] Test one joint at a time at a safe low speed under supervision.
- [ ] Only then test perception-driven autonomous positioning.
- [ ] Keep gripper autonomy disabled until separately validated.
