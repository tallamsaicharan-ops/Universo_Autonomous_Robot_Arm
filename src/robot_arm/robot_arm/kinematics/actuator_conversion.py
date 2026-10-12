"""
Joint-angle <-> linear-actuator geometry conversion placeholder.

Do not use for physical motion until CAD dimensions, angle conventions,
limits, and tests have been filled in and validated.
"""
class ActuatorConversion:
    def __init__(self, geometry):
        self.geometry = geometry

    def joint_angle_to_length(self, joint_angle_rad):
        raise NotImplementedError("Implement from measured CAD linkage geometry")

    def length_to_joint_angle(self, length_m):
        raise NotImplementedError("May have multiple solutions; use limits/current state")
