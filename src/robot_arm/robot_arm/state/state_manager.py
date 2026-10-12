from .arm_state import ArmState, ArmStatus
from .joint_state import JointState
from .gripper_state import GripperState


class StateManager:
    def __init__(self):
        self.arm = ArmState()
        self.gripper = GripperState()

        # Initialize the three robot joints.
        self.arm.add_joint(JointState(name="base"))
        self.arm.add_joint(JointState(name="shoulder"))
        self.arm.add_joint(JointState(name="elbow"))

    def get_arm_state(self) -> ArmState:
        return self.arm

    def get_gripper_state(self) -> GripperState:
        return self.gripper

    def update_joint_position(
        self, joint_name: str, measured_position: float
    ) -> bool:
        joint = self.arm.get_joint(joint_name)

        if joint is None:
            return False

        joint.update_position(measured_position)
        return True

    def set_arm_status(self, status: ArmStatus) -> None:
        self.arm.set_status(status)

    def set_gripper_status(self, status) -> None:
        self.gripper.set_status(status)