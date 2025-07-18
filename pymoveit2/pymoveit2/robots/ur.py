from typing import List

MOVE_GROUP_ARM: str = "ur_manipulator"


def joint_names() -> List[str]:
    return [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]


def base_link_name(prefix: str = "ur_") -> str:
    return "base_link"


def end_effector_name(prefix: str = "ur_") -> str:
    return "tcp"
