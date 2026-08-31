from ..registry import RobotAdapter, register_robot


def _joints(names, indices):
    return tuple(zip(names, indices))


_G1_NAMES = (
    "kLeftHipPitch", "kLeftHipRoll", "kLeftHipYaw", "kLeftKnee",
    "kLeftAnklePitch", "kLeftAnkleRoll", "kRightHipPitch",
    "kRightHipRoll", "kRightHipYaw", "kRightKnee", "kRightAnklePitch",
    "kRightAnkleRoll", "kWaistYaw", "kWaistRoll", "kWaistPitch",
    "kLeftShoulderPitch", "kLeftShoulderRoll", "kLeftShoulderYaw",
    "kLeftElbow", "kLeftWristRoll", "kLeftWristPitch", "kLeftWristyaw",
    "kRightShoulderPitch", "kRightShoulderRoll", "kRightShoulderYaw",
    "kRightElbow", "kRightWristRoll", "kRightWristPitch", "kRightWristYaw",
)

register_robot(RobotAdapter(
    name="G1_29", dds_family="hg", motor_count=35,
    joints=_joints(_G1_NAMES, range(29)),
    controller_module="dexfull.control.robots.unitree.arm",
    controller_class="G1_29_ArmController",
    ik_module="dexfull.control.robots.unitree.ik", ik_class="G1_29_ArmIK",
    pelvis_height=0.793,
))

_G1_23_NAMES = tuple(
    name for index, name in enumerate(_G1_NAMES)
    if index not in (13, 14, 20, 21, 27, 28)
)
_G1_23_INDICES = tuple(index for index in range(29) if index not in (13, 14, 20, 21, 27, 28))
register_robot(RobotAdapter(
    name="G1_23", dds_family="hg", motor_count=35,
    joints=_joints(_G1_23_NAMES, _G1_23_INDICES),
    controller_module="dexfull.control.robots.unitree.arm",
    controller_class="G1_23_ArmController",
    ik_module="dexfull.control.robots.unitree.ik", ik_class="G1_23_ArmIK",
    pelvis_height=0.793,
))

_H1_2_NAMES = (
    "kLeftHipYaw", "kLeftHipRoll", "kLeftHipPitch", "kLeftKnee",
    "kLeftAnkle", "kLeftAnkleRoll", "kRightHipYaw", "kRightHipRoll",
    "kRightHipPitch", "kRightKnee", "kRightAnkle", "kRightAnkleRoll",
    "kWaistYaw", "kLeftShoulderPitch", "kLeftShoulderRoll",
    "kLeftShoulderYaw", "kLeftElbowPitch", "kLeftElbowRoll",
    "kLeftWristPitch", "kLeftWristyaw", "kRightShoulderPitch",
    "kRightShoulderRoll", "kRightShoulderYaw", "kRightElbowPitch",
    "kRightElbowRoll", "kRightWristPitch", "kRightWristYaw",
)
register_robot(RobotAdapter(
    name="H1_2", dds_family="hg", motor_count=35,
    joints=_joints(_H1_2_NAMES, range(27)),
    controller_module="dexfull.control.robots.unitree.arm",
    controller_class="H1_2_ArmController",
    ik_module="dexfull.control.robots.unitree.ik", ik_class="H1_2_ArmIK",
    pelvis_height=1.03,
))

_H1_NAMES = (
    "kRightHipRoll", "kRightHipPitch", "kRightKnee", "kLeftHipRoll",
    "kLeftHipPitch", "kLeftKnee", "kWaistYaw", "kLeftHipYaw",
    "kRightHipYaw", "kLeftAnkle", "kRightAnkle", "kRightShoulderPitch",
    "kRightShoulderRoll", "kRightShoulderYaw", "kRightElbow",
    "kLeftShoulderPitch", "kLeftShoulderRoll", "kLeftShoulderYaw",
    "kLeftElbow",
)
_H1_INDICES = tuple(index for index in range(20) if index != 9)
register_robot(RobotAdapter(
    name="H1", dds_family="go", motor_count=20,
    joints=_joints(_H1_NAMES, _H1_INDICES),
    controller_module="dexfull.control.robots.unitree.arm",
    controller_class="H1_ArmController",
    ik_module="dexfull.control.robots.unitree.ik", ik_class="H1_ArmIK",
    pelvis_height=1.0, supports_motion_mode=False,
))

