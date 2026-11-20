"""Unitree wire order and per-physics-step motor servo math."""
from collections.abc import Mapping
import torch

BODY_JOINT_NAMES = tuple(
    f'{side}_{joint}_joint'
    for side in ('left', 'right')
    for joint in ('hip_yaw', 'hip_pitch', 'hip_roll', 'knee', 'ankle_pitch', 'ankle_roll')
) + ('torso_joint',) + tuple(
    f'{side}_{joint}_joint'
    for side in ('left', 'right')
    for joint in ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow', 'wrist_roll', 'wrist_pitch', 'wrist_yaw')
)
HAND_JOINT_NAMES = tuple(
    f'{side}_{joint}_joint'
    for side in ('R', 'L')
    for joint in ('pinky_proximal', 'ring_proximal', 'middle_proximal', 'index_proximal', 'thumb_proximal_pitch', 'thumb_proximal_yaw')
)
MOTOR_FIELDS = ('positions', 'velocities', 'torques', 'kp', 'kd', 'mode')
BODY_ACTION_DIM = 6 * len(BODY_JOINT_NAMES)
ACTION_DIM = BODY_ACTION_DIM + len(HAND_JOINT_NAMES)


def pack_motor_command(command, device):
    """Pack the 27 body motors; unused trailing IDL slots are ignored."""
    if not isinstance(command, Mapping):
        raise ValueError("LowCmd must be a mapping")
    n = len(BODY_JOINT_NAMES)
    if any(len(command.get(field, ())) < n for field in MOTOR_FIELDS):
        raise ValueError('LowCmd requires all six fields for 27 motors')
    values = torch.tensor([command[field][:n] for field in MOTOR_FIELDS], device=device, dtype=torch.float32)
    if not torch.isfinite(values).all() or (values[3:5] < 0).any():
        raise ValueError('LowCmd fields must be finite and gains nonnegative')
    return values.flatten()
