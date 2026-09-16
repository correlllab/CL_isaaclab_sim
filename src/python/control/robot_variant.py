"""Robot asset and attachment names shared by configuration and validation."""
from dataclasses import dataclass


@dataclass(frozen=True)
class RobotVariant:
    usd_path: str
    hand_joints: tuple[str, ...]
    head_link: str
    lidar_link: str
    wrist_links: tuple[str, str]
    inspire: bool


def robot_variant(name='magpie'):
    if name == 'magpie':
        return RobotVariant(
            'isaac_assets/robots/h1_2_magpie/h1_2_magpie.usd',
            tuple(f'{side}_{finger}_hinge_{i}' for side in ('lg', 'rg')
                  for finger in ('left', 'right') for i in (1, 2, 3)),
            'head_camera_link', 'livox_link', ('lg_base_top', 'rg_base_top'), False)
    if name == 'inspire':
        return RobotVariant(
            'isaac_assets/robots/h1_2-26dof-inspire-base-fix-usd/h1_2_26dof_with_inspire_rev_1_0.usd',
            tuple(f'{side}_{finger}_joint' for side in ('R', 'L') for finger in (
                'pinky_proximal', 'ring_proximal', 'middle_proximal', 'index_proximal',
                'thumb_proximal_pitch', 'thumb_proximal_yaw')),
            'camera_link', 'lidar_link', ('L_hand_base_link', 'R_hand_base_link'), True)
    raise ValueError(f'Unknown robot variant: {name}')
