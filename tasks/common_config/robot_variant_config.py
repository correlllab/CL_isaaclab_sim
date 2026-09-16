"""Select the published robot USD and its matching sensors and actuators."""
from pathlib import Path
import os
from isaaclab.actuators import ImplicitActuatorCfg
from src.python.control.motor_contract import BODY_JOINT_NAMES
from src.python.control.robot_variant import robot_variant


def configure_robot_variant(cfg, name='magpie', fix_base=False):
    variant = robot_variant(name)
    assets = Path(os.environ.get('GOLEM_ASSETS_ROOT', Path(__file__).resolve().parents[3] / 'CL_Assets'))
    usd_path = assets / variant.usd_path
    if not usd_path.is_file():
        raise FileNotFoundError(f'Robot USD is missing: {usd_path}; initialize CL_Assets and Git LFS')
    robot = cfg.scene.robot
    robot.spawn.usd_path = str(usd_path)
    if fix_base:
        robot.spawn.articulation_props.fix_root_link = True
    cfg.actions.motor.hand_type = name
    if variant.inspire:
        return
    robot.init_state.joint_pos = {
        name: value for name, value in robot.init_state.joint_pos.items() if name in BODY_JOINT_NAMES
    }
    robot.init_state.joint_pos.update({name: 0. for name in variant.hand_joints})
    # Body limits come from the new USD instead of the Inspire bench fixture.
    for actuator in robot.actuators.values():
        actuator.effort_limit_sim = actuator.effort_limit = None
        actuator.velocity_limit_sim = actuator.velocity_limit = None
    from .magpie_spawn import spawn_magpie
    robot.spawn.func = spawn_magpie
    driven = [joint for joint in variant.hand_joints if joint.endswith('_1')]
    passive = [joint for joint in variant.hand_joints if not joint.endswith('_1')]
    robot.actuators['hands'] = ImplicitActuatorCfg(
        joint_names_expr=driven, effort_limit_sim=10.,
        velocity_limit_sim=3.14, stiffness=10., damping=1.)
    robot.actuators['hand_linkage'] = ImplicitActuatorCfg(
        joint_names_expr=passive, effort_limit_sim=0.,
        velocity_limit_sim=3.14, stiffness=0., damping=0.)
    cfg.observations.policy.robot_inspire_state = None
    root = '/World/envs/env_.*/Robot'
    cfg.scene.front_camera.prim_path = f'{root}/{variant.head_link}/front_cam'
    cfg.scene.front_camera.offset.rot = (1., 0., 0., 0.)
    for sensor, link in zip((cfg.scene.left_wrist_camera, cfg.scene.right_wrist_camera), variant.wrist_links):
        sensor.prim_path = f'{root}/{link}/wrist_camera'
        # Match the RoboCasa Magpie mount camera: +Z viewing direction.
        sensor.offset.pos = (0., 0., .045)
        sensor.offset.rot = (0., 0., 0., 1.)
        sensor.offset.convention = 'ros'
    cfg.scene.imu.prim_path = f'{root}/{variant.lidar_link}'
    cfg.scene.lidar.prim_path = f'{root}/{variant.lidar_link}'
