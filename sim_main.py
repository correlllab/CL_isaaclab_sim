#!/usr/bin/env python3
"""Run the H1-2 Isaac Lab digital-twin interfaces on the simulation DDS domain."""
import argparse
from contextlib import ExitStack
import os
from pathlib import Path
import signal
import traceback


def main():
    from isaaclab.app import AppLauncher

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task', default='base')
    parser.add_argument('--robot_type', choices=['h1_2'], default='h1_2')
    parser.add_argument('--action_source', choices=['dds'], default='dds')
    parser.add_argument('--enable_inspire_dds', action='store_true')
    parser.add_argument('--hand_type', choices=['magpie', 'inspire'], default='magpie')
    parser.add_argument('--fix_base', action='store_true', help='Fix the root for bench tests; Magpie normally floats')
    parser.add_argument('--step_hz', type=float, default=None,
                        help='Wall-clock control frequency; default matches the environment step time')
    parser.add_argument('--physics_dt', type=float, default=None)
    parser.add_argument('--render_interval', type=int, default=None)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--modify_light', action='store_true')
    parser.add_argument('--modify_camera', action='store_true')
    parser.add_argument('--enable_profiling', action='store_true')
    parser.add_argument('--profile_interval', type=int, default=500)
    parser.add_argument('--max_steps', type=int, default=0,
                        help='Exit after this many environment steps (0 runs until stopped)')
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    if args.enable_inspire_dds:
        args.hand_type = 'inspire'
    args.enable_inspire_dds = args.hand_type == 'inspire'
    if args.physics_dt is not None and not 0 < args.physics_dt < float('inf'):
        parser.error('--physics_dt must be finite and positive')
    if args.step_hz is not None and not 0 < args.step_hz < float('inf'):
        parser.error('--step_hz must be finite and positive')
    if args.render_interval is not None and args.render_interval < 1:
        parser.error('--render_interval must be positive')
    if args.max_steps < 0 or args.profile_interval < 1:
        parser.error('--max_steps must be nonnegative and --profile_interval positive')
    domain = int(os.environ.get('ROS_DOMAIN_ID') or '1')
    if not 1 <= domain <= 232:
        parser.error('ROS_DOMAIN_ID must be 1..232; 0 is reserved for the real robot')
    os.environ['ROS_DOMAIN_ID'] = str(domain)
    os.environ['PROJECT_ROOT'] = str(Path(__file__).resolve().parent)

    with ExitStack() as resources:
        app = AppLauncher(args).app
        resources.callback(app.close)
        def report_exception(exc_type, exc, tb):
            if exc is not None:
                traceback.print_exception(exc_type, exc, tb)
            return False
        resources.push(report_exception)
        import gymnasium as gym
        import torch
        import tasks  # registers the local Isaac Lab environment
        from isaaclab_tasks.utils.parse_cfg import parse_env_cfg
        from src.python.control.RobotController import RobotController, ControlConfig
        from src.python.control.action_provider_dds import create_action_provider
        from src.python.dds.common.dds_create import create_dds_objects
        from src.python.sensors.ros_sensor_bridge import RosSensorBridge

        cfg = parse_env_cfg(args.task, device=args.device, num_envs=1)
        from tasks.common_config.robot_variant_config import configure_robot_variant
        configure_robot_variant(cfg, args.hand_type, args.fix_base)
        cfg.seed = args.seed
        if args.physics_dt is not None:
            cfg.sim.dt = args.physics_dt
        if args.render_interval is not None:
            cfg.sim.render_interval = args.render_interval
        if not args.enable_cameras:
            for name in ('front_camera', 'left_wrist_camera', 'right_wrist_camera'):
                setattr(cfg.scene, name, None)
        if not args.enable_inspire_dds:
            cfg.observations.policy.robot_inspire_state = None
        env = gym.make(args.task, cfg=cfg).unwrapped
        resources.callback(env.close)
        if args.modify_light or args.modify_camera:
            from tools.augmentation_utils import update_light, batch_augment_cameras_by_name
            if args.modify_light:
                update_light(prim_path='/World/light', color=(.75, .75, .75),
                             intensity=500., radius=.1, enabled=True, cast_shadows=True)
            if args.modify_camera:
                batch_augment_cameras_by_name(names=['front_cam'], focal_length=3.,
                    horizontal_aperture=22., vertical_aperture=16., exposure=.8, focus_distance=1.2)
        env.reset()
        sensors = RosSensorBridge(env.scene)
        resources.callback(sensors.close)
        if args.hand_type == 'magpie':
            from src.python.sensors.magpie_bridge import MagpieBridge
            from src.python.control.magpie_control import MagpieController
            magpie_bridge = MagpieBridge()
            resources.callback(magpie_bridge.close)
            env.magpie_controller = MagpieController(env.scene['robot'], magpie_bridge)
        dds = create_dds_objects(args, env)
        resources.callback(dds.stop_all_communication)
        controller = RobotController(env, ControlConfig(step_hz=args.step_hz or 1 / env.step_dt))
        resources.callback(controller.cleanup)
        controller.set_action_provider(create_action_provider(env, args))
        controller.set_profiling(args.enable_profiling, args.profile_interval)

        def stop(signum, frame):
            controller.stop()
        previous = {sig: signal.signal(sig, stop) for sig in (signal.SIGINT, signal.SIGTERM)}
        for sig, handler in previous.items():
            resources.callback(signal.signal, sig, handler)

        controller.start()
        print(f'[isaac] DDS domain={domain}; physics_dt={env.physics_dt}; control_dt={env.step_dt}', flush=True)
        with torch.inference_mode():
            while app.is_running() and controller.is_running:
                controller.step()
                sim_time = env._sim_step_counter * env.physics_dt
                sensors.publish(sim_time)
                if args.hand_type == 'magpie':
                    env.magpie_controller.publish(sim_time)
                if env.sim.is_stopped() or (args.max_steps and controller.step_count >= args.max_steps):
                    break
        print(f'[isaac] completed {controller.step_count} steps', flush=True)


if __name__ == '__main__':
    main()
