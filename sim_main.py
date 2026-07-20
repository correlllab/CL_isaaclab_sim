
# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
#!/usr/bin/env python3
# main.py
import sys
print(sys.stdin)
print(sys.stdin.isatty())
#breakpoint()
import os
import asyncio

import traceback
project_root = os.path.dirname(os.path.abspath(__file__))
os.environ["PROJECT_ROOT"] = project_root

import argparse
import contextlib
import time
import sys
import signal
import torch
import gymnasium as gym
from pathlib import Path
import time
from time import perf_counter

# Isaac Lab AppLauncher
from isaaclab.app import AppLauncher

from dds.dds_create import create_dds_objects
# add command line arguments
parser = argparse.ArgumentParser(description="Unitree Simulation")
parser.add_argument("--task", type=str, default="Isaac-PickPlace-G129-Head-Waist-Fix", help="task name")
parser.add_argument("--action_source", type=str, default="dds", 
                   choices=["dds", "file", "trajectory", "policy", "replay","dds_wholebody"], 
                   help="Action source")


parser.add_argument("--robot_type", type=str, default="h1_2", help="robot type")
parser.add_argument("--enable_dex1_dds", action="store_true", help="enable gripper DDS")
parser.add_argument("--enable_dex3_dds", action="store_true", help="enable dexterous hand DDS")
parser.add_argument("--enable_inspire_dds", action="store_true", help="enable inspire hand DDS")
parser.add_argument("--stats_interval", type=float, default=10.0, help="statistics print interval (seconds)")

parser.add_argument("--file_path", type=str, default="/home/unitree/Code/xr_teleoperate/teleop/utils/data", help="file path (when action_source=file)")
parser.add_argument("--generate_data_dir", type=str, default="./data", help="save data dir")
parser.add_argument("--generate_data", action="store_true", default=False, help="generate data")
parser.add_argument("--rerun_log", action="store_true", default=False, help="rerun log")
parser.add_argument("--replay_data",  action="store_true", default=False, help="replay data")

parser.add_argument("--modify_light",  action="store_true", default=False, help="modify light")
parser.add_argument("--modify_camera",  action="store_true", default=False,    help="modify camera")

# performance analysis parameters
parser.add_argument("--step_hz", type=int, default=100, help="control frequency")
parser.add_argument("--enable_profiling", action="store_true", default=True, help="enable performance analysis")
parser.add_argument("--profile_interval", type=int, default=500, help="performance analysis report interval (steps)")

parser.add_argument("--model_path", type=str, default="assets/model/policy.onnx", help="model path")
parser.add_argument("--reward_interval", type=int, default=10, help="step interval for reward calculation")
parser.add_argument("--enable_wholebody_dds", action="store_true", default=False, help="enable wh dds")

parser.add_argument("--physics_dt", type=float, default=None, help="physics time step, e.g., 0.005")
parser.add_argument("--render_interval", type=int, default=None, help="render interval steps (>=1)")
parser.add_argument("--camera_write_interval", type=int, default=None, help="camera write interval steps (>=1)")


parser.add_argument("--no_render",action="store_true",default=False,help="disable rendering updates entirely (overrides render interval)",)
parser.add_argument("--public_ip",type=str,default="127.0.0.1",help="public ip")
parser.add_argument("--livestream_type", type=int, default=2, help="livestream type (0: no livestream, 1: WebRTC public network, 2:  WebRTC private network)")

parser.add_argument("--solver_iterations", type=int, default=None, help="physx solver iteration count (e.g., 4)")
parser.add_argument("--gravity_z", type=float, default=None, help="override gravity z (e.g., -9.8)")
parser.add_argument("--skip_cvtcolor", action="store_true", default=False, help="skip cv2.cvtColor if upstream already BGR")

parser.add_argument("--camera_jpeg", action="store_true", default=True, help="enable JPEG compression for camera frames")
parser.add_argument("--camera_jpeg_quality", type=int, default=85, help="JPEG quality (1-100)")

parser.add_argument("--physx_substeps", type=int, default=None, help="physx substeps per step")
parser.add_argument("--camera_include", type=str, default="front_camera,left_wrist_camera,right_wrist_camera", help="comma-separated camera names to enable")
parser.add_argument("--camera_exclude", type=str, default="world_camera", help="comma-separated camera names to disable")

parser.add_argument("--env_reward_interval", type=int, default=5, help="environment reward compute interval (steps)")
parser.add_argument("--seed", type=int, default=42, help="environment seed")
# add AppLauncher parameters
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
if args_cli.no_render:
    os.environ["LIVESTREAM"] = str(args_cli.livestream_type)
    os.environ["PUBLIC_IP"] = args_cli.public_ip
else:
    os.environ["LIVESTREAM"] = "0"

if args_cli.enable_dex3_dds and args_cli.enable_dex1_dds and args_cli.enable_inspire_dds:
    print("Error: enable_dex3_dds and enable_dex1_dds and enable_inspire_dds cannot be enabled at the same time")
    print("Please select one of the options")
    sys.exit(1)


#import pinocchio                 
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from layeredcontrol.robot_control_system import (
    RobotController, 
    ControlConfig,
)

import tasks
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg

from tools.augmentation_utils import (
    update_light,
    batch_augment_cameras_by_name,
)
import sys
print(sys.stdin)
print(sys.stdin.isatty())
#breakpoint()

from tools.data_json_load import sim_state_to_json
#from dds.sim_state_dds import *
from action_provider.action_provider_dds import create_action_provider
from tools.get_stiffness import get_robot_stiffness_from_env
from tools.get_reward import get_step_reward_value,get_current_rewards
from PIL import Image

def setup_signal_handlers(controller,dds_manager=None,image_server=None):
    """set signal handlers"""
    def signal_handler(signum, frame):
        print(f"\nreceived signal {signum}, stopping controller...")
        try:
            controller.stop()
        except Exception as e:
            print(f"Failed to stop controller: {e}")
        try:
            if dds_manager is not None:
                dds_manager.stop_all_communication()
        except Exception as e:
            print(f"Failed to stop DDS: {e}")
        try:
            if image_server is not None:
                image_server.stop()
        except Exception as e:
            print(f"Failed to stop image server: {e}")
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)


sys.path.append("/workspace/hams/isaac/mateo_ws/CL_isaaclab_sim/src/cpp/correll-ros2-camera/build")
sys.path.append("/workspace/hams/isaac/mateo_ws/CL_isaaclab_sim/src/cpp/correll-ros2-imu/build")
sys.path.append("/workspace/hams/isaac/mateo_ws/CL_isaaclab_sim/src/cpp/correll-ros2-cloud/build")

#import ros2_point_cloud_node_py
#import ros2_camera_node_py
#import ros2_imu_node_py

def main():
    """main function"""
    # import cProfile
    # import pstats
    # import io
    # profiler = cProfile.Profile()
    # profiler.enable()
    #breakpoint()
    import os
    import atexit
    #try:
    #    os.setpgrp()
    #    #current_pgid = os.getpgrp()
    #    #print(f"Setting process group: {current_pgid}")
    #    
    #    #def cleanup_process_group():
    #    #    try:
    #    #        #print(f"Cleaning up process group: {current_pgid}")
    #    #        import signal
    #    #        #os.killpg(current_pgid, signal.SIGTERM)
    #    #    except Exception as e:
    #    #        print(f"Failed to clean up process group: {e}")
    #    #
    #    #atexit.register(cleanup_process_group)
    #    
    #except Exception as e:
    #    print(f"Failed to set process group: {e}")
    print("=" * 60)
    print("robot control system started")
    print(f"Task: {args_cli.task}")
    print(f"Action source: {args_cli.action_source}")
    print("=" * 60)

    # parse environment configuration
    try:
        env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=1)
        env_cfg.env_name = args_cli.task
    except Exception as e:
        print(f"Failed to parse environment configuration: {e}")
        return
    
    import isaacsim.core.utils.extensions as exts_utils
    # create environment
    print("\ncreate environment...")
    try:
        env_cfg.seed = args_cli.seed
        env = gym.make(args_cli.task, cfg=env_cfg).unwrapped
        env.seed(args_cli.seed)
        try:
            sensors_dict = getattr(env.scene, "sensors", {})
            if sensors_dict:
                print("Sensors in the environment:")
                for name, sensor in sensors_dict.items():
                    print(name, sensor)
                print("="*60)
        except Exception as e:
            print(f"[sim] failed to list sensors: {e}")
        print(f"\ncreate environment success ...")
        try:
            env._reward_interval = max(1, int(args_cli.env_reward_interval))
            env._reward_counter = 0
            env._reward_last = None
            print(f"[env] reward compute interval set to {env._reward_interval} steps")
        except Exception as e:
            print(f"[env] failed to set reward interval: {e}")
        if args_cli.physics_dt is not None:
            try:
                env.sim.set_substep_time(args_cli.physics_dt)
                print(f"[sim] physics dt set to {args_cli.physics_dt}")
            except Exception:
                try:
                    env.sim.dt = args_cli.physics_dt
                    print(f"[sim] physics dt assigned to env.sim.dt={args_cli.physics_dt}")
                except Exception as e:
                    print(f"[sim] failed to set physics dt: {e}")
        headless_mode = bool(getattr(args_cli, "headless", False))
        render_interval = None
        if args_cli.render_interval is not None:
            try:
                render_interval = max(1, int(args_cli.render_interval))
            except Exception as e:
                print(f"[sim] invalid render_interval value {args_cli.render_interval}: {e}")
        try:
            if args_cli.no_render:
                env.sim.render_interval = 1_000_000
                env.sim.render_mode = "offscreen"
                print("[sim] rendering disabled via --no_render")
            elif headless_mode:
                env.sim.render_mode = "offscreen"
                env.sim.render_interval = render_interval or 1
                print(f"[sim] headless offscreen rendering every {env.sim.render_interval} steps")
            elif render_interval is not None:
                env.sim.render_interval = render_interval
                print(f"[sim] render_interval set to {env.sim.render_interval}")
        except Exception as e:
            print(f"[sim] failed to configure rendering: {e}")
        #if args_cli.camera_write_interval is not None:
        #    try:
        #        import tasks.common_observations.camera_state as cam_state
        #        cam_state._camera_cache['write_interval_steps'] = max(1, int(args_cli.camera_write_interval))
        #        print(f"[camera] write interval steps set to {cam_state._camera_cache['write_interval_steps']}")
        #    except Exception as e:
        #        print(f"[camera] failed to set write interval: {e}")

        #try:
        #    if args_cli.solver_iterations is not None:
        #        env.sim.physx.solver_iteration_count = int(args_cli.solver_iterations)
        #        print(f"[sim] solver_iteration_count={env.sim.physx.solver_iteration_count}")
        #    if args_cli.physx_substeps is not None:
        #        try:
        #            env.sim.physx.substeps = int(args_cli.physx_substeps)
        #        except Exception:
        #            try:
        #                env.sim.set_substeps(int(args_cli.physx_substeps))
        #            except Exception:
        #                pass
        #        print(f"[sim] physx_substeps set to {args_cli.physx_substeps}")
        #    if args_cli.gravity_z is not None:
        #        g = float(args_cli.gravity_z)
        #        env.sim.physx.gravity = (0.0, 0.0, g)
        #        print(f"[sim] gravity set to {env.sim.physx.gravity}")
        #except Exception as e:
        #    print(f"[sim] failed to set physx params: {e}")
        #if args_cli.skip_cvtcolor:
        #    os.environ["CAMERA_SKIP_CVTCOLOR"] = "1"
        try:
            import tasks.common_observations.camera_state as cam_state
            enable_jpeg = bool(args_cli.camera_jpeg) or (os.getenv("CAMERA_JPEG") == "1")
            jpeg_quality = int(args_cli.camera_jpeg_quality if args_cli.camera_jpeg else os.getenv("CAMERA_JPEG_QUALITY", args_cli.camera_jpeg_quality))
            cam_state.set_writer_options(enable_jpeg=enable_jpeg, jpeg_quality=jpeg_quality, skip_cvtcolor=args_cli.skip_cvtcolor)
            include = [n.strip() for n in (args_cli.camera_include or "").split(',') if n.strip()]
            exclude = [n.strip() for n in (args_cli.camera_exclude or "").split(',') if n.strip()]
            try:
                cam_state.set_camera_allowlist(include)
            except Exception:
                pass
            try:
                sensors_dict = getattr(env.scene, "sensors", {})
                for name, sensor in sensors_dict.items():
                    lname = name.lower()
                    if "camera" not in lname:
                        continue
                    if exclude and name in exclude:
                        for attr_name, value in [("enabled", False), ("is_enabled", False)]:
                            if hasattr(sensor, attr_name):
                                try:
                                    setattr(sensor, attr_name, value)
                                except Exception:
                                    pass
                        for meth in ("set_active", "disable", "pause"):
                            if hasattr(sensor, meth):
                                try:
                                    getattr(sensor, meth)(False)
                                except Exception:
                                    pass
                        for attr_name in ("update_period", "_update_period"):
                            if hasattr(sensor, attr_name):
                                try:
                                    setattr(sensor, attr_name, 1e6)
                                except Exception:
                                    pass
                    elif include and name not in include:
                        for attr_name in ("update_period", "_update_period"):
                            if hasattr(sensor, attr_name):
                                try:
                                    setattr(sensor, attr_name, 1e6)
                                except Exception:
                                    pass
            except Exception as e:
                print(f"[camera] failed to tune sensors: {e}")
        except Exception as e:
            print(f"[camera] failed to apply writer options: {e}")
    except Exception as e:
        print(f"\nFailed to create environment: {e}")
        return
    
    # get robot stiffness and damping parameters from runtime environment
    print("\n" + "="*60)
    print("🔍 Getting robot stiffness and damping parameters from runtime environment")
    print("="*60)
    
    try:
        stiffness_data = get_robot_stiffness_from_env(env)
        if stiffness_data:
            print("✅ Successfully got robot parameters!")
        else:
            print("⚠️ Failed to get robot parameters, will try again after environment reset")
    except Exception as e:
        print(f"⚠️ Error getting robot parameters: {e}")
    
    print("="*60)
    
    if not getattr(args_cli, "headless", False) and not args_cli.no_render:
        print("\n")
        print("***  Please left-click on the Sim window to activate rendering. ***")
        print("\n")
    else:
        print("\n")
        print("***  Running without GUI; rendering handled offscreen. ***")
        print("\n")
    # reset environment
    if args_cli.modify_light:
        update_light(
            prim_path="/World/light",
            color=(0.75, 0.75, 0.75),
            intensity=500.0,
            # position=(1.0, 2.0, 3.0),
            radius=0.1,
            enabled=True,
            cast_shadows=True
        )
    if args_cli.modify_camera:
        batch_augment_cameras_by_name(
            names=["front_cam"],
            focal_length=3.0,
            horizontal_aperture=22.0,
            vertical_aperture=16.0,
            exposure=0.8,                
            focus_distance=1.2
        )
    env.sim.reset()
    env.reset()
    
    # create simplified control configuration
    try:    
        control_config = ControlConfig(
            step_hz=args_cli.step_hz,
            replay_mode=args_cli.replay_data
        )
    except Exception as e:
        print(f"Failed to create control configuration: {e}")
        return
    
    # create controller

    if not args_cli.replay_data:
        print("========= create image server =========")
        try:
            #image_server = run_isaacsim_server()
            pass
        except Exception as e:
            print(f"Failed to create image server: {e}")
            return
        print("========= create image server success =========")
        print("========= create dds =========")
        try:
            dds_manager = create_dds_objects(args_cli,env)
            pass
        except Exception as e:
            print(f"Failed to create dds: {e}")
            return
        print("========= create dds success =========")
    else:
        print("========= create dds =========")
        #try:
        #    create_dds_objects_replay(args_cli,env)
        #except Exception as e:
        #    print(f"Failed to create dds: {e}")
        #    return
        #print("========= create dds success =========")
        #from tools.data_json_load import get_data_json_list
        #print("========= get data json list =========")
        #data_idx=0
        #data_json_list = get_data_json_list(args_cli.file_path)
        #if args_cli.action_source != "replay":
        #    args_cli.action_source = "replay"
        #print("========= get data json list success =========")
    # create action provider
    
    print(f"\ncreate action provider: {args_cli.action_source}...")
    try:
        print(f"args_cli.task: {args_cli.task}")
        if not args_cli.replay_data and ("Wholebody" in args_cli.task or args_cli.enable_wholebody_dds):
            args_cli.action_source = "dds_wholebody"
            args_cli.enable_wholebody_dds = True
            control_config.use_rl_action_mode = True
        action_provider = create_action_provider(env,args_cli)
        if action_provider is None:
            print("action provider creation failed, exiting")
            return
    except Exception as e:
        print(f"Failed to create action provider: {e}")
        return
    
    # set action provider
    print("========= create controller =========")
    controller = RobotController(env, control_config)
    controller.set_action_provider(action_provider)
    print("========= create controller success =========")
    
    # configure performance analysis
    if args_cli.enable_profiling:
        controller.set_profiling(True, args_cli.profile_interval)
        print(f"performance analysis enabled, report every {args_cli.profile_interval} steps")
    else:
        controller.set_profiling(False)
        print("performance analysis disabled")


    # set signal handlers
    if not args_cli.replay_data:
        setup_signal_handlers(controller, dds_manager)
        #setup_signal_handlers(controller,dds_manager,image_server)
    else:
        setup_signal_handlers(controller)
        
    print("Note: The DDS in Sim transmits messages on channel 1. Please ensure that other DDS instances use the same channel for message exchange by setting: ChannelFactoryInitialize(1).")
    try:
        # start controller - start asynchronous components
        print("========= start controller =========")
        controller.start()
        print("========= start controller success =========")
        
        ## main loop - execute in main thread to support rendering
        #last_stats_time = time.time()
        #loop_start_time = time.time()
        #loop_count = 0
        #last_loop_time = time.time()
        #recent_loop_times = []  # for calculating moving average frequency
        #
        #
        #reward_interval = max(1, args_cli.reward_interval)

        #ros2_camera_node_py.spin_rclcpp()
        #camera_node = ros2_camera_node_py.ros2_camera_node(["/realsense/left_hand/aligned_depth_to_color/image_raw/compressed", "/realsense/right_hand/aligned_depth_to_color/image_raw/compressed","/realsense/left_hand/color/image_raw/compressed", "/realsense/right_hand/color/image_raw/compressed"])
        #cloud_node = ros2_point_cloud_node_py.ros2_point_cloud_node(["/lidar/first_link"])
        #imu_node = ros2_imu_node_py.ros2_imu_node("/imu/base_link")
        #breakpoint()
        with contextlib.suppress(KeyboardInterrupt), torch.inference_mode():

            while simulation_app.is_running() and controller.is_running:
                #breakpoint()
                #current_time = time.time()
                #loop_count += 1
                if not args_cli.replay_data:
                    try:
                        env_state = env.scene.get_state()
                        env_state_json =  sim_state_to_json(env_state)
                        sim_state = {"init_state":env_state_json,"task_name":args_cli.task}
                    except Exception as e:
                        print(f"Failed to get env state: {e}")
                        raise e
                    #try:

                    #    start = perf_counter()
                    #    cloud_data = env.scene['lidar']._data.ray_hits_w.flatten().cpu()
                    #    cloud_node.push_data_to_deque(cloud_data.data_ptr(), cloud_data.shape[0], "/lidar/first_link")

                    #    imu_node.push_data_to_deque(imu_data.quat_w.cpu().tolist()[0], [0, 0, 0, 0, 0, 0, 0, 0, 0], imu_data.ang_vel_b.cpu().tolist()[0], [0, 0, 0, 0, 0, 0, 0, 0, 0], imu_data.lin_acc_b.cpu().tolist()[0], [0, 0, 0, 0, 0, 0, 0, 0, 0])
                    #    elapsed = perf_counter() - start

                    #    print(f"POINT CLOUD PUBLISHING: {elapsed*1000:.3f} ms")
                    #except Exception as e:
                    #    print(f"faled to publish cloud data")

                    #try:

                    #    imu_data = env.scene['imu']._data
                    #    if not imu_data.quat_w[0][0] == 1:
                    #        start = perf_counter()

                    #        imu_node.push_data_to_deque(imu_data.quat_w.cpu().tolist()[0], [0, 0, 0, 0, 0, 0, 0, 0, 0], imu_data.ang_vel_b.cpu().tolist()[0], [0, 0, 0, 0, 0, 0, 0, 0, 0], imu_data.lin_acc_b.cpu().tolist()[0], [0, 0, 0, 0, 0, 0, 0, 0, 0])
                    #        elapsed = perf_counter() - start
                    #        print(f"IMU PUBLISHING: {elapsed*1000:.3f} ms")

                    #except Exception as e:
                    #    print(e)
                    #try:


                    #    start = perf_counter()
                    #    rgb_data = env.scene['left_wrist_camera']._data.output['rgb'].reshape((3, 480, 640)).cpu()
                    #    camera_node.push_data_to_deque(rgb_data.data_ptr(), rgb_data.shape[-1], rgb_data.shape[-2], "/realsense/left_hand/color/image_raw/compressed", "INTERLEAVED")

                    #    elapsed = perf_counter() - start
                    #    print(f"LEFT_RGB PUBLISHING: {elapsed*1000:.3f} ms")
                    #except Exception as e:
                    #    print(e)

                    #try:

                    #    start = perf_counter()
                    #    rgb_data = env.scene['right_wrist_camera']._data.output['rgb'].reshape((3, 480, 640)).cpu()
                    #    camera_node.push_data_to_deque(rgb_data.data_ptr(), rgb_data.shape[-1], rgb_data.shape[-2], "/realsense/right_hand/color/image_raw/compressed", "INTERLEAVED")

                    #    elapsed = perf_counter() - start
                    #    print(f"RIGHT_RGB PUBLISHING: {elapsed*1000:.3f} ms")
                    #except Exception as e:
                    #    print(e)

                    #try:

                    #    depth_data = env.scene['left_wrist_camera']._data.output['distance_to_image_plane'].reshape((1, 480, 640)).cpu()
                    #    valid_mask = torch.isfinite(depth_data)
                    #    valid_values = depth_data[valid_mask]
                    #    vmin, vmax = torch.min(valid_values), torch.max(valid_values)
                    #    depth_clean = torch.where(valid_mask, depth_data, vmin)
                    #    normalized = (depth_clean - vmin) / (vmax - vmin)
                    #    depth_u8 = (normalized * 255).to(torch.uint8)
                    #    camera_node.push_data_to_deque(depth_u8.data_ptr(), depth_u8.shape[-1], depth_u8.shape[-2], "/realsense/left_hand/aligned_depth_to_color/image_raw/compressed", "INTERLEAVED")
                    #except Exception as e:
                    #    print(e)

                    #try:

                    #    depth_data = env.scene['right_wrist_camera']._data.output['distance_to_image_plane'].reshape((1, 480, 640)).cpu()
                    #    valid_mask = torch.isfinite(depth_data)
                    #    valid_values = depth_data[valid_mask]
                    #    vmin, vmax = torch.min(valid_values), torch.max(valid_values)
                    #    depth_clean = torch.where(valid_mask, depth_data, vmin)
                    #    normalized = (depth_clean - vmin) / (vmax - vmin)
                    #    depth_u8 = (normalized * 255).to(torch.uint8)
                    #    camera_node.push_data_to_deque(depth_u8.data_ptr(), depth_u8.shape[-1], depth_u8.shape[-2], "/realsense/right_hand/aligned_depth_to_color/image_raw/compressed", "INTERLEAVED")
                    #except Exception as e:
                    #    print(e)
                     
                else:
                    pass
                controller.step()
                ## check environment state
                if env.sim.is_stopped():
                    print("\nenvironment stopped")
                    break
    except KeyboardInterrupt:
        print("\nuser interrupted program")
    
    except Exception as e:
        print(f"\nprogram exception: {e}")
    
    finally:
        # clean up resources
        print("\nclean up resources...")
        controller.cleanup()
        image_server.stop()
        env.close()
        print("cleanup completed")
    # profiler.disable()
    # s = io.StringIO()
    # ps = pstats.Stats(profiler, stream=s).strip_dirs().sort_stats("time")
    # ps.print_stats(30)  

    # print(s.getvalue())

if __name__ == "__main__":
    try:
        main()
    finally:
        print("Performing final cleanup...")
        
        ## Get current process information
        #import os
        #import subprocess
        #import signal
        #import time
        #
        #current_pid = os.getpid()
        #print(f"Current main process PID: {current_pid}")
        #
        #try:
        #    # Find all related Python processes
        #    result = subprocess.run(['pgrep', '-f', 'sim_main.py'], 
        #                          capture_output=True, text=True)
        #    if result.returncode == 0:
        #        pids = result.stdout.strip().split('\n')
        #        print(f"Found related processes: {pids}")
        #        
        #        for pid in pids:
        #            if pid and pid != str(current_pid):
        #                try:
        #                    print(f"Terminating child process: {pid}")
        #                    os.kill(int(pid), signal.SIGTERM)
        #                except ProcessLookupError:
        #                    print(f"Process {pid} does not exist")
        #                except Exception as e:
        #                    print(f"Failed to terminate process {pid}: {e}")
        #        
        #        # Wait for processes to exit
        #        time.sleep(2)
        #        
        #        # Check if there are any remaining processes, force kill them
        #        result2 = subprocess.run(['pgrep', '-f', 'sim_main.py'], 
        #                               capture_output=True, text=True)
        #        if result2.returncode == 0:
        #            remaining_pids = result2.stdout.strip().split('\n')
        #            for pid in remaining_pids:
        #                if pid and pid != str(current_pid):
        #                    try:
        #                        print(f"Force killing process: {pid}")
        #                        os.kill(int(pid), signal.SIGKILL)
        #                    except Exception as e:
        #                        print(f"Failed to force kill process {pid}: {e}")
        #                        
        #except Exception as e:
        #    print(f"Error during process cleanup: {e}")
        #
        #try:
        #    simulation_app.close()
        #except Exception as e:
        #    print(f"Failed to close simulation application: {e}")
        #    
        #print("Program exit completed")
        #
        ## Force exit
        os._exit(0)
