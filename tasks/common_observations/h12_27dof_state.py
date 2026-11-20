# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""Name-resolved joint observations in the motor wire contract order."""
from __future__ import annotations

import time
import torch
from src.python.control.motor_contract import BODY_JOINT_NAMES


def get_robot_boy_joint_names() -> list[str]:
    return list(BODY_JOINT_NAMES)


_h12_robot_dds = None


def _get_h12_robot_dds_instance():
    """Retry late DDS initialization; the DDS manager owns cleanup."""
    global _h12_robot_dds
    if _h12_robot_dds is None:
        from src.python.dds.common.dds_master import dds_manager
        _h12_robot_dds = dds_manager.get_object("h1_2")
    return _h12_robot_dds


def get_robot_boy_joint_states(env, enable_dds: bool = True) -> torch.Tensor:
    """Return positions, velocities and torques (27 each); publish environment zero at most every 20 ms.

    The returned tensor is a reusable buffer, valid until the next observation.
    """
    robot = env.scene["robot"]
    data = robot.data
    pos, vel, torque = data.joint_pos, data.joint_vel, data.applied_torque
    signature = (id(env.scene), id(robot), tuple(data.joint_names), pos.shape,
                 pos.device, pos.dtype, vel.dtype, torque.dtype)
    cache = getattr(env, "_h12_robot_observation_cache", None)
    if cache is None or cache["signature"] != signature:
        names = list(data.joint_names)
        indices = torch.tensor([names.index(name) for name in BODY_JOINT_NAMES],
                               dtype=torch.long, device=pos.device)
        n = len(BODY_JOINT_NAMES)
        combined = torch.empty((pos.shape[0], 3 * n), device=pos.device, dtype=pos.dtype)
        cache = {"signature": signature, "indices": indices,
                 "combined": combined, "last_publish": float("-inf")}
        setattr(env, "_h12_robot_observation_cache", cache)
    n = len(BODY_JOINT_NAMES)
    combined = cache["combined"]
    pos_buf, vel_buf, torque_buf = combined[:, :n], combined[:, n:2*n], combined[:, 2*n:]
    torch.index_select(pos, 1, cache["indices"], out=pos_buf)
    torch.index_select(vel, 1, cache["indices"], out=vel_buf)
    torch.index_select(torque, 1, cache["indices"], out=torque_buf)
    if enable_dds and pos.shape[0]:
        now = time.monotonic()
        if now - cache["last_publish"] >= .020:
            try:
                interface = _get_h12_robot_dds_instance()
                if interface is not None:
                    interface.write_robot_state(
                        pos_buf[0].cpu().numpy(), vel_buf[0].cpu().numpy(),
                        torque_buf[0].cpu().numpy(), get_robot_imu_data(env)[0].cpu().numpy())
                    cache["last_publish"] = now
            except Exception as exc:
                print(f"[h1_2_state] Failed to publish state: {exc}")
    return combined

def get_robot_arm_joint_names() -> list[str]:
    return list(BODY_JOINT_NAMES[13:])


def quat_to_rot_matrix(q):
    """
    q: [B,4] assumed (w,x,y,z)
    returns R: [B,3,3] such that v_world = R @ v_body
    """
    w = q[:, 0:1]
    x = q[:, 1:2]
    y = q[:, 2:3]
    z = q[:, 3:4]

    # precompute
    ww = w * w
    xx = x * x
    yy = y * y
    zz = z * z
    wx = w * x
    wy = w * y
    wz = w * z
    xy = x * y
    xz = x * z
    yz = y * z

    # rotation matrix elements
    r00 = ww + xx - yy - zz
    r01 = 2 * (xy - wz)
    r02 = 2 * (xz + wy)

    r10 = 2 * (xy + wz)
    r11 = ww - xx + yy - zz
    r12 = 2 * (yz - wx)

    r20 = 2 * (xz - wy)
    r21 = 2 * (yz + wx)
    r22 = ww - xx - yy + zz

    R = torch.cat([
        torch.cat([r00, r01, r02], dim=1).unsqueeze(1),
        torch.cat([r10, r11, r12], dim=1).unsqueeze(1),
        torch.cat([r20, r21, r22], dim=1).unsqueeze(1),
    ], dim=1)
    return R

def reset_robot_imu_cache(env, env_ids=None):
    """Isaac reset event: discard velocity history for the affected environments."""
    cache = getattr(env, "_h12_imu_cache", None)
    if cache is None:
        return
    if env_ids is None:
        env._h12_imu_cache = None
    else:
        cache["reset_pending"][env_ids] = True


def _simulation_time(env):
    sim = getattr(env, "sim", None)
    if sim is not None and hasattr(sim, "current_time"):
        return float(sim.current_time)
    # ManagerBasedRLEnv counts physics steps even with control decimation.
    return float(env._sim_step_counter) * float(env.physics_dt)


def get_robot_imu_data(env, use_torso_imu: bool = True, quat_w_first: bool = None) -> torch.Tensor:
    """Return world position, WXYZ quaternion, proper body acceleration and gyro.

    Isaac poses are WXYZ. Velocity differences use elapsed simulation time,
    including skipped wall-clock publications. Reset rows start at gravity only.
    """
    robot = env.scene["robot"]
    data = robot.data
    imu_name = next((name for name in ("imu_in_torso", "imu_link") if name in data.body_names), None)
    if use_torso_imu and imu_name is not None:
        idx = data.body_names.index(imu_name)
        pose, velocity = data.body_link_pose_w[:, idx], data.body_link_vel_w[:, idx]
        pos, quat = pose[:, :3], pose[:, 3:7]
        lin_vel, angular_vel = velocity[:, :3], velocity[:, 3:6]
        source = idx
    else:
        root = data.root_state_w
        pos, quat, lin_vel, angular_vel = root[:, :3], root[:, 3:7], root[:, 7:10], root[:, 10:13]
        source = None
    now = _simulation_time(env)
    signature = (id(env.scene), id(robot), source, lin_vel.shape, lin_vel.device, lin_vel.dtype)
    cache = getattr(env, "_h12_imu_cache", None)
    episode = getattr(env, "episode_length_buf", None)
    acceleration = torch.zeros_like(lin_vel)
    valid = cache is not None and cache["signature"] == signature and now > cache["time"]
    if valid:
        acceleration = (lin_vel - cache["velocity"]) / (now - cache["time"])
        if episode is not None and cache["episode"] is not None:
            reset = (episode < cache["episode"]) | (episode == 0)
            acceleration[reset] = 0
    elif cache is not None and cache["signature"] == signature and now == cache["time"]:
        # Repeated observations of one physics sample should have identical IMU data.
        acceleration.copy_(cache["acceleration"])
        if episode is not None and cache["episode"] is not None:
            acceleration[episode < cache["episode"]] = 0
    if cache is not None and cache["signature"] == signature:
        acceleration[cache["reset_pending"]] = 0
    env._h12_imu_cache = {"signature": signature, "time": now,
        "reset_pending": torch.zeros(lin_vel.shape[0], dtype=torch.bool, device=lin_vel.device),
        "velocity": lin_vel.detach().clone(), "acceleration": acceleration.detach().clone(),
        "episode": None if episode is None else episode.clone()}
    proper = acceleration.clone()
    proper[:, 2] += 9.81
    # No quaternion heuristics: Isaac Lab's documented pose convention is WXYZ.
    world_to_body = quat_to_rot_matrix(quat).transpose(1, 2)
    accel_body = torch.bmm(world_to_body, proper.unsqueeze(-1)).squeeze(-1)
    gyro_body = torch.bmm(world_to_body, angular_vel.unsqueeze(-1)).squeeze(-1)
    return torch.cat((pos, quat, accel_body, gyro_body), dim=1)
