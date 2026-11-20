# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""Name-resolved joint observations in the motor wire contract order."""
from __future__ import annotations

import time
import torch
from src.python.control.motor_contract import HAND_JOINT_NAMES


def get_robot_girl_joint_names() -> list[str]:
    return list(HAND_JOINT_NAMES)


_inspire_dds = None


def _get_inspire_dds_instance():
    """Retry late DDS initialization; the DDS manager owns cleanup."""
    global _inspire_dds
    if _inspire_dds is None:
        from src.python.dds.common.dds_master import dds_manager
        _inspire_dds = dds_manager.get_object("inspire")
    return _inspire_dds


def get_robot_inspire_joint_states(env, enable_dds: bool = True) -> torch.Tensor:
    """Return the 12 hand positions; publish environment zero at most every 20 ms.

    The returned tensor is a reusable buffer, valid until the next observation.
    """
    robot = env.scene["robot"]
    data = robot.data
    pos, vel, torque = data.joint_pos, data.joint_vel, data.applied_torque
    signature = (id(env.scene), id(robot), tuple(data.joint_names), pos.shape,
                 pos.device, pos.dtype, vel.dtype, torque.dtype)
    cache = getattr(env, "_inspire_observation_cache", None)
    if cache is None or cache["signature"] != signature:
        names = list(data.joint_names)
        indices = torch.tensor([names.index(name) for name in HAND_JOINT_NAMES],
                               dtype=torch.long, device=pos.device)
        n = len(HAND_JOINT_NAMES)
        combined = torch.empty((pos.shape[0], 3 * n), device=pos.device, dtype=pos.dtype)
        cache = {"signature": signature, "indices": indices,
                 "combined": combined, "last_publish": float("-inf")}
        setattr(env, "_inspire_observation_cache", cache)
    n = len(HAND_JOINT_NAMES)
    combined = cache["combined"]
    pos_buf, vel_buf, torque_buf = combined[:, :n], combined[:, n:2*n], combined[:, 2*n:]
    torch.index_select(pos, 1, cache["indices"], out=pos_buf)
    torch.index_select(vel, 1, cache["indices"], out=vel_buf)
    torch.index_select(torque, 1, cache["indices"], out=torque_buf)
    if enable_dds and pos.shape[0]:
        now = time.monotonic()
        if now - cache["last_publish"] >= .020:
            try:
                interface = _get_inspire_dds_instance()
                if interface is not None:
                    interface.write_inspire_state(
                        pos_buf[0].cpu().numpy(), vel_buf[0].cpu().numpy(),
                        torque_buf[0].cpu().numpy())
                    cache["last_publish"] = now
            except Exception as exc:
                print(f"[inspire_state] Failed to publish state: {exc}")
    return pos_buf

