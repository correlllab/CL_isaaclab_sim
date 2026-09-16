"""Isaac Lab action term implementing the Unitree body servo and Inspire linkage."""
import torch
from isaaclab.managers import ActionTerm, ActionTermCfg
from isaaclab.utils import configclass
from .motor_contract import BODY_JOINT_NAMES, HAND_JOINT_NAMES, ACTION_DIM, BODY_ACTION_DIM


class UnitreeMotorAction(ActionTerm):
    """Apply the complete motor command through PhysX implicit drives and Inspire linkage."""
    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        names = self._asset.data.joint_names
        self._body_ids = [names.index(name) for name in BODY_JOINT_NAMES]
        self._hand_ids = [names.index(name) for name in HAND_JOINT_NAMES]
        self._coupled_ids, sources, scales = [], [], []
        for side, offset in (('R', 0), ('L', 6)):
            for name, source, scale in (('pinky_intermediate',0,1.), ('ring_intermediate',1,1.),
                                        ('middle_intermediate',2,1.), ('index_intermediate',3,1.),
                                        ('thumb_intermediate',4,1.5), ('thumb_distal',4,2.4)):
                self._coupled_ids.append(names.index(f'{side}_{name}_joint'))
                sources.append(offset+source)
                scales.append(scale)
        self._hand_sources = torch.tensor(sources, device=self.device)
        self._hand_scales = torch.tensor(scales, device=self.device)
        self._raw_actions = torch.zeros(self.num_envs, ACTION_DIM, device=self.device)
        self._kp_cache = torch.full((self.num_envs, len(BODY_JOINT_NAMES)), float("nan"), device=self.device)
        self._kd_cache = torch.full_like(self._kp_cache, float("nan"))

    @property
    def action_dim(self):
        return ACTION_DIM

    @property
    def raw_actions(self):
        return self._raw_actions

    @property
    def processed_actions(self):
        return self._raw_actions

    def process_actions(self, actions):
        self._raw_actions.copy_(actions)
        command = self._raw_actions[:, :BODY_ACTION_DIM].reshape(-1, 6, len(BODY_JOINT_NAMES))
        enabled = command[:, 5] == 1
        kp = torch.where(enabled, command[:, 3], 0.)
        kd = torch.where(enabled, command[:, 4], 0.)
        # PhysX integrates spring/damping inside each physics solve. Explicit
        # held torques are unstable for the wrist inertia at the 5 ms timestep.
        # Gain writes cross GPU/CPU, so only perform them when a command changes.
        if not torch.equal(kp, self._kp_cache):
            self._asset.write_joint_stiffness_to_sim(kp, joint_ids=self._body_ids)
            self._kp_cache.copy_(kp)
        if not torch.equal(kd, self._kd_cache):
            self._asset.write_joint_damping_to_sim(kd, joint_ids=self._body_ids)
            self._kd_cache.copy_(kd)
        # Isaac's write_*_to_sim methods do not update actuator model buffers;
        # mirror gains so applied_torque/LowState estimates use the same servo.
        for actuator in self._asset.actuators.values():
            actuator.stiffness.copy_(self._asset.data.joint_stiffness[:, actuator.joint_indices])
            actuator.damping.copy_(self._asset.data.joint_damping[:, actuator.joint_indices])

    def apply_actions(self):
        command = self._raw_actions[:, :BODY_ACTION_DIM].reshape(-1, 6, len(BODY_JOINT_NAMES))
        self._asset.set_joint_position_target(command[:, 0], joint_ids=self._body_ids)
        self._asset.set_joint_velocity_target(command[:, 1], joint_ids=self._body_ids)
        effort = torch.where(command[:, 5] == 1, command[:, 2], 0.)
        self._asset.set_joint_effort_target(effort, joint_ids=self._body_ids)
        hands = self._raw_actions[:, BODY_ACTION_DIM:]
        self._asset.set_joint_position_target(hands, joint_ids=self._hand_ids)
        self._asset.set_joint_position_target(hands[:, self._hand_sources] * self._hand_scales,
                                             joint_ids=self._coupled_ids)

    def reset(self, env_ids=None):
        ids = slice(None) if env_ids is None else env_ids
        self._raw_actions[ids] = 0
        self._kp_cache[ids] = float("nan")
        self._kd_cache[ids] = float("nan")


@configclass
class UnitreeMotorActionCfg(ActionTermCfg):
    class_type: type = UnitreeMotorAction
    asset_name: str = 'robot'
