"""Receive complete Unitree commands without touching physics from DDS threads."""
import os
import torch
from .action_base import ActionProvider
from .motor_contract import ACTION_DIM, BODY_ACTION_DIM, pack_motor_command
from ..dds.common.dds_master import dds_manager


def create_action_provider(env, args):
    if args.action_source != 'dds':
        raise ValueError(f'Unsupported action source: {args.action_source}')
    return DDSActionProvider(env, args)


class DDSActionProvider(ActionProvider):
    def __init__(self, env, args_cli):
        super().__init__('DDSActionProvider')
        if args_cli.robot_type != 'h1_2':
            raise ValueError('Only h1_2 is supported')
        self.robot_dds = dds_manager.get_object('h1_2')
        self.inspire_dds = dds_manager.get_object('inspire') if args_cli.enable_inspire_dds else None
        if self.robot_dds is None:
            raise RuntimeError('H1-2 DDS bridge is unavailable')
        self._action = torch.zeros((1, ACTION_DIM), device=env.device)
        self._sequence = None
        self._last_command_time = None
        self._last_sim_time = None
        self.timeout = float(os.environ.get('GOLEM_CMD_TIMEOUT', '0.5'))
        if not 0 < self.timeout < float('inf'):
            raise ValueError('GOLEM_CMD_TIMEOUT must be finite and positive')

    def start(self):
        # Commands arrive in the DDS subscriber; this provider has no worker loop.
        self.is_running = True

    def stop(self):
        self.is_running = False

    def get_action(self, env):
        sim_time = env._sim_step_counter * env.physics_dt
        if self._last_sim_time is not None and sim_time < self._last_sim_time:
            self._last_command_time = None
        self._last_sim_time = sim_time
        command = self.robot_dds.get_robot_command()
        if command and command.get('sequence') != self._sequence:
            self._sequence = command.get('sequence')
            try:
                body = pack_motor_command(command['motor_cmd'], env.device)
                self._action[0, :BODY_ACTION_DIM].copy_(body)
                self._last_command_time = sim_time
            except (ValueError, KeyError, TypeError):
                self._last_command_time = None
        if self._last_command_time is None or sim_time - self._last_command_time > self.timeout:
            self._action[0, :BODY_ACTION_DIM].zero_()
        if self.inspire_dds is not None:
            hand = self.inspire_dds.get_inspire_hand_command()
            if hand and len(hand.get('positions', ())) == 12:
                positions = torch.tensor(hand['positions'], dtype=torch.float32, device=env.device)
                if torch.isfinite(positions).all():
                    self._action[0, BODY_ACTION_DIM:].copy_(positions)
        if getattr(env, "magpie_controller", None) is not None:
            self._action[0, BODY_ACTION_DIM:].copy_(env.magpie_controller.get_joint_targets(env.step_dt))
        return self._action

    def cleanup(self):
        self.stop()
        for bridge in (self.robot_dds, self.inspire_dds):
            if bridge is not None:
                bridge.stop_communication()
