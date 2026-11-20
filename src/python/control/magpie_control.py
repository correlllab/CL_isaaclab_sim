"""Apply Magpie commands on the physics thread and report measured joint state."""
import torch
from .robot_variant import robot_variant

OPEN_MM = 110.
CLOSED_RAD = 2.05
LEVER_M = .05


class MagpieController:
    def __init__(self, robot, bridge):
        self.robot, self.bridge = robot, bridge
        self.ids = [robot.data.joint_names.index(n) for n in robot_variant().hand_joints]
        self.targets = robot.data.joint_pos[0, self.ids].clone()
        self.pattern = torch.tensor([1., -1., 1., -1., 1., -1.], device=robot.device)
        self._force = None
        self._pending = None

    def get_joint_targets(self, dt):
        commands = self.bridge.get_commands()
        forces = []
        for index, side in enumerate(('left', 'right')):
            command = commands[side]
            target = (OPEN_MM - command['aperture']) / OPEN_MM * CLOSED_RAD
            section = slice(index * 6, (index + 1) * 6)
            max_step = 3.14 * command['speed'] * dt
            desired = self.pattern * target
            self.targets[section] += (desired - self.targets[section]).clamp(-max_step, max_step)
            forces += [min(10., command['force'] * LEVER_M), 0., 0.] * 2
        force_tuple = tuple(forces)
        if force_tuple != self._force:
            limits = torch.tensor([forces], device=self.robot.device)
            self.robot.write_joint_effort_limit_to_sim(limits, joint_ids=self.ids)
            for actuator in self.robot.actuators.values():
                actuator.effort_limit.copy_(self.robot.data.joint_effort_limits[:, actuator.joint_indices])
            self._force = force_tuple
        self._pending = commands
        return self.targets

    def publish(self, sim_time):
        q = self.robot.data.joint_pos[0, self.ids].detach().cpu().numpy()
        dq = self.robot.data.joint_vel[0, self.ids].detach().cpu().numpy()
        effort = self.robot.data.applied_torque[0, self.ids].detach().cpu().numpy()
        target = self.targets.detach().cpu().numpy()
        states = {}
        for offset, side in ((0, 'left'), (6, 'right')):
            left, right = offset, offset + 3
            def aperture(angle):
                return max(0., min(OPEN_MM, OPEN_MM * (1. - float(angle) / CLOSED_RAD)))
            fingers = [aperture(-q[right]), aperture(q[left])]
            moving = max(abs(float(dq[left])), abs(float(dq[right]))) > .02
            force = (abs(float(effort[left])) + abs(float(effort[right]))) / (2. * LEVER_M)
            error = max(abs(float(target[left] - q[left])), abs(float(target[right] - q[right])))
            states[side] = dict(position=sum(fingers)/2., finger_positions=fingers,
                                force=force, is_moving=moving,
                                contact_detected=not moving and error > .05 and force > .5)
        self.bridge.publish(sim_time, states)
        if self._pending is not None:
            self.bridge.acknowledge(self._pending)
            self._pending = None
