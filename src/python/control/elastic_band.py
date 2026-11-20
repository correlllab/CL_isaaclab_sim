"""Model the elastic band the Isaac sim can hang the H1-2 from until a policy drives it.

The band is the RoboCasa sims' (golem_robocasa/mujoco_env.py): a spring-damper pulling one
body (the torso) toward a fixed anchor near its spawn. sim_main.py makes one with
--elastic_band, UnitreeMotorAction.apply_actions applies it before every physics step, and
/elastic_band/toggle (sensors/elastic_band_service.py) releases it. This module imports only
torch, so the CPU tests run it without Kit.
"""
import torch

# The RoboCasa H1-2 sim's band (h12_robot.make_band), so both sims hold the robot alike.
BAND_BODY = 'torso_link'
BAND_DROP = 0.04          # m, the anchor below the torso's spawn: the feet carry part of the weight
BAND_STIFFNESS = 1e4      # N/m
BAND_DAMPING = 2e3        # N*s/m, on the body's whole linear velocity


def band_force(position, velocity, anchor, stiffness, damping, length=0.0):
    """Return the band's world-frame force (N) on a body, per row of (n, 3) tensors.

    position (m) and velocity (m/s) are the body's, anchor (m) the band's fixed end, all in
    the world frame. The spring pulls toward the anchor beyond its rest length (m); the
    damping acts on the whole velocity, since a zero-length spring pulls in every direction.
    """
    offset = position - anchor
    distance = offset.norm(dim=-1, keepdim=True)
    direction = offset / distance.clamp_min(1e-9)
    return -stiffness * (distance - length) * direction - damping * velocity


def world_to_body(quaternion, vector):
    """Return world-frame vectors (n, 3) in the frames of bodies with world orientations quaternion (n, 4, WXYZ)."""
    w, xyz = quaternion[:, :1], quaternion[:, 1:]
    cross = torch.linalg.cross(xyz, vector)
    return vector - 2. * w * cross + 2. * torch.linalg.cross(xyz, cross)


class ElasticBand:
    """Hold one body of the robot on a spring-damper, and toggle it."""

    def __init__(self, robot, body=BAND_BODY, drop=BAND_DROP, stiffness=BAND_STIFFNESS,
                 damping=BAND_DAMPING):
        """Anchor the band drop m below body's current position, enabled.

        robot is the Isaac Lab articulation; call this once the scene is reset, with the robot
        at its spawn. Raises ValueError, listing the robot's bodies, when it has no such body.
        """
        names = list(robot.data.body_names)
        if body not in names:
            raise ValueError(f'the elastic band needs body {body!r}; the robot has {names}')
        self._robot = robot
        self._body_ids = torch.tensor([names.index(body)], device=robot.device)
        self.body = body
        self.stiffness = stiffness
        self.damping = damping
        self.anchor = robot.data.body_pos_w[:, self._body_ids[0]].clone()   # m, world, (num_envs, 3)
        self.anchor[:, 2] -= drop
        self.enabled = True
        self._applied = False   # a nonzero force is stored in the robot's wrench buffer

    def toggle(self) -> bool:
        """Enable or disable the band, log it, and return whether it is now enabled."""
        self.enabled = not self.enabled
        print(f"[isaac] elastic band {'enabled' if self.enabled else 'disabled'}", flush=True)
        return self.enabled

    def apply(self):
        """Store the band's force on its body for the next physics step.

        Its pull while enabled; on the first call after a release, zero, since Isaac Lab keeps
        a stored wrench until it is overwritten. Call it before every physics step.
        """
        if self.enabled:
            body = self._body_ids[0]
            data = self._robot.data
            force = band_force(data.body_pos_w[:, body], data.body_lin_vel_w[:, body],
                               self.anchor, self.stiffness, self.damping)
            # Into the body's frame, at its orientation now: the wrench composer rotates a
            # world-frame force with the orientation it cached at its first call, which is
            # wrong once the torso has turned.
            force = world_to_body(data.body_quat_w[:, body], force)
        elif self._applied:
            force = torch.zeros_like(self.anchor)
        else:
            return
        self._applied = self.enabled
        # Body-frame force on the body, no torque; (num_envs, 1 body, 3).
        self._robot.permanent_wrench_composer.set_forces_and_torques(
            forces=force.unsqueeze(1), torques=torch.zeros_like(force).unsqueeze(1),
            body_ids=self._body_ids, is_global=False)
