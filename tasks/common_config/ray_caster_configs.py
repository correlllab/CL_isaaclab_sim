"""MID360 angular approximation with tracked scene-object ray targets."""
from isaaclab.sensors import RayCasterCfg, MultiMeshRayCasterCfg, patterns
from isaaclab.utils import configclass


@configclass
class RayCasterBaseCfg:
    @classmethod
    def get_ray_caster_cfg(
        cls,
        prim_path="/World/envs/env_.*/Robot/lidar_link",
        debug_vis=False,
        ray_alignment="base",
        update_period=0.1,
        pos_offset=(0., 0., 0.),
        rot_offset=(1., 0., 0., 0.),
    ) -> MultiMeshRayCasterCfg:
        fixed = ["/World/envs/env_0/Floor"] + [f"/World/envs/env_0/Wall{i}" for i in range(1, 5)]
        fixed += [f"/World/envs/env_0/{name}" for name in ("PackingTable", "PackingTable2")]
        moving = [f"/World/envs/env_0/{color}_block" for color in ("Red", "Yellow", "Green", "Pink", "Orange", "White")]
        moving.append("/World/envs/env_0/Battery")
        return MultiMeshRayCasterCfg(
            prim_path=prim_path,
            debug_vis=debug_vis,
            update_period=update_period,
            offset=RayCasterCfg.OffsetCfg(pos=pos_offset, rot=rot_offset),
            mesh_prim_paths=fixed + [MultiMeshRayCasterCfg.RaycastTargetCfg(prim_expr=path, track_mesh_transforms=True) for path in moving],
            ray_alignment=ray_alignment,
            max_distance=40.,
            # The USD lidar_link is upright. MID360's non-repeating pattern is
            # approximated by an elevation/azimuth grid, as in the MuJoCo bridge.
            pattern_cfg=patterns.LidarPatternCfg(channels=56, vertical_fov_range=(-7., 52.), horizontal_fov_range=(0., 360.), horizontal_res=1.),
        )


@configclass
class RayCasterPresets:
    @classmethod
    def livox_lidar(cls) -> MultiMeshRayCasterCfg:
        return RayCasterBaseCfg.get_ray_caster_cfg()
