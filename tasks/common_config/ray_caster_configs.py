
from isaaclab.sensors import RayCasterCfg 
from isaaclab.utils import configclass
from isaaclab.sensors import patterns

@configclass
class RayCasterBaseCfg:
    """camera base configuration class
    
    provide the default configuration for different types of cameras, support scene-specific parameter customization
    """
    
    @classmethod
    def get_ray_caster_cfg(
        cls,
        prim_path: str = "/World/envs/env_.*/Robot/lidar_link",
        debug_vis=True,
        ray_alignment='world', 
        update_period: float = 0.02, 
        pos_offset: tuple = (0, 0.0, 0),
        rot_offset=(0.0, 0.0, 0.0, 0.0)

    ) -> RayCasterCfg:

        return RayCasterCfg(
            prim_path=prim_path,
            update_period=update_period,
            offset=RayCasterCfg.OffsetCfg(
                pos=pos_offset,
                rot=rot_offset,
            ),
            debug_vis=debug_vis,
            mesh_prim_paths=["/World/envs/env_0/Room"],
            pattern_cfg=patterns.LidarPatternCfg(
                channels=100, vertical_fov_range=[-90,90], horizontal_fov_range=[-90,90], horizontal_res=1.0

            ),
            ray_alignment=ray_alignment
        )
    



@configclass
class RayCasterPresets:

    @classmethod
    def livox_lidar(cls) -> RayCasterCfg:
        """front camera configuration"""
        return RayCasterBaseCfg.get_ray_caster_cfg()
