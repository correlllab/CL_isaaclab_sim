
from isaaclab.sensors import RayCasterCfg, MultiMeshRayCaster, MultiMeshRayCasterCfg 
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

    ) -> MultiMeshRayCasterCfg:
        return MultiMeshRayCasterCfg(

            prim_path=prim_path,
            debug_vis=True,
            mesh_prim_paths=["/World/GroundPlane", "/World/envs/env_0/Floor/geometry/mesh", "/World/envs/env_0/Wall1/geometry/mesh", "/World/envs/env_0/Wall2/geometry/mesh", "/World/envs/env_0/Wall3/geometry/mesh", "/World/envs/env_0/Wall4/geometry/mesh", "/World/envs/env_0/PackingTable", "/World/envs/env_0/PackingTable2", "/World/envs/env_0/Red_block", "/World/envs/env_0/Yellow_block", "/World/envs/env_0/Green_block", "/World/envs/env_0/Pink_block", "/World/envs/env_0/Orange_block", "/World/envs/env_0/White_block"],
            #mesh_prim_paths=[MultiMeshRayCasterCfg.RaycastTargetCfg(prim_expr="/World/envs/env_0/Floor"), MultiMeshRayCasterCfg.RaycastTargetCfg(prim_expr="/World/envs/env_0/Wall1"), MultiMeshRayCasterCfg.RaycastTargetCfg(prim_expr="/World/envs/env_0/Wall2"), MultiMeshRayCasterCfg.RaycastTargetCfg(prim_expr="/World/envs/env_0/Wall3"), MultiMeshRayCasterCfg.RaycastTargetCfg(prim_expr="/World/envs/env_0/Wall4"), MultiMeshRayCasterCfg.RaycastTargetCfg(prim_expr="/World/envs/env_0/PackingTable"), MultiMeshRayCasterCfg.RaycastTargetCfg(prim_expr="/World/envs/env_0/PackingTable2"), MultiMeshRayCasterCfg.RaycastTargetCfg(prim_expr="/World/envs/env_0/Red_block"), MultiMeshRayCasterCfg.RaycastTargetCfg(prim_expr="/World/envs/env_0/Yellow_block"), MultiMeshRayCasterCfg.RaycastTargetCfg(prim_expr="/World/envs/env_0/Green_block"), MultiMeshRayCasterCfg.RaycastTargetCfg(prim_expr="/World/envs/env_0/Pink_block"), MultiMeshRayCasterCfg.RaycastTargetCfg(prim_expr="/World/envs/env_0/Orange_block"), MultiMeshRayCasterCfg.RaycastTargetCfg(prim_expr="/World/envs/env_0/White_block")],
            ray_alignment="world",
            pattern_cfg=patterns.GridPatternCfg(resolution=0.02, size=(3.0, 3.0), direction=(0, 0, -1)),
        )


    #    return MultiMeshRayCasterCameraCfg(
    #    # prim_path="/World/envs/env_.*/Robot/base_link",
    #    prim_path=prim_path,
    #    update_period=update_period,
    #    #data_types=["face_ids", "normals"],
    #    offset=RayCasterCameraCfg.OffsetCfg(
    #        pos=pos_offset,
    #        #rot=(0, 0, -0.7071068, 0.7071068),
    #        rot=rot_offset,
    #        convention="ros"
    #    ),
    #    pattern_cfg=patterns.PinholeCameraPatternCfg(
    #        height=500,
    #        width=500, 
    #        focal_length=24.0,
    #        horizontal_aperture=20.955,
    #    ),
    #    mesh_prim_paths=[
    #        MultiMeshRayCasterCameraCfg.RaycastTargetCfg(
    #            target_prim_expr=env_parameters["inspection_goal_prim_path"],
    #            track_mesh_transforms=True
    #        )
    #    ],
    #    update_mesh_ids=True,
    #    debug_vis=cfg_mode.debug
    #)
        #return MultiMeshRayCasterCameraCfg(
        #    prim_path=prim_path,
        #    update_period=update_period,
        #    offset=RayCasterCameraCfg.OffsetCfg(
        #        pos=pos_offset,
        #        rot=rot_offset,
        #        convention="ros"
        #    ),
        #    debug_vis=debug_vis,
        #    mesh_prim_paths=["/World/envs/env_0/Floor", "/World/envs/env_0/Wall1", "/World/envs/env_0/Wall2", "/World/envs/env_0/Wall3", "/World/envs/env_0/Wall4", "/World/envs/env_0/PackingTable", "/World/envs/env_0/PackingTable2", "/World/envs/env_0/Red_block", "/World/envs/env_0/Yellow_block", "/World/envs/env_0/Green_block", "/World/envs/env_0/Pink_block", "/World/envs/env_0/Orange_block", "/World/envs/env_0/White_block"],
        #    pattern_cfg=patterns.LidarPatternCfg(
        #        channels=100, vertical_fov_range=[-90,90], horizontal_fov_range=[-90,90], horizontal_res=1.0

        #    ),
        #    ray_alignment=ray_alignment
        #)
    



@configclass
class RayCasterPresets:

    @classmethod
    def livox_lidar(cls) -> RayCasterCfg:
        """front camera configuration"""
        return RayCasterBaseCfg.get_ray_caster_cfg()
