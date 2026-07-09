
import isaaclab.sim as sim_utils
from isaaclab.sensors import ImuCfg 
from isaaclab.utils import configclass
import os

@configclass
class ImuBaseCfg:

    @classmethod
    def get_imu_cfg(
        cls,
        prim_path: str = "/World/envs/env_.*/Robot/imu_link",
    ) -> ImuCfg:

        #if data_types is None:
        #    data_types = ["rgb"]

        return ImuCfg(
            prim_path=prim_path,
            debug_vis=True
        )


@configclass
class ImuPresets:

    @classmethod
    def livox_imu(cls) -> ImuCfg:
        return ImuBaseCfg.get_imu_cfg("/World/envs/env_.*/Robot/imu_link")

