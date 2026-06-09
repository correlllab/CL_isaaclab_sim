
# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  

import gymnasium as gym

from . import CorrellTabletopEnvCfg 


gym.register(
    id="Isaac-PickPlace-Cylinder-H12-27dof-Inspire-Joint",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": CorrellTabletopEnvCfg.CorrellTabletopEnvCfg,
    },
    disable_env_checker=True,
)
