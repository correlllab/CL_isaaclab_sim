# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import asyncio
import gc
import omni
import omni.usd


from .realsense import CameraSpecs, RealsenseCM 

class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        """Initialize extension and UI elements"""
        self.ext_id = ext_id
        self._usd_context = omni.usd.get_context()



        #publish realsense rgb and depth data
        l_depth = CameraSpecs(
                name = "l_depth",
                cam_path = "/World/envs/env_0/Robot/h1_2_26dof_with_inspire_rev_1_0_with_CL_realsense/L_hand_base_link/CL_L_realsense/rsd455/RSD455/Camera_Pseudo_Depth",
                )
        l_rgb = CameraSpecs(
                name = "l_rgb",
                cam_path = "/World/envs/env_0/Robot/h1_2_26dof_with_inspire_rev_1_0_with_CL_realsense/L_hand_base_link/CL_L_realsense/rsd455/RSD455/Camera_OmniVision_OV9782_Color",
                )
        r_depth = CameraSpecs(
                name = "r_depth",
                cam_path = "/World/envs/env_0/Robot/h1_2_26dof_with_inspire_rev_1_0_with_CL_realsense/R_hand_base_link/CL_R_realsense/rsd455/RSD455/Camera_Pseudo_Depth",
                )
        r_rgb = CameraSpecs(
                name = "r_rgb",
                cam_path = "/World/envs/env_0/Robot/h1_2_26dof_with_inspire_rev_1_0_with_CL_realsense/R_hand_base_link/CL_R_realsense/rsd455/RSD455/Camera_OmniVision_OV9782_Color",
                )
        _ = RealsenseCM((l_depth, l_rgb, r_depth, r_rgb))

    def on_shutdown(self):
        gc.collect()

