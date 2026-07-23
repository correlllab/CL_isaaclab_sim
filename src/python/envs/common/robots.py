# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0  
"""Configuration for Unitree robots."""

import isaaclab.sim as sim_utils
from isaaclab.actuators import ActuatorNetMLPCfg, DCMotorCfg, ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR
import os
project_root = os.environ.get("PROJECT_ROOT")

H12_CFG_WITH_INSPIRE_HAND = ArticulationCfg(

    #articulation_root_prim_path="/Robot",
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"/workspace/hams/isaac/mateo_ws/CL_Assets/isaac_assets/robots/h1_2-26dof-inspire-base-fix-usd/h1_2_26dof_with_inspire_rev_1_0.usd",
        #usd_path=f"/workspace/hams/isaac/Robot.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            kinematic_enabled=False,
            disable_gravity=False,
            retain_accelerations=False,  # 启用加速度计算 (Enable acceleration computation)
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, 
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=1,
        ),

    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.75),
        joint_pos={
            # legs joints
            "left_hip_yaw_joint": 0.0,
            "left_hip_roll_joint": 0.0,
            "left_hip_pitch_joint": -0.05,
            "left_knee_joint": 0.2,
            "left_ankle_pitch_joint": -0.15,
            "left_ankle_roll_joint": 0.0,
            
            "right_hip_yaw_joint": 0.0,
            "right_hip_roll_joint": 0.0,
            "right_hip_pitch_joint": -0.05,
            "right_knee_joint": 0.2,
            "right_ankle_pitch_joint": -0.15,
            "right_ankle_roll_joint": 0.0,
            
            
            # arms joints
            "left_shoulder_pitch_joint": 0.0,
            "left_shoulder_roll_joint": 0.0,
            "left_shoulder_yaw_joint": 0.0,
            "left_elbow_joint": 0.0,
            "left_wrist_roll_joint": 0.0,
            "left_wrist_pitch_joint": 0.0,
            "left_wrist_yaw_joint": 0.0,
            
            "right_shoulder_pitch_joint": 0.0,
            "right_shoulder_roll_joint": 0.0,
            "right_shoulder_yaw_joint": 0.0,
            "right_elbow_joint": 0.0,
            "right_wrist_roll_joint": 0.0,
            "right_wrist_pitch_joint": 0.0,
            "right_wrist_yaw_joint": 0.0,
            #torso
            "torso_joint": 0.0,
            
            # fingers joints
            "L_index_proximal_joint": 0.0,
            "L_index_intermediate_joint": 0.0,
            "L_middle_proximal_joint": 0.0,
            "L_middle_intermediate_joint": 0.0,
            "L_pinky_proximal_joint":0.0,
            "L_pinky_intermediate_joint":0.0,
            "L_ring_proximal_joint":0.0,
            "L_ring_intermediate_joint":0.0,
            "L_thumb_proximal_yaw_joint":0.0,
            "L_thumb_proximal_pitch_joint":0.0,
            "L_thumb_intermediate_joint":0.0,
            "L_thumb_distal_joint":0.0,

            "R_index_proximal_joint": 0.0,
            "R_index_intermediate_joint": 0.0,
            "R_middle_proximal_joint": 0.0,
            "R_middle_intermediate_joint": 0.0,
            "R_pinky_proximal_joint":0.0,
            "R_pinky_intermediate_joint":0.0,
            "R_ring_proximal_joint":0.0,
            "R_ring_intermediate_joint":0.0,
            "R_thumb_proximal_yaw_joint":0.0,
            "R_thumb_proximal_pitch_joint":0.0,
            "R_thumb_intermediate_joint":0.0,
            "R_thumb_distal_joint":0.0,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.95,
    actuators={
        "torso": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*torso.*", 
            ],
            effort_limit_sim={
                ".*torso.*": 200.0,
            },
            velocity_limit_sim={
                ".*torso.*": 23.0,
            },
            stiffness={
                ".*torso.*": 160.0,
            },
            damping={
                ".*torso.*": 10.0,
            },
        ),
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_hip_yaw_joint", 
                ".*_hip_roll_joint",
                ".*_hip_pitch_joint", 
                ".*_knee_joint",
            ],
            effort_limit_sim={
                ".*_hip_yaw_joint": 100000,
                ".*_hip_roll_joint": 100000,
                ".*_hip_pitch_joint": 100000,
                ".*_knee_joint": 1100000,
            },
            velocity_limit_sim={
                ".*_hip_yaw_joint": 100000,
                ".*_hip_roll_joint": 100000,
                ".*_hip_pitch_joint": 100000,
                ".*_knee_joint": 100000,
            },
            stiffness={
                ".*_hip_yaw_joint": 200.0,
                ".*_hip_roll_joint": 200.0,
                ".*_hip_pitch_joint": 200.0,
                ".*_knee_joint": 300.0,
            },
            damping={
                ".*_hip_yaw_joint": 2.5,
                ".*_hip_roll_joint": 2.5,
                ".*_hip_pitch_joint": 2.5,
                ".*_knee_joint": 4.0,
                #".*waist.*": 0.0,
            },
        ),
        "feet": ImplicitActuatorCfg(
            #effort_limit=None,
            joint_names_expr=[".*_ankle_pitch_joint", ".*_ankle_roll_joint"],
            #stiffness=None,
            #damping=None,
            # armature=0.001,
            effort_limit_sim={
                ".*_ankle_pitch_joint": 100000,
                ".*_ankle_roll_joint": 100000,
            },
            velocity_limit_sim={
                ".*_ankle_pitch_joint": 1000000,
                ".*_ankle_roll_joint": 1000000,
            },
            stiffness={
                ".*":2.0
            },
            damping={
                ".*":2.0
            }
        ),
        "arms": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_shoulder_.*_joint",
                ".*_elbow_joint",
                ".*_wrist_.*_joint"
            ],
            effort_limit=None,
            velocity_limit= {
                ".*shoulder_roll_joint.*": 9.0,
                ".*shoulder_pitch_joint.*": 9.0,
                ".*shoulder_yaw_joint.*": 20.0


            },
            stiffness={  # increase the stiffness (kp)
                ".*_shoulder_roll_joint": 200.0,
                ".*_shoulder_pitch_joint": 240.0,
                ".*_shoulder_yaw_joint": 150.0,
                ".*_elbow_joint": 150.0,
                ".*_wrist_roll_joint": 120.0,
                ".*_wrist_pitch_joint": 120.0,
                ".*_wrist_yaw_joint": 120.0,
            },
            damping={    # increase the damping (kd)
                ".*_shoulder_.*_joint": 12.0,
                ".*_elbow_joint": 12.0,
                ".*_wrist_.*_joint": 12.0,
             }
        ),
        "hands": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_index_proximal_joint",
                ".*_index_intermediate_joint",
                ".*_middle_proximal_joint",
                ".*_middle_intermediate_joint",
                ".*_pinky_proximal_joint",
                ".*_pinky_intermediate_joint",
                ".*_ring_proximal_joint",
                ".*_ring_intermediate_joint",
                ".*_thumb_proximal_yaw_joint",
                ".*_thumb_proximal_pitch_joint",
                ".*_thumb_intermediate_joint",
                ".*_thumb_distal_joint",
            ],
            effort_limit=100.0,
            velocity_limit=50,
            stiffness={
                ".*_index_proximal_joint":1000.0,
                ".*_index_intermediate_joint":1000.0,
                ".*_middle_proximal_joint":1000.0,
                ".*_middle_intermediate_joint":1000.0,
                ".*_pinky_proximal_joint":1000.0,
                ".*_pinky_intermediate_joint":1000.0,
                ".*_ring_proximal_joint":1000.0,
                ".*_ring_intermediate_joint":1000.0,
                ".*_thumb_proximal_yaw_joint":1000.0,
                ".*_thumb_proximal_pitch_joint":1000.0,
                ".*_thumb_intermediate_joint":1000.0,
                ".*_thumb_distal_joint":1000.0,
            },
            damping={
                ".*_index_proximal_joint":15,
                ".*_index_intermediate_joint":15,
                ".*_middle_proximal_joint":15,
                ".*_middle_intermediate_joint":15,
                ".*_pinky_proximal_joint":15,
                ".*_pinky_intermediate_joint":15,
                ".*_ring_proximal_joint":15,
                ".*_ring_intermediate_joint":15,
                ".*_thumb_proximal_yaw_joint":15,
                ".*_thumb_proximal_pitch_joint":15,
                ".*_thumb_intermediate_joint":15,
                ".*_thumb_distal_joint":15,
            },
            armature={
                ".*": 0.0
            },
        ),

        #"hands": ImplicitActuatorCfg(
        #    joint_names_expr=[
        #        ".*_index_proximal_joint",
        #        ".*_index_intermediate_joint",
        #        ".*_middle_proximal_joint",
        #        ".*_middle_intermediate_joint",
        #        ".*_pinky_proximal_joint",
        #        ".*_pinky_intermediate_joint",
        #        ".*_ring_proximal_joint",
        #        ".*_ring_intermediate_joint",
        #        ".*_thumb_proximal_yaw_joint",
        #        ".*_thumb_proximal_pitch_joint",
        #        ".*_thumb_intermediate_joint",
        #        ".*_thumb_distal_joint",
        #    ],
        #    effort_limit=10000000000000000.0,
        #    velocity_limit=5000000000000000000000,
        #    stiffness={
        #        ".*_index_proximal_joint": 0.001,
        #        ".*_index_intermediate_joint": 0.001,
        #        ".*_middle_proximal_joint": 0.001,
        #        ".*_middle_intermediate_joint": 0.001,
        #        ".*_pinky_proximal_joint": 0.001,
        #        ".*_pinky_intermediate_joint": 0.001,
        #        ".*_ring_proximal_joint": 0.001,
        #        ".*_ring_intermediate_joint": 0.001,
        #        ".*_thumb_proximal_yaw_joint": 0.001,
        #        ".*_thumb_proximal_pitch_joint": 0.001,
        #        ".*_thumb_intermediate_joint": 0.001,
        #        ".*_thumb_distal_joint": 0.001,
        #    },
        #    damping={
        #        ".*_index_proximal_joint": 0.001,
        #        ".*_index_intermediate_joint": 0.001,
        #        ".*_middle_proximal_joint": 0.001,
        #        ".*_middle_intermediate_joint": 0.001,
        #        ".*_pinky_proximal_joint": 0.001,
        #        ".*_pinky_intermediate_joint": 0.001,
        #        ".*_ring_proximal_joint": 0.001,
        #        ".*_ring_intermediate_joint": 0.001,
        #        ".*_thumb_proximal_yaw_joint": 0.001,
        #        ".*_thumb_proximal_pitch_joint": 0.001,
        #        ".*_thumb_intermediate_joint": 0.001,
        #        ".*_thumb_distal_joint": 0.001,
        #    },
        #    armature={
        #        ".*": 0.001
        #    },
        #),

    },
)

#H12_CFG_WITH_INSPIRE_HAND = ArticulationCfg(
#    spawn=sim_utils.UsdFileCfg(
#        usd_path=f"/workspace/hams/isaac/mateo_ws/CL_Assets/isaac_assets/robots/h1_2-26dof-inspire-base-fix-usd/h1_2_26dof_with_inspire_rev_1_0.usd",
#        activate_contact_sensors=True,
#        rigid_props=sim_utils.RigidBodyPropertiesCfg(
#            kinematic_enabled=False,
#            disable_gravity=False,
#            retain_accelerations=True,  # 启用加速度计算 (Enable acceleration computation)
#            linear_damping=0.0,
#            angular_damping=0.0,
#            max_linear_velocity=1000.0,
#            max_angular_velocity=1000.0,
#            max_depenetration_velocity=1.0,
#        ),
#        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
#            enabled_self_collisions=True, 
#            solver_position_iteration_count=8,
#            solver_velocity_iteration_count=4
#        ),
#
#    ),
#    init_state=ArticulationCfg.InitialStateCfg(
#        pos=(0.0, 0.0, 0.75),
#        joint_pos={
#            # legs joints
#            "left_hip_yaw_joint": 0.0,
#            "left_hip_roll_joint": 0.0,
#            "left_hip_pitch_joint": -0.05,
#            "left_knee_joint": 0.2,
#            "left_ankle_pitch_joint": -0.15,
#            "left_ankle_roll_joint": 0.0,
#            
#            "right_hip_yaw_joint": 0.0,
#            "right_hip_roll_joint": 0.0,
#            "right_hip_pitch_joint": -0.05,
#            "right_knee_joint": 0.2,
#            "right_ankle_pitch_joint": -0.15,
#            "right_ankle_roll_joint": 0.0,
#            
#            
#            # arms joints
#            "left_shoulder_pitch_joint": 0.0,
#            "left_shoulder_roll_joint": 0.0,
#            "left_shoulder_yaw_joint": 0.0,
#            "left_elbow_joint": 0.0,
#            "left_wrist_roll_joint": 0.0,
#            "left_wrist_pitch_joint": 0.0,
#            "left_wrist_yaw_joint": 0.0,
#            
#            "right_shoulder_pitch_joint": 0.0,
#            "right_shoulder_roll_joint": 0.0,
#            "right_shoulder_yaw_joint": 0.0,
#            "right_elbow_joint": 0.0,
#            "right_wrist_roll_joint": 0.0,
#            "right_wrist_pitch_joint": 0.0,
#            "right_wrist_yaw_joint": 0.0,
#            
#            # fingers joints
#            "L_index_proximal_joint": 0.0,
#            "L_index_intermediate_joint": 0.0,
#            "L_middle_proximal_joint": 0.0,
#            "L_middle_intermediate_joint": 0.0,
#            "L_pinky_proximal_joint":0.0,
#            "L_pinky_intermediate_joint":0.0,
#            "L_ring_proximal_joint":0.0,
#            "L_ring_intermediate_joint":0.0,
#            "L_thumb_proximal_yaw_joint":0.0,
#            "L_thumb_proximal_pitch_joint":0.0,
#            "L_thumb_intermediate_joint":0.0,
#            "L_thumb_distal_joint":0.0,
#
#            "R_index_proximal_joint": 0.0,
#            "R_index_intermediate_joint": 0.0,
#            "R_middle_proximal_joint": 0.0,
#            "R_middle_intermediate_joint": 0.0,
#            "R_pinky_proximal_joint":0.0,
#            "R_pinky_intermediate_joint":0.0,
#            "R_ring_proximal_joint":0.0,
#            "R_ring_intermediate_joint":0.0,
#            "R_thumb_proximal_yaw_joint":0.0,
#            "R_thumb_proximal_pitch_joint":0.0,
#            "R_thumb_intermediate_joint":0.0,
#            "R_thumb_distal_joint":0.0,
#        },
#        joint_vel={".*": 0.0},
#    ),
#    soft_joint_pos_limit_factor=0.9,
#    actuators={
#        "legs": ImplicitActuatorCfg(
#            joint_names_expr=[
#                ".*_hip_yaw_joint", 
#                ".*_hip_roll_joint",
#                ".*_hip_pitch_joint", 
#                ".*_knee_joint",
#            ],
#            effort_limit_sim={
#                ".*_hip_yaw_joint": 88.0,
#                ".*_hip_roll_joint": 139.0,
#                ".*_hip_pitch_joint": 88.0,
#                ".*_knee_joint": 139.0,
#                #".*waist_yaw_joint": 88.0,
#                #".*waist_roll_joint": 35.0,
#                #".*waist_pitch_joint": 35.0,
#            },
#            velocity_limit_sim={
#                ".*_hip_yaw_joint": 32.0,
#                ".*_hip_roll_joint": 20.0,
#                ".*_hip_pitch_joint": 32.0,
#                ".*_knee_joint": 20.0,
#                #".*waist_yaw_joint": 32.0,
#                #".*waist_roll_joint": 30.0,
#                #".*waist_pitch_joint": 30.0,
#            },
#            stiffness={
#                ".*_hip_yaw_joint": 150.0,
#                ".*_hip_roll_joint": 150.0,
#                ".*_hip_pitch_joint": 200.0,
#                ".*_knee_joint": 200.0,
#                #".*waist.*": 200.0,
#            },
#            damping={
#                ".*_hip_yaw_joint": 5.0,
#                ".*_hip_roll_joint": 5.0,
#                ".*_hip_pitch_joint": 5.0,
#                ".*_knee_joint": 5.0,
#                #".*waist.*": 5.0,
#            },
#            #effort_limit=None,
#            #velocity_limit=None,
#            #stiffness=None,
#            #damping=None,
#            #armature=None,
#        ),
#        "feet": ImplicitActuatorCfg(
#            #effort_limit=None,
#            joint_names_expr=[".*_ankle_pitch_joint", ".*_ankle_roll_joint"],
#            #stiffness=None,
#            #damping=None,
#            # armature=0.001,
#            effort_limit_sim={
#                ".*_ankle_pitch_joint": 35.0,
#                ".*_ankle_roll_joint": 35.0,
#            },
#            velocity_limit_sim={
#                ".*_ankle_pitch_joint": 30.0,
#                ".*_ankle_roll_joint": 30.0,
#            },
#            stiffness=20.0,
#            damping=2.0,
#            armature=0.01,
#        ),
#        "arms": ImplicitActuatorCfg(
#            joint_names_expr=[
#                ".*_shoulder_.*_joint",
#                ".*_elbow_joint",
#                ".*_wrist_.*_joint"
#            ],
#            effort_limit=None,
#            velocity_limit=None,
#             stiffness={  # increase the stiffness (kp)
#                 ".*_shoulder_.*_joint": 25.0,
#                 ".*_elbow_joint": 50.0,
#                 ".*_wrist_.*_joint": 40.0,
#            },
#             damping={    # increase the damping (kd)
#                 ".*_shoulder_.*_joint": 2.0,
#                 ".*_elbow_joint": 2.0,
#                 ".*_wrist_.*_joint": 2.0,
#             },
#            armature=None,
#        ),
#        "hands": ImplicitActuatorCfg(
#            joint_names_expr=[
#                ".*_index_proximal_joint",
#                ".*_index_intermediate_joint",
#                ".*_middle_proximal_joint",
#                ".*_middle_intermediate_joint",
#                ".*_pinky_proximal_joint",
#                ".*_pinky_intermediate_joint",
#                ".*_ring_proximal_joint",
#                ".*_ring_intermediate_joint",
#                ".*_thumb_proximal_yaw_joint",
#                ".*_thumb_proximal_pitch_joint",
#                ".*_thumb_intermediate_joint",
#                ".*_thumb_distal_joint",
#            ],
#            effort_limit=100.0,
#            velocity_limit=50,
#            stiffness={
#                ".*_index_proximal_joint":1000.0,
#                ".*_index_intermediate_joint":1000.0,
#                ".*_middle_proximal_joint":1000.0,
#                ".*_middle_intermediate_joint":1000.0,
#                ".*_pinky_proximal_joint":1000.0,
#                ".*_pinky_intermediate_joint":1000.0,
#                ".*_ring_proximal_joint":1000.0,
#                ".*_ring_intermediate_joint":1000.0,
#                ".*_thumb_proximal_yaw_joint":1000.0,
#                ".*_thumb_proximal_pitch_joint":1000.0,
#                ".*_thumb_intermediate_joint":1000.0,
#                ".*_thumb_distal_joint":1000.0,
#            },
#            damping={
#                ".*_index_proximal_joint":15,
#                ".*_index_intermediate_joint":15,
#                ".*_middle_proximal_joint":15,
#                ".*_middle_intermediate_joint":15,
#                ".*_pinky_proximal_joint":15,
#                ".*_pinky_intermediate_joint":15,
#                ".*_ring_proximal_joint":15,
#                ".*_ring_intermediate_joint":15,
#                ".*_thumb_proximal_yaw_joint":15,
#                ".*_thumb_proximal_pitch_joint":15,
#                ".*_thumb_intermediate_joint":15,
#                ".*_thumb_distal_joint":15,
#            },
#            armature={
#                ".*": 0.0
#            },
#        ),
#
#    },
#)
#H12_CFG_WITH_INSPIRE_HAND = ArticulationCfg(
#    spawn=sim_utils.UsdFileCfg(
#        usd_path=f"/workspace/hams/isaac/mateo_ws/CL_Assets/isaac_assets/robots/h1_2-26dof-inspire-base-fix-usd/h1_2_26dof_with_inspire_rev_1_0.usd",
#        activate_contact_sensors=True,
#        rigid_props=sim_utils.RigidBodyPropertiesCfg(
#            kinematic_enabled=False,
#            disable_gravity=False,
#            retain_accelerations=True,  # 启用加速度计算 (Enable acceleration computation)
#            linear_damping=0.0,
#            angular_damping=0.0,
#            max_linear_velocity=1000.0,
#            max_angular_velocity=1000.0,
#            max_depenetration_velocity=1.0,
#        ),
#        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
#            #enabled_self_collisions=False, 
#            enabled_self_collisions=True, 
#            solver_position_iteration_count=8,
#            solver_velocity_iteration_count=4
#        ),
#
#    ),
#    init_state=ArticulationCfg.InitialStateCfg(
#        pos=(0.0, 0.0, 0.75),
#        joint_pos={
#            # legs joints
#            "left_hip_yaw_joint": 0.0,
#            "left_hip_roll_joint": 0.0,
#            "left_hip_pitch_joint": -0.05,
#            "left_knee_joint": 0.2,
#            "left_ankle_pitch_joint": -0.15,
#            "left_ankle_roll_joint": 0.0,
#            
#            "right_hip_yaw_joint": 0.0,
#            "right_hip_roll_joint": 0.0,
#            "right_hip_pitch_joint": -0.05,
#            "right_knee_joint": 0.2,
#            "right_ankle_pitch_joint": -0.15,
#            "right_ankle_roll_joint": 0.0,
#            
#            
#            # arms joints
#            "left_shoulder_pitch_joint": 0.0,
#            "left_shoulder_roll_joint": 0.0,
#            "left_shoulder_yaw_joint": 0.0,
#            "left_elbow_joint": 0.0,
#            "left_wrist_roll_joint": 0.0,
#            "left_wrist_pitch_joint": 0.0,
#            "left_wrist_yaw_joint": 0.0,
#            
#            "right_shoulder_pitch_joint": 0.0,
#            "right_shoulder_roll_joint": 0.0,
#            "right_shoulder_yaw_joint": 0.0,
#            "right_elbow_joint": 0.0,
#            "right_wrist_roll_joint": 0.0,
#            "right_wrist_pitch_joint": 0.0,
#            "right_wrist_yaw_joint": 0.0,
#            
#            # fingers joints
#            "L_index_proximal_joint": 0.0,
#            "L_index_intermediate_joint": 0.0,
#            "L_middle_proximal_joint": 0.0,
#            "L_middle_intermediate_joint": 0.0,
#            "L_pinky_proximal_joint":0.0,
#            "L_pinky_intermediate_joint":0.0,
#            "L_ring_proximal_joint":0.0,
#            "L_ring_intermediate_joint":0.0,
#            "L_thumb_proximal_yaw_joint":0.0,
#            "L_thumb_proximal_pitch_joint":0.0,
#            "L_thumb_intermediate_joint":0.0,
#            "L_thumb_distal_joint":0.0,
#
#            "R_index_proximal_joint": 0.0,
#            "R_index_intermediate_joint": 0.0,
#            "R_middle_proximal_joint": 0.0,
#            "R_middle_intermediate_joint": 0.0,
#            "R_pinky_proximal_joint":0.0,
#            "R_pinky_intermediate_joint":0.0,
#            "R_ring_proximal_joint":0.0,
#            "R_ring_intermediate_joint":0.0,
#            "R_thumb_proximal_yaw_joint":0.0,
#            "R_thumb_proximal_pitch_joint":0.0,
#            "R_thumb_intermediate_joint":0.0,
#            "R_thumb_distal_joint":0.0,
#        },
#        joint_vel={".*": 0.0},
#    ),
#    soft_joint_pos_limit_factor=0.75,
#    actuators={
#        "legs": ImplicitActuatorCfg(
#            joint_names_expr=[
#                ".*_hip_yaw_joint", 
#                ".*_hip_roll_joint",
#                ".*_hip_pitch_joint", 
#                ".*_knee_joint",
#            ],
#            effort_limit_sim={
#                ".*_hip_yaw_joint": 200.0,
#                ".*_hip_roll_joint": 200.0,
#                ".*_hip_pitch_joint": 200.0,
#                ".*_knee_joint": 300.0,
#            },
#            velocity_limit_sim={
#                ".*_hip_yaw_joint": 23.0,
#                ".*_hip_roll_joint":23.0, 
#                ".*_hip_pitch_joint": 23.0,
#                ".*_knee_joint": 14.0,
#            },
#            stiffness={
#                ".*_hip_yaw_joint": 150.0,
#                ".*_hip_roll_joint": 150.0,
#                ".*_hip_pitch_joint": 200.0,
#                ".*_knee_joint": 200.0,
#            },
#            damping={
#                ".*_hip_yaw_joint": 5.0,
#                ".*_hip_roll_joint": 5.0,
#                ".*_hip_pitch_joint": 5.0,
#                ".*_knee_joint": 5.0,
#            },
#            #effort_limit=None,
#            #velocity_limit=None,
#            #stiffness=20,
#            #damping=2.0,
#            #armature=0.01,
#        ),
#        "feet": ImplicitActuatorCfg(
#            #effort_limit=None,
#            joint_names_expr=[".*_ankle_pitch_joint", ".*_ankle_roll_joint"],
#            #stiffness=None,
#            #damping=None,
#            # armature=0.001,
#            effort_limit_sim={
#                ".*_ankle_pitch_joint": 60.0,
#                ".*_ankle_roll_joint": 60.0,
#            },
#            velocity_limit_sim={
#                ".*_ankle_pitch_joint": 9.0,
#                ".*_ankle_roll_joint": 9.0,
#            },
#            stiffness=0.000000001,
#            damping=0.001,
#            armature=0.01,
#        ),
#        "arms": ImplicitActuatorCfg(
#
#            joint_names_expr=[
#
#                ".*_shoulder_.*_joint",
#                ".*_elbow_joint",
#                ".*_wrist_.*_joint"
#            ],
#            velocity_limit_sim={
#                ".*_shoulder_yaw.*_joint": 20.0,
#                ".*_shoulder_roll.*_joint": 9.0,
#                ".*_shoulder_pitch.*_joint": 9.0,
#                ".*_elbow_joint": 20.0,
#                ".*_wrist_yaw.*_joint": 31.4,
#                ".*_wrist_roll.*_joint": 31.4,
#                ".*_wrist_pitch.*_joint": 31.4
#             },
#            # stiffness={  # increase the stiffness (kp)
#            #     ".*_shoulder_.*_joint": 25.0,
#            #     ".*_elbow_joint": 50.0,
#            #     ".*_wrist_.*_joint": 40.0,
#            #},
#             #damping={    # increase the damping (kd)
#             #    ".*_shoulder_.*_joint": 2.0,
#             #    ".*_elbow_joint": 2.0,
#             #    ".*_wrist_.*_joint": 2.0,
#             #},
#            effort_limit=10000,
#            armature=0.01,
#            damping=0.001,
#            stiffness=0.00000001
#        ),
#        "hands": ImplicitActuatorCfg(
#            joint_names_expr=[
#                ".*_index_proximal_joint",
#                ".*_index_intermediate_joint",
#                ".*_middle_proximal_joint",
#                ".*_middle_intermediate_joint",
#                ".*_pinky_proximal_joint",
#                ".*_pinky_intermediate_joint",
#                ".*_ring_proximal_joint",
#                ".*_ring_intermediate_joint",
#                ".*_thumb_proximal_yaw_joint",
#                ".*_thumb_proximal_pitch_joint",
#                ".*_thumb_intermediate_joint",
#                ".*_thumb_distal_joint",
#            ],
#            effort_limit=100.0,
#            velocity_limit=50,
#            stiffness={
#
#                ".*": 0.0
#                #".*_index_proximal_joint":1000.0,
#                #".*_index_intermediate_joint":1000.0,
#                #".*_middle_proximal_joint":1000.0,
#                #".*_middle_intermediate_joint":1000.0,
#                #".*_pinky_proximal_joint":1000.0,
#                #".*_pinky_intermediate_joint":1000.0,
#                #".*_ring_proximal_joint":1000.0,
#                #".*_ring_intermediate_joint":1000.0,
#                #".*_thumb_proximal_yaw_joint":1000.0,
#                #".*_thumb_proximal_pitch_joint":1000.0,
#                #".*_thumb_intermediate_joint":1000.0,
#                #".*_thumb_distal_joint":1000.0,
#            },
#            damping={
#                ".*": 0.0
#                #".*_index_proximal_joint":15,
#                #".*_index_intermediate_joint":15,
#                #".*_middle_proximal_joint":15,
#                #".*_middle_intermediate_joint":15,
#                #".*_pinky_proximal_joint":15,
#                #".*_pinky_intermediate_joint":15,
#                #".*_ring_proximal_joint":15,
#                #".*_ring_intermediate_joint":15,
#                #".*_thumb_proximal_yaw_joint":15,
#                #".*_thumb_proximal_pitch_joint":15,
#                #".*_thumb_intermediate_joint":15,
#                #".*_thumb_distal_joint":15,
#            },
#            armature={
#                ".*": 0.0
#            },
#        ),
#
#    },
#)
