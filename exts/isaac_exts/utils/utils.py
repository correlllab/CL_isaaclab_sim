import omni.usd
import omni
import numpy as np
from scipy.spatial.transform import Rotation

import traceback

def log_func(fn: callable):
    def log(*args, **kwargs):
        name = fn.__name__

        print(f"{__file__}: currently calling: {name}...")
        res = fn(*args, **kwargs)
        print(f"{__file__}: {name} result: {res}")
        print(f"{__file__}: returning from {name} call...")
        return res
    return log

@log_func
def get_prim_transformations(prim):
    global_matrix = omni.usd.get_world_transform_matrix(prim)
    global_translate_pos = global_matrix.ExtractTranslation()
    tmp = global_matrix.ExtractRotationQuat()
    try:
        w = tmp.GetReal()
        x, y, z = tmp.GetImaginary()
    except Exception as e:
        traceback.print_exc()
        print(e)
        raise AssertionError
    try:
        global_translate_orient = np.array([w, x, y, z])
    except Exception as e:
        traceback.print_exc()
        print(e)
        raise RuntimeError
    try:
        local_translate_pos = omni.usd.get_local_transform_SRT(prim)
    except Exception as e:
        traceback.print_exc()
        print(e)
        raise RuntimeError
    return (global_translate_pos, global_translate_orient, local_translate_pos)

@log_func
def euler_to_quat(euler: list):
    #TODO: this is currently doing extrinsic rotations (fixed frame), maybe needs to be intrinsic(moving system)
    print(euler)
    rot = Rotation.from_euler('xyz', euler, degrees=True)
    rot_quat = rot.as_quat()
    return rot_quat

@log_func
def quat_to_euler(quat: list):
    rot = Rotation.from_quat(quat)
    rot_euler = rot.as_euler('xyz', degrees=True)
    return rot_euler
