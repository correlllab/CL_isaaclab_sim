"""Author Magpie's parallelogram linkages in the spawned stage, not the asset.

PhysxMimicJointAPI enforces q + gearing*q_reference + offset = 0 and applies
reaction impulses to both joints. The source parallelogram geometry therefore
requires gearing +1 on hinge_2 and -1 on hinge_3, both referencing hinge_1.
"""
import math


def add_magpie_couplings(root):
    """Constrain the eight passive hinges before articulation initialization."""
    from pxr import PhysxSchema, Usd, UsdPhysics

    joints = {prim.GetName(): prim for prim in Usd.PrimRange(root)
              if prim.IsA(UsdPhysics.RevoluteJoint)}
    required = [f'{hand}_{finger}_hinge_{i}' for hand in ('lg', 'rg')
                for finger in ('left', 'right') for i in (1, 2, 3)]
    missing = set(required) - joints.keys()
    if missing:
        raise ValueError(f'Magpie USD is missing linkage joints: {sorted(missing)}')
    for hand in ('lg', 'rg'):
        for finger in ('left', 'right'):
            prefix = f'{hand}_{finger}_hinge_'
            reference = UsdPhysics.RevoluteJoint(joints[prefix + '1'])
            lower = reference.GetLowerLimitAttr().Get()
            upper = reference.GetUpperLimitAttr().Get()
            if not (math.isfinite(lower) and math.isfinite(upper) and lower < upper):
                raise ValueError(f'Magpie motor {prefix}1 needs finite joint limits')
            for suffix, gearing in (('2', 1.), ('3', -1.)):
                follower = UsdPhysics.RevoluteJoint(joints[prefix + suffix])
                # The schema requires limited revolute joints. USD angles use
                # degrees; transforming the reference bounds preserves units.
                bounds = sorted((-gearing * lower, -gearing * upper))
                follower.CreateLowerLimitAttr(bounds[0])
                follower.CreateUpperLimitAttr(bounds[1])
                mimic = PhysxSchema.PhysxMimicJointAPI.Apply(follower.GetPrim(), 'rotX')
                mimic.CreateReferenceJointRel().SetTargets([reference.GetPath()])
                mimic.CreateReferenceJointAxisAttr('rotX')
                mimic.CreateGearingAttr(gearing)
                mimic.CreateOffsetAttr(0.)


def spawn_magpie(prim_path, cfg, translation=None, orientation=None, **kwargs):
    """Use Isaac's USD spawner, then couple joints before cloning environments."""
    from isaaclab.sim.spawners.from_files import spawn_from_usd
    from isaaclab.sim.utils import clone

    @clone
    def spawn_one(path, config, translation=None, orientation=None, **options):
        prim = spawn_from_usd(path, config, translation, orientation, **options)
        add_magpie_couplings(prim)
        return prim

    return spawn_one(prim_path, cfg, translation, orientation, **kwargs)
