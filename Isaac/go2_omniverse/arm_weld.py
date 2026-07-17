"""Build a single Go2+D1 articulation -- the `--arm_mount weld` path.

The alternative, `--arm_mount teleport`, is what this repo did originally: the
arm's root is written onto the dog's back every physics step. That keeps it in
place but tells the walking policy nothing, because the teleport rewrites the
arm's root pose and zeroes its velocity each tick, so no reaction force ever
reaches the quadruped. The arm is, dynamically, a ghost.

Welded instead: `build_welded_robot_usd` composes a stage that references Isaac
Lab's stock `go2.usd` and the URDF-imported D1, joins them with a
`UsdPhysics.FixedJoint`, and strips the D1's ArticulationRootAPI so PhysX folds
the arm's links into the Go2's articulation rather than parsing a second one.
The result is one articulation of 20 joints whose mass matrix includes the arm --
which is the point: the payload now perturbs the gait.

The cost is that the robot no longer has exactly the 12 joints the policy was
trained on, so every observation and action term has to be scoped back down to
the legs. `omniverse_sim.py` does that in `_scope_env_cfg_to_legs()`; see
LEG_JOINTS in `custom_rl_env.py`.

Developed and validated in the D1Training repo, which has the full measurement
write-up behind the mass model and the gain choices.
"""
from __future__ import annotations

import os
from dataclasses import dataclass

from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics


@dataclass
class WeldResult:
    """What the weld produced, and what the env cfg needs to know about it."""

    usd_path: str
    # Factor the arm's link masses were scaled by (1.0 if untouched). Reported
    # for logging only -- notably NOT applied to the drive gains or the effort
    # limits, which come from Unitree's published per-joint torques and are
    # independent of how heavy the arm is.
    arm_mass_scale: float = 1.0


def import_d1_urdf(urdf_path: str, dest_usd_path: str) -> str:
    """Convert the D1 URDF to USD. Mirrors Rescue's `generate_arm_usd()`.

    `fix_base=False` matters even though the arm ends up welded: a fixed base
    would make the importer author its own root joint to the world, and the arm
    would drag the dog to the origin. The weld is our job, not the importer's.
    """
    import omni.kit.app
    import omni.kit.commands

    ext_manager = omni.kit.app.get_app().get_extension_manager()
    try:
        ext_manager.set_extension_enabled_immediate("isaacsim.asset.importer.urdf", True)
        import isaacsim.asset.importer.urdf as urdf_importer
    except ImportError:
        ext_manager.set_extension_enabled_immediate("omni.importer.urdf", True)
        import omni.importer.urdf as urdf_importer

    import_config = urdf_importer._urdf.ImportConfig()
    import_config.merge_fixed_joints = False
    import_config.fix_base = False
    import_config.make_default_prim = True

    if os.path.exists(dest_usd_path):
        os.remove(dest_usd_path)

    print(f"[weld] Importing D1 URDF -> {dest_usd_path}")
    omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=urdf_path,
        import_config=import_config,
        dest_path=dest_usd_path,
    )
    return dest_usd_path


def _set_translate(xformable: UsdGeom.Xformable, vec: Gf.Vec3d) -> None:
    """Set the prim's translate op, reusing whichever one the reference brought in.

    The URDF importer authors a full [translate, orient, scale] op order on its
    default prim, and that arrives through the reference. Calling AddTranslateOp
    on top of it raises rather than overwriting.
    """
    for op in xformable.GetOrderedXformOps():
        if op.GetOpName() == "xformOp:translate":
            op.Set(vec)
            return
    xformable.AddTranslateOp().Set(vec)


def _find_prim_named(root: Usd.Prim, name: str) -> Usd.Prim | None:
    for prim in Usd.PrimRange(root):
        if prim.GetName() == name:
            return prim
    return None


def _rigid_body_names(root: Usd.Prim) -> list[str]:
    return [
        p.GetName()
        for p in Usd.PrimRange(root)
        if p.HasAPI(UsdPhysics.RigidBodyAPI)
    ]


def _demote_articulation_root(subtree_root: Usd.Prim) -> int:
    """Remove every ArticulationRootAPI under `subtree_root`.

    This is what fuses the two robots. PhysX discovers an articulation from a
    prim carrying ArticulationRootAPI and then walks the joint graph outward; if
    the D1 keeps its own root API, PhysX parses a second articulation and the
    fixed joint between them degrades to a soft maximal-coordinate constraint --
    the arm visibly jiggles and the force coupling is mush. Stripped, the arm's
    links are reached only by walking outward from the Go2's root, so they join
    the Go2's articulation as ordinary links.

    RemoveAPI authors a deletion into the `apiSchemas` list op in this stage's
    root layer, which is stronger than the reference that applied it.
    """
    removed = 0
    for prim in Usd.PrimRange(subtree_root):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
            removed += 1
        if prim.HasAPI(PhysxSchema.PhysxArticulationAPI):
            prim.RemoveAPI(PhysxSchema.PhysxArticulationAPI)
    return removed


# Bus-servo mass carried by each link, in kg. The servo driving a joint sits at
# that joint, which is its child link's frame origin -- so Link1 carries J0's
# servo, Link2 carries J1's, and so on. Link6 carries two: J5's and the
# gripper's (J6), which lives in the wrist.
#
# Unitree does not publish per-servo masses, so these come from comparable bus
# servos: the D1's 3.3 Nm joints are the class of a Feetech STS3215 (55 g at
# 30 kg.cm / 2.94 Nm), and its 1.7 Nm joints sit between that and a Feetech
# STS3032 (25 g at 4.5 kg.cm), with a Dynamixel XL430-W250 at 57 g / 1.5 Nm as a
# cross-check. Rounded to 60 g and 45 g. They are small either way -- 345 g out
# of the arm's 3152 g -- which is exactly the point. See _apply_d1_mass_model.
_SERVO_MASS_BY_LINK = {
    "Link1": 0.060,          # J0, 3.3 Nm
    "Link2": 0.060,          # J1, 3.3 Nm
    "Link3": 0.045,          # J2, 1.7 Nm
    "Link4": 0.045,          # J3, 1.7 Nm
    "Link5": 0.045,          # J4, 1.7 Nm
    "Link6": 0.045 + 0.045,  # J5, 1.7 Nm, plus the gripper servo J6, 1.7 Nm
}

# Where the arm's remaining mass lives: a heavy metal cylinder at the base.
# It is welded to the Go2, so it loads the dog but never any arm joint.
_ARM_BASE_LINK = "base_link"


def _apply_d1_mass_model(subtree_root: Usd.Prim, total_kg: float) -> float:
    """Bring the arm up to `total_kg`, putting the mass where it actually is.

    The shipped URDF's inertials are a SolidWorks export of the shells alone and
    total 0.719 kg, where Unitree publishes 3152 g for the D1-550. That 2.4 kg
    gap has to go somewhere, and where matters more than you would guess.

    Spreading it evenly across every link -- the obvious move, and what this did
    first -- is wrong, and wrong in a way that shows up immediately: it loads the
    wrist as heavily as the base, the shoulder then needs ~6 Nm against its
    published 3.3 Nm limit, and the arm sags to its stops instead of holding the
    pose the IK asks for. That is not a D1, it is an artefact of the smear.

    The real distribution: the base is a heavy metal cylinder and the servos are
    small. So the model is

      - every link keeps its shell inertial,
      - plus the servo sitting at its joint (_SERVO_MASS_BY_LINK),
      - and everything still missing goes on `base_link`.

    That leaves ~2.1 kg at the base and ~1.0 kg of moving arm, which a 3.3 Nm
    shoulder can hold.

    It matters for the gait too, and not in the direction you might guess: the
    base is welded to the Go2, so its share is dead payload bolted to the dog's
    back at the mount -- low and centred -- rather than swinging on the end of a
    lever. The arm's full mass still reaches the policy; it just reaches it in
    the right place.

    Each link's inertia is scaled by its own mass factor, which keeps the shell's
    shape and raises its density. For `base_link` that is not an approximation at
    all: a solid metal cylinder is the same shape as its shell, only denser. For
    the rest, the servo is treated as spread through the link rather than as a
    point mass at the joint -- they are tens of grams, so the error is small.
    """
    if total_kg <= 0.0:
        raise ValueError(f"arm mass must be positive, got {total_kg}")

    links = {
        p.GetName(): p for p in Usd.PrimRange(subtree_root) if p.HasAPI(UsdPhysics.MassAPI)
    }
    shell = {n: (UsdPhysics.MassAPI(p).GetMassAttr().Get() or 0.0) for n, p in links.items()}
    shell_total = sum(shell.values())
    if shell_total <= 0.0:
        print("[weld][WARN] Arm links report no mass; skipping the mass model.")
        return 1.0

    # A renamed link would otherwise silently drop a servo, or worse, silently
    # dump 2 kg nowhere.
    expected = set(_SERVO_MASS_BY_LINK) | {_ARM_BASE_LINK}
    if not expected <= links.keys():
        raise RuntimeError(
            f"[weld] Mass model does not match the URDF. Missing: "
            f"{sorted(expected - links.keys())}. Found: {sorted(links)}"
        )

    servo_total = sum(_SERVO_MASS_BY_LINK.values())
    base_extra = total_kg - shell_total - servo_total
    if base_extra < 0.0:
        raise ValueError(
            f"[weld] Arm mass {total_kg:.3f} kg is below shells plus servos "
            f"({shell_total + servo_total:.3f} kg); nothing left for the base."
        )

    target = {n: shell[n] + _SERVO_MASS_BY_LINK.get(n, 0.0) for n in links}
    target[_ARM_BASE_LINK] += base_extra

    for name, prim in links.items():
        factor = target[name] / shell[name] if shell[name] > 0.0 else 1.0
        mass_api = UsdPhysics.MassAPI(prim)
        mass_api.GetMassAttr().Set(target[name])
        diag_attr = mass_api.GetDiagonalInertiaAttr()
        diag = diag_attr.Get()
        if diag:
            diag_attr.Set(Gf.Vec3f(diag[0] * factor, diag[1] * factor, diag[2] * factor))

    moving = total_kg - target[_ARM_BASE_LINK]
    print(f"[weld] D1 mass model -> {total_kg:.3f} kg "
          f"(shells {shell_total:.3f} + servos {servo_total:.3f} + base fill {base_extra:.3f})")
    print(f"[weld]   {_ARM_BASE_LINK}: {shell[_ARM_BASE_LINK]:.3f} -> {target[_ARM_BASE_LINK]:.3f} kg "
          f"-- welded to the Go2, so it loads the dog, not the arm's joints")
    print(f"[weld]   moving arm above the base: {moving:.3f} kg")
    print(f"[weld]   joint effort limits left at the URDF's published-spec values")
    return total_kg / shell_total


def build_welded_robot_usd(
    go2_usd_path: str,
    d1_urdf_path: str,
    out_usd_path: str,
    mount_pos: tuple[float, float, float] = (0.0, 0.0, 0.08),
    go2_base_link: str = "base",
    d1_base_link: str = "base_link",
    arm_mass_kg: float | None = None,
) -> WeldResult:
    """Compose go2.usd + d1.usd into one articulation.

    `mount_pos` is the arm base's offset from the Go2's base link, in metres.
    It matches Rescue's ARM_MOUNT_Z so the reach limits and IK targets ported
    from there stay meaningful.
    """
    os.makedirs(os.path.dirname(out_usd_path), exist_ok=True)
    d1_usd_path = os.path.join(os.path.dirname(out_usd_path), "d1.usd")
    import_d1_urdf(d1_urdf_path, d1_usd_path)

    if os.path.exists(out_usd_path):
        os.remove(out_usd_path)

    stage = Usd.Stage.CreateNew(out_usd_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    # /Robot takes on go2.usd's default prim wholesale -- including its
    # ArticulationRootAPI, which becomes the root of the merged articulation.
    root = UsdGeom.Xform.Define(stage, "/Robot")
    root_prim = root.GetPrim()
    stage.SetDefaultPrim(root_prim)
    root_prim.GetReferences().AddReference(go2_usd_path)

    # The arm hangs under the same root so PhysX can reach it from there.
    arm_xform = UsdGeom.Xform.Define(stage, "/Robot/D1")
    arm_prim = arm_xform.GetPrim()
    arm_prim.GetReferences().AddReference(d1_usd_path)
    # Placing the Xform at the mount keeps the spawn pose consistent with the
    # joint frames below; without it the arm starts inside the dog and PhysX
    # resolves the penetration with a kick on the first step.
    _set_translate(arm_xform, Gf.Vec3d(*mount_pos))

    n_removed = _demote_articulation_root(arm_prim)
    print(f"[weld] Stripped {n_removed} ArticulationRootAPI from the arm subtree.")

    arm_mass_scale = 1.0
    if arm_mass_kg is not None:
        arm_mass_scale = _apply_d1_mass_model(arm_prim, arm_mass_kg)

    go2_base = _find_prim_named(root_prim, go2_base_link)
    arm_base = _find_prim_named(arm_prim, d1_base_link)
    if go2_base is None:
        raise RuntimeError(
            f"[weld] No prim named '{go2_base_link}' in {go2_usd_path}. "
            f"Rigid bodies found: {_rigid_body_names(root_prim)}"
        )
    if arm_base is None:
        raise RuntimeError(
            f"[weld] No prim named '{d1_base_link}' in {d1_usd_path}. "
            f"Rigid bodies found: {_rigid_body_names(arm_prim)}"
        )

    joint = UsdPhysics.FixedJoint.Define(stage, "/Robot/D1/arm_mount_joint")
    joint.CreateBody0Rel().SetTargets([go2_base.GetPath()])
    joint.CreateBody1Rel().SetTargets([arm_base.GetPath()])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*mount_pos))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    # Left default (False) on purpose: excluding it would make PhysX treat the
    # weld as a maximal joint and re-split the articulation in two.
    joint.CreateExcludeFromArticulationAttr().Set(False)

    # Duplicate body names are ambiguous to Isaac Lab's find_bodies() regexes,
    # and the failure is a silently wrong body index rather than an exception.
    names = _rigid_body_names(root_prim)
    dupes = {n for n in names if names.count(n) > 1}
    if dupes:
        raise RuntimeError(f"[weld] Duplicate rigid body names across Go2 and D1: {sorted(dupes)}")

    stage.GetRootLayer().Save()
    print(f"[weld] Welded robot written to {out_usd_path}")
    print(f"[weld] Rigid bodies ({len(names)}): {names}")
    return WeldResult(usd_path=out_usd_path, arm_mass_scale=arm_mass_scale)
