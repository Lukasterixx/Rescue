"""Export a standalone USD scene and optional preview without starting Isaac Sim.

    python3 -m competition.build --preview

Needs numpy, scipy, usd-core and Pillow; preview additionally needs matplotlib.
The sim launcher calls export_scene after Isaac has loaded its USD bindings.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

from .geometry import COLORS, Mesh, Options, build_lanes, combine, ground_mesh, stone_shape

ROOT = "/Competition"
TERRAIN_PATH = "/World/competition/terrain"


def _mesh(stage, path, data):
    from pxr import UsdGeom, Vt

    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(data.vertices.astype(np.float32)))
    mesh.CreateFaceVertexCountsAttr([3] * len(data.faces))
    mesh.CreateFaceVertexIndicesAttr(
        Vt.IntArray.FromNumpy(data.faces.ravel().astype(np.int32))
    )
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateExtentAttr(
        [tuple(data.vertices.min(axis=0)), tuple(data.vertices.max(axis=0))]
    )
    return mesh


def export_scene(output: Path, options=Options()):
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

    output = Path(output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    from .textures import write_textures

    write_textures(output.parent / "textures")
    lanes = build_lanes(options)
    stage = Usd.Stage.CreateNew(str(output))
    root = UsdGeom.Xform.Define(stage, ROOT)
    stage.SetDefaultPrim(root.GetPrim())
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetMetadata(
        "documentation",
        "RoboCup Rescue 2026C Korea: Shifty Gravel, Diagonal K-Rails and the K-Rail practice square with linear align/inspect tasks. See competition/README.md for source dimensions and approximations.",
    )
    materials = {}
    for name, color in COLORS.items():
        material = UsdShade.Material.Define(stage, f"{ROOT}/Materials/{name}")
        shader = UsdShade.Shader.Define(stage, f"{material.GetPath()}/Surface")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(color)
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.85)
        if name in ("wood", "osb") or name.startswith("target_"):
            reader = UsdShade.Shader.Define(stage, f"{material.GetPath()}/UV")
            reader.CreateIdAttr("UsdPrimvarReader_float2")
            reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
            texture = UsdShade.Shader.Define(stage, f"{material.GetPath()}/Texture")
            texture.CreateIdAttr("UsdUVTexture")
            texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(
                Sdf.AssetPath(f"textures/{name}.png")
            )
            texture.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set(
                "sRGB"
            )
            for axis in ("S", "T"):
                texture.CreateInput(f"wrap{axis}", Sdf.ValueTypeNames.Token).Set(
                    "repeat"
                )
            texture.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
                reader.ConnectableAPI(), "result"
            )
            shader.GetInput("diffuseColor").ConnectToSource(
                texture.ConnectableAPI(), "rgb"
            )
        material.CreateSurfaceOutput().ConnectToSource(
            shader.ConnectableAPI(), "surface"
        )
        physics = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        physics.CreateStaticFrictionAttr(0.85 if name != "gravel" else 0.7)
        physics.CreateDynamicFrictionAttr(0.65 if name != "gravel" else 0.55)
        physics.CreateRestitutionAttr(0.0)
        materials[name] = material

    all_meshes = [ground_mesh(lanes)] + [mesh for lane in lanes for mesh in lane.meshes]
    # Explicit scanner target. Excludes railings and support frames; includes
    # the K-Rails and the initial perceived surface of the loose aggregate.
    scan = combine([mesh for mesh in all_meshes if mesh.scan])
    scanner = _mesh(stage, f"{ROOT}/WalkableScan", scan)
    scanner.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
    centers = {}
    for data in all_meshes:
        if not data.collision and data.name.endswith("_gravel_scan"):
            continue
        owner = next(
            (lane.key for lane in lanes if any(data is m for m in lane.meshes)), "hall"
        )
        path = f"{ROOT}/Structure/{owner}/{data.name}"
        if data.dynamic:
            # Loose bodies (door leaf, avoid posts) are authored about their centroid with a
            # translate op, so the runtime can put them back with a pose.
            center = data.vertices.mean(axis=0)
            centers[(owner, data.name)] = center
            mesh = _mesh(stage, path, Mesh(data.name, data.vertices - center, data.faces))
            UsdGeom.Xformable(mesh).AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
        else:
            mesh = _mesh(stage, path, data)
        uv = None
        if data.uv is not None:
            # Authored per vertex (the inspect targets): one texture across the face.
            uv = data.uv[data.faces].astype(np.float32)
        elif data.material in ("wood", "osb"):
            triangles = data.vertices[data.faces]
            normals = np.cross(
                triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]
            )
            # Project each face on its dominant plane, at a fixed metric scale.
            uv = np.zeros((len(triangles), 3, 2), dtype=np.float32)
            for normal_axis, axes in enumerate(((1, 2), (0, 2), (0, 1))):
                selected = np.argmax(np.abs(normals), axis=1) == normal_axis
                uv[selected] = triangles[selected][:, :, axes] / 0.75
        if uv is not None:
            UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
                "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying
            ).Set(Vt.Vec2fArray.FromNumpy(uv.reshape(-1, 2)))
        material = materials[data.material]
        binding = UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim())
        binding.Bind(material)
        if data.collision:
            UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
            # PhysX cannot simulate a moving triangle mesh, so loose bodies get a hull.
            UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr(
                "convexHull" if data.dynamic else "none"
            )
            binding.Bind(material, materialPurpose="physics")
        if data.dynamic:
            UsdPhysics.RigidBodyAPI.Apply(mesh.GetPrim())
            UsdPhysics.MassAPI.Apply(mesh.GetPrim()).CreateMassAttr(float(data.mass))
    for lane in lanes:
        for joint in lane.joints:
            hinge = UsdPhysics.RevoluteJoint.Define(
                stage, f"{ROOT}/Structure/{lane.key}/{joint.name}"
            )
            if joint.body0:
                hinge.CreateBody0Rel().SetTargets([f"{ROOT}/Structure/{lane.key}/{joint.body0}"])
            hinge.CreateBody1Rel().SetTargets([f"{ROOT}/Structure/{lane.key}/{joint.body1}"])
            pivot = np.asarray(joint.pivot, dtype=float)
            hinge.CreateLocalPos0Attr(Gf.Vec3f(*map(float, pivot)))
            hinge.CreateLocalPos1Attr(
                Gf.Vec3f(*map(float, pivot - centers[(lane.key, joint.body1)]))
            )
            hinge.CreateAxisAttr("Z")
            hinge.CreateLowerLimitAttr(float(joint.limits[0]))
            hinge.CreateUpperLimitAttr(float(joint.limits[1]))
            drive = UsdPhysics.DriveAPI.Apply(hinge.GetPrim(), "angular")
            drive.CreateTypeAttr("force")
            # UsdPhysics angular drive gains are per DEGREE; Joint's are per radian.
            per_degree = math.pi / 180.0
            drive.CreateStiffnessAttr(float(joint.stiffness) * per_degree)
            drive.CreateDampingAttr(float(joint.damping) * per_degree)
            drive.CreateTargetPositionAttr(0.0)

    # Class prim prototypes author no extra visible/physical objects. Instance
    # roots are rigid bodies, allowing a batched RigidPrim reset in the runtime.
    for variant in range(6):
        shape, mass = stone_shape(variant)
        path = f"{ROOT}/StoneTemplates/Stone{variant}"
        mesh = _mesh(stage, path, shape)
        mesh.GetPrim().SetSpecifier(Sdf.SpecifierClass)
        binding = UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim())
        binding.Bind(materials["gravel"])
        binding.Bind(materials["gravel"], materialPurpose="physics")
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr(
            "convexHull"
        )
        if options.gravel == "dynamic":
            UsdPhysics.RigidBodyAPI.Apply(mesh.GetPrim())
            UsdPhysics.MassAPI.Apply(mesh.GetPrim()).CreateMassAttr(mass)
        # Small contact offsets prevent a 2 cm default envelope dwarfing stones.
        prim = mesh.GetPrim()
        schemas = list(prim.GetAppliedSchemas())
        prim.SetMetadata(
            "apiSchemas",
            Sdf.TokenListOp.CreateExplicit(schemas + ["PhysxCollisionAPI"]),
        )
        prim.CreateAttribute(
            "physxCollision:contactOffset", Sdf.ValueTypeNames.Float
        ).Set(0.001)
        prim.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(
            0.0
        )

    for lane in lanes:
        for i, stone in enumerate(lane.stones):
            path = f"{ROOT}/Gravel/stone_{i:05d}"
            prim = stage.DefinePrim(path)
            prim.GetReferences().AddInternalReference(
                f"{ROOT}/StoneTemplates/Stone{stone.variant}"
            )
            transform = UsdGeom.Xformable(prim)
            transform.AddTranslateOp().Set(Gf.Vec3d(*stone.position))
            transform.AddOrientOp().Set(
                Gf.Quatf(stone.rotation[0], Gf.Vec3f(*stone.rotation[1:]))
            )
            # Leaf mesh references share geometry; per-rock poses remain editable.
            prim.SetInstanceable(True)
    stage.GetRootLayer().Save()
    manifest = {
        "source": "RoboCupRescue-Arena-Fabrication-Guide-2026C-Korea-1.pdf",
        "options": vars(options),
        "lanes": [
            {
                "key": lane.key,
                "title": lane.title,
                "origin": lane.origin,
                "spawn": lane.spawn,
                "rotation_wxyz": lane.spawn_rotation,
                "railings": lane.railing_count,
                "stones": len(lane.stones),
                "loose_bodies": [m.name for m in lane.dynamic_meshes()],
                "joints": [j.name for j in lane.joints],
            }
            for lane in lanes
        ],
        "scanner_path": f"{TERRAIN_PATH}/WalkableScan",
    }
    output.with_suffix(".json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(
        f"[competition] Exported {output} ({len(lanes[0].stones)} {options.gravel} stones)"
    )
    return lanes


def preview_scene(output, options=Options()):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    lanes = build_lanes(options)
    count = len(lanes)
    cols = min(count, 6)
    bands = math.ceil(count / cols)
    figure = plt.figure(
        figsize=(4.4 * cols, 5.6 * bands), facecolor="#f1ede6", layout="constrained"
    )
    for i, lane in enumerate(lanes):
        band, col = divmod(i, cols)
        ax = figure.add_subplot(2 * bands, cols, 2 * band * cols + col + 1, projection="3d")
        top = figure.add_subplot(2 * bands, cols, (2 * band + 1) * cols + col + 1)
        extent = np.concatenate([m.vertices for m in lane.meshes]) - lane.origin
        lo, hi = extent.min(axis=0) - 0.3, extent.max(axis=0) + 0.3
        height = max(hi[2], 1.9)
        all_triangles, all_colors, top_faces = [], [], []
        for material, color in COLORS.items():
            meshes = [m for m in lane.meshes if m.material == material and m.collision]
            triangles = [m.vertices[m.faces] - lane.origin for m in meshes]
            if triangles:
                triangles = np.concatenate(triangles)
                all_triangles.extend(triangles)
                all_colors.extend([color] * len(triangles))
            # Project upward-facing faces for a readable fabrication plan.
            for m in meshes:
                tri = m.vertices[m.faces] - lane.origin
                normals = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
                top_faces.extend(
                    (t[:, 2].mean(), t[:, :2], color) for t in tri[normals[:, 2] > 1e-8]
                )
        if lane.stones:
            shapes = [stone_shape(i)[0] for i in range(6)]
            # Render full-size rocks, not points that disappear behind surfaces.
            for stone in lane.stones:
                shape = shapes[stone.variant]
                w, x, y, z = stone.rotation
                vector = np.array([x, y, z])
                v = shape.vertices
                rotated = v + 2 * np.cross(vector, np.cross(vector, v) + w * v)
                triangles = (rotated + stone.position - lane.origin)[shape.faces]
                color = np.array(COLORS["gravel"]) * (0.8 + stone.variant * 0.08)
                all_triangles.extend(triangles)
                all_colors.extend([color] * len(triangles))
                # Rock silhouettes on the plan, depth-sorted with the structure.
                top_faces.extend((t[:, 2].mean(), t[:, :2], color) for t in triangles)
        ax.add_collection3d(
            Poly3DCollection(all_triangles, facecolor=all_colors, edgecolor="none")
        )
        top_faces.sort(key=lambda item: item[0])
        top.add_collection(
            PolyCollection(
                [p for _, p, _ in top_faces],
                facecolor=[c for _, _, c in top_faces],
                edgecolor="none",
            )
        )
        spawn = np.array(lane.spawn) - lane.origin
        w, _, _, z = lane.spawn_rotation
        yaw = 2 * math.atan2(z, w)
        top.annotate(
            "START",
            xy=spawn[:2] + 0.45 * np.array([math.cos(yaw), math.sin(yaw)]),
            xytext=spawn[:2],
            ha="center",
            color="white",
            arrowprops={"arrowstyle": "->", "color": "white"},
            weight="bold",
            fontsize=9,
        )
        ax.set(xlim=(lo[0], hi[0]), ylim=(lo[1], hi[1]), zlim=(0, height))
        ax.set_box_aspect((hi[0] - lo[0], hi[1] - lo[1], height))
        ax.view_init(elev=32, azim=-65)
        ax.set_axis_off()
        ax.set_title(f"F{i+1}  {lane.title}", fontweight="bold", fontsize=13)
        top.set(
            xlim=(lo[0], hi[0]),
            ylim=(lo[1], hi[1]),
            aspect="equal",
            xlabel="metres",
            ylabel="metres",
        )
        top.set_title(lane.subtitle, fontsize=8)
        for a in (ax, top):
            a.set_facecolor("#f1ede6")
    figure.suptitle(
        f"RoboCup Rescue 2026 · {options.difficulty} · K-Rails {options.k_rail_height*100:g} cm",
        fontsize=22,
        fontweight="bold",
    )
    figure.savefig(output, dpi=140)
    plt.close(figure)
    print(f"[competition] Preview: {output}")


def add_geometry_args(parser):
    parser.add_argument("--difficulty", choices=("flat", "slopes"), default="flat")
    parser.add_argument(
        "--k-rail-height",
        type=float,
        default=0.1,
        help="metres; 0.10–0.40 in 0.05 increments",
    )
    parser.add_argument("--gravel", choices=("dynamic", "static"), default="dynamic")
    parser.add_argument("--gravel-seed", type=int, default=2026)
    parser.add_argument(
        "--alley-width", type=float, default=0.45, help="Center in Alleys doorway, metres"
    )
    parser.add_argument(
        "--door-floor",
        choices=("flat", "square", "half"),
        default="flat",
        help="Doors: full floor, yellow square steps removed, orange half steps removed too",
    )
    parser.add_argument("--stair-angle", type=float, default=45.0, help="Stair incline, 35-45")
    parser.add_argument(
        "--stair-debris", type=int, default=0, help="Hinged debris rails on the stair, 0-3"
    )


def options_from_args(args):
    return Options(
        args.difficulty,
        args.k_rail_height,
        args.gravel,
        args.gravel_seed,
        args.alley_width,
        args.door_floor,
        args.stair_angle,
        args.stair_debris,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    add_geometry_args(parser)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).parent / "generated" / "competition.usda",
    )
    parser.add_argument("--preview", action="store_true")
    args = parser.parse_args()
    options = options_from_args(args)
    export_scene(args.output, options)
    if args.preview:
        preview_scene(args.output.with_suffix(".png"), options)


if __name__ == "__main__":
    main()
