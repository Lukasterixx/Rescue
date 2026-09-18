"""Loose rigid bodies in the competition scene, put back where they were exported.

The gravel stones (when dynamic) and the avoid lane's posts are free bodies. A level load in the rescue sim
(`rescue_sim/runtime.py`) resets them through these. The door leaf is loose too, but its sprung hinge closes it by
itself.
"""

from __future__ import annotations

from .build import TERRAIN_PATH


class LooseReset:
    """Puts a family of loose rigid bodies back where they were exported.

    `poses` are (position, wxyz) indexed by the trailing integer of each prim's name,
    which is how the exported gravel stones and avoid posts are numbered.
    """

    def __init__(self, pattern, poses, name):
        self.pattern, self.poses, self.name = pattern, list(poses), name
        self.view = None
        self.order = []

    def bind(self):
        from isaacsim.core.prims import RigidPrim

        self.view = RigidPrim(
            prim_paths_expr=self.pattern,
            name=self.name,
            reset_xform_properties=False,
            # These bodies already exist in a running physics scene. The
            # default adds PhysxRigidBodyAPI/sleep attributes to every body,
            # invalidating their GPU body indices while the view is binding.
            prepare_contact_sensors=False,
        )
        self.view.initialize()
        if self.view.count != len(self.poses):
            raise RuntimeError(
                f"Expected {len(self.poses)} bodies for {self.name}; found {self.view.count}"
            )
        # PhysX view order need not match USD traversal or lexicographic order.
        self.order = [int(path.rsplit("_", 1)[1]) for path in self.view.prim_paths]

    def reset(self):
        import numpy as np
        import torch

        if self.view is None:
            return
        ordered = [self.poses[i] for i in self.order]
        positions = np.array([p for p, _ in ordered], dtype=np.float32)
        rotations = np.array([r for _, r in ordered], dtype=np.float32)
        velocities = np.zeros((len(ordered), 6), dtype=np.float32)
        if self.view._backend == "torch":
            positions, rotations, velocities = [
                torch.as_tensor(v, device=self.view._device)
                for v in (positions, rotations, velocities)
            ]
        self.view.set_world_poses(positions, rotations)
        self.view.set_velocities(velocities)


class GravelReset(LooseReset):
    def __init__(self, lanes):
        self.stones = [stone for lane in lanes for stone in lane.stones]
        super().__init__(
            f"{TERRAIN_PATH}/Gravel/stone_.*",
            [(s.position, s.rotation) for s in self.stones],
            "competition_gravel",
        )

    @property
    def ordered_stones(self):
        return [self.stones[i] for i in self.order]


def loose_resets(lanes, options):
    """One reset per family of loose bodies: the gravel (when dynamic) and the avoid posts.
    The door leaf is loose too but its sprung hinge closes it by itself."""
    resets = []
    if options.gravel == "dynamic":
        resets.append(GravelReset(lanes))
    for lane in lanes:
        posts = [m for m in lane.dynamic_meshes() if m.name.startswith("post_")]
        if posts:
            resets.append(
                LooseReset(
                    f"{TERRAIN_PATH}/Structure/{lane.key}/post_.*",
                    [(tuple(m.vertices.mean(axis=0)), (1.0, 0.0, 0.0, 0.0)) for m in posts],
                    f"competition_{lane.key}_posts",
                )
            )
    return resets
