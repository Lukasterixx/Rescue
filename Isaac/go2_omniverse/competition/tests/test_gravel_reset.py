"""Regression coverage for binding existing gravel without reauthoring physics."""

from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import numpy as np

from competition.geometry import Stone
from competition.runtime import GravelReset


class GravelResetTests(unittest.TestCase):
    def test_bind_preserves_running_bodies_and_uses_physics_view_order(self):
        stones = [
            Stone((1.0, 2.0, 3.0), (1.0, 0.0, 0.0, 0.0), 0),
            Stone((4.0, 5.0, 6.0), (0.0, 0.0, 0.0, 1.0), 1),
        ]
        view = NS(
            count=2,
            prim_paths=["/Gravel/stone_00001", "/Gravel/stone_00000"],
            initialize=Mock(),
        )

        def rigid_prim(**kwargs):
            # Isaac Sim's default is True: it authors PhysxRigidBodyAPI and
            # sleep thresholds onto already-running bodies, invalidating the
            # GPU indices. Binding the reset view must opt out explicitly.
            self.assertFalse(kwargs.get("prepare_contact_sensors", True))
            self.assertFalse(kwargs.get("reset_xform_properties", True))
            return view

        reset = GravelReset([NS(stones=stones)])
        with patch.dict(
            "sys.modules", {"isaacsim.core.prims": NS(RigidPrim=rigid_prim)}
        ):
            reset.bind()
        view.initialize.assert_called_once()
        np.testing.assert_equal(
            [s.position for s in reset.ordered_stones],
            [stones[1].position, stones[0].position],
        )


if __name__ == "__main__":
    unittest.main()
