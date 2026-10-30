#!/usr/bin/env python3
"""Geometry regressions for tunnel-map preparation, independent of ROS."""

import unittest

import numpy as np
from PIL import Image
import yaml

from prepare_rmul2027_tunnels import (
    DEFAULT_ARTIFACTS,
    DEFAULT_CONFIG,
    DEFAULT_MAP,
    DEFAULT_MESH,
    clip_height,
    intersects_cell,
    classify_tunnel_cells,
    load_binary_stl,
)


class TunnelGeometryTest(unittest.TestCase):
    def test_vertical_wall_is_not_lost_by_projection(self):
        wall = np.array([[0.5, 0.0, 0.0], [0.5, 1.0, 0.0], [0.5, 1.0, 0.4]])
        polygon = clip_height(wall, 0.001, 0.243)
        self.assertTrue(intersects_cell(polygon[:, :2], 0.45, 0.5, 0.1))
        self.assertFalse(intersects_cell(polygon[:, :2], 0.6, 0.5, 0.1))

    def test_ceiling_above_robot_does_not_block(self):
        ceiling = np.array([[0.0, 0.0, 0.25], [1.0, 0.0, 0.25], [1.0, 1.0, 0.25]])
        self.assertEqual(len(clip_height(ceiling, 0.001, 0.243)), 0)

    def test_new_low_obstacle_is_preserved(self):
        def plane(z):
            return [
                [[0.0, 0.0, z], [1.0, 0.0, z], [1.0, 1.0, z]],
                [[0.0, 0.0, z], [1.0, 1.0, z], [0.0, 1.0, z]],
            ]

        tunnel = dict(
            id="test",
            x_min=0.0,
            x_max=1.0,
            y_min=0.0,
            y_max=1.0,
            floor_z=0.0,
            ceiling_z=0.25,
        )
        config = dict(lowered_height=0.233, height_margin=0.01)
        info = dict(resolution=0.1, origin=[0.0, 0.0, 0.0])
        mesh = np.array(
            plane(0.0)
            + plane(0.25)
            + [[[0.45, 0.3, 0.05], [0.45, 0.7, 0.05], [0.45, 0.7, 0.2]]]
        )
        rows = classify_tunnel_cells(mesh, tunnel, config, info, (10, 10))
        by_index = {(r["map_x"], r["map_y"]): r for r in rows}
        self.assertTrue(by_index[(7, 5)]["free"])
        self.assertFalse(by_index[(4, 5)]["free"])
        self.assertEqual(by_index[(4, 5)]["reason"], "low_mesh_or_side_wall")

    def test_insufficient_net_clearance_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "insufficient headroom"):
            classify_tunnel_cells(
                np.zeros((1, 3, 3)),
                dict(
                    id="bad",
                    x_min=0.0,
                    x_max=1.0,
                    y_min=0.0,
                    y_max=1.0,
                    floor_z=0.02,
                    ceiling_z=0.25,
                ),
                dict(lowered_height=0.233, height_margin=0.01),
                dict(resolution=0.1, origin=[0.0, 0.0, 0.0]),
                (10, 10),
            )


class ActualFieldMapTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = yaml.safe_load(DEFAULT_CONFIG.read_text())
        cls.info = yaml.safe_load(DEFAULT_MAP.read_text())
        cls.image = np.asarray(Image.open(DEFAULT_MAP.parent / cls.info["image"]))
        mesh = load_binary_stl(DEFAULT_MESH)
        cls.rows = [
            row
            for tunnel in cls.config["tunnels"]
            for row in classify_tunnel_cells(
                mesh, tunnel, cls.config, cls.info, cls.image.shape
            )
        ]

    def test_actual_cells_are_verified_and_walls_preserved(self):
        original = np.asarray(Image.open(DEFAULT_ARTIFACTS / "before.png"))
        for tunnel in self.config["tunnels"]:
            rows = [r for r in self.rows if r["tunnel"] == tunnel["id"]]
            roof_rows = [
                r
                for r in rows
                if tunnel["x_min"] - 0.025 <= r["x"] <= tunnel["x_max"] + 0.025
            ]
            self.assertEqual(sum(r["free"] for r in roof_rows), 120)
            self.assertGreaterEqual(
                sum(r["reason"] == "low_mesh_or_side_wall" for r in rows), 16
            )
            for row in rows:
                value = self.image[row["image_row"], row["map_x"]]
                expected = (
                    254 if row["free"] else original[row["image_row"], row["map_x"]]
                )
                self.assertEqual(value, expected)

    def test_only_verified_cells_differ_from_original(self):
        original = np.asarray(Image.open(DEFAULT_ARTIFACTS / "before.png"))
        changed = set(zip(*np.nonzero(original != self.image)))
        verified = {(r["image_row"], r["map_x"]) for r in self.rows if r["free"]}
        self.assertTrue(changed.issubset(verified))
        self.assertEqual(len(changed), 302)

    def test_correction_is_idempotent(self):
        result = self.image.copy()
        for row in self.rows:
            if row["free"]:
                result[row["image_row"], row["map_x"]] = 254
        np.testing.assert_array_equal(self.image, result)

    def test_real_wheel_width_and_margin_fit_raster_centre(self):
        # The true mesh centre is +/-3.46. Conservative 5 cm rasterization
        # shifts the centre of completely free cells to +/-3.475.
        # Check the full configured footprint plus 1 cm position tolerance,
        # not only the centre point of a proposed path.
        res = self.info["resolution"]
        ox, oy, _ = self.info["origin"]
        for tunnel in self.config["tunnels"]:
            sign = 1 if tunnel["y_min"] > 0 else -1
            cy = tunnel["center_y"]
            cx = (tunnel["x_min"] + tunnel["x_max"]) / 2
            for dx in np.linspace(-0.37, 0.37, 31):
                for dy in np.linspace(-0.37, 0.37, 31):
                    mx, my = int((cx + dx - ox) / res), int((cy + dy - oy) / res)
                    self.assertEqual(self.image[self.image.shape[0] - 1 - my, mx], 254)


if __name__ == "__main__":
    unittest.main()
