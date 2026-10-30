#!/usr/bin/env python3
"""Offline geometry/PointCloud2 regression tests; source ROS setup first."""

from array import array
import importlib.util
from pathlib import Path
import struct
import unittest

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField

SOURCE = (
    Path(__file__).resolve().parents[1]
    / "icd2027sentry_nav_bringup/scripts/gazebo_ground_truth_tf.py"
)
spec = importlib.util.spec_from_file_location("gazebo_ground_truth_tf", SOURCE)
helper = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helper)


class GroundTruthGeometryTest(unittest.TestCase):
    def test_full_chassis_pose_preserves_tilt_and_exact_urdf_offset(self):
        base_from_chassis = helper.rigid_matrix([0, 0, .076], [0, 0, 0, 1])
        q = np.array([.025, -.035, .30, .953])
        actual = helper.rigid_matrix([2.0, -3.0, .023], q)
        base = helper.base_pose_from_full_chassis(
            actual[:3, 3], q, np.linalg.inv(base_from_chassis), -4.8, 3.5
        )
        reconstructed = base @ base_from_chassis
        reconstructed[:2, 3] += [-4.8, 3.5]
        np.testing.assert_allclose(reconstructed, actual, atol=1e-12)
        # A world floor point transformed through the physical sensor pose and
        # back must remain on z=0 even with body roll/pitch and suspension.
        chassis_from_sensor = helper.rigid_matrix([.16, 0, .21], [0, 0, 0, 1])
        world_from_sensor = actual @ chassis_from_sensor
        floor = np.array([3.0, -1.0, 0.0, 1.0])
        raw_point = np.linalg.inv(world_from_sensor) @ floor
        np.testing.assert_allclose(reconstructed @ chassis_from_sensor @ raw_point, floor, atol=1e-12)

    def test_full_pose_does_not_double_apply_lift_or_accept_invalid_numbers(self):
        chassis_from_base = helper.rigid_matrix([0, 0, -.076], [0, 0, 0, 1])
        for height in (.076, .021):
            base = helper.base_pose_from_full_chassis(
                [1, 2, height], [0, 0, 0, 1], chassis_from_base, 0, 0
            )
            self.assertAlmostEqual(base[2, 3], height-.076)
        with self.assertRaises(ValueError):
            helper.base_pose_from_full_chassis(
                [1, 2, float("nan")], [0, 0, 0, 1], chassis_from_base, 0, 0
            )

    def test_gt_publication_is_limited_without_losing_time_jump_recovery(self):
        previous = None
        published = []
        for stamp in range(0, 1_000_000_000, 1_000_000):
            if helper.publication_due(previous, stamp, 20_000_000):
                previous = stamp
                published.append(stamp)
        self.assertEqual(len(published), 50)
        self.assertTrue(helper.publication_due(previous, 1, 20_000_000))
        self.assertFalse(helper.publication_due(previous, previous + 1, 20_000_000))

    def test_lidar_odometry_uses_the_actual_sensor_extrinsic_and_frame(self):
        from builtin_interfaces.msg import Time

        stamp = Time(sec=12, nanosec=123)
        odom_from_base = helper.rigid_matrix([6.7, -6.975, -0.055], [0, 0, 0, 1])
        base_from_lidar = helper.rigid_matrix(
            [0.16, 0.0, 0.256], [0.183, 0.183, 0.683, 0.683]
        )
        actual = odom_from_base @ base_from_lidar
        msg = helper.pose_odometry(stamp, actual, "odom", "front_mid360")
        self.assertEqual(msg.header.stamp, stamp)
        self.assertEqual(msg.header.frame_id, "odom")
        self.assertEqual(msg.child_frame_id, "front_mid360")
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        np.testing.assert_allclose([p.x, p.y, p.z], [6.86, -6.975, 0.201])
        np.testing.assert_allclose(
            helper.quaternion_matrix([q.x, q.y, q.z, q.w]), actual[:3, :3], atol=1e-12
        )

    @staticmethod
    def make_padded_cloud(xyz, endian):
        message = PointCloud2()
        message.header.frame_id = "front_mid360"
        message.header.stamp.sec = 17
        message.header.stamp.nanosec = 4321
        message.width, message.height = 2, 2
        message.point_step, message.row_step = 20, 48
        message.is_bigendian = endian == ">"
        message.is_dense = True
        message.fields = [
            PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
            for name, offset in [("x", 0), ("y", 4), ("z", 8), ("intensity", 12)]
        ]
        message.fields.append(
            PointField(name="ring", offset=16, datatype=PointField.UINT32, count=1)
        )
        raw = bytearray([0xA5] * 96)
        for index, point in enumerate(xyz):
            row, col = divmod(index, 2)
            struct.pack_into(
                endian + "ffffI", raw, row * 48 + col * 20, *point, 42.0, 17
            )
        message.data = array("B", raw)
        return message

    def test_quaternion_roundtrip_including_pi_and_sensor_roll(self):
        for q in (
            [0, 0, 1, 0],
            [0, 0, 0, 1],
            [0.2, 0.3, 0.4, 0.5],
            [0.183, 0.183, 0.683, 0.683],
        ):
            rotation = helper.quaternion_matrix(q)
            actual = helper.quaternion_matrix(helper.matrix_quaternion(rotation))
            np.testing.assert_allclose(actual, rotation, atol=1e-12)

    def test_gt_interpolation_never_extrapolates(self):
        history = [
            (10, np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0])),
            (20, np.array([2.0, 0.0, -0.055]), np.array([0.0, 0.0, 1.0, 0.0])),
        ]
        matrix = helper.interpolate_pose(history, 15)
        np.testing.assert_allclose(matrix[:3, 3], [1.0, 0.0, -0.0275])
        np.testing.assert_allclose(matrix[:3, :3] @ [1, 0, 0], [0, 1, 0], atol=1e-12)
        self.assertIsNone(helper.interpolate_pose(history, 9))
        self.assertIsNone(helper.interpolate_pose(history, 21))

    def test_sensor_height_is_transformed_in_lidar_axes(self):
        # Sensor's extra .03 is along lidar z, not chassis z. A 90-degree
        # lidar roll moves this offset into chassis -y.
        base_from_lidar = helper.rigid_matrix(
            [0.16, 0.0, 0.256], [0.70710678, 0.0, 0.0, 0.70710678]
        )
        lidar_from_sensor = np.eye(4)
        lidar_from_sensor[2, 3] = 0.03
        point = base_from_lidar @ lidar_from_sensor @ [0, 0, 0, 1]
        np.testing.assert_allclose(point[:3], [0.16, -0.03, 0.256], atol=1e-10)

    def test_self_filter_bounds_are_validated(self):
        self.assertIsNone(helper.validated_self_filter_bounds([]))
        bounds = [-0.36, 0.36, -0.36, 0.36, -0.08, 0.22]
        self.assertEqual(helper.validated_self_filter_bounds(bounds), tuple(bounds))
        for invalid in (
            [0.0],
            bounds + [1.0],
            [0.0, 0.0, -0.36, 0.36, -0.08, 0.22],
            [-0.36, 0.36, 1.0, -1.0, -0.08, 0.22],
            [-0.36, float("inf"), -0.36, 0.36, -0.08, 0.22],
            [-0.36, 0.36, -0.36, 0.36, float("nan"), 0.22],
        ):
            with self.subTest(bounds=invalid), self.assertRaises(ValueError):
                helper.validated_self_filter_bounds(invalid)

    def test_self_filter_rotated_offset_chassis_preserves_external_points_and_layout(
        self,
    ):
        # Use nontrivial extrinsics: masking raw sensor XYZ directly would
        # discard the wrong points. Both output frames must mask the same slot.
        base_from_chassis = helper.rigid_matrix(
            [0.2, -0.1, 0.076], [0.0, 0.0, 0.70710678, 0.70710678]
        )
        base_from_lidar = helper.rigid_matrix(
            [0.16, 0.0, 0.256], [0.70710678, 0.0, 0.0, 0.70710678]
        )
        lidar_from_sensor = np.eye(4)
        lidar_from_sensor[2, 3] = 0.03
        chassis_from_sensor = (
            np.linalg.inv(base_from_chassis) @ base_from_lidar @ lidar_from_sensor
        )
        # The low obstacle lies just outside the vehicle, and the .229 high
        # roof lies above the .22 body envelope even at lowered ride height.
        chassis_xyz = np.array(
            [[0.32, 0.25, 0.07], [0.45, 0.25, -0.06], [0.0, 0.0, 0.229]]
        )
        sensor_xyz = (chassis_xyz - chassis_from_sensor[:3, 3]) @ chassis_from_sensor[
            :3, :3
        ]
        sensor_xyz = np.vstack([sensor_xyz, [np.nan, 2.0, 3.0]])
        bounds = [-0.36, 0.36, -0.36, 0.36, -0.08, 0.22]
        odom_from_sensor = (
            helper.rigid_matrix([4.0, 5.0, -0.055], [0.0, 0.0, 1.0, 0.0])
            @ base_from_lidar
            @ lidar_from_sensor
        )
        for endian in ("<", ">"):
            message = self.make_padded_cloud(sensor_xyz, endian)
            original_bytes = array("B", message.data)
            for frame, matrix in (
                ("odom", odom_from_sensor),
                ("front_mid360", lidar_from_sensor),
            ):
                with self.subTest(endian=endian, frame=frame):
                    result = helper.transform_cloud(
                        message, matrix, frame, chassis_from_sensor, bounds
                    )
                    self.assertEqual(result.header.frame_id, frame)
                    self.assertEqual(result.header.stamp, message.header.stamp)
                    self.assertEqual(result.fields, message.fields)
                    self.assertEqual((result.width, result.height), (2, 2))
                    self.assertEqual((result.point_step, result.row_step), (20, 48))
                    self.assertEqual(result.is_bigendian, message.is_bigendian)
                    self.assertFalse(result.is_dense)
                    self.assertEqual(result.data[40:48], message.data[40:48])
                    self.assertEqual(result.data[88:96], message.data[88:96])
                    for index in range(4):
                        row, col = divmod(index, 2)
                        offset = row * 48 + col * 20
                        x, y, z, intensity, ring = struct.unpack_from(
                            endian + "ffffI", result.data, offset
                        )
                        self.assertEqual((intensity, ring), (42.0, 17))
                        if index == 0:
                            self.assertTrue(np.isnan([x, y, z]).all())
                        elif index == 3:
                            self.assertTrue(np.isnan(x))
                            self.assertEqual((y, z), (2.0, 3.0))
                        else:
                            original = struct.unpack_from(
                                endian + "fff", message.data, offset
                            )
                            expected = matrix[:3, :3] @ original + matrix[:3, 3]
                            np.testing.assert_allclose([x, y, z], expected, atol=1e-6)
                    self.assertEqual(message.data, original_bytes)
            # Disabled filtering remains the backwards-compatible default.
            result = helper.transform_cloud(message, odom_from_sensor, "odom")
            self.assertTrue(
                np.isfinite(struct.unpack_from(endian + "fff", result.data, 0)).all()
            )
            self.assertTrue(result.is_dense)

        with self.assertRaisesRegex(ValueError, "chassis-from-sensor"):
            helper.transform_cloud(message, odom_from_sensor, "odom", None, bounds)

    def test_point_cloud_preserves_other_fields_nan_padding_and_stamp(self):
        for endian, big in [("<", False), (">", True)]:
            message = PointCloud2()
            message.header.frame_id = "front_mid360"
            message.header.stamp.sec = 7
            message.header.stamp.nanosec = 1234
            message.width, message.height = 2, 2
            message.point_step, message.row_step = 20, 48
            message.is_bigendian = big
            message.is_dense = False
            message.fields = [
                PointField(name=n, offset=o, datatype=PointField.FLOAT32, count=1)
                for n, o in [("x", 0), ("y", 4), ("z", 8), ("intensity", 12)]
            ]
            message.fields.append(
                PointField(name="ring", offset=16, datatype=PointField.UINT32, count=1)
            )
            raw = bytearray([0xA5] * 96)
            for row in range(2):
                for col in range(2):
                    struct.pack_into(
                        endian + "ffffI",
                        raw,
                        row * 48 + col * 20,
                        float("nan") if (row, col) == (1, 1) else 1.0,
                        2.0,
                        3.0,
                        42.0,
                        17,
                    )
            message.data = array("B", raw)
            matrix = helper.rigid_matrix([4.0, 5.0, -0.055], [0, 0, 1, 0])
            result = helper.transform_cloud(message, matrix, "odom")
            self.assertEqual(result.header.frame_id, "odom")
            self.assertEqual(result.header.stamp, message.header.stamp)
            self.assertEqual(result.fields, message.fields)
            self.assertEqual(result.data[40:48], message.data[40:48])
            self.assertEqual(result.data[88:96], message.data[88:96])
            for row in range(2):
                for col in range(2):
                    offset = row * 48 + col * 20
                    x, y, z, intensity, ring = struct.unpack_from(
                        endian + "ffffI", result.data, offset
                    )
                    self.assertEqual((intensity, ring), (42.0, 17))
                    if (row, col) == (1, 1):
                        self.assertTrue(np.isnan(x))
                        self.assertEqual((y, z), (2.0, 3.0))
                    else:
                        np.testing.assert_allclose(
                            [x, y, z], [3.0, 3.0, 2.945], atol=1e-6
                        )
            # Callback consumers must never mutate the incoming message.
            self.assertEqual(message.data, array("B", raw))


if __name__ == "__main__":
    unittest.main()
