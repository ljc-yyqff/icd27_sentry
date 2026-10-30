#!/usr/bin/env python3
"""Consistent Gazebo odometry/scan source, or the legacy LIO TF fallback.

authoritative=false preserves the original fallback behavior. With
authoritative=true this is the sole odom->base_footprint TF and odometry
publisher; Point-LIO/loam_interface/sensor_scan_generation must be disabled.
Raw Gazebo lidar scans can be registered using the same timestamped ground
truth and robot-state-publisher sensor extrinsics, without LIO drift.
"""

from array import array
from collections import deque
import copy
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import JointState, PointCloud2, PointField
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def quaternion_matrix(quaternion):
    """Return a 3x3 rotation; ROS quaternions use x,y,z,w ordering."""
    q = np.asarray(quaternion, dtype=float)
    norm = np.dot(q, q)
    if not np.isfinite(norm) or norm < 1e-15:
        raise ValueError("Invalid quaternion")
    x, y, z, w = q / np.sqrt(norm)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def matrix_quaternion(rotation):
    # Eigenvector of the symmetric quaternion matrix is robust at pi yaw.
    m = rotation
    k = (
        np.array(
            [
                [
                    m[0, 0] - m[1, 1] - m[2, 2],
                    m[0, 1] + m[1, 0],
                    m[0, 2] + m[2, 0],
                    m[2, 1] - m[1, 2],
                ],
                [
                    m[0, 1] + m[1, 0],
                    m[1, 1] - m[0, 0] - m[2, 2],
                    m[1, 2] + m[2, 1],
                    m[0, 2] - m[2, 0],
                ],
                [
                    m[0, 2] + m[2, 0],
                    m[1, 2] + m[2, 1],
                    m[2, 2] - m[0, 0] - m[1, 1],
                    m[1, 0] - m[0, 1],
                ],
                [m[2, 1] - m[1, 2], m[0, 2] - m[2, 0], m[1, 0] - m[0, 1], m.trace()],
            ]
        )
        / 3.0
    )
    _, vectors = np.linalg.eigh(k)
    q = vectors[:, -1]
    return q if q[3] >= 0 else -q


def rigid_matrix(translation, quaternion):
    result = np.eye(4)
    result[:3, :3] = quaternion_matrix(quaternion)
    result[:3, 3] = translation
    return result


def base_pose_from_full_chassis(position, quaternion, chassis_from_base, spawn_x, spawn_y):
    """Convert physical world->chassis into odom->URDF base, without flattening.

    The fixed URDF base->chassis offset is already represented by RSP. Subtract
    it in the tilted chassis axes exactly once, rather than using suspension
    travel as a substitute for physical height.
    """
    if not np.isfinite(position).all() or not np.isfinite(chassis_from_base).all():
        raise ValueError("Nonfinite physical chassis pose")
    pose = rigid_matrix(position, quaternion) @ chassis_from_base
    pose[0, 3] -= spawn_x
    pose[1, 3] -= spawn_y
    return pose


def transform_matrix(transform):
    p, q = transform.translation, transform.rotation
    return rigid_matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])


def interpolate_pose(history, stamp_ns):
    """Interpolate bracketed GT only; never extrapolate the latest pose."""
    if not history or stamp_ns < history[0][0] or stamp_ns > history[-1][0]:
        return None
    for index, (stamp, position, quaternion) in enumerate(history):
        if stamp == stamp_ns:
            return rigid_matrix(position, quaternion)
        if stamp > stamp_ns:
            previous, previous_position, previous_q = history[index - 1]
            fraction = (stamp_ns - previous) / (stamp - previous)
            q0, q1 = np.asarray(previous_q), np.asarray(quaternion)
            dot = float(np.dot(q0, q1))
            if dot < 0:
                q1, dot = -q1, -dot
            if dot > 0.9995:
                q = q0 + fraction * (q1 - q0)
                q /= np.linalg.norm(q)
            else:
                angle = np.arccos(np.clip(dot, -1.0, 1.0))
                q = (
                    np.sin((1 - fraction) * angle) * q0 + np.sin(fraction * angle) * q1
                ) / np.sin(angle)
            p = np.asarray(previous_position) + fraction * (
                np.asarray(position) - previous_position
            )
            return rigid_matrix(p, q)
    return None


def publication_due(previous_stamp_ns, stamp_ns, period_ns):
    return (
        previous_stamp_ns is None
        or stamp_ns < previous_stamp_ns
        or stamp_ns - previous_stamp_ns >= period_ns
    )


def pose_odometry(stamp, matrix, frame, child):
    """Construct a pose-only odometry message for the specified physical frame."""
    out = Odometry()
    out.header.stamp, out.header.frame_id, out.child_frame_id = stamp, frame, child
    p = out.pose.pose.position
    p.x, p.y, p.z = map(float, matrix[:3, 3])
    q = out.pose.pose.orientation
    q.x, q.y, q.z, q.w = map(float, matrix_quaternion(matrix[:3, :3]))
    return out


def validated_self_filter_bounds(bounds):
    """Validate [xmin,xmax,ymin,ymax,zmin,zmax]; an empty list disables it."""
    if len(bounds) == 0:
        return None
    values = np.asarray(bounds, dtype=float)
    if values.shape != (6,) or not np.isfinite(values).all():
        raise ValueError("self_filter_bounds must be [] or six finite numbers")
    if np.any(values[0::2] >= values[1::2]):
        raise ValueError("self_filter_bounds requires min < max on every axis")
    return tuple(float(value) for value in values)


def transform_cloud(
    message, matrix, frame, self_filter_matrix=None, self_filter_bounds=None
):
    """Preserve layout/stamp; transform XYZ and mask only the chassis envelope.

    self_filter_matrix maps the original sensor XYZ into the chassis frame at
    this message's timestamp. Masked points keep their other fields and slots.
    """
    bounds = validated_self_filter_bounds(
        [] if self_filter_bounds is None else self_filter_bounds
    )
    if bounds is not None and self_filter_matrix is None:
        raise ValueError(
            "A chassis-from-sensor transform is required for self-filtering"
        )
    fields = {field.name: field for field in message.fields}
    types = {PointField.FLOAT32: "f4", PointField.FLOAT64: "f8"}
    if any(
        name not in fields
        or fields[name].count != 1
        or fields[name].datatype not in types
        for name in ("x", "y", "z")
    ):
        raise ValueError("PointCloud2 must contain scalar floating x/y/z")
    if message.point_step <= 0 or message.row_step < message.point_step * message.width:
        raise ValueError("Invalid PointCloud2 stride")
    if len(message.data) < message.row_step * message.height:
        raise ValueError("Truncated PointCloud2 data")
    result = copy.deepcopy(message)
    raw = bytearray(message.data)
    endian = ">" if message.is_bigendian else "<"
    dtype = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": [endian + types[fields[n].datatype] for n in ("x", "y", "z")],
            "offsets": [fields[n].offset for n in ("x", "y", "z")],
            "itemsize": message.point_step,
        }
    )
    points = np.ndarray(
        (message.height, message.width),
        dtype=dtype,
        buffer=raw,
        strides=(message.row_step, message.point_step),
    )
    xyz = np.stack([points["x"], points["y"], points["z"]], axis=-1)
    valid = np.isfinite(xyz).all(axis=-1)
    transformed = xyz[valid] @ matrix[:3, :3].T + matrix[:3, 3]
    if bounds is not None:
        chassis_xyz = (
            xyz[valid] @ self_filter_matrix[:3, :3].T + self_filter_matrix[:3, 3]
        )
        limits = np.asarray(bounds).reshape(3, 2)
        inside = ((chassis_xyz >= limits[:, 0]) & (chassis_xyz <= limits[:, 1])).all(
            axis=-1
        )
        transformed[inside] = np.nan
        if inside.any():
            result.is_dense = False
    for dimension, name in enumerate(("x", "y", "z")):
        points[name][valid] = transformed[:, dimension]
    result.data = array("B", raw)
    result.header.frame_id = frame
    return result


class GazeboGroundTruthTf(Node):
    def __init__(self):
        super().__init__("gazebo_ground_truth_tf")
        self.declare_parameter("spawn_x", 0.0)
        self.declare_parameter("spawn_y", 0.0)
        self.spawn_x = self.get_parameter("spawn_x").value
        self.spawn_y = self.get_parameter("spawn_y").value
        self.authoritative = self.declare_parameter("authoritative", False).value
        self.full_ground_truth = self.declare_parameter("use_full_ground_truth", False).value
        self.chassis_from_base_matrix = None
        if self.full_ground_truth and not self.authoritative:
            raise ValueError("Full physical ground truth requires authoritative=true")
        self.publish_scan = self.declare_parameter(
            "publish_registered_scan", False
        ).value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_footprint").value
        self.robot_base_frame = self.declare_parameter(
            "robot_base_frame", "gimbal_yaw"
        ).value
        self.lidar_frame = self.declare_parameter("lidar_frame", "front_mid360").value
        self.chassis_frame = self.declare_parameter("chassis_frame", "chassis").value
        self.self_filter_bounds = validated_self_filter_bounds(
            self.declare_parameter(
                "self_filter_bounds",
                [],
                ParameterDescriptor(dynamic_typing=True),
            ).value
        )
        self.sensor_offset_z = self.declare_parameter(
            "lidar_sensor_offset_z", 0.03
        ).value
        self.tf_wait_timeout = self.declare_parameter("tf_wait_timeout", 0.5).value
        publish_rate = self.declare_parameter("odometry_publish_rate", 50.0).value
        if not np.isfinite(publish_rate) or publish_rate <= 0:
            raise ValueError("odometry_publish_rate must be positive")
        self.odometry_period_ns = max(1, int(1e9 / publish_rate))
        self.last_published_gt_ns = None
        if self.publish_scan and not self.authoritative:
            raise ValueError("Registered scan publishing requires authoritative=true")
        self.last_lio_odometry = None
        self.lift_position = 0.0
        self.broadcaster = TransformBroadcaster(self)
        self.history = deque(maxlen=1000)
        self.pending_odometry = deque(maxlen=200)
        self.pending_clouds = deque(maxlen=10)
        self.previous_odometry = None
        self.last_warning = -1e9
        if self.authoritative:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.odom_pub = self.create_publisher(Odometry, "odometry", 10)
            self.create_timer(0.02, self.drain)
            if self.publish_scan:
                self.scan_pub = self.create_publisher(PointCloud2, "registered_scan", 5)
                self.sensor_pub = self.create_publisher(PointCloud2, "sensor_scan", 5)
                self.lidar_odom_pub = self.create_publisher(
                    Odometry, "lidar_odometry", 5
                )
                self.create_subscription(
                    PointCloud2, "livox/lidar", self.on_cloud, qos_profile_sensor_data
                )
            self.get_logger().info(
                "Gazebo is the authoritative TF, odometry and scan source"
            )
        else:
            self.create_subscription(
                Odometry, "odometry", self.on_lio_odometry, qos_profile_sensor_data
            )
        self.create_subscription(
            Odometry,
            "chassis_pose_gt" if self.full_ground_truth else "chassis_odometry_gt",
            self.on_ground_truth,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointState, "joint_states", self.on_joints, qos_profile_sensor_data
        )

    def on_joints(self, message):
        for name, position in zip(message.name, message.position):
            if name == "chassis_lift_joint" and 0.0 <= position <= 0.060:
                self.lift_position = position

    def on_lio_odometry(self, _message):
        self.last_lio_odometry = time.monotonic()

    def on_ground_truth(self, message):
        if (
            not self.authoritative
            and self.last_lio_odometry is not None
            and time.monotonic() - self.last_lio_odometry < 1.0
        ):
            return

        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = message.pose.pose.position.x - self.spawn_x
        transform.transform.translation.y = message.pose.pose.position.y - self.spawn_y
        # MecanumDrive2's planar odometry does not populate z. The wheel carrier
        # remains on the level floor while the chassis drops by the measured
        # prismatic travel; account for it instead of shifting the lidar map up.
        transform.transform.translation.z = -self.lift_position
        transform.transform.rotation = message.pose.pose.orientation
        if self.full_ground_truth:
            try:
                if self.chassis_from_base_matrix is None:
                    base_from_chassis = self.tf_buffer.lookup_transform(
                        self.base_frame, self.chassis_frame, Time()
                    )
                    self.chassis_from_base_matrix = np.linalg.inv(
                        transform_matrix(base_from_chassis.transform)
                    )
                p, q = message.pose.pose.position, message.pose.pose.orientation
                physical_base = base_pose_from_full_chassis(
                    [p.x, p.y, p.z], [q.x, q.y, q.z, q.w],
                    self.chassis_from_base_matrix, self.spawn_x, self.spawn_y,
                )
                out_p, out_q = transform.transform.translation, transform.transform.rotation
                out_p.x, out_p.y, out_p.z = map(float, physical_base[:3, 3])
                out_q.x, out_q.y, out_q.z, out_q.w = map(
                    float, matrix_quaternion(physical_base[:3, :3])
                )
            except (TransformException, ValueError) as error:
                self.warn_drop("full physical ground truth", error)
                return
        if self.authoritative:
            stamp = (
                message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            )
            if self.history and stamp < self.history[-1][0]:
                self.history.clear()
                self.pending_clouds.clear()
                self.pending_odometry.clear()
                self.previous_odometry = None
                self.last_published_gt_ns = None
            p, q = transform.transform.translation, transform.transform.rotation
            self.history.append(
                (stamp, np.array([p.x, p.y, p.z]), np.array([q.x, q.y, q.z, q.w]))
            )
            # Keep full-rate GT for exact cloud-time interpolation, but do not
            # flood every Nav2 TF buffer and executor with the simulator's
            # ~1000 Hz odometry. Fifty Hz exceeds the control / costmap rates.
            if not publication_due(
                self.last_published_gt_ns, stamp, self.odometry_period_ns
            ):
                return
            self.last_published_gt_ns = stamp
            self.broadcaster.sendTransform(transform)
            self.pending_odometry.append(
                (
                    time.monotonic(),
                    message.header.stamp,
                    transform_matrix(transform.transform),
                )
            )
        else:
            self.broadcaster.sendTransform(transform)

    def on_cloud(self, message):
        self.pending_clouds.append((time.monotonic(), message))

    def warn_drop(self, kind, error):
        if time.monotonic() - self.last_warning > 2.0:
            self.get_logger().warning(
                f"Dropping {kind}; exact timestamp transform unavailable: {error}"
            )
            self.last_warning = time.monotonic()

    def drain(self):
        # Nonblocking lookups permit the TF listener to receive transforms in
        # this same executor before retrying; no latest-TF substitution.
        while self.pending_odometry:
            queued_at, stamp, odom_from_base = self.pending_odometry[0]
            try:
                base_from_robot = self.tf_buffer.lookup_transform(
                    self.base_frame, self.robot_base_frame, Time.from_msg(stamp)
                )
            except TransformException as error:
                if time.monotonic() - queued_at < self.tf_wait_timeout:
                    break
                self.pending_odometry.popleft()
                self.warn_drop("odometry", error)
                continue
            self.pending_odometry.popleft()
            self.publish_odometry(
                stamp, odom_from_base @ transform_matrix(base_from_robot.transform)
            )

        while self.pending_clouds:
            queued_at, cloud = self.pending_clouds[0]
            stamp = cloud.header.stamp.sec * 1_000_000_000 + cloud.header.stamp.nanosec
            odom_from_base = interpolate_pose(self.history, stamp)
            try:
                if odom_from_base is None:
                    raise ValueError("no bracketed Gazebo pose")
                base_from_lidar = self.tf_buffer.lookup_transform(
                    self.base_frame, self.lidar_frame, Time.from_msg(cloud.header.stamp)
                )
                if self.self_filter_bounds is not None:
                    base_from_chassis = self.tf_buffer.lookup_transform(
                        self.base_frame,
                        self.chassis_frame,
                        Time.from_msg(cloud.header.stamp),
                    )
            except (TransformException, ValueError) as error:
                if time.monotonic() - queued_at < self.tf_wait_timeout:
                    break
                self.pending_clouds.popleft()
                self.warn_drop("registered scan", error)
                continue
            self.pending_clouds.popleft()
            try:
                # gpu_lidar has an additional sensor pose (0,0,.03) inside
                # the front_mid360 link, despite using that link's frame id.
                lidar_from_sensor = np.eye(4)
                lidar_from_sensor[2, 3] = self.sensor_offset_z
                matrix = (
                    odom_from_base
                    @ transform_matrix(base_from_lidar.transform)
                    @ lidar_from_sensor
                )
                # The terrain analyzers consume lidar_odometry, NOT the virtual
                # navigation-base odometry. Preserve the original loam_interface
                # contract, including the real lidar installation rotation.
                self.lidar_odom_pub.publish(
                    pose_odometry(
                        cloud.header.stamp,
                        odom_from_base @ transform_matrix(base_from_lidar.transform),
                        self.odom_frame,
                        self.lidar_frame,
                    )
                )
                chassis_from_sensor = None
                if self.self_filter_bounds is not None:
                    chassis_from_sensor = (
                        np.linalg.inv(transform_matrix(base_from_chassis.transform))
                        @ transform_matrix(base_from_lidar.transform)
                        @ lidar_from_sensor
                    )
                self.scan_pub.publish(
                    transform_cloud(
                        cloud,
                        matrix,
                        self.odom_frame,
                        chassis_from_sensor,
                        self.self_filter_bounds,
                    )
                )
                self.sensor_pub.publish(
                    transform_cloud(
                        cloud,
                        lidar_from_sensor,
                        self.lidar_frame,
                        chassis_from_sensor,
                        self.self_filter_bounds,
                    )
                )
            except ValueError as error:
                self.warn_drop("malformed point cloud", error)

    def publish_odometry(self, stamp, matrix):
        out = pose_odometry(stamp, matrix, self.odom_frame, self.robot_base_frame)
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if self.previous_odometry is not None:
            previous_ns, previous = self.previous_odometry
            dt = (stamp_ns - previous_ns) / 1e9
            if 1e-6 < dt < 0.5:
                linear = matrix[:3, :3].T @ ((matrix[:3, 3] - previous[:3, 3]) / dt)
                delta = matrix[:3, :3] @ previous[:3, :3].T
                angle = np.arccos(np.clip((delta.trace() - 1) / 2, -1.0, 1.0))
                vector = np.array(
                    [
                        delta[2, 1] - delta[1, 2],
                        delta[0, 2] - delta[2, 0],
                        delta[1, 0] - delta[0, 1],
                    ]
                )
                factor = 0.5 if angle < 1e-6 else angle / (2 * np.sin(angle))
                angular = matrix[:3, :3].T @ (vector * factor / dt)
                v, w = out.twist.twist.linear, out.twist.twist.angular
                v.x, v.y, v.z = map(float, linear)
                w.x, w.y, w.z = map(float, angular)
        self.previous_odometry = (stamp_ns, matrix)
        for axis in range(6):
            out.pose.covariance[axis * 7] = 1e-6
            out.twist.covariance[axis * 7] = 1e-5
        self.odom_pub.publish(out)


def main():
    rclpy.init()
    node = GazeboGroundTruthTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
