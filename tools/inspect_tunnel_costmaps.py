#!/usr/bin/env python3
"""Read-only snapshot of tunnel costmaps, plans and cloud/CAD residuals.

No goals, velocity commands, parameters, reset requests or actuator commands are
sent. GetCostmap services are used only to obtain complete current costmaps.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import time

import numpy as np
import yaml


def transform_matrix(transform):
    q = transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    norm = x*x + y*y + z*z + w*w
    if norm < 1e-12:
        raise ValueError("Invalid quaternion")
    s = 2.0 / norm
    matrix = np.eye(4)
    matrix[:3, :3] = [
        [1-s*(y*y+z*z), s*(x*y-z*w), s*(x*z+y*w)],
        [s*(x*y+z*w), 1-s*(x*x+z*z), s*(y*z-x*w)],
        [s*(x*z-y*w), s*(y*z+x*w), 1-s*(x*x+y*y)],
    ]
    p = transform.translation
    matrix[:3, 3] = [p.x, p.y, p.z]
    return matrix


def cloud_points(message):
    fields = {field.name: field for field in message.fields}
    endian = ">" if message.is_bigendian else "<"
    names = [name for name in ("x", "y", "z", "intensity") if name in fields]
    formats = {7: "f4", 8: "f8", 2: "u1", 4: "u2", 6: "u4"}
    dtype = np.dtype(dict(
        names=names,
        formats=[endian + formats[fields[name].datatype] for name in names],
        offsets=[fields[name].offset for name in names],
        itemsize=message.point_step,
    ))
    points = np.ndarray((message.height, message.width), dtype=dtype,
                        buffer=message.data, strides=(message.row_step, message.point_step))
    xyz = np.column_stack([points[name].ravel() for name in ("x", "y", "z")])
    valid = np.isfinite(xyz).all(axis=1)
    intensity = points["intensity"].ravel()[valid] if "intensity" in names else None
    return xyz[valid], intensity


def histogram(values, step=0.002, limit=8):
    if not len(values):
        return []
    bins, counts = np.unique(np.rint(values/step).astype(int), return_counts=True)
    order = np.argsort(counts)[-limit:][::-1]
    return [[round(float(bins[index]*step), 5), int(counts[index])] for index in order]


def cloud_summary(points, intensity, config):
    result = {"count": len(points)}
    floor = points[(points[:, 2] > -0.08) & (points[:, 2] < 0.08), 2]
    result["ground_z_peaks_2mm"] = histogram(floor)
    result["ground_median"] = float(np.median(floor)) if len(floor) else None
    result["initial_robot_corner_probes"] = {}
    for label, x, y in (("front_right", -4.475, 3.225), ("front_left", -4.475, 3.725)):
        selected = ((np.abs(points[:, 0]-x) <= .025) & (np.abs(points[:, 1]-y) <= .025))
        selected_points = points[selected]
        result["initial_robot_corner_probes"][label] = dict(
            count=len(selected_points),
            z_range=[float(selected_points[:, 2].min()), float(selected_points[:, 2].max())]
            if len(selected_points) else None,
            xyz_samples=selected_points[:12].tolist(),
            intensity_samples=intensity[selected][:12].tolist() if intensity is not None else None)
    result["tunnels"] = {}
    for tunnel in config["tunnels"]:
        selected = ((points[:, 0] >= tunnel["x_min"] - .05) &
                    (points[:, 0] <= tunnel["x_max"] + .05) &
                    (points[:, 1] >= tunnel["y_min"] - .12) &
                    (points[:, 1] <= tunnel["y_max"] + .12))
        near = points[selected]
        summary = {"near_count": len(near)}
        if len(near):
            summary["z_range"] = [float(near[:, 2].min()), float(near[:, 2].max())]
            summary["z_peaks_2mm"] = histogram(near[:, 2])
            interior = ((near[:, 0] > tunnel["x_min"]-.01) &
                        (near[:, 0] < tunnel["x_max"]+.01) &
                        (near[:, 1] > tunnel["y_min"]+.01) &
                        (near[:, 1] < tunnel["y_max"]-.01))
            roof = (interior &
                    (near[:, 2] >= tunnel["floor_z"]+config["lowered_height"]+config["height_margin"]) &
                    (near[:, 2] >= tunnel["ceiling_z"]-.005) &
                    (near[:, 2] <= tunnel["ceiling_z"]+.055))
            summary["eligible_known_roof"] = int(roof.sum())
            summary["interior_below_roof"] = int((interior & (near[:, 2] > .08) &
                                                   (near[:, 2] < tunnel["ceiling_z"]-.005)).sum())
            walls = near[(near[:, 2] > .06) & (near[:, 2] < tunnel["ceiling_z"]-.02)]
            summary["wall_y_residuals"] = {}
            for side in ("y_min", "y_max"):
                residual = walls[:, 1] - tunnel[side]
                residual = residual[np.abs(residual) < .12]
                summary["wall_y_residuals"][side] = dict(
                    count=len(residual), peaks=histogram(residual),
                    median=float(np.median(residual)) if len(residual) else None)
            if intensity is not None:
                values = intensity[selected]
                summary["intensity_range"] = [float(values.min()), float(values.max())]
        result["tunnels"][tunnel["id"]] = summary
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain-id", type=int, default=91)
    parser.add_argument("--robot", default="red_standard_robot1")
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--wait-for-plan", action="store_true",
                        help="Finish two seconds after a plan arrives, or at the duration timeout")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--config", type=Path, default=Path(__file__).resolve().parents[1] /
                        "icd2027sentry_nav_bringup/config/simulation/rmul_2027_tunnels.yaml")
    args = parser.parse_args()
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    config = yaml.safe_load(args.config.read_text())
    import rclpy
    from nav2_msgs.srv import GetCostmap
    from nav2_msgs.msg import Costmap
    from nav_msgs.msg import OccupancyGrid, Path as RosPath
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.duration import Duration
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, Float64MultiArray
    from tf2_ros import Buffer, TransformListener

    rclpy.init()
    node = Node("inspect_tunnel_costmaps", namespace=args.robot,
                parameter_overrides=[Parameter("use_sim_time", value=True)],
                cli_args=["--ros-args", "-r", "/tf:=tf", "-r", "/tf_static:=tf_static"])
    tf = Buffer(cache_time=Duration(seconds=max(60., args.duration+5)))
    listener = TransformListener(tf, node)
    clouds, occupancy, plans = {}, {}, {}
    feedback = {}
    topic_samples = {}
    def record_message(topic, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec/1e9
        receive_ros = node.get_clock().now().nanoseconds/1e9
        topic_samples.setdefault(topic, []).append(dict(
            wall=time.monotonic(), receive_ros=receive_ros, stamp=stamp,
            age=receive_ros-stamp,
            points=getattr(message, "width", 0)*getattr(message, "height", 0),
            data_bytes=len(message.data) if hasattr(message, "data") else 0))
        if isinstance(message, PointCloud2):
            clouds[topic] = message
        elif isinstance(message, Odometry):
            feedback[topic] = message
    subscriptions = []
    for topic in ("registered_scan", "cloud_registered", "terrain_map", "terrain_map_ext", "livox/lidar"):
        subscriptions.append(node.create_subscription(
            PointCloud2, topic, lambda msg, key=topic: record_message(key, msg), qos_profile_sensor_data))
    for topic in ("odometry", "lidar_odometry", "chassis_odometry_gt", "chassis_pose_gt"):
        subscriptions.append(node.create_subscription(
            Odometry, topic, lambda msg, key=topic: record_message(key, msg), qos_profile_sensor_data))
    for topic in ("local_costmap/costmap_raw", "global_costmap/costmap_raw"):
        subscriptions.append(node.create_subscription(
            Costmap, topic, lambda msg, key=topic: record_message(key, msg),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    for topic in ("global_costmap/costmap", "local_costmap/costmap"):
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        subscriptions.append(node.create_subscription(
            OccupancyGrid, topic, lambda msg, key=topic: occupancy.__setitem__(key, msg), qos))
    for topic in ("plan", "local_plan"):
        subscriptions.append(node.create_subscription(
            RosPath, topic, lambda msg, key=topic: plans.__setitem__(key, msg), 10))
    subscriptions.append(node.create_subscription(
        JointState, "joint_states", lambda msg: feedback.__setitem__("joints", msg), qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(
        Bool, "tunnel_local_clearance_ready", lambda msg: feedback.__setitem__("local_ready", msg.data), 10))
    subscriptions.append(node.create_subscription(
        Float64MultiArray, "tunnel_guard_state", lambda msg: feedback.__setitem__("guard", list(msg.data)), 10))
    clients = {name: node.create_client(GetCostmap, name+"/get_costmap")
               for name in ("global_costmap", "local_costmap")}
    futures = {}
    costmap_messages = {}
    last_costmap_request = {}
    started = time.monotonic()
    first_plan_time = None
    print("READ_ONLY_SUBSCRIPTIONS_READY", flush=True)
    while time.monotonic()-started < args.duration:
        rclpy.spin_once(node, timeout_sec=.1)
        if plans and first_plan_time is None:
            first_plan_time = time.monotonic()
        if args.wait_for_plan and first_plan_time is not None and time.monotonic()-first_plan_time > 2:
            break
        for name, future in list(futures.items()):
            if future.done():
                if not future.exception():
                    costmap_messages[name] = future.result().map
                del futures[name]
        for name, client in clients.items():
            if (name not in futures and client.service_is_ready() and
                    time.monotonic()-last_costmap_request.get(name, -1e9) > .5):
                futures[name] = client.call_async(GetCostmap.Request())
                last_costmap_request[name] = time.monotonic()
    for name, future in futures.items():
        if future.done() and not future.exception():
            costmap_messages[name] = future.result().map
    result = {"ros_time": node.get_clock().now().nanoseconds/1e9,
              "seen_topics": sorted(clouds), "costmaps": {}, "clouds": {}, "plans": {}}
    result["feedback"] = {key: value for key, value in feedback.items()
                          if key not in ("joints", "odometry", "lidar_odometry", "chassis_odometry_gt", "chassis_pose_gt")}
    result["topic_timing"] = {}
    for topic, samples in topic_samples.items():
        valid = [item for item in samples if item["receive_ros"] > 0 and item["age"] > -1]
        ages = np.array([item["age"] for item in valid])
        result["topic_timing"][topic] = dict(
            messages=len(samples),
            wall_rate=(len(samples)-1)/(samples[-1]["wall"]-samples[0]["wall"])
            if len(samples)>1 else None,
            stamp_rate=(len(samples)-1)/(samples[-1]["stamp"]-samples[0]["stamp"])
            if len(samples)>1 and samples[-1]["stamp"]>samples[0]["stamp"] else None,
            age_min=float(ages.min()) if len(ages) else None,
            age_p50=float(np.median(ages)) if len(ages) else None,
            age_p95=float(np.quantile(ages,.95)) if len(ages) else None,
            age_max=float(ages.max()) if len(ages) else None,
            first=samples[0], last=samples[-1])
    result["odometry"] = {}
    for topic in ("odometry", "lidar_odometry", "chassis_odometry_gt", "chassis_pose_gt"):
        if topic in feedback:
            message = feedback[topic]
            p, q = message.pose.pose.position, message.pose.pose.orientation
            result["odometry"][topic] = dict(frame=message.header.frame_id,
                child_frame=message.child_frame_id, xyz=[p.x,p.y,p.z], quaternion=[q.x,q.y,q.z,q.w])
    if "joints" in feedback:
        message = feedback["joints"]
        stamp = message.header.stamp.sec + message.header.stamp.nanosec/1e9
        result["feedback"]["joints"] = dict(stamp=stamp, age=result["ros_time"]-stamp,
            positions=dict(zip(message.name, message.position)))
    grid_dump = {}

    def matrix_to(target, source, stamp=None):
        if source == target:
            return np.eye(4)
        lookup_time = rclpy.time.Time.from_msg(stamp) if stamp is not None else rclpy.time.Time()
        return transform_matrix(tf.lookup_transform(target, source, lookup_time).transform)

    for name, message in costmap_messages.items():
        meta = message.metadata
        values = np.asarray(message.data, dtype=np.uint8).reshape(meta.size_y, meta.size_x)
        grid_dump[name] = values
        grid_dump[name+"_origin"] = np.array([meta.origin.position.x, meta.origin.position.y])
        grid_dump[name+"_resolution"] = np.array(meta.resolution)
        try:
            grid_from_map = matrix_to(message.header.frame_id, config["frame"])
            grid_dump[name+"_from_map"] = grid_from_map
            def cost_at(x, y):
                point = grid_from_map @ np.array([x, y, 0., 1.])
                col = math.floor((point[0]-meta.origin.position.x)/meta.resolution)
                row = math.floor((point[1]-meta.origin.position.y)/meta.resolution)
                return int(values[row, col]) if 0 <= col < meta.size_x and 0 <= row < meta.size_y else None
            snapshot = dict(frame=message.header.frame_id, resolution=meta.resolution,
                            stamp=message.header.stamp.sec + message.header.stamp.nanosec/1e9,
                            origin=[meta.origin.position.x, meta.origin.position.y],
                            dimensions=[meta.size_x, meta.size_y], tunnels={})
            for tunnel in config["tunnels"]:
                xs = np.arange(tunnel["x_min"]-.5, tunnel["x_max"]+.501, .05)
                ys = np.arange(tunnel["center_y"]-.45, tunnel["center_y"]+.451, .05)
                rows = [[round(float(y), 3), [cost_at(x, y) for x in xs]] for y in ys]
                snapshot["tunnels"][tunnel["id"]] = dict(
                    x_values=[round(float(x), 3) for x in xs], rows=rows,
                    center_cost=cost_at((tunnel["x_min"]+tunnel["x_max"])/2, tunnel["center_y"]))
            xs = np.arange(-5.025, -3.324, .05)
            ys = np.arange(2.775, 3.926, .05)
            snapshot["red_approach"] = dict(
                x_values=[round(float(x), 3) for x in xs],
                rows=[[round(float(y), 3), [cost_at(x, y) for x in xs]] for y in ys],
                initial_center_cost=cost_at(-4.795096, 3.475203))
            if "plan" in plans:
                samples = [[pose.pose.position.x, pose.pose.position.y,
                            cost_at(pose.pose.position.x, pose.pose.position.y)]
                           for pose in plans["plan"].poses]
                snapshot["plan_samples"] = samples
                snapshot["first_rejected_plan_sample"] = next(
                    (point for point in samples if point[2] is None or point[2] >= 253), None)
            result["costmaps"][name] = snapshot
        except Exception as error:
            result["costmaps"][name] = dict(error=str(error))

    for name, message in clouds.items():
        try:
            xyz, intensity = cloud_points(message)
            matrix = matrix_to(config["frame"], message.header.frame_id, message.header.stamp)
            mapped = xyz @ matrix[:3, :3].T + matrix[:3, 3]
            grid_dump[name+"_points"] = mapped
            if intensity is not None:
                grid_dump[name+"_intensity"] = intensity
            result["clouds"][name] = dict(frame=message.header.frame_id,
                stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9,
                age=result["ros_time"]-message.header.stamp.sec-message.header.stamp.nanosec/1e9,
                width=message.width, height=message.height, data_bytes=len(message.data),
                **cloud_summary(mapped, intensity, config))
            if name == "livox/lidar":
                shifted = (xyz+np.array([0., 0., .03])) @ matrix[:3, :3].T + matrix[:3, 3]
                result["clouds"][name+"_with_link_offset_03"] = cloud_summary(shifted, intensity, config)
                result["clouds"][name]["world_offset_for_link_z03"] = list(matrix[:3, :3] @ [0., 0., .03])
        except Exception as error:
            result["clouds"][name] = dict(error=str(error), frame=message.header.frame_id)
    for name, message in plans.items():
        result["plans"][name] = dict(frame=message.header.frame_id,
            points=[[pose.pose.position.x, pose.pose.position.y] for pose in message.poses],
            orientations=[[pose.pose.orientation.x, pose.pose.orientation.y,
                           pose.pose.orientation.z, pose.pose.orientation.w] for pose in message.poses])
        try:
            map_from_plan = matrix_to(config["frame"], message.header.frame_id, message.header.stamp)
            points = np.array([[pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
                               for pose in message.poses])
            result["plans"][name]["map_points"] = (points @ map_from_plan[:3, :3].T + map_from_plan[:3, 3]).tolist()
        except Exception as error:
            result["plans"][name]["transform_error"] = str(error)
    result["transforms"] = {}
    for frame in ("chassis", "gimbal_yaw", "gimbal_yaw_fake"):
        try:
            result["transforms"][frame] = matrix_to(config["frame"], frame).tolist()
        except Exception as error:
            result["transforms"][frame] = dict(error=str(error))
    encoded = json.dumps(result, indent=2)
    if args.output:
        args.output.write_text(encoded+"\n")
        np.savez_compressed(args.output.with_suffix(".npz"), **grid_dump)
    print(encoded)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
