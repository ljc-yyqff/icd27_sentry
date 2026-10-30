#!/usr/bin/env python3
"""Exercise ordinary navigation AND both tunnel transitions in an isolated sim.

Only sends/cancels its own Nav2 goals. Never resets Gazebo, alters parameters,
clears costmaps, or publishes actuator commands. Start the isolated simulation
and navigation separately. JSONL preserves rejected-command and pose evidence.
"""

import argparse
import json
import math
import os
from pathlib import Path
import statistics
import tempfile
import time

import yaml

from test_tunnel_navigation import footprint_bounds, overlaps


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain-id", type=int, default=91)
    parser.add_argument("--robot", default="red_standard_robot1")
    parser.add_argument("--timeout", type=float, default=90.0)
    parser.add_argument("--goals", type=json.loads, default=[
        [-1.52, 3.54], [-0.28, -0.08], [0.425, -3.475],
        [4.575, -3.475], [0.425, -3.475], [-1.2, 3.475], [-4.575, 3.475],
    ])
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not 0 <= args.domain_id <= 232:
        parser.error("domain-id must be between 0 and 232")
    if not args.goals or any(
        len(goal) not in (2, 3) or not all(math.isfinite(v) for v in goal)
        for goal in args.goals
    ):
        parser.error("goals must be a nonempty JSON array of [x,y] or [x,y,yaw]")
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    args.output = args.output or Path(tempfile.mkdtemp(prefix="rmul2027_route_"))
    args.output.mkdir(parents=True, exist_ok=True)
    config = yaml.safe_load((Path(__file__).resolve().parents[1] /
        "icd2027sentry_nav_bringup/config/simulation/rmul_2027_tunnels.yaml").read_text())

    import rclpy
    from geometry_msgs.msg import Twist
    from lifecycle_msgs.srv import GetState
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64, Float64MultiArray, String

    rclpy.init(args=[])
    node = Node("navigation_route_test", namespace=args.robot,
        parameter_overrides=[Parameter("use_sim_time", value=True)])
    state = dict(guard=None, guard_received=0.0, joints={}, joint_received=0.0,
                 phase="", output=[0.0, 0.0, 0.0], input=[0.0, 0.0, 0.0])
    started = time.monotonic()
    log = (args.output / "events.jsonl").open("w")
    violations, phases, results, cruise, roof_speed = [], [], [], {}, {}
    goal_handle = None
    result_future = None
    blocked_since = None
    max_blocked = 0.0

    def event(kind, **data):
        log.write(json.dumps(dict(t=round(time.monotonic()-started, 4),
            ros_time=node.get_clock().now().nanoseconds/1e9, event=kind, **data)) + "\n")
        log.flush()

    def violation(message):
        if message not in violations:
            violations.append(message)
            event("violation", message=message)

    def guard(message):
        if len(message.data) >= 6 and all(math.isfinite(v) for v in message.data[:6]):
            state["guard"] = list(message.data[:6])
            state["guard_received"] = time.monotonic()

    def joints(message):
        state["joints"] = dict(zip(message.name, message.position))
        state["joint_received"] = time.monotonic()

    def phase(message):
        state["phase"] = message.data
        if not phases or phases[-1] != message.data:
            phases.append(message.data)
            event("phase", phase=message.data)
            print("phase:", message.data, flush=True)

    def command(message, key):
        state[key] = [message.linear.x, message.linear.y, message.angular.z]

    def lift_command(message):
        pose = state["guard"]
        if pose and message.data < config["lift_target"]-config["lift_tolerance"]:
            bounds = footprint_bounds(*pose[:3], config["footprint"])
            if any(overlaps(bounds, tunnel) for tunnel in config["tunnels"]):
                violation("raise command while full footprint overlaps a roof")

    subscriptions = [
        node.create_subscription(Float64MultiArray, "tunnel_guard_state", guard, 10),
        node.create_subscription(JointState, "joint_states", joints, qos_profile_sensor_data),
        node.create_subscription(String, "tunnel_navigation_state", phase,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        node.create_subscription(Twist, "cmd_vel", lambda msg: command(msg, "output"), 10),
        node.create_subscription(Twist, "cmd_vel_unguarded", lambda msg: command(msg, "input"), 10),
        node.create_subscription(Float64, "chassis_lift_cmd", lift_command, 10),
    ]
    action = ActionClient(node, NavigateToPose, "navigate_to_pose")
    lifecycle = node.create_client(GetState, "bt_navigator/get_state")
    last_sample = 0.0

    def sample():
        nonlocal last_sample, blocked_since, max_blocked
        now = time.monotonic()
        if now-last_sample < 0.05:
            return
        last_sample = now
        pose = state["guard"]
        if pose is None or now-state["guard_received"] > config["joint_timeout"]:
            return
        event("sample", **state)
        bounds = footprint_bounds(*pose[:3], config["footprint"])
        for tunnel in config["tunnels"]:
            if overlaps(bounds, tunnel):
                q = state["joints"].get("chassis_lift_joint", float("nan"))
                pitch = state["joints"].get("gimbal_pitch_joint", float("nan"))
                if (now-state["joint_received"] > config["joint_timeout"] or
                    not math.isfinite(q) or abs(q-config["lift_target"]) > config["lift_tolerance"] or
                    not math.isfinite(pitch) or abs(pitch) > config["pitch_tolerance"] or pose[5] < 0.5):
                    violation("invalid lowered feedback under " + tunnel["id"])
                if tunnel["x_min"] <= pose[0] <= tunnel["x_max"]:
                    roof_speed.setdefault(tunnel["id"], []).append(pose[3])
        if state["phase"].startswith("crossing:"):
            cruise.setdefault(state["phase"], []).append(pose[3])
        moving_phase = state["phase"].startswith((
            "normal_follow", "approach_wait_point", "crossing:",
            "following_axis_aligned_guide",
        ))
        blocked = moving_phase and math.hypot(*state["input"][:2]) > 0.15 and math.hypot(*state["output"][:2]) < 0.005
        if blocked:
            blocked_since = blocked_since or now
            max_blocked = max(max_blocked, now-blocked_since)
        else:
            blocked_since = None

    passed, reason = False, "not started"
    try:
        deadline, active, future, next_check = time.monotonic()+60, False, None, 0.0
        request_sent, state_received = 0.0, 0.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if future is not None and future.done():
                response = future.result()
                active = response is not None and response.current_state.id == 3
                state_received = now
                future = None
                next_check = now+0.5
                event("navigator_state", active=active)
            if future is not None and now-request_sent > 2.0:
                # DDS discovery can lose the first service response. A single
                # pending future must not stall the entire startup observation.
                lifecycle.remove_pending_request(future)
                future.cancel()
                future = None
                event("navigator_state_retry")
            if future is None and now >= next_check and lifecycle.service_is_ready():
                future = lifecycle.call_async(GetState.Request())
                request_sent = now
            if (active and now-state_received < 1.5 and action.server_is_ready() and state["guard"] is not None and
                now-state["guard_received"] < .5 and now-state["joint_received"] < .5):
                break
        else:
            raise RuntimeError("navigation or independent guard/joint feedback not ready")
        for index, point in enumerate(args.goals):
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = "map"
            goal.pose.header.stamp = node.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = map(float, point[:2])
            yaw = point[2] if len(point) == 3 else 0.0
            goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            begin = time.monotonic()
            event("goal", index=index, target=point)
            sent = action.send_goal_async(goal)
            rclpy.spin_until_future_complete(node, sent, timeout_sec=10)
            if not sent.done() or sent.result() is None or not sent.result().accepted:
                raise RuntimeError("goal rejected: " + str(point))
            goal_handle = sent.result()
            result_future = goal_handle.get_result_async()
            while time.monotonic()-begin < args.timeout and not result_future.done():
                rclpy.spin_once(node, timeout_sec=0.02)
                sample()
            status = result_future.result().status if result_future.done() else None
            result = dict(goal=point, status=status, elapsed=time.monotonic()-begin,
                          final_guard=state["guard"])
            if state["guard"] is not None:
                result["physical_goal_error"] = math.hypot(
                    state["guard"][0]-point[0], state["guard"][1]-point[1]
                )
                if status == 4 and result["physical_goal_error"] > 0.15:
                    violation("Nav2 succeeded outside the physical 0.15 m goal tolerance")
            results.append(result)
            event("goal_result", **result)
            print("result:", json.dumps(result), flush=True)
            if status != 4:
                raise RuntimeError("goal failed or timed out: " + str(point))
        passed, reason = not violations, "all route goals succeeded" if not violations else "; ".join(violations)
    except (Exception, KeyboardInterrupt) as error:
        reason = str(error) or "interrupted"
    finally:
        if goal_handle is not None and result_future is not None and not result_future.done():
            cancel = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel, timeout_sec=3)
        summary = dict(passed=passed, reason=reason, domain_id=args.domain_id,
            results=results, phases=phases, violations=violations,
            max_output_blocked_wall_seconds=max_blocked,
            crossing_speed={key: dict(max=max(values), median=statistics.median(values),
                samples=len(values)) for key, values in cruise.items()},
            roof_speed={key: dict(max=max(values), median=statistics.median(values),
                samples=len(values)) for key, values in roof_speed.items()},
            elapsed=time.monotonic()-started)
        (args.output / "summary.json").write_text(json.dumps(summary, indent=2)+"\n")
        log.close()
        node.destroy_node()
        rclpy.shutdown()
        print(json.dumps(summary, indent=2), flush=True)
        print("Artifacts:", args.output, flush=True)
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
