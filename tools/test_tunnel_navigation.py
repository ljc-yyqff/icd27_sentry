#!/usr/bin/env python3
"""Observe an isolated Nav2/Gazebo tunnel test and verify the complete crossing.

This tool does not launch, reset, or stop Gazebo. Start the simulation and
navigation in the same ROS_DOMAIN_ID first. Only its own NavigateToPose goal
is sent/cancelled. All observations are written to events.jsonl and summary.json.

Example (after sourcing ROS and install/setup.bash):
  python3 tools/test_tunnel_navigation.py --domain-id 91
  python3 tools/test_tunnel_navigation.py --domain-id 91 --goal-x -4.8 --goal-y 3.475
  python3 tools/test_tunnel_navigation.py --domain-id 91 --cancel-in-tunnel
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import tempfile
import time

import yaml


DEFAULT_CONFIG = (
    Path(__file__).resolve().parents[1]
    / "icd2027sentry_nav_bringup/config/simulation/rmul_2027_tunnels.yaml"
)


def footprint_bounds(x, y, yaw, footprint):
    c, s = math.cos(yaw), math.sin(yaw)
    points = [(x + c * px - s * py, y + s * px + c * py) for px, py in footprint]
    return (
        min(p[0] for p in points),
        max(p[0] for p in points),
        min(p[1] for p in points),
        max(p[1] for p in points),
    )


def overlaps(bounds, tunnel, margin=0.01):
    lo_x, hi_x, lo_y, hi_y = bounds
    return (
        hi_x + margin >= tunnel["x_min"]
        and lo_x - margin <= tunnel["x_max"]
        and hi_y + margin >= tunnel["y_min"]
        and lo_y - margin <= tunnel["y_max"]
    )


def quaternion_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def initial_joint_feedback_ready(lift, pitch, received, now, timeout):
    """Wait for this observer's first sample, not just another node's readiness.

    This is only a subscription bootstrap check, not a lowered-pose check.
    Once observation begins, missing/stale or incorrect feedback under the roof
    must still be recorded as a violation.
    """
    return (
        lift is not None
        and pitch is not None
        and math.isfinite(lift)
        and math.isfinite(pitch)
        and 0.0 <= now - received <= timeout
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain-id", type=int, default=91)
    parser.add_argument("--robot", default="red_standard_robot1")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--tunnel", default="red_tunnel")
    parser.add_argument("--goal-x", type=float, default=-1.2)
    parser.add_argument("--goal-y", type=float, default=3.475)
    parser.add_argument("--goal-yaw", type=float, default=0.0)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument(
        "--server-timeout",
        type=float,
        default=60.0,
        help="Seconds to wait for active bt_navigator, its action, and fresh guard feedback",
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--cancel-in-tunnel", action="store_true")
    mode.add_argument(
        "--resume-inside",
        action="store_true",
        help="Expect an initially lowered robot under the roof, e.g. after cancellation",
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not 0 <= args.domain_id <= 232:
        parser.error("ROS domain id must be between 0 and 232")
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    config = yaml.safe_load(args.config.read_text())
    tunnel = next(
        (item for item in config["tunnels"] if item["id"] == args.tunnel), None
    )
    if tunnel is None:
        parser.error("Unknown tunnel: " + args.tunnel)
    args.output = args.output or Path(tempfile.mkdtemp(prefix="rmul2027_nav_test_"))
    args.output.mkdir(parents=True, exist_ok=True)

    # Delay ROS imports so --help also works outside the ROS environment.
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped
    from lifecycle_msgs.msg import State
    from lifecycle_msgs.srv import GetState
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import Odometry
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, Float64, Float64MultiArray, String
    from tf2_ros import Buffer, TransformListener, TransformException

    class Observer(Node):
        def __init__(self):
            super().__init__(
                "tunnel_navigation_test",
                namespace=args.robot,
                parameter_overrides=[Parameter("use_sim_time", value=True)],
                cli_args=["--ros-args", "-r", "/tf:=tf", "-r", "/tf_static:=tf_static"],
            )
            self.started = time.monotonic()
            self.log_file = (args.output / "events.jsonl").open("w")
            self.phases = []
            self.lift = None
            self.pitch = None
            self.joint_received = -1e9
            self.guard = None
            self.guard_received = -1e9
            self.local_ready = False
            self.entered = False
            self.exited = False
            self.direction = None
            self.violations = []
            self.positions = []
            self.tf_samples = 0
            self.goal_handle = None
            self.result_future = None
            self.result_status = None
            self.cancel_requested = False
            self.stable_since = None
            self.last_sample = -1e9
            self.action = ActionClient(self, NavigateToPose, "navigate_to_pose")
            self.navigator_state_client = self.create_client(
                GetState, "bt_navigator/get_state"
            )
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.create_subscription(
                JointState, "joint_states", self.joints, qos_profile_sensor_data
            )
            self.create_subscription(Float64, "chassis_lift_cmd", self.lift_command, 10)
            self.create_subscription(
                Float64MultiArray,
                "tunnel_policy",
                lambda m: self.event("policy", data=list(m.data)),
                10,
            )
            self.create_subscription(
                Float64MultiArray, "tunnel_guard_state", self.guard_state, 10
            )
            state_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(
                String, "tunnel_navigation_state", self.phase, state_qos
            )
            self.create_subscription(
                Bool, "tunnel_local_clearance_ready", self.clearance, 10
            )
            self.create_subscription(
                Odometry,
                "chassis_odometry_gt",
                self.ground_truth,
                qos_profile_sensor_data,
            )

        def event(self, kind, **fields):
            self.log_file.write(
                json.dumps(
                    dict(
                        t=round(time.monotonic() - self.started, 4),
                        ros_time=self.get_clock().now().nanoseconds / 1e9,
                        event=kind,
                        **fields,
                    )
                )
                + "\n"
            )
            self.log_file.flush()

        def violation(self, text):
            if text not in self.violations:
                self.violations.append(text)
                self.event("violation", message=text)

        def joints(self, msg):
            positions = dict(zip(msg.name, msg.position))
            if "chassis_lift_joint" in positions:
                self.lift = positions["chassis_lift_joint"]
                self.joint_received = time.monotonic()
            if "gimbal_pitch_joint" in positions:
                self.pitch = positions["gimbal_pitch_joint"]

        def lift_command(self, msg):
            self.event("lift_command", position=msg.data)
            if (
                self.guard is not None
                and msg.data < config["lift_target"] - config["lift_tolerance"]
            ):
                if overlaps(
                    footprint_bounds(*self.guard[:3], config["footprint"]), tunnel
                ):
                    self.violation(
                        "raise command issued while footprint overlaps ceiling"
                    )

        def phase(self, msg):
            if not self.phases or self.phases[-1] != msg.data:
                self.phases.append(msg.data)
                self.event("phase", phase=msg.data)
                print("phase:", msg.data, flush=True)

        def clearance(self, msg):
            if msg.data != self.local_ready:
                self.local_ready = msg.data
                self.event("local_clearance_ready", ready=msg.data)

        def ground_truth(self, msg):
            # Retain an independent physical odometry trace in addition to
            # map TF/guard state. The message's own frame is preserved.
            p = msg.pose.pose.position
            self.event(
                "ground_truth",
                frame=msg.header.frame_id,
                x=p.x,
                y=p.y,
                yaw=quaternion_yaw(msg.pose.pose.orientation),
            )

        def guard_state(self, msg):
            if len(msg.data) < 6:
                self.violation("malformed tunnel_guard_state")
                return
            if not all(math.isfinite(value) for value in msg.data[:6]):
                if self.entered:
                    self.violation(
                        "guard pose/velocity feedback became invalid under roof"
                    )
                return
            self.guard = list(msg.data)
            self.guard_received = time.monotonic()

        def sample(self):
            now = time.monotonic()
            if now - self.last_sample < 0.1:
                return
            self.last_sample = now
            try:
                tf = self.tf_buffer.lookup_transform(
                    config["frame"], "chassis", rclpy.time.Time()
                )
                self.tf_samples += 1
                self.event(
                    "tf",
                    frame=config["frame"],
                    x=tf.transform.translation.x,
                    y=tf.transform.translation.y,
                    yaw=quaternion_yaw(tf.transform.rotation),
                )
            except TransformException:
                pass
            if (
                self.guard is None
                or now - self.guard_received > config["joint_timeout"]
            ):
                return
            if self.direction is None and not initial_joint_feedback_ready(
                self.lift, self.pitch, self.joint_received,
                now, config["joint_timeout"],
            ):
                # DDS subscriptions can discover guard state before joints.
                # Do not confuse observer startup with loss of runtime feedback.
                return
            x, y, yaw, speed, yaw_rate, ready = self.guard[:6]
            self.positions.append([x, y, yaw])
            self.event("sample", guard=self.guard, lift=self.lift, pitch=self.pitch)
            bounds = footprint_bounds(x, y, yaw, config["footprint"])
            inside = overlaps(bounds, tunnel)
            if self.direction is None:
                self.direction = 1 if args.goal_x > x else -1
                if inside and not args.resume_inside:
                    self.violation("test started with robot already under tunnel")
                elif args.resume_inside and not inside:
                    self.violation("resume test did not start under the tunnel")
            if inside:
                self.entered = True
                lowered = (
                    self.lift is not None
                    and now - self.joint_received <= config["joint_timeout"]
                    and abs(self.lift - config["lift_target"])
                    <= config["lift_tolerance"]
                    and self.pitch is not None
                    and abs(self.pitch) <= config["pitch_tolerance"]
                )
                if not lowered or not ready:
                    self.violation(
                        "robot entered ceiling area without valid lowered/level feedback"
                    )
                if abs(speed) > config["speed"] + 0.04:
                    self.violation(
                        "tunnel speed exceeded configured limit by more than 0.04 m/s"
                    )
            far_side = (
                (bounds[0] - 0.01 > tunnel["x_max"])
                if self.direction == 1
                else (bounds[1] + 0.01 < tunnel["x_min"])
            )
            if self.entered and far_side and not inside:
                self.exited = True
            if (
                args.cancel_in_tunnel
                and self.goal_handle is not None
                and self.entered
                and not self.cancel_requested
            ):
                # Centre inside the short roof ensures cancellation happens
                # during traversal rather than merely at the approach edge.
                if tunnel["x_min"] <= x <= tunnel["x_max"]:
                    self.cancel_requested = True
                    self.event("cancel_requested", x=x, y=y)
                    self.goal_handle.cancel_goal_async()
            stable = (
                self.cancel_requested
                and inside
                and self.lift is not None
                and abs(self.lift - config["lift_target"]) <= config["lift_tolerance"]
                and abs(speed) < 0.025
                and abs(yaw_rate) < 0.05
            )
            if not args.cancel_in_tunnel:
                stable = (
                    self.exited
                    and self.lift is not None
                    and abs(self.lift) <= config["lift_tolerance"]
                )
            self.stable_since = (self.stable_since or now) if stable else None

    rclpy.init(args=[])
    observer = Observer()
    reason = "timeout"
    passed = False
    try:
        deadline = time.monotonic() + args.server_timeout
        state_future = None
        navigator_state = None
        state_received = -1e9
        next_state_check = -1e9
        state_request_sent = -1e9
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(observer, timeout_sec=0.1)
            observer.sample()
            now = time.monotonic()
            if state_future is not None and state_future.done():
                response = state_future.result()
                state_future = None
                next_state_check = now + 0.5
                if response is not None:
                    state_received = now
                    if navigator_state != response.current_state.id:
                        navigator_state = response.current_state.id
                        observer.event(
                            "navigator_lifecycle_state",
                            id=navigator_state,
                            label=response.current_state.label,
                        )
            if state_future is not None and now - state_request_sent > 2.0:
                observer.navigator_state_client.remove_pending_request(state_future)
                state_future.cancel()
                state_future = None
                observer.event("navigator_state_retry")
            if (
                state_future is None
                and now >= next_state_check
                and observer.navigator_state_client.service_is_ready()
            ):
                state_future = observer.navigator_state_client.call_async(
                    GetState.Request()
                )
                state_request_sent = now
            # The action endpoint exists during configuration, before the
            # lifecycle action server starts accepting goals. Wait for a fresh
            # explicit ACTIVE response instead of racing Nav2 activation.
            if (
                observer.action.server_is_ready()
                and navigator_state == State.PRIMARY_STATE_ACTIVE
                and now - state_received <= 1.5
                and observer.guard is not None
                and now - observer.guard_received <= config["joint_timeout"]
                and initial_joint_feedback_ready(
                    observer.lift, observer.pitch, observer.joint_received,
                    now, config["joint_timeout"],
                )
            ):
                break
        else:
            raise RuntimeError(
                "Timed out waiting for active bt_navigator, Nav2 action server, "
                "or fresh tunnel guard/joint state "
                f"(navigator state: {navigator_state})"
            )
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = config["frame"]
        goal.pose.header.stamp = observer.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = args.goal_x, args.goal_y
        goal.pose.pose.orientation.z = math.sin(args.goal_yaw / 2)
        goal.pose.pose.orientation.w = math.cos(args.goal_yaw / 2)
        observer.event("goal", x=args.goal_x, y=args.goal_y, yaw=args.goal_yaw)
        sent = observer.action.send_goal_async(
            goal,
            feedback_callback=lambda msg: observer.event(
                "action_feedback",
                distance_remaining=msg.feedback.distance_remaining,
                recoveries=msg.feedback.number_of_recoveries,
            ),
        )
        rclpy.spin_until_future_complete(observer, sent, timeout_sec=10.0)
        if not sent.done() or sent.result() is None or not sent.result().accepted:
            raise RuntimeError("Nav2 rejected goal or did not acknowledge within 10 s")
        observer.goal_handle = sent.result()
        observer.result_future = observer.goal_handle.get_result_async()
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(observer, timeout_sec=0.05)
            observer.sample()
            if observer.result_future.done():
                observer.result_status = observer.result_future.result().status
                expected = (
                    GoalStatus.STATUS_CANCELED
                    if args.cancel_in_tunnel
                    else GoalStatus.STATUS_SUCCEEDED
                )
                if observer.result_status != expected:
                    reason = f"Nav2 action ended with status {observer.result_status}, expected {expected}"
                    break
                stable_duration = (
                    2.0 if args.cancel_in_tunnel else config["settle_time"]
                )
                if (
                    observer.stable_since
                    and time.monotonic() - observer.stable_since >= stable_duration
                ):
                    phase_text = " ".join(observer.phases).lower()
                    required = (
                        ["lowering", "crossing"]
                        if args.cancel_in_tunnel
                        else ["lowering", "crossing", "raising"]
                    )
                    missing = [phase for phase in required if phase not in phase_text]
                    if missing:
                        reason = "Missing BT phases: " + ", ".join(missing)
                    elif observer.violations:
                        reason = "; ".join(observer.violations)
                    elif observer.tf_samples == 0:
                        reason = "No independent map -> chassis TF samples received"
                    else:
                        passed, reason = (
                            True,
                            "cancelled safely under roof"
                            if args.cancel_in_tunnel
                            else "Nav2 succeeded; full footprint exited and chassis restored",
                        )
                    break
    except (Exception, KeyboardInterrupt) as error:
        reason = str(error) or "interrupted"
    finally:
        if (
            reason == "timeout"
            and observer.result_status == GoalStatus.STATUS_SUCCEEDED
        ):
            reason = "Nav2 succeeded but complete crossing / ride-height restoration was not verified before timeout"
        # Cancel only the goal created by this tool. The BT/guard must retain
        # low posture if it is still beneath a roof; never issue a lift command.
        if (
            observer.goal_handle
            and observer.result_future
            and not observer.result_future.done()
        ):
            future = observer.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(observer, future, timeout_sec=3.0)
        summary = dict(
            passed=passed,
            reason=reason,
            domain_id=args.domain_id,
            robot=args.robot,
            tunnel=args.tunnel,
            goal=[args.goal_x, args.goal_y, args.goal_yaw],
            cancel_test=args.cancel_in_tunnel,
            resume_test=args.resume_inside,
            nav2_status=observer.result_status,
            entered=observer.entered,
            complete_footprint_exited=observer.exited,
            final_lift=observer.lift,
            final_guard=observer.guard,
            tf_samples=observer.tf_samples,
            phases=observer.phases,
            violations=observer.violations,
            elapsed=time.monotonic() - observer.started,
        )
        (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
        observer.log_file.close()
        observer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(json.dumps(summary, indent=2), flush=True)
        print("Artifacts:", args.output, flush=True)
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
