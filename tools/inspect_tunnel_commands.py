#!/usr/bin/env python3
"""Read-only simultaneous command/actual-pose capture for tunnel diagnostics."""
import argparse
import json
import os
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=30.)
    parser.add_argument('--domain-id', type=int, default=91)
    parser.add_argument('--robot', default='red_standard_robot1')
    parser.add_argument('--output', type=Path, default=Path('/tmp/icd_tunnel_commands.json'))
    args = parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Float64MultiArray

    rclpy.init()
    node = Node('inspect_tunnel_commands', namespace=args.robot,
                parameter_overrides=[Parameter('use_sim_time', value=True)])
    samples = {}
    start = time.monotonic()

    def record(topic, msg):
        item = dict(wall=time.monotonic()-start,
                    ros=node.get_clock().now().nanoseconds/1e9)
        if isinstance(msg, Twist):
            item.update(vx=msg.linear.x, vy=msg.linear.y, wz=msg.angular.z)
        else:
            item['data'] = list(msg.data)
        samples.setdefault(topic, []).append(item)

    subscriptions = []
    for topic in ('cmd_vel_nav2_result', 'cmd_vel_unguarded', 'cmd_vel'):
        subscriptions.append(node.create_subscription(
            Twist, topic, lambda msg, key=topic: record(key, msg), qos_profile_sensor_data))
    for topic in ('tunnel_guard_state', 'tunnel_policy'):
        subscriptions.append(node.create_subscription(
            Float64MultiArray, topic, lambda msg, key=topic: record(key, msg), qos_profile_sensor_data))
    print('READ_ONLY_COMMAND_SUBSCRIPTIONS_READY', flush=True)
    while time.monotonic()-start < args.duration:
        rclpy.spin_once(node, timeout_sec=.05)
    args.output.write_text(json.dumps(samples, indent=2))
    for topic, rows in samples.items():
        if 'vy' in rows[0]:
            nonzero = [row for row in rows if abs(row['vy']) > .001]
            print(topic, 'messages', len(rows), 'nonzero_vy', len(nonzero),
                  'vy_range', [min(row['vy'] for row in rows), max(row['vy'] for row in rows)],
                  'first_nonzero', nonzero[0] if nonzero else None,
                  'last_nonzero', nonzero[-1] if nonzero else None)
        elif topic == 'tunnel_guard_state':
            print(topic, 'messages', len(rows), 'first', rows[0], 'last', rows[-1],
                  'x_range', [min(row['data'][0] for row in rows), max(row['data'][0] for row in rows)],
                  'y_range', [min(row['data'][1] for row in rows), max(row['data'][1] for row in rows)])
    print('Saved', args.output, flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
