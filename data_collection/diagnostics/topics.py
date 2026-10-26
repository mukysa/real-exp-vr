"""Read-only DDS probe for all left Quest collection inputs."""
import argparse
from collections import defaultdict
import json
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=30)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error('duration must be positive')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState, Image
    from std_msgs.msg import String
    from franka_msgs.msg import FrankaRobotState
    rclpy.init(args=[])
    node = rclpy.create_node('quest_m2_readonly_probe')
    topics = {
        '/left/joint_states': JointState,
        '/left/vr/accepted_joint_states': JointState,
        '/left/franka_robot_state_broadcaster/robot_state': FrankaRobotState,
        '/left/vr/source_status': String, '/left/vr/status': String,
        '/cameras/cam_left/image_raw': Image, '/cameras/cam_front/image_raw': Image,
    }
    values = defaultdict(lambda: {'count': 0, 'max_interval_s': 0., 'last_monotonic': None,
                                  'max_header_age_s': None, 'min_header_age_s': None})

    def receive(topic, msg):
        now = time.monotonic()
        item = values[topic]
        item['count'] += 1
        if item['last_monotonic'] is not None:
            item['max_interval_s'] = max(item['max_interval_s'], now - item['last_monotonic'])
        item['last_monotonic'] = now
        header = getattr(msg, 'header', None)
        if topic.endswith('/robot_state'):
            header = msg.o_t_ee.header
        if header is not None:
            stamp = header.stamp.sec + header.stamp.nanosec * 1e-9
            age = node.get_clock().now().nanoseconds * 1e-9 - stamp
            for key, function in [('max_header_age_s', max), ('min_header_age_s', min)]:
                item[key] = age if item[key] is None else function(item[key], age)
        if isinstance(msg, String):
            item['last_status'] = msg.data

    for topic, msg_type in topics.items():
        node.create_subscription(msg_type, topic, lambda msg, topic=topic: receive(topic, msg),
                                 qos_profile_sensor_data)
    start = time.monotonic()
    interrupted = False
    try:
        while time.monotonic() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        interrupted = True
    finally:
        end = time.monotonic()
        result = {}
        for topic in topics:
            item = values[topic]
            last = item.pop('last_monotonic')
            item['last_receive_age_s'] = end - last if last is not None else None
            item['average_receive_hz'] = item['count'] / (end - start)
            result[topic] = item
        node.destroy_node()
        rclpy.try_shutdown()
    report = {'topics': result, 'interrupted': interrupted,
              'all_topics_received': all(item['count'] for item in result.values()),
              'note': 'Best-effort probe receipt does not prove reliable bridge QoS compatibility. Header ages include host clock offset. READY without accepted targets may mean waiting for Y.'}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    return 0 if report['all_topics_received'] and not interrupted else 1


if __name__ == '__main__':
    raise SystemExit(main())
