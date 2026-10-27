"""Publish isolated M2 ROS fixtures. Never publishes robot command topics."""
import argparse
import math
import os
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=30)
    parser.add_argument('--drop', choices=['none', 'action', 'state', 'camera'], default='none')
    parser.add_argument('--drop-after', type=float, default=8)
    parser.add_argument('--stamp-offset', type=float, default=0,
                        help='Seconds added to ROS headers, to test bad clocks.')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '173':
        parser.error('isolated fixture requires ROS_DOMAIN_ID=173')
    if args.duration <= 0 or args.drop_after < 0:
        parser.error('invalid duration or drop-after')
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from sensor_msgs.msg import JointState, Image
    from std_msgs.msg import String
    from franka_msgs.msg import FrankaRobotState
    from builtin_interfaces.msg import Time
    rclpy.init(args=[])
    node = rclpy.create_node('m2_synthetic_inputs')
    types = {'joint': JointState, 'action': JointState, 'ee': FrankaRobotState,
             'cam_left': Image, 'source': String, 'status': String}
    topics = {'joint': '/m2/left/joint_states', 'action': '/m2/left/vr/accepted_joint_states',
              'ee': '/m2/left/franka_robot_state_broadcaster/robot_state',
              'cam_left': '/m2/cameras/cam_left/image_raw',
              'source': '/m2/left/vr/source_status', 'status': '/m2/left/vr/status'}
    publishers = {key: node.create_publisher(types[key], topic, 10) for key, topic in topics.items()}
    start = time.monotonic()

    def tick():
        elapsed = time.monotonic() - start
        ns = node.get_clock().now().nanoseconds + int(args.stamp_offset * 1e9)
        stamp = Time(sec=ns // 10**9, nanosec=ns % 10**9)
        dropped = args.drop if elapsed >= args.drop_after else 'none'
        q = [0., -0.5, 0., -2., 0., 1.5, 0.5]
        for key, category in [('joint', 'state'), ('action', 'action')]:
            if dropped == category:
                continue
            msg = JointState()
            msg.header.stamp = stamp
            msg.name = [f'left_fr3_joint{i}' for i in range(1, 8)]
            msg.position = q.copy()
            if key == 'action':
                msg.position[0] += 0.01 * math.sin(elapsed)
            publishers[key].publish(msg)
        if dropped != 'state':
            msg = FrankaRobotState()
            msg.o_t_ee.header.stamp = stamp
            msg.o_t_ee.pose.position.z = 0.5
            msg.o_t_ee.pose.orientation.w = 1.
            msg.f_t_ee.header.stamp = stamp
            msg.f_t_ee.pose.orientation.w = 1.
            publishers['ee'].publish(msg)
        for name in ('cam_left',):
            if dropped == 'camera':
                continue
            msg = Image()
            msg.header.stamp = stamp
            msg.height, msg.width, msg.step = 24, 32, 96
            msg.encoding = 'rgb8'
            msg.data = bytes([int(elapsed * 10) % 255]) * (24 * 32 * 3)
            publishers[name].publish(msg)
        publishers['source'].publish(String(data='{"state":"FOLLOWING","synthetic":true}'))
        publishers['status'].publish(String(data='{"synthetic":true}'))

    node.create_timer(1 / 30, tick)
    print('Synthetic inputs: domain 173, /m2 namespace; no robot command publishers.', flush=True)
    try:
        while time.monotonic() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
