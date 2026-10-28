from collections import deque
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Imu
import yaml

from robot_state_estimation.sensor_pose_node import SensorPose


class EkfTests(unittest.TestCase):
    def test_fuses_each_measurement_once_without_ground_truth(self):
        path = Path(__file__).resolve().parents[1]/'config/ekf.yaml'
        p = yaml.safe_load(path.read_text())['ekf_filter_node']['ros__parameters']
        self.assertEqual(p['odom0'], '/localization/odom')
        self.assertEqual(p['imu0'], '/imu/data')
        self.assertEqual([i for i, v in enumerate(p['odom0_config']) if v], [0, 1])
        self.assertEqual([i for i, v in enumerate(p['imu0_config']) if v], [5, 11])
        self.assertEqual(len(p['odom0_config']), 15)
        self.assertEqual(len(p['imu0_config']), 15)
        self.assertTrue(p['two_d_mode'])
        self.assertEqual(p['world_frame'], p['odom_frame'])
        self.assertEqual(len({p[k] for k in ('map_frame', 'odom_frame', 'base_link_frame')}), 3)

    def test_raw_pose_and_covariance_survive_with_tf_disabled(self):
        point, imu = PointStamped(), Imu()
        point.header.frame_id, point.header.stamp.sec = 'map', 1
        point.point.x, point.point.y = 2.0, 3.0
        imu.header.stamp.sec = 1
        imu.orientation.w = 1.0
        imu.angular_velocity.z = .1
        imu.angular_velocity_covariance[8] = .000256
        for broadcaster in (None, Mock()):
            node = SimpleNamespace(
                pending_fix=point, imu_history=deque([imu]), age=.3, skew=.1,
                height=.1, yaw_std=.08, gps_variance=.16**2, tf=broadcaster,
                odom_pub=Mock(),
                get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10**9)),
            )
            SensorPose.publish_pose(node)
            msg = node.odom_pub.publish.call_args.args[0]
            self.assertEqual((msg.pose.pose.position.x, msg.pose.pose.position.y), (2, 3))
            self.assertEqual(msg.header.frame_id, 'map')
            self.assertAlmostEqual(msg.pose.covariance[0], .0256)
            self.assertAlmostEqual(msg.pose.covariance[35], .0064)
            self.assertEqual(msg.twist.covariance[35], .000256)
            self.assertIsNone(node.pending_fix)
            if broadcaster is not None:
                broadcaster.sendTransform.assert_called_once()


if __name__ == '__main__':
    unittest.main()
