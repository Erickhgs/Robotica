import csv
import io
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from nav_msgs.msg import Odometry

from robot_control.ground_truth_node import GroundTruthRecorder


class SensorRecordingTests(unittest.TestCase):
    def test_raw_pose_validates_frame_and_orientation_before_recording(self):
        stream = io.StringIO()
        node = SimpleNamespace(raw_writer=csv.writer(stream), raw_count=19, raw_stream=Mock())
        msg = Odometry()
        GroundTruthRecorder.on_raw(node, msg)
        msg.header.frame_id, msg.child_frame_id = 'map', 'chassis'
        msg.pose.pose.orientation.w = 0.0
        GroundTruthRecorder.on_raw(node, msg)  # quaternion nulo
        self.assertEqual(stream.getvalue(), '')
        msg.header.stamp.sec, msg.header.stamp.nanosec = 2, 500000000
        msg.pose.pose.position.x, msg.pose.pose.position.y = 1.0, -2.0
        msg.pose.pose.orientation.w = 1.0
        GroundTruthRecorder.on_raw(node, msg)
        row = next(csv.reader(io.StringIO(stream.getvalue())))
        self.assertEqual([float(v) for v in row], [2.5, 1.0, -2.0, 0.0])
        node.raw_stream.flush.assert_called_once()


if __name__ == '__main__':
    unittest.main()
