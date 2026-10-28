"""Registra referencia do Gazebo e, opcionalmente, pose bruta para avaliacao."""
import csv
from datetime import datetime
from pathlib import Path as FilePath

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from .ros_common import parameter, run_node, to_pose


class GroundTruthRecorder(Node):
    def __init__(self):
        super().__init__('ground_truth_recorder')
        output = FilePath(parameter(self, 'output_dir', '~/pratica04_resultados')).expanduser()
        output.mkdir(parents=True, exist_ok=True)
        filename = output / ('ground_truth_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.csv')
        self.stream = filename.open('x', newline='', encoding='utf-8')
        self.writer = csv.writer(self.stream)
        self.writer.writerow(['ros_time', 'x', 'y', 'yaw'])
        self.raw_stream = None
        self.raw_count = 0
        if parameter(self, 'record_raw_pose', False):
            self.raw_stream = (output/'raw_pose.csv').open('x', newline='', encoding='utf-8')
            self.raw_writer = csv.writer(self.raw_stream)
            self.raw_writer.writerow(['ros_time', 'x', 'y', 'yaw'])
            self.create_subscription(Odometry, '/localization/odom', self.on_raw,
                                     qos_profile_sensor_data)
        self.count, self.last_path = 0, -1.0
        self.path = Path()
        self.pose_pub = self.create_publisher(PoseStamped, '/evaluation/ground_truth', 10)
        self.path_pub = self.create_publisher(Path, '/evaluation/ground_truth_path', 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_truth, qos_profile_sensor_data)
        self.get_logger().info(f'Referencia independente: {filename}')

    def on_raw(self, msg):
        if msg.header.frame_id != 'map' or msg.child_frame_id != 'chassis':
            return
        try:
            pose = to_pose(msg.pose.pose)
        except ValueError:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        self.raw_writer.writerow([t, pose.x, pose.y, pose.yaw])
        self.raw_count += 1
        if self.raw_count % 20 == 0:
            self.raw_stream.flush()

    def on_truth(self, msg):
        if msg.header.frame_id != 'world' or msg.child_frame_id != 'chassis':
            self.get_logger().error('Ground truth deve estar em world/chassis.', throttle_duration_sec=3.0)
            return
        actual = to_pose(msg.pose.pose)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        self.writer.writerow([t, actual.x, actual.y, actual.yaw])
        self.count += 1
        if self.count % 50 == 0:
            self.stream.flush()
        pose = PoseStamped()
        pose.header.stamp, pose.header.frame_id = msg.header.stamp, 'map'
        pose.pose = msg.pose.pose
        # world e map coincidem no mundo fornecido, com origem geodesica fixa.
        self.pose_pub.publish(pose)
        if t < self.last_path:
            self.path.poses.clear()
            self.last_path = -1.0
        if t-self.last_path >= 0.1:
            self.path.header = pose.header
            self.path.poses.append(pose)
            self.path.poses = self.path.poses[-5000:]
            self.path_pub.publish(self.path)
            self.last_path = t

    def stop(self):
        self.stream.close()
        if self.raw_stream is not None:
            self.raw_stream.close()


def main(args=None):
    run_node(GroundTruthRecorder, args)
