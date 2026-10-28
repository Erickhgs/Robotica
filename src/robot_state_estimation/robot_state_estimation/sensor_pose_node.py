"""Pose planar GPS + orientacao IMU. Nao e um EKF nem SLAM."""
from copy import deepcopy
from collections import deque
import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PointStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster

from .geodesy import LocalENU, add_world_yaw, fresh_pair, yaw_from_quaternion


def seconds(stamp):
    return stamp.sec + stamp.nanosec*1e-9


class SensorPose(Node):
    def __init__(self):
        super().__init__('sensor_pose')
        defaults = {
            'origin_latitude': -8.05, 'origin_longitude': -34.9, 'origin_altitude': 0.0,
            'noise_scale': 1.0, 'yaw_noise_stddev': 0.01, 'random_seed': 42,
            'max_sensor_age': 0.3, 'max_sensor_skew': 0.10,
            'base_height': 0.1,
            'publish_tf': True,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        p = lambda name: self.get_parameter(name).value
        self.enu = LocalENU(p('origin_latitude'), p('origin_longitude'), p('origin_altitude'))
        scale, yaw_std = float(p('noise_scale')), float(p('yaw_noise_stddev'))
        self.age, self.skew = float(p('max_sensor_age')), float(p('max_sensor_skew'))
        self.height = float(p('base_height'))
        if not all(math.isfinite(v) and v >= 0 for v in (scale, yaw_std, self.skew, self.height)):
            raise ValueError('Escalas, desvios e alturas devem ser finitos e nao negativos.')
        if not math.isfinite(self.age) or self.age <= 0:
            raise ValueError('max_sensor_age deve ser positivo.')
        self.yaw_std, self.gps_variance = yaw_std*scale, (0.02*scale)**2
        self.rng = random.Random(int(p('random_seed')))
        self.imu_history = deque(maxlen=200)
        self.pending_fix = None
        self.imu_pub = self.create_publisher(Imu, '/imu/data', qos_profile_sensor_data)
        self.yaw_pub = self.create_publisher(Float64, '/imu/yaw', 10)
        self.enu_pub = self.create_publisher(PointStamped, '/gps/enu', 10)
        self.odom_pub = self.create_publisher(Odometry, '/localization/odom', 10)
        self.tf = TransformBroadcaster(self) if p('publish_tf') else None
        self.create_subscription(Imu, '/imu/raw', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, '/gps/fix', self.on_gps, qos_profile_sensor_data)
        # DDS pode entregar o sensor antes do /clock correspondente. Processar
        # em timer permite esperar esse clock sem aceitar timestamps futuros.
        self.create_timer(0.02, self.publish_pose)
        self.get_logger().info('Pose em map: X/Y do GPS ENU, yaw da IMU. Sem /odom ou ground truth.')

    def on_imu(self, msg):
        # No Classic, apenas giroscopio/acelerometro recebem ruido nativo.
        # Modelamos tambem o erro da orientacao, antes de publica-la/usa-la.
        if msg.header.frame_id != 'imu_link' or msg.orientation_covariance[0] < 0:
            self.imu_history.clear()
            return
        noisy = deepcopy(msg)
        q = msg.orientation
        try:
            xyz = add_world_yaw(q.x, q.y, q.z, q.w, self.rng.gauss(0, self.yaw_std))
        except ValueError:
            self.imu_history.clear()
            return
        if not math.isfinite(msg.angular_velocity.z):
            self.imu_history.clear()
            return
        noisy.orientation.x, noisy.orientation.y, noisy.orientation.z, noisy.orientation.w = xyz
        noisy.orientation_covariance = [0.0]*9
        noisy.orientation_covariance[8] = self.yaw_std**2
        self.imu_history.append(noisy)
        self.imu_pub.publish(noisy)
        self.yaw_pub.publish(Float64(data=yaw_from_quaternion(*xyz)))

    def on_gps(self, msg):
        if msg.status.status < 0 or msg.header.frame_id != 'gps_link':
            return
        try:
            x, y, z = self.enu.convert(msg.latitude, msg.longitude, msg.altitude)
        except ValueError:
            return
        point = PointStamped()
        point.header.stamp, point.header.frame_id = msg.header.stamp, 'map'
        point.point.x, point.point.y, point.point.z = x, y, z
        self.enu_pub.publish(point)
        self.pending_fix = point

    def publish_pose(self):
        if self.pending_fix is None:
            return
        now = self.get_clock().now().nanoseconds*1e-9
        point = self.pending_fix
        gps_time = seconds(point.header.stamp)
        candidates = [m for m in self.imu_history
                      if fresh_pair(now, gps_time, seconds(m.header.stamp), self.age, self.skew)]
        if not candidates:
            self.get_logger().warning('GPS/IMU ausentes, antigos ou dessincronizados; sem pose.',
                                      throttle_duration_sec=3.0)
            if now-gps_time > self.age:
                self.pending_fix = None
            return
        imu = min(candidates, key=lambda m: abs(seconds(m.header.stamp)-gps_time))
        self.pending_fix = None
        x, y = point.point.x, point.point.y
        q = imu.orientation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        odom = Odometry()
        odom.header = point.header
        odom.child_frame_id = 'chassis'
        odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
        # Robo em solo plano: GPS Z e mostrado em /gps/enu, nao usado no controle 2D.
        odom.pose.pose.position.z = self.height
        odom.pose.pose.orientation.z = math.sin(yaw/2)
        odom.pose.pose.orientation.w = math.cos(yaw/2)
        odom.pose.covariance[0] = odom.pose.covariance[7] = self.gps_variance
        odom.pose.covariance[14] = odom.pose.covariance[21] = odom.pose.covariance[28] = 1e6
        odom.pose.covariance[35] = self.yaw_std**2
        for index in (0, 7, 14, 21, 28, 35):
            odom.twist.covariance[index] = 1e6  # velocidades nao estimadas, exceto wz
        odom.twist.twist.angular.z = imu.angular_velocity.z
        odom.twist.covariance[35] = imu.angular_velocity_covariance[8]
        self.odom_pub.publish(odom)
        transform = TransformStamped()
        transform.header, transform.child_frame_id = odom.header, 'chassis'
        transform.transform.translation.x, transform.transform.translation.y = x, y
        transform.transform.translation.z = self.height
        transform.transform.rotation = odom.pose.pose.orientation
        if self.tf is not None:
            self.tf.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = SensorPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
