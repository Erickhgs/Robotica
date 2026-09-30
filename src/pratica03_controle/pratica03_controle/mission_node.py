"""Exercício 1.2: publica waypoints sequencialmente e confirma a pose por odometria."""
import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import Bool, Int32

from .control import Pose, wrap
from .ros_common import number, parameter, pose_message, run_node, to_pose


class MissionExecutor(Node):
    def __init__(self):
        super().__init__('mission_executor')
        self.frame = parameter(self, 'frame_id', 'odom')
        self.position_tolerance = number(self, 'position_tolerance', 0.04, True)
        self.yaw_tolerance = number(self, 'yaw_tolerance', 0.06, True)
        self.dwell = number(self, 'dwell_time', 0.5, True)
        self.timeout = number(self, 'odom_timeout', 0.5, True)
        xs = parameter(self, 'waypoints.x', [1.0, 1.0, 0.0, 0.0])
        ys = parameter(self, 'waypoints.y', [0.0, 1.0, 1.0, 0.0])
        yaws = parameter(self, 'waypoints.yaw', [0.0, 1.57079632679, 3.14159265359, 0.0])
        if len(xs) < 3 or len(xs) != len(ys) or len(xs) != len(yaws):
            raise ValueError('Use no mínimo 3 waypoints; x, y e yaw devem ter mesmo tamanho.')
        if not all(math.isfinite(v) for values in (xs, ys, yaws) for v in values):
            raise ValueError('Waypoints precisam ser finitos.')
        self.waypoints = [Pose(float(x), float(y), wrap(float(a))) for x, y, a in zip(xs, ys, yaws)]
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', qos)
        self.index_pub = self.create_publisher(Int32, 'mission/index', qos)
        self.finished_pub = self.create_publisher(Bool, 'mission/finished', qos)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.on_odom, qos_profile_sensor_data)
        self.actual = None
        self.received = None
        self.index = 0
        self.inside_since = None
        self.last_publish = -math.inf
        self.last_tick = None
        self.sent_current = False
        self.done = False
        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info(f'Missão carregada: {len(self.waypoints)} waypoints.')

    def on_odom(self, msg):
        if msg.header.frame_id != self.frame:
            self.actual = None
            return
        try:
            self.actual = to_pose(msg.pose.pose)
            self.received = self.get_clock().now().nanoseconds*1e-9
        except ValueError:
            self.actual = None

    def tick(self):
        now = self.get_clock().now()
        now_s = now.nanoseconds*1e-9
        if self.done:
            self.finished_pub.publish(Bool(data=True))
            return
        if self.last_tick is not None and now_s < self.last_tick:
            self.inside_since = None
            self.last_publish = -math.inf
        self.last_tick = now_s
        if (self.actual is None or self.received is None or
                now_s-self.received > self.timeout or now_s < self.received or
                self.goal_pub.get_subscription_count() == 0):
            self.inside_since = None
            return
        target = self.waypoints[self.index]
        if now_s-self.last_publish >= 0.5:
            self.goal_pub.publish(pose_message(target, now.to_msg(), self.frame))
            self.index_pub.publish(Int32(data=self.index))
            self.finished_pub.publish(Bool(data=False))
            self.last_publish = now_s
            if not self.sent_current:
                self.sent_current = True
                self.get_logger().info(f'Executando waypoint {self.index+1}/{len(self.waypoints)}.')
        if not self.sent_current:
            return
        inside = (math.hypot(target.x-self.actual.x, target.y-self.actual.y)
                  <= self.position_tolerance and
                  abs(wrap(target.yaw-self.actual.yaw)) <= self.yaw_tolerance)
        if not inside:
            self.inside_since = None
            return
        if self.inside_since is None:
            self.inside_since = now_s
        if now_s-self.inside_since < self.dwell:
            return
        self.index += 1
        self.inside_since = None
        self.sent_current = False
        self.last_publish = -math.inf
        if self.index == len(self.waypoints):
            self.done = True
            self.finished_pub.publish(Bool(data=True))
            self.get_logger().info('Missão concluída. O controlador manterá comando zero.')


def main(args=None):
    run_node(MissionExecutor, args)
