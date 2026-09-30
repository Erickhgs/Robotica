"""Demonstração cinemática opcional; não reproduz física ou colisões do Gazebo."""
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from .control import Pose, integrate_unicycle, limit_command
from .ros_common import number, parameter, pose_message, run_node


class KinematicSimulator(Node):
    def __init__(self):
        super().__init__('kinematic_sim')
        self.rate = number(self, 'rate', 100.0, True)
        self.pose = Pose(number(self, 'initial.x', 0.0), number(self, 'initial.y', 0.0),
                         number(self, 'initial.yaw', 0.0))
        self.max_v = number(self, 'max_linear_speed', 0.8, True)
        self.max_w = number(self, 'max_angular_speed', 2.0, True)
        self.linear_scale = number(self, 'linear_scale', 1.0, True)
        self.angular_scale = number(self, 'angular_scale', 1.0, True)
        self.timeout = number(self, 'command_timeout', 0.5, True)
        self.frame = parameter(self, 'frame_id', 'odom')
        self.base_frame = parameter(self, 'base_frame_id', 'chassis')
        self.command = (0.0, 0.0)
        self.received = None
        self.last_tick = self.get_clock().now().nanoseconds*1e-9
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.on_command, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.timer = self.create_timer(1.0/self.rate, self.tick)
        self.get_logger().info('Simulador cinemático ideal ativo; sem Gazebo, colisões ou modelo visual.')

    def on_command(self, msg):
        try:
            cmd = limit_command(msg.linear.x, msg.angular.z, self.max_v, self.max_w)
        except ValueError:
            self.command = (0.0, 0.0)
            return
        self.command = (cmd.v, cmd.w)
        self.received = self.get_clock().now().nanoseconds*1e-9

    def tick(self):
        now = self.get_clock().now()
        now_s = now.nanoseconds*1e-9
        dt = now_s-self.last_tick
        self.last_tick = now_s
        v, w = self.command
        if self.received is None or now_s-self.received > self.timeout or now_s < self.received:
            v, w = 0.0, 0.0
        v, w = v*self.linear_scale, w*self.angular_scale
        if 0.0 < dt <= 0.25:
            self.pose = integrate_unicycle(self.pose, v, w, dt)
        msg = Odometry()
        stamped = pose_message(self.pose, now.to_msg(), self.frame)
        msg.header, msg.pose.pose = stamped.header, stamped.pose
        msg.child_frame_id = self.base_frame
        msg.twist.twist.linear.x, msg.twist.twist.angular.z = v, w
        self.odom_pub.publish(msg)


def main(args=None):
    run_node(KinematicSimulator, args)
