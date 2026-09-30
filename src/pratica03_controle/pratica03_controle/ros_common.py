"""Parâmetros, odometria, telemetria e encerramento compartilhados."""
import csv
from datetime import datetime
import math
import os
from pathlib import Path as FilePath
import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64, String

from .control import Command, Gains, Pose, PoseConfig, wrap


def parameter(node, name, default):
    if not node.has_parameter(name):
        node.declare_parameter(name, default)
    return node.get_parameter(name).value


def number(node, name, default, positive=False):
    value = float(parameter(node, name, float(default)))
    if not math.isfinite(value) or (positive and value <= 0.0):
        raise ValueError(f'Parâmetro inválido: {name}={value}')
    return value


def gains(node, prefix, kp):
    return Gains(*(number(node, prefix + '.' + key, default)
                   for key, default in (('kp', kp), ('ki', 0.0), ('kd', 0.0))))


def pose_config(node, prefix=''):
    return PoseConfig(
        method=parameter(node, prefix + 'method', 'continuous'),
        max_v=number(node, 'max_linear_speed', 0.8, True),
        max_w=number(node, 'max_angular_speed', 2.0, True),
        position_tolerance=number(node, prefix + 'position_tolerance', 0.04, True),
        yaw_tolerance=number(node, prefix + 'yaw_tolerance', 0.06, True),
        distance=gains(node, prefix + 'pid.distance', 0.8),
        heading=gains(node, prefix + 'pid.heading', 2.8),
        final_heading=gains(node, prefix + 'pid.final_heading', 2.0),
        beta=gains(node, prefix + 'pid.beta', 0.8),
    )


def to_pose(message):
    q = message.orientation
    values = (message.position.x, message.position.y, q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Pose contém valor não finito.')
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if norm < 1e-9:
        raise ValueError('Quaternion nulo; use w=1 para yaw=0.')
    x, y, z, w = (q.x/norm, q.y/norm, q.z/norm, q.w/norm)
    yaw = math.atan2(2.0*(w*z+x*y), 1.0-2.0*(y*y+z*z))
    return Pose(message.position.x, message.position.y, yaw)


def pose_message(pose, stamp, frame):
    msg = PoseStamped()
    msg.header.stamp, msg.header.frame_id = stamp, frame
    msg.pose.position.x, msg.pose.position.y = float(pose.x), float(pose.y)
    msg.pose.orientation.z = math.sin(pose.yaw / 2.0)
    msg.pose.orientation.w = math.cos(pose.yaw / 2.0)
    return msg


class MotionNode(Node):
    """Base: não calcula controle sem odometria recente no frame configurado."""
    def __init__(self, name):
        super().__init__(name)
        self.frame = parameter(self, 'frame_id', 'odom')
        self.rate = number(self, 'control_rate', 50.0, True)
        self.odom_timeout = number(self, 'odom_timeout', 0.5, True)
        self.actual = None
        self.odom_received = None
        self.last_tick = None
        self.telemetry = None
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.on_odom, qos_profile_sensor_data)

    def on_odom(self, msg):
        if msg.header.frame_id != self.frame:
            self.actual = None
            self.get_logger().error(
                f'Odometria em {msg.header.frame_id!r}; esperado {self.frame!r}. '
                'Este pacote não transforma frames automaticamente.', throttle_duration_sec=3.0)
            return
        try:
            self.actual = to_pose(msg.pose.pose)
            self.odom_received = self.get_clock().now().nanoseconds * 1e-9
        except ValueError as exc:
            self.actual = None
            self.get_logger().error(str(exc), throttle_duration_sec=3.0)

    def sample(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = 1.0 / self.rate if self.last_tick is None else now - self.last_tick
        self.last_tick = now
        if self.actual is None or self.odom_received is None or (
                now - self.odom_received > self.odom_timeout):
            self.send(Command())
            self.get_logger().warning(
                'Aguardando odometria recente; comando zerado.', throttle_duration_sec=3.0)
            return None
        if dt <= 0.0 or dt > max(1.0, 4.0/self.rate) or now < self.odom_received:
            self.send(Command())
            self.get_logger().warning(
                'Salto no relógio: ciclo descartado; referência pausada.', throttle_duration_sec=3.0)
            return None
        return now, dt

    def send(self, command):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(command.v), float(command.w)
        self.cmd_pub.publish(msg)

    def stop(self):
        if rclpy.ok():
            self.send(Command())
        if self.telemetry:
            self.telemetry.close()


class Telemetry:
    """Registra centro do robô e referência na mesma amostra de controle."""
    FIELDS = [
        'time', 'ros_time', 'phase', 'goal_index', 'x_ref', 'y_ref', 'yaw_ref',
        'x', 'y', 'yaw', 'error_x', 'error_y', 'error_yaw', 'error_position',
        'v_command', 'w_command', 'saturated', 'mode', 'kff', 'omega',
    ]

    def __init__(self, node, label, mode='', kff=0.0, omega=0.0):
        self.node, self.mode, self.kff, self.omega = node, mode, kff, omega
        output = FilePath(parameter(node, 'output_dir', '~/pratica03_resultados')).expanduser()
        output.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.filename = output / f'{label}_{stamp}_{os.getpid()}.csv'
        self.stream = self.filename.open('x', newline='', encoding='utf-8')
        self.writer = csv.DictWriter(self.stream, fieldnames=self.FIELDS)
        self.writer.writeheader()
        self.reference_pub = node.create_publisher(PoseStamped, 'tracking/reference', 10)
        self.actual_pub = node.create_publisher(PoseStamped, 'tracking/actual', 10)
        self.error_pub = node.create_publisher(Vector3Stamped, 'tracking/error', 10)
        self.reference_path_pub = node.create_publisher(Path, 'tracking/reference_path', 10)
        self.actual_path_pub = node.create_publisher(Path, 'tracking/actual_path', 10)
        self.position_rmse_pub = node.create_publisher(Float64, 'tracking/rmse_position', 10)
        self.yaw_rmse_pub = node.create_publisher(Float64, 'tracking/rmse_yaw', 10)
        self.phase_pub = node.create_publisher(String, 'tracking/phase', 10)
        self.ref_path, self.actual_path = Path(), Path()
        self.last_path_time = -math.inf
        self.count = 0
        self.rows = 0
        self.sum_position_sq = 0.0
        self.sum_yaw_sq = 0.0
        node.get_logger().info(f'Dados do ensaio: {self.filename}')

    def record(self, elapsed, actual, reference, command, phase, goal_index=-1):
        now = self.node.get_clock().now()
        stamp = now.to_msg()
        actual_msg = pose_message(actual, stamp, self.node.frame)
        ref_msg = pose_message(reference, stamp, self.node.frame)
        ex, ey = reference.x-actual.x, reference.y-actual.y
        eyaw = wrap(reference.yaw-actual.yaw)
        error = Vector3Stamped()
        error.header = ref_msg.header
        error.vector.x, error.vector.y, error.vector.z = ex, ey, eyaw
        self.reference_pub.publish(ref_msg)
        self.actual_pub.publish(actual_msg)
        self.error_pub.publish(error)
        self.phase_pub.publish(String(data=phase))
        if phase in ('run', 'active', 'hold'):
            self.count += 1
            self.sum_position_sq += ex*ex + ey*ey
            self.sum_yaw_sq += eyaw*eyaw
            self.position_rmse_pub.publish(Float64(data=math.sqrt(self.sum_position_sq/self.count)))
            self.yaw_rmse_pub.publish(Float64(data=math.sqrt(self.sum_yaw_sq/self.count)))
        now_s = now.nanoseconds * 1e-9
        if now_s < self.last_path_time:
            self.ref_path.poses.clear()
            self.actual_path.poses.clear()
            self.last_path_time = -math.inf
        if now_s-self.last_path_time >= 0.1:
            self.ref_path.header = self.actual_path.header = ref_msg.header
            self.ref_path.poses.append(ref_msg)
            self.actual_path.poses.append(actual_msg)
            self.ref_path.poses = self.ref_path.poses[-5000:]
            self.actual_path.poses = self.actual_path.poses[-5000:]
            self.reference_path_pub.publish(self.ref_path)
            self.actual_path_pub.publish(self.actual_path)
            self.last_path_time = now_s
        self.writer.writerow(dict(zip(self.FIELDS, [
            elapsed, now_s, phase, goal_index, reference.x, reference.y, reference.yaw,
            actual.x, actual.y, actual.yaw, ex, ey, eyaw, math.hypot(ex, ey),
            command.v, command.w, int(command.saturated), self.mode, self.kff, self.omega,
        ])))
        self.rows += 1
        if self.rows % 50 == 0:
            self.stream.flush()

    def close(self):
        if not self.stream.closed:
            self.stream.close()
            self.node.get_logger().info(f'CSV salvo: {self.filename}')


def run_node(node_class, args=None):
    # Mantém o contexto vivo até publicar zero e fechar o CSV no Ctrl+C/SIGTERM.
    def interrupt(signum, frame):
        raise KeyboardInterrupt

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    old_int = signal.signal(signal.SIGINT, interrupt)
    old_term = signal.signal(signal.SIGTERM, interrupt)
    node = None
    try:
        node = node_class()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if hasattr(node, 'stop'):
                node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGINT, old_int)
        signal.signal(signal.SIGTERM, old_term)
