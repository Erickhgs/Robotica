"""Exercício 1.1: controle contínuo ou por três manobras."""
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Int32

from .control import Command, PoseLaw
from .ros_common import MotionNode, Telemetry, parameter, pose_config, run_node, to_pose


class PoseController(MotionNode):
    def __init__(self):
        super().__init__('pose_controller')
        self.law = PoseLaw(pose_config(self))
        self.goal = None
        self.elapsed = 0.0
        self.goal_index = -1
        self.was_reached = False
        self.mission_done = False
        self.mission_mode = bool(parameter(self, 'mission_mode', False))
        self.telemetry = Telemetry(self, 'pose_' + self.law.config.method,
                                   mode=self.law.config.method)
        self.goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self.on_goal, 10)
        self.index_sub = self.create_subscription(Int32, 'mission/index', self.on_index, 10)
        self.done_sub = self.create_subscription(Bool, 'mission/finished', self.on_finished, 10)
        self.reached_pub = self.create_publisher(Bool, 'goal_reached', 10)
        self.timer = self.create_timer(1.0/self.rate, self.tick)
        self.get_logger().info(f'Controle de pose: {self.law.config.method}; aguardando /goal_pose.')

    def on_goal(self, msg):
        if self.mission_done:
            return
        if msg.header.frame_id != self.frame:
            self.get_logger().error(f'Objetivo deve usar frame_id={self.frame!r}.')
            return
        try:
            target = to_pose(msg.pose)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return
        # A missão repete o mesmo objetivo; não reiniciar os PIDs nessas repetições.
        if target != self.goal:
            self.goal = target
            self.law.reset()
            self.was_reached = False
            self.get_logger().info(
                f'Objetivo: x={target.x:.2f}, y={target.y:.2f}, yaw={target.yaw:.2f}')

    def on_index(self, msg):
        self.goal_index = msg.data

    def on_finished(self, msg):
        if self.mission_mode and msg.data:
            self.mission_done = True
            self.send(Command())
            self.telemetry.close()

    def tick(self):
        if self.mission_done:
            self.send(Command())
            return
        sample = self.sample()
        if sample is None:
            self.law.reset()
            return
        if self.goal is None:
            self.send(Command())
            return
        _, dt = sample
        reached = self.law.reached(self.actual, self.goal)
        command = self.law.command(self.actual, self.goal, dt)
        self.send(command)
        self.reached_pub.publish(Bool(data=reached))
        if reached and not self.was_reached:
            self.get_logger().info('Pose alcançada dentro das tolerâncias.')
        self.was_reached = reached
        self.telemetry.record(self.elapsed, self.actual, self.goal, command,
                              'hold' if reached else 'active', self.goal_index)
        self.elapsed += dt


def main(args=None):
    run_node(PoseController, args)
