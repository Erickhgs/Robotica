"""Exercício 2: trajetória em oito com malha aberta e feedback + feedforward."""
import math

from .control import Command, EightConfig, EightLaw, PoseLaw, figure_eight, trajectory_limits
from .ros_common import MotionNode, Telemetry, gains, number, parameter, pose_config, run_node


class FigureEightController(MotionNode):
    def __init__(self):
        super().__init__('figure_eight')
        self.config = EightConfig(
            amplitude_x=number(self, 'trajectory.A', 1.0, True),
            amplitude_y=number(self, 'trajectory.B', 2.0, True),
            omega=number(self, 'trajectory.omega', 0.5, True),
            center_x=number(self, 'trajectory.center_x', 0.0),
            center_y=number(self, 'trajectory.center_y', 0.0),
            mode=parameter(self, 'mode', 'feedback'),
            kff=number(self, 'Kff', 1.0),
            lookahead=number(self, 'lookahead_distance', 0.15, True),
            max_v=number(self, 'max_linear_speed', 0.8, True),
            max_w=number(self, 'max_angular_speed', 2.0, True),
            x_pid=gains(self, 'pid.x', 1.8),
            y_pid=gains(self, 'pid.y', 1.8),
        )
        self.law = EightLaw(self.config)
        self.alignment = PoseLaw(pose_config(self, 'alignment.'))
        self.cycles = int(parameter(self, 'trajectory.cycles', 1))
        if self.cycles < 1:
            raise ValueError('trajectory.cycles precisa ser um inteiro >= 1.')
        self.duration = self.cycles*2.0*math.pi/self.config.omega
        self.align_start = bool(parameter(self, 'align_start', True))
        self.settle_time = number(self, 'settle_time', 1.0, True)
        self.align_timeout = number(self, 'alignment.timeout', 120.0, True)
        self.phase = 'align' if self.align_start else 'run'
        self.align_elapsed = 0.0
        self.run_elapsed = 0.0
        self.settle_elapsed = 0.0
        self.initial = figure_eight(0.0, self.config).pose
        self.telemetry = Telemetry(
            self, f'eight_{self.config.mode}_kff{self.config.kff:g}_omega{self.config.omega:g}',
            mode=self.config.mode, kff=self.config.kff, omega=self.config.omega)
        max_v, max_w, omega_bound = trajectory_limits(self.config)
        self.get_logger().info(
            f'Trajetória: A={self.config.amplitude_x:g}, B={self.config.amplitude_y:g}, '
            f'Omega={self.config.omega:g}, modo={self.config.mode}, Kff={self.config.kff:g}. '
            f'Duração após alinhamento: {self.duration:.2f} s.')
        if self.config.mode == 'open_loop':
            self.get_logger().info('Malha aberta: usa apenas v_d e w_d; Kff não se aplica.')
        if max_v > self.config.max_v or max_w > self.config.max_w:
            self.get_logger().warning(
                f'Referência pede até {max_v:.3f} m/s e {max_w:.3f} rad/s. '
                f'Comandos serão limitados a {self.config.max_v:g} m/s e '
                f'{self.config.max_w:g} rad/s. Estimativa de Omega viável apenas '
                f'por velocidade: <= {omega_bound:.3f} rad/s (não é resultado experimental).')
        self.timer = self.create_timer(1.0/self.rate, self.tick)

    def tick(self):
        if self.phase in ('done', 'failed'):
            self.send(Command())
            return
        sample = self.sample()
        if sample is None:
            self.law.reset()
            self.alignment.reset()
            self.settle_elapsed = 0.0
            return
        _, dt = sample
        if self.phase in ('align', 'settle'):
            self.align_elapsed += dt
            if self.align_elapsed > self.align_timeout:
                self.send(Command())
                self.phase = 'failed'
                self.telemetry.record(0.0, self.actual, self.initial, Command(), 'failed')
                self.telemetry.close()
                self.get_logger().error('Tempo de alinhamento esgotado. Confira odometria e comandos.')
                return
            if not self.alignment.reached(self.actual, self.initial):
                self.phase = 'align'
                self.settle_elapsed = 0.0
                command = self.alignment.command(self.actual, self.initial, dt)
            else:
                self.phase = 'settle'
                self.settle_elapsed += dt
                command = Command()
            self.send(command)
            self.telemetry.record(0.0, self.actual, self.initial, command, self.phase)
            if self.settle_elapsed >= self.settle_time:
                self.phase = 'run'
                self.law.reset()
                self.get_logger().info('Alinhamento concluído; iniciando trajetória e RMSE.')
            return
        if self.run_elapsed >= self.duration:
            self.send(Command())
            final = figure_eight(self.duration, self.config).pose
            self.telemetry.record(self.duration, self.actual, final, Command(), 'complete')
            self.telemetry.close()
            self.phase = 'done'
            self.get_logger().info('Trajetória concluída. Comando zero; Ctrl+C para encerrar.')
            return
        reference = figure_eight(self.run_elapsed, self.config)
        command = self.law.command(self.actual, reference, dt)
        self.send(command)
        self.telemetry.record(self.run_elapsed, self.actual, reference.pose, command, 'run')
        self.run_elapsed += dt


def main(args=None):
    run_node(FigureEightController, args)
