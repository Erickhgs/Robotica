"""Leis de controle independentes do ROS, com PID da biblioteca simple-pid.

Todas as distâncias estão em metros, os ângulos em radianos e o tempo em segundos.
"""
from dataclasses import dataclass, field
import math

from simple_pid import PID


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Gains:
    kp: float
    ki: float = 0.0
    kd: float = 0.0


@dataclass(frozen=True)
class Command:
    v: float = 0.0
    w: float = 0.0
    saturated: bool = False


def limit_command(v, w, max_v, max_w):
    """Escala os dois comandos juntos para preservar a curvatura desejada."""
    if not all(math.isfinite(n) for n in (v, w, max_v, max_w)):
        raise ValueError('Comandos e limites precisam ser finitos.')
    if max_v <= 0.0 or max_w <= 0.0:
        raise ValueError('Os limites de velocidade precisam ser positivos.')
    scale = max(1.0, abs(v) / max_v, abs(w) / max_w)
    return Command(v / scale, w / scale, scale > 1.0 + 1e-12)


class ErrorPID:
    """Adaptador de erro para simple-pid, usando dt explícito do relógio ROS.

Nos PIDs angulares, desembrulha a entrada da derivada para evitar um salto
artificial de 2*pi ao cruzar -pi/pi. O erro proporcional continua normalizado.
    """
    def __init__(self, gains, limit, angular=False, nonnegative=False):
        self.angular = angular
        self.pid = PID(
            gains.kp, gains.ki, gains.kd, setpoint=0.0, sample_time=None,
            output_limits=(0.0 if nonnegative else -limit, limit),
            error_map=wrap if angular else None,
        )
        self.last_error = None
        self.unwrapped = 0.0

    def reset(self):
        self.pid.reset()
        self.last_error = None
        self.unwrapped = 0.0

    def __call__(self, error, dt):
        if dt <= 0.0 or not math.isfinite(dt):
            raise ValueError('dt precisa ser positivo e finito.')
        if not math.isfinite(error):
            raise ValueError('Erro de controle não finito.')
        if self.angular:
            self.unwrapped = (error if self.last_error is None else
                              self.unwrapped + wrap(error - self.last_error))
            self.last_error = error
            error = self.unwrapped
        return self.pid(-error, dt=dt)


@dataclass
class PoseConfig:
    method: str = 'continuous'
    max_v: float = 0.8
    max_w: float = 2.0
    position_tolerance: float = 0.04
    yaw_tolerance: float = 0.06
    distance: Gains = field(default_factory=lambda: Gains(0.8))
    heading: Gains = field(default_factory=lambda: Gains(2.8))
    final_heading: Gains = field(default_factory=lambda: Gains(2.0))
    beta: Gains = field(default_factory=lambda: Gains(0.8))


class PoseLaw:
    def __init__(self, config):
        self.config = config
        if config.method not in ('continuous', 'three_maneuvers'):
            raise ValueError('method deve ser continuous ou three_maneuvers.')
        for name in ('max_v', 'max_w', 'position_tolerance', 'yaw_tolerance'):
            if not math.isfinite(getattr(config, name)) or getattr(config, name) <= 0:
                raise ValueError(name + ' precisa ser positivo e finito.')
        self.rho_pid = ErrorPID(config.distance, config.max_v, nonnegative=True)
        self.alpha_pid = ErrorPID(config.heading, config.max_w, angular=True)
        self.beta_pid = ErrorPID(config.beta, config.max_w, angular=True)
        self.yaw_pid = ErrorPID(config.final_heading, config.max_w, angular=True)
        self.reset()

    def reset(self):
        for pid in (self.rho_pid, self.alpha_pid, self.beta_pid, self.yaw_pid):
            pid.reset()
        self.phase = 'rotate_to_path'

    def reached(self, actual, target):
        return (
            math.hypot(target.x - actual.x, target.y - actual.y)
            <= self.config.position_tolerance
            and abs(wrap(target.yaw - actual.yaw)) <= self.config.yaw_tolerance
        )

    def command(self, actual, target, dt):
        c = self.config
        dx, dy = target.x - actual.x, target.y - actual.y
        rho = math.hypot(dx, dy)
        yaw_error = wrap(target.yaw - actual.yaw)
        if rho <= c.position_tolerance:
            self.phase = 'rotate_final'
            if abs(yaw_error) <= c.yaw_tolerance:
                return Command()
            return limit_command(0.0, self.yaw_pid(yaw_error, dt), c.max_v, c.max_w)

        alpha = wrap(math.atan2(dy, dx) - actual.yaw)
        if c.method == 'three_maneuvers':
            # Uma nova perturbação de posição após a rotação final reinicia a manobra.
            if self.phase == 'rotate_final':
                self.phase = 'rotate_to_path'
                self.alpha_pid.reset()
            if self.phase == 'rotate_to_path':
                if abs(alpha) > c.yaw_tolerance:
                    return limit_command(0.0, self.alpha_pid(alpha, dt), c.max_v, c.max_w)
                self.phase = 'translate'
                self.rho_pid.reset()
                self.alpha_pid.reset()
            # Pequenas correções de rumo durante a translação evitam deriva lateral.
            if abs(alpha) > 0.6:
                self.phase = 'rotate_to_path'
                self.alpha_pid.reset()
                return Command()
            v = self.rho_pid(rho, dt) * max(0.0, math.cos(alpha))
            w = self.alpha_pid(alpha, dt)
        else:
            self.phase = 'continuous'
            beta = wrap(target.yaw - actual.yaw - alpha)
            v = self.rho_pid(rho, dt) * max(0.0, math.cos(alpha))
            # O sinal negativo corresponde a k_beta < 0 no controlador polar.
            w = self.alpha_pid(alpha, dt) - self.beta_pid(beta, dt)
        return limit_command(v, w, c.max_v, c.max_w)


@dataclass
class EightConfig:
    amplitude_x: float = 1.0
    amplitude_y: float = 2.0
    omega: float = 0.5
    center_x: float = 0.0
    center_y: float = 0.0
    mode: str = 'feedback'
    kff: float = 1.0
    lookahead: float = 0.15
    max_v: float = 0.8
    max_w: float = 2.0
    x_pid: Gains = field(default_factory=lambda: Gains(1.8))
    y_pid: Gains = field(default_factory=lambda: Gains(1.8))

    def __post_init__(self):
        if self.mode not in ('open_loop', 'feedback'):
            raise ValueError('mode deve ser open_loop ou feedback.')
        for name in ('amplitude_x', 'amplitude_y', 'omega', 'lookahead', 'max_v', 'max_w'):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(name + ' precisa ser positivo e finito.')
        if not all(math.isfinite(v) for v in (self.center_x, self.center_y, self.kff)):
            raise ValueError('Centro e Kff precisam ser finitos.')
        if self.kff < 0.0:
            raise ValueError('Kff não pode ser negativo.')


@dataclass(frozen=True)
class Reference:
    pose: Pose
    dx: float
    dy: float
    v: float
    w: float


def figure_eight(t, config):
    """x=A*sin(Omega*t), y=B*sin(2*Omega*t), mais um centro fixo.

A equação foi adotada porque o roteiro não inclui os slides citados.
    """
    a, b, om = config.amplitude_x, config.amplitude_y, config.omega
    s = om * t
    x = config.center_x + a * math.sin(s)
    y = config.center_y + b * math.sin(2.0 * s)
    dx, dy = a * om * math.cos(s), 2.0 * b * om * math.cos(2.0 * s)
    ddx, ddy = -a * om * om * math.sin(s), -4.0 * b * om * om * math.sin(2.0 * s)
    speed_sq = dx * dx + dy * dy
    return Reference(Pose(x, y, math.atan2(dy, dx)), dx, dy,
                     math.sqrt(speed_sq), (dx * ddy - dy * ddx) / speed_sq)


class EightLaw:
    """Linearização por ponto à frente, com referência do centro compensada.

p = [x + d*cos(theta), y + d*sin(theta)]
p_d = [x_d + d*cos(theta_d), y_d + d*sin(theta_d)]
u = Kff * dp_d/dt + PID(p_d-p)
[v,w] = inv([[cos(theta), -d*sin(theta)],
             [sin(theta),  d*cos(theta)]]) * u

Assim, as curvas publicadas continuam representando o centro do robô.
    """
    def __init__(self, config):
        self.config = config
        correction_limit = config.max_v + config.lookahead * config.max_w
        self.x_pid = ErrorPID(config.x_pid, correction_limit)
        self.y_pid = ErrorPID(config.y_pid, correction_limit)

    def reset(self):
        self.x_pid.reset()
        self.y_pid.reset()

    def command(self, actual, reference, dt):
        c, r = self.config, reference
        if c.mode == 'open_loop':
            return limit_command(r.v, r.w, c.max_v, c.max_w)
        ct, st = math.cos(actual.yaw), math.sin(actual.yaw)
        cd, sd = math.cos(r.pose.yaw), math.sin(r.pose.yaw)
        ex = r.pose.x + c.lookahead * cd - actual.x - c.lookahead * ct
        ey = r.pose.y + c.lookahead * sd - actual.y - c.lookahead * st
        ux = c.kff * (r.dx - c.lookahead * sd * r.w) + self.x_pid(ex, dt)
        uy = c.kff * (r.dy + c.lookahead * cd * r.w) + self.y_pid(ey, dt)
        v = ct * ux + st * uy
        w = (-st * ux + ct * uy) / c.lookahead
        return limit_command(v, w, c.max_v, c.max_w)


def trajectory_limits(config, samples=10000):
    """Estimativa por amostragem, sem confundir viabilidade cinemática com ensaio."""
    period = 2.0 * math.pi / config.omega
    refs = (figure_eight(i * period / samples, config) for i in range(samples))
    max_v, max_w = 0.0, 0.0
    for ref in refs:
        max_v, max_w = max(max_v, ref.v), max(max_w, abs(ref.w))
    omega_bound = config.omega * min(config.max_v / max_v, config.max_w / max_w)
    return max_v, max_w, omega_bound


def integrate_unicycle(pose, v, w, dt):
    """Integração exata de um comando constante no intervalo dt."""
    if abs(w) < 1e-10:
        return Pose(pose.x + v * math.cos(pose.yaw) * dt,
                    pose.y + v * math.sin(pose.yaw) * dt, pose.yaw)
    next_yaw = pose.yaw + w * dt
    return Pose(pose.x + v / w * (math.sin(next_yaw) - math.sin(pose.yaw)),
                pose.y - v / w * (math.cos(next_yaw) - math.cos(pose.yaw)),
                wrap(next_yaw))
