"""Testes matemáticos; não substituem um ensaio ROS/Gazebo."""
import csv
import math
from pathlib import Path
import tempfile
import unittest

from pratica03_controle.analyze import summarize
from pratica03_controle.control import (
    EightConfig, EightLaw, ErrorPID, Gains, Pose, PoseConfig, PoseLaw,
    figure_eight, integrate_unicycle, limit_command, trajectory_limits, wrap,
)


def run_eight(mode, kff, bias=True, offset=False):
    config = EightConfig(omega=0.1, mode=mode, kff=kff)
    law = EightLaw(config)
    p = figure_eight(0.0, config).pose
    if offset:
        p = Pose(p.x+0.2, p.y-0.1, p.yaw+0.3)
    errors = []
    for i in range(int(2*math.pi/config.omega/0.02)):
        ref = figure_eight(i*0.02, config)
        cmd = law.command(p, ref, 0.02)
        errors.append((p.x-ref.pose.x)**2 + (p.y-ref.pose.y)**2)
        p = integrate_unicycle(p, cmd.v*(0.95 if bias else 1.0),
                               cmd.w*(1.05 if bias else 1.0), 0.02)
    return math.sqrt(sum(errors)/len(errors)), math.sqrt(sum(errors[len(errors)//2:])/
                                                       len(errors[len(errors)//2:]))


class ControlTests(unittest.TestCase):
    def test_sequential_mission_converges_in_both_methods(self):
        goals = [Pose(1., 0., 0.), Pose(1., 1., math.pi/2),
                 Pose(0., 1., math.pi), Pose(0., 0., 0.), Pose(-1., -.5, -2.)]
        for method in ('continuous', 'three_maneuvers'):
            with self.subTest(method=method):
                law = PoseLaw(PoseConfig(method=method))
                actual = Pose(0., 0., 0.)
                for target in goals:
                    law.reset()
                    for _ in range(3000):
                        cmd = law.command(actual, target, .02)
                        self.assertLessEqual(abs(cmd.v), .8+1e-12)
                        self.assertLessEqual(abs(cmd.w), 2.+1e-12)
                        actual = integrate_unicycle(actual, cmd.v, cmd.w, .02)
                        if law.reached(actual, target):
                            break
                    self.assertTrue(law.reached(actual, target), (method, actual, target))

    def test_orientation_only_and_wrap_crossing(self):
        for method in ('continuous', 'three_maneuvers'):
            law = PoseLaw(PoseConfig(method=method))
            actual, target = Pose(1., 2., 3.0), Pose(1., 2., -3.0)
            for _ in range(500):
                cmd = law.command(actual, target, .02)
                self.assertEqual(cmd.v, 0.0)
                actual = integrate_unicycle(actual, cmd.v, cmd.w, .02)
            self.assertTrue(law.reached(actual, target))
            self.assertEqual((actual.x, actual.y), (1., 2.))

    def test_angular_derivative_does_not_spike_at_pi(self):
        pid = ErrorPID(Gains(0.0, 0.0, 1.0), 100.0, angular=True)
        pid(math.pi-.001, .02)
        self.assertAlmostEqual(pid(-math.pi+.001, .02), .1, places=8)

    def test_reference_derivatives_by_finite_difference(self):
        config = EightConfig(omega=.1)
        h = 1e-5
        for t in (0., 2.3, 11., 20., 50.):
            ref, before, after = (figure_eight(s, config) for s in (t, t-h, t+h))
            self.assertAlmostEqual((after.pose.x-before.pose.x)/(2*h), ref.dx, places=7)
            self.assertAlmostEqual((after.pose.y-before.pose.y)/(2*h), ref.dy, places=7)
            self.assertAlmostEqual(wrap(after.pose.yaw-before.pose.yaw)/(2*h), ref.w, places=7)

    def test_feedforward_is_exact_on_reference(self):
        config = EightConfig(omega=.1)
        law = EightLaw(config)
        for t in (0., 3., 12., 25., 47.):
            ref = figure_eight(t, config)
            cmd = law.command(ref.pose, ref, .02)
            self.assertAlmostEqual(cmd.v, ref.v, places=9)
            self.assertAlmostEqual(cmd.w, ref.w, places=9)

    def test_open_loop_is_independent_of_measured_pose(self):
        config = EightConfig(omega=.1, mode='open_loop')
        law, ref = EightLaw(config), figure_eight(4., config)
        self.assertEqual(law.command(Pose(0., 0., 0.), ref, .02),
                         law.command(Pose(10., -3., -2.), ref, .02))

    def test_feedback_rejects_actuation_bias(self):
        open_rmse, _ = run_eight('open_loop', 1.)
        feedback_rmse, _ = run_eight('feedback', 0.)
        ff_rmse, _ = run_eight('feedback', 1.)
        self.assertLess(feedback_rmse, open_rmse)
        self.assertLess(ff_rmse, .03)
        self.assertLess(ff_rmse, feedback_rmse/5.)

    def test_feedback_recovers_initial_offset(self):
        _, final_half_rmse = run_eight('feedback', 1., bias=True, offset=True)
        self.assertLess(final_half_rmse, .03)

    def test_ideal_reference_tracking(self):
        rmse, _ = run_eight('feedback', 1., bias=False)
        self.assertLess(rmse, .01)

    def test_velocity_saturation_and_feasibility(self):
        config = EightConfig()
        max_v, max_w, omega_bound = trajectory_limits(config)
        self.assertAlmostEqual(max_v, math.sqrt(17)*.5, places=8)
        self.assertGreater(max_w, 2.)
        self.assertLess(omega_bound, .2)
        for i in range(1000):
            ref = figure_eight(i*.01, config)
            cmd = limit_command(ref.v, ref.w, .8, 2.)
            self.assertLessEqual(abs(cmd.v), .8+1e-12)
            self.assertLessEqual(abs(cmd.w), 2.+1e-12)
            self.assertAlmostEqual(cmd.w/cmd.v, ref.w/ref.v, places=8)

    def test_invalid_parameters(self):
        for args in ({'omega': 0.}, {'lookahead': 0.}, {'kff': -1.}, {'mode': 'invalid'}):
            with self.assertRaises(ValueError):
                EightConfig(**args)
        with self.assertRaises(ValueError):
            limit_command(float('nan'), 0., .8, 2.)

    def test_rmse_excludes_alignment_and_detects_completion(self):
        fields = ['phase', 'time', 'error_x', 'error_y', 'error_yaw',
                  'saturated', 'mode', 'kff', 'omega']
        data = [
            ['align', 0., 100., 100., 3., 0, 'feedback', 1., .1],
            ['run', 0., 3., 4., .1, 1, 'feedback', 1., .1],
            ['run', .02, 0., 0., -.1, 0, 'feedback', 1., .1],
            ['complete', .04, 50., 50., 2., 0, 'feedback', 1., .1],
        ]
        with tempfile.TemporaryDirectory() as directory:
            filename = Path(directory)/'trial.csv'
            with filename.open('w', newline='') as stream:
                writer = csv.writer(stream)
                writer.writerow(fields)
                writer.writerows(data)
            result = summarize(filename)
        self.assertEqual(result['samples'], 2)
        self.assertAlmostEqual(result['rmse_position_m'], math.sqrt(12.5))
        self.assertAlmostEqual(result['rmse_yaw_rad'], .1)
        self.assertEqual(result['saturation_percent'], 50.)
        self.assertTrue(result['trajectory_complete'])


if __name__ == '__main__':
    unittest.main()
