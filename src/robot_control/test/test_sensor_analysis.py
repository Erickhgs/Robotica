import csv
import math
from pathlib import Path
import tempfile
import unittest

from robot_control.analyze_sensors import interpolate_truth, summarize_trial


class SensorAnalysisTests(unittest.TestCase):
    def test_interpolation_wraps_yaw(self):
        p = interpolate_truth([1, 2], [(0, 0, math.pi-.1), (2, 4, -math.pi+.1)], 1.5, 2)
        self.assertEqual(p[:2], (1, 2))
        self.assertAlmostEqual(abs(p[2]), math.pi)

    def test_rejects_extrapolation_and_large_gaps(self):
        for t in (0, 1.5, 3):
            self.assertIsNone(interpolate_truth([1, 2], [(0,0,0), (1,1,0)], t))

    def test_separates_real_error_from_noisy_measurement(self):
        with tempfile.TemporaryDirectory() as temp:
            p = Path(temp)
            fields = ['phase','ros_time','time','x_ref','y_ref','yaw_ref','x','y','yaw',
                      'v_command','w_command','saturated']
            with (p/'eight_test.csv').open('w', newline='') as f:
                w = csv.writer(f); w.writerow(fields)
                w.writerow(['run', 1.02, 0, 0, 0, 0, 3, 4, 0, .2, .1, 0])
                w.writerow(['run', 1.04, .02, 0, 0, 0, 3, 4, 0, .4, -.1, 1])
                w.writerow(['complete', 1.06, .04, 0, 0, 0, 3, 4, 0, 0, 0, 0])
            with (p/'ground_truth_test.csv').open('w', newline='') as f:
                w = csv.writer(f); w.writerow(['ros_time','x','y','yaw'])
                w.writerow([1,0,0,0]); w.writerow([1.05,0,0,0])
            s = summarize_trial(p)
            self.assertEqual(s['rmse_tracking_true_m'], 0)
            self.assertEqual(s['rmse_estimation_m'], 5)
            self.assertEqual(s['saturation_percent'], 50)
            self.assertTrue(s['complete'])
            self.assertAlmostEqual(s['mean_abs_delta_w'], .2)
            self.assertEqual(s['coverage_percent'], 100)


if __name__ == '__main__':
    unittest.main()
