import math
import random
import unittest

from robot_state_estimation.geodesy import (
    LocalENU, add_world_yaw, fresh_pair, geodetic_to_ecef, yaw_from_quaternion,
)


class GeodesyTests(unittest.TestCase):
    def test_equatorial_ecef(self):
        self.assertEqual(geodetic_to_ecef(0, 0, 0), (6378137.0, 0.0, 0.0))
        p = geodetic_to_ecef(0, 90, 10)
        self.assertAlmostEqual(p[0], 0, places=8)
        self.assertAlmostEqual(p[1], 6378147, places=8)

    def test_fixed_origin_and_altitude(self):
        enu = LocalENU(-8.05, -34.9, 0)
        self.assertEqual(enu.convert(-8.05, -34.9, 0), (0.0, 0.0, 0.0))
        e, n, u = enu.convert(-8.05, -34.9, 25)
        self.assertAlmostEqual(e, 0, places=7)
        self.assertAlmostEqual(n, 0, places=7)
        self.assertAlmostEqual(u, 25, places=7)

    def test_meter_and_cardinal_directions(self):
        enu = LocalENU(0, 0, 0)
        e, n, u = enu.convert(0, math.degrees(1/6378137), 0)
        self.assertAlmostEqual(e, 1, places=7)
        self.assertAlmostEqual(n, 0, places=7)
        e, n, u = enu.convert(math.degrees(1/6335439.3272928195), 0, 0)
        self.assertAlmostEqual(e, 0, places=7)
        self.assertAlmostEqual(n, 1, places=7)

    def test_rejects_invalid_fix(self):
        for lat, lon, alt in [(91, 0, 0), (0, 181, 0), (0, 0, float('nan'))]:
            with self.assertRaises(ValueError):
                geodetic_to_ecef(lat, lon, alt)

    def test_noise_preserves_quaternion_and_scales(self):
        for sigma in (0.01, 0.08):
            rng, values = random.Random(42), []
            for _ in range(10000):
                q = add_world_yaw(0, 0, 0, 1, rng.gauss(0, sigma))
                self.assertAlmostEqual(sum(v*v for v in q), 1.0, places=12)
                values.append(yaw_from_quaternion(*q))
            self.assertAlmostEqual(math.sqrt(sum(v*v for v in values)/len(values)), sigma, delta=sigma*.03)

    def test_noise_crosses_pi_without_discontinuity(self):
        yaw = math.pi-.01
        q = add_world_yaw(0, 0, math.sin(yaw/2), math.cos(yaw/2), .02)
        self.assertAlmostEqual(yaw_from_quaternion(*q), -math.pi+.01)

    def test_stale_unsynchronized_and_future_samples(self):
        self.assertTrue(fresh_pair(10, 9.98, 9.99, .3, .1))
        self.assertFalse(fresh_pair(10, 9, 9.99, .3, .1))
        self.assertFalse(fresh_pair(10, 9.8, 9.99, .3, .1))
        self.assertFalse(fresh_pair(1, 10, 10, .3, .1))
        self.assertFalse(fresh_pair(10, float('nan'), 9.99, .3, .1))


if __name__ == '__main__':
    unittest.main()
