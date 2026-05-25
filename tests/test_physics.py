"""
Life-Link physics unit tests — compatible with both pytest and unittest.
Run: python -m unittest tests.test_physics -v
"""
import math
import sys, os
import unittest
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.vehicle.physics import (
    update_kinematics, braking_distance, calculate_eta,
    euclidean_distance, in_detection_zone,
)


class TestBrakingDistance(unittest.TestCase):
    def test_stationary(self):
        self.assertEqual(braking_distance(0.0), 0.0)

    def test_negative_velocity(self):
        self.assertEqual(braking_distance(-5.0), 0.0)

    def test_known_value(self):
        # v=10 m/s, a=5 m/s^2 → d = 100/10 = 10 m
        self.assertAlmostEqual(braking_distance(10.0, 5.0), 10.0, places=6)

    def test_zero_deceleration(self):
        self.assertEqual(braking_distance(10.0, 0.0), 0.0)

    def test_proportional_to_v_squared(self):
        d1 = braking_distance(10.0, 5.0)
        d2 = braking_distance(20.0, 5.0)
        self.assertAlmostEqual(d2 / d1, 4.0, places=5)


class TestCalculateETA(unittest.TestCase):
    def test_zero_distance(self):
        self.assertEqual(calculate_eta(0.0, 10.0), 0.0)

    def test_zero_velocity_zero_accel(self):
        self.assertEqual(calculate_eta(100.0, 0.0, 0.0), float('inf'))

    def test_uniform_motion(self):
        eta = calculate_eta(100.0, 10.0, 0.0)
        self.assertAlmostEqual(eta, 10.0, places=5)

    def test_with_positive_acceleration(self):
        eta_uniform = calculate_eta(100.0, 10.0, 0.0)
        eta_accel   = calculate_eta(100.0, 10.0, 2.0)
        self.assertLess(eta_accel, eta_uniform)

    def test_heavy_braking_stops_short(self):
        eta = calculate_eta(5.0, 1.0, -10.0)
        self.assertEqual(eta, float('inf'))

    def test_gentle_braking_reachable(self):
        eta = calculate_eta(50.0, 20.0, -1.0)
        self.assertGreater(eta, 0)
        self.assertNotEqual(eta, float('inf'))


class TestUpdateKinematics(unittest.TestCase):
    def test_constant_velocity(self):
        pos, vel, _ = update_kinematics((0.0, 100.0), 10.0, 0.0, 1.0)
        self.assertAlmostEqual(vel, 10.0, places=5)
        self.assertLess(pos[1], 100.0)

    def test_acceleration_increases_speed(self):
        _, vel, _ = update_kinematics((0.0, 200.0), 5.0, 2.0, 1.0)
        self.assertAlmostEqual(vel, 7.0, places=5)

    def test_speed_cap(self):
        _, vel, _ = update_kinematics((0.0, 200.0), 24.0, 2.0, 1.0, max_speed=25.0)
        self.assertLessEqual(vel, 25.0)

    def test_deceleration_non_negative(self):
        _, vel, _ = update_kinematics((0.0, 10.0), 2.0, -5.0, 2.0)
        self.assertGreaterEqual(vel, 0.0)

    def test_analytical_displacement(self):
        # s = v0*t + 0.5*a*t^2
        v0, a, dt = 10.0, 2.0, 0.5
        s_expected = v0 * dt + 0.5 * a * dt**2
        pos_start = (0.0, 300.0)
        pos_new, _, _ = update_kinematics(pos_start, v0, a, dt)
        dy = pos_start[1] - pos_new[1]
        self.assertAlmostEqual(dy, s_expected, delta=0.01)


class TestDetectionZone(unittest.TestCase):
    def test_at_origin(self):
        self.assertEqual(euclidean_distance((0.0, 0.0)), 0.0)

    def test_pythagoras(self):
        self.assertAlmostEqual(euclidean_distance((3.0, 4.0)), 5.0, places=6)

    def test_inside_zone(self):
        self.assertTrue(in_detection_zone((0.0, 300.0), radius=500.0))

    def test_outside_zone(self):
        self.assertFalse(in_detection_zone((0.0, 600.0), radius=500.0))

    def test_on_boundary(self):
        self.assertTrue(in_detection_zone((0.0, 500.0), radius=500.0))


if __name__ == '__main__':
    unittest.main(verbosity=2)
