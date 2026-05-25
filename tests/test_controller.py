"""IntersectionController and FailSafe tests — unittest-compatible."""
import sys, os
import time
import unittest
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import src.config as cfg
from src.comm.broker import Broker
from src.controller.controller import IntersectionController
from src.controller.failsafe import FailSafeMonitor


class TestFailSafe(unittest.TestCase):
    def setUp(self):
        self.monitor = FailSafeMonitor()

    def test_legal_ns_green_to_yellow(self):
        ok, _ = self.monitor.validate_transition(cfg.STATE_NS_GREEN, cfg.STATE_NS_YELLOW)
        self.assertTrue(ok)

    def test_illegal_ns_green_to_ew_green(self):
        ok, reason = self.monitor.validate_transition(cfg.STATE_NS_GREEN, cfg.STATE_EW_GREEN)
        self.assertFalse(ok)
        self.assertIn("ILLEGAL", reason)

    def test_conflict_ns_ew_green(self):
        self.assertTrue(
            self.monitor.check_conflict(cfg.STATE_NS_GREEN, cfg.STATE_EW_GREEN)
        )

    def test_no_conflict_ns_green_all_red(self):
        self.assertFalse(
            self.monitor.check_conflict(cfg.STATE_NS_GREEN, cfg.STATE_ALL_RED)
        )

    def test_stats_tracking(self):
        self.monitor.validate_transition(cfg.STATE_NS_GREEN, cfg.STATE_NS_YELLOW)
        self.monitor.validate_transition(cfg.STATE_NS_GREEN, cfg.STATE_EW_GREEN)
        self.assertEqual(self.monitor.stats["accepted"], 1)
        self.assertEqual(self.monitor.stats["rejected"], 1)


class TestControllerNormalCycle(unittest.TestCase):
    def setUp(self):
        self.broker = Broker()
        self.ctrl   = IntersectionController(self.broker, zone_id="Test")

    def _step_n(self, n, dt=0.1):
        for _ in range(n):
            self.ctrl.step(dt)

    def test_initial_state_ns_green(self):
        self.assertEqual(self.ctrl.state, cfg.STATE_NS_GREEN)

    def test_no_simultaneous_green_200steps(self):
        for _ in range(200):
            self.ctrl.step(0.1)
            sig = self.ctrl.get_signal_state()
            self.assertFalse(
                sig["NS"] == "GREEN" and sig["EW"] == "GREEN",
                f"CONFLICT: NS={sig['NS']}, EW={sig['EW']}, state={self.ctrl.state}"
            )

    def test_signal_state_has_required_keys(self):
        sig = self.ctrl.get_signal_state()
        for key in ("state","NS","EW","zone_id","phase_timer","mode"):
            self.assertIn(key, sig)

    def test_phase_timer_advances(self):
        self.ctrl.step(0.5)
        self.assertGreater(self.ctrl.phase_timer, 0)


class TestPreemption(unittest.TestCase):
    def setUp(self):
        self.broker = Broker()
        self.ctrl   = IntersectionController(self.broker, zone_id="Emerg")

    def _push_ambulance(self, lane="north", eta=8.0):
        self.broker.publish({
            "vehicle_id": "AMB_TEST", "timestamp": time.time(),
            "zone_id": "Emerg", "lane_id": lane,
            "vehicle_type": "ambulance",
            "position": {"x": 0, "y": 300},
            "velocity": 20.0, "acceleration": 0.0,
            "priority_flag": 1,
            "distance_to_intersection": 160.0,
            "eta": eta, "wait_time": 0.0, "active": True,
        })

    def test_preemption_triggered(self):
        self._push_ambulance()
        for _ in range(50):
            self.ctrl.step(0.1)
        self.assertGreaterEqual(self.ctrl.preempt_count, 1)

    def test_no_conflict_during_preemption_200steps(self):
        self._push_ambulance(lane="east")
        for i in range(200):
            self.ctrl.step(0.05)
            sig = self.ctrl.get_signal_state()
            self.assertFalse(
                sig["NS"] == "GREEN" and sig["EW"] == "GREEN",
                f"CONFLICT at step {i}: {sig}"
            )

    def test_yellow_duration_enforced(self):
        """Yellow phase must last >= YELLOW_DURATION seconds."""
        yellow_start = None
        self._push_ambulance()
        for i in range(500):
            self.ctrl.step(0.05)
            if self.ctrl.state in (cfg.STATE_NS_YELLOW, cfg.STATE_EW_YELLOW):
                if yellow_start is None:
                    yellow_start = i * 0.05
            elif yellow_start is not None:
                yellow_duration = i * 0.05 - yellow_start
                self.assertGreaterEqual(
                    yellow_duration, cfg.YELLOW_DURATION - 0.15,
                    f"Yellow too short: {yellow_duration:.2f}s"
                )
                break


if __name__ == '__main__':
    unittest.main(verbosity=2)
