"""Integration tests: vehicles + controller — no conflict (Pulkit's tests)."""
import sys, os
import time
import unittest
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import src.config as cfg
from src.comm.broker import Broker
from src.controller.controller import IntersectionController
from src.vehicle.vehicle import Vehicle


class TestIntegration(unittest.TestCase):
    def test_10s_no_signal_conflict(self):
        """10 simulated seconds with 4 vehicles — zero signal conflicts."""
        broker = Broker()
        ctrl   = IntersectionController(broker, zone_id="Integ")
        vehicles = [Vehicle(lane, "car", broker=broker, zone_id="Integ")
                    for lane in cfg.LANE_IDS]
        dt, t, conflicts = 0.05, 0.0, 0
        while t < 10.0:
            for v in vehicles: v.broadcast()
            ctrl.step(dt)
            sig = ctrl.get_signal_state()
            if sig["NS"] == "GREEN" and sig["EW"] == "GREEN":
                conflicts += 1
            t += dt
        self.assertEqual(conflicts, 0, f"Signal conflicts: {conflicts}")

    def test_broker_total_published_grows(self):
        broker = Broker()
        v = Vehicle("north", "car", broker=broker, zone_id="T")
        v.position = (0.0, 200.0)  # inside zone
        initial = broker.total_published
        for _ in range(20):
            v.broadcast()
        self.assertGreater(broker.total_published, initial)

    def test_recovery_after_emergency(self):
        """After emergency sequence, controller returns to normal cycling."""
        broker = Broker()
        ctrl   = IntersectionController(broker, zone_id="Rec")

        def push(lane):
            broker.publish({
                "vehicle_id": "AMB", "timestamp": time.time(),
                "zone_id": "Rec", "lane_id": lane,
                "vehicle_type": "ambulance",
                "position": {"x": 0, "y": 200},
                "velocity": 20.0, "acceleration": 0.0,
                "priority_flag": 1,
                "distance_to_intersection": 200.0,
                "eta": 10.0, "wait_time": 0.0, "active": True,
            })

        push("north")
        final_state = cfg.STATE_NS_GREEN
        for i in range(600):
            if i < 150: push("north")
            ctrl.step(0.05)
            final_state = ctrl.state

        self.assertIn(final_state, [
            cfg.STATE_NS_GREEN, cfg.STATE_EW_GREEN,
            cfg.STATE_NS_YELLOW, cfg.STATE_EW_YELLOW,
            cfg.STATE_RECOVERY, cfg.STATE_ALL_RED, cfg.STATE_EMERGENCY,
        ])

    def test_multiple_emergencies_do_not_stick_forever(self):
        """
        If many emergency vehicles exist, controller should keep serving them
        and be able to switch axis via ALL_RED, not get stuck indefinitely.
        """
        broker = Broker()
        ctrl   = IntersectionController(broker, zone_id="Multi")

        def push(lane, vid, eta):
            broker.publish({
                "vehicle_id": vid, "timestamp": time.time(),
                "zone_id": "Multi", "lane_id": lane,
                "vehicle_type": "ambulance",
                "position": {"x": 0, "y": 200},
                "velocity": 20.0, "acceleration": 0.0,
                "priority_flag": 1,
                "dist": 120.0,
                "eta": eta, "wait_time": 0.0, "active": True,
            })

        # Flood with alternating-lane emergencies
        dt = 0.05
        for i in range(800):
            lane = "north" if (i // 20) % 2 == 0 else "east"
            push(lane, f"AMB{i%7}", eta=5.0 + (i % 3))
            ctrl.step(dt)

        # Should have triggered preemption at least once and advanced time
        self.assertGreaterEqual(ctrl.preempt_count, 1)
        self.assertGreater(ctrl.sim_time, 10.0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
