"""V2I packet and Broker tests — unittest-compatible."""
import time, sys, os, unittest
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.vehicle.vehicle import Vehicle
from src.comm.broker import Broker

REQUIRED_KEYS = {
    "vehicle_id","timestamp","zone_id","lane_id","vehicle_type",
    "position","velocity","acceleration","priority_flag",
    "distance_to_intersection","eta","wait_time","active",
}

class TestPacketStructure(unittest.TestCase):
    def test_packet_has_all_required_keys(self):
        v = Vehicle("north", "car")
        pkt = v.to_packet()
        missing = REQUIRED_KEYS - pkt.keys()
        self.assertFalse(missing, f"Missing keys: {missing}")

    def test_ambulance_priority_flag_1(self):
        self.assertEqual(Vehicle("south","ambulance").to_packet()["priority_flag"], 1)

    def test_car_priority_flag_0(self):
        self.assertEqual(Vehicle("east","car").to_packet()["priority_flag"], 0)

    def test_timestamp_is_recent(self):
        self.assertLess(abs(Vehicle("west","bike").to_packet()["timestamp"] - time.time()), 2.0)

    def test_position_has_xy(self):
        pos = Vehicle("north","auto").to_packet()["position"]
        self.assertIn("x", pos); self.assertIn("y", pos)

    def test_velocity_non_negative(self):
        self.assertGreaterEqual(Vehicle("south","car").to_packet()["velocity"], 0)

    def test_eta_non_negative_or_none(self):
        eta = Vehicle("north","car").to_packet()["eta"]
        self.assertTrue(eta is None or eta >= 0)


class TestBrokerAPI(unittest.TestCase):
    def _make_vehicle_inside_zone(self, lane, vtype, broker):
        v = Vehicle(lane, vtype, broker=broker, zone_id="Test")
        # Force inside 500m detection zone
        import math
        x, y = v.position
        dist = math.hypot(x, y)
        if dist > 0:
            scale = 200.0 / dist   # push to 200m
            v.position = (x * scale, y * scale)
        return v

    def test_publish_and_get(self):
        broker = Broker()
        v = self._make_vehicle_inside_zone("north", "car", broker)
        v.broadcast()
        pkts = broker.get_packets()
        self.assertEqual(len(pkts), 1)
        self.assertEqual(pkts[0]["vehicle_id"], v.vehicle_id)

    def test_get_drains_queue(self):
        broker = Broker()
        v = self._make_vehicle_inside_zone("north", "car", broker)
        v.broadcast()
        _ = broker.get_packets()
        self.assertEqual(broker.get_packets(), [])

    def test_malformed_packet_rejected(self):
        broker = Broker()
        with self.assertRaises(ValueError):
            broker.publish({"vehicle_id": "X"})

    def test_smoke_10_vehicles_publish(self):
        """10 vehicles all inside zone: each broadcasts, broker count >=10."""
        broker = Broker()
        vehicles = [
            self._make_vehicle_inside_zone(lane, "car", broker)
            for lane in ["north","south","east","west"]
            for _ in range(3)
        ]
        dt, t = 0.05, 0.0
        while t < 2.0:
            for v in vehicles: v.broadcast()
            t += dt
        self.assertGreaterEqual(broker.total_published, 10)

    def test_ambulance_far_away_no_publish(self):
        broker = Broker()
        v = Vehicle("north","ambulance", broker=broker, zone_id="Test")
        v.position = (0.0, 800.0)   # > 500m
        v.broadcast()
        self.assertEqual(broker.pending, 0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
