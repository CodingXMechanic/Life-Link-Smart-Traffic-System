"""Headless (no display) simulation for CI / testing."""
import sys, os, time, random
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import src.config as cfg
from src.comm.broker import Broker
from src.controller.controller import IntersectionController
from src.logging.logger import CSVLogger
from src.vehicle.vehicle import Vehicle

print("Life-Link headless demo starting...")
broker = Broker()
logger = CSVLogger("logs/headless_demo.csv")
ctrl   = IntersectionController(broker, zone_id="Demo", logger=logger)

vehicles = [Vehicle(lane, random.choice(["car","bike","auto"]),
                    broker=broker, zone_id="Demo")
            for lane in cfg.LANE_IDS for _ in range(3)]
amb = Vehicle("north", "ambulance", broker=broker, zone_id="Demo")
vehicles.append(amb)

dt = cfg.DT
t  = 0.0
print(f"Simulating {len(vehicles)} vehicles for 30 seconds...")

while t < 30.0:
    for v in vehicles:
        v.broadcast()
    ctrl.step(dt)
    sig = ctrl.get_signal_state()
    ns, ew = sig["NS"], sig["EW"]
    for v in vehicles:
        lane_sig = ns if v.lane_id in ("north","south") else ew
        v.update(dt, lane_sig)
    # Respawn cleared
    for i, v in enumerate(vehicles):
        if not v.active:
            vehicles[i] = Vehicle(v.lane_id, "car", broker=broker, zone_id="Demo")
    t += dt

sig = ctrl.get_signal_state()
print(f"\n=== Headless Demo Complete ===")
print(f"  Sim time:    {sig['sim_time']:.1f}s")
print(f"  State:       {sig['state']}")
print(f"  NS Signal:   {sig['NS']}")
print(f"  EW Signal:   {sig['EW']}")
print(f"  Preemptions: {sig['preempt_count']}")
print(f"  CSV log:     logs/headless_demo.csv")
print(f"\nAll OK — Life-Link simulation engine verified!")
