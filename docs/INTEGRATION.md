# Life-Link Integration Guide

### Step 1 — Import and instantiate

```python
from src.comm.broker import Broker
from src.vehicle.vehicle import Vehicle
from src.controller.controller import IntersectionController
from src.logging.logger import CSVLogger
import src.config as cfg

broker = Broker()
logger = CSVLogger("logs/vehicle_log.csv")
ctrl   = IntersectionController(broker, zone_id="Alpha", logger=logger)
```

### Step 2 — Create vehicles

```python
vehicles = []
for lane in cfg.LANE_IDS:
    for _ in range(3):
        v = Vehicle(lane, vehicle_type="car", broker=broker, zone_id="Alpha")
        vehicles.append(v)

# Spawn ambulance
amb = Vehicle("north", vehicle_type="ambulance", broker=broker, zone_id="Alpha")
vehicles.append(amb)
```

### Step 3 — Simulation loop

```python
import time
dt = cfg.DT   # 0.05 s
while True:
    # 1. Vehicles broadcast V2I packets
    for v in vehicles:
        v.broadcast()

    # 2. Controller consumes packets + steps FSM
    ctrl.step(dt)

    # 3. Read signal state for UI / vehicle physics
    sig = ctrl.get_signal_state()
    ns_sig = sig["NS"]    # "GREEN" / "YELLOW" / "RED"
    ew_sig = sig["EW"]

    # 4. Update vehicle physics with correct signal
    for v in vehicles:
        lane_signal = ns_sig if v.lane_id in ("north","south") else ew_sig
        v.update(dt, lane_signal)

    time.sleep(dt)
```

### Step 4 — Run Pygame UI with both modules (threaded)

```python
import threading
from src.ui.pygame_ui import run_simulation
thread = threading.Thread(target=run_simulation, daemon=True)
thread.start()
```

