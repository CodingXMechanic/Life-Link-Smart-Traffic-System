# Life-Link API Reference

## Broker API (src/comm/broker.py)

```python
from src.comm.broker import Broker
broker = Broker(maxsize=0)
```

### Methods

| Method | Signature | Returns | Description |
|--------|-----------|---------|-------------|
| `publish` | `publish(packet: dict) -> None` | None | Enqueue V2I packet. Raises `ValueError` if required keys missing. |
| `get_packets` | `get_packets() -> List[dict]` | List[dict] | Atomically drain and return all queued packets. |

### Required packet keys
`vehicle_id`, `timestamp`, `lane_id`, `priority_flag`

---

## Vehicle API (src/vehicle/vehicle.py)

```python
from src.vehicle.vehicle import Vehicle
v = Vehicle(lane_id="north", vehicle_type="car", broker=broker, zone_id="Alpha")
```

### Constructor Parameters
| Param | Type | Description |
|-------|------|-------------|
| `lane_id` | str | `"north"` / `"south"` / `"east"` / `"west"` |
| `vehicle_type` | str | `"car"` / `"bike"` / `"auto"` / `"ambulance"` |
| `broker` | Broker | Broker instance (optional — None disables broadcasting) |
| `zone_id` | str | Zone identifier |

### Methods

| Method | Signature | Description |
|--------|-----------|-------------|
| `update` | `update(dt: float, signal_state: str = "GREEN") -> None` | Advance physics one tick. `signal_state` is "GREEN"/"YELLOW"/"RED" |
| `to_packet` | `to_packet() -> dict` | Build and return V2I JSON packet dict |
| `broadcast` | `broadcast() -> None` | Publish packet to broker if within Detection Zone |

### Packet Schema
```json
{
  "vehicle_id": "VH_A1B2C3D4",
  "timestamp": 1700000000.123,
  "zone_id": "Alpha",
  "lane_id": "north",
  "vehicle_type": "car",
  "position": {"x": 0.0, "y": 340.2},
  "velocity": 14.2,
  "acceleration": 0.5,
  "priority_flag": 0,
  "distance_to_intersection": 340.2,
  "eta": 23.96,
  "wait_time": 0.0,
  "active": true
}
```

---

## IntersectionController API (src/controller/controller.py)

```python
from src.controller.controller import IntersectionController
ctrl = IntersectionController(broker=broker, zone_id="Alpha", logger=logger)
```

### Methods

| Method | Signature | Description |
|--------|-----------|-------------|
| `step` | `step(dt: float) -> None` | Advance controller one tick. Consumes packets, updates FSM. |
| `get_signal_state` | `get_signal_state() -> dict` | Returns current signal state dict (see below) |
| `log_state` | `log_state() -> None` | Appends current state event to CSV logger |

### Signal State Dict
```json
{
  "state": "NS_GREEN",
  "NS": "GREEN",
  "EW": "RED",
  "emergency_lane": null,
  "zone_id": "Alpha",
  "phase_timer": 4.35,
  "mode": "OPTIMIZATION",
  "adaptive_green": 22.5,
  "lane_vehicles": {"north": 2, "south": 1, "east": 0, "west": 3},
  "preempt_count": 0,
  "sim_time": 45.2
}
```

---

## Physics Utilities (src/vehicle/physics.py)

| Function | Signature | Returns |
|----------|-----------|---------|
| `update_kinematics` | `(pos, velocity, acceleration, dt, max_speed) -> (pos, vel, acc)` | Updated physics state |
| `braking_distance` | `(velocity, deceleration=5.0) -> float` | Min stopping distance (m) |
| `calculate_eta` | `(distance, velocity, acceleration=0.0) -> float` | ETA in seconds (inf if unreachable) |
| `euclidean_distance` | `(pos, center=(0,0)) -> float` | Distance to intersection (m) |
| `in_detection_zone` | `(pos, radius=500.0, center=(0,0)) -> bool` | True if within geofence |
