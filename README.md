# 🚦 Life-Link Smart Traffic System

A Smart City traffic management simulation using V2I Communication, Adaptive Signal Optimization, Emergency Vehicle Priority, and Real-Time Monitoring Dashboard.

## Features
- 🚑 Emergency Vehicle Green Corridor
- 🚦 Adaptive Traffic Signals
- 🌆 Multi-Zone Smart City Simulation
- 📡 V2I Packet Communication
- 📊 Real-Time Dashboard
- 🎮 Pygame Visualization
- 📈 Traffic Analytics & Comparison

## Tech Stack
Python, Pygame, NumPy, Matplotlib, HTML, CSS, JavaScript
## Overview

Life-Link is a complete Python simulation of a **V2I (Vehicle-to-Infrastructure)** smart traffic system featuring:

- **Optimization Mode** — Adaptive green-light timing based on real-time lane occupancy (eliminates static latency)
- **Priority Mode** — Instant emergency preemption with mandatory 3-second yellow clearance and green corridor
- **4-Zone Smart City** — Alpha, Beta, Gamma, Delta intersections running simultaneously
- **Realistic Pygame visualization** — car/bike/auto/ambulance sprites, proper road markings, signal heads
- **Live JSON dashboard** export for HTML monitoring
- **Full pytest test suite** — physics, packet, controller, and preemption tests
- **Matplotlib analysis** — fixed-timer vs adaptive wait-time comparison charts

---

## Repository Structure

```
life-link/
├── src/
│   ├── config.py                  # All constants (DETECTION_RADIUS=500, YELLOW=3s, etc.)
│   ├── vehicle/
│   │   ├── vehicle.py             # Vehicle class: UUID, kinematics, V2I packets, broadcast
│   │   └── physics.py             # Kinematic update, braking distance, ETA, detection zone
│   ├── comm/
│   │   └── broker.py              # Thread-safe V2I message broker (publish/get_packets)
│   ├── controller/
│   │   ├── controller.py          # IntersectionController: FSM, Optimization + Priority modes
│   │   └── failsafe.py            # Conflict monitor, illegal-transition rejection
│   ├── logging/
│   │   └── logger.py              # CSV logger: wait times, preemption events
│   ├── ui/
│   │   └── pygame_ui.py           # Full Pygame 2D simulation with HUD
│   └── analysis/
│       └── compare.py             # Fixed-timer vs adaptive comparison + charts
├── tests/
│   ├── test_physics.py            # Kinematic unit tests (Saumya)
│   ├── test_packet.py             # V2I packet + broker tests (Saumya)
│   ├── test_controller.py         # FSM + fail-safe tests (Pulkit)
│   └── test_preemption.py         # Integration + recovery tests (Pulkit)
├── notebooks/
│   └── vehicle_demo.ipynb         # Jupyter demo notebook
├── scripts/
│   ├── run_demo_headless.py       # Headless 30s simulation (no display needed)
│   ├── run_analysis.py            # Full adaptive vs fixed comparison
│   ├── generate_charts.py         # All Matplotlib charts
│   └── generate_sample_csv.py     # Sample CSV log generator
├── assets/diagrams/
│   ├── architecture_mermaid.mmd   # System architecture diagram
│   └── state_machine_mermaid.mmd  # Controller FSM diagram
├── docs/
│   ├── API.md                     # Full API specification (Saumya → Pulkit interface)
│   └── INTEGRATION.md             # Integration guide + code snippets
├── output/                        # Generated charts and live JSON
├── logs/                          # CSV event logs
├── requirements.txt
└── README.md
```

---

## Quick Start

### 1. Install Dependencies

```bash
pip install -r requirements.txt
```

### 2. Run Tests (verify everything works)

```bash
pytest tests/ -v
```

Expected: **all tests pass**

### 3. Run Headless Demo (no display required)

```bash
python scripts/run_demo_headless.py
```

### 4. Generate Charts

```bash
python scripts/generate_charts.py
# Charts saved in output/
```

### 5. Run Analysis (Fixed-Timer vs Adaptive)

```bash
python scripts/run_analysis.py
```

### 6. Launch Full Pygame Simulation

```bash
python -m src.ui.pygame_ui
```

#### Pygame Controls

| Key | Action |
|-----|--------|
| `1` `2` `3` `4` | Select zone (Alpha / Beta / Gamma / Delta) |
| `E` | Spawn ambulance in selected zone |
| `+` / `-` | Speed up / slow down simulation |
| `SPACE` | Pause / Resume |
| `Q` / `ESC` | Quit and save logs |

### 7. Jupyter Notebook Demo

```bash
jupyter notebook notebooks/vehicle_demo.ipynb
```

---

## System Design

### V2I Packet Schema

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

### Key Physics Formulas

| Formula | Usage |
|---------|-------|
| `v = u + at` | Velocity update each tick |
| `s = ut + ½at²` | Displacement per tick |
| `d = v²/(2a)` | Braking distance |
| `d = √(x²+y²)` | Euclidean distance to intersection (geofence) |
| `t = (-v ± √(v²+2ad)) / a` | ETA quadratic solution |

### State Machine Transitions

```
NS_GREEN → NS_YELLOW (min green elapsed OR preemption)
NS_YELLOW → EW_GREEN (3s yellow complete, normal)
NS_YELLOW → ALL_RED  (3s yellow complete, preemption path)
ALL_RED   → EMERGENCY (ambulance lane locked green)
EMERGENCY → ALL_RED   (ambulance clears intersection)
ALL_RED   → RECOVERY  (longest-red lane gets extended green)
RECOVERY  → NS_GREEN | EW_GREEN
```

---

## CSV Log Format

```
timestamp,vehicle_id,zone_id,lane_id,wait_time,event
1700000001.234,VH_A1B2,Alpha,north,0.0,tick
1700000002.100,SYSTEM,Alpha,north,8.3,preemption_triggered:north
1700000015.400,AMB_001,Alpha,north,0.0,emergency_detected
1700000021.700,SYSTEM,Alpha,,0.0,emergency_cleared
1700000021.750,SYSTEM,Alpha,east,0.0,recovery_start:east
```

## Configuration (src/config.py)

All constants are centralized:

| Constant | Default | Description |
|----------|---------|-------------|
| `DETECTION_RADIUS` | 500.0 m | V2I geofence radius |
| `YELLOW_DURATION` | 3.0 s | Mandatory yellow clearance |
| `MIN_GREEN_TIME` | 5.0 s | Minimum adaptive green |
| `MAX_GREEN_TIME` | 60.0 s | Maximum adaptive green |
| `RECOVERY_GREEN` | 15.0 s | Post-emergency extended green |
| `FIXED_GREEN_TIME` | 30.0 s | Baseline fixed-timer (comparison) |
| `BRAKING_DECEL` | 5.0 m/s² | Comfortable braking decel |
| `DT` | 0.05 s | Simulation time-step |

---

*Life-Link — "A connected city is a better city for everyone."*