# Life-Link — Viva Presentation Guide

## What you will demonstrate (step by step)

### WINDOW 1 — Pygame simulation (full screen, projected)
### WINDOW 2 — Live HTML dashboard (browser)
### WINDOW 3 — Real-time analysis chart (matplotlib, auto-updating)

Run all three with ONE command:
    python present.py

---

## Full Setup (Day of Viva)

1. Install once:
   pip install pygame numpy matplotlib

2. Run the presentation launcher:
   python present.py

3. Keys during demo:
   E  — spawn ambulance (shows Priority Mode live)
   1-4 — switch between 4 zones
   A  — trigger analysis comparison (shows graphs)
   +/- — speed up/slow down
   SPACE — pause to explain a concept
   Q   — quit

---

## What each examiner will see

### Simulation Panel (left side of Pygame)
- 4 intersections running simultaneously (Alpha, Beta, Gamma, Delta)
- Cars, bikes, autos, ambulances moving in real time
- Traffic signals changing adaptively based on vehicle density
- Ambulance triggers emergency preemption with red corridor glow
- 3-second mandatory yellow before green corridor opens

### Live Calculations Panel (right side of Pygame)
- ETA of each vehicle (calculated live using d = v²+2a·d quadratic)
- Adaptive green time per zone (changes as vehicles enter/leave)
- Phase timer counting down
- Preemption count
- Average wait time per lane

### Analysis Chart Window (separate matplotlib window)
- Fixed-timer vs Adaptive bar chart updating live
- Efficiency gain % per lane
- Cumulative wait time curves

### HTML Dashboard (browser tab)
- All 4 zones, signal states, emergency alerts
- Lane occupancy heatmap
- Event log

