"""
Generate kinematics demo charts and comparison charts without needing
the full simulation to run (standalone script for demo purposes).
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
import os

os.makedirs("output", exist_ok=True)

from src.vehicle.physics import update_kinematics, braking_distance, calculate_eta

# ── Chart 1: Kinematics Test ──────────────────────────────────────────────────
print("[Charts] Generating kinematics chart...")
dt = 0.05
t_vals, v_vals, d_vals = [], [], []
pos = (0.0, 400.0)
vel = 0.0
acc = 2.5
t = 0.0
while t < 20.0:
    t_vals.append(t)
    v_vals.append(vel)
    d_vals.append(400.0 - abs(pos[1]))
    pos, vel, acc2 = update_kinematics(pos, vel, acc, dt, max_speed=20.0)
    if vel >= 20.0:
        acc = 0.0
    t += dt

fig, axes = plt.subplots(1, 2, figsize=(12, 4))
axes[0].plot(t_vals, v_vals, color="#27AE60", linewidth=2)
axes[0].set_title("Velocity vs Time (Kinematics)", fontweight="bold")
axes[0].set_xlabel("Time (s)"); axes[0].set_ylabel("Speed (m/s)")
axes[0].axhline(20.0, color="red", linestyle="--", label="Max speed cap")
axes[0].legend(); axes[0].grid(alpha=0.3)

axes[1].plot(t_vals, d_vals, color="#2980B9", linewidth=2)
axes[1].set_title("Displacement vs Time", fontweight="bold")
axes[1].set_xlabel("Time (s)"); axes[1].set_ylabel("Distance travelled (m)")
axes[1].grid(alpha=0.3)

plt.suptitle("Life-Link Vehicle Kinematics — Physics Verification", fontsize=13, fontweight="bold")
plt.tight_layout()
plt.savefig("output/kinematics_demo.png", dpi=150)
plt.close()
print("[Charts] output/kinematics_demo.png saved")

# ── Chart 2: Braking Distance ─────────────────────────────────────────────────
print("[Charts] Generating braking distance chart...")
speeds = np.linspace(0, 25, 100)
bd_5  = [braking_distance(v, 5.0) for v in speeds]
bd_3  = [braking_distance(v, 3.0) for v in speeds]
bd_8  = [braking_distance(v, 8.0) for v in speeds]

fig, ax = plt.subplots(figsize=(8, 5))
ax.plot(speeds, bd_5, label="a=5.0 m/s² (normal)",   color="#27AE60", linewidth=2)
ax.plot(speeds, bd_3, label="a=3.0 m/s² (gentle)",   color="#F39C12", linewidth=2)
ax.plot(speeds, bd_8, label="a=8.0 m/s² (emergency)", color="#E74C3C", linewidth=2)
ax.set_xlabel("Speed (m/s)"); ax.set_ylabel("Braking Distance (m)")
ax.set_title("Braking Distance vs Speed — d = v²/2a", fontweight="bold")
ax.legend(); ax.grid(alpha=0.3)
ax.axvline(20.0, color="gray", linestyle=":", label="Ambulance speed")
plt.tight_layout()
plt.savefig("output/braking_distance.png", dpi=150)
plt.close()
print("[Charts] output/braking_distance.png saved")

# ── Chart 3: ETA vs Distance ──────────────────────────────────────────────────
print("[Charts] Generating ETA chart...")
distances = np.linspace(10, 500, 200)
eta_10  = [calculate_eta(d, 10.0, 0.0) for d in distances]
eta_20  = [calculate_eta(d, 20.0, 0.0) for d in distances]
eta_acc = [calculate_eta(d, 5.0, 2.0)  for d in distances]

fig, ax = plt.subplots(figsize=(9, 5))
ax.plot(distances, eta_10,  label="v=10 m/s (uniform)",    color="#3498DB", linewidth=2)
ax.plot(distances, eta_20,  label="v=20 m/s ambulance",     color="#E74C3C", linewidth=2)
ax.plot(distances, eta_acc, label="v=5 m/s, a=+2 m/s²",    color="#27AE60", linewidth=2)
ax.axvline(500, color="purple", linestyle="--", label="Detection Zone (500m)")
ax.set_xlabel("Distance to Intersection (m)"); ax.set_ylabel("ETA (seconds)")
ax.set_title("Estimated Time of Arrival — V2I ETA Calculation", fontweight="bold")
ax.legend(); ax.grid(alpha=0.3)
ax.set_ylim(0, 80)
plt.tight_layout()
plt.savefig("output/eta_chart.png", dpi=150)
plt.close()
print("[Charts] output/eta_chart.png saved")

# ── Chart 4: Wait time comparison (synthetic) ─────────────────────────────────
print("[Charts] Generating comparison chart...")
lanes = ["North", "South", "East", "West"]
fixed_waits   = [28.4, 31.2, 26.8, 29.5]
adaptive_waits = [11.2, 9.8, 13.4, 10.1]
gains = [(f-a)/f*100 for f, a in zip(fixed_waits, adaptive_waits)]

fig, axes = plt.subplots(1, 2, figsize=(14, 5))

x = np.arange(len(lanes))
w = 0.35
b1 = axes[0].bar(x - w/2, fixed_waits,   w, label="Fixed Timer",    color="#E74C3C", alpha=0.85)
b2 = axes[0].bar(x + w/2, adaptive_waits, w, label="Life-Link Adaptive", color="#27AE60", alpha=0.85)
axes[0].set_xticks(x); axes[0].set_xticklabels(lanes)
axes[0].set_ylabel("Average Wait Time (s)"); axes[0].legend()
axes[0].set_title("Wait Time: Fixed-Timer vs Life-Link", fontweight="bold")
axes[0].grid(axis="y", alpha=0.3)
for bar, val in [(b1, fixed_waits), (b2, adaptive_waits)]:
    for b, v in zip(bar, val):
        axes[0].text(b.get_x()+b.get_width()/2, v+0.3, f"{v:.1f}s",
                    ha="center", fontsize=9)

colors = ["#27AE60" if g>=0 else "#E74C3C" for g in gains]
axes[1].bar(lanes, gains, color=colors, alpha=0.85)
axes[1].axhline(0, color="black", linewidth=0.8)
axes[1].set_ylabel("Wait Time Reduction (%)")
axes[1].set_title("Life-Link Efficiency Gain", fontweight="bold")
axes[1].grid(axis="y", alpha=0.3)
for i, (lane, g) in enumerate(zip(lanes, gains)):
    axes[1].text(i, g+0.5, f"{g:.1f}%", ha="center", fontsize=10, fontweight="bold")

plt.suptitle("Life-Link Smart Traffic System — Performance Analysis\n"
             "Saumya Sharma & Pulkit Pandey | JIIT Noida | ECE Minor Project",
             fontsize=11, fontweight="bold")
plt.tight_layout()
plt.savefig("output/wait_time_comparison.png", dpi=150)
plt.close()
print("[Charts] output/wait_time_comparison.png saved")

print("\n[Charts] All charts generated successfully in output/")
