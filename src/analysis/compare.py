"""
Life-Link — Fixed-Timer vs Adaptive Comparison Analysis

Runs both simulation modes for a configurable duration and generates
Matplotlib comparison charts + summary CSV.

Usage:
    python -m src.analysis.compare
    python -m src.analysis.compare --duration 120 --vehicles 20
"""
from __future__ import annotations
import argparse
import csv
import os
import random
import time
from collections import defaultdict
from typing import Dict, List

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
import src.config as cfg
from src.comm.broker import Broker
from src.vehicle.vehicle import Vehicle
from src.controller.controller import IntersectionController


# ─── Fixed-Timer Baseline Simulator ──────────────────────────────────────────

class FixedTimerController:
    """
    Naive fixed-timer controller — alternates NS/EW every FIXED_GREEN_TIME seconds.
    No V2I, no occupancy logic, no emergency handling.
    """
    def __init__(self) -> None:
        self.state  = cfg.STATE_NS_GREEN
        self.timer  = 0.0
        self.phase  = "NS"

    def step(self, dt: float) -> None:
        self.timer += dt
        if self.state in (cfg.STATE_NS_GREEN, cfg.STATE_EW_GREEN):
            if self.timer >= cfg.FIXED_GREEN_TIME:
                # Jump to yellow
                self.state = cfg.STATE_NS_YELLOW if self.phase == "NS" else cfg.STATE_EW_YELLOW
                self.timer = 0.0
        elif self.state in (cfg.STATE_NS_YELLOW, cfg.STATE_EW_YELLOW):
            if self.timer >= cfg.YELLOW_DURATION:
                if self.phase == "NS":
                    self.phase = "EW"
                    self.state = cfg.STATE_EW_GREEN
                else:
                    self.phase = "NS"
                    self.state = cfg.STATE_NS_GREEN
                self.timer = 0.0

    def get_ns_ew(self):
        mapping = {
            cfg.STATE_NS_GREEN:  ("GREEN", "RED"),
            cfg.STATE_NS_YELLOW: ("YELLOW","RED"),
            cfg.STATE_EW_GREEN:  ("RED",   "GREEN"),
            cfg.STATE_EW_YELLOW: ("RED",   "YELLOW"),
        }
        return mapping.get(self.state, ("RED","RED"))


def _signal_for_lane(lane: str, ns: str, ew: str) -> str:
    if lane in ("north","south"):
        return ns
    return ew


def run_simulation(
    mode: str,
    duration: float,
    n_vehicles: int,
    seed: int = 42,
) -> Dict[str, float]:
    """
    Run one simulation pass and return average wait time per lane.

    Parameters
    ----------
    mode       : 'adaptive' or 'fixed'
    duration   : simulated seconds
    n_vehicles : initial vehicle count per lane
    seed       : random seed for reproducibility

    Returns
    -------
    dict: lane_id → average_wait_time (seconds)
    """
    random.seed(seed)
    broker = Broker()

    if mode == "adaptive":
        ctrl = IntersectionController(broker, zone_id="Analysis")
    else:
        ctrl = FixedTimerController()

    # Spawn vehicles
    vehicles: List[Vehicle] = []
    for lane in cfg.LANE_IDS:
        for _ in range(n_vehicles):
            vtype = random.choice(["car", "bike", "auto"])
            v = Vehicle(lane, vehicle_type=vtype, broker=broker, zone_id="Analysis")
            vehicles.append(v)

    # No ambulance in analysis runs (separate Priority Mode test)

    lane_waits: Dict[str, List[float]] = defaultdict(list)
    t = 0.0
    dt = cfg.DT

    while t < duration:
        # Broadcast V2I
        for v in vehicles:
            if v.active:
                v.broadcast()

        # Step controller
        if mode == "adaptive":
            ctrl.step(dt)
            sig_state = ctrl.get_signal_state()
            ns_sig = sig_state["NS"]
            ew_sig = sig_state["EW"]
        else:
            ctrl.step(dt)
            ns_sig, ew_sig = ctrl.get_ns_ew()

        # Update vehicles
        for v in vehicles:
            sig = _signal_for_lane(v.lane_id, ns_sig, ew_sig)
            v.update(dt, sig)

        # Respawn cleared vehicles
        for i, v in enumerate(vehicles):
            if not v.active:
                lane_waits[v.lane_id].append(v.wait_time)
                vtype = random.choice(["car","bike","auto"])
                vehicles[i] = Vehicle(v.lane_id, vehicle_type=vtype,
                                      broker=broker, zone_id="Analysis")

        t += dt

    # Collect remaining wait times
    for v in vehicles:
        if v.active:
            lane_waits[v.lane_id].append(v.wait_time)

    return {
        lane: (sum(waits) / len(waits) if waits else 0.0)
        for lane, waits in lane_waits.items()
    }


def generate_comparison_charts(
    duration: float = 60.0,
    n_vehicles: int = 5,
    out_dir: str = "output",
) -> None:
    """
    Run both modes, then produce Matplotlib comparison figures.

    Output files:
        output/wait_time_comparison.png
        output/efficiency_gain.png
        output/comparison_summary.csv
    """
    os.makedirs(out_dir, exist_ok=True)

    print(f"[Analysis] Running ADAPTIVE simulation ({duration:.0f}s) ...")
    adaptive_waits = run_simulation("adaptive", duration, n_vehicles)

    print(f"[Analysis] Running FIXED-TIMER simulation ({duration:.0f}s) ...")
    fixed_waits = run_simulation("fixed", duration, n_vehicles)

    lanes = sorted(set(list(adaptive_waits.keys()) + list(fixed_waits.keys())))
    adap_vals  = [adaptive_waits.get(l, 0) for l in lanes]
    fixed_vals = [fixed_waits.get(l, 0)   for l in lanes]

    # ── Chart 1: Side-by-side bar chart ──────────────────────────────────
    fig, ax = plt.subplots(figsize=(10, 6))
    x = np.arange(len(lanes))
    w = 0.35
    bars1 = ax.bar(x - w/2, fixed_vals,  w, label="Fixed Timer",    color="#E74C3C", alpha=0.85)
    bars2 = ax.bar(x + w/2, adap_vals,   w, label="Life-Link (Adaptive)", color="#27AE60", alpha=0.85)

    ax.set_xlabel("Lane", fontsize=12)
    ax.set_ylabel("Average Wait Time (seconds)", fontsize=12)
    ax.set_title("Life-Link vs Fixed-Timer: Average Wait Time per Lane", fontsize=14, fontweight="bold")
    ax.set_xticks(x)
    ax.set_xticklabels([l.capitalize() for l in lanes])
    ax.legend(fontsize=11)
    ax.grid(axis="y", alpha=0.3)

    # Annotate bars
    for bar in bars1:
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.3,
                f"{bar.get_height():.1f}s", ha="center", va="bottom", fontsize=9, color="#E74C3C")
    for bar in bars2:
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.3,
                f"{bar.get_height():.1f}s", ha="center", va="bottom", fontsize=9, color="#27AE60")

    plt.tight_layout()
    chart1_path = os.path.join(out_dir, "wait_time_comparison.png")
    plt.savefig(chart1_path, dpi=150)
    plt.close()
    print(f"[Analysis] Chart saved → {chart1_path}")

    # ── Chart 2: Efficiency gain % ────────────────────────────────────────
    gains = []
    for l in lanes:
        fv = fixed_waits.get(l, 0)
        av = adaptive_waits.get(l, 0)
        gain = ((fv - av) / fv * 100) if fv > 0 else 0
        gains.append(gain)

    fig, ax = plt.subplots(figsize=(8, 5))
    colors = ["#27AE60" if g >= 0 else "#E74C3C" for g in gains]
    bars = ax.bar([l.capitalize() for l in lanes], gains, color=colors, alpha=0.85)
    ax.axhline(0, color="black", linewidth=0.8)
    ax.set_xlabel("Lane", fontsize=12)
    ax.set_ylabel("Wait Time Reduction (%)", fontsize=12)
    ax.set_title("Life-Link Efficiency Gain over Fixed-Timer", fontsize=14, fontweight="bold")
    ax.grid(axis="y", alpha=0.3)

    for bar, g in zip(bars, gains):
        ax.text(bar.get_x() + bar.get_width()/2,
                bar.get_height() + (0.5 if g >= 0 else -1.5),
                f"{g:.1f}%", ha="center", va="bottom", fontsize=10, fontweight="bold")

    plt.tight_layout()
    chart2_path = os.path.join(out_dir, "efficiency_gain.png")
    plt.savefig(chart2_path, dpi=150)
    plt.close()
    print(f"[Analysis] Chart saved → {chart2_path}")

    # ── CSV Summary ───────────────────────────────────────────────────────
    csv_path = os.path.join(out_dir, "comparison_summary.csv")
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["lane","fixed_wait","adaptive_wait","gain_pct"])
        writer.writeheader()
        for l, fv, av, g in zip(lanes, fixed_vals, adap_vals, gains):
            writer.writerow({"lane": l, "fixed_wait": round(fv,2),
                             "adaptive_wait": round(av,2), "gain_pct": round(g,1)})
    print(f"[Analysis] CSV saved → {csv_path}")

    total_fixed = sum(fixed_vals)
    total_adap  = sum(adap_vals)
    overall_gain = (total_fixed - total_adap) / total_fixed * 100 if total_fixed > 0 else 0
    print(f"\n{'='*50}")
    print(f"  Overall average wait — Fixed:    {total_fixed/len(lanes):.1f}s")
    print(f"  Overall average wait — Adaptive: {total_adap/len(lanes):.1f}s")
    print(f"  Life-Link efficiency gain:       {overall_gain:.1f}%")
    print(f"{'='*50}\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Life-Link Analysis")
    parser.add_argument("--duration",  type=float, default=60.0,  help="Simulation seconds")
    parser.add_argument("--vehicles",  type=int,   default=5,     help="Vehicles per lane")
    parser.add_argument("--out",       type=str,   default="output")
    args = parser.parse_args()
    generate_comparison_charts(args.duration, args.vehicles, args.out)
