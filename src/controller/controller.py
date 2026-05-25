"""
Life-Link — Intersection Controller (Pulkit's module)

Full state-machine controller for one intersection:
  • Optimization Mode  — adaptive green times via lane occupancy ratios
  • Priority Mode      — emergency preemption with mandatory 3-second yellow
  • Post-Emergency Recovery — longest-red-lane extended green
  • Fail-Safe integration via FailSafeMonitor
  • Logging via CSVLogger

State machine:
    NS_GREEN → NS_YELLOW → EW_GREEN → EW_YELLOW → NS_GREEN  (normal cycle)
    Any → ALL_RED (mandatory clearance) → EMERGENCY → ALL_RED → RECOVERY → NS/EW_GREEN
"""
from __future__ import annotations
import time
from collections import defaultdict
from typing import Dict, List, Optional, Tuple

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
import src.config as cfg
from src.controller.failsafe import FailSafeMonitor
from src.vehicle.physics import calculate_eta, euclidean_distance

try:
    from src.logging.logger import CSVLogger
    _has_logger = True
except ImportError:
    _has_logger = False


class IntersectionController:
    """
    Controls signal phases for a single 4-way intersection zone.

    Parameters
    ----------
    broker    : Broker instance (must expose get_packets())
    zone_id   : zone name string ('Alpha', 'Beta', etc.)
    logger    : optional CSVLogger — if None, logging is skipped
    """

    def __init__(
        self,
        broker,
        zone_id: str = "Alpha",
        logger=None,
    ) -> None:
        self.broker   = broker
        self.zone_id  = zone_id
        self.logger   = logger
        self.failsafe = FailSafeMonitor()

        # ── State Machine ──────────────────────────────────────────────────
        self.state:         str   = cfg.STATE_NS_GREEN
        self.phase_timer:   float = 0.0          # seconds in current phase
        self.yellow_timer:  float = 0.0          # tracks yellow clearance
        self.all_red_timer: float = 0.0

        # ── Lane Occupancy ─────────────────────────────────────────────────
        # lane_id → list of latest packets from vehicles in that lane
        self.lane_packets: Dict[str, List[dict]] = defaultdict(list)

        # ── Red-wait tracking (for recovery) ──────────────────────────────
        self.lane_red_timer: Dict[str, float] = {l: 0.0 for l in cfg.LANE_IDS}

        # ── Emergency tracking ────────────────────────────────────────────
        self.emergency_lane:    Optional[str]  = None
        self.emergency_eta:     Optional[float] = None
        self.emergency_cleared: bool = False
        self.emergency_vehicle_id: Optional[str] = None
        self._emergency_last_seen_t: float = -1.0  # sim_time seconds

        # ── Recovery tracking ─────────────────────────────────────────────
        self.recovery_lane:  Optional[str] = None
        self.recovery_timer: float = 0.0

        # ── Adaptive green target ─────────────────────────────────────────
        self._adaptive_green: float = cfg.MIN_GREEN_TIME

        # ── Statistics ────────────────────────────────────────────────────
        self.cycle_count:   int   = 0
        self.preempt_count: int   = 0
        self.sim_time:      float = 0.0

    # ══ Public API (for Pygame UI & integration) ═══════════════════════════

    def step(self, dt: float) -> None:
        """
        Advance the controller by one simulation tick.

        1. Consume all pending V2I packets from broker.
        2. Scan for emergency vehicles → trigger preemption if detected.
        3. Run adaptive logic (Optimization Mode) or emergency FSM.
        4. Advance phase timers.

        Parameters
        ----------
        dt : simulation time-step in seconds
        """
        self.sim_time += dt

        # 1. Consume packets
        packets = self.broker.get_packets()
        self._ingest_packets(packets, dt)

        # 2. Check for emergency
        emerg_packet = self._find_emergency_packet()

        # 2.5 Emergency preemption should be responsive:
        # If an emergency packet is present, we must begin/continue the
        # preemption sequence immediately from *any* non-emergency state.
        # This guarantees a corridor within ~4–5 seconds (3s yellow + 1s all-red).
        if emerg_packet and self.state != cfg.STATE_EMERGENCY:
            self._trigger_preemption(emerg_packet)

        # 3. State machine dispatch
        if self.state == cfg.STATE_EMERGENCY:
            self._run_emergency_state(dt)
        elif self.state == cfg.STATE_ALL_RED:
            self._run_all_red(dt)
        elif self.state == cfg.STATE_RECOVERY:
            self._run_recovery(dt)
        else:
            # Normal cycle states (including yellow) MUST keep progressing even
            # while emergency packets are present; otherwise the controller can
            # freeze in yellow forever.
            self._run_normal_cycle(dt)

        # 4. Update red-wait timers
        self._update_red_timers(dt)

    def get_signal_state(self) -> dict:
        """
        Return the current signal state for UI rendering.

        Returns
        -------
        dict with keys:
            'state'    : current FSM state string
            'NS'       : 'GREEN' | 'YELLOW' | 'RED'
            'EW'       : 'GREEN' | 'YELLOW' | 'RED'
            'emergency_lane' : lane_id or None
            'zone_id'  : this controller's zone
            'phase_timer' : seconds in current phase
            'mode'     : 'PRIORITY' | 'OPTIMIZATION'
        """
        ns, ew = self._get_ns_ew_signals()
        return {
            "state":          self.state,
            "NS":             ns,
            "EW":             ew,
            "emergency_lane": self.emergency_lane,
            "zone_id":        self.zone_id,
            "phase_timer":    round(self.phase_timer, 2),
            "mode":           "PRIORITY" if self.state == cfg.STATE_EMERGENCY else "OPTIMIZATION",
            "adaptive_green": round(self._adaptive_green, 1),
            "lane_vehicles":  {l: len(v) for l, v in self.lane_packets.items()},
            "preempt_count":  self.preempt_count,
            "sim_time":       round(self.sim_time, 1),
        }

    def log_state(self) -> None:
        """Append current controller state event to CSV logger (if available)."""
        if self.logger:
            self.logger.log_event(
                event=f"state:{self.state}",
                zone_id=self.zone_id,
                lane_id=self.emergency_lane or "",
                vehicle_id="CTRL",
                wait_time=self.phase_timer,
            )

    # ══ Internal helpers ════════════════════════════════════════════════════

    def _ingest_packets(self, packets: List[dict], dt: float) -> None:
        """Process incoming V2I packets: update lane occupancy."""
        # Clear stale lane data each tick
        for lane in cfg.LANE_IDS:
            self.lane_packets[lane] = []

        for p in packets:
            lane = p.get("lane_id")
            if lane in cfg.LANE_IDS and p.get("active", True):
                self.lane_packets[lane].append(p)
            # Track last-seen timestamp for the currently served emergency vehicle
            if (
                self.emergency_vehicle_id
                and p.get("vehicle_id") == self.emergency_vehicle_id
                and p.get("priority_flag", 0) == cfg.PRIORITY_EMERGENCY
            ):
                self._emergency_last_seen_t = self.sim_time

        # Log each emergency packet
        if self.logger:
            for p in packets:
                if p.get("priority_flag", 0) == cfg.PRIORITY_EMERGENCY:
                    self.logger.log_from_packet(p, "emergency_detected")

    def _find_emergency_packet(self) -> Optional[dict]:
        """Return the highest-priority (closest ETA) emergency packet, or None."""
        best: Optional[dict] = None
        best_eta = float('inf')
        for lane_pkts in self.lane_packets.values():
            for p in lane_pkts:
                if p.get("priority_flag", 0) == cfg.PRIORITY_EMERGENCY:
                    eta = p.get("eta") or float('inf')
                    if eta < best_eta:
                        best_eta = eta
                        best = p
        return best

    def _trigger_preemption(self, packet: dict) -> None:
        """
        Initiate Priority Mode preemption sequence.

        Sequence:
          current phase → NS/EW_YELLOW (3 s) → ALL_RED (1 s) → EMERGENCY
        """
        # Ignore obviously invalid packets
        if not packet or packet.get("lane_id") not in cfg.LANE_IDS:
            return

        # If we are already serving the same emergency vehicle, don't retrigger.
        if (
            self.emergency_vehicle_id
            and packet.get("vehicle_id") == self.emergency_vehicle_id
            and self.state in (cfg.STATE_EMERGENCY, cfg.STATE_ALL_RED, cfg.STATE_NS_YELLOW, cfg.STATE_EW_YELLOW)
        ):
            return

        self.emergency_lane = packet["lane_id"]
        self.emergency_eta  = packet.get("eta")
        self.emergency_vehicle_id = packet.get("vehicle_id")
        self._emergency_last_seen_t = self.sim_time
        # Count a new preemption only when we are entering the preemption
        # sequence from normal operation (not while already in yellow/all-red).
        if self.state not in (cfg.STATE_NS_YELLOW, cfg.STATE_EW_YELLOW, cfg.STATE_ALL_RED, cfg.STATE_EMERGENCY):
            self.preempt_count += 1

        # Force yellow on conflicting directions before emergency green.
        # If we're already in YELLOW or ALL_RED, just latch emergency_lane and
        # let the normal FSM bridge into ALL_RED/EMERGENCY.
        if self.state == cfg.STATE_NS_GREEN:
            self._transition(cfg.STATE_NS_YELLOW)
        elif self.state == cfg.STATE_EW_GREEN:
            self._transition(cfg.STATE_EW_YELLOW)
        elif self.state == cfg.STATE_RECOVERY:
            # Recovery behaves like a "green" for the recovery axis; bridge through yellow.
            if self.recovery_lane in ("north", "south"):
                self._transition(cfg.STATE_NS_YELLOW)
            elif self.recovery_lane in ("east", "west"):
                self._transition(cfg.STATE_EW_YELLOW)
        # Yellow → ALL_RED → EMERGENCY handled in _run_normal_cycle yellow branch

        if self.logger:
            self.logger.log_event(
                f"preemption_triggered:{self.emergency_lane}",
                zone_id=self.zone_id,
                lane_id=self.emergency_lane or "",
            )

    def _run_normal_cycle(self, dt: float) -> None:
        """Optimization Mode: adaptive green timing and phase rotation."""
        self.phase_timer += dt

        if self.state == cfg.STATE_NS_GREEN:
            self._adaptive_green = self._compute_adaptive_green("north", "south")
            if self.phase_timer >= self._adaptive_green:
                self._transition(cfg.STATE_NS_YELLOW)

        elif self.state == cfg.STATE_NS_YELLOW:
            self.yellow_timer += dt
            if self.yellow_timer >= cfg.YELLOW_DURATION:
                self.yellow_timer = 0.0
                # Check if preemption was triggered
                if self.emergency_lane in ("east", "west"):
                    self._transition(cfg.STATE_ALL_RED)
                else:
                    self._transition(cfg.STATE_EW_GREEN)
                self.cycle_count += 1

        elif self.state == cfg.STATE_EW_GREEN:
            self._adaptive_green = self._compute_adaptive_green("east", "west")
            if self.phase_timer >= self._adaptive_green:
                self._transition(cfg.STATE_EW_YELLOW)

        elif self.state == cfg.STATE_EW_YELLOW:
            self.yellow_timer += dt
            if self.yellow_timer >= cfg.YELLOW_DURATION:
                self.yellow_timer = 0.0
                if self.emergency_lane in ("north", "south"):
                    self._transition(cfg.STATE_ALL_RED)
                else:
                    self._transition(cfg.STATE_NS_GREEN)
                self.cycle_count += 1

    def _compute_adaptive_green(self, lane_a: str, lane_b: str) -> float:
        """
        Compute adaptive green duration based on lane occupancy ratio.

        Logic:
          • If both lanes empty → MIN_GREEN_TIME (early cut-off)
          • Otherwise → lerp between MIN and MAX based on vehicle count ratio
        """
        count_a = len(self.lane_packets.get(lane_a, []))
        count_b = len(self.lane_packets.get(lane_b, []))
        total   = count_a + count_b

        if total == 0:
            return cfg.MIN_GREEN_TIME   # early cut-off: no demand

        # Ratio of active vehicles vs plausible maximum (10 vehicles = max green)
        ratio = min(total / 10.0, 1.0)
        return cfg.MIN_GREEN_TIME + ratio * (cfg.MAX_GREEN_TIME - cfg.MIN_GREEN_TIME)

    def _run_all_red(self, dt: float) -> None:
        """1-second all-red clearance between emergency phases."""
        self.all_red_timer += dt
        self.phase_timer   += dt
        if self.all_red_timer >= 1.0:
            self.all_red_timer = 0.0
            if self.emergency_lane:
                self._transition(cfg.STATE_EMERGENCY)
            else:
                # Determine which axis was waiting longest for recovery
                self._enter_recovery()

    def _run_emergency_state(self, dt: float) -> None:
        """
        Priority Mode: maintain a green corridor for emergency vehicles.

        Supports multiple ambulances:
          - Serve the currently tracked emergency vehicle_id (if any)
          - When it clears (not seen recently or distance ~0), immediately
            continue serving the next closest-ETA emergency vehicle.
          - If the next emergency is on the other axis, switch via ALL_RED.
        """
        self.phase_timer += dt

        def _pkt_dist(p: dict) -> Optional[float]:
            # Support both packet schemas: src.vehicle.Vehicle uses
            # distance_to_intersection; present.py uses dist.
            d = p.get("distance_to_intersection")
            if d is None:
                d = p.get("dist")
            try:
                return float(d) if d is not None else None
            except Exception:
                return None

        # Find the best emergency currently in the zone (smallest ETA)
        best = self._find_emergency_packet()

        # If no emergency packets at all, exit priority handling
        if not best:
            if self.logger and self.emergency_lane:
                self.logger.log_event(
                    "emergency_clear_all",
                    zone_id=self.zone_id,
                    lane_id=self.emergency_lane or "",
                )
            self.emergency_lane = None
            self.emergency_eta = None
            self.emergency_vehicle_id = None
            self.emergency_cleared = True
            self._transition(cfg.STATE_ALL_RED)
            return

        # Determine whether the currently served vehicle has cleared
        current_seen_recently = (
            self.emergency_vehicle_id is not None
            and self._emergency_last_seen_t >= 0
            and (self.sim_time - self._emergency_last_seen_t) <= 0.6
        )

        current_dist: Optional[float] = None
        if self.emergency_vehicle_id:
            for lane_pkts in self.lane_packets.values():
                for p in lane_pkts:
                    if p.get("vehicle_id") == self.emergency_vehicle_id and p.get("priority_flag", 0) == cfg.PRIORITY_EMERGENCY:
                        current_dist = _pkt_dist(p)
                        break

        current_cleared = False
        if self.emergency_vehicle_id is None:
            current_cleared = True
        elif not current_seen_recently and self.phase_timer > 0.8:
            current_cleared = True
        elif current_dist is not None and current_dist <= 5.0:
            current_cleared = True

        # If the current emergency is cleared, choose the next one (best ETA)
        if current_cleared:
            next_lane = best.get("lane_id")
            next_vid = best.get("vehicle_id")
            self.emergency_eta = best.get("eta")

            # No lane? fail safe to all red
            if next_lane not in cfg.LANE_IDS:
                self.emergency_lane = None
                self.emergency_vehicle_id = None
                self._transition(cfg.STATE_ALL_RED)
                return

            # If switching axis (NS <-> EW), bridge through ALL_RED
            axis_now = "NS" if self.emergency_lane in ("north", "south") else "EW"
            axis_next = "NS" if next_lane in ("north", "south") else "EW"
            self.emergency_lane = next_lane
            self.emergency_vehicle_id = next_vid
            self._emergency_last_seen_t = self.sim_time

            if axis_now != axis_next:
                if self.logger:
                    self.logger.log_event(
                        f"emergency_switch_axis:{axis_now}->{axis_next}",
                        zone_id=self.zone_id,
                        lane_id=next_lane,
                        vehicle_id=next_vid or "",
                    )
                self._transition(cfg.STATE_ALL_RED)
                return

            # Same axis: keep EMERGENCY state but reset timer for responsiveness
            self.phase_timer = 0.0
            if self.logger:
                self.logger.log_event(
                    "emergency_next_vehicle",
                    zone_id=self.zone_id,
                    lane_id=next_lane,
                    vehicle_id=next_vid or "",
                )

    def _enter_recovery(self) -> None:
        """Begin Recovery Phase: give extended green to longest-waiting lane."""
        # Find lane held at red longest
        worst_lane = max(self.lane_red_timer, key=self.lane_red_timer.get)
        self.recovery_lane  = worst_lane
        self.recovery_timer = 0.0
        self._transition(cfg.STATE_RECOVERY)
        if self.logger:
            self.logger.log_event(
                f"recovery_start:{worst_lane}",
                zone_id=self.zone_id,
                lane_id=worst_lane,
            )
        # Reset wait timers
        for l in cfg.LANE_IDS:
            self.lane_red_timer[l] = 0.0

    def _run_recovery(self, dt: float) -> None:
        """Extended green for the recovery lane, then return to normal cycle."""
        self.recovery_timer += dt
        self.phase_timer    += dt
        if self.recovery_timer >= cfg.RECOVERY_GREEN:
            # Determine whether recovery lane is NS or EW
            if self.recovery_lane in ("north", "south"):
                self._transition(cfg.STATE_EW_GREEN)
            else:
                self._transition(cfg.STATE_NS_GREEN)
            self.recovery_lane = None

    def _update_red_timers(self, dt: float) -> None:
        """Track how long each lane has been at red for recovery logic."""
        ns, ew = self._get_ns_ew_signals()
        for lane in ("north", "south"):
            if ns == "RED":
                self.lane_red_timer[lane] += dt
            else:
                self.lane_red_timer[lane] = 0.0
        for lane in ("east", "west"):
            if ew == "RED":
                self.lane_red_timer[lane] += dt
            else:
                self.lane_red_timer[lane] = 0.0

    def _get_ns_ew_signals(self) -> Tuple[str, str]:
        """Return (NS_signal, EW_signal) strings for current state."""
        mapping = {
            cfg.STATE_NS_GREEN:  ("GREEN",  "RED"),
            cfg.STATE_NS_YELLOW: ("YELLOW", "RED"),
            cfg.STATE_EW_GREEN:  ("RED",    "GREEN"),
            cfg.STATE_EW_YELLOW: ("RED",    "YELLOW"),
            cfg.STATE_ALL_RED:   ("RED",    "RED"),
            cfg.STATE_RECOVERY:  self._recovery_signals(),
            cfg.STATE_EMERGENCY: self._emergency_signals(),
        }
        return mapping.get(self.state, ("RED", "RED"))

    def _emergency_signals(self) -> Tuple[str, str]:
        """Signal state during EMERGENCY: emergency lane gets GREEN."""
        if self.emergency_lane in ("north", "south"):
            return ("GREEN", "RED")
        elif self.emergency_lane in ("east", "west"):
            return ("RED", "GREEN")
        return ("RED", "RED")

    def _recovery_signals(self) -> Tuple[str, str]:
        """Signal state during RECOVERY: recovery lane gets GREEN."""
        if self.recovery_lane in ("north", "south"):
            return ("GREEN", "RED")
        elif self.recovery_lane in ("east", "west"):
            return ("RED", "GREEN")
        return ("GREEN", "RED")   # default

    def _transition(self, new_state: str) -> None:
        """Apply a validated state transition."""
        ok, reason = self.failsafe.validate_transition(self.state, new_state)
        if not ok:
            # Force via ALL_RED safe bridge if direct jump is illegal
            # (can happen if preemption interrupts mid-yellow)
            if new_state in (cfg.STATE_EMERGENCY, cfg.STATE_ALL_RED):
                self.state       = cfg.STATE_ALL_RED
                self.phase_timer = 0.0
                return
            return   # reject silently — keep current state

        self.state       = new_state
        self.phase_timer = 0.0
        # Reset per-phase timers when state changes to prevent "stuck" phases.
        if new_state not in (cfg.STATE_NS_YELLOW, cfg.STATE_EW_YELLOW):
            self.yellow_timer = 0.0
        if new_state != cfg.STATE_ALL_RED:
            self.all_red_timer = 0.0
