"""
Life-Link — Vehicle Class (Saumya's module)

Models a single road vehicle: UUID, kinematics, lane assignment, JSON V2I
packet builder, and broker broadcast.
"""
from __future__ import annotations
import time
import uuid
import random
from typing import Optional, Tuple, TYPE_CHECKING

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from src.vehicle.physics import (
    update_kinematics, braking_distance, calculate_eta,
    euclidean_distance, in_detection_zone,
)
import src.config as cfg

if TYPE_CHECKING:
    from src.comm.broker import Broker


# ─── Lane spawn/direction data ────────────────────────────────────────────────
_LANE_META = {
    "north": {"spawn": (0.0,  480.0), "stop_y":  40.0, "axis": "y"},
    "south": {"spawn": (0.0, -480.0), "stop_y": -40.0, "axis": "y"},
    "east":  {"spawn": (480.0, 0.0),  "stop_x":  40.0, "axis": "x"},
    "west":  {"spawn": (-480.0, 0.0), "stop_x": -40.0, "axis": "x"},
}

_VEHICLE_SPEEDS = {
    "car":       (8.0,  14.0),
    "bike":      (6.0,  18.0),
    "auto":      (5.0,  10.0),
    "ambulance": (18.0, 22.0),
}


class Vehicle:
    """
    Represents a single vehicle in the Life-Link simulation.

    Attributes
    ----------
    vehicle_id   : globally unique identifier string
    vehicle_type : one of 'car', 'bike', 'auto', 'ambulance'
    lane_id      : 'north' | 'south' | 'east' | 'west'
    position     : (x, y) in world metres
    velocity     : scalar speed m/s
    acceleration : scalar m/s²
    priority_flag: 0 = civilian, 1 = emergency
    wait_time    : accumulated seconds spent stopped at red
    active       : True while vehicle has not cleared intersection
    """

    def __init__(
        self,
        lane_id: str,
        vehicle_type: str = "car",
        broker: Optional["Broker"] = None,
        zone_id: str = "Alpha",
    ) -> None:
        self.vehicle_id:   str   = f"VH_{uuid.uuid4().hex[:8].upper()}"
        self.lane_id:      str   = lane_id
        self.vehicle_type: str   = vehicle_type
        self.zone_id:      str   = zone_id
        self.broker               = broker
        self.active:       bool  = True
        self.wait_time:    float = 0.0
        self._stopped:     bool  = False
        self._stop_timer:  float = 0.0

        # Priority
        self.priority_flag: int = (
            cfg.PRIORITY_EMERGENCY if vehicle_type == "ambulance"
            else cfg.PRIORITY_CIVILIAN
        )

        # Position: spawn at lane head
        meta = _LANE_META[lane_id]
        self.position: Tuple[float, float] = tuple(meta["spawn"])  # type: ignore

        # Velocity: randomised per vehicle type
        lo, hi = _VEHICLE_SPEEDS[vehicle_type]
        self.velocity:     float = random.uniform(lo, hi)
        self.acceleration: float = cfg.DEFAULT_ACCEL

        # Offset position slightly so vehicles don't overlap in same lane
        jitter = random.uniform(-15.0, 15.0)
        x, y = self.position
        if meta["axis"] == "y":
            # north/south lane — spread on x slightly
            self.position = (x + jitter, y)
        else:
            self.position = (x, y + jitter)

        # Push start further back for variety
        extra = random.uniform(0, 200)
        x, y = self.position
        if meta["axis"] == "y":
            self.position = (x, y + (extra if y > 0 else -extra))
        else:
            self.position = (x + (extra if x > 0 else -extra), y)

        self._last_broadcast: float = 0.0

    # ── Kinematics ────────────────────────────────────────────────────────────

    def update(
        self,
        dt: float,
        signal_state: str = "GREEN",
        lead_gap_m: Optional[float] = None,
        lead_speed_ms: Optional[float] = None,
        lead_length_m: Optional[float] = None,
    ) -> None:
        """
        Advance vehicle physics by one time-step dt (seconds).

        Applies stop-line logic: vehicle decelerates to halt if the signal
        for its lane axis is RED/YELLOW and it is within braking distance of
        the stop line.

        Parameters
        ----------
        dt           : simulation tick (seconds)
        signal_state : 'GREEN', 'YELLOW', or 'RED' for this vehicle's lane axis
        """
        if not self.active:
            return

        dist = euclidean_distance(self.position)

        # Determine stop-line distance
        meta = _LANE_META[self.lane_id]
        if meta["axis"] == "y":
            stop_dist = abs(self.position[1]) - abs(meta.get("stop_y", 40.0))
        else:
            stop_dist = abs(self.position[0]) - abs(meta.get("stop_x", 40.0))
        stop_dist = max(0.0, stop_dist)

        # Should the vehicle brake for the signal?
        brk_dist = braking_distance(self.velocity, cfg.BRAKING_DECEL)
        must_stop = (
            signal_state in ("RED", "YELLOW")
            and stop_dist <= brk_dist + 2.0
            and stop_dist > 0.5
        )

        # Simple car-following: enforce safe gap to the lead vehicle in-lane.
        # safe_gap = standstill_gap + time_headway * v + lead_length
        follow_brake = False
        if lead_gap_m is not None and lead_speed_ms is not None:
            lead_len = lead_length_m if lead_length_m is not None else 0.0
            safe_gap = cfg.MIN_STANDSTILL_GAP_M + cfg.TIME_HEADWAY_S * self.velocity + lead_len
            if lead_gap_m < safe_gap:
                follow_brake = True

        if must_stop or follow_brake:
            decel = cfg.BRAKING_DECEL * (cfg.FOLLOW_BRAKE_MULT if follow_brake else 1.0)
            self.acceleration = -decel
        elif signal_state == "GREEN" and self.velocity < cfg.MAX_SPEED_MS:
            self.acceleration = cfg.DEFAULT_ACCEL
        else:
            self.acceleration = 0.0

        self.position, self.velocity, self.acceleration = update_kinematics(
            self.position, self.velocity, self.acceleration, dt,
            max_speed=cfg.EMERGENCY_SPEED if self.priority_flag else cfg.MAX_SPEED_MS,
            lane_axis=_LANE_META[self.lane_id]["axis"],
        )

        # Wait time tracking
        if self.velocity < 0.5 and signal_state in ("RED", "YELLOW"):
            self.wait_time += dt

        # Check if vehicle has cleared the intersection (passed centre)
        if dist < 10.0 and self.velocity > 0.1:
            self.active = False   # mark as cleared

    # ── Packet Building ───────────────────────────────────────────────────────

    def to_packet(self) -> dict:
        """
        Build a V2I JSON-serialisable data packet for this vehicle.

        Returns
        -------
        dict conforming to the Life-Link V2I packet spec:
            vehicle_id, timestamp, lane_id, position{x,y},
            velocity, acceleration, priority_flag, vehicle_type,
            distance_to_intersection, eta, zone_id
        """
        dist = euclidean_distance(self.position)
        eta  = calculate_eta(dist, self.velocity, self.acceleration)
        return {
            "vehicle_id":             self.vehicle_id,
            "timestamp":              time.time(),
            "zone_id":                self.zone_id,
            "lane_id":                self.lane_id,
            "vehicle_type":           self.vehicle_type,
            "position":               {"x": round(self.position[0], 3),
                                       "y": round(self.position[1], 3)},
            "velocity":               round(self.velocity, 3),
            "acceleration":           round(self.acceleration, 3),
            "priority_flag":          self.priority_flag,
            "distance_to_intersection": round(dist, 3),
            "eta":                    round(eta, 3) if eta != float('inf') else None,
            "wait_time":              round(self.wait_time, 3),
            "active":                 self.active,
        }

    # ── Broadcast ─────────────────────────────────────────────────────────────

    def broadcast(self) -> None:
        """
        Publish a V2I packet to the broker if within Detection Zone.

        Only vehicles within DETECTION_RADIUS metres of the intersection
        transmit packets (mimics wireless geofence).
        """
        if not self.active:
            return
        if not in_detection_zone(self.position, cfg.DETECTION_RADIUS):
            return
        if self.broker is None:
            return
        packet = self.to_packet()
        self.broker.publish(packet)
        self._last_broadcast = time.time()

    # ── Repr ─────────────────────────────────────────────────────────────────

    def __repr__(self) -> str:
        return (
            f"Vehicle({self.vehicle_id!r}, type={self.vehicle_type!r}, "
            f"lane={self.lane_id!r}, v={self.velocity:.1f}m/s, "
            f"pos={self.position}, prio={self.priority_flag})"
        )
