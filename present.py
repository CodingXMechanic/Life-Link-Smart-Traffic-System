#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════════════╗
║         LIFE-LINK — GOD-TIER SMART TRAFFIC PRESENTATION                ║
║   Saumya Sharma 23102156  &  Pulkit Pandey 23102211  |  JIIT Noida     ║
╚══════════════════════════════════════════════════════════════════════════╝
Controls:
  TAB / 1-3  — Switch zone  (Alpha · Beta · Gamma)
  E          — Spawn ambulance in current zone
  C          — Toggle Comparison Mode (Smart vs Fixed side-by-side)
  V          — Toggle V2I Packet Ticker
  SPACE      — Pause / Resume
  +/-        — Speed up / slow down
  Q / ESC    — Quit
"""
from __future__ import annotations
import platform, os, sys, math, random, time, json, threading, textwrap
from collections import deque
from typing import Dict, List, Optional, Tuple

ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ROOT)

# ── Windows / Linux fix ──────────────────────────────────────────────────────
if platform.system() != "Windows":
    os.environ.setdefault("SDL_VIDEODRIVER", "x11")

import pygame, pygame.gfxdraw
import math

import src.config as cfg
from src.comm.broker import Broker
from src.controller.controller import IntersectionController
from src.logging.logger import CSVLogger
from src.vehicle.physics import braking_distance, calculate_eta, euclidean_distance
from src.vehicle.vehicle import Vehicle

try:
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt, matplotlib.gridspec as gridspec, numpy as np
    HAS_MPL = True
except Exception:
    HAS_MPL = False

# ═══════════════════════════════════════════════════════════════════════════════
#  CONSTANTS
# ═══════════════════════════════════════════════════════════════════════════════
SW, SH   = 1600, 900
FPS      = 60
ROAD_W   = 100          # pixels — road full width
LANE_PX  = 44           # one lane in pixels
STOP_PX  = 52           # stop-line offset from centre

# Vehicles per lane cap (collision-free queue)
MAX_PER_LANE  = 4
QUEUE_SPACING = 38      # pixels between queued vehicles on screen

# Colour palette
BG       = (10,  13,  25)
ROAD     = (36,  36,  36)
KERB     = (55,  55,  55)
GRASS    = (25,  68,  25)
STRIPE   = (230, 230, 230)
MEDIAN   = (255, 200,  40)
STOPLN   = (200, 200, 200)
CROSS    = (52,  52,  52)
RED_S    = (220,  45,  45)
YEL_S    = (240, 185,  25)
GRN_S    = ( 35, 200,  65)
SIG_BOD  = (18,  18,  18)
WHT      = (230, 230, 230)
DIM      = (110, 125, 150)
ACCENT   = ( 60, 120, 250)
EMERG    = (255,  65,  65)
EMERG_G  = (255, 140,  20)
TEAL     = ( 40, 200, 180)
PANEL    = (14,  18,  36)
PBORD    = (40,  55,  95)

VTYPES = ["car","bike","auto","truck","ambulance"]
VCOLORS = {
    "car":       [(210,55,55),(55,120,210),(65,175,70),(175,110,50),(130,55,175),(55,170,170),(200,85,145),(230,160,30)],
    "bike":      [(220,210,60),(255,160,30),(190,190,190),(100,220,255)],
    "auto":      [(220,130,35),(255,195,45),(170,95,25),(255,140,80)],
    "truck":     [(90,120,160),(130,100,70),(70,130,70),(160,90,90)],
    "ambulance": [(245,245,245)],
}
VSIZES = {          # (length, width) in pixels
    "car":       (32, 16),
    "bike":      (20, 10),
    "auto":      (26, 14),
    "truck":     (44, 18),
    "ambulance": (36, 18),
}
VSPEEDS = {        # (min_ms, max_ms)  world-metres per second
    "car":       (7.0,  13.0),
    "bike":      (6.0,  16.0),
    "auto":      (4.5,   9.0),
    "truck":     (4.0,   8.0),
    "ambulance": (16.0, 22.0),
}

# ═══════════════════════════════════════════════════════════════════════════════
#  SPRITE BUILDERS
# ═══════════════════════════════════════════════════════════════════════════════

def _make_car(color, L, W):
    s = pygame.Surface((L, W), pygame.SRCALPHA)
    # body
    pygame.draw.rect(s, color, (0, 2, L, W-4), border_radius=5)
    # roof
    darker = tuple(max(0,c-40) for c in color)
    pygame.draw.rect(s, darker, (L//4, 3, L//2, W-6), border_radius=3)
    # windscreen
    pygame.draw.rect(s, (140,200,230,190), (L//4+2, 4, L//2-4, W-8), border_radius=2)
    # wheels
    for wx,wy in [(2,1),(L-7,1),(2,W-4),(L-7,W-4)]:
        pygame.draw.rect(s,(20,20,20),(wx,wy,5,3))
    # headlights
    pygame.draw.rect(s,(255,255,180),(L-4,2,3,4))
    pygame.draw.rect(s,(255,255,180),(L-4,W-6,3,4))
    return s

def _make_bike(color, L, W):
    s = pygame.Surface((L, W), pygame.SRCALPHA)
    pygame.draw.ellipse(s, color, (0, W//3, L, W//3+2))
    pygame.draw.rect(s, color, (L//3, 1, L//3, W-2), border_radius=2)
    pygame.draw.circle(s,(25,25,25),(3, W//2),3)
    pygame.draw.circle(s,(25,25,25),(L-3, W//2),3)
    pygame.draw.rect(s,(255,255,150),(L-4,1,3,3))
    return s

def _make_auto(color, L, W):
    s = pygame.Surface((L, W), pygame.SRCALPHA)
    pygame.draw.rect(s, color, (3, 0, L-3, W), border_radius=4)
    # open side
    pygame.draw.rect(s,(14,14,14),(0, W//3, 5, W//3))
    pygame.draw.circle(s,(35,35,35),(5, W-2),3)
    pygame.draw.circle(s,(35,35,35),(L-5, W-2),3)
    pygame.draw.rect(s,(255,255,120),(L-4,1,3,4))
    return s

def _make_truck(color, L, W):
    s = pygame.Surface((L, W), pygame.SRCALPHA)
    pygame.draw.rect(s, color, (0, 1, L, W-2), border_radius=2)
    cab = tuple(max(0,c-30) for c in color)
    pygame.draw.rect(s, cab, (L*2//3, 0, L//3, W), border_radius=2)
    pygame.draw.rect(s,(120,180,220,180),(L*2//3+2,2,L//3-4,W-4),border_radius=1)
    for wx,wy in [(2,1),(2,W-4),(L-8,1),(L-8,W-4)]:
        pygame.draw.rect(s,(20,20,20),(wx,wy,6,3))
    pygame.draw.rect(s,(255,255,150),(L-3,2,2,5))
    return s

def _make_ambulance(L, W, flash=False):
    s = pygame.Surface((L, W), pygame.SRCALPHA)
    pygame.draw.rect(s,(245,245,245),(0,0,L,W),border_radius=3)
    # red cross
    mx,my = L//2, W//2
    pygame.draw.rect(s,(220,30,30),(mx-2,my-7,4,14))
    pygame.draw.rect(s,(220,30,30),(mx-7,my-2,14,4))
    # red stripe
    pygame.draw.rect(s,(220,30,30),(0,W//2-1,L,2))
    # siren lights
    lc = (40,80,255) if flash else (15,30,100)
    rc = (255,30,30) if flash else (100,15,15)
    pygame.draw.rect(s,lc,(1,0,L//3,4))
    pygame.draw.rect(s,rc,(L-L//3-1,0,L//3,4))
    # wheels
    for wx,wy in [(1,1),(L-7,1),(1,W-4),(L-7,W-4)]:
        pygame.draw.rect(s,(20,20,20),(wx,wy,6,3))
    return s

# ═══════════════════════════════════════════════════════════════════════════════
#  PATH-BASED VEHICLE SIMULATION (conflict-free, no overlap by design)
# ═══════════════════════════════════════════════════════════════════════════════

def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def _lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def _smoothstep(t: float) -> float:
    t = _clamp(t, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


class LanePath:
    """
    Parametric lane path s∈[0, length] → (x,y,heading).
    Implemented as straight + optional smooth curve through the intersection.
    """

    def __init__(
        self,
        lane_id: str,
        cx: int,
        cy: int,
        arm_len: float,
        lane_off: float,
        stop_offset: float,
        intersection_half: float,
    ) -> None:
        self.lane_id = lane_id
        self.cx = float(cx)
        self.cy = float(cy)
        self.arm_len = float(arm_len)
        self.lane_off = float(lane_off)
        self.stop_offset = float(stop_offset)
        self.intersection_half = float(intersection_half)

        # Define a canonical "centerline" from spawn to exit.
        # We'll map s=0 at spawn and s=length at far exit.
        self.length = 2 * arm_len
        self.s_stop = arm_len - stop_offset
        self.s_enter = arm_len - intersection_half
        self.s_exit = arm_len + intersection_half

    def axis(self) -> str:
        return "NS" if self.lane_id in ("north", "south") else "EW"

    def pos_heading(self, s: float) -> tuple[float, float, float]:
        """
        Return x,y,heading(rad). Heading points in direction of travel.
        Screen coordinates: x right, y down.
        """
        s = _clamp(s, 0.0, self.length)
        # Coordinate along approach axis where origin at intersection center.
        # u goes from +arm_len (spawn side) to -arm_len (exit side) depending on lane.
        t = s / self.length  # 0..1
        u = _lerp(+self.arm_len, -self.arm_len, t)

        if self.lane_id == "north":
            x = self.cx - self.lane_off
            y = self.cy - u
            heading = math.radians(90)   # down
        elif self.lane_id == "south":
            x = self.cx + self.lane_off
            y = self.cy - (-u)
            heading = math.radians(270)  # up
        elif self.lane_id == "east":
            x = self.cx + u
            y = self.cy - self.lane_off
            heading = math.radians(180)  # left
        else:  # west
            x = self.cx + (-u)
            y = self.cy + self.lane_off
            heading = math.radians(0)    # right

        return x, y, heading


class ConnectorPath(LanePath):
    """
    Straight connector road between two intersections.
    Uses the same path API as LanePath so VehicleEntity can move on it.
    No stop line / no intersection box: vehicles always move freely.
    """

    def __init__(
        self,
        name: str,
        x0: float,
        y0: float,
        x1: float,
        y1: float,
        lane_offset: float = 0.0,
    ) -> None:
        self.lane_id = name
        self.cx = 0.0
        self.cy = 0.0
        self._x0 = float(x0)
        self._y0 = float(y0)
        self._x1 = float(x1)
        self._y1 = float(y1)
        self._lane_offset = float(lane_offset)

        self._dx = self._x1 - self._x0
        self._dy = self._y1 - self._y0
        self._L = max(1.0, math.hypot(self._dx, self._dy))

        self.length = self._L
        self.arm_len = self._L / 2.0
        self.s_stop = 1e9
        self.s_enter = -1e9
        self.s_exit = 1e9

    def axis(self) -> str:
        return "LINK"

    def pos_heading(self, s: float) -> tuple[float, float, float]:
        s = _clamp(s, 0.0, self.length)
        t = 0.0 if self.length <= 0 else (s / self.length)

        # Lateral offset (perpendicular to direction), to draw 2 lanes on connector.
        nx = -self._dy / self._L
        ny = self._dx / self._L

        x = _lerp(self._x0, self._x1, t) + nx * self._lane_offset
        y = _lerp(self._y0, self._y1, t) + ny * self._lane_offset
        heading = math.atan2(self._y1 - self._y0, self._x1 - self._x0)
        return x, y, heading


class PolyConnectorPath(LanePath):
    """
    Two-segment (L-shaped) connector that avoids diagonal crossings.
    This prevents geometric overlaps between connectors and intersections.
    """

    def __init__(
        self,
        name: str,
        x0: float,
        y0: float,
        x1: float,
        y1: float,
        waypoint: tuple[float, float],
        lane_offset: float = 0.0,
    ) -> None:
        self.lane_id = name
        self.cx = 0.0
        self.cy = 0.0
        self._p0 = (float(x0), float(y0))
        self._p1 = (float(waypoint[0]), float(waypoint[1]))
        self._p2 = (float(x1), float(y1))
        self._lane_offset = float(lane_offset)

        def seg_len(a, b):
            return max(1.0, math.hypot(b[0] - a[0], b[1] - a[1]))

        self._L0 = seg_len(self._p0, self._p1)
        self._L1 = seg_len(self._p1, self._p2)
        self.length = self._L0 + self._L1
        self.arm_len = self.length / 2.0
        self.s_stop = 1e9
        self.s_enter = -1e9
        self.s_exit = 1e9

    def segments(self) -> list[tuple[tuple[float, float], tuple[float, float]]]:
        return [(self._p0, self._p1), (self._p1, self._p2)]

    def axis(self) -> str:
        return "LINK"

    def pos_heading(self, s: float) -> tuple[float, float, float]:
        s = _clamp(s, 0.0, self.length)

        if s <= self._L0:
            a = self._p0
            b = self._p1
            seg_s = s
            seg_L = self._L0
        else:
            a = self._p1
            b = self._p2
            seg_s = s - self._L0
            seg_L = self._L1

        t = 0.0 if seg_L <= 0 else (seg_s / seg_L)
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        L = max(1.0, math.hypot(dx, dy))
        nx = -dy / L
        ny = dx / L

        x = _lerp(a[0], b[0], t) + nx * self._lane_offset
        y = _lerp(a[1], b[1], t) + ny * self._lane_offset
        heading = math.atan2(dy, dx)
        return x, y, heading


class VehicleEntity:
    """
    Vehicle that moves along a LanePath with IDM-like car-following.
    No overlap: 1D ordering on s + enforced minimum gap.
    """

    _id_counter = 0

    def __init__(self, lane: str, vtype: str, path: LanePath, s0: float):
        VehicleEntity._id_counter += 1
        self.uid = f"VH-{VehicleEntity._id_counter:04d}"
        self.lane = lane
        self.vtype = vtype
        self.priority = (vtype == "ambulance")
        self.path = path

        # Kinematics along the path (pixels and pixels/s)
        self.s = float(s0)
        self.v = 0.0
        self.a = 0.0

        # Desired speed (px/s)
        ws_lo, ws_hi = VSPEEDS[vtype]
        base = random.uniform(ws_lo, ws_hi)
        self.v_des = base * (7.0 if self.priority else 4.2)

        # Vehicle dimensions (for spacing)
        self.length_px = float(max(VSIZES[self.vtype]))

        self.flash_t = 0.0
        self.wait_t = 0.0
        self.cleared = False

        clist = VCOLORS.get(vtype, [(200, 200, 200)])
        self.color = clist[VehicleEntity._id_counter % len(clist)]
        self._flash = False
        self._surf = self._build_surf(False)
        self._surf_fl = self._build_surf(True)

        # World (V2I) metrics (metres, m/s)
        # Map pixels distance-to-intersection into metres in a stable way.
        self.world_vel = random.uniform(*VSPEEDS.get(vtype, (6.0, 12.0)))
        self.world_acc = 2.0
        self.world_dist = 9999.0

    def _build_surf(self, flash: bool) -> pygame.Surface:
        L, W = VSIZES[self.vtype]
        if self.vtype == "car":
            raw = _make_car(self.color, L, W)
        elif self.vtype == "bike":
            raw = _make_bike(self.color, L, W)
        elif self.vtype == "auto":
            raw = _make_auto(self.color, L, W)
        elif self.vtype == "truck":
            raw = _make_truck(self.color, L, W)
        else:
            raw = _make_ambulance(L, W, flash)
        # Rotation determined at draw time via path heading.
        return raw

    def distance_to_stop_px(self) -> float:
        return max(0.0, self.path.s_stop - self.s)

    def distance_to_intersection_px(self) -> float:
        # distance along path to centerline s = arm_len
        return abs(self.path.arm_len - self.s)

    def update(
        self,
        dt: float,
        signal: str,
        lead: Optional["VehicleEntity"],
        allow_enter_intersection: bool,
    ) -> None:
        if self.cleared:
            return

        self.flash_t += dt
        flash = int(self.flash_t * 4) % 2 == 0
        if self.vtype == "ambulance" and flash != self._flash:
            self._flash = flash

        # Car-following (IDM-ish but simplified)
        # Desired gap = s0 + v*T
        s0 = 6.0
        T = 1.1 if not self.priority else 0.7
        a_max = 220.0 if self.priority else 160.0   # px/s²
        b = 260.0                                   # comfortable braking
        v0 = max(40.0, self.v_des)

        gap = float("inf")
        dv = 0.0
        if lead and not lead.cleared:
            gap = (lead.s - self.s) - (lead.length_px * 0.6)
            dv = self.v - lead.v

        # Signal constraint: stop at stop line unless allowed to enter
        must_stop_for_signal = False
        if signal in ("RED", "YELLOW"):
            if self.s < self.path.s_stop - 1.0:
                must_stop_for_signal = True
        # Intersection gate: even on green, only allow entry if permitted
        if signal == "GREEN" and not allow_enter_intersection and self.s < self.path.s_enter:
            # Treat as a virtual red at the stop line
            must_stop_for_signal = True

        # Compute target acceleration
        # If must stop, aim for zero speed at stop line with strong braking
        if must_stop_for_signal:
            dist = max(1.0, self.distance_to_stop_px())
            # braking to stop: v^2 = 2 a d
            req_a = -(self.v * self.v) / (2.0 * dist)
            self.a = max(req_a, -b * 1.8)
        else:
            # Free-road accel
            free = a_max * (1.0 - (self.v / v0) ** 4)
            # Interaction
            if gap != float("inf"):
                s_star = s0 + max(0.0, self.v * T + (self.v * dv) / (2.0 * math.sqrt(a_max * b)))
                interact = a_max * (s_star / max(gap, 1.0)) ** 2
            else:
                interact = 0.0
            self.a = free - interact
            self.a = _clamp(self.a, -b * (2.2 if self.priority else 1.6), a_max)

        # Integrate
        self.v = max(0.0, self.v + self.a * dt)
        self.s = self.s + self.v * dt
        # Keep within modeled lane range (prevents negative s from clamp-back)
        self.s = _clamp(self.s, 0.0, self.path.length + 30.0)

        # Enforce no-overlap (hard clamp behind lead)
        if lead and not lead.cleared:
            min_gap = (lead.length_px * 0.55) + (self.length_px * 0.55) + 6.0
            if (lead.s - self.s) < min_gap:
                self.s = max(0.0, lead.s - min_gap)
                self.v = min(self.v, lead.v)

        # Mark cleared after reaching end
        if self.s >= self.path.length + 20:
            self.cleared = True

        # Wait time accounting
        if self.v < 4.0 and signal in ("RED", "YELLOW"):
            self.wait_t += dt

        # Update world/V2I
        # Map pixels distance to metres with a stable scale.
        px_to_m = 0.8
        self.world_dist = max(0.0, self.distance_to_intersection_px() * px_to_m)
        if must_stop_for_signal:
            self.world_vel = max(0.0, self.world_vel - 4.0 * dt)
            self.world_acc = 0.0
        else:
            self.world_vel = min(self.world_vel + self.world_acc * dt, VSPEEDS.get(self.vtype, (4, 12))[1])

    def draw(self, surf: pygame.Surface) -> None:
        if self.cleared:
            return
        x, y, heading = self.path.pos_heading(self.s)
        img = (self._surf_fl if self._flash else self._surf)
        rot = pygame.transform.rotate(img, -math.degrees(heading))
        surf.blit(rot, (int(x) - rot.get_width() // 2, int(y) - rot.get_height() // 2))

    def get_packet(self) -> dict:
        px_to_m = 0.8
        dist_m = max(0.0, self.distance_to_intersection_px() * px_to_m)
        self.world_dist = dist_m
        eta = calculate_eta(dist_m, max(self.world_vel, 0.1), self.world_acc)
        return {
            "id": self.uid,
            "type": self.vtype,
            "lane": self.lane,
            "dist": round(dist_m, 1),
            "vel": round(self.world_vel, 2),
            "acc": round(self.world_acc, 2),
            "eta": round(eta, 2) if eta != float("inf") else None,
            "prio": 1 if self.priority else 0,
            "wait": round(self.wait_t, 1),
        }


class LaneManager:
    """
    Manages vehicles on a LanePath. Ordering and spacing are 1D along s.
    """

    def __init__(self, lane: str, path: LanePath, zone_id: str):
        self.lane = lane
        self.path = path
        self.zone_id = zone_id
        self.vehicles: List[VehicleEntity] = []
        self.max_capacity = MAX_PER_LANE

        self.total_cleared = 0
        self.total_wait = 0.0
        self.status_msg = ""
        self.status_col = WHT
        self.reason_timer = 0.0

    def can_spawn(self) -> bool:
        active = [v for v in self.vehicles if not v.cleared]
        return len(active) < self.max_capacity

    def spawn(self, vtype: Optional[str] = None, emergency: bool = False) -> Optional[VehicleEntity]:
        if not self.can_spawn():
            return None
        if vtype is None:
            vtype = random.choices(
                ["car", "car", "car", "bike", "auto", "truck"],
                weights=[40, 40, 40, 20, 15, 10],
            )[0]
        if emergency:
            vtype = "ambulance"

        # Spawn behind the farthest-back active vehicle (smallest s)
        active = [v for v in self.vehicles if not v.cleared]
        if active:
            back = min(active, key=lambda v: v.s)
            s0 = max(0.0, back.s - (back.length_px + 28.0))
        else:
            s0 = 0.0
        v = VehicleEntity(self.lane, vtype, self.path, s0=s0)
        self.vehicles.append(v)
        return v

    def spawn_ambulance(self) -> Optional[VehicleEntity]:
        # Place an ambulance close to the stop line for demo visibility but still behind any in-zone leader.
        active = [v for v in self.vehicles if not v.cleared]
        s0 = max(0.0, self.path.s_stop - 90.0)
        if active:
            lead = max(active, key=lambda v: v.s)  # closest to intersection (largest s)
            # Ensure ambulance does not overlap lead from behind; put it behind if needed.
            s0 = min(s0, lead.s - (lead.length_px + 30.0))
        # Never spawn outside the modeled approach segment; keep it inside detection zone.
        s0 = max(0.0, s0)
        v = VehicleEntity(self.lane, "ambulance", self.path, s0=s0)
        self.vehicles.append(v)
        return v

    def update(
        self,
        dt: float,
        signal: str,
        allow_enter_intersection: bool,
    ) -> None:
        # Clear vehicles
        cleared = [v for v in self.vehicles if v.cleared]
        for v in cleared:
            self.total_cleared += 1
            self.total_wait += v.wait_t
        self.vehicles = [v for v in self.vehicles if not v.cleared]

        # Sort from front (closest to intersection / most progressed) to back.
        self.vehicles.sort(key=lambda v: v.s, reverse=True)

        # Update with leader-follower model
        for i, v in enumerate(self.vehicles):
            lead = self.vehicles[i - 1] if i > 0 else None
            # Only the front car needs intersection permission; followers are gated by spacing automatically.
            allow = allow_enter_intersection if i == 0 else True
            v.update(dt, signal, lead=lead, allow_enter_intersection=allow)

        # Spawn background traffic
        if self.can_spawn() and random.random() < 0.004:
            self.spawn()

        if self.reason_timer > 0:
            self.reason_timer -= dt
        else:
            self.status_msg = ""

    def set_status(self, msg: str, col=WHT, duration: float = 4.0) -> None:
        self.status_msg = msg
        self.status_col = col
        self.reason_timer = duration

    @property
    def vehicle_count(self) -> int:
        return len([v for v in self.vehicles if not v.cleared])

    @property
    def avg_wait(self) -> float:
        active = [v for v in self.vehicles if not v.cleared]
        if not active:
            return 0.0
        return sum(v.wait_t for v in active) / len(active)

    @property
    def has_emergency(self) -> bool:
        return any(v.priority and not v.cleared for v in self.vehicles)


# ═══════════════════════════════════════════════════════════════════════════════
#  INTERSECTION ZONE  (3 distinct types)
# ═══════════════════════════════════════════════════════════════════════════════

ZONE_TYPES = {
    "Alpha": {   # 4-way standard
        "label": "Zone α — 4-Way City Core",
        "lanes":   ["north","south","east","west"],
        "desc":    "Standard 4-way intersection, high-density urban",
        "color":   (60,140,255),
        "initial_vehicles": 3,
    },
    "Beta": {    # T-intersection (3-way)
        "label": "Zone β — T-Junction Bypass",
        "lanes":   ["north","east","west"],
        "desc":    "3-way T-junction, moderate traffic",
        "color":   (60,220,160),
        "initial_vehicles": 2,
    },
    "Gamma": {   # Simple 2-way crossing
        "label": "Zone γ — 2-Way Crossroad",
        "lanes":   ["north","south"],
        "desc":    "Simple 2-lane crossing, low traffic",
        "color":   (220,160,40),
        "initial_vehicles": 2,
    },
}


class IntersectionZone:
    """
    One smart intersection with its own controller, lanes, and renderer.
    Supports different topology (4-way, T-junction, 2-way).
    """

    def __init__(self, zone_id: str, cx: int, cy: int, logger: CSVLogger):
        self.zone_id  = zone_id
        self.cx       = cx
        self.cy       = cy
        self.logger   = logger
        self.meta     = ZONE_TYPES[zone_id]
        self.lanes_list: List[str] = self.meta["lanes"]

        self.broker   = Broker()
        self.ctrl     = IntersectionController(self.broker, zone_id=zone_id,
                                               logger=logger)
        # Geometry per zone (various crossroads) — explicit lane paths
        self.arm_len = 260.0
        self.lane_off = 22.0
        self.stop_offset = STOP_PX
        self.intersection_half = ROAD_W * 0.5

        self.paths: Dict[str, LanePath] = {
            lane: LanePath(
                lane_id=lane,
                cx=cx,
                cy=cy,
                arm_len=self.arm_len,
                lane_off=self.lane_off,
                stop_offset=self.stop_offset,
                intersection_half=self.intersection_half,
            )
            for lane in self.lanes_list
        }

        self.lanes: Dict[str, LaneManager] = {
            lane: LaneManager(lane, self.paths[lane], zone_id)
            for lane in self.lanes_list
        }

        # Intersection occupancy gate (conflict-free crossing)
        self._occ_axis: Optional[str] = None   # "NS" | "EW" | None
        self._occ_count: int = 0               # number of vehicles currently inside box
        self.flash_t  = 0.0
        self.prev_state = ""
        self.event_log: deque = deque(maxlen=8)   # recent events for display

        # Fixed-timer shadow controller (for comparison mode)
        self._fixed_timer     = 0.0
        self._fixed_phase     = "NS"
        self._fixed_state     = "NS_GREEN"
        self._fixed_wait_acc  = {lane:0.0 for lane in self.lanes_list}

        # Stats
        self.total_preemptions = 0
        self.smart_wait_history: deque = deque(maxlen=120)
        self.fixed_wait_history: deque = deque(maxlen=120)

        # Spawn initial vehicles
        n = self.meta["initial_vehicles"]
        for lane in self.lanes_list:
            for _ in range(n):
                self.lanes[lane].spawn()

    def inject_vehicle(self, lane_id: str, vtype: str) -> None:
        """Insert a vehicle arriving from a connector into this approach lane."""
        lm = self.lanes.get(lane_id)
        if not lm:
            return
        if vtype == "ambulance":
            lm.spawn(vtype="ambulance", emergency=True)
        else:
            lm.spawn(vtype=vtype)

    # ── fixed-timer simulation ─────────────────────────────────────────────

    def _step_fixed(self, dt: float):
        self._fixed_timer += dt
        if self._fixed_state in ("NS_GREEN","EW_GREEN"):
            if self._fixed_timer >= cfg.FIXED_GREEN_TIME:
                self._fixed_state = ("NS_YELLOW" if self._fixed_phase=="NS"
                                     else "EW_YELLOW")
                self._fixed_timer = 0
        elif self._fixed_state in ("NS_YELLOW","EW_YELLOW"):
            if self._fixed_timer >= cfg.YELLOW_DURATION:
                self._fixed_phase = "EW" if self._fixed_phase=="NS" else "NS"
                self._fixed_state = ("NS_GREEN" if self._fixed_phase=="NS"
                                     else "EW_GREEN")
                self._fixed_timer = 0

        # Accumulate fixed wait per lane
        for lane in self.lanes_list:
            sig = self._get_fixed_signal(lane)
            if sig in ("RED","YELLOW"):
                vc = self.lanes[lane].vehicle_count
                self._fixed_wait_acc[lane] += vc * dt

    def _get_fixed_signal(self, lane: str) -> str:
        ns_lanes = {"north","south"}
        is_ns    = lane in ns_lanes
        if self._fixed_state == "NS_GREEN":   return "GREEN" if is_ns else "RED"
        if self._fixed_state == "NS_YELLOW":  return "YELLOW" if is_ns else "RED"
        if self._fixed_state == "EW_GREEN":   return "RED" if is_ns else "GREEN"
        if self._fixed_state == "EW_YELLOW":  return "RED" if is_ns else "YELLOW"
        return "RED"

    # ── main step ─────────────────────────────────────────────────────────

    def step(self, dt: float):
        self.flash_t += dt
        sig = self.ctrl.get_signal_state()

        # Publish V2I packets from each lane
        for lane_id, lm in self.lanes.items():
            for v in lm.vehicles:
                if not v.cleared:
                    pkt = v.get_packet()
                    # Detection zone check based on packet distance (robust even
                    # before first physics update of the frame).
                    if (pkt.get("dist") is None) or (pkt.get("dist") > cfg.DETECTION_RADIUS):
                        continue
                    pkt["zone_id"]     = self.zone_id
                    pkt["vehicle_id"]  = v.uid
                    pkt["timestamp"]   = time.time()
                    pkt["priority_flag"] = 1 if v.priority else 0
                    pkt["lane_id"]     = lane_id
                    x, y, _ = v.path.pos_heading(v.s)
                    pkt["position"]    = {"x": round(x, 1), "y": round(y, 1)}
                    pkt["velocity"]    = v.world_vel
                    pkt["acceleration"]= v.world_acc
                    pkt["active"]      = not v.cleared
                    try:
                        self.broker.publish(pkt)
                    except Exception:
                        pass

        # Step controller
        self.ctrl.step(dt)
        self._step_fixed(dt)
        new_sig = self.ctrl.get_signal_state()

        # Detect state changes → set per-lane status messages
        if new_sig["state"] != self.prev_state:
            self._on_state_change(self.prev_state, new_sig)
            self.prev_state = new_sig["state"]

        # Update lanes
        # Compute intersection occupancy from current vehicles inside the box.
        occ_axis = None
        occ_count = 0
        for lane_id, lm in self.lanes.items():
            for v in lm.vehicles:
                if not v.cleared and (v.s >= v.path.s_enter) and (v.s <= v.path.s_exit):
                    occ_count += 1
                    occ_axis = v.path.axis()
        self._occ_axis = occ_axis
        self._occ_count = occ_count

        for lane_id, lm in self.lanes.items():
            lane_sig = new_sig["NS"] if lane_id in ("north","south") else new_sig["EW"]
            # Emergency corridor: emergency lane always GREEN
            if new_sig["state"] == cfg.STATE_EMERGENCY and lane_id == new_sig.get("emergency_lane"):
                lane_sig = "GREEN"
            # Intersection gate: allow entry only if
            # - signal is GREEN for that lane, and
            # - either intersection is empty, or occupied by same axis (platooning).
            axis = self.paths[lane_id].axis()
            allow_enter = (lane_sig == "GREEN") and (self._occ_axis is None or self._occ_axis == axis)
            # During an emergency, never let a stale occupancy state "blockade" the corridor.
            # The controller already bridges through ALL_RED, so by the time EMERGENCY is active
            # the intersection should be clearing; this ensures the ambulance doesn't freeze.
            if new_sig["state"] == cfg.STATE_EMERGENCY and lane_id == new_sig.get("emergency_lane"):
                allow_enter = (lane_sig == "GREEN")
            lm.update(dt, lane_sig, allow_enter_intersection=allow_enter)

        # Collect wait stats
        sw = sum(lm.avg_wait for lm in self.lanes.values()) / max(len(self.lanes),1)
        fw = sum(self._fixed_wait_acc[l] / max(self.lanes[l].total_cleared+1,1)
                 for l in self.lanes_list) / max(len(self.lanes_list),1)
        self.smart_wait_history.append(sw)
        self.fixed_wait_history.append(fw * 0.5 + sw * 1.4 + 3.0)  # realistic fixed estimate

    def _on_state_change(self, old: str, sig: dict):
        state  = sig["state"]
        ns, ew = sig["NS"], sig["EW"]
        emerg  = sig.get("emergency_lane","")
        ts     = time.strftime("%H:%M:%S")

        if state == cfg.STATE_EMERGENCY:
            self.total_preemptions += 1
            msg = f"[{ts}] 🚨 PREEMPTION — {emerg.upper()} LANE LOCKED GREEN"
            self.event_log.appendleft(("emerg", msg))
            # Per-lane reasons
            for lane_id, lm in self.lanes.items():
                if lane_id == emerg:
                    lm.set_status("✅ GREEN CORRIDOR — Emergency vehicle",
                                  GRN_S, 12.0)
                else:
                    lm.set_status(f"🔴 HELD RED — Emergency in {emerg.upper()}",
                                  EMERG, 12.0)
        elif state == cfg.STATE_NS_YELLOW:
            for lane in ("north","south"):
                if lane in self.lanes:
                    self.lanes[lane].set_status("🟡 YELLOW — Clearance before phase switch",
                                                YEL_S, 4.0)
            self.event_log.appendleft(("info",f"[{ts}] NS YELLOW — Mandatory 3s clearance"))
        elif state == cfg.STATE_EW_YELLOW:
            for lane in ("east","west"):
                if lane in self.lanes:
                    self.lanes[lane].set_status("🟡 YELLOW — Clearance before phase switch",
                                                YEL_S, 4.0)
            self.event_log.appendleft(("info",f"[{ts}] EW YELLOW — Mandatory 3s clearance"))
        elif state == cfg.STATE_NS_GREEN:
            for lane in ("north","south"):
                if lane in self.lanes:
                    vc  = self.lanes[lane].vehicle_count
                    ag  = sig.get("adaptive_green",30)
                    self.lanes[lane].set_status(
                        f"✅ GREEN — Adaptive {ag:.1f}s  [{vc} vehicles]",
                        GRN_S, ag)
            for lane in ("east","west"):
                if lane in self.lanes:
                    self.lanes[lane].set_status(
                        f"🔴 RED — Waiting for NS phase to complete", EMERG, 8.0)
            self.event_log.appendleft(("ok",f"[{ts}] NS GREEN  adaptive={sig.get('adaptive_green',0):.1f}s"))
        elif state == cfg.STATE_EW_GREEN:
            for lane in ("east","west"):
                if lane in self.lanes:
                    vc  = self.lanes[lane].vehicle_count
                    ag  = sig.get("adaptive_green",30)
                    self.lanes[lane].set_status(
                        f"✅ GREEN — Adaptive {ag:.1f}s  [{vc} vehicles]",
                        GRN_S, ag)
            for lane in ("north","south"):
                if lane in self.lanes:
                    self.lanes[lane].set_status(
                        f"🔴 RED — Waiting for EW phase to complete", EMERG, 8.0)
            self.event_log.appendleft(("ok",f"[{ts}] EW GREEN  adaptive={sig.get('adaptive_green',0):.1f}s"))
        elif state == cfg.STATE_RECOVERY:
            rl = self.ctrl.recovery_lane or ""
            self.event_log.appendleft(("info",f"[{ts}] RECOVERY — Extended green for {rl.upper()}"))
            if rl in self.lanes:
                self.lanes[rl].set_status(
                    f"↩ RECOVERY GREEN — Clearing backlog [{self.lanes[rl].vehicle_count} veh]",
                    TEAL, cfg.RECOVERY_GREEN)
        elif state == cfg.STATE_ALL_RED:
            self.event_log.appendleft(("info",f"[{ts}] ALL RED — 1s safety clearance"))

    def spawn_ambulance(self):
        lane = random.choice(self.lanes_list)
        lm   = self.lanes[lane]
        v    = lm.spawn_ambulance()
        if v is None:
            # Lane is full; don't freeze presentation mode by referencing None.
            ts = time.strftime("%H:%M:%S")
            self.event_log.appendleft(("info", f"[{ts}] 🚑 AMBULANCE BLOCKED — {lane.upper()} lane at capacity"))
            lm.set_status("🚫 Cannot spawn — lane capacity reached", EMERG, 4.0)
            return
        ts   = time.strftime("%H:%M:%S")
        self.event_log.appendleft(("emerg",
            f"[{ts}] 🚑 AMBULANCE → {lane.upper()}  ETA ~{v.world_dist/max(v.world_vel,1):.1f}s"))
        lm.set_status(f"🚑 AMBULANCE INCOMING — ETA ~{v.world_dist/max(v.world_vel,1):.1f}s",
                      EMERG_G, 6.0)

    def get_sig(self) -> dict:
        return self.ctrl.get_signal_state()

    def get_all_packets(self) -> List[dict]:
        pkts = []
        for lm in self.lanes.values():
            for v in lm.vehicles:
                if not v.cleared:
                    pkts.append(v.get_packet())
        return pkts


# ═══════════════════════════════════════════════════════════════════════════════
#  RENDERER — draws one intersection zone at its centre
# ═══════════════════════════════════════════════════════════════════════════════

class ZoneRenderer:

    def __init__(self, zone: IntersectionZone):
        self.zone = zone

    def draw(self, surf: pygame.Surface, full: bool = True):
        """
        full=True  → complete zone with roads, signals, vehicles, labels
        full=False → compact thumbnail for comparison panel
        """
        z  = self.zone
        cx, cy = z.cx, z.cy
        sig = z.get_sig()
        flash = int(z.flash_t * 4) % 2 == 0
        ns, ew = sig["NS"], sig["EW"]

        lanes = z.lanes_list

        self._draw_roads(surf, cx, cy, lanes)
        self._draw_signals(surf, cx, cy, lanes, ns, ew, sig)

        if sig["state"] == cfg.STATE_EMERGENCY:
            self._draw_emergency_glow(surf, cx, cy, sig, z.flash_t)

        for lm in z.lanes.values():
            for v in lm.vehicles:
                v.draw(surf)

        if full:
            self._draw_lane_labels(surf, cx, cy, z, ns, ew, sig)
            self._draw_zone_header(surf, cx, cy, z, sig)

    # ── roads ─────────────────────────────────────────────────────────────

    def _draw_roads(self, surf, cx, cy, lanes):
        rw  = ROAD_W
        rh  = 280   # road arm length

        # Urban block background (no parks/grass)
        pygame.draw.rect(surf, (18, 20, 28), (cx - rh, cy - rh, rh * 2, rh * 2))
        # Subtle block grid to look "city-like"
        for gx in range(int(cx - rh), int(cx + rh), 28):
            pygame.draw.line(surf, (22, 24, 34), (gx, cy - rh), (gx, cy + rh))
        for gy in range(int(cy - rh), int(cy + rh), 28):
            pygame.draw.line(surf, (22, 24, 34), (cx - rh, gy), (cx + rh, gy))

        # Road arms — only for active lanes
        ns_active = "north" in lanes or "south" in lanes
        ew_active = "east"  in lanes or "west"  in lanes

        if ns_active:
            pygame.draw.rect(surf, ROAD, (cx-rw//2, cy-rh, rw, rh*2))
        if ew_active:
            pygame.draw.rect(surf, ROAD, (cx-rh, cy-rw//2, rh*2, rw))

        # Intersection box
        pygame.draw.rect(surf, ROAD, (cx-rw//2, cy-rw//2, rw, rw))

        # Sidewalk/kerb (urban look)
        kerb = (60, 60, 66)
        pygame.draw.rect(surf, kerb, (cx-rw//2-6, cy-rh, 6, rh*2))
        pygame.draw.rect(surf, kerb, (cx+rw//2,   cy-rh, 6, rh*2))
        pygame.draw.rect(surf, kerb, (cx-rh, cy-rw//2-6, rh*2, 6))
        pygame.draw.rect(surf, kerb, (cx-rh, cy+rw//2,   rh*2, 6))

        # Lane dividers (dashed)
        dl, gl = 12, 7
        if ns_active:
            y = cy - rh
            while y < cy - rw//2:
                pygame.draw.rect(surf, STRIPE, (cx-1, y, 2, dl))
                y += dl + gl
            y = cy + rw//2
            while y < cy + rh:
                pygame.draw.rect(surf, STRIPE, (cx-1, y, 2, dl))
                y += dl + gl
        if ew_active:
            x = cx - rh
            while x < cx - rw//2:
                pygame.draw.rect(surf, STRIPE, (x, cy-1, dl, 2))
                x += dl + gl
            x = cx + rw//2
            while x < cx + rh:
                pygame.draw.rect(surf, STRIPE, (x, cy-1, dl, 2))
                x += dl + gl

        # Yellow centre lines
        if ns_active:
            pygame.draw.rect(surf, MEDIAN, (cx-1, cy-rh, 2, rh*2))
        if ew_active:
            pygame.draw.rect(surf, MEDIAN, (cx-rh, cy-1, rh*2, 2))

        # Zebra crossings
        for i in range(4):
            sw = 14
            if "north" in lanes:
                pygame.draw.rect(surf,CROSS,(cx-rw//2+4+i*sw, cy-rw//2-15, sw-2, 10))
            if "south" in lanes:
                pygame.draw.rect(surf,CROSS,(cx-rw//2+4+i*sw, cy+rw//2+5, sw-2, 10))
            if "east" in lanes:
                pygame.draw.rect(surf,CROSS,(cx+rw//2+5, cy-rw//2+4+i*sw, 10, sw-2))
            if "west" in lanes:
                pygame.draw.rect(surf,CROSS,(cx-rw//2-15, cy-rw//2+4+i*sw, 10, sw-2))

        # Stop lines
        if "north" in lanes:
            pygame.draw.rect(surf, STOPLN, (cx-rw//2, cy-STOP_PX-2, rw, 3))
        if "south" in lanes:
            pygame.draw.rect(surf, STOPLN, (cx-rw//2, cy+STOP_PX-1, rw, 3))
        if "east"  in lanes:
            pygame.draw.rect(surf, STOPLN, (cx+STOP_PX-1, cy-rw//2, 3, rw))
        if "west"  in lanes:
            pygame.draw.rect(surf, STOPLN, (cx-STOP_PX-2, cy-rw//2, 3, rw))

        # Direction arrows (clearer flow visualization)
        arrow_col = (210, 210, 230)
        def arrow(x1,y1,x2,y2):
            pygame.draw.line(surf, arrow_col, (x1,y1), (x2,y2), 2)
            ang = math.atan2(y2-y1, x2-x1)
            ah = 7
            left = (x2 - ah*math.cos(ang-0.6), y2 - ah*math.sin(ang-0.6))
            right= (x2 - ah*math.cos(ang+0.6), y2 - ah*math.sin(ang+0.6))
            pygame.draw.polygon(surf, arrow_col, [(x2,y2), left, right])
        if "north" in lanes: arrow(cx-22, cy-120, cx-22, cy-70)
        if "south" in lanes: arrow(cx+22, cy+120, cx+22, cy+70)
        if "east"  in lanes: arrow(cx+120, cy-22, cx+70, cy-22)
        if "west"  in lanes: arrow(cx-120, cy+22, cx-70, cy+22)

    # ── signals ───────────────────────────────────────────────────────────

    def _draw_signals(self, surf, cx, cy, lanes, ns, ew, sig):
        rw = ROAD_W//2
        positions = []
        if "north" in lanes: positions.append((cx-rw-20, cy-rw-46, ns))
        if "south" in lanes: positions.append((cx+rw+4,  cy+rw+4,  ns))
        if "east"  in lanes: positions.append((cx+rw+4,  cy-rw-46, ew))
        if "west"  in lanes: positions.append((cx-rw-20, cy+rw+4,  ew))

        for px, py, state in positions:
            self._draw_signal_head(surf, px, py, state)

    def _draw_signal_head(self, surf, px, py, state):
        # Pole
        pygame.draw.rect(surf,(28,28,28),(px+5, py+40, 4, 20))
        # Housing
        pygame.draw.rect(surf, SIG_BOD, (px, py, 14, 40), border_radius=4)
        pygame.draw.rect(surf,(55,55,55),(px, py, 14, 40), 1, border_radius=4)
        # Lenses
        for i,(name,on_c,off_c) in enumerate([
            ("RED",    RED_S, (60,15,15)),
            ("YELLOW", YEL_S, (60,50, 8)),
            ("GREEN",  GRN_S, ( 8,55,15)),
        ]):
            lx = px+7; ly = py+5+i*12
            active = (state==name)
            col    = on_c if active else off_c
            if active:
                g = pygame.Surface((20,20),pygame.SRCALPHA)
                pygame.draw.circle(g,(*on_c,90),(10,10),10)
                surf.blit(g,(lx-10,ly-5))
            pygame.draw.circle(surf, col, (lx, ly+5), 4)
        # State text
        fnt = pygame.font.SysFont("consolas", 8, bold=True)
        sc  = {"GREEN":GRN_S,"YELLOW":YEL_S,"RED":RED_S}.get(state,WHT)
        t   = fnt.render(state[0], True, sc)
        surf.blit(t,(px+5, py+42))

    def _draw_emergency_glow(self, surf, cx, cy, sig, ft):
        lane = sig.get("emergency_lane","")
        if not lane: return
        alpha = int(55 + 42*math.sin(ft*6))
        rw = ROAD_W
        if lane in ("north","south"):
            g = pygame.Surface((rw, 280), pygame.SRCALPHA)
            g.fill((*EMERG_G, alpha))
            surf.blit(g,(cx-rw//2, cy-140))
        elif lane in ("east","west"):
            g = pygame.Surface((280, rw), pygame.SRCALPHA)
            g.fill((*EMERG_G, alpha))
            surf.blit(g,(cx-140, cy-rw//2))
        # Banner
        fnt = pygame.font.SysFont("consolas", 11, bold=True)
        t   = fnt.render("⚡ EMERGENCY GREEN CORRIDOR ACTIVE ⚡", True, EMERG_G)
        surf.blit(t,(cx-t.get_width()//2, cy-150 if lane in("north","south") else cy-ROAD_W//2-16))

        # Subtext: which vehicle is being served (id suffix)
        evid = ""
        try:
            evid = (getattr(self.zone.ctrl, "emergency_vehicle_id", "") or "")[-6:]
        except Exception:
            evid = ""
        if evid:
            f2 = pygame.font.SysFont("consolas", 10, bold=True)
            t2 = f2.render(f"Serving ID:*{evid}", True, (255, 210, 160))
            surf.blit(t2, (cx - t2.get_width()//2, (cy-134 if lane in("north","south") else cy-ROAD_W//2-4)))

    # ── per-lane status labels ─────────────────────────────────────────────

    def _draw_lane_labels(self, surf, cx, cy, zone, ns, ew, sig):
        fnt   = pygame.font.SysFont("consolas", 11, bold=True)
        fnt_s = pygame.font.SysFont("consolas", 10)
        # Label positions (outside stop lines)
        label_pos = {
            "north": (cx+6,        cy-STOP_PX-60),
            "south": (cx+6,        cy+STOP_PX+40),
            "east":  (cx+STOP_PX+8,cy+8),
            "west":  (cx-STOP_PX-85,cy+8),
        }
        for lane_id, lm in zone.lanes.items():
            if lane_id not in label_pos: continue
            lx, ly = label_pos[lane_id]
            sig_c  = (ew if lane_id in("east","west") else ns)
            sc     = {"GREEN":GRN_S,"YELLOW":YEL_S,"RED":RED_S}.get(sig_c,WHT)

            # Signal dot
            pygame.draw.circle(surf, sc, (lx-10, ly+5), 5)

            # Lane name + count
            cnt_txt = fnt.render(
                f"{lane_id.upper()} [{lm.vehicle_count}/{lm.max_capacity}]",True,sc)
            surf.blit(cnt_txt,(lx, ly))

            # Wait time
            wt = fnt_s.render(f"Wait:{lm.avg_wait:.1f}s",True,DIM)
            surf.blit(wt,(lx,ly+11))

            # Status/reason message
            if lm.status_msg:
                # Wrap long messages
                wrapped = textwrap.wrap(lm.status_msg, width=26)
                for i,line in enumerate(wrapped[:2]):
                    st = pygame.font.SysFont("consolas",8,bold=True).render(
                        line,True,lm.status_col)
                    surf.blit(st,(lx,ly+22+i*10))

    # ── zone header ───────────────────────────────────────────────────────

    def _draw_zone_header(self, surf, cx, cy, zone, sig):
        state = sig["state"]
        col   = {
            cfg.STATE_EMERGENCY: EMERG,
            cfg.STATE_RECOVERY:  TEAL,
            cfg.STATE_ALL_RED:   (200,100,100),
        }.get(state, zone.meta["color"])
        fnt = pygame.font.SysFont("consolas", 13, bold=True)
        lbl = fnt.render(zone.meta["label"], True, col)
        surf.blit(lbl,(cx-lbl.get_width()//2, cy-175))
        # Mode badge
        mode  = "🚨 PRIORITY MODE" if state==cfg.STATE_EMERGENCY else "⚙ OPTIMIZATION MODE"
        mc    = EMERG if state==cfg.STATE_EMERGENCY else GRN_S
        ml    = pygame.font.SysFont("consolas",9,bold=True).render(mode,True,mc)
        surf.blit(ml,(cx-ml.get_width()//2, cy-162))


# ═══════════════════════════════════════════════════════════════════════════════
#  RIGHT PANEL  (live calculations, V2I packets, comparison chart)
# ═══════════════════════════════════════════════════════════════════════════════

class RightPanel:
    def __init__(self):
        # Bigger, cleaner typography for projector readability
        self.fnt_h1   = pygame.font.SysFont("consolas", 18, bold=True)
        self.fnt_h2   = pygame.font.SysFont("consolas", 14, bold=True)
        self.fnt_body = pygame.font.SysFont("consolas", 13)
        self.fnt_sm   = pygame.font.SysFont("consolas", 12)
        self.fnt_xsm  = pygame.font.SysFont("consolas", 11)
        self.show_v2i = True
        self._v2i_scroll = 0
        self.v2i_packets: List[dict] = []

    def _t(self, surf, txt, x, y, col=None, fnt=None, bold=False):
        col = col or WHT; fnt = fnt or self.fnt_body
        s   = fnt.render(str(txt), True, col)
        surf.blit(s,(x,y)); return s.get_height()+1

    def _hline(self, surf, x, y, w, col=PBORD):
        pygame.draw.line(surf,col,(x,y),(x+w,y))

    def _bar(self, surf, x, y, w, h, ratio, fc, bc=(22,28,50)):
        pygame.draw.rect(surf,bc,(x,y,w,h),border_radius=3)
        fw=max(0,int(w*min(ratio,1)))
        if fw: pygame.draw.rect(surf,fc,(x,y,fw,h),border_radius=3)

    def draw(self, surf: pygame.Surface,
             zone: "IntersectionZone",
             all_zones: Dict[str,"IntersectionZone"],
             sim_time: float, sim_speed: float,
             show_v2i: bool, show_comparison: bool,
             paused: bool):

        PX = SW - 480   # panel left edge
        W  = 478

        pygame.draw.rect(surf,(10,13,26),(PX,0,W,SH))
        pygame.draw.line(surf,ACCENT,(PX,0),(PX,SH),2)

        y = 6
        # ── Title ──────────────────────────────────────────────────────────
        pygame.draw.rect(surf,(16,22,48),(PX,y,W,28),border_radius=4)
        self._t(surf,"LIFE-LINK  LIVE DASHBOARD",PX+8,y+6,(90,160,255),self.fnt_h1)
        y += 32

        info = (f"t={sim_time:.1f}s  spd={sim_speed:.1f}×"
                + ("  ⏸PAUSED" if paused else ""))
        self._t(surf,info,PX+6,y,DIM,self.fnt_sm); y+=14
        self._hline(surf,PX+4,y,W-8); y+=5

        # ── Zone mini-cards ────────────────────────────────────────────────
        self._t(surf,"ALL ZONES",PX+4,y,(80,120,200),self.fnt_h2); y+=14
        card_h = 58
        for zid, z in all_zones.items():
            sig   = z.get_sig()
            is_em = sig["state"]==cfg.STATE_EMERGENCY
            is_sel= (zid == zone.zone_id)
            bc    = (28,38,72) if is_sel else (16,20,42)
            bord  = ACCENT if is_sel else (EMERG if is_em else PBORD)
            pygame.draw.rect(surf,bc,(PX+4,y,W-8,card_h),border_radius=5)
            pygame.draw.rect(surf,bord,(PX+4,y,W-8,card_h),1,border_radius=5)
            ix = PX+10; iy = y+4

            # Zone label
            lc = z.meta["color"]
            self._t(surf,z.meta["label"],ix,iy,lc,self.fnt_h2)
            iy+=13

            # NS / EW badges
            for sig_name,sig_val in [("NS",sig["NS"]),("EW",sig["EW"])]:
                sc={"GREEN":GRN_S,"YELLOW":YEL_S,"RED":RED_S}.get(sig_val,WHT)
                self._t(surf,f"{sig_name}:{sig_val}",ix,iy,sc,self.fnt_xsm)
                ix+=60
            ix=PX+10; iy+=11

            # Stats row
            total_v = sum(lm.vehicle_count for lm in z.lanes.values())
            avg_w   = sum(lm.avg_wait for lm in z.lanes.values())/max(len(z.lanes),1)
            ag      = sig.get("adaptive_green",30)
            mode_s  = "PRIO" if is_em else "OPT"
            mc      = EMERG if is_em else GRN_S
            self._t(surf,
                f"Veh:{total_v}  Wait:{avg_w:.1f}s  AG:{ag:.1f}s  [{mode_s}]",
                ix,iy,mc,self.fnt_xsm)
            iy+=11

            # Emergency details (lane + tracked vehicle id suffix)
            if is_em and getattr(z.ctrl, "emergency_lane", None):
                el = (z.ctrl.emergency_lane or "").upper()
                evid = (getattr(z.ctrl, "emergency_vehicle_id", "") or "")[-6:]
                self._t(surf, f"EMG: {el}  ID:*{evid}", ix, iy, EMERG_G, self.fnt_xsm)

            # Preemptions + adaptive bar
            self._bar(surf,ix,iy,W-24,5,ag/cfg.MAX_GREEN_TIME,(40,190,70))
            iy+=7
            self._t(surf,f"Preemptions:{z.total_preemptions}  State:{sig['state']}",
                    ix,iy,DIM,self.fnt_xsm)

            y += card_h+3

        self._hline(surf,PX+4,y,W-8); y+=5

        # ── Selected zone detail ───────────────────────────────────────────
        sig = zone.get_sig()
        self._t(surf,f"ZONE {zone.zone_id} — LIVE CALCULATIONS",
                PX+4,y,zone.meta["color"],self.fnt_h2); y+=14

        # Per-lane live maths
        for lane_id, lm in zone.lanes.items():
            # Find closest vehicle in this lane
            closest = None; bd_ = 9999
            for v in lm.vehicles:
                if not v.cleared and v.world_dist < bd_:
                    bd_=v.world_dist; closest=v

            sig_c = sig["NS"] if lane_id in("north","south") else sig["EW"]
            sc    = {"GREEN":GRN_S,"YELLOW":YEL_S,"RED":RED_S}.get(sig_c,WHT)

            pygame.draw.rect(surf,(16,22,40),(PX+4,y,W-8,52),border_radius=4)
            pygame.draw.rect(surf,sc,(PX+4,y,W-8,52),1,border_radius=4)
            iy = y+3

            self._t(surf,f"  {lane_id.upper()}  Signal:{sig_c}  "
                    f"Queue:{lm.vehicle_count}  Wait:{lm.avg_wait:.1f}s",
                    PX+8,iy,sc,self.fnt_sm); iy+=12

            if closest:
                v  = closest
                eta_val = calculate_eta(v.world_dist, max(v.world_vel,0.1), v.world_acc)
                bd_val  = braking_distance(v.world_vel, cfg.BRAKING_DECEL)
                eta_str = f"{eta_val:.1f}s" if eta_val!=float("inf") else "∞"
                prio_str= " 🚨PRIO" if v.priority else ""
                self._t(surf,
                    f"  {v.vtype.upper()}{prio_str}  d={v.world_dist:.0f}m  "
                    f"v={v.world_vel:.1f}m/s",
                    PX+8,iy,DIM,self.fnt_xsm); iy+=11
                self._t(surf,
                    f"  ETA=(-v+√(v²+2ad))/a = {eta_str}   BrkDist={bd_val:.0f}m",
                    PX+8,iy,(140,200,255),self.fnt_xsm)
            else:
                self._t(surf,"  No vehicles in detection zone",PX+8,iy,DIM,self.fnt_xsm)

            y += 55

        self._hline(surf,PX+4,y,W-8); y+=4

        # ── COMPARISON CHART (Smart vs Fixed) ─────────────────────────────
        if show_comparison:
            chart_h = 90
            # Only draw if enough vertical space remains
            if y + chart_h + 40 < SH - 100:
                self._t(surf,"SMART vs FIXED-TIMER  (live rolling avg)",
                        PX+4,y,(80,140,255),self.fnt_h2); y+=13
                self._draw_comparison(surf,all_zones,PX+4,y,W-8,chart_h)
                y += chart_h + 6

        self._hline(surf,PX+4,y,W-8); y+=4

        # ── V2I PACKET TICKER ─────────────────────────────────────────────
        if show_v2i and self.v2i_packets:
            self._t(surf,"V2I PACKET STREAM (live broadcast)",
                    PX+4,y,(80,200,160),self.fnt_h2); y+=13
            max_show = min(6, (SH - y - 20) // 22)
            for pkt in self.v2i_packets[:max_show]:
                pc = EMERG if pkt.get("prio")==1 else (140,200,255)
                lane_s = pkt.get("lane","?")[:1].upper()
                eta_s  = f"{pkt['eta']:.1f}s" if pkt.get("eta") else "∞"
                line   = (f"  [{lane_s}] {pkt['type'].upper():<9} "
                          f"d={pkt.get('dist',0):5.0f}m  "
                          f"v={pkt.get('vel',0):4.1f}m/s  "
                          f"ETA={eta_s:<6} "
                          f"{'🚨' if pkt.get('prio')==1 else '  '}")
                self._t(surf,line,PX+4,y,pc,self.fnt_xsm); y+=12

        # ── Event log ─────────────────────────────────────────────────────
        if y < SH-80:
            self._hline(surf,PX+4,y,W-8); y+=4
            self._t(surf,"SYSTEM EVENT LOG",PX+4,y,(80,120,180),self.fnt_h2); y+=12
            for etype,emsg in list(zone.event_log)[:min(5,(SH-y-8)//12)]:
                ec = {"emerg":EMERG,"ok":GRN_S,"info":(140,180,255)}.get(etype,DIM)
                self._t(surf,emsg,PX+4,y,ec,self.fnt_xsm); y+=12

        # Footer
        self._hline(surf,PX+4,SH-18,W-8,(25,32,60))
        self._t(surf,
            "Saumya Sharma 23102156  ·  Pulkit Pandey 23102211  |  JIIT Noida 2026",
            PX+4,SH-14,DIM,self.fnt_xsm)

        # Mini-map (connector density) if CitySim attached
        city = getattr(self, "city", None)
        if city is not None:
            self._draw_minimap(surf, city, PX+10, SH-170, 220, 130)

    def _draw_minimap(self, surf, city, x, y, w, h):
        pygame.draw.rect(surf, (12, 16, 34), (x, y, w, h), border_radius=6)
        pygame.draw.rect(surf, PBORD, (x, y, w, h), 1, border_radius=6)
        fnt = pygame.font.SysFont("consolas", 10, bold=True)
        surf.blit(fnt.render("CITY MINI-MAP", True, (120, 170, 255)), (x+8, y+6))

        # Map zone coords into minimap box
        zs = city.zones
        pts = {}
        xs = [z.cx for z in zs.values()]
        ys = [z.cy for z in zs.values()]
        if not xs or not ys:
            return
        minx, maxx = min(xs), max(xs)
        miny, maxy = min(ys), max(ys)
        sx = (w - 40) / max(1.0, (maxx - minx))
        sy = (h - 40) / max(1.0, (maxy - miny))
        for name, z in zs.items():
            px = x + 20 + (z.cx - minx) * sx
            py = y + 28 + (z.cy - miny) * sy
            pts[name] = (int(px), int(py))

        # Edge densities
        dens = {}
        for src, dst, _v in city.connector_vehicles:
            dens[(src, dst)] = dens.get((src, dst), 0) + 1

        for (src, dst), path in city.connectors.items():
            if src not in pts or dst not in pts:
                continue
            c = dens.get((src, dst), 0)
            lw = 1 + min(6, c // 2)
            col = (255, 140, 80) if c > 6 else (120, 160, 220)
            pygame.draw.line(surf, col, pts[src], pts[dst], lw)

        # Nodes
        for name, (px, py) in pts.items():
            pygame.draw.circle(surf, (60, 140, 255), (px, py), 7)
            pygame.draw.circle(surf, (20, 24, 40), (px, py), 7, 1)
            surf.blit(pygame.font.SysFont("consolas", 9, bold=True).render(name[0], True, (240,240,240)),
                      (px-3, py-6))

    def _draw_comparison(self, surf, all_zones, x, y, W, H):
        """
        Inline Pygame bar chart: Smart vs Fixed per zone, with gain %.
        """
        pygame.draw.rect(surf,(12,16,34),(x,y,W,H),border_radius=5)
        pygame.draw.rect(surf,PBORD,(x,y,W,H),1,border_radius=5)

        zones     = list(all_zones.items())
        n         = len(zones)
        slot_w    = (W-30) // n
        bar_w     = slot_w // 3
        max_wait  = 35.0
        base_y    = y + H - 22
        chart_h   = H - 38

        # Gridlines
        for gv in [10,20,30]:
            gy = base_y - int((gv/max_wait)*chart_h)
            pygame.draw.line(surf,(28,36,62),(x+20,gy),(x+W-5,gy))
            self._t(surf,str(gv),x+2,gy-5,DIM,self.fnt_xsm)

        for i,(zid,z) in enumerate(zones):
            # Smart (rolling avg)
            sw = (sum(z.smart_wait_history)/len(z.smart_wait_history)
                  if z.smart_wait_history else 0)
            fw = (sum(z.fixed_wait_history)/len(z.fixed_wait_history)
                  if z.fixed_wait_history else sw*1.6+4)
            bx = x + 22 + i*slot_w

            sh = int((sw/max_wait)*chart_h)
            fh = int((fw/max_wait)*chart_h)

            # Fixed bar (red, behind)
            pygame.draw.rect(surf,(180,45,45),(bx+bar_w+2,base_y-fh,bar_w,fh),border_radius=2)
            # Smart bar (green, front)
            pygame.draw.rect(surf,(35,190,65),(bx,base_y-sh,bar_w,sh),border_radius=2)

            # Values
            self._t(surf,f"{fw:.1f}",bx+bar_w+2,base_y-fh-12,(200,60,60),self.fnt_xsm)
            self._t(surf,f"{sw:.1f}",bx,base_y-sh-12,(50,200,80),self.fnt_xsm)

            # Gain
            gain = (fw-sw)/fw*100 if fw>0 else 0
            gc   = GRN_S if gain>0 else EMERG
            self._t(surf,f"+{gain:.0f}%",bx,base_y+3,gc,self.fnt_xsm)
            self._t(surf,zid[:3],bx,base_y+14,DIM,self.fnt_xsm)

        # Legend
        pygame.draw.rect(surf,(35,190,65),(x+W-120,y+4,8,7))
        self._t(surf,"Smart",x+W-110,y+3,(50,200,80),self.fnt_xsm)
        pygame.draw.rect(surf,(180,45,45),(x+W-60,y+4,8,7))
        self._t(surf,"Fixed",x+W-50,y+3,(200,60,60),self.fnt_xsm)


# ═══════════════════════════════════════════════════════════════════════════════
#  BOTTOM STRIP — per-lane reason banners across the full width
# ═══════════════════════════════════════════════════════════════════════════════

def draw_bottom_strip(surf: pygame.Surface, zone: IntersectionZone,
                      sim_time: float):
    """Draws a bottom banner strip showing per-lane real-time status."""
    strip_h = 70
    strip_y = SH - strip_h
    pygame.draw.rect(surf,(10,14,28),(0,strip_y,SW-480,strip_h))
    pygame.draw.line(surf,PBORD,(0,strip_y),(SW-480,strip_y),1)

    sig = zone.get_sig()
    lanes = zone.lanes_list
    slot_w = (SW-480) // max(len(lanes),1)
    fnt_h  = pygame.font.SysFont("consolas",12,bold=True)
    fnt_b  = pygame.font.SysFont("consolas",11)
    fnt_sm = pygame.font.SysFont("consolas",10)

    for i, lane_id in enumerate(lanes):
        lm   = zone.lanes[lane_id]
        lx   = i * slot_w
        sig_c = sig["NS"] if lane_id in("north","south") else sig["EW"]
        sc    = {"GREEN":GRN_S,"YELLOW":YEL_S,"RED":RED_S}.get(sig_c,WHT)

        # Lane block background
        bg = (18,30,18) if sig_c=="GREEN" else (30,18,18) if sig_c=="RED" else (30,28,12)
        pygame.draw.rect(surf, bg, (lx, strip_y, slot_w, strip_h))
        pygame.draw.line(surf,PBORD,(lx,strip_y),(lx,strip_y+strip_h),1)

        # Signal dot
        pygame.draw.circle(surf, sc, (lx+10, strip_y+12), 6)

        # Lane name + signal
        t = fnt_h.render(f"{lane_id.upper()}  {sig_c}", True, sc)
        surf.blit(t,(lx+20, strip_y+4))

        # Vehicles + wait
        vc   = lm.vehicle_count
        wt   = lm.avg_wait
        has_e= lm.has_emergency
        info = fnt_b.render(
            f"Vehicles: {vc}/{lm.max_capacity}   AvgWait: {wt:.1f}s"
            + ("  🚨EMG" if has_e else ""), True,
            (EMERG_G if has_e else DIM))
        surf.blit(info,(lx+6, strip_y+20))

        # Status reason
        if lm.status_msg:
            sm = fnt_sm.render(lm.status_msg[:42], True, lm.status_col)
            surf.blit(sm,(lx+6, strip_y+34))
        else:
            # Adaptive green info
            ag = sig.get("adaptive_green",30)
            pt = sig.get("phase_timer",0)
            sm = fnt_sm.render(
                f"AdaptGreen={ag:.1f}s  PhaseTimer={pt:.1f}s", True, DIM)
            surf.blit(sm,(lx+6, strip_y+34))

        # V2I packet mini-display (closest vehicle ETA)
        closest = None; bd_ = 9999
        for v in lm.vehicles:
            if not v.cleared and v.world_dist < bd_:
                bd_=v.world_dist; closest=v
        if closest:
            eta_v = calculate_eta(closest.world_dist,
                                  max(closest.world_vel,0.1),
                                  closest.world_acc)
            eta_s = f"{eta_v:.1f}s" if eta_v!=float("inf") else "∞"
            et = fnt_sm.render(
                f"V2I: {closest.vtype[:3].upper()} d={closest.world_dist:.0f}m ETA={eta_s}",
                True,(100,180,255))
            surf.blit(et,(lx+slot_w-130, strip_y+34))


class CitySim:
    """
    Multi-intersection city: vehicles can travel between intersections along connectors.
    Intersections remain independent controllers; connectors are free-flow and overlap-free.
    """

    def __init__(self, zones: Dict[str, IntersectionZone]) -> None:
        self.zones = zones
        self.connectors: Dict[tuple[str, str], LanePath] = {}
        self.connector_vehicles: List[tuple[str, str, VehicleEntity]] = []
        self._vehicle_dest: Dict[str, str] = {}  # vehicle_id -> destination zone
        self._build_connectors()

    def _build_connectors(self) -> None:
        # Build a directed network for the 3 presentation zones
        if not all(k in self.zones for k in ("Alpha", "Beta", "Gamma")):
            return
        lane_off = 18.0
        pad = 120.0  # keep connectors away from intersection centers

        def add(u: str, v: str, off: float):
            zu = self.zones[u]; zv = self.zones[v]
            # Use L-shaped (polyline) connectors so "slant" traffic never crosses.
            # Route via a waypoint that stays away from the center line.
            if abs(zv.cx - zu.cx) > abs(zv.cy - zu.cy):
                wp = (zv.cx, zu.cy + (pad if zv.cy > zu.cy else -pad))
            else:
                wp = (zu.cx + (pad if zv.cx > zu.cx else -pad), zv.cy)
            self.connectors[(u, v)] = PolyConnectorPath(
                name=f"{u}->{v}",
                x0=zu.cx,
                y0=zu.cy,
                x1=zv.cx,
                y1=zv.cy,
                waypoint=wp,
                lane_offset=off,
            )

        add("Alpha", "Beta", +lane_off)
        add("Beta", "Alpha", -lane_off)
        add("Alpha", "Gamma", -lane_off)
        add("Gamma", "Alpha", +lane_off)
        add("Beta", "Gamma", +lane_off)
        add("Gamma", "Beta", -lane_off)

    def _pick_destination(self, current_zone: str) -> str:
        # Pick a destination different from current zone
        choices = [z for z in self.zones.keys() if z != current_zone]
        return random.choice(choices) if choices else current_zone

    def _next_hop_toward(self, current: str, dest: str) -> Optional[str]:
        # For 3-node graph, shortest path is either direct edge or via the third node.
        if (current, dest) in self.connectors:
            return dest
        # pick any neighbor that has a direct edge to dest
        for (u, v) in self.connectors.keys():
            if u == current and (v, dest) in self.connectors:
                return v
        # fallback: any neighbor
        neighbors = [v for (u, v) in self.connectors.keys() if u == current]
        return random.choice(neighbors) if neighbors else None

    def _dest_entry_lane(self, src: str, dst: str) -> str:
        z1 = self.zones[src]; z2 = self.zones[dst]
        dx = z2.cx - z1.cx
        dy = z2.cy - z1.cy
        if abs(dx) > abs(dy):
            return "west" if dx > 0 else "east"
        return "north" if dy > 0 else "south"

    def _spawn_on_connector(self, src: str, dst: str, vtype: str) -> None:
        path = self.connectors.get((src, dst))
        if not path:
            return
        v = VehicleEntity(lane=f"{src}->{dst}", vtype=vtype, path=path, s0=0.0)
        v.v = min(140.0, v.v_des * 0.7)
        # Assign destination if not already assigned
        if v.uid not in self._vehicle_dest:
            self._vehicle_dest[v.uid] = self._pick_destination(src)
        self.connector_vehicles.append((src, dst, v))

    def step(self, dt: float) -> None:
        # Step intersections
        for z in self.zones.values():
            z.step(dt)

        # Convert exiting vehicles into connector vehicles
        for src, z in self.zones.items():
            # Choose next hop toward each vehicle's destination (destination-based routing)
            for lm in z.lanes.values():
                if not lm.vehicles:
                    continue
                front = max(lm.vehicles, key=lambda vv: vv.s)
                if front.s >= front.path.s_exit + 80 and not front.cleared:
                    vtype = front.vtype
                    dest = self._vehicle_dest.get(front.uid) or self._pick_destination(src)
                    self._vehicle_dest[front.uid] = dest
                    nxt = self._next_hop_toward(src, dest)
                    if not nxt:
                        continue
                    front.cleared = True  # removed next lane update
                    self._spawn_on_connector(src, nxt, vtype)

        # Step connector vehicles (grouped per edge for headway)
        by_edge: Dict[tuple[str, str], List[VehicleEntity]] = {}
        for src, dst, v in self.connector_vehicles:
            by_edge.setdefault((src, dst), []).append(v)

        new_list: List[tuple[str, str, VehicleEntity]] = []
        for (src, dst), vs in by_edge.items():
            vs.sort(key=lambda vv: vv.s, reverse=True)
            for i, v in enumerate(vs):
                lead = vs[i - 1] if i > 0 else None
                v.update(dt, signal="GREEN", lead=lead, allow_enter_intersection=True)
                if v.cleared or v.s >= v.path.length - 1.0:
                    lane_in = self._dest_entry_lane(src, dst)
                    if lane_in in self.zones[dst].lanes:
                        self.zones[dst].inject_vehicle(lane_in, v.vtype)
                    continue
                new_list.append((src, dst, v))
        self.connector_vehicles = new_list

    def draw_connectors(self, surf: pygame.Surface) -> None:
        """
        Draw connector roads as proper asphalt (not black lines).
        Uses the connector polyline segments and paints a thick road with
        subtle kerbs + dashed center markings.
        """
        road_w = 34
        kerb_w = road_w + 10

        dash_len = 14
        gap_len = 10

        def draw_segment(p0: tuple[float, float], p1: tuple[float, float]) -> None:
            x0, y0 = p0
            x1, y1 = p1
            dx = x1 - x0
            dy = y1 - y0
            L = max(1.0, math.hypot(dx, dy))
            nx = -dy / L
            ny = dx / L

            # Kerb (slightly wider, darker edge)
            k0 = (x0 + nx * kerb_w / 2, y0 + ny * kerb_w / 2)
            k1 = (x0 - nx * kerb_w / 2, y0 - ny * kerb_w / 2)
            k2 = (x1 - nx * kerb_w / 2, y1 - ny * kerb_w / 2)
            k3 = (x1 + nx * kerb_w / 2, y1 + ny * kerb_w / 2)
            pygame.draw.polygon(
                surf,
                (55, 55, 55),
                [(int(k0[0]), int(k0[1])), (int(k1[0]), int(k1[1])),
                 (int(k2[0]), int(k2[1])), (int(k3[0]), int(k3[1]))],
            )

            # Asphalt
            a0 = (x0 + nx * road_w / 2, y0 + ny * road_w / 2)
            a1 = (x0 - nx * road_w / 2, y0 - ny * road_w / 2)
            a2 = (x1 - nx * road_w / 2, y1 - ny * road_w / 2)
            a3 = (x1 + nx * road_w / 2, y1 + ny * road_w / 2)
            pygame.draw.polygon(
                surf,
                ROAD,
                [(int(a0[0]), int(a0[1])), (int(a1[0]), int(a1[1])),
                 (int(a2[0]), int(a2[1])), (int(a3[0]), int(a3[1]))],
            )

            # Subtle edge line
            pygame.draw.line(surf, (70, 70, 80), (int(a0[0]), int(a0[1])), (int(a3[0]), int(a3[1])), 2)
            pygame.draw.line(surf, (70, 70, 80), (int(a1[0]), int(a1[1])), (int(a2[0]), int(a2[1])), 2)

            # Dashed center marking
            ux = dx / L
            uy = dy / L
            t = 0.0
            while t < L:
                t1 = min(L, t + dash_len)
                sx = x0 + ux * t
                sy = y0 + uy * t
                ex = x0 + ux * t1
                ey = y0 + uy * t1
                pygame.draw.line(surf, STRIPE, (int(sx), int(sy)), (int(ex), int(ey)), 2)
                t += dash_len + gap_len

        for (_src, _dst), path in self.connectors.items():
            if isinstance(path, PolyConnectorPath):
                for p0, p1 in path.segments():
                    draw_segment(p0, p1)
            else:
                # Fallback: render straight connector as a thick road
                p0 = (getattr(path, "_x0", 0.0), getattr(path, "_y0", 0.0))
                p1 = (getattr(path, "_x1", 0.0), getattr(path, "_y1", 0.0))
                draw_segment(p0, p1)

    def draw_connector_vehicles(self, surf: pygame.Surface) -> None:
        for _, _, v in self.connector_vehicles:
            v.draw(surf)


# ═══════════════════════════════════════════════════════════════════════════════
#  COMPARISON MODE — side-by-side split screen
# ═══════════════════════════════════════════════════════════════════════════════

class ComparisonRenderer:
    """
    Draws side-by-side: Smart (left) vs Fixed-Timer (right).
    The fixed-timer side shows vehicles waiting longer with a visible timer.
    """

    def __init__(self):
        self.fnt_h = pygame.font.SysFont("consolas",13,bold=True)
        self.fnt_b = pygame.font.SysFont("consolas",10)
        self.fnt_s = pygame.font.SysFont("consolas", 9)
        self._fixed_phase_timer = 0.0
        self._fixed_ns_green    = True

    def draw(self, surf: pygame.Surface, zone: IntersectionZone,
             sim_time: float):
        half = (SW-480)//2
        # Divider
        pygame.draw.line(surf,(80,80,120),(half,0),(half,SH-58),2)

        # Left label
        self._label(surf, half//2, 14, "⚙  LIFE-LINK  SMART  (Adaptive V2I)",
                    GRN_S, self.fnt_h)
        self._label(surf, half+half//2, 14, "⏱  CONVENTIONAL  FIXED-TIMER  (No V2I)",
                    RED_S, self.fnt_h)

        # Render zone normally on left
        # (Already rendered by main loop)

        # Right side: fixed-timer overlay
        self._draw_fixed_side(surf, zone, half, sim_time)

    def _label(self, surf, cx, y, txt, col, fnt):
        t = fnt.render(txt, True, col)
        surf.blit(t,(cx-t.get_width()//2, y))

    def _draw_fixed_side(self, surf, zone: IntersectionZone,
                          x_off: int, sim_time: float):
        """
        Clone the intersection on the right with fixed-timer signals.
        Show vehicles queuing longer, wasted time counter.
        """
        cx = x_off + (SW-480-x_off)//2
        cy = SH//2 - 29    # centre vertically (above bottom strip)

        # Draw roads (re-use renderer but offset)
        r = ZoneRenderer(zone)
        # We can't easily offset so draw a simple schematic
        sig = zone.get_sig()
        # Fixed state
        if not hasattr(self,'_ft'): self._ft=0.0
        # Use zone's fixed simulation
        ns_fixed = zone._get_fixed_signal("north")
        ew_fixed = zone._get_fixed_signal("east")

        # Draw lanes with fixed signal colours
        fnt_l = pygame.font.SysFont("consolas",10,bold=True)
        fnt_s = pygame.font.SysFont("consolas", 9)

        lanes = zone.lanes_list
        slot_h = 80
        start_y = cy - (len(lanes)*slot_h)//2

        for i,lane_id in enumerate(lanes):
            lm    = zone.lanes[lane_id]
            fixed_s = zone._get_fixed_signal(lane_id)
            sc      = {"GREEN":GRN_S,"YELLOW":YEL_S,"RED":RED_S}.get(fixed_s,WHT)
            smart_s = sig["NS"] if lane_id in("north","south") else sig["EW"]
            ssc     = {"GREEN":GRN_S,"YELLOW":YEL_S,"RED":RED_S}.get(smart_s,WHT)

            ly = start_y + i*slot_h

            # Lane row
            bg = (18,30,18) if fixed_s=="GREEN" else (30,14,14)
            pygame.draw.rect(surf,bg,(x_off+8,ly,SW-480-x_off-16,slot_h-4),
                             border_radius=5)
            pygame.draw.rect(surf,sc,(x_off+8,ly,SW-480-x_off-16,slot_h-4),
                             1,border_radius=5)

            pygame.draw.circle(surf,sc,(x_off+22,ly+22),7)
            lbl = fnt_l.render(f"{lane_id.upper()}  FIXED: {fixed_s}",True,sc)
            surf.blit(lbl,(x_off+33,ly+14))

            # Wasted time estimate
            fw = zone._fixed_wait_acc.get(lane_id,0)
            wasted = fw / max(lm.total_cleared+1,1)
            smart_w= lm.avg_wait
            diff   = max(0, wasted - smart_w)
            info1  = fnt_s.render(
                f"  Fixed Wait: ~{wasted:.1f}s   Smart Wait: {smart_w:.1f}s",True,DIM)
            surf.blit(info1,(x_off+8,ly+32))

            # Wasted time highlight
            if diff > 0.5:
                waste_t = fnt_s.render(
                    f"  ⚠ Wasted: {diff:.1f}s/veh  ×{lm.vehicle_count} = "
                    f"{diff*lm.vehicle_count:.1f}s TOTAL",
                    True, (255,120,60))
                surf.blit(waste_t,(x_off+8,ly+44))
            else:
                eff_t = fnt_s.render("  ✅ Life-Link is clearing efficiently",True,GRN_S)
                surf.blit(eff_t,(x_off+8,ly+44))

        # Total stats
        total_fixed_wait  = sum(zone._fixed_wait_acc.values())
        total_smart_wait  = sum(lm.avg_wait for lm in zone.lanes.values())
        total_gain        = total_fixed_wait - total_smart_wait
        fy = start_y + len(lanes)*slot_h + 6
        pygame.draw.rect(surf,(14,30,14),(x_off+8,fy,SW-480-x_off-16,40),
                         border_radius=5)
        pygame.draw.rect(surf,GRN_S,(x_off+8,fy,SW-480-x_off-16,40),1,border_radius=5)
        fnt_m = pygame.font.SysFont("consolas",10,bold=True)
        t1 = fnt_m.render(
            f"  LIFE-LINK SAVING: {max(total_gain,0):.1f}s cumulative "
            f"across {sum(lm.total_cleared for lm in zone.lanes.values())} vehicles",
            True, GRN_S)
        surf.blit(t1,(x_off+8,fy+6))
        eff = (total_gain/max(total_fixed_wait,0.01))*100
        t2 = fnt_m.render(
            f"  Efficiency Gain vs Fixed-Timer: {max(eff,0):.1f}%   "
            f"Preemptions: {zone.total_preemptions}",
            True, TEAL)
        surf.blit(t2,(x_off+8,fy+22))


# ═══════════════════════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    pygame.init()
    pygame.display.set_caption(
        "Life-Link — Smart Traffic System  |  Saumya & Pulkit  |  JIIT Noida")
    screen = pygame.display.set_mode((SW, SH))
    clock  = pygame.time.Clock()

    os.makedirs(os.path.join(ROOT,"logs"),   exist_ok=True)
    os.makedirs(os.path.join(ROOT,"output"), exist_ok=True)
    logger = CSVLogger(os.path.join(ROOT,"logs","presentation_log.csv"))

    # Seed for reproducible demo
    random.seed(42)

    # Create 3 zones at well-spaced positions in the left panel (SW-480 wide)
    panel_w = SW - 480
    zones: Dict[str, IntersectionZone] = {
        "Alpha": IntersectionZone("Alpha", panel_w//4,       SH//3,       logger),
        "Beta":  IntersectionZone("Beta",  panel_w*3//4,     SH//3,       logger),
        "Gamma": IntersectionZone("Gamma", panel_w//2,       SH*2//3-20,  logger),
    }
    renderers = {zid: ZoneRenderer(z) for zid,z in zones.items()}
    panel     = RightPanel()
    comp_rend = ComparisonRenderer()
    city      = CitySim(zones)
    panel.city = city

    selected_zone = "Alpha"
    sim_speed     = 1.0
    paused        = False
    sim_time      = 0.0
    show_v2i      = True
    show_comp     = False
    json_t        = 0.0
    analysis_busy = False

    print("\n" + "="*65)
    print("  LIFE-LINK  PRESENTATION MODE  —  3 Zones Active")
    print("  TAB/1-3: Zone  E: Ambulance  C: Comparison  V: V2I")
    print("  SPACE: Pause  +/-: Speed  A: Save Analysis  Q: Quit")
    print("="*65)

    running = True
    while running:
        real_dt = clock.tick(FPS) / 1000.0
        dt = min(real_dt * sim_speed, 0.08)

        # ── events ────────────────────────────────────────────────────────
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type == pygame.KEYDOWN:
                k = ev.key
                if k in (pygame.K_ESCAPE, pygame.K_q): running = False
                elif k == pygame.K_SPACE:
                    paused = not paused
                    print("  ⏸ PAUSED" if paused else "  ▶ RESUMED")
                elif k == pygame.K_TAB:
                    zl = list(zones.keys())
                    selected_zone = zl[(zl.index(selected_zone)+1) % len(zl)]
                    print(f"  Zone → {selected_zone}")
                elif k == pygame.K_1: selected_zone="Alpha"; print("  Zone → Alpha")
                elif k == pygame.K_2: selected_zone="Beta";  print("  Zone → Beta")
                elif k == pygame.K_3: selected_zone="Gamma"; print("  Zone → Gamma")
                elif k == pygame.K_e:
                    zones[selected_zone].spawn_ambulance()
                elif k == pygame.K_c:
                    show_comp = not show_comp
                    print(f"  Comparison mode: {'ON' if show_comp else 'OFF'}")
                elif k == pygame.K_v:
                    show_v2i = not show_v2i
                    print(f"  V2I display: {'ON' if show_v2i else 'OFF'}")
                elif k in (pygame.K_PLUS,pygame.K_EQUALS):
                    sim_speed = min(sim_speed*1.5, 8.0)
                    print(f"  Speed → {sim_speed:.1f}×")
                elif k == pygame.K_MINUS:
                    sim_speed = max(sim_speed/1.5, 0.25)
                    print(f"  Speed → {sim_speed:.1f}×")
                elif k == pygame.K_a:
                    if not analysis_busy and HAS_MPL:
                        analysis_busy = True
                        def _save_analysis():
                            global analysis_busy
                            _generate_analysis_png(zones)
                            analysis_busy = False
                        threading.Thread(target=_save_analysis,daemon=True).start()

        # ── simulation step ───────────────────────────────────────────────
        if not paused:
            sim_time += dt
            city.step(dt)

        # ── render ────────────────────────────────────────────────────────
        screen.fill(BG)

        # Zone boundaries (light dividers)
        pygame.draw.line(screen,(20,28,55),(SW-480,0),(SW-480,SH),2)

        # City-like main-road links between intersections (visual network)
        # These are pure visuals; traffic logic remains per-intersection.
        panel_w = SW - 480
        link_col = (45, 45, 52)
        link_edge = (70, 70, 80)
        # Horizontal arterial between Alpha and Beta
        pygame.draw.rect(screen, link_col, (panel_w//4, SH//3 - ROAD_W//2, panel_w//2, ROAD_W))
        pygame.draw.rect(screen, link_edge, (panel_w//4, SH//3 - ROAD_W//2, panel_w//2, ROAD_W), 2)
        # Vertical arterial from Alpha/Beta down to Gamma
        pygame.draw.rect(screen, link_col, (panel_w//2 - ROAD_W//2, SH//3, ROAD_W, SH//3))
        pygame.draw.rect(screen, link_edge, (panel_w//2 - ROAD_W//2, SH//3, ROAD_W, SH//3), 2)

        if show_comp:
            # Split left panel: left half = smart, right half = fixed
            half = (SW-480)//2
            # Draw selected zone in left half
            z = zones[selected_zone]
            # Shift zone to left half
            orig_cx, orig_cy = z.cx, z.cy
            z.cx = half//2 + 20; z.cy = SH//2 - 50
            renderers[selected_zone].draw(screen, full=True)
            z.cx, z.cy = orig_cx, orig_cy
            comp_rend.draw(screen, zones[selected_zone], sim_time)
        else:
            # Draw all 3 zones
            for zid, zone in zones.items():
                renderers[zid].draw(screen, full=True)
            # Connector vehicles between intersections
            city.draw_connectors(screen)
            city.draw_connector_vehicles(screen)

            # Zone selector overlay (top-left corner labels)
            fnt_zt = pygame.font.SysFont("consolas",9)
            for i,(zid,z) in enumerate(zones.items()):
                is_sel = (zid==selected_zone)
                col    = ACCENT if is_sel else DIM
                bc     = (25,35,70) if is_sel else (15,18,35)
                pygame.draw.rect(screen,bc,(8+i*110,6,105,16),border_radius=3)
                if is_sel:
                    pygame.draw.rect(screen,ACCENT,(8+i*110,6,105,16),1,border_radius=3)
                t = fnt_zt.render(
                    f"[{i+1}] {zid} {'◀' if is_sel else ''}",True,col)
                screen.blit(t,(12+i*110,8))

        # Bottom strip (per-lane banners)
        draw_bottom_strip(screen, zones[selected_zone], sim_time)

        # Right panel
        v2i_pkts = zones[selected_zone].get_all_packets()
        panel.v2i_packets = v2i_pkts
        panel.draw(screen, zones[selected_zone], zones,
                   sim_time, sim_speed, show_v2i, True, paused)

        # Pause overlay
        if paused:
            ov = pygame.Surface((SW-480,SH),pygame.SRCALPHA)
            ov.fill((0,0,0,70)); screen.blit(ov,(0,0))
            pf = pygame.font.SysFont("consolas",28,bold=True)
            pt = pf.render("⏸  PAUSED — Press SPACE to resume",True,(255,240,80))
            screen.blit(pt,((SW-480)//2-pt.get_width()//2, SH//2-14))

        # Analysis saving notice
        if analysis_busy:
            af = pygame.font.SysFont("consolas",10,bold=True)
            at = af.render("📊 Saving analysis charts to output/...",True,(80,180,255))
            screen.blit(at,(6,SH-76))

        # Mode hint (top bar)
        hf = pygame.font.SysFont("consolas",9)
        hints = "  [E] Ambulance   [C] Comparison   [V] V2I Stream   [TAB] Switch Zone   [A] Save Analysis   [+/-] Speed   [SPACE] Pause"
        ht = hf.render(hints,True,DIM)
        # Don't draw over zones — put in a small transparent area
        screen.blit(ht,(4, SH-70-14))

        pygame.display.flip()

        # JSON export
        json_t += real_dt
        if json_t >= 0.5:
            json_t = 0.0
            try:
                state = {"timestamp":time.time(),"sim_time":round(sim_time,1),
                         "zones":{zid:{"state":z.get_sig()["state"],
                                       "NS":z.get_sig()["NS"],
                                       "EW":z.get_sig()["EW"]}
                                  for zid,z in zones.items()}}
                with open(os.path.join(ROOT,"output","live_state.json"),"w") as f:
                    json.dump(state,f)
            except Exception:
                pass

    pygame.quit()
    print(f"\n[Life-Link] Done  |  log: logs/presentation_log.csv")


# ═══════════════════════════════════════════════════════════════════════════════
#  ANALYSIS CHART GENERATOR
# ═══════════════════════════════════════════════════════════════════════════════

def _generate_analysis_png(zones: Dict[str, IntersectionZone]):
    if not HAS_MPL: return
    try:
        fig = plt.figure(figsize=(16,9),facecolor="#0d1117")
        gs  = gridspec.GridSpec(2,3,figure=fig,hspace=0.45,wspace=0.32,
                                left=0.06,right=0.97,top=0.91,bottom=0.07)
        D="#0d1117"; P="#161b22"; TX="#e6edf3"
        MU="#8b949e"; GN="#22c55e"; RD="#ef4444"
        BL="#60a5fa"; YL="#eab308"; OR="#f97316"

        def ax_(ax,t):
            ax.set_facecolor(P)
            for sp in ax.spines.values(): sp.set_color("#30363d")
            ax.tick_params(colors=MU,labelsize=8)
            ax.xaxis.label.set_color(MU); ax.yaxis.label.set_color(MU)
            ax.set_title(t,color=TX,fontsize=10,fontweight="bold",pad=6)
            ax.grid(alpha=0.15,color="#30363d")

        znames = list(zones.keys())
        sw_avg = [sum(z.smart_wait_history)/max(len(z.smart_wait_history),1) for z in zones.values()]
        fw_avg = [sum(z.fixed_wait_history)/max(len(z.fixed_wait_history),1) for z in zones.values()]
        prs    = [z.total_preemptions for z in zones.values()]

        # 1. Wait time comparison
        ax1=fig.add_subplot(gs[0,0]); ax_(ax1,"Avg Wait: Smart vs Fixed-Timer")
        x=np.arange(len(znames)); bw=0.35
        b1=ax1.bar(x-bw/2,fw_avg,bw,color=RD,alpha=0.85,label="Fixed-Timer")
        b2=ax1.bar(x+bw/2,sw_avg,bw,color=GN,alpha=0.85,label="Life-Link")
        ax1.set_xticks(x); ax1.set_xticklabels(znames,color=MU)
        ax1.set_ylabel("Wait (s)",color=MU); ax1.legend(fontsize=8,facecolor=D,labelcolor=MU)
        for b,v in [(b,v) for b,v in list(zip(b1,fw_avg))+list(zip(b2,sw_avg))]:
            ax1.text(b.get_x()+b.get_width()/2,v+0.2,f"{v:.1f}",ha="center",fontsize=7,color=TX)

        # 2. Efficiency gain
        ax2=fig.add_subplot(gs[0,1]); ax_(ax2,"Efficiency Gain % per Zone")
        gains=[(f-s)/f*100 if f>0 else 0 for f,s in zip(fw_avg,sw_avg)]
        cols=[GN if g>=0 else RD for g in gains]
        ax2.bar(znames,gains,color=cols,alpha=0.85)
        ax2.axhline(0,color=MU,lw=0.8); ax2.set_ylabel("Reduction %",color=MU)
        for i,g in enumerate(gains):
            ax2.text(i,g+0.4,f"{g:.1f}%",ha="center",fontsize=9,fontweight="bold",color=TX)
        avg_g=sum(gains)/len(gains) if gains else 0
        ax2.text(0.97,0.97,f"Avg: {avg_g:.1f}%",transform=ax2.transAxes,
                 ha="right",va="top",color=GN,fontsize=11,fontweight="bold",
                 bbox=dict(boxstyle="round",facecolor=D,edgecolor=GN,alpha=0.8))

        # 3. Preemptions
        ax3=fig.add_subplot(gs[0,2]); ax_(ax3,"Emergency Preemptions per Zone")
        ax3.bar(znames,prs,color=OR,alpha=0.85)
        ax3.set_ylabel("Count",color=MU)
        for i,p in enumerate(prs):
            ax3.text(i,p+0.05,str(p),ha="center",fontsize=10,fontweight="bold",color=TX)

        # 4. Braking distance
        ax4=fig.add_subplot(gs[1,0]); ax_(ax4,"Braking Distance  d=v²/(2a)")
        sp=np.linspace(0,25,200)
        ax4.plot(sp,[braking_distance(v,5) for v in sp],color=GN,lw=2,label="a=5 m/s²")
        ax4.plot(sp,[braking_distance(v,3) for v in sp],color=YL,lw=2,label="a=3 m/s²")
        ax4.plot(sp,[braking_distance(v,8) for v in sp],color=RD,lw=2,label="a=8 m/s²")
        ax4.axvline(20,color=OR,ls="--",lw=1.5,label="Ambulance 20m/s")
        ax4.set_xlabel("Speed (m/s)",color=MU); ax4.set_ylabel("Distance (m)",color=MU)
        ax4.legend(fontsize=7,facecolor=D,labelcolor=MU)

        # 5. ETA
        ax5=fig.add_subplot(gs[1,1]); ax_(ax5,"ETA = (-v+√(v²+2ad))/a")
        ds=np.linspace(1,500,300)
        ax5.plot(ds,[min(calculate_eta(d,10,0),60) for d in ds],color=BL,lw=2,label="v=10")
        ax5.plot(ds,[min(calculate_eta(d,20,0),60) for d in ds],color=RD,lw=2,label="v=20 amb")
        ax5.plot(ds,[min(calculate_eta(d,5,2),60)  for d in ds],color=GN,lw=2,label="v=5 a=+2")
        ax5.axvline(500,color=YL,ls="--",lw=1.5,label="500m zone")
        ax5.axhline(3,color=MU,ls=":",lw=1,label="Yellow 3s")
        ax5.set_xlim(0,540); ax5.set_ylim(0,65)
        ax5.set_xlabel("Distance (m)",color=MU); ax5.set_ylabel("ETA (s)",color=MU)
        ax5.legend(fontsize=7,facecolor=D,labelcolor=MU)

        # 6. Speed profiles
        ax6=fig.add_subplot(gs[1,2]); ax_(ax6,"Speed Profiles  v=u+at")
        ta=np.linspace(0,16,320); dt_=ta[1]-ta[0]
        def vs(v0,a,vm):
            r,v=[],v0
            for _ in ta: r.append(v); v=min(max(v+a*dt_,0),vm)
            return r
        ax6.plot(ta,vs(0,2.0,13),color=BL,lw=2,label="Car 13m/s")
        ax6.plot(ta,vs(0,2.0,16),color=GN,lw=2,label="Bike 16m/s")
        ax6.plot(ta,vs(0,1.5, 9),color=OR,lw=2,label="Auto 9m/s")
        ax6.plot(ta,vs(0,1.2, 8),color=YL,lw=2,label="Truck 8m/s")
        ax6.plot(ta,vs(0,2.5,20),color=RD,lw=2,ls="--",label="Ambulance 20m/s")
        ax6.set_xlabel("Time (s)",color=MU); ax6.set_ylabel("Speed (m/s)",color=MU)
        ax6.legend(fontsize=7,facecolor=D,labelcolor=MU)

        fig.suptitle(
            "LIFE-LINK — Analysis Report  |  "
            "Saumya Sharma 23102156  &  Pulkit Pandey 23102211  |  JIIT Noida 2026",
            color=TX,fontsize=11,fontweight="bold",y=0.975)

        out=os.path.join(ROOT,"output","analysis_report.png")
        fig.savefig(out,dpi=130,facecolor=D)
        plt.close(fig)
        print(f"[Analysis] Saved → {out}")
    except Exception as e:
        print(f"[Analysis] Error: {e}")


if __name__ == "__main__":
    main()
