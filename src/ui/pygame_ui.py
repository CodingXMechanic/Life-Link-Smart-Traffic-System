"""
Life-Link — Pygame 2D Simulation UI (Pulkit's module)

Renders a 4-zone smart city with:
  • Realistic road + lane markings (asphalt, white dashes, stop lines)
  • Traffic signals with proper R/Y/G lenses
  • Distinct vehicle sprites: car, bike, auto, ambulance
  • Emergency green corridor glow
  • Live stats HUD per zone
  • Zone selector (1–4 keys or click panel)
  • Speed multiplier control
  • Emergency spawn button
  • Real-time JSON state export for HTML dashboard
"""
from __future__ import annotations
import json
import math
import os
import random
import sys
import time
from typing import Dict, List, Optional, Tuple

# Windows/Linux/Mac compatibility — never force x11 on Windows
import platform as _platform
if _platform.system() != "Windows":
    os.environ.setdefault("SDL_VIDEODRIVER", "x11")

import pygame
import pygame.gfxdraw

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
import src.config as cfg
from src.comm.broker import Broker
from src.controller.controller import IntersectionController
from src.logging.logger import CSVLogger
from src.vehicle.vehicle import Vehicle

# ─── Colours ─────────────────────────────────────────────────────────────────
C_BG          = (15,  20,  35)       # dark navy background
C_ROAD        = (40,  40,  40)       # asphalt
C_LANE_MARK   = (240, 240, 240)      # white lane markings
C_MEDIAN      = (255, 200,  50)      # yellow median line
C_KERB        = (80,  80,  80)       # kerb/footpath
C_GRASS       = (34,  85,  34)       # grassed corner
C_STOP_LINE   = (220, 220, 220)
C_RED         = (220,  50,  50)
C_YELLOW      = (250, 200,  30)
C_GREEN       = ( 50, 210,  80)
C_SIGNAL_BODY = (20,  20,  20)
C_SIGNAL_GLOW = (255, 100,  50)
C_TEXT        = (230, 230, 230)
C_TEXT_EMERG  = (255,  80,  80)
C_PANEL       = (20,  25,  45)
C_PANEL_BORD  = (60,  70, 110)
C_EMERG_GLOW  = (255, 150,  20, 80)   # RGBA semi-transparent
C_ZONE_ACTIVE = ( 60, 120, 220)
C_ZONE_IDLE   = ( 40,  55,  90)

# ─── Zone layout (pixel centres in 1400×900) ─────────────────────────────────
ZONE_CENTERS: Dict[str, Tuple[int,int]] = {
    "Alpha": (350,  270),
    "Beta":  (1050, 270),
    "Gamma": (350,  630),
    "Delta": (1050, 630),
}
ROAD_W  = 110    # total road width (pixels)
LANE_W  =  48    # per-lane width

# ─── Vehicle sprite sizes ─────────────────────────────────────────────────────
SPRITE_SIZES = {
    "car":       (28, 16),
    "bike":      (18, 10),
    "auto":      (22, 14),
    "ambulance": (34, 18),
}
VEHICLE_COLORS = {
    "car":       [(200, 60, 60), (60, 130, 200), (80, 180, 80),
                  (180, 120, 60), (140, 60, 180), (60, 180, 180)],
    "bike":      [(220, 220,  80)],
    "auto":      [(220, 140,  40)],
    "ambulance": [(255, 255, 255)],
}


# ═══════════════════════════════════════════════════════════════════════════════
#  Sprite drawing helpers
# ═══════════════════════════════════════════════════════════════════════════════

def _draw_car(surf: pygame.Surface, cx: int, cy: int,
              color: Tuple, w: int, h: int, angle: float = 0) -> None:
    """Draw a rounded-rect car body with windscreen."""
    body = pygame.Surface((w, h), pygame.SRCALPHA)
    pygame.draw.rect(body, color, (0, 0, w, h), border_radius=4)
    # Windscreen
    pygame.draw.rect(body, (160, 210, 240, 200), (w//4, 2, w//2, h-4), border_radius=2)
    # Wheels
    wc = (30, 30, 30)
    for wx, wy in [(2, 1), (w-6, 1), (2, h-4), (w-6, h-4)]:
        pygame.draw.rect(body, wc, (wx, wy, 4, 3))
    rotated = pygame.transform.rotate(body, math.degrees(angle))
    surf.blit(rotated, (cx - rotated.get_width()//2, cy - rotated.get_height()//2))


def _draw_bike(surf: pygame.Surface, cx: int, cy: int,
               color: Tuple, w: int, h: int, angle: float = 0) -> None:
    body = pygame.Surface((w, h), pygame.SRCALPHA)
    pygame.draw.ellipse(body, color, (0, h//4, w, h//2))
    pygame.draw.circle(body, (30,30,30), (3, h//2), 3)
    pygame.draw.circle(body, (30,30,30), (w-3, h//2), 3)
    rotated = pygame.transform.rotate(body, math.degrees(angle))
    surf.blit(rotated, (cx - rotated.get_width()//2, cy - rotated.get_height()//2))


def _draw_auto(surf: pygame.Surface, cx: int, cy: int,
               color: Tuple, w: int, h: int, angle: float = 0) -> None:
    body = pygame.Surface((w, h), pygame.SRCALPHA)
    pygame.draw.rect(body, color, (2, 0, w-4, h), border_radius=3)
    pygame.draw.rect(body, (20,20,20), (0, h//3, 4, h//3))   # side open
    pygame.draw.circle(body, (40,40,40), (4, h-2), 3)
    pygame.draw.circle(body, (40,40,40), (w-4, h-2), 3)
    rotated = pygame.transform.rotate(body, math.degrees(angle))
    surf.blit(rotated, (cx - rotated.get_width()//2, cy - rotated.get_height()//2))


def _draw_ambulance(surf: pygame.Surface, cx: int, cy: int,
                    w: int, h: int, angle: float = 0,
                    flash: bool = False) -> None:
    body = pygame.Surface((w, h), pygame.SRCALPHA)
    pygame.draw.rect(body, (240, 240, 240), (0, 0, w, h), border_radius=3)
    # Red cross
    mid_x, mid_y = w//2, h//2
    pygame.draw.rect(body, (220,40,40), (mid_x-2, mid_y-6, 4, 12))
    pygame.draw.rect(body, (220,40,40), (mid_x-6, mid_y-2, 12, 4))
    # Blue/red flash lights
    if flash:
        pygame.draw.rect(body, (50,100,255), (2, 0, w//3, 4))
        pygame.draw.rect(body, (255,50,50),  (w-2-w//3, 0, w//3, 4))
    # Wheels
    for wx, wy in [(2, 1), (w-6, 1), (2, h-4), (w-6, h-4)]:
        pygame.draw.rect(body, (20,20,20), (wx, wy, 4, 3))
    rotated = pygame.transform.rotate(body, math.degrees(angle))
    surf.blit(rotated, (cx - rotated.get_width()//2, cy - rotated.get_height()//2))


# ═══════════════════════════════════════════════════════════════════════════════
#  IntersectionRenderer — draws one zone
# ═══════════════════════════════════════════════════════════════════════════════

class IntersectionRenderer:
    """Handles drawing of one zone's roads, signals, and vehicles."""

    def __init__(self, cx: int, cy: int, zone_id: str) -> None:
        self.cx = cx
        self.cy = cy
        self.zone_id = zone_id
        self.flash_t = 0.0

    def draw(self, surf: pygame.Surface, sig: dict,
             vehicles: List[Vehicle], dt: float) -> None:
        self.flash_t += dt
        flash = int(self.flash_t * 3) % 2 == 0

        self._draw_roads(surf, sig)
        self._draw_signals(surf, sig)
        if sig.get("state") == cfg.STATE_EMERGENCY:
            self._draw_emergency_corridor(surf, sig)
        self._draw_vehicles(surf, vehicles, sig, flash)
        self._draw_zone_label(surf, sig)

    def _draw_roads(self, surf: pygame.Surface, sig: dict) -> None:
        cx, cy = self.cx, self.cy
        rw = ROAD_W

        # Grass corners
        corners = [
            (cx - 130, cy - 130, 110, 110),
            (cx +  20, cy - 130, 110, 110),
            (cx - 130, cy +  20, 110, 110),
            (cx +  20, cy +  20, 110, 110),
        ]
        for r in corners:
            pygame.draw.rect(surf, C_GRASS, r)

        # Road rectangles
        pygame.draw.rect(surf, C_ROAD, (cx - rw//2, cy - 160, rw, 320))  # NS road
        pygame.draw.rect(surf, C_ROAD, (cx - 160, cy - rw//2, 320, rw))  # EW road

        # Lane markings (dashed white)
        dash_len, gap_len = 14, 8
        # NS road — vertical dashes
        y = cy - 155
        while y < cy - rw//2:
            pygame.draw.rect(surf, C_LANE_MARK, (cx - 1, y, 2, dash_len))
            y += dash_len + gap_len
        y = cy + rw//2
        while y < cy + 155:
            pygame.draw.rect(surf, C_LANE_MARK, (cx - 1, y, 2, dash_len))
            y += dash_len + gap_len

        # EW road — horizontal dashes
        x = cx - 155
        while x < cx - rw//2:
            pygame.draw.rect(surf, C_LANE_MARK, (x, cy - 1, dash_len, 2))
            x += dash_len + gap_len
        x = cx + rw//2
        while x < cx + 155:
            pygame.draw.rect(surf, C_LANE_MARK, (x, cy - 1, dash_len, 2))
            x += dash_len + gap_len

        # Yellow median lines (NS)
        pygame.draw.rect(surf, C_MEDIAN, (cx - 1, cy - 160, 2, 320))
        pygame.draw.rect(surf, C_MEDIAN, (cx - 160, cy - 1, 320, 2))

        # Stop lines
        ns, ew = sig.get("NS", "RED"), sig.get("EW", "RED")
        # North stop line
        pygame.draw.rect(surf, C_STOP_LINE, (cx - rw//2, cy - rw//2 - 4, rw, 3))
        # South stop line
        pygame.draw.rect(surf, C_STOP_LINE, (cx - rw//2, cy + rw//2 + 1, rw, 3))
        # East stop line
        pygame.draw.rect(surf, C_STOP_LINE, (cx + rw//2 + 1, cy - rw//2, 3, rw))
        # West stop line
        pygame.draw.rect(surf, C_STOP_LINE, (cx - rw//2 - 4, cy - rw//2, 3, rw))

    def _draw_signals(self, surf: pygame.Surface, sig: dict) -> None:
        cx, cy = self.cx, self.cy
        ns, ew = sig.get("NS", "RED"), sig.get("EW", "RED")
        rw = ROAD_W // 2

        # Signal pole positions: top-left corner of each junction arm
        positions = [
            (cx - rw - 18, cy - rw - 38, ns, "NS"),   # NW corner → NS signal
            (cx + rw +  2, cy + rw +  2, ns, "NS"),   # SE corner → NS signal
            (cx + rw +  2, cy - rw - 38, ew, "EW"),   # NE corner → EW signal
            (cx - rw - 18, cy + rw +  2, ew, "EW"),   # SW corner → EW signal
        ]
        for px, py, state, _ in positions:
            self._draw_signal_head(surf, px, py, state)

    def _draw_signal_head(self, surf: pygame.Surface,
                          px: int, py: int, state: str) -> None:
        """Draw a traffic signal head (3 lenses)."""
        bw, bh = 14, 38
        pygame.draw.rect(surf, C_SIGNAL_BODY, (px, py, bw, bh), border_radius=3)
        # Lenses: red, yellow, green top-to-bottom
        lens_y = [py + 3, py + 14, py + 25]
        on_colors  = [C_RED, C_YELLOW, C_GREEN]
        off_colors = [(60,20,20), (60,50,10), (10,50,10)]
        active_idx = {"RED": 0, "YELLOW": 1, "GREEN": 2}.get(state, 0)
        for i, (ly, oc, fc) in enumerate(zip(lens_y, on_colors, off_colors)):
            color = oc if i == active_idx else fc
            cx_l = px + bw//2
            cy_l = ly + 5
            # Glow halo for active lens
            if i == active_idx:
                glow = pygame.Surface((22, 22), pygame.SRCALPHA)
                pygame.draw.circle(glow, (*color, 80), (11,11), 10)
                surf.blit(glow, (cx_l - 11, cy_l - 11))
            pygame.draw.circle(surf, color, (cx_l, cy_l), 5)

    def _draw_emergency_corridor(self, surf: pygame.Surface, sig: dict) -> None:
        """Highlight the emergency green corridor with a pulsing amber glow."""
        lane = sig.get("emergency_lane")
        if not lane:
            return
        cx, cy = self.cx, self.cy
        rw = ROAD_W // 2
        alpha = int(60 + 40 * math.sin(self.flash_t * 6))
        glow = pygame.Surface((ROAD_W, 320), pygame.SRCALPHA)
        glow.fill((255, 180, 0, alpha))
        if lane in ("north", "south"):
            surf.blit(glow, (cx - ROAD_W//2, cy - 160))
        else:
            glow2 = pygame.Surface((320, ROAD_W), pygame.SRCALPHA)
            glow2.fill((255, 180, 0, alpha))
            surf.blit(glow2, (cx - 160, cy - ROAD_W//2))

        # "EMERGENCY" text
        font = pygame.font.SysFont("consolas", 11, bold=True)
        txt = font.render("⚡ EMERGENCY CORRIDOR", True, C_TEXT_EMERG)
        surf.blit(txt, (cx - txt.get_width()//2, cy - 155 if lane in ("north","south") else cy - ROAD_W//2 - 16))

    def _draw_vehicles(self, surf: pygame.Surface, vehicles: List[Vehicle],
                       sig: dict, flash: bool) -> None:
        cx, cy = self.cx, self.cy
        scale = 0.35   # world-metres → pixels for the zone view

        for v in vehicles:
            if not v.active:
                continue
            wx, wy = v.position
            # Map world coords to screen
            px = cx + int(wx * scale)
            py = cy - int(wy * scale)   # y-axis flipped

            # Lane offset to keep vehicles in their lane
            lane_offsets = {
                "north": (-16, 0), "south": (16, 0),
                "east":  (0, -16), "west":  (0, 16),
            }
            off = lane_offsets.get(v.lane_id, (0,0))
            px += off[0]; py += off[1]

            # Angle based on travel direction
            angles = {"north": 90, "south": -90, "east": 180, "west": 0}
            angle_deg = angles.get(v.lane_id, 0)
            angle_rad = math.radians(angle_deg)

            w, h = SPRITE_SIZES[v.vehicle_type]
            color_list = VEHICLE_COLORS[v.vehicle_type]
            # Stable color per vehicle (hash of id)
            color = color_list[hash(v.vehicle_id) % len(color_list)]

            if v.vehicle_type == "car":
                _draw_car(surf, px, py, color, w, h, angle_rad)
            elif v.vehicle_type == "bike":
                _draw_bike(surf, px, py, color, w, h, angle_rad)
            elif v.vehicle_type == "auto":
                _draw_auto(surf, px, py, color, w, h, angle_rad)
            elif v.vehicle_type == "ambulance":
                _draw_ambulance(surf, px, py, w, h, angle_rad, flash)

    def _draw_zone_label(self, surf: pygame.Surface, sig: dict) -> None:
        cx, cy = self.cx, self.cy
        font_sm = pygame.font.SysFont("consolas", 10, bold=True)
        state_colors = {
            cfg.STATE_EMERGENCY: C_TEXT_EMERG,
            cfg.STATE_RECOVERY:  (255, 180, 50),
            cfg.STATE_ALL_RED:   (200, 100, 100),
        }
        col = state_colors.get(sig.get("state",""), C_TEXT)
        mode_txt = "🚨 PRIORITY MODE" if sig.get("mode") == "PRIORITY" else "⚙ OPTIMIZATION"
        lbl = font_sm.render(f"{self.zone_id}  {mode_txt}", True, col)
        surf.blit(lbl, (cx - lbl.get_width()//2, cy - 180))


# ═══════════════════════════════════════════════════════════════════════════════
#  Zone — bundles controller + vehicles for one intersection
# ═══════════════════════════════════════════════════════════════════════════════

class Zone:
    """One smart intersection zone: controller + vehicles + renderer."""

    def __init__(self, zone_id: str, cx: int, cy: int, logger: CSVLogger) -> None:
        self.zone_id  = zone_id
        self.broker   = Broker()
        self.ctrl     = IntersectionController(self.broker, zone_id=zone_id, logger=logger)
        self.renderer = IntersectionRenderer(cx, cy, zone_id)
        self.vehicles: List[Vehicle] = []
        self.logger   = logger
        self._spawn_initial_vehicles()

    def _spawn_initial_vehicles(self) -> None:
        for lane in cfg.LANE_IDS:
            count = random.randint(2, 5)
            for _ in range(count):
                vtype = random.choices(
                    ["car", "car", "car", "bike", "auto"],
                    weights=[50, 50, 50, 20, 15]
                )[0]
                v = Vehicle(lane, vehicle_type=vtype,
                            broker=self.broker, zone_id=self.zone_id)
                self.vehicles.append(v)

    def spawn_ambulance(self) -> None:
        lane = random.choice(cfg.LANE_IDS)
        amb = Vehicle(lane, vehicle_type="ambulance",
                      broker=self.broker, zone_id=self.zone_id)
        # Ensure ambulance is immediately inside the detection zone so that
        # corridor preemption starts within a few seconds (not after a long approach).
        # Keep it behind the stop line to avoid instant intersection entry.
        try:
            if lane in ("north", "south"):
                x, y = amb.position
                y = max(min(y, 420.0), -420.0)
                if abs(y) < 120.0:
                    y = 180.0 if y >= 0 else -180.0
                amb.position = (x, y)
            else:
                x, y = amb.position
                x = max(min(x, 420.0), -420.0)
                if abs(x) < 120.0:
                    x = 180.0 if x >= 0 else -180.0
                amb.position = (x, y)
        except Exception:
            pass
        self.vehicles.append(amb)
        print(f"[{self.zone_id}] 🚑 Ambulance spawned in {lane} lane — ETA ≈ {amb.to_packet().get('eta','?')}s")

    def step(self, dt: float) -> None:
        # Broadcast V2I packets
        for v in self.vehicles:
            v.broadcast()

        # Step controller
        self.ctrl.step(dt)

        # Fetch updated signals AFTER controller step (prevents stale-signal artefacts)
        sig = self.ctrl.get_signal_state()
        ns = sig["NS"]; ew = sig["EW"]

        # Update vehicle physics with lane-wise safe-gap following to prevent overlap.
        lanes: Dict[str, List[Vehicle]] = {l: [] for l in cfg.LANE_IDS}
        for v in self.vehicles:
            if v.active:
                lanes[v.lane_id].append(v)

        def dist_to_center_along_lane(v: Vehicle) -> float:
            x, y = v.position
            return abs(y) if v.lane_id in ("north", "south") else abs(x)

        for lane_id, vs in lanes.items():
            # Sort so the lead vehicle is closest to the intersection (smallest distance)
            vs.sort(key=dist_to_center_along_lane)
            for i, v in enumerate(vs):
                lane_sig = ns if lane_id in ("north", "south") else ew
                if i == 0:
                    v.update(dt, lane_sig)
                else:
                    lead = vs[i - 1]
                    gap = dist_to_center_along_lane(v) - dist_to_center_along_lane(lead)
                    lead_len = cfg.VEHICLE_LENGTH_M.get(lead.vehicle_type, 4.0)
                    lead_speed = lead.velocity
                    # gap is centerline distance; subtract lead length to approximate bumper gap
                    bumper_gap = max(0.0, gap - lead_len)
                    v.update(dt, lane_sig, lead_gap_m=bumper_gap, lead_speed_ms=lead_speed, lead_length_m=lead_len)

            # Post-step overlap resolution (last-resort): enforce monotonic ordering.
            # If two vehicles are closer than standstill gap, push follower back.
            for i in range(1, len(vs)):
                lead = vs[i - 1]
                fol  = vs[i]
                lead_d = dist_to_center_along_lane(lead)
                fol_d  = dist_to_center_along_lane(fol)
                min_gap = cfg.MIN_STANDSTILL_GAP_M + cfg.VEHICLE_LENGTH_M.get(lead.vehicle_type, 4.0)
                if fol_d < lead_d + min_gap:
                    # Set follower distance to lead_d + min_gap along its lane axis
                    target_d = lead_d + min_gap
                    x, y = fol.position
                    if lane_id in ("north", "south"):
                        fol.position = (x, target_d if y > 0 else -target_d)
                    else:
                        fol.position = (target_d if x > 0 else -target_d, y)
                    fol.velocity = min(fol.velocity, lead.velocity)

        # Log occasionally & respawn cleared vehicles
        new_vehicles = []
        for v in self.vehicles:
            if v.active:
                new_vehicles.append(v)
            else:
                # Log final wait time
                self.logger.log_vehicle(
                    v.vehicle_id, self.zone_id, v.lane_id,
                    v.wait_time, "vehicle_cleared"
                )
                # Respawn with different vehicle
                vtype = random.choices(["car","bike","auto"], weights=[6,2,2])[0]
                nv = Vehicle(v.lane_id, vehicle_type=vtype,
                             broker=self.broker, zone_id=self.zone_id)
                new_vehicles.append(nv)
        self.vehicles = new_vehicles

        # Throttled: spawn extras occasionally
        if random.random() < 0.002:
            lane = random.choice(cfg.LANE_IDS)
            vtype = random.choices(["car","bike","auto"], weights=[6,2,2])[0]
            self.vehicles.append(Vehicle(lane, vtype, self.broker, self.zone_id))

    def draw(self, surf: pygame.Surface, dt: float) -> None:
        sig = self.ctrl.get_signal_state()
        self.renderer.draw(surf, sig, self.vehicles, dt)

    def get_stats(self) -> dict:
        sig = self.ctrl.get_signal_state()
        total_wait = sum(v.wait_time for v in self.vehicles if v.active)
        return {
            "zone":       self.zone_id,
            "state":      sig["state"],
            "NS":         sig["NS"],
            "EW":         sig["EW"],
            "mode":       sig["mode"],
            "vehicles":   len([v for v in self.vehicles if v.active]),
            "emergency":  sig["emergency_lane"],
            "phase_timer": sig["phase_timer"],
            "adaptive_green": sig["adaptive_green"],
            "preemptions": sig["preempt_count"],
            "avg_wait":   round(total_wait / max(len(self.vehicles),1), 2),
            "sim_time":   sig["sim_time"],
        }


# ═══════════════════════════════════════════════════════════════════════════════
#  HUD / Stats Panel
# ═══════════════════════════════════════════════════════════════════════════════

class HUDPanel:
    """Draws the right-side statistics and control panel."""

    def __init__(self) -> None:
        self.font_title = pygame.font.SysFont("consolas", 14, bold=True)
        self.font_body  = pygame.font.SysFont("consolas", 11)
        self.font_big   = pygame.font.SysFont("consolas", 20, bold=True)

    def draw(self, surf: pygame.Surface, zones: Dict[str, Zone],
             sim_speed: float, selected_zone: str, sim_time: float) -> None:
        panel_x = 700
        # Semi-transparent panel background
        panel = pygame.Surface((700, 900), pygame.SRCALPHA)
        panel.fill((15, 20, 40, 200))
        surf.blit(panel, (panel_x, 0))

        y = 15
        # Title
        title = self.font_big.render("LIFE-LINK  Smart Traffic System", True, (100, 180, 255))
        surf.blit(title, (panel_x + 10, y)); y += 30

        sub = self.font_body.render(f"Sim Time: {sim_time:.1f}s   Speed: {sim_speed:.1f}×   Press 1-4 to select zone", True, (140,160,200))
        surf.blit(sub, (panel_x + 10, y)); y += 22

        pygame.draw.line(surf, C_PANEL_BORD, (panel_x+5, y), (1395, y)); y += 8

        # Per-zone stats
        for z_id, zone in zones.items():
            stats = zone.get_stats()
            is_sel = (z_id == selected_zone)
            bg_col = (40, 60, 100) if is_sel else (22, 30, 55)
            bord   = (80, 140, 255) if is_sel else C_PANEL_BORD

            row_h = 80
            rect = pygame.Rect(panel_x + 5, y, 688, row_h)
            pygame.draw.rect(surf, bg_col, rect, border_radius=6)
            pygame.draw.rect(surf, bord, rect, 1, border_radius=6)

            # Zone name + mode badge
            z_col = (100, 220, 100) if stats["mode"]=="OPTIMIZATION" else C_TEXT_EMERG
            lbl = self.font_title.render(f"  Zone {z_id}", True, (200, 220, 255))
            surf.blit(lbl, (panel_x + 12, y + 6))
            mode_lbl = self.font_body.render(stats["mode"], True, z_col)
            surf.blit(mode_lbl, (panel_x + 130, y + 8))

            # Signal indicators
            self._draw_mini_signal(surf, panel_x + 280, y + 8, stats["NS"], "NS")
            self._draw_mini_signal(surf, panel_x + 350, y + 8, stats["EW"], "EW")

            # Stats row
            stat_txt = (
                f"  Vehicles: {stats['vehicles']:3d}  "
                f"Avg Wait: {stats['avg_wait']:5.1f}s  "
                f"Phase: {stats['state'][:12]:12s}  "
                f"Timer: {stats['phase_timer']:5.1f}s  "
                f"Preemptions: {stats['preemptions']}"
            )
            st = self.font_body.render(stat_txt, True, C_TEXT)
            surf.blit(st, (panel_x + 8, y + 30))

            # Adaptive green bar
            ratio = min(stats["adaptive_green"] / cfg.MAX_GREEN_TIME, 1.0)
            bar_w = int(ratio * 300)
            pygame.draw.rect(surf, (30,60,30),  (panel_x+10, y+52, 300, 10), border_radius=3)
            pygame.draw.rect(surf, (50,200,80), (panel_x+10, y+52, bar_w, 10), border_radius=3)
            ag_lbl = self.font_body.render(
                f"  Adaptive Green: {stats['adaptive_green']:.1f}s", True, (120,200,120))
            surf.blit(ag_lbl, (panel_x + 310, y + 50))

            if stats["emergency"]:
                em_lbl = self.font_body.render(
                    f"  🚨 EMERGENCY — {stats['emergency'].upper()} LANE", True, C_TEXT_EMERG)
                surf.blit(em_lbl, (panel_x + 8, y + 64))

            y += row_h + 6

        # Controls legend
        y += 5
        pygame.draw.line(surf, C_PANEL_BORD, (panel_x+5, y), (1395, y)); y += 8
        controls = [
            "CONTROLS:",
            "  1-4      : Select zone (Alpha/Beta/Gamma/Delta)",
            "  E        : Spawn ambulance in selected zone",
            "  +/-      : Increase/decrease sim speed",
            "  SPACE    : Pause / Resume",
            "  Q / ESC  : Quit & save logs",
        ]
        for line in controls:
            col = (180, 200, 255) if line == "CONTROLS:" else (140, 155, 190)
            surf.blit(self.font_body.render(line, True, col), (panel_x + 10, y))
            y += 16

        # Bottom info bar
        info = self.font_body.render(
            "Life-Link v1.0  |  ECE Minor Project  |  "
            "Saumya Sharma (23102156)  &  Pulkit Pandey (23102211)  |  JIIT Noida",
            True, (80, 100, 140)
        )
        surf.blit(info, (panel_x + 10, 875))

    def _draw_mini_signal(self, surf, x, y, state, label):
        colors = {"GREEN": C_GREEN, "YELLOW": C_YELLOW, "RED": C_RED}
        col = colors.get(state, C_RED)
        pygame.draw.rect(surf, (20,20,20), (x, y, 22, 38), border_radius=3)
        ofs = {"GREEN": 2, "YELLOW": 1, "RED": 0}
        for i, c in enumerate([C_RED, C_YELLOW, C_GREEN]):
            active = (ofs.get(state,-1) == i)
            pygame.draw.circle(surf, c if active else (30,30,30), (x+11, y+7+i*11), 5)
        fnt = pygame.font.SysFont("consolas", 9)
        lbl = fnt.render(label, True, (180,180,180))
        surf.blit(lbl, (x+2, y+40))


# ═══════════════════════════════════════════════════════════════════════════════
#  Main simulation loop
# ═══════════════════════════════════════════════════════════════════════════════

def run_simulation() -> None:
    """Launch the Pygame Life-Link simulation."""
    pygame.init()
    pygame.display.set_caption("Life-Link — Smart Traffic System Simulation")
    screen = pygame.display.set_mode((cfg.SCREEN_W, cfg.SCREEN_H))
    clock  = pygame.time.Clock()

    os.makedirs("logs", exist_ok=True)
    os.makedirs("output", exist_ok=True)
    logger = CSVLogger("logs/vehicle_log.csv")

    # Create all 4 zones
    zones: Dict[str, Zone] = {}
    for name, meta in cfg.ZONES.items():
        cx, cy = meta["center"]
        zones[name] = Zone(name, cx, cy, logger)

    hud           = HUDPanel()
    selected_zone = "Alpha"
    sim_speed     = 1.0
    paused        = False
    sim_time      = 0.0
    json_dump_t   = 0.0

    print("=" * 60)
    print("  LIFE-LINK Smart Traffic System — Pygame Simulation")
    print("=" * 60)
    print("  Zones: Alpha · Beta · Gamma · Delta")
    print("  Keys: 1-4 zones | E: ambulance | +/-: speed | SPACE: pause")
    print("=" * 60)

    running = True
    while running:
        real_dt = clock.tick(cfg.FPS) / 1000.0
        dt = min(real_dt * sim_speed, 0.1)   # capped to prevent physics explosions

        # ── Event handling ─────────────────────────────────────────────────
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_ESCAPE, pygame.K_q):
                    running = False
                elif event.key == pygame.K_SPACE:
                    paused = not paused
                elif event.key == pygame.K_1: selected_zone = "Alpha"
                elif event.key == pygame.K_2: selected_zone = "Beta"
                elif event.key == pygame.K_3: selected_zone = "Gamma"
                elif event.key == pygame.K_4: selected_zone = "Delta"
                elif event.key == pygame.K_e:
                    zones[selected_zone].spawn_ambulance()
                elif event.key in (pygame.K_PLUS, pygame.K_EQUALS):
                    sim_speed = min(sim_speed * 1.5, 10.0)
                elif event.key == pygame.K_MINUS:
                    sim_speed = max(sim_speed / 1.5, 0.25)

        if not paused:
            sim_time += dt
            for zone in zones.values():
                zone.step(dt)

        # ── Rendering ──────────────────────────────────────────────────────
        screen.fill(C_BG)

        # Draw zone separators
        pygame.draw.line(screen, (30, 40, 70), (700, 0), (700, 900), 2)
        pygame.draw.line(screen, (30, 40, 70), (0, 450), (700, 450), 2)
        pygame.draw.line(screen, (30, 40, 70), (700, 450), (1400, 450), 2)

        # Draw left-side zones (Alpha=top-left, Gamma=bottom-left)
        for name, zone in zones.items():
            zone.draw(screen, dt if not paused else 0)

        # HUD
        hud.draw(screen, zones, sim_speed, selected_zone, sim_time)

        # Pause overlay
        if paused:
            ov = pygame.Surface((700, 900), pygame.SRCALPHA)
            ov.fill((0, 0, 0, 90))
            screen.blit(ov, (0,0))
            pfont = pygame.font.SysFont("consolas", 36, bold=True)
            pt = pfont.render("⏸  PAUSED  (SPACE to resume)", True, (255,255,100))
            screen.blit(pt, (350 - pt.get_width()//2, 430))

        pygame.display.flip()

        # ── JSON state export (every 0.5 real seconds) ─────────────────────
        json_dump_t += real_dt
        if json_dump_t >= 0.5:
            json_dump_t = 0.0
            state_dump = {
                "timestamp": time.time(),
                "sim_time":  round(sim_time, 1),
                "zones": {name: zone.get_stats() for name, zone in zones.items()},
            }
            try:
                with open("output/live_state.json", "w") as jf:
                    json.dump(state_dump, jf, indent=2)
            except IOError:
                pass

    pygame.quit()
    print(f"\n[Life-Link] Simulation ended — logs saved to logs/vehicle_log.csv")
    print(f"[Life-Link] Live state JSON: output/live_state.json")


if __name__ == "__main__":
    run_simulation()
