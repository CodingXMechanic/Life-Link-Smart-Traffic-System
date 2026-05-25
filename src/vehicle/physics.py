"""
Life-Link — Physics Utilities (Saumya's module)

Provides discrete-time kinematic update, braking distance, and ETA helpers.
All maths uses SI units: metres, m/s, m/s², seconds.
"""
from __future__ import annotations
import math
from typing import Tuple


def update_kinematics(
    position: Tuple[float, float],
    velocity: float,
    acceleration: float,
    dt: float,
    max_speed: float = 25.0,
    lane_axis: str | None = None,
) -> Tuple[Tuple[float, float], float, float]:
    """
    Discrete Euler integration of 1-D kinematics projected onto a 2-D lane.

    Physics:
        v_new = clamp(v + a*dt, 0, max_speed)
        ds    = v*dt + 0.5*a*dt²          (kinematic equation #2)
        The vehicle travels along its lane direction, so displacement is
        applied proportionally to whichever axis is non-zero.

    Parameters
    ----------
    position     : (x, y) current world position in metres
    velocity     : current scalar speed  (m/s)
    acceleration : current scalar accel  (m/s²)  — can be negative (braking)
    dt           : time step (s)
    max_speed    : speed cap (m/s)

    Returns
    -------
    (new_position, new_velocity, acceleration)  — acceleration unchanged
    """
    # v = u + at
    new_velocity = velocity + acceleration * dt
    new_velocity = max(0.0, min(new_velocity, max_speed))   # clamp [0, v_max]

    # s = u*t + ½at²  (average of old & new velocity * dt is equivalent)
    ds = velocity * dt + 0.5 * acceleration * dt * dt

    x, y = position
    # If lane axis is known, do not infer direction from (x,y) magnitude.
    # Inferring by abs(x) vs abs(y) can cause a vehicle to "switch axes" near
    # the intersection due to small lane jitter, leading to overlaps/collisions.
    axis = lane_axis
    if axis not in (None, "x", "y"):
        axis = None

    if axis == "x":               # travelling east/west  → move along x
        new_pos = (x + math.copysign(ds, x) * -1, y)   # converge toward origin
    elif axis == "y":             # travelling north/south → move along y
        new_pos = (x, y + math.copysign(ds, y) * -1)
    else:
        # Fallback for non-lane callers (tests/utilities): infer dominant axis.
        if abs(x) >= abs(y):      # travelling east/west  → move along x
            new_pos = (x + math.copysign(ds, x) * -1, y)
        else:                     # travelling north/south → move along y
            new_pos = (x, y + math.copysign(ds, y) * -1)

    return new_pos, new_velocity, acceleration


def braking_distance(velocity: float, deceleration: float = 5.0) -> float:
    """
    Compute the minimum stopping distance from current speed.

    Formula:  d = v² / (2 * a)     (kinematic: v²= u² - 2as, final v=0)

    Parameters
    ----------
    velocity     : current speed (m/s)
    deceleration : braking deceleration magnitude (m/s²)  — positive value

    Returns
    -------
    distance in metres; 0.0 if velocity ≤ 0 or deceleration ≤ 0
    """
    if velocity <= 0.0 or deceleration <= 0.0:
        return 0.0
    return (velocity ** 2) / (2.0 * deceleration)


def calculate_eta(distance: float, velocity: float, acceleration: float = 0.0) -> float:
    """
    Estimate time-to-arrival at the intersection (distance = 0).

    Cases
    -----
    • acceleration == 0  → t = d / v   (uniform motion)
    • acceleration != 0  → solve d = v*t + ½*a*t²  for positive real root
      Discriminant: (v/a)² + 2d/a must be ≥ 0 for a real solution.
    • velocity == 0 and acceleration == 0 → returns float('inf') (vehicle stopped)

    Parameters
    ----------
    distance     : metres to intersection (positive)
    velocity     : current speed m/s (≥ 0)
    acceleration : m/s² (may be negative for braking)

    Returns
    -------
    ETA in seconds (float); float('inf') if unreachable
    """
    if distance <= 0.0:
        return 0.0

    if abs(acceleration) < 1e-9:   # uniform motion
        if velocity <= 0.0:
            return float('inf')
        return distance / velocity

    # Quadratic: ½a·t² + v·t − d = 0  →  at² + 2v·t − 2d = 0
    a_coeff = acceleration
    b_coeff = velocity
    c_coeff = -distance

    discriminant = b_coeff ** 2 - 4 * (a_coeff / 2) * c_coeff
    # = v² + 2·a·d
    discriminant_val = velocity ** 2 + 2 * acceleration * distance

    if discriminant_val < 0:
        return float('inf')    # vehicle decelerates to zero before reaching

    sqrt_disc = math.sqrt(discriminant_val)

    # Two candidate times from quadratic formula
    t1 = (-velocity + sqrt_disc) / acceleration
    t2 = (-velocity - sqrt_disc) / acceleration

    # Pick smallest positive root
    candidates = [t for t in (t1, t2) if t > 0]
    if not candidates:
        return float('inf')
    return min(candidates)


def euclidean_distance(
    pos: Tuple[float, float],
    center: Tuple[float, float] = (0.0, 0.0),
) -> float:
    """
    Euclidean distance from a vehicle position to the intersection centre.

    Parameters
    ----------
    pos    : (x, y) vehicle world coordinates
    center : intersection centre (default origin)

    Returns
    -------
    Distance in metres (float)
    """
    return math.hypot(pos[0] - center[0], pos[1] - center[1])


def in_detection_zone(
    pos: Tuple[float, float],
    radius: float = 500.0,
    center: Tuple[float, float] = (0.0, 0.0),
) -> bool:
    """Return True if vehicle is within the Digital Detection Zone."""
    return euclidean_distance(pos, center) <= radius
