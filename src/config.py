"""
Life-Link Smart Traffic System — Central Configuration
All tunable constants live here. Import from this module everywhere.
"""

# ─── Detection & Communication ───────────────────────────────────────────────
DETECTION_RADIUS: float = 500.0          # metres — Digital Detection Zone radius
PACKET_INTERVAL: float  = 0.1            # seconds between V2I broadcast ticks

# ─── Signal Timing ───────────────────────────────────────────────────────────
# Methodology requirement: mandatory clearance interval before preemption.
YELLOW_DURATION: float  = 3.0            # mandatory yellow clearance (seconds)
MIN_GREEN_TIME:  float  = 4.0            # minimum green before early cut-off (s)
MAX_GREEN_TIME:  float  = 60.0           # maximum green phase duration (s)
RECOVERY_GREEN:  float  = 15.0          # extended green for recovery phase (s)
FIXED_GREEN_TIME: float = 30.0          # baseline fixed-timer green (s)

# ─── Vehicle Physics ─────────────────────────────────────────────────────────
MAX_SPEED_MS:    float  = 25.0           # m/s (~90 km/h) — civilian max
EMERGENCY_SPEED: float  = 20.0          # m/s — ambulance speed
DEFAULT_ACCEL:   float  = 2.0           # m/s² default acceleration
BRAKING_DECEL:   float  = 5.0           # m/s² comfortable braking deceleration
DT:              float  = 0.05          # simulation time-step (seconds)

# ─── Vehicle Interaction (anti-overlap / car-following) ──────────────────────
# These constraints ensure vehicles don't geometrically overlap in the
# simulation and don't enter the intersection when they cannot maintain a safe gap.
MIN_STANDSTILL_GAP_M: float = 3.0        # metres (bumper-to-bumper)
TIME_HEADWAY_S:       float = 1.2        # seconds (dynamic gap component)
FOLLOW_BRAKE_MULT:    float = 1.6        # stronger braking when closing in

# Approx vehicle lengths (metres) for spacing (used by lane-wise following).
VEHICLE_LENGTH_M = {
    "car": 4.5,
    "bike": 2.0,
    "auto": 3.0,
    "ambulance": 6.0,
}

# ─── Intersection Geometry ───────────────────────────────────────────────────
INTERSECTION_CENTER = (0.0, 0.0)        # world origin
LANE_IDS = ["north", "south", "east", "west"]

# ─── Pygame UI ───────────────────────────────────────────────────────────────
SCREEN_W:  int  = 1400
SCREEN_H:  int  = 900
FPS:       int  = 60
SCALE:     float = 0.8

# ─── Zones (Multi-Intersection Smart City) ───────────────────────────────────
ZONES = {
    "Alpha":  {"center": (350,  260), "label": "Alpha  — City Core"},
    "Beta":   {"center": (1050, 260), "label": "Beta   — North Ring"},
    "Gamma":  {"center": (350,  630), "label": "Gamma  — East Corridor"},
    "Delta":  {"center": (1050, 630), "label": "Delta  — West Gateway"},
}

# ─── Logging ─────────────────────────────────────────────────────────────────
LOG_DIR:   str  = "logs"
CSV_FILE:  str  = "logs/vehicle_log.csv"

# ─── Priority Flags ──────────────────────────────────────────────────────────
PRIORITY_CIVILIAN:  int = 0
PRIORITY_EMERGENCY: int = 1

# ─── State Machine States ────────────────────────────────────────────────────
STATE_NS_GREEN    = "NS_GREEN"
STATE_NS_YELLOW   = "NS_YELLOW"
STATE_EW_GREEN    = "EW_GREEN"
STATE_EW_YELLOW   = "EW_YELLOW"
STATE_EMERGENCY   = "EMERGENCY"
STATE_RECOVERY    = "RECOVERY"
STATE_ALL_RED     = "ALL_RED"

# ─── Vehicle Types ───────────────────────────────────────────────────────────
VEHICLE_TYPES = ["car", "bike", "auto", "ambulance"]
