"""
Life-Link — CSV Logger (Saumya's module)

Logs per-vehicle wait times and system events to a CSV file compatible with
Saumya's analysis pipeline and Pulkit's controller state logging.

CSV columns:  timestamp, vehicle_id, zone_id, lane_id, wait_time, event
"""
from __future__ import annotations
import csv
import os
import time
from typing import Optional


_CSV_HEADER = ["timestamp", "vehicle_id", "zone_id", "lane_id", "wait_time", "event"]


class CSVLogger:
    """
    Append-mode CSV logger for Life-Link events.

    Parameters
    ----------
    filepath : path to the CSV file (created if not existing)
    """

    def __init__(self, filepath: str = "logs/vehicle_log.csv") -> None:
        self.filepath = filepath
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        self._ensure_header()

    def _ensure_header(self) -> None:
        """Write CSV header if the file is new or empty."""
        write_header = not os.path.exists(self.filepath) or os.path.getsize(self.filepath) == 0
        if write_header:
            with open(self.filepath, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=_CSV_HEADER)
                writer.writeheader()

    # ── Public API ────────────────────────────────────────────────────────────

    def log_vehicle(
        self,
        vehicle_id: str,
        zone_id: str,
        lane_id: str,
        wait_time: float,
        event: str = "tick",
    ) -> None:
        """
        Append one vehicle state row to the log.

        Parameters
        ----------
        vehicle_id : unique vehicle identifier
        zone_id    : zone name (Alpha/Beta/Gamma/Delta)
        lane_id    : 'north'|'south'|'east'|'west'
        wait_time  : cumulative seconds vehicle has waited at red
        event      : free-text event label (e.g. 'tick', 'preemption_start')
        """
        row = {
            "timestamp":  round(time.time(), 4),
            "vehicle_id": vehicle_id,
            "zone_id":    zone_id,
            "lane_id":    lane_id,
            "wait_time":  round(wait_time, 3),
            "event":      event,
        }
        with open(self.filepath, "a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=_CSV_HEADER)
            writer.writerow(row)

    def log_event(
        self,
        event: str,
        zone_id: str = "",
        lane_id: str = "",
        vehicle_id: str = "SYSTEM",
        wait_time: float = 0.0,
    ) -> None:
        """
        Log a system-level event (preemption, recovery, phase change, etc.).

        Parameters
        ----------
        event      : descriptive event string
        zone_id    : zone generating the event
        lane_id    : affected lane (empty if zone-wide)
        vehicle_id : vehicle triggering the event, or 'SYSTEM'
        wait_time  : contextual wait time (0 for system events)
        """
        self.log_vehicle(vehicle_id, zone_id, lane_id, wait_time, event)

    def log_from_packet(self, packet: dict, event: str = "tick") -> None:
        """
        Convenience: log directly from a V2I packet dict.

        Parameters
        ----------
        packet : V2I packet (must contain vehicle_id, zone_id, lane_id, wait_time)
        event  : event label
        """
        self.log_vehicle(
            vehicle_id=packet.get("vehicle_id", "UNKNOWN"),
            zone_id=packet.get("zone_id", ""),
            lane_id=packet.get("lane_id", ""),
            wait_time=packet.get("wait_time", 0.0),
            event=event,
        )


def generate_sample_csv(filepath: str = "logs/sample_log.csv", n_rows: int = 50) -> None:
    """
    Generate a sample CSV log file with synthetic data for demonstration.

    Parameters
    ----------
    filepath : output path
    n_rows   : number of sample rows to write
    """
    import random
    logger = CSVLogger(filepath)
    vehicle_ids = [f"VH_{i:04X}" for i in range(10)]
    lanes = ["north", "south", "east", "west"]
    zones = ["Alpha", "Beta", "Gamma", "Delta"]
    events = ["tick", "tick", "tick", "preemption_start", "preemption_end",
              "recovery_start", "phase_change", "tick"]

    for i in range(n_rows):
        logger.log_vehicle(
            vehicle_id=random.choice(vehicle_ids),
            zone_id=random.choice(zones),
            lane_id=random.choice(lanes),
            wait_time=round(random.uniform(0, 45), 2),
            event=random.choice(events),
        )
    print(f"[Logger] Sample CSV written → {filepath}  ({n_rows} rows)")


if __name__ == "__main__":
    generate_sample_csv()
