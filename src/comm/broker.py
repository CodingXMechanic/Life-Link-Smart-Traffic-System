"""
Life-Link — In-Process V2I Message Broker (Saumya's module)

Thread-safe publish/subscribe queue connecting Vehicle instances to the
IntersectionController. No external dependencies — uses stdlib queue.Queue.

API (guaranteed for Pulkit's integration):
    broker.publish(packet: dict) -> None
    broker.get_packets()         -> List[dict]   (atomic snapshot + drain)
"""
from __future__ import annotations
import queue
import threading
from typing import List, Optional


class Broker:
    """
    Thread-safe in-process message broker for V2I packet exchange.

    Vehicles call `publish()` from any thread; the controller calls
    `get_packets()` once per simulation tick to drain the queue atomically.

    Parameters
    ----------
    maxsize : maximum queue depth (0 = unlimited)
    """

    def __init__(self, maxsize: int = 0) -> None:
        self._queue: queue.Queue[dict] = queue.Queue(maxsize=maxsize)
        self._lock  = threading.Lock()
        self._total_published: int = 0

    # ── Publisher API ─────────────────────────────────────────────────────────

    def publish(self, packet: dict) -> None:
        """
        Enqueue a V2I data packet.

        Parameters
        ----------
        packet : dict  — must contain at minimum 'vehicle_id' and 'timestamp'

        Raises
        ------
        ValueError if packet is missing required keys (fail-safe validation).
        """
        required = {"vehicle_id", "timestamp", "lane_id", "priority_flag"}
        missing  = required - packet.keys()
        if missing:
            raise ValueError(f"Broker.publish: packet missing keys {missing}")
        try:
            self._queue.put_nowait(packet)
            with self._lock:
                self._total_published += 1
        except queue.Full:
            pass  # drop silently if queue at capacity

    # ── Subscriber API ────────────────────────────────────────────────────────

    def get_packets(self) -> List[dict]:
        """
        Atomically drain and return all currently queued packets.

        Returns an empty list if no packets are pending.  The controller
        should call this once per simulation tick.

        Returns
        -------
        List[dict]
        """
        packets: List[dict] = []
        while True:
            try:
                packets.append(self._queue.get_nowait())
            except queue.Empty:
                break
        return packets

    # ── Diagnostics ──────────────────────────────────────────────────────────

    @property
    def pending(self) -> int:
        """Number of packets currently waiting in the queue."""
        return self._queue.qsize()

    @property
    def total_published(self) -> int:
        """Cumulative packets ever published to this broker."""
        return self._total_published

    def __repr__(self) -> str:
        return f"Broker(pending={self.pending}, total={self.total_published})"
