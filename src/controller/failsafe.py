"""
Life-Link — Fail-Safe Conflict Monitor (Pulkit's module)

Prevents illegal signal state transitions that would create simultaneous
green lights on conflicting lane pairs (NS + EW both GREEN = collision risk).

This is the safety-critical layer — every state transition must pass through
`validate_transition()` before being applied.
"""
from __future__ import annotations
from typing import Dict, Set

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
import src.config as cfg


# Legal next-states for each current state
_LEGAL_TRANSITIONS: Dict[str, Set[str]] = {
    cfg.STATE_NS_GREEN:  {cfg.STATE_NS_YELLOW},
    cfg.STATE_NS_YELLOW: {cfg.STATE_EW_GREEN, cfg.STATE_ALL_RED, cfg.STATE_EMERGENCY},
    cfg.STATE_EW_GREEN:  {cfg.STATE_EW_YELLOW},
    cfg.STATE_EW_YELLOW: {cfg.STATE_NS_GREEN, cfg.STATE_ALL_RED, cfg.STATE_EMERGENCY},
    cfg.STATE_EMERGENCY: {cfg.STATE_ALL_RED, cfg.STATE_RECOVERY},
    cfg.STATE_RECOVERY:  {cfg.STATE_NS_GREEN, cfg.STATE_EW_GREEN},
    cfg.STATE_ALL_RED:   {cfg.STATE_NS_GREEN, cfg.STATE_EW_GREEN, cfg.STATE_EMERGENCY},
}

# Conflicting state pairs — NEVER both active simultaneously
_CONFLICT_PAIRS: Set[frozenset] = {
    frozenset({cfg.STATE_NS_GREEN, cfg.STATE_EW_GREEN}),
    frozenset({cfg.STATE_NS_GREEN, cfg.STATE_EW_YELLOW}),
    frozenset({cfg.STATE_NS_YELLOW, cfg.STATE_EW_GREEN}),
}


class FailSafeMonitor:
    """
    Validates signal state transitions for the intersection controller.

    Usage
    -----
    monitor = FailSafeMonitor()
    ok, reason = monitor.validate_transition(current_state, proposed_state)
    if ok:
        apply_transition(proposed_state)
    else:
        log_warning(reason)
    """

    def __init__(self) -> None:
        self._rejected_count: int = 0
        self._accepted_count: int = 0

    def validate_transition(
        self, current: str, proposed: str
    ) -> tuple[bool, str]:
        """
        Check whether transitioning from `current` to `proposed` is legal.

        Parameters
        ----------
        current  : current signal state string
        proposed : proposed next signal state string

        Returns
        -------
        (allowed: bool, reason: str)
            allowed=True  → transition is safe; apply it
            allowed=False → transition is illegal; reason explains why
        """
        legal_next = _LEGAL_TRANSITIONS.get(current, set())

        if proposed not in legal_next:
            self._rejected_count += 1
            return False, (
                f"ILLEGAL: {current} → {proposed}  "
                f"(allowed: {sorted(legal_next)})"
            )

        self._accepted_count += 1
        return True, "OK"

    def check_conflict(self, state_a: str, state_b: str) -> bool:
        """
        Return True if two simultaneous states would create a signal clash.

        Parameters
        ----------
        state_a, state_b : two active states to cross-check

        Returns
        -------
        True  → CONFLICT detected (danger)
        False → no conflict
        """
        return frozenset({state_a, state_b}) in _CONFLICT_PAIRS

    @property
    def stats(self) -> Dict[str, int]:
        return {
            "accepted": self._accepted_count,
            "rejected": self._rejected_count,
        }
