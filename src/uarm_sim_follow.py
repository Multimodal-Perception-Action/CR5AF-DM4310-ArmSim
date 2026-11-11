"""Pure six-joint mapping for a visual simulation; no file or device access.

Encoder changes use the shortest step per sample. Sampling must therefore be
fast enough that each physical joint travels less than half a turn per sample.
Model joint limits constrain the picture only, not a certified robot workspace.
"""
from __future__ import annotations

import math
from numbers import Real
from typing import Sequence

TICKS_PER_TURN = 4096
DEGREES_PER_TICK = 360.0 / TICKS_PER_TURN


def _finite_six(values: Sequence[float], name: str) -> list[float]:
    if not isinstance(values, Sequence) or isinstance(values, (str, bytes)) or len(values) != 6:
        raise ValueError(f"{name} must contain exactly six numbers")
    if any(isinstance(value, bool) or not isinstance(value, Real) for value in values):
        raise ValueError(f"{name} must contain numeric values")
    result = [float(value) for value in values]
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must contain only finite numbers")
    return result


def _ticks(values: Sequence[int]) -> list[int]:
    checked = _finite_six(values, "leader_ticks")
    if any(value != int(value) or not 0 <= value < TICKS_PER_TURN for value in checked):
        raise ValueError("leader_ticks must be integers within 0..4095")
    return [int(value) for value in checked]


class SimFollowMapping:
    """A 1:1 relative joint-angle mapping, constrained only by model limits."""

    def __init__(self, signs: Sequence[float], limits_deg: Sequence[Sequence[float]],
                 scales: Sequence[float] | None = None):
        checked_signs = _finite_six(signs, "signs")
        if any(value not in (-1.0, 1.0) for value in checked_signs):
            raise ValueError("each sign must be -1 or +1")
        checked_scales = _finite_six(scales if scales is not None else [1.0] * 6,
                                     "scales")
        if any(not 0.1 <= value <= 2.0 for value in checked_scales):
            raise ValueError("each scale must be within 0.1..2.0")
        if not isinstance(limits_deg, Sequence) or len(limits_deg) != 6:
            raise ValueError("limits_deg must contain six [lower, upper] pairs")
        checked_limits = []
        for pair in limits_deg:
            if not isinstance(pair, Sequence) or isinstance(pair, (str, bytes)) or len(pair) != 2:
                raise ValueError("each limit must be a [lower, upper] pair")
            if any(isinstance(v, bool) or not isinstance(v, Real) for v in pair):
                raise ValueError("limits must be numeric")
            lower, upper = map(float, pair)
            if not all(math.isfinite(v) for v in (lower, upper)) or lower >= upper:
                raise ValueError("limits must be finite and lower must be less than upper")
            checked_limits.append((lower, upper))
        self._signs = tuple(checked_signs)
        self._scales = tuple(checked_scales)
        self._limits = tuple(checked_limits)
        self.anchor_version = 0
        self._leader_anchor: list[int] | None = None
        self._robot_anchor: list[float] | None = None
        self._previous: list[int] | None = None
        self._accumulated = [0] * 6
        self._reason: str | None = None

    def rebase(self, leader_ticks: Sequence[int], robot_deg: Sequence[float], reason: str) -> None:
        """Pair two poses; repeating these ticks now reproduces robot_deg exactly."""
        leader = _ticks(leader_ticks)
        robot = _finite_six(robot_deg, "robot_deg")
        if not isinstance(reason, str) or not reason.strip():
            raise ValueError("reason must be a non-empty string")
        for axis, (value, (lower, upper)) in enumerate(zip(robot, self._limits), start=1):
            if not lower <= value <= upper:
                raise ValueError(f"J{axis} anchor {value} is outside model limits [{lower}, {upper}]")
        # No state is changed before every input has passed validation.
        self._leader_anchor = leader[:]
        self._previous = leader[:]
        self._robot_anchor = robot[:]
        self._accumulated = [0] * 6
        self._reason = reason
        self.anchor_version += 1

    def update(self, leader_ticks: Sequence[int]) -> dict:
        if self._previous is None or self._robot_anchor is None:
            raise RuntimeError("rebase must be called before update")
        current = _ticks(leader_ticks)
        changes = [
            ((current[i] - self._previous[i] + TICKS_PER_TURN // 2) % TICKS_PER_TURN)
            - TICKS_PER_TURN // 2
            for i in range(6)
        ]
        accumulated = [self._accumulated[i] + changes[i] for i in range(6)]
        delta = [self._signs[i] * self._scales[i] * accumulated[i] * DEGREES_PER_TICK
                 for i in range(6)]
        raw = [self._robot_anchor[i] + delta[i] for i in range(6)]
        target = [max(lower, min(upper, value)) for value, (lower, upper) in zip(raw, self._limits)]
        clipped = [i + 1 for i in range(6) if target[i] != raw[i]]
        self._previous = current
        self._accumulated = accumulated
        return {
            "target_deg": target,
            "delta_deg": delta,
            "unclamped_target_deg": raw,
            "clipped_axes": clipped,
            "scales": list(self._scales),
        }

    def as_dict(self) -> dict:
        """Return a detached snapshot, including an explicit uninitialized state."""
        return {
            "initialized": self._leader_anchor is not None,
            "version": self.anchor_version,
            "leader_anchor_ticks": None if self._leader_anchor is None else self._leader_anchor[:],
            "robot_anchor_deg": None if self._robot_anchor is None else self._robot_anchor[:],
            "signs": list(self._signs),
            "scales": list(self._scales),
            "limits_deg": [list(pair) for pair in self._limits],
            "reason": self._reason,
        }
