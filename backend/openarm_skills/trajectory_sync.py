"""Dependency-free time alignment for trajectories two arms run at once.

`move_group` plans one goal at a time and each plan sees the other arm frozen
at its start pose, so a pair of trajectories dispatched together is never
checked against itself. The ROS adapter samples both onto a common time grid
with these helpers and feeds the combined states to `/check_state_validity`.
"""

from __future__ import annotations

import math
from typing import Sequence

# One trajectory point: seconds from the start of the motion, joint positions.
TrajectoryPoint = tuple[float, tuple[float, ...]]


def duration(points: Sequence[TrajectoryPoint]) -> float:
    if not points:
        return 0.0
    return points[-1][0]


def sample_at(points: Sequence[TrajectoryPoint], when: float) -> tuple[float, ...]:
    """Positions at `when`, holding the endpoints outside the trajectory.

    Holding matters after the end: an arm that already stopped is still
    standing somewhere, and the arm starting later can collide with it there.
    """

    if not points:
        raise ValueError("cannot sample an empty trajectory")
    if when <= points[0][0]:
        return points[0][1]
    if when >= points[-1][0]:
        return points[-1][1]
    for index in range(1, len(points)):
        end_time, end_positions = points[index]
        if when > end_time:
            continue
        start_time, start_positions = points[index - 1]
        span = end_time - start_time
        if span <= 0.0:
            return end_positions
        ratio = (when - start_time) / span
        return tuple(
            start + (end - start) * ratio
            for start, end in zip(start_positions, end_positions)
        )
    return points[-1][1]


def sample_times(
    total: float, step: float = 0.1, max_samples: int = 30
) -> list[float]:
    """Check times spanning [0, total], both endpoints included.

    The grid is coarsened rather than truncated once `max_samples` is reached,
    so a long trajectory stays covered end to end for a bounded number of
    service round trips. Both defaults are a latency budget: every sample is
    one round trip to move_group, and the arm being checked cannot start until
    the sweep finishes.
    """

    if total <= 0.0:
        return [0.0]
    intervals = min(max_samples, max(1, math.ceil(total / step)))
    return [total * index / intervals for index in range(intervals + 1)]
