"""Two stops a fraction of a grid step apart must not be crossed at MIN_PROFILE_SPEED.

skills (2026-10-23): a stop action point 0.1 in before a stop node was left as a
neighbouring sample with no grid point between, and the profile crept across the gap
for 8.2 s. Where the gap lands against the grid decides whether it happens, so the
test slides the action point across a whole grid step.
"""
import numpy as np
import pytest

from utilities import headless_profile as hp

MOTION = {"max_velocity": 3.5, "max_acceleration": 8.0}


def _duration(action_t=None):
    nodes = [
        hp.HeadlessNode(0.0, 0.0, is_start_node=True),
        hp.HeadlessNode(4.0, 0.0, stop_mode=hp.STOP_FORCED),
        hp.HeadlessNode(8.0, 0.0, is_end_node=True),
    ]
    points = [] if action_t is None else [hp.HeadlessActionPoint(0, 0, t=action_t, stop=True)]
    manager = hp.build_spline_manager(nodes, points)
    constraints = hp.constraints_from_config(MOTION, 11.35)
    return hp.profile_route(manager, constraints).times[-1]


@pytest.mark.parametrize("gap_t", np.linspace(0.0005, 0.006, 12))
def test_stop_point_just_before_a_stop_node(gap_t):
    # 1.0 is the middle node; gap_t of the parameter is a few hundredths of an inch
    # to a little over a grid step
    assert _duration(1.0 - gap_t) < _duration() + 0.5
