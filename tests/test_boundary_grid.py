"""A node landing next to a distance-grid point must not slow the profile.

truelocalsawpalt (OverrideAutonomous boot 0092, 2026-10-23): node 9 sat 0.0003 ft past a
grid point. The curvature step at the node (same tangent derivative, unequal handle
magnitudes) over that sliver read as dkappa/ds ~ 3.2 /ft^2, and the yaw-accel bound
cut a 35 in/s cruise to 18 in/s for no physical reason.
"""
import numpy as np
import pytest

from motion_profiling_v2 import motion_profile_generator as mpg
from utilities import headless_profile as hp

DD = 0.005


def make_constraints():
    return mpg.Constraints(
        max_vel=3.5, max_acc=12.0, max_dec=12.0, friction_coef=0.8, max_jerk=16.0,
        track_width=11.35 / 12, stall_accel=12.0, free_speed=4.8, traction_accel=12.0,
        steering_speed_penalty=0.28, max_angular_velocity=1.8,
        max_angular_acceleration=7.0,
    )


def curved_node_path(shift):
    """Three nodes; the middle one has a tangent derivative and unequal handles, so
    curvature steps slightly across it. shift slides the start node, which moves the
    middle node's distance relative to the grid."""
    nodes = [
        hp.HeadlessNode(-shift, 0.0, is_start_node=True),
        hp.HeadlessNode(
            4.0, 0.5, tangent=np.array([0.998643, 0.052086]),
            curvature=np.array([0.166137, -0.211988]),
            incoming_magnitude=1.65, outgoing_magnitude=2.63,
        ),
        hp.HeadlessNode(8.0, -0.5, is_end_node=True),
    ]
    return hp.build_spline_manager(nodes)


def speed_at_node(shift):
    manager = curved_node_path(shift)
    hp.profile_route(manager, make_constraints())  # builds the lookup tables
    velocities, node_samples, dists = mpg.forward_backward_pass(
        manager, make_constraints(), DD
    )
    i = node_samples[1]
    return velocities[i], np.diff(dists)


@pytest.mark.parametrize("shift", np.linspace(0.0, DD, 11))
def test_grid_has_no_sliver_intervals(shift):
    _, gaps = speed_at_node(shift)
    assert gaps.min() >= DD / 2 - 1e-9


def test_node_speed_does_not_depend_on_grid_alignment():
    # The shift moves the start node, so the geometry (and the speed) drifts a little
    # on its own: ~0.08 ft/s across the sweep. A sliver interval dipped it ~0.6 ft/s.
    speeds = [speed_at_node(s)[0] for s in np.linspace(0.0, DD, 41)]
    assert max(speeds) - min(speeds) < 0.15, speeds
