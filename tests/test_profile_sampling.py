"""The reference pose must integrate the reference velocity within each interval."""
import pytest

from motion_profiling_v2.motion_profile_generator import sample_profile_state


@pytest.mark.parametrize("velocities", [[0.12, 3.12], [3.12, 0.12], [1.62, 1.62]])
def test_position_integrates_velocity(velocities):
    duration = 0.1
    distance = duration * sum(velocities) / 2
    times, distances = [0, duration], [7, 7 + distance]
    # Trapezoid integration is exact for this interval's linear velocity.
    previous_s, previous_v = sample_profile_state(0, times, distances, velocities)
    for j in range(1, 101):
        s, v = sample_profile_state(j * 0.001, times, distances, velocities)
        assert s - previous_s == pytest.approx((v + previous_v) * 0.0005)
        previous_s, previous_v = s, v
    assert previous_s == pytest.approx(distances[-1])


def test_nonuniform_intervals_and_clamped_endpoints():
    times, distances, velocities = [0, 1, 3], [0, 1, 3], [0, 2, 0]
    for t, expected in [(-1, (0, 0)), (0.5, (0.25, 1)), (1, (1, 2)),
                        (2, (2.5, 1)), (3, (3, 0)), (4, (3, 0))]:
        assert sample_profile_state(t, times, distances, velocities) == pytest.approx(expected)
