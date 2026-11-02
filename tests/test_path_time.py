"""Status bar path time text, checked against a real profiled route."""

from pathlib import Path

import pytest
import yaml

from gui.gui_manager import format_path_time
from utilities import headless_profile as hp

REPO = Path(__file__).resolve().parents[1]
SUITE_ROUTE = REPO / "suite" / "routes" / "arc_R12.txt"


@pytest.fixture(scope="module")
def profiled():
    config = yaml.safe_load((REPO / "config.yaml").read_text())
    route = hp.load_route_file(SUITE_ROUTE, config["robot"].get("actions") or [])
    nodes, action_points = hp.nodes_from_route(route)
    constraints = hp.constraints_from_config(
        config["motion"], config["robot"]["track_width"]
    )
    return hp.profile_route(hp.build_spline_manager(nodes, action_points), constraints)


def test_label_shows_profile_duration(profiled):
    text, tooltip = format_path_time(
        profiled.times, profiled.positions, profiled.linear_vels, profiled.report
    )
    assert text == f"Path time: {profiled.report.duration:.2f} s"
    assert f"Path time: {profiled.times[-1]:.2f} s" in tooltip
    assert f"Distance: {profiled.positions[-1] * 12:.1f} in" in tooltip


def test_stationary_time_counts_waits(profiled):
    # Append a 1 s wait at the end: the robot is stopped for exactly that long.
    times = list(profiled.times) + [profiled.times[-1] + 0.01 * i for i in range(1, 101)]
    vels = list(profiled.linear_vels) + [0.0] * 100
    _, tooltip = format_path_time(times, profiled.positions, vels, profiled.report)
    stopped = float(tooltip.split("Stopped (waits and turns): ")[1].split(" s")[0])
    assert stopped == pytest.approx(1.0, abs=0.02)


def test_warnings_are_listed(profiled):
    report = profiled.report
    original = report.warnings
    report.warnings = ["yaw rate over budget"]
    try:
        _, tooltip = format_path_time(
            profiled.times, profiled.positions, profiled.linear_vels, report
        )
    finally:
        report.warnings = original
    assert "Warning: yaw rate over budget" in tooltip
