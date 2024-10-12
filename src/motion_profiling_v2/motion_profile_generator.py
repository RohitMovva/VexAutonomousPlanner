import math
from bisect import bisect_left  # Binary Search

import numpy as np

from bezier import cubic_bezier, quadratic_bezier


def max_speed_based_on_curvature(curvature, V_base, K):
    return V_base / (1 + K * curvature)


def distance_to_time(distance, segments):
    left = 0
    right = len(segments) - 1
    mid = None
    while left <= right:
        mid = int(left + (right - left) / 2)
        if segments[mid] < distance:
            left = mid + 1
        elif segments[mid] > distance:
            right = mid - 1
        else:
            break

    return mid / 1000.0


def forward_backwards_smoothing(arr, max_step, depth, delta_dist):
    for i in range(0, len(arr) - 2):  # forward pass
        # TODO Figure out what this equation will look like for acceleration with jerk applied (derivative)
        adjusted_max_step = (
            math.sqrt(arr[i] ** 2 + 2 * max_step * delta_dist) - arr[i]
        )  # only works for trap
        dif = arr[i + 1] - arr[i]

        if dif > adjusted_max_step:  # If negative then we gotta go down anyways
            dif = adjusted_max_step

        arr[i + 1] = arr[i] + dif

    for i in range(len(arr) - 1, 1, -1):  # backward pass nyoom
        adjusted_max_step = (
            math.sqrt(arr[i] ** 2 + 2 * max_step * delta_dist) - arr[i]
        )  # only works for trap

        dif = arr[i] - arr[i - 1]
        if dif < -adjusted_max_step:
            dif = -adjusted_max_step

        arr[i - 1] = arr[i] - dif

    return arr


def generate_other_lists(velocities, control_points, segments, dt):
    # Initialize lists to store positions and accelerations
    positions = [0]  # Assuming initial position is 0
    accelerations = []
    time_intervals = [i * dt for i in range(0, len(velocities))]
    headings = []
    nodes_map = [0]
    coords = []

    # Calculate positions
    for i in range(1, len(velocities)):
        position = positions[-1] + velocities[i] * dt
        positions.append(position)

    # Calculate accelerations
    for i in range(len(velocities) - 1):
        acceleration = (velocities[i + 1] - velocities[i]) / dt
        accelerations.append(acceleration)

    # Add the last acceleration (assuming constant acceleration for the last interval)
    accelerations.append(accelerations[-1])

    # Calculate headings
    current_dist = 0
    current_segment = 0

    for i in range(len(velocities)):
        if (
            current_dist >= segments[current_segment][-1]
            and current_segment < len(segments) - 1
        ):
            current_dist = 0
            current_segment += 1
            nodes_map.append(i)

        t_along_curve = distance_to_time(current_dist, segments[current_segment])
        x = None
        y = None
        if len(control_points[current_segment]) == 3:  # Quadratic Bezier curve
            headings.append(
                quadratic_bezier.quad_bezier_angle(
                    t_along_curve,
                    control_points[current_segment][0],
                    control_points[current_segment][2],
                    control_points[current_segment][1],
                )
            )
            x, y = quadratic_bezier.quadratic_bezier_point(
                control_points[current_segment][0],
                control_points[current_segment][2],
                control_points[current_segment][1],
                t_along_curve,
            )
        else:  # Cubic
            headings.append(
                cubic_bezier.cubic_bezier_angle(
                    t_along_curve,
                    control_points[current_segment][0],
                    control_points[current_segment][2],
                    control_points[current_segment][3],
                    control_points[current_segment][1],
                )
            )
            x, y = cubic_bezier.cubic_bezier_point(
                control_points[current_segment][0],
                control_points[current_segment][2],
                control_points[current_segment][3],
                control_points[current_segment][1],
                t_along_curve,
            )

        coords.append(
            (
                ((x / 2000) - 0.5) * 12.3266567842 * 12,
                ((y / 2000) - 0.5) * 12.3266567842 * 12,
            )
        )

        if i > 0:
            current_dist += positions[i] - positions[i - 1]
        else:
            current_dist = positions[i]

    return (
        time_intervals,
        positions,
        velocities,
        accelerations,
        headings,
        nodes_map,
        coords,
    )


def get_times(velocities, dd):
    res = [0]

    curr_t = 0
    prev_v = 0
    for i in range(1, len(velocities)):
        current_dt = 0
        curr_accel = (velocities[i] ** 2 - prev_v**2) / (dd * 2)  # CORRECT FORMULA

        if abs(curr_accel) > 1e-5:
            current_dt = (velocities[i] - prev_v) / curr_accel
        elif abs(prev_v) > 1e-5:
            current_dt = dd / velocities[i]

        curr_t += current_dt
        prev_v = velocities[i]

        res.append(curr_t)

    return res


def interpolate_velocity(velocities, times, tt):
    place = bisect_left(times, tt)

    if place == 0:
        return 0

    new_velo = np.interp(
        tt, [times[place - 1], times[place]], [velocities[place - 1], velocities[place]]
    )

    return new_velo


def get_wheel_velocities(velocity, curvature, track_width):
    # Credit to robotsquiggles for this math even though it's pretty simple lol
    omega = velocity * curvature

    return (velocity - (track_width / 2) * omega, velocity + (track_width / 2) * omega)


def limit_velocity(velocity, v_max, curvature, track_width):
    wheel_velocities = get_wheel_velocities(velocity, curvature, track_width)

    left_velocity = wheel_velocities[0]
    right_velocity = wheel_velocities[1]

    max_velocity = max(left_velocity, right_velocity)
    if max_velocity > v_max:
        left_velocity = (left_velocity / max_velocity) * v_max
        right_velocity = (right_velocity / max_velocity) * v_max

    return (left_velocity + right_velocity) / 2  # average of both velocities


def generate_motion_profile(
    setpoint_velocities,
    control_points,
    segments,
    v_max,
    a_max,
    j_max,
    track_width,
    dd=0.005,
    dt=0.025,
    K=15.0,
):
    velocities = []

    totalDist = 0
    for segmentList in segments:
        totalDist += segmentList[-1]
    curpos = 0
    while curpos < totalDist - 1e-5:
        velocities.append(0)

        curpos += dd

    velocities.append(0)

    current_dist = 0
    current_segment = 0
    # print(segments)
    for i in range(0, len(velocities)):
        if (
            current_dist >= segments[current_segment][-1]
            and current_segment < len(segments) - 1
        ):
            current_dist = dd
            current_segment += 1

        t_along_curve = distance_to_time(current_dist, segments[current_segment])
        curvature = None
        if len(control_points[current_segment]) < 4:  # Quadratic
            curvature = quadratic_bezier.quadratic_bezier_curvature(
                control_points[current_segment][0],
                control_points[current_segment][2],
                control_points[current_segment][1],
                t_along_curve,
            )
        else:
            curvature = cubic_bezier.cubic_bezier_curvature(
                control_points[current_segment][0],
                control_points[current_segment][2],
                control_points[current_segment][3],
                control_points[current_segment][1],
                t_along_curve,
            )
        curvature *= 2000 / 12.3266567842  # Change from pixels to feet
        adjusted_vmax = limit_velocity(v_max, v_max, curvature, track_width)

        velocities[i] = adjusted_vmax
        current_dist += dd

    velocities[0] = 0
    velocities[-1] = 0

    velocities = forward_backwards_smoothing(velocities, a_max, 0, dd)
    time_stamps = get_times(velocities, dd)

    path_time = time_stamps[-1]

    time_steps = int(path_time / dt) + 1

    new_velocities = []
    for i in range(time_steps):
        new_velo = interpolate_velocity(velocities, time_stamps, i * dt)
        new_velocities.append(new_velo)
    new_velocities.append(0)

    return generate_other_lists(new_velocities, control_points, segments, dt)
