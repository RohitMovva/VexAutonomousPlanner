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

    return mid / len(segments)


def forward_backwards_smoothing(velocities, max_accel, delta_dist, track_width, curvatures):
    # Forward pass
    all_wheel_velocities = [[0, 0]]
    old_wheel_velocities = list(get_wheel_velocities(velocities[0], curvatures[0], track_width))
    print("=== FORWARD PASS START ===")
    for i in range(0, len(velocities) - 2):  # forward pass
        # Compute adjusted max step
        adjusted_max_step = math.sqrt(velocities[i] ** 2 + 2 * max_accel * delta_dist) - abs(velocities[i])
        dif = abs(velocities[i + 1]) - abs(velocities[i])

        if dif > adjusted_max_step:
            dif = adjusted_max_step

        dif = np.sign(velocities[i + 1] - velocities[i]) * abs(dif)
        # dif = np.copysign(dif, velocities[i + 1] - velocities[i])

        # old_wheel_velocities = list(get_wheel_velocities(velocities[i], curvatures[i], track_width))
        new_wheel_velocities = list(get_wheel_velocities(velocities[i] + dif, curvatures[i+1], track_width))

        left_arc_length = delta_dist * (1 - curvatures[i] * track_width / 2)
        right_arc_length = delta_dist * (1 + curvatures[i] * track_width / 2)

        left_max_step = (math.sqrt(all_wheel_velocities[-1][0] ** 2 + 2 * -1*max_accel * -1*abs(left_arc_length))) - abs(all_wheel_velocities[-1][0])
        right_max_step = (math.sqrt(all_wheel_velocities[-1][1] ** 2 + 2 * -1*max_accel * -1*abs(right_arc_length))) - abs(all_wheel_velocities[-1][1])

        left_wheel_dif = abs(new_wheel_velocities[0]) - abs(old_wheel_velocities[0])
        right_wheel_dif = abs(new_wheel_velocities[1]) - abs(old_wheel_velocities[1])

        # Debug prints
        print(f"Forward i={i}")
        print(f" Left max step calculation breakdown:")
        print(f"  curvatures[i+1]: {curvatures[i+1]}, track_width: {track_width}, velocities[i]+dif: {velocities[i]+dif}")
        print(f" velocities[i]: {velocities[i]} vel[i+1]: {velocities[i+1]} dif: {dif} adjusted_max_step: {adjusted_max_step}")
        print(f"  old_wheel_velocities: {old_wheel_velocities}")
        print(f" new_wheel_velocities: {new_wheel_velocities}")
        print(f"  left_arc_length: {left_arc_length}, right_arc_length: {right_arc_length}")
        print(f" left_step_info arc_length: {math.sqrt(old_wheel_velocities[0] ** 2 + 2 * max_accel * abs(left_arc_length))}, other thing: {abs(all_wheel_velocities[-1][0])}")
        print(f" left_max_step: {left_max_step}, right_max_step: {right_max_step}")

        # Clamp wheel diffs
        if left_wheel_dif > left_max_step:
            left_wheel_dif = left_max_step
        if right_wheel_dif > right_max_step:
            right_wheel_dif = right_max_step

        left_wheel_dif = np.sign(new_wheel_velocities[0] - old_wheel_velocities[0]) * abs(left_wheel_dif)
        # left_wheel_dif = np.copysign(left_wheel_dif, new_wheel_velocities[0] - old_wheel_velocities[0])
        right_wheel_dif = np.sign(new_wheel_velocities[1] - old_wheel_velocities[1]) * abs(right_wheel_dif)
        # right_wheel_dif = np.copysign(right_wheel_dif, new_wheel_velocities[1] - old_wheel_velocities[1])
        print(f"  left_wheel_dif: {left_wheel_dif}, right_wheel_dif: {right_wheel_dif}")

        old_wheel_velocities[0] += left_wheel_dif
        old_wheel_velocities[1] += right_wheel_dif

        # Debug after clamping
        print(f"  After clamp wheel velocities: {old_wheel_velocities}")

        all_wheel_velocities.append(old_wheel_velocities[:])

        velocities[i + 1] = (old_wheel_velocities[0] + old_wheel_velocities[1]) / 2
        # old_wheel_velocities = list(get_wheel_velocities(velocities[i+1], curvatures[i+1], track_width))
        # print(f"  right: {old_wheel_velocities[1]}, left: {old_wheel_velocities[0]}")
        print()

    # Instead of resetting, just ensure final velocity is zero at the end
    # Or store forward pass results first
    # For diagnosing, let's just print what we got:
    print("=== FORWARD PASS COMPLETE ===")
    print("all_wheel_velocities after forward pass:", all_wheel_velocities)
    print("velocities after forward pass:", velocities)
    print()

    # Backward pass
    # Set the final wheel velocities to zero (end condition)
    # Just overwrite the last entry in all_wheel_velocities
    # or if you must reinit, store forward results first.
    # For now, let's try:
    # end_index = len(velocities) - 1
    # # Set final velocity to 0
    # velocities[end_index] = 0
    # last_wheel_vel = get_wheel_velocities(velocities[end_index], curvatures[end_index], track_width)
    # all_wheel_velocities = [[last_wheel_vel[0], last_wheel_vel[1]]]  # Should be [0,0]
    # old_wheel_velocities = all_wheel_velocities[-1]
    
    # print("=== BACKWARD PASS START ===")
    # for i in range(len(velocities) - 1, 1, -1):
    #     adjusted_max_step = math.sqrt(velocities[i] ** 2 + 2 * max_accel * delta_dist) - abs(velocities[i])
    #     dif = abs(velocities[i]) - abs(velocities[i - 1])
    #     if dif < adjusted_max_step:
    #         dif = adjusted_max_step

    #     dif = np.sign(velocities[i] - velocities[i - 1]) * abs(dif)

    #     # old_wheel_velocities = list(get_wheel_velocities(velocities[i], curvatures[i], track_width))
    #     new_wheel_velocities = list(get_wheel_velocities(velocities[i] - dif, curvatures[i-1], track_width))

    #     left_arc_length = delta_dist * (1 - curvatures[i] * track_width / 2)
    #     right_arc_length = delta_dist * (1 + curvatures[i] * track_width / 2)

    #     left_max_step = math.sqrt(old_wheel_velocities[0] ** 2 + 2 * max_accel * abs(left_arc_length)) - abs(all_wheel_velocities[-1][0])
    #     right_max_step = math.sqrt(old_wheel_velocities[1] ** 2 + 2 * max_accel * abs(right_arc_length)) - abs(all_wheel_velocities[-1][1])

    #     left_wheel_dif = abs(old_wheel_velocities[0]) - abs(new_wheel_velocities[0])
    #     right_wheel_dif = abs(old_wheel_velocities[1]) - abs(new_wheel_velocities[1])

    #     # Debug prints for backward pass
    #     print(f"Backward i={i}")
    #     print(f" Left max step calculation breakdown:")
    #     print(f" velocities[i]: {velocities[i]} vel[i+1]: {velocities[i-1]} dif: {dif} adjusted_max_step: {adjusted_max_step}")
    #     print(f"  old_wheel_velocities: {old_wheel_velocities}")
    #     print(f" new_wheel_velocities: {new_wheel_velocities}")
    #     # print(f"  left_arc_length: {left_arc_length}, right_arc_length: {right_arc_length}")
    #     # print(f" left_step_info arc_length: {math.sqrt(old_wheel_velocities[0] ** 2 + 2 * max_accel * abs(left_arc_length))}, other thing: {abs(all_wheel_velocities[-1][0])}")
    #     print(f" left_max_step: {left_max_step}, right_max_step: {right_max_step}")
    #     # print(f"  Initial Vel: {velocities[i]}")
    #     # print(f"  adjusted_max_step: {adjusted_max_step}, dif: {dif}")
    #     # print(f"  old_wheel_velocities: {old_wheel_velocities}, new_wheel_velocities: {new_wheel_velocities}")
    #     # print(f"  left_max_step: {left_max_step}, right_max_step: {right_max_step}")
    #     print(f"  left_wheel_dif: {left_wheel_dif}, right_wheel_dif: {right_wheel_dif}")

    #     if left_wheel_dif < -left_max_step:
    #         left_wheel_dif = left_max_step
    #     if right_wheel_dif < -right_max_step:
    #         right_wheel_dif = right_max_step

    #     left_wheel_dif = abs(left_wheel_dif)*np.sign(old_wheel_velocities[0] - new_wheel_velocities[0])
    #     right_wheel_dif = abs(right_wheel_dif)*np.sign(old_wheel_velocities[1] - new_wheel_velocities[1])

    #     old_wheel_velocities[0] -= left_wheel_dif
    #     old_wheel_velocities[1] -= right_wheel_dif

    #     # Debug after clamping
    #     print(f" left wheel dif: {left_wheel_dif}, right wheel dif: {right_wheel_dif}")
    #     print(f"  After clamp wheel velocities: {old_wheel_velocities}")

    #     all_wheel_velocities.append(old_wheel_velocities[:])
    #     velocities[i - 1] = (old_wheel_velocities[0] + old_wheel_velocities[1]) / 2

    #     # print(f"  curvatures[i+1]: {curvatures[i-1]}, track_width: {track_width}, velocities[i+1]: {velocities[i-1]}")
    #     # print(f"  new old velocities[i+1]: {old_wheel_velocities}")
    #     print()

    # print("=== BACKWARD PASS COMPLETE ===")
    # # print("all_wheel_velocities after backward pass:", all_wheel_velocities)
    # # print("velocities after backward pass:", velocities)
    for velpair in all_wheel_velocities:
        print(velpair[0], velpair[1])

    print("=== SMOOTHING COMPLETE ===")
    
    return velocities



def generate_other_lists(velocities, control_points, segments, dt, turn_insertions, turn_vals, reverse_values, wait_times):
    # Initialize lists to store positions and accelerations
    positions = [0]  # Assuming initial position is 0
    accelerations = []
    time_intervals = [i * dt for i in range(0, len(velocities))]
    headings = []
    angular_velocities = []  # New list for angular velocities
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

        if (reverse_values[current_segment]):
            headings[-1] = (headings[-1] - 180)
            velocities[i] = -velocities[i]
            accelerations[i] = -accelerations[i]

        if (headings[-1] < -180):
            headings[-1] += 360

        coords.append(
            (
                ((x / 2000) - 0.5) * 12.3266567842 * 12,
                ((y / 2000) - 0.5) * 12.3266567842 * 12 * -1,
            )
        )

        if i > 0:
            current_dist += positions[i] - positions[i - 1]
        else:
            current_dist = positions[i]

        # Calculate angular velocity
        if i > 0:
            # Calculate the change in heading angle
            delta_heading = headings[i] - headings[i - 1]
            # Normalize the angle difference to be between -180 and 180 degrees
            if delta_heading > 180:
                delta_heading -= 360
            elif delta_heading < -180:
                delta_heading += 360
            # Convert to radians per second
            angular_velocity = (delta_heading * (math.pi / 180)) / dt
            angular_velocities.append(angular_velocity)
        else:
            # For the first point, assume zero angular velocity
            angular_velocities.append(0.0)

    # Insert the turn on point trapezoidal velocity profiles into the motion profile
    # x, y, linear velocity will stay the same, just insert a bunch of the same values
    # update the heading and angular velocity
    coffset = 0
    for k in range(len(nodes_map)):
        i = nodes_map[k] + coffset
        temp_headings = [headings[i]-turn_vals[k]] # need to reach that heading so subtract value we're turning by before that point
        if (temp_headings[0] < -180):
            temp_headings[0] += 360
        temp_angular_velocities = []
        temp_coords = []
        temp_positions = []
        temp_velocities = []
        temp_accelerations = []
        temp_time_intervals = []

        for j in range(len(turn_insertions[k])):
            temp_headings.append((turn_insertions[k][j] * dt) * (180/math.pi) + temp_headings[-1])
            if (temp_headings[-1] < -180):
                temp_headings[-1] += 360
            elif (temp_headings[-1] > 180):
                temp_headings[-1] -= 360
            temp_angular_velocities.append(turn_insertions[k][j])
            temp_coords.append(coords[i])
            temp_positions.append(positions[i])
            temp_velocities.append(velocities[i])
            temp_accelerations.append(accelerations[i])
            temp_time_intervals.append(time_intervals[i] + j * dt)

        coffset += len(temp_time_intervals)
        # Insert the temporary lists into the main lists
        headings[i:i] = temp_headings[1:]
        angular_velocities[i:i] = temp_angular_velocities
        coords[i:i] = temp_coords
        positions[i:i] = temp_positions
        velocities[i:i] = temp_velocities
        accelerations[i:i] = temp_accelerations
        time_intervals[i:i] = temp_time_intervals

        # Update the time intervals after the insertion
        offset = dt * len(temp_time_intervals)
        for j in range(i + len(temp_time_intervals), len(time_intervals)):
            time_intervals[j] += offset

        nodes_map[k] += coffset

    coffset = 0
    for k in range(len(nodes_map)):
        
        # nodes_map[k] += offset
        i = nodes_map[k] + coffset
        temp_headings = []
        temp_angular_velocities = []
        temp_coords = []
        temp_positions = []
        temp_velocities = []
        temp_accelerations = []
        temp_time_intervals = []

        for j in range(int(wait_times[k]/dt)):
            temp_headings.append(headings[i])
            temp_angular_velocities.append(0)
            temp_coords.append(coords[i])
            temp_positions.append(positions[i])
            temp_velocities.append(0)
            temp_accelerations.append(0)
            temp_time_intervals.append(time_intervals[i] + j * dt)

        coffset += len(temp_time_intervals)
        # Insert the temporary lists into the main lists
        headings[i:i] = temp_headings
        angular_velocities[i:i] = temp_angular_velocities
        coords[i:i] = temp_coords
        positions[i:i] = temp_positions
        velocities[i:i] = temp_velocities
        accelerations[i:i] = temp_accelerations
        time_intervals[i:i] = temp_time_intervals

        # Update the time intervals after the insertion
        offset = dt * len(temp_time_intervals)
        for j in range(i + len(temp_time_intervals), len(time_intervals)):
            time_intervals[j] += offset

        nodes_map[k] += coffset
    
    return (
        time_intervals,
        positions,
        velocities,
        accelerations,
        headings,
        angular_velocities,  # Added to return tuple
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
    # Credit to robotsquiggles for this math even though it's pretty simple ¯\_(ツ)_/¯
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


def calculate_turn_velocities(turn_angle, track_width, v_max, a_max, j_max, dt):
    # Figure out how far each side of the drivetrain needs to travel to make the turn
    radius = track_width / 2
    # Find arc length based on radius and angle
    # arc_length = (turn_angle / 360) * 2 * math.pi * radius
    arc_length = (math.pi/180 * abs(turn_angle)) * radius

    # Find time needed for each phase, this is acceleration, constant velocity, and deceleration
    acceleration_time = v_max / a_max
    deceleration_time = v_max / a_max
    # constant velocity time will be equal to distance left over by acceleration and deceleration over v_max
    constant_velocity_time = (
        arc_length - (v_max * acceleration_time) - (v_max * deceleration_time)
    ) / v_max

    # If we can't reach v_max in the acceleration phase, we need to adjust the acceleration time
    if constant_velocity_time < 0:
        # We need to figure out how long we can accelerate for before reaching half of our turn_angle
        acceleration_time = math.sqrt(arc_length / a_max)
        deceleration_time = acceleration_time
        constant_velocity_time = 0

    # Find the angular velocity every dt along the turn
    angular_velocities = []
    acc_val = 0
    acc_pos = 0
    for i in range(0, math.ceil(acceleration_time / dt)):
        # Figure out our angular velocity based on how quickly we are turning at this point in time. Result should be in radians per second
        # Find speed at this point
        new_dt = dt
        new_acc = a_max
        if (i == int(acceleration_time / dt)):
            new_dt = acceleration_time - (i)*dt
            new_acc = (new_dt/dt) * a_max

        speed = (((i-1) * dt + dt) * new_acc)
        acc_pos += speed * dt + 0.5 * new_acc * (dt ** 2)
        # Find the radius of the circle we are turning on
        radius = track_width / 2
        # Find the angular velocity
        angular_velocity = speed / radius
        acc_val += angular_velocity * dt
        angular_velocities.append(angular_velocity * np.sign(turn_angle))

    for i in range(0, int(constant_velocity_time / dt)):
        angular_velocities.append(angular_velocity * np.sign(turn_angle))

    acc_val = 0
    acc_pos = 0
    for i in range(0, math.ceil(deceleration_time / dt)):
        # Figure out our angular velocity based on how quickly we are turning at this point in time. Result should be in radians per second
        # Find speed at this point
        # Find speed that we reached after the acceleration phase and subtract how much we've decelerated from that
        new_dt = dt
        new_acc = a_max
        if (i == int(deceleration_time / dt)):
            new_dt = deceleration_time - (i)*dt
            new_acc = (new_dt/dt) * a_max

        max_reached_speed = acceleration_time * new_acc
        speed = max_reached_speed - (((i-1) * dt + dt) * new_acc)
        acc_pos += speed * dt - 0.5 * new_acc * (dt ** 2)
        # Find the radius of the circle we are turning on
        radius = track_width / 2
        # Find the angular velocity
        angular_velocity = speed / radius
        acc_val += angular_velocity * dt
        angular_velocities.append(angular_velocity * np.sign(turn_angle))

    return angular_velocities


def generate_motion_profile(
    setpoint_velocities,
    control_points,
    segments,
    v_max,
    a_max,
    j_max,
    track_width,
    turn_values,
    reverse_values,
    wait_times,
    dd=0.005,
    dt=0.025,
    K=15.0,
):
    velocities = []
    curvatures = []
    headings = []

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
            heading = quadratic_bezier.quad_bezier_angle(
                t_along_curve,
                control_points[current_segment][0],
                control_points[current_segment][2],
                control_points[current_segment][1],
            )
        else:
            curvature = cubic_bezier.cubic_bezier_curvature(
                control_points[current_segment][0],
                control_points[current_segment][2],
                control_points[current_segment][3],
                control_points[current_segment][1],
                t_along_curve,
            )
            heading = cubic_bezier.cubic_bezier_angle(
                t_along_curve,
                control_points[current_segment][0],
                control_points[current_segment][2],
                control_points[current_segment][3],
                control_points[current_segment][1],
            )

        curvature *= 2000 / 12.3266567842  # Change from pixels to feet
        curvatures.append(curvature)
        headings.append(heading)
        adjusted_vmax = limit_velocity(v_max, v_max, curvature, track_width)

        velocities[i] = adjusted_vmax
        current_dist += dd

    velocities[0] = 0
    velocities[-1] = 0

    velocities = forward_backwards_smoothing(velocities, a_max, dd, track_width, curvatures)
    # velocities = forward_backwards_smoothing(velocities, a_max, dd, track_width, curvatures)
    # velocities = forward_backwards_smoothing(velocities, a_max, dd, track_width, curvatures)
    # velocities = forward_backwards_smoothing(velocities, a_max, dd, track_width, curvatures)

    # Calculate angular velocities based on velocities list and curvatures at each point
    angular_velocities = []
    for i in range(len(velocities)):
        angular_velocity = velocities[i] * curvatures[i]
        angular_velocities.append(angular_velocity)
    
    time_stamps = get_times(velocities, dd)

    path_time = time_stamps[-1]

    time_steps = int(path_time / dt) + 1

    new_velocities = []
    for i in range(time_steps):
        new_velo = interpolate_velocity(velocities, time_stamps, i * dt)
        new_velocities.append(new_velo)
    new_velocities.append(0)

    turn_insertions = []
    for turn in turn_values:
        # Calculate a trapezoidal velocity profile for the turn
        angular_velocities = calculate_turn_velocities(
            turn, track_width, v_max, a_max, j_max, dt
        )

        acc_val = 0
        for velo in angular_velocities:
            acc_val += velo * dt

        
        turn_insertions.append(angular_velocities)
    res = generate_other_lists(new_velocities, control_points, segments, dt, turn_insertions, turn_values, reverse_values, wait_times)
    # print(res)
    return res
