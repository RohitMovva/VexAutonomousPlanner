import math
from bisect import bisect_left  # Binary Search

import numpy as np
from splines.spline_manager import QuinticHermiteSplineManager

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
    for i in range(0, len(velocities) - 2):  # forward pass
        # TODO Figure out what this equation will look like for acceleration with jerk applied (derivative)
        adjusted_max_step = math.sqrt(velocities[i] ** 2 + 2 * max_accel * delta_dist) - abs(velocities[i])
          # only works for trap
        dif = abs(velocities[i + 1]) - abs(velocities[i])

        if dif > adjusted_max_step:
            dif = adjusted_max_step

        dif = np.sign(velocities[i + 1] - velocities[i]) * abs(dif)

        if dif > adjusted_max_step:  # If negative then we gotta go down anyways
            dif = adjusted_max_step

        velocities[i + 1] = velocities[i] + dif

    for i in range(len(velocities) - 1, 1, -1):  # backward pass nyoom
        adjusted_max_step = math.sqrt(velocities[i] ** 2 + 2 * max_accel * delta_dist) - abs(velocities[i])
        dif = abs(velocities[i]) - abs(velocities[i - 1])
        if dif < adjusted_max_step:
            dif = adjusted_max_step

        dif = np.sign(velocities[i] - velocities[i - 1]) * abs(dif)

        velocities[i - 1] = velocities[i] - dif

    return velocities



def generate_other_lists(velocities, spline_manager: QuinticHermiteSplineManager, dt, turn_insertions, turn_vals, reverse_values, wait_times):
    # Initialize lists to store positions and accelerations
    positions = [0]  # Assuming initial position is 0
    accelerations = []
    time_intervals = [i * dt for i in range(0, len(velocities))]
    headings = []
    angular_velocities = []  # New list for angular velocities
    nodes_map = [0]
    coords = []
    spline_manager.rebuild_tables()

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
    old_t = 0
    for i in range(len(velocities)):

        t_along_curve = spline_manager.distance_to_time(current_dist)
        if (t_along_curve%1 < old_t%1):
            nodes_map.append(i)
        old_t = t_along_curve
        x = None
        y = None

        headings.append(spline_manager.get_heading(t_along_curve))
        x, y = spline_manager.get_point_at_parameter(t_along_curve)
        if (reverse_values[current_segment]):
            headings[-1] = (headings[-1] - math.pi)
            velocities[i] = -velocities[i]
            accelerations[i] = -accelerations[i]

        if (headings[-1] < -math.pi):
            headings[-1] += 2*math.pi

        coords.append(
            (
                x,
                y * -1,
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
            # Normalize the angle difference to be between -pi and pi
            if delta_heading > math.pi:
                delta_heading -= 2*math.pi
            elif delta_heading < -math.pi:
                delta_heading += 2*math.pi
            # Convert to radians per second
            angular_velocity = (delta_heading) / dt
            angular_velocities.append(angular_velocity)
        else:
            # For the first point, assume zero angular velocity
            angular_velocities.append(0.0)

    # Insert the turn on point trapezoidal velocity profiles into the motion profile
    # x, y, linear velocity will stay the same, just insert a bunch of the same values
    # update the heading and angular velocity
    coffset = 0
    for k in range(len(turn_vals)):
        i = nodes_map[k] + coffset
        temp_headings = [headings[i]-turn_vals[k]] # need to reach that heading so subtract value we're turning by before that point
        if (temp_headings[0] < -math.pi):
            temp_headings[0] += 2*math.pi
        temp_angular_velocities = []
        temp_coords = []
        temp_positions = []
        temp_velocities = []
        temp_accelerations = []
        temp_time_intervals = []

        for j in range(len(turn_insertions[k])):
            temp_headings.append((turn_insertions[k][j] * dt) + temp_headings[-1])
            if (temp_headings[-1] < -math.pi):
                temp_headings[-1] += 2*math.pi
            elif (temp_headings[-1] > math.pi):
                temp_headings[-1] -= 2*math.pi
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
    for k in range(len(wait_times)):
        
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
    arc_length = (abs(turn_angle)) * radius

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
    spline_manager: QuinticHermiteSplineManager,
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

    # totalDist = 0
    # for segmentList in segments:
    #     totalDist += segmentList[-1]
    dist = spline_manager.get_total_arc_length()
    curpos = 0
    while curpos < dist - 1e-5:
        velocities.append(0)

        curpos += dd

    velocities.append(0)

    current_dist = 0
    for i in range(0, len(velocities)):
        # print("i = ", i)
        t_along_curve = spline_manager.distance_to_time(current_dist)
        curvature = spline_manager.get_curvature(t_along_curve)
        heading = spline_manager.get_heading(t_along_curve)

        curvatures.append(curvature)
        headings.append(heading)
        adjusted_vmax = limit_velocity(v_max, v_max, curvature, track_width)
        velocities[i] = adjusted_vmax
        current_dist += dd

    velocities[0] = 0
    velocities[-1] = 0

    velocities = forward_backwards_smoothing(velocities, a_max, dd, track_width, curvatures)

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
    res = generate_other_lists(new_velocities, spline_manager, dt, turn_insertions, turn_values, reverse_values, wait_times)
    return res
