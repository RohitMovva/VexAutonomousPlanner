import math
import numpy as np
from bisect import bisect_left # Binary Search
from bezier.quadratic_bezier import *
from bezier.cubic_bezier import *

def max_speed_based_on_curvature(curvature, V_base, K):
    return V_base / (1 + K * curvature)


def distToTime(distance, segments):
    l = 0
    r = len(segments)-1
    mid = None
    while (l <= r):
        mid = int(l + (r-l)/2)
        if (segments[mid] < distance):
            l = mid+1
        elif (segments[mid] > distance):
            r = mid-1
        else:
            break
        
    return mid/1000.0

def forward_backwards_smoothing(arr, max_step, depth, delta_dist):
    for i in range(0, len(arr)-2): # forward pass
        # TODO Figure out what this equation will look like for acceleration with jerk applied (derivative)
        adjusted_max_step = math.sqrt(arr[i]**2 + 2*max_step*delta_dist) - arr[i] # only works for trap
        dif = arr[i+1]-arr[i]
        
        if (dif > adjusted_max_step): # If negative then we gotta go down anyways
            dif = adjusted_max_step

        arr[i+1] = arr[i] + dif

    for i in range(len(arr)-1, 1, -1): # backward pass nyoom
        adjusted_max_step = math.sqrt(arr[i]**2 + 2*max_step*delta_dist) - arr[i] # only works for trap

        dif = arr[i] - arr[i-1]
        if (dif < -adjusted_max_step):
            dif = -adjusted_max_step

        arr[i-1] = arr[i] - dif
    
    return arr

def generate_other_lists(velocities, control_points, segments, dt):
    # Initialize lists to store positions and accelerations
    positions = [0]  # Assuming initial position is 0
    accelerations = []
    time_intervals = [i*dt for i in range(0, len(velocities))]
    headings = []
    nodes_map = [0]

    # Calculate positions
    for i in range(1, len(velocities)):
        position = positions[-1] + velocities[i-1] * dt
        positions.append(position)

    # Calculate accelerations
    for i in range(len(velocities) - 1):
        acceleration = (velocities[i+1] - velocities[i]) / dt
        accelerations.append(acceleration)

    # Add the last acceleration (assuming constant acceleration for the last interval)
    accelerations.append(accelerations[-1])

    # Calculate headings
    current_dist = 0
    current_segment = 0
    for i in range(len(velocities)):
        if (current_dist >= segments[current_segment][-1] and current_segment < len(segments)-1):
            current_dist = 0
            current_segment += 1
            nodes_map.append(i)

        t_along_curve = distToTime(current_dist, segments[current_segment])
        if (len(control_points[current_segment]) == 3): # Quadratic Bezier curve
            headings.append(
                quad_bezier_angle(t_along_curve, control_points[current_segment][0], control_points[current_segment][2], control_points[current_segment][1]))

        else: # Cubic
            headings.append(
                cubic_bezier_angle(t_along_curve, control_points[current_segment][0], control_points[current_segment][2], control_points[current_segment][3], control_points[current_segment][1]))

        if (i > 0):
            current_dist += positions[i]-positions[i-1]
        else:
            current_dist = positions[i]

    return time_intervals, positions, velocities, accelerations, headings, nodes_map

def get_times(velocities, dd):
    res = []

    curr_t = 0
    prev_v = 0
    for i in range(len(velocities)):
        
        current_dt = 0
        curr_accel = (velocities[i]**2 - prev_v**2)/(dd*2) # CORRECT FORMULA 
        if (i > 0):
            if (abs(curr_accel) > 1e-5):
                current_dt = (velocities[i]-prev_v) / curr_accel
            elif (abs(prev_v) > 1e-5):
                current_dt = dd / velocities[i]
        
        curr_t += current_dt
        prev_v = velocities[i]

        res.append(curr_t)

    return res

def interpolate_velocity(velocities, times, tt):
    place = bisect_left(times, tt)

    if (place == 0):
        return 0

    new_velo = np.interp(tt, [times[place-1], times[place]], [velocities[place-1], velocities[place]])

    return new_velo

def generate_motion_profile(setpoint_velocities, control_points, segments, v_max, a_max, j_max, dd=0.0025, dt=0.0005, K=15.0):
    
    velocities = []

    totalDist = 0
    for segmentList in segments:

        totalDist += segmentList[-1]
    print(totalDist)
    curpos = 0
    while (curpos <= totalDist):
        velocities.append(0)

        curpos += dd

    current_dist = 0
    current_segment = 0
    for i in range(0, len(velocities)):
        if (current_dist > segments[current_segment][-1]):
            current_dist = 0
            current_segment += 1

        t_along_curve = distToTime(current_dist, segments[current_segment])
        curvature = None
        if (len(control_points[current_segment]) < 4): # Quadratic
            curvature = quadratic_bezier_curvature(control_points[current_segment][0], control_points[current_segment][2], control_points[current_segment][1], t_along_curve)
        else:
            curvature = cubic_bezier_curvature(control_points[current_segment][0], control_points[current_segment][2], control_points[current_segment][3], control_points[current_segment][1], t_along_curve)

        adjusted_vmax = max_speed_based_on_curvature(curvature, v_max, K)

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
        new_velo = interpolate_velocity(velocities, time_stamps, i*dt)
        new_velocities.append(new_velo)
    
    return generate_other_lists(new_velocities, control_points, segments, dt)
