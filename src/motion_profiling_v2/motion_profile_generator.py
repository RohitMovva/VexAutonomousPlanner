from math_utilities.newton_raphson_method import *
from math_utilities.time_constant_acceleration import *
from math_utilities.miscmethods import *
from bezier.quadratic_bezier import *
from bezier.cubic_bezier import *

def forward_backwards_smoothing(arr, max_step, depth, delta_dist):
    lower_order_mvs = [] # lower order motion variables (e.g. velocity and in some cases acceleration)
    for i in range(depth):
        lower_order_mvs.append(0)

    for i in range(0, len(arr)-1): # forward pass
        dt = 0
        if (depth == 1):
            dt = calculate_travel_time_constant_acceleration(delta_dist, 0, lower_order_mvs[0], max_step)
        else:
            dt = calculate_travel_time(delta_dist, 0, lower_order_mvs[0], lower_order_mvs[1], max_step)
        adjusted_max_step = max_step * dt
        
        dif = arr[i+1]-arr[i]
        
        if (dif > adjusted_max_step): # If negative then we gotta go down anyways
            dif = adjusted_max_step

        # update motion variables based on dif
        lower_order_mvs[0] += dif*dt # Velocity or Acceleration, depending on which one is directly calculated
        if (depth == 1):
            lower_order_mvs[1] = ((dif*dt)**2)/2 + lower_order_mvs[0]*dt # Calculate velocity if we have const jerk not accel

        arr[i+1] = arr[i] + dif

    for i in range(len(arr)-1, 0, -1): # backward pass nyoom
        dt = 0
        if (depth == 1):
            dt = calculate_travel_time_constant_acceleration(delta_dist, 0, lower_order_mvs[0], max_step)
        else:
            dt = calculate_travel_time(delta_dist, 0, lower_order_mvs[0], lower_order_mvs[1], max_step)
        adjusted_max_step = max_step * dt

        dif = arr[i] - arr[i-1]
        if (dif < -adjusted_max_step):
            dif = -adjusted_max_step

        # update motion variables based on dif
        lower_order_mvs[depth] += (-1*dif)*dt # Velocity or Acceleration, depending on which one is directly calculated
        if (depth == 1):
            lower_order_mvs[0] = (((-1*dif)*dt)**2)/2 + lower_order_mvs[0]*dt # Calculate velocity if we have const jerk not accel

        arr[i-1] = arr[i] - dif
    
    return arr

def generate_other_lists(velocities, control_points, segments, v_max, a_max, j_max, dt, K):
    # Initialize lists to store positions and accelerations
    positions = [0]  # Assuming initial position is 0
    accelerations = []
    time_intervals = [i*dt for i in range(0, len(velocities))]
    headings = []
    nodes_map = []

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
        if (current_dist > segments[current_segment]):
            current_dist = 0
            current_segment += 1

        t_along_curve = distToTime(current_dist, segments[current_segment])
        headings.append(getHeading(t_along_curve, 
            control_points[current_segment][0], control_points[current_segment][2], control_points[current_segment][3], control_points[current_segment][1]))
        
        if (i > 0):
            current_dist += positions[i]-positions[i-1]
        else:
            current_dist = positions[i]

    return time_intervals, positions, velocities, accelerations, headings
    

def generate_motion_profile(setpoint_velocities, control_points, segments, v_max, a_max, j_max, dd=0.0025, K=10.0):
    curvelo = 0
    velocities = []
    accelerations = []
    disttraveled = 0

    while (curvelo < len(setpoint_velocities)):
        velocities.append(-1)
        accelerations.append(0)

        if (disttraveled >= setpoint_velocities[curvelo]):
            velocities[-1] = setpoint_velocities[curvelo]
            curvelo += 1

        disttraveled += dd
    

    current_dist = 0
    current_segment = 0
    for i in range(0, len(velocities)):
        if (current_dist > segments[current_segment]):
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

    forward_backwards_smoothing(velocities, a_max, 1, dd)

    velocities = convert_velocity_parameterization(velocities, dd, dt=0.0005)




"""
have constraint in seconds
have dif in distance

converting from distance to seconds is complicated
converting from seconds to distance is easy

finding continous velo: building trapezoidal
starting velocity
CONSTANT acceleartion

finding continous acceleration: smoothing into s curve
starting velocity <- finding time it takes to move a distance so velo still matters
starting accelration
constant jerk

how do we find velocity acceleration and jerk in our smoothing function?

might pass extra parameter representing depth
calculate lower order motion variables based on applied variable (acceleration or jerk)
"""