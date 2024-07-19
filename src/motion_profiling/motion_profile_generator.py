import numpy as np
from scipy.interpolate import interp1d

def evaluate_bezier(t, p0, p1, p2, p3=None):
    p0, p1, p2 = np.array(p0), np.array(p1), np.array(p2)
    if p3 is None:
        return (1-t)**2 * p0 + 2*(1-t)*t * p1 + t**2 * p2
    p3 = np.array(p3)
    return (1-t)**3 * p0 + 3*(1-t)**2*t * p1 + 3*(1-t)*t**2 * p2 + t**3 * p3

def bezier_derivative(t, p0, p1, p2, p3=None):
    p0, p1, p2 = np.array(p0), np.array(p1), np.array(p2)
    if p3 is None:
        return 2 * ((p1 - p0) * (1 - t) + (p2 - p1) * t)
    p3 = np.array(p3)
    return 3 * ((p1 - p0) * (1-t)**2 + (p2 - p1) * 2*(1-t)*t + (p3 - p2) * t**2)

def get_bezier_length(p0, p1, p2, p3=None, num_samples=1000):
    t = np.linspace(0, 1, num_samples)
    if p3 is None:
        points = np.array([evaluate_bezier(ti, p0, p1, p2) for ti in t])
    else:
        points = np.array([evaluate_bezier(ti, p0, p1, p2, p3) for ti in t])
    
    distances = np.sqrt(np.sum(np.diff(points, axis=0)**2, axis=1))
    return np.sum(distances)

def distance_to_max_velocity(v_max, distance, segments, p0, p1, p2, p3=None, K=1.0):
    def curvature(t):
        d1 = bezier_derivative(t, p0, p1, p2, p3)
        if p3 is None:
            d2 = 2 * (np.array(p2) - 2*np.array(p1) + np.array(p0))
        else:
            d2 = 6 * ((np.array(p2) - 2*np.array(p1) + np.array(p0))*(1-t) + (np.array(p3) - 2*np.array(p2) + np.array(p1))*t)
        
        num = np.linalg.norm(np.cross(d1, d2))
        denom = np.linalg.norm(d1)**3
        return num / denom if denom != 0 else 0

    t = distance / get_bezier_length(p0, p1, p2, p3)
    curve = curvature(t)
    
    # Calculate velocity based on curvature
    v_curve = v_max / (1 + K * curve)
    
    return min(v_max, v_curve)

def generate_motion_profile(control_points, target_velocities, max_velocity, max_acceleration, max_jerk, max_points=100000):
    def generate_trapezoid_profile(start_v, end_v, distance):
        # Calculate time to reach max velocity
        t_acc = (max_velocity - start_v) / max_acceleration
        d_acc = start_v * t_acc + 0.5 * max_acceleration * t_acc**2

        # Calculate time to decelerate from max velocity
        t_dec = (max_velocity - end_v) / max_acceleration
        d_dec = end_v * t_dec + 0.5 * max_acceleration * t_dec**2

        if d_acc + d_dec > distance:
            # We can't reach max velocity, so let's find the peak velocity
            # Solve quadratic equation: d = v_peak^2 / a
            v_peak = np.sqrt(max_acceleration * distance)
            t_acc = (v_peak - start_v) / max_acceleration
            t_dec = (v_peak - end_v) / max_acceleration
            d_acc = start_v * t_acc + 0.5 * max_acceleration * t_acc**2
            d_const = 0
        else:
            d_const = distance - d_acc - d_dec

        return d_acc, d_const, d_dec

    def limit_acceleration(velocities, times):
        accelerations = np.gradient(velocities, times)
        for i in range(1, len(velocities)):
            if abs(accelerations[i]) > max_acceleration:
                sign = np.sign(accelerations[i])
                velocities[i] = velocities[i-1] + sign * max_acceleration * (times[i] - times[i-1])
        return velocities

    def limit_jerk(velocities, times):
        accelerations = np.gradient(velocities, times)
        jerks = np.gradient(accelerations, times)
        for i in range(1, len(accelerations) - 1):
            if abs(jerks[i]) > max_jerk:
                sign = np.sign(jerks[i])
                accelerations[i] = accelerations[i-1] + sign * max_jerk * (times[i] - times[i-1])
                velocities[i+1] = velocities[i] + accelerations[i] * (times[i+1] - times[i])
        return velocities


    # Calculate total path length
    total_distance = sum(get_bezier_length(*cp) for cp in control_points)

    # Initialize path
    num_samples = min(max_points, int(total_distance / 0.01))  # Sample every 1cm or use max_points
    distances = np.linspace(0, total_distance, num_samples)
    velocities = np.zeros_like(distances)

    # Generate velocity profile
    current_distance = 0
    for i, (cp, target_v) in enumerate(zip(control_points, target_velocities)):
        segment_length = get_bezier_length(*cp)
        max_v = min(distance_to_max_velocity(max_velocity, current_distance, len(control_points), *cp), target_v)
        
        start_v = velocities[np.searchsorted(distances, current_distance) - 1]
        end_v = target_v if i < len(control_points) - 1 else 0
        
        accel_d, const_d, decel_d = generate_trapezoid_profile(start_v, end_v, segment_length)
        
        segment_distances = distances[(distances >= current_distance) & (distances < current_distance + segment_length)] - current_distance
        segment_velocities = np.piecewise(segment_distances,
                                          [segment_distances < accel_d,
                                           (segment_distances >= accel_d) & (segment_distances < accel_d + const_d),
                                           segment_distances >= accel_d + const_d],
                                          [lambda x: np.sqrt(start_v**2 + 2*max_acceleration*x),
                                           lambda x: max_v,
                                           lambda x: np.sqrt(end_v**2 + 2*max_acceleration*(segment_length - x))])
        
        velocities[(distances >= current_distance) & (distances < current_distance + segment_length)] = segment_velocities
        current_distance += segment_length

    # Ensure final velocity is 0
    velocities[-1] = 0

    # Calculate times
    with np.errstate(divide='ignore', invalid='ignore'):
        time_diffs = 2 * np.diff(distances) / (velocities[1:] + velocities[:-1])  # Use average velocity
    time_diffs[~np.isfinite(time_diffs)] = 0  # Replace inf and NaN with 0
    times = np.cumsum(time_diffs)
    times = np.insert(times, 0, 0)

    # Apply acceleration limiting
    velocities = limit_acceleration(velocities, times)

    # Recalculate times after acceleration limiting
    with np.errstate(divide='ignore', invalid='ignore'):
        time_diffs = 2 * np.diff(distances) / (velocities[1:] + velocities[:-1])  # Use average velocity
    time_diffs[~np.isfinite(time_diffs)] = 0  # Replace inf and NaN with 0
    times = np.cumsum(time_diffs)
    times = np.insert(times, 0, 0)

    # Apply jerk limiting
    velocities = limit_jerk(velocities, times)

    # Final acceleration limiting pass
    velocities = limit_acceleration(velocities, times)

    # Resample to 20ms intervals
    target_times = np.arange(0, times[-1], 0.02)
    position_interpolator = interp1d(times, distances, bounds_error=False, fill_value="extrapolate")
    velocity_interpolator = interp1d(times, velocities, bounds_error=False, fill_value="extrapolate")
    
    resampled_positions = position_interpolator(target_times)
    resampled_velocities = velocity_interpolator(target_times)
    resampled_accelerations = np.gradient(resampled_velocities, target_times)

    # Trim any points that exceed the total distance
    valid_indices = resampled_positions <= total_distance
    resampled_positions = resampled_positions[valid_indices]
    resampled_velocities = resampled_velocities[valid_indices]
    resampled_accelerations = resampled_accelerations[valid_indices]
    target_times = target_times[valid_indices]

    # Ensure the last point exactly matches the total distance
    if resampled_positions[-1] < total_distance:
        resampled_positions = np.append(resampled_positions, total_distance)
        resampled_velocities = np.append(resampled_velocities, 0)
        resampled_accelerations = np.append(resampled_accelerations, 0)
        target_times = np.append(target_times, target_times[-1] + 0.02)

    return resampled_positions, resampled_velocities, resampled_accelerations, target_times

# Example usage
control_points = [
    [[0, 0], [1, 1], [2, 0]],
    [[2, 0], [3, -1], [4, 0], [5, 1]],
    [[5, 1], [6, 2], [7, 1]]
]
target_velocities = [5, 2, 0]
max_velocity = 10
max_acceleration = 5
max_jerk = 2

positions, velocities, accelerations, times = generate_motion_profile(control_points, target_velocities, max_velocity, max_acceleration, max_jerk)

# You can now use these outputs for further processing or visualization