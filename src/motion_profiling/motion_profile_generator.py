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

def generate_motion_profile(control_points, target_velocities, max_velocity, max_acceleration, max_jerk, time_step=0.02):
    def limit_acceleration(velocity, prev_velocity, dt):
        acceleration = (velocity - prev_velocity) / dt
        if abs(acceleration) > max_acceleration:
            acceleration = np.sign(acceleration) * max_acceleration
        return prev_velocity + acceleration * dt

    def limit_jerk(velocity, prev_velocity, prev_acceleration, dt):
        acceleration = (velocity - prev_velocity) / dt
        jerk = (acceleration - prev_acceleration) / dt
        if abs(jerk) > max_jerk:
            jerk = np.sign(jerk) * max_jerk
        acceleration = prev_acceleration + jerk * dt
        return prev_velocity + acceleration * dt, acceleration

    total_distance = sum(get_bezier_length(*cp) for cp in control_points)
    
    # Initialize arrays
    times = [0]
    positions = [0]
    velocities = [0]
    accelerations = [0]

    current_distance = 0
    current_velocity = 0
    current_acceleration = 0

    for i, (cp, target_v) in enumerate(zip(control_points, target_velocities)):
        segment_length = get_bezier_length(*cp)
        segment_max_v = min(distance_to_max_velocity(max_velocity, current_distance, len(control_points), *cp), target_v)

        while current_distance < sum(get_bezier_length(*control_points[:i+1])):
            # Calculate desired velocity
            remaining_distance = sum(get_bezier_length(*control_points[:i+1])) - current_distance
            desired_v = min(segment_max_v, np.sqrt(2 * max_acceleration * remaining_distance))

            # Apply acceleration and jerk limits
            limited_v = limit_acceleration(desired_v, current_velocity, time_step)
            limited_v, new_acceleration = limit_jerk(limited_v, current_velocity, current_acceleration, time_step)

            # Update current state
            current_distance += (current_velocity + limited_v) / 2 * time_step
            current_velocity = limited_v
            current_acceleration = new_acceleration

            # Append to arrays
            times.append(times[-1] + time_step)
            positions.append(current_distance)
            velocities.append(current_velocity)
            accelerations.append(current_acceleration)

    # Ensure we reach the exact total distance
    if positions[-1] < total_distance:
        times.append(times[-1] + time_step)
        positions.append(total_distance)
        velocities.append(0)
        accelerations.append(0)

    # Convert lists to numpy arrays
    times = np.array(times)
    positions = np.array(positions)
    velocities = np.array(velocities)
    accelerations = np.array(accelerations)

    return positions, velocities, accelerations, times
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