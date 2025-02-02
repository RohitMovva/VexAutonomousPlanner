import numpy as np

def generate_trapezoidal_profile(max_velocity, max_acceleration, total_distance, time_step=0.01):
    """
    Generate a trapezoidal motion profile given constraints.
    
    Args:
        max_velocity (float): Maximum allowed velocity
        max_acceleration (float): Maximum allowed acceleration
        total_distance (float): Total distance to travel
        time_step (float): Time step for profile generation (default: 0.01s)
    
    Returns:
        tuple: Arrays of (time, position, velocity, acceleration)
    """
    # Calculate time needed to reach max velocity
    time_to_max_vel = max_velocity / max_acceleration
    
    # Calculate distance covered during acceleration/deceleration
    dist_accel = 0.5 * max_acceleration * time_to_max_vel**2
    
    # Check if we can reach max velocity
    if 2 * dist_accel > total_distance:
        # Triangle profile needed (can't reach max velocity)
        time_to_max_vel = np.sqrt(total_distance / max_acceleration)
        max_velocity = max_acceleration * time_to_max_vel
        total_time = 2 * time_to_max_vel
    else:
        # Trapezoidal profile possible
        dist_constant_vel = total_distance - 2 * dist_accel
        time_constant_vel = dist_constant_vel / max_velocity
        total_time = 2 * time_to_max_vel + time_constant_vel

    # Generate time array
    time_array = np.arange(0, total_time + time_step, time_step)
    
    # Initialize arrays
    position = np.zeros_like(time_array)
    velocity = np.zeros_like(time_array)
    acceleration = np.zeros_like(time_array)
    
    # Fill arrays based on profile phases
    for i, t in enumerate(time_array):
        if t <= time_to_max_vel:
            # Acceleration phase
            acceleration[i] = max_acceleration
            velocity[i] = max_acceleration * t
            position[i] = 0.5 * max_acceleration * t**2
        
        elif t <= total_time - time_to_max_vel:
            # Constant velocity phase
            acceleration[i] = 0
            velocity[i] = max_velocity
            position[i] = (dist_accel + 
                         max_velocity * (t - time_to_max_vel))
        
        else:
            # Deceleration phase
            time_in_decel = t - (total_time - time_to_max_vel)
            acceleration[i] = -max_acceleration
            velocity[i] = max_velocity - max_acceleration * time_in_decel
            position[i] = (total_distance - 
                         0.5 * max_acceleration * (time_to_max_vel - time_in_decel)**2)
    
    return velocity