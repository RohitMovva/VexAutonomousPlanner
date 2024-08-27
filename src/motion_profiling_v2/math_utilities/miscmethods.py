import numpy as np
from bezier.quadratic_bezier import *
from bezier.cubic_bezier import *

def convert_velocity_parameterization(velocities, distance_interval, time_interval):
    """
    Convert a list of velocities from constant distance parameterization to constant time parameterization.
    
    Args:
    velocities (list): List of velocities, each measured after a constant distance interval.
    distance_interval (float): The constant distance interval between velocity measurements.
    time_interval (float): The desired constant time interval for the new parameterization.
    
    Returns:
    list: A new list of velocities parameterized by the constant time interval.
    """
    # Convert input to numpy array for vectorized operations
    v = np.array(velocities)
    
    # Handle zero velocities to avoid division by zero
    v = np.maximum(v, 1e-2)  # Replace zeros with a small positive number
    
    # Calculate the times at which the original velocities were measured
    times = np.cumsum(distance_interval / v)
    
    # Remove any potential inf or nan values
    valid_indices = np.isfinite(times)
    times = times[valid_indices]
    v = v[valid_indices]
    
    if len(times) == 0:
        return []  # Return empty list if all times were invalid
    
    # Create a new time array with constant time intervals
    new_times = np.arange(0, times[-1], time_interval)
    # Interpolate velocities at the new time points
    new_velocities = np.interp(new_times, times, v)
    
    return new_velocities.tolist()

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

def getHeading(t, cp1, cp2, cp3, cp4=None):
    if (cp4 == None):
        return quad_bezier_angle(t, cp1, cp2, cp3)
    return cubic_bezier_angle(t, cp1, cp2, cp3, cp4)