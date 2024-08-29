import numpy as np
from bezier.quadratic_bezier import *
from bezier.cubic_bezier import *
from scipy.interpolate import interp1d


def reparametrize_velocity(velocities, distance_interval, time_interval):
    """
    Reparametrize a list of velocities from distance to time.
    
    :param velocities: List of velocities parametrized by distance
    :param distance_interval: Distance between each velocity measurement
    :param time_interval: Desired time interval for the output
    :return: Tuple of (times, reparametrized_velocities)
    """
    # Calculate the distances at which velocities are given
    distances = np.arange(len(velocities)) * distance_interval

    # Calculate the cumulative time at each distance point
    times = np.cumsum(distance_interval / np.array(velocities))

    # Create an interpolation function for velocity vs. distance
    velocity_interp = interp1d(distances, velocities, kind='linear', fill_value='extrapolate')

    # Create a new distance array based on the desired time interval
    total_time = times[-1]
    new_times = np.arange(0, total_time, time_interval)
    
    # Calculate new distances for each time point
    new_distances = np.zeros_like(new_times)
    for i in range(1, len(new_times)):
        # Use average velocity over the interval to estimate distance
        avg_velocity = velocity_interp((new_distances[i-1] + new_distances[i-1] + distance_interval) / 2)
        new_distances[i] = new_distances[i-1] + avg_velocity * time_interval

    # Interpolate velocities at the new distance points
    new_velocities = velocity_interp(new_distances)

    return new_times, new_velocities

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