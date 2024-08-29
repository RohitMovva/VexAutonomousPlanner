import numpy as np
from scipy.interpolate import interp1d
# from miscmethods import *

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
def test_reparametrize_velocity():
    # Test case 1: Constant velocity
    velocities = [2.0] * 10
    distance_interval = 1.0
    time_interval = 0.5
    
    new_times, new_velocities = reparametrize_velocity(velocities, distance_interval, time_interval)
    
    print("Test Case 1: Constant Velocity")
    print(f"Original velocities: {velocities}")
    print(f"New times: {new_times}")
    print(f"New velocities: {new_velocities}")
    
    # Check if velocities remain constant
    assert np.allclose(new_velocities, 2.0, rtol=1e-2), "Velocities should remain constant"
    
    # Test case 2: Linearly increasing velocity
    velocities = [1.0, 2.0, 3.0, 4.0, 5.0]
    distance_interval = 1.0
    time_interval = 0.25
    
    new_times, new_velocities = reparametrize_velocity(velocities, distance_interval, time_interval)
    
    print("\nTest Case 2: Linearly Increasing Velocity")
    print(f"Original velocities: {velocities}")
    print(f"New times: {new_times}")
    print(f"New velocities: {new_velocities}")
    
    # Check if velocities increase approximately linearly
    velocity_diffs = np.diff(new_velocities)
    assert np.allclose(velocity_diffs, velocity_diffs[0], rtol=1e-2), "Velocities should increase approximately linearly"
    
    # Test case 3: Check total distance traveled
    velocities = [1.0, 2.0, 3.0, 2.0, 1.0]
    distance_interval = 2.0
    time_interval = 0.5
    
    new_times, new_velocities = reparametrize_velocity(velocities, distance_interval, time_interval)
    
    original_distance = sum(velocities) * distance_interval
    new_distance = np.trapz(new_velocities, new_times)
    
    print("\nTest Case 3: Check Total Distance")
    print(f"Original velocities: {velocities}")
    print(f"New times: {new_times}")
    print(f"New velocities: {new_velocities}")
    print(f"Original distance: {original_distance:.6f}")
    print(f"New distance: {new_distance:.6f}")
    print(f"Difference: {abs(original_distance - new_distance):.6f}")
    
    assert np.isclose(original_distance, new_distance, rtol=1e-2), "Total distance should be approximately equal"

    print("\nAll tests passed successfully!")

# if __name__ == "__main__":
test_reparametrize_velocity()