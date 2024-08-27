import math

def calculate_travel_time_constant_acceleration(x, x0, v0, a):
    """
    Calculate the time it takes to travel a certain distance with initial velocity
    and constant acceleration.

    Parameters:
    x (float): Final position
    x0 (float): Initial position
    v0 (float): Initial velocity
    a (float): Constant acceleration

    Returns:
    float: Time taken to travel the distance
    """
    
    # Calculate displacement
    displacement = x - x0
    
    # Use the quadratic formula to solve for time
    # The equation is: 0.5 * a * t^2 + v0 * t - displacement = 0
    
    # Check if acceleration is zero (constant velocity case)
    if a == 0:
        if v0 == 0:
            raise ValueError("Both acceleration and velocity are zero. Cannot calculate time.")
        return displacement / v0
    
    # Calculate the discriminant
    discriminant = v0**2 + 2*a*displacement
    
    # Check if the discriminant is negative (which would give imaginary solutions)
    if discriminant < 0:
        raise ValueError("The object never reaches the target position with these parameters.")
    
    # Calculate the two possible times
    t1 = (-v0 + math.sqrt(discriminant)) / a
    t2 = (-v0 - math.sqrt(discriminant)) / a
    
    # Return the positive time (or the least positive if both are positive)
    if t1 > 0 and t2 > 0:
        return min(t1, t2)
    elif t1 > 0:
        return t1
    elif t2 > 0:
        return t2
    else:
        raise ValueError("No positive time solution found. Check your parameters.")

# Example usage
# try:
#     time = calculate_travel_time_constant_acceleration(x=100, x0=0, v0=10, a=2)
#     print(f"Time taken: {time:.4f} seconds")
# except ValueError as e:
#     print(f"Error: {e}")
