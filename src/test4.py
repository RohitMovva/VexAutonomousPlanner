import numpy as np
from typing import List, Tuple
from splines.quintic_hermite_spline import QuinticHermiteSpline  # Assuming this is the filename

def create_test_points() -> Tuple[np.ndarray, np.ndarray]:
    """Create a set of test points forming a quarter circle-like curve."""
    t = np.linspace(0, np.pi/2, 5)
    x = np.cos(t)
    y = np.sin(t)
    return x, y

def test_arc_length_methods():
    """Test the arc length-related methods of QuinticHermiteSpline."""
    # Initialize spline
    spline = QuinticHermiteSpline()
    
    # Create test points
    x, y = create_test_points()
    
    # Fit the spline
    success = spline.fit(x, y)
    if not success:
        print("Failed to fit spline")
        return
    
    # Test 1: Get total arc length
    total_length = spline.get_total_arc_length()
    print(f"\nTest 1: Total arc length: {total_length:.4f}")
    # For a quarter circle with radius 1, the theoretical length should be π/2 ≈ 1.5708
    print(f"Expected length (quarter circle): {np.pi/2:.4f}")
    
    # Test 2: Get arc length for different segments
    t_start = spline.parameters[0]
    t_end = spline.parameters[-1]
    
    # Test several intervals
    intervals = [
        (t_start, (t_start + t_end)/2),  # First half
        ((t_start + t_end)/2, t_end),    # Second half
        (t_start, t_end),                # Full curve
    ]
    
    print("\nTest 2: Arc lengths for different segments:")
    for start, end in intervals:
        length = spline.get_arc_length(start, end)
        print(f"Arc length from t={start:.2f} to t={end:.2f}: {length:.4f}")
    
    # Test 3: Parameter by arc length
    print("\nTest 3: Finding parameters for given arc lengths:")
    
    # Test at various fractions of total length
    fractions = [0.0, 0.25, 0.5, 0.75, 1.0]
    
    for fraction in fractions:
        target_length = fraction * total_length
        try:
            t = spline.get_parameter_by_arc_length(target_length)
            # Verify by computing the arc length up to this parameter
            verified_length = spline.get_arc_length(t_start, t)
            error = abs(verified_length - target_length)
            
            print(f"\nTarget arc length: {target_length:.4f}")
            print(f"Found parameter t: {t:.4f}")
            print(f"Verified length: {verified_length:.4f}")
            print(f"Error: {error:.6f}")
            
        except ValueError as e:
            print(f"Error for fraction {fraction}: {str(e)}")

if __name__ == "__main__":
    test_arc_length_methods()