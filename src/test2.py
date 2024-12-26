import numpy as np
import matplotlib.pyplot as plt
from math import atan2, degrees

def analyze_spline_headings(spline, points, num_samples=200):
    """
    Analyze and plot the headings along the spline
    """
    # Initialize the spline
    spline.initialize_spline(points, None)
    
    # Generate evenly spaced parameters
    t_values = np.linspace(0, 1, num_samples)
    
    # Compute points and derivatives
    path_points = np.array([spline.get_point(t) for t in t_values])
    derivatives = np.array([spline.get_derivative(t) for t in t_values])
    
    # Calculate headings (angles in degrees)
    headings = np.array([degrees(atan2(deriv[1], deriv[0])) 
                        for deriv in derivatives])
    
    # Create subplots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
    
    # Plot the spline and control points
    ax1.plot(path_points[:, 0], path_points[:, 1], 'b-', label='Spline')
    ax1.plot(points[:, 0], points[:, 1], 'ro', label='Control points')
    
    # Plot tangent vectors at control points
    if spline.tangents is not None:
        for point, tangent in zip(points, spline.tangents):
            ax1.arrow(point[0], point[1], 
                     tangent[0]*0.2, tangent[1]*0.2,
                     head_width=0.05, head_length=0.1, fc='g', ec='g')
    
    ax1.grid(True)
    ax1.legend()
    ax1.set_title('Spline Path with Control Points')
    ax1.axis('equal')
    
    # Plot the heading vs parameter value
    ax2.plot(t_values, headings, 'b-')
    ax2.grid(True)
    ax2.set_title('Spline Heading vs Parameter')
    ax2.set_xlabel('Parameter (t)')
    ax2.set_ylabel('Heading (degrees)')
    
    # Add vertical lines at control point parameters
    if spline.t_points is not None:
        for t in spline.t_points:
            ax2.axvline(x=t, color='r', linestyle='--', alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    
    # Print detailed heading analysis at control points
    print("\nHeading Analysis at Control Points:")
    for i, t in enumerate(spline.t_points):
        if i < len(spline.t_points) - 1:
            # Get headings just before and after the control point
            t_before = t - 0.001
            t_after = t + 0.001
            
            if t_before >= 0:
                deriv_before = spline.get_derivative(t_before)
                heading_before = degrees(atan2(deriv_before[1], deriv_before[0]))
            else:
                heading_before = None
                
            deriv_at = spline.get_derivative(t)
            heading_at = degrees(atan2(deriv_at[1], deriv_at[0]))
            
            deriv_after = spline.get_derivative(t_after)
            heading_after = degrees(atan2(deriv_after[1], deriv_after[0]))
            
            print(f"\nControl Point {i}:")
            if heading_before is not None:
                print(f"Heading before: {heading_before:.2f}°")
            print(f"Heading at point: {heading_at:.2f}°")
            print(f"Heading after: {heading_after:.2f}°")
            if heading_before is not None:
                print(f"Change through point: {abs(heading_after - heading_before):.2f}°")

def test_spline_headings():
    """
    Test the spline with different point configurations
    """
    from splines.quintic_hermite_spline import G2HermiteSpline
    
    # Test cases
    test_points = [
        # Simple curve
        np.array([
            [0, 0],
            [1, 1],
            [2, 0]
        ]),
        
        # S-curve
        np.array([
            [0, 0],
            [1, 1],
            [2, 0],
            [3, -1],
            [4, 0]
        ])
    ]
    
    for i, points in enumerate(test_points):
        print(f"\nTest Case {i + 1}:")
        print("Points:", points)
        
        spline = G2HermiteSpline()
        analyze_spline_headings(spline, points)

if __name__ == "__main__":
    test_spline_headings()