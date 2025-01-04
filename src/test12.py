import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple
from splines.quintic_hermite_spline import QuinticHermiteSpline  # Assuming this is the filename

def calculate_curvature(spline: QuinticHermiteSpline, t: float) -> float:
    """
    Calculate the curvature at a point on the spline.
    Curvature formula: κ = |x'y'' - y'x''| / (x'² + y'²)^(3/2)
    """
    d1 = spline.get_derivative(t)
    d2 = spline.get_second_derivative(t)
    
    # Extract components
    dx_dt = d1[0]
    dy_dt = d1[1]
    d2x_dt2 = d2[0]
    d2y_dt2 = d2[1]
    
    # Calculate curvature using the formula
    numerator = abs(dx_dt * d2y_dt2 - dy_dt * d2x_dt2)
    denominator = (dx_dt * dx_dt + dy_dt * dy_dt) ** 1.5
    
    # Handle near-zero denominator
    if denominator < 1e-10:
        return 0.0
        
    return numerator / denominator

def debug_derivatives_at_point(spline: QuinticHermiteSpline, t: float):
    """Print detailed derivative information at a specific parameter value"""
    point = spline.get_point(t)
    d1 = spline.get_derivative(t)
    d2 = spline.get_second_derivative(t)
    curvature = calculate_curvature(spline, t)
    
    print(f"\nDebugging at t = {t}:")
    print(f"Position: ({point[0]:.4f}, {point[1]:.4f})")
    print(f"First derivative: ({d1[0]:.4f}, {d1[1]:.4f}) magnitude: {np.linalg.norm(d1):.4f}")
    print(f"Second derivative: ({d2[0]:.4f}, {d2[1]:.4f}) magnitude: {np.linalg.norm(d2):.4f}")
    print(f"Curvature: {curvature:.4f}")
    
    # Get the segment information
    local_t, segment_idx = spline._normalize_parameter(t)
    print(f"\nSegment info:")
    print(f"Local t: {local_t:.4f}")
    print(f"Segment index: {segment_idx}")
    
    # Print the basis function values
    basis = spline._get_basis_functions(local_t)
    basis_d1 = spline._get_basis_derivatives(local_t)
    basis_d2 = spline._get_basis_second_derivatives(local_t)
    
    print("\nBasis function values:")
    names = ["H₀", "H₁", "H₂", "H₃", "H₄", "H₅"]
    for i, name in enumerate(names):
        print(f"{name}: {basis[i]:.4f}, {name}': {basis_d1[i]:.4f}, {name}'': {basis_d2[i]:.4f}")

def main():
    # Create a simple test case with 3 points
    x = np.array([0.0, 1.0, 2.0])
    y = np.array([0.0, 1.0, 0.0])
    points = np.column_stack((x, y))
    
    # Create and fit the spline
    spline = QuinticHermiteSpline()
    success = spline.fit(x, y)
    
    if not success:
        print("Failed to fit spline!")
        return
        
    # Generate points along the spline for plotting
    t_values = np.linspace(0, 2, 200)  # 200 points for smooth plotting
    spline_points = np.array([spline.get_point(t) for t in t_values])
    curvature_values = np.array([calculate_curvature(spline, t) for t in t_values])
    
    # Create subplots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 12))
    
    # Plot the spline
    ax1.plot(spline_points[:, 0], spline_points[:, 1], 'b-', label='Spline')
    ax1.plot(x, y, 'ro', label='Control Points')
    ax1.grid(True)
    ax1.set_aspect('equal')
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_title('Quintic Hermite Spline')
    ax1.legend()
    
    # Plot the curvature
    ax2.plot(t_values, curvature_values, 'g-', label='Curvature')
    ax2.axvline(x=1, color='r', linestyle='--', alpha=0.5, label='Control Point')
    ax2.grid(True)
    ax2.set_xlabel('Parameter t')
    ax2.set_ylabel('Curvature')
    ax2.set_title('Spline Curvature')
    ax2.legend()
    
    # Add vertical lines for control point locations
    for t in range(3):
        ax2.axvline(x=t, color='r', linestyle='--', alpha=0.5)
    
    plt.tight_layout()
    plt.show()
    
    # Print some statistics about the curvature
    print(f"Maximum curvature: {np.max(curvature_values):.4f}")
    print(f"Mean curvature: {np.mean(curvature_values):.4f}")
    # Debug points around the problematic area (t=1)
    test_points = [0.95, 0.975, 0.99, 1.0, 1.01, 1.025, 1.05]
    for t in test_points:
        debug_derivatives_at_point(spline, t)
        
    # Print the stored derivatives at control points
    print("\nStored derivatives at control points:")
    for i in range(len(spline.control_points)):
        print(f"\nControl point {i}:")
        print(f"Position: {spline.control_points[i]}")
        print(f"First derivative: {spline.first_derivatives[i]}")
        print(f"Second derivative: {spline.second_derivatives[i]}")

if __name__ == "__main__":
    main()