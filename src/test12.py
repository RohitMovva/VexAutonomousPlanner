import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple
from splines.quintic_hermite_spline import QuinticHermiteSpline  # Assuming this is the filename

def calculate_curvature(spline: QuinticHermiteSpline, t: float, debug: bool = False) -> float:
    """
    Calculate the curvature at a point on the spline with optional detailed logging.
    Curvature formula: κ = |x'y'' - y'x''| / (x'² + y'²)^(3/2)
    """
    if debug:
        print(f"\n=== Calculating Curvature at t={t} ===")
        local_t, segment_idx = spline._normalize_parameter(t)
        print(f"Local t: {local_t}, Segment: {segment_idx}")
    
    # Get derivatives
    d1 = spline.get_derivative(t, debug)
    d2 = spline.get_second_derivative(t, debug)
    
    # Extract components
    dx_dt = d1[0]
    dy_dt = d1[1]
    d2x_dt2 = d2[0]
    d2y_dt2 = d2[1]
    
    if debug:
        print(f"First derivative (dx/dt, dy/dt): ({dx_dt:.6f}, {dy_dt:.6f})")
        print(f"Second derivative (d²x/dt², d²y/dt²): ({d2x_dt2:.6f}, {d2y_dt2:.6f})")
    
    # Calculate intermediate values for curvature formula
    numerator = abs(dx_dt * d2y_dt2 - dy_dt * d2x_dt2)
    denominator = (dx_dt * dx_dt + dy_dt * dy_dt) ** 1.5
    
    if debug:
        print(f"Numerator |x'y'' - y'x''|: {numerator:.6f}")
        print(f"Denominator (x'² + y'²)^(3/2): {denominator:.6f}")
    
    # Handle near-zero denominator
    if denominator < 1e-10:
        if debug:
            print("Warning: Near-zero denominator, returning 0 curvature")
        return 0.0
    
    curvature = numerator / denominator
    if debug:
        print(f"Resulting curvature: {curvature:.6f}")
    return curvature

def get_derivative(self, t: float, debug: bool = False) -> np.ndarray:
    """Enhanced get_derivative with optional logging"""
    if debug:
        print(f"\n=== Computing First Derivative at t={t} ===")
    
    if not self.segments:
        raise ValueError("Spline has not been fitted yet")
    
    local_t, segment_idx = self._normalize_parameter(t)
    if debug:
        print(f"Local t: {local_t}, Segment: {segment_idx}")
    
    basis_derivatives = self._get_basis_derivatives(local_t)
    if debug:
        print(f"Basis derivatives: {basis_derivatives}")
        print(f"Segment data:\n{self.segments[segment_idx]}")
    
    derivative = np.zeros(2)
    if debug:
        print("\nComputing derivative contributions:")
        for i in range(6):
            contribution = basis_derivatives[i] * self.segments[segment_idx][i]
            print(f"Basis[{i}] * segment[{i}] = {basis_derivatives[i]:.6f} * {self.segments[segment_idx][i]} = {contribution}")
            derivative += contribution
        print(f"Final derivative: {derivative}")
    else:
        for i in range(6):
            derivative += basis_derivatives[i] * self.segments[segment_idx][i]
    
    return derivative

def get_second_derivative(self, t: float, debug: bool = False) -> np.ndarray:
    """Enhanced get_second_derivative with optional logging"""
    if debug:
        print(f"\n=== Computing Second Derivative at t={t} ===")
    
    if not self.segments:
        raise ValueError("Spline has not been fitted yet")
    
    local_t, segment_idx = self._normalize_parameter(t)
    if debug:
        print(f"Local t: {local_t}, Segment: {segment_idx}")
    
    basis_second_derivatives = self._get_basis_second_derivatives(local_t)
    if debug:
        print(f"Basis second derivatives: {basis_second_derivatives}")
        print(f"Segment data:\n{self.segments[segment_idx]}")
    
    second_derivative = np.zeros(2)
    if debug:
        print("\nComputing second derivative contributions:")
        for i in range(6):
            contribution = basis_second_derivatives[i] * self.segments[segment_idx][i]
            print(f"Basis[{i}] * segment[{i}] = {basis_second_derivatives[i]:.6f} * {self.segments[segment_idx][i]} = {contribution}")
            second_derivative += contribution
        print(f"Final second derivative: {second_derivative}")
    else:
        for i in range(6):
            second_derivative += basis_second_derivatives[i] * self.segments[segment_idx][i]
    
    return second_derivative

def analyze_critical_points(spline: QuinticHermiteSpline):
    """Analyze spline behavior at critical points"""
    print("\n=== Analyzing Critical Points ===")
    
    # Test points around the middle control point
    test_points = [0.9, 0.95, 0.99, 1.0, 1.01, 1.05, 1.1]
    
    for t in test_points:
        print(f"\n--- Testing at t = {t} ---")
        # Get all relevant values
        point = spline.get_point(t)
        d1 = spline.get_derivative(t, debug=True)
        d2 = spline.get_second_derivative(t, debug=True)
        curvature = calculate_curvature(spline, t, debug=True)
        
        # Print summary
        print(f"Position: {point}")
        print(f"First derivative magnitude: {np.linalg.norm(d1):.6f}")
        print(f"Second derivative magnitude: {np.linalg.norm(d2):.6f}")
        print(f"Curvature: {curvature:.6f}")

def debug_derivatives_at_point(spline: QuinticHermiteSpline, t: float, debug: bool = True):
    """Print detailed derivative information at a specific parameter value"""
    point = spline.get_point(t)
    d1 = spline.get_derivative(t, debug)
    d2 = spline.get_second_derivative(t, debug)
    curvature = calculate_curvature(spline, t, debug)
    
    if debug:
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
        
    # Generate points along the spline for plotting (without detailed logging)
    t_values = np.linspace(0, 2, 2000)
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

    
    
    analyze_critical_points(spline)

if __name__ == "__main__":
    main()