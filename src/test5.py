import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass
from typing import List
from splines.spline_manager import QuinticHermiteSplineManager

def unwrap_headings(headings: np.ndarray) -> np.ndarray:
    """
    Unwrap heading angles to avoid discontinuities from the -π to π transition.
    This is similar to np.unwrap but ensures the unwrapping accounts for
    the specific nature of heading angles.
    
    Args:
        headings: Array of heading values in radians
    
    Returns:
        Unwrapped heading values
    """
    # First use numpy's unwrap to handle basic wrapping
    unwrapped = np.unwrap(headings)
    
    # Detect any sudden jumps that might have been missed
    derivatives = np.diff(unwrapped)
    jump_indices = np.where(np.abs(derivatives) > np.pi)[0]
    
    # Fix any remaining jumps
    for idx in jump_indices:
        if derivatives[idx] > 0:
            unwrapped[idx+1:] -= 2*np.pi
        else:
            unwrapped[idx+1:] += 2*np.pi
            
    return unwrapped

# Mock Node class since we don't have access to the original
@dataclass
class Node:
    is_reverse_node: bool = False

def verify_curvature_heading_relationship(manager, t_vals, headings, curvatures, title=""):
    """
    Verify that curvature matches the rate of change of heading with respect to arc length.
    
    κ = dθ/ds where:
    κ = curvature
    θ = heading
    s = arc length
    
    Handles heading angle wrapping by unwrapping angles before computing derivatives.
    """
    # Calculate arc lengths for each parameter value
    arc_lengths = np.zeros_like(t_vals)
    for i in range(1, len(t_vals)):
        arc_lengths[i] = arc_lengths[i-1] + manager.splines[0].get_arc_length(t_vals[i-1], t_vals[i])
    
    # Unwrap headings to handle -π to π transitions
    unwrapped_headings = unwrap_headings(headings)
    
    # Calculate heading rate of change with respect to arc length
    heading_derivatives = np.gradient(unwrapped_headings, arc_lengths)
    
    # Compare with curvature
    rms_error = np.sqrt(np.mean((heading_derivatives - curvatures)**2))
    max_error = np.max(np.abs(heading_derivatives - curvatures))
    
    print(f"\nCurvature-Heading Relationship Verification ({title}):")
    print(f"RMS Error between κ and dθ/ds: {rms_error:.6f}")
    print(f"Maximum Error: {max_error:.6f}")
    
    # Plot comparison
    plt.figure(figsize=(10, 6))
    plt.plot(arc_lengths, curvatures, 'b-', label='Curvature (κ)')
    plt.plot(arc_lengths, heading_derivatives, 'r--', label='Heading Derivative (dθ/ds)')
    plt.grid(True)
    plt.legend()
    plt.title(f'Curvature vs Heading Derivative ({title})')
    plt.xlabel('Arc Length')
    plt.ylabel('Value')
    plt.show()
    
    return rms_error, max_error

def test_circle_path():
    """Test heading and curvature on a circular path."""
    # Initialize spline manager
    
    # Create points along a circle
    radius = 1.0
    num_points = 8
    theta = np.linspace(0, 2*np.pi, num_points, endpoint=False)
    points = np.array([[radius * np.cos(t), radius * np.sin(t)] for t in theta])
    
    # Create nodes (no reverse nodes for this test)
    nodes = [Node(is_reverse_node=False) for _ in range(num_points)]
    
    # Initialize spline manager
    manager = QuinticHermiteSplineManager()
    success = manager.build_path(points, nodes)
    assert success, "Failed to build circular path"
    
    # Sample points along the path
    num_samples = 100
    t_vals = np.linspace(0, len(points)-1, num_samples)
    
    # Get headings and curvatures
    headings = np.array([manager.get_heading(t) for t in t_vals])
    curvatures = np.array([manager.get_curvature(t) for t in t_vals])
    
    # For a circle:
    # 1. Heading should change linearly with parameter
    # 2. Curvature should be constant and equal to 1/radius
    
    # Check heading changes
    heading_diffs = np.diff(headings)
    heading_variance = np.var(heading_diffs)
    print(f"\nCircle Test Results:")
    print(f"Heading change variance: {heading_variance:.6f}")
    print(f"Expected: Close to 0 for uniform heading changes")
    
    # Check curvature
    expected_curvature = 1.0 / radius
    curvature_mean = np.mean(curvatures)
    curvature_std = np.std(curvatures)
    print(f"Mean curvature: {curvature_mean:.6f}")
    print(f"Expected curvature: {expected_curvature:.6f}")
    print(f"Curvature std dev: {curvature_std:.6f}")
    
    # Plot results
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
    
    # Plot path
    path_points = np.array([manager.get_point_at_parameter(t) for t in t_vals])
    ax1.plot(path_points[:, 0], path_points[:, 1], 'b-', label='Spline')
    ax1.plot(points[:, 0], points[:, 1], 'ro', label='Control Points')
    ax1.set_aspect('equal')
    ax1.grid(True)
    ax1.legend()
    ax1.set_title('Path Shape')
    
    # Plot heading
    ax2.plot(t_vals, headings)
    ax2.set_title('Heading vs Parameter')
    ax2.grid(True)
    
    # Plot heading changes
    ax3.plot(t_vals[1:], heading_diffs)
    ax3.set_title('Heading Changes')
    ax3.grid(True)
    
    # Plot curvature
    ax4.plot(t_vals, curvatures)
    ax4.axhline(y=expected_curvature, color='r', linestyle='--', label='Expected')
    ax4.set_title('Curvature vs Parameter')
    ax4.grid(True)
    ax4.legend()
    
    plt.tight_layout()
    plt.show()
    
    # Verify curvature-heading relationship
    verify_curvature_heading_relationship(manager, t_vals, headings, curvatures, "Circle Path")

def test_s_curve():
    """Test heading and curvature on an S-curve path."""
    # Initialize spline manager
    
    # Create points for an S-curve
    x = np.linspace(-2, 2, 5)
    y = 2 * np.tanh(x)  # Using tanh to create smooth S-curve
    points = np.column_stack((x, y))
    
    # Create nodes (no reverse nodes)
    nodes = [Node(is_reverse_node=False) for _ in range(len(points))]
    
    # Initialize spline manager
    manager = QuinticHermiteSplineManager()
    success = manager.build_path(points, nodes)
    assert success, "Failed to build S-curve path"
    
    # Sample points along the path
    num_samples = 100
    t_vals = np.linspace(0, len(points)-1, num_samples)
    
    # Get headings and curvatures
    headings = np.array([manager.get_heading(t) for t in t_vals])
    curvatures = np.array([manager.get_curvature(t) for t in t_vals])
    
    # For an S-curve:
    # 1. Heading should transition smoothly from negative to positive
    # 2. Curvature should be symmetric and change sign at the midpoint
    
    print(f"\nS-Curve Test Results:")
    print(f"Max curvature: {np.max(curvatures):.6f}")
    print(f"Min curvature: {np.min(curvatures):.6f}")
    print(f"Curvature at midpoint: {curvatures[num_samples//2]:.6f}")
    
    # Plot results
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
    
    # Plot path
    path_points = np.array([manager.get_point_at_parameter(t) for t in t_vals])
    ax1.plot(path_points[:, 0], path_points[:, 1], 'b-', label='Spline')
    ax1.plot(points[:, 0], points[:, 1], 'ro', label='Control Points')
    ax1.set_aspect('equal')
    ax1.grid(True)
    ax1.legend()
    ax1.set_title('Path Shape')
    
    # Plot heading
    ax2.plot(t_vals, headings)
    ax2.set_title('Heading vs Parameter')
    ax2.grid(True)
    
    # Plot heading derivative
    heading_derivative = np.diff(headings) / np.diff(t_vals)
    ax3.plot(t_vals[1:], heading_derivative)
    ax3.set_title('Heading Rate of Change')
    ax3.grid(True)
    
    # Plot curvature
    ax4.plot(t_vals, curvatures)
    ax4.set_title('Curvature vs Parameter')
    ax4.grid(True)
    
    plt.tight_layout()
    plt.show()
    
    # Verify curvature-heading relationship
    verify_curvature_heading_relationship(manager, t_vals, headings, curvatures, "S-Curve")

def test_turn_around():
    """Test heading and curvature on an extreme path."""
    # Initialize spline manager
    
    # Create points for an extreme path
    # x = np.linspace(0, 5, 4)
    # y = x  # 45-degree line
    x = np.array([0, 1, 2])
    y = np.array([0, 15, 0])
    points = np.column_stack((x, y))
    
    # Create nodes
    nodes = [Node(is_reverse_node=False) for _ in range(len(points))]
    
    # Initialize spline manager
    manager = QuinticHermiteSplineManager()
    success = manager.build_path(points, nodes)
    assert success, "Failed to build extreme path"
    
    # Sample points along the path
    num_samples = 100
    t_vals = np.linspace(0, len(points)-1, num_samples)
    
    # Get headings and curvatures
    headings = np.array([manager.get_heading(t) for t in t_vals])
    curvatures = np.array([manager.get_curvature(t) for t in t_vals])
    
    # For a straight line:
    # 1. Heading should be constant (π/4 for a 45-degree line)
    # 2. Curvature should be zero
    
    print(f"\Extreme Line Test Results:")
    print(f"Expected heading: {np.pi/4:.6f}")
    print(f"Mean heading: {np.mean(headings):.6f}")
    print(f"Heading std dev: {np.std(headings):.6f}")
    print(f"Mean curvature: {np.mean(curvatures):.6f}")
    print(f"Max abs curvature: {np.max(np.abs(curvatures)):.6f}")
    
    # Plot results
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
    
    # Plot path
    path_points = np.array([manager.get_point_at_parameter(t) for t in t_vals])
    ax1.plot(path_points[:, 0], path_points[:, 1], 'b-', label='Spline')
    ax1.plot(points[:, 0], points[:, 1], 'ro', label='Control Points')
    ax1.set_aspect('equal')
    ax1.grid(True)
    ax1.legend()
    ax1.set_title('Path Shape')
    
    # Plot heading
    ax2.plot(t_vals, headings)
    ax2.axhline(y=np.pi/4, color='r', linestyle='--', label='Expected')
    ax2.set_title('Heading vs Parameter')
    ax2.grid(True)
    ax2.legend()
    
    # Plot heading deviation
    heading_deviation = headings - np.pi/4
    ax3.plot(t_vals, heading_deviation)
    ax3.set_title('Heading Deviation from Expected')
    ax3.grid(True)
    
    # Plot curvature
    ax4.plot(t_vals, curvatures)
    ax4.set_title('Curvature vs Parameter')
    ax4.grid(True)
    
    plt.tight_layout()
    plt.show()
    
    # Verify curvature-heading relationship
    verify_curvature_heading_relationship(manager, t_vals, headings, curvatures, "Extreme case")

if __name__ == "__main__":
    # Run all tests
    test_circle_path()
    test_s_curve()
    test_turn_around()


