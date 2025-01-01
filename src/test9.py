import numpy as np
import matplotlib.pyplot as plt
from splines.spline_manager import QuinticHermiteSplineManager

class Node:
    def __init__(self, is_reverse_node=False):
        self.is_reverse_node = is_reverse_node

    def __str__(self):
        return f"Node(reverse={self.is_reverse_node})"
    
def diagnose_spline_distances(manager):
    """Detailed diagnostic of distance calculations"""
    print("\n=== Spline Distance Diagnostics ===\n")
    
    # 1. Analyze individual spline segments
    print("Individual Spline Segment Analysis:")
    total_arc_length = 0
    for i, spline in enumerate(manager.splines):
        # Get segment arc length
        segment_length = spline.get_total_arc_length()
        total_arc_length += segment_length
        
        # Get control points for this segment
        start_point = spline.control_points[0]
        end_point = spline.control_points[-1]
        chord_length = np.linalg.norm(end_point - start_point)
        
        print(f"\nSpline Segment {i}:")
        print(f"  Arc Length: {segment_length:.6f}")
        print(f"  Chord Length: {chord_length:.6f}")
        print(f"  Arc/Chord Ratio: {segment_length/chord_length:.6f}")
        
        # Sample points along this segment for curvature analysis
        num_samples = 50
        points = []
        for j in range(num_samples):
            t = j / (num_samples - 1)
            point = spline.get_point(t)
            points.append(point)
        points = np.array(points)
        
        # Calculate point-to-point distances
        diffs = np.diff(points, axis=0)
        local_distances = np.linalg.norm(diffs, axis=1)
        
        print(f"  Min point-to-point distance: {np.min(local_distances):.6f}")
        print(f"  Max point-to-point distance: {np.max(local_distances):.6f}")
        print(f"  Distance variation: {np.std(local_distances):.6f}")
    
    print(f"\nTotal arc length from individual segments: {total_arc_length:.6f}")
    
    # 2. Analyze lookup table construction
    print("\nLookup Table Analysis:")
    if manager.lookup_table is not None:
        distances = manager.lookup_table.distances
        parameters = manager.lookup_table.parameters
        
        # Check distance increments
        distance_increments = np.diff(distances)
        param_increments = np.diff(parameters)
        
        print(f"Number of points in lookup table: {len(distances)}")
        print(f"Distance range: [{distances[0]:.6f}, {distances[-1]:.6f}]")
        print(f"Parameter range: [{parameters[0]:.6f}, {parameters[-1]:.6f}]")
        print(f"Average distance increment: {np.mean(distance_increments):.6f}")
        print(f"Min distance increment: {np.min(distance_increments):.6f}")
        print(f"Max distance increment: {np.max(distance_increments):.6f}")
        
        # Check for any anomalies in distance calculations
        large_jumps = distance_increments > np.mean(distance_increments) * 2
        if np.any(large_jumps):
            jump_indices = np.where(large_jumps)[0]
            print("\nFound unusually large distance increments at indices:")
            for idx in jump_indices[:5]:  # Show first 5 anomalies
                print(f"  Index {idx}: {distance_increments[idx]:.6f}")
    
    # 3. Check parameter mapping
    print("\nParameter Mapping Test:")
    test_distances = np.linspace(0, manager.lookup_table.total_length, 10)
    for dist in test_distances:
        t = manager.distance_to_time(dist)
        print(f"Distance {dist:.3f} maps to parameter {t:.3f}")
    
    # 4. Compare methods at key points
    print("\nKey Point Comparison:")
    total_param = len(manager.nodes) - 1
    key_params = np.linspace(0, total_param, 5)
    
    for t in key_params:
        point = manager.get_point_at_parameter(t)
        deriv = manager.get_derivative_at_parameter(t)
        speed = np.linalg.norm(deriv)
        print(f"\nAt t = {t:.3f}:")
        print(f"  Position: ({point[0]:.3f}, {point[1]:.3f})")
        print(f"  Speed: {speed:.3f}")

    print("\n Gaussian arc length: ", manager.get_total_arc_length())
    print("\n Total arc length: ", distances[-1])
    print("\n Difference: ", abs(manager.get_total_arc_length() - distances[-1]))
    print("\n Relative error: ", abs(manager.get_total_arc_length() - distances[-1]) / distances[-1] * 100)
    print("\n=== End of Diagnostics ===\n")
    
def run_diagnostics():
    # Create test path (using your original circle example)
    theta = np.linspace(0, 2*np.pi, 5)
    x = np.cos(theta) * 2
    y = np.sin(theta)
    points = np.column_stack((x, y))
    
    # Create nodes
    nodes = [Node() for _ in range(len(points))]
    
    # Initialize and build path
    manager = QuinticHermiteSplineManager()
    success = manager.build_path(points, nodes)
    
    if success:
        # Build tables
        manager.build_lookup_table()
        
        # Run diagnostics
        diagnose_spline_distances(manager)
    else:
        print("Failed to build path")


if __name__ == "__main__":
    run_diagnostics()