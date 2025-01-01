import numpy as np
from splines.spline_manager import QuinticHermiteSplineManager

class Node:
    def __init__(self, is_reverse_node=False):
        self.is_reverse_node = is_reverse_node

    def __str__(self):
        return f"Node(reverse={self.is_reverse_node})"
    
    
def run_arc_length_test(points, name=""):
    """Run arc length test for a given set of points"""
    # Create nodes (no reverse nodes)
    nodes = [Node() for _ in range(len(points))]
    
    # Initialize and build path
    manager = QuinticHermiteSplineManager()
    success = manager.build_path(points, nodes)
    
    if not success:
        print(f"\nFailed to build path for test: {name}")
        return
        
    # Build lookup table and get lengths
    manager.build_lookup_table()
    gauss_length = manager.get_total_arc_length()
    table_length = manager.lookup_table.total_length
    
    # Calculate error
    diff = abs(gauss_length - table_length)
    rel_error = diff / table_length * 100
    
    print(f"\nTest: {name}")
    print(f"Number of points: {len(points)}")
    print(f"Gaussian arc length: {gauss_length:.6f}")
    print(f"Lookup table length: {table_length:.6f}")
    print(f"Absolute difference: {diff:.6f}")
    print(f"Relative error: {rel_error:.6f}%")
    
    return rel_error

def generate_test_cases():
    test_cases = []
    
    # Test 1: Simple circle (similar to original)
    theta = np.linspace(0, 2*np.pi, 5)
    circle = np.column_stack((2*np.cos(theta), np.sin(theta)))
    test_cases.append((circle, "Circle with 5 points"))
    
    # Test 2: Dense circle
    theta = np.linspace(0, 2*np.pi, 20)
    dense_circle = np.column_stack((2*np.cos(theta), np.sin(theta)))
    test_cases.append((dense_circle, "Circle with 20 points"))
    
    # Test 3: Straight line
    x = np.linspace(0, 10, 5)
    line = np.column_stack((x, np.zeros_like(x)))
    test_cases.append((line, "Straight line"))
    
    # Test 4: Sine wave
    x = np.linspace(0, 4*np.pi, 15)
    sine = np.column_stack((x, np.sin(x)))
    test_cases.append((sine, "Sine wave"))
    
    # Test 5: Square spiral
    t = np.linspace(0, 4, 20)
    spiral = np.column_stack((t*np.cos(2*np.pi*t), t*np.sin(2*np.pi*t)))
    test_cases.append((spiral, "Spiral"))
    
    # Test 6: Figure-8
    t = np.linspace(0, 2*np.pi, 25)
    figure8 = np.column_stack((2*np.sin(t), np.sin(2*t)))
    test_cases.append((figure8, "Figure-8"))
    
    # Test 7: Minimal path (2 points)
    line2 = np.array([[0, 0], [1, 1]])
    test_cases.append((line2, "Minimal path (2 points)"))
    
    # Test 8: Sharp turns
    x = np.array([0, 1, 1, 0, 0, 1])
    y = np.array([0, 0, 1, 1, 2, 2])
    zigzag = np.column_stack((x, y))
    test_cases.append((zigzag, "Sharp turns"))
    
    # Test 9: Different scalings
    theta = np.linspace(0, 2*np.pi, 10)
    ellipse = np.column_stack((5*np.cos(theta), np.sin(theta)))
    test_cases.append((ellipse, "Ellipse (different scales)"))
    
    # Test 10: High curvature regions
    t = np.linspace(0, 4*np.pi, 30)
    cuspy = np.column_stack((t, np.sin(t) + np.sin(3*t)))
    test_cases.append((cuspy, "High curvature path"))
    
    return test_cases

def main():
    print("Running comprehensive arc length tests...")
    test_cases = generate_test_cases()
    
    max_error = 0
    worst_case = ""
    total_error = 0
    
    print("\n=== Test Results ===")
    for points, name in test_cases:
        error = run_arc_length_test(points, name)
        if error > max_error:
            max_error = error
            worst_case = name
        total_error += error
    
    print("\n=== Summary ===")
    print(f"Number of test cases: {len(test_cases)}")
    print(f"Average relative error: {total_error/len(test_cases):.6f}%")
    print(f"Maximum relative error: {max_error:.6f}% (in {worst_case})")

if __name__ == "__main__":
    main()