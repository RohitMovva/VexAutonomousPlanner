import unittest
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from gui.node import Node  # You'll need to create a minimal Node class if not available
from splines.quintic_hermite_spline import QuinticHermiteSpline
from splines.spline_manager import QuinticHermiteSplineManager

# Minimal Node class if not available
class Node:
    def __init__(self, is_reverse_node=False):
        self.is_reverse_node = is_reverse_node

    def __str__(self):
        return f"Node(reverse={self.is_reverse_node})"

class TestQuinticHermiteSplineManager(unittest.TestCase):
    def setUp(self):
        """Set up test fixtures before each test method."""
        self.manager = QuinticHermiteSplineManager()
        
    def plot_spline(self, points, nodes, title="Spline Visualization", show_derivatives=False):
        """Helper method to plot the spline and its control points."""
        # Create figure and axis
        fig, ax = plt.subplots(figsize=(10, 8))
        ax.set_aspect('equal')
        ax.grid(True)
        ax.set_title(title)
        
        # Plot control points
        ax.plot(points[:, 0], points[:, 1], 'k.', markersize=10, label='Control Points')
        
        # Mark reverse nodes with circles
        for i, node in enumerate(nodes):
            if node.is_reverse_node:
                circle = Circle((points[i, 0], points[i, 1]), 0.1, fill=False, color='red')
                ax.add_patch(circle)
                
        # Sample points along the spline
        t_values = np.linspace(0, len(points) - 1, 200)
        spline_points = np.array([self.manager.get_point_at_parameter(t) for t in t_values])
        
        # Plot the spline
        ax.plot(spline_points[:, 0], spline_points[:, 1], 'b-', label='Spline')
        
        if show_derivatives:
            # Plot first derivatives at regular intervals
            t_samples = np.linspace(0, len(points) - 1, 20)
            for t in t_samples:
                point = self.manager.get_point_at_parameter(t)
                derivative = self.manager.get_derivative_at_parameter(t)
                # Scale derivative for visualization
                scale = 0.2
                ax.arrow(point[0], point[1], 
                        derivative[0] * scale, derivative[1] * scale,
                        head_width=0.05, head_length=0.1, fc='g', ec='g')
        
        ax.legend()
        plt.show()
        
    def test_basic_path_creation(self):
        """Test creating a basic path with no reverse nodes."""
        # Create simple test points
        points = np.array([
            [0.0, 0.0],  # Start point
            [1.0, 1.0],  # Middle point
            [2.0, 0.0]   # End point
        ])

        # points = np.array([
        #     [0.0, 0.0],  # Start point
        #     [50.0, 50.0],  # Middle point
        #     [100.0, 0.0]   # End point
        # ])
        
        # Create regular nodes
        nodes = [Node(False) for _ in range(3)]
        
        # Build the path
        success = self.manager.build_path(points, nodes)
        
        # Assert path was built successfully
        self.assertTrue(success)
        self.assertEqual(len(self.manager.splines), 1)
        
        # Test point evaluation at different parameters
        mid_point = self.manager.get_point_at_parameter(1.0)
        self.assertIsInstance(mid_point, np.ndarray)
        # np.testing.assert_array_almost_equal(mid_point, [50.0, 50.0])
        
        # Visualize the spline
        self.plot_spline(points, nodes, "Basic Path Test")
        
    def test_reverse_node_handling(self):
        """Test path creation with a reverse node in the middle."""
        points = np.array([
            [0.0, 0.0],    # Start point
            [1.0, 1.0],    # Middle point (reverse)
            [2.0, 0.0]     # End point
        ])
        
        # Create nodes with middle one as reverse
        nodes = [
            Node(False),
            Node(True),    # Reverse node
            Node(False)
        ]
        
        # Build the path
        success = self.manager.build_path(points, nodes)
        
        # Assert path was built successfully
        self.assertTrue(success)
        # self.assertEqual(len(self.manager.splines), 2)  # Should create two splines
        
        # Test continuity at the reverse node
        t_reverse = 1.0  # Parameter at reverse node
        
        # Get derivatives before and after reverse node
        derivative_before = self.manager.get_derivative_at_parameter(t_reverse - 0.01)
        derivative_after = self.manager.get_derivative_at_parameter(t_reverse + 0.01)
        
        # Derivatives should be similar in magnitude but possibly different direction
        mag_before = np.linalg.norm(derivative_before)
        mag_after = np.linalg.norm(derivative_after)
        # self.assertAlmostEqual(mag_before, mag_after, places=2)
        
        # Visualize the spline with reverse node
        self.plot_spline(points, nodes, "Path with Reverse Node", show_derivatives=True)
        
    def test_complex_path(self):
        """Test a more complex path with multiple reverse nodes."""
        points = np.array([
            [0.0, 0.0],    # Start
            [1.0, 2.0],    # Regular point
            [2.0, 1.0],    # Reverse node
            [3.0, 2.0],    # Regular point
            [4.0, 0.0],    # Reverse node
            [5.0, 1.0]     # End
        ])
        
        nodes = [
            Node(False),
            Node(False),
            Node(True),    # Reverse node
            Node(False),
            Node(True),    # Reverse node
            Node(False)
        ]
        
        # Build the path
        success = self.manager.build_path(points, nodes)
        self.assertTrue(success)
        
        # Visualize the complex path
        self.plot_spline(points, nodes, "Complex Path with Multiple Reverse Nodes", 
                        show_derivatives=True)
                        
    def test_parameter_mapping(self):
        """Test parameter mapping between global and local parameters."""
        points = np.array([
            [0.0, 0.0],
            [1.0, 0.0],
            [1.0, 1.0],
            [0.0, 1.0]
        ])
        
        nodes = [Node(False) for _ in range(4)]
        
        # Build the path
        success = self.manager.build_path(points, nodes)
        self.assertTrue(success)
        
        # Test points at various parameters
        test_params = [0.0, 1.0, 2.0, 3.0]
        expected_points = points  # In this case, parameters map directly to points
        
        for t, expected in zip(test_params, expected_points):
            point = self.manager.get_point_at_parameter(t)
            np.testing.assert_array_almost_equal(point, expected, decimal=2)
            
    # def test_derivatives(self):
    #     """Test first and second derivatives computation."""
    #     points = np.array([
    #         [0.0, 0.0],
    #         [1.0, 1.0],
    #         [2.0, 0.0]
    #     ])
        
    #     nodes = [Node(False) for _ in range(3)]
        
    #     # Build the path
    #     success = self.manager.build_path(points, nodes)
    #     self.assertTrue(success)
        
    #     # Test that derivatives exist and have correct shape
    #     t = 0.5
    #     first_deriv = self.manager.get_derivative_at_parameter(t)
    #     second_deriv = self.manager.get_second_derivative_at_parameter(t)
        
    #     self.assertEqual(first_deriv.shape, (2,))
    #     self.assertEqual(second_deriv.shape, (2,))
        
    # def test_invalid_inputs(self):
    #     """Test handling of invalid inputs."""
    #     # Test mismatched points and nodes
    #     points = np.array([[0.0, 0.0], [1.0, 1.0]])
    #     nodes = [Node(False)]
        
    #     success = self.manager.build_path(points, nodes)
    #     self.assertFalse(success)
        
    #     # Test too few points
    #     points = np.array([[0.0, 0.0]])
    #     nodes = [Node(False)]
        
    #     success = self.manager.build_path(points, nodes)
    #     self.assertFalse(success)
        
    #     # Test invalid parameter access
    #     points = np.array([[0.0, 0.0], [1.0, 1.0], [2.0, 0.0]])
    #     nodes = [Node(False) for _ in range(3)]
        
    #     self.manager.build_path(points, nodes)
        
    #     with self.assertRaises(ValueError):
    #         self.manager.get_point_at_parameter(-1.0)
            
    #     with self.assertRaises(ValueError):
    #         self.manager.get_point_at_parameter(10.0)

if __name__ == '__main__':
    unittest.main()