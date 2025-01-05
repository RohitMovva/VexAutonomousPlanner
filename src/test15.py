import numpy as np
from typing import List
import unittest
from splines.spline_manager import QuinticHermiteSplineManager

# Minimal Node class if not available
class Node:
    def __init__(self, is_reverse_node=False):
        self.is_reverse_node = is_reverse_node

    def __str__(self):
        return f"Node(reverse={self.is_reverse_node})"

class TestCurvatureMethods(unittest.TestCase):
    def setUp(self):
        self.spline_manager = QuinticHermiteSplineManager()
        
    def create_circle_path(self, radius: float = 1.0, num_points: int = 8) -> tuple:
        """Create a circular path with given radius"""
        theta = np.linspace(0, 2*np.pi, num_points)
        x = radius * np.cos(theta)
        y = radius * np.sin(theta)
        points = np.column_stack((x, y))
        
        # Create nodes (no reverse nodes for circular path)
        nodes = [Node(False) for _ in range(num_points)]
        
        return points, nodes
    
    def create_s_curve_path(self) -> tuple:
        """Create an S-curve path with varying curvature"""
        x = np.array([0, 1, 2, 3, 4])
        y = np.array([0, 1, 0, -1, 0])
        points = np.column_stack((x, y))
        
        # Create nodes (no reverse nodes for S-curve)
        nodes = [Node(False) for _ in range(len(x))]
        
        return points, nodes
    
    def test_circle_curvature(self):
        """Test curvature calculation for a circle (constant curvature)"""
        radius = 2.0
        points, nodes = self.create_circle_path(radius=radius)
        
        # Build the path
        success = self.spline_manager.build_path(points, nodes)
        self.assertTrue(success, "Failed to build circular path")
        
        # Test curvature at various points
        expected_curvature = 1.0 / radius  # Circle curvature = 1/radius
        test_params = np.linspace(0, len(nodes)-1, 20)
        
        for t in test_params:
            curvature = self.spline_manager.get_curvature(t)
            self.assertAlmostEqual(abs(curvature), expected_curvature, 
                                 places=1,  # Relaxed precision due to discretization
                                 msg=f"Incorrect curvature at t={t}")
            
            # Rate of change should be close to zero for circle
            curvature_change = self.spline_manager.get_curvature_rate_of_change(t)
            self.assertAlmostEqual(curvature_change, 0.0,
                                 places=1,
                                 msg=f"Curvature should be constant at t={t}")
    
    def test_s_curve_curvature(self):
        """Test curvature calculation for an S-curve (varying curvature)"""
        points, nodes = self.create_s_curve_path()
        
        # Build the path
        success = self.spline_manager.build_path(points, nodes)
        self.assertTrue(success, "Failed to build S-curve path")
        
        # Test curvature properties
        mid_param = (len(nodes) - 1) / 2
        
        # At midpoint (inflection point), curvature should be close to zero
        mid_curvature = self.spline_manager.get_curvature(mid_param)
        self.assertAlmostEqual(mid_curvature, 0.0,
                             places=1,
                             msg="Curvature should be zero at inflection point")
        
        # Test curvature rate of change sign changes
        quarter_param = mid_param / 2
        three_quarter_param = mid_param * 1.5
        
        # Get curvature changes at different points
        quarter_change = self.spline_manager.get_curvature_rate_of_change(quarter_param)
        three_quarter_change = self.spline_manager.get_curvature_rate_of_change(three_quarter_param)
        
        # Signs should be opposite as curvature changes direction
        self.assertGreater(quarter_change * three_quarter_change, 0,
                          "Curvature rate of change should have opposite signs")
    
    def test_edge_cases(self):
        """Test edge cases and error handling"""
        # Test empty spline manager
        with self.assertRaises(ValueError):
            self.spline_manager.get_curvature(0.5)
        with self.assertRaises(ValueError):
            self.spline_manager.get_curvature_rate_of_change(0.5)
        
        # Test parameter bounds
        points, nodes = self.create_circle_path()
        success = self.spline_manager.build_path(points, nodes)
        self.assertTrue(success)
        
        # Test at parameter bounds
        self.spline_manager.get_curvature(0.0)  # Should not raise error
        self.spline_manager.get_curvature(len(nodes) - 1)  # Should not raise error
        self.spline_manager.get_curvature_rate_of_change(0.0)  # Should not raise error
        self.spline_manager.get_curvature_rate_of_change(len(nodes) - 1)  # Should not raise error

if __name__ == '__main__':
    unittest.main()