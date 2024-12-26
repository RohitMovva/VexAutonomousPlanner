from typing import List, Optional, Tuple, Dict
import numpy as np
from dataclasses import dataclass
from splines.quintic_hermite_spline import QuinticHermiteSpline
from gui.node import Node

class QuinticHermiteSplineManager:
    """
    Manages multiple QuinticHermiteSplines to create a complete path through nodes
    with specific constraints.
    """
    
    def __init__(self):
        """
        Initialize the spline manager with empty splines and nodes lists.
        """
        self.splines: List[QuinticHermiteSpline]
        self.nodes: List[Node]
        self.path_parameters: Dict  # Store parameter mappings between global and local
        
    def build_path(self, points: np.ndarray, nodes: List[Node]) -> bool:
        """
        Build a complete path through the given points and nodes.
        Handles reverse nodes by creating separate spline segments with appropriate tangent handling.
        """

        if len(points) != len(nodes) or len(points) < 2:
            return False
   
        self.splines = []
        self.nodes = nodes
        
        current_start_idx = 0
        start_tangent = None
        
        for i in range(1, len(points)):
            if nodes[i].is_reverse_node or i == len(points) - 1:
                # Create spline segment
                current_points = points[current_start_idx:i+1]
                spline = QuinticHermiteSpline()
                
                if not spline.fit(current_points[:, 0], current_points[:, 1]):
                    return False
                
                if (start_tangent is not None):
                    spline.set_starting_tangent(start_tangent)
                    start_tangent = None
                    
                # Handle tangent continuity at reverse nodes
                if nodes[i].is_reverse_node:
                    # Calculate tangent considering direction
                    prev_vector = points[i] - points[i-1]
                    next_vector = points[i+1] - points[i] if i < len(points)-1 else points[i] - points[i-1]
                    
                    prev_vector = prev_vector / np.linalg.norm(prev_vector)
                    next_vector = next_vector / np.linalg.norm(next_vector)
                    # Set average tangent, average first term then average second term
                    dif_vector = (prev_vector - next_vector) / 2
                    
                    # Set end tangent for current spline
                    spline.set_ending_tangent(dif_vector)
                    start_tangent = -1*dif_vector


                self.splines.append(spline)
                
                if nodes[i].is_reverse_node and i < len(points) - 1:
                    current_start_idx = i

        return True
    
    def get_point_at_parameter(self, t: float) -> np.ndarray:
        """
        Get point on the complete path at parameter t.
        t is normalized to [0, 1] for the entire path.
        """
        if not self.splines:
            raise ValueError("No splines have been initialized")
            
        # Map to specific spline and local parameter
        spline_idx, local_t = self._map_parameter_to_spline(t)
        
        return self.splines[spline_idx].get_point(local_t)
    
    def get_derivative_at_parameter(self, t: float) -> np.ndarray:
        """
        Get first derivative on the complete path at parameter t.
        Handles direction changes at reverse nodes.
        """
        if not self.splines:
            raise ValueError("No splines have been initialized")
            
        spline_idx, local_t = self._map_parameter_to_spline(t)
        derivative = self.splines[spline_idx].get_derivative(local_t)
        
        # Check if we're in a reverse segment
        reverse_count = sum(1 for node in self.nodes[:spline_idx+1] if node.is_reverse_node)
        if reverse_count % 2 == 1:  # Odd number of reversals means we're going backwards
            derivative = -derivative
            
        return derivative
        
    def get_second_derivative_at_parameter(self, t: float) -> np.ndarray:
        """
        Get second derivative on the complete path at parameter t.
        Handles direction changes at reverse nodes.
        """
        if not self.splines:
            raise ValueError("No splines have been initialized")
            
        spline_idx, local_t = self._map_parameter_to_spline(t)
        second_derivative = self.splines[spline_idx].get_second_derivative(local_t)
        
        # Check if we're in a reverse segment
        reverse_count = sum(1 for node in self.nodes[:spline_idx+1] if node.is_reverse_node)
        if reverse_count % 2 == 1:  # Odd number of reversals means we're going backwards
            second_derivative = -second_derivative
            
        return second_derivative
        
    def _map_parameter_to_spline(self, t: float) -> Tuple[int, float]:
        """
        Map a global parameter t to a specific spline index and local parameter.
        The global parameter t ranges from 0 to N-1 where N is the total number of control points.
        
        Args:
            t: Global parameter value
            
        Returns:
            Tuple[int, float]: (spline_index, local_parameter)
            
        Raises:
            ValueError: If no splines exist
        """
        if not self.splines:
            raise ValueError("No splines have been initialized")
        
        # Find which spline segment we're in by checking the parameter ranges
        cumulative_points = 0
        for i, spline in enumerate(self.splines):
            num_points = len(spline.control_points)
            segment_start = cumulative_points
            segment_end = cumulative_points + num_points - 1
            
            if t <= segment_end or i == len(self.splines) - 1:
                # We've found our segment
                local_t = t - segment_start
                return i, local_t
                
            cumulative_points = cumulative_points + (num_points - 1)
        
        # This should never happen due to our handling
        raise ValueError("Failed to map parameter to spline segment")
    
    def _initialize_spline_segment(self, points: np.ndarray, 
                                 start_node: Node, 
                                 end_node: Node) -> QuinticHermiteSpline:
        """
        Initialize a single spline segment between two nodes with given constraints.
        """
        pass
    
    def _compute_transition_derivatives(self, prev_spline: QuinticHermiteSpline,
                                     next_spline: QuinticHermiteSpline,
                                     node: Node) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute derivatives at transition points between splines to ensure continuity.
        
        Returns:
            Tuple of (first_derivative, second_derivative) at transition
        """
        pass
    
    def get_total_arc_length(self) -> float:
        """
        Calculate total arc length of the complete path.
        """
        pass
    
    def get_curvature_at_parameter(self, t: float) -> float:
        """
        Calculate curvature at parameter t on the complete path.
        """
        pass
    
    def validate_path_continuity(self) -> bool:
        """
        Validate C2 continuity at all transition points between splines.
        """
        pass