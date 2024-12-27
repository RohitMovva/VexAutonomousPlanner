from typing import List, Tuple, Dict
import numpy as np
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
        self.arc_length: float  # Store the total arc length of the path
        
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

        self.arc_length = None

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

    def distance_to_time(self, distance: float) -> float:
        """
        Convert a distance along the path to the corresponding parameter t.
        Uses binary search to find the parameter value that corresponds to the given arc length.
        
        Args:
            distance: Distance along the path from the start
            
        Returns:
            float: Parameter t that corresponds to the given distance
            
        Raises:
            ValueError: If no splines have been initialized or distance is invalid
        """
        if not self.splines:
            raise ValueError("No splines have been initialized")
            
        # Get total path length for validation
        total_length = self.get_total_arc_length()
        if (distance < 0):
            distance = 0
        if (distance > total_length):
            distance = total_length
        # if distance < 0 or distance > total_length:
            # raise ValueError(f"Distance {distance} outside valid range [0, {total_length}]")
            
        # Special cases
        if distance == 0:
            return 0
        if distance == total_length:
            return len(self.nodes) - 1
            
        # Binary search to find parameter t
        t_min = 0
        t_max = len(self.nodes) - 1
        tolerance = 1e-6  # Tolerance for distance comparison
        max_iterations = 50  # Maximum number of binary search iterations
        
        # Function to compute arc length from start to parameter t
        def compute_length_to_t(t: float) -> float:
            length = 0.0
            # For each spline segment up to t
            curr_t = 0
            for spline in self.splines:
                segment_length = t - curr_t
                if segment_length <= 0:
                    break
                    
                # Calculate length for this segment using Simpson's rule
                n = 100  # number of intervals
                h = min(1.0, segment_length) / n  # step size
                
                segment_length = 0.0
                # First point
                derivative = spline.get_derivative(0)
                segment_length += np.linalg.norm(derivative)
                
                # Middle points
                for i in range(1, n):
                    local_t = i * h
                    derivative = spline.get_derivative(local_t)
                    weight = 4 if i % 2 == 1 else 2
                    segment_length += weight * np.linalg.norm(derivative)
                
                # Last point
                derivative = spline.get_derivative(min(1.0, segment_length))
                segment_length += np.linalg.norm(derivative)
                
                # Complete Simpson's rule calculation
                length += (h / 3) * segment_length
                curr_t += 1
                
                if curr_t >= t:
                    break
                    
            return length
            
        # Binary search
        for _ in range(max_iterations):
            t_mid = (t_min + t_max) / 2
            length_at_t = compute_length_to_t(t_mid)
            
            if abs(length_at_t - distance) < tolerance:
                return t_mid
                
            if length_at_t < distance:
                t_min = t_mid
            else:
                t_max = t_mid
                
        # Return best approximation after max iterations
        return (t_min + t_max) / 2

    
    def get_total_arc_length(self) -> float:
        """
        Calculate the total arc length of the entire path by summing the
        arc lengths of all individual spline segments.

        Returns:
            float: Total arc length of the complete path

        Raises:
            ValueError: If no splines have been initialized
        """
        if not self.splines:
            raise ValueError("No splines have been initialized")
            
        # Check if we have cached arc length
        if self.arc_length is not None:
            return self.arc_length
            
        # Sum up the arc lengths of all spline segments
        total_length = 0.0
        for spline in self.splines:
            total_length += spline.get_total_arc_length()
            
        # Cache the result for future use
        self.arc_length = total_length
        
        return total_length
    
    def get_heading(self, t: float) -> float:
        """
        Get the heading (angle in radians) at parameter t on the complete path.
        The heading is calculated as the angle of the tangent vector (first derivative)
        relative to the positive x-axis.
        
        Args:
            t: Parameter value normalized to the entire path length
            
        Returns:
            float: Heading angle in radians in range [-π, π]
            
        Raises:
            ValueError: If no splines have been initialized
        """
        if not self.splines:
            raise ValueError("No splines have been initialized")
            
        # Get first derivative at parameter t
        derivative = self.get_derivative_at_parameter(t)
        
        # Calculate heading angle using arctan2
        # arctan2(y, x) returns angle in range [-π, π]
        heading = np.arctan2(derivative[1], derivative[0])
        
        return heading

    def get_curvature_at_parameter(self, t: float) -> float:
        """
        Calculate curvature at parameter t on the complete path.
        
        The curvature is calculated using the formula:
        κ = |x'y'' - y'x''| / (x'² + y'²)^(3/2)
        where x', y' are components of the first derivative and
        x'', y'' are components of the second derivative.
        
        Args:
            t: Parameter value normalized to the entire path length
            
        Returns:
            float: Curvature at the given parameter value
            
        Raises:
            ValueError: If no splines have been initialized
        """
        if not self.splines:
            raise ValueError("No splines have been initialized")
            
        # Get first and second derivatives at parameter t
        first_deriv = self.get_derivative_at_parameter(t)
        second_deriv = self.get_second_derivative_at_parameter(t)
        
        # Extract x and y components
        x_prime = first_deriv[0]
        y_prime = first_deriv[1]
        x_double_prime = second_deriv[0]
        y_double_prime = second_deriv[1]
        
        # Calculate denominator (speed squared)
        speed_squared = x_prime**2 + y_prime**2
        
        # Handle special case where speed is zero (singular point)
        if speed_squared < 1e-10:  # Small threshold to avoid division by zero
            return 0.0
            
        # Calculate numerator (cross product of first and second derivatives)
        numerator = abs(x_prime * y_double_prime - y_prime * x_double_prime)
        
        # Calculate curvature
        curvature = numerator / (speed_squared ** 1.5)
        
        return curvature
        
    def validate_path_continuity(self) -> bool:
        """
        Validate C2 continuity at all transition points between splines.
        """
        pass