from typing import List, Tuple, Dict, Optional
import numpy as np
from splines.quintic_hermite_spline import QuinticHermiteSpline
from gui.node import Node
from dataclasses import dataclass

@dataclass
class PathLookupTable:
    """Cache for quick parameter lookups based on distance"""
    distances: np.ndarray  # Sorted array of distances
    parameters: np.ndarray  # Corresponding parameter values
    total_length: float

class QuinticHermiteSplineManager:
    """
    Manages multiple QuinticHermiteSplines to create a complete path through nodes
    with specific constraints.
    """
    
    def __init__(self):
        """
        Initialize the spline manager with empty splines and nodes lists.
        """
        self.splines: List[QuinticHermiteSpline] = []
        self.nodes: List[Node] = []
        self.path_parameters: Dict = {}  # Store parameter mappings between global and local
        self.arc_length: float = 0.0  # Store the total arc length of the path
        self.lookup_table: Optional[PathLookupTable] = None
        self._precomputed_properties: Optional[Dict] = None
        
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
                    # print("\n=== Reverse Node Analysis ===")
                    # print(f"Node index: {i}")
                    # print(f"Node position: {points[i]}")
                    # print(f"Previous point: {points[i-1]}")
                    # print(f"Next point: {points[i+1]}" if i < len(points)-1 else "No next point")
                    
                    # Calculate segment lengths
                    prev_length = np.linalg.norm(points[i] - points[i-1])
                    next_length = np.linalg.norm(points[i+1] - points[i]) if i < len(points)-1 else prev_length
                    
                    # print(f"Previous segment length: {prev_length}")
                    # print(f"Next segment length: {next_length}")
                    
                    # Calculate vectors and scale by segment lengths
                    prev_vector = (points[i] - points[i-1]) * (1.0 / prev_length if prev_length > 0 else 1.0)
                    next_vector = (points[i+1] - points[i]) * (1.0 / next_length if next_length > 0 else 1.0)
                    
                    # print(f"Scaled previous vector: {prev_vector}")
                    # print(f"Scaled next vector: {next_vector}")
                    
                    # Calculate difference vector and normalize
                    dif_vector = prev_vector - next_vector
                    dif_norm = np.linalg.norm(dif_vector)
                    # print(f"Initial difference vector: {dif_vector}")
                    # print(f"Difference vector norm: {dif_norm}")
                    
                    if dif_norm > 0:
                        dif_vector = dif_vector / dif_norm
                    
                    # Scale the difference vector by the minimum segment length
                    min_length = min(prev_length, next_length)
                    dif_vector = dif_vector * min_length
                    
                    # print(f"Final difference vector: {dif_vector}")
                    # print(f"Min segment length used for scaling: {min_length}")
                    # print("===========================\n")
                    
                    # Set end tangent for current spline
                    spline.set_ending_tangent(dif_vector)
                    start_tangent = -1*dif_vector


                self.splines.append(spline)
                
                if nodes[i].is_reverse_node and i < len(points) - 1:
                    current_start_idx = i

        self.arc_length = None
        self.lookup_table = None
        # self.rebuild_tables()
        # After successful path building, initialize optimization structures
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
        Convert distance to parameter t using the lookup table and linear interpolation.
        Much faster than binary search + numerical integration.
        """
        if self.lookup_table is None:
            self.build_lookup_table()
            
        # Handle edge cases
        if distance <= 0:
            return 0
        if distance >= self.lookup_table.total_length:
            # print("Distance exceeds total length")
            return len(self.nodes) - 1
            
        # Find closest indices in lookup table
        idx = np.searchsorted(self.lookup_table.distances, distance)
        if idx == 0:
            return self.lookup_table.parameters[0]
            
        # Linear interpolation between points
        d0 = self.lookup_table.distances[idx-1]
        d1 = self.lookup_table.distances[idx]
        t0 = self.lookup_table.parameters[idx-1]
        t1 = self.lookup_table.parameters[idx]
        
        # Interpolate
        t = t0 + (t1 - t0) * (distance - d0) / (d1 - d0)
        return t

    
    def get_total_arc_length(self) -> float:
        """
        Calculate the total arc length using adaptive Gaussian quadrature.
        Adaptively subdivides based on curvature and speed variation.
        """
        if not self.splines:
            raise ValueError("No splines have been initialized")
        if self.lookup_table is None:
            self.build_lookup_table()
            
        return self.lookup_table.total_length
    def get_heading(self, t: float) -> float:
        """
        Get heading at parameter t using precomputed values and interpolation.
        """
        if self._precomputed_properties is None:
            self.precompute_path_properties()
        return self._interpolate_property(t, 'headings')
    
    def get_curvature(self, t: float) -> float:
        """
        Get curvature at parameter t using precomputed values and interpolation.
        """
        if self._precomputed_properties is None:
            self.precompute_path_properties()
        return self._interpolate_property(t, 'curvatures')
    
    def _get_heading(self, t: float) -> float:
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

    def _get_curvature(self, t: float) -> float:
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
        curvature = (x_prime * y_double_prime - y_prime * x_double_prime) / (speed_squared ** 1.5)
        
        return curvature
        
    def validate_path_continuity(self) -> bool:
        """
        Validate C2 continuity at all transition points between splines.
        """
        pass

    def build_lookup_table(self, min_samples=1000, max_samples=50000, tolerance=1e-6) -> None:
        """
        Build a distance-to-parameter lookup table using adaptive Gaussian quadrature.
        """
        if not self.splines:
            raise ValueError("No splines initialized")
        
        all_parameters = []
        all_distances = []
        current_dist = 0.0
        
        # Gauss-Legendre quadrature points and weights (7-point)
        gauss_points = np.array([
            -0.949107912342759,
            -0.741531185599394,
            -0.405845151377397,
            0.000000000000000,
            0.405845151377397,
            0.741531185599394,
            0.949107912342759
        ])
        
        gauss_weights = np.array([
            0.129484966168870,
            0.279705391489277,
            0.381830050505119,
            0.417959183673469,
            0.381830050505119,
            0.279705391489277,
            0.129484966168870
        ])
        
        for spline_idx, spline in enumerate(self.splines):
            param_start = spline.parameters[0]
            param_end = spline.parameters[-1]
            
            # Determine initial segment count based on curvature
            curvature_samples = 10
            test_params = np.linspace(param_start, param_end, curvature_samples)
            max_curvature = max(abs(spline.get_curvature(t)) for t in test_params)
            base_segments = max(int(100 * max_curvature + 50), min_samples // len(self.splines))
            n_segments = min(base_segments, max_samples // len(self.splines))
            
            # Create parameter points with higher density in high-curvature regions
            local_params = np.linspace(param_start, param_end, n_segments + 1)
            segment_distances = np.zeros(n_segments)
            
            # Compute arc length for each segment using Gaussian quadrature
            for i in range(n_segments):
                t0, t1 = local_params[i], local_params[i + 1]
                mid = (t0 + t1) / 2
                half_length = (t1 - t0) / 2
                
                # Transform Gaussian points to segment interval
                t_points = mid + half_length * gauss_points
                
                # Compute derivatives at all points efficiently
                derivatives = np.array([spline.get_derivative(t) for t in t_points])
                speeds = np.linalg.norm(derivatives, axis=1)
                
                # Compute segment length using Gaussian quadrature
                segment_distances[i] = half_length * np.sum(gauss_weights * speeds)
            
            # Compute cumulative distances
            cumulative_distances = np.zeros(len(local_params))
            cumulative_distances[1:] = np.cumsum(segment_distances)
            
            # Add offset from previous splines
            spline_distances = cumulative_distances + current_dist
            current_dist = spline_distances[-1]
            
            # Store results
            all_parameters.extend(local_params)
            all_distances.extend(spline_distances)
        
        # Convert to numpy arrays
        all_parameters = np.array(all_parameters)
        all_distances = np.array(all_distances)
        
        # Create final lookup table
        self.lookup_table = PathLookupTable(
            distances=all_distances,
            parameters=all_parameters,
            total_length=current_dist
        )
        
    def precompute_path_properties(self, num_samples: int = 100000) -> None:
        """
        Precompute curvature and heading at regular intervals for faster lookup.
        """
        if not self.splines:
            raise ValueError("No splines initialized")
            
        parameters = np.linspace(0, len(self.nodes) - 1, num_samples)
        
        # Precompute properties
        curvatures = np.zeros(num_samples)
        headings = np.zeros(num_samples)
        
        for i, t in enumerate(parameters):
            curvatures[i] = self._get_curvature(t)
            headings[i] = self._get_heading(t)
            
        self._precomputed_properties = {
            'parameters': parameters,
            'curvatures': curvatures,
            'headings': headings
        }


    def _interpolate_property(self, t: float, property_name: str) -> float:
        """
        Get a property value at parameter t using linear interpolation of precomputed values.
        """
        parameters = self._precomputed_properties['parameters']
        values = self._precomputed_properties[property_name]
        
        # Find surrounding indices
        idx = np.searchsorted(parameters, t)
        if idx == 0:
            return values[0]
        if idx >= len(parameters):
            return values[-1]
            
        # Linear interpolation
        t0 = parameters[idx-1]
        t1 = parameters[idx]
        v0 = values[idx-1]
        v1 = values[idx]
        
        return v0 + (v1 - v0) * (t - t0) / (t1 - t0)
    
    def rebuild_tables(self):
        """
        Rebuild the spline tables after modifying control points or constraints.
        """
        self.build_lookup_table()
        self.precompute_path_properties()