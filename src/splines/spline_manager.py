import logging
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

from gui.action_point import ActionPoint
from gui.node import Node
from splines.quintic_hermite_spline import QuinticHermiteSpline

logger = logging.getLogger(__name__)

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
        self.action_points: List[ActionPoint] = []
        self.path_parameters: Dict = {}  # Store parameter mappings between global and local
        self.arc_length: float = 0.0  # Store the total arc length of the path
        self.lookup_table: Optional[PathLookupTable] = None
        self._precomputed_properties: Optional[Dict] = None
        self.set_tangents: Optional[np.ndarray] = None

    def build_path(
        self, points: np.ndarray, nodes: List[Node], action_points: List[ActionPoint], set_tangents: Optional[np.ndarray] = None
    ) -> bool:
        """
        Build a complete path through the given points and nodes.
        Handles reverse nodes and turn angles by creating separate spline segments
        with appropriate tangent handling.
        """
        if len(points) != len(nodes) or len(points) < 2:
            return False

        self.splines = []
        self.nodes = nodes
        self.action_points = action_points
        self.set_tangents = set_tangents

        current_start_idx = 0
        start_tangent = None

        for i in range(1, len(points)):
            if nodes[i].is_reverse_node or nodes[i].turn != 0 or i == len(points) - 1:
                # Create spline segment
                current_points = points[current_start_idx : i + 1]
                spline = QuinticHermiteSpline()
                logger.info(f"Setting tangents: {self.set_tangents}")
                tangents = []
                for i in range(len(self.set_tangents)):
                    if self.set_tangents[i] is not None and self.set_tangents[i][0] is not None:
                        tangents.append([self.set_tangents[i][0]*self.set_tangents[i][1], self.set_tangents[i][0]*self.set_tangents[i][2]])
                    else:
                        tangents.append([None, None])

                spline.set_all_tangents(tangents)

                if start_tangent is not None:
                    spline.starting_tangent = start_tangent
                    start_tangent = None

                # Handle tangent continuity at split points
                if nodes[i].is_reverse_node or nodes[i].turn != 0:
                    # Calculate segment lengths
                    prev_length = np.linalg.norm(points[i] - points[i - 1])
                    next_length = (
                        np.linalg.norm(points[i + 1] - points[i])
                        if i < len(points) - 1
                        else prev_length
                    )

                    # Calculate vectors and scale by segment lengths
                    prev_vector = (points[i] - points[i - 1]) * (
                        1.0 / prev_length if prev_length > 0 else 1.0
                    )
                    next_vector = (points[i + 1] - points[i]) * (
                        1.0 / next_length if next_length > 0 else 1.0
                    )

                    min_length = min(prev_length, next_length)

                    if nodes[i].turn != 0:
                        # Apply the turn angle directly (convert to radians)
                        target_angle_rad = np.radians(nodes[i].turn)
                        if nodes[i].is_reverse_node:
                            target_angle_rad = target_angle_rad + np.pi

                        # Create rotation matrix for the target angle
                        rotation_matrix = np.array(
                            [
                                [np.cos(target_angle_rad), -np.sin(target_angle_rad)],
                                [np.sin(target_angle_rad), np.cos(target_angle_rad)],
                            ]
                        )

                        # Apply rotation to the previous vector to get the next tangent
                        next_tangent = rotation_matrix @ prev_vector
                        next_tangent = next_tangent# * min_length

                        # Set end tangent for current spline and start tangent for next spline
                        spline.set_ending_tangent(prev_vector)# * min_length)
                        start_tangent = next_tangent

                    else:  # Reverse node
                        # Calculate difference vector and normalize for reverse nodes
                        dif_vector = prev_vector - next_vector
                        dif_norm = np.linalg.norm(dif_vector)

                        if dif_norm > 0:
                            dif_vector = dif_vector / dif_norm

                        # Scale the difference vector by the minimum segment length
                        min_length = min(prev_length, next_length)
                        dif_vector = dif_vector * min_length

                        # Set end tangent for current spline
                        spline.ending_tangent = dif_vector
                        start_tangent = -1 * dif_vector

                if not spline.fit(current_points[:, 0], current_points[:, 1]):
                    logger.error(f"Failed to fit spline for points {current_points}")
                    return False
                self.splines.append(spline)

                if (nodes[i].is_reverse_node or nodes[i].turn != 0) and i < len(
                    points
                ) - 1:
                    current_start_idx = i

            

        self.arc_length = None
        self.lookup_table = None
        return True

    def set_tangent_at_node(self, node: Node, tangent: np.ndarray, incoming_magnitude: float, outgoing_magnitude: float) -> None:
        """
        Set the tangent at the given node.
        """
        if not self.splines:
            raise ValueError("No splines have been initialized")
        
        self.set_tangents[self.nodes.index(node)] = [tangent, incoming_magnitude, outgoing_magnitude]
        spline_idx, local_t = self._map_parameter_to_spline(self.nodes.index(node))
        
        self.splines[spline_idx].set_tangent([tangent*incoming_magnitude, tangent*outgoing_magnitude], round(local_t))

    def get_magnitudes_at_parameter(self, idx):
        spline_idx, local_t = self._map_parameter_to_spline(idx)
        if (self.set_tangents[idx] is not None and self.set_tangents[idx][0] is not None):
            return [self.set_tangents[idx][1], self.set_tangents[idx][2]]
        
        if (spline_idx == 0 and local_t == 0):
            return [0, self.splines[spline_idx].get_magnitude(0)]
        elif (spline_idx == len(self.splines)-1 and local_t == self.splines[spline_idx].percent_to_parameter(100)):
            return [self.splines[spline_idx].get_magnitude(-1), 0]
        
        return [self.splines[spline_idx].get_magnitude(round(local_t)-1), self.splines[spline_idx].get_magnitude(round(local_t))]
        

        

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

    def percent_to_parameter(self, percent: float) -> np.ndarray:
        """
        Convert a percentage to a parameter on the spline.
        """
        if not self.splines:
            raise ValueError("No splines have been initialized")

        t = 0 + len(self.nodes) * (percent)
        # Clamp t to the range of the lookup table
        t = max(t, 0)
        t = min(t, len(self.nodes) - 1)

        return t

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
            return len(self.nodes) - 1

        # Find closest indices in lookup table
        idx = np.searchsorted(self.lookup_table.distances, distance)
        if idx == 0:
            return self.lookup_table.parameters[0]

        # Linear interpolation between points
        d0 = self.lookup_table.distances[idx - 1]
        d1 = self.lookup_table.distances[idx]
        t0 = self.lookup_table.parameters[idx - 1]
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
        return self._interpolate_property(t, "headings")

    def get_curvature(self, t: float) -> float:
        """
        Get curvature at parameter t using precomputed values and interpolation.
        """
        if self._precomputed_properties is None:
            self.precompute_path_properties()
        return self._interpolate_property(t, "curvatures")

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

        # Calculate curvature
        curvature = (x_prime * y_double_prime - y_prime * x_double_prime) / (
            speed_squared**1.5
        )

        return curvature

    def validate_path_continuity(self) -> bool:
        """
        Validate C2 continuity at all transition points between splines.
        """
        pass

    def build_lookup_table(
        self, min_samples=1000, max_samples=20000, tolerance=1e-6
    ) -> None:
        """Build lookup table for parameter to distance mapping."""
        if not self.splines:
            raise ValueError("No splines initialized")

        all_parameters = []
        all_distances = []
        current_dist = 0.0
        prev_param = 0.0
        for spline_idx, spline in enumerate(self.splines):
            # Get parameter range for this spline
            param_start = spline.parameters[0]
            param_end = spline.parameters[-1]

            # Create uniform parameter sampling
            local_params = np.linspace(param_start, param_end, min_samples)
            dt = local_params[1] - local_params[0]

            # Calculate derivatives and accumulate distances using trapezoidal rule
            derivatives = np.array([spline.get_derivative(t) for t in local_params])
            derivative_magnitudes = np.linalg.norm(derivatives, axis=1)

            # Calculate distances using trapezoidal rule
            partial_distances = np.zeros(len(local_params))
            partial_distances[1:] = np.cumsum(
                (derivative_magnitudes[:-1] + derivative_magnitudes[1:]) * 0.5 * dt
            )

            # Add offset from previous splines
            spline_distances = partial_distances + current_dist
            current_dist = spline_distances[-1]

            # Store results
            all_parameters.extend(local_params + prev_param)
            all_distances.extend(spline_distances)

            prev_param += param_end - param_start

        # Convert to numpy arrays
        all_parameters = np.array(all_parameters)
        all_distances = np.array(all_distances)

        # Create final lookup table
        self.lookup_table = PathLookupTable(
            distances=all_distances,
            parameters=all_parameters,
            total_length=current_dist,
        )

    def precompute_path_properties(self, samples_per_node: int = 1000) -> None:

        """
        Precompute curvature and heading at regular intervals for faster lookup.
        Optimized version with vectorization and reduced redundant calculations.
        """
        if not self.splines:
            raise ValueError("No splines initialized")

        num_samples = len(self.nodes) * samples_per_node
        
        parameters = np.linspace(0, len(self.nodes) - 1, num_samples)
        
        # Precompute properties
        curvatures = np.zeros(num_samples)

        headings = np.zeros(num_samples)
        
        # Time measurement variables
        start_time_total = time.time()
        
        # Vectorize the derivative calculations where possible
        start_time_derivative = time.time()
        first_derivs = np.array([self.get_derivative_at_parameter(t) for t in parameters])
        second_derivs = np.array([self.get_second_derivative_at_parameter(t) for t in parameters])
        end_time_derivative = time.time()
        logger.info(f"Derivative compute time: {end_time_derivative - start_time_derivative} seconds")

        # Extract components
        x_primes = first_derivs[:, 0]
        y_primes = first_derivs[:, 1]
        x_double_primes = second_derivs[:, 0]
        y_double_primes = second_derivs[:, 1]
        
        # Calculate speed squared (denominator for curvature)
        speed_squared = x_primes**2 + y_primes**2
        
        # Start curvature calculation timing
        start_time_curvature = time.time()
        
        # Vectorized curvature calculation
        numerator = x_primes * y_double_primes - y_primes * x_double_primes
        
        # Handle special case where speed is near zero
        mask = speed_squared >= 1e-10
        curvatures[mask] = numerator[mask] / (speed_squared[mask]**1.5)
        # For points where speed is near zero, curvature remains 0.0
        
        total_curvature_compute_time = time.time() - start_time_curvature
        
        # Start heading calculation timing
        start_time_heading = time.time()
        
        # Vectorized heading calculation
        headings = np.arctan2(y_primes, x_primes)
        
        total_heading_compute_time = time.time() - start_time_heading
        
        logger.info(f"Total compute time: {time.time() - start_time_total} seconds")
        logger.info(f"Curvature compute time: {total_curvature_compute_time} seconds")
        logger.info(f"Heading compute time: {total_heading_compute_time} seconds")
        
        self._precomputed_properties = {
            "parameters": parameters,
            "curvatures": curvatures,

            "headings": headings,
        }

    def _interpolate_property(self, t: float, property_name: str) -> float:
        """
        Get a property value at parameter t using linear interpolation of precomputed values.
        
        Args:
            t: Parameter value to interpolate at
            property_name: Name of the property to interpolate ('curvatures' or 'headings')
            
        Returns:
            float: Interpolated property value
        """
        # Cache property values to avoid repeated dictionary access
        params = self._precomputed_properties["parameters"]
        vals = self._precomputed_properties[property_name]
        
        # Find surrounding indices
        idx = np.searchsorted(params, t)
        if idx == 0:
            return vals[0]
        if idx >= len(params):
            return vals[-1]

        # Linear interpolation
        t0, t1 = params[idx - 1], params[idx]
        v0, v1 = vals[idx - 1], vals[idx]

        # Handle spline segment transitions
        if t0 % 1 != t1 % 1:
            return vals[idx - 1] if t % 1 > 0.5 else vals[idx]
            
        return v0 + (v1 - v0) * (t - t0) / (t1 - t0)

    def rebuild_tables(self):
        """
        Rebuild the spline tables after modifying control points or constraints.
        """

        start_time = time.time()
        self.build_lookup_table()
        build_time = time.time() - start_time
        logger.info(f"Lookup table build time: {build_time} seconds")
        start_time = time.time()
        self.precompute_path_properties()
        precompute_time = time.time() - start_time
        logger.info(f"Precompute time: {precompute_time} seconds")

