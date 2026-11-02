import logging
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

from gui.action_point import ActionPoint
from gui.node import Node
from splines.quintic_hermite_spline import QuinticHermiteSpline

logger = logging.getLogger(__name__)


def _unit(tangent) -> np.ndarray:
    """A node's tangent direction, normalised.

    A node stores a direction and a magnitude separately, but the direction it stores is
    whatever the user left in the dX/dY boxes and is not necessarily unit length - zeroing
    dY to point a node straight down the x axis leaves (0.857, 0), not (1, 0). Multiplying
    that by the magnitude hands the spline a tangent ~15% shorter than the magnitude says,
    and a short tangent is a tight curve. Normalise first so the magnitude means what the
    GUI claims it means.
    """
    tangent = np.asarray(tangent, dtype=float)
    norm = np.linalg.norm(tangent)
    if norm < 1e-9:
        return tangent
    return tangent / norm


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

    def build_path(
        self, points: np.ndarray, nodes: List[Node], action_points: List[ActionPoint]
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

        current_start_idx = 0
        start_tangent = None

        for i in range(1, len(points)):
            if nodes[i].is_reverse_node or nodes[i].turn != 0 or i == len(points) - 1:
                # Create spline segment
                current_points = points[current_start_idx : i + 1]
                spline = QuinticHermiteSpline()

                tangents = []
                curvatures = []
                for node in self.nodes[current_start_idx : i + 1]:
                    if node.tangent is not None:
                        direction = _unit(node.tangent)
                        tangents.append(
                            [
                                direction * node.incoming_magnitude,
                                direction * node.outgoing_magnitude,
                            ]
                        )
                    else:
                        tangents.append([None, None])

                    if node.curvature is not None:
                        curvatures.append(
                            [
                                node.curvature,
                                node.curvature,
                            ]
                        )
                    else:
                        curvatures.append([None, None])

                if (start_tangent is not None):
                    tangents[0][1] = start_tangent

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

                    next_vector = (
                        (points[i + 1] - points[i])
                        * (1.0 / next_length if next_length > 0 else 1.0)
                        if i < len(points) - 1
                        else prev_vector
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
                        next_tangent = next_tangent * min_length

                        prev_vector = prev_vector * min_length

                        if nodes[i].tangent is not None:
                            # Rotate the custom tangent by the same angle, so the
                            # outgoing direction is still incoming + turn. The pi for
                            # a reverse node is already folded into target_angle_rad.
                            direction = _unit(nodes[i].tangent)
                            prev_vector = direction * nodes[i].incoming_magnitude

                            next_tangent = rotation_matrix @ direction
                            next_tangent = next_tangent * nodes[i].outgoing_magnitude

                        # Set end tangent for current spline and start tangent for next spline
                        # (fit() reads a segment's ending tangent from slot 0 of its last point)
                        tangents[-1][0] = prev_vector
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

                        tangents[-1][0] = dif_vector
                        start_tangent = -1 * dif_vector
                        
                        if nodes[i].tangent is not None:
                            # If tangent is set, scale it by the outgoing magnitude
                            direction = _unit(nodes[i].tangent)
                            tangents[-1][0] = direction * nodes[i].incoming_magnitude

                            start_tangent = -1 * direction * nodes[i].outgoing_magnitude


                spline.set_all_tangents(tangents)
                spline.set_all_curvatures(curvatures)

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

    def get_magnitudes_at_parameter(self, idx):
        spline_idx, local_t = self._map_parameter_to_spline(idx)
        if self.nodes[idx].tangent is not None:
            return [
                self.nodes[idx].incoming_magnitude,
                self.nodes[idx].outgoing_magnitude,
            ]

        if spline_idx == 0 and local_t == 0:
            return [0, self.splines[spline_idx].get_magnitude(0)]
        elif spline_idx == len(self.splines) - 1 and local_t == self.splines[
            spline_idx
        ].percent_to_parameter(100):
            return [self.splines[spline_idx].get_magnitude(-1), 0]
        elif local_t == 0:
            return [
                self.splines[spline_idx - 1].get_magnitude(-1),
                self.splines[spline_idx].get_magnitude(0),
            ]
        elif local_t == self.splines[spline_idx].percent_to_parameter(100):
            return [
                self.splines[spline_idx].get_magnitude(-1),
                self.splines[spline_idx + 1].get_magnitude(0),
            ]

        return [
            self.splines[spline_idx].get_magnitude(round(local_t) - 1),
            self.splines[spline_idx].get_magnitude(round(local_t)),
        ]

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

    def _spline_param_offsets(self) -> np.ndarray:
        """Global parameter value at which each spline starts; last entry is the
        end of the path. Spline i spans [offsets[i], offsets[i+1]]."""
        counts = [len(spline.control_points) - 1 for spline in self.splines]
        return np.concatenate(([0], np.cumsum(counts)))

    def _map_parameters_to_splines(
        self, ts: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Vectorized _map_parameter_to_spline. Matches the scalar version's
        boundary convention: a parameter exactly on a split resolves to the
        spline on its left."""
        if not self.splines:
            raise ValueError("No splines have been initialized")

        offsets = self._spline_param_offsets()
        ts = np.clip(np.asarray(ts, dtype=float), 0.0, offsets[-1])
        spline_idxs = np.clip(
            np.searchsorted(offsets, ts, side="left") - 1, 0, len(self.splines) - 1
        )
        local_ts = ts - offsets[spline_idxs]
        return spline_idxs, local_ts

    def _evaluate_batch(self, ts: np.ndarray, method_name: str) -> np.ndarray:
        """Evaluate a per-spline batch method over the whole path, grouping the
        parameters by the spline they fall in."""
        ts = np.asarray(ts, dtype=float)
        spline_idxs, local_ts = self._map_parameters_to_splines(ts)
        out = np.empty((len(ts), 2))
        for i in np.unique(spline_idxs):
            mask = spline_idxs == i
            out[mask] = getattr(self.splines[i], method_name)(local_ts[mask])
        return out

    def get_points_at_parameters(self, ts: np.ndarray) -> np.ndarray:
        """Batch get_point_at_parameter. Returns (N, 2)."""
        return self._evaluate_batch(ts, "get_points")

    def get_derivatives_at_parameters(self, ts: np.ndarray) -> np.ndarray:
        """Batch get_derivative_at_parameter. Returns (N, 2)."""
        return self._evaluate_batch(ts, "get_derivatives")

    def get_second_derivatives_at_parameters(self, ts: np.ndarray) -> np.ndarray:
        """Batch get_second_derivative_at_parameter. Returns (N, 2)."""
        return self._evaluate_batch(ts, "get_second_derivatives")

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

    def time_to_distance(self, t: float) -> float:
        """Inverse of distance_to_time: arc length (feet) from the start to
        parameter t, clamped to the ends of the path."""
        if self.lookup_table is None:
            self.build_lookup_table()
        return float(
            np.interp(t, self.lookup_table.parameters, self.lookup_table.distances)
        )

    def distances_to_times(self, distances: np.ndarray) -> np.ndarray:
        """Batch distance_to_time. np.interp clamps to the table ends exactly
        like the scalar version's edge cases."""
        if self.lookup_table is None:
            self.build_lookup_table()
        return np.interp(
            distances, self.lookup_table.distances, self.lookup_table.parameters
        )

    def get_curvatures(self, ts: np.ndarray) -> np.ndarray:
        """Batch get_curvature."""
        if self._precomputed_properties is None:
            self.precompute_path_properties()
        return self._interpolate_properties(np.asarray(ts, dtype=float), "curvatures")

    def get_headings(self, ts: np.ndarray) -> np.ndarray:
        """Batch get_heading."""
        if self._precomputed_properties is None:
            self.precompute_path_properties()
        return self._interpolate_properties(np.asarray(ts, dtype=float), "headings")

    def _interpolate_properties(self, ts: np.ndarray, property_name: str) -> np.ndarray:
        """Vectorized _interpolate_property - same split-snapping and heading
        wrap rules, applied to a whole array of parameters at once."""
        params = self._precomputed_properties["parameters"]
        vals = self._precomputed_properties[property_name]

        idx = np.searchsorted(params, ts)
        idx_in = np.clip(idx, 1, len(params) - 1)
        t0, t1 = params[idx_in - 1], params[idx_in]
        v0, v1 = vals[idx_in - 1], vals[idx_in]

        # Split snapping (see _interpolate_property for the reasoning).
        floor_t0 = np.floor(t0)
        split = np.where(t0 == floor_t0, t0, floor_t0 + 1)
        snap = (split > 0) & (split <= t1)
        snapped = np.where(ts <= split, v0, v1)

        span = t1 - t0
        alpha = (ts - t0) / np.where(span > 0, span, 1.0)
        if property_name == "headings":
            delta = (v1 - v0 + np.pi) % (2 * np.pi) - np.pi
            lerped = v0 + delta * alpha
        else:
            lerped = v0 + (v1 - v0) * alpha

        out = np.where(snap, snapped, np.where(span == 0, v0, lerped))
        out = np.where(idx == 0, vals[0], out)
        out = np.where(idx >= len(params), vals[-1], out)
        return out

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
            derivatives = spline.get_derivatives(local_params)
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

    def precompute_path_properties(self, samples_per_foot: int = 1000) -> None:
        """
        Precomputes commonly queried properties to speed up motion profiling (python is slow)
        """
        if not self.splines:
            raise ValueError("No splines initialized")
        
        total_arc_length = self.get_total_arc_length()
        num_samples = int(total_arc_length * samples_per_foot)

        parameters = np.linspace(0, len(self.nodes) - 1, num_samples)

        curvatures = np.zeros(num_samples)

        start_time_total = time.time()

        first_derivs = self.get_derivatives_at_parameters(parameters)
        second_derivs = self.get_second_derivatives_at_parameters(parameters)

        # Extract components
        x_primes = first_derivs[:, 0]
        y_primes = first_derivs[:, 1]
        x_double_primes = second_derivs[:, 0]
        y_double_primes = second_derivs[:, 1]

        # Calculate speed squared (denominator for curvature)
        speed_squared = x_primes**2 + y_primes**2

        # Vectorized curvature calculation
        numerator = x_primes * y_double_primes - y_primes * x_double_primes

        # Handle special case where speed is near zero
        mask = speed_squared >= 1e-10
        curvatures[mask] = numerator[mask] / (speed_squared[mask] ** 1.5)

        # Vectorized heading calculation
        headings = np.arctan2(y_primes, x_primes)

        logger.info(f"Total compute time: {time.time() - start_time_total} seconds")

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

        t0, t1 = params[idx - 1], params[idx]
        v0, v1 = vals[idx - 1], vals[idx]

        # The path is split into separate splines at reverse and turn nodes, and
        # heading/curvature jump across such a split (by pi, at a reverse node).
        # Splits land on integer parameters, so if an integer falls inside this
        # sample interval, snap to whichever side of it t is on. Blending across a
        # split would return a value the path never actually takes. Nodes that
        # aren't splits are continuous and lose nothing by being snapped too.
        # At the split itself, take the left-hand value, matching what
        # _map_parameter_to_spline (and so get_point_at_parameter) resolves to.
        floor_t0 = np.floor(t0)
        split = t0 if t0 == floor_t0 else floor_t0 + 1
        if 0 < split <= t1:
            return v0 if t <= split else v1

        if t1 == t0:
            return v0

        alpha = (t - t0) / (t1 - t0)

        if property_name == "headings":
            # Headings come from arctan2 and wrap at +-pi. Interpolate along the
            # shortest arc so a wrap doesn't sweep the heading back through zero.
            delta = (v1 - v0 + np.pi) % (2 * np.pi) - np.pi
            return v0 + delta * alpha

        return v0 + (v1 - v0) * alpha

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
