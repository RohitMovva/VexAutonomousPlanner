import logging
from typing import Optional, Tuple

import numpy as np

from splines.spline import Spline  # type: ignore

logger = logging.getLogger(__name__)


class QuinticHermiteSpline(Spline):
    """
    Quintic Hermite spline implementation that interpolates between points using
    position, first derivative, and second derivative constraints.
    """

    def __init__(self):
        """
        Initialize the QuinticHermiteSpline with additional arrays for storing
        derivative constraints at control points.
        """
        super().__init__()
        self.first_derivatives: Optional[np.ndarray] = None
        self.second_derivatives: Optional[np.ndarray] = None
        self.starting_tangent: Optional[np.ndarray] = None
        self.ending_tangent: Optional[np.ndarray] = None

        self.set_tangents: Optional[np.ndarray] = None

    def fit(
        self,
        x: np.ndarray,
        y: np.ndarray,
        first_derivatives: Optional[np.ndarray] = None,
        second_derivatives: Optional[np.ndarray] = None,
    ) -> bool:
        if len(x) != len(y):
            return False

        if len(x) < 2:
            return False

        logger.debug("\n=== Fitting Spline ===")
        logger.debug(f"Input points: x={x}, y={y}")

        self.control_points = np.column_stack((x, y))

        try:
            self.parameters = self._compute_parameters(self.control_points)
            logger.debug(f"Computed parameters: {self.parameters}")

            # Calculate and log segment lengths
            diffs = np.diff(self.control_points, axis=0)
            segment_lengths = np.linalg.norm(diffs, axis=1)
            logger.debug(f"Segment lengths: {segment_lengths}")
            logger.debug(f"Average segment length: {np.mean(segment_lengths)}")

            if first_derivatives is not None:
                if len(first_derivatives) != len(x):
                    return False
                self.first_derivatives = first_derivatives
                logger.debug(f"Using provided first derivatives: {first_derivatives}")

            if second_derivatives is not None:
                if len(second_derivatives) != len(x):
                    return False
                self.second_derivatives = second_derivatives
                logger.debug(f"Using provided second derivatives: {second_derivatives}")

            if self.first_derivatives is None or self.second_derivatives is None:
                self._compute_derivatives()

            self.segments = []
            self.segment_lengths = []
            logger.debug("\n=== Computing Segments ===")
            for i in range(len(x) - 1):
                p0 = self.control_points[i]
                p1 = self.control_points[i + 1]
                d0 = self.first_derivatives[i]
                d1 = self.first_derivatives[i + 1]
                dd0 = self.second_derivatives[i]
                dd1 = self.second_derivatives[i + 1]

                segment_length = np.linalg.norm(p1 - p0)
                self.segment_lengths.append(segment_length)
                if (i > 0):
                    prev_segment_length = (np.linalg.norm(self.control_points[i+1] - self.control_points[i]) + np.linalg.norm(self.control_points[i] - self.control_points[i-1])) / 2
                # else:
                prev_segment_length = segment_length

                if (i < len(x) - 2):
                    next_segment_length = (np.linalg.norm(self.control_points[i+2] - self.control_points[i+1]) + np.linalg.norm(self.control_points[i+1] - self.control_points[i])) / 2
                # else:
                next_segment_length = segment_length

                logger.debug(f"\nSegment {i}:")
                logger.debug(f"  Points: p0={p0}, p1={p1}")
                logger.debug(f"  Original derivatives: d0={d0}, d1={d1}")
                logger.debug(f"  Original second derivatives: dd0={dd0}, dd1={dd1}")

                if segment_length > 0:
                    # Scale derivatives by segment length
                    d0_scaled = d0 * prev_segment_length
                    d1_scaled = d1 * next_segment_length
                    dd0_scaled = dd0 * (prev_segment_length**2)
                    dd1_scaled = dd1 * (next_segment_length**2)

                    logger.info(f"Set tanents: {self.set_tangents}")
                    if (self.set_tangents and self.set_tangents[i] and self.set_tangents[i][1]):
                        logger.info(f"Setting tangent: {self.set_tangents[i][1]}")
                        d0_scaled = self.set_tangents[i][1]
                    if (self.set_tangents and self.set_tangents[i+1] and self.set_tangents[i+1][0]):
                        logger.info(f"Setting tangent: {self.set_tangents[i+1][0]}")
                        d1_scaled = self.set_tangents[i+1][0]

                    logger.debug(f"  Scaled derivatives: d0={d0_scaled}, d1={d1_scaled}")
                    logger.debug(
                        f"  Scaled second derivatives: dd0={dd0_scaled}, dd1={dd1_scaled}"
                    )

                    segment = np.vstack(
                        [p0, p1, d0_scaled, d1_scaled, dd0_scaled, dd1_scaled]
                    )
                else:
                    logger.debug("  Warning: Zero segment length detected")
                    segment = np.vstack([p0, p1, d0, d1, dd0, dd1])

                self.segments.append(segment)

            if self.starting_tangent is not None:
                self.set_starting_tangent(self.starting_tangent)
            if self.ending_tangent is not None:
                self.set_ending_tangent(self.ending_tangent)

            return True

        except Exception as e:
            logger.error(f"Error during fitting: {str(e)}")
            return False

    def set_tangent(self, tangent: np.ndarray, index: int):
        if (self.set_tangents is None):
            self.set_tangents = np.zeros_like(self.control_points, dtype=float)
        self.set_tangents[index] = tangent
        print(f"Set tangents: {self.set_tangents}")

    def set_all_tangents(self, tangents: np.ndarray):
        logger.debug(f"Setting all tangents: {tangents}")
        self.set_tangents = tangents

    def _compute_derivatives(self) -> None:
        """Modified version with corrected second derivative scaling"""
        logger.debug("\n=== Computing Derivatives ===")
        num_points = len(self.control_points)

        if self.first_derivatives is None:
            self.first_derivatives = np.zeros_like(self.control_points, dtype=float)
        if self.second_derivatives is None:
            self.second_derivatives = np.zeros_like(self.control_points, dtype=float)

        diffs = np.diff(self.control_points, axis=0)
        distances = np.linalg.norm(diffs, axis=1)
        logger.debug(f"Segment distances: {distances}")

        # First derivative calculation
        chords = diffs.copy()
        scale_factor = 1.0

        logger.debug("\nComputing first derivatives:")
        for i in range(num_points):
            # if (self.set_tangents is not None):
            #     logger.info(f"Comparing {self.set_tangents[i]} to {np.array([0, 0], dtype=float)}")
            # if (self.set_tangents is not None and self.set_tangents[i][0] is not None):
            #     logger.info(f"Setting tangent at index {i} to {self.set_tangents[i]}")
            #     print(f"Setting tangent at index {i} to {self.set_tangents[i]}")
            #     self.first_derivatives[i] = self.set_tangents[i]
            #     if (i == 0):
            #         self.first_derivatives[i] /= np.linalg.norm(self.control_points[i+1] - self.control_points[i])
            #     elif i == num_points - 1:
            #         self.first_derivatives[i] /= np.linalg.norm(self.control_points[i] - self.control_points[i-1])
            #     else:
            #         self.first_derivatives[i] /= (np.linalg.norm(self.control_points[i+1] - self.control_points[i]) + np.linalg.norm(self.control_points[i] - self.control_points[i-1])) / 2
            #     self.first_derivatives[i] *= scale_factor

            #     logger.info(f"First derivative: {self.first_derivatives[i]}")
            #     continue
            
            if i == 0:
                if (num_points == 2 and self.ending_tangent is not None):
                    # If only one point and ending tangent is set, use it
                    self.first_derivatives[i] = chords[0] * scale_factor
                else:
                    self.first_derivatives[i] = chords[0] * scale_factor / distances[0]
                logger.debug(
                    f"First point: {self.first_derivatives[i]} (using distance {distances[0]})"
                )
            elif i == num_points - 1:
                if (num_points == 2 and self.starting_tangent is not None):
                    self.first_derivatives[i] = chords[-1] * scale_factor
                else:
                    self.first_derivatives[i] = chords[-1] * scale_factor / distances[-1]
                logger.debug(
                    f"Last point: {self.first_derivatives[i]} (using distance {distances[-1]})"
                )
            else:
                prev_chord = chords[i - 1] / distances[i - 1]
                next_chord = chords[i] / distances[i]
                self.first_derivatives[i] = (prev_chord + next_chord) * scale_factor / 2
                logger.debug(f"Interior point {i}: {self.first_derivatives[i]}")

        # Second derivative calculation
        logger.debug("\nComputing second derivatives:")
        for i in range(num_points):
            if i == 0:
                # For the first point, use forward difference
                forward_tangent = self.first_derivatives[1]
                current_tangent = self.first_derivatives[0]
                self.second_derivatives[i] = (
                    forward_tangent - current_tangent
                ) / distances[0]
                self.second_derivatives[i] = 0.0  # Cannot start with a nonzero curvature
                logger.debug(f"First point: {self.second_derivatives[i]}")

            elif i == num_points - 1:
                # For the last point, use backward difference
                backward_tangent = self.first_derivatives[-2]
                current_tangent = self.first_derivatives[-1]
                self.second_derivatives[i] = (
                    current_tangent - backward_tangent
                ) / distances[-1]
                self.second_derivatives[i] = 0.0  # Cannot end with a nonzero curvature
                logger.debug(f"Last point: {self.second_derivatives[i]}")

            else:
                # For interior points
                prev_dist = distances[i - 1]
                next_dist = distances[i]

                avg_dist = (prev_dist + next_dist) / 2
                self.second_derivatives[i] = (
                    self.first_derivatives[i + 1] - self.first_derivatives[i - 1]
                ) / (0.5 * avg_dist)

                logger.debug(f"Interior point {i}: {self.second_derivatives[i]}")

    def get_point(self, t: float) -> np.ndarray:
        if not self.segments:
            raise ValueError("Spline has not been fitted yet")

        local_t, segment_idx = self._normalize_parameter(t)
        basis = self._get_basis_functions(local_t)

        # Debug logging for specific parameter values
        if (
            abs(local_t - 0.0) < 1e-6
            or abs(local_t - 1.0) < 1e-6
            or abs(local_t - 0.5) < 1e-6
        ):
            logger.debug(f"\n=== Computing point at t={t} (local_t={local_t}) ===")
            logger.debug(f"Segment index: {segment_idx}")
            logger.debug(f"Basis functions: {basis}")

        segment = self.segments[segment_idx]
        point = np.zeros(2)
        for i in range(6):
            contribution = basis[i] * segment[i]
            if abs(local_t - 0.5) < 1e-6:  # Log details at midpoint
                logger.debug(
                    f"  Basis[{i}] * segment[{i}] = {basis[i]} * {segment[i]} = {contribution}"
                )
            point += contribution

        if abs(local_t - 0.5) < 1e-6:
            logger.debug(f"Final point: {point}")

        return point
    
    def get_magnitude(self, idx):
        logger.info(f"hello {idx}, {self.segment_lengths}")
        return self.segment_lengths[idx]
    
    def percent_to_point(self, percent: float) -> np.ndarray:
        """
        Convert a percentage to a point on the spline.

        Args:
            percent: Percentage to convert (0 to 100)

        Returns:
            np.ndarray: Point on the spline
        """
        if not self.segments:
            raise ValueError("Spline has not been fitted yet")
        
        t = self.parameters[0] + self.parameters[-1] * (percent/100)

        return self.get_point(t)
    
    def percent_to_parameter(self, percent: float) -> float:
        """
        Convert a percentage to a parameter on the spline.

        Args:
            percent: Percentage to convert (0 to 100)

        Returns:
            float: Parameter value between 0 and 1
        """
        if not self.segments:
            raise ValueError("Spline has not been fitted yet")

        return self.parameters[0] + self.parameters[-1] * (percent / 100)

    def _get_basis_functions(self, t: float) -> np.ndarray:
        """
        Compute the quintic Hermite basis functions at parameter t.

        The basis functions are:
        H₀(t) = 1 - 10t³ + 15t⁴ - 6t⁵    # Position at start point
        H₁(t) = 10t³ - 15t⁴ + 6t⁵        # Position at end point
        H₂(t) = t - 6t³ + 8t⁴ - 3t⁵      # First derivative at start point
        H₃(t) = -4t³ + 7t⁴ - 3t⁵         # First derivative at end point
        H₄(t) = 0.5t² - 1.5t³ + 1.5t⁴ - 0.5t⁵  # Second derivative at start point
        H₅(t) = 0.5t³ - t⁴ + 0.5t⁵       # Second derivative at end point

        Args:
            t: Parameter value between 0 and 1

        Returns:
            np.ndarray: Array containing the six basis functions [H₀, H₁, H₂, H₃, H₄, H₅]
        """
        # Precompute powers of t for efficiency
        t2 = t * t
        t3 = t2 * t
        t4 = t3 * t
        t5 = t4 * t

        # Compute the basis functions
        H0 = 1 - 10 * t3 + 15 * t4 - 6 * t5  # Position at start point
        H1 = 10 * t3 - 15 * t4 + 6 * t5  # Position at end point
        H2 = t - 6 * t3 + 8 * t4 - 3 * t5  # First derivative at start point
        H3 = -4 * t3 + 7 * t4 - 3 * t5  # First derivative at end point
        H4 = (
            0.5 * t2 - 1.5 * t3 + 1.5 * t4 - 0.5 * t5
        )  # Second derivative at start point
        H5 = 0.5 * t3 - t4 + 0.5 * t5  # Second derivative at end point

        return np.array([H0, H1, H2, H3, H4, H5])

    def _get_basis_derivatives(self, t: float) -> np.ndarray:
        """
        Compute the derivatives of quintic Hermite basis functions at parameter t.

        The derivatives of the basis functions are:
        H₀'(t) = -30t² + 60t³ - 30t⁴        # Position at start point
        H₁'(t) = 30t² - 60t³ + 30t⁴         # Position at end point
        H₂'(t) = 1 - 18t² + 32t³ - 15t⁴     # First derivative at start point
        H₃'(t) = -12t² + 28t³ - 15t⁴        # First derivative at end point
        H₄'(t) = t - 4.5t² + 6t³ - 2.5t⁴    # Second derivative at start point
        H₅'(t) = 1.5t² - 4t³ + 2.5t⁴        # Second derivative at end point

        Args:
            t: Parameter value between 0 and 1

        Returns:
            np.ndarray: Array containing the derivatives of basis functions [H₀', H₁', H₂', H₃', H₄', H₅']
        """
        # Precompute powers of t for efficiency
        t2 = t * t
        t3 = t2 * t
        t4 = t3 * t

        # Compute the derivatives of basis functions
        H0_prime = -30 * t2 + 60 * t3 - 30 * t4  # Derivative of position at start point
        H1_prime = 30 * t2 - 60 * t3 + 30 * t4  # Derivative of position at end point
        H2_prime = (
            1 - 18 * t2 + 32 * t3 - 15 * t4
        )  # Derivative of first derivative at start point
        H3_prime = (
            -12 * t2 + 28 * t3 - 15 * t4
        )  # Derivative of first derivative at end point
        H4_prime = (
            t - 4.5 * t2 + 6 * t3 - 2.5 * t4
        )  # Derivative of second derivative at start point
        H5_prime = (
            1.5 * t2 - 4 * t3 + 2.5 * t4
        )  # Derivative of second derivative at end point

        return np.array([H0_prime, H1_prime, H2_prime, H3_prime, H4_prime, H5_prime])

    def _get_basis_second_derivatives(self, t: float) -> np.ndarray:
        """
        Compute the second derivatives of quintic Hermite basis functions at parameter t.

        The second derivatives of the basis functions are:
        H₀''(t) = -60t + 180t² - 120t³      # Position at start point
        H₁''(t) = 60t - 180t² + 120t³       # Position at end point
        H₂''(t) = -36t + 96t² - 60t³        # First derivative at start point
        H₃''(t) = -24t + 84t² - 60t³        # First derivative at end point
        H₄''(t) = 1 - 9t + 18t² - 10t³      # Second derivative at start point
        H₅''(t) = 3t - 12t² + 10t³          # Second derivative at end point

        Args:
            t: Parameter value between 0 and 1

        Returns:
            np.ndarray: Array containing the second derivatives of basis functions [H₀'', H₁'', H₂'', H₃'', H₄'', H₅'']
        """
        # Precompute powers of t for efficiency
        t2 = t * t
        t3 = t2 * t

        # Compute the second derivatives of basis functions
        H0_double_prime = (
            -60 * t + 180 * t2 - 120 * t3
        )  # Second derivative of position at start point
        H1_double_prime = (
            60 * t - 180 * t2 + 120 * t3
        )  # Second derivative of position at end point
        H2_double_prime = (
            -36 * t + 96 * t2 - 60 * t3
        )  # Second derivative of first derivative at start point
        H3_double_prime = (
            -24 * t + 84 * t2 - 60 * t3
        )  # Second derivative of first derivative at end point
        H4_double_prime = (
            1 - 9 * t + 18 * t2 - 10 * t3
        )  # Second derivative of second derivative at start point
        H5_double_prime = (
            3 * t - 12 * t2 + 10 * t3
        )  # Second derivative of second derivative at end point

        return np.array(
            [
                H0_double_prime,
                H1_double_prime,
                H2_double_prime,
                H3_double_prime,
                H4_double_prime,
                H5_double_prime,
            ]
        )

    def _get_basis_third_derivatives(self, t: float) -> np.ndarray:
        """
        Compute the third derivatives of quintic Hermite basis functions at parameter t.

        The third derivatives of the basis functions are:
        H₀'''(t) = -60 + 360t - 360t²      # Position at start point
        H₁'''(t) = 60 - 360t + 360t²       # Position at end point
        H₂'''(t) = -36 + 192t - 180t²      # First derivative at start point
        H₃'''(t) = -24 + 168t - 180t²      # First derivative at end point
        H₄'''(t) = -9 + 36t - 30t²         # Second derivative at start point
        H₅'''(t) = 3 - 24t + 30t²          # Second derivative at end point

        Args:
            t: Parameter value between 0 and 1

        Returns:
            np.ndarray: Array containing the third derivatives of basis functions
                    [H₀''', H₁''', H₂''', H₃''', H₄''', H₅''']
        """
        # Precompute power of t for efficiency
        t2 = t * t

        # Compute the third derivatives of basis functions
        H0_triple_prime = (
            -60 + 360 * t - 360 * t2
        )  # Third derivative of position at start point
        H1_triple_prime = (
            60 - 360 * t + 360 * t2
        )  # Third derivative of position at end point
        H2_triple_prime = (
            -36 + 192 * t - 180 * t2
        )  # Third derivative of first derivative at start point
        H3_triple_prime = (
            -24 + 168 * t - 180 * t2
        )  # Third derivative of first derivative at end point
        H4_triple_prime = (
            -9 + 36 * t - 30 * t2
        )  # Third derivative of second derivative at start point
        H5_triple_prime = (
            3 - 24 * t + 30 * t2
        )  # Third derivative of second derivative at end point

        return np.array(
            [
                H0_triple_prime,
                H1_triple_prime,
                H2_triple_prime,
                H3_triple_prime,
                H4_triple_prime,
                H5_triple_prime,
            ]
        )
    
    # def get_tangent(self, t: float) -> np.ndarray:

    def get_derivative(self, t: float, debug: bool = False) -> np.ndarray:
        """Enhanced get_derivative with optional logging"""
        if not self.segments:
            raise ValueError("Spline has not been fitted yet")

        local_t, segment_idx = self._normalize_parameter(t)

        basis_derivatives = self._get_basis_derivatives(local_t)

        derivative = np.zeros(2)
        for i in range(6):
            derivative += basis_derivatives[i] * self.segments[segment_idx][i]
        # logger.info(f"t: {t}, local t: {local_t}, derivative: {derivative}")
        # print(int(round(t)), "round")
        # logger.info(f"tangent: {self.segments[segment_idx][2]}, {self.segments[segment_idx][3]}")

        return derivative

    def get_second_derivative(self, t: float, debug: bool = False) -> np.ndarray:
        """Enhanced get_second_derivative with optional logging"""

        if not self.segments:
            raise ValueError("Spline has not been fitted yet")

        local_t, segment_idx = self._normalize_parameter(t)

        basis_second_derivatives = self._get_basis_second_derivatives(local_t)

        second_derivative = np.zeros(2)
        for i in range(6):
            second_derivative += (
                basis_second_derivatives[i] * self.segments[segment_idx][i]
            )

        return second_derivative

    def get_third_derivative(self, t: float, debug: bool = False) -> np.ndarray:
        """
        Get the third derivative of the spline at parameter t.

        Args:
            t: Parameter value normalized to the entire path length
            debug: Optional flag for debugging output

        Returns:
            np.ndarray: Third derivative vector [x''', y''']

        Raises:
            ValueError: If spline has not been fitted yet
        """
        if not self.segments:
            raise ValueError("Spline has not been fitted yet")

        local_t, segment_idx = self._normalize_parameter(t)

        basis_third_derivatives = self._get_basis_third_derivatives(local_t)

        third_derivative = np.zeros(2)
        for i in range(6):
            third_derivative += (
                basis_third_derivatives[i] * self.segments[segment_idx][i]
            )

        return third_derivative

    def _normalize_parameter(self, t: float) -> Tuple[float, int]:
        """
        Convert global parameter t to local parameter and segment index.

        Args:
            t: Global parameter value between self.parameters[0] and self.parameters[-1]

        Returns:
            Tuple[float, int]: Local parameter value (0 to 1) and segment index
        """
        if not self.parameters.size:
            raise ValueError("Spline has not been fitted yet")

        # Get parameter range
        t_min = self.parameters[0]
        t_max = self.parameters[-1]

        # Clamp parameter to valid range instead of raising error
        t = max(t_min, min(t, t_max))

        # Find which segment the parameter falls into
        segment_length = 1.0  # Since we normalize to [0,1] for each segment
        num_segments = len(self.segments)

        # Calculate segment index
        segment_idx = int((t - t_min) / segment_length)

        # Handle edge case where t == t_max
        if segment_idx == num_segments:
            segment_idx = num_segments - 1

        # Calculate local parameter (0 to 1) within the segment
        segment_start = t_min + segment_idx * segment_length
        local_t = (t - segment_start) / segment_length

        return local_t, segment_idx

    def set_starting_tangent(self, tangent: np.ndarray) -> bool:
        """
        Set the tangent (first derivative) at the start of the spline.

        Args:
            tangent: np.ndarray of shape (2,) representing the tangent vector [dx/dt, dy/dt]

        Returns:
            bool: True if setting the tangent was successful, False otherwise
        """
        # Validate input
        if not isinstance(tangent, np.ndarray) or tangent.shape != (2,):
            return False

        # Check if spline has been fitted
        # if self.first_derivatives is None or not len(self.segments):
        #     return False

        # Normalize the tangent vector and scale by the average segment length
        # norm = np.linalg.norm(tangent)
        # if norm > 0:
        #     distances = np.linalg.norm(np.diff(self.control_points, axis=0), axis=1)
        #     scale = np.mean(distances)
        #     tangent = (tangent / norm) * scale

        # Update the first derivative
        self.first_derivatives[0] = tangent

        # Update only the first segment since other segments are unaffected
        if len(self.segments) > 0:
            p0 = self.control_points[0]
            p1 = self.control_points[1]
            d0 = tangent  # New tangent
            d1 = self.first_derivatives[1]
            dd0 = self.second_derivatives[0]
            dd1 = self.second_derivatives[1]

            # Update the first segment matrix
            self.segments[0] = np.vstack([p0, p1, d0, d1, dd0, dd1])

        self.starting_tangent = tangent
        return True

    def set_ending_tangent(self, tangent: np.ndarray) -> bool:
        """
        Set the tangent (first derivative) at the end of the spline.

        Args:
            tangent: np.ndarray of shape (2,) representing the tangent vector [dx/dt, dy/dt]

        Returns:
            bool: True if setting the tangent was successful, False otherwise
        """
        # Validate input
        if not isinstance(tangent, np.ndarray) or tangent.shape != (2,):
            return False

        # Check if spline has been fitted
        # if self.first_derivatives is None or not len(self.segments):
        #     return False

        # Normalize the tangent vector and scale by the average segment length
        # norm = np.linalg.norm(tangent)
        # if norm > 0:
        #     distances = np.linalg.norm(np.diff(self.control_points, axis=0), axis=1)
        #     scale = np.mean(distances)
        #     tangent = (tangent / norm) * scale

        # Update the last first derivative
        self.first_derivatives[-1] = tangent

        # Update only the last segment since other segments are unaffected
        if len(self.segments) > 0:
            p0 = self.control_points[-2]  # Second-to-last point
            p1 = self.control_points[-1]  # Last point
            d0 = self.first_derivatives[-2]  # Second-to-last derivative
            d1 = tangent  # New ending tangent
            dd0 = self.second_derivatives[-2]  # Second-to-last second derivative
            dd1 = self.second_derivatives[-1]  # Last second derivative

            # Update the last segment matrix
            self.segments[-1] = np.vstack([p0, p1, d0, d1, dd0, dd1])

        self.ending_tangent = tangent

        return True

    def get_arc_length(
        self, t_start: float, t_end: float, num_points: int = 20
    ) -> float:
        """
        Calculate the arc length of the spline between two parameter values using
        Gaussian quadrature numerical integration.

        The arc length is computed by integrating the magnitude of the first derivative
        vector over the parameter range: ∫ |dP/dt| dt from t_start to t_end

        Args:
            t_start: Starting parameter value
            t_end: Ending parameter value
            num_points: Number of Gaussian quadrature points (default=20)

        Returns:
            float: Approximate arc length between the two parameter values

        Raises:
            ValueError: If t_start or t_end are outside the valid parameter range,
                    or if t_start >= t_end
        """
        if not self.segments:
            raise ValueError("Spline has not been fitted yet")

        if t_start >= t_end:
            raise ValueError("t_start must be less than t_end")

        # Validate parameter range
        t_min = self.parameters[0]
        t_max = self.parameters[-1]
        if t_start < t_min or t_end > t_max:
            raise ValueError(f"Parameters must be within range [{t_min}, {t_max}]")

        # Gauss-Legendre quadrature points and weights for the interval [-1, 1]
        # We'll use numpy's built-in function
        points, weights = np.polynomial.legendre.leggauss(num_points)

        # Transform Gaussian quadrature points from [-1, 1] to [t_start, t_end]
        half_length = (t_end - t_start) / 2
        midpoint = (t_start + t_end) / 2
        transformed_points = points * half_length + midpoint

        # Calculate the derivative magnitude at each quadrature point
        derivative_magnitudes = np.array(
            [np.linalg.norm(self.get_derivative(t)) for t in transformed_points]
        )

        # Compute the integral using the quadrature weights
        # The half_length factor is due to the change of variables formula
        arc_length = half_length * np.sum(weights * derivative_magnitudes)

        return float(arc_length)

    def get_total_arc_length(self) -> float:
        """
        Calculate the total arc length of the entire spline.

        Returns:
            float: Total arc length of the spline

        Raises:
            ValueError: If the spline has not been fitted yet
        """
        if not self.segments:
            raise ValueError("Spline has not been fitted yet")

        return self.get_arc_length(self.parameters[0], self.parameters[-1])

    def get_parameter_by_arc_length(
        self, arc_length: float, tolerance: float = 1e-6, max_iterations: int = 50
    ) -> float:
        """
        Find the parameter value t that corresponds to traveling a specific arc length
        along the spline from the start. Uses binary search.

        Args:
            arc_length: Desired arc length from the start of the spline
            tolerance: Acceptable error in arc length (default=1e-6)
            max_iterations: Maximum number of binary search iterations (default=50)

        Returns:
            float: Parameter value t that gives the desired arc length

        Raises:
            ValueError: If arc_length is negative or greater than the total arc length
        """
        if not self.segments:
            raise ValueError("Spline has not been fitted yet")

        if arc_length < 0:
            raise ValueError("Arc length must be non-negative")

        total_length = self.get_total_arc_length()
        if arc_length > total_length:
            raise ValueError(
                f"Arc length {arc_length} exceeds total length {total_length}"
            )

        # Handle edge cases
        if arc_length == 0:
            return self.parameters[0]
        if arc_length == total_length:
            return self.parameters[-1]

        # Binary search for the parameter value
        t_start = self.parameters[0]
        t_end = self.parameters[-1]
        t_min = t_start
        t_max = t_end

        for _ in range(max_iterations):
            t_mid = (t_min + t_max) / 2
            current_length = self.get_arc_length(t_start, t_mid)

            error = current_length - arc_length
            if abs(error) < tolerance:
                return t_mid

            if error > 0:
                t_max = t_mid
            else:
                t_min = t_mid

        # If we reach here, we've hit max iterations but should still have a good approximation
        return (t_min + t_max) / 2

    def _compute_parameters(self, points: np.ndarray) -> np.ndarray:
        """
        Compute parameter values using normalized chord-length parameterization.
        Points will be parameterized from 0 to n-1 where n is the number of points.
        """
        # Calculate distances between consecutive points
        diffs = np.diff(points, axis=0)
        segment_lengths = np.linalg.norm(diffs, axis=1)

        # Compute cumulative distances
        cumulative_lengths = np.concatenate(([0], np.cumsum(segment_lengths)))

        # Handle case where all points are identical
        if cumulative_lengths[-1] == 0:
            return np.linspace(0, len(points) - 1, len(points))

        # Normalize to range [0, n-1] where n is number of points
        return cumulative_lengths * (len(points) - 1) / cumulative_lengths[-1]

    def get_end_parameter(self) -> float:
        """
        Get the parameter value corresponding to the last control point.

        Returns:
            float: Parameter value corresponding to the last control point
        """
        if not self.parameters.size:
            raise ValueError("Spline has not been fitted yet")

        return self.parameters[-1]
