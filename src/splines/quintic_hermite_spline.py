from typing import Optional, Tuple
from splines.spline import Spline  # type: ignore
import numpy as np

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
        
    def fit(self, x: np.ndarray, y: np.ndarray, 
            first_derivatives: Optional[np.ndarray] = None,
            second_derivatives: Optional[np.ndarray] = None) -> bool:
        """
        Fit the quintic Hermite spline to a set of points with optional derivative constraints.
        """
        # Validate input arrays
        if len(x) != len(y):
            return False
        
        if len(x) < 2:
            return False
        
        # Store control points
        self.control_points = np.column_stack((x, y))
        
        # Compute parameters using chord-length parameterization
        try:
            self.parameters = self._compute_parameters(self.control_points)
        except Exception as e:
            return False
        
        # print(f"Computed parameters: {self.parameters}")
        
        # Handle provided derivatives
        if first_derivatives is not None:
            if len(first_derivatives) != len(x):
                return False
            self.first_derivatives = first_derivatives
        
        if second_derivatives is not None:
            if len(second_derivatives) != len(x):
                return False
            self.second_derivatives = second_derivatives
        
        # Compute derivatives if not provided
        if self.first_derivatives is None or self.second_derivatives is None:
            try:
                self._compute_derivatives()
            except Exception as e:
                self.first_derivatives = None
                self.second_derivatives = None
                self.segments = []
                return False
        
        # Initialize segment matrices
        try:
            self.segments = []
            for i in range(len(x) - 1):
                # Get the control points and derivatives for this segment
                p0 = self.control_points[i]
                p1 = self.control_points[i + 1]
                d0 = self.first_derivatives[i]
                d1 = self.first_derivatives[i + 1]
                dd0 = self.second_derivatives[i]
                dd1 = self.second_derivatives[i + 1]
                
                # Scale derivatives by segment length
                segment_length = np.linalg.norm(p1 - p0)
                
                if segment_length > 0:
                    d0 = d0 * segment_length
                    d1 = d1 * segment_length
                    dd0 = dd0 * (segment_length)
                    dd1 = dd1 * (segment_length)
                
                # Compute the coefficient matrix for this segment
                segment = np.vstack([p0, p1, d0, d1, dd0, dd1])
                self.segments.append(segment)
                        
        except Exception as e:
            self.segments = []
            return False
        
        # Restore any previously set tangents
        if self.starting_tangent is not None:
            self.set_starting_tangent(self.starting_tangent)
        if self.ending_tangent is not None:
            self.set_ending_tangent(self.ending_tangent)

        return True
            
    def _compute_derivatives(self) -> None:
        """
        Compute missing first and second derivatives at control points using local segment lengths
        and improved curvature handling.
        """
        num_points = len(self.control_points)

        # Initialize derivative arrays
        if self.first_derivatives is None:
            self.first_derivatives = np.zeros_like(self.control_points, dtype=float)
        if self.second_derivatives is None:
            self.second_derivatives = np.zeros_like(self.control_points, dtype=float)

        # Calculate distances between consecutive points
        diffs = np.diff(self.control_points, axis=0)
        distances = np.linalg.norm(diffs, axis=1)
        
        # Compute unit tangent vectors for each segment
        tangents = np.zeros_like(diffs)
        for i in range(len(diffs)):
            if distances[i] > 1e-10:
                tangents[i] = diffs[i] / distances[i]
            else:
                if i > 0:
                    tangents[i] = tangents[i-1]
                elif i < len(diffs) - 1:
                    tangents[i] = diffs[i+1] / np.linalg.norm(diffs[i+1])
                else:
                    tangents[i] = np.zeros(2)

        # Compute first derivatives using weighted average of adjacent tangents
        for i in range(num_points):
            if i == 0:
                # First point: use forward tangent
                self.first_derivatives[i] = tangents[0] * distances[0] * 0.5
            elif i == num_points - 1:
                # Last point: use backward tangent
                self.first_derivatives[i] = tangents[-1] * distances[-1] * 0.5
            else:
                # Interior points: weighted average of adjacent tangents
                w1 = distances[i]
                w2 = distances[i-1]
                if w1 + w2 > 1e-10:
                    weighted_tangent = (w1 * tangents[i-1] + w2 * tangents[i]) / (w1 + w2)
                    local_scale = min(w1, w2) * 0.5
                    self.first_derivatives[i] = weighted_tangent * local_scale
                else:
                    self.first_derivatives[i] = np.zeros(2)

        # Compute second derivatives with improved curvature handling
        for i in range(num_points):
            # Get the first derivative magnitude for scaling
            d1_mag = np.linalg.norm(self.first_derivatives[i])
            
            if i == 0:
                # Start point: use forward difference for curvature
                if d1_mag > 1e-10:
                    tangent = self.first_derivatives[i] / d1_mag
                    normal = np.array([-tangent[1], tangent[0]])
                    
                    # Compute curvature using forward difference
                    next_tangent = self.first_derivatives[i+1] / np.linalg.norm(self.first_derivatives[i+1])
                    angle = np.arccos(np.clip(np.dot(tangent, next_tangent), -1.0, 1.0))
                    curvature = angle / distances[0]
                    
                    # Scale second derivative by first derivative magnitude
                    self.second_derivatives[i] = normal * (curvature * d1_mag)
                    
            elif i == num_points - 1:
                # End point: use backward difference for curvature
                if d1_mag > 1e-10:
                    tangent = self.first_derivatives[i] / d1_mag
                    normal = np.array([-tangent[1], tangent[0]])
                    
                    # Compute curvature using backward difference
                    prev_tangent = self.first_derivatives[i-1] / np.linalg.norm(self.first_derivatives[i-1])
                    angle = np.arccos(np.clip(np.dot(tangent, prev_tangent), -1.0, 1.0))
                    curvature = angle / distances[-1]
                    
                    # Scale second derivative by first derivative magnitude
                    self.second_derivatives[i] = normal * (curvature * d1_mag)
                    
            else:
                # Interior points: use central difference for curvature
                if d1_mag > 1e-10:
                    tangent = self.first_derivatives[i] / d1_mag
                    normal = np.array([-tangent[1], tangent[0]])
                    
                    # Compute curvature using central difference
                    prev_tangent = self.first_derivatives[i-1] / np.linalg.norm(self.first_derivatives[i-1])
                    next_tangent = self.first_derivatives[i+1] / np.linalg.norm(self.first_derivatives[i+1])
                    
                    # Use average of backward and forward angles
                    angle_prev = np.arccos(np.clip(np.dot(tangent, prev_tangent), -1.0, 1.0))
                    angle_next = np.arccos(np.clip(np.dot(tangent, next_tangent), -1.0, 1.0))
                    total_length = distances[i-1] + distances[i]
                    
                    if total_length > 1e-10:
                        # Weight the curvature by relative segment lengths
                        curvature = (angle_prev + angle_next) / total_length
                        
                        # Scale second derivative by first derivative magnitude
                        self.second_derivatives[i] = normal * (curvature * d1_mag)
                
    def _get_basis_functions(self, t: float) -> np.ndarray:
        """
        Compute the quintic Hermite basis functions at parameter t with improved boundary behavior.
        """
        t2 = t * t
        t3 = t2 * t
        t4 = t3 * t
        t5 = t4 * t
        
        # Modified basis functions with better boundary behavior
        H0 = 1 - 10*t3 + 15*t4 - 6*t5
        H1 = 10*t3 - 15*t4 + 6*t5
        H2 = t - 6*t3 + 8*t4 - 3*t5
        H3 = -4*t3 + 7*t4 - 3*t5
        H4 = 0.5*(t2 - 2*t3 + t4)  # Modified for better C2 continuity
        H5 = 0.5*(t3 - 2*t4 + t5)  # Modified for better C2 continuity
        
        return np.array([H0, H1, H2, H3, H4, H5])

    def _get_basis_derivatives(self, t: float) -> np.ndarray:
        """
        Compute derivatives of the modified basis functions.
        """
        t2 = t * t
        t3 = t2 * t
        t4 = t3 * t
        
        # Derivatives of modified basis functions
        H0_prime = -30*t2 + 60*t3 - 30*t4
        H1_prime = 30*t2 - 60*t3 + 30*t4
        H2_prime = 1 - 18*t2 + 32*t3 - 15*t4
        H3_prime = -12*t2 + 28*t3 - 15*t4
        H4_prime = t - 3*t2 + 2*t3  # Modified derivative
        H5_prime = 1.5*t2 - 4*t3 + 2.5*t4  # Modified derivative
        
        return np.array([H0_prime, H1_prime, H2_prime, H3_prime, H4_prime, H5_prime])

    def _get_basis_second_derivatives(self, t: float) -> np.ndarray:
        """
        Compute second derivatives of the modified basis functions.
        """
        t2 = t * t
        t3 = t2 * t
        
        # Second derivatives of modified basis functions
        H0_double_prime = -60*t + 180*t2 - 120*t3
        H1_double_prime = 60*t - 180*t2 + 120*t3
        H2_double_prime = -36*t + 96*t2 - 60*t3
        H3_double_prime = -24*t + 84*t2 - 60*t3
        H4_double_prime = 1 - 6*t + 6*t2  # Modified second derivative
        H5_double_prime = 3*t - 12*t2 + 7.5*t3  # Modified second derivative
        
        return np.array([H0_double_prime, H1_double_prime, H2_double_prime, 
                        H3_double_prime, H4_double_prime, H5_double_prime])

        
    def get_point(self, t: float) -> np.ndarray:
        """
        Get point on the quintic Hermite spline at parameter t.
        Uses the basis functions to interpolate between control points.
        
        Args:
            t: Parameter value between the first and last control point parameters
            
        Returns:
            np.ndarray: Point coordinates [x, y]
        """
        if not self.segments:
            raise ValueError("Spline has not been fitted yet")
            
        # Convert global parameter to local parameter and segment index
        local_t, segment_idx = self._normalize_parameter(t)
        
        # Get the basis functions at the local parameter value
        basis = self._get_basis_functions(local_t)
        
        # Get the control points and derivatives for this segment
        segment = self.segments[segment_idx]
        
        # Compute the point using the basis functions
        # The segment matrix contains [p0, p1, d0, d1, dd0, dd1]
        # Each row represents a 2D point or vector
        point = np.zeros(2)
        for i in range(6):
            point += basis[i] * segment[i]
            
        return point
        
    def get_derivative(self, t: float) -> np.ndarray:
        """
        Get first derivative of the quintic Hermite spline at parameter t.
        Uses the derivatives of basis functions.
        
        Args:
            t: Parameter value between the first and last control point parameters
            
        Returns:
            np.ndarray: First derivative [dx/dt, dy/dt]
        """
        if not self.segments:
            raise ValueError("Spline has not been fitted yet")
            
        # Convert global parameter to local parameter and segment index
        local_t, segment_idx = self._normalize_parameter(t)
        
        # Get the basis function derivatives at the local parameter value
        basis_derivatives = self._get_basis_derivatives(local_t)
        
        # Get the control points and derivatives for this segment
        segment = self.segments[segment_idx]
        
        # Compute the derivative using the basis function derivatives
        # The segment matrix contains [p0, p1, d0, d1, dd0, dd1]
        derivative = np.zeros(2)
        for i in range(6):
            derivative += basis_derivatives[i] * segment[i]
            
        return derivative
        
    def get_second_derivative(self, t: float) -> np.ndarray:
        """
        Get second derivative of the quintic Hermite spline at parameter t.
        Uses the second derivatives of basis functions.
        
        Args:
            t: Parameter value between the first and last control point parameters
            
        Returns:
            np.ndarray: Second derivative [d²x/dt², d²y/dt²]
        """
        if not self.segments:
            raise ValueError("Spline has not been fitted yet")
            
        # Convert global parameter to local parameter and segment index
        local_t, segment_idx = self._normalize_parameter(t)
        
        # Get the basis function second derivatives at the local parameter value
        basis_second_derivatives = self._get_basis_second_derivatives(local_t)
        
        # Get the control points and derivatives for this segment
        segment = self.segments[segment_idx]
        
        # Compute the second derivative using the basis function second derivatives
        # The segment matrix contains [p0, p1, d0, d1, dd0, dd1]
        second_derivative = np.zeros(2)
        for i in range(6):
            second_derivative += basis_second_derivatives[i] * segment[i]
            
        return second_derivative
        
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
        if self.first_derivatives is None or not len(self.segments):
            return False
            
        # Normalize the tangent vector and scale by the average segment length
        norm = np.linalg.norm(tangent)
        if norm > 0:
            distances = np.linalg.norm(np.diff(self.control_points, axis=0), axis=1)
            scale = np.mean(distances)
            tangent = (tangent / norm) * scale
        
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
        if self.first_derivatives is None or not len(self.segments):
            return False
        
        # Normalize the tangent vector and scale by the average segment length
        norm = np.linalg.norm(tangent)
        if norm > 0:
            distances = np.linalg.norm(np.diff(self.control_points, axis=0), axis=1)
            scale = np.mean(distances)
            tangent = (tangent / norm) * scale
        
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
    
    def get_arc_length(self, t_start: float, t_end: float, num_points: int = 20) -> float:
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
        derivative_magnitudes = np.array([
            np.linalg.norm(self.get_derivative(t)) 
            for t in transformed_points
        ])
        
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

    def get_parameter_by_arc_length(self, arc_length: float, 
                                tolerance: float = 1e-6, 
                                max_iterations: int = 50) -> float:
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
            raise ValueError(f"Arc length {arc_length} exceeds total length {total_length}")
            
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