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
        
        Args:
            x: x-coordinates of control points
            y: y-coordinates of control points
            first_derivatives: First derivatives at control points (dx/dt, dy/dt)
            second_derivatives: Second derivatives at control points (d²x/dt², d²y/dt²)
            
        Returns:
            bool: True if fitting was successful, False otherwise
        """
        # Validate input arrays
        if len(x) != len(y):
            return False
        
        if len(x) < 2:  # Need at least 2 points to create a spline
            return False
        
        # Store control points
        self.control_points = np.column_stack((x, y))
        
        # Validate derivative arrays if provided
        if first_derivatives is not None:
            if len(first_derivatives) != len(x):
                return False
            self.first_derivatives = first_derivatives
        
        if second_derivatives is not None:
            if len(second_derivatives) != len(x):
                return False
            self.second_derivatives = second_derivatives
        
        # Compute parameter values (assuming uniform parameterization)
        num_points = len(x)
        self.parameters = np.linspace(0, num_points - 1, num_points)
        # If derivatives weren't provided, compute them
        if self.first_derivatives is None or self.second_derivatives is None:
            try:
                self._compute_derivatives()
            except Exception:
                return False
        
        # Initialize segment matrices for efficient evaluation
        self.segments = []
        for i in range(len(x) - 1):
            # Get the control points and derivatives for this segment
            p0 = self.control_points[i]
            p1 = self.control_points[i + 1]
            d0 = self.first_derivatives[i]
            d1 = self.first_derivatives[i + 1]
            dd0 = self.second_derivatives[i]
            dd1 = self.second_derivatives[i + 1]
            
            # Compute the coefficient matrix for this segment
            # [p0, p1, d0, d1, dd0, dd1]
            segment = np.vstack([p0, p1, d0, d1, dd0, dd1])
            self.segments.append(segment)
        
        if (self.starting_tangent is not None):
            self.set_starting_tangent(self.starting_tangent)
        if (self.ending_tangent is not None):
            self.set_ending_tangent(self.ending_tangent)

        return True
        
    def _compute_derivatives(self) -> None:
        
        """
        Compute missing first and second derivatives at control points if not provided.
        Uses finite difference approximations or other numerical methods.
        """
        num_points = len(self.control_points)
    
        # Initialize derivative arrays if they don't exist
        if self.first_derivatives is None:
            self.first_derivatives = np.zeros_like(self.control_points)
        if self.second_derivatives is None:
            self.second_derivatives = np.zeros_like(self.control_points)

        # Calculate scale factor based on average distance between points
        distances = np.linalg.norm(np.diff(self.control_points, axis=0), axis=1)
        scale_factor = np.mean(distances)
        
        # Use scale-aware step size
        h = self.parameters[1] - self.parameters[0]
        h = h * scale_factor  # Scale the step size

        # Initialize derivative arrays if they don't exist
        if self.first_derivatives is None:
            self.first_derivatives = np.zeros_like(self.control_points)
        if self.second_derivatives is None:
            self.second_derivatives = np.zeros_like(self.control_points)

        h = self.parameters[1] - self.parameters[0]

        if num_points == 2:
            # Linear case
            derivative = (self.control_points[1] - self.control_points[0]) / h
            self.first_derivatives[0] = derivative
            self.first_derivatives[1] = derivative
            
            # Zero second derivatives for linear case
            self.second_derivatives[0] = np.zeros_like(self.control_points[0])
            self.second_derivatives[1] = np.zeros_like(self.control_points[0])
            
        elif num_points == 3:
            # First derivatives
            # Forward difference for first point
            self.first_derivatives[0] = (
                self.control_points[1] - self.control_points[0]
            ) / h
            
            # Central difference for middle point
            self.first_derivatives[1] = (
                self.control_points[2] - self.control_points[0]
            ) / (2 * h)
            
            # Backward difference for last point
            self.first_derivatives[2] = (
                self.control_points[2] - self.control_points[1]
            ) / h
            
            # Second derivatives
            # Basic three-point formula for middle point
            middle_second_deriv = (
                self.control_points[0] - 
                2 * self.control_points[1] + 
                self.control_points[2]
            ) / (h * h)
            
            # Copy to endpoints
            self.second_derivatives[0] = middle_second_deriv
            self.second_derivatives[1] = middle_second_deriv
            self.second_derivatives[2] = middle_second_deriv
            
        else: # 4+ points          
            # Compute first derivatives
            # For interior points: central difference
            for i in range(1, num_points - 1):
                self.first_derivatives[i] = (
                    self.control_points[i + 1] - self.control_points[i - 1]
                ) / (2 * h)
            
            # For endpoints: one-sided differences
            # Forward difference for first point
            self.first_derivatives[0] = (
                -3 * self.control_points[0] + 
                4 * self.control_points[1] - 
                self.control_points[2]
            ) / (2 * h)
            
            # Backward difference for last point
            self.first_derivatives[-1] = (
                3 * self.control_points[-1] - 
                4 * self.control_points[-2] + 
                self.control_points[-3]
            ) / (2 * h)
            
            # Compute second derivatives
            # For interior points: central difference
            for i in range(1, num_points - 1):
                self.second_derivatives[i] = (
                    self.control_points[i - 1] - 
                    2 * self.control_points[i] + 
                    self.control_points[i + 1]
                ) / (h * h)
            
            # For endpoints: forward/backward differences
            # Forward difference for first point
            self.second_derivatives[0] = (
                2 * self.control_points[0] - 
                5 * self.control_points[1] + 
                4 * self.control_points[2] - 
                self.control_points[3]
            ) / (h * h)
            
            # Backward difference for last point
            self.second_derivatives[-1] = (
                2 * self.control_points[-1] - 
                5 * self.control_points[-2] + 
                4 * self.control_points[-3] - 
                self.control_points[-4]
            ) / (h * h)
        
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
        H0 = 1 - 10*t3 + 15*t4 - 6*t5    # Position at start point
        H1 = 10*t3 - 15*t4 + 6*t5        # Position at end point
        H2 = t - 6*t3 + 8*t4 - 3*t5      # First derivative at start point
        H3 = -4*t3 + 7*t4 - 3*t5         # First derivative at end point
        H4 = 0.5*t2 - 1.5*t3 + 1.5*t4 - 0.5*t5  # Second derivative at start point
        H5 = 0.5*t3 - t4 + 0.5*t5        # Second derivative at end point
        
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
        H0_prime = -30*t2 + 60*t3 - 30*t4    # Derivative of position at start point
        H1_prime = 30*t2 - 60*t3 + 30*t4     # Derivative of position at end point
        H2_prime = 1 - 18*t2 + 32*t3 - 15*t4 # Derivative of first derivative at start point
        H3_prime = -12*t2 + 28*t3 - 15*t4    # Derivative of first derivative at end point
        H4_prime = t - 4.5*t2 + 6*t3 - 2.5*t4  # Derivative of second derivative at start point
        H5_prime = 1.5*t2 - 4*t3 + 2.5*t4    # Derivative of second derivative at end point
        
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
        H0_double_prime = -60*t + 180*t2 - 120*t3    # Second derivative of position at start point
        H1_double_prime = 60*t - 180*t2 + 120*t3     # Second derivative of position at end point
        H2_double_prime = -36*t + 96*t2 - 60*t3      # Second derivative of first derivative at start point
        H3_double_prime = -24*t + 84*t2 - 60*t3      # Second derivative of first derivative at end point
        H4_double_prime = 1 - 9*t + 18*t2 - 10*t3    # Second derivative of second derivative at start point
        H5_double_prime = 3*t - 12*t2 + 10*t3        # Second derivative of second derivative at end point
        
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
            
        Raises:
            ValueError: If t is outside the valid parameter range
        """
        if not self.parameters.size:
            raise ValueError("Spline has not been fitted yet")
        # Check if parameter is within valid range
        t_min = self.parameters[0]
        t_max = self.parameters[-1]
        
        if t < t_min or t > t_max:
            raise ValueError(f"Parameter t={t} outside valid range [{t_min}, {t_max}]")
        
        # Find which segment the parameter falls into
        num_segments = len(self.segments)
        segment_length = (t_max - t_min) / num_segments
        
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
