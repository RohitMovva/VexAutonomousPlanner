from typing import Dict, Optional
from splines.spline import Spline # type: ignore
import numpy as np

class NaturalCubicSpline(Spline):
    """Natural cubic spline implementation"""
    
    def __init__(self):
        super().__init__()
        self.coefficients: Optional[Dict] = None
        self.path_points: Optional[np.ndarray] = None
        self.steps: int = 50  # Number of points to generate per segment
        self.t_points: Optional[np.ndarray] = None
        
    def compute_parameters(self, points: np.ndarray) -> np.ndarray:
        """
        Compute parameter values for spline fitting using cumulative chord length
        
        Args:
            points: Array of shape (n, 2) containing x,y coordinates
            
        Returns:
            np.ndarray: Parameter values normalized to [0, 1]
        """
        # Compute distances between consecutive points
        diffs = np.diff(points, axis=0)
        segment_lengths = np.sqrt(np.sum(diffs**2, axis=1))
        
        # Compute cumulative distances
        t = np.zeros(len(points))
        t[1:] = np.cumsum(segment_lengths)
        
        # Normalize to [0, 1]
        if t[-1] > 0:
            t = t / t[-1]
            
        print("Computed parameters:", t)
        return t
        
    def initialize_spline(self, points: np.ndarray) -> bool:
        """
        Initialize the spline with points
        
        Args:
            points: Array of shape (n, 2) containing x,y coordinates
            
        Returns:
            bool: True if initialization was successful
        """
        if len(points) < 2:
            print("Not enough points")
            return False
            
        try:
            # Compute parameter values
            self.t_points = self.compute_parameters(points)
            print("Parameter values:", self.t_points)
            
            # Compute spline coefficients
            coeffs_x = self.compute_spline_coefficients(self.t_points, points[:, 0])
            coeffs_y = self.compute_spline_coefficients(self.t_points, points[:, 1])
            
            if coeffs_x is None or coeffs_y is None:
                print("Failed to compute coefficients")
                return False
                
            self.coefficients = {
                'x': coeffs_x,
                'y': coeffs_y
            }
            return True
            
        except Exception as e:
            print(f"Error initializing spline: {e}")
            return False
            
    def _find_segment(self, t: float) -> int:
        """Find the spline segment containing parameter t"""
        if self.t_points is None:
            raise ValueError("Spline not initialized")
            
        # Ensure t is in [0, 1] range
        t = np.clip(t, 0, 1)
        
        if t <= self.t_points[0]:
            return 0
        if t >= self.t_points[-1]:
            return len(self.t_points) - 2
            
        # Find segment index
        return np.searchsorted(self.t_points, t) - 1
        
    def get_point(self, t: float) -> np.ndarray:
        """
        Get point on spline at parameter t
        
        Args:
            t: Parameter value between 0 and 1
            
        Returns:
            np.ndarray: Point coordinates
        """
        if self.t_points is None or self.coefficients is None:
            raise ValueError("Spline not initialized")
            
        i = self._find_segment(t)
        x = self.evaluate_spline_segment(t, self.coefficients['x'], i, self.t_points[i])
        y = self.evaluate_spline_segment(t, self.coefficients['y'], i, self.t_points[i])
        return np.array([x, y])
        
    def build_path(self, points: np.ndarray, steps: Optional[int] = None) -> np.ndarray:
        """
        Build a path through the given points using natural cubic splines
        
        Args:
            points: Array of shape (n, 2) containing x,y coordinates
            steps: Number of points to generate per segment. If None, uses self.steps
            
        Returns:
            np.ndarray: Array of shape (m, 2) containing interpolated path points
        """
        print("\nStarting build_path")
        print("Input points shape:", points.shape)
        
        if steps is not None:
            self.steps = steps
            
        # Initialize the spline
        if not self.initialize_spline(points):
            return points
            
        # Generate points segment by segment
        path_points = []
        n_segments = len(points) - 1
        
        for i in range(n_segments):
            # Generate parameter values for this segment
            t_start = self.t_points[i]
            t_end = self.t_points[i + 1]
            t_segment = np.linspace(t_start, t_end, self.steps)
            
            # Generate points for this segment
            for t in t_segment:
                try:
                    point = self.get_point(t)
                    # Add basic sanity check for point values
                    if np.all(np.isfinite(point)) and np.all(np.abs(point) < 1e6):
                        path_points.append(point)
                    else:
                        print(f"Warning: Invalid point generated at t={t}: {point}")
                except Exception as e:
                    print(f"Error generating point at t={t}: {e}")
                    continue
        
        self.path_points = np.array(path_points)
    
        # Add basic validation of the generated path
        if len(self.path_points) > 0:
            print("\nPath statistics:")
            print("Min values:", np.min(self.path_points, axis=0))
            print("Max values:", np.max(self.path_points, axis=0))
            print("Number of points:", len(self.path_points))
            
        return self.path_points

    def compute_spline_coefficients(self, t: np.ndarray, y: np.ndarray) -> Optional[Dict]:
        """Compute natural cubic spline coefficients for a set of points"""
        n = len(t)
        if n < 3:
            return None
            
        # Build the tridiagonal system for the second derivatives
        h = np.diff(t)  # Intervals between parameter values
        
        # Build the tridiagonal matrix A
        A = np.zeros((n, n))
        r = np.zeros(n)
        
        # Interior points
        for i in range(1, n-1):
            A[i, i-1] = h[i-1]
            A[i, i] = 2 * (h[i-1] + h[i])
            A[i, i+1] = h[i]
            
            r[i] = 3 * ((y[i+1] - y[i]) / h[i] - (y[i] - y[i-1]) / h[i-1])
        
        # Boundary conditions for natural spline (second derivatives = 0 at endpoints)
        A[0, 0] = 1
        A[-1, -1] = 1
        
        # Debug output
        print("Matrix A:")
        print(A)
        print("\nVector r:")
        print(r)
        
        # Solve for second derivatives
        try:
            m = np.linalg.solve(A, r)
            print("\nSecond derivatives (m):")
            print(m)
        except np.linalg.LinAlgError:
            print("Failed to solve linear system")
            return None
        
        # Calculate coefficients for each segment
        coeffs = {'a': np.zeros(n-1), 'b': np.zeros(n-1), 
                 'c': np.zeros(n-1), 'd': np.zeros(n-1)}
                 
        for i in range(n-1):
            coeffs['a'][i] = y[i]
            coeffs['b'][i] = (y[i+1] - y[i]) / h[i] - h[i] * (2 * m[i] + m[i+1]) / 3
            coeffs['c'][i] = m[i]
            coeffs['d'][i] = (m[i+1] - m[i]) / (3 * h[i])
        
        # Debug output
        print("\nSpline coefficients:")
        for k, v in coeffs.items():
            print(f"{k}: {v}")
            
        return coeffs
            
    def get_derivative(self, t: float) -> np.ndarray:
        """
        Get first derivative at parameter t
        
        Args:
            t: Parameter value
            
        Returns:
            np.ndarray: First derivative
        """
        i = self._find_segment(t)
        t_norm = t - self.t_points[i]
        
        def evaluate_derivative(coeffs):
            return (coeffs['b'][i] + 
                   2 * coeffs['c'][i] * t_norm + 
                   3 * coeffs['d'][i] * t_norm**2)
        
        dx = evaluate_derivative(self.coefficients['x'])
        dy = evaluate_derivative(self.coefficients['y'])
        return np.array([dx, dy])
            
    def get_second_derivative(self, t: float) -> np.ndarray:
        """
        Get second derivative at parameter t
        
        Args:
            t: Parameter value
            
        Returns:
            np.ndarray: Second derivative
        """
        i = self._find_segment(t)
        t_norm = t - self.t_points[i]
        
        def evaluate_second_derivative(coeffs):
            return (2 * coeffs['c'][i] + 
                   6 * coeffs['d'][i] * t_norm)
        
        ddx = evaluate_second_derivative(self.coefficients['x'])
        ddy = evaluate_second_derivative(self.coefficients['y'])
        return np.array([ddx, ddy])
    
    def fit(self, x: np.ndarray, y: np.ndarray) -> bool:
        """
        Fit natural cubic spline to points
        
        Args:
            x: x-coordinates or parameter values
            y: y-coordinates
            
        Returns:
            bool: True if fitting was successful, False otherwise
        """
        if len(x) != len(y) or len(x) < 3:
            return False
            
        # Store parameter values
        self.t_points = x
        
        # Handle 2D points
        if y.ndim == 2:
            coeffs_x = self.compute_spline_coefficients(x, y[:, 0])
            coeffs_y = self.compute_spline_coefficients(x, y[:, 1])
            if coeffs_x is None or coeffs_y is None:
                return False
            self.coefficients = {
                'x': coeffs_x,
                'y': coeffs_y
            }
        else:
            coeffs = self.compute_spline_coefficients(x, y)
            if coeffs is None:
                return False
            self.coefficients = {'x': coeffs}
            
        return True
        
    def evaluate_spline_segment(self, t: float, coeffs: dict, i: int, x0: float) -> float:
        """
        Evaluate the spline segment i at parameter t with robust normalization
        """
        # Get the parameter interval for this segment
        h = self.t_points[i+1] - self.t_points[i]
        
        # Ensure t is within the segment bounds
        t = np.clip(t, self.t_points[i], self.t_points[i+1])
        
        # Normalize t to [0, h] for this segment
        t_norm = t - self.t_points[i]
        
        # Compute the polynomial terms
        term1 = coeffs['a'][i]
        term2 = coeffs['b'][i] * t_norm
        term3 = coeffs['c'][i] * (t_norm ** 2)
        term4 = coeffs['d'][i] * (t_norm ** 3)
        
        # Debug output for suspicious values
        if not np.isfinite(term1 + term2 + term3 + term4):
            print(f"Warning: Non-finite value in segment {i} at t={t}")
            print(f"Terms: {term1}, {term2}, {term3}, {term4}")
            print(f"t_norm: {t_norm}, h: {h}")
            
        return term1 + term2 + term3 + term4
    
    def get_path_points(self) -> Optional[np.ndarray]:
        """
        Get the most recently generated path points
        
        Returns:
            Optional[np.ndarray]: Array of path points or None if no path has been built
        """
        return self.path_points