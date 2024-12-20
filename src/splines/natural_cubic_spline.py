from typing import Dict, Optional
from splines.spline import Spline
import numpy as np


class NaturalCubicSpline(Spline):
    """Natural cubic spline implementation"""
    
    def __init__(self):
        super().__init__()
        self.coefficients: Optional[Dict] = None
        self.path_points: Optional[np.ndarray] = None
        self.steps: int = 50  # Number of points to generate per segment
        self.t_points: Optional[np.ndarray] = None  # Parameter values
        
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
            
        # Generate evenly spaced points
        total_segments = len(points) - 1
        points_per_segment = self.steps
        total_points = points_per_segment * total_segments + 1
        
        print(f"\nGenerating {total_points} points")
        # Generate parameter values
        t_values = np.linspace(0, 1, total_points)
        print("t_values range:", t_values[0], "to", t_values[-1])
        print("t_points range:", self.t_points[0], "to", self.t_points[-1])
        
        # Generate points
        try:
            self.path_points = np.array([self.get_point(t) for t in t_values])
            print("Successfully generated path points")
            return self.path_points
            
        except Exception as e:
            print(f"Error generating path points: {e}")
            import traceback
            traceback.print_exc()
            return points

    def build_path(self, points: np.ndarray, steps: Optional[int] = None) -> np.ndarray:
        """
        Build a path through the given points using natural cubic splines
        
        Args:
            points: Array of shape (n, 2) containing x,y coordinates
            steps: Number of points to generate per segment. If None, uses self.steps
            
        Returns:
            np.ndarray: Array of shape (m, 2) containing interpolated path points
        """
        print("Input points:", points)
        
        if len(points) < 2:
            return points
            
        if steps is not None:
            self.steps = steps
            
        # Compute parameter values based on cumulative chord length
        self.t_points = self.compute_parameters(points)
        
        # Verify t_points is set correctly
        print("Parameter values:", self.t_points)
        
        # Extract x and y coordinates
        x = points[:, 0]
        y = points[:, 1]
        
        # Compute splines for x and y coordinates
        coeffs_x = self.compute_spline_coefficients(self.t_points, x)
        coeffs_y = self.compute_spline_coefficients(self.t_points, y)
        
        if coeffs_x is None or coeffs_y is None:
            print("Failed to compute spline coefficients")
            return points
            
        # Store coefficients
        self.coefficients = {
            'x': coeffs_x,
            'y': coeffs_y
        }
        
        # Generate evenly spaced points
        total_segments = len(points) - 1
        points_per_segment = self.steps
        total_points = points_per_segment * total_segments + 1
        
        # Generate parameter values
        t_values = np.linspace(0, 1, total_points)
        
        # Generate points
        try:
            path_points = []
            for t in t_values:
                point = self.get_point(t)
                path_points.append(point)
            self.path_points = np.array(path_points)
        except Exception as e:
            print(f"Error generating path points: {e}")
            return points
            
        return self.path_points

    def _find_segment(self, t: float) -> int:
        """Find the spline segment containing parameter t"""
        if self.t_points is None:
            raise ValueError("t_points is not initialized")
            
        # Ensure t is in [0, 1]
        t = np.clip(t, 0, 1)
        
        # Handle edge cases
        if t <= self.t_points[0]:
            return 0
        if t >= self.t_points[-1]:
            return len(self.t_points) - 2
            
        # Find the appropriate segment
        for i in range(len(self.t_points) - 1):
            if self.t_points[i] <= t <= self.t_points[i + 1]:
                return i
                
        # Fallback (shouldn't reach here due to earlier checks)
        return len(self.t_points) - 2
        
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
        
    def _find_segment(self, t: float) -> int:
        """Find the spline segment containing parameter t"""
        if t <= self.t_points[0]:
            return 0
        if t >= self.t_points[-1]:
            return len(self.t_points) - 2
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
            raise ValueError("Spline has not been initialized. Call build_path first.")
            
        i = self._find_segment(t)
        x = self.evaluate_spline_segment(t, self.coefficients['x'], i, self.t_points[i])
        y = self.evaluate_spline_segment(t, self.coefficients['y'], i, self.t_points[i])
        return np.array([x, y])
            
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
        
    def build_path(self, points: np.ndarray, steps: Optional[int] = None) -> np.ndarray:
        """
        Build a path through the given points using natural cubic splines
        
        Args:
            points: Array of shape (n, 2) containing x,y coordinates
            steps: Number of points to generate per segment. If None, uses self.steps
            
        Returns:
            np.ndarray: Array of shape (m, 2) containing interpolated path points
        """
        if len(points) < 2:
            return points
            
        if steps is not None:
            self.steps = steps
            
        # Extract x and y coordinates
        x = points[:, 0]
        y = points[:, 1]
        
        # Compute parameter values based on cumulative chord length
        self.t_points = self.compute_parameters(points)
        
        # Compute splines for x and y coordinates
        coeffs_x = self.compute_spline_coefficients(self.t_points, x)
        coeffs_y = self.compute_spline_coefficients(self.t_points, y)
        
        if coeffs_x is None or coeffs_y is None:
            # Fall back to linear interpolation for very short paths
            return points
            
        # Store coefficients
        self.coefficients = {
            'x': coeffs_x,
            'y': coeffs_y
        }
        
        # Generate evenly spaced points
        total_segments = len(points) - 1
        points_per_segment = self.steps
        total_points = points_per_segment * total_segments + 1
        
        # Use normalized parameter range
        t_values = np.linspace(0, 1, total_points)
        self.path_points = np.array([self.get_point(t) for t in t_values])
        
        return self.path_points
        
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
        
    def compute_spline_coefficients(self, x: np.ndarray, y: np.ndarray) -> Optional[Dict]:
        """Compute natural cubic spline coefficients for a set of points"""
        n = len(x)
        if n < 3:
            return None
            
        # Build the tridiagonal system for the second derivatives
        h = np.diff(x)  # Intervals between x points
        
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
        
    def evaluate_spline_segment(self, t: np.ndarray, coeffs: dict, i: int, x0: float) -> np.ndarray:
        """Evaluate the spline segment i at points t"""
        t_norm = t - x0
        return (coeffs['a'][i] + 
                coeffs['b'][i] * t_norm + 
                coeffs['c'][i] * t_norm**2 + 
                coeffs['d'][i] * t_norm**3)
                
    def _find_segment(self, t: float) -> int:
        """Find the spline segment containing parameter t"""
        if t <= self.t_points[0]:
            return 0
        if t >= self.t_points[-1]:
            return len(self.t_points) - 2
        return np.searchsorted(self.t_points, t) - 1
        
    def get_point(self, t: float) -> np.ndarray:
        """
        Get point on spline at parameter t
        
        Args:
            t: Parameter value
            
        Returns:
            np.ndarray: Point coordinates
        """
        i = self._find_segment(t)
        
        if 'y' in self.coefficients:
            # 2D point
            x = self.evaluate_spline_segment(t, self.coefficients['x'], i, self.x_points[i])
            y = self.evaluate_spline_segment(t, self.coefficients['y'], i, self.x_points[i])
            return np.array([x, y])
        else:
            # 1D point
            return self.evaluate_spline_segment(t, self.coefficients['x'], i, self.x_points[i])
            
    def get_derivative(self, t: float) -> np.ndarray:
        """
        Get first derivative at parameter t
        
        Args:
            t: Parameter value
            
        Returns:
            np.ndarray: First derivative
        """
        i = self._find_segment(t)
        t_norm = t - self.x_points[i]
        
        def evaluate_derivative(coeffs):
            return (coeffs['b'][i] + 
                   2 * coeffs['c'][i] * t_norm + 
                   3 * coeffs['d'][i] * t_norm**2)
        
        if 'y' in self.coefficients:
            dx = evaluate_derivative(self.coefficients['x'])
            dy = evaluate_derivative(self.coefficients['y'])
            return np.array([dx, dy])
        else:
            return evaluate_derivative(self.coefficients['x'])
            
    def get_second_derivative(self, t: float) -> np.ndarray:
        """
        Get second derivative at parameter t
        
        Args:
            t: Parameter value
            
        Returns:
            np.ndarray: Second derivative
        """
        i = self._find_segment(t)
        t_norm = t - self.x_points[i]
        
        def evaluate_second_derivative(coeffs):
            return (2 * coeffs['c'][i] + 
                   6 * coeffs['d'][i] * t_norm)
        
        if 'y' in self.coefficients:
            ddx = evaluate_second_derivative(self.coefficients['x'])
            ddy = evaluate_second_derivative(self.coefficients['y'])
            return np.array([ddx, ddy])
        else:
            return evaluate_second_derivative(self.coefficients['x'])
            
    def compute_parameters(self, points: np.ndarray) -> np.ndarray:
        """
        Compute parameter values for spline fitting using cumulative chord length
        
        Args:
            points: Array of shape (n, 2) containing x,y coordinates
            
        Returns:
            np.ndarray: Parameter values
        """
        # Compute distances between consecutive points
        diffs = np.diff(points, axis=0)
        segment_lengths = np.sqrt(np.sum(diffs**2, axis=1))
        
        # Compute cumulative distances
        t = np.zeros(len(points))
        t[1:] = np.cumsum(segment_lengths)
        
        return t
        
    def build_path(self, points: np.ndarray, steps: Optional[int] = None) -> np.ndarray:
        """
        Build a path through the given points using natural cubic splines
        
        Args:
            points: Array of shape (n, 2) containing x,y coordinates
            steps: Number of points to generate per segment. If None, uses self.steps
            
        Returns:
            np.ndarray: Array of shape (m, 2) containing interpolated path points
        """
        if len(points) < 2:
            return points
            
        if steps is not None:
            self.steps = steps
            
        # Compute parameter values based on cumulative chord length
        t = self.compute_parameters(points)
        
        # Extract x and y coordinates
        x = points[:, 0]
        y = points[:, 1]
        
        # Fit the spline
        coeffs_x = self.compute_spline_coefficients(t, x)
        coeffs_y = self.compute_spline_coefficients(t, y)
        
        if coeffs_x is None or coeffs_y is None:
            # Fall back to linear interpolation for very short paths
            return points
            
        self.coefficients = {
            'x': coeffs_x,
            'y': coeffs_y
        }
        
        # Calculate total number of points needed
        total_segments = len(points) - 1
        points_per_segment = self.steps
        total_points = points_per_segment * total_segments + 1
        
        # Generate evenly spaced parameter values
        t_values = np.linspace(t[0], t[-1], total_points)
        
        # Get points along the entire spline
        self.path_points = np.array([self.get_point(ti) for ti in t_values])
        
        return self.path_points
        
    def get_path_points(self) -> Optional[np.ndarray]:
        """
        Get the most recently generated path points
        
        Returns:
            Optional[np.ndarray]: Array of path points or None if no path has been built
        """
        return self.path_points