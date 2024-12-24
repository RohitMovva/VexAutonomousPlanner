from typing import Dict, Optional, Tuple
from splines.spline import Spline  # type: ignore
import numpy as np
import inspect

class G2HermiteSpline(Spline):
    """G2 continuous Hermite spline implementation"""
    
    def __init__(self):
        super().__init__()
        self.coefficients: Optional[Dict] = None
        self.path_points: Optional[np.ndarray] = None
        self.steps: int = 50  # Number of points to generate per segment
        self.t_points: Optional[np.ndarray] = None
        self.tangents: Optional[np.ndarray] = None
        self.second_derivatives: Optional[np.ndarray] = None
        
    def compute_parameters(self, points: np.ndarray) -> np.ndarray:
        """
        Compute parameter values with improved spacing for G2 continuity
        """
        diffs = np.diff(points, axis=0)
        segment_lengths = np.sqrt(np.sum(diffs**2, axis=1))
        
        # Use centripetal parameterization (sqrt of chord length)
        # This generally gives better shape preservation than uniform or chord-length
        sqrt_lengths = np.sqrt(segment_lengths)
        t = np.zeros(len(points))
        t[1:] = np.cumsum(sqrt_lengths)
        
        # Normalize to [0, 1] with endpoint handling
        if t[-1] > 0:
            t = t / t[-1]
            
            # Adjust spacing near endpoints for better control
            alpha = 0.1  # Controls end-point spacing
            t = t ** alpha * (1 - (1 - t) ** alpha)
            
        return t
        
    def estimate_tangents_and_derivatives(self, points: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Estimate tangent vectors and second derivatives for G2 continuity with proper scaling
        """
        n = len(points)
        tangents = np.zeros_like(points, dtype=np.float64)
        second_derivatives = np.zeros_like(points, dtype=np.float64)
        
        # Compute segment lengths for scaling
        diffs = np.diff(points, axis=0)
        segment_lengths = np.sqrt(np.sum(diffs**2, axis=1))
        avg_length = np.mean(segment_lengths)
        
        # Compute scaled parameter spacing
        h = 1.0 / (n - 1)  # Base parameter spacing
        
        # Compute properly scaled second derivatives at interior points
        for i in range(1, n-1):
            # Use length-weighted finite differences
            scale = (avg_length / max(segment_lengths[i-1], segment_lengths[i])) ** 2
            second_derivatives[i] = scale * (points[i-1] - 2*points[i] + points[i+1]) / (h * h)
        
        # Handle endpoint second derivatives with proper decay
        second_derivatives[0] = second_derivatives[1] * 0.8  # Slightly reduce magnitude at ends
        second_derivatives[-1] = second_derivatives[-2] * 0.8
        
        # Compute tangents that ensure G2 continuity
        for i in range(1, n-1):
            prev_diff = points[i] - points[i-1]
            next_diff = points[i+1] - points[i]
            
            # Scale by segment lengths for better shape preservation
            prev_len = segment_lengths[i-1]
            next_len = segment_lengths[i]
            scale = avg_length / max(prev_len, next_len)
            
            # Combine first and second derivatives with proper scaling
            # The 1/6 factor helps balance the influence of second derivatives
            tangents[i] = scale * ((prev_diff + next_diff)/2 + (h * h * second_derivatives[i])/6)
        
        # Handle endpoints with G2 consideration
        tangents[0] = scale * (points[1] - points[0] + (h * h * second_derivatives[0])/6)
        tangents[-1] = scale * (points[-1] - points[-2] + (h * h * second_derivatives[-1])/6)
        
        return tangents, second_derivatives
        
    def initialize_spline(self, points: np.ndarray, nodes, tangents: Optional[np.ndarray] = None) -> bool:
        print("\nG2HermiteSpline initialization:")
        print(f"Class name: {self.__class__.__name__}")
        print(f"Method being called: {inspect.currentframe().f_code.co_name}")
        print(f"Has estimate_tangents_and_derivatives: {hasattr(self, 'estimate_tangents_and_derivatives')}")
        print(f"Has compute_parameters: {hasattr(self, 'compute_parameters')}")
        if len(points) < 2:
            print("Not enough points")
            return False

        try:
            self.t_points = self.compute_parameters(points)
            
            # Estimate both tangents and second derivatives for G2 continuity
            if tangents is None:
                self.tangents, self.second_derivatives = self.estimate_tangents_and_derivatives(points)
            else:
                self.tangents = tangents
                _, self.second_derivatives = self.estimate_tangents_and_derivatives(points)
                
            self.coefficients = self.compute_g2_coefficients(
                points, self.tangents, self.second_derivatives, self.t_points)
            
            return True

        except Exception as e:
            print(f"Error initializing spline: {e}")
            return False
            
    def compute_g2_coefficients(self, points: np.ndarray, 
                                tangents: np.ndarray,
                                second_derivatives: np.ndarray, 
                                t: np.ndarray) -> Dict:
        """
        Compute G2 continuous Hermite spline coefficients with proper parameter scaling
        """
        n = len(points) - 1
        
        # Extended basis matrix for G2 continuity
        M_g2 = np.array([
            [ 1,  0,  0,  0,  0,  0],
            [ 0,  1,  0,  0,  0,  0],
            [ 0,  0,  1/2, 0,  0,  0],
            [-10, 10, -6, -4,  1/2, 0],
            [ 15, -15, 8,  7, -1, -1/2],
            [-6,  6, -3, -3,  1/2, 1/2]
        ])
        
        coeffs_x = []
        coeffs_y = []
        
        for i in range(n):
            # Compute proper parameter scaling
            dt = t[i+1] - t[i]
            dt2 = dt * dt
            
            # Scale tangents and second derivatives appropriately
            scaled_tangent_i = tangents[i] * dt
            scaled_tangent_ip1 = tangents[i+1] * dt
            scaled_second_i = second_derivatives[i] * dt2
            scaled_second_ip1 = second_derivatives[i+1] * dt2
            
            # Create geometry matrices with proper scaling
            Gx = np.array([
                points[i][0],
                points[i+1][0],
                scaled_tangent_i[0],
                scaled_tangent_ip1[0],
                scaled_second_i[0],
                scaled_second_ip1[0]
            ])
            
            Gy = np.array([
                points[i][1],
                points[i+1][1],
                scaled_tangent_i[1],
                scaled_tangent_ip1[1],
                scaled_second_i[1],
                scaled_second_ip1[1]
            ])
            
            # Compute coefficients
            cx = M_g2 @ Gx
            cy = M_g2 @ Gy
            
            coeffs_x.append(cx)
            coeffs_y.append(cy)
        
        return {
            'x': np.array(coeffs_x),
            'y': np.array(coeffs_y)
        }


        
    def _find_segment(self, t: float) -> int:
        """Find the spline segment containing parameter t"""
        if self.t_points is None:
            raise ValueError("Spline not initialized")
            
        t = np.clip(t, 0, 1)
        
        if t <= self.t_points[0]:
            return 0
        if t >= self.t_points[-1]:
            return len(self.t_points) - 2
            
        return np.searchsorted(self.t_points, t) - 1
        
    def get_point(self, t: float) -> np.ndarray:
        """
        Get point on spline at parameter t with improved numerical stability
        """
        if self.t_points is None or self.coefficients is None:
            raise ValueError("Spline not initialized")
            
        i = self._find_segment(t)
        
        # Normalize t to [0, 1] for this segment
        t_local = (t - self.t_points[i]) / (self.t_points[i+1] - self.t_points[i])
        t_local = np.clip(t_local, 0, 1)
        
        # Compute powers of t efficiently and with better numerical stability
        t2 = t_local * t_local
        t3 = t2 * t_local
        t4 = t3 * t_local
        t5 = t4 * t_local
        
        # Extended basis vector for G2 continuity
        T = np.array([1, t_local, t2, t3, t4, t5])
        
        # Compute point coordinates using coefficient matrices
        x = T @ self.coefficients['x'][i]
        y = T @ self.coefficients['y'][i]
        
        return np.array([x, y])
        
    def get_derivative(self, t: float) -> np.ndarray:
        """
        Get first derivative at parameter t
        """
        if self.t_points is None or self.coefficients is None:
            raise ValueError("Spline not initialized")
            
        i = self._find_segment(t)
        dt = self.t_points[i+1] - self.t_points[i]
        
        # Normalize t to [0, 1] for this segment
        t_local = (t - self.t_points[i]) / dt
        t_local = np.clip(t_local, 0, 1)
        
        # Derivative of the extended basis
        T = np.array([0, 1, 2*t_local, 3*t_local**2, 4*t_local**3, 5*t_local**4])
        
        # Compute derivatives using coefficient matrices
        dx = T @ self.coefficients['x'][i] / dt
        dy = T @ self.coefficients['y'][i] / dt
        
        return np.array([dx, dy])
        
    def get_second_derivative(self, t: float) -> np.ndarray:
        """
        Get second derivative at parameter t
        """
        if self.t_points is None or self.coefficients is None:
            raise ValueError("Spline not initialized")
            
        i = self._find_segment(t)
        dt = self.t_points[i+1] - self.t_points[i]
        dt2 = dt * dt
        
        # Normalize t to [0, 1] for this segment
        t_local = (t - self.t_points[i]) / dt
        t_local = np.clip(t_local, 0, 1)
        
        # Second derivative of the extended basis
        T = np.array([0, 0, 2, 6*t_local, 12*t_local**2, 20*t_local**3])
        
        # Compute second derivatives using coefficient matrices
        d2x = T @ self.coefficients['x'][i] / dt2
        d2y = T @ self.coefficients['y'][i] / dt2
        
        return np.array([d2x, d2y])
        
    def build_path(self, points: np.ndarray, 
                   nodes = None,
                   tangents: Optional[np.ndarray] = None,
                   steps: Optional[int] = None) -> np.ndarray:
        """
        Build a path through the given points using G2 continuous Hermite splines
        """
        if steps is not None:
            self.steps = steps
            
        # Initialize the spline
        if not self.initialize_spline(points, nodes, tangents):
            return points
            
        # Generate points segment by segment
        path_points = []
        n_segments = len(points) - 1
        
        for i in range(n_segments):
            t_start = self.t_points[i]
            t_end = self.t_points[i + 1]
            t_segment = np.linspace(t_start, t_end, self.steps)
            
            for t in t_segment:
                try:
                    point = self.get_point(t)
                    if np.all(np.isfinite(point)) and np.all(np.abs(point) < 1e6):
                        path_points.append(point)
                    else:
                        print(f"Warning: Invalid point generated at t={t}: {point}")
                except Exception as e:
                    print(f"Error generating point at t={t}: {e}")
                    continue
                    
        self.path_points = np.array(path_points)
        return self.path_points

    def update_end_tangent(self, new_tangent: np.ndarray) -> bool:
        """
        Update the end tangent while maintaining G2 continuity
        """
        if (self.coefficients is None or self.t_points is None or 
            self.tangents is None or self.second_derivatives is None):
            print("Cannot update: spline not initialized")
            return False
            
        try:
            # Update the last tangent
            self.tangents[-1] = new_tangent
            
            # Recompute second derivatives to maintain G2 continuity
            last_idx = len(self.t_points) - 1
            if last_idx > 1:
                dt = self.t_points[-1] - self.t_points[-2]
                self.second_derivatives[-1] = (new_tangent - self.tangents[-2]) / dt
            
            # Recompute coefficients for the last segment
            points = np.array([self.get_point(t) for t in self.t_points])
            last_coeffs = self.compute_g2_coefficients(
                points[-2:], 
                self.tangents[-2:],
                self.second_derivatives[-2:],
                self.t_points[-2:]
            )
            
            # Update the last segment's coefficients
            self.coefficients['x'][-1] = last_coeffs['x'][0]
            self.coefficients['y'][-1] = last_coeffs['y'][0]
            
            return True
            
        except Exception as e:
            print(f"Error updating end tangent: {e}")
            return False
            
    def get_path_points(self) -> Optional[np.ndarray]:
        """Get the most recently generated path points"""
        return self.path_points