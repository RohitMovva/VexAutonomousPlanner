from typing import Dict, Optional, Tuple
from splines.spline import Spline  # type: ignore
import numpy as np

class CubicHermiteSpline(Spline):
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
        Compute parameter values for spline fitting using cumulative chord length
        """
        diffs = np.diff(points, axis=0)
        segment_lengths = np.sqrt(np.sum(diffs**2, axis=1))
        t = np.zeros(len(points))
        t[1:] = np.cumsum(segment_lengths)
        
        # Normalize to [0, 1]
        if t[-1] > 0:
            t = t / t[-1]
            
        return t
        
    def estimate_tangents_and_derivatives(self, points: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Estimate tangent vectors and second derivatives for G2 continuity
        
        Args:
            points: Array of points shape (N, 2)
            
        Returns:
            Tuple[np.ndarray, np.ndarray]: Tangents and second derivatives arrays
        """
        n = len(points)
        tangents = np.zeros_like(points, dtype=np.float64)
        second_derivatives = np.zeros_like(points, dtype=np.float64)
        
        # Compute second derivatives at interior points
        for i in range(1, n-1):
            # Use three adjacent points to estimate second derivative
            second_derivatives[i] = points[i-1] - 2*points[i] + points[i+1]
        
        # Estimate second derivatives at endpoints using forward/backward differences
        second_derivatives[0] = second_derivatives[1]
        second_derivatives[-1] = second_derivatives[-2]
        
        # Compute segment lengths for scaling
        diffs = np.diff(points, axis=0)
        segment_lengths = np.sqrt(np.sum(diffs**2, axis=1))
        avg_length = np.mean(segment_lengths)
        
        # Compute tangents that ensure G2 continuity
        for i in range(1, n-1):
            prev_diff = points[i] - points[i-1]
            next_diff = points[i+1] - points[i]
            
            # Scale by segment lengths
            prev_len = segment_lengths[i-1]
            next_len = segment_lengths[i]
            scale = avg_length / max(prev_len, next_len)
            
            # Combine first and second derivatives with scaling
            tangents[i] = scale * ((prev_diff + next_diff)/2 + second_derivatives[i]/6)
        
        # Handle endpoints with G2 consideration
        tangents[0] = scale * (points[1] - points[0] + second_derivatives[0]/6)
        tangents[-1] = scale * (points[-1] - points[-2] + second_derivatives[-1]/6)
        
        return tangents, second_derivatives
        
    def initialize_spline(self, points: np.ndarray, nodes, tangents: Optional[np.ndarray] = None) -> bool:
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
        Compute G2 continuous Hermite spline coefficients
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
            dt = t[i+1] - t[i]
            dt2 = dt * dt
            
            # Create geometry matrices including second derivative information
            Gx = np.array([
                points[i][0],
                points[i+1][0],
                tangents[i][0],
                tangents[i+1][0],
                second_derivatives[i][0] * dt2,
                second_derivatives[i+1][0] * dt2
            ])
            
            Gy = np.array([
                points[i][1],
                points[i+1][1],
                tangents[i][1],
                tangents[i+1][1],
                second_derivatives[i][1] * dt2,
                second_derivatives[i+1][1] * dt2
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
        Get point on spline at parameter t using G2 basis functions
        """
        if self.t_points is None or self.coefficients is None:
            raise ValueError("Spline not initialized")
            
        i = self._find_segment(t)
        
        # Normalize t to [0, 1] for this segment
        t_local = (t - self.t_points[i]) / (self.t_points[i+1] - self.t_points[i])
        t_local = np.clip(t_local, 0, 1)
        
        # Extended basis vector for G2 continuity
        T = np.array([1, t_local, t_local**2, t_local**3, t_local**4, t_local**5])
        
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