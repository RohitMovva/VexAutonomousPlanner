from typing import Dict, Optional, Tuple
from splines.spline import Spline  # type: ignore
import numpy as np

class CubicHermiteSpline(Spline):
    """Cubic Hermite spline implementation"""
    
    def __init__(self):
        super().__init__()
        self.coefficients: Optional[Dict] = None
        self.path_points: Optional[np.ndarray] = None
        self.steps: int = 50  # Number of points to generate per segment
        self.t_points: Optional[np.ndarray] = None
        self.tangents: Optional[np.ndarray] = None
        
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
        
    def estimate_tangents(self, points: np.ndarray) -> np.ndarray:
        """
        Estimate tangent vectors at each point using Catmull-Rom method
        """
        n = len(points)
        tangents = np.zeros_like(points)
        tension = 0.8  # Catmull-Rom tension parameter
        
        # Interior points: Catmull-Rom tangents
        for i in range(1, n-1):
            tangents[i] = tension * (points[i+1] - points[i-1])
        
        # Endpoints: match slope of the adjacent segment
        tangents[0] = tension * (points[1] - points[0]) * 2
        tangents[-1] = tension * (points[-1] - points[-2]) * 2
        
        return tangents
        
    def initialize_spline(self, points: np.ndarray, nodes, tangents: Optional[np.ndarray] = None) -> bool:
        if len(points) < 2:
            print("Not enough points")
            return False

        try:
            self.t_points = self.compute_parameters(points)
            
            # Use provided tangents or estimate them
            if tangents is None:
                self.tangents = self.estimate_tangents(points)
            else:
                self.tangents = tangents

            # ---------------------------------------------------------------------
            # Force cusp logic at reverse nodes:
            # Instead of flipping the angle by 180°, we set the tangent to zero
            # at the reverse node, then optionally flip the tangent for the next node.
            # ---------------------------------------------------------------------
            # for i, node in enumerate(nodes):
            #     if node.is_reverse_node:
            #         # 1) Zero out tangent at i -> cusp in velocity
            #         self.tangents[i] = np.zeros_like(self.tangents[i])
                    
            #         # 2) (Optional) Flip the tangent at i+1 if you really want to go back
            #         #    *immediately* in the opposite direction on the next segment.
            #         if i + 1 < len(self.tangents):
            #             self.tangents[i + 1] = -self.tangents[i + 1]

            # Compute spline coefficients
            self.coefficients = self.compute_hermite_coefficients(points, self.tangents, self.t_points)

            return True

        except Exception as e:
            print(f"Error initializing spline: {e}")
            return False
            
    def compute_hermite_coefficients(self, points: np.ndarray, 
                                     tangents: np.ndarray, 
                                     t: np.ndarray) -> Dict:
        """
        Compute Hermite spline coefficients for each segment using the standard matrix form
        """
        n = len(points) - 1
        
        # Hermite basis matrix
        M = np.array([
            [ 2, -2,  1,  1],
            [-3,  3, -2, -1],
            [ 0,  0,  1,  0],
            [ 1,  0,  0,  0]
        ])
        
        coeffs_x = []
        coeffs_y = []
        
        for i in range(n):
            dt = t[i+1] - t[i]
            
            # Geometry matrix for x coordinates
            Gx = np.array([
                points[i][0],
                points[i+1][0],
                dt * tangents[i][0],
                dt * tangents[i+1][0]
            ])
            
            # Geometry matrix for y coordinates
            Gy = np.array([
                points[i][1],
                points[i+1][1],
                dt * tangents[i][1],
                dt * tangents[i+1][1]
            ])
            
            # Compute coefficients
            cx = M @ Gx
            cy = M @ Gy
            
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
        Get point on spline at parameter t
        """
        if self.t_points is None or self.coefficients is None:
            raise ValueError("Spline not initialized")
            
        i = self._find_segment(t)
        
        # Normalize t to [0, 1] for this segment
        t_local = (t - self.t_points[i]) / (self.t_points[i+1] - self.t_points[i])
        t_local = np.clip(t_local, 0, 1)
        
        # Compute basis vector
        T = np.array([t_local**3, t_local**2, t_local, 1.0])
        
        # Compute point coordinates using coefficient matrices
        x = T @ self.coefficients['x'][i]
        y = T @ self.coefficients['y'][i]
             
        return np.array([x, y])
        
    def get_derivative(self, t: float) -> np.ndarray:
        if self.t_points is None or self.coefficients is None:
            raise ValueError("Spline not initialized")
            
        i = self._find_segment(t)
        dt = self.t_points[i+1] - self.t_points[i]
        
        # Normalize t to [0, 1]
        t_local = (t - self.t_points[i]) / dt
        t_local = np.clip(t_local, 0, 1)
        
        # Derivative of cubic polynomial: [3a, 2b, c] * [t², t, 1]
        T = np.array([3 * t_local**2, 2 * t_local, 1.0, 0.0])
        
        dx = T @ self.coefficients['x'][i]
        dy = T @ self.coefficients['y'][i]
        
        return np.array([dx, dy]) / dt
        
    def get_second_derivative(self, t: float) -> np.ndarray:
        """
        Get second derivative at parameter t
        """
        if self.t_points is None or self.coefficients is None:
            raise ValueError("Spline not initialized")
            
        i = self._find_segment(t)
        dt = self.t_points[i+1] - self.t_points[i]
        
        # Normalize t to [0, 1]
        t_local = (t - self.t_points[i]) / dt
        t_local = np.clip(t_local, 0, 1)
        
        # The second derivative logic (if needed) would be adapted similarly,
        # but since your main concern is the tangent angle at reversed nodes,
        # we can leave this as is.
        
        # This part of code would need a proper Hermite second derivative calculation if required.
        # Currently, it's not fully implemented and would need Hermite basis second derivative computation.
        
        raise NotImplementedError("Second derivative computation not fully implemented.")
        
    def build_path(self, points: np.ndarray, 
                   nodes = None,
                   tangents: Optional[np.ndarray] = None,
                   steps: Optional[int] = None) -> np.ndarray:
        """
        Build a path through the given points using cubic Hermite splines
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
        # epsilon = 1e-7
        # for j in range(0, 5):
        #     epsilon *= 10
        #     print("Epsilon:", epsilon)

        #     t_node = self.t_points[1]
        #     deriv_before = self.get_derivative(t_node - epsilon)
        #     deriv_after = self.get_derivative(t_node + epsilon)
        #     print("Derivative before node:", deriv_before)
        #     print("Derivative after node:", deriv_after)
        #     angle_before = np.arctan2(deriv_before[1], deriv_before[0])
        #     angle_after = np.arctan2(deriv_after[1], deriv_after[0])
        #     print("Angle before node:", np.degrees(angle_before))
        #     print("Angle after node:", np.degrees(angle_after))

        return self.path_points
        
    def fit(self, x: np.ndarray, y: np.ndarray) -> bool:
        """
        Fit cubic Hermite spline to points
        """
        if len(x) != len(y) or len(x) < 2:
            return False
            
        # Store parameter values
        self.t_points = x
        
        # Handle 2D points
        if y.ndim == 2:
            points = np.column_stack([x, y])
        else:
            points = np.column_stack([x, y.reshape(-1, 1)])
            
        # Estimate tangents and initialize spline
        tangents = self.estimate_tangents(points)
        return self.initialize_spline(points, tangents)
        
    def get_path_points(self) -> Optional[np.ndarray]:
        """
        Get the most recently generated path points
        """
        return self.path_points
