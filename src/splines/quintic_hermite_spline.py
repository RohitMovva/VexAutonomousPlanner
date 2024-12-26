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
        Compute parameter values with improved spacing
        """
        diffs = np.diff(points, axis=0)
        segment_lengths = np.sqrt(np.sum(diffs**2, axis=1))
        
        # Use chord length parameterization
        t = np.zeros(len(points))
        t[1:] = np.cumsum(segment_lengths)
        
        # Normalize to [0, 1]
        if t[-1] > 0:
            t = t / t[-1]
            
            # Use gentler endpoint adjustment
            alpha = 0.4  # Reduced from 0.8 for less aggressive adjustment
            t = t * (1.0 - alpha) + (t ** 2) * alpha
                
        return t

            
    def estimate_tangents_and_derivatives(self, points: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Estimate tangent vectors and second derivatives with additional debugging
        """
        n = len(points)
        tangents = np.zeros_like(points, dtype=np.float64)
        second_derivatives = np.zeros_like(points, dtype=np.float64)
        
        print("\n=== Tangent and Second Derivative Estimation ===")
        
        # Compute segment properties
        diffs = np.diff(points, axis=0)
        segment_lengths = np.sqrt(np.sum(diffs**2, axis=1))
        
        print(f"Segment lengths: {segment_lengths}")
        
        # Estimate interior tangents
        for i in range(1, n-1):
            prev_diff = points[i] - points[i-1]
            next_diff = points[i+1] - points[i]
            
            # Normalize segment vectors
            prev_dir = prev_diff / np.linalg.norm(prev_diff)
            next_dir = next_diff / np.linalg.norm(next_diff)
            
            # Use bisector of angle between segments
            tangent = prev_dir + next_dir
            tangent_norm = np.linalg.norm(tangent)
            if tangent_norm > 0:
                tangents[i] = tangent / tangent_norm
                
            print(f"\nPoint {i}:")
            print(f"Previous direction: {prev_dir}")
            print(f"Next direction: {next_dir}")
            print(f"Computed tangent: {tangents[i]}")
        
        # End point tangents
        tangents[0] = diffs[0] / segment_lengths[0]
        tangents[-1] = diffs[-1] / segment_lengths[-1]
        
        print(f"\nEndpoint tangents:")
        print(f"Start: {tangents[0]}")
        print(f"End: {tangents[-1]}")
        
        # Compute second derivatives
        for i in range(1, n-1):
            h = (segment_lengths[i-1] + segment_lengths[i]) / 2
            second_derivatives[i] = (tangents[i+1] - tangents[i-1]) / (2 * h)
            
            print(f"\nPoint {i} second derivative:")
            print(f"h = {h}")
            print(f"Value: {second_derivatives[i]}")
        
        # End point second derivatives
        second_derivatives[0] = second_derivatives[1] * 0.5
        second_derivatives[-1] = second_derivatives[-2] * 0.5
        
        print("\nFinal derivatives:")
        for i in range(n):
            print(f"Point {i}:")
            print(f"Tangent: {tangents[i]}")
            print(f"Second derivative: {second_derivatives[i]}")
        
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
        Compute G2 continuous Hermite spline coefficients with detailed debugging
        """
        n = len(points) - 1
        print("\nComputing G2 coefficients with detailed debugging:")
        
        # Quintic Hermite basis matrix
        M_h = np.array([
            [ 1,  0,  0,  0,  0,  0],     # Position at t=0
            [ 0,  1,  0,  0,  0,  0],     # First derivative at t=0
            [ 0,  0,  0.5,  0,  0,  0],   # Second derivative at t=0
            [ 1,  1,  0.5,  1,  1,  0.5], # Position at t=1
            [ 0,  1,  1,  0,  1,  1],     # First derivative at t=1
            [ 0,  0,  1,  0,  0,  1]      # Second derivative at t=1
        ])
        
        print("\nBasis Matrix M_h:")
        print(M_h)
        
        coeffs_x = []
        coeffs_y = []
        
        # Compute segment properties
        diffs = np.diff(points, axis=0)
        segment_lengths = np.sqrt(np.sum(diffs**2, axis=1))
        
        for i in range(n):
            dt = t[i+1] - t[i]
            dt2 = dt * dt
            
            # Normalize and scale derivatives
            chord = points[i+1] - points[i]
            chord_length = segment_lengths[i]
            
            print(f"\n=== Segment {i} Analysis ===")
            print(f"dt = {dt:.3f}")
            print(f"Chord length = {chord_length:.3f}")
            print(f"Points: {points[i]} -> {points[i+1]}")
            print(f"Original tangents: {tangents[i]} -> {tangents[i+1]}")
            print(f"Original second derivatives: {second_derivatives[i]} -> {second_derivatives[i+1]}")
            
            # Scale derivatives by segment properties
            scaled_tang_i = tangents[i] * chord_length
            scaled_tang_ip1 = tangents[i+1] * chord_length
            scaled_acc_i = second_derivatives[i] * chord_length * chord_length
            scaled_acc_ip1 = second_derivatives[i+1] * chord_length * chord_length
            
            print(f"Scaled tangents: {scaled_tang_i} -> {scaled_tang_ip1}")
            print(f"Scaled accelerations: {scaled_acc_i} -> {scaled_acc_ip1}")
            
            # Create geometry vectors
            Gx = np.array([
                points[i][0],                    # Position at t=0
                scaled_tang_i[0] / dt,          # First derivative at t=0
                scaled_acc_i[0] / dt2,          # Second derivative at t=0
                points[i+1][0],                  # Position at t=1
                scaled_tang_ip1[0] / dt,        # First derivative at t=1
                scaled_acc_ip1[0] / dt2         # Second derivative at t=1
            ])
            
            Gy = np.array([
                points[i][1],
                scaled_tang_i[1] / dt,
                scaled_acc_i[1] / dt2,
                points[i+1][1],
                scaled_tang_ip1[1] / dt,
                scaled_acc_ip1[1] / dt2
            ])
            
            print("\nGeometry vectors:")
            print(f"Gx: {Gx}")
            print(f"Gy: {Gy}")
            
            # Compute coefficients
            cx = M_h @ Gx
            cy = M_h @ Gy
            
            print("\nComputed coefficients:")
            print(f"cx: {cx}")
            print(f"cy: {cy}")
            
            # Verify continuity at key points
            t_values = [0.0, 0.001, 0.999, 1.0]
            for t_val in t_values:
                T = np.array([1, t_val, t_val**2, t_val**3, t_val**4, t_val**5])
                dT = np.array([0, 1, 2*t_val, 3*t_val**2, 4*t_val**3, 5*t_val**4])
                
                pos = np.array([T @ cx, T @ cy])
                deriv = np.array([dT @ cx, dT @ cy])
                heading = np.degrees(np.arctan2(deriv[1], deriv[0]))
                
                print(f"\nt={t_val:.3f}:")
                print(f"T: {T}")
                print(f"dT: {dT}")
                print(f"Position: {pos}")
                print(f"Derivative: {deriv}")
                print(f"Heading: {heading:.2f}°")
            
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
        
    # In get_point method:
    def get_point(self, t: float) -> np.ndarray:
        """
        Get point on spline at parameter t with improved numerical stability
        """
        if self.t_points is None or self.coefficients is None:
            raise ValueError("Spline not initialized")
            
        i = self._find_segment(t)
        # print(f"\nPoint generation for t={t}:")
        # print(f"Selected segment: {i}")
        # print(f"Segment t_points: [{self.t_points[i]}, {self.t_points[i+1]}]")
        
        # Normalize t to [0, 1] for this segment
        t_local = (t - self.t_points[i]) / (self.t_points[i+1] - self.t_points[i])
        t_local = np.clip(t_local, 0, 1)
        # print(f"Local parameter t_local: {t_local}")
        
        # Compute powers of t efficiently
        t2 = t_local * t_local
        t3 = t2 * t_local
        t4 = t3 * t_local
        t5 = t4 * t_local
        
        # Print coefficient matrices for this segment
        # print(f"X coefficients: {self.coefficients['x'][i]}")
        # print(f"Y coefficients: {self.coefficients['y'][i]}")
        
        # Extended basis vector
        T = np.array([1, t_local, t2, t3, t4, t5])
        # print(f"Basis vector T: {T}")
        
        # Compute point coordinates
        x = T @ self.coefficients['x'][i]
        y = T @ self.coefficients['y'][i]
        point = np.array([x, y])
        # print(f"Generated point: {point}")
        
        return point
            
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