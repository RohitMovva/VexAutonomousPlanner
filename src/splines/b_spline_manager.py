from typing import List, Optional, Type, Dict, Any, Tuple
from dataclasses import dataclass
from gui.node import Node
from splines.spline import Spline
import numpy as np
from splines.bspline import BSpline  # Import our BSpline class
from splines.spline_manager import SplineManager
from splines.spline_manager import SplineSegment

@dataclass
class BSplineSegment(SplineSegment):
    """Extended SplineSegment class with B-spline specific attributes"""
    knots: Optional[np.ndarray] = None
    control_points: Optional[np.ndarray] = None
    degree: int = 3

class BSplineManager(SplineManager):
    """Manages a chain of B-spline segments with G2 continuity"""
    
    def __init__(self):
        super().__init__(spline_class=BSpline)
        
    def _estimate_default_derivatives(self, points: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Provide B-spline specific derivative estimation
        
        Args:
            points: Array of shape (n, 2) containing x,y coordinates
            
        Returns:
            Tuple[np.ndarray, np.ndarray]: Estimated tangents and second derivatives
        """
        n = len(points)
        tangents = np.zeros_like(points)
        second_derivatives = np.zeros_like(points)
        
        # Use B-spline specific tension parameter
        tension = 1.0  # Lower tension for B-splines compared to other spline types
        
        # Compute second derivatives using B-spline approach
        for i in range(1, n-1):
            # Use central difference approximation
            h1 = np.linalg.norm(points[i] - points[i-1])
            h2 = np.linalg.norm(points[i+1] - points[i])
            if h1 > 0 and h2 > 0:
                d1 = (points[i] - points[i-1]) / h1
                d2 = (points[i+1] - points[i]) / h2
                second_derivatives[i] = (d2 - d1) / ((h1 + h2) / 2)
        
        # Estimate second derivatives at endpoints using forward/backward differences
        if n > 2:
            h = np.linalg.norm(points[1] - points[0])
            if h > 0:
                second_derivatives[0] = (points[2] - 2*points[1] + points[0]) / (h*h)
            h = np.linalg.norm(points[-1] - points[-2])
            if h > 0:
                second_derivatives[-1] = (points[-1] - 2*points[-2] + points[-3]) / (h*h)
        
        # Compute tangents with G2 consideration
        for i in range(1, n-1):
            # Use weighted average of adjacent segments
            prev_diff = points[i] - points[i-1]
            next_diff = points[i+1] - points[i]
            prev_len = np.linalg.norm(prev_diff)
            next_len = np.linalg.norm(next_diff)
            
            if prev_len > 0 and next_len > 0:
                # Weight by segment lengths for better shape preservation
                w1 = next_len / (prev_len + next_len)
                w2 = prev_len / (prev_len + next_len)
                tangents[i] = tension * (w1 * prev_diff/prev_len + w2 * next_diff/next_len)
                
                # Adjust tangent based on second derivative
                tangents[i] += second_derivatives[i] * min(prev_len, next_len) / 6
        
        # Endpoint tangents
        if n > 1:
            h1 = np.linalg.norm(points[1] - points[0])
            if h1 > 0:
                tangents[0] = tension * (points[1] - points[0]) / h1
                tangents[0] += second_derivatives[0] * h1 / 6
                
            hn = np.linalg.norm(points[-1] - points[-2])
            if hn > 0:
                tangents[-1] = tension * (points[-1] - points[-2]) / hn
                tangents[-1] += second_derivatives[-1] * hn / 6
        
        return tangents, second_derivatives
    
    def update_splines(self, points: np.ndarray, nodes: List[Node]) -> np.ndarray:
        """Update B-spline chain with improved handling for few points"""
        if len(points) < 2:
            return np.array([])
            
        print("\n=== Starting B-spline Update ===")
        
        # Create segments
        self.segments = []
        current_start = 0
        
        for i in range(1, len(points)):
            is_reverse = nodes[i].is_reverse_node if i < len(nodes) else False
            prev_reverse = nodes[i-1].is_reverse_node if i-1 < len(nodes) else False
            
            if prev_reverse or is_reverse or i == len(points) - 1:
                segment = BSplineSegment(
                    start_idx=current_start,
                    end_idx=i,
                    is_reverse=nodes[current_start].is_reverse_node if current_start < len(nodes) else False,
                    transition_point=i,
                    degree=3
                )
                self.segments.append(segment)
                current_start = i
        
        # Process segments
        path_points = []
        for i, segment in enumerate(self.segments):
            seg_points = points[segment.start_idx:segment.end_idx + 1]
            
            # Create spline instance
            sub_spline = BSpline(degree=segment.degree)
            segment.spline = sub_spline
            
            # Store points and compute derivatives
            segment.control_points = seg_points
            segment.tangents, segment.second_derivatives = self._estimate_default_derivatives(seg_points)
            
            try:
                # Fit the B-spline
                success = segment.spline.fit(seg_points[:, 0], seg_points[:, 1])
                if not success:
                    print(f"Warning: Failed to fit B-spline for segment {i}, using fallback")
                    continue
                
                # Generate points along the spline
                t_values = np.linspace(0, 1, max(20, len(seg_points) * 10))
                segment_points = np.array([segment.spline.get_point(t) for t in t_values])
                path_points.append(segment_points)
                
            except Exception as e:
                print(f"Error processing segment {i}: {str(e)}")
                # Fallback to piecewise linear interpolation
                segment_points = []
                n_segments = len(seg_points) - 1
                points_per_segment = max(10, int(20 / n_segments))
                
                for i in range(n_segments):
                    t_local = np.linspace(0, 1, points_per_segment)
                    for t in t_local[:-1]:  # Exclude last point except for final segment
                        point = seg_points[i] * (1-t) + seg_points[i+1] * t
                        segment_points.append(point)
                
                segment_points.append(seg_points[-1])  # Add final point
                path_points.append(np.array(segment_points))
        
        return np.vstack(path_points) if path_points else np.array([])
    def _get_segment_and_parameter(self, t: float) -> tuple[int, float]:
        """
        B-spline specific parameter conversion
        """
        segment_idx, local_t = super()._get_segment_and_parameter(t)
        
        # Additional B-spline specific parameter mapping if needed
        segment = self.segments[segment_idx]
        if isinstance(segment, BSplineSegment) and segment.knots is not None:
            # Map local_t to appropriate knot interval
            knots = segment.knots
            n_internal_knots = len(knots) - 2 * (segment.degree + 1)
            if n_internal_knots > 0:
                # Map local_t to knot space
                knot_t = knots[segment.degree] + local_t * (knots[-segment.degree-1] - knots[segment.degree])
                local_t = (knot_t - knots[segment.degree]) / (knots[-segment.degree-1] - knots[segment.degree])
        
        return segment_idx, local_t