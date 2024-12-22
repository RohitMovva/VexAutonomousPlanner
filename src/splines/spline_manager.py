from typing import List, Optional, Type, Dict, Any, Tuple
from dataclasses import dataclass
from gui.node import Node
from splines.spline import Spline
import numpy as np

@dataclass
class SplineSegment:
    """Data class to store information about a spline segment"""
    start_idx: int
    end_idx: int
    is_reverse: bool
    transition_point: int
    tangents: Optional[np.ndarray] = None
    second_derivatives: Optional[np.ndarray] = None  # Added for G2 continuity
    spline: Optional[Any] = None
    points: Optional[np.ndarray] = None

class SplineManager:
    """Manages a chain of spline segments with G2 continuity"""
    
    def __init__(self, spline_class: Type['Spline']):
        self.spline_class = spline_class
        self.segments: List[SplineSegment] = []
        self.path_points: Optional[np.ndarray] = None
        self._parameter_map: Dict[float, tuple[int, float]] = {}
        
    def _estimate_default_derivatives(self, points: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Provide default tangent and second derivative estimation if not provided by spline class
        """
        n = len(points)
        tangents = np.zeros_like(points)
        second_derivatives = np.zeros_like(points)
        tension = 1.5
        
        # Compute second derivatives at interior points
        for i in range(1, n-1):
            second_derivatives[i] = points[i-1] - 2*points[i] + points[i+1]
        
        # Estimate second derivatives at endpoints
        second_derivatives[0] = second_derivatives[1]
        second_derivatives[-1] = second_derivatives[-2]
        
        # Interior points with G2 consideration
        for i in range(1, n-1):
            prev_diff = points[i] - points[i-1]
            next_diff = points[i+1] - points[i]
            tangents[i] = tension * ((prev_diff + next_diff)/2 + second_derivatives[i]/6)
            
        # Endpoints with G2 consideration
        tangents[0] = tension * (points[1] - points[0] + second_derivatives[0]/6)
        tangents[-1] = tension * (points[-1] - points[-2] + second_derivatives[-1]/6)
        
        return tangents, second_derivatives

    def update_splines(self, points: np.ndarray, nodes: List[Node]) -> np.ndarray:
        """Update spline chain with G2 continuity"""
        if len(points) < 2:
            return np.array([])

        print("\n=== Starting G2 Spline Update ===")
        
        # Create segments (same as before)
        self.segments = []
        current_start = 0
        
        for i in range(1, len(points)):
            is_reverse = nodes[i].is_reverse_node if i < len(nodes) else False
            prev_reverse = nodes[i-1].is_reverse_node if i-1 < len(nodes) else False
            
            if prev_reverse or is_reverse or i == len(points) - 1:
                segment = SplineSegment(
                    start_idx=current_start,
                    end_idx=i,
                    is_reverse=nodes[current_start].is_reverse_node if current_start < len(nodes) else False,
                    transition_point=i,
                    second_derivatives=None  # Initialize new field
                )
                self.segments.append(segment)
                current_start = i

        # Add in update_splines method, after initial segment creation:
        print("\n=== Segment Details ===")
        for i, segment in enumerate(self.segments):
            print(f"\nSegment {i}:")
            print(f"  Start idx: {segment.start_idx}")
            print(f"  End idx: {segment.end_idx}")
            print(f"  Points range: {points[segment.start_idx:segment.end_idx + 1]}")
            print(f"  Is reverse: {segment.is_reverse}")
            print(f"  Transition point: {segment.transition_point}")

        # First pass: Calculate initial derivatives
        print("\n=== Initial Derivative Calculation ===")
        for i, segment in enumerate(self.segments):
            
            sub_spline = self.spline_class()
            segment.spline = sub_spline
            
            seg_points = points[segment.start_idx:segment.end_idx + 1]
            
            # Check if spline class provides G2 estimation
            if hasattr(sub_spline, 'estimate_tangents_and_derivatives'):
                segment.tangents, segment.second_derivatives = sub_spline.estimate_tangents_and_derivatives(seg_points)
            else:
                segment.tangents, segment.second_derivatives = self._estimate_default_derivatives(seg_points)

            # Add in first pass of derivative calculation:
            print(f"\nSegment {i} Derivatives:")
            print(f"  Points shape: {seg_points.shape}")
            print(f"  Tangents shape: {segment.tangents.shape}")
            print(f"  Second derivatives shape: {segment.second_derivatives.shape}")
            print(f"  Sample tangents: \n{segment.tangents}")
            print(f"  Sample second derivatives: \n{segment.second_derivatives}")

        # Second pass: Handle reverse nodes and ensure G2 continuity
        print("\n=== Processing Reverse Nodes with G2 Continuity ===")
        for i, segment in enumerate(self.segments):
            if segment.is_reverse:
                prev_segment = self.segments[i-1] if i > 0 else None
                next_segment = self.segments[i+1] if i + 1 < len(self.segments) else None
                
                if prev_segment:
                    # Handle tangent continuity
                    incoming_tangent = prev_segment.tangents[-1]
                    outgoing_tangent = segment.tangents[0]
                    
                    # Average tangents while preserving magnitude
                    if np.linalg.norm(incoming_tangent) > 0 and np.linalg.norm(outgoing_tangent) > 0:
                        reversed_incoming = -incoming_tangent
                        averaged_tangent = (reversed_incoming + outgoing_tangent) / 2
                        avg_magnitude = (np.linalg.norm(incoming_tangent) + np.linalg.norm(outgoing_tangent)) / 2
                        if np.linalg.norm(averaged_tangent) > 0:
                            averaged_tangent *= (avg_magnitude / np.linalg.norm(averaged_tangent))
                        
                        # Update tangents
                        prev_segment.tangents[-1] = -averaged_tangent
                        segment.tangents[0] = averaged_tangent
                        
                        # Ensure G2 continuity by matching second derivatives
                        segment.second_derivatives[0] = -prev_segment.second_derivatives[-1]
                    
                elif next_segment:
                    # Similar process for connection to next segment
                    current_tangent = segment.tangents[-1]
                    next_tangent = next_segment.tangents[0]
                    
                    if np.linalg.norm(current_tangent) > 0 and np.linalg.norm(next_tangent) > 0:
                        reversed_current = -current_tangent
                        averaged_tangent = (reversed_current + next_tangent) / 2
                        avg_magnitude = (np.linalg.norm(current_tangent) + np.linalg.norm(next_tangent)) / 2
                        if np.linalg.norm(averaged_tangent) > 0:
                            averaged_tangent *= (avg_magnitude / np.linalg.norm(averaged_tangent))
                        
                        # Update tangents
                        segment.tangents = np.array([-averaged_tangent, -averaged_tangent])
                        next_segment.tangents[0] = averaged_tangent
                        
                        # Ensure G2 continuity
                        next_segment.second_derivatives[0] = -segment.second_derivatives[-1]

            # Add in second pass for reverse node processing:
            if segment.is_reverse:
                print(f"\nProcessing reverse segment {i}:")
                print(f"  Start point: {points[segment.start_idx]}")
                print(f"  End point: {points[segment.end_idx]}")
                
                if prev_segment:
                    print("  Previous segment exists:")
                    print(f"    Incoming tangent: {incoming_tangent}")
                    print(f"    Outgoing tangent: {outgoing_tangent}")
                    print(f"    Averaged tangent: {averaged_tangent}")
                    print(f"    Updated prev segment end tangent: {prev_segment.tangents[-1]}")
                    print(f"    Updated current segment start tangent: {segment.tangents[0]}")
                    print(f"    Second derivatives at boundary: {segment.second_derivatives[0]}, {prev_segment.second_derivatives[-1]}")


        # Final pass: Generate path points with G2 continuity
        print("\n=== Generating G2 Continuous Path ===")
        all_path_points = []
        
        for i, segment in enumerate(self.segments):
            seg_points = points[segment.start_idx:segment.end_idx + 1]
            
            try:
                if hasattr(segment.spline, 'initialize_spline'):
                    # Ensure all arrays match in length
                    if len(segment.tangents) != len(seg_points):
                        print(f"Warning: Adjusting tangent array length")
                        if len(segment.tangents) > len(seg_points):
                            segment.tangents = segment.tangents[:len(seg_points)]
                            segment.second_derivatives = segment.second_derivatives[:len(seg_points)]
                        else:
                            segment.tangents = np.vstack([segment.tangents, 
                                                        np.tile(segment.tangents[-1], 
                                                        (len(seg_points) - len(segment.tangents), 1))])
                            segment.second_derivatives = np.vstack([segment.second_derivatives,
                                                                 np.tile(segment.second_derivatives[-1],
                                                                 (len(seg_points) - len(segment.second_derivatives), 1))])
                    
                    # Initialize with both tangents and second derivatives
                    if hasattr(segment.spline, 'initialize_spline_g2'):
                        segment.spline.initialize_spline_g2(seg_points, None, 
                                                         segment.tangents, 
                                                         segment.second_derivatives)
                    else:
                        segment.spline.initialize_spline(seg_points, None, segment.tangents)
                else:
                    segment.spline.fit(seg_points[:, 0], seg_points[:, 1])
                    
                # Generate points
                t_vals = np.linspace(0, 1, 250)
                segment.points = np.array([segment.spline.get_point(t) for t in t_vals])
                
                # Handle segment connections
                if i > 0 and len(all_path_points) > 0:
                    if np.allclose(all_path_points[-1], segment.points[0], atol=1e-7):
                        segment_points = segment.points[1:]
                    else:
                        segment_points = segment.points
                else:
                    segment_points = segment.points
                    
                all_path_points.extend(segment_points)
                
            except Exception as e:
                print(f"Error processing segment {i}: {str(e)}")

            # Add in final pass for point generation:
            print(f"\nGenerating points for segment {i}:")
            print(f"  Input points: {seg_points}")
            print(f"  Final tangents: {segment.tangents}")
            print(f"  Final second derivatives: {segment.second_derivatives}")
            
            if i > 0:
                print(f"  Previous end point: {all_path_points[-1] if len(all_path_points) > 0 else None}")
                print(f"  Current start point: {segment.points[0] if segment.points is not None else None}")


        # Store and return final path
        self.path_points = np.array(all_path_points)
        self._compute_parameter_map()

        # Add after final path generation:
        if len(self.path_points) > 0:
            print("\n=== Final Path Statistics ===")
            print(f"Total points: {len(self.path_points)}")
            print(f"Start point: {self.path_points[0]}")
            print(f"End point: {self.path_points[-1]}")
            print(f"Original end point: {points[-1]}")
            endpoint_diff = np.linalg.norm(self.path_points[-1] - points[-1])
            print(f"Distance to target endpoint: {endpoint_diff}")
            
            # Check continuity between segments
            print("\n=== Segment Continuity Check ===")
            for i in range(len(self.segments)-1):
                if self.segments[i].points is not None and self.segments[i+1].points is not None:
                    end_point = self.segments[i].points[-1]
                    start_point = self.segments[i+1].points[0]
                    gap = np.linalg.norm(end_point - start_point)
                    print(f"Gap between segments {i} and {i+1}: {gap}")
                    if gap > 1e-6:
                        print(f"  Warning: Significant gap detected!")
                        print(f"  End point of segment {i}: {end_point}")
                        print(f"  Start point of segment {i+1}: {start_point}")
        
        return self.path_points
    
    # def _estimate_default_tangents(self, points: np.ndarray) -> np.ndarray:
    #     """Provide default tangent estimation if not provided by spline class"""
    #     n = len(points)
    #     tangents = np.zeros_like(points)
    #     tension = 1.5
        
    #     # Interior points
    #     for i in range(1, n-1):
    #         tangents[i] = tension * (points[i+1] - points[i-1])
            
    #     # Endpoints
    #     tangents[0] = tension * (points[1] - points[0]) * 2
    #     tangents[-1] = tension * (points[-1] - points[-2]) * 2
        
    #     return tangents

    def _compute_parameter_map(self):
        """Compute mapping from global parameter to (segment_index, local_parameter)"""
        total_length = 0
        segment_lengths = []
        
        # Calculate length of each segment
        for segment in self.segments:
            if segment.points is not None and len(segment.points) > 1:
                diffs = np.diff(segment.points, axis=0)
                length = np.sum(np.sqrt(np.sum(diffs**2, axis=1)))
                segment_lengths.append(length)
                total_length += length
            else:
                segment_lengths.append(0)
                
        if total_length == 0:
            return
            
        # Create parameter mapping
        cumulative_length = 0
        self._parameter_map.clear()  # Clear existing mappings
        
        for i, length in enumerate(segment_lengths):
            if length > 0:
                start_t = cumulative_length / total_length
                end_t = (cumulative_length + length) / total_length
                
                # Store mapping points
                self._parameter_map[start_t] = (i, 0.0)
                self._parameter_map[end_t] = (i, 1.0)
                
                cumulative_length += length
                
    def _get_segment_and_parameter(self, t: float) -> tuple[int, float]:
        """
        Convert global parameter to segment index and local parameter
        
        Args:
            t: Global parameter value [0, 1]
                    
        Returns:
            tuple: (segment_index, local_parameter)
        """
        # Number of segments
        n_segments = len(self.segments)
        if n_segments == 0:
            return 0, t
            
        # Clip t to [0, 1]
        t = np.clip(t, 0, 1)
        
        # Convert global parameter to segment index and local parameter
        # t * n_segments gives us a value in [0, n_segments]
        segment_and_local = t * n_segments
        
        # Integer division for segment index
        segment_idx = int(segment_and_local)
        
        # Ensure we don't exceed the last segment
        if segment_idx >= n_segments:
            return n_segments - 1, 1.0
            
        # Fractional part becomes the local parameter
        local_t = segment_and_local - segment_idx
        
        # print(f"\nParameter conversion debug:")
        # print(f"Global t: {t}")
        # print(f"Segment and local: {segment_and_local}")
        # print(f"Segment idx: {segment_idx}")
        # print(f"Local t: {local_t}")
        
        return segment_idx, local_t
    
    def get_position(self, t: float) -> np.ndarray:
        """
        Get position on the spline path at parameter t
        
        Args:
            t: Global parameter value [0, 1]
            
        Returns:
            np.ndarray: Point coordinates [x, y]
        """
        segment_idx, local_t = self._get_segment_and_parameter(t)
        
        if not self.segments or segment_idx >= len(self.segments):
            raise ValueError("Invalid segment index")
            
        segment = self.segments[segment_idx]
        if segment.spline is None:
            raise ValueError("Segment spline not initialized")
            
        return segment.spline.get_point(local_t)

    def get_derivative(self, t: float) -> np.ndarray:
        """
        Get first derivative (tangent vector) at parameter t
        
        Args:
            t: Global parameter value [0, 1]
            
        Returns:
            np.ndarray: Tangent vector [dx, dy]
        """
        segment_idx, local_t = self._get_segment_and_parameter(t)
        
        if not self.segments or segment_idx >= len(self.segments):
            raise ValueError("Invalid segment index")
            
        segment = self.segments[segment_idx]
        if segment.spline is None:
            raise ValueError("Segment spline not initialized")
            
        derivative = segment.spline.get_derivative(local_t)

                # Determine if we're in a reversed state by checking the number of reverse nodes
        # we've encountered up to this point
        num_reverse_nodes = 0
        for prev_segment in self.segments[:segment_idx + 1]:
            if prev_segment.is_reverse:
                num_reverse_nodes += 1
                
        # If we've encountered an odd number of reverse nodes, we're in a reversed state
        is_reversed = (num_reverse_nodes % 2) == 1
        
        # If we're in a reversed state, rotate the angle by 180 degrees
        if is_reversed:
            derivative = -derivative
            
        
        # # Handle reverse segments by reversing the derivative
        # if segment.is_reverse:
        #     derivative = -derivative
            
        return derivative

    def get_tangent_angle(self, t: float) -> float:
        """
        Get tangent angle in radians at parameter t
        
        Args:
            t: Global parameter value [0, 1]
            
        Returns:
            float: Angle in radians
        """
        # Get the derivative which is already adjusted for reverse state
        derivative = self.get_derivative(t)
        return np.arctan2(derivative[1], derivative[0])

    def get_curvature(self, t: float) -> float:
        """
        Get curvature at parameter t, properly accounting for parameter scaling
        
        Args:
            t: Global parameter value [0, 1]
            
        Returns:
            float: Curvature value (positive for counterclockwise, negative for clockwise)
        """
        segment_idx, local_t = self._get_segment_and_parameter(t)
        
        if not self.segments or segment_idx >= len(self.segments):
            raise ValueError("Invalid segment index")
            
        segment = self.segments[segment_idx]
        if segment.spline is None:
            raise ValueError("Segment spline not initialized")
            
        try:
            # Get derivatives in normalized parameter space
            d1 = segment.spline.get_derivative(local_t)
            d2 = segment.spline.get_second_derivative(local_t)
            
            # Get the parameter scaling factor (dt)
            if hasattr(segment.spline, 't_points') and segment.spline.t_points is not None:
                t_points = segment.spline.t_points
                seg_idx = segment.spline._find_segment(local_t)
                dt = t_points[seg_idx + 1] - t_points[seg_idx]
            else:
                dt = 1.0
                
            # Determine if we're in a reversed state
            num_reverse_nodes = 0
            for prev_segment in self.segments[:segment_idx + 1]:
                if prev_segment.is_reverse:
                    num_reverse_nodes += 1
                    
            is_reversed = (num_reverse_nodes % 2) == 1
            
            # If we're in a reversed state, negate both derivatives
            if is_reversed:
                d1 = -d1
                d2 = -d2
                
            # Calculate curvature with proper parameter scaling
            # κ = (x'y'' - y'x'') / (x'² + y'²)^(3/2)
            num = d1[0] * d2[1] - d1[1] * d2[0]
            den = (d1[0]**2 + d1[1]**2)**(3/2)
            
            # Apply parameter scaling
            if abs(den) < 1e-10:  # Avoid division by very small numbers
                return 0.0
                
            k = (num / den) * dt
            
            return k
            
        except Exception as e:
            print(f"Error calculating curvature: {str(e)}")
            return 0.0

    def get_segment_at_parameter(self, t: float) -> Optional[SplineSegment]:
        """
        Get the spline segment at parameter t
        
        Args:
            t: Global parameter value [0, 1]
            
        Returns:
            Optional[SplineSegment]: The segment containing parameter t, or None if invalid
        """
        segment_idx, _ = self._get_segment_and_parameter(t)
        
        if not self.segments or segment_idx >= len(self.segments):
            return None
            
        return self.segments[segment_idx]

    def get_arc_length(self, t1: float = 0.0, t2: float = 1.0, num_samples: int = 100) -> float:
        """
        Calculate approximate arc length between parameters t1 and t2
        
        Args:
            t1: Start parameter [0, 1]
            t2: End parameter [0, 1]
            num_samples: Number of samples for approximation
            
        Returns:
            float: Approximate arc length
        """
        if t1 > t2:
            t1, t2 = t2, t1
            
        t_vals = np.linspace(t1, t2, num_samples)
        points = np.array([self.get_position(t) for t in t_vals])
        
        # Calculate sum of segment lengths
        diffs = np.diff(points, axis=0)
        lengths = np.sqrt(np.sum(diffs**2, axis=1))
        return np.sum(lengths)

    def get_parameter_by_arc_length(self, s: float, tolerance: float = 1e-6) -> float:
        """
        Get parameter t corresponding to arc length s using binary search
        
        Args:
            s: Desired arc length
            tolerance: Error tolerance for binary search
            
        Returns:
            float: Parameter t corresponding to arc length s
        """
        total_length = self.get_arc_length()
        if total_length == 0:
            return 0.0
            
        # Normalize s to be within [0, total_length]
        s = np.clip(s, 0, total_length)
        
        # Binary search for parameter t
        t_low, t_high = 0.0, 1.0
        while (t_high - t_low) > tolerance:
            t_mid = (t_low + t_high) / 2
            length_mid = self.get_arc_length(0, t_mid)
            
            if abs(length_mid - s) < tolerance:
                return t_mid
            elif length_mid < s:
                t_low = t_mid
            else:
                t_high = t_mid
                
        return (t_low + t_high) / 2