from typing import List, Optional, Type, Dict, Any
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
    spline: Optional[Any] = None
    points: Optional[np.ndarray] = None

class SplineManager:
    """Manages a chain of spline segments"""
    
    def __init__(self, spline_class: Type['Spline']):
        """
        Initialize the spline manager
        
        Args:
            spline_class: Class type of spline to use (must extend base Spline class)
        """
        self.spline_class = spline_class
        self.segments: List[SplineSegment] = []
        self.path_points: Optional[np.ndarray] = None
        self._parameter_map: Dict[float, tuple[int, float]] = {}
        
    def update_splines(self, points: np.ndarray, nodes: List[Node]) -> np.ndarray:
        """
        Update spline chain with new points and nodes
        
        Args:
            points: Array of points shape (N, 2)
            nodes: List of Node objects containing node properties
                        
        Returns:
            np.ndarray: Array of points defining the complete path
        """
        if len(points) < 2:
            return np.array([])

        print("\n=== Starting Spline Update ===")
        print(f"Input points shape: {points.shape}")
        print(f"Number of nodes: {len(nodes)}")

        # Create segments
        self.segments = []
        current_start = 0
        
        print("\n=== Segment Creation ===")
        # Identify all segments
        for i in range(1, len(points)):
            # Check if current point is a reverse node
            is_reverse = nodes[i].is_reverse_node if i < len(nodes) else False
            # Check if previous point was a reverse node
            prev_reverse = nodes[i-1].is_reverse_node if i-1 < len(nodes) else False
            
            # Create a new segment if we hit:
            # 1. A reverse node (previous point ends a segment)
            # 2. A point after a reverse node (starts a new segment)
            # 3. The last point
            if prev_reverse or is_reverse or i == len(points) - 1:
                segment = SplineSegment(
                    start_idx=current_start,
                    end_idx=i,
                    is_reverse=nodes[current_start].is_reverse_node if current_start < len(nodes) else False,
                    transition_point=i
                )
                print(f"Created segment: {current_start} -> {i} (reverse: {segment.is_reverse})")
                self.segments.append(segment)
                current_start = i

        print(f"\nTotal segments created: {len(self.segments)}")

        # First pass: Calculate initial tangents
        print("\n=== Initial Tangent Calculation ===")
        for i, segment in enumerate(self.segments):
            sub_spline = self.spline_class()
            segment.spline = sub_spline
            
            seg_points = points[segment.start_idx:segment.end_idx + 1]
            print(f"\nSegment {i}:")
            print(f"Points: {seg_points}")
            
            if hasattr(sub_spline, 'estimate_tangents'):
                segment.tangents = sub_spline.estimate_tangents(seg_points)
            else:
                segment.tangents = self._estimate_default_tangents(seg_points)
            print(f"Initial tangents: {segment.tangents}")

        # Second pass: Handle reverse nodes and segment connections
        print("\n=== Processing Reverse Nodes ===")
        for i, segment in enumerate(self.segments):
            print(f"\nChecking segment {i} (is_reverse: {segment.is_reverse})")
            
            if segment.is_reverse:
                # Get the previous and next segments if they exist
                prev_segment = self.segments[i-1] if i > 0 else None
                next_segment = self.segments[i+1] if i + 1 < len(self.segments) else None
                
                if prev_segment:  # Handle connection to previous segment
                    incoming_tangent = prev_segment.tangents[-1]
                    outgoing_tangent = segment.tangents[0]
                    
                    print(f"Processing reverse connection:")
                    print(f"Incoming tangent: {incoming_tangent}")
                    print(f"Outgoing tangent: {outgoing_tangent}")
                    
                    # Check vector norms
                    incoming_norm = np.linalg.norm(incoming_tangent)
                    outgoing_norm = np.linalg.norm(outgoing_tangent)
                    
                    if incoming_norm > 0 and outgoing_norm > 0:
                        # Simply average the reversed incoming tangent with the outgoing tangent
                        reversed_incoming = -incoming_tangent
                        averaged_tangent = (reversed_incoming + outgoing_tangent) / 2
                        
                        # Normalize and scale to maintain magnitude
                        avg_magnitude = (incoming_norm + outgoing_norm) / 2
                        if np.linalg.norm(averaged_tangent) > 0:
                            averaged_tangent = averaged_tangent * (avg_magnitude / np.linalg.norm(averaged_tangent))
                        
                        print(f"Computed averaged tangent: {averaged_tangent}")
                        
                        # Update both segments' tangents
                        prev_segment.tangents[-1] = -averaged_tangent
                        segment.tangents[0] = averaged_tangent
                    else:
                        print("Zero-length tangent detected - using fallback")
                        if incoming_norm > 0:
                            segment.tangents[0] = -incoming_tangent
                        elif outgoing_norm > 0:
                            prev_segment.tangents[-1] = -outgoing_tangent

                elif next_segment:  # First segment with next segment
                    # Similar simple averaging approach for first segment
                    current_tangent = segment.tangents[-1]
                    next_tangent = next_segment.tangents[0]
                    
                    cur_norm = np.linalg.norm(current_tangent)
                    next_norm = np.linalg.norm(next_tangent)
                    
                    if cur_norm > 0 and next_norm > 0:
                        reversed_current = -current_tangent
                        averaged_tangent = (reversed_current + next_tangent) / 2
                        
                        # Scale to maintain average magnitude
                        avg_magnitude = (cur_norm + next_norm) / 2
                        if np.linalg.norm(averaged_tangent) > 0:
                            averaged_tangent = averaged_tangent * (avg_magnitude / np.linalg.norm(averaged_tangent))
                        
                        # Update both segments
                        segment.tangents = np.array([-averaged_tangent, -averaged_tangent])
                        next_segment.tangents[0] = averaged_tangent

        # Final pass: Generate path points
        print("\n=== Generating Final Path ===")
        all_path_points = []
        
        for i, segment in enumerate(self.segments):
            print(f"\nGenerating points for segment {i}:")
            print(f"Using tangents: {segment.tangents}")
            
            seg_points = points[segment.start_idx:segment.end_idx + 1]
            
            # Initialize spline
            try:
                if hasattr(segment.spline, 'initialize_spline'):
                    # Ensure tangents match the number of points
                    if len(segment.tangents) != len(seg_points):
                        print(f"Warning: Tangent count mismatch. Points: {len(seg_points)}, Tangents: {len(segment.tangents)}")
                        # Adjust tangents to match point count
                        if len(segment.tangents) > len(seg_points):
                            segment.tangents = segment.tangents[:len(seg_points)]
                        else:
                            # Extend tangents by repeating the last tangent
                            extra_tangents = [segment.tangents[-1]] * (len(seg_points) - len(segment.tangents))
                            segment.tangents = np.vstack([segment.tangents, extra_tangents])
                    
                    segment.spline.initialize_spline(seg_points, None, segment.tangents)
                else:
                    segment.spline.fit(seg_points[:, 0], seg_points[:, 1])
            except Exception as e:
                print(f"Error initializing spline: {str(e)}")
            
            # Generate points
            t_vals = np.linspace(0, 1, 250)
            segment.points = np.array([segment.spline.get_point(t) for t in t_vals])
            
            # Add points to path
            if i > 0 and len(all_path_points) > 0:
                if np.allclose(all_path_points[-1], segment.points[0], atol=1e-7):
                    print("Removing duplicate point at segment boundary")
                    segment_points = segment.points[1:]
                else:
                    segment_points = segment.points
            else:
                segment_points = segment.points
                
            print(f"Added {len(segment_points)} points to path")
            all_path_points.extend(segment_points)
            
        # Store and return final path
        self.path_points = np.array(all_path_points)
        self._compute_parameter_map()
        
        print(f"\n=== Spline Update Complete ===")
        print(f"Final path contains {len(self.path_points)} points")
        
        return self.path_points
    
    def _estimate_default_tangents(self, points: np.ndarray) -> np.ndarray:
        """Provide default tangent estimation if not provided by spline class"""
        n = len(points)
        tangents = np.zeros_like(points)
        tension = 1.5
        
        # Interior points
        for i in range(1, n-1):
            tangents[i] = tension * (points[i+1] - points[i-1])
            
        # Endpoints
        tangents[0] = tension * (points[1] - points[0]) * 2
        tangents[-1] = tension * (points[-1] - points[-2]) * 2
        
        return tangents

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
        
        print(f"\nParameter conversion debug:")
        print(f"Global t: {t}")
        print(f"Segment and local: {segment_and_local}")
        print(f"Segment idx: {segment_idx}")
        print(f"Local t: {local_t}")
        
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