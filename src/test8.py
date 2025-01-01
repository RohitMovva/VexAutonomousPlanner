import numpy as np
from dataclasses import dataclass

@dataclass
class PathLookupTable:
    """Cache for quick parameter lookups based on distance"""
    distances: np.ndarray  # Sorted array of distances
    parameters: np.ndarray  # Corresponding parameter values
    total_length: float

class TestSpline:
    """Mock spline class that matches your QuinticHermiteSpline interface"""
    def __init__(self):
        self.arc_length = 2.0  # Example fixed length for testing
        
    def get_derivative(self, t):
        """Simple derivative function for testing"""
        # Return a vector that would give us our known arc length
        return np.array([2.0, 0.0])  # Constant derivative for predictable length
        
    def get_total_arc_length(self):
        return self.arc_length

def build_lookup_table_original(splines, num_nodes, num_samples=1000):
    """Original total arc length calculation"""
    total_length = 0.0
    for spline in splines:
        total_length += spline.get_total_arc_length()
    return total_length

def build_lookup_table_new(splines, num_nodes, num_samples=1000):
    """New vectorized lookup table builder"""
    if not splines:
        raise ValueError("No splines initialized")
        
    total_param_length = num_nodes - 1
    parameters = np.linspace(0, total_param_length, num_samples)
    distances = np.zeros(num_samples)
    
    # Process each spline segment
    current_idx = 0
    current_dist = 0.0
    
    print("\nDetailed segment analysis:")
    for spline_idx, spline in enumerate(splines):
        # Find parameters for this spline segment
        mask = (parameters >= spline_idx) & (parameters < spline_idx + 1)
        spline_params = parameters[mask]
        
        if len(spline_params) > 1:
            # Map to local spline parameters [0,1]
            local_params = spline_params - spline_idx
            
            # Vectorized derivative calculation
            derivatives = np.array([spline.get_derivative(t) for t in local_params[:-1]])
            dt = local_params[1] - local_params[0]
            
            # Calculate segment lengths vectorized
            segment_lengths = np.linalg.norm(derivatives, axis=1) * dt
            cumulative_lengths = np.cumsum(segment_lengths)
            
            # Calculate the number of points in this segment
            points_in_segment = len(spline_params)
            
            # Fill distances array
            distances[current_idx + 1:current_idx + points_in_segment] = current_dist + cumulative_lengths
            
            # Update tracking variables
            current_dist += cumulative_lengths[-1] if len(cumulative_lengths) > 0 else 0
            current_idx += points_in_segment - 1
            
            print(f"\nSpline {spline_idx}:")
            print(f"  Points in segment: {points_in_segment}")
            print(f"  Current index: {current_idx}")
            print(f"  Current distance: {current_dist}")
            print(f"  Segment length: {cumulative_lengths[-1] if len(cumulative_lengths) > 0 else 0}")
    
    # Ensure the last point has the total distance
    distances[-1] = current_dist
    
    return distances[-1]  # Return total length for comparison

def main():
    # Create test splines with known lengths
    num_splines = 3
    splines = [TestSpline() for _ in range(num_splines)]
    num_nodes = num_splines + 1
    
    # Test both methods
    original_length = build_lookup_table_original(splines, num_nodes)
    print(f"\nOriginal method total length: {original_length}")
    
    # Test with different sample sizes
    sample_sizes = [100, 1000, 10000]
    for samples in sample_sizes:
        new_length = build_lookup_table_new(splines, num_nodes, samples)
        print(f"\nNew method (samples={samples}):")
        print(f"  Total length: {new_length}")
        print(f"  Difference: {abs(new_length - original_length)}")
        print(f"  Relative error: {abs(new_length - original_length)/original_length*100:.6f}%")
        
    # Detailed segment analysis
    print("\nDetailed segment analysis:")
    parameters = np.linspace(0, num_nodes - 1, 1000)
    for spline_idx, spline in enumerate(splines):
        mask = (parameters >= spline_idx) & (parameters < spline_idx + 1)
        spline_params = parameters[mask]
        if len(spline_params) > 1:
            local_params = spline_params - spline_idx
            dt = local_params[1] - local_params[0]
            derivatives = np.array([spline.get_derivative(t) for t in local_params[:-1]])
            segment_length = np.sum(np.linalg.norm(derivatives, axis=1) * dt)
            print(f"Spline {spline_idx}:")
            print(f"  Local parameter range: {local_params[0]:.3f} to {local_params[-1]:.3f}")
            print(f"  dt: {dt:.6f}")
            print(f"  Computed length: {segment_length}")
            print(f"  Expected length: {spline.get_total_arc_length()}")

if __name__ == "__main__":
    main()