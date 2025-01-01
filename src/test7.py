import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass
import time
from functools import wraps
from concurrent.futures import ProcessPoolExecutor
import multiprocessing

@dataclass
class PathLookupTable:
    """Cache for quick parameter lookups based on distance"""
    distances: np.ndarray  # Sorted array of distances
    parameters: np.ndarray  # Corresponding parameter values
    total_length: float

def measure_performance(func):
    @wraps(func)
    def wrapper(*args, **kwargs):
        start_time = time.perf_counter()
        result = func(*args, **kwargs)
        end_time = time.perf_counter()
        execution_time = end_time - start_time
        print(f"{func.__name__} took {execution_time:.4f} seconds")
        return result, execution_time
    return wrapper

class SimpleSpline:
    """A simplified spline class for testing arc length calculations"""
    
    def __init__(self, control_points):
        """Initialize with control points for a basic cubic interpolation"""
        self.control_points = np.array(control_points)
        self._derivative_cache = {}
        
    def get_point(self, t):
        """Get point on spline at parameter t"""
        t = np.clip(t, 0, 1)
        return (1-t)**3 * self.control_points[0] + \
               3*t*(1-t)**2 * self.control_points[1] + \
               3*t**2*(1-t) * self.control_points[2] + \
               t**3 * self.control_points[3]
    
    def get_derivative(self, t):
        """Get derivative at parameter t with caching"""
        t = float(t)  # Ensure t is hashable
        if t not in self._derivative_cache:
            t_clipped = np.clip(t, 0, 1)
            self._derivative_cache[t] = -3*(1-t_clipped)**2 * self.control_points[0] + \
                                      (3*(1-t_clipped)**2 - 6*t_clipped*(1-t_clipped)) * self.control_points[1] + \
                                      (6*t_clipped*(1-t_clipped) - 3*t_clipped**2) * self.control_points[2] + \
                                      3*t_clipped**2 * self.control_points[3]
        return self._derivative_cache[t]

    def get_arc_length(self, t_start, t_end, num_points=20):
        """Calculate arc length using Gaussian quadrature"""
        points, weights = np.polynomial.legendre.leggauss(num_points)
        half_length = (t_end - t_start) / 2
        midpoint = (t_start + t_end) / 2
        transformed_points = points * half_length + midpoint
        derivative_magnitudes = np.array([
            np.linalg.norm(self.get_derivative(t))
            for t in transformed_points
        ])
        return float(half_length * np.sum(weights * derivative_magnitudes))

@measure_performance
def build_lookup_table_simple(spline, num_samples=1000):
    """Original simple method (for comparison)"""
    parameters = np.linspace(0, 1, num_samples)
    distances = np.zeros(num_samples)
    current_dist = 0.0
    
    for i in range(1, num_samples):
        prev_param = parameters[i-1]
        curr_param = parameters[i]
        derivative = spline.get_derivative(prev_param)
        segment_length = np.linalg.norm(derivative) * (curr_param - prev_param)
        current_dist += segment_length
        distances[i] = current_dist
        
    return PathLookupTable(
        distances=distances,
        parameters=parameters,
        total_length=current_dist
    )

@measure_performance
def build_lookup_table_simple_vectorized(spline, num_samples=1000):
    """Improved vectorized version using Simpson's rule integration"""
    # Ensure we have an odd number of samples for Simpson's rule
    if num_samples % 2 == 0:
        num_samples += 1
    
    parameters = np.linspace(0, 1, num_samples)
    dt = parameters[1] - parameters[0]
    
    # Calculate derivatives at all points
    derivatives = np.array([spline.get_derivative(t) for t in parameters])
    derivative_magnitudes = np.linalg.norm(derivatives, axis=1)
    
    # Apply Simpson's rule
    # For Simpson's rule: integral ≈ (h/3)(f₀ + 4f₁ + 2f₂ + 4f₃ + 2f₄ + ... + 4f_{n-1} + f_n)
    coeffs = np.ones(num_samples)
    coeffs[1:-1:2] = 4  # Multiply middle points by 4
    coeffs[2:-1:2] = 2  # Multiply even-indexed points by 2
    
    # Calculate segment lengths using Simpson's rule
    segment_lengths = derivative_magnitudes * coeffs * (dt / 3)
    
    # For cumulative distances, we need to calculate partial sums
    # We'll use trapezoidal rule for intermediate points to maintain monotonicity
    partial_lengths = (derivative_magnitudes[:-1] + derivative_magnitudes[1:]) * 0.5 * dt
    
    # Compute cumulative distances
    distances = np.zeros(num_samples)
    distances[1:] = np.cumsum(partial_lengths)
    
    # Adjust final distance to match Simpson's rule total
    total_length = np.sum(segment_lengths)
    if distances[-1] != 0:  # Avoid division by zero
        distances[1:] *= (total_length / distances[-1])
    
    return PathLookupTable(
        distances=distances,
        parameters=parameters,
        total_length=total_length
    )

@measure_performance
def build_lookup_table_gauss_optimized(spline, num_samples=1000, gauss_points=10):
    """Optimized version of Gaussian quadrature lookup table builder"""
    parameters = np.linspace(0, 1, num_samples)
    distances = np.zeros(num_samples)
    
    # Precompute Gaussian quadrature points and weights once
    points, weights = np.polynomial.legendre.leggauss(gauss_points)
    dt = parameters[1] - parameters[0]
    half_dt = dt / 2
    
    for i in range(1, num_samples):
        prev_param = parameters[i-1]
        midpoint = prev_param + half_dt
        transformed_points = points * half_dt + midpoint
        
        derivative_magnitudes = np.array([
            np.linalg.norm(spline.get_derivative(t))
            for t in transformed_points
        ])
        
        segment_length = half_dt * np.sum(weights * derivative_magnitudes)
        distances[i] = distances[i-1] + segment_length
    
    return PathLookupTable(
        distances=distances,
        parameters=parameters,
        total_length=distances[-1]
    )

@measure_performance
def build_lookup_table_adaptive(spline, min_samples=100, max_samples=2000, tolerance=1e-6):
    """Build lookup table with adaptive sampling based on curvature"""
    # Start with minimum samples
    parameters = np.linspace(0, 1, min_samples)
    derivatives = np.array([spline.get_derivative(t) for t in parameters])
    
    # Calculate curvature approximation
    curvature = np.diff(derivatives, axis=0)
    curvature_magnitude = np.linalg.norm(curvature, axis=1)
    
    # Identify regions needing more samples
    threshold = np.mean(curvature_magnitude) + np.std(curvature_magnitude)
    high_curvature_regions = curvature_magnitude > threshold
    
    # Add more samples in high curvature regions
    new_parameters = []
    for i in range(len(high_curvature_regions)):
        if high_curvature_regions[i]:
            t1, t2 = parameters[i], parameters[i+1]
            new_parameters.extend(np.linspace(t1, t2, 5)[1:-1])
    
    # Combine and sort all parameters
    all_parameters = np.sort(np.concatenate([parameters, new_parameters]))
    
    # Ensure we don't exceed max_samples
    if len(all_parameters) > max_samples:
        indices = np.linspace(0, len(all_parameters)-1, max_samples, dtype=int)
        all_parameters = all_parameters[indices]
    
    # Use vectorized method with adaptive parameters
    num_samples = len(all_parameters)
    # Ensure we have an odd number of samples for Simpson's rule
    if num_samples % 2 == 0:
        num_samples += 1
    
    parameters = np.linspace(0, 1, num_samples)
    dt = parameters[1] - parameters[0]
    
    # Calculate derivatives at all points
    derivatives = np.array([spline.get_derivative(t) for t in parameters])
    derivative_magnitudes = np.linalg.norm(derivatives, axis=1)
    
    # Apply Simpson's rule
    # For Simpson's rule: integral ≈ (h/3)(f₀ + 4f₁ + 2f₂ + 4f₃ + 2f₄ + ... + 4f_{n-1} + f_n)
    coeffs = np.ones(num_samples)
    coeffs[1:-1:2] = 4  # Multiply middle points by 4
    coeffs[2:-1:2] = 2  # Multiply even-indexed points by 2
    
    # Calculate segment lengths using Simpson's rule
    segment_lengths = derivative_magnitudes * coeffs * (dt / 3)
    
    # For cumulative distances, we need to calculate partial sums
    # We'll use trapezoidal rule for intermediate points to maintain monotonicity
    partial_lengths = (derivative_magnitudes[:-1] + derivative_magnitudes[1:]) * 0.5 * dt
    
    # Compute cumulative distances
    distances = np.zeros(num_samples)
    distances[1:] = np.cumsum(partial_lengths)
    
    # Adjust final distance to match Simpson's rule total
    total_length = np.sum(segment_lengths)
    if distances[-1] != 0:  # Avoid division by zero
        distances[1:] *= (total_length / distances[-1])
    
    return PathLookupTable(
        distances=distances,
        parameters=parameters,
        total_length=total_length
    )

def plot_results(spline, results, timings):
    """Plot spline and performance comparison"""
    t = np.linspace(0, 1, 1000)
    points = np.array([spline.get_point(t_) for t_ in t])
    
    fig = plt.figure(figsize=(15, 10))
    
    # Plot spline
    ax1 = plt.subplot(221)
    ax1.plot(points[:, 0], points[:, 1], 'b-', label='Spline')
    ax1.plot(spline.control_points[:, 0], spline.control_points[:, 1], 'ro--', 
             label='Control points')
    ax1.grid(True)
    ax1.legend()
    ax1.set_title('Test Spline')
    ax1.set_aspect('equal')
    
    # Plot total lengths
    ax2 = plt.subplot(222)
    methods = list(results.keys())
    lengths = [results[method].total_length for method in methods]
    exec_times = [timings[method] for method in methods]
    
    ax2.bar(methods, lengths)
    ax2.set_title('Total Arc Length by Method')
    plt.xticks(rotation=45)
    
    # Plot execution times
    ax3 = plt.subplot(223)
    ax3.bar(methods, exec_times)
    ax3.set_title('Execution Time by Method')
    plt.xticks(rotation=45)
    
    # Plot relative differences
    ax4 = plt.subplot(224)
    reference_length = results['gauss_optimized'].total_length
    differences = [abs(results[method].total_length - reference_length) / reference_length * 100 
                  for method in methods]
    ax4.bar(methods, differences)
    ax4.set_title('Relative Difference from Gaussian Method (%)')
    plt.xticks(rotation=45)
    
    plt.tight_layout()
    plt.show()

def main():
    # Create a test spline with significant curvature
    control_points = np.array([
        [0, 0],
        [1, 5],
        [3, -1],
        [4, 0]
    ])
    spline = SimpleSpline(control_points)
    
    # Test all methods
    sample_size = 1000
    results = {}
    timings = {}
    
    # Run each method and collect results
    simple_result, simple_time = build_lookup_table_simple(spline, sample_size)
    results['simple'] = simple_result
    timings['simple'] = simple_time
    
    vectorized_result, vectorized_time = build_lookup_table_simple_vectorized(spline, sample_size)
    results['vectorized'] = vectorized_result
    timings['vectorized'] = vectorized_time
    
    gauss_result, gauss_time = build_lookup_table_gauss_optimized(spline, sample_size)
    results['gauss_optimized'] = gauss_result
    timings['gauss_optimized'] = gauss_time
    
    adaptive_result, adaptive_time = build_lookup_table_adaptive(spline)
    results['adaptive'] = adaptive_result
    timings['adaptive'] = adaptive_time
    
    # Print detailed results
    print("\nDetailed Results:")
    print("-" * 50)
    reference_length = results['gauss_optimized'].total_length
    
    for method in results:
        length = results[method].total_length
        time = timings[method]
        diff = abs(length - reference_length) / reference_length * 100
        
        print(f"\n{method}:")
        print(f"  Total length: {length:.6f}")
        print(f"  Execution time: {time:.4f} seconds")
        print(f"  Difference from Gaussian: {diff:.6f}%")
    
    # Plot results
    plot_results(spline, results, timings)

if __name__ == "__main__":
    main()