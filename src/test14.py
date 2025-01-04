import numpy as np
import matplotlib.pyplot as plt
from typing import Optional, Tuple

# First, let's copy over the QuinticHermiteSpline class
# [Previous class implementation would go here]
from splines.quintic_hermite_spline import QuinticHermiteSpline

def analyze_spline():
    # Create a triangular set of points
    x = np.array([0.0, 1.0, 2.0])
    y = np.array([0.0, 1.0, 0.0])
    points = np.column_stack((x, y))
    
    # Initialize and fit the spline
    spline = QuinticHermiteSpline()
    success = spline.fit(x, y)
    
    if not success:
        print("Failed to fit spline")
        return
    
    # Print the computed derivatives at control points
    print("\nFirst derivatives at control points:")
    print(spline.first_derivatives)
    print("\nSecond derivatives at control points:")
    print(spline.second_derivatives)
    
    # Analyze behavior around t=1
    t_values = np.linspace(0.0, 2.0, 200)  # Sample points around t=1
    second_derivs = []
    
    print("\nSecond derivatives around t=1:")
    print("t\td²x/dt²\td²y/dt²")
    print("-" * 30)
    
    for t in t_values:
        second_deriv = spline.get_second_derivative(t)
        second_derivs.append(second_deriv)
        if abs(t - 1.0) < 0.01:  # Print values very close to t=1
            print(f"{t:.3f}\t{second_deriv[0]:.3f}\t{second_deriv[1]:.3f}")
    
    second_derivs = np.array(second_derivs)
    
    # Create visualization
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 12))
    
    # Plot the spline and control points
    t = np.linspace(0, 2, 200)
    points = np.array([spline.get_point(ti) for ti in t])
    
    ax1.plot(points[:, 0], points[:, 1], 'b-', label='Spline')
    ax1.plot(x, y, 'ro', label='Control Points')
    ax1.grid(True)
    ax1.legend()
    ax1.set_title('Quintic Hermite Spline')
    ax1.axis('equal')
    
    # Plot second derivatives
    ax2.plot(t_values, second_derivs[:, 0], 'r-', label='d²x/dt²')
    ax2.plot(t_values, second_derivs[:, 1], 'b-', label='d²y/dt²')
    ax2.axvline(x=1.0, color='k', linestyle='--', alpha=0.3)
    ax2.grid(True)
    ax2.legend()
    ax2.set_title('Second Derivatives near t=1')
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    analyze_spline()