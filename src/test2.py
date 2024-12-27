import numpy as np
import matplotlib.pyplot as plt
from typing import List, Tuple
from splines.quintic_hermite_spline import QuinticHermiteSpline

def create_test_cases() -> List[Tuple[np.ndarray, np.ndarray]]:
    """Create various test cases for the spline."""
    test_cases = []
    
    # Test case 1: Simple curve
    x1 = np.array([0, .1, 2])
    y1 = np.array([0, .1, 0])
    test_cases.append((x1, y1))
    
    # Test case 2: S-curve
    x2 = np.array([0, 1, 2, 3])
    y2 = np.array([0, 1, -1, 0])
    test_cases.append((x2, y2))
    
    # Test case 3: Circle-like curve
    theta = np.linspace(0, 2*np.pi, 8)[:-1]  # Remove last point to avoid overlap
    x3 = np.cos(theta)
    y3 = np.sin(theta)
    test_cases.append((x3, y3))
    
    return test_cases

def test_spline(spline: QuinticHermiteSpline, x: np.ndarray, y: np.ndarray, 
                title: str, ax: plt.Axes) -> None:
    """Test the spline with given control points and plot the results."""
    # Fit the spline
    success = spline.fit(x, y)
    if not success:
        print(f"Failed to fit spline for {title}")
        return
    
    # Generate points along the spline
    t = np.linspace(spline.parameters[0], spline.parameters[-1], 200)
    points = np.array([spline.get_point(ti) for ti in t])
    derivatives = np.array([spline.get_derivative(ti) for ti in t])
    second_derivatives = np.array([spline.get_second_derivative(ti) for ti in t])
    
    # Plot the spline
    ax.plot(points[:, 0], points[:, 1], 'b-', label='Spline')
    ax.plot(x, y, 'ro', label='Control Points')
    
    # Plot derivatives at control points
    if spline.first_derivatives is not None:
        scale = 0.2  # Scale factor for derivative vectors
        for i in range(len(x)):
            dx, dy = spline.first_derivatives[i]
            ax.arrow(x[i], y[i], dx*scale, dy*scale, 
                    head_width=0.05, head_length=0.1, fc='g', ec='g')
    
    ax.set_title(title)
    ax.grid(True)
    ax.axis('equal')
    ax.legend()

def main():
    # Create figure
    fig = plt.figure(figsize=(15, 5))
    test_cases = create_test_cases()
    
    # Test each case
    for i, (x, y) in enumerate(test_cases, 1):
        ax = fig.add_subplot(1, 3, i)
        spline = QuinticHermiteSpline()
        test_spline(spline, x, y, f'Test Case {i}', ax)
    
    plt.tight_layout()
    plt.show()
    
    # Additional test: Verify continuity
    print("\nTesting continuity at segment joints...")
    spline = QuinticHermiteSpline()
    x = np.array([0, 1, 2, 3])
    y = np.array([0, 1, -1, 0])
    spline.fit(x, y)
    
    # Check continuity at internal points
    for i in range(1, len(x)-1):
        t = spline.parameters[i]
        
        # Check position continuity
        point_left = spline.get_point(t - 1e-6)
        point_right = spline.get_point(t + 1e-6)
        position_diff = np.linalg.norm(point_right - point_left)
        print(f"Position difference at t={t}: {position_diff:.2e}")
        
        # Check derivative continuity
        deriv_left = spline.get_derivative(t - 1e-6)
        deriv_right = spline.get_derivative(t + 1e-6)
        deriv_diff = np.linalg.norm(deriv_right - deriv_left)
        print(f"Derivative difference at t={t}: {deriv_diff:.2e}")
        
        # Check second derivative continuity
        second_deriv_left = spline.get_second_derivative(t - 1e-6)
        second_deriv_right = spline.get_second_derivative(t + 1e-6)
        second_deriv_diff = np.linalg.norm(second_deriv_right - second_deriv_left)
        print(f"Second derivative difference at t={t}: {second_deriv_diff:.2e}\n")

if __name__ == "__main__":
    main()