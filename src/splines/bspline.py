from typing import Optional
import numpy as np
from splines.spline import Spline  # type: ignore

class BSpline(Spline):
    """Cubic B-spline implementation with G2 continuity"""
    
    def __init__(self, degree: int = 3):
        super().__init__()
        self.degree = degree
        self.knots: Optional[np.ndarray] = None
        self.control_points: Optional[np.ndarray] = None
        self._center: Optional[np.ndarray] = None
        self._scale: Optional[np.ndarray] = None
    
    def _find_span(self, t: float) -> int:
        """Find the knot span index"""
        if self.knots is None:
            raise ValueError("Knots not initialized")
            
        if t == self.knots[-self.degree-1]:  # Special case for t at end of domain
            return len(self.knots) - self.degree - 2
            
        # Binary search to find knot span
        low = self.degree
        high = len(self.knots) - self.degree - 1
        
        while low <= high:
            mid = (low + high) // 2
            if t >= self.knots[mid] and t < self.knots[mid + 1]:
                return mid
            elif t < self.knots[mid]:
                high = mid - 1
            else:
                low = mid + 1
                
        return self.degree  # Fallback for t at start of domain
    
    def _compute_basis(self, span: int, t: float) -> np.ndarray:
        """Compute all basis function values at t"""
        if self.knots is None:
            raise ValueError("Knots not initialized")
            
        basis = np.zeros(self.degree + 1)
        left = np.zeros(self.degree + 1)
        right = np.zeros(self.degree + 1)
        
        # Initialize zeroth degree basis
        basis[0] = 1.0
        
        # Compute basis functions using de Boor's algorithm
        for j in range(1, self.degree + 1):
            left[j] = t - self.knots[span + 1 - j]
            right[j] = self.knots[span + j] - t
            saved = 0.0
            
            for r in range(j):
                temp = basis[r] / (right[r + 1] + left[j - r])
                basis[r] = saved + right[r + 1] * temp
                saved = left[j - r] * temp
                
            basis[j] = saved
            
        return basis
    
    def _normalize_parameter(self, t: float) -> float:
        """Convert global parameter t to local knot space parameter"""
        if self.knots is None:
            raise ValueError("Knots not initialized")
            
        if t <= 0.0:
            return self.knots[self.degree]
        if t >= 1.0:
            return self.knots[-self.degree-1]
            
        # Map t to the valid knot span
        domain_start = self.knots[self.degree]
        domain_end = self.knots[-self.degree-1]
        return domain_start + t * (domain_end - domain_start)
    
    def _generate_knot_vector(self, n_points: int) -> np.ndarray:
        """Generate knot vector with improved endpoint handling"""
        if n_points < self.degree + 1:
            raise ValueError(f"Need at least {self.degree + 1} points for degree {self.degree} B-spline")
            
        # Create knot vector with higher multiplicity at endpoints
        n_knots = n_points + self.degree + 1
        knots = np.zeros(n_knots)
        
        # Multiple knots at start
        knots[:self.degree+1] = 0.0
        
        # Multiple knots at end
        knots[-self.degree-1:] = 1.0
        
        # Internal knots with better spacing
        if n_points > self.degree + 1:
            internal_knots = np.linspace(0, 1, n_points - self.degree + 1)
            knots[self.degree:-self.degree] = internal_knots[1:-1]
        
        return knots
        
    def basis_function(self, i: int, k: int, t: float) -> float:
        """
        Compute B-spline basis function using Cox-de Boor recursion
        
        Args:
            i: Index of control point
            k: Degree of basis function
            t: Parameter value
            
        Returns:
            float: Basis function value
        """
        if self.knots is None:
            raise ValueError("Knots not initialized")
            
        # Handle edge cases
        if t == self.knots[-self.degree-1] and i == len(self.knots) - self.degree - 2:
            return 1.0
        if t < self.knots[i] or t >= self.knots[i + k + 1]:
            return 0.0
            
        if k == 0:
            return 1.0 if self.knots[i] <= t < self.knots[i + 1] else 0.0
            
        # Compute the basis function recursively with safeguards
        denom1 = self.knots[i + k] - self.knots[i]
        denom2 = self.knots[i + k + 1] - self.knots[i + 1]
        
        sum = 0.0
        if abs(denom1) > 1e-10:
            sum += ((t - self.knots[i]) / denom1) * self.basis_function(i, k-1, t)
            
        if abs(denom2) > 1e-10:
            sum += ((self.knots[i + k + 1] - t) / denom2) * self.basis_function(i+1, k-1, t)
            
        return sum
    
    def basis_derivative(self, i: int, k: int, t: float, d: int = 1) -> float:
        """
        Compute derivative of B-spline basis function
        
        Args:
            i: Index of control point
            k: Degree of basis function
            t: Parameter value
            d: Derivative order
            
        Returns:
            float: Basis function derivative value
        """
        if self.knots is None:
            raise ValueError("Knots not initialized")
            
        if d == 0:
            return self.basis_function(i, k, t)
            
        # Compute derivative using difference of lower degree basis functions
        denom1 = self.knots[i + k] - self.knots[i]
        term1 = 0.0 if abs(denom1) < 1e-10 else \
                k / denom1 * self.basis_derivative(i, k - 1, t, d - 1)
                
        denom2 = self.knots[i + k + 1] - self.knots[i + 1]
        term2 = 0.0 if abs(denom2) < 1e-10 else \
                k / denom2 * self.basis_derivative(i + 1, k - 1, t, d - 1)
                
        return term1 - term2
    
    def fit(self, x: np.ndarray, y: np.ndarray) -> bool:
        """Fit B-spline with improved handling for few points"""
        try:
            # Convert inputs and normalize
            x = np.asarray(x)
            y = np.asarray(y)
            points = np.column_stack((x, y))
            n_original = len(points)

            # Store scale factors for denormalization
            self._center = np.mean(points, axis=0)
            self._scale = np.max(np.abs(points - self._center), axis=0)
            self._scale[self._scale == 0] = 1.0
            
            # Normalize points to [-1, 1] range
            points = (points - self._center) / self._scale
            
            # Special handling for fewer points
            if n_original < 4:  # Minimum points needed for cubic B-spline
                # Generate intermediate points using quadratic interpolation
                new_points = []
                for i in range(n_original):
                    new_points.append(points[i])
                    if i < n_original - 1:
                        # Add two interpolated points between each pair
                        t_values = [1/3, 2/3]
                        for t in t_values:
                            if i == 0 and n_original == 3:
                                # Quadratic interpolation using 3 points
                                p0, p1, p2 = points[0], points[1], points[2]
                                # Quadratic Bezier formula
                                point = (1-t)**2 * p0 + 2*(1-t)*t * p1 + t**2 * p2
                            else:
                                # Linear interpolation for other cases
                                point = (1-t) * points[i] + t * points[i+1]
                            new_points.append(point)
                
                points = np.array(new_points)

            # Generate knot vector with proper multiplicity
            self.knots = self._generate_knot_vector(len(points))
            
            # Build coefficient matrix with improved conditioning
            A = np.zeros((len(points), len(points)))
            t = np.linspace(0, 1, len(points))
            
            for i in range(len(points)):
                for j in range(len(points)):
                    A[i,j] = self.basis_function(j, self.degree, self._normalize_parameter(t[i]))
            
            # Add Tikhonov regularization
            reg_param = 1e-6 * np.trace(A.T @ A) / len(points)
            A = A + reg_param * np.eye(len(points))
            
            # Solve system using more stable SVD approach
            u, s, vh = np.linalg.svd(A, full_matrices=False)
            rcond = 1e-10  # Condition number threshold
            s_inv = np.where(s > rcond * s[0], 1/s, 0)
            A_inv = vh.T @ np.diag(s_inv) @ u.T
            
            # Compute control points with separate handling for each coordinate
            self.control_points = np.zeros((len(points), 2))
            for i in range(2):
                self.control_points[:,i] = A_inv @ points[:,i]
            
            # Transform back to original space
            self.control_points = self.control_points * self._scale + self._center
            return True
            
        except Exception as e:
            print(f"Error fitting B-spline: {str(e)}")
            # Fallback to piecewise linear interpolation
            if n_original >= 2:
                # Create a finer set of points using piecewise linear interpolation
                refined_points = []
                for i in range(n_original - 1):
                    t_local = np.linspace(0, 1, 10)  # 10 points per segment
                    for t in t_local[:-1]:  # Exclude last point except for final segment
                        point = points[i] * (1-t) + points[i+1] * t
                        refined_points.append(point)
                refined_points.append(points[-1])  # Add final point
                
                self.control_points = np.array(refined_points) * self._scale + self._center
                return True
            return False
            
    def get_point(self, t: float) -> np.ndarray:
        """Get point on B-spline at parameter t"""
        if self.control_points is None or self.knots is None:
            raise ValueError("B-spline not initialized")
            
        # Convert parameter to knot space
        t = self._normalize_parameter(t)
        
        # Find the knot span
        span = self._find_span(t)
        
        # Compute the basis functions
        basis = self._compute_basis(span, t)
        
        # Compute point
        point = np.zeros(2)
        for i in range(self.degree + 1):
            point += basis[i] * self.control_points[span - self.degree + i]
            
        return point
        
    def get_derivative(self, t: float) -> np.ndarray:
        """Get first derivative at parameter t"""
        if self.control_points is None or self.knots is None:
            raise ValueError("B-spline not initialized")
            
        # Convert parameter to knot space
        t = self._normalize_parameter(t)
        
        # Compute derivative using basis function derivatives
        derivative = np.zeros(2)
        for i in range(len(self.control_points)):
            basis_der = self.basis_derivative(i, self.degree, t, d=1)
            derivative += basis_der * self.control_points[i]
            
        return derivative
        
    def get_second_derivative(self, t: float) -> np.ndarray:
        """Get second derivative at parameter t"""
        if self.control_points is None or self.knots is None:
            raise ValueError("B-spline not initialized")
            
        # Convert parameter to knot space
        t = self._normalize_parameter(t)
        
        # Compute second derivative using basis function derivatives
        derivative = np.zeros(2)
        for i in range(len(self.control_points)):
            basis_der = self.basis_derivative(i, self.degree, t, d=2)
            derivative += basis_der * self.control_points[i]
            
        return derivative