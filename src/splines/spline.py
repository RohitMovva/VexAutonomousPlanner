import numpy as np
from abc import ABC, abstractmethod
from typing import Optional, Dict, Tuple


class Spline(ABC):
    """Abstract base class for spline curves"""
    
    def __init__(self):
        self.x_points: Optional[np.ndarray] = None
        self.y_points: Optional[np.ndarray] = None
        self._length_cache: Dict[Tuple[int, int], float] = {}
        
    def fit(self, x: np.ndarray, y: np.ndarray) -> bool:
        """
        Fit the spline to a set of points
        
        Args:
            x: x-coordinates of points
            y: y-coordinates of points
            
        Returns:
            bool: True if fitting was successful, False otherwise
        """
        if len(x) != len(y) or len(x) < 2:
            return False
            
        self.x_points = np.array(x)
        self.y_points = np.array(y)
        return True
        
    @abstractmethod
    def get_point(self, t: float) -> np.ndarray:
        """Get point on spline at parameter t"""
        pass
        
    @abstractmethod
    def get_derivative(self, t: float) -> np.ndarray:
        """Get first derivative at parameter t"""
        pass
        
    @abstractmethod
    def get_second_derivative(self, t: float) -> np.ndarray:
        """Get second derivative at parameter t"""
        pass
        
    def get_heading(self, t: float) -> float:
        """
        Get heading (angle) at parameter t
        
        Args:
            t: Parameter value
            
        Returns:
            float: Heading angle in radians
        """
        derivative = self.get_derivative(t)
        return np.arctan2(derivative[1], derivative[0])
        
    def get_curvature(self, t: float) -> float:
        """
        Get curvature at parameter t
        
        Args:
            t: Parameter value
            
        Returns:
            float: Curvature value
        """
        d1 = self.get_derivative(t)
        d2 = self.get_second_derivative(t)
        
        num = d1[0] * d2[1] - d1[1] * d2[0]
        den = (d1[0]**2 + d1[1]**2)**(3/2)
        
        if abs(den) < 1e-10:
            return 0.0
            
        return num / den
        
    def get_arc_length(self, t1: float, t2: float, segments: int = 100) -> float:
        """
        Calculate arc length between two parameter values
        
        Args:
            t1: Starting parameter
            t2: Ending parameter
            segments: Number of segments for numerical integration
            
        Returns:
            float: Arc length
        """
        cache_key = (int(t1 * 1000), int(t2 * 1000))
        if cache_key in self._length_cache:
            return self._length_cache[cache_key]
            
        t = np.linspace(t1, t2, segments)
        points = np.array([self.get_point(ti) for ti in t])
        diffs = np.diff(points, axis=0)
        lengths = np.sqrt(np.sum(diffs**2, axis=1))
        total_length = np.sum(lengths)
        
        self._length_cache[cache_key] = total_length
        return total_length
