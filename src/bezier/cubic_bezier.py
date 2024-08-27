import numpy as np

def point_to_array(p):
    return np.array([p.x(), p.y()])

def calculate_curvature(first_derivative, second_derivative):
    cross_product = np.cross(first_derivative, second_derivative)
    return np.linalg.norm(cross_product) / (np.linalg.norm(first_derivative) ** 3)

def cubic_bezier_point(p0, p1, p2, p3, t):
    return (1-t)**3 * point_to_array(p0) + 3*(1-t)**2*t * point_to_array(p1) + 3*(1-t)*t**2 * point_to_array(p2) + t**3 * point_to_array(p3)

def cubic_bezier_derivative(p0, p1, p2, p3, t):
    return 3 * ((1-t)**2*(point_to_array(p1)-point_to_array(p0)) + 2*(1-t)*t*(point_to_array(p2)-point_to_array(p1)) + t**2*(point_to_array(p3)-point_to_array(p2)))

def cubic_bezier_second_derivative(p0, p1, p2, p3, t):
    return 6 * ((1-t)*(point_to_array(p2)-2*point_to_array(p1)+point_to_array(p0)) + t*(point_to_array(p3)-2*point_to_array(p2)+point_to_array(p1)))

def calculate_curvature(first_derivative, second_derivative):
    cross_product = np.cross(first_derivative, second_derivative)
    return np.linalg.norm(cross_product) / (np.linalg.norm(first_derivative) ** 3)

def cubic_bezier_curvature(p0, p1, p2, p3, t):
    first_derivative = cubic_bezier_derivative(p0, p1, p2, p3, t)
    second_derivative = cubic_bezier_second_derivative(p0, p1, p2, p3, t)
    return calculate_curvature(first_derivative, second_derivative)

def cubic_bezier_point(P0, P1, P2, P3, t):
    x = (1 - t)**3 * P0.x() + 3 * (1 - t)**2 * t * P1.x() + 3 * (1 - t) * t**2 * P2.x() + t**3 * P3.x()
    y = (1 - t)**3 * P0.y() + 3 * (1 - t)**2 * t * P1.y() + 3 * (1 - t) * t**2 * P2.y() + t**3 * P3.y()
    return x, y

# 3(1- t)^2(P1 - P0) + 6(1 - t)t(P2 - P1) + 3t^2(P3 - P2)
def cubic_bezier_angle(t, P0, P1, P2, P3):
    tangentX = 3*(1-t)**2*(P1.x()-P0.x()) + 6*(1-t)*t*(P2.x()-P1.x()) + 3*t**2*(P3.x()-P2.x())
    tangentY = 3*(1-t)**2*(P1.y()-P0.y()) + 6*(1-t)*t*(P2.y()-P1.y()) + 3*t**2*(P3.y()-P2.y())
    return -1*np.arctan2(tangentY, tangentX)*(180/np.pi)