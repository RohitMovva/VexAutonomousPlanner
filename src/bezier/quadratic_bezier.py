import numpy as np


def point_to_array(p):
    return np.array([p.x(), p.y()])


def calculate_curvature(first_derivative, second_derivative):
    cross_product = np.cross(first_derivative, second_derivative)
    return np.linalg.norm(cross_product) / (np.linalg.norm(first_derivative) ** 3)


def quadratic_bezier_point(p0, p1, p2, t):
    return (
        (1 - t) ** 2 * point_to_array(p0)
        + 2 * (1 - t) * t * point_to_array(p1)
        + t**2 * point_to_array(p2)
    )


def quadratic_bezier_derivative(p0, p1, p2, t):
    return 2 * (
        (1 - t) * (point_to_array(p1) - point_to_array(p0))
        + t * (point_to_array(p2) - point_to_array(p1))
    )


def quadratic_bezier_second_derivative(p0, p1, p2):
    return 2 * (point_to_array(p2) - 2 * point_to_array(p1) + point_to_array(p0))


def quadratic_bezier_curvature(p0, p1, p2, t):
    first_derivative = quadratic_bezier_derivative(p0, p1, p2, t)
    second_derivative = quadratic_bezier_second_derivative(p0, p1, p2)
    return calculate_curvature(first_derivative, second_derivative)


def quad_bezier_angle(t, P0, P1, P2):
    tangentX = (2 * (1 - t) * (P1.x() - P0.x())) + (2 * t * (P2.x() - P1.x()))
    tangentY = (2 * (1 - t) * (P1.y() - P0.y())) + (2 * t * (P2.y() - P1.y()))
    return -1 * np.arctan2(tangentY, tangentX) * (180 / np.pi)
