import numpy as np
import matplotlib.pyplot as plt

def generate_path(t):
    # Example path: a figure-eight
    x = np.sin(t)
    y = np.sin(t) * np.cos(t)
    return x, y

def calculate_curvature(x, y, dx, dy, ddx, ddy):
    numerator = np.abs(dx * ddy - dy * ddx)
    denominator = (dx**2 + dy**2)**(3/2)
    return numerator / denominator

def limit_velocity(curvature, max_lateral_accel):
    return np.sqrt(max_lateral_accel / np.abs(curvature))

def generate_s_curve_profile(distance, v_max, a_max, j_max):
    # S-curve profile generation
    t_j = a_max / j_max
    t_a = v_max / a_max
    
    if t_a < 2 * t_j:
        t_j = t_a / 2
        a_max = v_max / (2 * t_j)
    
    d_j = j_max * t_j**3 / 6
    d_a = a_max * t_j**2 / 2 + a_max * (t_a - t_j) * t_j
    
    if distance < 4 * d_j + 2 * d_a:
        # Recalculate for shorter distances
        t_a = np.sqrt(distance / a_max)
        v_max = a_max * t_a / 2
        t_j = t_a / 2
    
    t_c = (distance - 4 * d_j - 2 * d_a) / v_max
    
    return t_j, t_a, t_c, v_max, a_max

def main():
    # Generate path
    t = np.linspace(0, 2*np.pi, 1000)
    x, y = generate_path(t)
    
    # Calculate derivatives
    dx = np.gradient(x, t)
    dy = np.gradient(y, t)
    ddx = np.gradient(dx, t)
    ddy = np.gradient(dy, t)
    
    # Calculate curvature
    curvature = calculate_curvature(x, y, dx, dy, ddx, ddy)
    
    # Limit velocity based on curvature
    max_lateral_accel = 1.0  # adjust as needed
    v_max_curvature = limit_velocity(curvature, max_lateral_accel)
    
    # Calculate path length
    path_length = np.sum(np.sqrt(np.diff(x)**2 + np.diff(y)**2))
    
    # Generate S-curve profile
    v_max = np.min(v_max_curvature)  # Use the minimum allowed velocity
    a_max = 2.0  # adjust as needed
    j_max = 5.0  # adjust as needed
    
    t_j, t_a, t_c, v_max, a_max = generate_s_curve_profile(path_length, v_max, a_max, j_max)
    
    # Plot results
    plt.figure(figsize=(12, 8))
    
    plt.subplot(2, 2, 1)
    plt.plot(x, y)
    plt.title('Path')
    plt.xlabel('X')
    plt.ylabel('Y')
    
    plt.subplot(2, 2, 2)
    plt.plot(t, curvature)
    plt.title('Curvature')
    plt.xlabel('t')
    plt.ylabel('Curvature')
    
    plt.subplot(2, 2, 3)
    plt.plot(t, v_max_curvature)
    plt.title('Max Velocity (Curvature Limited)')
    plt.xlabel('t')
    plt.ylabel('Velocity')
    
    plt.subplot(2, 2, 4)
    t_profile = np.linspace(0, 2*(t_j + t_a) + t_c, 1000)
    v_profile = np.piecewise(t_profile, 
                             [t_profile < t_j, 
                              (t_profile >= t_j) & (t_profile < t_a), 
                              (t_profile >= t_a) & (t_profile < t_a + t_c),
                              (t_profile >= t_a + t_c) & (t_profile < t_a + t_c + t_j),
                              (t_profile >= t_a + t_c + t_j) & (t_profile < 2*t_a + t_c),
                              t_profile >= 2*t_a + t_c],
                             [lambda t: (j_max * t**2) / 2,
                              lambda t: v_max - a_max * (t_a - t)**2 / (2 * t_j),
                              lambda t: v_max,
                              lambda t: v_max - (j_max * (t - t_a - t_c)**2) / 2,
                              lambda t: a_max * (2*t_a + t_c - t)**2 / (2 * t_j),
                              0])
    plt.plot(t_profile, v_profile)
    plt.title('S-curve Velocity Profile')
    plt.xlabel('Time')
    plt.ylabel('Velocity')
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()