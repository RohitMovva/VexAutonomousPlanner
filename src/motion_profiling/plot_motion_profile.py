import matplotlib.pyplot as plt
import numpy as np

def plot_motion_profile(positions, velocities, accelerations, times):
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 15))
    fig.suptitle('Motion Profile', fontsize=16)

    # Position plot
    ax1.plot(times, positions, 'b-')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Position (ft)')
    ax1.set_title('Position vs Time')
    ax1.grid(True)

    # Velocity plot
    ax2.plot(times, velocities, 'g-')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Velocity (ft/s)')
    ax2.set_title('Velocity vs Time')
    ax2.grid(True)

    # Acceleration plot
    ax3.plot(times, accelerations, 'r-')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Acceleration (ft/s²)')
    ax3.set_title('Acceleration vs Time')
    ax3.grid(True)

    # Adjust layout and display the plot
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    # Example usage
    times = np.linspace(0, 10, 500)
    positions = np.sin(times)
    velocities = np.cos(times)
    accelerations = -np.sin(times)

    plot_motion_profile(positions, velocities, accelerations, times)