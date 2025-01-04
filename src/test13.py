import numpy as np
import matplotlib.pyplot as plt

def get_basis_functions(t):
    t2 = t * t
    t3 = t2 * t
    t4 = t3 * t
    t5 = t4 * t
    
    H0 = 1 - 10*t3 + 15*t4 - 6*t5    # Position at start point
    H1 = 10*t3 - 15*t4 + 6*t5        # Position at end point
    H2 = t - 6*t3 + 8*t4 - 3*t5      # First derivative at start point
    H3 = -4*t3 + 7*t4 - 3*t5         # First derivative at end point
    H4 = 0.5*t2 - 1.5*t3 + 1.5*t4 - 0.5*t5  # Second derivative at start point
    H5 = 0.5*t3 - t4 + 0.5*t5        # Second derivative at end point
    
    return np.array([H0, H1, H2, H3, H4, H5])

def get_basis_derivatives(t):
    t2 = t * t
    t3 = t2 * t
    t4 = t3 * t
    
    H0_prime = -30*t2 + 60*t3 - 30*t4
    H1_prime = 30*t2 - 60*t3 + 30*t4
    H2_prime = 1 - 18*t2 + 32*t3 - 15*t4
    H3_prime = -12*t2 + 28*t3 - 15*t4
    H4_prime = t - 4.5*t2 + 6*t3 - 2.5*t4
    H5_prime = 1.5*t2 - 4*t3 + 2.5*t4
    
    return np.array([H0_prime, H1_prime, H2_prime, H3_prime, H4_prime, H5_prime])

def get_basis_second_derivatives(t):
    t2 = t * t
    t3 = t2 * t
    
    H0_double_prime = -60*t + 180*t2 - 120*t3
    H1_double_prime = 60*t - 180*t2 + 120*t3
    H2_double_prime = -36*t + 96*t2 - 60*t3
    H3_double_prime = -24*t + 84*t2 - 60*t3
    H4_double_prime = 1 - 9*t + 18*t2 - 10*t3
    H5_double_prime = 3*t - 12*t2 + 10*t3
    
    return np.array([H0_double_prime, H1_double_prime, H2_double_prime, 
                    H3_double_prime, H4_double_prime, H5_double_prime])

# Create parameter space
t = np.linspace(0, 1, 1000)

# Compute all basis functions and their derivatives
basis_funcs = np.array([get_basis_functions(ti) for ti in t])
basis_derivs = np.array([get_basis_derivatives(ti) for ti in t])
basis_second_derivs = np.array([get_basis_second_derivatives(ti) for ti in t])

# Create subplots
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 15))
fig.suptitle('Quintic Hermite Basis Functions and Their Derivatives', fontsize=16)

# Plot basis functions
labels = ['H₀ (Position Start)', 'H₁ (Position End)', 
          'H₂ (First Deriv Start)', 'H₃ (First Deriv End)',
          'H₄ (Second Deriv Start)', 'H₅ (Second Deriv End)']
colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']

for i in range(6):
    ax1.plot(t, basis_funcs[:, i], label=labels[i], color=colors[i])
ax1.set_title('Basis Functions')
ax1.set_xlabel('Parameter t')
ax1.set_ylabel('Value')
ax1.grid(True, alpha=0.3)
ax1.legend()

# Plot first derivatives
for i in range(6):
    ax2.plot(t, basis_derivs[:, i], label=f"{labels[i]}'", color=colors[i])
ax2.set_title('First Derivatives')
ax2.set_xlabel('Parameter t')
ax2.set_ylabel('Value')
ax2.grid(True, alpha=0.3)
ax2.legend()

# Plot second derivatives
for i in range(6):
    ax3.plot(t, basis_second_derivs[:, i], label=f"{labels[i]}''", color=colors[i])
ax3.set_title('Second Derivatives')
ax3.set_xlabel('Parameter t')
ax3.set_ylabel('Value')
ax3.grid(True, alpha=0.3)
ax3.legend()

plt.tight_layout()
plt.show()