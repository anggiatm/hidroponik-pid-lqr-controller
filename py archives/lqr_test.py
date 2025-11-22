import numpy as np
from scipy.linalg import solve_continuous_are

# Define system dynamics
A = np.array([[1, 0], [0, 1]])  # Assuming a simple identity matrix, update based on actual dynamics
B = np.array([[0.1, 0, 0, 0], [0, -0.1, 0, 0]])  # Update based on relationships mentioned

# Q and R matrices for the LQR controller
Q = np.diag([1, 1])  # Adjust weights for state variables
R = np.diag([1, 1, 1, 1])  # Adjust weights for control inputs

# Solve the continuous-time algebraic Riccati equation
P = solve_continuous_are(A, B, Q, R)

# Compute the LQR gain matrix K
K = np.linalg.inv(R) @ B.T @ P

print("LQR Gain Matrix (K):")
print(K)
