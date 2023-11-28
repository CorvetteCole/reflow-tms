import numpy as np
from scipy.signal import cont2discrete

pwm_bounds = (0, 100)  # pwm outputs
temperature_bounds = (20, 270)  # temperatures MPC controller is expected to stay between

# Given system parameters
k = 4.2266348441803645
ω = 0.005328475532226316
ξ = 1.2586207495932575
theta = 10  # Delay time in seconds

# Convert to discrete-time state-space representation
# Since the sampling time isn't defined, we assume Ts such that steps_delay is an integer
Ts = 0.5  # Example sampling interval in seconds, choose based on system dynamics
steps_delay = int(theta / Ts)  # Number of delay steps

# Original state-space model without delay
A_c = np.array([[0, 1], [-ω * ω, -2 * ξ * ω]])
B_c = np.array([[0], [k * ω * ω]])
C_c = np.array([[1, 0]])
D_c = np.array([[0]])

# Discrete-time state-space model without delay
A_d, B_d, C_d, D_d, dt = cont2discrete((A_c, B_c, C_c, D_c), Ts)

# Extend the A matrix to include the states for delay
A_aug = np.eye(A_d.shape[0] + steps_delay)
A_aug[:A_d.shape[0], :A_d.shape[1]] = A_d
for i in range(steps_delay - 1):
    A_aug[A_d.shape[0] + i, A_d.shape[0] + i + 1] = 1

# Extend the B matrix to include the new inputs into the augmented states
B_aug = np.zeros((A_d.shape[0] + steps_delay, B_d.shape[1]))
B_aug[:B_d.shape[0], :B_d.shape[1]] = B_d
B_aug[A_d.shape[0], 0] = 1  # First fake state gets the input


# Higher weight on the first state, lower on the delay states
Q_aug = np.diag([10] + [1]*(A_d.shape[0] - 1) + [0.1]*steps_delay)
R_aug = np.eye(1)
# Penalty on control usage
# R_aug = np.array([[10]])

# Input constraint matrices
A_u_aug = np.array([[1], [-1]])
b_u_aug = np.array([pwm_bounds[1], pwm_bounds[0]])  # Bounds for the input (0 to 100)

# State constraint matrices
# We'll assume the first state corresponds to the output 'y' for illustrative purposes
A_x_aug = np.array([[1, 0] + [0] * steps_delay, [-1, 0] + [0] * steps_delay])
b_x_aug = np.array([temperature_bounds[1], -temperature_bounds[0]])  # Bounds for the state/output (20 to 250)

# Parameters for Python MPC helper
# N = 20  # Prediction horizon
# import LinearMPCFactor as lMf
# mpc_factory = lMf.LinearMPCFactor(A_aug, B_aug, Q_aug, R_aug, N, A_x_aug, b_x_aug, A_u_aug, b_u_aug)

# MPC initialization parameters
# e_V = 0.001  # Convergence error tolerance
# e_g = 0.001  # Constraint violation tolerance
# max_iter = 1000  # Maximum solver iterations

# Generate C++ initialization code
# mpc_factory.PrintCppCode(e_V, e_g, max_iter)