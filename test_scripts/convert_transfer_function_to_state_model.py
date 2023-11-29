import numpy as np
from scipy.signal import cont2discrete
import LinearMPCFactor as lMf
import control

pwm_bounds = (0, 100)  # pwm outputs
temperature_bounds = (-20, 270)  # temperatures MPC controller is expected to stay between

# Given system parameters
k = 4.7875771211019
ω = 0.0027731675792143345
ξ = 1.54264888649055
theta = 22.912482438708693  # Delay time in seconds

# Sampling time
Ts = 0.1  # Choose a sampling time appropriate for your system

num = [0, 0, k * ω ** 2]
den = [1, 2 * ξ * ω, ω ** 2]  # Coefficients for the denominator (poles)

sys_tf = control.TransferFunction(num, den)

# convert to state-space
sys_ss = control.tf2ss(sys_tf)

# Discretize the state-space system
sys_ss_d = sys_ss.sample(Ts, method='zoh')
A_d = sys_ss_d.A
B_d = sys_ss_d.B

Q = np.eye(A_d.shape[0])  # cost function Q, which determines the convergence rate of the state
R = np.eye(B_d.shape[1])  # cost function R, which determines the convergence rate of the input

# State constraint matrices
A_x = np.array([[1, 0], [-1, 0]])  # state constraints A_x @ x_k <= b_x
b_x = np.array([temperature_bounds[1], -temperature_bounds[0]])

# Input constraint matrices
A_u = np.array([[1], [-1]])  # input constraints A_u @ u_k <= b_u
b_u = np.array([pwm_bounds[1], pwm_bounds[0]])  # Bounds for the input (0 to 100)

N = 100  # Prediction horizon

mpc = lMf.LinearMPCFactor(A_d, B_d, Q, R, N, A_x, b_x, A_u, b_u)  # print the

# Tolerances and maximum iterations
e_V = 0.001  # tolerance of the error between optimal cost and real cost
e_g = 0.001  # tolerance of the violation of constraints

max_iter = 1000  # maximum steps of the solver

mpc.PrintCppCode(e_V, e_g, max_iter)

#######


# N = 20  # Prediction horizon
#
# # State-space model
# A = np.array([[0, 1], [-ω * ω, -2 * ξ * ω]])
# B = np.array([[0], [k * ω * ω]])  # state space equation B
#
# # Cost function
# #Q = np.array([[1, 0], [0, 3]])  # cost function Q, which determines the convergence rate of the state
# Q = np.diag([1] + [1] * (A.shape[0] - 1))
# R = np.array([[1]])  # cost function R, which determines the convergence rate of the input
#
# # Constraints
# A_x = np.array([[1, 0], [-1, 0]])  # state constraints A_x @ x_k <= b_x
# b_x = np.array([temperature_bounds[1], -temperature_bounds[0]])
#
# A_u = np.array([[1], [-1]])  # input constraints A_u @ u_k <= b_u
# b_u = np.array([pwm_bounds[1], pwm_bounds[0]])  # Bounds for the input (0 to 100)
#
# mpc = lMf.LinearMPCFactor(A, B, Q, R, N, A_x, b_x, A_u, b_u)  # print the
# # Tolerances and maximum iterations
# e_V = 0.1  # tolerance of the error between optimal cost and real cost
# e_g = 0.1  # tolerance of the violation of constraints
# max_iter = 1  # maximum steps of the solver
#
# mpc.PrintCppCode(e_V, e_g, max_iter)


##############3


# # Convert to discrete-time state-space representation
# # Since the sampling time isn't defined, we assume Ts such that steps_delay is an integer
# Ts = 0.5  # Example sampling interval in seconds, choose based on system dynamics
# steps_delay = int(theta / Ts)  # Number of delay steps

# # Original state-space model without delay
# A_c = np.array([[0, 1], [-ω * ω, -2 * ξ * ω]])
# B_c = np.array([[0], [k * ω * ω]])
# C_c = np.array([[1, 0]])
# D_c = np.array([[0]])
#
# # Discrete-time state-space model without delay
# A_d, B_d, C_d, D_d, dt = cont2discrete((A_c, B_c, C_c, D_c), Ts)
#
# # Extend the A matrix to include the states for delay
# A_aug = np.eye(A_d.shape[0] + steps_delay)
# A_aug[:A_d.shape[0], :A_d.shape[1]] = A_d
# for i in range(steps_delay - 1):
#     A_aug[A_d.shape[0] + i, A_d.shape[0] + i + 1] = 1
#
# # Extend the B matrix to include the new inputs into the augmented states
# B_aug = np.zeros((A_d.shape[0] + steps_delay, B_d.shape[1]))
# B_aug[:B_d.shape[0], :B_d.shape[1]] = B_d
# B_aug[A_d.shape[0], 0] = 1  # First fake state gets the input
#
# # Higher weight on the first state, lower on the delay states
# Q_aug = np.diag([10] + [1] * (A_d.shape[0] - 1) + [0.1] * steps_delay)
# R_aug = np.array([[1]])
# # Penalty on control usage
# # R_aug = np.array([[10]])
#
# # Input constraint matrices
# A_u_aug = np.array([[1], [-1]])
# b_u_aug = np.array([pwm_bounds[1], pwm_bounds[0]])  # Bounds for the input (0 to 100)
#
# # State constraint matrices
# # We'll assume the first state corresponds to the output 'y' for illustrative purposes
# A_x_aug = np.array([[1, 0] + [0] * steps_delay, [-1, 0] + [0] * steps_delay])
# b_x_aug = np.array([temperature_bounds[1], -temperature_bounds[0]])  # Bounds for the state/output (20 to 250)


##################333


#
# # Parameters for Python MPC helper
# N = 20  # Prediction horizon
# mpc_factory = lMf.LinearMPCFactor(A_aug, B_aug, Q_aug, R_aug, N, A_x_aug, b_x_aug, A_u_aug, b_u_aug)
#
# # MPC initialization parameters
# e_V = 0.001  # Convergence error tolerance
# e_g = 0.001  # Constraint violation tolerance
# max_iter = 1000  # Maximum solver iterations
#
# # Generate C++ initialization code
# mpc_factory.PrintCppCode(e_V, e_g, max_iter)
