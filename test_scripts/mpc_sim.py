import importlib.util
import numpy as np
from casadi import *
import do_mpc
import matplotlib.pyplot as plt
import matplotlib as mpl
from scipy.interpolate import interp1d
from do_mpc.data import save_results

# Define the reflow curve
reflow_curve = np.array([
    [90, 90],
    [180, 130],
    [210, 138],
    [240, 165]])
# reflow_curve = np.array([
#     [30, 100],
#     [120, 150],
#     [150, 183],
#     [210, 235]])

# add 30 seconds @ 35°C to the beginning of the reflow curve, shift the rest accordingly
reflow_curve = np.vstack((np.array([0, 35]), reflow_curve))
reflow_curve[:, 0] += 60

reflow_curve_function = interp1d(reflow_curve[:, 0], reflow_curve[:, 1], kind='linear', bounds_error=False,
                                 fill_value='extrapolate')

# Parameters for the 2nd order transfer function
k = 4.7875771211019  # 4.2266348441803645
omega = 0.005328475532226316
xi = 1.2586207495932575
theta = 0  # Assuming no time-delay (e^-theta s) for simplicity

# Set up the MPC problem
model_type = 'continuous'  # either 'discrete' or 'continuous'
model = do_mpc.model.Model(model_type)

# Define the states (temperature and its derivative)
T = model.set_variable(var_type='_x', var_name='T')
dT = model.set_variable(var_type='_x', var_name='dT')

# Define the input (heater PWM value)
u = model.set_variable(var_type='_u', var_name='u')

# Target temperature as time-varying parameter
T_ref = model.set_variable('_tvp', 'T_ref')

# Differential equations
a1 = SX(k * omega ** 2)
a2 = SX(2 * xi * omega)
a3 = SX(omega ** 2)

dT_next = a1 * u - a2 * dT - a3 * T
T_next = dT

# Set the differential equations
model.set_rhs('T', T_next)
model.set_rhs('dT', dT_next)

model.setup()

# Setup MPC
mpc = do_mpc.controller.MPC(model)
mpc.settings.supress_ipopt_output()

lookahead_s = 120
t_step = 1
n_horizon = int(lookahead_s / t_step)

setup_mpc = {
    'n_horizon': n_horizon,
    't_step': t_step,
    # 'n_robust': 1,
    # 'store_full_solution': True,
}
mpc.set_param(**setup_mpc)

tvp_template = mpc.get_tvp_template()


# Define the tvp function to return the correct setpoint based on the reflow curve function


def tvp_fun(t_now):
    for k in range(n_horizon):
        t = t_now + k * setup_mpc['t_step']
        if t > reflow_curve[-1, 0]:
            tvp_template['_tvp', k, 'T_ref'] = reflow_curve_function(reflow_curve[-1, 0])
        else:
            tvp_template['_tvp', k, 'T_ref'] = reflow_curve_function(t)

    return tvp_template


mpc.set_tvp_fun(tvp_fun)

# Define a high penalty for the peak temperature not being reached at the desired time:
peak_temp_penalty_weight = 1e6
peak_temp = max(reflow_curve[:, 1])  # Define what the peak temperature should be

# Penalty weights
P_T = 1e4  # Penalty weight for temperature deviation
P_u = 1e-8  # Penalty weight for control action

# Define the terminal cost (mterm) and Lagrange term (lterm)
# mterm = P_T * (T - T_ref)**2  # Terminal cost
lterm = P_T * (T - T_ref) ** 2 + P_u * u ** 2  # Running cost
mterm = P_T * (T - T_ref) ** 2 + P_T * (1 / (0.01 + casadi.fabs(T_ref - peak_temp))) * (T - peak_temp) ** 2

# Set the objective function terms in MPC
mpc.set_objective(mterm=mterm, lterm=mterm)
mpc.set_rterm(u=0.01)  # You may additionally use set_rterm() to penalize the control input rate of change

# Define the bounds for PWM
mpc.bounds['lower', '_u', 'u'] = 0
mpc.bounds['upper', '_u', 'u'] = 100

# Define the bounds for temperature
# mpc.bounds['lower', '_x', 'T'] = 0
mpc.bounds['upper', '_x', 'T'] = 270

mpc.setup()

# Setup Simulator
simulator = do_mpc.simulator.Simulator(model)
simulator.set_param(t_step=t_step)

sim_template = simulator.get_tvp_template()

initial_temperature = 22
initial_derivative = 0.0

mpc.x0['T'] = initial_temperature  # Set initial temperature for MPC
simulator.x0['T'] = initial_temperature  # Set initial temperature for the simulator
mpc.x0['dT'] = initial_derivative  # Set initial temperature derivative for MPC
simulator.x0['dT'] = initial_derivative  # Set initial temperature derivative for the simulator
simulator.reset_history()  # Reset any previous simulation history

mpc.set_initial_guess()  # Set the initial guess for the optimization problem based


def sim_tvp_fun(t_now):
    sim_template['T_ref'] = reflow_curve_function(t_now)
    return sim_template


simulator.set_tvp_fun(sim_tvp_fun)
simulator.setup()

# Initialize the states and inputs
mpc.x0['T'] = reflow_curve[0, 1]
mpc.x0['dT'] = 0
mpc.set_initial_guess()

# Running the MPC
extra_time_s = 120
n_steps = int((reflow_curve[-1, 0] + extra_time_s) / setup_mpc['t_step'])

# Set up the initial plot with dual y-axis for temperature and PWM signal.
plt.ion()  # Turn on interactive mode.
fig, ax1 = plt.subplots(figsize=(16, 9))

ax2 = ax1.twinx()  # Instantiate a second y-axis that shares the same x-axis.

# Plot the reflow curve on the primary y-axis.
ax1.plot(reflow_curve[:, 0], reflow_curve[:, 1], 'r--', label='Reflow Curve')
ax1.set_xlabel('Time [s]')
ax1.set_ylabel('Temperature [°C]', color='b')
ax1.tick_params(axis='y', labelcolor='b')

# Prepare the PWM axis (secondary y-axis).
ax2.set_ylabel('PWM [%]', color='g')
ax2.tick_params(axis='y', labelcolor='g')
ax2.set_ylim(0, 100)

# Initialize lines for temperature and control action.
line1, = ax1.plot([], [], 'b-', label='Temperature')
line2, = ax2.step([], [], 'g-', label='Control action', where='post')

plt.title('Reflow Oven Temperature Control')
fig.tight_layout()  # To ensure the right y-label is not clipped.

# Prepare legends separately for the two axes.
ax1.legend(loc='upper left')
ax2.legend(loc='upper right')

# Run the control loop and update the plot in each iteration.
U = []  # Initialize list for control actions.
peak_hit = False

for k in range(n_steps):
    t_now = k * setup_mpc['t_step']
    u_mpc = mpc.make_step(simulator.x0)
    if t_now > reflow_curve[-1, 0] and peak_hit:
        u_mpc = numpy.array([[0]])
    U.append(u_mpc[0, 0])  # Extract scalar value from u_mpc
    simulator.make_step(u_mpc)

    # Update the plot.
    mpc_time = np.array(mpc.data['_time'])
    mpc_temp = np.array(mpc.data['_x', 'T']).flatten()

    line1.set_data(mpc_time, mpc_temp)
    line2.set_data(mpc_time, U)

    if mpc_temp[-1] > peak_temp:
        peak_hit = True

    # Adjust plot limits.
    ax1.set_xlim(0, reflow_curve[-1, 0] + extra_time_s)
    ax1.set_ylim(min(reflow_curve[:, 1]) - 25, max(reflow_curve[:, 1]) + 10)

    ax2.set_ylim(0, 100)
    fig.canvas.draw()

    # Refresh the plot.
    # plt.draw()
    plt.pause(0.01)

# After loop completion, keep the figure open.
plt.ioff()  # Turn off interactive mode to keep the figure open.
plt.show()
