import json
import threading
import time
from datetime import datetime
from pathlib import Path
import serial
from enum import Enum

import numpy as np
from casadi import *
import do_mpc
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt

should_exit = threading.Event()
status_lock = threading.Lock()
serial_lock = threading.Lock()

# Directory for log files
log_dir = Path(f"test_data/{datetime.now().strftime('%Y-%m-%d_%H:%M:%S')}")
log_dir.mkdir(exist_ok=True, parents=True)

control_pwm = 0
settle_time_s = 60
mpc_lookahead_s = 120
time_step_s = 1


class State(Enum):
    IDLE = 'idle'
    HEATING = 'heating'
    COOLING = 'cooling'
    FAULT = 'fault'


last_status = datetime.now()
status = {
    'temperature': 0,
    'state': State.IDLE,
    'pwm': 0,
    'door_open': False,
    'error': 0
}

reflow_curve = np.array([
    [90, 90],
    [180, 130],
    [210, 138],
    [240, 165]])

# add 30 seconds @ 35°C to the beginning of the reflow curve, shift the rest accordingly
reflow_curve = np.vstack((np.array([0, 35]), reflow_curve))
reflow_curve[:, 0] += settle_time_s
peak_temp = max(reflow_curve[:, 1])  # Define what the peak temperature should be

reflow_curve_function = interp1d(reflow_curve[:, 0], reflow_curve[:, 1], kind='linear', bounds_error=False,
                                 fill_value='extrapolate')

ui_heartbeat_interval_millis = 500

mpc_horizon = int(mpc_lookahead_s / time_step_s)

# Parameters for the 2nd order transfer function
k = 4.7875771211019  # 4.2266348441803645
omega = 0.005328475532226316
xi = 1.2586207495932575

# Serial port configuration
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)


def setup_model_and_mpc():
    model = do_mpc.model.Model('continuous')

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

    mpc = do_mpc.controller.MPC(model)
    mpc.settings.supress_ipopt_output()

    setup_mpc = {
        'n_horizon': mpc_horizon,
        't_step': time_step_s,
        # 'n_robust': 1,
        # 'store_full_solution': True,
    }
    mpc.set_param(**setup_mpc)

    tvp_template = mpc.get_tvp_template()

    def tvp_fun(t_now):
        for k in range(mpc_horizon):
            t = t_now + k * setup_mpc['t_step']
            tvp_template['_tvp', k, 'T_ref'] = reflow_curve_function(t)
        return tvp_template

    mpc.set_tvp_fun(tvp_fun)

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

    return model, mpc


model, mpc = setup_model_and_mpc()


def read_from_serial():
    while should_exit.is_set() is False:
        try:
            serial_lock.acquire()
            line = ser.readline().decode().strip()
            serial_lock.release()
            if line:  # Ignore empty lines
                print(line)  # Print to stdout
                data = json.loads(line)
                now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

                with status_lock:
                    if 'current' in data:
                        status['temperature'] = data['current']
                    if 'state' in data:
                        status['state'] = State(data['state'])
                    if 'top' in data:
                        status['pwm'] = data['top']
                    if 'bottom' in data:
                        status['pwm'] = data['bottom']
                    if 'door' in data:
                        status['door_open'] = data['door'] == 'open'
                    if 'error' in data:
                        status['error'] = data['error']
                    last_status = datetime.now()

                if 'current' in data:
                    log_file = log_dir / 'status.log'
                else:
                    log_file = log_dir / 'message.log'

                with log_file.open('a') as f:
                    f.write(f"{now} - {line}\n")

        except json.JSONDecodeError:
            print("Received non-JSON data")
        except Exception as e:
            print(f"Error: {e}")


def send_target_pwm():
    with serial_lock:
        ser.write(json.dumps({'top': control_pwm, 'bottom': control_pwm}).encode())


def send_state(state):
    with serial_lock:
        ser.write(json.dumps({'state': state.value}).encode())


def send_pwm_interval():
    # send the target pwm every interval
    while should_exit.is_set() is False:
        send_target_pwm()
        time.sleep(ui_heartbeat_interval_millis / 1000)


def run_curve():
    global control_pwm
    peak_hit = False
    send_state(State.HEATING)
    start_time = datetime.now()
    while should_exit.is_set() is False:
        duration = datetime.now() - start_time
        loop_start_time = time.time()
        if status['temperature'] >= peak_temp and not peak_hit:
            peak_hit = True
            print(f"Peak temperature of {peak_temp}°C reached at t={duration.seconds}s")
            print(f'Starting cooldown')
            send_state(State.COOLING)
        u0 = mpc.make_step(numpy.array(status['temperature']))
        if duration.seconds > reflow_curve[-1, 0] and peak_hit:
            u0 = np.array([[0]])
        control_pwm = u0[0, 0]
        print(f"t={duration.seconds}: {u0}")
        time.sleep(max(0, int(time_step_s - (time.time() - loop_start_time))))


def main():
    serial_status_thread = threading.Thread(target=read_from_serial)
    serial_status_thread.start()

    heartbeat_thread = threading.Thread(target=send_pwm_interval)
    heartbeat_thread.start()

    try:
        run_curve()
    except:
        print("Exiting...")
        send_state(State.IDLE)
        should_exit.set()
        heartbeat_thread.join()
        serial_status_thread.join()
        ser.close()


if __name__ == '__main__':
    main()
