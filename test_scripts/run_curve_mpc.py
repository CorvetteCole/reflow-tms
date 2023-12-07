import importlib.util
import json
import multiprocessing
import threading
import time
from datetime import datetime, timedelta
from pathlib import Path
import serial
from enum import Enum
import traceback

from casadi import *
import do_mpc
from scipy.interpolate import interp1d

should_exit = multiprocessing.Event()
status_lock = multiprocessing.Lock()
serial_lock = multiprocessing.Lock()

# Directory for log files
log_dir = Path(f"test_data/mpc_{datetime.now().strftime('%Y-%m-%d_%H:%M:%S')}")
log_dir.mkdir(exist_ok=True, parents=True)

mgr = multiprocessing.Manager()

control_pwm = mgr.Value('i', 0)
control_state = mgr.Value('i', 0)

settle_time_s = 60
mpc_lookahead_s = 120
time_step_s = 1


class State(Enum):
    IDLE = 0
    HEATING = 1
    COOLING = 2
    FAULT = 3

    # from string
    @classmethod
    def from_string(cls, s):
        # case-insensitive string of name
        try:
            return cls[s.upper()]
        except KeyError:
            raise ValueError()


# array of (time, temperature), used for dT
temperature_data = mgr.list()
temperature_derivative_timescale = timedelta(seconds=1)

# convert status dictionary to a shared dictionary with Manager
status = mgr.dict({
    'temperature': 0,
    'state': State.IDLE.value,
    'pwm': 0,
    'door_open': False,
    'error': 0
})

reflow_curve = np.array([
    [90, 90],
    [180, 130],
    [210, 138],
    [240, 165]])

end_temperatue = 138

# add 30 seconds @ 35°C to the beginning of the reflow curve, shift the rest accordingly
reflow_curve = np.vstack((np.array([0, 35]), reflow_curve))
reflow_curve[:, 0] += settle_time_s
peak_temp = max(reflow_curve[:, 1])  # Define what the peak temperature should be

reflow_curve_function = interp1d(reflow_curve[:, 0], reflow_curve[:, 1], kind='linear', bounds_error=False,
                                 fill_value='extrapolate')

ui_heartbeat_interval_millis = 500

mpc_horizon = int(mpc_lookahead_s / time_step_s)


def setup_model_and_mpc():
    # Parameters for the 2nd order transfer function
    k = 4.7875771211019  # 4.2266348441803645
    omega = 0.005328475532226316
    xi = 1.2586207495932575

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


def process_serial_line(line, temperature_data, status):
    try:
        data = json.loads(line)
        now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        if 'current' in data:
            status['temperature'] = data['current']
            temperature_data.append((datetime.now(), data['current']))
        if 'state' in data:
            status['state'] = State.from_string(data['state'])
        if 'top' in data:
            status['pwm'] = data['top']
        if 'bottom' in data:
            status['pwm'] = data['bottom']
        if 'door' in data:
            status['door_open'] = data['door'] == 'open'
        if 'error' in data:
            status['error'] = data['error']

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


def handle_communication(should_exit, temperature_data, status, control_pwm, control_state):
    with serial.Serial('/dev/ttyUSB0', 115200, timeout=1) as ser:
        last_sent_time = time.monotonic()
        while not should_exit.is_set():
            # Read from serial
            line = ser.readline().decode().strip()
            if line:  # Ignore empty lines
                process_serial_line(line, temperature_data, status)
            current_time = time.monotonic()
            if current_time - last_sent_time > ui_heartbeat_interval_millis / 1000:
                # Write to serial if PWM value changed or timeout happened
                control_state_enum = State(control_state.value)
                if control_state != status['state']:
                    print(f"Sending new state {control_state_enum.name}")
                    ser.write(json.dumps({'state': control_state_enum.name}).encode())
                if control_pwm.value != status['pwm']:
                    print(f"Sending new pwm {control_pwm.value}")
                ser.write(json.dumps(
                    {'top': control_pwm.value, 'bottom': control_pwm.value}).encode())
                last_sent_time = current_time


def calculate_temperature_derivative(temperature_data):
    one_second_ago = datetime.now() - temperature_derivative_timescale

    temperature_data = [data for data in temperature_data if data[0] > one_second_ago]

    if len(temperature_data) < 2:
        return 0

        # Calculate the differences and average them.
    diffs = []
    for i in range(1, len(temperature_data)):
        time_diff = (temperature_data[i][0] - temperature_data[i - 1][0]).total_seconds()
        temp_diff = temperature_data[i][1] - temperature_data[i - 1][1]
        diffs.append(temp_diff / time_diff)

    return sum(diffs) / len(diffs)


def get_current_state(temperature_data):
    return np.array([[status['temperature']], [calculate_temperature_derivative(temperature_data)]])


def run_curve():
    global control_pwm, control_state, temperature_data
    peak_hit = False
    control_state = State.HEATING
    mpc.x0['T'] = status['temperature']
    mpc.x0['dT'] = calculate_temperature_derivative(temperature_data)
    mpc.set_initial_guess()
    start_time = datetime.now()
    while should_exit.is_set() is False:
        duration = datetime.now() - start_time
        loop_start_time = time.time()
        if status['temperature'] >= peak_temp and not peak_hit:
            peak_hit = True
            print(f"Peak temperature of {peak_temp}°C reached at t={duration.seconds}s")
            print(f'Starting cooldown')
            control_state = State.COOLING

        if status['temperature'] <= end_temperatue and peak_hit:
            print(f"End temperature of {end_temperatue}°C reached at t={duration.seconds}s")
            print(f'Ending reflow curve')
            control_state = State.IDLE
            break

        x0 = np.array([[status['temperature']], [calculate_temperature_derivative(temperature_data)]])
        u0 = mpc.make_step(x0)
        print(f"t={duration.seconds} x0: {x0}")
        if duration.seconds > reflow_curve[-1, 0] and peak_hit:
            u0 = np.array([[0]])
        # clamp to 0-100 integer
        control_pwm = int(np.clip(u0[0, 0], 0, 100))
        print(f"t={duration.seconds}: {u0}")
        time.sleep(max(0, int(time_step_s - (time.time() - loop_start_time))))


def main():
    global control_pwm, control_state, temperature_data, should_exit, status

    serial_process = multiprocessing.Process(target=handle_communication, args=(should_exit, temperature_data, status,
                                                                                control_pwm, control_state))
    serial_process.start()

    print('Waiting for 5 seconds')
    time.sleep(5)

    try:
        run_curve()
    except Exception as e:
        print(f'Caught exception {e}')
        # print stack trace
        traceback.print_exc()
        print("Exiting...")
    finally:
        control_state = State.IDLE
        should_exit.set()
        serial_process.join()


if __name__ == '__main__':
    main()
