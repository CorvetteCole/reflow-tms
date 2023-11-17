import serial
import json
import os
import threading
import time
from pathlib import Path
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from datetime import datetime
from scipy.interpolate import interp1d

# Serial port configuration
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

# Data points for temperature interpolation, with time in seconds
temperature_curve = [(0, 35), (90, 90), (180, 130), (210, 138), (240, 165), (270, 138)]
times, temperatures = zip(*temperature_curve)
interpolator = interp1d(times, temperatures, fill_value="extrapolate")

# Directory for log files
log_dir = Path(f"test_data/{datetime.now().strftime('%Y-%m-%d_%H:%M:%S')}")
log_dir.mkdir(exist_ok=True, parents=True)

# Thread-safe lists for graph
actual_temperatures = []
target_temperatures = []
time_data = []

should_exit = threading.Event()

# Lock for thread-safe operations on shared resources
data_lock = threading.Lock()

# Define a global variable for the maximum number of updates
MAX_UPDATES = int(times[-1] + 10)

# Define a global variable to indicate if we've started following the temperature curve
following_temperature_curve = False


# Function to read and save data from the serial port
def read_from_serial():
    global following_temperature_curve  # Allow the function to modify this global variable
    first_setpoint = temperatures[0]  # Get the first set point temperature from the curve
    while should_exit.is_set() is False:
        try:
            line = ser.readline().decode().strip()
            if line:  # Ignore empty lines
                print(line)  # Print to stdout
                data = json.loads(line)
                now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

                with data_lock:
                    if 'current' in data:
                        actual_temperatures.append(data['current'])
                        time_data.append(datetime.now().timestamp())
                        # Check if the current temperature exceeds first setpoint and thereby start the curve
                        if not following_temperature_curve and data['current'] >= first_setpoint:
                            following_temperature_curve = True

                    if 'target' in data:
                        target_temperatures.append(data['target'])

                if 'target' in data and 'current' in data:
                    log_file = log_dir / 'status.log'
                else:
                    log_file = log_dir / 'message.log'

                with log_file.open('a') as f:
                    f.write(f"{now} - {line}\n")

        except json.JSONDecodeError:
            print("Received non-JSON data")
        except Exception as e:
            print(f"Error: {e}")


# Function to update the live graph
def update_graph(i):
    plt.cla()
    with data_lock:
        plt.plot(time_data, actual_temperatures, label='Actual Temperature')
        plt.plot(time_data, target_temperatures, label='Target Temperature')

    plt.legend(loc='upper left')
    plt.xlabel('Time (s)')
    plt.ylabel('Temperature (°C)')
    plt.tight_layout()


# Function to control the starting of the temperature curve
def control_temperature_curve():
    global following_temperature_curve

    first_setpoint = temperatures[0]

    # Wait until the actual temperature reaches the first setpoint
    while not following_temperature_curve and should_exit.is_set() is False:
        should_exit.wait(5)

        # Send the first target temperature
        with data_lock:
            ser.write((json.dumps({"target": first_setpoint}) + '\n').encode())

    # Now start the main temperature control loop
    send_target_temperature(time.time())


# Function to send the target temperature after starting the curve
def send_target_temperature(start_time, interval=0.5):
    elapsed_time = 0
    while elapsed_time < times[
        -1] + interval and should_exit.is_set() is False:  # Continue until past the last time in the curve
        with data_lock:
            if len(actual_temperatures) > 0:
                current_time = elapsed_time
                target_temp = interpolator(current_time)
                # if we are decreasing the temperature, just set it low (but not 0 until the end)
                if target_temp < max(target_temperatures):
                    target_temp = 30

                command = {"target": int(target_temp)}
                ser.write((json.dumps(command) + '\n').encode())
        should_exit.wait(interval)
        elapsed_time = time.time() - start_time

    with data_lock:
        ser.write(b'{"target": 0}\n')  # Set target temperature to 0 after the curve is complete


# Main execution
def main():
    # Start the serial reading thread
    serial_thread = threading.Thread(target=read_from_serial)
    serial_thread.start()

    # Initialize the live graph
    plt.style.use('ggplot')  # Optional: for prettier graphs
    ani = FuncAnimation(plt.gcf(), update_graph, interval=1000, cache_frame_data=False)
    plt.tight_layout()

    # Note that we'll now call send_target_temperature in a thread
    # to allow the GUI to remain responsive.
    temperature_thread = threading.Thread(target=control_temperature_curve)
    temperature_thread.start()

    try:
        # Finally, we run the plot in the main thread to comply with Matplotlib and GUI requirements
        plt.show()
    except KeyboardInterrupt:
        # save plot to file
        plt.savefig(f'{log_dir}/plot.png')
        print('Exiting...')
        should_exit.set()
        temperature_thread.join()
        serial_thread.join()
        ser.close()


if __name__ == "__main__":
    main()
