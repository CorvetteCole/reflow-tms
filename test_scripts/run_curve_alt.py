import serial
import json
import os
import threading
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from datetime import datetime
from scipy.interpolate import interp1d

# Serial port configuration
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

# Data points for temperature interpolation, with time in seconds
temperature_curve = [(0, 25), (90, 90), (180, 130), (210, 138), (240, 165), (270, 138)]
times, temperatures = zip(*temperature_curve)
interpolator = interp1d(times, temperatures, fill_value="extrapolate")

# Directory for log files
log_dir = "test_data"
if not os.path.exists(log_dir):
    os.mkdir(log_dir)

# Thread-safe lists for graph
actual_temperatures = []
target_temperatures = []
time_data = []

# Lock for thread-safe operations on shared resources
data_lock = threading.Lock()


# Function to read and save data from the serial port
def read_from_serial():
    while True:
        try:
            line = ser.readline().decode().strip()
            if line:  # Ignore empty lines
                data = json.loads(line)
                now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

                with data_lock:
                    if 'current' in data and 'target' in data:
                        actual_temperatures.append(data['current'])
                        target_temperatures.append(data['target'])
                        time_data.append(datetime.now().timestamp())

                    if 'state' in data and data['state'] == 'fault':
                        continue  # Skip sending target temperatures if in 'fault' state

                if 'target' in data and 'current' in data:
                    log_filename = f"{log_dir}/status_data.log"
                else:
                    log_filename = f"{log_dir}/log_messages.log"

                with open(log_filename, 'a') as f:
                    f.write(f"{now} - {line}\n")

        except json.JSONDecodeError:
            print("Received non-JSON data")


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


# Function to send the target temperature
def send_target_temperature(start_time, interval=0.5):
    elapsed_time = 0
    while elapsed_time < times[-1] + interval:  # Continue until past the last time in the curve
        with data_lock:
            if len(actual_temperatures) > 0 and actual_temperatures[-1] != temperatures[-1]:
                current_time = elapsed_time
                target_temp = interpolator(current_time)
                command = {"target": int(target_temp)}
                ser.write((json.dumps(command) + '\n').encode())
        time.sleep(interval)
        elapsed_time = time.time() - start_time

    with data_lock:
        ser.write(b'{"target": 0}\n')  # Set target temperature to 0 after the curve is complete


# Main execution
def main():
    # Start the serial reading thread
    serial_thread = threading.Thread(target=read_from_serial, daemon=True)
    serial_thread.start()

    # Initialize and start the live graph
    plt.style.use('ggplot')  # Optional: for prettier graphs
    ani = FuncAnimation(plt.gcf(), update_graph, interval=1000)
    plt.tight_layout()
    graphing_thread = threading.Thread(target=plt.show, daemon=True)
    graphing_thread.start()

    # Start to send target temperatures
    start_time = time.time()
    send_target_temperature(start_time)


if __name__ == "__main__":
    main()
