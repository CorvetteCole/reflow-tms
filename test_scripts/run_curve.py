import json
import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import time
import os


# Function to create the temperature profile
def create_temp_profile(time_temp_pairs, current_time):
    times, temps = zip(*time_temp_pairs)
    current_temp = np.interp(current_time, times, temps)
    return current_temp


# Open serial connection
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

# Make sure test_data directory exists
os.makedirs("test_data", exist_ok=True)

# Open log files
status_log_file = open("test_data/status_log.txt", "a")
message_log_file = open("test_data/message_log.txt", "a")

# Temperature profile - list of tuples (time since start in seconds, temperature)
temperature_profile = [(0, 25), (90, 90), (180, 130), (210, 138), (240, 165), (270, 138), (300, 0)]

# Initialize matplotlib for live plotting
plt.ion()
fig, ax = plt.subplots()
x_data, y_data = [], []
line, = plt.plot_date(x_data, y_data, '-')

plt.xlabel('Time (s)')
plt.ylabel('Temperature (°C)')
plt.title('Temperature Profile')

start_time = time.time()


def update_graph(frame):
    current_time = time.time() - start_time
    target = create_temp_profile(temperature_profile, current_time)

    # Read and process serial data
    if ser.in_waiting:
        line = ser.readline().decode('utf-8').strip()
        try:
            data = json.loads(line)
            if 'current' in data and 'target' in data:
                status_log_file.write(line + "\n")
                x_data.append(current_time)
                y_data.append(data['current'])
                line.set_xdata(x_data)
                line.set_ydata(y_data)
                ax.relim()
                ax.autoscale_view()
                fig.canvas.draw()
                fig.canvas.flush_events()

                if data['state'] != 'fault':
                    command = json.dumps({'target': target})
                    ser.write(command.encode('utf-8'))

            elif 'time' in data and 'message' in data:
                message_log_file.write(line + "\n")

        except json.JSONDecodeError:
            pass  # In a real scenario, you would handle parsing errors here


ani = FuncAnimation(fig, update_graph, interval=500)

# Run the program indefinitely
try:
    plt.show()
except KeyboardInterrupt:
    print('Exiting...')

# Close log files and serial port on exit
status_log_file.close()
message_log_file.close()
ser.close()
