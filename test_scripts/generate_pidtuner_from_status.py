"""
A file contains lines that are either of "status" or "logging" type. Every line contains a valid JSON object. A status
line might look like 2023-11-19 00:01:41 - {"target":0,"current":134.1619,"state":"idle","top":0,"bottom":0,"door":"closed","error":0}. Where a
logging line might look like: 2023-11-19 00:02:41 - {"time":605085,"severity":"INFO","message":"Finished step response"}

Read this file in, and generate a new CSV file. The first column should be the time. The status messages are sent every
500ms, so start at 0 and increment by 0.5. The second column should be the current duty cycle. Top and bottom are
always the same, so just use the top duty cycle. The third column should be the value represented by "current".
"""
import json

if __name__ == "__main__":
    # Read the file
    with open("status.log", "r") as f:
        lines = f.readlines()

    # Parse the lines
    data = []
    for line in lines:
        # Parse the line as JSON
        try:
            line = json.loads(line.split(" - ")[1])
        except json.JSONDecodeError:
            continue

        # If the line is a status message, add it to the data
        if "current" in line:
            data.append(line)

    # Generate the CSV file
    with open("status.csv", "w") as f:
        f.write("time,duty_cycle,current\n")
        for i, line in enumerate(data):
            if line["door"] == "open":
                pass
                # f.write(f"{i * 0.5},{-1},{line['current']}\n")
            else:
                f.write(f"{i * 0.5},{line['top']},{line['current']}\n")
