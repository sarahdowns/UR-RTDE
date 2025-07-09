# This code works together with the rtde_control_loop.urp file running on the teach 
# pendant. Run this example code first then the teach pendant to make the robot move
# to between 3 defined points. CSV file is stored in current directory .csv
# Then the TCP z position will be plotted after the finishing the program is ended. 

import sys
import logging
import csv
import time
import matplotlib.pyplot as plt  # For plotting
from datetime import datetime

import rtde.rtde as rtde
import rtde.rtde_config as rtde_config

ROBOT_HOST = "192.168.5.4"
ROBOT_PORT = 30004
config_filename = "control_loop_configuration.xml"

conf = rtde_config.ConfigFile(config_filename)
state_names, state_types = conf.get_recipe("state")
setp_names, setp_types = conf.get_recipe("setp")
watchdog_names, watchdog_types = conf.get_recipe("watchdog")

con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
con.connect()
con.get_controller_version()
con.send_output_setup(state_names, state_types)
setp = con.send_input_setup(setp_names, setp_types)
watchdog = con.send_input_setup(watchdog_names, watchdog_types)

setp1 = [-0.2495, 0.1487, 0.5278, -1.3069, 0.2661, 0.8860] 
setp2 = [-0.1002, 0.1487, 0.6546, -1.3069, 0.2661, 0.8861]    
setp3 = [-0.1640, 0.1487, 0.6255, -1.5306, -0.2770, 0.3898]
setpoints = [setp1, setp2, setp3]

# Helper
def list_to_setp(sp, lst):
    for i in range(6):
        sp.__dict__[f"input_double_register_{i}"] = lst[i]
    return sp

# Init watchdog
watchdog.input_int_register_0 = 0
list_to_setp(setp, setp1)

# Start RTDE
if not con.send_start():
    sys.exit("Failed to start RTDE connection")

# For logging data
log_data = []
start_time = time.time()

# Control loop
current_setpoint_index = 0
move_completed = True
print("Starting robot movement and logging...")

try:
    while True:
        state = con.receive()
        if state is None:
            print("Lost connection.")
            break

        timestamp = time.time() - start_time

        tcp = state.actual_TCP_pose if hasattr(state, 'actual_TCP_pose') else [0]*6
        tcp_force = state.actual_TCP_force if hasattr(state, 'actual_TCP_force') else [0]*6
        joint_torque = state.target_moment if hasattr(state, 'target_moment') else [0]*6
        joints = state.actual_q if hasattr(state, 'actual_q') else [0]*6

        log_data.append([timestamp] + tcp + joints + tcp_force + joint_torque)


        # Switch setpoints based on completion
        if move_completed and state.output_int_register_0 == 1:
            move_completed = False
            current_setpoint_index = (current_setpoint_index + 1) % len(setpoints)
            new_setp = setpoints[current_setpoint_index]
            list_to_setp(setp, new_setp)
            print(f"Sending new pose: {new_setp}")
            con.send(setp)
            watchdog.input_int_register_0 = 1

        elif not move_completed and state.output_int_register_0 == 0:
            print(f"Reached target. Joint pos: {state.target_q}")
            move_completed = True
            watchdog.input_int_register_0 = 0

        con.send(watchdog)

except KeyboardInterrupt:
    print("Interrupted. Saving log and disconnecting...")

finally:
    con.send_pause()
    con.disconnect()

    # Save to CSV
    filename = f"robot_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        headers = ["time"] + [f"tcp_{i}" for i in range(6)] + [f"joint_{i}" for i in range(6)] + [f"tcp_force_{i}" for i in range(6)] + [f"joint_torque_{i}" for i in range(6)]
        writer.writerow(headers)
        writer.writerows(log_data)
    print(f"Data saved to {filename}")


    ## Plot TCP Z position
    times = [row[0] for row in log_data]
    tcp_z = [row[3] for row in log_data]  # tcp[2] = z
    plt.plot(times, tcp_z)
    plt.title("TCP Z Position Over Time")
    plt.xlabel("Time (s)")
    plt.ylabel("TCP Z (m)")
    plt.grid()
    plt.show()

    # Plot TCP force (Fx, Fy, Fz)
    tcp_fx = [row[12] for row in log_data]  # Offset = time(1) + tcp(6) + joints(6)
    plt.figure()
    plt.plot(times, tcp_fx, label='Fx')
    plt.plot(times, [row[13] for row in log_data], label='Fy')
    plt.plot(times, [row[14] for row in log_data], label='Fz')
    plt.title("TCP Forces Over Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Force (N)")
    plt.legend()
    plt.grid()

    # Plot Joint Torque
    # Is this current? Filter?
    plt.figure()
    for j in range(6):
        torque_series = [row[18 + j] for row in log_data]
        plt.plot(times, torque_series, label=f"Joint {j}")
    plt.title("Joint Torques (target_moment)")
    plt.xlabel("Time (s)")
    plt.ylabel("Torque (Nm)")
    plt.legend()
    plt.grid()
    plt.show()
