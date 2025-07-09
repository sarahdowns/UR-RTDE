# This code works together with the rtde_control_loop.urp file running on the teach 
# pendant. Run this example code first then the teach pendant to make the robot move
# to between 3 defined points.

import sys

sys.path.append("..")
import logging

import rtde.rtde as rtde
import rtde.rtde_config as rtde_config


# logging.basicConfig(level=logging.INFO)

ROBOT_HOST = "192.168.5.4"      # Robot IP address
ROBOT_PORT = 30004
config_filename = "control_loop_configuration.xml"

keep_running = True

logging.getLogger().setLevel(logging.INFO)

conf = rtde_config.ConfigFile(config_filename)
state_names, state_types = conf.get_recipe("state")
setp_names, setp_types = conf.get_recipe("setp")
watchdog_names, watchdog_types = conf.get_recipe("watchdog")

con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
con.connect()
print("Connected!")     # Debug: Test if socket is connected

# get controller version
con.get_controller_version()

# setup recipes
con.send_output_setup(state_names, state_types)
setp = con.send_input_setup(setp_names, setp_types)
watchdog = con.send_input_setup(watchdog_names, watchdog_types)

# TCP poses for robot movement
setp1 = [-0.2495226685495815, 0.14874174759594838, 0.5278506963352518, -1.3069381137270244, 
0.26618097315376366, 0.8860891500099016] 
setp2 = [-0.10022229897292662, 0.14874012535279876, 0.6546663701260047, -1.306935279942788, 
0.2661978853574831, 0.8861360137161748]    
setp3 = [-0.1639995303168894, 0.1487454285028731, 0.6255272200759749, -1.530653065326208, 
-0.27709591454737253, 0.3897994609907399]
print("Robot now knows where to move")  

setp.input_double_register_0 = 0
setp.input_double_register_1 = 0
setp.input_double_register_2 = 0
setp.input_double_register_3 = 0
setp.input_double_register_4 = 0
setp.input_double_register_5 = 0

# The function "rtde_set_watchdog" in the "rtde_control_loop.urp" creates a 1 Hz watchdog
watchdog.input_int_register_0 = 0

def setp_to_list(sp):
    sp_list = []
    for i in range(0, 6):
        sp_list.append(sp.__dict__["input_double_register_%i" % i])
    return sp_list


def list_to_setp(sp, list):
    for i in range(0, 6):
        sp.__dict__["input_double_register_%i" % i] = list[i]
    return sp


# start data synchronization
if not con.send_start():
    sys.exit()

# control loop
setpoints = [setp1, setp2, setp3]
current_setpoint_index = 0

# control loop
move_completed = True
while keep_running:
    state = con.receive()

    if state is None:
        print("Received None from robot! Ending loop.")
        break

    if move_completed and state.output_int_register_0 == 1:
        move_completed = False

        # Move to the next setpoint
        current_setpoint_index = (current_setpoint_index + 1) % len(setpoints)
        new_setp = setpoints[current_setpoint_index]

        list_to_setp(setp, new_setp)
        print("New pose = " + str(new_setp))
        con.send(setp)
        watchdog.input_int_register_0 = 1

    elif not move_completed and state.output_int_register_0 == 0:
        print("Move to confirmed pose = " + str(state.target_q))
        move_completed = True
        watchdog.input_int_register_0 = 0

    con.send(watchdog)

con.send_pause()

con.disconnect()