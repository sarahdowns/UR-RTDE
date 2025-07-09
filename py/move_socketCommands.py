# Author: Sarah Downs
# This program uses the socket (not RTDE) to move the UR robot arm to a defined TCP position or a joint position
# based on whether move_tcp_first is True or False

import socket
import time

HOST = "192.168.5.4"
PORT = 30002

# Target TCP pose: p[x, y, z, Rx, Ry, Rz]
tcp_pose = [-0.1384, 0.1487, 0.4167, -1.3069, 0.2662, 0.8861]
# Target joint angles: [base, shoulder, elbow, ...]
joint_target = [-0.0003, 0.3543, -2.3819, -1.8780, -2.7141, 0.0016]

### Change this to decide the order ##############################
move_tcp_first = False

# Compose command string
command = ""
if move_tcp_first:
    command += f"movel(p{tcp_pose}, a=1.2, v=0.25)\n"
    command += "sleep(1.0)\n"
    command += f"movej({joint_target}, a=1.2, v=0.25)\n"
else:
    command += f"movej({joint_target}, a=1.2, v=0.25)\n"
    command += "sleep(1.0)\n"
    command += f"movel(p{tcp_pose}, a=1.2, v=0.25)\n"

# Send the command
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    s.send(command.encode('utf-8'))
    time.sleep(2)
