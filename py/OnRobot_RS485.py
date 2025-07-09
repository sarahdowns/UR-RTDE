# Sarah Downs
# Outputs socket hex code. Currently uninterpreted.

import socket
import time

ROBOT_IP = "192.168.5.4"
URSCRIPT_PORT = 30002           # Robot port
GRIPPER_PORT = 54321            # Gripper port

def send_urscript(script):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((ROBOT_IP, URSCRIPT_PORT))
        s.sendall(script.encode('utf-8'))
        time.sleep(2)

def send_gripper_command(position):
    """
    position: 0 (open) to 255 (close)
    """
    command = bytearray(8)
    command[0] = 0xA1  # RG2 Command ID
    command[1] = 0x01  # Set position
    command[2] = position
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((ROBOT_IP, GRIPPER_PORT))
        s.sendall(command)
        time.sleep(0.2)
        response = s.recv(1024)
        print("Gripper response:", repr(response))

if __name__ == "__main__":
    print("Closing gripper...")
    send_gripper_command(255)
    time.sleep(1)

    # print("Moving robot...")
    # send_urscript("movej([-0.2495, 0.1487, 0.5278, -1.3069, 0.2661, 0.8860] , a=1.2, v=0.25)\n")
    # time.sleep(3)

    print("Opening gripper...")
    send_gripper_command(0)
