from rtde_receive import RTDEReceiveInterface

robot_ip = "192.168.5.5"      # UR5 Poole
# robot_ip = "192.168.5.4"      # UR5 Floyd
# robot_ip = "192.168.5.3"        # UR20 Bowman

try:
    rtde_r = RTDEReceiveInterface(robot_ip)
    print("Robot is communicating!")

    pos = rtde_r.getActualQ()
    print("Current joint positions:", ', '.join(f"{p:.4f}" for p in pos))

    pos2 = rtde_r.getActualTCPPose()
    print("Current TCP pose:", ', '.join(f"{p:.4f}" for p in pos2))

except Exception as e:
    print("Could not connect to robot:", e)


## Test whether socket and IP can connect ##
# import rtde.rtde as rtde

# con = rtde.RTDE("192.168.5.4", 30004)
# con.connect()
# print("Connected!")
