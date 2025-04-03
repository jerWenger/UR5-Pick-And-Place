import rtde_receive
import rtde_control
import time

# Initialize the RTDE interfaces
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.103")
rtde_c = rtde_control.RTDEControlInterface("192.168.1.103")

#get the pose of the robot and print it
while(True):
    print(rtde_r.getActualTCPPose())
    time.sleep(0.5)
    