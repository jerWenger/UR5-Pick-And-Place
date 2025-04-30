import rtde_receive
import rtde_control
import time
import numpy as np
import esp_interface.esp_interface as esp
import signal
import sys


#Initialize interfaces
joystick = esp.ESPInterface('/dev/ttyACM0')

rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.103")
rtde_c = rtde_control.RTDEControlInterface("192.168.1.103")

#get initial pose
pose = rtde_r.getActualTCPPose()

#Enter test loop


def signal_handler(sig, frame):
    print("\nCaught Ctrl+C. Disconnecting from robot and ending program.")
    rtde_c.speedL([0, 0, 0, 0, 0, 0], 2, 0)
    rtde_c.disconnect()
    rtde_r.disconnect()
    sys.exit(0)

# Attach the signal handler
signal.signal(signal.SIGINT, signal_handler)

while True:

    #get current pose
    pose = rtde_r.getActualQ()
    
    #Read the Joystick
    got_joystick, joystick_data = joystick.read_serial()


    #apply joystick data to speed

    joy_x = joystick_data[1] # -1 to 1 centered at 0
    joy_y = joystick_data[2] # -1 to 1 centered at 0
    joy_z = joystick_data[3] # -1 to 1 centered at 0
    endpoint_omega = joystick_data[4] #-1 to 1 cetnered at 0

    speed = [0, 0, 0, 0, 0, 0] # TCP speed commands

    # Implement software limits for the robot axes to prevent collisions with the camera pole
    if pose[0] > -0.3:
        speed[0] = min(joy_x, 0)
    elif pose[0] < -0.7:
        speed[0] = max(joy_x, 0)
    else:
        speed[0] = joy_x

    if pose[1] > -0.1:
        speed[1] = min(joy_y, 0)
    elif pose[1] < -0.5:
        speed[1] = max(joy_y, 0)
    else:
        speed[1] = joy_y

    if pose[2] > 0.25: # Prevent collision with conveyor belt
        speed[2] = min(joy_z, 0)
    elif pose[2] < 0.06:
        speed[2] = max(joy_z, 0)
    else:
        speed[2] = joy_z

    speed[5] = endpoint_omega

    rtde_c.speedL(speed, 2, 0) #send speed data to the UR5

    pose = [round(i,2) for i in pose]
    print(pose)

    #sleep for a bit
    time.sleep(0.01)