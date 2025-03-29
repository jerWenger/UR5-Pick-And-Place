import rtde_receive
import rtde_control
import pygame
import time
#import robotiq_gripper
#import pyrealsense2 as rs
import numpy as np
#import cv2
import serial

# Initialize the RTDE interfaces
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.103")
rtde_c = rtde_control.RTDEControlInterface("192.168.1.103")


#Initialize ESP Joystick
port_name = '/dev/ttyACM0'
serial_port = serial.Serial(port=port_name, baudrate=115200, timeout=1, write_timeout=1)
data = [0, 0]

#Camera code skipped

while (True):
    #get pose values
    pose = rtde_r.getActualTCPPose()
    #get joystick values

    try: # Be careful: any code inside this try block that fails will not display an error. Instead, the block will simply exit.
        # We recommend moving code outside of the try block for testing.
        if serial_port.in_waiting > 0:
            esp32_output = str(serial_port.readline()) # The ESP32 output should be a series of values seperated by commas and terminated by "\n", e.g. "1,2,3,4,\n".
                                                           # This termination occurs automatically if you use Serial.println();
                
            esp32_output = esp32_output[2:-3]
            vals = esp32_output.split(",") # Split into a list of strings
            print(vals) # Useful for debugging
                
            for i in range(len(data)):
                data[i] = float(vals[i])
                
            print(data)
            serial_port.reset_input_buffer()
        else:
            time.sleep(0.00101) # Sleep for at least 1 ms to give the loop a chance to rest
    except:
            pass

    # Map joystick axes to robot axes, using a 2D rotation matrix of 45 degrees, since the robot axes are rotated 45 degrees w.r.t. the conveyor belt axes
    joy_x = data[0] / 2 #joy.get_axis(1)/4.0*0.7071 - joy.get_axis(0)/4.0*0.7071
    joy_y = data[1] / 2#joy.get_axis(1)/4.0*0.7071 + joy.get_axis(0)/4.0*0.7071
    joy_z = 0 #(-1*joy.get_button(2) + joy.get_button(3))/5.0
    joy_grip = 0 #joy.get_axis(3)
    endpoint_omega = 0 #joy.get_axis(2)/4.0 # Gripper twist

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

    #gripper.move((int)((joy_grip+1)*127.5), 255, 10) # Control the amount the gripper is opened or closed


    rtde_c.speedL(speed, 2, 0)

    #pose = [round(i,2) for i in pose]
    #print(pose)
                      
    time.sleep(0.03)

# User has pressed exit button on joystick. Stop robot and close vision pipeline.
rtde_c.speedL([0,0,0,0,0,0])