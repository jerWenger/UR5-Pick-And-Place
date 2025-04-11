# control/system_controller.py

import rtde_receive
import rtde_control
import time
import numpy as np
import esp_interface.esp_interface as esp
import signal
import sys

class SystemController:
    def __init__(self):
        """
        Initializes the system controller with instances of the CV and ESP interfaces.
        """
        #Initialize interfaces
        self.joystick = esp.ESPInterface('/dev/ttyACM0')

        self.rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.103")
        self.rtde_c = rtde_control.RTDEControlInterface("192.168.1.103")

        #get initial pose
        self.pose = self.rtde_r.getActualTCPPose()
        self.running = True

    def step(self):
        """
        Perform a single control step. This method can be called repeatedly in a loop or manually.
        """
        
        #get current pose
        self.pose = self.rtde_r.getActualTCPPose()
        
        #get joystick data
        self.got_joystick, self.joystick_data = self.joystick.read_serial()

        #decide if we are in joystick mode operate accordingly
        if (self.joystick_data[0] == 0):
            joy_x = self.joystick_data[1] # -1 to 1 centered at 0
            joy_y = self.joystick_data[2] # -1 to 1 centered at 0
            joy_z = self.joystick_data[3] # -1 to 1 centered at 0
            endpoint_omega = self.joystick_data[4] #-1 to 1 cetnered at 0
        elif(self.joystick_data[0] == 1):
            joy_x = 0
            joy_y = 0
            joy_z = 0
            endpoint_omega = 0

        speed = [0, 0, 0, 0, 0, 0] # TCP speed commands

        # Implement software limits for the robot axes to prevent collisions with the camera pole
        if self.pose[0] > -0.3:
            speed[0] = min(joy_x, 0)
        elif self.pose[0] < -0.7:
            speed[0] = max(joy_x, 0)
        else:
            speed[0] = joy_x

        if self.pose[1] > -0.1:
            speed[1] = min(joy_y, 0)
        elif self.pose[1] < -0.5:
            speed[1] = max(joy_y, 0)
        else:
            speed[1] = joy_y

        if self.pose[2] > 0.25: # Prevent collision with conveyor belt
            speed[2] = min(joy_z, 0)
        elif self.pose[2] < 0.06:
            speed[2] = max(joy_z, 0)
        else:
            speed[2] = joy_z

        speed[5] = endpoint_omega

        self.rtde_c.speedL(speed, 2, 0) #send speed data to the UR5

        pose = [round(i,2) for i in self.pose]
        
        return {
        "pose": pose,
        "speed": speed,
        "joystick": self.joystick_data
        }


    def shutdown(self):
        """
        Perform any necessary cleanup on shutdown.
        This should safely stop any robot actions and close all interfaces.
        """
        self.running = False
        print("[SystemController] Stopping robot safely...")
        self.rtde_c.speedL([0, 0, 0, 0, 0, 0], 2, 0)
        self.rtde_c.disconnect()
        self.rtde_r.disconnect()
        print("[SystemController] Shutdown complete.")

