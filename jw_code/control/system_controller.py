# control/system_controller.py

import rtde_receive
import rtde_control
import time
import numpy as np
import esp_interface.esp_interface as esp
import signal
import sys
from cv_interface import cv_interface
from cv_interface import bottle

class SystemController:
    def __init__(self):
        """
        Initializes the system controller with instances of the CV and ESP interfaces.
        """
        #Initialize interfaces
        self.joystick = esp.ESPInterface('COM7') #Change this to your port

        self.rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.103")
        self.rtde_c = rtde_control.RTDEControlInterface("192.168.1.103")

<<<<<<< HEAD
        self.cv_input = cv_interface.CVInterface()
        self.cv_input.set_camera_fps(10)
        self.current_bottle = None
=======
        #self.cv_input = cv_interface.CVInterface()
        self.bottle_target = ["",[],[]]
>>>>>>> 32fe252713280223be8e29eb1e95396d6cdd2f22
        self.belt_velocity = .1 #m/s  <--- INPUT THE REAL VALUE
        self.UR5status = "idle"
        # status options: idle, prep, pickup, drop, reset

        #get initial pose
        self.pose = self.rtde_r.getActualTCPPose()
        self.running = True

        # Autonomous path parameters
        self.autonomous_path = self.generate_circle_path(radius=0.15, center=[0, -0.5, 0.1], num_points=50)
        self.target_index = 0
        self.lookahead_distance = 0.02  # How close before switching to next point

    def generate_circle_path(self, radius, center, num_points):
        """
        Generate a list of (x, y, z) poses forming a circle in the XY plane.
        """
        path = []
        cx, cy, cz = center
        for i in range(num_points):
            angle = 2 * np.pi * i / num_points
            x = cx + radius * np.cos(angle)
            y = cy + radius * np.sin(angle)
            z = cz
            pose = [x, y, z, 0, 0, 0]
            path.append(pose)
        return path
    

    def compute_velocity_to_pose(self, current_pose, target_pose, max_speed=2):
        """
        Compute a velocity vector to move from current pose to target pose.
        """
        current_pos = np.array(current_pose[:3])
        target_pos = np.array(target_pose[:3])
        pos_error = target_pos - current_pos
        distance = np.linalg.norm(pos_error)

        if distance < 1e-4:
            linear_velocity = np.zeros(3)
        else:
            direction = pos_error / distance
            speed = min(distance, max_speed)
            linear_velocity = direction * speed

        #AngularStuf
        current_ang = np.array((current_pose[3:6]))
        target_ang = np.array((target_pose[3:6]))
        #ang_error = target_ang - current_ang
        ang_error = (target_ang - current_ang + 0.5) % 1.0 - 0.5
        ang_distance = np.linalg.norm(ang_error)

        if ang_distance < 1e-3:
            angular_velocity = np.zeros(3)
        else:
            ang_direction = ang_error / ang_distance
            speed = min(ang_distance, max_speed)
            angular_velocity = ang_direction * speed
        if current_ang[0] < 0:
            angular_velocity = -angular_velocity
        angular_velocity = [angular_velocity[0], 0, 0]


        #angular_velocity = [0, 0, 0]  # No orientation changes for now

        return np.concatenate((linear_velocity, angular_velocity)).tolist(), distance
    
    def apply_software_limits(self, speed):
        """
        Enforces software-defined workspace limits regardless of control mode.
        """
        limited_speed = speed.copy()

        # X axis limits
        if self.pose[0] < -0.6:
            limited_speed[0] = max(speed[0], 0)
        elif self.pose[0] > 0.4:
            limited_speed[0] = min(speed[0], 0)

        # Y axis limits
        if self.pose[1] > -0.1:
            limited_speed[1] = min(speed[1], 0)
        elif self.pose[1] < -0.8:
            limited_speed[1] = max(speed[1], 0)

        # Z axis limits
        if self.pose[2] > 0.25:
            limited_speed[2] = min(speed[2], 0)
        elif self.pose[2] < -0.06:
            limited_speed[2] = max(speed[2], 0)

        return limited_speed

    def update_bottle(self):
        first, _ = self.cv_input.bottle_identification()
        if first and self.UR5status == "idle": #if there was no bottle, create one
            self.current_bottle = first
            self.UR5status = "prep"
        elif first and self.UR5status == "prep":
            self.current_bottle = first.update(self.current_bottle)
            if self.current_bottle.get_status() == "ready":
                self.UR5status = "pickup"
        elif self.UR5status == "ready":
            self.current_bottle.step_pos()
        # if first and self.bottle_target[0] != "ready": # if bottle in frame: update target with coords
        #     # might want to check if the new coordinates are close to previous
        #     self.bottle_target[0] = "inFrame"
        #     self.bottle_target[1] = first
        #     #self.bottle_target[2].append(first[2])
        # elif self.bottle_target and type(self.bottle_target[2]) == list: 
        #     # if bottle not in frame and bottle target has something: change status
        #     self.bottle_target[0] = "ready"
        #     self.bottle_target[1][0] -= self.belt_velocity*.1
        # # if bottle not in frame and bottle target has nothing: do nothing

    def step(self):
        """
        Perform a single control step. This method can be called repeatedly in a loop or manually.
        """
        
        #get current pose
        self.pose = self.rtde_r.getActualTCPPose()
        
        #get joystick data
        self.got_joystick, self.joystick_data = self.joystick.read_serial()

        speed = [0,0,0,0,0,0]

        #decide if we are in joystick mode operate accordingly
        if (self.joystick_data[0] == 0):
            speed[0] = self.joystick_data[1] / 3# X velocity -1 to 1 centered at 0
            speed[1] = self.joystick_data[2] / 3 # Y velocity -1 to 1 centered at 0
            speed[2] = self.joystick_data[3] / 5 # Z velocity -1 to 1 centered at 0
            speed[3] = self.joystick_data[4] / 3# Omega velocity -1 to 1 cetnered at 0

        elif(self.joystick_data[0] == 1):
            # Autonomous mode
<<<<<<< HEAD
            target_pose = [0.15, -0.4, 0, 0, 3.13, 0] #neutral
            self.update_bottle #this will update self.
            if self.UR5status == "prep":
                target_pose[1] = self.current_bottle.get_y()
            elif self.UR5status == "pickup":
                target_pose[0] = self.current_bottle.get_x()
                target_pose[1] = self.current_bottle.get_y()
                # when it arrives at location, lower y
                # then set status to drop
            elif self.UR5status == "drop":
                color = self.current_bottle.get_color()
                # move to bin
                # change status to reset once it drops
            elif self.UR5status == "reset":
                self.current_bottle = None
                # target pose is neutral
           
           
            #target_pose = self.autonomous_path[self.target_index]
=======
            # self.bottle_status() #this will update self.
            #if self.bottle_target[0] == "ready":
            #    self.cv_input.step_position(self.bottle_target[1], self.belt_velocity)
            
            target_pose = self.autonomous_path[self.target_index]

>>>>>>> 32fe252713280223be8e29eb1e95396d6cdd2f22
            #Pose Target
            #target_pose = self.autonomous_path[self.target_index]
            target_pose = [0.15, -0.4, 0, 3, 0, 0] #Neutral
            #target_pose = [-0.436, -0.567, 0, 0, 3.13, 0] #Green
            #target_pose = [-0.116, -0.636, 0, 0, 3.13, 0] #Blue
            #target_pose = [0.14, -0.631, 0, 0, 3.13, 0] #Yellow
            #target_pose = [0.369, -0.531, 0, 0, 3.13, 0] #Shared
            speed, distance = self.compute_velocity_to_pose(self.pose, target_pose)

            #For auto circle
            # If close enough to the target, move to the next one
            #if distance < self.lookahead_distance:
            #    self.target_index = (self.target_index + 1) % len(self.autonomous_path)

        #safety check
        speed = self.apply_software_limits(speed)

        self.rtde_c.speedL(speed, 2, 0) #send speed data to the UR5

        pose = [round(i,2) for i in self.pose]
        
        return {
        "pose": pose,
        "speed": speed,
        "joystick": self.joystick_data,
        "status": self.UR5status
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

