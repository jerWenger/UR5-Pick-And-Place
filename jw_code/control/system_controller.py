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
        self.joystick = esp.ESPInterface('/dev/ttyACM0') #Change this to your port

        self.rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.103")
        self.rtde_c = rtde_control.RTDEControlInterface("192.168.1.103")

        self.cv_input = cv_interface.CVInterface()
<<<<<<< HEAD
        self.cv_input.set_camera_fps(10)
=======
>>>>>>> 397a405ad07b0cb37dcf54e30c80ed0885acf036
        self.current_bottle = None
        #self.bottle_target = ["",[],[]]
        self.belt_velocity = .1 #m/s  <--- INPUT THE REAL VALUE
        #self.UR5status = "idle"
        # status options: idle, prep, pickup, drop, reset
        #self.dropped = False

        #get initial pose
        self.pose = self.rtde_r.getActualTCPPose()
        self.running = True

        # Autonomous path parameters
        #self.lookahead_distance = 0.02  # How close before switching to next point
        self.last_rot_error = np.zeros(3)
        self.error = [0,0,0,0]

        #Timing code for pd controller
        self.last_time = time.time()
        self.dt = 0.005 

        #Timing for testing PD gains
        self.switch_interval = 5.0  # seconds between switches
        self.last_switch_time = time.time()
        self.current_target_index = 0  # 0 or 1

        #state machine
        self.state = "GO_TO_NEUTRAL"
        self.throw = False
        self.margin = 0.01
        self.safe_height = 0.1
        self.pickup_height = 0.0
        self.last_actuator_status = "None"

        self.neutral_rotation = [0.5, 3.0, 0.0]

        self.neutral_pose = [0.15, -0.4, self.safe_height, self.neutral_rotation]

        self.bin_poses = {
            "clear": [-0.436, -0.567, self.safe_height, self.neutral_rotation],
            "blue": [-0.116, -0.636, self.safe_height, self.neutral_rotation],
            "yellow": [0.140, -0.631, self.safe_height, self.neutral_rotation],
            "shared": [0.369, -0.531, self.safe_height, self.neutral_rotation]
        }
    
    def set_actuator(self, desired_action):
        """Wrapper for ESP To prevent sending a signal too many times"""
        if self.last_actuator_status == desired_action:
            pass
        elif desired_action == "PICKUP":
            self.joystick.write_serial(1, 0)
        elif desired_action == "DROP":
            self.joystick.write_serial(0,1)
            
    def compute_velocity_to_pose(self, current_pose, target_pose, max_speed=0.5, max_angular_speed=0.5):
        """
        Compute a 6D velocity vector (vx, vy, vz, wx, wy, wz) to move from current_pose to target_pose.
        Linear and angular velocities are scaled to avoid exceeding max_speed.
        """

        # --- Linear Part ---
        current_pos = np.array(current_pose[:3])
        target_pos = np.array(target_pose[:3])
        pos_error = target_pos - current_pos
        distance = np.linalg.norm(pos_error)

        if distance > 1e-4:
            linear_direction = pos_error / distance
            linear_speed = min(distance, max_speed)
            linear_velocity = linear_direction * linear_speed
        else:
            linear_velocity = np.zeros(3)

        # --- Angular Part ---
        current_rot = np.array(current_pose[3:6])
        target_rot = np.array(target_pose[3:6])

        # Rotation error wrapped to [-pi, pi]
        rot_error = (target_rot - current_rot + np.pi) % (2 * np.pi) - np.pi

        # Derivative of rotation 
        rot_error_derivative = (rot_error - self.last_rot_error) / self.dt

        # PD control
        Kp_ang = 1  # proportional gain
        Kd_ang = 0.5  # derivative gain (damping)

        angular_velocity = Kp_ang * rot_error + Kd_ang * rot_error_derivative

        # Clip angular velocity to max
        if np.linalg.norm(angular_velocity) > max_angular_speed:
            angular_velocity = (angular_velocity / np.linalg.norm(angular_velocity)) * max_angular_speed

        # --- Combine
        velocity_command = np.concatenate((linear_velocity, angular_velocity))

        # --- Save errors for next step
        self.last_rot_error = rot_error.copy()
        self.error = [pos_error, distance, rot_error, np.linalg.norm(rot_error)]

        return velocity_command.tolist(), distance

    
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
       
    def step(self):
        """
        Perform a single control step. This method can be called repeatedly in a loop or manually.
        """
        
        # --- Timing
        now = time.time()
        self.dt = now - self.last_time
        self.last_time = now
        

        #get current pose
        self.pose = self.rtde_r.getActualTCPPose()
        
        #get joystick data
        self.got_joystick, self.joystick_data = self.joystick.read_serial()

        speed = [0,0,0,0,0,0]

        #get cv data
        self.update_bottle()
        if (self.current_bottle != None) and (self.current_bottle.get_status == "ready"):
            self.state = "MOVE_OVER_BOTTLE"
            bottleX = self.current_bottle.get_x
            bottleY = self.current_bottle.get_y
            bottle_color = self.current_bottle.get_color
        else:
            self.state = "WAIT"
        


        #decide if we are in joystick mode operate accordingly
        if (self.joystick_data[0] == 0):
            speed[0] = self.joystick_data[1] / 3# X velocity -1 to 1 centered at 0
            speed[1] = self.joystick_data[2] / 3 # Y velocity -1 to 1 centered at 0
            speed[2] = self.joystick_data[3] / 5 # Z velocity -1 to 1 centered at 0
            speed[3] = self.joystick_data[4] / 3# Omega velocity -1 to 1 cetnered at 0
        elif(self.joystick_data[0] == 1):
           #Autonomous mode
           if self.state == "GO_TO_NEUTRAL":
               success = False

               #WE are moving to neutral

               if (success):
                   self.state = "WAIT"
           elif self.state == "WAIT":
               success = False
            
               #WE are waiting for the next bottle
               speed = [0,0,0,0,0,0]
           
               if(success):
                   self.state = "MOVE_OVER_BOTTLE"
           elif self.state == "MOVE_OVER_BOTTLE":
               success = False

               #We are moving above the bottle
               target = [bottleX, bottleY, self.safe_height, self.neutral_rotation]
               speed = self.compute_velocity_to_pose(self.pose, target)

               if(success):
                   self.state = "LOWER_ON_BOTTLE"
           elif self.state == "LOWER_ON_BOTTLE":
               success = False

               #WE are lowering to pick up the bottle
               self.set_actuator("PICKUP")
               target = [bottleX, bottleY, self.pickup_height, self.neutral_rotation]
               speed = self.compute_velocity_to_pose(self.pose, target)

               if(success):
                   self.state = "GO_TO_BIN"
           elif self.state == "GO_TO_BIN":
               success = False

               #WE are going to the correct bin
               speed = self.compute_velocity_to_pose(self.pose, self.bin_poses(bottle_color))

               if (success):
                   self.state = "DROP_BOTTLE"
               elif (success and self.throw):
                   self.state = "THROW_BOTTLE"
           elif self.state == "DROP_BOTTLE":
               success = False

               #We are dropping the bottle
               self.set_actuator("DROP")

               speed = [0,0,0,0,0,0]

               if(success):
                   self.state = "GO_TO_NEUTRAL"
                   self.current_bottle = None
           elif self.state == "THROW_BOTTLE":
               success = False

                #We are throwwing the bottle
               self.set_actuator("DROP")
               speed = [0,0,0,0,0,0]
               
               if(success):
                   self.state = "GO_TO_NEUTRAL"
                   self.current_bottle = None
           else:
               speed = [0,0,0,0,0,0]

        #safety check
        speed = self.apply_software_limits(speed)

        self.rtde_c.speedL(speed, 2, 0) #send speed data to the UR5

        pose = [round(i,2) for i in self.pose]
        
        return {
        "pose": pose,
        "speed": speed,
        "joystick": self.joystick_data,
        "status": self.UR5status,
        "error": self.error,
        "state": self.state
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

