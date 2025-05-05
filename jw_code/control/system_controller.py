# control/system_controller.py

import rtde_receive
import rtde_control
import time
import numpy as np
import esp_interface.esp_interface as esp
import signal
import sys
import cv2
import math
from cv_interface import depth_integrated
import cv_interface.bottle as bottle

class SystemController:
    def __init__(self):
        """
        Initializes the system controller with instances of the CV and ESP interfaces.
        """
        #Initialize interfaces

        #Connect to ESP and set joystick scalars
        self.joystick = esp.ESPInterface('/dev/ttyACM0') #Change this to your port
        self.JOYSTICK_SCALE_XY = 1/3
        self.JOYSTICK_SCALE_Z = 1/5

        #Connect to UR5
        self.rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.103")
        self.rtde_c = rtde_control.RTDEControlInterface("192.168.1.103")

        #Connect to real sense and set some tuning parameters
        self.cv_last_time = 0
        self.cv_time_step = 0.1
        self.cv_input = depth_integrated.CVInterface()
        self.current_bottle = None
        self.belt_velocity = .1 #m/s  <--- INPUT THE REAL VALUE

        #bottle pose
        self.bottleX = None
        self.bottleY = None
        self.bottle_color = None

        #get initial pose
        self.pose = self.rtde_r.getActualTCPPose()
        self.running = True

        # Move to known joint-space neutral position
        neutral_joint_pose = [-4.68, -1.2, 2.35, -2.73, -1.56, 0.00]
        print("Sending Robot Home")
        self.rtde_c.moveJ(neutral_joint_pose)

        time.sleep(2)
        # Get the actual TCP pose and extract rotation
        tcp_pose = self.rtde_r.getActualTCPPose()
        self.neutral_rotation = tcp_pose[3:6]
        print(f"Neutral Position Set: {self.neutral_rotation}")

        # Autonomous path parameters
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
        self.linear_margin = 0.01
        self.linear_margin_fast = 0.05
        self.rotation_margin = 0.3
        self.rotation_margin_fast = 0.4
        self.safe_height = 0.01
        self.pickup_height = -0.08
        self.last_actuator_status = "None"
        self.low_height = 0.1

        self.neutral_pose = self.make_pose(0.11, -0.4, self.safe_height)

        self.bin_poses = {
            "clear": self.make_pose(-0.42, -0.78, self.safe_height),
            "blue": self.make_pose(-0.12, -0.82, self.safe_height),
            "yellow": self.make_pose(0.140, -0.84, self.safe_height),
            "SHARED": self.make_pose(0.39, -0.74, self.safe_height),
            "orange": self.make_pose(0.39, -0.74, self.safe_height),
        }
        self.bin_poses_tilt = {
            "clear": [self.pose[0], self.pose[1], self.pose[2], self.pose[3], self.pose[4]+0.4, self.pose[5]],
            "blue": [self.pose[0], self.pose[1], self.pose[2], self.pose[3], self.pose[4]+0.4, self.pose[5]], 
            "yellow": [self.pose[0], self.pose[1], self.pose[2], self.pose[3], self.pose[4]+0.4, self.pose[5]],
        }
        self.bin_throw_poses = {
            "clear": self.make_pose(-0.42, -0.78, self.safe_height+0.15), #-0.42, -0.8
            "blue": self.make_pose(-0.12, -0.82, self.safe_height+0.15), #-.012, -0.8
            "yellow": self.make_pose(0.14, -0.82, self.safe_height+0.15), #0.14, -0.8
        }
    def set_actuator(self, desired_action):
        """Wrapper for ESP To prevent sending a signal too many times"""
        if self.last_actuator_status == desired_action:
            pass
        elif desired_action == "PICKUP":
            self.joystick.write_serial(1, 0)
            self.last_actuator_status = desired_action
        elif desired_action == "DROP":
            self.joystick.write_serial(0,1)
            self.last_actuator_status = desired_action
            
    def compute_velocity_to_pose(self, current_pose, target_pose, max_speed=1, max_angular_speed=0.5):
        """
        Compute a 6D velocity vector (vx, vy, vz, wx, wy, wz) to move from current_pose to target_pose.
        Linear and angular velocities are scaled to avoid exceeding max_speed.
        """

        # --- Linear Part ---
        current_pos = np.array(current_pose[:3])
        target_pos = np.array(target_pose[:3])
        pos_error = target_pos - current_pos
        distance = np.linalg.norm(pos_error)

        if distance > 1e-2:
            linear_direction = pos_error / distance
            linear_speed = min(distance*1.75, max_speed)
            linear_velocity = linear_direction * linear_speed
        else:
            linear_velocity = np.zeros(3)

        # --- Angular Part ---
        #current_rot = np.array(current_pose[3:6])
        #target_rot = np.array(target_pose[3:6])

        # Rotation error wrapped to [-pi, pi]
        #rot_error = (target_rot - current_rot + np.pi) % (2 * np.pi) - np.pi

        # Derivative of rotation 
        #rot_error_derivative = (rot_error - self.last_rot_error) / self.dt

        # PD control
        #Kp_ang = 1  # proportional gain
        #Kd_ang = 0.5  # derivative gain (damping)

        #angular_velocity = Kp_ang * rot_error + Kd_ang * rot_error_derivative

        # Clip angular velocity to max
        #if np.linalg.norm(angular_velocity) > max_angular_speed:
        #    angular_velocity = (angular_velocity / np.linalg.norm(angular_velocity)) * max_angular_speed

        # --- Combine
        angular_velocity = np.zeros(3) #no angular velocity for now
        velocity_command = np.concatenate((linear_velocity, angular_velocity))

        #Sum errors
        #rot_error_norm = np.linalg.norm(rot_error)

        # --- Save errors for next step
        #self.last_rot_error = rot_error.copy()
        rot_error = np.zeros(3) #no angular velocity for now
        rot_error_norm = 0 #no angular velocity for now
        self.error = [pos_error, distance, rot_error, rot_error_norm]

        return velocity_command.tolist(), distance
    def vector_magnitude(self, delta_pose, joint = False):
        if joint:
            return (sum([d*d for d in delta_pose[:]]))**0.5  # Total joint position error.
        return (sum([d*d for d in delta_pose[:3]]))**0.5  # 
    
    def moveJ(self,target_pose):
        move_duration = 0.1
        acceleration = 2.0
        tolerance = 0.005

        pose = self.rtde_r.getActualQ()
        delta_pose = [target_pose[i] - pose[i] for i in range(6)]

        while self.vector_magnitude(delta_pose, joint=True) > tolerance:


            pose = self.rtde_r.getActualQ()
            delta_pose = [2*(target_pose[i] - pose[i]) for i in range(6)]

            speed = [0, 0, 0, 0, 0, 0]
            for i in range(6):
                speed[i] = max(-0.25, min(0.25, delta_pose[i]))  # Clip 


            self.rtde_c.speedJ(speed, acceleration, move_duration)
            time.sleep(move_duration)

        # Stop the robot
        self.rtde_c.speedJ([0, 0, 0, 0, 0, 0], acceleration, 0)
    def compute_velocity_to_pose_fast(self, current_pose, target_pose, max_speed=1.15, max_angular_speed=0.5):
        """
        Compute a 6D velocity vector (vx, vy, vz, wx, wy, wz) to move from current_pose to target_pose.
        Linear and angular velocities are scaled to avoid exceeding max_speed.
        """

        # --- Linear Part ---
        current_pos = np.array(current_pose[:3])
        target_pos = np.array(target_pose[:3])
        pos_error = target_pos - current_pos
        distance = np.linalg.norm(pos_error)

        if distance > 5e-2:
            linear_direction = pos_error / distance
            linear_speed = min(distance*2.25, max_speed) #increase speed by changing distance multiplier
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

        #Sum errors
        rot_error_norm = np.linalg.norm(rot_error)

        # --- Save errors for next step
        self.last_rot_error = rot_error.copy()
        self.error = [pos_error, distance, rot_error, rot_error_norm]

        return velocity_command.tolist(), distance

    def make_pose(self, x, y, z):
        """
        Simple helper method to apply the neutral rotation to a pose
        """
        return [x, y, z, *self.neutral_rotation]
    
    def is_pose_reached(self):
        """
        A simple helper method to see if the error is below our margins
        """
        return self.error[1] <= self.linear_margin and self.error[3] <= self.rotation_margin
    
    def is_pose_reached_fast(self):
        """
        A simple helper method to see if the error is below our margins
        """
        return self.error[1] <= self.linear_margin_fast


    def apply_software_limits(self, speed):
        """
        Enforces software-defined workspace limits regardless of control mode.
        Applies soft limits to keep the end-effector in a safe region.
        """
        limited_speed = speed.copy()

        x, y, z, _, _, _ = self.pose

        # === X axis limits ===
        if x < -0.6:
            limited_speed[0] = max(speed[0], 0)
        elif x > 0.42:
            limited_speed[0] = min(speed[0], 0)

        # === Y axis limits (circular exclusion zone near edge) ===
        # Exclusion zone parameters
        exclusion_center = np.array([0.0, 0.0])  # center of the danger zone (tweak this!)
        exclusion_radius = 0.33  # radius of the exclusion zone

        current_pos = np.array([x, y])
        distance = np.linalg.norm(current_pos - exclusion_center)

        # If we're inside the exclusion zone
        if distance < exclusion_radius:
            # Allow only limited motion in +Y (away from danger zone)
            unit_vec_out = (current_pos - exclusion_center) / (distance + 1e-6)
            projected_speed = np.dot(speed[:2], unit_vec_out)

            # Smooth clamping factor: gradually increase limit as we go deeper
            max_speed_toward_center = np.interp(distance, [0.0, exclusion_radius], [0.0, 0.02])

            # If moving deeper into the exclusion zone, clamp that component
            if projected_speed < 0:
                speed_out = unit_vec_out * max(projected_speed, -max_speed_toward_center)
                limited_speed[0] = speed_out[0]
                limited_speed[1] = speed_out[1]

        # Hard Y limit beyond which we must move back in
        if y > 0.0:
            limited_speed[1] = min(speed[1], 0)
        elif y < -0.85:
            limited_speed[1] = max(speed[1], 0)

        # === Z axis limits ===
        if z > 0.5:
            limited_speed[2] = min(speed[2], 0)
        elif z < -0.081:
            limited_speed[2] = max(speed[2], 0)

        return limited_speed


    def update_bottle(self, timestep = .1):
        if self.state == "WAIT":
            first, display = self.cv_input.bottle_identification()
            #print (f"Current Bottle: {self.current_bottle} First Bottle: {first}")
            if self.current_bottle == None: #if there was no bottle, create one
                if first:
                    self.current_bottle = first
            elif first: #current bottle is not None

                first.update(self.current_bottle, timestep)
                self.current_bottle = first
        elif self.state == "MOVE_OVER_BOTTLE" or self.state == "LOWER_ON_BOTTLE":
            step_x = self.current_bottle.get_x() + self.current_bottle.get_velocity()*timestep
            self.current_bottle.set_x(step_x)
            display = self.cv_input.bottle_identification(display_only=True)
        else:
            display = self.cv_input.bottle_identification(display_only=True)
        return display
    
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

        #get current force
        yForce = self.rtde_r.getActualTCPForce()[2] #force of end effector
        
        #get joystick data
        self.got_joystick, self.joystick_data = self.joystick.read_serial()

        speed = [0,0,0,0,0,0]

        #get cv data
        cv_dt = now - self.cv_last_time
        if (cv_dt >= self.cv_time_step):
            self.cv_last_time = now
            display = self.update_bottle(cv_dt)
            cv2.imshow("Output", display)
            cv2.waitKey(1)

            if self.current_bottle is not None:
                self.bottleX = round(0.1 + self.current_bottle.get_x(), 4) #changes around here for camera offset
                self.bottleY = round(-0.36 + self.current_bottle.get_y(), 4)
                self.bottle_color = self.current_bottle.get_color()
                if self.joystick_data[5] == 1:
                    self.bottle_color = "SHARED"

                if(self.bottleY < -0.5):
                    self.bottleY = self.bottleY + (0.05 * self.bottleY)
                
                if (self.bottleX > 0.1):
                    self.bottleX = self.bottleX - (0.01 * self.bottleX)
                print(self.current_bottle)
                #print(f"Bottle Color: {self.bottle_color}, BottleX: {self.bottleX}, Bottle Certainty: {self.current_bottle.get_uncertainty()}, Bottle Status: {self.current_bottle.get_status()}")
            

            
        #decide if we are in joystick mode operate accordingly
        if (self.joystick_data[0] == 0):
            self.state = "GO_TO_NEUTRAL"
            self.current_bottle = None
            speed[0] = self.joystick_data[1] * self.JOYSTICK_SCALE_XY # X velocity -1 to 1 centered at 0
            speed[1] = self.joystick_data[2] * self.JOYSTICK_SCALE_XY # Y velocity -1 to 1 centered at 0
            speed[2] = self.joystick_data[3] * self.JOYSTICK_SCALE_Z # Z velocity -1 to 1 centered at 0
            speed[3] = self.joystick_data[4] * self.JOYSTICK_SCALE_XY# Omega velocity -1 to 1 centered at 0
        
        elif(self.joystick_data[0] == 1):
            #Autonomous mode
            if self.state == "GO_TO_NEUTRAL":
                #WE are moving to neutral
                target = self.neutral_pose
                speed, _ = self.compute_velocity_to_pose(self.pose, target)

                #evaluate success criteria
                if (self.is_pose_reached()):
                    self.state = "WAIT"
            elif self.state == "WAIT":
                #WE are waiting for the next bottle       
                speed = [0,0,0,0,0,0]
    
                #evaluate success criteria
                #if self.bottleX is not None and self.bottleY is not None:
                if (self.current_bottle != None) and (self.bottle_color == "orange"):
                    self.state = "WAIT"
                    self.current_bottle = None
                if (self.current_bottle != None) and (self.current_bottle.get_status() == "ready"):
                    self.state = "MOVE_OVER_BOTTLE"
            elif self.state == "MOVE_OVER_BOTTLE":
                success = False
                #We are moving above the bottle
                target = self.make_pose(self.bottleX, self.bottleY, self.safe_height)
                speed, _ = self.compute_velocity_to_pose(self.pose, target)

                #Evaluate success criteria (Over top of bottle)
                if(self.is_pose_reached_fast()):
                    self.state = "LOWER_ON_BOTTLE"
            elif self.state == "LOWER_ON_BOTTLE":
                success = False

                #WE are lowering to pick up the bottle
                self.set_actuator("PICKUP")
                target = self.make_pose(self.bottleX, self.bottleY, self.pickup_height)
                speed, _ = self.compute_velocity_to_pose(self.pose, target)

                #evaluate success criteria (Bottle has been picked up / we have completely lowered)
                if yForce > 10: #ange this for force threshold in y direction (how hard pressing on bottle)
                    success = True
                if (self.pose[2] < -0.079 and yForce < 5):
                    self.current_bottle = None
                    self.state = "GO_TO_NEUTRAL"
                if(success):
                    self.state = "GO_UP_A_BIT"

                    if (self.bottleY > -0.2):
                        self.state = "GO_TO_BIN"

            elif self.state == "GO_UP_A_BIT":
                success = False

                #WE are going up a bit
                target = self.make_pose(self.bottleX, self.bottleY, self.safe_height)
                speed, _ = self.compute_velocity_to_pose(self.pose, target)

                #Evaluate success criteria (We have made it to the bin)
                if (self.is_pose_reached()):
                    self.state = "GO_TO_BIN"
            elif self.state == "GO_TO_BIN":
                success = False
                
                #WE are going to the correct bin
                
                speed, _ = self.compute_velocity_to_pose(self.pose, self.bin_poses[self.bottle_color])

                #Evaluate success criteria (We have made it to the bin)
                if (self.is_pose_reached()):
                    #force into throw for testing purposes
                    if self.bottle_color == "blue":
                        self.throw = True 
                    elif self.bottle_color == "yellow":
                        self.throw = True
                    elif self.bottle_color == "clear":
                        self.throw = False
                    if self.throw:
                        self.state = "THROW_BOTTLE"
                    else:
                        self.state = "DROP_BOTTLE"
            elif self.state == "DROP_BOTTLE":
                success = False

                #We are dropping the bottle
                self.set_actuator("DROP")

                speed = [0,0,0,0,0,0]
                time.sleep(1.1)
                success = True
                #Evaluate success criteria (Bottle has been dropped)
                if(success):
                    self.state = "GO_TO_NEUTRAL"
                    self.current_bottle = None
            elif self.state == "THROW_BOTTLE":
                success = False

                if self.bottle_color == "blue":
                    target = self.bin_poses["blue"]
                    speed, _ = self.compute_velocity_to_pose_fast(self.pose, target)
                    if self.error[1] < 0.08:
                        original_pose = self.rtde_r.getActualQ()
                        speed = [0,0,0,-1,0,0] #[Base, Shoulder, Elbow, Wrist1, Wrist2, Wrist3]
                        self.rtde_c.speedJ(speed, 4, .5)
                        time.sleep(.5)
                        self.set_actuator("DROP")
                        time.sleep(.5)
                        self.rtde_c.speedJ([0,0,0,0,0,0], 2, 0.1)
                        time.sleep(0.1)
                        self.moveJ(original_pose)
                        time.sleep(0.5)
                        success = True

                elif self.bottle_color == "yellow":
                    target = self.bin_poses["yellow"]
                    speed, _ = self.compute_velocity_to_pose(self.pose, target)
                    if self.error[1] < 0.08:
                        original_pose = self.rtde_r.getActualQ()
                        speed = [0,0,0,-1,0,0] #[Base, Shoulder, Elbow, Wrist1, Wrist2, Wrist3]
                        self.rtde_c.speedJ(speed, 4, .5)
                        time.sleep(.5)
                        self.set_actuator("DROP")
                        time.sleep(.5)
                        self.rtde_c.speedJ([0,0,0,0,0,0], 2, 0.1)
                        time.sleep(0.1)
                        self.moveJ(original_pose)
                        time.sleep(0.5)
                        success = True

                elif self.bottle_color == "clear":
                    target = self.bin_poses["clear"]
                    speed, _ = self.compute_velocity_to_pose(self.pose, target)
                    if self.error[1] < 0.08:
                        original_pose = self.rtde_r.getActualQ()
                        speed = [0,0,0,-1,0,0] #[Base, Shoulder, Elbow, Wrist1, Wrist2, Wrist3]
                        self.rtde_c.speedJ(speed, 4, .5)
                        time.sleep(.5)
                        self.set_actuator("DROP")
                        time.sleep(.5)
                        self.rtde_c.speedJ([0,0,0,0,0,0], 2, 0.1)
                        time.sleep(0.1)
                        self.moveJ(original_pose)
                        time.sleep(0.5)
                        success = True
                
                if(success):
                    speed = [0,0,0,0,0,0]
                    self.state = "GO_TO_NEUTRAL"
                    self.current_bottle = None
            else:
                speed = [0,0,0,0,0,0]

        #safety check
        speed = self.apply_software_limits(speed)

        self.rtde_c.speedL(speed, 3, 0) #send speed data to the UR5

        pose = [round(i,2) for i in self.pose]
        
        return {
        "pose": pose,
        "speed": speed,
        "joystick": self.joystick_data,
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

