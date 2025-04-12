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

        # Autonomous path parameters
        self.autonomous_path = self.generate_circle_path(radius=0.1, center=[-0.5, -0.3, 0.1], num_points=36)
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
    

    def compute_velocity_to_pose(self, current_pose, target_pose, max_speed=0.05):
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

        angular_velocity = [0, 0, 0]  # No orientation changes for now

        return np.concatenate((linear_velocity, angular_velocity)).tolist(), distance
    
    def apply_software_limits(self, speed):
        """
        Enforces software-defined workspace limits regardless of control mode.
        """
        limited_speed = speed.copy()

        # X axis limits
        if self.pose[0] > -0.3:
            limited_speed[0] = min(speed[0], 0)
        elif self.pose[0] < -0.7:
            limited_speed[0] = max(speed[0], 0)

        # Y axis limits
        if self.pose[1] > -0.1:
            limited_speed[1] = min(speed[1], 0)
        elif self.pose[1] < -0.5:
            limited_speed[1] = max(speed[1], 0)

        # Z axis limits
        if self.pose[2] > 0.25:
            limited_speed[2] = min(speed[2], 0)
        elif self.pose[2] < 0.06:
            limited_speed[2] = max(speed[2], 0)

        return limited_speed

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
            speed[0] = self.joystick_data[1] # X velocity -1 to 1 centered at 0
            speed[1] = self.joystick_data[2] # Y velocity -1 to 1 centered at 0
            speed[2] = self.joystick_data[3] # Z velocity -1 to 1 centered at 0
            speed[5] = self.joystick_data[4] # Omega velocity -1 to 1 cetnered at 0
        elif(self.joystick_data[0] == 1):
            # Autonomous mode
            target_pose = self.autonomous_path[self.target_index]
            speed, distance = self.compute_velocity_to_pose(self.pose, target_pose)

            # If close enough to the target, move to the next one
            if distance < self.lookahead_distance:
                self.target_index = (self.target_index + 1) % len(self.autonomous_path)

        #safety check
        speed = self.apply_software_limits(speed)

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

