# UR5-Pick-And-Place
## Team Ducklings: Corinna Berger, Allen Chen, Rian Evans, Shane Lovett, Tom Nguyen, Helena Usey, Jeremy Wenger

## Project Overview:
This project was an effort to detect, segment, pickup, and sort plastic bottles on a moving conveyor belt. It leverages a UR5 robot and an Intel Realsense camera. 

## Dependencies:
- UR_RTDE
- Open CV
- Pyrealsense
- Numpy
- Pyserial
- PyQT5

## How to run:
- Clone repository
- Install dependencies
- In jw_code/control/system_controller.py: specify ESP32 port and UR5 IP address
- In jw_code run: python3 -m gui.gui_main

## Organization
This project is organized into two main sections: the ESP32 code (C++) as well as the python code that runs the entire system. The main directory /src includes two subdirectories: robot and joystick. The joystick code runs on the ESP32 connected to the joystick controller. The robot code runs on the ESP32 that interfaces between the joystick, computer, and actuators. In the main directory /jw_code includes all of the python code that maintains the rest of the project. This is broken down into python modules that can be run seperately or imported as modules to be run in a main script. The modules are as follows:

- Control: The main portion of the project that handles bringing most of the submodules together. Contains the Finite State Machine (FSM) as well as the UR5 control and safety software. 
- ESP Interface: A small submodule that acts as a wapper for the pyserial and handles communication between the joystick and the robot interface ESP32.
- CV Interface: Contains the code responsible for interfacing with the Intel RealSense camera as well as the open cv based bottle segmentation and tracking. 
- GUI: A lightweight PyQt5 gui that calls the init and step function of system_controller and updates a GUI to display helpful system data like pose, speed, and FSM status.


## Strategy:
The strategy for identifying and picking up bottles is based in breaking down the task into smaller definite tasks that are linked together. At a high level this looks like: 

Identify bottle Pose, Color, and Velocity -> Pickup bottle -> Place (or throw) bottle in the desired bin -> reset for next bottle

## Finite State Machine:
Due to the linear nature of our task a finite state machine (FSM) made most sense for triggering tasks, executing them, then evaluating their success and whether or not to begin the next task. This section lists each state, what is executed in said state, how success is evaluated, and what next state is triggered. The state name alone gives a decent idea of what each state is meant to achieve. 

### GO_TO_NEUTRAL
- Trigger: system init, failed pickup, successful drop or throw
- Task: return to a central, known location
- Success Criteria: current pose is less than 1 cm from setpoint
- Next State: WAIT

### WAIT
- Trigger: GO_TO_NEUTRAL success
- Task: wait for valid computer vision data
- Success Criteria: if there is a bottle of the appropriate color and it is within the valid execution zone of the workspace
- Next State: MOVE_OVER_BOTTLE

### MOVE_OVER_BOTTLE
- Trigger: WAIT has received a valid bottle
- Task: Move over bottle and track its velocity
- Success Criteria: if the tcp is within 1cm of the desired bottle centroid
- Next State: LOWER_ON_BOTTLE

### LOWER_ON_BOTTLE
- Trigger: MOVE_OVER_BOTTLE has reached the bottle centroid
- Task: Turn on suction, lower end effector and monitor end effector force
- Success Criteria: end effector force breaches a threshold or end effector reaches the lower z limit
- Next State: if force threshold: Go_TO_BIN; if height threshold: GO_TO_NEUTRAL (attempt a regrab)

### GO_TO_BIN
- Trigger: Force success in LOWER_ON_BOTTLE
- Task: Translate to bin pose based on bottle color
- Success Criteria: current pose is less than 1 cm from setpoint
- Next State: if throwing is enabled: THROW otherwise: DROP

### DROP
- Trigger: Successful GO_TO_BIN
- Task: Turn of suction and wait 1.1 seconds
- Success Criteria: 1.1 second wait complete
- Next State: GO_TO_NEUTRAL

### THROW
- Trigger: Successful GO_TO_BIN and THROW is set to true (Based on bottle color)
- Task: Rotate end effector and drop
- Success Criteria: return to original end effector position 
- Next State: GO_TO_NEUTRAL

