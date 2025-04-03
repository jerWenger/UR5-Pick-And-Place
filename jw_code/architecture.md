# Code Architecture

This document serves as a central location to organize the code, understand what each component does, and standardize class structures and interface methods.

# TO DO:
 - Determine functional requirements for each script
 - Assign code tasks to team members
 - Build code framework
 - Build testing framework
 - Implement main.py 
 - Implement esp_interface.py
 - Implement cv_interface.py
 - Expand system_controller.py for shared runtime logic
 - Build initial GUI interface

# Table of Contents
1. main.py
    - Startup
    - While Loop
    - Shutdown
2. control/system_controller.py
    - Central runtime control
    - Shared interface to CV and ESP modules
    - Step and shutdown functions
3. gui/gui_main.py
    - GUI wrapper using PyQt5
    - Visualization and debug tools
4. esp_interface/esp_interface.py
    - Class interface to ESP32
    - Init, Getters, Setters, Aux
5. cv_interface/cv_interface.py
    - Class interface to CV camera
    - Init, Getters, Setters, Aux

## main.py
This is the CLI entry point of the system. It creates a SystemController instance and calls `step()` in a loop.

### Startup
Initializes SystemController.

### While Loop
Calls `step()` until a KeyboardInterrupt is received.

### Shutdown
Calls `shutdown()` on the SystemController.

## control/system_controller.py
Encapsulates all runtime logic. This is where CV and ESP interfaces are coordinated.
- `step()` executes a single control iteration.
- `shutdown()` handles cleanup.

## gui/gui_main.py
Optional PyQt5 GUI to interact with the system visually.
- Shows live state updates
- Provides interactive control and debug tools

## esp_interface/esp_interface.py
Handles communication with the ESP32 modules (joystick, end-effector).
- Abstract class with generic `get`, `set`, and `aux_function` methods.

## cv_interface/cv_interface.py
Handles the connection to the RealSense camera (or other CV hardware).
- Abstract class with generic `get`, `set`, and `aux_function` methods.
