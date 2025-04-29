# gui/gui_main.py

import sys
import signal
import random
import esp_interface.esp_interface as esp
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QLabel, QPushButton
)
from PyQt5.QtCore import QTimer

# Flag to use mock data
USE_MOCK_CONTROLLER = False

# --- Mock Controller ---
class MockSystemController:
    def __init__(self):
        self.counter = 0
        self.joystick = False

        try:
            self.esp_device = esp.ESPInterface("/dev/ttyACM0")
            self.joystick = True
        except Exception as e:
            print("No Joystick Connected")
            print(e)

    def step(self):
        self.counter += 1

        # Simulate pose data slowly moving
        pose = [(0.1 * (self.counter + i)) % 6 for i in range(6)]

        # Simulate TCP speed fluctuating
        speed = [(0.01 * random.uniform(-1, 1)) for _ in range(6)]

        # Simulate joystick toggling control modes and pickup/thrower
        if (self.joystick):
            got_data, joystick_data = self.esp_device.read_serial()

                #decide if we are in joystick mode operate accordingly
            if (joystick_data[0] == 0):
                joy_x = joystick_data[1] # -1 to 1 centered at 0
                joy_y = joystick_data[2] # -1 to 1 centered at 0
                joy_z = joystick_data[3] # -1 to 1 centered at 0
                endpoint_omega = joystick_data[4] #-1 to 1 cetnered at 0
            elif(joystick_data[0] == 1):
                joy_x = 0
                joy_y = 0
                joy_z = 0
                endpoint_omega = 0


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
        else:
            mode = 0 if self.counter % 20 < 10 else 1  # Toggle every ~1s
            pickup_active = 1 if random.random() < 0.1 else 0  # Randomly activate
            thrower_active = 1 if random.random() < 0.1 else 0

            joystick_data = [
                mode,
                random.uniform(-1, 1),
                random.uniform(-1, 1),
                random.uniform(-1, 1),
                random.uniform(-1, 1),
                pickup_active,
                thrower_active
            ]

        return {
            "pose": pose,
            "speed": speed,
            "joystick": joystick_data
        }

    def shutdown(self):
        print("[MockSystemController] Shutdown called.")

# --- Real Controller (import only if needed) ---
if not USE_MOCK_CONTROLLER:
    from control.system_controller import SystemController

# --- Main GUI Class ---
class ControlGUI(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("UR5 Debug Panel")
        self.setGeometry(100, 100, 400, 500)

        # Controller setup
        if USE_MOCK_CONTROLLER:
            print("[GUI] Using MOCK controller.")
            self.system = MockSystemController()
        else:
            print("[GUI] Using REAL SystemController.")
            self.system = SystemController()

        self.last_result = None

        # Layout setup
        self.layout = QVBoxLayout()

        self.mode_label = QLabel("Mode: Unknown")
        self.mode_label.setStyleSheet("font-size: 18px; font-weight: bold;")
        self.status_label = QLabel("Status: Idle")
        self.shutdown_button = QPushButton("Shutdown")

        self.shutdown_button.clicked.connect(self.shutdown_system)

        self.layout.addWidget(self.mode_label)
        self.layout.addWidget(self.status_label)
        self.layout.addWidget(self.shutdown_button)
        self.setLayout(self.layout)
        
        # Timers
        self.control_timer = QTimer()
        self.control_timer.timeout.connect(self.run_step_fast)
        self.control_timer.start(5)  # fast loop (~200Hz)

        self.gui_timer = QTimer()
        self.gui_timer.timeout.connect(self.update_gui)
        self.gui_timer.start(200)  # slow loop (5Hz)

    def run_step_fast(self):
        self.last_result = self.system.step()

    def update_gui(self):
        if self.last_result is None:
            return

        result = self.last_result
        pose = result["pose"]
        speed = result["speed"]
        joystick = result["joystick"]
        error = result["error"]

        mode = "Joystick" if joystick[0] == 0 else "Autonomous"
        pickup_active = "Yes" if joystick[5] else "No"
        thrower_active = "Yes" if joystick[6] else "No"

        # Update Mode Label with color
        if mode == "Autonomous":
            self.mode_label.setText("⚠️ AUTONOMOUS MODE ACTIVE ⚠️")
            self.mode_label.setStyleSheet("color: red; font-size: 22px; font-weight: bold;")
        else:
            self.mode_label.setText("Manual Control (Joystick)")
            self.mode_label.setStyleSheet("color: green; font-size: 22px; font-weight: bold;")

        # Update Status Label
        status_text = (
            "<b>UR5 TCP Pose</b><br>"
            f"X Position: {pose[0]:.3f} m<br>"
            f"Y Position: {pose[1]:.3f} m<br>"
            f"Z Position: {pose[2]:.3f} m<br>"
            f"Rotation X: {pose[3]:.3f} rad<br>"
            f"Rotation Y: {pose[4]:.3f} rad<br>"
            f"Rotation Z: {pose[5]:.3f} rad<br>"

            "<b>TCP Speed</b><br>"
            f"X Speed: {speed[0]:.2f} m/s<br>"
            f"Y Speed: {speed[1]:.2f} m/s<br>"
            f"Z Speed: {speed[2]:.2f} m/s<br>"
            f"Rotation Speed: {speed[5]:.2f} rad/s<br><br>"

            "<b>Joystick / Control Data</b><br>"
            f"Mode: {mode}<br>"
            f"Command X: {joystick[1]:.2f}<br>"
            f"Command Y: {joystick[2]:.2f}<br>"
            f"Command Z: {joystick[3]:.2f}<br>"
            f"Command Rotation: {joystick[4]:.2f}<br>"
            f"Pickup Active: {pickup_active}<br>"
            f"Thrower Active: {thrower_active}<br>"
        )

        self.status_label.setText(status_text)

    def shutdown_system(self):
        self.control_timer.stop()
        self.gui_timer.stop()
        self.system.shutdown()
        self.status_label.setText("System shutdown.")

    def closeEvent(self, event):
        print("[GUI] Window closed. Running system shutdown...")
        self.shutdown_system()
        event.accept()

# --- Launch GUI ---
def launch_gui():
    app = QApplication(sys.argv)
    window = ControlGUI()
    window.show()

    # Handle Ctrl+C clean shutdown
    def handle_sigint(*args):
        print("[GUI] SIGINT received. Shutting down...")
        window.shutdown_system()
        window.close()

    signal.signal(signal.SIGINT, handle_sigint)

    # Keep app responsive to SIGINT
    timer = QTimer()
    timer.start(100)
    timer.timeout.connect(lambda: None)

    sys.exit(app.exec_())

if __name__ == "__main__":
    launch_gui()
