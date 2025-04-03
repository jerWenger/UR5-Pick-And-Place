# control/system_controller.py

from cv_interface.cv_interface import CVInterface
from esp_interface.esp_interface import ESPInterface

class SystemController:
    def __init__(self):
        """
        Initializes the system controller with instances of the CV and ESP interfaces.
        """
        self.cv = CVInterface()
        self.esp = ESPInterface()
        self.running = True

    def step(self):
        """
        Perform a single control step. This method can be called repeatedly in a loop or manually.
        """
        joystick_input = self.esp.get("joystick") or "neutral"
        self.cv.set("last_input", joystick_input)

        return {
            "joystick": joystick_input,
            "cv_status": self.cv.get("last_input"),
            "status": "step completed"
        }

    def shutdown(self):
        """
        Perform any necessary cleanup on shutdown.
        This should safely stop any robot actions and close all interfaces.
        """
        self.running = False
        print("[SystemController] Stopping robot safely...")
        # Example cleanup steps — replace with actual hardware shutdowns
        # self.esp.send_command("stop")
        # self.cv.release()
        print("[SystemController] Shutdown complete.")

