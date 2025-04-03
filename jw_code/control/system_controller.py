# gui/gui_main.py

import sys
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QLabel, QPushButton
)
from cv_interface.cv_interface import CVInterface
from esp_interface.esp_interface import ESPInterface

class ControlGUI(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("UR5 Debug Panel")
        self.setGeometry(100, 100, 400, 200)

        self.cv = CVInterface()
        self.esp = ESPInterface()

        self.layout = QVBoxLayout()

        self.status_label = QLabel("Status: Idle")
        self.debug_button = QPushButton("Run Debug")

        self.debug_button.clicked.connect(self.run_debug)

        self.layout.addWidget(self.status_label)
        self.layout.addWidget(self.debug_button)
        self.setLayout(self.layout)

    def run_debug(self):
        # Example call to your interfaces
        self.cv.set("test", "debug_value")
        self.esp.set("mode", "debug")

        cv_val = self.cv.get("test")
        esp_val = self.esp.get("mode")

        self.status_label.setText(f"CV: {cv_val}, ESP: {esp_val}")

def launch_gui():
    app = QApplication(sys.argv)
    window = ControlGUI()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    launch_gui()
