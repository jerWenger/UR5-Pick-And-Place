import sys
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QLabel, QPushButton
)
from control.system_controller import SystemController

class ControlGUI(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("UR5 Debug Panel")
        self.setGeometry(100, 100, 400, 200)

        self.system = SystemController()

        self.layout = QVBoxLayout()
        self.status_label = QLabel("Status: Idle")
        self.debug_button = QPushButton("Run Step")
        self.shutdown_button = QPushButton("Shutdown")

        self.debug_button.clicked.connect(self.run_step)
        self.shutdown_button.clicked.connect(self.shutdown_system)

        self.layout.addWidget(self.status_label)
        self.layout.addWidget(self.debug_button)
        self.layout.addWidget(self.shutdown_button)
        self.setLayout(self.layout)

    def run_step(self):
        result = self.system.step()
        self.status_label.setText(f"Joystick: {result['joystick']} | CV: {result['cv_status']}")

    def shutdown_system(self):
        self.system.shutdown()
        self.status_label.setText("System shutdown.")


def launch_gui():
    app = QApplication(sys.argv)
    window = ControlGUI()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    launch_gui()
