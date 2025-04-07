#include <Arduino.h>
#include "util.h"
#include "robot_drive.h"

// TODO: Initialize motors, encoders, and PID controllers.
// Kp, Ki, Kd, tau have all been defined in robot_drive.h, so you may use them here.
// Hint: Each motor-encoder pair needs its own PID controller.

// TODO: Initialize setpoints, velocities, and control efforts.

void setup() {

    Serial.begin();

    setupWireless();

}

void loop() {

    // Update setpoint at 50Hz
    EVERY_N_MILLIS(5) {
        // Flag that checks if there is a new message received
        if (freshWirelessData) {
            // TODO: Set the setpoints here.
            // For driving forward, set all of them to 2.
            // For driving via joystick, set them to be
            // dependent on joystick.x and joystick.y.

        }
        // Note: Do not place a delay here.
    }

    
    EVERY_N_MILLIS(20) {
        // TODO: Consider changing this to print other variables you need to check.
        Serial.printf("%d,%.3f,%.3f,%.3f,%.3f,%d,%d\n",
                    dual_joystick.control_state, (dual_joystick.x/2048.0)-1.0, (dual_joystick.y/2048.0)-1.0, (dual_joystick.z/2048.0)-1.0, (dual_joystick.t/2048.0)-1.0, dual_joystick.pickup_state, dual_joystick.thrower_state);
    }

}