#include <Arduino.h>
#include "pinout.h"
#include "MotorDriver.h"
#include "EncoderVelocity.h"
#include "PID.h"
#include "util.h"
#include "robot_drive.h"

// TODO: Initialize motors, encoders, and PID controllers.
// Kp, Ki, Kd, tau have all been defined in robot_drive.h, so you may use them here.
// Hint: Each motor-encoder pair needs its own PID controller.

// TODO: Initialize setpoints, velocities, and control efforts.

void setup() {

    Serial.begin();

    setupWireless();

    // TODO: Set up motors.

}

void loop() {

    // Update setpoint at 50Hz
    EVERY_N_MILLIS(20) {
        // Flag that checks if there is a new message received
        if (freshWirelessData) {
            // TODO: Set the setpoints here.
            // For driving forward, set all of them to 2.
            // For driving via joystick, set them to be
            // dependent on joystick.x and joystick.y.

        }
        // Note: Do not place a delay here.
    }

    // Update PID at 200Hz
    EVERY_N_MILLIS(5) {
        // TODO: For each motor, get encoder velocity, calculate control effort,
        // and then drive the motor using control effort.
        // Use motor_velocity_control.cpp as a reference.
        // Hint: Some motors are wired to drive in the opposite direction 
        // as motor 1. You may need to negate controlEffort.



        // Note: Do not place a delay here.
    }

    // Print values at 20Hz
    EVERY_N_MILLIS(50) {
        // TODO: Consider changing this to print other variables you need to check.
        Serial.printf("%.3f,%.3f\n",
                    (joystick.x/2048.0)-1.0, (joystick.y/2048.0)-1.0);
    }

}