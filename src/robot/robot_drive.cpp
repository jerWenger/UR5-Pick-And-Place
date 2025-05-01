#include <Arduino.h>
#include "util.h"
#include "robot_drive.h"

#define VACUUM_PIN 40
#define PURGE_PIN 41
#define SOLENOID_PIN 14

void flushSerialInput(unsigned long maxDurationMs = 10) {
    unsigned long startTime = millis();
    while (Serial.available() > 0) {
        Serial.read();
        if (millis() - startTime > maxDurationMs) {
            break; // Safety break
        }
    }
}

void setSystemState(bool vacuum, bool purge, bool solenoid) {
    digitalWrite(VACUUM_PIN, vacuum ? HIGH : LOW);
    digitalWrite(PURGE_PIN, purge ? HIGH : LOW);
    digitalWrite(SOLENOID_PIN, solenoid ? LOW : HIGH);
}

int autonomous_pickup_state = 0;
int autonomous_thrower_state = 0;
bool freshLaptopData = false;
bool laptopDataReceived = false;
unsigned long lastFlushTime = 0;
const unsigned long flushInterval = 30000;
unsigned long dropModeStartTime = 0;
bool inDropMode = false;

void setup() {

    Serial.begin();
    setupWireless();

    pinMode(VACUUM_PIN, OUTPUT);
    pinMode(PURGE_PIN, OUTPUT);
    pinMode(SOLENOID_PIN, OUTPUT);
}

void loop() {
    if (Serial.available() > 0){
        String incoming = Serial.readStringUntil('\n');
        incoming.trim();

        int commaIndex = incoming.indexOf(',');
        if (commaIndex != -1){
            String pickupStr = incoming.substring(0, commaIndex);
            String throwerStr = incoming.substring(commaIndex + 1);

            autonomous_pickup_state = pickupStr.toInt();
            autonomous_thrower_state = throwerStr.toInt();
            freshLaptopData = true;
            laptopDataReceived = true;
        }
    }

    // Periodic flush every ~10 seconds
    if (millis() - lastFlushTime >= flushInterval) {
        flushSerialInput(10); // Try flushing for up to 10 ms
        lastFlushTime = millis(); // Reset timer
    }

    // Update setpoint at 50Hz
    EVERY_N_MILLIS(5) {
        // Track if we are currently in drop mode
        bool currentlyInDropMode = false;

        if (freshWirelessData || freshLaptopData) {
            if (dual_joystick.control_state == 0) {
                // Joystick mode
                if (dual_joystick.pickup_state == 1) {
                    setSystemState(true, false, false);  // pickup
                    currentlyInDropMode = false;
                }
                else {
                    // drop
                    currentlyInDropMode = true;
                }
            }
            else if ((dual_joystick.control_state == 1) && laptopDataReceived) {
                // Autonomous mode
                if (autonomous_pickup_state == 1) {
                    setSystemState(true, false, false);  // pickup
                    currentlyInDropMode = false;
                }
                else {
                    // drop
                    currentlyInDropMode = true;
                }
            }
            else {
                // Fallback broken state — treat as drop mode
                currentlyInDropMode = true;
            }

            if (currentlyInDropMode) {
                if (!inDropMode) {
                    dropModeStartTime = millis();  // just entered drop mode
                    inDropMode = true;
                }

                // Determine whether to turn off solenoid
                if (millis() - dropModeStartTime > 2000) {
                    setSystemState(false, false, false); // Solenoid LOW after timeout
                } else {
                    setSystemState(false, true, true);  // Normal drop behavior
                }
            } else {
                inDropMode = false;  // reset drop mode timer
            }

            freshWirelessData = false;
            freshLaptopData = false;
            }
        }

    
    EVERY_N_MILLIS(20) {
        // TODO: Consider changing this to print other variables you need to check.
        Serial.printf("%d,%.3f,%.3f,%.3f,%.3f,%d,%d\n",
                    dual_joystick.control_state, (dual_joystick.x/2048.0)-1.0, (dual_joystick.y/2048.0)-1.0, (dual_joystick.z/2048.0)-1.0, (dual_joystick.t/2048.0)-1.0, dual_joystick.pickup_state, dual_joystick.thrower_state);
    }

}

