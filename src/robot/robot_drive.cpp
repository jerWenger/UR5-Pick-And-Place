#include <Arduino.h>
#include "util.h"
#include "robot_drive.h"

int autonomous_pickup_state = 0;
int autonomous_thrower_state = 0;
bool freshLaptopData = false;

void setup() {

    Serial.begin();

    setupWireless();
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
        }
    }

    // Update setpoint at 50Hz
    EVERY_N_MILLIS(5) {
        // Flag that checks if there is a new message received
        if (freshWirelessData || freshLaptopData) {
            if (dual_joystick.control_state == 0){
                //we are in joystick mode

            }
            else if (dual_joystick.control_state == 1){
                //we are in autonomous mode
                
            }
           
        }
        // Note: Do not place a delay here.
    }

    
    EVERY_N_MILLIS(20) {
        // TODO: Consider changing this to print other variables you need to check.
        Serial.printf("%d,%.3f,%.3f,%.3f,%.3f,%d,%d\n",
                    dual_joystick.control_state, (dual_joystick.x/2048.0)-1.0, (dual_joystick.y/2048.0)-1.0, (dual_joystick.z/2048.0)-1.0, (dual_joystick.t/2048.0)-1.0, dual_joystick.pickup_state, dual_joystick.thrower_state);
    }

}