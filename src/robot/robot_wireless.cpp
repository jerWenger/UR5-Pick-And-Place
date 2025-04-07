#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "wireless.h"

ourControllerData dual_joystick;

bool freshWirelessData = false;

void onDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    memcpy(&dual_joystick, incomingData, sizeof(dual_joystick));
    freshWirelessData = true;
    // Serial.printf("JOYSTICK: X %d   Y %d\n", joystick.x, joystick.y);
}

void setupWireless(){
    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    //Tell the microcontroller which functions to call when
    //data is sent or received
    esp_now_register_recv_cb(onDataRecv);
    
    // ESP-NOW Setup Complete

    // Default the joystick values
    dual_joystick.x = 0;
    dual_joystick.y = 0;
    dual_joystick.z = 0;
    dual_joystick.t = 0;
    dual_joystick.control_state = 0;
    dual_joystick.pickup_state = 0;
    dual_joystick.thrower_state = 0;
}