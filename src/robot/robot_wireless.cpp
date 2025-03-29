#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "wireless.h"

joystickData joystick;

bool freshWirelessData = false;

void onDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    memcpy(&joystick, incomingData, sizeof(joystick));
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
    joystick.x = 0;
    joystick.y = 0;
}