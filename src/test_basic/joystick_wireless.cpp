#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <UMS3.h>
#include "wireless.h"
#include "pinout.h"
#include "util.h"


// TODO: Replace with the mac address of the robot.
uint8_t broadcastAddress[] = {0xEC, 0xDA, 0x3B, 0x41, 0xA3, 0xC0}; //This is in our bin

// Note: You do not need to define struct joystickData.
// joystickData has been defined in wireless.h
joystickData joystick;
esp_now_peer_info_t peerInfo;


// TODO: Initialize any other global variable/s.
// Hint: You only need one for your code to work at the bare minimum.

// TODO: Define the callback function when data is sent.
// Consider printing the data to Serial before sending.
// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 

void setup(void){
  Serial.begin();

    // TODO: Initialize Serial communication.

    // TODO: Fill in code to set up ESP_NOW.
    // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

    // TODO: Initialize joystick pins as INPUT.
    pinMode(X_PIN, INPUT);
    pinMode(Y_PIN, INPUT);

}

void loop(){


  // Read and send joystick data at 20Hz
  EVERY_N_MILLIS(50) {

    // TODO: Read data from joystick pins.
      joystick.x = analogRead(X_PIN);//(analogRead(X_PIN)/2048.0)-1.0;
      joystick.y = analogRead(Y_PIN);//(analogRead(Y_PIN)/2048.0)-1.0;
    // TODO: Populate joystick.x and joystick.y.

    // TODO: Send message via ESP-NOW.
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &joystick, sizeof(joystick));

    // Note: Do not place a delay here. Everything in EVERY_N_MILLIS(50) 
    // already runs every 50ms, so placing a delay will only lengthen 
    // the read/send cycle period.
  }
}