#include <Arduino.h>
#include <UMS3.h>
#include <Bounce2.h>
#include <esp_now.h>
#include <WiFi.h>
#include <wireless.h>
#include "util.h"

//set pins for reading the controller
#define CONTROL_PIN 12
#define PICKUP_PIN 40
#define DROP_PIN 41
#define THROW_PIN 42

#define Y_AXIS A7
#define X_AXIS A8

#define Z_AXIS A6
#define T_AXIS A5

//set default / init values
int X = 0;
int Y = 0;
int Z = 0;
int T = 0;

bool control_state = LOW;
bool pickup_state = LOW;
bool drop_state = LOW;
bool throw_state = LOW;

//configure LED and buttons
UMS3 ums3;
Bounce2::Button control = Bounce2::Button();
Bounce2::Button pickup = Bounce2::Button();
Bounce2::Button drop = Bounce2::Button();
Bounce2::Button thrower = Bounce2::Button();

//address to send to 
uint8_t broadcastAddress[] = {0xEC, 0xDA, 0x3B, 0x41, 0xA3, 0xC0};

//set peer and joystick data types
ourControllerData dual_joystick;
esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


void setup() {
  // Initialize all board peripherals, call this first
  ums3.begin();
  control.attach( CONTROL_PIN ,  INPUT_PULLUP );
  control.setPressedState(LOW);
  control.interval(5); 

  pickup.attach( PICKUP_PIN ,  INPUT_PULLUP );
  pickup.setPressedState(LOW);
  pickup.interval(5); 

  drop.attach( DROP_PIN ,  INPUT_PULLUP );
  drop.setPressedState(LOW);
  drop.interval(5); 

  thrower.attach( THROW_PIN ,  INPUT_PULLUP );
  thrower.setPressedState(LOW);
  thrower.interval(5); 

  pinMode(Y_AXIS, INPUT);
  pinMode(X_AXIS, INPUT);
  pinMode(Z_AXIS, INPUT);
  pinMode(T_AXIS, INPUT);



  // Brightness is 0-255.
  ums3.setPixelBrightness(150);

  //after local hardware is set, set up comms
  Serial.begin();
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

}

int color = 0;

void loop() {
  EVERY_N_MILLIS(5) {
    control.update();
    pickup.update();
    drop.update();
    thrower.update();

    X = analogRead(X_AXIS);
    Y = 4095 - analogRead(Y_AXIS);
    Z = 4095 - analogRead(Z_AXIS);
    T = analogRead(T_AXIS);

    // colorWheel cycles red, orange, ..., back to red at 256
    if (control.fell()){
      control_state = !control_state;
    }

    if (pickup.changed()){
      pickup_state = HIGH;
    }

    if (drop.changed()){
      pickup_state = LOW;
    }

    if (thrower.changed()){
      pickup_state = LOW;
      throw_state = thrower.isPressed();
    }

    if(control_state)
      ums3.setPixelColor(UMS3::colorWheel(0));
    else
      ums3.setPixelColor(UMS3::colorWheel(100));

    //Serial.printf("X: %d Y: %d Z: %d T: %d Control: %d Pickup: %d Throw: %d\n", X, Y, Z, T, control_state, pickup_state, throw_state);

    dual_joystick.x = X;
    dual_joystick.y = Y;
    dual_joystick.z = Z;
    dual_joystick.t = T;
    dual_joystick.control_state = control_state;
    dual_joystick.pickup_state = pickup_state;
    dual_joystick.thrower_state = throw_state;

    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &dual_joystick, sizeof(dual_joystick));
  }
}
