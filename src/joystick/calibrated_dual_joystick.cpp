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

// Calibration and deadzone constants
#define CALIBRATION_SAMPLES 200  // Number of samples to take during calibration
#define DEADZONE_THRESHOLD 100  // Deadzone threshold (0-4095)
#define CALIBRATION_DELAY 5     // Delay between calibration readings in ms
#define TRUE_CENTER 2048        // The value we want to represent center position
#define CALIBRATION_PHASE_DELAY 5000 // Time between calibration phases in ms

//set default / init values
int X = 0;
int Y = 0;
int Z = 0;
int T = 0;

// Calibration values
int xCenter = 2048;
int yCenter = 2048;
int zCenter = 2048;
int tCenter = 2048;

// Min/Max values for each axis (will be determined during calibration)
int xMin = 0, xMax = 4095;
int yMin = 0, yMax = 4095;
int zMin = 0, zMax = 4095;
int tMin = 0, tMax = 4095;

bool control_state = LOW;
bool pickup_state = LOW;
bool drop_state = LOW;
bool throw_state = LOW;
bool calibrationComplete = false;

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
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Function to normalize joystick readings to ensure center is TRUE_CENTER and range is 0-4095
int normalizeJoystickValue(int value, int center, int minVal, int maxVal) {
  // First apply deadzone
  if (abs(value - center) < DEADZONE_THRESHOLD) {
    return TRUE_CENTER;
  }
  
  // Then normalize the value
  if (value < center) {
    // Value is below center, map from [minVal, center] to [0, TRUE_CENTER]
    if (center == minVal) return 0; // Avoid division by zero
    return map(value, minVal, center, 0, TRUE_CENTER);
  } else {
    // Value is above center, map from [center, maxVal] to [TRUE_CENTER, 4095]
    if (maxVal == center) return 4095; // Avoid division by zero
    return map(value, center, maxVal, TRUE_CENTER, 4095);
  }
}

// Function to do LED animation for indicating phase transitions
void animateTransition(int startColor, int endColor, int steps, int duration) {
  int delayTime = duration / steps;
  for (int i = 0; i < steps; i++) {
    int color = startColor + ((endColor - startColor) * i / steps);
    // Keep color within range
    color = color % 256;
    ums3.setPixelColor(UMS3::colorWheel(color));
    delay(delayTime);
  }
}

// Function to calibrate joysticks with visual feedback only
void calibrateJoysticks() {
  long xSum = 0, ySum = 0, zSum = 0, tSum = 0;
  
  // PHASE 1: CENTER CALIBRATION
  // ---------------------------------
  // Purple light indicates "center joysticks"
  ums3.setPixelColor(UMS3::colorWheel(0)); // Purple color
  
  // Print instructions if serial is available
  if (Serial) {
    Serial.println("Starting joystick calibration...");
    Serial.println("Please keep joysticks centered");
  }
  
  // Give user time to prepare, blinking purple
  for (int i = 0; i < 5; i++) {
    ums3.setPixelBrightness(200);
    delay(200);
    ums3.setPixelBrightness(50);
    delay(200);
  }
  ums3.setPixelBrightness(150);
  
  // Take multiple readings at center position and average them
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    xSum += analogRead(X_AXIS);
    ySum += 4095 - analogRead(Y_AXIS); // Y axis is inverted in original code
    zSum += 4095 - analogRead(Z_AXIS); // Z axis is inverted in original code
    tSum += analogRead(T_AXIS);
    
    // Pulse LED to indicate progress
    int brightness = 1 + (100 * i / CALIBRATION_SAMPLES);
    ums3.setPixelBrightness(brightness);
    delay(CALIBRATION_DELAY);
  }
  
  // Calculate center positions
  xCenter = xSum / CALIBRATION_SAMPLES;
  yCenter = ySum / CALIBRATION_SAMPLES;
  zCenter = zSum / CALIBRATION_SAMPLES;
  tCenter = tSum / CALIBRATION_SAMPLES;
  
  // PHASE 2: MIN/MAX CALIBRATION
  // ---------------------------------
  // Transition animation from purple to blue
  animateTransition(0, 100, 20, CALIBRATION_PHASE_DELAY);
  
  if (Serial) {
    Serial.println("Center calibration complete!");
    Serial.println("Now move joysticks in full circles to capture min/max values");
  }
  
  // Initialize min/max with center values
  xMin = xMax = xCenter;
  yMin = yMax = yCenter;
  zMin = zMax = zCenter;
  tMin = tMax = tCenter;
  
  // Blue rotating light indicates "move joysticks to extremes"
  // This will run for ~5 seconds while user moves joysticks
  unsigned long startTime = millis();
  unsigned long calibrationDuration = CALIBRATION_SAMPLES * CALIBRATION_DELAY * 5;
  
  while (millis() - startTime < calibrationDuration) {
    // Read current values
    int xVal = analogRead(X_AXIS);
    int yVal = 4095 - analogRead(Y_AXIS);
    int zVal = 4095 - analogRead(Z_AXIS);
    int tVal = analogRead(T_AXIS);
    
    // Update min/max values
    xMin = min(xMin, xVal);
    xMax = max(xMax, xVal);
    yMin = min(yMin, yVal);
    yMax = max(yMax, yVal);
    zMin = min(zMin, zVal);
    zMax = max(zMax, zVal);
    tMin = min(tMin, tVal);
    tMax = max(tMax, tVal);
    
    // Rotating blue color to indicate "keep moving"
    int blueShade = 100 + ((millis() - startTime) / 50) % 30;
    ums3.setPixelColor(UMS3::colorWheel(blueShade));
    delay(CALIBRATION_DELAY);
  }
  
  // Add some margin to the min/max values to ensure we can reach extremes
  int marginX = (xMax - xMin) / 20; // 5% margin
  int marginY = (yMax - yMin) / 20;
  int marginZ = (zMax - zMin) / 20;
  int marginT = (tMax - tMin) / 20;
  
  xMin = max(0, xMin - marginX);
  xMax = min(4095, xMax + marginX);
  yMin = max(0, yMin - marginY);
  yMax = min(4095, yMax + marginY);
  zMin = max(0, zMin - marginZ);
  zMax = min(4095, zMax + marginZ);
  tMin = max(0, tMin - marginT);
  tMax = min(4095, tMax + marginT);
  
  if (Serial) {
    Serial.println("Calibration complete!");
    Serial.printf("X - Center: %d, Min: %d, Max: %d\n", xCenter, xMin, xMax);
    Serial.printf("Y - Center: %d, Min: %d, Max: %d\n", yCenter, yMin, yMax);
    Serial.printf("Z - Center: %d, Min: %d, Max: %d\n", zCenter, zMin, zMax);
    Serial.printf("T - Center: %d, Min: %d, Max: %d\n", tCenter, tMin, tMax);
  }
  
  // CALIBRATION COMPLETE
  // ---------------------
  // Green success animation
  ums3.setPixelBrightness(150);
  
  // Do three green flashes to indicate completion
  for (int i = 0; i < 3; i++) {
    ums3.setPixelColor(UMS3::colorWheel(85)); // Green
    delay(200);
    ums3.setPixelColor(0, 0, 0); // Off
    delay(200);
  }
  
  calibrationComplete = true;
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
  // get the status of Transmitted packet
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
  
  // Run calibration procedure
  calibrateJoysticks();
}

int color = 0;

void loop() {
  EVERY_N_MILLIS(5) {
    control.update();
    pickup.update();
    drop.update();
    thrower.update();

    // Read raw values
    int rawX = analogRead(X_AXIS);
    int rawY = 4095 - analogRead(Y_AXIS);
    int rawZ = 4095 - analogRead(Z_AXIS);
    int rawT = analogRead(T_AXIS);
    
    // Normalize values to ensure center is at TRUE_CENTER (2048) and range is 0-4095
    X = normalizeJoystickValue(rawX, xCenter, xMin, xMax);
    Y = normalizeJoystickValue(rawY, yCenter, yMin, yMax);
    Z = normalizeJoystickValue(rawZ, zCenter, zMin, zMax);
    T = normalizeJoystickValue(rawT, tCenter, tMin, tMax);

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