#include <Arduino.h>
#include "pinout.h"

#define PRINT_DELAY 50

int X = 0;
int Y = 0;

void setup() {
    Serial.begin();
    pinMode(X_PIN, INPUT);
    pinMode(Y_PIN, INPUT);
}

void loop() {
    X = analogRead(X_PIN);
    Y = analogRead(Y_PIN);
    Serial.printf("X: %d, Y: %d\n", X, Y);
    
    delay(PRINT_DELAY);
}
