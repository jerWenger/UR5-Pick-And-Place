#include "pinout.h"
#include "util.h"
#include "imu.h"

#define PRINT_DELAY 50

IMU imu(BNO08X_RESET, BNO08X_CS, BNO08X_INT);
EulerAngles imuAngles;

void setup() {
    Serial.begin();
    imu.setup();
}

void loop() {
    imu.update();

    EVERY_N_MILLIS(PRINT_DELAY) {
        imuAngles = imu.getEulerAngles();

        Serial.printf("RPY (rad): %.2f   %.2f   %.2f\n", 
                      imuAngles.roll, imuAngles.pitch, imuAngles.yaw);
    }
}

