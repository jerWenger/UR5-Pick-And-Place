#ifndef DRIVE_H
#define DRIVE_H

#include "wireless.h"

#define Kp 0.25
#define Ki 0.01
#define Kd 0
#define tau 0.1

#define MAX_FORWARD 6
#define MAX_TURN 3

extern joystickData joystick;

#endif // DRIVE_H