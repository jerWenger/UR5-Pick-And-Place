#ifndef WIRELESS_H
#define WIRELESS_H

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);

void setupWireless(void);

struct joystickData {
    uint16_t x;
    uint16_t y;
} ;

struct ourControllerData {
    uint16_t x;
    uint16_t y;
    uint16_t z;
    uint16_t t;
    uint16_t control_state;
    uint16_t pickup_state;
    uint16_t thrower_state;
};

extern joystickData joystick;
extern ourControllerData dual_joystick;
extern bool freshWirelessData;

#endif // WIRELESS_H