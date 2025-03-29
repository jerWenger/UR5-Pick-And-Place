#ifndef WIRELESS_H
#define WIRELESS_H

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);

void setupWireless(void);

struct joystickData {
    uint16_t x;
    uint16_t y;
} ;

extern joystickData joystick;
extern bool freshWirelessData;

#endif // WIRELESS_H