#ifndef XBee_H
#define XBee_H

#include <Arduino.h>

#define XBEE Serial2

class XBee {
private:
    bool initialized;

public:
    bool xbeeInit();
    void xbeeTransmit(String data);
    String processCommands();
};
    

#endif 