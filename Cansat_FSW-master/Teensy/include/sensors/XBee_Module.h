#ifndef XBEE_MODULE_H
#define XBEE_MODULE_H

#include <Arduino.h>

class XBeeModule {
public:
    void xbeeInit();
    void xbeeTransmit(String data);
    bool xbeeAvailable();
    String xbeeReceive();
};
    

#endif