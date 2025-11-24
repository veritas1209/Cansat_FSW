#ifndef XBee_H
#define XBee_H

#include <Arduino.h>

class XBee {
public:
    void xbeeInit();
    void xbeeTransmit(String data);
    bool xbeeAvailable();
    String xbeeReceive();
};
    

#endif 