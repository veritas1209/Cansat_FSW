#include "XBee.h"

#define XBEE Serial2
#define XBEE_BAUDRATE 9600

void XBee::xbeeInit() {
    XBEE.begin(XBEE_BAUDRATE);
    delay(100); // Give some time for the module to initialize
}

void XBee::xbeeTransmit(String data) {
    XBEE.print(data);
}

bool XBee::xbeeAvailable() {
    return XBEE.available() > 0;
}

String XBee::xbeeReceive() {
    return XBEE.readStringUntil('\n');
}