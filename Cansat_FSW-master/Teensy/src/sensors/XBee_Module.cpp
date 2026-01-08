#include "XBee_Module.h"

#define XBEE Serial2
#define XBEE_BAUDRATE 9600

void XBeeModule::xbeeInit() {
    XBEE.begin(XBEE_BAUDRATE);
    delay(100); // Give some time for the module to initialize
}

void XBeeModule::xbeeTransmit(String data) {
    XBEE.print(data);
}

bool XBeeModule::xbeeAvailable() {
    return XBEE.available() > 0;
}

String XBeeModule::xbeeReceive() {
    return XBEE.readStringUntil('\n');
}