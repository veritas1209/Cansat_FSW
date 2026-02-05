// src/sensors/GPS.cpp
#include "sensors/GPS.h"

GPS::GPS() : gps(&GPSSerial) {
    initialized = false;
}

bool GPS::begin() {
    Serial.print("GPS 초기화 중 (TX=0, RX=1)... ");
    
    gps.begin(9600);
    gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    
    Serial.println("완료!");
    initialized = true;
    return true;
}

void GPS::update() {
    if (!initialized) return;
    
    // GPS 데이터 읽기 (항상 수행)
    char c = gps.read();
    
    // 새 NMEA 문장이 수신되면 파싱
    if (gps.newNMEAreceived()) {
        gps.parse(gps.lastNMEA());
    }
}

String GPS::getTimeString() {
    if (!initialized || !gps.fix) return "NONE";
    
    char timeStr[9];
    sprintf(timeStr, "%02d:%02d:%02d", gps.hour, gps.minute, gps.seconds);
    return String(timeStr);
}