// include/sensors/GPS.h
#ifndef GPS_H
#define GPS_H

#include <Arduino.h>
#include <Adafruit_GPS.h>

#define GPSSerial Serial1

class GPS {
private:
    Adafruit_GPS gps;
    bool initialized;

public:
    GPS();
    
    // 초기화 (Serial1 사용 - 핀 0/1)
    bool begin();
    
    // GPS 데이터 읽기 (loop에서 계속 호출 필요)
    void update();
    
    // 데이터 접근
    bool hasFix() { return gps.fix; }
    float getLatitude() { return gps.latitudeDegrees; }
    float getLongitude() { return gps.longitudeDegrees; }
    float getAltitude() { return gps.altitude; }
    int getSatellites() { return gps.satellites; }
    
    // GPS 시간 (HH:MM:SS 문자열)
    String getTimeString();
    
    // 상태 확인
    bool isInitialized() { return initialized; }
};

#endif