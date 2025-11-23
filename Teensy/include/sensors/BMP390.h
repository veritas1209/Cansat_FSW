// include/sensors/BMP390.h
#ifndef BMP390_H
#define BMP390_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP3XX.h>

class BMP390 {
private:
    Adafruit_BMP3XX bmp;
    float baseAltitude;
    bool initialized;

public:
    BMP390();
    
    // 초기화 (Wire 사용 - 핀 18/19)
    bool begin();
    
    // 센서 데이터 읽기
    bool update();
    
    // 데이터 접근
    float getTemperature();      // 온도 (°C)
    float getPressure();         // 기압 (hPa)
    float getAltitude();         // 고도 (m)
    float getRelativeAltitude(); // 상대 고도 (m)
    
    // 고도 캘리브레이션
    void calibrateAltitude(int samples = 100);
    
    // 상태 확인
    bool isInitialized() { return initialized; }
};

#endif