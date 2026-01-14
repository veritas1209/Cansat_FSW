// include/SensorManager.h
#ifndef SENSORMANAGER_H
#define SENSORMANAGER_H

#include <Arduino.h>
#include "sensors/BMP390.h"
#include "sensors/BNO085.h"
#include "sensors/GPS.h"
//#include "XBee_Module.h"

class SensorManager {
private:
    BMP390* bmp;
    BNO085* imu;
    GPS* gps;
    
    bool bmp_initialized;
    bool imu_initialized;
    bool gps_initialized;
    //bool xbee_initialized;

public:
    SensorManager();
    
    // 센서 객체 연결
    void attachSensors(BMP390* bmp390, BNO085* bno085, GPS* gpsModule);
    
    // 모든 센서 초기화
    void initializeAll();
    
    // 개별 센서 초기화
    bool initBMP390();
    bool initBNO085();
    bool initGPS();
    //bool initXBee();
    
    // BNO085 칼만 필터 활성화
    void enableIMUFilter(float gyro_process_noise = 0.01, 
                         float gyro_measurement_noise = 0.1,
                         float accel_process_noise = 0.01,
                         float accel_measurement_noise = 0.5);
    
    // 고도 캘리브레이션
    void calibrateAltitude(int samples = 100);
    
    // 모든 센서 업데이트 (loop에서 호출)
    void updateAll();
    
    // 초기화 상태 확인
    bool isBMPInitialized() { return bmp_initialized; }
    bool isIMUInitialized() { return imu_initialized; }
    bool isGPSInitialized() { return gps_initialized; }
    bool allInitialized() { return bmp_initialized && imu_initialized && gps_initialized; }
    
    // 초기화 결과 출력
    void printInitStatus();
};

#endif