// src/SensorManager.cpp
#include "SensorManager.h"

SensorManager::SensorManager() {
    bmp = nullptr;
    imu = nullptr;
    gps = nullptr;
    
    bmp_initialized = false;
    imu_initialized = false;
    gps_initialized = false;
}

void SensorManager::attachSensors(BMP390* bmp390, BNO085* bno085, GPS* gpsModule) {
    bmp = bmp390;
    imu = bno085;
    gps = gpsModule;
    
    Serial.println("SensorManager - 센서 연결 완료");
}

void SensorManager::initializeAll() {
    Serial.println("=== 센서 초기화 시작 ===");
    Serial.println();
    
    // 각 센서 초기화
    bmp_initialized = initBMP390();
    imu_initialized = initBNO085();
    gps_initialized = initGPS();
    
    Serial.println();
    
    // 초기화 결과 출력
    printInitStatus();
}

bool SensorManager::initBMP390() {
    if (!bmp) {
        Serial.println("BMP390 - 센서 객체가 연결되지 않음");
        return false;
    }
    
    return bmp->begin();
}

bool SensorManager::initBNO085() {
    if (!imu) {
        Serial.println("BNO085 - 센서 객체가 연결되지 않음");
        return false;
    }
    
    return imu->begin();
}

bool SensorManager::initGPS() {
    if (!gps) {
        Serial.println("GPS - 센서 객체가 연결되지 않음");
        return false;
    }
    
    return gps->begin();
}

void SensorManager::enableIMUFilter(float gyro_process_noise, 
                                     float gyro_measurement_noise,
                                     float accel_process_noise,
                                     float accel_measurement_noise) {
    if (!imu || !imu_initialized) {
        Serial.println("SensorManager - IMU가 초기화되지 않아 필터를 활성화할 수 없습니다.");
        return;
    }
    
    Serial.println("[ BNO085 칼만 필터 활성화 ]");
    imu->enableFilter(gyro_process_noise, 
                      gyro_measurement_noise,
                      accel_process_noise,
                      accel_measurement_noise);
    Serial.println();
}

void SensorManager::calibrateAltitude(int samples) {
    if (!bmp || !bmp_initialized) {
        Serial.println("SensorManager - BMP390이 초기화되지 않아 캘리브레이션할 수 없습니다.");
        return;
    }
    
    Serial.println("[ BMP390 고도 캘리브레이션 ]");
    bmp->calibrateAltitude(samples);
    Serial.println();
}

void SensorManager::updateAll() {
    if (bmp && bmp_initialized) {
        bmp->update();
    }
    
    if (imu && imu_initialized) {
        imu->update();
    }
    
    if (gps && gps_initialized) {
        gps->update();
    }
}

void SensorManager::printInitStatus() {
    Serial.println("[ 센서 초기화 결과 ]");
    Serial.print("  BMP390: ");
    Serial.println(bmp_initialized ? "OK" : "FAIL");
    
    Serial.print("  BNO085: ");
    Serial.println(imu_initialized ? "OK" : "FAIL");
    
    Serial.print("  GPS: ");
    Serial.println(gps_initialized ? "OK" : "FAIL");
    
    Serial.println();
    
    if (allInitialized()) {
        Serial.println("✓ 모든 센서 초기화 성공!");
    } else {
        Serial.println("⚠ 일부 센서 초기화 실패");
    }
    Serial.println();
}