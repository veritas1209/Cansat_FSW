// src/SensorManager.cpp
#include "SensorManager.h"

SensorManager::SensorManager() {
    bmp = nullptr;
    imu = nullptr;
    gps = nullptr;
    
    bmp_initialized = false;
    imu_initialized = false;
    gps_initialized = false;
    xbee_initialized = false;
}

void SensorManager::attachSensors(BMP390* bmp390, BNO085* bno085, GPS* gpsModule, XBee* xbee) {
    bmp = bmp390;
    imu = bno085;
    gps = gpsModule;
    xbee = xbee;

    Serial.println("SensorManager - 센서 연결 완료");
    Serial.flush();
}

void SensorManager::initializeAll() {
    Serial.println("=== 센서 초기화 시작 ===");
    Serial.println();
    Serial.flush();
    
    // BMP390 먼저 초기화 (가장 안정적)
    Serial.println("[ 1/3 ] BMP390 초기화...");
    Serial.flush();
    bmp_initialized = initBMP390();
    delay(100);
    
    // GPS 초기화 (두 번째로 안정적)
    Serial.println("[ 2/3 ] GPS 초기화...");
    Serial.flush();
    gps_initialized = initGPS();
    delay(100);
    
    // BNO085 마지막 초기화 (가장 문제가 많음)
    Serial.println("[ 3/3 ] BNO085 초기화...");
    Serial.flush();
    imu_initialized = initBNO085();
    delay(100);
    
    // XBee 초기화
    Serial.println("[ 4/4 ] XBee 초기화...");
    Serial.flush();
    xbee_initialized = initXBee();
    delay(100);
    
    Serial.println();
    printInitStatus();
}

bool SensorManager::initBMP390() {
    if (!bmp) {
        Serial.println("  BMP390 - 센서 객체가 연결되지 않음");
        Serial.flush();
        return false;
    }
    
    bool success = bmp->begin();
    
    if (success) {
        Serial.println("  BMP390 - 초기화 성공!");
    } else {
        Serial.println("  BMP390 - 초기화 실패 (계속 진행)");
    }
    Serial.flush();
    
    return success;
}

bool SensorManager::initBNO085() {
    if (!imu) {
        Serial.println("  BNO085 - 센서 객체가 연결되지 않음");
        Serial.flush();
        return false;
    }
    
    bool success = imu->begin();
    
    if (success) {
        Serial.println("  BNO085 - 초기화 성공!");
    } else {
        Serial.println("  BNO085 - 초기화 실패 (계속 진행)");
    }
    Serial.flush();
    
    return success;
}

bool SensorManager::initGPS() {
    if (!gps) {
        Serial.println("  GPS - 센서 객체가 연결되지 않음");
        Serial.flush();
        return false;
    }
    
    bool success = gps->begin();
    
    if (success) {
        Serial.println("  GPS - 초기화 성공!");
    } else {
        Serial.println("  GPS - 초기화 실패 (계속 진행)");
    }
    Serial.flush();
    
    return success;
}

void SensorManager::enableIMUFilter(float gyro_process_noise, 
                                     float gyro_measurement_noise,
                                     float accel_process_noise,
                                     float accel_measurement_noise) {
    if (!imu || !imu_initialized) {
        Serial.println("SensorManager - IMU가 초기화되지 않아 필터를 활성화할 수 없습니다.");
        Serial.flush();
        return;
    }
    
    Serial.println("[ BNO085 칼만 필터 활성화 ]");
    Serial.flush();
    
    imu->enableFilter(gyro_process_noise, 
                      gyro_measurement_noise,
                      accel_process_noise,
                      accel_measurement_noise);
    Serial.println();
    Serial.flush();
}

void SensorManager::calibrateAltitude(int samples) {
    if (!bmp || !bmp_initialized) {
        Serial.println("SensorManager - BMP390이 초기화되지 않아 캘리브레이션할 수 없습니다.");
        Serial.flush();
        return;
    }
    
    Serial.println("[ BMP390 고도 캘리브레이션 ]");
    Serial.flush();
    bmp->calibrateAltitude(samples);
    Serial.println();
    Serial.flush();
}

// src/sensors/SensorManager.cpp
void SensorManager::updateAll() {
    // BMP390 업데이트
    if (bmp && bmp_initialized) {
        bmp->update();
    }
    
    // BNO085 업데이트 - 여러 이벤트를 수집하기 위해 여러 번 호출
    if (imu && imu_initialized) {
        // BNO085는 각 센서 타입(가속도, 자이로, 회전벡터)마다
        // 별도의 이벤트를 발생시키므로 여러 번 폴링 필요
        for (int i = 0; i < 20; i++) {  // 20번 폴링
            imu->update();
            delayMicroseconds(500);  // 0.5ms 대기
        }
    }
    
    // GPS 업데이트
    if (gps && gps_initialized) {
        gps->update();
    }
}

void SensorManager::printInitStatus() {
    Serial.println("==============================");
    Serial.println("[ 센서 초기화 결과 ]");
    Serial.print("  BMP390: ");
    Serial.println(bmp_initialized ? "OK ✓" : "FAIL ✗");
    
    Serial.print("  BNO085: ");
    Serial.println(imu_initialized ? "OK ✓" : "FAIL ✗");
    
    Serial.print("  GPS: ");
    Serial.println(gps_initialized ? "OK ✓" : "FAIL ✗");

    Serial.print("  XBee: ");
    Serial.println(xbee_initialized ? "OK ✓" : "FAIL ✗");
    
    Serial.println("==============================");
    Serial.println();
    
    if (allInitialized()) {
        Serial.println("✓ 모든 센서 초기화 성공!");
    } else {
        Serial.println("⚠ 일부 센서 초기화 실패 (시스템은 계속 작동)");
        
        if (!bmp_initialized) {
            Serial.println("  - BMP390 없이 계속 진행 (고도/온도 데이터 없음)");
        }
        if (!imu_initialized) {
            Serial.println("  - BNO085 없이 계속 진행 (자이로/가속도 데이터 없음)");
        }
        if (!gps_initialized) {
            Serial.println("  - GPS 없이 계속 진행 (위치 데이터 없음)");
        }
        if (!xbee_initialized) {
            Serial.println("  - XBee 없이 계속 진행 (무선 통신 불가)");
        }
    }
    Serial.println();
    Serial.flush();
}