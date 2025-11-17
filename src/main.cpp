// src/main.cpp
#include <Arduino.h>
#include "sensors/BMP390.h"
#include "sensors/BNO085.h"
#include "sensors/GPS.h"
#include "Filter.h"
#include "Packet.h"

// 센서 객체 생성
BMP390 bmp;
BNO085 imu;
GPS gps;

// IMU 칼만 필터 객체 생성
IMUFilter imuFilter;

// 패킷 객체 생성
Packet telemetry;

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("=== Teensy 4.1 CanSat FSW ===");
    Serial.println();
    
    // 센서 초기화
    Serial.println("[ 센서 초기화 시작 ]");
    bool bmp_ok = bmp.begin();
    bool imu_ok = imu.begin();
    bool gps_ok = gps.begin();
    Serial.println();
    
    // 센서 초기화 결과 출력
    Serial.println("[ 센서 초기화 결과 ]");
    Serial.print("  BMP390: "); Serial.println(bmp_ok ? "OK" : "FAIL");
    Serial.print("  BNO085: "); Serial.println(imu_ok ? "OK" : "FAIL");
    Serial.print("  GPS: "); Serial.println(gps_ok ? "OK" : "FAIL");
    Serial.println();
    
    // 고도 캘리브레이션 (선택사항)
    if (bmp_ok) {
        Serial.println("[ 기압계 캘리브레이션 ]");
        bmp.calibrateAltitude(100);
        Serial.println();
    }
    
    // IMU 칼만 필터 초기화
    Serial.println("[ IMU 칼만 필터 초기화 ]");
    imuFilter.begin(
        0.01,  // 자이로 프로세스 노이즈
        0.1,   // 자이로 측정 노이즈
        0.01,  // 가속도 프로세스 노이즈
        0.5    // 가속도 측정 노이즈
    );
    Serial.println();
    
    // 패킷 시스템에 센서 연결
    telemetry.attachSensors(&bmp, &imu, &gps);
    
    // 패킷 시스템에 IMU 필터 연결
    telemetry.attachIMUFilter(&imuFilter);
    
    // 미션 시작
    telemetry.beginMission();
    
    Serial.println("\n=== 패킷 전송 시작 ===\n");
    
    // CSV 헤더 출력
    Serial.println("TEAM_ID,MISSION_TIME,PACKET_COUNT,MODE,STATE,ALTITUDE,TEMPERATURE,ATM_PRESSURE,VOLTAGE,CURRENT,GYRO_R,GYRO_P,GYRO_Y,ACCEL_R,ACCEL_P,ACCEL_Y,GPS_TIME,GPS_ALTITUDE,GPS_LATITUDE,GPS_LONGITUDE,GPS_SATS,CMD_ECHO");
    
    delay(1000);
}

void loop() {
    static unsigned long lastPrint = 0;
    
    // 센서 업데이트 (계속 호출)
    bmp.update();
    imu.update();
    gps.update();
    
    // IMU 칼만 필터 업데이트 (센서 읽기 직후)
    if (imu.isInitialized() && imuFilter.isInitialized()) {
        // Raw 데이터 읽기
        float raw_gyro_r = imu.getGyroRoll();
        float raw_gyro_p = imu.getGyroPitch();
        float raw_gyro_y = imu.getGyroYaw();
        
        float raw_accel_r = imu.getAccelRoll();
        float raw_accel_p = imu.getAccelPitch();
        float raw_accel_y = imu.getAccelYaw();
        
        // 칼만 필터 업데이트
        imuFilter.updateGyro(raw_gyro_r, raw_gyro_p, raw_gyro_y);
        imuFilter.updateAccel(raw_accel_r, raw_accel_p, raw_accel_y);
    }
    
    // 1초마다 패킷 전송
    if (millis() - lastPrint >= 1000) {
        lastPrint = millis();
        
        // 디버그 출력 (선택사항 - 주석 처리 가능)
        /*
        Serial.println("\n--- 센서 데이터 (Raw vs Filtered) ---");
        
        if (imu.isInitialized()) {
            Serial.println("자이로 (Raw -> Filtered):");
            Serial.print("  Roll:  ");
            Serial.print(imu.getGyroRoll(), 2);
            Serial.print(" -> ");
            Serial.println(imuFilter.getGyroRoll(), 2);
            
            Serial.print("  Pitch: ");
            Serial.print(imu.getGyroPitch(), 2);
            Serial.print(" -> ");
            Serial.println(imuFilter.getGyroPitch(), 2);
            
            Serial.print("  Yaw:   ");
            Serial.print(imu.getGyroYaw(), 2);
            Serial.print(" -> ");
            Serial.println(imuFilter.getGyroYaw(), 2);
            
            Serial.println("가속도 (Raw -> Filtered):");
            Serial.print("  Roll:  ");
            Serial.print(imu.getAccelRoll(), 2);
            Serial.print(" -> ");
            Serial.println(imuFilter.getAccelRoll(), 2);
            
            Serial.print("  Pitch: ");
            Serial.print(imu.getAccelPitch(), 2);
            Serial.print(" -> ");
            Serial.println(imuFilter.getAccelPitch(), 2);
            
            Serial.print("  Yaw:   ");
            Serial.print(imu.getAccelYaw(), 2);
            Serial.print(" -> ");
            Serial.println(imuFilter.getAccelYaw(), 2);
        }
        
        Serial.println("-------------------\n");
        */
        
        // 패킷 전송 (필터링된 데이터 사용)
        telemetry.transmit();
    }
}