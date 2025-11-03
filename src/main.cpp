// src/main.cpp
#include <Arduino.h>
#include "sensors/BMP390.h"
#include "sensors/BNO085.h"
#include "sensors/GPS.h"
#include "Packet.h"

// 센서 객체 생성
BMP390 bmp;
BNO085 imu;
GPS gps;

// 패킷 객체 생성
Packet telemetry;

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("=== Teensy 4.1 CanSat FSW ===");
    Serial.println();
    
    // 센서 초기화
    bmp.begin();
    imu.begin();
    gps.begin();
    
    // 고도 캘리브레이션 (선택사항)
    // bmp.calibrateAltitude(100);
    
    // 패킷 시스템에 센서 연결
    telemetry.attachSensors(&bmp, &imu, &gps);
    
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
    
    // 1초마다 패킷 전송
    if (millis() - lastPrint >= 1000) {
        lastPrint = millis();
        
        // 패킷 전송
        telemetry.transmit();
    }
}