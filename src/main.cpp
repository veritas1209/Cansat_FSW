// src/main.cpp
#include <Arduino.h>
#include "sensors/BMP390.h"
#include "sensors/BNO085.h"
#include "sensors/GPS.h"

// 센서 객체 생성
BMP390 bmp;
BNO085 imu;
GPS gps;

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
    
    Serial.println("\n=== 센서 데이터 읽기 시작 ===\n");
    delay(1000);
}

void loop() {
    static unsigned long lastPrint = 0;
    
    // 센서 업데이트 (계속 호출)
    bmp.update();
    imu.update();
    gps.update();
    
    // 1초마다 출력
    if (millis() - lastPrint >= 1000) {
        lastPrint = millis();
        
        Serial.println("--- 센서 데이터 ---");
        
        // BMP390
        if (bmp.isInitialized()) {
            Serial.print("BMP390 - 온도: ");
            Serial.print(bmp.getTemperature());
            Serial.print(" °C, 기압: ");
            Serial.print(bmp.getPressure());
            Serial.print(" hPa, 고도: ");
            Serial.print(bmp.getAltitude());
            Serial.println(" m");
        }
        
        // BNO085
        if (imu.hasAccelData()) {
            Serial.print("BNO085 - 가속도: X=");
            Serial.print(imu.getAccelX());
            Serial.print(" Y=");
            Serial.print(imu.getAccelY());
            Serial.print(" Z=");
            Serial.print(imu.getAccelZ());
            Serial.println(" m/s²");
        }
        
        if (imu.hasQuatData()) {
            Serial.print("BNO085 - 자세: Roll=");
            Serial.print(imu.getGyroRoll());
            Serial.print("° Pitch=");
            Serial.print(imu.getGyroPitch());
            Serial.print("° Yaw=");
            Serial.print(imu.getGyroYaw());
            Serial.println("°");
        }
        
        // GPS
        Serial.print("GPS - Fix: ");
        Serial.print(gps.hasFix() ? "YES" : "NO");
        if (gps.hasFix()) {
            Serial.print(", 위도: ");
            Serial.print(gps.getLatitude(), 6);
            Serial.print(", 경도: ");
            Serial.print(gps.getLongitude(), 6);
            Serial.print(", 위성: ");
            Serial.println(gps.getSatellites());
        } else {
            Serial.println(" (신호 없음)");
        }
        
        Serial.println();
    }
}