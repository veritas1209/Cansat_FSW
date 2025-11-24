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
    
    // BNO085 칼만 필터 활성화
    if (imu_ok) {
        Serial.println("[ BNO085 칼만 필터 활성화 ]");
        imu.enableFilter(
            0.01,  // 자이로 프로세스 노이즈
            0.1,   // 자이로 측정 노이즈
            0.01,  // 가속도 프로세스 노이즈
            0.5    // 가속도 측정 노이즈
        );
        Serial.println();
    }
    
    // 패킷 시스템에 센서 연결
    telemetry.attachSensors(&bmp, &imu, &gps);
    
    // CSV 헤더 출력 (Serial)
    Serial.println("\n=== CSV 헤더 ===");
    Serial.println(Packet::getCSVHeader());
    
    Serial.println("\n=== 시스템 준비 완료 ===");
    Serial.println("명령어 대기 중... (CX_ON으로 미션 시작)");
    Serial.println();
    
    delay(1000);
}

void loop() {
    static unsigned long lastTransmit = 0;
    
    // 센서 업데이트 (계속 호출)
    bmp.update();
    imu.update();  // BNO085 내부에서 필터링 자동 처리
    gps.update();
    
    // 1초마다 패킷 전송 (미션 시작된 경우만)
    if (millis() - lastTransmit >= 1000) {
        lastTransmit = millis();
        
        if (telemetry.isMissionStarted()) {
            // 패킷 문자열 생성
            String packetStr = telemetry.generatePacketString();
            
            // Serial 출력
            Serial.println(packetStr);
            
            // 패킷 카운터 증가
            telemetry.incrementPacketCount();
        }
    }
    
    // 임시 테스트용: Serial로 명령어 입력 받기
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        
        Serial.print("수신된 명령어: ");
        Serial.println(command);
        
        if (command == "CX_ON") {
            Serial.println(">>> 미션 시작!");
            telemetry.startMission();
            telemetry.setCommandEcho("CX,ON");
        } 
        else if (command == "CX_OFF") {
            Serial.println(">>> 미션 종료!");
            telemetry.stopMission();
            telemetry.setCommandEcho("CX,OFF");
        }
        else if (command == "CAL") {
            Serial.println(">>> 고도 캘리브레이션 시작!");
            bmp.calibrateAltitude(100);
            telemetry.setCommandEcho("CAL");
        }
        else {
            Serial.println(">>> 알 수 없는 명령어");
            telemetry.setCommandEcho("UNKNOWN");
        }
    }
}