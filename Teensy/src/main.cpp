// src/main.cpp
#include <Arduino.h>
#include "sensors/BMP390.h"
#include "sensors/BNO085.h"
#include "sensors/GPS.h"
#include "sensors/Audio.h"
#include "SensorManager.h"
#include "Packet.h"
#include "State.h"  // State 모듈 추가

// 센서 객체 생성
BMP390 bmp;
BNO085 imu;
GPS gps;
Audio audio;

// 센서 매니저 생성
SensorManager sensorManager;

// 패킷 객체 생성
Packet telemetry;

// State 객체 생성
State flightState;

void setup() {
    Serial.begin(9600));
    delay(2000);
    
    Serial.println("=== Teensy 4.1 CanSat FSW ===");
    Serial.println();
    
    // 센서 매니저에 센서 연결
    sensorManager.attachSensors(&bmp, &imu, &gps);
    
    // 모든 센서 초기화
    sensorManager.initializeAll();
    
    // BNO085 칼만 필터 활성화
    sensorManager.enableIMUFilter(
        0.01,  // 자이로 프로세스 노이즈
        0.1,   // 자이로 측정 노이즈
        0.01,  // 가속도 프로세스 노이즈
        0.5    // 가속도 측정 노이즈
    );
    
    // 오디오 부저 초기화
    Serial.println("[ 오디오 부저 초기화 ]");
    audio.begin();
    Serial.println();
    
    // State 모듈 초기화
    Serial.println("[ State 모듈 초기화 ]");
    flightState.attachSensors(&bmp, &imu, &gps);
    flightState.begin();
    Serial.println();
    
    // 패킷 시스템에 센서 및 State 연결
    telemetry.attachSensors(&bmp, &imu, &gps);
    telemetry.attachState(&flightState);
    
    // CSV 헤더 출력 (Serial)
    Serial.println("\n=== CSV 헤더 ===");
    Serial.println(Packet::getCSVHeader());
    
    Serial.println("\n=== 시스템 준비 완료 ===");
    Serial.println("명령어 대기 중...");
    Serial.println("  CX_ON  - 미션 시작");
    Serial.println("  CX_OFF - 미션 종료");
    Serial.println("  CAL    - 고도 캘리브레이션");
    Serial.println("  STATUS - State 상태 출력");
    Serial.println("  BEEP   - 부저 테스트 (연속 비프)");
    Serial.println("  STOP   - 부저 중지");
    Serial.println();
    
    delay(1000);
}

void loop() {
    static unsigned long lastTransmit = 0;
    
    // 모든 센서 업데이트 (SensorManager가 처리)
    sensorManager.updateAll();
    
    // State 업데이트 (비행 상태 자동 전환)
    flightState.update();
    
    // 오디오 부저 업데이트 (연속 비프용)
    audio.update();
    
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
            flightState.startMission();  // State에도 미션 시작 알림
            telemetry.startMission();
            telemetry.setCommandEcho("CX,ON");
        } 
        else if (command == "CX_OFF") {
            Serial.println(">>> 미션 종료!");
            flightState.stopMission();  // State에도 미션 종료 알림
            telemetry.stopMission();
            telemetry.setCommandEcho("CX,OFF");
        }
        else if (command == "CAL") {
            Serial.println(">>> 고도 캘리브레이션 시작!");
            flightState.calibrate();  // State 모듈에서 보정
            telemetry.setCommandEcho("CAL");
        }
        else if (command == "STATUS") {
            Serial.println(">>> State 상태 출력!");
            flightState.printStatus();
        }
        else if (command == "BEEP") {
            Serial.println(">>> 부저 테스트 시작! (1초마다 200ms 비프)");
            audio.startBeep(1000, 200);
        }
        else if (command == "STOP") {
            Serial.println(">>> 부저 중지!");
            audio.stopBeep();
        }
        else {
            Serial.println(">>> 알 수 없는 명령어");
            telemetry.setCommandEcho("UNKNOWN");
        }
    }
}