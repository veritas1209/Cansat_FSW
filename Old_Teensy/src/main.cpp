// src/main.cpp
#include <Arduino.h>
#include "sensors/BMP390.h"
#include "sensors/BNO085.h"
#include "sensors/GPS.h"
#include "sensors/Audio.h"
#include "sensors/SDCARD.h"
#include "SensorManager.h"
#include "sensors/XBee.h"
#include "Packet.h"
#include "State.h"
#include "Teensy_Camera.h"
#include "Mode.h"

// 센서 객체 생성
BMP390 bmp;
BNO085 imu;
GPS gps;
XBee xbee;
Audio audio;
SDCard sdcard;

Mode mode;

// 센서 매니저 생성
SensorManager sensorManager;

// 카메라 객체 생성
Teensy_Camera camera;

// 패킷 객체 생성
Packet telemetry;

// State 객체 생성
State flightState;

float currentPressurePa = 0.0; // 현재 압력 저장 변수

float groundPressurePa = 0.0; // 지면 기준 압력 저장 변수

void setup() {
    Serial.begin(9600);
    delay(2000);
    
    Serial.println("=== Teensy 4.1 CanSat FSW ===");
    Serial.println("시스템 시작 중...");
    Serial.flush();
    
    // Watchdog: 10초 안에 초기화 완료되지 않으면 강제 진행
    unsigned long setupStart = millis();
    const unsigned long setupTimeout = 10000;  // 10초
    
    // 센서 매니저에 센서 연결
    sensorManager.attachSensors(&bmp, &imu, &gps, &xbee);
    
    // 모든 센서 초기화 (개별적으로 실패해도 계속 진행)
    sensorManager.initializeAll();
    
    // 타임아웃 체크
    if (millis() - setupStart > setupTimeout) {
        Serial.println("⚠ 초기화 타임아웃! 강제로 계속 진행합니다.");
        Serial.flush();
    }
    
    // BNO085 칼만 필터 활성화 (초기화 성공했을 때만)
    if (sensorManager.isIMUInitialized()) {
        sensorManager.enableIMUFilter(0.01, 0.1, 0.01, 0.5);
    }
    
    // 나머지 초기화...
    Serial.println("[ 오디오 부저 초기화 ]");
    audio.begin();
    Serial.println();
    
    // State 모듈 초기화
    Serial.println("[ State 모듈 초기화 ]");
    flightState.attachSensors(&bmp, &imu, &gps);
    flightState.begin();
    Serial.println();

    camera.init();

    // 초기 지면 기압 설정 (부팅 시 센서값 기준)
    if (bmp.isInitialized()) {
        groundPressurePa = bmp.getPressure() * 100.0; // hPa -> Pa 변환
    }
    
    telemetry.setMode('F');
    
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
    Serial.println("  START_RECORD   - 카메라 녹화 시작");
    Serial.println("  STOP_RECORD    - 카메라 녹화 종료");
    Serial.println();
    
    delay(1000);
}

void loop() {
    static unsigned long lastTransmit = 0;
    
    // 모든 센서 업데이트 (SensorManager가 처리)
    sensorManager.updateAll();
    
    // State 업데이트 (비행 상태 자동 전환)
    flightState.update();

    if (xbee.receiveCommand())
    {
        Serial.println("XBee 명령어 수신됨:");
        for (int i = 0; i < 4; i++) {
            Serial.print("  cmdParts[");
            Serial.print(i);
            Serial.print("]: ");
            Serial.println(xbee.cmdParts[i]);
    }
    

    Serial.println("From XBee: ");
    Serial.println(xbee.cmdParts[2]);
    
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
    
    // TODO : xbee나 serial 명령어(ex: CMD,1062,CX,ON)을 문자열로 받은 뒤 CX와 ON만 분리하여 조건문 처리
    // 임시 테스트용: Serial로 명령어 입력 받기
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        
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
        else if (command == "START_RECORD") {
            Serial.println(">>> 카메라 데이터 전송 테스트!");
            camera.sendCommand('1');

        }
        else if (command == "STOP_RECORD") {
            Serial.println(">>> 카메라 데이터 전송 테스트!");
            camera.sendCommand('0');
        }
        else {
            Serial.println(">>> 알 수 없는 명령어");
            telemetry.setCommandEcho("UNKNOWN");
        }
    }

    camera.receiveData(); // 카메라 테스트할려면 이 함수 무조건 조건문 밖에 빼둬야 함
}