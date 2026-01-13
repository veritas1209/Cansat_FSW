#include "sensors/Teensy_Camera.h"

void Teensy_Camera::init() {
    CAMERA_SERIAL.begin(9600); // 시리얼 통신 속도 설정
}

// 명령어 전송
void Teensy_Camera::sendCommand() {
    char command = '0';

    // 시리얼 포트로 명령어 전송
    CAMERA_SERIAL.write(command);
}

// 데이터 수신
void Teensy_Camera::receiveData() {
    if (CAMERA_SERIAL.available()){ // 데이터가 수신되었는지 확인
        char data = CAMERA_SERIAL.read(); // 수신된 데이터 읽기
        
        // 시리얼 모니터에 출력
        Serial.print("From ESP32S3 : "); 
        Serial.println(data); 
    }
}