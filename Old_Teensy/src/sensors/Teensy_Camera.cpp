#include "sensors/Teensy_Camera.h"

void Teensy_Camera::init() {
    // 시리얼 통신 속도 설정
    G_CAMERA_SERIAL.begin(9600); 
    P_CAMERA_SERIAL.begin(9600);

    command = '0';
    isG_Recording = false; 
    isP_Recording = false;
}

// 명령어 전송
void Teensy_Camera::sendCommand(char command) {   
    // 시리얼 포트로 명령어 전송
    if (command == '1') {
        Serial.println("Paglider 및 Ground 카메라 녹화 시작");
        G_CAMERA_SERIAL.write(command);
        P_CAMERA_SERIAL.write(command);
    } else if(command == '0') {
        Serial.println("Paglider 및 Ground 카메라 녹화 중지");
        G_CAMERA_SERIAL.write(command);
        P_CAMERA_SERIAL.write(command);
    }
}

// 데이터 수신
void Teensy_Camera::receiveData() {
    // 1. Ground Camera 처리
    if (G_CAMERA_SERIAL.available()) {
        char data = G_CAMERA_SERIAL.read();
        
        Serial.print("[Ground Cam] : "); // 어느 카메라인지 명시
        Serial.println(data); 

        if (data == '0' && isG_Recording) {
            Serial.println(" -> 녹화 중지 확인");
            isG_Recording = false;
        } else if (data == '1' && !isG_Recording) {
            Serial.println(" -> 녹화 시작 확인");
            isG_Recording = true;
        }
    }

    // 2. Paglider Camera 처리
    if (P_CAMERA_SERIAL.available()) {
        char data = P_CAMERA_SERIAL.read();
        
        Serial.print("[Paglider Cam] : "); // 어느 카메라인지 명시
        Serial.println(data); 

        if (data == '0' && isP_Recording) {
            Serial.println(" -> 녹화 중지 확인");
            isP_Recording = false;
        } else if (data == '1' && !isP_Recording) {
            Serial.println(" -> 녹화 시작 확인");
            isP_Recording = true;
        }
    }
}