#include "Teensy_Camera.h"

TeensyCamera::TeensyCamera() {
    responseTimeout = 100;  // 기본 타임아웃 100ms
}

void TeensyCamera::begin() {
    Wire.begin();  // I2C Master로 초기화
    delay(100);    // 안정화 대기
}

void TeensyCamera::sendToBoth(uint8_t data) {
    // Ground ESP32에 전송
    Wire.beginTransmission(ESP32_GROUND_ADDR);
    Wire.write(data);
    Wire.endTransmission();
    
    delay(5);  // 짧은 딜레이
    
    // Paraglider ESP32에 전송
    Wire.beginTransmission(ESP32_PARAGLIDER_ADDR);
    Wire.write(data);
    Wire.endTransmission();
}

void TeensyCamera::receiveFromBoth(String& response1, String& response2) {
    // Ground ESP32로부터 수신
    response1 = "";
    Wire.requestFrom(ESP32_GROUND_ADDR, MAX_RESPONSE_SIZE);
    
    unsigned long timeout1 = millis() + responseTimeout;
    while (Wire.available() == 0 && millis() < timeout1) {
        delay(1);
    }
    
    while (Wire.available()) {
        char c = Wire.read();
        if (c == '\0') break;
        response1 += c;
    }
    
    delay(10);  // 다음 요청 전 딜레이
    
    // Paraglider ESP32로부터 수신
    response2 = "";
    Wire.requestFrom(ESP32_PARAGLIDER_ADDR, MAX_RESPONSE_SIZE);
    
    unsigned long timeout2 = millis() + responseTimeout;
    while (Wire.available() == 0 && millis() < timeout2) {
        delay(1);
    }
    
    while (Wire.available()) {
        char c = Wire.read();
        if (c == '\0') break;
        response2 += c;
    }
}