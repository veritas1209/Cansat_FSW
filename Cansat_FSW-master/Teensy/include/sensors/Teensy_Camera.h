#ifndef TEENSY_CAMERA_H
#define TEENSY_CAMERA_H

#include <Arduino.h>
#include <Wire.h>

// 고정 디바이스 주소
#define ESP32_GROUND_ADDR 0x10
#define ESP32_PARAGLIDER_ADDR 0x11

class TeensyCamera {
private:
    static const uint8_t MAX_RESPONSE_SIZE = 32;
    unsigned long responseTimeout;
    
public:
    TeensyCamera();
    
    // 초기화
    void begin();
    
    // 송신 후 수신 (한 번에 처리)
    String sendAndReceive(uint8_t address, uint8_t data);
    
    // 두 디바이스에 동시 전송
    void sendToBoth(uint8_t data);
    
    // 두 디바이스로부터 응답 수신
    void receiveFromBoth(String& response1, String& response2);
    
    // 타임아웃 설정 (기본 100ms)
    void setResponseTimeout(unsigned long timeout) { responseTimeout = timeout; }
};

#endif