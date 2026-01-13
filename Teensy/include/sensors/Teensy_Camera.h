#ifndef TEENSY_CAMERA_H
#define TEENSY_CAMERA_H

#include <Arduino.h>

#define CAMERA_SERIAL Serial3 // Teensy 4.1의 Serial3(TX3, RX3) 사용

class Teensy_Camera
{
public:
    void init(); // 시리얼 통신 초기화
    void sendCommand(); // 명령어 전송
    void receiveData(); // 데이터 수신
};



#endif