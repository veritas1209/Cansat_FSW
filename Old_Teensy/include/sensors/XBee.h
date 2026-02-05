#ifndef XBee_H
#define XBee_H

#include <Arduino.h>

#define XBEE Serial2
#define XBEE_BAUDRATE 9600

class XBee {
private:
    bool initialized;
    String cmdBuffer; // 수신 데이터 버퍼

public:
    // 파싱된 명령어를 담을 배열 (최대 5개 토큰: CMD, TEAM_ID, ACTION, PARAM1, PARAM2)
    String cmdParts[5]; 

    bool xbeeInit();
    void xbeeTransmit(String data);
    
    // 명령어가 수신되면 true를 반환하고, cmdParts에 토큰을 채우는 함수
    bool receiveCommand(); 
};

#endif