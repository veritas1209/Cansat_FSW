#include "XBee_Module.h"


#define XBEE_BAUDRATE 9600

XBee::XBee(){
    initialized = false;
}

bool XBee::xbeeInit() {
    Serial.println("XBee 모듈 초기화 중...");

    XBEE.begin(XBEE_BAUDRATE);
    Serial.begin(9600);

    Serial.println("XBee 모듈 초기화 완료!");
    initialized = true;
    return true;

}

// TODO : Packet을 받고 전송
void XBee::xbeeTransmit(String data) {
    XBEE.println(data);
}

// TODO : 명령어 받고 문자열 파싱 후 파싱된 문자들 반환
String XBee::processCommands() {
    String command = "";
    String cmdParts[4]; // 쉼표로 분할된 명령어 부분

    int startIdx = 0;
    int tokenIndex = 0;
  
  // 부분에서 공백 제거
  for (int i = 0; i < 4; i++) {
    cmdParts[i].trim();
  }
    

    if (XBEE.available()) {
        command = XBEE.readStringUntil('\n');
    }
    else if (Serial.available()) {
        command = Serial.readStringUntil('\n');
    }

    command.trim();

    
}

