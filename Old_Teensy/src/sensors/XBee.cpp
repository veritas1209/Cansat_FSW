#include "sensors/XBee.h"

XBee::XBee(){
    initialized = false;
}

bool XBee::xbeeInit() {
    Serial.println("XBee 모듈 초기화 중...");
    XBEE.begin(XBEE_BAUDRATE);
    initialized = true;
    return true;
}

void XBee::xbeeTransmit(String data) {
    if(initialized) {
        XBEE.println(data);
    }
}

// 명령어가 들어왔는지 확인하고 파싱하는 함수
bool XBee::receiveCommand() {
    if (!initialized) return false;

    // 데이터가 들어올 때까지 대기하지 않고, 있는 만큼만 읽음 (Non-blocking)
    while (XBEE.available() || Serial.available()) {
        char c;
        if (XBEE.available()) c = XBEE.read();
        else c = Serial.read(); // USB 디버깅용

        if (c == '\n') { // 개행 문자(명령어 끝) 감지
            cmdBuffer.trim(); // 공백 제거
            
            // 콤마로 분리하여 cmdParts 배열에 저장
            int stringStart = 0;
            int arrayIndex = 0;
            
            for (int i = 0; i < cmdBuffer.length(); i++) {
                if (cmdBuffer.charAt(i) == ',') {
                    cmdParts[arrayIndex] = cmdBuffer.substring(stringStart, i);
                    cmdParts[arrayIndex].trim();
                    stringStart = i + 1;
                    arrayIndex++;
                    if (arrayIndex >= 4) break; // 배열 크기 초과 방지
                }
            }
            // 마지막 토큰 저장
            cmdParts[arrayIndex] = cmdBuffer.substring(stringStart);
            cmdParts[arrayIndex].trim();
            
            // 버퍼 비우기 및 초기화
            cmdBuffer = "";
            return true; // 새 명령어가 있음을 알림
        } else {
            cmdBuffer += c;
        }
    }
    return false; // 아직 완성된 명령어가 없음
}

