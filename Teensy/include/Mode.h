#ifndef MODE_H
#define MODE_H

#include <Arduino.h>

class Mode {
private:
    char currentMode;      // 'F' or 'S'
    bool isSimEnabled;     // SIM,ENABLE 수신 여부
    bool isSimActivated;   // SIM,ACTIVATE 수신 여부

    void updateMode();     // 플래그 상태를 보고 모드 결정

public:
    Mode();
    void init();

    // 명령어 처리 함수
    void setSimEnable(bool enable);
    void setSimActivate(bool activate);
    
    // 모드 강제 설정 (예: DISABLE 명령 시)
    void resetToFlightMode();

    // 현재 상태 반환
    char getMode();
    bool isSimulation();
};

#endif