// include/sensors/Audio.h
#ifndef AUDIO_H
#define AUDIO_H

#include <Arduino.h>

#define BUZZER_PIN 2

class Audio {
private:
    bool initialized;
    bool isBeeping;
    unsigned long lastToggle;
    int beepInterval;  // 비프음 간격 (ms)
    int beepDuration;  // 비프음 지속 시간 (ms)
    bool beepState;    // 현재 비프 상태 (ON/OFF)

public:
    Audio();
    
    // 초기화
    bool begin();
    
    // 비프음 시작 (연속 비프)
    void startBeep(int interval_ms = 1000, int duration_ms = 200);
    
    // 비프음 중지
    void stopBeep();
    
    // 단일 비프 (한 번만 울림)
    void singleBeep(int duration_ms = 200);
    
    // 비프음 업데이트 (loop에서 호출 - 연속 비프용)
    void update();
    
    // 직접 제어
    void on();   // 부저 켜기
    void off();  // 부저 끄기
    
    // 상태 확인
    bool isInitialized() { return initialized; }
    bool isActive() { return isBeeping; }
    
    // 비프 패턴 설정
    void setBeepPattern(int interval_ms, int duration_ms);
};

#endif