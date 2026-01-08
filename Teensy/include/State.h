// include/State.h
#ifndef STATE_H
#define STATE_H

#include <Arduino.h>
#include "sensors/BMP390.h"
#include "sensors/BNO085.h"
#include "sensors/GPS.h"

// 비행 상태 열거형
enum FlightState {
    LAUNCH_PAD,        // 발사대 대기
    ASCENT,            // 상승 중
    APOGEE,            // 최고점 도달
    DESCENT,           // 하강 중 (Container + Payload)
    PAYLOAD_RELEASE,   // Payload 분리 (80% 고도)
    PROBE_RELEASE,     // 계기(계란) 방출 준비
    LANDED             // 착륙 완료
};

// 상태 전환 임계값 설정
struct StateThresholds {
    float launchAltitude;          // 발사 감지 고도 (m)
    float apogeeDetectionWindow;   // Apogee 감지 시간 창 (ms)
    float payloadReleasePercent;   // Payload 분리 고도 비율 (0.80 = 80%)
    float probeReleaseAltitude;    // 계란 방출 고도 (m, 지면으로부터)
    float probeReleaseMargin;      // 계란 방출 허용 오차 (m)
    float landedAltitude;          // 착륙 판정 고도 (m)
    float landedTimeWindow;        // 착륙 판정 시간 (ms)
};

class State {
private:
    // 센서 참조
    BMP390* bmp;
    BNO085* imu;
    GPS* gps;
    
    // 현재 상태
    FlightState currentState;
    FlightState previousState;
    
    // 보정 및 초기화 플래그
    bool isCalibrated;
    bool isMissionStarted;
    
    // 고도 추적 변수
    float maxAltitude;           // 기록된 최대 고도
    float prevAltitude;          // 이전 고도
    float groundAltitude;        // 지면 기준 고도 (보정값)
    float currentAltitude;       // 현재 고도
    
    // 상태 전환 타이밍
    unsigned long stateChangeTime;      // 마지막 상태 변경 시간
    unsigned long apogeeDetectStart;    // Apogee 감지 시작 시간
    unsigned long landedDetectStart;    // 착륙 감지 시작 시간
    
    // 임계값 설정
    StateThresholds thresholds;
    
    // 상승/하강 감지용
    int descentCount;            // 연속 하강 카운터
    int stableCount;             // 고도 안정 카운터
    
    // Payload 분리 플래그
    bool payloadReleased;
    bool probeReleased;
    
    // 상태 전환 로직
    void checkLaunchPadToAscent();
    void checkAscentToApogee();
    void checkApogeeToDescentOrPayloadRelease();
    void checkDescentToPayloadRelease();
    void checkPayloadReleaseToProbeRelease();
    void checkProbeReleaseToLanded();
    void checkLanded();
    
    // 헬퍼 함수
    bool isAscending();
    bool isDescending();
    bool isStableAltitude();
    float getRelativeAltitude();  // 지면 기준 상대 고도
    
    // 상태 변경 처리
    void changeState(FlightState newState);

public:
    State();
    
    // 센서 연결
    void attachSensors(BMP390* bmp390, BNO085* bno085, GPS* gpsModule);
    
    // 초기화 및 보정
    void begin();
    void calibrate();  // 고도 보정 (지면 기준점 설정)
    bool isSystemCalibrated() { return isCalibrated; }
    
    // 미션 제어
    void startMission();
    void stopMission();
    bool isMissionActive() { return isMissionStarted; }
    
    // 상태 업데이트 (매 루프마다 호출)
    void update();
    
    // 현재 상태 조회
    FlightState getCurrentState() { return currentState; }
    String getStateString();
    String getStateString(FlightState state);
    
    // 고도 정보 조회
    float getMaxAltitude() { return maxAltitude; }
    float getCurrentAltitude() { return currentAltitude; }
    float getGroundAltitude() { return groundAltitude; }
    
    // 임계값 설정
    void setThresholds(const StateThresholds& newThresholds);
    StateThresholds getThresholds() { return thresholds; }
    
    // 이벤트 플래그 확인
    bool isPayloadReleased() { return payloadReleased; }
    bool isProbeReleased() { return probeReleased; }
    
    // 수동 상태 변경 (테스트/시뮬레이션용)
    void forceState(FlightState newState);
    
    // 리셋
    void reset();
    
    // 디버그 출력
    void printStatus();
};

#endif