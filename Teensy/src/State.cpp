// src/State.cpp
#include "State.h"

State::State() {
    bmp = nullptr;
    imu = nullptr;
    gps = nullptr;
    
    currentState = LAUNCH_PAD;
    previousState = LAUNCH_PAD;
    
    isCalibrated = false;
    isMissionStarted = false;
    
    maxAltitude = 0.0;
    prevAltitude = 0.0;
    groundAltitude = 0.0;
    currentAltitude = 0.0;
    
    stateChangeTime = 0;
    apogeeDetectStart = 0;
    landedDetectStart = 0;
    
    descentCount = 0;
    stableCount = 0;
    
    payloadReleased = false;
    probeReleased = false;
    
    // 기본 임계값 설정
    thresholds.launchAltitude = 10.0;           // 10m
    thresholds.apogeeDetectionWindow = 2000;    // 2초
    thresholds.payloadReleasePercent = 0.80;    // 80%
    thresholds.probeReleaseAltitude = 2.0;      // 2m
    thresholds.probeReleaseMargin = 0.5;        // ±0.5m
    thresholds.landedAltitude = 5.0;            // 5m
    thresholds.landedTimeWindow = 3000;         // 3초
}

void State::attachSensors(BMP390* bmp390, BNO085* bno085, GPS* gpsModule) {
    bmp = bmp390;
    imu = bno085;
    gps = gpsModule;
    
    Serial.println("State - 센서 연결 완료");
}

void State::begin() {
    Serial.println("State - 초기화 시작");
    
    if (!bmp || !bmp->isInitialized()) {
        Serial.println("State - 경고: BMP390이 초기화되지 않음");
    }
    
    currentState = LAUNCH_PAD;
    previousState = LAUNCH_PAD;
    stateChangeTime = millis();
    
    Serial.println("State - 초기화 완료");
    Serial.println("State - 현재 상태: LAUNCH_PAD");
}

void State::calibrate() {
    if (!bmp || !bmp->isInitialized()) {
        Serial.println("State - 보정 실패: BMP390이 초기화되지 않음");
        return;
    }
    
    Serial.println("State - 고도 보정 시작...");
    
    // BMP390 고도 보정
    bmp->calibrateAltitude(100);
    
    // 지면 고도 저장
    groundAltitude = bmp->getAltitude();
    currentAltitude = 0.0;  // 상대 고도는 0부터 시작
    prevAltitude = 0.0;
    maxAltitude = 0.0;
    
    isCalibrated = true;
    
    Serial.print("State - 보정 완료! 지면 고도: ");
    Serial.print(groundAltitude, 2);
    Serial.println(" m");
}

void State::startMission() {
    if (!isCalibrated) {
        Serial.println("State - 경고: 시스템이 보정되지 않았습니다!");
    }
    
    isMissionStarted = true;
    stateChangeTime = millis();
    
    Serial.println("State - 미션 시작!");
}

void State::stopMission() {
    isMissionStarted = false;
    Serial.println("State - 미션 종료!");
}

void State::update() {
    if (!isMissionStarted || !isCalibrated) {
        return;
    }
    
    if (!bmp || !bmp->isInitialized()) {
        return;
    }
    
    // 현재 고도 업데이트 (상대 고도)
    currentAltitude = getRelativeAltitude();
    
    // 최대 고도 추적
    if (currentAltitude > maxAltitude) {
        maxAltitude = currentAltitude;
    }
    
    // 상태별 전환 로직 실행
    switch (currentState) {
        case LAUNCH_PAD:
            checkLaunchPadToAscent();
            break;
            
        case ASCENT:
            checkAscentToApogee();
            break;
            
        case APOGEE:
            checkApogeeToDescentOrPayloadRelease();
            break;
            
        case DESCENT:
            checkDescentToPayloadRelease();
            break;
            
        case PAYLOAD_RELEASE:
            checkPayloadReleaseToProbeRelease();
            break;
            
        case PROBE_RELEASE:
            checkProbeReleaseToLanded();
            break;
            
        case LANDED:
            checkLanded();
            break;
    }
    
    // 이전 고도 업데이트
    prevAltitude = currentAltitude;
}

void State::checkLaunchPadToAscent() {
    // 발사 감지: 일정 고도 이상 + 상승 중
    if (currentAltitude >= thresholds.launchAltitude && isAscending()) {
        changeState(ASCENT);
    }
}

void State::checkAscentToApogee() {
    // 최대 고도 갱신
    if (currentAltitude > maxAltitude) {
        maxAltitude = currentAltitude;
        descentCount = 0;  // 상승 중이면 카운터 리셋
    }
    
    // 하강 시작 감지 (연속 3회 하강)
    if (isDescending()) {
        descentCount++;
        
        if (descentCount >= 3) {
            changeState(APOGEE);
            apogeeDetectStart = millis();
        }
    } else {
        descentCount = 0;
    }
}

void State::checkApogeeToDescentOrPayloadRelease() {
    // Apogee는 짧은 전환 상태
    // 일정 시간 경과 후 DESCENT로 전환
    if (millis() - apogeeDetectStart >= thresholds.apogeeDetectionWindow) {
        changeState(DESCENT);
    }
}

void State::checkDescentToPayloadRelease() {
    // Payload 분리: 최대 고도의 80% 도달
    float releaseAltitude = maxAltitude * thresholds.payloadReleasePercent;
    
    if (currentAltitude <= releaseAltitude && !payloadReleased) {
        changeState(PAYLOAD_RELEASE);
        payloadReleased = true;
        
        Serial.println("State - Payload 분리 신호 발생!");
        Serial.print("  현재 고도: ");
        Serial.print(currentAltitude, 2);
        Serial.print(" m, 목표 고도: ");
        Serial.print(releaseAltitude, 2);
        Serial.println(" m");
    }
}

void State::checkPayloadReleaseToProbeRelease() {
    // 계란 방출: 지면으로부터 2m ± 0.5m
    float targetAltitude = thresholds.probeReleaseAltitude;
    float margin = thresholds.probeReleaseMargin;
    
    if (currentAltitude <= (targetAltitude + margin) && 
        currentAltitude >= (targetAltitude - margin) && 
        !probeReleased) {
        
        changeState(PROBE_RELEASE);
        probeReleased = true;
        
        Serial.println("State - 계란 방출 신호 발생!");
        Serial.print("  현재 고도: ");
        Serial.print(currentAltitude, 2);
        Serial.print(" m, 목표 고도: ");
        Serial.print(targetAltitude, 2);
        Serial.println(" m");
    }
}

void State::checkProbeReleaseToLanded() {
    // 착륙 감지: 낮은 고도 + 안정된 상태
    if (currentAltitude < thresholds.landedAltitude) {
        if (isStableAltitude()) {
            stableCount++;
            
            // 안정 상태가 일정 시간 유지되면 착륙 판정
            if (landedDetectStart == 0) {
                landedDetectStart = millis();
            }
            
            if (millis() - landedDetectStart >= thresholds.landedTimeWindow) {
                changeState(LANDED);
            }
        } else {
            stableCount = 0;
            landedDetectStart = 0;
        }
    }
}

void State::checkLanded() {
    // 착륙 후 상태 - 특별한 전환 없음
    // 추가 로직 (비콘 활성화 등)은 여기에 구현 가능
}

bool State::isAscending() {
    // 현재 고도가 이전 고도보다 0.5m 이상 높으면 상승
    return (currentAltitude - prevAltitude) > 0.5;
}

bool State::isDescending() {
    // 현재 고도가 이전 고도보다 0.5m 이상 낮으면 하강
    return (prevAltitude - currentAltitude) > 0.5;
}

bool State::isStableAltitude() {
    // 고도 변화가 0.3m 이내면 안정
    return abs(currentAltitude - prevAltitude) < 0.3;
}

float State::getRelativeAltitude() {
    if (!bmp || !bmp->isInitialized()) {
        return 0.0;
    }
    
    // 절대 고도 - 지면 고도 = 상대 고도
    float absoluteAlt = bmp->getAltitude();
    return absoluteAlt - groundAltitude;
}

void State::changeState(FlightState newState) {
    if (newState == currentState) {
        return;  // 같은 상태로는 전환하지 않음
    }
    
    previousState = currentState;
    currentState = newState;
    stateChangeTime = millis();
    
    // 카운터 리셋
    descentCount = 0;
    stableCount = 0;
    apogeeDetectStart = 0;
    landedDetectStart = 0;
    
    Serial.println("========================================");
    Serial.print("State - 상태 전환: ");
    Serial.print(getStateString(previousState));
    Serial.print(" → ");
    Serial.println(getStateString(currentState));
    Serial.print("  시간: ");
    Serial.print(millis() / 1000.0, 1);
    Serial.println(" s");
    Serial.print("  현재 고도: ");
    Serial.print(currentAltitude, 2);
    Serial.println(" m");
    Serial.print("  최대 고도: ");
    Serial.print(maxAltitude, 2);
    Serial.println(" m");
    Serial.println("========================================");
}

String State::getStateString() {
    return getStateString(currentState);
}

String State::getStateString(FlightState state) {
    switch (state) {
        case LAUNCH_PAD:
            return "LAUNCH_PAD";
        case ASCENT:
            return "ASCENT";
        case APOGEE:
            return "APOGEE";
        case DESCENT:
            return "DESCENT";
        case PAYLOAD_RELEASE:
            return "PAYLOAD_RELEASE";
        case PROBE_RELEASE:
            return "PROBE_RELEASE";
        case LANDED:
            return "LANDED";
        default:
            return "UNKNOWN";
    }
}

void State::setThresholds(const StateThresholds& newThresholds) {
    thresholds = newThresholds;
    
    Serial.println("State - 임계값 업데이트:");
    Serial.print("  발사 고도: ");
    Serial.print(thresholds.launchAltitude, 1);
    Serial.println(" m");
    Serial.print("  Payload 분리: ");
    Serial.print(thresholds.payloadReleasePercent * 100, 0);
    Serial.println(" %");
    Serial.print("  계란 방출 고도: ");
    Serial.print(thresholds.probeReleaseAltitude, 1);
    Serial.print(" ± ");
    Serial.print(thresholds.probeReleaseMargin, 1);
    Serial.println(" m");
    Serial.print("  착륙 고도: ");
    Serial.print(thresholds.landedAltitude, 1);
    Serial.println(" m");
}

void State::forceState(FlightState newState) {
    Serial.print("State - 강제 상태 변경: ");
    Serial.println(getStateString(newState));
    
    changeState(newState);
}

void State::reset() {
    Serial.println("State - 리셋");
    
    currentState = LAUNCH_PAD;
    previousState = LAUNCH_PAD;
    
    isCalibrated = false;
    isMissionStarted = false;
    
    maxAltitude = 0.0;
    prevAltitude = 0.0;
    currentAltitude = 0.0;
    
    descentCount = 0;
    stableCount = 0;
    
    payloadReleased = false;
    probeReleased = false;
    
    apogeeDetectStart = 0;
    landedDetectStart = 0;
    stateChangeTime = millis();
}

void State::printStatus() {
    Serial.println("=== State Status ===");
    Serial.print("현재 상태: ");
    Serial.println(getStateString());
    Serial.print("미션 활성: ");
    Serial.println(isMissionStarted ? "YES" : "NO");
    Serial.print("보정 완료: ");
    Serial.println(isCalibrated ? "YES" : "NO");
    Serial.print("현재 고도: ");
    Serial.print(currentAltitude, 2);
    Serial.println(" m");
    Serial.print("최대 고도: ");
    Serial.print(maxAltitude, 2);
    Serial.println(" m");
    Serial.print("지면 고도: ");
    Serial.print(groundAltitude, 2);
    Serial.println(" m");
    Serial.print("Payload 분리: ");
    Serial.println(payloadReleased ? "YES" : "NO");
    Serial.print("계란 방출: ");
    Serial.println(probeReleased ? "YES" : "NO");
    Serial.println("===================");
}