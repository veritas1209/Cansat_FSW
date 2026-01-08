// src/sensors/Audio.cpp
#include "sensors/Audio.h"

Audio::Audio() {
    initialized = false;
    isBeeping = false;
    lastToggle = 0;
    beepInterval = 1000;
    beepDuration = 200;
    beepState = false;
}

bool Audio::begin() {
    Serial.print("Audio 부저 초기화 중 (핀 ");
    Serial.print(BUZZER_PIN);
    Serial.print(")... ");
    
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);  // 초기 상태: OFF
    
    initialized = true;
    Serial.println("완료!");
    
    return true;
}

void Audio::startBeep(int interval_ms, int duration_ms) {
    if (!initialized) return;
    
    beepInterval = interval_ms;
    beepDuration = duration_ms;
    isBeeping = true;
    beepState = false;
    lastToggle = millis();
    
    Serial.println("Audio - 비프음 시작");
    Serial.print("  간격: ");
    Serial.print(beepInterval);
    Serial.print(" ms, 지속: ");
    Serial.print(beepDuration);
    Serial.println(" ms");
}

void Audio::stopBeep() {
    if (!initialized) return;
    
    isBeeping = false;
    beepState = false;
    noTone(BUZZER_PIN);  // tone 끄기
    
    Serial.println("Audio - 비프음 중지");
}

void Audio::singleBeep(int duration_ms) {
    if (!initialized) return;
    
    tone(BUZZER_PIN, 2000);  // 2000Hz
    delay(duration_ms);
    noTone(BUZZER_PIN);
}

void Audio::update() {
    if (!initialized || !isBeeping) return;
    
    unsigned long currentTime = millis();
    
    if (beepState) {
        // 비프 ON 상태 - duration 지나면 OFF
        if (currentTime - lastToggle >= beepDuration) {
            noTone(BUZZER_PIN);  // 소리 끄기
            beepState = false;
            lastToggle = currentTime;
        }
    } else {
        // 비프 OFF 상태 - interval 지나면 ON
        if (currentTime - lastToggle >= (beepInterval - beepDuration)) {
            tone(BUZZER_PIN, 2000);  // 2000Hz 소리 내기
            beepState = true;
            lastToggle = currentTime;
        }
    }
}

void Audio::on() {
    if (!initialized) return;
    tone(BUZZER_PIN, 2000);  // 2000Hz 소리
}

void Audio::off() {
    if (!initialized) return;
    noTone(BUZZER_PIN);
}

void Audio::setBeepPattern(int interval_ms, int duration_ms) {
    beepInterval = interval_ms;
    beepDuration = duration_ms;
    
    Serial.println("Audio - 비프 패턴 변경");
    Serial.print("  간격: ");
    Serial.print(beepInterval);
    Serial.print(" ms, 지속: ");
    Serial.print(beepDuration);
    Serial.println(" ms");
}