// src/sensors/BNO085.cpp
#include "sensors/BNO085.h"
#include <math.h>

BNO085::BNO085() {
    accel_x = accel_y = accel_z = 0.0;
    gyro_r = gyro_p = gyro_y = 0.0;
    quat_i = quat_j = quat_k = quat_real = 0.0;
    
    initialized = false;
    has_accel = false;
    has_gyro = false;
    has_quat = false;
    lastResetCheck = 0;
}

bool BNO085::begin() {
    Serial.println("BNO085 초기화 시작...");
    
    // Wire1이 이미 begin()되었는지 확인하지 않고 다시 호출
    // 중복 호출은 문제없지만, 핀 설정 순서 변경
    pinMode(BNO085_INT, INPUT);
    Wire1.begin();
    Wire1.setClock(100000); // 100kHz (안정성)
    delay(500); // 대기 시간 증가
    
    // I2C 주소 스캔
    Serial.print("I2C Wire1 스캔 중... ");
    Wire1.beginTransmission(0x4A);
    byte error1 = Wire1.endTransmission();
    Wire1.beginTransmission(0x4B);
    byte error2 = Wire1.endTransmission();
    Serial.print("0x4A: ");
    Serial.print(error1 == 0 ? "발견!" : "없음");
    Serial.print(" (오류코드: ");
    Serial.print(error1);
    Serial.print("), 0x4B: ");
    Serial.print(error2 == 0 ? "발견!" : "없음");
    Serial.print(" (오류코드: ");
    Serial.print(error2);
    Serial.println(")");
    
    delay(100);
    Serial.print("BNO085 연결 시도 (0x4A)... ");
    
    // 여러 번 시도 - 기존 코드와 동일하게
    bool bno_success = false;
    for (int attempt = 0; attempt < 3; attempt++) {
        if (attempt > 0) {
            Serial.print("재시도 ");
            Serial.print(attempt);
            Serial.print("... ");
            delay(500);
        }
        
        if (bno08x.begin_I2C(0x4A, &Wire1, BNO085_INT)) {
            Serial.println("성공!");
            bno_success = true;
            delay(100);
            
            // 센서 리포트 활성화
            Serial.println("센서 리포트 활성화 중...");
            if (bno08x.enableReport(SH2_ACCELEROMETER, 50000)) {
                Serial.println("  - 가속도계 활성화 성공");
            }
            if (bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 50000)) {
                Serial.println("  - 자이로 활성화 성공");
            }
            if (bno08x.enableReport(SH2_ROTATION_VECTOR, 50000)) {
                Serial.println("  - 회전벡터 활성화 성공");
            }
            break;
        }
    }
    
    if (!bno_success) {
        Serial.println("실패! RST 핀 확인 또는 센서 재부팅 필요");
    }
    
    initialized = bno_success;
    return bno_success;
}

void BNO085::update() {
    if (!initialized) return;
    
    // 리셋 체크 (1초에 한 번)
    if (millis() - lastResetCheck > 1000) {
        lastResetCheck = millis();
        if (bno08x.wasReset()) {
            Serial.println("BNO085 - 리셋 감지됨, 재활성화 중...");
            bno08x.enableReport(SH2_ACCELEROMETER, 50000);
            bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 50000);
            bno08x.enableReport(SH2_ROTATION_VECTOR, 50000);
            delay(100);
        }
    }
    
    // 센서 이벤트 읽기 - 기존 코드와 완전히 동일
    while (bno08x.getSensorEvent(&sensorValue)) {
        switch (sensorValue.sensorId) {
            case SH2_ACCELEROMETER:
                accel_x = sensorValue.un.accelerometer.x;
                accel_y = sensorValue.un.accelerometer.y;
                accel_z = sensorValue.un.accelerometer.z;
                has_accel = true;
                break;
                
            case SH2_GYROSCOPE_CALIBRATED:
                has_gyro = true;
                break;
                
            case SH2_ROTATION_VECTOR:
                quat_i = sensorValue.un.rotationVector.i;
                quat_j = sensorValue.un.rotationVector.j;
                quat_k = sensorValue.un.rotationVector.k;
                quat_real = sensorValue.un.rotationVector.real;
                has_quat = true;
                
                // 쿼터니언을 오일러각으로 변환
                quaternionToEuler();
                break;
        }
    }
}

void BNO085::quaternionToEuler() {
    // 쿼터니언 -> 오일러각 (Roll, Pitch, Yaw) 변환
    // Roll (X축 회전)
    float sinr_cosp = 2.0 * (quat_real * quat_i + quat_j * quat_k);
    float cosr_cosp = 1.0 - 2.0 * (quat_i * quat_i + quat_j * quat_j);
    gyro_r = atan2(sinr_cosp, cosr_cosp) * 180.0 / M_PI;
    
    // Pitch (Y축 회전)
    float sinp = 2.0 * (quat_real * quat_j - quat_k * quat_i);
    if (fabs(sinp) >= 1)
        gyro_p = copysign(M_PI / 2, sinp) * 180.0 / M_PI;
    else
        gyro_p = asin(sinp) * 180.0 / M_PI;
    
    // Yaw (Z축 회전)
    float siny_cosp = 2.0 * (quat_real * quat_k + quat_i * quat_j);
    float cosy_cosp = 1.0 - 2.0 * (quat_j * quat_j + quat_k * quat_k);
    gyro_y = atan2(siny_cosp, cosy_cosp) * 180.0 / M_PI;
}

/*test*/