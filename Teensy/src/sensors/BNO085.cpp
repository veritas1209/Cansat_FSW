// src/sensors/BNO085.cpp
#include "sensors/BNO085.h"
#include <math.h>

BNO085::BNO085() {
    accel_x = accel_y = accel_z = 0.0;
    gyro_r = gyro_p = gyro_y = 0.0;
    quat_i = quat_j = quat_k = quat_real = 0.0;
    accel_roll = accel_pitch = accel_yaw = 0.0;
    
    initialized = false;
    has_accel = false;
    has_gyro = false;
    has_quat = false;
    lastResetCheck = 0;
}

bool BNO085::begin() {
    Serial.println("BNO085 초기화 시작...");
    
    pinMode(BNO085_INT, INPUT);
    Wire1.begin();
    Wire1.setClock(100000); // 100kHz (안정성)
    delay(500);
    
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
    
    // 여러 번 시도
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
    
    // 센서 이벤트 읽기
    while (bno08x.getSensorEvent(&sensorValue)) {
        switch (sensorValue.sensorId) {
            case SH2_ACCELEROMETER:
                accel_x = sensorValue.un.accelerometer.x;
                accel_y = sensorValue.un.accelerometer.y;
                accel_z = sensorValue.un.accelerometer.z;
                has_accel = true;
                
                // 가속도 변환 (쿼터니언 있을 때만)
                if (has_quat) {
                    transformAccelToRPY();
                }
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
                
                // 가속도 변환 (가속도 데이터 있을 때만)
                if (has_accel) {
                    transformAccelToRPY();
                }
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

void BNO085::transformAccelToRPY() {
    // 쿼터니언을 사용하여 센서 좌표계(XYZ)의 가속도를
    // 월드 좌표계(RPY 방향)로 변환
    
    // 쿼터니언 회전 공식: v' = q * v * q^(-1)
    // 여기서는 역회전을 적용 (센서->월드)
    
    // 쿼터니언의 켤레 (역회전용)
    float q_conj_i = -quat_i;
    float q_conj_j = -quat_j;
    float q_conj_k = -quat_k;
    float q_conj_real = quat_real;
    
    // 첫 번째 곱셈: q * v
    float temp_real = -quat_i * accel_x - quat_j * accel_y - quat_k * accel_z;
    float temp_i = quat_real * accel_x + quat_j * accel_z - quat_k * accel_y;
    float temp_j = quat_real * accel_y + quat_k * accel_x - quat_i * accel_z;
    float temp_k = quat_real * accel_z + quat_i * accel_y - quat_j * accel_x;
    
    // 두 번째 곱셈: (q * v) * q^(-1)
    accel_roll = temp_i * q_conj_real + temp_real * q_conj_i + 
                 temp_j * q_conj_k - temp_k * q_conj_j;
    accel_pitch = temp_j * q_conj_real + temp_real * q_conj_j + 
                  temp_k * q_conj_i - temp_i * q_conj_k;
    accel_yaw = temp_k * q_conj_real + temp_real * q_conj_k + 
                temp_i * q_conj_j - temp_j * q_conj_i;
}

/*test*/