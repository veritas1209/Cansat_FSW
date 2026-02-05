// src/sensors/BNO085.cpp
#include "sensors/BNO085.h"
#include <math.h>

BNO085::BNO085() : bno08x(-1) {  // RESET 핀 -1
    accel_x = accel_y = accel_z = 0.0;
    gyro_r = gyro_p = gyro_y = 0.0;
    quat_i = quat_j = quat_k = quat_real = 0.0;
    accel_roll_raw = accel_pitch_raw = accel_yaw_raw = 0.0;
    accel_roll_filtered = accel_pitch_filtered = accel_yaw_filtered = 0.0;
    gyro_roll_filtered = gyro_pitch_filtered = gyro_yaw_filtered = 0.0;
    
    initialized = false;
    has_accel = false;
    has_gyro = false;
    has_quat = false;
    filterEnabled = false;
}

bool BNO085::begin() {
    Serial.println("\n=== BNO085 초기화 시작 ===");
    Serial.println("I2C 핀: SCL=19, SDA=18 (Wire 버스)");
    
    // Wire 초기화 (SCL=19, SDA=18)
    Wire.begin();
    Wire.setClock(400000);  // 400kHz
    delay(100);
    
    // BNO085 초기화
    Serial.print("BNO08x 칩 연결 시도... ");
    if (!bno08x.begin_I2C()) {
        Serial.println("실패!");
        Serial.println("✗ BNO08x를 찾을 수 없습니다");
        Serial.println("\n배선 확인:");
        Serial.println("  SCL -> 19번 핀");
        Serial.println("  SDA -> 18번 핀");
        Serial.println("  VCC -> 3.3V");
        Serial.println("  GND -> GND");
        initialized = false;
        return false;
    }
    Serial.println("성공!");
    
    Serial.println("\n센서 리포트 활성화 중...");
    
    // 가속도계 활성화 (100Hz = 10000us)
    if (bno08x.enableReport(SH2_ACCELEROMETER, 10000)) {
        Serial.println("  ✓ 가속도계 (100Hz)");
    } else {
        Serial.println("  ✗ 가속도계 실패");
    }
    delay(50);
    
    // 자이로 활성화 (100Hz)
    if (bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000)) {
        Serial.println("  ✓ 자이로 보정 (100Hz)");
    } else {
        Serial.println("  ✗ 자이로 실패");
    }
    delay(50);
    
    // 회전 벡터 활성화 (100Hz)
    if (bno08x.enableReport(SH2_ROTATION_VECTOR, 10000)) {
        Serial.println("  ✓ 회전 벡터 (100Hz)");
    } else {
        Serial.println("  ✗ 회전 벡터 실패");
    }
    delay(50);
    
    // 자력계 활성화 (50Hz = 20000us)
    if (bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 20000)) {
        Serial.println("  ✓ 자력계 (50Hz)");
    } else {
        Serial.println("  ✗ 자력계 실패");
    }
    
    Serial.println("\n센서 안정화 대기 중 (3초)...");
    delay(3000);
    
    // 첫 데이터 수신 확인
    Serial.print("첫 데이터 수신 대기... ");
    unsigned long waitStart = millis();
    bool gotData = false;
    
    while (millis() - waitStart < 5000) {
        if (bno08x.wasReset()) {
            Serial.println("\n  센서 리셋 감지! 리포트 재활성화...");
            setReports();
            delay(500);
        }
        
        if (bno08x.getSensorEvent(&sensorValue)) {
            gotData = true;
            Serial.println("성공!");
            Serial.print("  첫 센서 타입: 0x");
            Serial.println(sensorValue.sensorId, HEX);
            break;
        }
        delay(10);
    }
    
    if (!gotData) {
        Serial.println("타임아웃!");
        Serial.println("  ⚠ 센서가 데이터를 보내지 않습니다");
    }
    
    initialized = true;
    Serial.println("\n=== BNO085 초기화 완료 ===\n");
    return true;
}

void BNO085::setReports() {
    bno08x.enableReport(SH2_ACCELEROMETER, 10000);
    bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);
    bno08x.enableReport(SH2_ROTATION_VECTOR, 10000);
    bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 20000);
}

void BNO085::enableFilter(float gyro_process_noise, 
                          float gyro_measurement_noise,
                          float accel_process_noise,
                          float accel_measurement_noise) {
    if (!initialized) {
        Serial.println("BNO085 - 센서가 초기화되지 않아 필터를 활성화할 수 없습니다.");
        return;
    }
    
    Serial.println("BNO085 - 칼만 필터 활성화 중...");
    
    imuFilter.begin(gyro_process_noise, 
                    gyro_measurement_noise,
                    accel_process_noise,
                    accel_measurement_noise);
    
    filterEnabled = true;
    
    Serial.println("BNO085 - 칼만 필터 활성화 완료!");
}

void BNO085::disableFilter() {
    filterEnabled = false;
    Serial.println("BNO085 - 칼만 필터 비활성화");
}

void BNO085::setGyroNoise(float process_noise, float measurement_noise) {
    if (filterEnabled) {
        imuFilter.setGyroNoise(process_noise, measurement_noise);
    }
}

void BNO085::setAccelNoise(float process_noise, float measurement_noise) {
    if (filterEnabled) {
        imuFilter.setAccelNoise(process_noise, measurement_noise);
    }
}

void BNO085::update() {
    if (!initialized) return;
    
    static int updateCount = 0;
    static int eventCount = 0;
    static int resetCount = 0;
    static unsigned long lastDebug = 0;
    
    updateCount++;
    
    // 센서 리셋 체크
    if (bno08x.wasReset()) {
        resetCount++;
        Serial.println("\n⚠ BNO085 리셋 감지! 리포트 재활성화 중...");
        setReports();
        delay(100);
    }
    
    // 센서 이벤트 읽기
    if (bno08x.getSensorEvent(&sensorValue)) {
        eventCount++;
        
        switch (sensorValue.sensorId) {
            case SH2_ACCELEROMETER:
                accel_x = sensorValue.un.accelerometer.x;
                accel_y = sensorValue.un.accelerometer.y;
                accel_z = sensorValue.un.accelerometer.z;
                has_accel = true;
                break;
                
            case SH2_GYROSCOPE_CALIBRATED:
                // 자이로는 rad/s로 들어오므로 도(degree)로 변환하지 않음
                // getGyroXXX() 함수에서 quaternion 기반 euler 각도 사용
                has_gyro = true;
                break;
                
            case SH2_ROTATION_VECTOR:
                quat_i = sensorValue.un.rotationVector.i;
                quat_j = sensorValue.un.rotationVector.j;
                quat_k = sensorValue.un.rotationVector.k;
                quat_real = sensorValue.un.rotationVector.real;
                has_quat = true;
                
                // 쿼터니언으로 오일러각 계산
                quaternionToEuler();
                
                // 가속도를 RPY 방향으로 변환
                if (has_accel) {
                    transformAccelToRPY();
                    
                    // 필터 적용
                    if (filterEnabled) {
                        // 3축 한번에 업데이트
                        imuFilter.updateAccel(accel_roll_raw, accel_pitch_raw, accel_yaw_raw);
                        imuFilter.updateGyro(gyro_r, gyro_p, gyro_y);
                        
                        // 필터링된 값 가져오기
                        accel_roll_filtered = imuFilter.getAccelRoll();
                        accel_pitch_filtered = imuFilter.getAccelPitch();
                        accel_yaw_filtered = imuFilter.getAccelYaw();
                        
                        gyro_roll_filtered = imuFilter.getGyroRoll();
                        gyro_pitch_filtered = imuFilter.getGyroPitch();
                        gyro_yaw_filtered = imuFilter.getGyroYaw();
                    }
                }
                break;
                
            case SH2_MAGNETIC_FIELD_CALIBRATED:
                // 자력계 데이터는 필요시 추가
                break;
        }
    }
}

void BNO085::quaternionToEuler() {
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
    // 쿼터니언의 켤레
    float q_conj_i = -quat_i;
    float q_conj_j = -quat_j;
    float q_conj_k = -quat_k;
    float q_conj_real = quat_real;
    
    // q * v (쿼터니언 곱셈 첫 단계)
    float temp_real = -quat_i * accel_x - quat_j * accel_y - quat_k * accel_z;
    float temp_i = quat_real * accel_x + quat_j * accel_z - quat_k * accel_y;
    float temp_j = quat_real * accel_y + quat_k * accel_x - quat_i * accel_z;
    float temp_k = quat_real * accel_z + quat_i * accel_y - quat_j * accel_x;
    
    // (q * v) * q_conj (쿼터니언 곱셈 두 번째 단계)
    accel_roll_raw = temp_i * q_conj_real + temp_real * q_conj_i + 
                     temp_j * q_conj_k - temp_k * q_conj_j;
    accel_pitch_raw = temp_j * q_conj_real + temp_real * q_conj_j + 
                      temp_k * q_conj_i - temp_i * q_conj_k;
    accel_yaw_raw = temp_k * q_conj_real + temp_real * q_conj_k + 
                    temp_i * q_conj_j - temp_j * q_conj_i;
}