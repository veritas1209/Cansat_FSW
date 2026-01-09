// src/sensors/BNO085.cpp
#include "sensors/BNO085.h"
#include <math.h>

BNO085::BNO085() {
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
    Serial.println("BNO085 초기화 시작 (2025 방식)...");
    
    // Wire1 초기화
    Wire1.begin();
    Wire1.setClock(100000);  // 100kHz (과거 코드와 동일)
    delay(100);
    
    Serial.print("BNO085 연결 시도... ");
    
    // 주소 지정 없이 begin_I2C() 호출 (라이브러리가 자동 검색)
    if (bno08x.begin_I2C(0x4A, &Wire1)) {  // 기본 주소만 시도
        Serial.println("성공!");
        
        // 센서 리포트 활성화
        Serial.println("센서 리포트 활성화 중...");
        bno08x.enableReport(SH2_ACCELEROMETER);
        bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED);
        bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED);  // 자력계 추가
        bno08x.enableReport(SH2_ROTATION_VECTOR);
        
        Serial.println("  - 가속도계 활성화");
        Serial.println("  - 자이로 활성화");
        Serial.println("  - 자력계 활성화");
        Serial.println("  - 회전벡터 활성화");
        
        delay(2000);  // 센서 안정화 대기
        
        initialized = true;
        return true;
    } else {
        Serial.println("실패! 센서 연결 확인 필요");
        Serial.println("  연결 확인:");
        Serial.println("    VIN → 3.3V");
        Serial.println("    GND → GND");
        Serial.println("    SCL → Pin 16");
        Serial.println("    SDA → Pin 17");
        
        initialized = false;
        return false;
    }
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
    
    // 센서 이벤트 읽기 시도
    if (bno08x.getSensorEvent(&sensorValue)) {
        
        switch (sensorValue.sensorId) {
            case SH2_ACCELEROMETER:
                accel_x = sensorValue.un.accelerometer.x;
                accel_y = sensorValue.un.accelerometer.y;
                accel_z = sensorValue.un.accelerometer.z;
                has_accel = true;
                
                if (has_quat) {
                    transformAccelToRPY();
                    
                    if (filterEnabled && imuFilter.isInitialized()) {
                        imuFilter.updateAccel(accel_roll_raw, accel_pitch_raw, accel_yaw_raw);
                        accel_roll_filtered = imuFilter.getAccelRoll();
                        accel_pitch_filtered = imuFilter.getAccelPitch();
                        accel_yaw_filtered = imuFilter.getAccelYaw();
                    }
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
                
                quaternionToEuler();
                
                if (filterEnabled && imuFilter.isInitialized()) {
                    imuFilter.updateGyro(gyro_r, gyro_p, gyro_y);
                    gyro_roll_filtered = imuFilter.getGyroRoll();
                    gyro_pitch_filtered = imuFilter.getGyroPitch();
                    gyro_yaw_filtered = imuFilter.getGyroYaw();
                }
                
                if (has_accel) {
                    transformAccelToRPY();
                    
                    if (filterEnabled && imuFilter.isInitialized()) {
                        imuFilter.updateAccel(accel_roll_raw, accel_pitch_raw, accel_yaw_raw);
                        accel_roll_filtered = imuFilter.getAccelRoll();
                        accel_pitch_filtered = imuFilter.getAccelPitch();
                        accel_yaw_filtered = imuFilter.getAccelYaw();
                    }
                }
                break;
                
            case SH2_MAGNETIC_FIELD_CALIBRATED:
                // 자력계 데이터 (나중에 필요하면 추가)
                break;
        }
    }
}

void BNO085::quaternionToEuler() {
    float sinr_cosp = 2.0 * (quat_real * quat_i + quat_j * quat_k);
    float cosr_cosp = 1.0 - 2.0 * (quat_i * quat_i + quat_j * quat_j);
    gyro_r = atan2(sinr_cosp, cosr_cosp) * 180.0 / M_PI;
    
    float sinp = 2.0 * (quat_real * quat_j - quat_k * quat_i);
    if (fabs(sinp) >= 1)
        gyro_p = copysign(M_PI / 2, sinp) * 180.0 / M_PI;
    else
        gyro_p = asin(sinp) * 180.0 / M_PI;
    
    float siny_cosp = 2.0 * (quat_real * quat_k + quat_i * quat_j);
    float cosy_cosp = 1.0 - 2.0 * (quat_j * quat_j + quat_k * quat_k);
    gyro_y = atan2(siny_cosp, cosy_cosp) * 180.0 / M_PI;
}

void BNO085::transformAccelToRPY() {
    float q_conj_i = -quat_i;
    float q_conj_j = -quat_j;
    float q_conj_k = -quat_k;
    float q_conj_real = quat_real;
    
    float temp_real = -quat_i * accel_x - quat_j * accel_y - quat_k * accel_z;
    float temp_i = quat_real * accel_x + quat_j * accel_z - quat_k * accel_y;
    float temp_j = quat_real * accel_y + quat_k * accel_x - quat_i * accel_z;
    float temp_k = quat_real * accel_z + quat_i * accel_y - quat_j * accel_x;
    
    accel_roll_raw = temp_i * q_conj_real + temp_real * q_conj_i + 
                     temp_j * q_conj_k - temp_k * q_conj_j;
    accel_pitch_raw = temp_j * q_conj_real + temp_real * q_conj_j + 
                      temp_k * q_conj_i - temp_i * q_conj_k;
    accel_yaw_raw = temp_k * q_conj_real + temp_real * q_conj_k + 
                    temp_i * q_conj_j - temp_j * q_conj_i;
}