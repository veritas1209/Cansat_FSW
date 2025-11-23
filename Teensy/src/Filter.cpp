// src/Filter.cpp
#include "Filter.h"

// ========== Filter1D 구현 ==========

Filter1D::Filter1D() {
    x = 0.0;
    P = 1.0;
    Q = 0.01;
    R = 0.1;
    initialized = false;
}

void Filter1D::begin(float initial_value, float process_noise, float measurement_noise) {
    x = initial_value;
    P = 1.0;
    Q = process_noise;
    R = measurement_noise;
    initialized = true;
}

float Filter1D::update(float measurement) {
    if (!initialized) {
        // 첫 측정값으로 초기화
        begin(measurement, Q, R);
        return x;
    }
    
    // 예측 단계 (Prediction)
    // 상태는 그대로 유지 (등속 모델)
    // x_pred = x
    // P_pred = P + Q
    float x_pred = x;
    float P_pred = P + Q;
    
    // 업데이트 단계 (Update)
    // 칼만 게인: K = P_pred / (P_pred + R)
    float K = P_pred / (P_pred + R);
    
    // 상태 업데이트: x = x_pred + K * (measurement - x_pred)
    x = x_pred + K * (measurement - x_pred);
    
    // 공분산 업데이트: P = (1 - K) * P_pred
    P = (1.0 - K) * P_pred;
    
    return x;
}

void Filter1D::reset() {
    x = 0.0;
    P = 1.0;
    initialized = false;
}

// ========== IMUFilter 구현 ==========

IMUFilter::IMUFilter() {
    initialized = false;
}

void IMUFilter::begin(float gyro_process_noise, 
                      float gyro_measurement_noise,
                      float accel_process_noise,
                      float accel_measurement_noise) {
    
    Serial.println("IMU 칼만 필터 초기화 중...");
    
    // 자이로 필터 초기화 (초기값 0으로 설정, 첫 측정값으로 자동 초기화됨)
    gyro_r_filter.begin(0.0, gyro_process_noise, gyro_measurement_noise);
    gyro_p_filter.begin(0.0, gyro_process_noise, gyro_measurement_noise);
    gyro_y_filter.begin(0.0, gyro_process_noise, gyro_measurement_noise);
    
    // 가속도 필터 초기화
    accel_r_filter.begin(0.0, accel_process_noise, accel_measurement_noise);
    accel_p_filter.begin(0.0, accel_process_noise, accel_measurement_noise);
    accel_y_filter.begin(0.0, accel_process_noise, accel_measurement_noise);
    
    initialized = true;
    
    Serial.println("IMU 칼만 필터 초기화 완료!");
    Serial.print("  자이로 노이즈 - Q: ");
    Serial.print(gyro_process_noise, 4);
    Serial.print(", R: ");
    Serial.println(gyro_measurement_noise, 4);
    Serial.print("  가속도 노이즈 - Q: ");
    Serial.print(accel_process_noise, 4);
    Serial.print(", R: ");
    Serial.println(accel_measurement_noise, 4);
}

void IMUFilter::updateGyro(float raw_roll, float raw_pitch, float raw_yaw) {
    if (!initialized) return;
    
    // 각 축별로 필터 업데이트
    gyro_r_filter.update(raw_roll);
    gyro_p_filter.update(raw_pitch);
    gyro_y_filter.update(raw_yaw);
}

void IMUFilter::updateAccel(float raw_roll, float raw_pitch, float raw_yaw) {
    if (!initialized) return;
    
    // 각 축별로 필터 업데이트
    accel_r_filter.update(raw_roll);
    accel_p_filter.update(raw_pitch);
    accel_y_filter.update(raw_yaw);
}

void IMUFilter::setGyroNoise(float process_noise, float measurement_noise) {
    gyro_r_filter.setProcessNoise(process_noise);
    gyro_r_filter.setMeasurementNoise(measurement_noise);
    
    gyro_p_filter.setProcessNoise(process_noise);
    gyro_p_filter.setMeasurementNoise(measurement_noise);
    
    gyro_y_filter.setProcessNoise(process_noise);
    gyro_y_filter.setMeasurementNoise(measurement_noise);
    
    Serial.println("자이로 노이즈 파라미터 업데이트:");
    Serial.print("  Q: "); Serial.print(process_noise, 4);
    Serial.print(", R: "); Serial.println(measurement_noise, 4);
}

void IMUFilter::setAccelNoise(float process_noise, float measurement_noise) {
    accel_r_filter.setProcessNoise(process_noise);
    accel_r_filter.setMeasurementNoise(measurement_noise);
    
    accel_p_filter.setProcessNoise(process_noise);
    accel_p_filter.setMeasurementNoise(measurement_noise);
    
    accel_y_filter.setProcessNoise(process_noise);
    accel_y_filter.setMeasurementNoise(measurement_noise);
    
    Serial.println("가속도 노이즈 파라미터 업데이트:");
    Serial.print("  Q: "); Serial.print(process_noise, 4);
    Serial.print(", R: "); Serial.println(measurement_noise, 4);
}

void IMUFilter::reset() {
    gyro_r_filter.reset();
    gyro_p_filter.reset();
    gyro_y_filter.reset();
    
    accel_r_filter.reset();
    accel_p_filter.reset();
    accel_y_filter.reset();
    
    initialized = false;
    
    Serial.println("IMU 칼만 필터 리셋 완료");
}