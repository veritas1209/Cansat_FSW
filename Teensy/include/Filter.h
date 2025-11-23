// include/Filter.h
#ifndef FILTER_H
#define FILTER_H

#include <Arduino.h>

// 1차원 칼만 필터 (단일 센서값 필터링용)
class Filter1D {
private:
    float x;           // 상태 (필터링된 값)
    float P;           // 오차 공분산
    float Q;           // 프로세스 노이즈
    float R;           // 측정 노이즈
    bool initialized;
    
public:
    Filter1D();
    
    // 초기화
    void begin(float initial_value, float process_noise, float measurement_noise);
    
    // 필터 업데이트 (새 측정값 입력)
    float update(float measurement);
    
    // 현재 필터링된 값 반환
    float getValue() { return x; }
    
    // 노이즈 파라미터 설정
    void setProcessNoise(float q) { Q = q; }
    void setMeasurementNoise(float r) { R = r; }
    
    // 상태 확인
    bool isInitialized() { return initialized; }
    
    // 리셋
    void reset();
};

// IMU 센서용 칼만 필터 (자이로 3축 + 가속도 3축)
class IMUFilter {
private:
    // 자이로 필터 (Roll, Pitch, Yaw)
    Filter1D gyro_r_filter;
    Filter1D gyro_p_filter;
    Filter1D gyro_y_filter;
    
    // 가속도 필터 (Roll, Pitch, Yaw)
    Filter1D accel_r_filter;
    Filter1D accel_p_filter;
    Filter1D accel_y_filter;
    
    bool initialized;

public:
    IMUFilter();
    
    // 초기화 (노이즈 파라미터 설정)
    void begin(float gyro_process_noise = 0.01, 
               float gyro_measurement_noise = 0.1,
               float accel_process_noise = 0.01,
               float accel_measurement_noise = 0.5);
    
    // 자이로 데이터 필터링
    void updateGyro(float raw_roll, float raw_pitch, float raw_yaw);
    
    // 가속도 데이터 필터링
    void updateAccel(float raw_roll, float raw_pitch, float raw_yaw);
    
    // 필터링된 자이로 데이터 반환
    float getGyroRoll() { return gyro_r_filter.getValue(); }
    float getGyroPitch() { return gyro_p_filter.getValue(); }
    float getGyroYaw() { return gyro_y_filter.getValue(); }
    
    // 필터링된 가속도 데이터 반환
    float getAccelRoll() { return accel_r_filter.getValue(); }
    float getAccelPitch() { return accel_p_filter.getValue(); }
    float getAccelYaw() { return accel_y_filter.getValue(); }
    
    // 노이즈 파라미터 튜닝
    void setGyroNoise(float process_noise, float measurement_noise);
    void setAccelNoise(float process_noise, float measurement_noise);
    
    // 상태 확인
    bool isInitialized() { return initialized; }
    
    // 필터 리셋
    void reset();
};

#endif