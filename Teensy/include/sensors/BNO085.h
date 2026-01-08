// include/sensors/BNO085.h
#ifndef BNO085_H
#define BNO085_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include "Filter.h"

// INT 핀 사용 안 함 (폴링 모드)

class BNO085 {
private:
    Adafruit_BNO08x bno08x;
    sh2_SensorValue_t sensorValue;
    
    // 센서 데이터 (원본 XYZ)
    float accel_x, accel_y, accel_z;
    float gyro_r, gyro_p, gyro_y;  // Roll, Pitch, Yaw (Euler angles)
    float quat_i, quat_j, quat_k, quat_real;
    
    // 변환된 가속도 (RPY 방향) - Raw 값
    float accel_roll_raw, accel_pitch_raw, accel_yaw_raw;
    
    // 필터링된 값들
    float accel_roll_filtered, accel_pitch_filtered, accel_yaw_filtered;
    float gyro_roll_filtered, gyro_pitch_filtered, gyro_yaw_filtered;
    
    // IMU 칼만 필터 (내부에서 관리)
    IMUFilter imuFilter;
    bool filterEnabled;
    
    bool initialized;
    bool has_accel;
    bool has_gyro;
    bool has_quat;
    
    // 쿼터니언을 오일러각으로 변환
    void quaternionToEuler();
    
    // 가속도를 RPY 방향으로 변환
    void transformAccelToRPY();

public:
    BNO085();
    
    // 초기화 (Wire1 사용 - 핀 17/16)
    bool begin();
    
    // 칼만 필터 활성화 (선택적)
    void enableFilter(float gyro_process_noise = 0.01, 
                      float gyro_measurement_noise = 0.1,
                      float accel_process_noise = 0.01,
                      float accel_measurement_noise = 0.5);
    
    // 칼만 필터 비활성화
    void disableFilter();
    
    // 필터 노이즈 파라미터 튜닝
    void setGyroNoise(float process_noise, float measurement_noise);
    void setAccelNoise(float process_noise, float measurement_noise);
    
    // 센서 데이터 읽기 (폴링 방식) + 필터링
    void update();
    
    // 가속도 데이터 접근 (m/s²) - 원본 XYZ
    float getAccelX() { return accel_x; }
    float getAccelY() { return accel_y; }
    float getAccelZ() { return accel_z; }
    
    // 가속도 데이터 접근 (m/s²) - RPY 방향 (필터링된 값 또는 Raw)
    float getAccelRoll() { return filterEnabled ? accel_roll_filtered : accel_roll_raw; }
    float getAccelPitch() { return filterEnabled ? accel_pitch_filtered : accel_pitch_raw; }
    float getAccelYaw() { return filterEnabled ? accel_yaw_filtered : accel_yaw_raw; }
    
    // 자이로 데이터 접근 (도, degrees) - 필터링된 값 또는 Raw
    float getGyroRoll() { return filterEnabled ? gyro_roll_filtered : gyro_r; }
    float getGyroPitch() { return filterEnabled ? gyro_pitch_filtered : gyro_p; }
    float getGyroYaw() { return filterEnabled ? gyro_yaw_filtered : gyro_y; }
    
    // Raw 데이터 접근 (필터링 전)
    float getGyroRollRaw() { return gyro_r; }
    float getGyroPitchRaw() { return gyro_p; }
    float getGyroYawRaw() { return gyro_y; }
    float getAccelRollRaw() { return accel_roll_raw; }
    float getAccelPitchRaw() { return accel_pitch_raw; }
    float getAccelYawRaw() { return accel_yaw_raw; }
    
    // 쿼터니언 데이터 접근
    float getQuatI() { return quat_i; }
    float getQuatJ() { return quat_j; }
    float getQuatK() { return quat_k; }
    float getQuatReal() { return quat_real; }
    
    // 상태 확인
    bool isInitialized() { return initialized; }
    bool hasAccelData() { return has_accel; }
    bool hasGyroData() { return has_gyro; }
    bool hasQuatData() { return has_quat; }
    bool isFilterEnabled() { return filterEnabled; }
};

#endif