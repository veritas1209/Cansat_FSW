// include/sensors/BNO085.h
#ifndef BNO085_H
#define BNO085_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

#define BNO085_INT 15

class BNO085 {
private:
    Adafruit_BNO08x bno08x;
    sh2_SensorValue_t sensorValue;
    
    // 센서 데이터 (원본 XYZ)
    float accel_x, accel_y, accel_z;
    float gyro_r, gyro_p, gyro_y;  // Roll, Pitch, Yaw (Euler angles)
    float quat_i, quat_j, quat_k, quat_real;
    
    // 변환된 가속도 (RPY 방향)
    float accel_roll, accel_pitch, accel_yaw;
    
    bool initialized;
    bool has_accel;
    bool has_gyro;
    bool has_quat;
    
    unsigned long lastResetCheck;
    
    // 쿼터니언을 오일러각으로 변환
    void quaternionToEuler();
    
    // 가속도를 RPY 방향으로 변환
    void transformAccelToRPY();

public:
    BNO085();
    
    // 초기화 (Wire1 사용 - 핀 17/16)
    bool begin();
    
    // 센서 데이터 읽기 (폴링 방식)
    void update();
    
    // 가속도 데이터 접근 (m/s²) - 원본 XYZ
    float getAccelX() { return accel_x; }
    float getAccelY() { return accel_y; }
    float getAccelZ() { return accel_z; }
    
    // 가속도 데이터 접근 (m/s²) - RPY 방향 (변환된 값)
    float getAccelRoll() { return accel_roll; }
    float getAccelPitch() { return accel_pitch; }
    float getAccelYaw() { return accel_yaw; }
    
    // 자이로 데이터 접근 (도, degrees)
    float getGyroRoll() { return gyro_r; }
    float getGyroPitch() { return gyro_p; }
    float getGyroYaw() { return gyro_y; }
    
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
};

#endif