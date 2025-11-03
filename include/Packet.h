// include/Packet.h
#ifndef PACKET_H
#define PACKET_H

#include <Arduino.h>
#include "sensors/BMP390.h"
#include "sensors/BNO085.h"
#include "sensors/GPS.h"

// 패킷 데이터 구조체
struct TelemetryPacket {
    // 기본 정보
    String teamId;
    String missionTime;      // hh:mm:ss (UTC)
    uint32_t packetCount;
    char mode;               // 'F' or 'S'
    String state;            // LAUNCH_PAD, ASCENT, etc.
    
    // BMP390 데이터
    float altitude;          // m
    float temperature;       // °C
    float pressure;          // hPa
    
    // 전원 데이터
    float voltage;           // V
    float current;           // A
    
    // BNO085 데이터
    float gyro_r;            // Roll (°)
    float gyro_p;            // Pitch (°)
    float gyro_y;            // Yaw (°)
    float accel_r;           // Roll acceleration
    float accel_p;           // Pitch acceleration
    float accel_y;           // Yaw acceleration
    
    // GPS 데이터
    String gpsTime;          // hh:mm:ss
    float gpsAltitude;       // m
    float gpsLatitude;       // degrees
    float gpsLongitude;      // degrees
    int gpsSats;             // satellite count
    
    // 명령어 에코
    String cmdEcho;
};

class Packet {
private:
    // 센서 참조
    BMP390* bmp;
    BNO085* imu;
    GPS* gps;
    
    // 패킷 카운터
    uint32_t packetCount;
    
    // 임시 상태 (나중에 State.h로 이동)
    String currentState;
    char currentMode;
    String lastCommand;
    
    // 미션 시작 시간 (밀리초)
    unsigned long missionStartTime;
    
    // CSV 문자열 생성 헬퍼
    String formatCSV(const TelemetryPacket& packet);
    
    // 시간 포맷 헬퍼 (millis를 hh:mm:ss로 변환)
    String formatMissionTime(unsigned long elapsedMillis);

public:
    Packet();
    
    // 센서 연결
    void attachSensors(BMP390* bmp390, BNO085* bno085, GPS* gpsModule);
    
    // 미션 시작 (타이머 시작)
    void beginMission();
    
    // 패킷 데이터 수집 및 생성
    TelemetryPacket collectData();
    
    // CSV 문자열로 패킷 생성
    String generatePacketString();
    
    // 패킷 전송 (Serial)
    void transmit();
    
    // 상태 설정 (임시 - 나중에 State 모듈로 대체)
    void setState(String state) { currentState = state; }
    void setMode(char mode) { currentMode = mode; }
    void setCommandEcho(String cmd) { lastCommand = cmd; }
    
    // 카운터 접근
    uint32_t getPacketCount() { return packetCount; }
};

#endif