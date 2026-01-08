// src/Packet.cpp
#include "Packet.h"

Packet::Packet() {
    bmp = nullptr;
    imu = nullptr;
    gps = nullptr;
    state = nullptr;  // State 초기화
    
    packetCount = 0;
    currentMode = 'F';
    lastCommand = "NONE";
    missionStartTime = 0;
    missionStarted = false;
}

void Packet::attachSensors(BMP390* bmp390, BNO085* bno085, GPS* gpsModule) {
    bmp = bmp390;
    imu = bno085;
    gps = gpsModule;
    
    Serial.println("Packet - 센서 연결 완료");
}

void Packet::attachState(State* stateModule) {
    state = stateModule;
    Serial.println("Packet - State 모듈 연결 완료");
}

void Packet::startMission() {
    missionStartTime = millis();
    packetCount = 0;
    missionStarted = true;
    Serial.println("Packet - 미션 시작!");
    Serial.print("  시작 시간: ");
    Serial.println(missionStartTime);
}

void Packet::stopMission() {
    missionStarted = false;
    Serial.println("Packet - 미션 종료!");
}

TelemetryPacket Packet::collectData() {
    TelemetryPacket packet;
    
    // 기본 정보
    packet.teamId = "1062";
    
    // 미션 시간 계산
    if (missionStarted) {
        packet.missionTime = formatMissionTime(millis() - missionStartTime);
    } else {
        packet.missionTime = "00:00:00";
    }
    
    packet.packetCount = packetCount;
    packet.mode = currentMode;
    
    // State 모듈에서 상태 가져오기
    if (state) {
        packet.state = state->getStateString();
    } else {
        packet.state = "UNKNOWN";
    }
    
    // BMP390 데이터
    if (bmp && bmp->isInitialized()) {
        // State가 있으면 상대 고도 사용, 없으면 절대 고도 사용
        if (state) {
            packet.altitude = state->getCurrentAltitude();
        } else {
            packet.altitude = bmp->getAltitude();
        }
        packet.temperature = bmp->getTemperature();
        packet.pressure = bmp->getPressure();
    } else {
        packet.altitude = 0.0;
        packet.temperature = 0.0;
        packet.pressure = 0.0;
    }
    
    // 전원 데이터 (임시 더미값 - 나중에 실제 측정)
    packet.voltage = 0.0;
    packet.current = 0.0;
    
    // BNO085 데이터 - 필터링된 값 사용 (BNO085 내부에서 필터링)
    if (imu && imu->isInitialized()) {
        packet.gyro_r = imu->getGyroRoll();
        packet.gyro_p = imu->getGyroPitch();
        packet.gyro_y = imu->getGyroYaw();
        
        packet.accel_r = imu->getAccelRoll();
        packet.accel_p = imu->getAccelPitch();
        packet.accel_y = imu->getAccelYaw();
    } else {
        packet.gyro_r = 0.0;
        packet.gyro_p = 0.0;
        packet.gyro_y = 0.0;
        packet.accel_r = 0.0;
        packet.accel_p = 0.0;
        packet.accel_y = 0.0;
    }
    
    // GPS 데이터
    if (gps && gps->isInitialized()) {
        if (gps->hasFix()) {
            packet.gpsTime = gps->getTimeString();
            packet.gpsAltitude = gps->getAltitude();
            packet.gpsLatitude = gps->getLatitude();
            packet.gpsLongitude = gps->getLongitude();
            packet.gpsSats = gps->getSatellites();
        } else {
            packet.gpsTime = "00:00:00";
            packet.gpsAltitude = 0.0;
            packet.gpsLatitude = 0.0;
            packet.gpsLongitude = 0.0;
            packet.gpsSats = 0;
        }
    } else {
        packet.gpsTime = "00:00:00";
        packet.gpsAltitude = 0.0;
        packet.gpsLatitude = 0.0;
        packet.gpsLongitude = 0.0;
        packet.gpsSats = 0;
    }
    
    // 명령어 에코
    packet.cmdEcho = lastCommand;
    
    return packet;
}

String Packet::generatePacketString() {
    TelemetryPacket packet = collectData();
    return formatCSV(packet);
}

String Packet::formatCSV(const TelemetryPacket& packet) {
    // CSV 형식: 각 필드를 쉼표로 구분
    String csv = "";
    
    csv += packet.teamId + ",";
    csv += packet.missionTime + ",";
    csv += String(packet.packetCount) + ",";
    csv += String(packet.mode) + ",";
    csv += packet.state + ",";
    
    // BMP390
    csv += String(packet.altitude, 2) + ",";
    csv += String(packet.temperature, 2) + ",";
    csv += String(packet.pressure, 2) + ",";
    
    // 전원
    csv += String(packet.voltage, 2) + ",";
    csv += String(packet.current, 2) + ",";
    
    // BNO085 자이로 (필터링된 값)
    csv += String(packet.gyro_r, 2) + ",";
    csv += String(packet.gyro_p, 2) + ",";
    csv += String(packet.gyro_y, 2) + ",";
    
    // BNO085 가속도 (필터링된 값)
    csv += String(packet.accel_r, 2) + ",";
    csv += String(packet.accel_p, 2) + ",";
    csv += String(packet.accel_y, 2) + ",";
    
    // GPS
    csv += packet.gpsTime + ",";
    csv += String(packet.gpsAltitude, 2) + ",";
    csv += String(packet.gpsLatitude, 6) + ",";
    csv += String(packet.gpsLongitude, 6) + ",";
    csv += String(packet.gpsSats) + ",";
    
    // 명령어 에코
    csv += packet.cmdEcho;
    
    return csv;
}

String Packet::getCSVHeader() {
    return "TEAM_ID,MISSION_TIME,PACKET_COUNT,MODE,STATE,ALTITUDE,TEMPERATURE,ATM_PRESSURE,VOLTAGE,CURRENT,GYRO_R,GYRO_P,GYRO_Y,ACCEL_R,ACCEL_P,ACCEL_Y,GPS_TIME,GPS_ALTITUDE,GPS_LATITUDE,GPS_LONGITUDE,GPS_SATS,CMD_ECHO";
}

String Packet::formatMissionTime(unsigned long elapsedMillis) {
    // millis를 hh:mm:ss로 변환
    unsigned long totalSeconds = elapsedMillis / 1000;
    
    int hours = (totalSeconds / 3600) % 24;
    int minutes = (totalSeconds / 60) % 60;
    int seconds = totalSeconds % 60;