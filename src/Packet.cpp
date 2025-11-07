// src/Packet.cpp
#include "Packet.h"

Packet::Packet() {
    bmp = nullptr;
    imu = nullptr;
    gps = nullptr;
    
    packetCount = 0;
    currentState = "LAUNCH_PAD";
    currentMode = 'F';
    lastCommand = "NONE";
    missionStartTime = 0;
}

void Packet::attachSensors(BMP390* bmp390, BNO085* bno085, GPS* gpsModule) {
    bmp = bmp390;
    imu = bno085;
    gps = gpsModule;
    
    Serial.println("Packet - 센서 연결 완료");
}

void Packet::beginMission() {
    missionStartTime = millis();
    packetCount = 0;
    Serial.println("Packet - 미션 시작!");
}

TelemetryPacket Packet::collectData() {
    TelemetryPacket packet;
    
    // 기본 정보
    packet.teamId = "1062";
    packet.missionTime = formatMissionTime(millis() - missionStartTime);
    packet.packetCount = packetCount;
    packet.mode = currentMode;
    packet.state = currentState;
    
    // BMP390 데이터
    if (bmp && bmp->isInitialized()) {
        packet.altitude = bmp->getAltitude();
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
    
    // BNO085 자이로 데이터
    if (imu && imu->isInitialized()) {
        packet.gyro_r = imu->getGyroRoll();
        packet.gyro_p = imu->getGyroPitch();
        packet.gyro_y = imu->getGyroYaw();
        
        // 가속도 데이터 - RPY 방향으로 변환된 값 사용
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
    
    // BNO085 자이로
    csv += String(packet.gyro_r, 2) + ",";
    csv += String(packet.gyro_p, 2) + ",";
    csv += String(packet.gyro_y, 2) + ",";
    
    // BNO085 가속도
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

void Packet::transmit() {
    String packetStr = generatePacketString();
    Serial.println(packetStr);
    packetCount++;  // 전송 후 카운터 증가
}

String Packet::formatMissionTime(unsigned long elapsedMillis) {
    // millis를 hh:mm:ss로 변환
    unsigned long totalSeconds = elapsedMillis / 1000;
    
    int hours = (totalSeconds / 3600) % 24;
    int minutes = (totalSeconds / 60) % 60;
    int seconds = totalSeconds % 60;
    
    char timeStr[9];
    sprintf(timeStr, "%02d:%02d:%02d", hours, minutes, seconds);
    return String(timeStr);
}