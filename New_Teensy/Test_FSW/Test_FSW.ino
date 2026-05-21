#include <Arduino.h>
#include <Adafruit_GPS.h>
#include <Adafruit_BNO08x.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_INA260.h>
#include <SPI.h>
#include <SD.h>
#include <PWMServo.h>          // Teensyduino 기본 포함. 표준 Arduino Servo 라이브러리는 Teensy 4.x 미지원.
#include "Wire.h"
#include <EEPROM.h>   // 리커버리를 위한 내부 EEPROM 라이브러리 추가

// =====================================================
// Test_FSW — Real_FSW 베이스 테스트용 스케치
// 추가 명령:
//   CMD,1062,TEST              → 모든 센서 데이터 1회 읽고 패킷 출력
//   CMD,1062,CAMERA,ON|OFF     → 두 카메라 동시 녹화 시작/종료
//   CMD,1062,SERVO,ON|OFF      → 서보 동작 / 원위치 복귀
// 기존 Real_FSW의 CX/ST/SIM/SIMP/CAL/MEC 명령도 그대로 동작.
// =====================================================

// ---------------------------------------------------
// 설정
// ---------------------------------------------------

// 상수
#define TEAM_ID 1062
#define GPSSerial Serial3
#define XBEE Serial2
#define PL_RELS_CAM Serial3        // Payload Release Camera (Xiao ESP32S3) — RX3=15, TX3=14
#define GROUND_CAM Serial5         // Ground Camera (Xiao ESP32S3)         — RX5=21, TX5=20
#define CAM_BAUD 9600
#define CAM_CMD_START '1'          // ESP32S3 펌웨어 약속: '1' = 녹화 시작
#define CAM_CMD_STOP  '0'          // ESP32S3 펌웨어 약속: '0' = 녹화 종료
#define DEBUG true
#define CSV_FILENAME "Flight_1062.csv"
#define STANDARD_SEA_LEVEL_PRESSURE_HPA 1013.25

// 서보 핀 (PCB 핀맵 기준)
#define SERVO_LEFT_PIN  2          // PARA_CTRL_L → 핀 2
#define SERVO_RIGHT_PIN 4          // PARA_CTRL_R → 핀 4 (LRCLK2)

// SG90 360도(연속회전) 서보용 — 펄스폭(μs)으로 직접 제어.
// PWMServo.write(angle)이 라이브러리/보드 매핑 따라 정지 펄스에서 어긋날 수 있어 writeMicroseconds 사용.
//   1500us = 정지 (중립)
//   1000us = 한쪽 방향 최고속
//   2000us = 반대 방향 최고속
// 위치 피드백이 없으므로 "원위치 복귀"는 ON에서 회전한 시간만큼 OFF에서 역회전시키는 방식으로 흉내낸다.
// 실 모터마다 정지 펄스가 1480~1520us 사이에서 미세 조정 필요 — 트림 문제 있으면 SERVO_STOP_US 조정.
#define SERVO_STOP_US      1500
#define SERVO_FORWARD_US   1000  // ON 시 회전 속도/방향
#define SERVO_REVERSE_US   2000  // OFF 시 복귀 회전
#define TARGET_LAT 37.123456   // TODO: 실제 목표 위도로 변경
#define TARGET_LON 127.123456  // TODO: 실제 목표 경도로 변경

const int PULL_TIME_MS = 200;     // 줄을 당기는 시간 (0.2초)
const int RELEASE_TIME_MS = 190;  // 줄을 푸는 시간 (바람 장력을 고려해 당기는 시간보다 살짝 짧게 세팅)
const int COOLDOWN_TIME_MS = 800; // 한 번 당긴 후 기체가 안정될 때까지 기다리는 시간 (0.8초)
const float DEADZONE_DEG = 15.0;  // 오차가 ±15도 이내면 직진(조향 안함)

enum SteerState { 
  STEER_IDLE, 
  STEER_PULL_LEFT, STEER_RELEASE_LEFT, 
  STEER_PULL_RIGHT, STEER_RELEASE_RIGHT, 
  STEER_COOLDOWN 
};
SteerState steerState = STEER_IDLE;
unsigned long steerTimer = 0;

float headingYaw = 0; // BNO085에서 계산된 기체의 현재 절대 방향(0~360도)

// ---------------------------------------------------
// 전역 객체
// ---------------------------------------------------

// 센서 객체
Adafruit_GPS GPS(&GPSSerial);       // GPS 객체
Adafruit_BNO08x bno08x;             // BNO085 객체
Adafruit_BMP3XX bmp;                // BMP390 객체
sh2_SensorValue_t sensorValue;      // BNO085 센서 데이터 객체
Adafruit_INA260 ina260;             // INA260 객체

// 서보 객체 (PWMServo: 핀이 PWM 가능해야 함. Teensy 4.1에서 2, 4 모두 OK)
PWMServo servoLeft;
PWMServo servoRight;

// SD 카드 파일 객체
File packetFile;

// ---------------------------------------------------
// 전역 상태
// ---------------------------------------------------

// 비행 상태 정의
enum FlightState {
  LAUNCH_PAD,        // 발사대
  ASCENT,            // 상승
  APOGEE,            // 최고점
  DESCENT,           // 하강
  PAYLOAD_RELEASE,   // 페이로드 방출
  PROBE_RELEASE,     // 프로브(계란) 방출
  LANDED             // 착륙
};

// 비행 상태 문자열 (열거형 순서와 일치해야 함)
const String STATE_STRINGS[] = {
  "LAUNCH_PAD",
  "ASCENT",
  "APOGEE",
  "DESCENT",
  "PAYLOAD_RELEASE",
  "PROBE_RELEASE",
  "LANDED"
};

// ==========================================
// 리커버리(EEPROM) 데이터 구조체 및 설정
// ==========================================
#define EEPROM_ADDR 0
#define MAGIC_NUMBER 0xA5A5 // 올바른 데이터인지 판별하는 고유 암호

struct RecoveryData {
  uint16_t magicNumber;
  FlightState currentState;
  float maxAltitude;
  int packetCount;
  byte missionTime[3];
  float groundPressure;
};

RecoveryData flightData;
float lastSavedAltitude = 0; // 잦은 EEPROM 쓰기를 방지하기 위한 변수

// 시스템 상태 변수
bool telemetryEnabled = false;
bool simModeEnabled = false;
bool simEnableReceived = false;
bool simActivateReceived = false;
bool isCalibrated = false;
bool firstValidReadingDiscarded = false;
FlightState currentState = LAUNCH_PAD;
unsigned long lastSensorUpdate = 0;
const unsigned long SENSOR_INTERVAL = 20;
byte descendingCount = 0;
const byte DESCENT_CONFIRM_COUNT = 3;

// ★ 노이즈 방지용 연속 카운터 변수 추가
byte ascentCount = 0;
byte payloadReleaseCount = 0;
byte landedCount = 0;
const byte CONFIRM_COUNT = 5;         // ASCENT, LANDED 용 (100ms 유지)
const byte PAYLOAD_CONFIRM_COUNT = 3; // PAYLOAD_RELEASE 용 (60ms 유지)

// 미션 시간
byte missionTime[3] = {0, };

// 패킷 및 모드 변수
int packetCount = 0;
char flightMode = 'F';

// 센서 데이터 변수
float altitude = 0, temperature = 0, pressure = 0;
float batteryVoltage = 0;
float batteryCurrent = 0;
float gpsAltitude = 0, gpsLatitude = 0, gpsLongitude = 0;
byte gpsSatellites = 0;
float gyroR = 0, gyroP = 0, gyroY = 0;
float accelX = 0, accelY = 0, accelZ = 0;

// 명령어 변수
String cmdBuffer;
String cmdEcho = "";
String cmdParts[5];

// 기압 변수
float simPressure = 0;
float groundPressure = 0.0;
float currentPressure = 0.0;
float maxAltitude = 0;
float prevAltitude = 0;

// 타이밍 변수
unsigned long previousMillis = 0;

// 카메라 상태 변수
bool plRelsCamRecording = false;
bool groundCamRecording = false;

// 서보 상태 변수 (연속회전 SG90 — ON 시간만큼 역회전 후 정지)
bool servoActive = false;             // 현재 전진 회전 중인지
bool servoReversing = false;          // 역회전(복귀) 중인지
unsigned long servoOnStartMs = 0;     // ON 시작 시각
unsigned long servoReverseEndMs = 0;  // 역회전 종료 예정 시각

// 텔레메트리 문자열
String telemetryCSV;

// ==========================================
// 함수 원형 선언
// ==========================================
void configureBMP390();
void configureBNO085();
void configureINA260();
void updatePressureAndAltitude();
void readIMUData();
void readVoltageAndCurrent();
void readGPSData();
bool calibratePressure();
void resetParameters();
void parseCommand(String commandString);
void executeCXCommand();
void executeSTCommand();
void executeSIMCommand();
void executeSIMPCommand();
void executeMECCommand();
void executeTestCommand();
void executeCameraCommand();
void executeServoCommand();
String generateTelemetry();
void updateMissionTime();
bool writeTelemetryToSD();
void handleTelemetryTransmission();
void updateFlightState();
void processCommands();
bool initSensors();
bool updateSensorData();
float convertNMEAtoDecimalDegrees(float nmeaCoord);
void controlCamera(HardwareSerial& cam, bool& recordingFlag, const char* camName, bool start);
void readCameraAck(HardwareSerial& cam, const char* camName);
void startAllCameras();
void stopAllCameras();
void moveServos(bool active);
void updateServos();

// ---------------------------------------------------
// 센서 함수
// ---------------------------------------------------

bool initSensors() {
  bool success = true;

  Wire.begin();
  Wire.setClock(400000);
  delay(100);

  analogReadResolution(12);

  if (DEBUG) Serial.print("SD 카드 초기화 중...");
  if (!SD.begin(BUILTIN_SDCARD)) {
    if (DEBUG) Serial.println("SD 카드 초기화 실패!");
    success = false;
  } else {
    if (DEBUG) Serial.println("SD 카드 초기화 완료.");
  }
  delay(200);

  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  delay(200);

  if (DEBUG) Serial.println("BMP390 센서 초기화 중...");
  if (!bmp.begin_I2C(0x77)) {
    if (DEBUG) Serial.println("주소 0x77에서 실패, 0x76 시도 중...");
    if (!bmp.begin_I2C(0x76)) {
      if (DEBUG) Serial.println("유효한 BMP3 센서를 찾을 수 없습니다. 배선을 확인하세요!");
      success = false;
    } else {
      configureBMP390();
    }
  } else {
    configureBMP390();
  }
  delay(300);

  if (!bno08x.begin_I2C()) {
    if (DEBUG) Serial.println("유효한 BNO085 센서를 찾을 수 없습니다. 배선을 확인하세요!");
    success = false;
  } else {
    configureBNO085();
  }

  if (!ina260.begin()) {
    if (DEBUG) Serial.println("유효한 INA260 센서를 찾을 수 없습니다. 배선을 확인하세요!");
    success = false;
  } else {
    configureINA260();
  }

  delay(1000);
  return success;
}

void configureBMP390() {
  if (DEBUG) Serial.println("BMP390 초기화 성공!");
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

void configureBNO085() {
  if (DEBUG) Serial.println("BNO085 초기화 성공!");
  bno08x.enableReport(SH2_ACCELEROMETER);
  bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED);
  bno08x.enableReport(SH2_ROTATION_VECTOR);
}

void configureINA260() {
  if (DEBUG) Serial.println("INA260 초기화 성공!");
  ina260.setMode(INA260_MODE_CONTINUOUS);
}

bool updateSensorData() {
  if (!simModeEnabled) {
    if (!bmp.performReading()) {
      if (DEBUG) Serial.println("BMP390 읽기 실패!");
      return false;
    }
    if (!firstValidReadingDiscarded) {
      firstValidReadingDiscarded = true;
      if (DEBUG) Serial.println("안정화를 위해 첫 번째 BMP390 읽기 무시 중");
      return true;
    }
    
    // ★ 실제 측정 모드일 때만 온도를 업데이트 (SIM일 땐 0 고정 등 방지 위함)
    temperature = bmp.temperature;
  }

  updatePressureAndAltitude();
  readIMUData();
  readVoltageAndCurrent();
  readGPSData();

  return true;
}

void updatePressureAndAltitude() {
  if (simModeEnabled) {
    currentPressure = simPressure / 100.0;
  } else {
    currentPressure = bmp.pressure / 100.0;
  }

  pressure = currentPressure;

  if (isCalibrated && groundPressure > 0) {
    if (simModeEnabled) {
      altitude = 44330.0 * (1.0 - pow(currentPressure / groundPressure, 0.19));
    } else {
      altitude = bmp.readAltitude(groundPressure);
    }
  } else {
    if (simModeEnabled) {
      altitude = 44330.0 * (1.0 - pow(currentPressure / STANDARD_SEA_LEVEL_PRESSURE_HPA, 0.19));
    } else {
      altitude = bmp.readAltitude(STANDARD_SEA_LEVEL_PRESSURE_HPA);
    }
  }

  if (altitude < 0) {
    altitude = 0;
  }
}

void readIMUData() {
  // if를 while로 변경하여 버퍼에 남은 데이터를 모두 소진
  while (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_ACCELEROMETER:
        accelX = sensorValue.un.accelerometer.x;
        accelY = sensorValue.un.accelerometer.y;
        accelZ = sensorValue.un.accelerometer.z;
        break;
      case SH2_GYROSCOPE_CALIBRATED:
        gyroR = sensorValue.un.gyroscope.x * RAD_TO_DEG;
        gyroP = sensorValue.un.gyroscope.y * RAD_TO_DEG;
        gyroY = sensorValue.un.gyroscope.z * RAD_TO_DEG;
        break;
      case SH2_ROTATION_VECTOR: 
        float qr = sensorValue.un.rotationVector.real;
        float qi = sensorValue.un.rotationVector.i;
        float qj = sensorValue.un.rotationVector.j;
        float qk = sensorValue.un.rotationVector.k;
        
        headingYaw = atan2(2.0 * (qr * qk + qi * qj), 1.0 - 2.0 * (qj * qj + qk * qk)) * RAD_TO_DEG;
        if (headingYaw < 0) headingYaw += 360.0; 
        break;
    }
  }
}

void readVoltageAndCurrent() {
  batteryVoltage = ina260.readBusVoltage() / 1000.0;
  batteryCurrent = ina260.readCurrent() / 1000.0;
}

void readGPSData() {
  while (GPSSerial.available()) {
    GPS.read();
    if (GPS.newNMEAreceived()) {
      if (GPS.parse(GPS.lastNMEA())) {
        if (GPS.fix) {
          gpsLatitude = convertNMEAtoDecimalDegrees(GPS.latitude);
          if (GPS.lat == 'S') gpsLatitude = -gpsLatitude;
          gpsLongitude = convertNMEAtoDecimalDegrees(GPS.longitude);
          if (GPS.lon == 'W') gpsLongitude = -gpsLongitude;
          gpsAltitude = GPS.altitude;
          gpsSatellites = GPS.satellites;
        }
      }
    }
  }
}

float convertNMEAtoDecimalDegrees(float nmeaCoord) {
  int degrees = (int)(nmeaCoord / 100);
  float minutes = nmeaCoord - (degrees * 100);
  return degrees + (minutes / 60.0);
}

bool calibratePressure() {
  if (simModeEnabled) {
    groundPressure = simPressure / 100.0;
    isCalibrated = true;
    if (DEBUG) {
      Serial.print("시뮬레이션 보정 완료. 지면 기압: ");
      Serial.print(groundPressure, 1);
      Serial.println(" hPa");
    }
    return true;
  } else {
    if (bmp.performReading()) {
      groundPressure = bmp.pressure / 100.0;
      isCalibrated = true;
      if (DEBUG) {
        Serial.print("보정 완료. 지면 기압: ");
        Serial.print(groundPressure, 1);
        Serial.println(" hPa");
      }
      return true;
    } else {
      if (DEBUG) Serial.println("보정 실패: BMP 센서 읽기 오류.");
      return false;
    }
  }
}

// ---------------------------------------------------
// 텔레메트리 함수
// ---------------------------------------------------

String generateTelemetry() {
  char missionTimeBuf[12];
  sprintf(missionTimeBuf, "%02d:%02d:%02d", missionTime[0], missionTime[1], missionTime[2]);

  char gpsTimeBuf[12];
  sprintf(gpsTimeBuf, "%02d:%02d:%02d", GPS.hour, GPS.minute, GPS.seconds);

 String csvData =
    String(TEAM_ID) + ","
    + String(missionTimeBuf) + ","
    + String(packetCount) + ","
    + String(flightMode) + ","
    + STATE_STRINGS[currentState] + ","
    + String(altitude, 1) + ","
    + String(temperature, 2) + ","
    + String(pressure, 2) + ","
    + String(batteryVoltage, 2) + ","
    + String(batteryCurrent, 2) + ","
    + String(gyroR, 2) + ","
    + String(gyroP, 2) + ","
    + String(gyroY, 2) + ","
    + String(accelX, 2) + ","
    + String(accelY, 2) + ","
    + String(accelZ, 2) + ","
    + String(gpsTimeBuf) + ","
    + String(gpsAltitude, 1) + ","
    + String(gpsLatitude, 4) + ","
    + String(gpsLongitude, 4) + ","
    + String(gpsSatellites) + ","
    + cmdEcho;

  return csvData;
}

bool writeTelemetryToSD() {
  // 루프 돌 때마다 open/close 하지 않고 파일이 열려있으면 즉시 쓰기 및 flush
  if (packetFile) {
    packetFile.println(telemetryCSV);
    packetFile.flush(); 
    return true;
  } 
  if (DEBUG && currentState != LANDED) {
      Serial.println("SD 파일 기록 오류: 파일이 열려있지 않음");
    }
    return false;
}

void updateMissionTime() {
  missionTime[2]++;
  if (missionTime[2] >= 60) { missionTime[2] = 0; missionTime[1]++; }
  if (missionTime[1] >= 60) { missionTime[1] = 0; missionTime[0]++; }
  if (missionTime[0] >= 24) { missionTime[0] = 0; missionTime[1] = 0; missionTime[2] = 0; }
}

void handleTelemetryTransmission() {
  bool firstPacket = (packetCount == 0);
  if (!firstPacket && (millis() - previousMillis < 1000)) return;

  if (firstPacket) {
    previousMillis = millis();
  } else {
    previousMillis += 1000;
    updateMissionTime();
  }

  telemetryCSV = generateTelemetry();
  Serial.println(telemetryCSV);
  XBEE.println(telemetryCSV);
  writeTelemetryToSD();
  packetCount++;
}

// ---------------------------------------------------
// 명령어 처리 함수
// ---------------------------------------------------

void parseCommand(String commandString) {
  for (int i = 0; i < 5; i++) cmdParts[i] = "";

  int startIdx = 0;
  int tokenIndex = 0;

  while (tokenIndex < 5) {
    int commaIndex = commandString.indexOf(',', startIdx);
    if (commaIndex == -1) {
      cmdParts[tokenIndex] = commandString.substring(startIdx);
      break;
    }
    cmdParts[tokenIndex] = commandString.substring(startIdx, commaIndex);
    startIdx = commaIndex + 1;
    tokenIndex++;
  }

  for (int i = 0; i < 5; i++) {
    cmdParts[i].trim();
  }
}

void executeCXCommand() {
  if (cmdParts[3].equals("ON")) {
    telemetryEnabled = true;
    memset(missionTime, 0, sizeof(missionTime));
    packetCount = 0;
    previousMillis = millis();
    clearRecoveryData(); // ★ 새 비행 시작! 기존 EEPROM 리커버리 데이터 삭제
  } else if (cmdParts[3].equals("OFF")) {
    telemetryEnabled = false;
    isCalibrated = false;
    resetParameters();
  }
  cmdEcho = cmdParts[2] + cmdParts[3];
}

void executeSTCommand() {
  if (cmdParts[3].equals("GPS")) {
    missionTime[0] = GPS.hour;
    missionTime[1] = GPS.minute;
    missionTime[2] = GPS.seconds;
  } else {
    int colon1 = cmdParts[3].indexOf(':');
    int colon2 = cmdParts[3].lastIndexOf(':');
    if (colon1 != -1 && colon2 != -1 && colon1 != colon2) {
      missionTime[0] = (byte)cmdParts[3].substring(0, colon1).toInt();
      missionTime[1] = (byte)cmdParts[3].substring(colon1+1, colon2).toInt();
      missionTime[2] = (byte)cmdParts[3].substring(colon2+1).toInt();
    }
  }
  cmdEcho = cmdParts[2] + cmdParts[3];
}

void executeSIMCommand() {
  if (cmdParts[3].equalsIgnoreCase("ENABLE")) {
    simEnableReceived = true;
    isCalibrated = false;
  } else if (cmdParts[3].equalsIgnoreCase("ACTIVATE")) {
    simActivateReceived = true;
  } else if (cmdParts[3].equalsIgnoreCase("DISABLE")) {
    simEnableReceived = false;
    simActivateReceived = false;
    simModeEnabled = false;
    flightMode = 'F';
    isCalibrated = false;
  }
  cmdEcho = cmdParts[2] + cmdParts[3];
  if (simEnableReceived && simActivateReceived) {
    simModeEnabled = true;
    flightMode = 'S';
  }
}

void executeSIMPCommand() {
  if (simModeEnabled) {
    simPressure = cmdParts[3].toFloat();
  }
  cmdEcho = cmdParts[2] + cmdParts[3];
}

void executeMECCommand() {
  bool turnOn = cmdParts[4].equalsIgnoreCase("ON");

  if (cmdParts[3].equalsIgnoreCase("SERVO")) {
    moveServos(turnOn);
  } else if (cmdParts[3].equalsIgnoreCase("CAM_PL")) {
    controlCamera(PL_RELS_CAM, plRelsCamRecording, "PL_RELS_CAM", turnOn);
  } else if (cmdParts[3].equalsIgnoreCase("CAM_GND")) {
    controlCamera(GROUND_CAM, groundCamRecording, "GROUND_CAM", turnOn);
  } else {
    if (DEBUG) {
      Serial.print("알 수 없는 MEC device: ");
      Serial.println(cmdParts[3]);
    }
  }

  String onOff = turnOn ? "ON" : "OFF";
  cmdEcho = cmdParts[2] + "_" + cmdParts[3] + "_" + onOff;
}

/**
 * TEST 명령: 현재 모든 센서 값을 즉시 1회 읽어 패킷을 출력.
 * 정기 텔레메트리 송신과 무관하게 즉시 동작 — packetCount/미션시간 흐름은 건드리지 않는다.
 */
void executeTestCommand() {
  updateSensorData();
  cmdEcho = "TEST";
  telemetryCSV = generateTelemetry();
  Serial.println("=== TEST PACKET ===");
  Serial.println(telemetryCSV);
  XBEE.println(telemetryCSV);
  writeTelemetryToSD();
}

/**
 * CAMERA 명령: 두 카메라 동시 ON/OFF
 * 포맷: CMD,1062,CAMERA,ON|OFF  (cmdParts[3] = ON/OFF)
 */
void executeCameraCommand() {
  bool turnOn = cmdParts[3].equalsIgnoreCase("ON");
  if (turnOn) {
    startAllCameras();
  } else {
    stopAllCameras();
  }
  cmdEcho = "CAMERA_" + String(turnOn ? "ON" : "OFF");
}

/**
 * SERVO 명령: 두 서보 동작/원위치
 * 포맷: CMD,1062,SERVO,ON|OFF  (cmdParts[3] = ON/OFF)
 *   ON  → SERVO_ACTIVE_ANGLE
 *   OFF → SERVO_HOME_ANGLE
 */
void executeServoCommand() {
  // ★ 자율 조향 중일 때는 수동 서보 제어 명령 무시
  if (currentState == PAYLOAD_RELEASE || currentState == PROBE_RELEASE) {
    if (DEBUG) Serial.println("경고: 자율 조향 중이므로 수동 SERVO 명령을 무시합니다.");
    cmdEcho = "SERVO_IGNORED";
    return;
  }

  bool turnOn = cmdParts[3].equalsIgnoreCase("ON");
  moveServos(turnOn);
  cmdEcho = "SERVO_" + String(turnOn ? "ON" : "OFF");
}

// ---------------------------------------------------
// 서보 모터 제어 함수
// ---------------------------------------------------

// 목표 GPS 좌표까지의 방위각(Bearing)을 계산하는 함수
float calculateBearing(float lat1, float lon1, float lat2, float lon2) {
  float dLon = (lon2 - lon1) * DEG_TO_RAD;
  lat1 = lat1 * DEG_TO_RAD;
  lat2 = lat2 * DEG_TO_RAD;

  float y = sin(dLon) * cos(lat2);
  float x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
  float bearing = atan2(y, x) * RAD_TO_DEG;
  
  if (bearing < 0) bearing += 360.0;
  return bearing;
}

// 패러글라이더 펄스(짧게 끊어치는) 조향 제어 로직
void controlParaglider() {
  // 1. 고도 안전 장치 (2.3m 이하에서는 낙하 준비를 위해 조향 완전 중지)
  if (altitude <= 2.3) {
    servoLeft.writeMicroseconds(SERVO_STOP_US);
    servoRight.writeMicroseconds(SERVO_STOP_US);
    steerState = STEER_IDLE;
    return;
  }

  // 2. 비블로킹 타이머 기반 상태 머신 (툭 -> 툭 -> 대기)
  unsigned long currentMillis = millis();
  
  switch (steerState) {
    case STEER_IDLE:
      {
        // GPS 데이터가 정상(Fix)일 때만 계산
        if (gpsLatitude == 0 || gpsLongitude == 0) return;

        float targetBearing = calculateBearing(gpsLatitude, gpsLongitude, TARGET_LAT, TARGET_LON);
        float headingError = targetBearing - headingYaw;

        // 오차를 -180 ~ +180도로 정규화
        while (headingError <= -180.0) headingError += 360.0;
        while (headingError > 180.0) headingError -= 360.0;

        // 데드존 판단 (오차가 크면 턴 시작)
        if (headingError > DEADZONE_DEG) {
          // 목표가 오른쪽에 있음 -> 우측 턴 시작
          servoRight.writeMicroseconds(SERVO_FORWARD_US); // 당기기
          steerTimer = currentMillis;
          steerState = STEER_PULL_RIGHT;
          if(DEBUG) Serial.println("조향: 우측 턴 (당기기)");
        } 
        else if (headingError < -DEADZONE_DEG) {
          // 목표가 왼쪽에 있음 -> 좌측 턴 시작
          servoLeft.writeMicroseconds(SERVO_FORWARD_US); // 당기기
          steerTimer = currentMillis;
          steerState = STEER_PULL_LEFT;
          if(DEBUG) Serial.println("조향: 좌측 턴 (당기기)");
        }
      }
      break;

    // --- 좌측 서보 제어 사이클 ---
    case STEER_PULL_LEFT:
      if (currentMillis - steerTimer >= PULL_TIME_MS) {
        servoLeft.writeMicroseconds(SERVO_REVERSE_US); // 풀기
        steerTimer = currentMillis;
        steerState = STEER_RELEASE_LEFT;
      }
      break;
    case STEER_RELEASE_LEFT:
      if (currentMillis - steerTimer >= RELEASE_TIME_MS) {
        servoLeft.writeMicroseconds(SERVO_STOP_US); // 정지
        steerTimer = currentMillis;
        steerState = STEER_COOLDOWN;
      }
      break;

    // --- 우측 서보 제어 사이클 ---
    case STEER_PULL_RIGHT:
      if (currentMillis - steerTimer >= PULL_TIME_MS) {
        servoRight.writeMicroseconds(SERVO_REVERSE_US); // 풀기
        steerTimer = currentMillis;
        steerState = STEER_RELEASE_RIGHT;
      }
      break;
    case STEER_RELEASE_RIGHT:
      if (currentMillis - steerTimer >= RELEASE_TIME_MS) {
        servoRight.writeMicroseconds(SERVO_STOP_US); // 정지
        steerTimer = currentMillis;
        steerState = STEER_COOLDOWN;
      }
      break;

    // --- 쿨다운 (기체가 회전할 시간을 줌) ---
    case STEER_COOLDOWN:
      if (currentMillis - steerTimer >= COOLDOWN_TIME_MS) {
        steerState = STEER_IDLE; // 다시 각도 측정 시작
      }
      break;
  }
}

// ---------------------------------------------------
// 카메라 제어 함수
// ---------------------------------------------------

void controlCamera(HardwareSerial& cam, bool& recordingFlag, const char* camName, bool start) {
  // 테스트 스케치에서는 dedupe 가드 제거 — 명령 유실 시 재전송이 막히는 문제 방지.
  char cmd = start ? CAM_CMD_START : CAM_CMD_STOP;
  cam.write(cmd);
  cam.flush();   // TX 버퍼를 즉시 비워서 ESP32가 바로 받도록 함
  recordingFlag = start;
  if (DEBUG) {
    Serial.print(camName);
    Serial.print(" → '");
    Serial.print(cmd);
    Serial.println(start ? "' (녹화 시작 명령)" : "' (녹화 종료 명령)");
  }
}

void readCameraAck(HardwareSerial& cam, const char* camName) {
  while (cam.available()) {
    char ack = cam.read();
    if (DEBUG) {
      Serial.print(camName);
      Serial.print(" ACK: ");
      Serial.println(ack);
    }
  }
}

void startAllCameras() {
  controlCamera(PL_RELS_CAM, plRelsCamRecording, "PL_RELS_CAM", true);
  controlCamera(GROUND_CAM,  groundCamRecording, "GROUND_CAM",  true);
}

void stopAllCameras() {
  controlCamera(PL_RELS_CAM, plRelsCamRecording, "PL_RELS_CAM", false);
  controlCamera(GROUND_CAM,  groundCamRecording, "GROUND_CAM",  false);
}

// ---------------------------------------------------
// 서보 제어 함수
// ---------------------------------------------------

/**
 * 연속회전 SG90 두 개 동시 제어 — 비블로킹 방식.
 *
 *   active=true  → 전진 회전 시작 (정지 신호 보낼 때까지 계속 회전).
 *                  ON 시각을 기록하여 OFF 시점에 동일 시간 역회전으로 복귀시킨다.
 *   active=false → 직전 ON 지속시간만큼 역회전 예약 → updateServos()가 시각 도래 시 정지.
 *
 * 좌/우 서보가 거울대칭 회전이 필요하면 servoRight.write()의 인자를 (180 - speed)로 바꿀 것.
 */
void moveServos(bool active) {
  if (active) {
    // 이미 전진 중이면 무시 (재 ON 명령은 시작 시각을 리셋하지 않도록)
    if (servoActive) return;
    servoLeft.writeMicroseconds(SERVO_FORWARD_US);
    servoRight.writeMicroseconds(SERVO_FORWARD_US);
    servoOnStartMs = millis();
    servoActive = true;
    servoReversing = false;
    if (DEBUG) Serial.println("서보 ON — 전진 회전 시작");
  } else {
    if (servoActive) {
      unsigned long duration = millis() - servoOnStartMs;
      servoLeft.writeMicroseconds(SERVO_REVERSE_US);
      servoRight.writeMicroseconds(SERVO_REVERSE_US);
      servoReverseEndMs = millis() + duration;
      servoActive = false;
      servoReversing = true;
      if (DEBUG) {
        Serial.print("서보 OFF — 역회전 시작, ");
        Serial.print(duration);
        Serial.println("ms 후 정지");
      }
    } else {
      // ON 없이 OFF가 들어왔거나 이미 정지 상태 — 안전상 정지 신호 발송
      servoLeft.writeMicroseconds(SERVO_STOP_US);
      servoRight.writeMicroseconds(SERVO_STOP_US);
      servoReversing = false;
      if (DEBUG) Serial.println("서보 OFF — (이미 정지 상태) 정지 신호 재전송");
    }
  }
}

/**
 * 역회전 완료 시점이 도래했는지 매 루프에서 확인 — 도래 시 정지 신호 발송.
 * (long)(now - target) >= 0 으로 비교해야 millis() 래핑에 안전.
 */
void updateServos() {
  if (servoReversing && (long)(millis() - servoReverseEndMs) >= 0) {
    servoLeft.writeMicroseconds(SERVO_STOP_US);
    servoRight.writeMicroseconds(SERVO_STOP_US);
    servoReversing = false;
    if (DEBUG) Serial.println("서보 정지 — 원위치 복귀 완료");
  }
}

/**
 * Serial 또는 XBee에서 수신한 명령어 처리
 */
void processCommands() {
  String inputCmd = "";

  if (XBEE.available() > 0) {
    inputCmd = XBEE.readStringUntil('\n');
  }
  else if (Serial.available() > 0) {
    inputCmd = Serial.readStringUntil('\n');
  }

  if (inputCmd.length() > 0) {
    cmdBuffer = inputCmd;
    parseCommand(cmdBuffer);

    if (cmdParts[0].equals("CMD") && cmdParts[1].equals(String(TEAM_ID))) {
      if (cmdParts[2].equals("CX")) {
        executeCXCommand();
      }
      else if (cmdParts[2].equals("ST")) {
        executeSTCommand();
      }
      else if (cmdParts[2].equals("SIM")) {
        executeSIMCommand();
      }
      else if (cmdParts[2].equals("SIMP")) {
        executeSIMPCommand();
      }
      else if (cmdParts[2].equals("CAL")) {
        calibratePressure();
        cmdEcho = cmdParts[2];
      }
      else if (cmdParts[2].equals("MEC")) {
        executeMECCommand();
      }
      else if (cmdParts[2].equals("TEST")) {
        executeTestCommand();
      }
      else if (cmdParts[2].equals("CAMERA")) {
        executeCameraCommand();
      }
      else if (cmdParts[2].equals("SERVO")) {
        executeServoCommand();
      }

      if (DEBUG) {
        Serial.print("명령어 수신됨: ");
        Serial.println(cmdBuffer);
      }
    }
  }
}

// ---------------------------------------------------
// 비행 상태 관리
// ---------------------------------------------------

void updateFlightState() {
  if (!isCalibrated) return;

  FlightState oldState = currentState; // 이전 상태 기억

  switch (currentState) {
    case LAUNCH_PAD:
      if (altitude >= 10 && altitude > prevAltitude) {
        ascentCount++;
        if (ascentCount >= CONFIRM_COUNT) {
          currentState = ASCENT;
          if (DEBUG) Serial.println("상태 전환: ASCENT (상승)");
          startAllCameras();
        }
      } else {
        ascentCount = 0; // 조건 불만족 시 초기화
      }
      break;

    case ASCENT:
      if (altitude > maxAltitude) {
        maxAltitude = altitude;
        descendingCount = 0;
      } else if (altitude < prevAltitude) {
        descendingCount++;
        if (descendingCount >= DESCENT_CONFIRM_COUNT) {
          currentState = APOGEE;
          if (DEBUG) Serial.println("상태 전환: APOGEE (최고점)");
        }
      } else {
        descendingCount = 0;
      }
      break;

    case APOGEE:
      if (altitude < prevAltitude) {
        currentState = DESCENT;
        if (DEBUG) Serial.println("상태 전환: DESCENT (하강)");
      }
      break;

    case DESCENT:
      if (altitude <= maxAltitude * 0.80) {
        payloadReleaseCount++;
        if (payloadReleaseCount >= PAYLOAD_CONFIRM_COUNT) {
          currentState = PAYLOAD_RELEASE;
          if (DEBUG) Serial.println("상태 전환: PAYLOAD_RELEASE (페이로드 방출)");
        }
      } else {
        payloadReleaseCount = 0;
      }
      break;

    case PAYLOAD_RELEASE:
      if (altitude <= 2.3) {
        currentState = PROBE_RELEASE;
        if (DEBUG) Serial.println("상태 전환: PROBE_RELEASE (프로브 방출)");
      }
      break;

    case PROBE_RELEASE:
      if (altitude < 1) {
        landedCount++;
        if (landedCount >= CONFIRM_COUNT) {
          currentState = LANDED;
          if (DEBUG) Serial.println("상태 전환: LANDED (착륙)");
          stopAllCameras();
          if (packetFile) { // 임무 종료 시 안전하게 파일 닫기
            packetFile.flush();
            packetFile.close(); 
          }
        }
      } else {
        landedCount = 0;
      }
      break;

    case LANDED:
      break;
  }

  // ★ 리커버리 데이터 저장 로직 
  // 1. 비행 상태(State)가 넘어가는 결정적 순간
  if (currentState != oldState) {
    saveRecoveryData();
  } 
  // 2. ASCENT 상태에서 최고 고도가 10미터 이상 갱신될 때 간헐적 저장
  else if (currentState == ASCENT && (maxAltitude - lastSavedAltitude >= 10.0)) {
    saveRecoveryData();
    lastSavedAltitude = maxAltitude;
  }

  prevAltitude = altitude;
}

void resetParameters() {
  simEnableReceived = false;
  simActivateReceived = false;
  simModeEnabled = false;
  flightMode = 'F';
  memset(missionTime, 0, sizeof(missionTime));
  packetCount = 0;
  currentState = LAUNCH_PAD;
  cmdEcho = "";
  groundPressure = 0.0;
  isCalibrated = false;
  firstValidReadingDiscarded = false;
  maxAltitude = 0;
  prevAltitude = 0;
  descendingCount = 0;
  ascentCount = 0;
  payloadReleaseCount = 0;
  landedCount = 0;
}

// ---------------------------------------------------
// 리커버리(EEPROM) 제어 함수
// ---------------------------------------------------
void saveRecoveryData() {
  flightData.magicNumber = MAGIC_NUMBER;
  flightData.currentState = currentState;
  flightData.maxAltitude = maxAltitude;
  flightData.packetCount = packetCount;
  flightData.missionTime[0] = missionTime[0];
  flightData.missionTime[1] = missionTime[1];
  flightData.missionTime[2] = missionTime[2];
  flightData.groundPressure = groundPressure;
  
  EEPROM.put(EEPROM_ADDR, flightData);
  if (DEBUG) Serial.println("EEPROM: 비행 상태 저장 완료!");
}

void loadRecoveryData() {
  EEPROM.get(EEPROM_ADDR, flightData);
  
  if (flightData.magicNumber == MAGIC_NUMBER) {
    currentState = flightData.currentState;
    maxAltitude = flightData.maxAltitude;
    packetCount = flightData.packetCount;
    missionTime[0] = flightData.missionTime[0];
    missionTime[1] = flightData.missionTime[1];
    missionTime[2] = flightData.missionTime[2];
    lastSavedAltitude = maxAltitude;
    groundPressure = flightData.groundPressure;
    
    // 리커버리가 완료되면 기존의 지표면 기압(보정치) 대신 표준 기압으로 
    // 임시 계산되도록 isCalibrated를 강제 활성화 (공중에서 재부팅 시 막힘 방지)
    isCalibrated = true; 
    if (DEBUG) Serial.println("EEPROM: 공중 재부팅 감지! 기존 비행 데이터 복구 완료!");
  } else {
    if (DEBUG) Serial.println("EEPROM: 정상 초기 구동 (저장된 복구 데이터 없음)");
  }
}

void clearRecoveryData() {
  flightData.magicNumber = 0x0000; // 암호를 지워서 초기화 상태로 만듦
  EEPROM.put(EEPROM_ADDR, flightData);
  if (DEBUG) Serial.println("EEPROM: 리커버리 데이터 초기화 완료");
}

// ---------------------------------------------------
// 아두이노 메인 함수
// ---------------------------------------------------

void setup() {
  Serial.begin(115200);
  XBEE.begin(115200);

  PL_RELS_CAM.begin(CAM_BAUD);
  GROUND_CAM.begin(CAM_BAUD);

  // 서보 초기화 — 연속회전 서보는 90이 정지. 깨끗하게 멈춘 상태로 시작.
  servoLeft.attach(SERVO_LEFT_PIN);
  servoRight.attach(SERVO_RIGHT_PIN);
  servoLeft.writeMicroseconds(SERVO_STOP_US);
  servoRight.writeMicroseconds(SERVO_STOP_US);

  initSensors();
  // ★ 센서 초기화 직후(SD begin 완료 후) 파일을 한 번만 열어둠
  packetFile = SD.open(CSV_FILENAME, FILE_WRITE);
  loadRecoveryData();

  if (DEBUG) Serial.println("Test_FSW 시작 — CMD,1062,TEST/CAMERA,ON/OFF/SERVO,ON/OFF 사용 가능");
}

void loop() {
  processCommands();

  // 연속회전 서보 역회전 완료 시점 체크
  updateServos();

  readCameraAck(GROUND_CAM, "GROUND_CAM");

  unsigned long currentMillis = millis();
  if (currentMillis - lastSensorUpdate >= SENSOR_INTERVAL) {
    lastSensorUpdate = currentMillis;
    updateSensorData();
    if (isCalibrated) {
      updateFlightState();
    }
    
  }

  if (currentState == PAYLOAD_RELEASE || currentState == PROBE_RELEASE) {
      controlParaglider();
    }

  if (telemetryEnabled) {
    handleTelemetryTransmission();
  }
}
