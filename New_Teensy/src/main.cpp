/**
 * CanSat_3167_FSW
 * 
 * 이 코드는 Teensy 4.1과 여러 센서를 사용하는 텔레메트리 시스템을 구현합니다:
 * - BMP390: 기압 및 온도 센서
 * - BNO085: IMU (가속도계, 자이로스코프, 자력계)
 * - Adafruit Ultimate GPS Breakout v3: 위치 데이터
 * 
 * 데이터는 XBee를 통해 전송되고 SD 카드에도 기록됩니다.
 */

#include <Arduino.h>
#include <Adafruit_GPS.h>
#include <Adafruit_BNO08x.h>
#include "Adafruit_BMP3XX.h"
#include <SPI.h>
#include <SD.h>
#include "Wire.h"

// ---------------------------------------------------
// 설정
// ---------------------------------------------------

// 상수
#define TEAM_ID 1062
#define GPSSerial Serial3
#define XBEE Serial2
#define DEBUG true
#define CSV_FILENAME "Flight_1062.csv"
#define STANDARD_SEA_LEVEL_PRESSURE_HPA 1013.25

// 핀 정의
const int BATTERY_VOLTAGE_PIN = A2;  // 배터리 전압용 ADC 입력 핀 (16Pin)

// 전압 분배기 구성
const float REFERENCE_VOLTAGE = 3.3;     // Teensy 4.1 기준 전압 (3.3V)
const int ADC_RESOLUTION = 1023;         // 10비트 ADC 최대값
const float R1 = 10.0;                   // R1 저항 (10kΩ)
const float R2 = 36.0;                   // R2 저항 (36kΩ)

// 물리 상수
//const float RAD_TO_DEG = 180.0 / 3.14159265358;  // 라디안에서 도(°)로 변환

// ---------------------------------------------------
// 전역 객체
// ---------------------------------------------------

// 센서 객체
Adafruit_GPS GPS(&GPSSerial);       // GPS 객체
Adafruit_BNO08x bno08x;             // BNO085 객체
Adafruit_BMP3XX bmp;                // BMP388 객체
sh2_SensorValue_t sensorValue;      // BNO085 센서 데이터 객체

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
  PROBE_RELEASE,     // 프로브 방출
  LANDED             // 착륙
};

// 비행 상태 문자열 (열거형 순서와 일치해야 함)
const String STATE_STRINGS[] = {
  "LAUNCH_PAD", "ASCENT", "APOGEE", "DESCENT", "PROBE_RELEASE", "LANDED"
};

// 시스템 상태 변수
bool telemetryEnabled = false;           // 텔레메트리 전송 상태 (CX ON/OFF)
bool simModeEnabled = false;             // 시뮬레이션 모드 상태
bool simEnableReceived = false;          // SIM "ENABLE" 명령 플래그
bool simActivateReceived = false;        // SIM "ACTIVATE" 명령 플래그
bool isCalibrated = false;               // CAL 명령 수신 플래그
bool isLaunched = false;                 // 발사 상태
bool firstValidReadingDiscarded = false; // BMP388 안정화를 위한 플래그
FlightState currentState = LAUNCH_PAD;   // 현재 비행 상태

// 미션 시간 (HH, MM, SS)
byte missionTime[3] = {0, };     // 미션 시간

// 패킷 및 모드 변수
int packetCount = 0;             // 전송된 패킷 수
char flightMode = 'F';           // 운용 모드 ('F' = 비행, 'S' = 시뮬레이션)

// 센서 데이터 변수
float altitude = 0, temperature = 0, pressure = 0;   // BMP388: 고도(m), 온도(°C), 기압(kPa)
float batteryVoltage = 0;                            // 배터리 전압(V)
float gpsAltitude = 0, gpsLatitude = 0, gpsLongitude = 0; // GPS 데이터
byte gpsSatellites = 0;                              // GPS 위성 수
float gyroR = 0, gyroP = 0, gyroY = 0;              // 자이로스코프(BNO085) - 도/초
float accelX = 0, accelY = 0, accelZ = 0;           // 가속도계(BNO085) - m/s²
float magR = 0, magP = 0, magY = 0;                 // 자력계(BNO085) - 가우스
float propellerRate = 0;                             // 프로펠러 회전 속도

// 명령어 변수
String cmdBuffer;                  // 수신된 명령어 버퍼
String cmdEcho = "";               // 처리된 명령어 에코
String cmdParts[4];                // 쉼표로 분할된 명령어 부분

// 기압 변수
float simPressure = 0;             // 시뮬레이션 모드 기압(hPa)
float groundPressure = 0.0;        // 기준 지표 기압(hPa)
float currentPressure = 0.0;       // 현재 기압(hPa)
float maxAltitude = 0;             // 최대 기록 고도
float prevAltitude = 0;            // 이전 고도 측정값

// 타이밍 변수
unsigned long previousMillis = 0;  // 시간 측정용

// 텔레메트리 문자열
String telemetryCSV;               // CSV 형식 텔레메트리 데이터 문자열

// ---------------------------------------------------
// 센서 함수
// ---------------------------------------------------

/**
 * 모든 센서 초기화: SD 카드, GPS, BNO08x, BMP388
 * @return 초기화 성공 시 true, 실패 시 false
 */
bool initSensors() {
  bool success = true;
  
  // Wire(I2C) 초기화
  Wire.begin();
  Wire.setClock(400000);  // I2C 클럭 속도를 400kHz로 설정
  delay(100);  // I2C 초기화 후 잠시 대기
  
  // ADC 해상도 설정
  analogReadResolution(12);
  
  // SD 카드 초기화
  if (DEBUG) Serial.print("SD 카드 초기화 중...");
  if (!SD.begin(BUILTIN_SDCARD)) { 
    if (DEBUG) Serial.println("SD 카드 초기화 실패!");
    success = false;
  } else {
    if (DEBUG) Serial.println("SD 카드 초기화 완료.");
  }
  delay(200);
  
  // GPS 초기화
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  delay(200);
  
  // BMP388(기압 및 온도 센서) 초기화
  if (DEBUG) Serial.println("BMP390 센서 초기화 중...");
  if (!bmp.begin_I2C(0x77)) {
    if (DEBUG) Serial.println("주소 0x77에서 실패, 0x76 시도 중...");
    if (!bmp.begin_I2C(0x76)) {
      if (DEBUG) Serial.println("유효한 BMP3 센서를 찾을 수 없습니다. 배선을 확인하세요!");
      success = false;
    } else {
      configureBMP388();
    }
  } else {
    configureBMP388();
  }
  delay(300);
  
  // BNO08x(IMU 센서) 초기화
  if (!bno08x.begin_I2C()) {
    if (DEBUG) Serial.println("유효한 BNO085 센서를 찾을 수 없습니다. 배선을 확인하세요!");
    success = false;
  } else {
    configureBNO085();
  }
  
  delay(1000);  // 센서 안정화 대기
  return success;
}

/**
 * BMP388 센서 설정 구성
 */
void configureBMP388() {
  if (DEBUG) Serial.println("BMP390 초기화 성공!");
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

/** 
 * BNO085 센서 설정 구성
 */
void configureBNO085() {
  if (DEBUG) Serial.println("BNO085 초기화 성공!");
  bno08x.enableReport(SH2_ACCELEROMETER);
  bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED);
  bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED);
  bno08x.enableReport(SH2_ROTATION_VECTOR);
}

/**
 * 모든 센서에서 데이터를 읽고 업데이트
 * @return 성공 시 true, 오류 시 false
 */
bool updateSensorData() {
  // 배터리 전압 읽기
  int adcValue = analogRead(BATTERY_VOLTAGE_PIN);
  float measuredVoltage = (adcValue / float(ADC_RESOLUTION)) * REFERENCE_VOLTAGE;
  batteryVoltage = measuredVoltage * ((R1 + R2) / R2);
  
  // 일반 모드에서는 실제 센서에서 읽기
  if (!simModeEnabled) {
    // BMP390 읽기
    if (!bmp.performReading()) {
      if (DEBUG) Serial.println("BMP390 읽기 실패!");
      return false;
    }
    
    // 안정화를 위해 첫 번째 읽기 무시
    if (!firstValidReadingDiscarded) {
      firstValidReadingDiscarded = true;
      if (DEBUG) Serial.println("안정화를 위해 첫 번째 BMP388 읽기 무시 중");
      delay(1000);
      return true;  // 다음 주기에 다시 읽을 것이므로 일찍 반환
    } 
  }
  
  // 센서에서 온도 업데이트
  temperature = bmp.temperature;  // °C
  
  // 모드에 따라 기압 업데이트
  updatePressureAndAltitude();
  
  // BNO085 IMU 데이터 읽기
  readIMUData();
  
  // GPS 데이터 읽기
  readGPSData();
  
  return true;
}

/**
 * 현재 모드에 따라 기압 및 고도 값 업데이트
 */
void updatePressureAndAltitude() {
  // 모드에 따라 현재 기압 가져오기
  if (simModeEnabled) {
    currentPressure = simPressure / 100.0;  // hPa
  } else {
    currentPressure = bmp.pressure / 100.0; // Pa -> hPa
  }
  
  // 저장을 위해 kPa로 변환
  pressure = currentPressure / 10.0;  // hPa -> kPa
  
  // 기압에 따른 고도 계산
  if (isCalibrated && groundPressure > 0) {
    // 가능한 경우 보정된 지표 기압 사용
    if (simModeEnabled) {
      // 국제 기압 공식 사용하여 계산
      altitude = 44330.0 * (1.0 - pow(currentPressure / groundPressure, 0.19));
    } else {
      altitude = bmp.readAltitude(groundPressure);
    }
  } else {
    // 보정되지 않은 경우 표준 해수면 기압 사용
    if (simModeEnabled) {
      altitude = 44330.0 * (1.0 - pow(currentPressure / STANDARD_SEA_LEVEL_PRESSURE_HPA, 0.19));
    } else {
      altitude = bmp.readAltitude(STANDARD_SEA_LEVEL_PRESSURE_HPA);
    }
  }
  
  // 고도가 음수가 되지 않도록 함
  if (altitude < 0) {
    altitude = 0;
  }
}

/**
 * BNO085 IMU 센서에서 데이터 읽기
 */
void readIMUData() {
  if (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_ACCELEROMETER:
        accelX = sensorValue.un.accelerometer.x;
        accelY = sensorValue.un.accelerometer.y;
        accelZ = sensorValue.un.accelerometer.z;
        break;
      case SH2_GYROSCOPE_CALIBRATED:
        // rad/s에서 도/초로 변환
        gyroR = sensorValue.un.gyroscope.x * RAD_TO_DEG;
        gyroP = sensorValue.un.gyroscope.y * RAD_TO_DEG;
        gyroY = sensorValue.un.gyroscope.z * RAD_TO_DEG;
        break;
      case SH2_MAGNETIC_FIELD_CALIBRATED:
        // μT에서 가우스로 변환 (1 μT = 0.01 가우스)
        magR = sensorValue.un.magneticField.x * 0.01;
        magP = sensorValue.un.magneticField.y * 0.01;
        magY = sensorValue.un.magneticField.z * 0.01;
        break;
    }
  }
}

/**
 * GPS 데이터 읽기 및 처리
 */
void readGPSData() {
  bool newData = false;
  
  // 최대 200ms 동안 GPS 데이터 시도
  unsigned long start = millis();
  while (millis() - start < 200) {  
    if (GPSSerial.available()) {
      char c = GPS.read();
      
      // 새 NMEA 문장 확인
      if (GPS.newNMEAreceived()) {
        if (GPS.parse(GPS.lastNMEA())) {
          newData = true;
          break;  // 유효한 데이터 수신, 루프 종료
        }
      }
    }
  }
}

float convertNMEAtoDecimalDegrees(float nmeaCoord) {
  // NMEA 형식: DDMM.MMMM 또는 DDDMM.MMMM
  // DD(D) = 도, MM.MMMM = 분
  
  int degrees = (int)(nmeaCoord / 100);  // 도 추출
  float minutes = nmeaCoord - (degrees * 100);  // 분 추출
  
  // Decimal Degrees = 도 + (분/60)
  return degrees + (minutes / 60.0);
}

/**
 * 현재 측정값이나 시뮬레이션 값을 사용하여 지면 기압 보정
 * @return 보정 성공 시 true, 실패 시 false
 */
bool calibratePressure() {
  isCalibrated = true;
  if (simModeEnabled) {
    groundPressure = simPressure / 100.0;
    if (DEBUG) {
      Serial.print("시뮬레이션 보정 완료. 지면 기압: ");
      Serial.print(groundPressure, 1);
      Serial.println(" hPa");
    }
    return true;
  } else {
    if (bmp.performReading()) {
      groundPressure = bmp.pressure / 100.0;
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

/**
 * 모든 센서 데이터가 포함된 CSV 형식 텔레메트리 문자열 생성
 * @return CSV 형식의 텔레메트리 문자열
 */
String generateTelemetry() {
  char missionTimeBuf[12];
  sprintf(missionTimeBuf, "%02d:%02d:%02d", missionTime[0], missionTime[1], missionTime[2]);

  char gpsTimeBuf[12];
  sprintf(gpsTimeBuf, "%02d:%02d:%02d", GPS.hour, GPS.minute, GPS.seconds);
  
 String csvData = String(TEAM_ID) + ","                 // 팀 ID
    + String(missionTimeBuf) + ","                       // 미션 시간(HH:MM:SS)
    + String(packetCount) + ","                          // 패킷 번호
    + String(flightMode) + ","                           // 모드(F 또는 S)
    + STATE_STRINGS[currentState] + ","                  // 비행 상태
    + String(altitude, 1) + ","                          // 고도(m)
    + String(temperature, 2) + ","                       // 온도(°C)
    + String(pressure, 2) + ","                          // 기압(kPa)
    + String(batteryVoltage, 2) + ","                    // 배터리 전압(V)
    + String(gyroR, 2) + ","                             // 자이로 롤(도/초)
    + String(gyroP, 2) + ","                             // 자이로 피치(도/초)
    + String(gyroY, 2) + ","                             // 자이로 요(도/초)
    + String(accelX, 2) + ","                            // 가속도 X(m/s²)
    + String(accelY, 2) + ","                            // 가속도 Y(m/s²)
    + String(accelZ, 2) + ","                            // 가속도 Z(m/s²)
    + String(magR, 2) + ","                              // 자기장 롤(가우스)
    + String(magP, 2) + ","                              // 자기장 피치(가우스)
    + String(magY, 2) + ","                              // 자기장 요(가우스)
    + String(propellerRate, 2) + ","                     // 프로펠러 회전 속도
    + String(gpsTimeBuf) + ","                           // GPS 시간
    + String(GPS.altitude, 1) + ","                      // GPS 고도 (0.1m resolution)
    + String(convertNMEAtoDecimalDegrees(GPS.latitude), 4) + ","   // GPS 위도 (변환됨)
    + String(convertNMEAtoDecimalDegrees(GPS.longitude), 4) + ","  // GPS 경도 (변환됨)
    + String(GPS.satellites) + ","                       // 위성 수
    + cmdEcho;                                           // 명령어 에코
      
  return csvData;
}

/**
 * 현재 텔레메트리 데이터를 SD 카드에 기록
 * @return 성공 시 true, 실패 시 false
 */
bool writeTelemetryToSD() {
  packetFile = SD.open(CSV_FILENAME, FILE_WRITE);
  if (packetFile) {
    packetFile.println(telemetryCSV);
    packetFile.close();
    return true;
  } else {
    if (DEBUG) {
      Serial.print("파일 열기 오류: ");
      Serial.println(CSV_FILENAME);
    }
    return false;
  }
}

/**
 * 미션 시간을 1초 증가시키고 자리올림 처리
 */
void updateMissionTime() {
  missionTime[2]++;  // 초 증가
  
  // 자리올림 처리
  if (missionTime[2] >= 60) {
    missionTime[2] = 0;
    missionTime[1]++;
  }
  if (missionTime[1] >= 60) {
    missionTime[1] = 0;
    missionTime[0]++;
  }
  if (missionTime[0] >= 24) {
    missionTime[0] = 0;
    missionTime[1] = 0;
    missionTime[2] = 0;
  }
}

/**
 * 정기적인 간격으로 텔레메트리 전송 처리
 */
void handleTelemetryTransmission() {
  // 1초가 지났는지 확인
  if (millis() - previousMillis >= 1000) {
    previousMillis = millis();
    updateMissionTime();

    // 활성화된 경우 텔레메트리 생성 및 전송
    if (telemetryEnabled) {
      telemetryCSV = generateTelemetry();
      
      if (packetCount > 0){
        // 시리얼 및 XBee로 전송
        Serial.println(telemetryCSV);
        XBEE.println(telemetryCSV);
      }
      
      packetCount++;
    }
  }
}

// ---------------------------------------------------
// 명령어 처리 함수
// ---------------------------------------------------

/**
 * 수신된 명령어 문자열을 명령어 부분으로 처리
 * @param commandString 처리할 명령어 문자열
 */
void parseCommand(String commandString) {
  // 쉼표를 기준으로 명령어를 부분으로 분할
  int startIdx = 0;
  int tokenIndex = 0;
  
  while (tokenIndex < 4) {
    int commaIndex = commandString.indexOf(',', startIdx);
    if (commaIndex == -1) {
      cmdParts[tokenIndex] = commandString.substring(startIdx);
      break;
    }
    cmdParts[tokenIndex] = commandString.substring(startIdx, commaIndex);
    startIdx = commaIndex + 1;
    tokenIndex++;
  }
  
  // 부분에서 공백 제거
  for (int i = 0; i < 4; i++) {
    cmdParts[i].trim();
  }
}

/**
 * CX 명령어 실행 (텔레메트리 제어)
 */
void executeCXCommand() {
  if (cmdParts[3].equals("ON")) {
    telemetryEnabled = true;
    initSensors();  // 텔레메트리가 활성화되면 센서 초기화
  } else if (cmdParts[3].equals("OFF")) {
    telemetryEnabled = false;
    isCalibrated = false;
    resetParameters();  // 텔레메트리가 비활성화되면 매개변수 재설정
  }
  cmdEcho = cmdParts[2] + cmdParts[3];
}

/**
 * ST 명령어 실행 (시간 설정)
 */
void executeSTCommand() {
  if (cmdParts[3].equals("GPS")) {
    // GPS에서 시간 설정
    missionTime[0] = GPS.hour;
    missionTime[1] = GPS.minute;
    missionTime[2] = GPS.seconds;
  } else {
    // 문자열에서 시간 파싱 (HH:MM:SS)
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

/**
 * SIM 명령어 실행 (시뮬레이션 모드 제어)
 */
void executeSIMCommand() {
  if (cmdParts[3].equalsIgnoreCase("ENABLE")) {
    if (DEBUG) Serial.println("시뮬레이션 모드: ENABLE 명령 수신됨");
    simEnableReceived = true;
    isCalibrated = false;  // 시뮬레이션 모드를 위해 보정 재설정
  } else if (cmdParts[3].equalsIgnoreCase("ACTIVATE")) {
    if (DEBUG) Serial.println("시뮬레이션 모드: ACTIVATE 명령 수신됨");
    simActivateReceived = true;
  } else if (cmdParts[3].equalsIgnoreCase("DISABLE")) {
    if (DEBUG) Serial.println("시뮬레이션 모드: DISABLE 명령 수신됨");
    simEnableReceived = false;
    simActivateReceived = false;
    simModeEnabled = false;
    flightMode = 'F';
    isCalibrated = false;
  }
  
  cmdEcho = cmdParts[2] + cmdParts[3];
  
  // ENABLE과 ACTIVATE가 모두 수신되면 시뮬레이션 모드 활성화
  if (simEnableReceived && simActivateReceived) {
    simModeEnabled = true;
    flightMode = 'S';
    if (DEBUG) Serial.println("시뮬레이션 모드 완전히 활성화됨");
  }
}

/**
 * SIMP 명령어 실행 (시뮬레이션 기압 설정)
 */
void executeSIMPCommand() {
  if (simModeEnabled) {
    simPressure = cmdParts[3].toFloat();
    if (DEBUG) {
      Serial.print("시뮬레이션 모드 기압 설정: ");
      Serial.println(simPressure);
    }
  }
}

/**
 * MEC 명령어 실행 (기계장치 제어)
 */
void executeMECCommand() {
  if (cmdParts[3].equals("ON")) {
    if (DEBUG) Serial.println("서보 모터 활성화!");
    
  } else {
    if (DEBUG) Serial.println("서보 모터 비활성화!");
    // 여기에 서보 모터 비활성화 코드 추가
  }
  cmdEcho = cmdParts[2] + cmdParts[3];
}

/**
 * Serial 또는 XBee에서 수신한 명령어 처리
 */
void processCommands() {
  String inputCmd = "";
  
  // XBee 또는 Serial에서 명령어 확인
  if (XBEE.available() > 0) {
    inputCmd = XBEE.readStringUntil('\n');
  }
  else if (Serial.available() > 0) {
    inputCmd = Serial.readStringUntil('\n');
  }
  
  // 명령어가 수신되고 발사되지 않은 경우 처리
  if (inputCmd.length() > 0 && !isLaunched) {
    cmdBuffer = inputCmd;
    parseCommand(cmdBuffer);
    
    // 명령어 유형에 따라 처리
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
        cmdEcho = cmdParts[2];
      } 
      else if (cmdParts[2].equals("CAL")) {
        calibratePressure();
        cmdEcho = cmdParts[2];
      } 
      else if (cmdParts[2].equals("MEC")) {
        executeMECCommand();
      }
      
      // 수신된 명령어 기록
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

/**
 * 현재 고도와 궤적에 따라 비행 상태 업데이트
 */
void updateFlightState() {
  // 보정된 경우에만 상태 업데이트
  if (!isCalibrated) return;
  
  switch (currentState) {
    case LAUNCH_PAD:
      // 10m 이상이고 상승 중이면 ASCENT로 전환
      if (altitude >= 10 && altitude > prevAltitude) {
        currentState = ASCENT;
        if (DEBUG) Serial.println("상태 전환: ASCENT (상승)");
      }
      break;

    case ASCENT:
      // 최대 고도 추적
      if (altitude > maxAltitude) {
        maxAltitude = altitude;
      }
      // 더 이상 상승하지 않으면 APOGEE로 전환
      else if (altitude < prevAltitude) {
        currentState = APOGEE;
        if (DEBUG) Serial.println("상태 전환: APOGEE (최고점)");
      }
      break;

    case APOGEE:
      // 하강 중이면 DESCENT로 전환
      if (altitude < prevAltitude) {
        currentState = DESCENT;
        if (DEBUG) Serial.println("상태 전환: DESCENT (하강)");
      }
      break;

    case DESCENT:
      // 최대 고도의 75%에서 PROBE_RELEASE로 전환
      if (altitude <= maxAltitude * 0.75) {
        currentState = PROBE_RELEASE;
        if (DEBUG) Serial.println("상태 전환: PROBE_RELEASE (프로브 방출)");
        // 여기에 오토-자이로 시스템 배치 코드 추가
      }
      break;

    case PROBE_RELEASE:
      // 10m 미만일 때 LANDED로 전환
      if (altitude < 10) {
        currentState = LANDED;
        if (DEBUG) Serial.println("상태 전환: LANDED (착륙)");
      }
      break;

    case LANDED:
      // 여기에 착륙 상태 코드 추가
      break;
  }
  
  // 다음 비교를 위해 이전 고도 업데이트
  prevAltitude = altitude;
}

/**
 * 모든 매개변수를 기본값으로 재설정
 */
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
}

// ---------------------------------------------------
// 아두이노 메인 함수
// ---------------------------------------------------

void setup() {
  // 시리얼 통신 초기화
  Serial.begin(9600);
  XBEE.begin(9600);
  
  if (DEBUG) Serial.println("비행 텔레메트리 시스템 시작");
}

void loop() {
  // 수신된 명령어 처리
  processCommands();
  
  // 텔레메트리가 활성화된 경우에만 다른 작업 진행
  if (telemetryEnabled) {
    // 센서 데이터 업데이트
    updateSensorData();
    
    // 첫번째 값을 무시한 상태일 때 패킷 송신
    if (firstValidReadingDiscarded) {
      // 텔레메트리 전송 처리 (1초 간격)
      handleTelemetryTransmission();
    }
    
    
    // 보정된 경우 비행 상태 업데이트
    if (isCalibrated) {
      updateFlightState();
    }
    
    // SD 카드에 데이터 기록
    writeTelemetryToSD();
  }
}