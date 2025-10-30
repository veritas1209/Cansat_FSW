#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_GPS.h>

// BMP390 (I2C - Wire, pins 18/19)
Adafruit_BMP3XX bmp;

// BNO085 (I2C - Wire1, pins 17/16)
Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;
#define BNO085_INT 15

// GPS (Serial1, pins 0/1)
#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);

void setup() {
  Serial.begin(115200);
  delay(2000); // 2초 대기로 변경
  
  Serial.println("=== Teensy 4.1 CanSat Sensor Test ===");
  Serial.println();

  // BMP390 초기화 (Wire 사용 - 핀 18/19)
  Serial.print("BMP390 초기화 중 (SDA=18, SCL=19)... ");
  Wire.begin();
  Wire.setClock(400000); // 400kHz
  
  if (bmp.begin_I2C(0x77, &Wire)) {
    Serial.println("성공!");
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  } else {
    Serial.println("실패! 연결 확인 필요");
  }

  // BNO085 초기화 (Wire1 사용 - 핀 17/16)
  Serial.println("BNO085 초기화 시작...");
  Wire1.begin();
  Wire1.setClock(100000); // 100kHz로 낮춤 (안정성)
  pinMode(BNO085_INT, INPUT);
  delay(500); // 대기 시간 증가
  
  // I2C 스캔으로 주소 확인
  Serial.print("I2C Wire1 스캔 중... ");
  Wire1.beginTransmission(0x4A);
  byte error1 = Wire1.endTransmission();
  Wire1.beginTransmission(0x4B);
  byte error2 = Wire1.endTransmission();
  Serial.print("0x4A: ");
  Serial.print(error1 == 0 ? "발견!" : "없음");
  Serial.print(" (오류코드: ");
  Serial.print(error1);
  Serial.print("), 0x4B: ");
  Serial.print(error2 == 0 ? "발견!" : "없음");
  Serial.print(" (오류코드: ");
  Serial.print(error2);
  Serial.println(")");
  
  delay(100);
  Serial.print("BNO085 연결 시도 (0x4A)... ");
  
  // RST 핀이 있다면 리셋 (보통 BNO085는 자동 리셋)
  // 여러 번 시도
  bool bno_success = false;
  for (int attempt = 0; attempt < 3; attempt++) {
    if (attempt > 0) {
      Serial.print("재시도 ");
      Serial.print(attempt);
      Serial.print("... ");
      delay(500);
    }
    
    if (bno08x.begin_I2C(0x4A, &Wire1, BNO085_INT)) {
      Serial.println("성공!");
      bno_success = true;
      delay(100);
      
      // 센서 리포트 활성화
      Serial.println("센서 리포트 활성화 중...");
      if (bno08x.enableReport(SH2_ACCELEROMETER, 50000)) {
        Serial.println("  - 가속도계 활성화 성공");
      }
      if (bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 50000)) {
        Serial.println("  - 자이로 활성화 성공");
      }
      if (bno08x.enableReport(SH2_ROTATION_VECTOR, 50000)) {
        Serial.println("  - 회전벡터 활성화 성공");
      }
      break;
    }
  }
  
  if (!bno_success) {
    Serial.println("실패! RST 핀 확인 또는 센서 재부팅 필요");
  }

  // GPS 초기화
  Serial.print("GPS 초기화 중 (TX=0, RX=1)... ");
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  Serial.println("완료!");

  Serial.println("\n=== 센서 데이터 읽기 시작 ===\n");
  delay(1000);
}

void loop() {
  static unsigned long lastPrint = 0;
  
  // GPS 데이터 읽기 (항상 수행)
  char c = GPS.read();
  if (GPS.newNMEAreceived()) {
    GPS.parse(GPS.lastNMEA());
  }
  
  // BNO085 데이터 계속 읽기 (빠른 폴링)
  static float last_accel_x = 0, last_accel_y = 0, last_accel_z = 0;
  static float last_quat_i = 0, last_quat_j = 0, last_quat_k = 0;
  static bool has_accel = false, has_quat = false;
  static unsigned long last_reset_check = 0;
  
  // 리셋 체크는 1초에 한 번만
  if (millis() - last_reset_check > 1000) {
    last_reset_check = millis();
    if (bno08x.wasReset()) {
      Serial.println("BNO085 - 리셋 감지됨, 재활성화 중...");
      bno08x.enableReport(SH2_ACCELEROMETER, 50000);
      bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 50000);
      bno08x.enableReport(SH2_ROTATION_VECTOR, 50000);
      delay(100);
    }
  }
  
  // 센서 이벤트 읽기 (여러 개 읽을 수 있음)
  while (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_ACCELEROMETER:
        last_accel_x = sensorValue.un.accelerometer.x;
        last_accel_y = sensorValue.un.accelerometer.y;
        last_accel_z = sensorValue.un.accelerometer.z;
        has_accel = true;
        break;
      case SH2_ROTATION_VECTOR:
        last_quat_i = sensorValue.un.rotationVector.i;
        last_quat_j = sensorValue.un.rotationVector.j;
        last_quat_k = sensorValue.un.rotationVector.k;
        has_quat = true;
        break;
    }
  }

  // 1초마다 출력
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    
    Serial.println("--- 센서 데이터 ---");
    
    // BMP390 데이터
    if (bmp.performReading()) {
      Serial.print("BMP390 - 온도: ");
      Serial.print(bmp.temperature);
      Serial.print(" °C, 기압: ");
      Serial.print(bmp.pressure / 100.0);
      Serial.print(" hPa, 고도: ");
      Serial.print(bmp.readAltitude(1013.25));
      Serial.println(" m");
    } else {
      Serial.println("BMP390 - 읽기 실패");
    }

    // BNO085 데이터 (마지막으로 읽은 값 출력)
    if (has_accel) {
      Serial.print("BNO085 - 가속도: X=");
      Serial.print(last_accel_x);
      Serial.print(" Y=");
      Serial.print(last_accel_y);
      Serial.print(" Z=");
      Serial.print(last_accel_z);
      Serial.println(" m/s²");
    }
    
    if (has_quat) {
      Serial.print("BNO085 - 회전(Quat): ");
      Serial.print(last_quat_i, 3);
      Serial.print(", ");
      Serial.print(last_quat_j, 3);
      Serial.print(", ");
      Serial.println(last_quat_k, 3);
    }
    
    if (!has_accel && !has_quat) {
      Serial.println("BNO085 - 데이터 없음");
    }

    // GPS 데이터
    Serial.print("GPS - Fix: ");
    Serial.print((int)GPS.fix);
    if (GPS.fix) {
      Serial.print(", 위도: ");
      Serial.print(GPS.latitude, 4);
      Serial.print(GPS.lat);
      Serial.print(", 경도: ");
      Serial.print(GPS.longitude, 4);
      Serial.print(GPS.lon);
      Serial.print(", 위성: ");
      Serial.println((int)GPS.satellites);
    } else {
      Serial.println(" (신호 없음 - 실내에서는 정상)");
    }
    
    Serial.println();
  }
}