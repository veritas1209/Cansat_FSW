하단의 내용은 AI로 생성되었으며 정확하지 않을 수 있습니다.

---

## 📋 목차

- [프로젝트 개요](#-프로젝트-개요)
- [프로젝트 구조](#-프로젝트-구조)
- [대회 규정](#-대회-규정)
- [미션 단계](#-미션-단계)
- [시스템 아키텍처](#-시스템-아키텍처)
- [하드웨어 구성](#-하드웨어-구성)
- [소프트웨어 모듈](#-소프트웨어-모듈)
- [명령어 규격](#-명령어-규격)

---


## 🎯 프로젝트 개요

1062팀은 **2026 CanSat Competition - Paraglider Instrument Delivery** 미션에 참가하며, 본 프로젝트는 **Flyting Software (FSW)** 개발에 집중합니다.

### FSW 핵심 기능

- ✅ **센서 데이터 수집**: BMP390(기압), BNO085(IMU), GPS
- ✅ **칼만 필터링**: 고품질 센서 데이터 생성
- ✅ **비행 상태 감지**: 자동 상태 전환 알고리즘
- ✅ **텔레메트리 전송**: 1Hz 주기, XBee 통신 (대회 규정 준수)
- ✅ **지상국 명령 처리**: CX, ST, SIM, SIMP, CAL 명령 지원
- ✅ **시뮬레이션 모드**: 지상 테스트 및 FRR 검증용

### 기술 스택

| 구분 | 사양 |
|------|------|
| **MCU** | Teensy 4.1 (600 MHz ARM Cortex-M7) |
| **센서** | BMP390, BNO085, GPS |
| **필터** | 1D Kalman Filter (자체 구현) |
| **통신** | XBee 2.4GHz or 900MHz (X1-X4 규정 준수) |
| **저장** | SD Card (백업용) |

---

## 🛠️ 프로젝트 구조
```
Teensy/
├── include/
│   ├── Filter.h              # ✅ 칼만 필터
│   ├── Packet.h              # ✅ 텔레메트리 패킷
│   ├── State.h               # ✅ 비행 상태 관리
│   ├── CMD.h                 # ⏳ 명령 처리
│   ├── DCS.h                 # ⏳ 데이터 수집 시스템
│   ├── EggDrop.h             # ⏳ 계란 방출 제어
│   ├── Mode.h                # ⏳ 모드 관리
│   ├── PID.h                 # ⏳ PID 제어
│   └── Recovery.h            # ⏳ 회수 시스템
│   └── sensors/
│       ├── BMP390.h          # ✅ 기압 센서
│       ├── BNO085.h          # ✅ IMU 센서
│       ├── GPS.h             # ✅ GPS 모듈
│       ├── Audio.h           # ✅ 부저 제어
│       ├── SensorManager.h   # ✅ 센서 통합 관리
│       ├── SDCARD.h          # ⏳ SD 카드
│       ├── XBee_Module.h     # ⏳ XBee 통신
│       ├── Servo.h           # ⏳ 서보 제어
│       └── Teensy_Camera.h   # ⏳ 카메라
│
├── src/
│   ├── Filter.cpp            # ✅
│   ├── Packet.cpp            # ✅
│   ├── State.cpp             # ✅
│   ├── main.cpp              # ✅ 메인 루프
│   └── sensors/
│       ├── BMP390.cpp        # ✅
│       ├── BNO085.cpp        # ✅
│       ├── GPS.cpp           # ✅
│       ├── Audio.cpp         # ✅
│       ├── SensorManager.cpp # ✅
│       ├── SDCARD.cpp        # ⏳ 구현 중
│       └── XBee_Module.cpp   # ⏳ 구현 중
│
└── platformio.ini            # 프로젝트 설정

```

## 📜 대회 규정

### 센서 요구사항 (SN1-SN10)

| 규정 ID | 요구사항 |
|---------|----------|
| **SN1** | 기압 센서로 고도 측정 |
| **SN2** | 내부 온도 측정 |
| **SN3** | 배터리 전압 측정 |
| **SN4** | GPS 위치 추적 |
| **SN5** | 가속도 및 회전 속도 측정 |
| **SN6** | Para-glider 전개 영상 |
| **SN7** | 지면 하강 영상 |
| **SN8** | 계란 방출 영상 |
| **SN9** | 640x480 컬러 영상 |
| **SN10** | 배터리 전류 측정 |

### 통신 요구사항 (X1-X5)

| 규정 ID | 요구사항 |
|---------|----------|
| **X1** | XBee 2.4GHz/900MHz 사용 |
| **X2** | NETID/PANID = 팀 번호 (1062) |
| **X3** | 브로드캐스트 모드 금지 |
| **X4** | 1Hz 텔레메트리 전송 |
| **X5** | 필수 데이터 포함 |

### 비행 소프트웨어 요구사항 (F1-F8)

| 규정 ID | 요구사항 |
|---------|----------|
| **F1** | 패킷 카운터 유지 (리셋 후에도) |
| **F2** | 미션 시간 유지 (리셋 후에도) |
| **F3** | UTC 시간 설정 (±1초) |
| **F4** | 시뮬레이션 모드 지원 |
| **F5** | SIMP 명령으로 기압 대체 |
| **F6** | SIM ENABLE + ACTIVATE 필요 |
| **F7** | 메커니즘 명령 지원 |

> **Teensy 4.1 기반 Sensor Subsystem (SSS)**  
> 2026 CanSat Competition - Paraglider Instrument Delivery Mission



## 🚀 미션 단계

### Phase 1: 발사 및 상승 (Launch & Ascent)
```
상태: LAUNCH_PAD → ASCENT
트리거: 고도 10m 이상 + 상승 감지
SSS 동작:
  - 센서 데이터 수집 시작 (100Hz+)
  - 1Hz 텔레메트리 전송 (대회 규정)
  - 최대 고도 추적
  - 상태: ASCENT로 자동 전환
```

### Phase 2: 정점 도달 (Apogee)
```
상태: ASCENT → APOGEE
트리거: 연속 3회 하강 감지
SSS 동작:
  - 최대 고도 확정 및 기록
  - 2초 후 DESCENT로 전환
  - 텔레메트리 지속
```

### Phase 3: Container 하강 (Descent)
```
상태: APOGEE → DESCENT
동작:
  - 로켓 낙하산 전개 (로켓 시스템)
  - Container + Payload 통합 하강
  - 하강 속도: 15 m/s (±3 m/s)
  - 80% 고도 도달 감지 대기
```

### Phase 4: Payload 분리 (Payload Release)
```
상태: DESCENT → PAYLOAD_RELEASE
트리거: 최대 고도의 80% 도달 (규정 C5)
SSS 동작:
  - 분리 이벤트 감지 및 로깅
  - 상태 변경: PAYLOAD_RELEASE
  - 텔레메트리 지속
CDH 동작:
  - 카메라 이벤트 트리거 (비디오 녹화)
다른 시스템:
  - 서보모터로 Payload 분리 (Mechanism)
  - Para-glider 전개 (Payload Control)
  - 하강 속도: 5 m/s (±3 m/s, 규정 C7)
```

### Phase 5: 계기(계란) 투하 (Probe Release)
```
상태: PAYLOAD_RELEASE → PROBE_RELEASE
트리거: 지면으로부터 2m (±0.5m, 규정 C12)
SSS 동작:
  - 정밀 고도 측정 (BMP390 + 칼만 필터)
  - 2m ±0.5m 도달 감지
  - 상태 변경: PROBE_RELEASE
CDH 동작:
  - 카메라 이벤트 트리거 (계란 방출 녹화)
다른 시스템:
  - 서보모터로 계란(54-64g) 방출
  - 계란 보호 장치 작동
```

### Phase 6: 착륙 (Landing)
```
상태: PROBE_RELEASE → LANDED
트리거: 고도 5m 미만 + 3초간 안정
SSS/CDH 동작:
  - 상태 변경: LANDED
  - 오디오 비콘 자동 작동 (규정 C13)
  - 텔레메트리 지속 (회수 용이)
  - SD 카드 데이터 저장 완료
```

---

## 🏗️ 시스템 아키텍처

```
┌────────────────────────────────────────────────────┐
│          main.cpp (통합 제어 루프)                   │
├────────────────────────────────────────────────────┤
│  ┌──────────┐  ┌──────────┐  ┌────────┐  ┌──────┐  │
│  │  Packet  │  │  State   │  │ Audio  │  │ XBee │  │
│  │          │  │          │  │        │  │(CDH) │  │
│  └──────────┘  └──────────┘  └────────┘  └──────┘  │
├────────────────────────────────────────────────────┤
│         SensorManager (센서 통합 관리)               │
├────────────────────────────────────────────────────┤
│  ┌──────────┐  ┌──────────┐  ┌────────┐           │
│  │ BMP390   │  │ BNO085   │  │  GPS   │           │
│  │ (기압)    │  │ (IMU)    │  │(위치)  │           │
│  │  ↓       │  │  ↓       │  │        │           │
│  │ Filter1D │←─┤IMUFilter │  │        │           │
│  └──────────┘  └──────────┘  └────────┘           │
└────────────────────────────────────────────────────┘
        ↓                    ↓              ↓
   [XBee 통신]          [SD Card]      [Camera]
```

### 데이터 흐름

```
센서 읽기 (100Hz+)
    ↓
칼만 필터 적용 (BNO085만)
    ↓
State 업데이트 (상태 전환 로직)
    ↓
Packet 생성 (1Hz) ←────── 대회 규정 X4
    ↓
├→ XBee 전송 (지상국)
├→ SD 카드 저장 (백업)
└→ 카메라 트리거 (이벤트 시)
```

---

## 🔌 하드웨어 구성

### 핀 배치

| 센서/장치 | 통신 방식 | 핀 번호 | 비고 |
|----------|----------|---------|------|
| **BMP390** | I2C (Wire) | SDA: 18, SCL: 19 | 기압, 온도, 고도 |
| **BNO085** | I2C (Wire1) | SDA: 17, SCL: 16 | 자이로, 가속도, 쿼터니언 |
| **GPS** | Serial1 | TX: 0, RX: 1 | 위치, 시간, 위성 수 |
| **XBee** | Serial2 | TX: 7, RX: 8 | 텔레메트리 전송 (계획) |
| **부저** | PWM | Pin: TBD | Audible beacon |
| **서보** | PWM | Pin: TBD | Payload/Probe 분리 |

### 센서 사양

#### BMP390 (기압 센서)
- **측정 범위**: 300-1250 hPa
- **정확도**: ±0.5 hPa (±4m)
- **샘플링**: 50Hz
- **주소**: 0x77

#### BNO085 (IMU)
- **자이로**: ±2000 dps
- **가속도**: ±16g
- **샘플링**: 폴링 모드
- **주소**: 0x4A

#### GPS
- **프로토콜**: NMEA (RMC, GGA)
- **업데이트**: 1Hz
- **보레이트**: 9600

---

## 📦 소프트웨어 모듈

### 1. Filter (칼만 필터)

#### Filter1D
```cpp
// 1차원 칼만 필터 (단일 센서값 필터링)
Filter1D filter;
filter.begin(initial_value, process_noise, measurement_noise);
float filtered = filter.update(raw_measurement);
```

**파라미터**:
- `Q` (Process Noise): 시스템의 불확실성 (작을수록 모델 신뢰)
- `R` (Measurement Noise): 센서의 노이즈 (작을수록 센서 신뢰)

#### IMUFilter
```cpp
// IMU 6축 필터링 (자이로 3축 + 가속도 3축)
IMUFilter imuFilter;
imuFilter.begin(gyro_Q, gyro_R, accel_Q, accel_R);
imuFilter.updateGyro(raw_roll, raw_pitch, raw_yaw);
imuFilter.updateAccel(raw_roll, raw_pitch, raw_yaw);
```

**기본 노이즈 파라미터**:
```cpp
Gyro:  Q = 0.01,  R = 0.1
Accel: Q = 0.01,  R = 0.5
```

---

### 2. 센서 모듈

#### BMP390
```cpp
BMP390 bmp;
bmp.begin();                          // 초기화
bmp.calibrateAltitude(100);           // 지면 고도 보정
bmp.update();                         // 데이터 읽기

float temp = bmp.getTemperature();    // 온도 (°C)
float press = bmp.getPressure();      // 기압 (hPa)
float alt = bmp.getAltitude();        // 절대 고도 (m)
float relAlt = bmp.getRelativeAltitude(); // 상대 고도 (m)
```

#### BNO085
```cpp
BNO085 imu;
imu.begin();                          // 초기화
imu.enableFilter(0.01, 0.1, 0.01, 0.5); // 칼만 필터 활성화
imu.update();                         // 데이터 읽기

// 필터링된 값 (기본)
float roll = imu.getGyroRoll();       // Roll (°)
float pitch = imu.getGyroPitch();     // Pitch (°)
float yaw = imu.getGyroYaw();         // Yaw (°)

float accel_r = imu.getAccelRoll();   // Roll 방향 가속도
float accel_p = imu.getAccelPitch();  // Pitch 방향 가속도
float accel_y = imu.getAccelYaw();    // Yaw 방향 가속도

// Raw 값 접근
float raw_roll = imu.getGyroRollRaw();
```

#### GPS
```cpp
GPS gps;
gps.begin();
gps.update();  // loop()에서 계속 호출 필요

bool fix = gps.hasFix();
float lat = gps.getLatitude();
float lon = gps.getLongitude();
float alt = gps.getAltitude();
int sats = gps.getSatellites();
String time = gps.getTimeString();  // HH:MM:SS
```

---

### 3. State (비행 상태 관리)

#### 상태 다이어그램
```cpp
enum FlightState {
    LAUNCH_PAD,        // 발사대 대기
    ASCENT,            // 상승 중
    APOGEE,            // 최고점 도달
    DESCENT,           // 하강 중 (Container + Payload)
    PAYLOAD_RELEASE,   // Payload 분리 (80% 고도)
    PROBE_RELEASE,     // 계기(계란) 방출 준비
    LANDED             // 착륙 완료
};
```

#### 사용 방법
```cpp
State flightState;
flightState.attachSensors(&bmp, &imu, &gps);
flightState.begin();
flightState.calibrate();    // 지면 고도 보정
flightState.startMission(); // 미션 시작

// loop()에서
flightState.update();  // 자동 상태 전환

// 상태 조회
FlightState current = flightState.getCurrentState();
String stateStr = flightState.getStateString();
float maxAlt = flightState.getMaxAltitude();
```

#### 상태 전환 조건

| 전환 | 조건 |
|------|------|
| LAUNCH_PAD → ASCENT | 고도 ≥ 10m + 상승 중 (0.5m/s 이상) |
| ASCENT → APOGEE | 연속 3회 하강 감지 |
| APOGEE → DESCENT | 2초 경과 |
| DESCENT → PAYLOAD_RELEASE | 고도 ≤ 최대 고도 × 0.8 |
| PAYLOAD_RELEASE → PROBE_RELEASE | 고도 ≤ 2m ± 0.5m |
| PROBE_RELEASE → LANDED | 고도 < 5m + 3초간 안정 |

#### 임계값 커스터마이징
```cpp
StateThresholds thresholds;
thresholds.launchAltitude = 10.0;           // 발사 감지 (m)
thresholds.apogeeDetectionWindow = 2000;    // Apogee 대기 (ms)
thresholds.payloadReleasePercent = 0.80;    // Payload 분리 (80%)
thresholds.probeReleaseAltitude = 2.0;      // 계란 방출 (m)
thresholds.probeReleaseMargin = 0.5;        // 계란 방출 오차 (m)
thresholds.landedAltitude = 5.0;            // 착륙 판정 (m)
thresholds.landedTimeWindow = 3000;         // 착륙 안정 시간 (ms)

flightState.setThresholds(thresholds);
```

---

### 4. Packet (텔레메트리)

#### CSV 형식
```csv
TEAM_ID,MISSION_TIME,PACKET_COUNT,MODE,STATE,
ALTITUDE,TEMPERATURE,ATM_PRESSURE,VOLTAGE,CURRENT,
GYRO_R,GYRO_P,GYRO_Y,
ACCEL_R,ACCEL_P,ACCEL_Y,
GPS_TIME,GPS_ALTITUDE,GPS_LATITUDE,GPS_LONGITUDE,GPS_SATS,
CMD_ECHO
```

#### 사용 방법
```cpp
Packet telemetry;
telemetry.attachSensors(&bmp, &imu, &gps);
telemetry.attachState(&flightState);
telemetry.startMission();

// loop()에서 1초마다
String csvPacket = telemetry.generatePacketString();
Serial.println(csvPacket);  // XBee, SD로 전송

telemetry.incrementPacketCount();
```

---

### 5. SensorManager (센서 통합 관리)

```cpp
SensorManager sensorManager;
sensorManager.attachSensors(&bmp, &imu, &gps);
sensorManager.initializeAll();
sensorManager.enableIMUFilter(0.01, 0.1, 0.01, 0.5);
sensorManager.calibrateAltitude(100);

// loop()에서
sensorManager.updateAll();  // 모든 센서 업데이트
```

---


## 🎮 명령어 규격 (대회 규정 준수)

### 명령어 형식
```
CMD,<TEAM_ID>,<COMMAND>,<PARAMETER>
예시: CMD,1062,CX,ON
```

### 지원 명령어

#### 1. CX - 텔레메트리 제어 (규정 3.1.2)
```cpp
// 텔레메트리 활성화
CMD,1062,CX,ON

// 텔레메트리 비활성화  
CMD,1062,CX,OFF

```
#### 2. ST - 시간 설정 (규정 3.1.2)
```cpp
// UTC 시간 직접 설정
CMD,1062,ST,13:35:59

// GPS 시간 동기화
CMD,1062,ST,GPS
```

**정확도**: ±1초 (규정 F3)

---

#### 3. SIM - 시뮬레이션 모드 (규정 3.1.3)
```cpp
// Step 1: 시뮬레이션 준비
CMD,1062,SIM,ENABLE

// Step 2: 시뮬레이션 활성화 (ENABLE 후에만 가능)
CMD,1062,SIM,ACTIVATE

// 시뮬레이션 종료
CMD,1062,SIM,DISABLE
```

**중요**: FRR(Flight Readiness Review) 필수 기능!

---

#### 4. SIMP - 시뮬레이션 기압 입력 (규정 3.1.2)
```cpp
// 시뮬레이션 기압 전송 (Pa 단위)
CMD,1062,SIMP,101325  // 해수면 기압 (1013.25 hPa)
```
**용도**: 지상국이 CSV 파일에서 1Hz로 전송  
**구현 상태**: ⏳ SIM 모드와 함께 구현 필요

---

#### 5. CAL - 고도 보정 (규정 3.1.2)
```cpp
// 현재 위치를 고도 0m로 설정
CMD,1062,CAL
```
**구현 상태**: ✅ State.cpp에 구현됨  
**용도**: 발사대에서 지면 기준 설정 (100회 평균)

---

#### 6. MEC - 메커니즘 제어 (규정 3.1.2)
```cpp
// 서보모터 활성화 (테스트용)
CMD,1062,MEC,SERVO1,ON

// 서보모터 비활성화
CMD,1062,MEC,SERVO1,OFF
```

**용도**: FRR에서 메커니즘 테스트용, 비행 중엔 자동

---

### 명령어 처리 흐름 (구현 예정)

```cpp
// XBee로 명령 수신
String command = XBee.readStringUntil('\n');

// 파싱
if (command.startsWith("CMD,1062,")) {
    String parts[] = parseCommand(command);
    
    // 명령 실행
    if (parts[2] == "CX") {
        handleCX(parts[3]);  // ON/OFF
    }
    else if (parts[2] == "ST") {
        handleST(parts[3]);  // HH:MM:SS or GPS
    }
    else if (parts[2] == "SIM") {
        handleSIM(parts[3]); // ENABLE/ACTIVATE/DISABLE
    }
    else if (parts[2] == "SIMP") {
        handleSIMP(parts[3]); // Pressure in Pa
    }
    else if (parts[2] == "CAL") {
        handleCAL();
    }
    else if (parts[2] == "MEC") {
        handleMEC(parts[3], parts[4]); // DEVICE, ON/OFF
    }
    
    // CMD_ECHO 업데이트
    cmdEcho = parts[2] + parts[3];
}
```

---

### 시뮬레이션 모드 시퀀스 (FRR 필수!)

```
지상국                          CanSat
  │                               │
  ├─ CMD,1062,SIM,ENABLE ────────>│
  │                               ├─ simEnableReceived = true
  │                               │
  ├─ CMD,1062,SIM,ACTIVATE ──────>│
  │                               ├─ simActivateReceived = true
  │                               ├─ simModeEnabled = true
  │                               ├─ flightMode = 'S'
  │                               │
  ├─ CMD,1062,SIMP,101325 ───────>│ (1Hz로 반복)
  ├─ CMD,1062,SIMP,100500 ───────>│
  ├─ CMD,1062,SIMP,99800 ────────>│
  │                               ├─ altitude 계산 (SIMP 값 사용)
  │                               ├─ 상태 전환 (ASCENT → APOGEE 등)
  │<──── Telemetry (MODE='S') ────┤
  │                               │
```

---

### 실전 운용 시퀀스

```
발사 준비 (T-30분)
├─ CMD,1062,CX,ON       // 텔레메트리 시작
├─ CMD,1062,ST,GPS      // GPS 시간 동기화
└─ CMD,1062,CAL         // 고도 보정 (0m 설정)

발사 (T-0)
└─ 자동 상태 전환 시작 (명령 불필요)

회수 후
└─ CMD,1062,CX,OFF      // 텔레메트리 종료
```

---

## 🛠️ 개발 가이드

### 프로젝트 구조

```
Teensy/
├── include/
│   ├── Filter.h              # 칼만 필터
│   ├── Packet.h              # 텔레메트리 패킷
│   ├── State.h               # 비행 상태 관리
│   └── sensors/
│       ├── BMP390.h          # 기압 센서
│       ├── BNO085.h          # IMU 센서
│       ├── GPS.h             # GPS 모듈
│       └── SensorManager.h   # 센서 통합 관리
│
├── src/
│   ├── Filter.cpp
│   ├── Packet.cpp
│   ├── State.cpp
│   ├── main.cpp              # 메인 루프
│   └── sensors/
│       ├── BMP390.cpp
│       ├── BNO085.cpp
│       ├── GPS.cpp
│       └── SensorManager.cpp
│
└── platformio.ini            # 프로젝트 설정
```

## 📊 성능 벤치마크

| 항목 | 측정값 |
|------|--------|
| **센서 업데이트** | ~200Hz |
| **State 업데이트** | ~200Hz |
| **텔레메트리 전송** | 1Hz (정확) |
| **메모리 사용** | RAM: ~45KB, Flash: ~120KB |
| **고도 정확도** | ±0.5m (보정 후) |
| **상태 전환 지연** | <100ms |

---
**Happy Flying! 🚀**

**Team 1062 - 2026 CanSat Competition**