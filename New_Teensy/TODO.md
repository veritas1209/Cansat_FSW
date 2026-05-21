# 남은 작업 (TODO)

## 1. F1/F2/F8 영속화 (담당자 별도)
- `packetCount`, `missionTime`, `isCalibrated` 등을 EEPROM/플래시에 저장
- 재부팅 시 복원하여 미션 연속성 확보
- 미션가이드 F1(packetCount 복원), F2(missionTime 복원), F8(calibration 상태 복원) 항목 대응

## 2. MEC SERVO 실제 동작 코드
- 현재 `executeMECCommand`는 디버그 로그 출력 + `cmdEcho` 갱신만 수행
- 실제로 서보 PWM 출력 / GPIO 토글 등 하드웨어 제어 로직 추가 필요
- 안전 동작(이중 트리거 방지, 비상 정지 등) 고려

## 3. 카메라 제어 (구현 완료, 추가 검증 필요)
- Xiao ESP32S3 Sense 2대 (PL_RELS_CAM=Serial3, GROUND_CAM=Serial5) UART 제어 구현
- 프로토콜: 9600 baud, '1'=녹화 시작 / '0'=녹화 종료, ESP32가 같은 문자로 ACK
- MEC 수동 제어: `CMD,1062,MEC,CAM_PL,ON|OFF`, `CMD,1062,MEC,CAM_GND,ON|OFF`
- 자동 트리거: LAUNCH_PAD→ASCENT 시 두 카메라 모두 시작, →LANDED 시 모두 종료
- ⚠ 알려진 충돌: PL_RELS_CAM(Serial3) == GPSSerial(Serial3). 실 PCB는 GPS가 Serial6에 연결되어 있으므로 GPSSerial을 Serial6으로 이동하는 후속 작업 필요

## 참고
- 미션가이드 3.1.1.1 / 3.1.2 명령어 포맷 준수 유지
- `cmdEcho` 는 쉼표 금지 (가이드 3.1.1.1 #18)
