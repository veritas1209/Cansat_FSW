# 남은 작업 (TODO)

## 1. F1/F2/F8 영속화 (담당자 별도)
- `packetCount`, `missionTime`, `isCalibrated` 등을 EEPROM/플래시에 저장
- 재부팅 시 복원하여 미션 연속성 확보
- 미션가이드 F1(packetCount 복원), F2(missionTime 복원), F8(calibration 상태 복원) 항목 대응

## 2. MEC SERVO 실제 동작 코드
- 현재 `executeMECCommand`는 디버그 로그 출력 + `cmdEcho` 갱신만 수행
- 실제로 서보 PWM 출력 / GPIO 토글 등 하드웨어 제어 로직 추가 필요
- 안전 동작(이중 트리거 방지, 비상 정지 등) 고려

## 3. 카메라 제어 (미구현)
- 카메라 ON/OFF, 녹화 시작/종료 제어 로직 작성
- MEC 명령 또는 별도 명령으로 트리거할지 결정 필요
- 비행 단계 자동 트리거(LAUNCH 시 녹화 시작 등) 검토

## 참고
- 미션가이드 3.1.1.1 / 3.1.2 명령어 포맷 준수 유지
- `cmdEcho` 는 쉼표 금지 (가이드 3.1.1.1 #18)
