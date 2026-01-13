#include <Arduino.h>

#define ESP Serial1
#define RXD1 44 // D7
#define TXD1 43 // D6

char receivedChar;

void setup() {
  Serial.begin(9600); // 시리얼 모니터 출력용
  ESP.begin(9600, SERIAL_8N1, RXD1, TXD1); // Teensy와의 통신용
}

void loop() {
  if (ESP.available()) // Teensy로부터 데이터가 도착했는지 확인
  {
    while(ESP.available()){
      receivedChar = ESP.read(); // 데이터 읽기
    }
    // 받은 데이터를 다시 Teensy로 전송
    ESP.write("G");
    
    // TODO : 받은 데이터에 따라 녹화 시작/종료 명령 처리
    // if (receivedChar == 'R'){ // 녹화 시작(Racode)

    // } else { // 녹화 종료

    // }
  }
  else ESP.write("F");

  delay(100);
}
