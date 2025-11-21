#include "SDCARD.h"

#define FILE_NAME "Flight_1062.csv"

void SDCard::init() {
    Serial.println("SD 카드 초기화 중...");
    if (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("SD 카드 초기화 실패!");
        initialized = false;
        return;
    }
    Serial.println("SD 카드 초기화 성공!");
    initialized = true;
}

void SDCard::writeData(String data) {
    if (!initialized) {
        Serial.println("SD 카드가 초기화되지 않았습니다.");
        return;
    }

    File dataFile = SD.open(FILE_NAME, FILE_WRITE);
    if (dataFile) {
        dataFile.println(data);
        dataFile.close();
        Serial.println("데이터가 SD 카드에 기록되었습니다: ");
    } else {
        Serial.println("파일을 열 수 없습니다.");
    }
}

String SDCard::readData(){
    String content = "";
    if (!initialized) {
        Serial.println("SD 카드가 초기화되지 않았습니다.");
        return content;
    }

    File dataFile = SD.open(FILE_NAME, FILE_READ);
    if (dataFile) {
        Serial.println("SD 카드에서 데이터 읽는 중...");
        while (dataFile.available()) {
            content += (char)dataFile.read();
        }
        dataFile.close();
        Serial.println("데이터 읽기 완료.");
    } else {
        Serial.println("파일을 열 수 없습니다.");
    }
    return content;
}
