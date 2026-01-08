// src/sensors/BMP390.cpp
#include "sensors/BMP390.h"

BMP390::BMP390() {
    baseAltitude = 0.0;
    initialized = false;
}

bool BMP390::begin() {
    Serial.print("BMP390 초기화 중 (SDA=18, SCL=19)... ");
    
    Wire.begin();
    Wire.setClock(400000); // 400kHz
    
    if (bmp.begin_I2C(0x77, &Wire)) {
        Serial.println("성공!");
        
        // 센서 설정
        bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
        bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
        bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
        bmp.setOutputDataRate(BMP3_ODR_50_HZ);
        
        initialized = true;
        return true;
    } else {
        Serial.println("실패! 연결 확인 필요");
        initialized = false;
        return false;
    }
}

bool BMP390::update() {
    if (!initialized) return false;
    return bmp.performReading();
}

float BMP390::getTemperature() {
    if (!initialized) return 0.0;
    return bmp.temperature;
}

float BMP390::getPressure() {
    if (!initialized) return 0.0;
    return bmp.pressure / 100.0; // Pa to hPa
}

float BMP390::getAltitude() {
    if (!initialized) return 0.0;
    return bmp.readAltitude(1013.25); // 해수면 기압 기준
}

float BMP390::getRelativeAltitude() {
    if (!initialized) return 0.0;
    return getAltitude() - baseAltitude;
}

void BMP390::calibrateAltitude(int samples) {
    if (!initialized) return;
    
    Serial.print("BMP390 고도 캘리브레이션 중 (");
    Serial.print(samples);
    Serial.print("회)... ");
    
    float sum = 0.0;
    int count = 0;
    
    for (int i = 0; i < samples; i++) {
        if (bmp.performReading()) {
            sum += bmp.readAltitude(1013.25);
            count++;
        }
        delay(10);
    }
    
    if (count > 0) {
        baseAltitude = sum / count;
        Serial.print("완료! 기준 고도: ");
        Serial.print(baseAltitude, 2);
        Serial.println(" m");
    } else {
        Serial.println("실패!");
    }
}