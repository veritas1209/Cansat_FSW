#ifndef SDCARD_H
#define SDCARD_H

#include <Arduino.h>
#include <SD.h>

class SDCard {
public:
    void init();
    void writeData(String data);
    String readData();
private:
    bool initialized = false;
};

#endif