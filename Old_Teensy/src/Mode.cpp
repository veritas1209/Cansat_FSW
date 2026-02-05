#include "Mode.h"

Mode::Mode() {
    currentMode = 'F';
    isSimEnabled = false;
    isSimActivated = false;
}

void Mode::init() {
    currentMode = 'F';
    isSimEnabled = false;
    isSimActivated = false;
}

void Mode::updateMode() {
    // 순서 상관없이 둘 다 true여야 S모드 진입
    if (isSimEnabled && isSimActivated) {
        currentMode = 'S';
    } else {
        currentMode = 'F';
    }
}

void Mode::setSimEnable(bool enable) {
    isSimEnabled = enable;
    updateMode();
}

void Mode::setSimActivate(bool activate) {
    isSimActivated = activate;
    updateMode();
}

void Mode::resetToFlightMode() {
    isSimEnabled = false;
    isSimActivated = false;
    currentMode = 'F';
}

char Mode::getMode() {
    return currentMode;
}

bool Mode::isSimulation() {
    return (currentMode == 'S');
}