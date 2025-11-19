#include "SafetyMonitor.h"

SafetyMonitor::SafetyMonitor(int fanPin, int endstopPin) {
    _fanPin = fanPin;
    _endstopPin = endstopPin;
}

void SafetyMonitor::init() {
    pinMode(_fanPin, OUTPUT);
    pinMode(_endstopPin, INPUT_PULLUP); // Assuming NPN Open Collector, pullup needed. Check schematic if external pullup exists.
    // Netlist showed R16/R18 pullups might exist, but internal pullup is safer default.

    analogReadResolution(12); // RP2040/2350 ADC is 12-bit

    setFanState(false);
}

void SafetyMonitor::run() {
    unsigned long now = millis();
    if (now - _lastTempCheck > 1000) { // Check every second
        updateTemperature();
        checkSafety();
        _lastTempCheck = now;
    }
}

float SafetyMonitor::getMCUChipTemp() {
    return _currentMCUTemp;
}

bool SafetyMonitor::getEndstopStatus() {
    // Invert if needed based on NPN logic (Low = Triggered usually)
    return digitalRead(_endstopPin);
}

bool SafetyMonitor::isFanOn() {
    return _fanState;
}

void SafetyMonitor::setFanState(bool on) {
    _fanState = on;
    digitalWrite(_fanPin, _fanState ? HIGH : LOW);
}

void SafetyMonitor::setFanOverride(bool enabled) {
    _fanOverride = enabled;
}

void SafetyMonitor::setMaxMCUTemp(float tempC) {
    _maxMCUTemp = tempC;
}

void SafetyMonitor::updateTemperature() {
    // RP2040/RP2350 internal temp sensor is on ADC 4
    // Formula from datasheet: T = 27 - (ADC_voltage - 0.706) / 0.001721
    // But Arduino-Pico core might have a helper `analogReadTemp()`?
    // Let's use the raw calculation for portability if helper missing, but `analogReadTemp()` is standard in this core.

    _currentMCUTemp = analogReadTemp();
}

void SafetyMonitor::checkSafety() {
    if (_currentMCUTemp > _maxMCUTemp) {
        // Overtemp! Turn on fan regardless of override
        setFanState(true);
        // TODO: Disable motor driver here when we have the driver object
    } else if (!_fanOverride) {
        // Auto fan control hysteresis
        if (_currentMCUTemp > 50.0f) {
            setFanState(true);
        } else if (_currentMCUTemp < 45.0f) {
            setFanState(false);
        }
    }
}

void SafetyMonitor::commander(char* cmd) {
    // Simple command parser for "F" (Fan) and "T" (Temp)
    // This is designed to be called by SimpleFOC Commander if we register it,
    // or we can manually parse in main loop.

    switch(cmd[0]) {
        case 'F': // Fan Control: F1 = On, F0 = Off, F? = Status
            if (cmd[1] == '?') {
                Serial.print("Fan: "); Serial.println(_fanState);
            } else {
                bool state = atoi(&cmd[1]);
                setFanOverride(true); // Manual command implies override
                setFanState(state);
                Serial.print("Fan set: "); Serial.println(state);
            }
            break;
        case 'T': // Temp Status: T?
            if (cmd[1] == '?') {
                Serial.print("MCU Temp: "); Serial.println(_currentMCUTemp);
            }
            break;
        case 'E': // Endstop Status: E?
            if (cmd[1] == '?') {
                Serial.print("Endstop: "); Serial.println(getEndstopStatus());
            }
            break;
    }
}
