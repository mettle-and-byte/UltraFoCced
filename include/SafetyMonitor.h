#ifndef SAFETY_MONITOR_H
#define SAFETY_MONITOR_H

#include <Arduino.h>
#include <SimpleFOC.h>

class SafetyMonitor {
public:
    SafetyMonitor(int fanPin, int endstopPin);

    void init();
    void run(); // Call this in the main loop

    // Getters
    float getMCUChipTemp();
    bool getEndstopStatus();
    bool isFanOn();

    // Manual Control
    void setFanState(bool on);
    void setFanOverride(bool enabled); // If true, manual control overrides auto

    // Safety Limits
    void setMaxMCUTemp(float tempC);

    // Commander Interface
    void commander(char* cmd);

private:
    int _fanPin;
    int _endstopPin;

    float _maxMCUTemp = 80.0f; // Default safe limit
    bool _fanOverride = false;
    bool _fanState = false;

    unsigned long _lastTempCheck = 0;
    float _currentMCUTemp = 0.0f;

    void updateTemperature();
    void checkSafety();
};

#endif
