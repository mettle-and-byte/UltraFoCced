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

void SafetyMonitor::setMotorTempCallback(float (*callback)()) {
    _motorTempCallback = callback;
}

void SafetyMonitor::updateTemperature() {
    // RP2040/RP2350 internal temp sensor is on ADC 4
    _currentMCUTemp = analogReadTemp();
}

void SafetyMonitor::checkSafety() {
    extern TMC2240Driver driver;
    extern StepperMotor motor;
    extern QueueStream serial_stream;

    // Fast check every loop - no SPI transaction
    if (driver.hasDriverError()) {
        // Critical fault detected - get details immediately
        uint32_t drv_status = driver.getDriverStatusFlags();
        uint32_t gstat = driver.getGSTAT();

        // Log fault details
        serial_stream.print("CRITICAL Driver Fault! GSTAT=0x");
        serial_stream.print(gstat, HEX);
        serial_stream.print(" DRV_STATUS=0x");
        serial_stream.println(drv_status, HEX);

        // Decode critical fault flags using constants
        if (drv_status & DRV_STATUS_S2GA) serial_stream.println("  S2GA: Short to Ground Phase A");
        if (drv_status & DRV_STATUS_S2GB) serial_stream.println("  S2GB: Short to Ground Phase B");
        if (drv_status & DRV_STATUS_OLA) serial_stream.println("  OLA: Open Load Phase A");
        if (drv_status & DRV_STATUS_OLB) serial_stream.println("  OLB: Open Load Phase B");

        // Disable motor immediately
        motor.disable();

        serial_stream.println("Motor disabled. Use 'C' to clear faults, then 'ME1' to re-enable.");
    }

    // Periodic status check (~100ms) for non-critical flags
    static unsigned long last_status_check = 0;
    unsigned long now = millis();
    if (now - last_status_check > 100) {
        last_status_check = now;

        uint32_t drv_status = driver.getDriverStatusFlags();

        // Log non-critical status flags (info only, don't disable motor)
        if (drv_status & DRV_STATUS_STANDSTILL) {
            // Motor is in standstill - this is normal, don't log every time
        }
        if (drv_status & DRV_STATUS_SG2) {
            serial_stream.println("Info: StallGuard active");
        }
    }

    // Then check temperature safety
    float motorTemp = 0.0f;
    if (_motorTempCallback) {
        motorTemp = _motorTempCallback();
    }

    if (_currentMCUTemp > _maxMCUTemp || motorTemp > _maxMCUTemp) {
        // Overtemp! Turn on fan regardless of override
        setFanState(true);
        // TODO: Disable motor driver here when we have the driver object
    } else if (!_fanOverride) {
        // Auto fan control hysteresis
        if (_currentMCUTemp > 50.0f || motorTemp > 50.0f) {
            setFanState(true);
        } else if (_currentMCUTemp < 45.0f && motorTemp < 45.0f) {
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
                Serial.print("MCU Temp: "); Serial.print(_currentMCUTemp);
                if (_motorTempCallback) {
                    Serial.print(" | Motor Temp: "); Serial.println(_motorTempCallback());
                } else {
                    Serial.println();
                }
            }
            break;
        case 'E': // Endstop Status: E?
            if (cmd[1] == '?') {
                Serial.print("Endstop: "); Serial.println(getEndstopStatus());
            }
            break;
    }
}
