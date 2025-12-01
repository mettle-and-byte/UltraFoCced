#include "Core1.h"
#include "Shared.h"
#include <pico/bootrom.h>
#include <hardware/watchdog.h>

// --- Command Callbacks ---

void doSafety(char* cmd) { safety.commander(cmd); }

void doMotor(char* cmd){command.motor(&motor, cmd);}

void doTune(char* cmd) {
    autotuner.startTuning();
}

void doBootloader(char* cmd) {
    Serial.println("Entering Bootloader...");
    delay(100);
    reset_usb_boot(0, 0);
}

void doReboot(char* cmd) {
    Serial.println("Rebooting...");
    delay(100);
    watchdog_enable(1, 1);
    while(1);
}

void doDebug(char* cmd) {
    Serial.println("--- Debug Info ---");
    Serial.print("Motor Phase Resistance: "); Serial.println(motor.phase_resistance, 4);
    Serial.print("Motor Current Q: "); Serial.println(motor.current.q, 4);
    Serial.print("Motor Voltage Q: "); Serial.println(motor.voltage.q, 4);
    Serial.print("Driver Has Error: "); Serial.println(driver.hasDriverError() ? "YES" : "NO");

    if (motor.enabled) {
        Serial.println("WARNING: Cannot read driver registers while motor is enabled (SPI conflict).");
        Serial.println("Disable motor (ME0) to view detailed driver status.");
    } else {
        Serial.print("Driver GSTAT: 0x"); Serial.println(driver.getGSTAT(), HEX);
        Serial.print("Driver DRV_STATUS: 0x"); Serial.println(driver.getDRVSTATUS(), HEX);
        Serial.print("Driver IOIN: 0x"); Serial.println(driver.getIOIN(), HEX);
    }
    Serial.println("------------------");
}

void Core1::setup() {
    pinMode(LED_PIN, OUTPUT);
    Serial.begin(115200);

    // Add Commander commands
    command.add('M', doMotor, "motor");
    command.add('S', doSafety, "Safety Monitor");
    command.add('T', doTune, "Auto-Tune");
    command.add('B', doBootloader, "Enter Bootloader");
    command.add('R', doReboot, "Reboot");
    command.add('D', doDebug, "Debug Info");
    delay(5000);
}

void Core1::loop() {
    // 1. Bridge Serial (RX) -> QueueStream (for Commander)
    while (Serial.available()) {
        int c = Serial.read();
        if (c != -1) {
            serial_stream.pushRX((uint8_t)c);
        }
    }

    // 2. Run Commander (now reads from QueueStream)
    command.run();

    // 3. Bridge QueueStream (TX) -> Serial (Monitor Output from Core 0)
    // Pop and write as much as possible, but don't block if Serial is full/disconnected
    while (true) {
        // Check if Serial can accept data
        if (Serial.availableForWrite() <= 0) {
            break;
        }

        int c = serial_stream.popTX();
        if (c == -1) break;
        Serial.write((uint8_t)c);
    }

    // 4. Check Overflow (TX from Core 0)
    uint32_t overflow = serial_stream.getAndClearOverflow();
    if (overflow > 0) {
        Serial.print("Warn: TX Buffer Overflow! Dropped ");
        Serial.print(overflow);
        Serial.println(" bytes.");
    }

    // 4. Blink LED (Heartbeat)
    static unsigned long last_blink = 0;
    if (millis() - last_blink > 500) {
        last_blink = millis();
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
}
