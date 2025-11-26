#include "Core1.h"
#include "Shared.h"
#include <pico/bootrom.h>

// --- Command Callbacks ---

void doSafety(char* cmd) { safety.commander(cmd); }

void doMotor(char* cmd){command.motor(&motor, cmd);}

void doBootloader(char* cmd) {
    Serial.println("Entering Bootloader...");
    delay(100);
    reset_usb_boot(0, 0);
}

void Core1::setup() {
    pinMode(LED_PIN, OUTPUT);
    Serial.begin(115200);

    command.eol = '\r'; // Match picocom default

    // Add Commander commands
    command.add('M', doMotor, "motor");
    command.add('S', doSafety, "Safety Monitor");
    command.add('B', doBootloader, "Enter Bootloader");
    delay(5000);
}

void Core1::loop() {
    // 1. Run Commander (Reads from Serial directly)
    command.run();

    // 2. Bridge QueueStream (TX) -> Serial (Monitor Output from Core 0)
    // Pop and write as much as possible
    while (true) {
        int c = serial_stream.popTX();
        if (c == -1) break;
        Serial.write((uint8_t)c);
    }

    // 3. Check Overflow (TX from Core 0)
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
