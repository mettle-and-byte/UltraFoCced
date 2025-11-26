#include <Arduino.h>
#include "Core0.h"
#include "Core1.h"

// Give Core 1 its own 8KB stack to prevent collisions
bool core1_separate_stack = true;

void setup() {
    Core0::setup();
}

void loop() {
    Core0::loop();
}

void setup1() {
    Core1::setup();
}

void loop1() {
    Core1::loop();
}
