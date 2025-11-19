#include <Arduino.h>
#include <SimpleFOC.h>
#include "SafetyMonitor.h"

// Pin Definitions
#define LED_PIN 23
#define FAN_PIN 16
#define ENDSTOP_PIN 18

// Instantiate SafetyMonitor
SafetyMonitor safety(FAN_PIN, ENDSTOP_PIN);

// Instantiate Commander
Commander command = Commander(Serial);

// Commander Callbacks
void doSafety(char* cmd) { safety.commander(cmd); }

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);

  // Wait for serial connection for up to 5 seconds
  unsigned long start = millis();
  while (!Serial && millis() - start < 5000);
  Serial.println("MnB Ultralight N17 Firmware - Phase 2");

  // Initialize Safety Monitor
  safety.init();

  // Add Commander commands
  command.add('S', doSafety, "Safety Monitor: F(an), T(emp), E(ndstop)");

  Serial.println("Ready.");
}

void loop() {
  // Heartbeat
  static unsigned long last_blink = 0;
  if (millis() - last_blink > 500) {
    last_blink = millis();
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }

  // Run Safety Monitor
  safety.run();

  // Run Commander
  command.run();
}
