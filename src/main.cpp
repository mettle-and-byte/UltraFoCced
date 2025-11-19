#include <Arduino.h>
#include <SimpleFOC.h>
#include "SafetyMonitor.h"
#include "FixedPioSPI.h"

// Pin Definitions
#define LED_PIN 23
#define FAN_PIN 16
#define ENDSTOP_PIN 18

// Encoder Pins
#define ENC_CS 19
#define ENC_SCK 20
#define ENC_MISO 21
#define ENC_MOSI 22
#define DUMMY_CS 24 // Unused pin for PioSPI to toggle (we let SimpleFOC handle real CS)

// Instantiate SafetyMonitor
SafetyMonitor safety(FAN_PIN, ENDSTOP_PIN);

// Instantiate Commander
Commander command = Commander(Serial);

// Instantiate PIO SPI
// MOSI, MISO, SCK, CS, MODE, FREQ
FixedPioSPI pio_spi(ENC_MOSI, ENC_MISO, ENC_SCK, DUMMY_CS, SPI_MODE1, 1000000);

// Instantiate Encoder
MagneticSensorSPI sensor = MagneticSensorSPI(AS5047_SPI, ENC_CS);

// Commander Callbacks
void doSafety(char* cmd) { safety.commander(cmd); }
void doAngle(char* cmd) {
    float angle = sensor.getAngle();
    Serial.print("Angle: "); Serial.println(angle);
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);

  // Wait for serial connection for up to 5 seconds
  unsigned long start = millis();
  while (!Serial && millis() - start < 5000);
  Serial.println("MnB Ultralight N17 Firmware - Phase 3 (Encoder)");

  // Initialize Safety Monitor
  safety.init();

  // Initialize PIO SPI
  pio_spi.begin();

  // Initialize Encoder
  sensor.init(&pio_spi);

  // Add Commander commands
  command.add('S', doSafety, "Safety Monitor: F(an), T(emp), E(ndstop)");
  command.add('A', doAngle, "Get Angle");

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

  // Update Encoder
  sensor.update();

  // Run Commander
  command.run();
}
