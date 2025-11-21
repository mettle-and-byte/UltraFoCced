#include <Arduino.h>
#include <SimpleFOC.h>
#include "SafetyMonitor.h"
#include <SimplePioSPI.h>
#include "encoders/as5047/MagneticSensorAS5047.h"

// Pin Definitions
#define LED_PIN 23
#define FAN_PIN 16
#define ENDSTOP_PIN 18

// Encoder Pins
#define ENC_CS 19
#define ENC_SCK 20
#define ENC_MISO 21
#define ENC_MOSI 22
#define DUMMY_CS 24 // Unused

// Instantiate SafetyMonitor
SafetyMonitor safety(FAN_PIN, ENDSTOP_PIN);

// Instantiate Commander
Commander command = Commander(Serial);

// Instantiate PIO SPI
// MOSI, MISO, SCK, CS, PIO, SM
SimplePioSPI pio_spi(ENC_MOSI, ENC_MISO, ENC_SCK, DUMMY_CS, pio0, 0);

// Instantiate Encoder
MagneticSensorAS5047 sensor = MagneticSensorAS5047(ENC_CS);

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
  Serial.println("MnB Ultralight N17 Firmware - Phase 3 (Encoder SimplePioSPI)");

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
    // Serial.println("DEBUG: Heartbeat"); // Optional: Uncomment if loop hangs
  }

  // Run Safety Monitor
  safety.run();

  // Update Encoder
  sensor.update();

  // Run Commander
  command.run();

  // Periodic Angle Query
  static unsigned long last_angle_query = 0;
  if (millis() - last_angle_query > 200) {
    last_angle_query = millis();

    float angle_deg = sensor.getAngle() * (180.0 / PI);
    uint16_t agc = sensor.readMagnitude();
    bool error = sensor.isErrorFlag();

    Serial.print("Angle: "); Serial.print(angle_deg); Serial.print(" deg");
    Serial.print(" | AGC: "); Serial.print(agc);
    Serial.print(" | Err: "); Serial.println(error ? "YES" : "NO");
  }
}
