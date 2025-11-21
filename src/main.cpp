#include <Arduino.h>
#include <SimpleFOC.h>
#include "SafetyMonitor.h"
#include <SimplePioSPI.h>
#include "TMC2240Driver.h"
#include "encoders/as5047/MagneticSensorAS5047.h"

// Pin Definitions
#define LED_PIN 23
#define FAN_PIN 16
#define ENDSTOP_PIN 18

// TMC2240 Pins
#define DRV_CS 5
#define DRV_EN 0
#define DRV_UART_EN 29

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

// Instantiate Motor Driver
TMC2240Driver driver(DRV_CS, DRV_EN, DRV_UART_EN);

// Instantiate Motor
// 50 pole pairs for 1.8deg stepper
StepperMotor motor = StepperMotor(50);

// Commander Callbacks
void doSafety(char* cmd) { safety.commander(cmd); }
void doTarget(char* cmd) { command.motion(&motor, cmd); }

// Helper for SafetyMonitor
float readMotorTemp() {
    return driver.getChipTemperature();
}


void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);

  unsigned long start = millis();
  while (!Serial && millis() - start < 5000);
  Serial.println("MnB Ultralight N17 Firmware - Phase 4 (Motor + Temp + Encoder Fix)");

  // Initialize Safety Monitor
  safety.setMotorTempCallback(readMotorTemp);
  safety.init();

  // Initialize Encoder
  pio_spi.begin();
  sensor.init(&pio_spi);

  // Link Encoder to Motor
  motor.linkSensor(&sensor);

  // Initialize Driver
  driver.voltage_power_supply = 24.0f; // Assume 24V
  driver.voltage_limit = 12.0f; // Limit for safety
  driver.init();

  // Link Driver to Motor
  motor.linkDriver(&driver);

  // Configure Motor
  motor.controller = MotionControlType::velocity_openloop; // Start with open loop
  motor.voltage_limit = 6.0f; // Low voltage for open loop test

  // Initialize Motor
  motor.init();
  // motor.initFOC(); // Skip for open loop

  // Add Commander commands
  command.add('S', doSafety, "Safety Monitor");
  command.add('M', doTarget, "Motor Target");

  Serial.println("Ready. Mode: Open Loop Velocity");
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

  // Run Motor
  motor.move();
  sensor.update(); // Keep updating sensor for telemetry

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
