#include <Arduino.h>
#include <SimpleFOC.h>
#include "SafetyMonitor.h"
#include <SimplePioSPI.h>
#include "TMC2240Driver.h"
#include "encoders/as5047/MagneticSensorAS5047.h"

#include "BoardConfig.h"
#include <pico/bootrom.h>

// Instantiate SafetyMonitor
SafetyMonitor safety(FAN_PIN, ENDSTOP_PIN);

// Instantiate Commander
Commander command = Commander(Serial);

// Instantiate PIO SPI
// MOSI, MISO, SCK, CS, PIO, SM
SimplePioSPI pio_spi(ENC_MOSI, ENC_MISO, ENC_SCK, DUMMY_CS, pio0, 0);

// Instantiate Encoder
MagneticSensorAS5047 sensor = MagneticSensorAS5047(ENC_CS);

// Motor Configuration (Will be loaded from config.ini later)
float motor_phase_resistance = 2.5f;
int motor_max_current_ma = 2000;

// Instantiate Motor Driver
TMC2240Driver driver(DRV_CS, DRV_EN, DRV_UART_EN, DRV_MISO, DRV_MOSI, DRV_SCK, TMC2240_RREF, motor_phase_resistance, motor_max_current_ma);

// Instantiate Motor
// 50 pole pairs for 1.8deg stepper
StepperMotor motor = StepperMotor(50);

// Commander Callbacks
bool monitor_enabled = false;
void doSafety(char* cmd) { safety.commander(cmd); }
void doTarget(char* cmd) { command.motion(&motor, cmd); }
void doMonitor(char* cmd) {
    monitor_enabled = !monitor_enabled;
    if(monitor_enabled) Serial.println("Monitor: ON");
    else Serial.println("Monitor: OFF");
}
void doBootloader(char* cmd) {
    Serial.println("Entering Bootloader...");
    delay(100);
    reset_usb_boot(0, 0);
}

void doDebug(char* cmd) {
    uint32_t gstat = driver.getGSTAT();
    uint32_t drv_status = driver.getDRVSTATUS();
    uint32_t gconf = driver.readRegister(0x00);
    uint32_t ioin = driver.getIOIN();

    Serial.println("--- TMC2240 Debug ---");

    // GSTAT
    Serial.print("GSTAT: 0x"); Serial.println(gstat, HEX);
    if(gstat & (1<<0)) Serial.println("  [0] reset");
    if(gstat & (1<<1)) Serial.println("  [1] drv_err");
    if(gstat & (1<<2)) Serial.println("  [2] uv_cp");
    if(gstat & (1<<11)) Serial.println("  [11] vm_uvlo");

    // DRV_STATUS
    Serial.print("DRV_STATUS: 0x"); Serial.println(drv_status, HEX);
    if(drv_status & (1<<0)) Serial.println("  [0] stst");
    if(drv_status & (1<<1)) Serial.println("  [1] stealth");
    if(drv_status & (1<<24)) Serial.println("  [24] s2ga");
    if(drv_status & (1<<25)) Serial.println("  [25] s2gb");
    if(drv_status & (1<<26)) Serial.println("  [26] ola");
    if(drv_status & (1<<27)) Serial.println("  [27] olb");
    if(drv_status & (1<<28)) Serial.println("  [28] otpw");
    if(drv_status & (1<<29)) Serial.println("  [29] ot");
    if(drv_status & (1<<30)) Serial.println("  [30] stallGuard");

    // GCONF
    Serial.print("GCONF: 0x"); Serial.println(gconf, HEX);
    if(gconf & (1<<0)) Serial.println("  [0] analog_gain");
    if(gconf & (1<<1)) Serial.println("  [1] internal_rsense");
    if(gconf & (1<<2)) Serial.println("  [2] en_spreadCycle");
    if(gconf & (1<<3)) Serial.println("  [3] shaft");
    if(gconf & (1<<4)) Serial.println("  [4] step_interpol");
    if(gconf & (1<<5)) Serial.println("  [5] stop_enable");
    if(gconf & (1<<6)) Serial.println("  [6] direct_mode");
    if(gconf & (1<<7)) Serial.println("  [7] test_mode");

    // IOIN
    Serial.print("IOIN: 0x"); Serial.println(ioin, HEX);
    if(ioin & (1<<0)) Serial.println("  [0] step");
    if(ioin & (1<<1)) Serial.println("  [1] dir");
    if(ioin & (1<<2)) Serial.println("  [2] enc_a");
    if(ioin & (1<<3)) Serial.println("  [3] enc_b");
    if(ioin & (1<<4)) Serial.println("  [4] enc_n");
    if(ioin & (1<<5)) Serial.println("  [5] drv_enn");
    if(ioin & (1<<6)) Serial.println("  [6] uart_en");
    if(ioin & (1<<7)) Serial.println("  [7] clk");
    Serial.print("  Version: 0x"); Serial.println((ioin >> 24) & 0xFF, HEX);
}

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
  command.add('T', doTarget, "Motor Target");
  command.add('M', doMonitor, "Toggle Monitor");
  command.add('B', doBootloader, "Enter Bootloader");
  command.add('D', doDebug, "Decode Registers");

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

    if (monitor_enabled) {
        float angle_deg = sensor.getAngle() * (180.0 / PI);
        uint16_t agc = sensor.readMagnitude();
        bool error = sensor.isErrorFlag();

        float temp = driver.getChipTemperature();
        uint32_t gstat = driver.getGSTAT();
        uint32_t drv_status = driver.getDRVSTATUS();
        uint32_t gconf = driver.readRegister(0x00); // GCONF
        uint32_t ioin = driver.getIOIN();
        uint32_t adc_raw = driver.readRegister(0x51); // Raw ADC

        // Debug Encoder Error
        uint16_t enc_err = 0;
        if(error) {
            // Manual read of ERRFL (0x0001)
            // Command: Read (Bit 14) | Address 0x0001 | Parity (Bit 15)
            // 0x4001 (Binary 0100 0000 0000 0001) -> 2 bits set -> Parity 0 -> 0x4001

            SPISettings enc_settings(1000000, MSBFIRST, SPI_MODE1);
            pio_spi.beginTransaction(enc_settings);
            digitalWrite(ENC_CS, LOW);
            pio_spi.transfer16(0x4001); // Send Command
            digitalWrite(ENC_CS, HIGH);
            pio_spi.endTransaction();

            delayMicroseconds(10);

            pio_spi.beginTransaction(enc_settings);
            digitalWrite(ENC_CS, LOW);
            enc_err = pio_spi.transfer16(0xC000); // Read Result (NOP)
            digitalWrite(ENC_CS, HIGH);
            pio_spi.endTransaction();

            enc_err &= 0x3FFF; // Mask data
        }

        Serial.print("Angle: "); Serial.print(angle_deg); Serial.print(" deg");
        Serial.print(" | AGC: "); Serial.print(agc);
        Serial.print(" | Err: "); Serial.print(error ? "YES" : "NO");
        if(error) { Serial.print(" (0x"); Serial.print(enc_err, HEX); Serial.print(")"); }
        Serial.print(" | Temp: "); Serial.print(temp);
        Serial.print(" | ADC: 0x"); Serial.print(adc_raw, HEX);
        Serial.print(" | GSTAT: 0x"); Serial.print(gstat, HEX);
        Serial.print(" | DRV: 0x"); Serial.print(drv_status, HEX);
        Serial.print(" | GCONF: 0x"); Serial.print(gconf, HEX);
        Serial.print(" | IOIN: 0x"); Serial.println(ioin, HEX);
    }
  }
}
