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

void doEstop(char* cmd) {
    Serial.println("!!! ESTOP TRIGGERED !!!");
    motor.disable();
    driver.disable();
    Serial.println("Motor and Driver Disabled.");
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
  Serial.print("Config: Max Current = "); Serial.print(motor_max_current_ma); Serial.println(" mA");
  Serial.print("Config: Phase Res = "); Serial.print(motor_phase_resistance); Serial.println(" Ohm");

  // Initialize Safety Monitor
  safety.setMotorTempCallback(readMotorTemp);
  safety.init();

  // Initialize Encoder
  pio_spi.begin();
  sensor.init(&pio_spi);

  // Link Encoder to Motor
  motor.linkSensor(&sensor);

  // Calculate Virtual Voltage Limits
  // V = I * R
  float max_voltage = (motor_max_current_ma / 1000.0f) * motor_phase_resistance;

  Serial.print("Config: Virtual Voltage Limit = "); Serial.print(max_voltage); Serial.println(" V");

  // Initialize Driver
  driver.voltage_power_supply = max_voltage;
  driver.voltage_limit = max_voltage;
  driver.init();

  // Link Driver to Motor
  motor.linkDriver(&driver);

  // Configure Motor
  motor.controller = MotionControlType::angle; // Closed Loop Position
  motor.voltage_limit = max_voltage; // Torque Limit
  motor.velocity_limit = 20.0f;
  motor.PID_velocity.P = 0.2f;
  motor.P_angle.P = 20.0f;

  // Initialize Motor
  motor.init();
  motor.initFOC();

  // Add Commander commands
  command.add('S', doSafety, "Safety Monitor");
  command.add('T', doTarget, "Motor Target");
  command.add('M', doMonitor, "Toggle Monitor");
  command.add('B', doBootloader, "Enter Bootloader");
  command.add('D', doDebug, "Decode Registers");
  command.add('E', doEstop, "ESTOP");

  Serial.println("Ready. Mode: Closed Loop Position");
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

  // Run Motor
  motor.move();
  motor.loopFOC();

  // Run Commander
  command.run();

  // Periodic Angle Query
  static unsigned long last_angle_query = 0;
  if (millis() - last_angle_query > 100) {
    last_angle_query = millis();

    if (monitor_enabled) {
        float target = motor.target;
        float current = sensor.getAngle();
        float voltage_q = motor.voltage.q;
        float voltage_d = motor.voltage.d;

        Serial.print("Tgt: "); Serial.print(target, 3);
        Serial.print(" | Cur: "); Serial.print(current, 3);
        Serial.print(" | Vq: "); Serial.print(voltage_q, 2);
        Serial.print(" | Vd: "); Serial.print(voltage_d, 2);
        Serial.print(" | Err: "); Serial.println(sensor.isErrorFlag() ? "YES" : "NO");
    }
  }
}
