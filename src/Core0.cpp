#include "Core0.h"
#include "Shared.h"
#include <pico/time.h>

// FOC Timer Configuration
const uint32_t FOC_FREQUENCY_KHZ = 25;  // 25kHz
const int32_t FOC_TIMER_PERIOD_US = -(1000000 / (FOC_FREQUENCY_KHZ * 1000));  // -40us

// Global timer handle
static struct repeating_timer foc_timer;

// FOC callback - runs at 25kHz in hard real-time
bool foc_timer_callback(struct repeating_timer *t) {
    motor.move();      // Motion control first
    motor.loopFOC();   // Then FOC algorithm
    return true;       // Keep repeating
}

// Helper for SafetyMonitor
float readMotorTemp() {
    // Disable interrupts to prevent FOC ISR from colliding with this SPI transaction
    noInterrupts();
    float temp = driver.getChipTemperature();
    interrupts();
    return temp;
}

// --- Setup & Loop ---

void Core0::setup() {
    // Initialize QueueStream
    serial_stream.begin();

    // Initialize Encoder
    pio_spi.begin();
    sensor.init(&pio_spi);

    // Link Encoder to Motor
    motor.linkSensor(&sensor);

    // Calculate Virtual Voltage Limits
    float max_voltage = (motor_max_current_ma / 1000.0f) * motor_phase_resistance;

    serial_stream.print("Config: Virtual Voltage Limit = "); serial_stream.print(max_voltage); serial_stream.println(" V");
    serial_stream.print("Config: Phase Resistance = "); serial_stream.println(motor_phase_resistance);

    driver.setMotorConfig(motor_phase_resistance, motor_max_current_ma);
    driver.voltage_power_supply = max_voltage;
    driver.voltage_limit = max_voltage;

    driver.init();    // Initialize Driver


    // Link Driver to Motor
    motor.linkDriver(&driver);

    // Configure Motor
    motor.phase_resistance = motor_phase_resistance;

    /*
    motor.phase_inductance = motor_phase_inductance;
    motor.KV_rating = motor_kv;*/

    motor.controller = MotionControlType::angle; // Closed Loop Position
    motor.torque_controller = TorqueControlType::voltage; // Voltage Control (FOC)
    motor.voltage_limit = max_voltage; // Torque Limit
    motor.current_limit = motor_max_current_ma / 1000.0f; // Current Limit (Virtual)

    // Velocity PID
    motor.PID_velocity.P = 0.5;
    motor.PID_velocity.I = 10.0;
    motor.PID_velocity.D = 0.0;
    motor.PID_velocity.output_ramp = 1000.0;
    motor.PID_velocity.limit = max_voltage;
    motor.LPF_velocity.Tf = 0.1;

    // Angle PID
    motor.P_angle.P = 20.0;
    motor.P_angle.I = 0.0;
    motor.P_angle.D = 0.0;
    motor.P_angle.output_ramp = 0.0;
    motor.P_angle.limit = 200.0; // Max Velocity
    motor.LPF_angle.Tf = 0.0;

    motor.phase_resistance = motor_phase_resistance;


    delay(5000);

    motor.useMonitoring(serial_stream);
    motor.monitor_downsample = 2000;

    // Initialize Safety Monitor
    safety.setMotorTempCallback(readMotorTemp);
    safety.init();

    // Initialize Motor
    if(motor.init() == false) {
        Serial.println("Motor init failed");
    }
    if(motor.initFOC() == false) {
        Serial.println("FOC init failed");
    }

    // Start FOC timer
    if (!add_repeating_timer_us(FOC_TIMER_PERIOD_US, foc_timer_callback, NULL, &foc_timer)) {
        serial_stream.println("ERROR: Failed to start FOC timer!");
        return;
    }

    serial_stream.print("FOC timer started at ");
    serial_stream.print(FOC_FREQUENCY_KHZ);
    serial_stream.println("kHz");
}

void Core0::loop() {
    // Run Safety Monitor
    safety.run();

    // Run Auto-Tuner (if active)
    autotuner.run();

    // Motor monitoring
    motor.monitor();
}
