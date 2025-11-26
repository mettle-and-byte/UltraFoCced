#include "Core0.h"
#include "Shared.h"

// Helper for SafetyMonitor
float readMotorTemp() {
    return driver.getChipTemperature();
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

    // Initialize Driver
    driver.voltage_power_supply = max_voltage;
    driver.voltage_limit = max_voltage;
    driver.init();

    // Link Driver to Motor
    motor.linkDriver(&driver);

    // Configure Motor
    motor.controller = MotionControlType::angle; // Closed Loop Position
    motor.voltage_limit = max_voltage; // Torque Limit
    motor.current_limit = motor_max_current_ma / 1000.0f;
    motor.velocity_limit = 100.0f;
    motor.motion_downsample = 4.0;

    motor.PID_velocity.P = 0.25;
    motor.PID_velocity.I = 5.0;
    motor.PID_velocity.D = 0.0;
    motor.PID_velocity.output_ramp = 500.0;
    motor.PID_velocity.limit = 20.0;
    motor.LPF_velocity.Tf = 0.1;

    motor.phase_resistance = motor_phase_resistance;

    motor.useMonitoring(serial_stream);
    motor.monitor_downsample = 1000;

    // Initialize Safety Monitor
    safety.setMotorTempCallback(readMotorTemp);
    safety.init();

    // Initialize Motor
    motor.init();
    motor.initFOC();

    serial_stream.println("Ready. Mode: Closed Loop Position");
}

void Core0::loop() {
    // Run Safety Monitor
    safety.run();

    // Run Motor
    motor.loopFOC();
    motor.move();

    motor.monitor();
}
