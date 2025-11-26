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

    motor.useMonitoring(serial_stream);
    motor.monitor_downsample = 1000;

    // Initialize Safety Monitor
    safety.setMotorTempCallback(readMotorTemp);
    safety.init();

    // Initialize Motor
    motor.init();
    motor.initFOC();
}

void Core0::loop() {
    // Run Safety Monitor
    safety.run();

    // Run Motor
    motor.loopFOC();
    motor.move();

    motor.monitor();
}
