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
    serial_stream.print("Config: Phase Resistance = "); serial_stream.println(motor_phase_resistance);

    // Initialize Driver
    driver.voltage_power_supply = max_voltage;
    driver.voltage_limit = max_voltage;
    driver.init();

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
    motor.PID_velocity.limit = motor.current_limit; // Limit PID output to physical current limit


    delay(5000);

    motor.useMonitoring(serial_stream);
    motor.monitor_downsample = 2000;

    // Initialize Safety Monitor
    safety.setMotorTempCallback(readMotorTemp);
    safety.init();

    // Initialize Motor
    motor.init();
    motor.LPF_velocity.Tf = 0.05f; // Increase LPF to filter noise
    motor.initFOC();

}

void Core0::loop() {
    // Run Safety Monitor
    safety.run();

    // Run Motor
    motor.loopFOC();
    motor.move();

    // Run Auto-Tuner (if active)
    autotuner.run();

    motor.monitor();
}
