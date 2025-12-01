#include "Shared.h"

// Motor Parameters
float motor_phase_resistance = 2.0f;
float motor_phase_inductance = 3.3f;
float motor_kv = 592.0f;
int motor_max_current_ma = 2500;
int motor_pole_pairs = 50;

// Global Flags
bool monitor_enabled = false;

// Global Objects
QueueStream serial_stream(2048); // 2KB buffer

SafetyMonitor safety(FAN_PIN, ENDSTOP_PIN);

SimplePioSPI pio_spi(ENC_MOSI, ENC_MISO, ENC_SCK, DUMMY_CS, pio0, 0);

MagneticSensorAS5047 sensor = MagneticSensorAS5047(ENC_CS);

TMC2240Driver driver(DRV_CS, DRV_EN, DRV_UART_EN, DRV_MISO, DRV_MOSI, DRV_SCK, TMC2240_RREF, motor_phase_resistance, motor_max_current_ma);

StepperMotor motor = StepperMotor(motor_pole_pairs);

// Commander
Commander command = Commander(serial_stream);

// Auto-Tuner
AutoTuner autotuner(&motor);
