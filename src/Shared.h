#ifndef SHARED_H
#define SHARED_H

#include <Arduino.h>
#include <SimpleFOC.h>
#include <SimplePioSPI.h>
#include "encoders/as5047/MagneticSensorAS5047.h"
#include "TMC2240Driver.h"
#include "QueueStream.h"
#include "TMC2240Driver.h"
// #include "TMC2240CurrentSense.h"
#include "AutoTuner.h"
#include "SafetyMonitor.h"
#include "BoardConfig.h"

// Global Objects
extern QueueStream serial_stream;
extern SimplePioSPI pio_spi;
extern StepperMotor motor;
extern TMC2240Driver driver;
extern MagneticSensorAS5047 sensor;
extern Commander command;
extern SafetyMonitor safety;
extern AutoTuner autotuner;

// Global Flags
extern bool monitor_enabled;

// Motor Parameters (Configurable)
extern float motor_phase_resistance;
extern float motor_phase_inductance;
extern float motor_kv;
extern int motor_max_current_ma;
extern int motor_pole_pairs;

#endif // SHARED_H
