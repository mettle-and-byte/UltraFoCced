#ifndef AUTOTUNER_H
#define AUTOTUNER_H

#include <Arduino.h>
#include <SimpleFOC.h>

class AutoTuner {
public:
    AutoTuner(FOCMotor* motor);

    // Start the auto-tuning sequence
    void startTuning();

    // Call this in loop() - returns true when tuning is complete
    bool run();

    // Check if tuning is active
    bool isRunning() { return _state != IDLE; }

    // Get tuning progress (0.0 to 1.0)
    float getProgress();

private:
    enum State {
        IDLE,
        INIT,
        ALIGN_FWD,      // Spin forward
        ALIGN_BRAKE_FWD,// New: Wait for stop after forward
        ALIGN_REV,      // Spin reverse
        ALIGN_BRAKE_REV,// New: Wait for stop after reverse
        ALIGN_CALC,     // Calculate and adjust angle
        MEASURE_BASELINE,
        TUNE_VEL_P_STABILITY, // Phase 1: Find max stable P (Static)
        TUNE_VEL_P,           // Phase 2: Find best tracking P (Dynamic)
        TUNE_VEL_I,
        TUNE_ANG_P,
        COMPLETE
    };

    FOCMotor* _motor;
    State _state;

    // Tuning parameters
    float _p_min, _p_max, _p_step;
    float _i_min, _i_max, _i_step;
    float _current_gain;

    // Alignment parameters
    float _align_voltage;
    float _fwd_velocity;
    float _rev_velocity;
    int _align_iterations;
    static const int MAX_ALIGN_ITERATIONS = 10;

    // Alignment averaging
    double _velocity_sum;
    int _velocity_samples;

    // Newton-Raphson alignment history
    float _prev_diff;
    float _prev_angle;

    // Measurement window
    // Measurement window
    static const int WINDOW_SIZE = 2048;
    static const int SAMPLE_INTERVAL_US = 250; // 4kHz sampling
    static const int STABILITY_DURATION_MS = (WINDOW_SIZE * SAMPLE_INTERVAL_US) / 1000;
    static constexpr float MAX_LPF_VELOCITY = 0.1f;

    // Tuning Thresholds & Constants
    static constexpr float BASELINE_JITTER_THRESHOLD = 0.2f;
    static constexpr float HF_RATIO_THRESHOLD = 0.5f;
    static constexpr float ABS_JITTER_THRESHOLD = 0.5f;
    static constexpr float LPF_MULTIPLIER = 2.0f;
    static constexpr float MIN_NOISE_THRESHOLD = 0.01f;
    static constexpr float NOISE_THRESHOLD_MULTIPLIER = 1.5f;

    static constexpr float VEL_I_TARGET = 2.0f;
    static constexpr float VEL_I_RIPPLE_WEIGHT = 0.5f;
    static constexpr float VEL_I_MAX_STDDEV = 5.0f;

    static constexpr float ANG_P_OFFSET = 0.5f;

    static constexpr float DEGRADATION_THRESH_P = 1.2f;   // 20%
    static constexpr float DEGRADATION_THRESH_I = 1.5f;   // 50%
    static constexpr float DEGRADATION_THRESH_ANG = 1.1f; // 10%

    // Alignment
    static constexpr float ALIGN_VOLTAGE = 2.0f;

    // Velocity P Tuning
    static constexpr float VEL_P_INIT_MIN = 0.05f;
    static constexpr float VEL_P_INIT_MAX = 2.0f;
    static constexpr float VEL_P_ABS_MAX = 10.0f;
    static constexpr float VEL_P_STEP = 0.05f;

    // Velocity I Tuning
    static constexpr float VEL_I_MIN = 0.0f;
    static constexpr float VEL_I_MAX = 50.0f;
    static constexpr float VEL_I_STEP = 2.5f;

    // Angle P Tuning
    static constexpr float ANG_P_MIN = 0.0f;
    static constexpr float ANG_P_MAX = 20.0f;
    static constexpr float ANG_P_STEP = 0.5f;
    static constexpr float ANG_P_MAX_STDDEV = 1.0f;

    float _velocity_buffer[WINDOW_SIZE];
    float _velocity_change_buffer[WINDOW_SIZE];  // Track velocity changes for HF noise
    int _sample_count;
    unsigned long _last_sample_time;
    float _prev_velocity;  // For calculating velocity changes (deprecated for noise, used for buffer?)
    float _prev_shaft_velocity; // Always tracks velocity for noise calculation
    float _start_angle;    // Reference for Angle tuning

    // Original PID values (to restore if needed)
    float _original_vel_p, _original_vel_i;
    float _original_ang_p;
    uint16_t _original_downsample;

    // Results
    float _baseline_jitter;
    float _best_vel_p, _best_vel_i;
    float _best_ang_p;

    // Optimization state
    float _min_score;
    float _last_score;

    // Helper functions
    float measureJitter(); // Keeps existing for baseline
    float calculateMetrics(float& avg_error, float& error_std_dev); // New: Calculates error stats
    float measureHFNoise();
    void storeSample(float value);
    void setState(State new_state);
    void applyGains(float p, float i);
    void printStatus(const char* message);
};

#endif // AUTOTUNER_H
