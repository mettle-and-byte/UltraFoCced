#include "AutoTuner.h"
#include <math.h>

AutoTuner::AutoTuner(FOCMotor* motor)
    : _motor(motor), _state(IDLE), _sample_count(0), _last_sample_time(0), _prev_velocity(0.0) {
}

void AutoTuner::startTuning() {
    // Do not start tuning if the motor is not enabled
    if (!_motor->enabled) {
        Serial.println("AutoTuner: Motor is not enabled!");
        return;
    }

    if (_state != IDLE) {
        Serial.println("AutoTuner: Already running!");
        return;
    }

    // Store original values
    _original_vel_p = _motor->PID_velocity.P;
    _original_vel_i = _motor->PID_velocity.I;
    _original_ang_p = _motor->P_angle.P;

    printStatus("Starting auto-tune sequence...");
    setState(INIT);
}

bool AutoTuner::run() {
    if (_state == IDLE) {
        return false;
    }

    unsigned long now = millis();

    switch (_state) {
        case INIT:
            // Reset measurement buffer
            _sample_count = 0;
            _last_sample_time = now;

            // Disable motor monitoring but save downsample value
            _original_downsample = _motor->monitor_downsample;
            _motor->monitor_downsample = 0;

            // Start with conservative gains
            _motor->PID_velocity.P = 0.05;
            _motor->PID_velocity.I = 0.0;

            // Set search ranges
            _p_min = 0.05;
            _p_max = 2.0;
            _p_step = 0.05;
            _current_gain = _p_min;

            // Setup alignment
            _align_voltage = 2.0; // 2V test voltage
            _align_iterations = 0;

            // Reset LPF to default to ensure consistent starting point
            _motor->LPF_velocity.Tf = 0.01;
            _motor->LPF_angle.Tf = 0.01;

            // Reset PID internals to prevent "runaway" from previous aborts
            _motor->PID_velocity.reset();
            _motor->P_angle.reset();

            _start_angle = 0.0;

            // Switch to voltage mode for alignment (bypasses PIDs!)
            _motor->torque_controller = TorqueControlType::voltage;
            _motor->controller = MotionControlType::torque;

            printStatus("Starting Electrical Angle Alignment...");
            setState(ALIGN_FWD);
            break;

        case ALIGN_FWD: {
            // Spin forward
            _motor->target = _align_voltage;
            unsigned long elapsed = now - _last_sample_time;

            // Phase 1: Settling (0-1000ms) - Ignore acceleration and ringing
            if (elapsed < 1000) {
                _velocity_sum = 0;
                _velocity_samples = 0;
                return false;
            }

            // Phase 2: Averaging (1000-2000ms)
            float vel = _motor->shaft_velocity;

            // Check for wrong direction (180 degree error) - Fast Fail
            if (vel < -0.5) {
                Serial.println("Alignment: Motor spinning backwards! Flipping electrical angle...");
                _motor->zero_electric_angle += _PI;
                _motor->zero_electric_angle = _normalizeAngle(_motor->zero_electric_angle);
                _align_iterations = 0;
                _last_sample_time = now;
                return false;
            }

            _velocity_sum += abs(vel);
            _velocity_samples++;

            if (elapsed >= 2000) {
                if (_velocity_samples > 0) {
                    _fwd_velocity = _velocity_sum / _velocity_samples;
                } else {
                    _fwd_velocity = abs(vel); // Fallback
                }
                _last_sample_time = now;
                setState(ALIGN_BRAKE_FWD);
            }
            break;
        }

        case ALIGN_BRAKE_FWD:
            // Brake (0V)
            _motor->target = 0;

            // Wait for stop (vel < 0.1) OR timeout (2s)
            if (abs(_motor->shaft_velocity) < 0.1 || (now - _last_sample_time > 2000)) {
                // Add a small buffer after stop detection
                if (now - _last_sample_time > 200) {
                    _last_sample_time = now;
                    setState(ALIGN_REV);
                }
            }
            break;

        case ALIGN_REV: {
            // Spin reverse
            _motor->target = -_align_voltage;
            unsigned long elapsed = now - _last_sample_time;

            // Phase 1: Settling (0-1000ms)
            if (elapsed < 1000) {
                _velocity_sum = 0;
                _velocity_samples = 0;
                return false;
            }

            // Phase 2: Averaging (1000-2000ms)
            float vel = _motor->shaft_velocity;

            // Check for wrong direction
            if (vel > 0.5) {
                 Serial.println("Alignment: Motor spinning backwards (in reverse)! Flipping electrical angle...");
                _motor->zero_electric_angle += _PI;
                _motor->zero_electric_angle = _normalizeAngle(_motor->zero_electric_angle);
                _align_iterations = 0;
                _last_sample_time = now;
                setState(ALIGN_FWD);
                return false;
            }

            _velocity_sum += abs(vel);
            _velocity_samples++;

            if (elapsed >= 2000) {
                if (_velocity_samples > 0) {
                    _rev_velocity = _velocity_sum / _velocity_samples;
                } else {
                    _rev_velocity = abs(vel);
                }
                _last_sample_time = now;
                setState(ALIGN_BRAKE_REV);
            }
            break;
        }

        case ALIGN_BRAKE_REV:
            // Brake (0V)
            _motor->target = 0;

            // Wait for stop (vel < 0.1) OR timeout (2s)
            if (abs(_motor->shaft_velocity) < 0.1 || (now - _last_sample_time > 2000)) {
                 if (now - _last_sample_time > 200) {
                    _last_sample_time = now;
                    setState(ALIGN_CALC);
                 }
            }
            break;

        case ALIGN_CALC: {
            float diff = _fwd_velocity - _rev_velocity;
            float avg_speed = (_fwd_velocity + _rev_velocity) / 2.0;

            Serial.print("Iter "); Serial.print(_align_iterations);
            Serial.print(": Fwd="); Serial.print(_fwd_velocity);
            Serial.print(" Rev="); Serial.print(_rev_velocity);
            Serial.print(" Diff="); Serial.println(diff);

            // Check for convergence (diff < 2% of speed)
            if (abs(diff) < avg_speed * 0.02 || _align_iterations >= MAX_ALIGN_ITERATIONS) {
                printStatus("Alignment complete!");
                _motor->target = 0;

                // Switch to VELOCITY mode for baseline and velocity tuning
                // This prevents "snap back" issues if we were in angle mode
                _motor->controller = MotionControlType::velocity;
                _motor->torque_controller = TorqueControlType::voltage;

                printStatus("Measuring baseline jitter (Velocity Mode)...");
                _sample_count = 0;
                _last_sample_time = now;
                delay(2000); // Wait for motor to settle completely
                setState(MEASURE_BASELINE);
            } else {
                // Adjust zero_electric_angle using Newton-Raphson
                float adjustment = 0.0;
                float current_angle = _motor->zero_electric_angle;

                if (_align_iterations == 0) {
                    // First iteration: Use Proportional Gain (Probe)
                    // Gain: 0.01 rad per rad/s difference
                    adjustment = diff * 0.01;
                } else {
                    // Subsequent iterations: Use Newton-Raphson
                    // Slope = d(Diff) / d(Angle)
                    float d_diff = diff - _prev_diff;
                    float d_angle = current_angle - _prev_angle;

                    if (abs(d_angle) > 0.0001) { // Avoid divide by zero
                        float slope = d_diff / d_angle;
                        // We want Diff = 0
                        // 0 = Diff + Slope * (TargetAngle - CurrentAngle)
                        // TargetAngle - CurrentAngle = -Diff / Slope
                        adjustment = -diff / slope;
                    } else {
                        // Fallback to P-gain if angle didn't change (shouldn't happen)
                        adjustment = diff * 0.01;
                    }
                }

                // Clamp adjustment to prevent wild swings (Safety)
                // Max step: 0.2 rad (~11.4 degrees)
                adjustment = constrain(adjustment, -0.2f, 0.2f);

                // Store history for next iteration
                _prev_diff = diff;
                _prev_angle = current_angle;

                _motor->zero_electric_angle += adjustment;

                Serial.print("Adjusting angle by ");
                Serial.println(adjustment, 4);

                _align_iterations++;
                _last_sample_time = now;
                setState(ALIGN_FWD);
            }
            break;
        }

        case MEASURE_BASELINE: {
             // Collect samples (250us interval)
            unsigned long now_us = micros();
            if (now_us - _last_sample_time >= SAMPLE_INTERVAL_US) {
                storeSample(_motor->shaft_velocity);
                _last_sample_time = now_us;

                if (_sample_count >= WINDOW_SIZE) {
                    _baseline_jitter = measureJitter();
                    float hf_noise = measureHFNoise();
                    float hf_ratio = (_baseline_jitter > 0.0001) ? hf_noise / _baseline_jitter : 0.0;

                    Serial.print("Baseline jitter: ");
                    Serial.print(_baseline_jitter, 4);
                    Serial.print(" HF noise: ");
                    Serial.print(hf_noise, 4);
                    Serial.print(" HF ratio: ");
                    Serial.println(hf_ratio, 2);

                    // Safety check: If baseline is huge, we are moving or unstable
                    // Strict threshold to catch bad electrical angle alignment
                    if (_baseline_jitter > BASELINE_JITTER_THRESHOLD) {
                        Serial.print("ERROR: Baseline jitter too high (> ");
                        Serial.print(BASELINE_JITTER_THRESHOLD);
                        Serial.println(" rad/s)!");
                        Serial.println("Possible causes: Bad electrical angle alignment, unstable motor, or excessive noise.");
                        Serial.println("Aborting tuning. Please check motor and try again.");
                        setState(IDLE);
                        return true;
                    }

                    // Auto-tune LPF:
                    // Increase if HF noise is significant (>50% of baseline) OR if absolute jitter is too high (>0.5)
                    if (hf_ratio > HF_RATIO_THRESHOLD || _baseline_jitter > ABS_JITTER_THRESHOLD) {
                        float current_lpf = _motor->LPF_velocity.Tf;

                        // If we are already at max LPF and still noisy, we might need to abort or just accept it
                        if (current_lpf >= MAX_LPF_VELOCITY) {
                             Serial.print("Warn: LPF at max (");
                             Serial.print(MAX_LPF_VELOCITY);
                             Serial.println(") but jitter still high.");
                             // Proceed to Stability Tuning anyway
                             _sample_count = 0;
                             printStatus("Phase 1: Tuning Velocity P (Stability)...");
                             _current_gain = _p_min;
                             _best_vel_p = _p_min;
                             _last_score = 1000.0; // Reset score
                             setState(TUNE_VEL_P_STABILITY);
                        } else {
                            // Increase LPF
                            _motor->LPF_velocity.Tf *= LPF_MULTIPLIER;
                            _motor->LPF_angle.Tf *= LPF_MULTIPLIER;

                            // Constrain LPF to reasonable range
                            if (_motor->LPF_velocity.Tf > MAX_LPF_VELOCITY) _motor->LPF_velocity.Tf = MAX_LPF_VELOCITY;
                            if (_motor->LPF_angle.Tf > MAX_LPF_VELOCITY) _motor->LPF_angle.Tf = MAX_LPF_VELOCITY;

                            Serial.print("Noise detected (Jitter=");
                            Serial.print(_baseline_jitter, 2);
                            Serial.print("). Increasing LPF from ");
                            Serial.print(current_lpf, 4);
                            Serial.print(" to ");
                            Serial.println(_motor->LPF_velocity.Tf, 4);

                            // Re-measure with new LPF
                            _sample_count = 0;
                            _last_sample_time = now;
                            delay(500);  // Let filter settle
                        }
                    } else {
                        // LPF is good, proceed to Stability Tuning
                        _sample_count = 0;
                        printStatus("Phase 1: Tuning Velocity P (Stability)...");
                        _current_gain = _p_min;
                        _best_vel_p = _p_min;
                        _last_score = 1000.0; // Reset score
                        setState(TUNE_VEL_P_STABILITY);
                    }
                }
            }
            break;
        }

        case TUNE_VEL_P_STABILITY: {
            // Phase 1: Static Stability Test (Target = 0)
            // Goal: Find the maximum P gain before HF noise (buzzing) becomes excessive.
            _motor->PID_velocity.P = _current_gain;
            _motor->PID_velocity.I = 0;
            _motor->target = 0; // Hold zero

            // Collect samples (250us interval)
            unsigned long now_us = micros();
            if (now_us - _last_sample_time >= SAMPLE_INTERVAL_US) {
                storeSample(_motor->shaft_velocity);
                _last_sample_time = now_us;

                if (_sample_count >= WINDOW_SIZE) {
                    // Analyze HF Noise
                    float hf_noise = measureHFNoise();

                    // Thresholds
                    // If HF noise > 2x Baseline (or absolute threshold), we are unstable.
                    float noise_threshold = max(_baseline_jitter * NOISE_THRESHOLD_MULTIPLIER, MIN_NOISE_THRESHOLD);

                    Serial.print("P="); Serial.print(_current_gain);
                    Serial.print(" HF_Noise="); Serial.print(hf_noise, 4);
                    Serial.print(" Thresh="); Serial.println(noise_threshold, 4);

                    if (hf_noise > noise_threshold) {
                        Serial.println("Instability detected (Buzzing). Stopping Stability Phase.");
                        // Back off slightly to be safe
                        _p_max = max(_current_gain - _p_step, _p_min);

                        // Proceed to Phase 2
                        printStatus("Phase 2: Tuning Velocity P (Performance)...");
                        _current_gain = _p_min;
                        _best_vel_p = _p_min;
                        _min_score = 10000.0;
                        _sample_count = 0;
                        setState(TUNE_VEL_P);
                        return true;
                    }

                    // Next Step
                    _current_gain += _p_step;
                    _sample_count = 0;

                    // Stop if we hit safety limit
                    if (_current_gain > VEL_P_ABS_MAX) { // Hard limit
                         _p_max = VEL_P_ABS_MAX;
                         printStatus("Phase 2: Tuning Velocity P (Performance)...");
                         _current_gain = _p_min;
                         _best_vel_p = _p_min;
                         _min_score = 10000.0;
                         _sample_count = 0;
                         setState(TUNE_VEL_P);
                    }
                }
            }
            break;
        }

        case TUNE_VEL_P: {
            // Phase 2: Dynamic Performance Test (Square Wave)
            // Goal: Find best tracking error within the stable range found in Phase 1.

            // Generate excitation (Square wave)
            unsigned long time_in_step = now - _last_sample_time;
            float target = ((time_in_step / 200) % 2 == 0) ? 0.3 : -0.3; // 200ms period
            _motor->target = target;
            _motor->PID_velocity.P = _current_gain;
            _motor->PID_velocity.I = 0;

            storeSample(_motor->shaft_velocity);

            if (_sample_count >= WINDOW_SIZE) {
                float avg_error, error_std_dev;
                float score = calculateMetrics(avg_error, error_std_dev);

                Serial.print("P="); Serial.print(_current_gain);
                Serial.print(" AvgErr="); Serial.print(avg_error, 4);
                Serial.print(" StdDev="); Serial.print(error_std_dev, 4);
                Serial.print(" Score="); Serial.println(score, 4);

                // Optimization Logic (U-Curve)
                if (score < _min_score) {
                    _min_score = score;
                    _best_vel_p = _current_gain;
                } else if (score > _min_score * DEGRADATION_THRESH_P) { // Degradation check
                    Serial.println("Stopping: Score degrading.");
                    // Done with P tuning
                    Serial.print("Applying Best Vel P: "); Serial.println(_best_vel_p);
                    _motor->PID_velocity.P = _best_vel_p;
                    printStatus("Tuning Velocity I gain...");
                    _current_gain = VEL_I_MIN;
                    _i_step = VEL_I_STEP; // Initialize I step
                    _i_max = VEL_I_MAX;
                    _min_score = 10000.0;
                    _sample_count = 0;
                    setState(TUNE_VEL_I);
                    return true;
                }

                // Next Step
                _current_gain += _p_step;
                _sample_count = 0;

                // Stop if we hit the limit found in Phase 1
                if (_current_gain > _p_max) {
                    Serial.println("Stopping: Reached stability limit.");
                    Serial.print("Applying Best Vel P: "); Serial.println(_best_vel_p);
                    _motor->PID_velocity.P = _best_vel_p;
                    printStatus("Tuning Velocity I gain...");
                    _current_gain = VEL_I_MIN;
                    _i_step = VEL_I_STEP; // Initialize I step
                    _i_max = VEL_I_MAX;
                    _min_score = 10000.0;
                    _sample_count = 0;
                    setState(TUNE_VEL_I);
                }
            }
            break;
        }

        case TUNE_VEL_I: {
            // Tuning Velocity I gain
            // Goal: Minimize steady-state error and velocity ripple.
            // Strategy: Run at constant velocity and measure variance.

            _motor->target = VEL_I_TARGET; // Constant target
            _motor->PID_velocity.P = _best_vel_p;
            _motor->PID_velocity.I = _current_gain;

            // Collect samples (250us interval)
            unsigned long now_us = micros();
            if (now_us - _last_sample_time >= SAMPLE_INTERVAL_US) {
                storeSample(_motor->shaft_velocity);
                _last_sample_time = now_us;

                if (_sample_count >= WINDOW_SIZE) {
                    // Calculate Ripple (StdDev) and Mean Error
                    float avg_error, error_std_dev;
                    // We manually calculate metrics here because calculateMetrics assumes square wave

                    float sum_error = 0.0;
                    float sum_val = 0.0;
                    for(int i=0; i<_sample_count; i++) {
                        sum_error += abs(_velocity_buffer[i] - VEL_I_TARGET);
                        sum_val += _velocity_buffer[i];
                    }
                    avg_error = sum_error / _sample_count;
                    float mean_val = sum_val / _sample_count;

                    float sum_sq_diff = 0.0;
                    for(int i=0; i<_sample_count; i++) {
                        float diff = _velocity_buffer[i] - mean_val;
                        sum_sq_diff += diff * diff;
                    }
                    error_std_dev = sqrt(sum_sq_diff / _sample_count);

                    // Score: We want low error (I term fixes this) and low ripple.
                    // Weight error higher than ripple for I term.
                    float score = avg_error + (error_std_dev * VEL_I_RIPPLE_WEIGHT);

                    Serial.print("I="); Serial.print(_current_gain);
                    Serial.print(" AvgErr="); Serial.print(avg_error, 4);
                    Serial.print(" StdDev="); Serial.print(error_std_dev, 4);
                    Serial.print(" Score="); Serial.println(score, 4);

                    if (score < _min_score) {
                        _min_score = score;
                        _best_vel_i = _current_gain;
                    }

                    // We do NOT stop for degradation anymore, we scan the full range.
                    // Unless instability is massive (safety check)
                    if (error_std_dev > VEL_I_MAX_STDDEV) { // Massive oscillation
                         Serial.println("Stopping: Instability detected.");
                         Serial.print("Applying Best Vel I: "); Serial.println(_best_vel_i);
                         _motor->PID_velocity.I = _best_vel_i;
                         _motor->target = 0;
                         printStatus("Tuning Angle P gain...");
                         _current_gain = ANG_P_MIN; // Start Angle P from 0
                         _p_step = ANG_P_STEP;       // Larger steps for Angle P
                         _p_max = ANG_P_MAX;
                         _min_score = 10000.0;
                         _sample_count = 0;
                         // Switch to Angle Mode
                         _motor->controller = MotionControlType::angle;
                         _start_angle = _motor->shaft_angle;
                         setState(TUNE_ANG_P);
                         return true;
                    }

                    _current_gain += _i_step;
                    _sample_count = 0;

                    if (_current_gain > _i_max) {
                         Serial.print("Applying Best Vel I: "); Serial.println(_best_vel_i);
                         _motor->PID_velocity.I = _best_vel_i;
                         _motor->target = 0;
                         printStatus("Tuning Angle P gain...");
                         _current_gain = ANG_P_MIN;
                         _p_step = ANG_P_STEP;
                         _p_max = ANG_P_MAX;
                         _min_score = 10000.0;
                         _sample_count = 0;
                         _motor->controller = MotionControlType::angle;
                         _start_angle = _motor->shaft_angle;
                         setState(TUNE_ANG_P);
                    }
                }
            }
            break;
        }

        case TUNE_ANG_P: {
            // Collect samples at current angle P
            unsigned long now_us = micros();
            if (now_us - _last_sample_time >= SAMPLE_INTERVAL_US) {
                // Capture start angle on first sample
                if (_sample_count == 0) _start_angle = _motor->shaft_angle;

                storeSample(_motor->shaft_angle);
                _last_sample_time = now_us;

                // Add larger movements for angle loop testing (±0.5 rad)
                if (_sample_count % 20 == 0) {
                    float offset = ((_sample_count / 20) % 2 == 0) ? ANG_P_OFFSET : -ANG_P_OFFSET;
                    _motor->target = _start_angle + offset;
                }

                if (_sample_count >= WINDOW_SIZE) {
                    float avg_error, error_std_dev;
                    // Use the full score which includes HF noise!
                    float score = calculateMetrics(avg_error, error_std_dev);
                    float hf_noise = measureHFNoise();

                    // Return to start
                    _motor->target = _start_angle;

                    Serial.print("Angle P=");
                    Serial.print(_current_gain, 1);
                    Serial.print(" AvgErr="); Serial.print(avg_error, 4);
                    Serial.print(" StdDev="); Serial.print(error_std_dev, 4);
                    Serial.print(" HF="); Serial.print(hf_noise, 4);
                    Serial.print(" Score="); Serial.println(score, 4);

                    // Initialize min_score
                    if (_current_gain == ANG_P_MIN + ANG_P_STEP) { // First valid gain
                        _min_score = score;
                        _best_ang_p = _current_gain;
                    }

                    // Check for improvement
                    if (score < _min_score) {
                        _min_score = score;
                        _best_ang_p = _current_gain;
                    }

                    // Stop conditions
                    bool degradation = score > _min_score * DEGRADATION_THRESH_ANG;
                    bool instability = error_std_dev > ANG_P_MAX_STDDEV;
                    // Check for buzzing (HF Noise)
                    // Use a threshold similar to Vel P stability (e.g., 0.02 or 0.05)
                    bool buzzing = hf_noise > 0.05f;

                    if (degradation || instability || buzzing) {
                         Serial.print("Stopping: ");
                         if (instability) Serial.println("Instability detected!");
                         else if (buzzing) Serial.println("Buzzing detected!");
                         else Serial.println("Score degrading.");

                        Serial.print("Applying Best Angle P: "); Serial.println(_best_ang_p);
                        _motor->P_angle.P = _best_ang_p;

                        printStatus("Auto-tune complete!");
                        setState(COMPLETE);
                    } else {
                        // Try next P gain
                        _current_gain += ANG_P_STEP;

                        if (_current_gain > ANG_P_MAX) {
                            // Hit max
                            Serial.print("Applying Best Angle P (limit): "); Serial.println(_best_ang_p);
                            _motor->P_angle.P = _best_ang_p;

                            printStatus("Auto-tune complete!");
                            setState(COMPLETE);
                        } else {
                            _motor->P_angle.P = _current_gain;
                            _sample_count = 0;
                        }
                    }
                }
            }
            break;
        }

        case COMPLETE: {
            // Apply final tuned values (already set during tuning, but make it explicit)
            _motor->PID_velocity.P = _best_vel_p;
            _motor->PID_velocity.I = _best_vel_i;
            _motor->P_angle.P = _best_ang_p;

            // Keep the LPF values determined during baseline measurement
            // Do NOT recalculate based on P gain, as this leads to absurdly high time constants for low P.
            float best_lpf_vel = _motor->LPF_velocity.Tf;
            float best_lpf_ang = _motor->LPF_angle.Tf;

            Serial.println("=== TUNING RESULTS ===");
            Serial.print("Velocity P: "); Serial.println(_best_vel_p, 2);
            Serial.print("Velocity I: "); Serial.println(_best_vel_i, 1);
            Serial.print("Velocity LPF: "); Serial.println(best_lpf_vel, 4);
            Serial.print("Angle P: "); Serial.println(_best_ang_p, 1);
            Serial.print("Angle LPF: "); Serial.println(best_lpf_ang, 4);
            Serial.print("Zero Angle: "); Serial.println(_motor->zero_electric_angle, 4);
            Serial.println("======================");
            Serial.println("Values applied and motor is now using tuned parameters!");
            setState(IDLE);

            // Restore motor monitoring
            _motor->monitor_downsample = _original_downsample;
            return true;
        }

        default:
            break;
    }

    return false;
}

float AutoTuner::measureJitter() {
    if (_sample_count == 0) return 0.0;

    // Calculate mean
    float sum = 0.0;
    for (int i = 0; i < _sample_count; i++) {
        sum += _velocity_buffer[i];
    }
    float mean = sum / _sample_count;

    // Calculate variance
    float variance = 0.0;
    for (int i = 0; i < _sample_count; i++) {
        float diff = _velocity_buffer[i] - mean;
        variance += diff * diff;
    }
    variance /= _sample_count;

    // Return standard deviation
    return sqrt(variance);
}

float AutoTuner::measureHFNoise() {
    if (_sample_count <= 1) return 0.0;

    // Calculate mean of velocity changes
    float sum = 0.0;
    int count = _sample_count - 1;  // We have one less change than samples
    for (int i = 0; i < count; i++) {
        sum += _velocity_change_buffer[i];
    }
    float mean = sum / count;

    // Calculate variance of velocity changes
    float variance = 0.0;
    for (int i = 0; i < count; i++) {
        float diff = _velocity_change_buffer[i] - mean;
        variance += diff * diff;
    }
    variance /= count;

    // Return standard deviation (HF noise level)
    return sqrt(variance);
}

float AutoTuner::calculateMetrics(float& avg_error, float& error_std_dev) {
    if (_sample_count == 0) {
        avg_error = 0.0;
        error_std_dev = 0.0;
        return 1000.0;
    }

    // Calculate Average Absolute Error (MAE)
    float sum_error = 0.0;
    for (int i = 0; i < _sample_count; i++) {
        float target = 0.0;
        if (_state == TUNE_VEL_P || _state == TUNE_VEL_I) {
            target = ((i / 20) % 2 == 0) ? 0.3 : -0.3;
        } else if (_state == TUNE_ANG_P) {
             target = _start_angle + (((i / 20) % 2 == 0) ? 0.5 : -0.5);
        }

        // _velocity_buffer holds VELOCITY in Vel mode, and POSITION in Ang mode
        float error = abs(_velocity_buffer[i] - target);
        sum_error += error;
    }
    avg_error = sum_error / _sample_count;

    // Calculate Standard Deviation of Error (Stability)
    // We want the deviation of the ERROR, not the raw signal
    // But since target is square wave, simple stddev of signal is wrong.
    // We need stddev of (Signal - Target).

    float sum_sq_diff = 0.0;
    float mean_signed_error = 0.0;

    // First calculate mean signed error (bias)
    for (int i = 0; i < _sample_count; i++) {
        float target = 0.0;
        if (_state == TUNE_VEL_P || _state == TUNE_VEL_I) {
            target = ((i / 20) % 2 == 0) ? 0.3 : -0.3;
        } else if (_state == TUNE_ANG_P) {
             target = _start_angle + (((i / 20) % 2 == 0) ? 0.5 : -0.5);
        }
        mean_signed_error += (_velocity_buffer[i] - target);
    }
    mean_signed_error /= _sample_count;

    // Now variance of error
    for (int i = 0; i < _sample_count; i++) {
        float target = 0.0;
        if (_state == TUNE_VEL_P || _state == TUNE_VEL_I) {
            target = ((i / 20) % 2 == 0) ? 0.3 : -0.3;
        } else if (_state == TUNE_ANG_P) {
             target = _start_angle + (((i / 20) % 2 == 0) ? 0.5 : -0.5);
        }

        float signed_error = (_velocity_buffer[i] - target);
        float diff = signed_error - mean_signed_error;
        sum_sq_diff += diff * diff;
    }
    error_std_dev = sqrt(sum_sq_diff / _sample_count);

    // Calculate HF Noise (Buzzing)
    float hf_noise = measureHFNoise();

    // Combined Score: Lower is better
    // Score = Tracking Error + Stability + (Buzzing * Penalty)
    // Penalty Factor: 2.0 (Moderate penalty for buzzing)
    return avg_error + error_std_dev + (hf_noise * 2.0);
}

void AutoTuner::storeSample(float value) {
    if (_sample_count < WINDOW_SIZE) {
        _velocity_buffer[_sample_count] = value;

        // Calculate and store velocity change (for HF noise detection)
        // ALWAYS use shaft_velocity for this, regardless of what 'value' is (Angle or Velocity)
        float current_vel = _motor->shaft_velocity;
        if (_sample_count > 0) {
            _velocity_change_buffer[_sample_count - 1] = abs(current_vel - _prev_shaft_velocity);
        }
        _prev_shaft_velocity = current_vel;

        _sample_count++;
    }
}

void AutoTuner::setState(State new_state) {
    _state = new_state;
}

float AutoTuner::getProgress() {
    switch (_state) {
        case IDLE: return 0.0;
        case INIT: return 0.05;
        case MEASURE_BASELINE: return 0.10;
        case TUNE_VEL_P: return 0.30 + (_current_gain / _p_max) * 0.25;
        case TUNE_VEL_I: return 0.55 + (_current_gain / _i_max) * 0.25;
        case TUNE_ANG_P: return 0.80 + (_current_gain / 40.0) * 0.19;
        case COMPLETE: return 1.0;
        default: return 0.0;
    }
}

void AutoTuner::printStatus(const char* message) {
    Serial.print("AutoTuner: ");
    Serial.println(message);
}
