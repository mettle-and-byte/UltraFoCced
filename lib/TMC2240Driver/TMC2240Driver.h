#ifndef TMC2240_DRIVER_H
#define TMC2240_DRIVER_H

#include <Arduino.h>
#include <SimpleFOC.h>

class TMC2240Driver : public StepperDriver {
public:
    TMC2240Driver(int cs_pin, int en_pin, int uart_en_pin, int miso_pin, int mosi_pin, int sck_pin, float r_ref = 12000.0f, float phase_resistance = 2.5f, int max_current_ma = 2000);

    int init();
    void enable();
    void disable();
    void setPwm(float Ua, float Ub);
    void setPhaseState(PhaseState sa, PhaseState sb);
    void setMotorConfig(float phase_resistance, int max_current_ma);

    // Configuration
    float voltage_power_supply;
    float voltage_limit;

    // Helpers
    int16_t voltageToCurrentCode(float voltage);
    void writeRegister(uint8_t reg, uint32_t data);
    uint32_t readRegister(uint8_t reg);
    float getChipTemperature();
    uint32_t getGSTAT();
    uint32_t getDRVSTATUS();
    uint32_t getIOIN();

private:
    int _cs_pin;
    int _en_pin;
    int _uart_en_pin;
    int _miso_pin;
    int _mosi_pin;
    int _sck_pin;
    float _r_ref;
    float _phase_resistance;
    int _max_current_ma;
    float _actual_max_current;
    SPISettings _spi_settings;
};

#endif // TMC2240_DRIVER_H
