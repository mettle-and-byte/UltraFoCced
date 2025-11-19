#ifndef TMC2240_DRIVER_H
#define TMC2240_DRIVER_H

#include <Arduino.h>
#include <SimpleFOC.h>
#include <SPI.h>

class TMC2240Driver : public StepperDriver {
public:
    TMC2240Driver(int cs_pin, int en_pin, int uart_en_pin, float r_sense = 0.11f);

    // StepperDriver implementation
    int init() override;
    void enable() override;
    void disable() override;
    void setPwm(float Ua, float Ub) override;
    void setPhaseState(PhaseState sa, PhaseState sb) override;

    // TMC2240 specific
    void writeRegister(uint8_t reg, uint32_t data);
    uint32_t readRegister(uint8_t reg);
    float getChipTemperature();

private:
    int _cs_pin;
    int _en_pin;
    int _uart_en_pin;
    float _r_sense;

    SPISettings _spi_settings;

    // Helper to convert voltage to current to register value
    int16_t voltageToCurrentCode(float voltage);
};

#endif
