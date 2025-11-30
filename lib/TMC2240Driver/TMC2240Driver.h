#ifndef TMC2240_DRIVER_H
#define TMC2240_DRIVER_H

#include <Arduino.h>
#include <SimpleFOC.h>
#include <SPI.h>

// DRV_STATUS flag bit positions
#define DRV_STATUS_STANDSTILL (1UL << 31)
#define DRV_STATUS_OLB        (1UL << 30)  // Open Load Phase B
#define DRV_STATUS_OLA        (1UL << 29)  // Open Load Phase A
#define DRV_STATUS_S2GB       (1UL << 28)  // Short to Ground Phase B
#define DRV_STATUS_S2GA       (1UL << 27)  // Short to Ground Phase A
#define DRV_STATUS_SG2        (1UL << 24)  // StallGuard flag

// Critical error flags (disable motor immediately)
#define DRV_STATUS_CRITICAL (DRV_STATUS_S2GA | DRV_STATUS_S2GB | DRV_STATUS_OLA | DRV_STATUS_OLB)

class TMC2240Driver : public StepperDriver {
public:
    TMC2240Driver(int cs_pin, int en_pin, int uart_en_pin, int miso_pin, int mosi_pin, int sck_pin, float r_ref = 12000.0f, float phase_resistance = 2.5f, int max_current_ma = 2000);

    int init();
    void enable();
    void disable();
    void setPwm(float Ua, float Ub);
    void setPhaseState(PhaseState sa, PhaseState sb);
    void setMotorConfig(float phase_resistance, int max_current_ma);
    void setChopperConfig(uint8_t toff, uint8_t tbl, uint8_t hstrt, uint8_t hend);

    // Configuration
    float voltage_power_supply;
    float voltage_limit;

    // Helpers
    int16_t voltageToCurrentCode(float voltage);
    uint8_t writeRegister(uint8_t reg, uint32_t data);
    uint32_t readRegister(uint8_t reg);
    float getChipTemperature();
    uint32_t getGSTAT();
    uint32_t getDRVSTATUS();
    uint32_t getIOIN();

    // Driver status checking API
    bool hasDriverError();           // Returns true if driver_error flag was set

private:
    // Internal fault tracking
    bool _hasFault = false;

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

    // Chopper Config
    uint8_t _toff = 3;
    uint8_t _tbl = 2;
    uint8_t _hstrt = 4;
    uint8_t _hend = 1;
};

#endif // TMC2240_DRIVER_H
