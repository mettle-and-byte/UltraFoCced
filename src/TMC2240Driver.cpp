#include "TMC2240Driver.h"

// TMC2240 Registers
#define TMC2240_GCONF 0x00
#define TMC2240_IHOLD_IRUN 0x10
#define TMC2240_DIRECT_MODE 0x2D
#define TMC2240_CHOPCONF 0x6C

TMC2240Driver::TMC2240Driver(int cs_pin, int en_pin, int uart_en_pin, float r_sense)
    : _cs_pin(cs_pin), _en_pin(en_pin), _uart_en_pin(uart_en_pin), _r_sense(r_sense),
      _spi_settings(1000000, MSBFIRST, SPI_MODE3) // TMC2240 uses Mode 3 usually
{
    // SimpleFOC StepperDriver defaults
    voltage_power_supply = 12.0f;
    voltage_limit = 12.0f;
}

int TMC2240Driver::init() {
    // Configure Pins
    pinMode(_cs_pin, OUTPUT);
    digitalWrite(_cs_pin, HIGH);

    pinMode(_en_pin, OUTPUT);
    digitalWrite(_en_pin, HIGH); // Disable (Active Low)

    pinMode(_uart_en_pin, OUTPUT);
    digitalWrite(_uart_en_pin, LOW); // Force SPI Mode

    // Initialize SPI
    // We assume standard SPI0 pins are used by default SPI object
    SPI.begin();

    // Wait for driver to boot
    delay(100);

    // Configure TMC2240

    // 1. Set GCONF: Enable Direct Mode (Bit 16)
    // Read current GCONF first? No, just overwrite for now or read-modify-write
    // Reset value is 0x00000000 usually
    uint32_t gconf = readRegister(TMC2240_GCONF);
    gconf |= (1 << 16); // Set direct_mode
    writeRegister(TMC2240_GCONF, gconf);

    // 2. Set IHOLD_IRUN
    // IHOLD=31 (Max scaling for Direct Mode), IRUN=31, IHOLDDELAY=1
    // Bits: 0-4 IHOLD, 8-12 IRUN, 16-19 IHOLDDELAY
    uint32_t ihold_irun = 0;
    ihold_irun |= (31 << 0);  // IHOLD = 31
    ihold_irun |= (31 << 8);  // IRUN = 31
    ihold_irun |= (1 << 16);  // IHOLDDELAY = 1
    writeRegister(TMC2240_IHOLD_IRUN, ihold_irun);

    // 3. Set CHOPCONF (Optional, but good defaults help)
    // TOFF=3, HSTRT=4, HEND=1, TBL=2, CHM=0 (SpreadCycle)
    // This is less critical in Direct Mode as we control current targets,
    // but the chopper still runs.
    // Default reset value is usually okay.

    // Enable Driver
    enable();

    return 1;
}

void TMC2240Driver::enable() {
    digitalWrite(_en_pin, LOW); // Active Low
}

void TMC2240Driver::disable() {
    digitalWrite(_en_pin, HIGH);
}

void TMC2240Driver::setPwm(float Ua, float Ub) {
    // Convert Voltage to Current Code
    int16_t codeA = voltageToCurrentCode(Ua);
    int16_t codeB = voltageToCurrentCode(Ub);

    // Construct DIRECT_MODE register value
    // Coil A: Bits 0-8 (Signed 9-bit)
    // Coil B: Bits 16-24 (Signed 9-bit)

    uint32_t direct_reg = 0;
    direct_reg |= (codeA & 0x1FF); // Mask to 9 bits
    direct_reg |= ((codeB & 0x1FF) << 16);

    writeRegister(TMC2240_DIRECT_MODE, direct_reg);
}

void TMC2240Driver::setPhaseState(PhaseState sa, PhaseState sb) {
    if (sa == PhaseState::PHASE_OFF && sb == PhaseState::PHASE_OFF) {
        disable();
    } else {
        enable();
    }
}

int16_t TMC2240Driver::voltageToCurrentCode(float voltage) {
    // I = V / R_phase (approx)
    // But we don't know R_phase here easily unless passed.
    // Let's assume 'voltage' passed by SimpleFOC is actually 'Target Voltage'.
    // If we want to map this to Current, we need a scaling factor.
    // For now, let's assume voltage_limit maps to Max Current (255).

    float max_v = voltage_limit;
    if (max_v < 0.1f) max_v = 12.0f; // Safety

    float ratio = voltage / max_v;

    // Clamp ratio -1 to 1
    if (ratio > 1.0f) ratio = 1.0f;
    if (ratio < -1.0f) ratio = -1.0f;

    // Map to -255 to 255
    return (int16_t)(ratio * 255.0f);
}

void TMC2240Driver::writeRegister(uint8_t reg, uint32_t data) {
    SPI.beginTransaction(_spi_settings);
    digitalWrite(_cs_pin, LOW);

    // TMC2240 SPI Format: 40 bits
    // Byte 0: Register Address + Write bit (0x80)
    // Byte 1-4: Data (MSB first)

    SPI.transfer(reg | 0x80); // Write flag
    SPI.transfer((data >> 24) & 0xFF);
    SPI.transfer((data >> 16) & 0xFF);
    SPI.transfer((data >> 8) & 0xFF);
    SPI.transfer(data & 0xFF);

    digitalWrite(_cs_pin, HIGH);
    SPI.endTransaction();
}

uint32_t TMC2240Driver::readRegister(uint8_t reg) {
    SPI.beginTransaction(_spi_settings);
    digitalWrite(_cs_pin, LOW);

    // Read request: Address (Bit 7 = 0) + 4 Dummy Bytes
    // MISO returns: Status + 32-bit Data

    uint8_t status = SPI.transfer(reg & 0x7F);
    uint32_t data = 0;
    data |= (uint32_t)SPI.transfer(0) << 24;
    data |= (uint32_t)SPI.transfer(0) << 16;
    data |= (uint32_t)SPI.transfer(0) << 8;
    data |= (uint32_t)SPI.transfer(0);

    digitalWrite(_cs_pin, HIGH);
    SPI.endTransaction();

    return data;
}
