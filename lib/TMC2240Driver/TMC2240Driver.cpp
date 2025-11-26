#include "TMC2240Driver.h"

// TMC2240 Registers
#define TMC2240_GCONF 0x00
#define TMC2240_IHOLD_IRUN 0x10
#define TMC2240_DIRECT_MODE 0x2D
#define TMC2240_CHOPCONF 0x6C

TMC2240Driver::TMC2240Driver(int cs_pin, int en_pin, int uart_en_pin, int miso_pin, int mosi_pin, int sck_pin, float r_ref, float phase_resistance, int max_current_ma)
    : _cs_pin(cs_pin), _en_pin(en_pin), _uart_en_pin(uart_en_pin),
      _miso_pin(miso_pin), _mosi_pin(mosi_pin), _sck_pin(sck_pin),
      _r_ref(r_ref), _phase_resistance(phase_resistance), _max_current_ma(max_current_ma),
      _spi_settings(4000000, MSBFIRST, SPI_MODE3) // Run SPI at 4MHz
{
}

int TMC2240Driver::init() {
    // Configure Pins

    pinMode(_uart_en_pin, OUTPUT);
    digitalWrite(_uart_en_pin, LOW); // Force SPI Mode

    pinMode(_cs_pin, OUTPUT);
    digitalWrite(_cs_pin, HIGH);

    pinMode(_en_pin, OUTPUT);
    digitalWrite(_en_pin, HIGH); // Disable (Active Low)


    // Initialize SPI
    // Use board-specific pins passed in constructor
    SPI.setRX(_miso_pin);
    SPI.setTX(_mosi_pin);
    SPI.setSCK(_sck_pin);
    SPI.begin();

    // Wait for driver to boot
    delay(100);

    // Configure TMC2240

    // 0. Set DRV_CONF: Set Current Range (Bits 1..0)
    // Select the best Full Scale Range to maximize resolution.
    // Table 17 (RREF=12k):
    // 10 -> KIFS = 36 -> IFS = 3.0A
    // 01 -> KIFS = 24 -> IFS = 2.0A
    // 00 -> KIFS = 11.75 -> IFS = ~1.0A

    const float ifs_3A = 36000.0f / _r_ref;
    const float ifs_2A = 24000.0f / _r_ref;
    const float ifs_1A = 11750.0f / _r_ref;

    const float target_amps = _max_current_ma / 1000.0f;

    uint8_t drv_conf_bits = 0;
    float selected_ifs = 0;

    if (target_amps <= ifs_1A) {
        drv_conf_bits = 0x00;
        selected_ifs = ifs_1A;
    } else if (target_amps <= ifs_2A) {
        drv_conf_bits = 0x01;
        selected_ifs = ifs_2A;
    } else {
        drv_conf_bits = 0x02; // 3A
        selected_ifs = ifs_3A;
    }

    uint32_t drv_conf = readRegister(0x0A); // DRV_CONF
    drv_conf &= ~(0x03); // Clear bits 1..0
    drv_conf |= drv_conf_bits;
    writeRegister(0x0A, drv_conf);



    // 2. Set IHOLD_IRUN based on Max Current
    // Use the selected Full Scale Current (selected_ifs)
    float full_scale_max = selected_ifs;
    float target_max = _max_current_ma / 1000.0f;

    // Clamp target to hardware limit
    if (target_max > full_scale_max) target_max = full_scale_max;

    // Calculate Current Setting (CS) for IHOLD (0-31)
    uint8_t cs = (uint8_t)((target_max / full_scale_max) * 31.0f);
    if (cs > 31) cs = 31;

    // Store actual scaled current for voltageToCurrentCode
    _actual_max_current = full_scale_max * ((float)(cs + 1) / 32.0f);

    uint32_t ihold_irun = 0;
    ihold_irun |= (cs << 0);  // IHOLD = CS
    ihold_irun |= (cs << 8);  // IRUN = CS
    ihold_irun |= (1 << 16);  // IHOLDDELAY = 1
    writeRegister(TMC2240_IHOLD_IRUN, ihold_irun);

    // 3. Set CHOPCONF
    // Use configured values (or defaults: TOFF=3, TBL=2, HSTRT=4, HEND=1)
    // CHM=0 (SpreadCycle)
    uint32_t chopconf = 0;
    chopconf |= (_toff & 0x0F);          // TOFF
    chopconf |= ((_hstrt & 0x07) << 4);  // HSTRT
    chopconf |= ((_hend & 0x0F) << 7);   // HEND
    chopconf |= ((_tbl & 0x03) << 15);   // TBL
    chopconf &= ~(1 << 14);              // CHM = 0 (SpreadCycle)
    writeRegister(TMC2240_CHOPCONF, chopconf);

    // 4. Set GCONF: Enable Direct Mode (Bit 16) and Disable StealthChop (Bit 2)
    uint32_t gconf = readRegister(TMC2240_GCONF);
    gconf |= (1 << 16); // Set direct_mode
    gconf &= ~(1 << 2); // Clear en_pwm_mode (Disable StealthChop)
    writeRegister(TMC2240_GCONF, gconf);

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

void TMC2240Driver::setMotorConfig(float phase_resistance, int max_current_ma) {
    _phase_resistance = phase_resistance;
    _max_current_ma = max_current_ma;
}

void TMC2240Driver::setChopperConfig(uint8_t toff, uint8_t tbl, uint8_t hstrt, uint8_t hend) {
    _toff = toff;
    _tbl = tbl;
    _hstrt = hstrt;
    _hend = hend;
}

void TMC2240Driver::setPhaseState(PhaseState sa, PhaseState sb) {
    if (sa == PhaseState::PHASE_OFF && sb == PhaseState::PHASE_OFF) {
        disable();
    } else {
        enable();
    }
}

int16_t TMC2240Driver::voltageToCurrentCode(float voltage) {
    // 1. Ohm's Law: I_target = V / R
    float i_target = voltage / _phase_resistance;

    // 2. Scale to Driver Limit: Ratio = I_target / I_max
    float ratio = i_target / _actual_max_current;

    // 3. Clamp
    if (ratio > 1.0f) ratio = 1.0f;
    if (ratio < -1.0f) ratio = -1.0f;

    // 4. Map to -255..255
    return (int16_t)(ratio * 255.0f);
}

void TMC2240Driver::writeRegister(uint8_t reg, uint32_t data) {
    SPI.beginTransaction(_spi_settings);
    digitalWrite(_cs_pin, LOW);
    delayMicroseconds(1);

    // TMC2240 SPI Format: 40 bits
    // Byte 0: Register Address + Write bit (0x80)
    // Byte 1-4: Data (MSB first)

    SPI.transfer(reg | 0x80); // Write flag
    SPI.transfer((data >> 24) & 0xFF);
    SPI.transfer((data >> 16) & 0xFF);
    SPI.transfer((data >> 8) & 0xFF);
    SPI.transfer(data & 0xFF);

    delayMicroseconds(1);
    digitalWrite(_cs_pin, HIGH);
    SPI.endTransaction();
}

uint32_t TMC2240Driver::readRegister(uint8_t reg) {
    // Transaction 1: Request Read
    SPI.beginTransaction(_spi_settings);
    digitalWrite(_cs_pin, LOW);
    delayMicroseconds(1);

    SPI.transfer(reg & 0x7F);
    SPI.transfer(0);
    SPI.transfer(0);
    SPI.transfer(0);
    SPI.transfer(0);

    delayMicroseconds(1);
    digitalWrite(_cs_pin, HIGH);
    SPI.endTransaction();

    // Transaction 2: Retrieve Data (and trigger next read, effectively a dummy read)
    // We repeat the read command to get the data requested in Transaction 1.
    // The data returned here is the result of Transaction 1.
    SPI.beginTransaction(_spi_settings);
    digitalWrite(_cs_pin, LOW);
    delayMicroseconds(1);

    uint8_t status = SPI.transfer(reg & 0x7F);
    uint32_t data = 0;
    data |= (uint32_t)SPI.transfer(0) << 24;
    data |= (uint32_t)SPI.transfer(0) << 16;
    data |= (uint32_t)SPI.transfer(0) << 8;
    data |= (uint32_t)SPI.transfer(0);

    delayMicroseconds(1);
    digitalWrite(_cs_pin, HIGH);
    SPI.endTransaction();

    return data;
}

float TMC2240Driver::getChipTemperature() {
    uint32_t adc_val = readRegister(0x51); // ADC_TEMP
    // Formula: T = (ADC - 2038) / 7.7
    return ((float)(adc_val & 0xFFF) - 2038.0f) / 7.7f;
}

uint32_t TMC2240Driver::getGSTAT() {
    return readRegister(0x01); // GSTAT
}

uint32_t TMC2240Driver::getDRVSTATUS() {
    return readRegister(0x6F); // DRV_STATUS
}

uint32_t TMC2240Driver::getIOIN() {
    return readRegister(0x04); // IOIN
}
