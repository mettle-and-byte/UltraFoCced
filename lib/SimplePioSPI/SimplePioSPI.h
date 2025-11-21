#pragma once

#include <Arduino.h>
#include <SPI.h>
#include "hardware/pio.h"

class SimplePioSPI : public SPIClass {
public:
    SimplePioSPI(int mosi, int miso, int sck, int cs, PIO pio = pio0, uint sm = 0);

    void begin() override;
    void end() override;

    void beginTransaction(SPISettings settings) override;
    void endTransaction() override;

    byte transfer(uint8_t data) override;
    uint16_t transfer16(uint16_t data) override;
    void transfer(void *buf, size_t count) override;
    void transfer(const void *txbuf, void *rxbuf, size_t count) override;

    // Interrupt stubs
    void usingInterrupt(int interruptNumber) override {}
    void notUsingInterrupt(int interruptNumber) override {}
    void attachInterrupt() override {}
    void detachInterrupt() override {}

private:
    int _mosi, _miso, _sck, _cs;
    PIO _pio;
    uint _sm;
    uint _offset;
    bool _initted;
    uint8_t _current_mode;
    uint32_t _current_freq;
};
