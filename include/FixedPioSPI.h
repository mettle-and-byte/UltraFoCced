#ifndef FIXED_PIO_SPI_H
#define FIXED_PIO_SPI_H

#include "PioSPI.h"

class FixedPioSPI : public PioSPI {
public:
    FixedPioSPI(pin_size_t tx, pin_size_t rx, pin_size_t sck, pin_size_t cs, uint8_t data_mode, uint32_t frequency)
        : PioSPI(tx, rx, sck, cs, data_mode, frequency) {}

    // Fix for missing const correctness in PioSPI library
    void transfer(const void *txbuf, void *rxbuf, size_t count) override {
        // Cast away const because PioSPI::transfer expects void*
        PioSPI::transfer(const_cast<void*>(txbuf), rxbuf, count);
    }

    // Forward other overloads just in case, though inheritance should handle them if they are not virtual/hidden
    using PioSPI::transfer;
};

#endif
