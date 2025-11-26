#ifndef QUEUE_STREAM_H
#define QUEUE_STREAM_H

#include <Arduino.h>
#include <Stream.h>
#include "pico/util/queue.h"

class QueueStream : public Stream {
public:
    QueueStream(uint size = 512);
    virtual ~QueueStream();

    void begin();

    // Stream implementation
    virtual int available();
    virtual int read();
    virtual int peek();
    virtual void flush();
    virtual size_t write(uint8_t c);
    virtual size_t write(const uint8_t *buffer, size_t size);

    // Custom methods
    uint32_t getAndClearOverflow();

    // Direct access for the other core to push/pop
    bool pushRX(uint8_t c);
    int popTX();

private:
    queue_t _rx_queue; // Data coming FROM Serial (Core 1) TO Commander (Core 0)
    queue_t _tx_queue; // Data going FROM Commander (Core 0) TO Serial (Core 1)
    uint _size;
    volatile uint32_t _overflow_count;
};

#endif // QUEUE_STREAM_H
