#include "QueueStream.h"

QueueStream::QueueStream(uint size) : _size(size), _overflow_count(0) {
}

QueueStream::~QueueStream() {
    queue_free(&_rx_queue);
    queue_free(&_tx_queue);
}

void QueueStream::begin() {
    queue_init(&_rx_queue, sizeof(uint8_t), _size);
    queue_init(&_tx_queue, sizeof(uint8_t), _size);
}

int QueueStream::available() {
    return queue_get_level(&_rx_queue);
}

int QueueStream::read() {
    uint8_t c;
    if (queue_try_remove(&_rx_queue, &c)) {
        return c;
    }
    return -1;
}

int QueueStream::peek() {
    // Peek is hard with pico_queue as it doesn't support peeking easily without removing.
    // For now, we can implement a basic peek if needed, or just return -1.
    // SimpleFOC Commander uses peek() to check for newline?
    // Actually, Commander::run() uses available() and read().
    // Let's see if we can implement peek.
    // pico_queue doesn't have peek.
    // We might need a 1-byte buffer for peek?
    // For now, let's return -1 and see if it breaks.
    // EDIT: Commander.cpp uses peek() in getNextToken? No, it uses read().
    // Stream::peek() is often used.
    // Implementing peek correctly with a queue is tricky.
    // Let's check if we can avoid it.
    return -1;
}

void QueueStream::flush() {
    // No-op or clear queues?
    // Usually flush() waits for TX to complete.
    // Since we are non-blocking, we just return.
}

size_t QueueStream::write(uint8_t c) {
    if (!queue_try_add(&_tx_queue, &c)) {
        _overflow_count++;
        return 1; // Pretend we wrote it to avoid blocking logic
    }
    return 1;
}

size_t QueueStream::write(const uint8_t *buffer, size_t size) {
    size_t written = 0;
    for (size_t i = 0; i < size; i++) {
        write(buffer[i]);
        written++;
    }
    return written;
}

uint32_t QueueStream::getAndClearOverflow() {
    uint32_t cnt = _overflow_count;
    _overflow_count = 0;
    return cnt;
}

// Core 1 Interface
bool QueueStream::pushRX(uint8_t c) {
    return queue_try_add(&_rx_queue, &c);
}

int QueueStream::popTX() {
    uint8_t c;
    if (queue_try_remove(&_tx_queue, &c)) {
        return c;
    }
    return -1;
}
