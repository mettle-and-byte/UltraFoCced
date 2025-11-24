#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include <Arduino.h>

// Pin Definitions
#define LED_PIN 23
#define FAN_PIN 16
#define ENDSTOP_PIN 18

// TMC2240 Pins
#define DRV_CS 5
#define DRV_EN 0
#define DRV_UART_EN 29
#define DRV_MISO 4
#define DRV_MOSI 7
#define DRV_SCK 6
#define TMC2240_RREF 12000.0f

// Encoder Pins
#define ENC_CS 19
#define ENC_SCK 20
#define ENC_MISO 21
#define ENC_MOSI 22
#define DUMMY_CS 24 // Unused

#endif // BOARD_CONFIG_H
