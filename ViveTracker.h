#ifndef VIVETRACKER_H
#define VIVETRACKER_H

#include <Arduino.h>
#include "vive510.h" // Include the library for Vive

// Pin definition for the signal
#define SIGNALPIN1 16
#define SIGNALPIN2 15

typedef struct {
    uint16_t x;
    uint16_t y;
} VivePosition;

// Function declarations
void setupVive();
void resetVive();
VivePosition readVive1();
VivePosition readVive2();


uint32_t med3filt(uint32_t a, uint32_t b, uint32_t c);

#endif // VIVETRACKER_H
