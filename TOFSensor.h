#ifndef TOFSENSOR_H
#define TOFSENSOR_H

#include "Adafruit_VL53L0X.h"

// Addresses for the sensors
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32

// Pins to shutdown
#define SHT_LOX1 20
#define SHT_LOX2 19
#define SHT_LOX3 2

// Function declarations
void setupTOF();
void setID();
int* read_sensors();

#endif // TOFSENSOR_H
