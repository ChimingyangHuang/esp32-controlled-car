#ifndef I2C_H
#define I2C_H

#include <Wire.h>
#include <Arduino.h>

// Constants
#define I2C_SLAVE_ADDR 0x28

// Function Prototypes
void initI2CMaster();
void sendI2CByte(uint8_t data);
uint8_t receiveI2CByte();

#endif // I2C_MASTER_H
