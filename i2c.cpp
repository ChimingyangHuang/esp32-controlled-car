#include "i2c.h"
/*
int ENLeftFront = 9;
int IN1LeftFront = 10;
int IN2LeftFront = 11;
int ENCODERLeftFront = 12;

int ENLeftRear = 7;
int IN1LeftRear = 6;
int IN2LeftRear = 5;
int ENCODERLeftRear = 4;

// Right Motors
int ENRightFront = 39;
int IN1RightFront = 37;
int IN2RightFront = 38;
int ENCODERRightFront = 36;

int ENRightRear = 40;
int IN1RightRear = 41;
int IN2RightRear = 42;
int ENCODERRightRear = 2;

#define SHT_LOX1 20
#define SHT_LOX2 19

//sda21 scl33
Wire.begin(21, 33, 40000);  TOF sensor
Wire1.begin(34, 35, 40000); tophat

int servoPin = 14;

#define SIGNALPIN1 15
#define SIGNALPIN2 16
*/
void initI2CMaster() {
  // SDA_PIN1 34  SCL_PIN1 35
  Wire1.begin(34, 35, 40000);  // Initialize I2C with pins and speed
}

void sendI2CByte(uint8_t data) {
  // Send data to slave
  Wire1.beginTransmission(I2C_SLAVE_ADDR);
  Wire1.write(data);
  uint8_t error = Wire1.endTransmission();
  if (error == 0) {
      Serial.println("Data sent successfully");
      // Change LED status (implement `rgbLedWrite` accordingly)
      rgbLedWrite(2, 0, 20, 0);  // Green
  } else {
      Serial.printf("Error sending data: %d\n", error);
      rgbLedWrite(2, 20, 0, 0);  // Red
  }
}

uint8_t receiveI2CByte() {
  uint8_t bytesReceived = Wire1.requestFrom(I2C_SLAVE_ADDR, 1);
  uint8_t byteIn = 0;

  if (bytesReceived > 0) {
      Serial.print("Received from slave: ");
      while (Wire1.available()) {
          byteIn = Wire1.read();
          Serial.printf("0x%02X ", byteIn);
      }
      Serial.println();
  } else {
      return 0;
  }
  return byteIn;
}
