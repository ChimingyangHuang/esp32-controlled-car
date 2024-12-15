#include "TOFSensor.h"
#include <Arduino.h>

// TOF sensor objects
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();


// Measurement data for each sensor
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
VL53L0X_RangingMeasurementData_t measure3;

void setupTOF() {
  //sda scl
    Wire.begin(21, 33, 40000); 
    pinMode(SHT_LOX1, OUTPUT);
    pinMode(SHT_LOX2, OUTPUT);
    // pinMode(SHT_LOX3, OUTPUT);

    Serial.println(F("Shutdown pins initialized..."));

    // Put all sensors in reset mode
    digitalWrite(SHT_LOX1, LOW);
    digitalWrite(SHT_LOX2, LOW);
    // digitalWrite(SHT_LOX3, LOW);

    Serial.println(F("All sensors in reset mode...(pins are low)"));
    Serial.println(F("Starting..."));

    setID(); // Initialize sensors with unique IDs
}

void setID() {
    // Reset all sensors
    digitalWrite(SHT_LOX1, LOW);
    digitalWrite(SHT_LOX2, LOW);
    // digitalWrite(SHT_LOX3, LOW);
    delay(10);

    // Activate LOX1
    digitalWrite(SHT_LOX1, HIGH);
    delay(10);
    if (!lox1.begin(LOX1_ADDRESS)) {
        Serial.println(F("Failed to boot first VL53L0X"));
        while (1);
    }

    // Activate LOX2
    digitalWrite(SHT_LOX2, HIGH);
    delay(10);
    if (!lox2.begin(LOX2_ADDRESS)) {
        Serial.println(F("Failed to boot second VL53L0X"));
        while (1);
    }

    // // Activate LOX3
    // digitalWrite(SHT_LOX3, HIGH);
    // delay(10);
    // if (!lox3.begin(LOX3_ADDRESS)) {
    //     Serial.println(F("Failed to boot third VL53L0X"));
    //     while (1);
    // }

    // Serial.println(F("Sensors initialized."));
}

int* read_sensors() {
    static int ranges[3];

    // Read data from each sensor
    lox1.rangingTest(&measure1, false);
    lox2.rangingTest(&measure2, false);
    // lox3.rangingTest(&measure3, false);

    // Process sensor 1
    if (measure1.RangeStatus != 4) {
        ranges[0] = measure1.RangeMilliMeter;
    } else {
        ranges[0] = -1; // Use -1 to indicate "Out of range"
    }

    // Process sensor 2
    if (measure2.RangeStatus != 4) {
        ranges[1] = measure2.RangeMilliMeter;
    } else {
        ranges[1] = -1; // Use -1 to indicate "Out of range"
    }

    // // Process sensor 3
    // if (measure3.RangeStatus != 4) {
    //     ranges[2] = measure3.RangeMilliMeter;
    // } else {
    //     ranges[2] = -1; // Use -1 to indicate "Out of range"
    // }

    return ranges; // Return the array of ranges
}
