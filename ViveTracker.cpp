#include "ViveTracker.h"

// Initialize the Vive trackers
Vive510 vive1(SIGNALPIN1);
Vive510 vive2(SIGNALPIN2);

void setupVive() {
    vive1.begin();
    vive2.begin();
    pinMode(LED_BUILTIN, OUTPUT);
}

void resetVive() {
    // No internal filtering state to reset anymore
}

VivePosition readVive1() {
    uint16_t x, y;
    
    if (vive1.status() == VIVE_RECEIVING) {
        x = vive1.xCoord();
        y = vive1.yCoord();

        // Basic range checks can remain if desired
        if (x > 8000 || y > 8000 || x < 1000 || y < 1000) {
            x = 0;
            y = 0;
            digitalWrite(LED_BUILTIN, LOW);
        }
    } else {
        // If not receiving, return -1 to indicate no data
        digitalWrite(LED_BUILTIN, LOW);
        x = (uint16_t)-1;
        y = (uint16_t)-1;
        vive1.sync(5); // Attempt to resync
    }

    return {x, y};
}

VivePosition readVive2() {
    uint16_t x2, y2;

    if (vive2.status() == VIVE_RECEIVING) {
        x2 = vive2.xCoord();
        y2 = vive2.yCoord();

        // Basic range checks can remain if desired
        if (x2 > 8000 || y2 > 8000 || x2 < 1000 || y2 < 1000) {
            x2 = 0;
            y2 = 0;
            digitalWrite(LED_BUILTIN, LOW);
        }
    } else {
        // If not receiving, return -1 to indicate no data
        digitalWrite(LED_BUILTIN, LOW);
        x2 = (uint16_t)-1;
        y2 = (uint16_t)-1;
        vive2.sync(5); // Attempt to resync
    }

    return {x2, y2};
}
