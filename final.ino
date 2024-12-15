#include <WiFi.h>
#include <WiFiUdp.h>
#include "html510.h"
#include "webpage.h"
#include "TOFSensor.h"
#include "driver/gpio.h"


//setupTOF();
//needed to call in setup(),setup 3 TOF

// int* read_sensors() return:
// ranges[0]: Measurement from the first sensor.
// ranges[1]: Measurement from the second sensor.
// ranges[2]: Measurement from the third sensor.
// If a sensor is out of range, the corresponding array value is -1.

#include "MotorControl.h"
// void setupMotor();
// void loopMotor();
// void forward(int forward_RPM);
// void backward(int backward_RPM);
// void turnLeft(int turn_RPM);
// void turnRight(int turn_RPM);
// void stop();
// void setMotorParameters(double leftFrontRPM, int leftFrontDir, 
//                         double leftRearRPM, int leftRearDir, 
//                         double rightFrontRPM, int rightFrontDir, 
//                         double rightRearRPM, int rightRearDir);

#include "i2c.h"
#include "ViveTracker.h"



static float i2cTime=0;
static float rotateTime=0;
static float rotateStart=0;
static float tmpTime=0;
static float servoTime=0;
static bool servoIncrease=true;
static int servoPin = 14;
static bool hitButtonTurn=false;
static bool onHitButton=false;
static bool onRotate=false;
static int rpm=50;
static int leftX=1;
static int leftY=1;
static int rightX=1;
static int rightY=1;
static int hitTol=100;
static int servoPos=0;
static int packetSent=0;
//--------------------WIFI-----------------------//
HTML510Server h(80);
const char* ssid = "MYESPCAR";
const char* password = "12488674";
IPAddress myIP(192, 168, 1, 178);
WiFiUDP UDPTestServer;


static bool motor=true;
static bool wallFollowingLeft=false;
static bool wallFollowingRight=false;
static bool TOFSensor=true;
static bool topHat=true;
static bool servo=false;
static bool vive=true;
static bool onHitLeft=false;
static bool onHitRight=false;
static bool onHitmidRed=false;
static bool onHitBlue=false;
static bool sensorSetup=false;
static int angle=0;

void setupServo() {
  ledcAttachChannel(servoPin, 50, 12, 7);
}

void updateAngle(float x1, float y1, float x2, float y2) {
    angle = atan2(y2 - y1, x2 - x1) * (180.0 / PI) + 90;
    if(angle<0){
      angle+=360;
    } 
    else if(angle>360) {
      angle-=360;
    }
}

// void rotateToAngle(float targetAngle) {
//     const int maxRotationSpeed = 80;  // Maximum motor speed for rotation
//     const int minRotationSpeed = 50;  // Minimum motor speed to overcome inertia
//     const float tolerance = 8.0;      // Acceptable angle error in degrees
//     while (true) {

//         // Access individual values from the array
//         float x1 = coordinates[0];
//         float y1 = coordinates[1];
//         float x2 = coordinates[2];
//         float y2 = coordinates[3];
//         float midX = coordinates[4];
//         float midY = coordinates[5];
//         updateAngle(x1,y1,x2,y2);
//         float angleDifference = targetAngle - angle;

//         // Normalize the angle difference to the range [-180, 180]
//         if (angleDifference > 180.0) angleDifference -= 360.0;
//         // Check if within tolerance
//         if (abs(angleDifference) <= tolerance) {
//             stop(); // Stop motors when the target angle is reached
//             break;
//         }
//         // Calculate rotation speed based on the angle difference
//         int rotationSpeed = 60;
//         // Determine rotation direction and rotate
//         if (angleDifference > 0) {
//             turnLeft(rotationSpeed);  // Rotate clockwise
//         } else {
//             turnRight(rotationSpeed);   // Rotate counterclockwise
//         }
//         loopMotor();
//         delay(10);
//     }
// }

void hitLeft() {
  // VivePosition pos1 = readVive1();
  // VivePosition pos2 = readVive1();
  // int midx=(pos1.x+pos2.x)/2;
  // int midy=(pos1.y+pos2.y)/2;
  // int targetAngle=atan2(leftY-midy,leftX-midx)*(180/PI);
  // rotateToAngle(targetAngle);
  // forward(70);
}

void setupWifiAndServer() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password,5);
    WiFi.softAPConfig(myIP, IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));

    // Serial.print("Use this URL to connect: http://");
    // Serial.print(WiFi.softAPIP());
    // Serial.println("/");

    h.begin();
    h.attachHandler("/", handleRoot);
    h.attachHandler("/speed?value=", speedControl);
    h.attachHandler("/turn?dir=", turnControl);
    h.attachHandler("/direction?dir=", dirControl);
    h.attachHandler("/mode?value=", modeControl);
}

void modeControl(){
  packetSent++;
  int mode=h.getVal();
  if(mode==0){
    wallFollowingRight=true;
  }
  if(mode==1){
    wallFollowingRight=false;
  }
  if(mode==2){
    servo=true;
  }
  if(mode==3){
    servo=false;
  }
  if(mode==4){
    onHitLeft=true;
  }
  if(mode==5){
    onHitLeft=false;
  }
  if(mode==6){
     //rotateToAngle(0);
  }
  if(mode==7){
     //rotateToAngle(90);
  }
  if(mode==8){
     //rotateToAngle(180);
  }
  if(mode==9){
     //rotateToAngle(270);
  }
  if(mode==10){
    onHitButton=true;
  }
  if(mode==11){
    onHitButton=false;
    hitButtonTurn=false;
  }
}

void handleRoot() {
  packetSent++;
  h.sendhtml(body);
}

void speedControl() {
  packetSent++;
  rpm=h.getVal();
  setSpeed(rpm,rpm,rpm,rpm);
}

void turnControl() {
  packetSent++;
  int dir = h.getVal();
  if (dir == 1) {
      turnLeft(80);
  }else if (dir == 2) { 
      turnRight(80);
  }
}

void dirControl(){
  packetSent++;
  int dir = h.getVal(); // Assuming h.getVal() retrieves the direction command as a string
  if (dir == 1){
    forward(rpm);
  }
  else if(dir==-1){
    backward(rpm);
  }else if (dir == 0) { // If direction is "left"
      stop(); // Call the turnLeft function
  }
}

void rotate(int deg=90){
  const int rotationSpeed = 70;  // Speed for rotation
  const int timePerDegree = 15; // Approximate time (in ms) required for 1° of rotation

    // Calculate the rotation duration
    rotateTime=abs(deg) * timePerDegree;
    rotateStart=millis();
    onRotate=true;
    if (deg > 0) {
        // Positive degrees: Rotate right        
        turnRight(rotationSpeed);
    } else {
        // Negative degrees: Rotate left
        turnLeft(rotationSpeed);
    }
}

void wallFollowingLeftHand() {
    if(onRotate) return;
    const int desiredDistance = 110;       // Desired distance from the wall in mm
    const int desiredDistanceErrorin = 5; // Allowable error in distance (±10 mm)
    const int desiredDistanceErrorout = 10; // Allowable error in distance (±10 mm)
    const int frontObstacleThreshold = 200; // Threshold to detect a front obstacle in mm
    const int adjustmentSpeed = 70;      // Speed for turning or adjustment
    const int adjustmentSpeedDiff = 20; 
    const int baseSpeed = 60;            // Base speed for forward movement

    int* sensorRanges = read_sensors();  // Read sensor data: [left, front, right]

    // Check for an obstacle in front
    if (sensorRanges[1] != -1 && sensorRanges[1] < frontObstacleThreshold) {
        // Obstacle detected in front, stop and turn left
        rotate(90); // Turn right to avoid the obstacle
        return;
    }

    // Wall-following logic
    else if (sensorRanges[2] != -1) {  // Left sensor is active
        if (sensorRanges[2] < desiredDistance - desiredDistanceErrorin) {
            // Too close to the left wall, adjust to the right
            setMotorParameters(adjustmentSpeed+adjustmentSpeedDiff, 1, adjustmentSpeed+adjustmentSpeedDiff, 1, adjustmentSpeed-adjustmentSpeedDiff, 1, adjustmentSpeed-adjustmentSpeedDiff, 1);
            return;
        } else if (sensorRanges[2] > desiredDistance + desiredDistanceErrorout) {
            // Too far from the left wall, adjust to the left
            setMotorParameters(adjustmentSpeed-adjustmentSpeedDiff, 1, adjustmentSpeed-adjustmentSpeedDiff, 1, adjustmentSpeed+adjustmentSpeedDiff, 1, adjustmentSpeed+adjustmentSpeedDiff, 1);
            return;
        } else {
            // At the desired distance from the left wall, move forward
            forward(baseSpeed);
        }
    } else {
        // If the left sensor is inactive, proceed cautiously
        forward(baseSpeed);
    }
}
void hitButton() {
  if(onRotate) return;
    const int desiredDistance = 150;       // Desired distance from the wall in mm
    const int desiredDistanceErrorin = 5; // Allowable error in distance (±10 mm)
    const int desiredDistanceErrorout = 5; // Allowable error in distance (±10 mm)
    const int frontObstacleThreshold = 50; // Threshold to detect a front obstacle in mm
    const int adjustmentSpeed = 55;      // Speed for turning or adjustment
    const int adjustmentSpeedDiff = 25; 
    const int baseSpeed = 60;            // Base speed for forward movement

    int* sensorRanges = read_sensors();  // Read sensor data: [left, front, right]
    Serial.print("right(mm): ");
    Serial.println(sensorRanges[0]);
    Serial.print("front(mm): ");
    Serial.println(sensorRanges[1]);
    // Check for an obstacle in front
    if (sensorRanges[1] != -1 && sensorRanges[1] < frontObstacleThreshold && !hitButtonTurn) {
        hitButtonTurn=true;
        // Obstacle detected in front, stop and turn left
        rotate(-90); // Turn right to avoid the obstacle
        return;
    }

    // Wall-following logic
    else if (sensorRanges[0] != -1) {  // Left sensor is active
        if (sensorRanges[0] < desiredDistance - desiredDistanceErrorin) {
            // Too close to the left wall, adjust to the right
            setMotorParameters(adjustmentSpeed-adjustmentSpeedDiff, 1, adjustmentSpeed-adjustmentSpeedDiff, 1, adjustmentSpeed+adjustmentSpeedDiff, 1, adjustmentSpeed+adjustmentSpeedDiff, 1);
            return;
        } else if (sensorRanges[0] > desiredDistance + desiredDistanceErrorout) {
            // Too far from the left wall, adjust to the left
            setMotorParameters(adjustmentSpeed+adjustmentSpeedDiff+20, 1, adjustmentSpeed+adjustmentSpeedDiff+20, 1, adjustmentSpeed-adjustmentSpeedDiff, 1, adjustmentSpeed-adjustmentSpeedDiff, 1);
            return;
        } else {
            // At the desired distance from the left wall, move forward
            forward(baseSpeed);
        }
    } else {
        // If the left sensor is inactive, proceed cautiously
        forward(baseSpeed/2);
    }
}

void wallFollowingRightHand() {
  if(onRotate) return;
    const int desiredDistance = 120;       // Desired distance from the wall in mm
    const int desiredDistanceErrorin = 5; // Allowable error in distance (±10 mm)
    const int desiredDistanceErrorout = 5; // Allowable error in distance (±10 mm)
    const int frontObstacleThreshold = 200; // Threshold to detect a front obstacle in mm
    const int adjustmentSpeed = 55;      // Speed for turning or adjustment
    const int adjustmentSpeedDiff = 25; 
    const int baseSpeed = 60;            // Base speed for forward movement

    int* sensorRanges = read_sensors();  // Read sensor data: [left, front, right]
    Serial.print("right(mm): ");
    Serial.println(sensorRanges[0]);
    Serial.print("front(mm): ");
    Serial.println(sensorRanges[1]);
    // Check for an obstacle in front
    if (sensorRanges[1] != -1 && sensorRanges[1] < frontObstacleThreshold) {
        // Obstacle detected in front, stop and turn left
        rotate(-90); // Turn right to avoid the obstacle
        return;
    }

    // Wall-following logic
    else if (sensorRanges[0] != -1) {  // Left sensor is active
        if (sensorRanges[0] < desiredDistance - desiredDistanceErrorin-10) {
            // Too close to the left wall, adjust to the right
            setMotorParameters(adjustmentSpeed-adjustmentSpeedDiff, 1, adjustmentSpeed-adjustmentSpeedDiff, 1, adjustmentSpeed+adjustmentSpeedDiff, 1, adjustmentSpeed+adjustmentSpeedDiff, 1);
            return;
        } else if (sensorRanges[0] > desiredDistance + desiredDistanceErrorout) {
            // Too far from the left wall, adjust to the left
            setMotorParameters(adjustmentSpeed+adjustmentSpeedDiff+20, 1, adjustmentSpeed+adjustmentSpeedDiff+20, 1, adjustmentSpeed-adjustmentSpeedDiff, 1, adjustmentSpeed-adjustmentSpeedDiff, 1);
            return;
        } else {
            // At the desired distance from the left wall, move forward
            forward(baseSpeed);
        }
    } else {
        // If the left sensor is inactive, proceed cautiously
        forward(baseSpeed/2);
    }
}

void setup() {
  gpio_set_direction(GPIO_NUM_20, GPIO_MODE_INPUT); 
  gpio_set_direction(GPIO_NUM_19, GPIO_MODE_OUTPUT); 
  setupWifiAndServer();
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);

  setupServo();
  
  if(topHat){
    initI2CMaster();
  }

  if(motor){
    setupMotor();
  }
  stop();
  setupTOF();
}

void loop() {
  if (topHat && millis() - i2cTime >= 500) {
      i2cTime = millis();
      sendI2CByte(packetSent);       // Send a byte to the slave
      packetSent=0; //TODO
      if(receiveI2CByte()==0){
        servo=false;
        stop();
        return;
      }
  }

  h.serve();


  if(servo&&millis()-servoTime>100){
    servoTime=millis();
    if(servoIncrease&&servoPos==180){
      servoIncrease=false;
    }
    else if(!servoIncrease&&servoPos==0){
      servoIncrease=true;
    }
    servoPos+=(servoIncrease?10:-10);
    servoPos = constrain(servoPos, 0, 180);
    ledcWrite(servoPin, map(servoPos, 0, 180, 102, 512));
  }


  if(onRotate&&millis() - rotateStart >= rotateTime){
    stop();
    onRotate=false;
  }


  
  if(TOFSensor&&wallFollowingLeft){
    wallFollowingLeftHand();
  }else if(TOFSensor&&wallFollowingRight){
    wallFollowingRightHand();
  }
  if(onHitButton){
    hitButton();
  }

  if(TOFSensor){
    // int* sensorRanges = read_sensors();
    // Serial.print("right(mm): ");
    // Serial.println(sensorRanges[0]);
    // Serial.print("front(mm): ");
    // Serial.println(sensorRanges[1]);
  }

  if(vive){
    // dualViveTracker.update();
    // float* coordinates = dualViveTracker.printCoordinates();

    // // Access individual values from the array
    // float x1 = coordinates[0];
    // float y1 = coordinates[1];
    // float x2 = coordinates[2];
    // float y2 = coordinates[3];
    // float midX = coordinates[4];
    // float midY = coordinates[5];
    // updateAngle(x1,y1,x2,y2);
    // Serial.println(angle);
    // Serial.printf("Tracker 1: X = %.2f, Y = %.2f\n", x1, y1);
    // Serial.printf("Tracker 2: X = %.2f, Y = %.2f\n", x2, y2);
    // Serial.printf("Midpoint: X = %.2f, Y = %.2f\n", midX, midY);
  }

  if(motor){
    loopMotor();
  }

}
