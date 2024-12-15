#include "MotorControl.h"
// Pins
//17,3,35,0 F

// Left Motors
int ENLeftFront =  7;
int IN1LeftFront =  5;
int IN2LeftFront =  6;
int ENCODERLeftFront =  4;

int ENLeftRear = 9;
int IN1LeftRear = 10;
int IN2LeftRear = 11;
int ENCODERLeftRear = 12;

// Right Motors
int ENRightFront = 40;
int IN1RightFront = 42;
int IN2RightFront = 41;
int ENCODERRightFront = 1;

int ENRightRear = 39;
int IN1RightRear = 38;
int IN2RightRear = 37;
int ENCODERRightRear = 36;



// Variables
portMUX_TYPE muxLeftFront = portMUX_INITIALIZER_UNLOCKED; // Critical section guard
portMUX_TYPE muxLeftRear = portMUX_INITIALIZER_UNLOCKED; // Critical section guard
portMUX_TYPE muxRightFront = portMUX_INITIALIZER_UNLOCKED; // Critical section guard
portMUX_TYPE muxRightRear = portMUX_INITIALIZER_UNLOCKED; // Critical section guard

volatile int encoderCountLeftFront = 0; // Volatile for ISR safety
volatile int encoderCountLeftRear = 0; // Volatile for ISR safety
volatile int encoderCountRightFront = 0; // Volatile for ISR safety
volatile int encoderCountRightRear = 0; // Volatile for ISR safety

int dirLeftFront=1;
int dirLeftRear=1;
int dirRightFront=1;
int dirRightRear=1;

unsigned long previousMillis = 0; // Time tracking for RPM calculation
unsigned long pidInterval = 100;  // Interval for RPM update (ms)
int freq=2000;
int resolution=14;

double dt;
double previousLeftFront = 0;
double previousLeftRear = 0;
double previousRightFront = 0;
double previousRightRear = 0;
double kp = 0.30, kd = 0.003;
double desiredRPMLeftFront = 50;
double desiredRPMLeftRear = 50;
double desiredRPMRightFront = 50;
double desiredRPMRightRear = 50;

int dutyCycleLeftFront=0;
int dutyCycleLeftRear=0;
int dutyCycleRightFront=0;
int dutyCycleRightRear=0;

void IRAM_ATTR encoderISRLeftFront() {
  portENTER_CRITICAL_ISR(&muxLeftFront); // Enter critical section
  encoderCountLeftFront++;               // Increment encoder count
  portEXIT_CRITICAL_ISR(&muxLeftFront);  // Exit critical section
}

void IRAM_ATTR encoderISRLeftRear() {
  portENTER_CRITICAL_ISR(&muxLeftRear); // Enter critical section
  encoderCountLeftRear++;               // Increment encoder count
  portEXIT_CRITICAL_ISR(&muxLeftRear);  // Exit critical section
}

void IRAM_ATTR encoderISRRightFront() {
  portENTER_CRITICAL_ISR(&muxRightFront); // Enter critical section
  encoderCountRightFront++;               // Increment encoder count
  portEXIT_CRITICAL_ISR(&muxRightFront);  // Exit critical section
}
void IRAM_ATTR encoderISRRightRear() {
  portENTER_CRITICAL_ISR(&muxRightRear); // Enter critical section
  encoderCountRightRear++;               // Increment encoder count
  portEXIT_CRITICAL_ISR(&muxRightRear);  // Exit critical section
}

void setupEncoder() {
  pinMode(ENCODERLeftFront, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODERLeftFront), encoderISRLeftFront, CHANGE);
  pinMode(ENCODERLeftRear, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODERLeftRear), encoderISRLeftRear, CHANGE);
  pinMode(ENCODERRightFront, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODERRightFront), encoderISRRightFront, CHANGE);
  pinMode(ENCODERRightRear, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODERRightRear), encoderISRRightRear, CHANGE);
}


void setDir() {
  // Set direction for Left Front motor
  if (dirLeftFront == 0) {
    digitalWrite(IN1LeftFront, LOW);
    digitalWrite(IN2LeftFront, LOW);
  } else if (dirLeftFront == 1) {
    digitalWrite(IN1LeftFront, HIGH);
    digitalWrite(IN2LeftFront, LOW);
  } else if (dirLeftFront == -1) {
    digitalWrite(IN1LeftFront, LOW);
    digitalWrite(IN2LeftFront, HIGH);
  }

  // Set direction for Left Rear motor
  if (dirLeftRear == 0) {
    digitalWrite(IN1LeftRear, LOW);
    digitalWrite(IN2LeftRear, LOW);
  } else if (dirLeftRear == 1) {
    digitalWrite(IN1LeftRear, HIGH);
    digitalWrite(IN2LeftRear, LOW);
  } else if (dirLeftRear == -1) {
    digitalWrite(IN1LeftRear, LOW);
    digitalWrite(IN2LeftRear, HIGH);
  }

  // Set direction for Right Front motor
  if (dirRightFront == 0) {
    digitalWrite(IN1RightFront, LOW);
    digitalWrite(IN2RightFront, LOW);
  } else if (dirRightFront == 1) {
    digitalWrite(IN1RightFront, HIGH);
    digitalWrite(IN2RightFront, LOW);
  } else if (dirRightFront == -1) {
    digitalWrite(IN1RightFront, LOW);
    digitalWrite(IN2RightFront, HIGH);
  }

  // Set direction for Right Rear motor
  if (dirRightRear == 0) {
    digitalWrite(IN1RightRear, LOW);
    digitalWrite(IN2RightRear, LOW);
  } else if (dirRightRear == 1) {
    digitalWrite(IN1RightRear, HIGH);
    digitalWrite(IN2RightRear, LOW);
  } else if (dirRightRear == -1) {
    digitalWrite(IN1RightRear, LOW);
    digitalWrite(IN2RightRear, HIGH);
  }
}


void setupMotor() {
  // Serial.begin(115200); // Initialize serial communication

  // Motor driver pins setup
  // Motor driver pins setup for Left Front motor
  pinMode(IN1LeftFront, OUTPUT);
  pinMode(IN2LeftFront, OUTPUT);
  pinMode(ENLeftFront, OUTPUT);

  // Motor driver pins setup for Left Rear motor
  pinMode(IN1LeftRear, OUTPUT);
  pinMode(IN2LeftRear, OUTPUT);
  pinMode(ENLeftRear, OUTPUT);

  // Motor driver pins setup for Right Front motor
  pinMode(IN1RightFront, OUTPUT);
  pinMode(IN2RightFront, OUTPUT);
  pinMode(ENRightFront, OUTPUT);

  // Motor driver pins setup for Right Rear motor
  pinMode(IN1RightRear, OUTPUT);
  pinMode(IN2RightRear, OUTPUT);
  pinMode(ENRightRear, OUTPUT);
  

  setupEncoder();
  ledcAttach(ENLeftFront, freq, resolution);
  ledcAttach(ENLeftRear, freq, resolution);
  ledcAttach(ENRightFront, freq, resolution);
  ledcAttach(ENRightRear, freq, resolution);
}

void loopMotor() {
  setDir();

  // Calculate RPM every pidInterval milliseconds
  if (millis() - previousMillis >= pidInterval) {
    unsigned long currentMillis = millis();
    dt = (currentMillis - previousMillis) / 1000.0;
    unsigned long elapsedTime = currentMillis - previousMillis;

    
    // RPM Calculation
    // PPR = 12, 1 revolution = 12 ticks, 60000 ms in 1 minute gear ratio 48
    double rpmLeftFront = (encoderCountLeftFront / 408.0) / (elapsedTime/60000.0);
    double rpmLeftRear = (encoderCountLeftRear / 540.0) / (elapsedTime/60000.0);
    double rpmRightFront = (encoderCountRightFront / 540.0) / (elapsedTime/60000.0);
    double rpmRightRear = (encoderCountRightRear / 408.0) / (elapsedTime/60000.0);
    
    // //Print RPM to Serial Monitor
    // Serial.print("RPM (Left Front): ");
    // Serial.print(rpmLeftFront);
    // Serial.print(",");

    // Serial.print("RPM (Left Rear): ");
    // Serial.print(rpmLeftRear);
    // Serial.print(",");

    // Serial.print("RPM (Right Front): ");
    // Serial.print(rpmRightFront);
    // Serial.print(",");

    // Serial.print("RPM (Right Rear): ");
    // Serial.print(rpmRightRear);
    // Serial.println(",");

    double errorLeftFront = desiredRPMLeftFront - rpmLeftFront;
    double errorLeftRear = desiredRPMLeftRear - rpmLeftRear;
    double errorRightFront = desiredRPMRightFront - rpmRightFront;
    double errorRightRear = desiredRPMRightRear - rpmRightRear;
    
    double outputLeftFront = pid(errorLeftFront, previousLeftFront);
    double outputLeftRear = pid(errorLeftRear, previousLeftRear);
    double outputRightFront = pid(errorRightFront, previousRightFront);
    double outputRightRear = pid(errorRightRear, previousRightRear);

    dutyCycleLeftFront = constrain(outputLeftFront + dutyCycleLeftFront, 0, 100);
    dutyCycleLeftRear = constrain(outputLeftRear + dutyCycleLeftRear, 0, 100);
    dutyCycleRightFront = constrain(outputRightFront + dutyCycleRightFront, 0, 100);
    dutyCycleRightRear = constrain(outputRightRear + dutyCycleRightRear, 0, 100);

    ledcWrite(ENLeftFront, map(dutyCycleLeftFront, 0, 100, 0, 16382));
    ledcWrite(ENLeftRear, map(dutyCycleLeftRear, 0, 100, 0, 16382));
    ledcWrite(ENRightFront, map(dutyCycleRightFront, 0, 100, 0, 16382));
    ledcWrite(ENRightRear, map(dutyCycleRightRear, 0, 100, 0, 16382));

    
    // Update the previousMillis timestamp
    encoderCountLeftFront=0;
    encoderCountLeftRear=0;
    encoderCountRightFront=0;
    encoderCountRightRear=0;
    previousMillis = millis();
  }
}

double pid(double error,double& previous) { //todo can add ki and kd
    double proportional = error;
    // integral += error * dt;
    // integral = constrain(integral, -100, 100);
    double derivative = (error - previous) / dt;
    previous = error;
    double delta_u = (kp * proportional) + (kd * derivative); //+ (ki * integral) +
    return delta_u;
}
void forward(int forward_RPM){
  setMotorParameters(forward_RPM, 1, forward_RPM, 1, forward_RPM, 1, forward_RPM, 1);
}

void backward(int backward_RPM){
  setMotorParameters(backward_RPM, -1, backward_RPM, -1, backward_RPM, -1, backward_RPM, -1);
}

void turnLeft(int turn_RPM){
  setMotorParameters(turn_RPM, -1, turn_RPM, -1, turn_RPM, 1, turn_RPM, 1);
}

void turnRight(int turn_RPM){
  setMotorParameters(turn_RPM, 1, turn_RPM, 1, turn_RPM, -1, turn_RPM, -1);
}

void stop(){
  setMotorParameters(0, 0, 0, 0, 0, 0, 0, 0);
}

void setMotorParameters(double leftFrontRPM, int leftFrontDir, 
                        double leftRearRPM, int leftRearDir, 
                        double rightFrontRPM, int rightFrontDir, 
                        double rightRearRPM, int rightRearDir) {
  // Update desired RPMs
  desiredRPMLeftFront = leftFrontRPM;
  desiredRPMLeftRear = leftRearRPM;
  desiredRPMRightFront = rightFrontRPM;
  desiredRPMRightRear = rightRearRPM;

  // Update directions
  dirLeftFront = leftFrontDir;
  dirLeftRear = leftRearDir;
  dirRightFront = rightFrontDir;
  dirRightRear = rightRearDir;
}
void setSpeed(double leftFrontRPM,
                        double leftRearRPM,
                        double rightFrontRPM, 
                        double rightRearRPM) {
  // Update desired RPMs
  desiredRPMLeftFront = leftFrontRPM;
  desiredRPMLeftRear = leftRearRPM;
  desiredRPMRightFront = rightFrontRPM;
  desiredRPMRightRear = rightRearRPM;
}

