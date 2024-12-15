#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <Arduino.h>

// Pin definitions
extern int ENLeftFront;
extern int IN1LeftFront;
extern int IN2LeftFront;
extern int ENCODERLeftFront;

extern int ENLeftRear;
extern int IN1LeftRear;
extern int IN2LeftRear;
extern int ENCODERLeftRear;

extern int ENRightFront;
extern int IN1RightFront;
extern int IN2RightFront;
extern int ENCODERRightFront;

extern int ENRightRear;
extern int IN1RightRear;
extern int IN2RightRear;
extern int ENCODERRightRear;

// Mutexes for ISR safety
extern portMUX_TYPE muxLeftFront;
extern portMUX_TYPE muxLeftRear;
extern portMUX_TYPE muxRightFront;
extern portMUX_TYPE muxRightRear;

// Encoder counters
extern volatile int encoderCountLeftFront;
extern volatile int encoderCountLeftRear;
extern volatile int encoderCountRightFront;
extern volatile int encoderCountRightRear;

// Motor direction variables
extern int dirLeftFront;
extern int dirLeftRear;
extern int dirRightFront;
extern int dirRightRear;

// Timing variables
extern unsigned long previousMillis;
extern unsigned long pidInterval;

// Motor control constants
extern int freq;
extern int resolution;

// PID control variables
extern double dt;
extern double previousLeftFront;
extern double previousLeftRear;
extern double previousRightFront;
extern double previousRightRear;
extern double kp, kd;
extern double desiredRPMLeftFront;
extern double desiredRPMLeftRear;
extern double desiredRPMRightFront;
extern double desiredRPMRightRear;

// Duty cycles
extern int dutyCycleLeftFront;
extern int dutyCycleLeftRear;
extern int dutyCycleRightFront;
extern int dutyCycleRightRear;

// Function declarations
void IRAM_ATTR encoderISRLeftFront();
void IRAM_ATTR encoderISRLeftRear();
void IRAM_ATTR encoderISRRightFront();
void IRAM_ATTR encoderISRRightRear();

void setupEncoder();
void setDir();
void setupMotor();
void loopMotor();
void forward(int forward_RPM);
void backward(int backward_RPM);
void turnLeft(int turn_RPM);
void turnRight(int turn_RPM);
void stop();
void setMotorParameters(double leftFrontRPM, int leftFrontDir,
                        double leftRearRPM, int leftRearDir,
                        double rightFrontRPM, int rightFrontDir,
                        double rightRearRPM, int rightRearDir);
void setSpeed(double leftFrontRPM,
              double leftRearRPM,
              double rightFrontRPM, 
              double rightRearRPM);
double pid(double error, double& previous);

#endif // MOTORCONTROL_H
