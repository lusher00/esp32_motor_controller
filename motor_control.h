#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include <Preferences.h>

// External variables accessible to other modules
extern float targetRPM;
extern float currentRPM;
extern float pwmOutput;
extern bool motorDirection;
extern bool motorEnabled;

// PID gains
extern float Kp;
extern float Ki;
extern float Kd;

// Encoder variables
extern volatile unsigned long encoderCount;
extern volatile unsigned long validPulseCount;
extern unsigned long avgPulseWidth;
extern bool pidEnabled;

// Function prototypes
void initMotorControl();
void handleMotorCommand(char cmd, int value);
void calculateRPMTask();
void pidTask();
void heartbeatTask();
void saveConfig();
void loadConfig();
void resetConfig();

// Interrupt handler
void IRAM_ATTR encoderISR();

#endif // MOTOR_CONTROL_H
