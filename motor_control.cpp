#include "motor_control.h"
#include "config.h"
#include "led_animation.h"
#include "commands.h"
#include "pov_display.h"
#include "debug.h"

// Global variables
Preferences prefs;

float targetRPM = 0;
float currentRPM = 0;
float pwmOutput = 0;
bool motorDirection = true;
bool motorEnabled = false;

// PID gains
float Kp = 2.0;
float Ki = 0.5;
float Kd = 0.1;

// PID state
float errorSum = 0;
float lastError = 0;
unsigned long lastPIDUpdate = 0;

// Encoder/Interrupt variables
volatile unsigned long encoderCount = 0;
volatile unsigned long validPulseCount = 0;
volatile unsigned long lastInterruptTime = 0;
volatile unsigned long pulseWidth = 0;
volatile bool syncDetected = false;
volatile unsigned long syncPulseWidth = 0;
unsigned long avgPulseWidth = 0;
bool pidEnabled = false;

// RPM calculation
unsigned long lastRPMCalc = 0;
unsigned long lastEncoderCount = 0;

// Heartbeat
unsigned long lastBeat = 0;
bool beatState = false;

void IRAM_ATTR encoderISR() {
  unsigned long currentTime = micros();
  
  // Debounce
  if (currentTime - lastInterruptTime < DEBOUNCE_US) {
    isrDebugCounters.debounce_rejects++;
    return;
  }
  
  // Calculate pulse width
  pulseWidth = currentTime - lastInterruptTime;
  lastInterruptTime = currentTime;
  
  // Always count total transitions
  encoderCount++;
  isrDebugCounters.encoder_transitions++;
  
  // Check if this looks like a sync pulse
  bool isSync = false;
  if (avgPulseWidth > 0 && pulseWidth > (avgPulseWidth * SYNC_THRESHOLD)) {
    isSync = true;
    syncDetected = true;
    syncPulseWidth = pulseWidth;
    isrDebugCounters.sync_pulses++;
  } else {
    // Valid pulse - count it and update running average
    validPulseCount++;
    
    // Update average pulse width (simple moving average)
    if (avgPulseWidth == 0) {
      avgPulseWidth = pulseWidth;
    } else {
      avgPulseWidth = (avgPulseWidth * 9 + pulseWidth) / 10;
    }
  }
  
  // Update debug counters (ISR-safe)
  isrDebugCounters.last_pulse_width = pulseWidth;
  isrDebugCounters.last_avg_pulse_width = avgPulseWidth;
  
  // Update POV display
  povEncoderUpdate(isSync);
  isrDebugCounters.pov_updates++;
}

void initMotorControl() {
  // Load saved configuration
  loadConfig();
  
  // Configure motor pins
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  digitalWrite(MOTOR_DIR_PIN, motorDirection ? HIGH : LOW);
  DEBUG_INFO("✓ Motor pins configured");
  
  // Configure encoder pin with pullup and interrupt
  pinMode(ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, CHANGE);
  DEBUG_INFOF("✓ Encoder interrupt attached (Pin %d)", ENCODER_PIN);
  
  // Configure heartbeat
  pinMode(HEARTBEAT_PIN, OUTPUT);
  DEBUG_INFO("✓ Heartbeat configured");
}

void handleMotorCommand(char cmd, int value) {
  switch(cmd) {
    case 'M': // Set target RPM (0-1500)
      targetRPM = constrain(value, 0, 1500);
      DEBUG_MOTORF("Target RPM set to: %.1f", targetRPM);
      
      saveConfig();
      break;
      
    case 'D': // Set direction (0=backward, 1=forward)
      motorDirection = (value != 0);
      digitalWrite(MOTOR_DIR_PIN, motorDirection ? HIGH : LOW);
      DEBUG_MOTORF("Direction: %s", motorDirection ? "FORWARD" : "BACKWARD");
      
      saveConfig();
      break;
      
    case 'E': // Enable/disable motor (0=off, 1=on)
      motorEnabled = (value != 0);
      DEBUG_MOTOR(motorEnabled ? "Motor ENABLED" : "Motor DISABLED");
      break;
      
    case 'P': // Set Kp gain (value / 10)
      Kp = value / 10.0;
      DEBUG_MOTORF("Kp set to: %.2f", Kp);
      
      saveConfig();
      break;
      
    case 'I': // Set Ki gain (value / 100)
      Ki = value / 100.0;
      DEBUG_MOTORF("Ki set to: %.3f", Ki);
      
      saveConfig();
      break;
      
    case 'K': // Set Kd gain (value / 100)
      Kd = value / 100.0;
      DEBUG_MOTORF("Kd set to: %.3f", Kd);
      
      saveConfig();
      break;
      
    case 'L': // Set LED pattern (0-6)
      setLEDPattern(constrain(value, 0, 6));
      DEBUG_INFOF("LED pattern set to: %d", value);
      
      break;
      
    case 'C': // Save current config
      saveConfig();
      DEBUG_INFO("✓ Config saved");
      break;
      
    case 'R': // Reset to defaults
      resetConfig();
      DEBUG_INFO("✓ Config reset to defaults");
      break;
      
    default:
      DEBUG_WARN("Unknown command");
      break;
  }
}

void saveConfig() {
  prefs.begin("motor", false);
  
  prefs.putFloat("Kp", Kp);
  prefs.putFloat("Ki", Ki);
  prefs.putFloat("Kd", Kd);
  prefs.putFloat("targetRPM", targetRPM);
  prefs.putBool("motorDir", motorDirection);
  
  prefs.end();
  
  DEBUG_INFO("✓ Config saved to flash");
}

void loadConfig() {
  prefs.begin("motor", true);
  
  Kp = prefs.getFloat("Kp", 2.0);
  Ki = prefs.getFloat("Ki", 0.5);
  Kd = prefs.getFloat("Kd", 0.1);
  targetRPM = prefs.getFloat("targetRPM", 0.0);
  motorDirection = prefs.getBool("motorDir", true);
  
  prefs.end();
  
  DEBUG_INFO("✓ Config loaded from flash");

void resetConfig() {
  Kp = 2.0;
  Ki = 0.5;
  Kd = 0.1;
  targetRPM = 0.0;
  motorDirection = true;
  
  saveConfig();
}

void calculateRPMTask() {
  if (millis() - lastRPMCalc >= RPM_INTERVAL) {
    unsigned long pulses = validPulseCount - lastEncoderCount;
    lastEncoderCount = validPulseCount;
    
    // RPM = (pulses / TRANSITIONS_PER_REV) * (60000 / interval_ms)
    currentRPM = (float)pulses / VALID_TRANSITIONS_PER_REV * (60000.0 / RPM_INTERVAL);
    
    lastRPMCalc = millis();
    
    // Enable PID once we have stable pulse width measurements
    if (!pidEnabled && validPulseCount > 20 && avgPulseWidth > 0) {

void pidTask() {
  if (millis() - lastPIDUpdate >= PID_INTERVAL) {
    
    // Only run PID if enabled AND motor is enabled
    if (!pidEnabled || !motorEnabled) {
      errorSum = 0;
      lastError = 0;
      pwmOutput = 0;
      analogWrite(MOTOR_PWM_PIN, 0);
      lastPIDUpdate = millis();
      return;
    }
    
    // Calculate error
    float error = targetRPM - currentRPM;
    
    // Proportional term
    float P = Kp * error;
    
    // Integral term
    errorSum += error * (PID_INTERVAL / 1000.0);
    errorSum = constrain(errorSum, -100, 100); // Anti-windup
    float I = Ki * errorSum;
    
    // Derivative term
    float D = Kd * (error - lastError) / (PID_INTERVAL / 1000.0);
    lastError = error;
    
    // Calculate output
    pwmOutput = P + I + D;
    pwmOutput = constrain(pwmOutput, 0, 255);
    
    // Apply to motor
    analogWrite(MOTOR_PWM_PIN, (int)pwmOutput);
    
    lastPIDUpdate = millis();

void heartbeatTask() {
  if (millis() - lastBeat >= HEARTBEAT_INTERVAL) {
    lastBeat = millis();
    beatState = !beatState;
    digitalWrite(HEARTBEAT_PIN, beatState);
  }
}

// ============================================================================
// NEW PACKET-BASED API FUNCTIONS
// ============================================================================

void setTargetRPM(float rpm) {
  targetRPM = constrain(rpm, 0, 1500);
  DEBUG_MOTORF("Target RPM set to: %.1f", targetRPM);
  
  saveConfig();
}

void setMotorDirection(bool forward) {
  motorDirection = forward;
  digitalWrite(MOTOR_DIR_PIN, motorDirection ? HIGH : LOW);
  DEBUG_MOTORF("Direction: %s", motorDirection ? "FORWARD" : "BACKWARD");
  
  saveConfig();
}

void setMotorEnable(bool enable) {
  motorEnabled = enable;
  DEBUG_MOTOR(motorEnabled ? "Motor ENABLED" : "Motor DISABLED");
}

void setDirectPWM(uint8_t pwm) {
  // Direct PWM mode bypasses PID
  pidEnabled = false;
  analogWrite(MOTOR_PWM_PIN, pwm);
  pwmOutput = pwm;
  DEBUG_MOTORF("Direct PWM set to: %d", pwm);
}

void emergencyStop() {
  motorEnabled = false;
  pidEnabled = false;
  analogWrite(MOTOR_PWM_PIN, 0);
  pwmOutput = 0;
  DEBUG_ERROR("EMERGENCY STOP");
}

void setPIDEnable(bool enable) {
  pidEnabled = enable;
  DEBUG_MOTORF("PID %s", enable ? "ENABLED" : "DISABLED");
}

void setKp(float kp) {
  Kp = kp;
  DEBUG_MOTORF("Kp set to: %.2f", Kp);
  saveConfig();
}

void setKi(float ki) {
  Ki = ki;
  DEBUG_MOTORF("Ki set to: %.3f", Ki);
  saveConfig();
}

void setKd(float kd) {
  Kd = kd;
  DEBUG_MOTORF("Kd set to: %.3f", Kd);
  saveConfig();
}

void setPIDParams(float kp, float ki, float kd) {
  Kp = kp;
  Ki = ki;
  Kd = kd;
  DEBUG_MOTORF("PID params set - Kp: %.2f, Ki: %.3f, Kd: %.3f", Kp, Ki, Kd);
  saveConfig();
}

float getCurrentRPM() {
  return currentRPM;
}

void getPIDParams(float& kp, float& ki, float& kd) {
  kp = Kp;
  ki = Ki;
  kd = Kd;
}

void getEncoderCounts(uint32_t& total, uint32_t& valid) {
  total = encoderCount;
  valid = validPulseCount;
}

void getStatusData(StatusData& status) {
  status.currentRPM = currentRPM;
  status.targetRPM = targetRPM;
  status.pwmOutput = pwmOutput;
  status.motorEnabled = motorEnabled ? 1 : 0;
  status.motorDirection = motorDirection ? 1 : 0;
  status.pidEnabled = pidEnabled ? 1 : 0;
  status.encoderCount = encoderCount;
  status.validPulseCount = validPulseCount;
  status.Kp = Kp;
  status.Ki = Ki;
  status.Kd = Kd;
}

