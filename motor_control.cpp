#include "motor_control.h"
#include "config.h"
#include "led_animation.h"

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
    return;
  }
  
  // Calculate pulse width
  pulseWidth = currentTime - lastInterruptTime;
  lastInterruptTime = currentTime;
  
  // Always count total transitions
  encoderCount++;
  
  // Check if this looks like a sync pulse
  bool isSync = false;
  if (avgPulseWidth > 0 && pulseWidth > (avgPulseWidth * SYNC_THRESHOLD)) {
    isSync = true;
    syncDetected = true;
    syncPulseWidth = pulseWidth;
    Serial.println("SYNC");
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
  
  // Print high/low based on pin state
  if (digitalRead(ENCODER_PIN) == HIGH) {
    if (isSync) {
      Serial.println("HIGH (SYNC)");
    } else {
      Serial.println("HIGH");
    }
  } else {
    if (isSync) {
      Serial.println("LOW (SYNC)");
    } else {
      Serial.println("LOW");
    }
  }
}

void initMotorControl() {
  // Load saved configuration
  loadConfig();
  
  // Configure motor pins
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  digitalWrite(MOTOR_DIR_PIN, motorDirection ? HIGH : LOW);
  Serial.println("✓ Motor pins configured");
  
  // Configure encoder pin with pullup and interrupt
  pinMode(ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, CHANGE);
  Serial.println("✓ Encoder interrupt attached (Pin 12)");
  
  // Configure heartbeat
  pinMode(HEARTBEAT_PIN, OUTPUT);
  Serial.println("✓ Heartbeat configured");
}

void handleMotorCommand(char cmd, int value) {
  switch(cmd) {
    case 'M': // Set target RPM (0-1500)
      targetRPM = constrain(value, 0, 1500);
      Serial.print("Target RPM set to: ");
      Serial.println(targetRPM);
      saveConfig();
      break;
      
    case 'D': // Set direction (0=backward, 1=forward)
      motorDirection = (value != 0);
      digitalWrite(MOTOR_DIR_PIN, motorDirection ? HIGH : LOW);
      Serial.print("Direction: ");
      Serial.println(motorDirection ? "FORWARD" : "BACKWARD");
      saveConfig();
      break;
      
    case 'E': // Enable/disable motor (0=off, 1=on)
      motorEnabled = (value != 0);
      Serial.println(motorEnabled ? "Motor ENABLED" : "Motor DISABLED");
      break;
      
    case 'P': // Set Kp gain (value / 10)
      Kp = value / 10.0;
      Serial.print("Kp set to: ");
      Serial.println(Kp);
      saveConfig();
      break;
      
    case 'I': // Set Ki gain (value / 100)
      Ki = value / 100.0;
      Serial.print("Ki set to: ");
      Serial.println(Ki);
      saveConfig();
      break;
      
    case 'K': // Set Kd gain (value / 100)
      Kd = value / 100.0;
      Serial.print("Kd set to: ");
      Serial.println(Kd);
      saveConfig();
      break;
      
    case 'L': // Set LED pattern (0-6)
      setLEDPattern(constrain(value, 0, 6));
      Serial.print("LED pattern set to: ");
      Serial.println(value);
      break;
      
    case 'C': // Save current config
      saveConfig();
      Serial.println("Config saved");
      break;
      
    case 'R': // Reset to defaults
      resetConfig();
      Serial.println("Config reset to defaults");
      break;
      
    default:
      Serial.println("Unknown command");
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
  
  Serial.println("✓ Config saved to flash");
}

void loadConfig() {
  prefs.begin("motor", true);
  
  Kp = prefs.getFloat("Kp", 2.0);
  Ki = prefs.getFloat("Ki", 0.5);
  Kd = prefs.getFloat("Kd", 0.1);
  targetRPM = prefs.getFloat("targetRPM", 0.0);
  motorDirection = prefs.getBool("motorDir", true);
  
  prefs.end();
  
  Serial.println("✓ Config loaded from flash");
  Serial.print("  Kp: "); Serial.println(Kp);
  Serial.print("  Ki: "); Serial.println(Ki);
  Serial.print("  Kd: "); Serial.println(Kd);
  Serial.print("  Target RPM: "); Serial.println(targetRPM);
  Serial.print("  Direction: "); Serial.println(motorDirection ? "FORWARD" : "BACKWARD");
}

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
      pidEnabled = true;
      Serial.println("PID ENABLED - Sync pulse detection active");
      Serial.print("Average pulse width: ");
      Serial.print(avgPulseWidth);
      Serial.println(" us");
    }
  }
}

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
    
    // Debug output
    Serial.print("Target: ");
    Serial.print(targetRPM);
    Serial.print(" | Current: ");
    Serial.print(currentRPM);
    Serial.print(" | PWM: ");
    Serial.print((int)pwmOutput);
    Serial.print(" | Valid Pulses: ");
    Serial.println(validPulseCount);
  }
}

void heartbeatTask() {
  if (millis() - lastBeat >= HEARTBEAT_INTERVAL) {
    lastBeat = millis();
    beatState = !beatState;
    digitalWrite(HEARTBEAT_PIN, beatState);
//    int value = digitalRead(ENCODER_PIN);
//    Serial.println(value);  // Should print 1 (HIGH) when floating 
  }
}
