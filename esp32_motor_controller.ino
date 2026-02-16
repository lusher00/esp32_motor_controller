/*
 * ESP32 Motor Controller v1.0
 * 
 * Features:
 * - BLE control interface
 * - PID speed control with encoder feedback
 * - Sync pulse detection (64 transitions/rev)
 * - NeoPixel animation
 * - Telemetry streaming
 * - Flash-based configuration storage
 * 
 * Author: Your Name
 * Date: 2026
 */

#include "config.h"
#include "motor_control.h"
#include "ble_handler.h"
#include "led_animation.h"
#include "telemetry.h"

void setup() {
  Serial.begin(115200);
  
  // Wait for serial connection and print boot message
  delay(2000);
  Serial.println("\n\n=================================");
  Serial.println("ESP32 Motor Controller v1.0");
  Serial.println("=================================");
  Serial.println("Booting...");
  
  // Initialize all subsystems
  initMotorControl();
  initBLE();
  initLEDs();
  initTelemetry();
  
  Serial.println("\n=================================");
  Serial.println("READY - Waiting for connections");
  Serial.println("=================================\n");
}

void loop() {
  heartbeatTask();
  animationTask();
  sendTelemetryTask();
  calculateRPMTask();
  pidTask();
}
