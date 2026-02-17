/*
 * ESP32 Motor Controller v2.0
 * 
 * Features:
 * - Unified packet-based command protocol (BLE, WiFi, UART)
 * - PID speed control with encoder feedback
 * - Direct PWM control mode
 * - Sync pulse detection (34 transitions/rev)
 * - NeoPixel animation
 * - Telemetry streaming
 * - Flash-based configuration storage
 * 
 * Encoder disk: 16 posts (12°) + 1 wide post (20°) + voids (16×8° + 1×20°)
 * 
 * Author: Claude + User
 * Date: 2026
 */

#include "config.h"
#include "motor_control.h"
#include "ble_handler.h"
#include "led_animation.h"
#include "telemetry.h"
#include "commands.h"

// Packet parser for Serial
PacketParser serialParser;

void setup() {
  Serial.begin(115200);
  
  // Wait for serial connection and print boot message
  delay(2000);
  Serial.println("\n\n=================================");
  Serial.println("ESP32 Motor Controller v2.0");
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
  handleSerialPackets();
}

void handleSerialPackets() {
  while (Serial.available()) {
    uint8_t byte = Serial.read();
    
    Packet packet;
    if (serialParser.processByte(byte, packet)) {
      // Valid packet received
      uint8_t responseBuffer[PACKET_MAX_LENGTH];
      size_t responseLength = 0;
      
      // Execute command with ACK enabled
      executeCommand(packet, responseBuffer, &responseLength, true);
      
      // Send response if generated
      if (responseLength > 0) {
        Serial.write(responseBuffer, responseLength);
      }
    }
  }
}
