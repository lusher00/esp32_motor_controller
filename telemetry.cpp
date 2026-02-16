#include "telemetry.h"
#include "config.h"
#include "ble_handler.h"
#include "motor_control.h"

unsigned long lastTelemetryUpdate = 0;

void initTelemetry() {
  pinMode(TEMP_SENSOR_PIN, INPUT);
  pinMode(DISTANCE_SENSOR_PIN, INPUT);
  Serial.println("✓ Sensors configured");
}

void sendTelemetry() {
  // Read sensors
  int tempRaw = analogRead(TEMP_SENSOR_PIN);
  int distanceRaw = analogRead(DISTANCE_SENSOR_PIN);
  
  // Convert to meaningful values
  float temperature = (tempRaw * 3.3 / 4095.0) * 100.0;
  float distance = (distanceRaw * 3.3 / 4095.0) * 200.0;
  
  // Format telemetry
  char telemetryData[150];
  
  snprintf(telemetryData, sizeof(telemetryData), 
           "{\"temp\":%.1f,\"dist\":%.1f,\"speed\":%d,\"rpm\":%.1f,\"enc\":%lu}",
           temperature, distance, (int)pwmOutput, currentRPM, encoderCount);
  
  pTelemetryCharacteristic->setValue(telemetryData);
  pTelemetryCharacteristic->notify();
  
  // Debug output
  Serial.print("Telemetry sent: ");
  Serial.println(telemetryData);
}

void sendTelemetryTask() {
  if (deviceConnected && (millis() - lastTelemetryUpdate >= TELEMETRY_INTERVAL)) {
    sendTelemetry();
    lastTelemetryUpdate = millis();
  }
}
