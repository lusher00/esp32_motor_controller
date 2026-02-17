#include "debug.h"

// Global debug flags (can be toggled at runtime)
DebugFlags debugFlags = {
  .motor_control = false,
  .encoder = false,
  .pov_display = false,
  .ble = false,
  .commands = false,
  .timing = false,
  .telemetry = false,
  .sync_pulse = false
};

// ISR debug counters (incremented in ISR, read in main loop)
ISRDebugCounters isrDebugCounters = {
  .encoder_transitions = 0,
  .sync_pulses = 0,
  .debounce_rejects = 0,
  .pov_updates = 0,
  .last_pulse_width = 0,
  .last_avg_pulse_width = 0
};

void initDebug() {
  DEBUG_INFO("Debug system initialized");
  DEBUG_INFOF("Debug level: %d", DEBUG_LEVEL);
  
  #if DEBUG_LEVEL == 0
    DEBUG_INFO("Production mode - no debug output");
  #elif DEBUG_LEVEL == 1
    DEBUG_INFO("Error messages only");
  #elif DEBUG_LEVEL == 2
    DEBUG_INFO("Warnings and errors");
  #elif DEBUG_LEVEL == 3
    DEBUG_INFO("Info, warnings, and errors");
  #elif DEBUG_LEVEL == 4
    DEBUG_INFO("Debug, info, warnings, and errors");
  #elif DEBUG_LEVEL >= 5
    DEBUG_INFO("Verbose mode - all debug output");
  #endif
}

void setDebugFlag(const char* flag, bool enable) {
  if (strcmp(flag, "motor") == 0) {
    debugFlags.motor_control = enable;
    DEBUG_INFOF("Motor debug: %s", enable ? "ON" : "OFF");
  }
  else if (strcmp(flag, "encoder") == 0) {
    debugFlags.encoder = enable;
    DEBUG_INFOF("Encoder debug: %s", enable ? "ON" : "OFF");
  }
  else if (strcmp(flag, "pov") == 0) {
    debugFlags.pov_display = enable;
    DEBUG_INFOF("POV debug: %s", enable ? "ON" : "OFF");
  }
  else if (strcmp(flag, "ble") == 0) {
    debugFlags.ble = enable;
    DEBUG_INFOF("BLE debug: %s", enable ? "ON" : "OFF");
  }
  else if (strcmp(flag, "commands") == 0) {
    debugFlags.commands = enable;
    DEBUG_INFOF("Commands debug: %s", enable ? "ON" : "OFF");
  }
  else if (strcmp(flag, "timing") == 0) {
    debugFlags.timing = enable;
    DEBUG_INFOF("Timing debug: %s", enable ? "ON" : "OFF");
  }
  else if (strcmp(flag, "telemetry") == 0) {
    debugFlags.telemetry = enable;
    DEBUG_INFOF("Telemetry debug: %s", enable ? "ON" : "OFF");
  }
  else if (strcmp(flag, "sync") == 0) {
    debugFlags.sync_pulse = enable;
    DEBUG_INFOF("Sync pulse debug: %s", enable ? "ON" : "OFF");
  }
  else if (strcmp(flag, "all") == 0) {
    debugFlags.motor_control = enable;
    debugFlags.encoder = enable;
    debugFlags.pov_display = enable;
    debugFlags.ble = enable;
    debugFlags.commands = enable;
    debugFlags.timing = enable;
    debugFlags.telemetry = enable;
    debugFlags.sync_pulse = enable;
    DEBUG_INFOF("All debug flags: %s", enable ? "ON" : "OFF");
  }
  else {
    DEBUG_WARNF("Unknown debug flag: %s", flag);
  }
}

void printDebugStatus() {
  Serial.println("\n=== Debug Status ===");
  Serial.printf("Compile-time level: %d\n", DEBUG_LEVEL);
  Serial.println("\nRuntime flags:");
  Serial.printf("  motor_control: %s\n", debugFlags.motor_control ? "ON" : "OFF");
  Serial.printf("  encoder: %s\n", debugFlags.encoder ? "ON" : "OFF");
  Serial.printf("  pov_display: %s\n", debugFlags.pov_display ? "ON" : "OFF");
  Serial.printf("  ble: %s\n", debugFlags.ble ? "ON" : "OFF");
  Serial.printf("  commands: %s\n", debugFlags.commands ? "ON" : "OFF");
  Serial.printf("  timing: %s\n", debugFlags.timing ? "ON" : "OFF");
  Serial.printf("  telemetry: %s\n", debugFlags.telemetry ? "ON" : "OFF");
  Serial.printf("  sync_pulse: %s\n", debugFlags.sync_pulse ? "ON" : "OFF");
  Serial.println("===================\n");
}

void printISRStats() {
  Serial.println("\n=== ISR Statistics ===");
  Serial.printf("Encoder transitions: %lu\n", isrDebugCounters.encoder_transitions);
  Serial.printf("Sync pulses: %lu\n", isrDebugCounters.sync_pulses);
  Serial.printf("Debounce rejects: %lu\n", isrDebugCounters.debounce_rejects);
  Serial.printf("POV updates: %lu\n", isrDebugCounters.pov_updates);
  Serial.printf("Last pulse width: %lu us\n", isrDebugCounters.last_pulse_width);
  Serial.printf("Average pulse width: %lu us\n", isrDebugCounters.last_avg_pulse_width);
  
  // Calculate percentage of sync pulses
  if (isrDebugCounters.encoder_transitions > 0) {
    float syncPercent = (float)isrDebugCounters.sync_pulses / isrDebugCounters.encoder_transitions * 100.0;
    Serial.printf("Sync percentage: %.2f%%\n", syncPercent);
  }
  
  Serial.println("======================\n");
}

void resetISRStats() {
  isrDebugCounters.encoder_transitions = 0;
  isrDebugCounters.sync_pulses = 0;
  isrDebugCounters.debounce_rejects = 0;
  isrDebugCounters.pov_updates = 0;
  isrDebugCounters.last_pulse_width = 0;
  isrDebugCounters.last_avg_pulse_width = 0;
  
  DEBUG_INFO("ISR statistics reset");
}
