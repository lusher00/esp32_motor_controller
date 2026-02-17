#include "ble_handler.h"
#include "motor_control.h"
#include "config.h"
#include "commands.h"

BLEServer* pServer = NULL;
BLECharacteristic* pMotorCharacteristic = NULL;
BLECharacteristic* pTelemetryCharacteristic = NULL;
bool deviceConnected = false;

// Packet parser for BLE
PacketParser bleParser;

class ServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Device connected");
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Device disconnected");
    BLEDevice::startAdvertising();
  }
};

class MotorCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    
    if (value.length() > 0) {
      // Try packet protocol first
      bool packetReceived = false;
      Packet packet;
      
      for (size_t i = 0; i < value.length(); i++) {
        if (bleParser.processByte(value[i], packet)) {
          packetReceived = true;
          
          // Execute command
          uint8_t responseBuffer[PACKET_MAX_LENGTH];
          size_t responseLength = 0;
          
          executeCommand(packet, responseBuffer, &responseLength, true);
          
          // Send response via telemetry characteristic if generated
          if (responseLength > 0 && pTelemetryCharacteristic) {
            pTelemetryCharacteristic->setValue(responseBuffer, responseLength);
            pTelemetryCharacteristic->notify();
          }
        }
      }
      
      // Fall back to legacy command format if no packet detected
      if (!packetReceived && value.length() > 0) {
        char cmd = value[0];
        int val = 0;
        if (value.length() > 1) {
          val = atoi(value.substr(1).c_str());
        }
        
        Serial.print("Legacy command received: ");
        Serial.println(cmd);
        Serial.print("Value: ");
        Serial.println(val);
        
        handleMotorCommand(cmd, val);
      }
    }
  }
};

void initBLE() {
  Serial.println("Initializing Bluetooth...");
  BLEDevice::init("ESP32_Motor_Control");
  
  // Create BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  
  // Create BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  // Create Motor Control Characteristic (Write)
  pMotorCharacteristic = pService->createCharacteristic(
                    MOTOR_CHAR_UUID,
                    BLECharacteristic::PROPERTY_WRITE
                  );
  pMotorCharacteristic->setCallbacks(new MotorCallbacks());
  
  // Create Telemetry Characteristic (Read + Notify)
  pTelemetryCharacteristic = pService->createCharacteristic(
                    TELEMETRY_CHAR_UUID,
                    BLECharacteristic::PROPERTY_READ |
                    BLECharacteristic::PROPERTY_NOTIFY
                  );
  pTelemetryCharacteristic->addDescriptor(new BLE2902());
  
  // Start service
  pService->start();
  
  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  
  Serial.println("✓ BLE Server started");
  Serial.println("Device name: ESP32_Motor_Control");
}
