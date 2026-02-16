#ifndef BLE_HANDLER_H
#define BLE_HANDLER_H

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// External BLE objects accessible to other modules
extern BLEServer* pServer;
extern BLECharacteristic* pMotorCharacteristic;
extern BLECharacteristic* pTelemetryCharacteristic;
extern bool deviceConnected;

// Function prototypes
void initBLE();

#endif // BLE_HANDLER_H
