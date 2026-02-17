#ifndef COMMANDS_H
#define COMMANDS_H

#include <Arduino.h>

// ============================================================================
// PACKET PROTOCOL DEFINITION
// ============================================================================

// Packet structure:
// [START_BYTE] [LENGTH] [COMMAND] [PAYLOAD...] [CRC16_HIGH] [CRC16_LOW] [END_BYTE]
//
// START_BYTE: 0xAA (1 byte)
// LENGTH: Total packet length including start/end (1 byte)
// COMMAND: Command ID (1 byte)
// PAYLOAD: 0-1018 bytes of data
// CRC16: CRC-16/XMODEM checksum (2 bytes, big-endian)
// END_BYTE: 0x55 (1 byte)
//
// Minimum packet: 6 bytes (no payload)
// Maximum packet: 1024 bytes (1018 bytes payload)

#define PACKET_START_BYTE    0xAA
#define PACKET_END_BYTE      0x55
#define PACKET_MIN_LENGTH    6
#define PACKET_MAX_LENGTH    1024
#define PACKET_MAX_PAYLOAD   1018

// Packet parser states
enum PacketState {
  WAITING_FOR_START,
  READING_LENGTH,
  READING_COMMAND,
  READING_PAYLOAD,
  READING_CRC_HIGH,
  READING_CRC_LOW,
  READING_END
};

// ============================================================================
// COMMAND DEFINITIONS
// ============================================================================

// Motor Control Commands (0x10 - 0x1F)
#define CMD_SET_TARGET_RPM       0x10  // Payload: float (4 bytes)
#define CMD_SET_DIRECTION        0x11  // Payload: uint8 (0=backward, 1=forward)
#define CMD_SET_MOTOR_ENABLE     0x12  // Payload: uint8 (0=off, 1=on)
#define CMD_SET_PWM_DIRECT       0x13  // Payload: uint8 (0-255) - bypasses PID
#define CMD_EMERGENCY_STOP       0x14  // No payload

// PID Control Commands (0x20 - 0x2F)
#define CMD_SET_PID_ENABLE       0x20  // Payload: uint8 (0=off, 1=on)
#define CMD_SET_KP               0x21  // Payload: float (4 bytes)
#define CMD_SET_KI               0x22  // Payload: float (4 bytes)
#define CMD_SET_KD               0x23  // Payload: float (4 bytes)
#define CMD_SET_PID_ALL          0x24  // Payload: 3 floats (12 bytes: Kp, Ki, Kd)

// Configuration Commands (0x30 - 0x3F)
#define CMD_SAVE_CONFIG          0x30  // No payload
#define CMD_LOAD_CONFIG          0x31  // No payload
#define CMD_RESET_CONFIG         0x32  // No payload
#define CMD_SET_LED_PATTERN      0x33  // Payload: uint8 (0-6)

// Query Commands (0x40 - 0x4F)
#define CMD_GET_STATUS           0x40  // No payload - returns full status
#define CMD_GET_RPM              0x41  // No payload - returns current RPM
#define CMD_GET_PID_PARAMS       0x42  // No payload - returns Kp, Ki, Kd
#define CMD_GET_ENCODER_COUNT    0x43  // No payload - returns encoder counts

// Response Commands (0x80 - 0xFF)
#define CMD_ACK                  0x80  // Payload: uint8 (original command ID)
#define CMD_NACK                 0x81  // Payload: uint8 (original command ID) + uint8 (error code)
#define CMD_STATUS_RESPONSE      0x82  // Payload: status structure
#define CMD_RPM_RESPONSE         0x83  // Payload: float (current RPM)
#define CMD_PID_RESPONSE         0x84  // Payload: 3 floats (Kp, Ki, Kd)
#define CMD_ENCODER_RESPONSE     0x85  // Payload: 2 uint32 (total count, valid count)

// Error Codes for NACK
#define ERR_INVALID_COMMAND      0x01
#define ERR_INVALID_LENGTH       0x02
#define ERR_INVALID_PAYLOAD      0x03
#define ERR_CRC_MISMATCH         0x04
#define ERR_OUT_OF_RANGE         0x05
#define ERR_MOTOR_FAULT          0x06
#define ERR_TIMEOUT              0x07

// ============================================================================
// PACKET STRUCTURE
// ============================================================================

struct Packet {
  uint8_t command;
  uint8_t payloadLength;
  uint8_t payload[PACKET_MAX_PAYLOAD];
  bool requiresAck;
  
  Packet() : command(0), payloadLength(0), requiresAck(false) {}
};

// ============================================================================
// STATUS STRUCTURE (for CMD_STATUS_RESPONSE)
// ============================================================================

struct __attribute__((packed)) StatusData {
  float currentRPM;
  float targetRPM;
  float pwmOutput;
  uint8_t motorEnabled;
  uint8_t motorDirection;
  uint8_t pidEnabled;
  uint32_t encoderCount;
  uint32_t validPulseCount;
  float Kp;
  float Ki;
  float Kd;
};

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================

// CRC-16/XMODEM calculation
uint16_t calculateCRC16(const uint8_t* data, size_t length);

// Packet building
size_t buildPacket(uint8_t* buffer, uint8_t command, const uint8_t* payload, uint8_t payloadLength);

// Build specific response packets
size_t buildAckPacket(uint8_t* buffer, uint8_t originalCommand);
size_t buildNackPacket(uint8_t* buffer, uint8_t originalCommand, uint8_t errorCode);
size_t buildStatusPacket(uint8_t* buffer, const StatusData& status);
size_t buildRPMPacket(uint8_t* buffer, float rpm);
size_t buildPIDPacket(uint8_t* buffer, float kp, float ki, float kd);
size_t buildEncoderPacket(uint8_t* buffer, uint32_t totalCount, uint32_t validCount);

// Command execution
bool executeCommand(const Packet& packet, uint8_t* responseBuffer, size_t* responseLength, bool sendAck);

// Packet parser class
class PacketParser {
private:
  PacketState state;
  uint8_t buffer[PACKET_MAX_LENGTH];
  size_t bufferIndex;
  uint8_t packetLength;
  uint8_t command;
  uint16_t receivedCRC;
  
public:
  PacketParser();
  void reset();
  bool processByte(uint8_t byte, Packet& packet);
};

#endif // COMMANDS_H
