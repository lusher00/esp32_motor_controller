#!/usr/bin/env python3
"""
ESP32 Motor Controller - Packet Command Interface
Test script for sending commands over Serial

Usage:
    python packet_test.py COM3      # Windows
    python packet_test.py /dev/ttyUSB0  # Linux
"""

import serial
import struct
import sys
import time

# Packet constants
PACKET_START_BYTE = 0xAA
PACKET_END_BYTE = 0x55

# Command definitions
CMD_SET_TARGET_RPM = 0x10
CMD_SET_DIRECTION = 0x11
CMD_SET_MOTOR_ENABLE = 0x12
CMD_SET_PWM_DIRECT = 0x13
CMD_EMERGENCY_STOP = 0x14

CMD_SET_PID_ENABLE = 0x20
CMD_SET_KP = 0x21
CMD_SET_KI = 0x22
CMD_SET_KD = 0x23
CMD_SET_PID_ALL = 0x24

CMD_SAVE_CONFIG = 0x30
CMD_LOAD_CONFIG = 0x31
CMD_RESET_CONFIG = 0x32
CMD_SET_LED_PATTERN = 0x33

CMD_GET_STATUS = 0x40
CMD_GET_RPM = 0x41
CMD_GET_PID_PARAMS = 0x42
CMD_GET_ENCODER_COUNT = 0x43

# Response commands
CMD_ACK = 0x80
CMD_NACK = 0x81
CMD_STATUS_RESPONSE = 0x82
CMD_RPM_RESPONSE = 0x83
CMD_PID_RESPONSE = 0x84
CMD_ENCODER_RESPONSE = 0x85


def crc16_xmodem(data):
    """Calculate CRC-16/XMODEM checksum"""
    crc = 0x0000
    
    for byte in data:
        crc ^= byte << 8
        
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc = crc << 1
            
            crc &= 0xFFFF
    
    return crc


def build_packet(command, payload=b''):
    """Build a packet with given command and payload"""
    packet_length = 6 + len(payload)
    
    # Build packet without CRC
    packet = bytearray()
    packet.append(PACKET_START_BYTE)
    packet.append(packet_length)
    packet.append(command)
    packet.extend(payload)
    
    # Calculate CRC over length, command, and payload
    crc = crc16_xmodem(packet[1:])
    
    # Add CRC (big-endian)
    packet.append((crc >> 8) & 0xFF)
    packet.append(crc & 0xFF)
    
    # Add end byte
    packet.append(PACKET_END_BYTE)
    
    return bytes(packet)


def parse_response(data):
    """Parse a response packet"""
    if len(data) < 6:
        return None
    
    if data[0] != PACKET_START_BYTE or data[-1] != PACKET_END_BYTE:
        return None
    
    packet_length = data[1]
    command = data[2]
    payload_length = packet_length - 6
    
    if payload_length > 0:
        payload = data[3:3+payload_length]
    else:
        payload = b''
    
    # Verify CRC
    received_crc = (data[-3] << 8) | data[-2]
    calculated_crc = crc16_xmodem(data[1:-3])
    
    if received_crc != calculated_crc:
        print(f"CRC mismatch! Received: {received_crc:04X}, Calculated: {calculated_crc:04X}")
        return None
    
    return {
        'command': command,
        'payload': payload
    }


def send_command(ser, command, payload=b'', wait_response=True):
    """Send a command and optionally wait for response"""
    packet = build_packet(command, payload)
    
    print(f"Sending command 0x{command:02X} ({len(payload)} bytes payload)")
    print(f"Packet: {packet.hex()}")
    
    ser.write(packet)
    
    if wait_response:
        time.sleep(0.1)  # Give device time to respond
        
        response_data = bytearray()
        while ser.in_waiting > 0:
            response_data.extend(ser.read(ser.in_waiting))
            time.sleep(0.01)
        
        if response_data:
            response = parse_response(bytes(response_data))
            
            if response:
                if response['command'] == CMD_ACK:
                    print(f"✓ ACK received for command 0x{response['payload'][0]:02X}")
                elif response['command'] == CMD_NACK:
                    error_code = response['payload'][1] if len(response['payload']) > 1 else 0
                    print(f"✗ NACK received - Error code: 0x{error_code:02X}")
                elif response['command'] == CMD_RPM_RESPONSE:
                    rpm = struct.unpack('<f', response['payload'])[0]
                    print(f"Current RPM: {rpm:.1f}")
                elif response['command'] == CMD_PID_RESPONSE:
                    kp, ki, kd = struct.unpack('<fff', response['payload'])
                    print(f"PID Params - Kp: {kp:.2f}, Ki: {ki:.2f}, Kd: {kd:.2f}")
                elif response['command'] == CMD_ENCODER_RESPONSE:
                    total, valid = struct.unpack('<II', response['payload'])
                    print(f"Encoder - Total: {total}, Valid: {valid}")
                else:
                    print(f"Response: Command 0x{response['command']:02X}, Payload: {response['payload'].hex()}")
            else:
                print(f"Invalid response: {response_data.hex()}")
        else:
            print("No response received")
    
    print()


def main():
    if len(sys.argv) < 2:
        print("Usage: python packet_test.py <serial_port>")
        print("Example: python packet_test.py COM3")
        sys.exit(1)
    
    port = sys.argv[1]
    
    print(f"Connecting to {port}...")
    
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        time.sleep(2)  # Wait for device to boot
        
        print("Connected!\n")
        
        # Example commands
        
        # 1. Set PID enable ON
        print("=== Enable PID ===")
        send_command(ser, CMD_SET_PID_ENABLE, struct.pack('B', 1))
        
        # 2. Set target RPM to 1000
        print("=== Set Target RPM to 1000 ===")
        send_command(ser, CMD_SET_TARGET_RPM, struct.pack('<f', 1000.0))
        
        # 3. Set direction to forward
        print("=== Set Direction Forward ===")
        send_command(ser, CMD_SET_DIRECTION, struct.pack('B', 1))
        
        # 4. Enable motor
        print("=== Enable Motor ===")
        send_command(ser, CMD_SET_MOTOR_ENABLE, struct.pack('B', 1))
        
        time.sleep(2)
        
        # 5. Query current RPM
        print("=== Query Current RPM ===")
        send_command(ser, CMD_GET_RPM, wait_response=True)
        
        # 6. Query PID parameters
        print("=== Query PID Parameters ===")
        send_command(ser, CMD_GET_PID_PARAMS, wait_response=True)
        
        # 7. Query encoder counts
        print("=== Query Encoder Counts ===")
        send_command(ser, CMD_GET_ENCODER_COUNT, wait_response=True)
        
        time.sleep(2)
        
        # 8. Switch to direct PWM mode
        print("=== Disable PID ===")
        send_command(ser, CMD_SET_PID_ENABLE, struct.pack('B', 0))
        
        print("=== Set Direct PWM to 128 ===")
        send_command(ser, CMD_SET_PWM_DIRECT, struct.pack('B', 128))
        
        time.sleep(2)
        
        # 9. Emergency stop
        print("=== Emergency Stop ===")
        send_command(ser, CMD_EMERGENCY_STOP)
        
        # 10. Save config
        print("=== Save Config ===")
        send_command(ser, CMD_SAVE_CONFIG)
        
        ser.close()
        print("Done!")
        
    except serial.SerialException as e:
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()
