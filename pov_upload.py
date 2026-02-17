#!/usr/bin/env python3
"""
POV Animation Uploader

Upload custom animations to ESP32 POV display

Usage:
    python pov_upload.py COM3 animation.json
"""

import serial
import struct
import sys
import time
import json
from PIL import Image
import numpy as np

# Import packet functions from packet_test
from packet_test import build_packet, parse_response, crc16_xmodem

# POV Commands
CMD_SELECT_ANIMATION = 0x70
CMD_SET_FRAME_TIMING = 0x71
CMD_GET_REVOLUTION_COUNT = 0x72
CMD_RESET_REVOLUTION = 0x73
CMD_SET_POV_ENABLE = 0x74
CMD_GET_CURRENT_FRAME = 0x75
CMD_UPLOAD_FRAME_START = 0x76
CMD_UPLOAD_FRAME_DATA = 0x77
CMD_UPLOAD_FRAME_END = 0x78

POV_COLUMNS = 34
POV_LEDS = 34


def send_command(ser, command, payload=b'', wait_response=True, timeout=1.0):
    """Send command and wait for ACK"""
    packet = build_packet(command, payload)
    ser.write(packet)
    
    if wait_response:
        start_time = time.time()
        response_data = bytearray()
        
        while (time.time() - start_time) < timeout:
            if ser.in_waiting > 0:
                response_data.extend(ser.read(ser.in_waiting))
                
                # Try to parse response
                if len(response_data) >= 6:
                    response = parse_response(bytes(response_data))
                    if response:
                        return response
            time.sleep(0.01)
        
        return None
    
    return True


def polar_to_cartesian_image(image_path, columns=34, leds=34):
    """
    Convert a polar coordinate image to column data
    
    Image format: 
    - Width = angular resolution (e.g., 360 pixels = 1° per pixel)
    - Height = radial resolution (should match LED count)
    
    Returns: [columns][leds][3] array
    """
    img = Image.open(image_path).convert('RGB')
    width, height = img.size
    
    # Resize height to match LED count
    if height != leds:
        img = img.resize((width, leds), Image.LANCZOS)
        height = leds
    
    # Convert to numpy array
    img_array = np.array(img)
    
    # Sample columns from image
    column_data = []
    for col in range(columns):
        # Calculate which column in the image to sample
        img_col = int((col / columns) * width)
        
        # Extract column (top to bottom = inner to outer radius)
        column = img_array[:, img_col, :]
        column_data.append(column)
    
    return column_data


def upload_animation(ser, animation_id, frames, revolutions_per_frame=1):
    """
    Upload animation to ESP32
    
    frames: list of frame data, where each frame is [columns][leds][3]
    """
    num_frames = len(frames)
    
    print(f"Uploading animation {animation_id} with {num_frames} frames...")
    
    # Start upload
    payload = struct.pack('<BH', animation_id, num_frames)
    response = send_command(ser, CMD_UPLOAD_FRAME_START, payload, timeout=2.0)
    
    if not response or response['command'] != 0x80:  # ACK
        print("Failed to start upload")
        return False
    
    # Upload each frame
    for frame_num, frame in enumerate(frames):
        print(f"Uploading frame {frame_num + 1}/{num_frames}...", end='\r')
        
        for col_num, column in enumerate(frame):
            # Build payload: frame_num (uint16) + column (uint8) + RGB data (102 bytes)
            payload = struct.pack('<HB', frame_num, col_num)
            
            # Flatten RGB data
            rgb_data = b''
            for led in column:
                rgb_data += bytes([led[0], led[1], led[2]])
            
            payload += rgb_data
            
            # Send with minimal delay
            response = send_command(ser, CMD_UPLOAD_FRAME_DATA, payload, timeout=0.5)
            
            if not response or response['command'] != 0x80:
                print(f"\nFailed to upload frame {frame_num}, column {col_num}")
                return False
    
    print(f"\nUploaded all {num_frames} frames")
    
    # End upload
    response = send_command(ser, CMD_UPLOAD_FRAME_END, timeout=2.0)
    
    if not response or response['command'] != 0x80:
        print("Failed to end upload")
        return False
    
    # Set frame timing
    payload = struct.pack('<H', revolutions_per_frame)
    send_command(ser, CMD_SET_FRAME_TIMING, payload)
    
    print(f"Animation {animation_id} uploaded successfully!")
    return True


def create_test_pattern():
    """Create a simple test pattern"""
    frames = []
    
    # Single frame - rainbow gradient
    frame = []
    for col in range(POV_COLUMNS):
        column = []
        angle = (col / POV_COLUMNS) * 360
        
        for led in range(POV_LEDS):
            radius = led / POV_LEDS
            
            # HSV to RGB (simplified)
            hue = angle
            r = int(255 * max(0, min(1, 1.5 - abs((hue / 60) % 6 - 3))) * radius)
            g = int(255 * max(0, min(1, 1.5 - abs(((hue / 60) - 2) % 6 - 3))) * radius)
            b = int(255 * max(0, min(1, 1.5 - abs(((hue / 60) - 4) % 6 - 3))) * radius)
            
            column.append([r, g, b])
        
        frame.append(column)
    
    frames.append(frame)
    return frames


def main():
    if len(sys.argv) < 2:
        print("Usage: python pov_upload.py <serial_port> [image.png]")
        print("Example: python pov_upload.py COM3 clock.png")
        sys.exit(1)
    
    port = sys.argv[1]
    
    print(f"Connecting to {port}...")
    
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        time.sleep(2)
        print("Connected!")
        
        if len(sys.argv) >= 3:
            # Upload image file
            image_path = sys.argv[2]
            print(f"Converting image: {image_path}")
            
            frames = [polar_to_cartesian_image(image_path, POV_COLUMNS, POV_LEDS)]
            upload_animation(ser, 0, frames, revolutions_per_frame=1)
        else:
            # Upload test pattern
            print("Creating test pattern...")
            frames = create_test_pattern()
            upload_animation(ser, 0, frames, revolutions_per_frame=1)
        
        # Select and enable animation
        print("\nActivating animation...")
        send_command(ser, CMD_SELECT_ANIMATION, struct.pack('B', 0))
        send_command(ser, CMD_SET_POV_ENABLE, struct.pack('B', 1))
        
        print("\nPOV display enabled! Animation should be visible when motor is spinning.")
        
        ser.close()
        
    except serial.SerialException as e:
        print(f"Error: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
