# ESP32 Motor Controller v1.0

Professional ESP32-based motor controller with BLE interface, PID speed control, and encoder feedback.

## Features

- **BLE Control Interface** - Wireless motor control via Bluetooth Low Energy
- **PID Speed Control** - Precise RPM control with tunable PID gains
- **Encoder Feedback** - 64 transitions/rev with sync pulse detection
- **NeoPixel Animation** - 34 LED status display
- **Telemetry Streaming** - Real-time sensor data via BLE
- **Flash Configuration** - Settings persist across reboots

## Hardware Requirements

- ESP32 development board
- DC motor with H-bridge driver
- Optical encoder (64 transitions/rev)
- WS2812B LED strip (34 LEDs)
- Temperature sensor (analog)
- Distance sensor (analog)

## Pin Configuration

| Function | Pin |
|----------|-----|
| NeoPixel Data | 13 |
| Encoder Input | 14 |
| Motor PWM | 6 |
| Motor Direction | 7 |
| Temperature Sensor | A0 |
| Distance Sensor | A1 |
| Heartbeat LED | LED_BUILTIN |

## Installation

1. Open `esp32_motor_controller.ino` in Arduino IDE
2. Install required libraries:
   - Adafruit_NeoPixel
   - ESP32 BLE Arduino (included with ESP32 board support)
3. Select board: **ESP32 Dev Module**
4. Upload to your ESP32

## BLE Commands

Send commands via the Motor Control characteristic (`beb5483e-36e1-4688-b7f5-ea07361b26a8`):

| Command | Format | Description | Example |
|---------|--------|-------------|---------|
| M | M\<value\> | Set target RPM (0-1500) | `M500` = 500 RPM |
| D | D\<0/1\> | Set direction (0=back, 1=fwd) | `D1` = forward |
| E | E\<0/1\> | Enable/disable motor | `E1` = enable |
| P | P\<value\> | Set Kp gain (value/10) | `P25` = Kp=2.5 |
| I | I\<value\> | Set Ki gain (value/100) | `I50` = Ki=0.50 |
| K | K\<value\> | Set Kd gain (value/100) | `K10` = Kd=0.10 |
| L | L\<0-6\> | Set LED pattern (see below) | `L2` = Segments |
| C | C | Save config to flash | `C` |
| R | R | Reset to defaults | `R` |

## LED Patterns (Optimized for POV at 1000 RPM)

The 34-LED strip creates stunning visual effects when spinning:

| Pattern | Command | Effect |
|---------|---------|--------|
| 0 | `L0` | **Rainbow Wheel** - Full color wheel visible when spinning |
| 1 | `L1` | **Speedometer** - Shows current RPM with color gradient |
| 2 | `L2` | **Segments** - 4 colored pie slices (red/green/blue/yellow) |
| 3 | `L3` | **Comet** - Single bright dot with trailing tail |
| 4 | `L4` | **Strobe** - Pulsing glow effect |
| 5 | `L5` | **Spiral** - Alternating bands create hypnotic spiral |
| 6 | `L6` | **Fire** - Flickering warm colors |

Patterns auto-cycle every 10 seconds, or manually select with `L` command.

## Telemetry Data

Subscribe to the Telemetry characteristic (`1c95d5e3-d8f7-413a-bf3d-7a2e5d7be87e`) to receive JSON data every 100ms:

```json
{
  "temp": 25.3,
  "dist": 45.2,
  "speed": 128,
  "rpm": 487.5,
  "enc": 12450
}
```

## Project Structure

```
esp32_motor_controller/
├── esp32_motor_controller.ino  # Main sketch
├── config.h                    # Pin definitions & constants
├── motor_control.h/cpp         # Motor & PID control
├── ble_handler.h/cpp           # BLE communication
├── led_animation.h/cpp         # NeoPixel animations
├── telemetry.h/cpp             # Sensor reading & streaming
└── README.md                   # This file
```

## Tuning PID

1. Start with Kp=2.0, Ki=0.5, Kd=0.1 (defaults)
2. Set a target RPM with `M500`
3. Monitor response via Serial Monitor
4. Adjust gains:
   - **Kp** too high → oscillation
   - **Ki** too high → overshoot
   - **Kd** too high → sluggish response

## Troubleshooting

**Motor doesn't start:**
- Verify motor is enabled (`E1`)
- Check target RPM is > 0 (`M500`)
- Ensure encoder is connected properly

**Erratic speed:**
- Reduce PID gains
- Check encoder wiring
- Verify power supply is stable

**BLE won't connect:**
- Device name: `ESP32_Motor_Control`
- Check phone's Bluetooth is enabled
- Try power cycling the ESP32

## License

MIT License - feel free to use in your projects!

## Author

Your Name - 2026
