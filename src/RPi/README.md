# WRO Future Engineers - Split Architecture Implementation

This implementation splits the robot control into two main components:
- **RPI Controller** (`rpi_controller.py`): Vision processing, decision making, and command generation
- **ESP32 Remote** (`esp32_remote.ino`): Motor control, sensor reading, and autonomous fallback

## Architecture Overview

```
┌─────────────────┐    Serial Commands    ┌─────────────────┐
│   Raspberry Pi  │ ──────────────────────→│      ESP32      │
│                 │                        │                 │
│ • Vision        │←──────────────────────│ • Motor Control │
│ • Block Detection│    Sensor Data        │ • Servo Control │
│ • Decision Logic│                        │ • Sensor Reading│
│ • State Machine │                        │ • Autonomous    │
└─────────────────┘                        └─────────────────┘
```

## Command Protocol

### RPI → ESP32 Commands
- `CMD:MOVE,speed,direction,steer` - Move with specified parameters
  - `speed`: 0-255 motor speed
  - `direction`: 0=forward, 1=reverse
  - `steer`: -90 to +90 steering angle
- `CMD:STOP` - Stop all motors immediately
- `CMD:AUTONOMOUS` - Switch to autonomous RTOS mode

### ESP32 → RPI Data
- CSV format: `angle,left,right,front`
- Human-readable status messages
- Mode change notifications

## Safety Features

1. **Command Timeout**: ESP32 automatically switches to autonomous mode if no commands received for 2 seconds
2. **Multi-frame Confirmation**: Reverse decisions require confirmation across multiple frames
3. **Failsafe Modes**: Stuck detection, wrong direction detection, collision avoidance
4. **Error Recovery**: Robust error handling and recovery mechanisms

## Hardware Requirements

### Raspberry Pi
- Camera (USB or CSI)
- Serial connection to ESP32
- Python 3.7+ with required packages

### ESP32
- BNO055 IMU (I2C: SDA=21, SCL=22)
- TF-Luna sensors: Left (UART1: RX=18, TX=19), Right (UART2: RX=17, TX=16)
- TF-Luna front sensor (I2C address 0x10)
- Servo motor (Pin 12)
- Motor driver (Pins: mf=27, mb=14, ME=32)

## Quick Start Guide

### 1. Hardware Setup
1. Connect RPI to ESP32 via USB serial
2. Mount camera on RPI
3. Connect sensors and actuators to ESP32 as specified above

### 2. Software Installation
```bash
# Install required Python packages
pip install opencv-python numpy pyserial

# Install ESP32 libraries (in Arduino IDE):
# - FreeRTOS
# - Adafruit_BNO055
# - ESP32Servo
```

### 3. Camera Calibration
```bash
# Run interactive calibration tool
python camera_calibration.py

# Adjust color thresholds using trackbars
# Save calibration data to camera_calibration.json
```

### 4. Upload ESP32 Firmware
1. Open `esp32_remote/esp32_remote.ino` in Arduino IDE
2. Select ESP32 board and correct port
3. Upload firmware

### 5. Test System
```bash
# Test communication
python test_communication.py

# Test complete system
python test_complete_system.py

# Run main controller
python rpi_controller.py
```

## Testing Tools

### Camera Calibration (`camera_calibration.py`)
Interactive tool for calibrating color detection thresholds:
- Real-time color threshold adjustment using OpenCV trackbars
- LAB color space tuning for optimal block detection  
- Image enhancement controls (brightness, contrast, gamma)
- JSON export/import for persistent calibration settings
- Live preview with color masks and morphological operations

### Communication Test (`test_communication.py`)
Validates the communication protocol between RPI and ESP32:
- Tests command sending and sensor data reception
- Verifies command parsing and error handling
- Protocol validation and timing analysis

### Complete System Test (`test_complete_system.py`)
Comprehensive system validation:
- Camera connectivity and frame capture testing
- Serial communication validation
- Vision processing pipeline verification  
- Decision logic testing with mock data
- Interactive control mode for manual testing

## File Structure

```
├── rpi_controller.py           # Main RPI vision and control system
├── esp32_remote/
│   └── esp32_remote.ino        # ESP32 firmware with FreeRTOS
├── camera_calibration.py       # Interactive color calibration tool
├── test_communication.py       # Communication protocol testing
├── test_complete_system.py     # Full system integration testing
├── camera_calibration.json     # Generated calibration settings (auto-generated)
└── README.md                   # This documentation
```

## Installation

### 1. ESP32 Setup
```bash
# Install required libraries in Arduino IDE:
# - Adafruit BNO055
# - ESP32Servo
# - Wire (built-in)
# - HardwareSerial (built-in)

# Upload esp32_remote.ino to ESP32
```

### 2. Raspberry Pi Setup
```bash
# Install Python dependencies
pip install opencv-python numpy pyserial

# Make sure camera and serial port are accessible
sudo usermod -a -G dialout $USER
sudo usermod -a -G video $USER

# Logout and login again for group changes to take effect
```

### 3. Hardware Connections
```
ESP32 → Sensors:
- Pin 21 (SDA) → BNO055 SDA
- Pin 22 (SCL) → BNO055 SCL
- Pin 18 (RX1) → Left TF-Luna TX
- Pin 19 (TX1) → Left TF-Luna RX
- Pin 17 (RX2) → Right TF-Luna TX
- Pin 16 (TX2) → Right TF-Luna RX
- Pin 12 → Servo signal
- Pin 27 → Motor Forward
- Pin 14 → Motor Backward
- Pin 32 → Motor Enable

RPI → ESP32:
- USB cable for serial communication
```

## Configuration

### RPI Controller (`rpi_controller.py`)
```python
# Serial settings
PORT = "/dev/ttyUSB0"  # Adjust for your system
BAUDRATE = 115200

# Camera settings
CAMERA_DEVICE = "/dev/video0"  # Adjust for your camera

# Speed constants (adjust for your robot)
SPEED = 100
TURNING_SPEED = 80
PARKING_SPEED = 60
```

### ESP32 Remote (`esp32_remote.ino`)
```cpp
// Timeout before switching to autonomous mode
const unsigned long COMMAND_TIMEOUT_MS = 2000;

// Servo limits (adjust for your servo)
const int MAX_LEFT = 160;
const int MAX_RIGHT = 20;
const int STRAIGHT_ANGLE = 90;

// Sensor thresholds for autonomous mode
const int FRONT_THRESHOLD = 120;   // cm
const int SIDE_THRESHOLD = 100;    // cm
```

## Testing

### 1. Communication Test
```bash
cd /path/to/project
python test_communication.py [/dev/ttyUSB0]
```

This will:
- Test handshake protocol
- Verify sensor data reception
- Test all command types
- Provide interactive control mode

### 2. Vision Test
```bash
# Enable debug mode in rpi_controller.py
DEBUG = True

# Run the main controller
python rpi_controller.py
```

### 3. Individual Component Tests

#### Test ESP32 Autonomous Mode
1. Upload `esp32_remote.ino`
2. Open Serial Monitor
3. Send `CMD:AUTONOMOUS`
4. Verify autonomous navigation works

#### Test RPI Vision Processing
1. Run `rpi_controller.py` with `DEBUG = True`
2. Verify block detection visualization
3. Check decision logic output

## Usage

### Normal Operation
```bash
# 1. Power on ESP32 and wait for "OK" message
# 2. Start RPI controller
python rpi_controller.py

# The system will automatically:
# - Establish communication
# - Exit parking lot
# - Navigate the course
# - Handle block avoidance
# - Execute parking sequence
```

### Manual Control Mode
```bash
# Use the test script for manual control
python test_communication.py

# Interactive commands:
# f [speed] - Move forward
# b [speed] - Move backward
# l [steer] - Turn left
# r [steer] - Turn right
# s - Stop
# a - Autonomous mode
# q - Quit
```

## Troubleshooting

### Common Issues

1. **Serial Connection Failed**
   ```bash
   # Check port permissions
   ls -l /dev/ttyUSB*
   sudo chmod 666 /dev/ttyUSB0
   
   # Check if port is in use
   lsof /dev/ttyUSB0
   ```

2. **Camera Not Found**
   ```bash
   # List available cameras
   ls /dev/video*
   v4l2-ctl --list-devices
   
   # Test camera
   python -c "import cv2; cap=cv2.VideoCapture(0); print(cap.isOpened())"
   ```

3. **ESP32 Not Responding**
   - Check power supply
   - Verify upload was successful
   - Check serial monitor for error messages
   - Press reset button on ESP32

4. **Sensor Reading Issues**
   - Check I2C connections (SDA/SCL)
   - Verify UART connections
   - Check sensor power supply
   - Use I2C scanner sketch to detect BNO055

5. **Block Detection Problems**
   - Adjust lighting conditions
   - Calibrate color ranges in `COLOR_RANGES`
   - Check camera focus and position
   - Enable debug mode to see detection visualization

### Debug Mode

Enable debug output:
```python
# In rpi_controller.py
DEBUG = True
```

This will show:
- Block detection visualization
- Sensor data in real-time
- Decision logic output
- Communication status

### Performance Optimization

1. **Reduce CPU Usage**
   ```python
   # Adjust camera resolution
   CAM_W = "640"
   CAM_H = "480"
   
   # Reduce OpenCV threads
   cv2.setNumThreads(1)
   ```

2. **Improve Response Time**
   ```cpp
   // Increase control task priority in ESP32
   xTaskCreatePinnedToCore(controlTask, "ControlTask", 6144, NULL, 5, NULL, 1);
   ```

## Development

### Adding New Commands

1. **Define command in ESP32**:
   ```cpp
   // In parseCommand function
   else if (input == "NEW_COMMAND") {
     cmd.type = Command::NEW_COMMAND;
     return true;
   }
   ```

2. **Handle in control task**:
   ```cpp
   // In controlTask
   case Command::NEW_COMMAND:
     // Implementation
     break;
   ```

3. **Send from RPI**:
   ```python
   self.comm.send_command("NEW_COMMAND")
   ```

### Modifying Block Detection

Color ranges are defined in `COLOR_RANGES` dictionary:
```python
COLOR_RANGES = {
    "red":    ([L_min, A_min, B_min], [L_max, A_max, B_max]),
    "green":  ([L_min, A_min, B_min], [L_max, A_max, B_max]),
    # Add new colors here
}
```

### Tuning Parameters

- **Turning matrices**: `red_turning_values`, `green_turning_values`
- **Speed constants**: `SPEED`, `TURNING_SPEED`, `PARKING_SPEED`
- **Failsafe timeouts**: `COOLDOWN`, `COMMAND_TIMEOUT_MS`
- **PID parameters**: `AFTER_BLOCK_PID_SCALE`, `AFTER_BLOCK_PID_MS`

## Safety Notes

⚠️ **Always test in a safe environment first!**

- Start with low speeds for initial testing
- Have an emergency stop method ready
- Test individual components before full integration
- Monitor serial output for error messages
- Keep the robot within a contained area during testing

## License

This code is provided for educational purposes for WRO Future Engineers competition.
