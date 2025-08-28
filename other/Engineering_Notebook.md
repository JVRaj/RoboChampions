# Engineering Notebook
*One team, one robot, limitless possibilities...*

---

## Introduction

### Skill Building

**3D Printing**
We utilised an Ender 3D printer. This allowed us to rapidly manufacture and test our products.

**Python**
We used Python in our programming, offering ease-of-use and proficiency.

**CAD**
We used Solidworks, an industry grade CAD software to design our robot. This allowed us to use tools in the software that helped us refine and improve our robot's systems to fit the challenge.

---

## List of Parts

| Serial No. | Part Name | Quantity | Purpose |
| :--- | :--- | :--- | :--- |
| 1 | ESP32 | 1 | Motor and sensor control |
| 2 | BNO055 IMU | 1 | Orientation sensing |
| 3 | TF Luna LIDAR | 1 | Distance measurement |
| 4 | TF-Luna | 3 | Short range obstacle detection |
| 5 | N20 DC Motor 12V 400RPM | 1 | Bot movement |
| 6 | TB6612FNG Motor Driver | 1 | Motor driver for N20 motors |
| 7 | MG90 Servo Motor | 1 | Steering mechanism / sensor actuation |
| 8 | Raspberry Pi 5 | 1 | High-level decision making and image processing |
| 9 | USB Webcam | 1 | Signal color detection |
| 10 | Powerbank | 1 | Power supply for Raspberry Pi |
| 11 | Lithium-ion Battery 3S | 1 | Main power source for motors and ESP32 |
| 12 | NeoPixel LED | 1 | Status indicator |

---

## Our Journey

### 3. System Architecture
The robot has two main computing units, each with specific responsibilities:

**1. Raspberry Pi 5**
- Handles camera input and real-time image processing using OpenCV.
- Detects signals, classifies their colors, and decides movement directions.
- Sends commands to the ESP32 for execution.

**2. ESP32**
- Acts as the low-level controller and executes commands from the Raspberry Pi.
- Manages sensors: LIDAR, ultrasonic, and IMU for obstacle detection and orientation.
- Controls DC motors and servo motors via the TB6612FNG driver for movement and turning.

Together, the Raspberry Pi and ESP32 enable the robot to combine vision-based decision making with precise motion control for smooth navigation and obstacle avoidance.

### 4. Circuit and Wiring
The power system is divided into two parts:
- A powerbank supplies the Raspberry Pi 5.
- A 3S Li-ion battery powers the ESP32, motor driver, and motors.

**Key Connections:**
- ESP32 is connected to the TB6612FNG for motor control.
- N20 motors are connected to the motor driver outputs.
- TF Luna LIDAR and ultrasonic sensors are connected to the ESP32 for distance measurement.
- BNO055 is connected to the ESP32 via I2C for orientation sensing.
- The servo motor is controlled by an ESP32 PWM pin.
- The Raspberry Pi communicates with the ESP32 via UART.

---

### 5. Software and Control Logic
The control software is divided into two levels:

**1. Raspberry Pi Software:**
- Captures video using a USB webcam.
- Performs signal detection and color classification using OpenCV.
- Determines the direction of turn based on detected signal color.
- Sends control commands to the ESP32 over UART.

**2. ESP32 Firmware:**
- Receives commands from the Raspberry Pi.
- Reads distance data from LIDAR and ultrasonic sensors.
- Uses the IMU for orientation correction.
- Drives N20 motors via the TB6612FNG.
- Controls the servo motor.
- Executes obstacle avoidance and path correction.

### 6. Working Process

**1. Time Trial - Obstacle Avoidance:**
- The ESP32 acts as the main controller (brain) of the robot.
- Ultrasonic and LIDAR sensors measure distances to detect obstacles.
- The IMU provides orientation feedback to maintain a correct heading.
- Servo motors handle turning, while DC motors control forward movement.
- The robot navigates the track, avoiding obstacles automatically.

**2. Signal Detection & Navigation**
- The camera detects signals and classifies their color.
- The Raspberry Pi decides the movement direction based on signal color:
    - Green Signal → Turn Left
    - Red Signal → Turn Right
- Commands are sent to the ESP32 for execution.
- The ESP32 verifies obstacle clearance using sensors.
- Motors are actuated to execute the turn or continue forward.

---

### 7. Testing and Results
The robot was tested in a simulated WRO environment with colored signals and obstacles.

**Key Results:**
- Signal detection accuracy: ~90% under normal lighting.
- Obstacle detection range: Up to 4 meters (LIDAR), 30cm (ultrasonic).
- Orientation correction is effective for small drifts.
- Smooth coordination between the Raspberry Pi and ESP32 was achieved.

### 8. Future Improvements
- Upgrade to higher torque motors for faster movement.
- Implement deep learning-based object detection.
- Integrate SLAM for autonomous navigation.
- Improve energy efficiency with optimized power management.
