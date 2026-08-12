ESP32 MAZE NAVIGATION ROBOT - README
=====================================

This project implements an autonomous maze-navigating robot using an ESP32
microcontroller. It combines:
- BNO055 absolute orientation sensor for heading feedback,
- TF-Luna I2C lidar distance sensors for obstacle detection,
- a PCA9548A I2C multiplexer to manage multiple I2C devices,
- a steering servo and a DC motor for locomotion.

The robot follows a straight line using a PID controller, avoids front
obstacles by turning left or right, locks the turn direction after the
first decision, and counts turns up to a maximum of 12 before stopping.


HARDWARE REQUIREMENTS
---------------------
| Component                | Qty | Notes                         |
|--------------------------|-----|-------------------------------|
| ESP32 Development Board  | 1   | SDA=21, SCL=22 by default     |
| Adafruit BNO055 IMU      | 1   | I2C address 0x28 (default)    |
| TF-Luna (I2C version)    | 3   | I2C address 0x10, for left,   |
|                          |     | right, front                  |
| PCA9548A I2C Multiplexer | 1   | Address 0x70                  |
| Servo motor              | 1   | Mounted upside-down (see code)|
| DC motor driver (e.g.    | 1   | Enable, Forward, Reverse pins |
|   L298N)                 |     |                               |
| DC motor                 | 1   | For propulsion                |
| Push button              | 1   | Optional start button         |
| Power supply             |     | Suitable for all components   |


PIN CONNECTIONS (ESP32)
-----------------------
Pin 21 (SDA)  → PCA9548A SDA (and all devices behind mux)
Pin 22 (SCL)  → PCA9548A SCL (and all devices behind mux)
Pin 13        → Servo signal wire
Pin 25        → Motor forward (IN1)
Pin 26        → Motor backward (IN2)
Pin 33        → Motor enable (ENA)
Pin 32        → Start button (other side to GND)


MUX CHANNEL ASSIGNMENTS
-----------------------
PCA9548A Channel 0  → Front TF-Luna
PCA9548A Channel 2  → Right TF-Luna
PCA9548A Channel 3  → Left TF-Luna
PCA9548A Channel 4  → BNO055 IMU


SOFTWARE DEPENDENCIES
---------------------
Install via Arduino Library Manager:
- Adafruit Unified Sensor by Adafruit
- Adafruit BNO055 by Adafruit
- ESP32Servo by Kevin Harrington / John K. Bennett

The Wire library is built-in for ESP32.


INSTALLATION
------------
1. Clone or download this repository.
2. Open the .ino file in the Arduino IDE.
3. Select your ESP32 board (e.g., "ESP32 Dev Module") and the correct COM port.
4. Verify the pin assignments match your wiring.
5. Upload the code.
6. Open the Serial Monitor at 115200 baud to observe debug output.


CONFIGURATION
-------------
All adjustable parameters are at the top of the sketch:

// PID gains
float Kp = 0.8;
float Ki = 0.0;
float Kd = 0.2;

// Motor speeds (PWM 0-255)
int straightSpeed = 255;
int turnSpeed    = 255;

// Obstacle thresholds (cm)
const int FRONT_THRESHOLD = 120;
const int SIDE_THRESHOLD  = 120;

// Maximum number of turns before stopping
const int MAX_TURNS = 12;

Servo limits:
The code assumes an upside-down steering servo. Straight is 90°,
full left is 160°, full right is 20°. These values are mirrored internally.

Start button:
The button wait is commented out by default. To use it, uncomment the
while (digitalRead(BUTTON_PIN) == HIGH) block in setup(). The button is
connected between pin 32 and GND, using the internal pull-up.


HOW IT WORKS
------------

1. Initialisation
   - I2C bus starts on pins 21/22.
   - PCA9548A channel 4 is selected and BNO055 is initialised.
   - Servo attached and set to straight.
   - Motor pins set low; motor enable set to straightSpeed.
   - Waits for button press (if enabled) or delay before starting.

2. Main Loop (robot behaviour)
   A. Reading sensors
      - During a turn or within a 1-second suppression window after
        turning, distance readings are forced to 500 cm (ignored).
      - Otherwise, each TF-Luna is read via the multiplexer.
      - Heading is read from BNO055 on channel 4 and normalised.
   B. After max turns
      - Once turnCount reaches MAX_TURNS, the robot stops completely
        and prints "Complete" repeatedly.
   C. Turning state
      - Servo set to extreme angle (mirrored), motor runs at turnSpeed.
      - Turns until heading changes by ±70° from start of turn.
      - Then target heading updated, turn count incremented, sensor
        suppression starts.
   D. Obstacle detection
      - If front distance < FRONT_THRESHOLD, robot prepares to turn.
      - On first obstacle, code locks turn direction (leftMode)
        based on side distances.
   E. Straight-line driving (PID)
      - Relative heading fed into PID controller.
      - PID output adjusts steering servo to keep heading aligned.
      - Motor runs at straightSpeed.

3. Debug Output
   Every loop prints: TurnCount | Left | Right | Front | Heading | Target | Servo


TROUBLESHOOTING
---------------
- BNO055 not detected: Check wiring and mux channel 4.
- TF-Luna read errors: Verify I2C address (0x10) and mux channels.
- Servo behaves backwards: Code already includes mirror calculation.
  If still reversed, swap MAX_LEFT/MAX_RIGHT or remove mirror line.
- Motor doesn't run: Check enable pin PWM, driver power, speed values.
- Robot turns too sharply/not enough: Adjust ±70° threshold or turnSpeed.


LICENSE
-------
This code is provided as-is for educational and hobbyist use.
Modify and distribute freely.


AUTHOR
------
Created for an autonomous maze navigation project.
For questions, refer to the code comments or open an issue.