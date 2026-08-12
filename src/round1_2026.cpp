#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <ESP32Servo.h>

// ======================== I2C MULTIPLEXER ========================
#define PCA9548A_ADDR      0x70
#define MUX_CHANNEL_LEFT   3
#define MUX_CHANNEL_RIGHT  2
#define MUX_CHANNEL_FRONT  0
#define MUX_CHANNEL_BNO    4

// ======================== TF-LUNA I2C ============================
#define TFLUNA_I2C_ADDR    0x10

// ======================== PIN DEFINITIONS ========================
#define SERVO_PIN          13
#define MOTOR_FWD          25
#define MOTOR_BWD          26
#define MOTOR_EN           33
#define BUTTON_PIN         32

// ======================== SERVO CONSTANTS ========================
const int STRAIGHT_ANGLE = 90;
const int MAX_LEFT       = 160;   // physical left extreme
const int MAX_RIGHT      = 20;    // physical right extreme

// ======================== SENSOR THRESHOLDS ======================
const int FRONT_THRESHOLD = 120;   // cm
const int SIDE_THRESHOLD  = 120;   // cm

// ======================== PID GAINS ==============================
float Kp = 0.8;
float Ki = 0.0;
float Kd = 0.2;

// ======================== MOTOR SPEEDS (PWM 0-255) ===============
int straightSpeed = 255;   // speed while going straight
int turnSpeed    = 255;    // speed while turning

// ======================== GLOBAL VARIABLES =======================
Adafruit_BNO055 bno = Adafruit_BNO055(55);
Servo steeringServo;

int leftDist, rightDist, frontDist;
float headingX;

float targetHeading = 0;
float startHeading = 0;
float previousError = 0;
float integral = 0;

bool turning = false;
bool turnLeft = false;
bool turnModeLocked = false;
bool leftMode = false;

int turnCount = 0;
const int MAX_TURNS = 12;
bool finishedTurns = false;
const int stopDistance = 150;

unsigned long suppressSensorsUntil = 0;
const int SUPPRESSION_TIME = 1000;   // ms

// ======================== MUX HELPER =============================
void selectMuxChannel(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(PCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
  delay(1);   // brief settling time
}

// ======================== READ TF-LUNA (I2C) =====================
int readTFLunaI2C(uint8_t muxChannel) {
  selectMuxChannel(muxChannel);
  Wire.beginTransmission(TFLUNA_I2C_ADDR);
  Wire.write(0x00);
  if (Wire.endTransmission(false) != 0) {
    return -1;   // communication error
  }
  Wire.requestFrom(TFLUNA_I2C_ADDR, (uint8_t)2);
  if (Wire.available() == 2) {
    int dist = Wire.read() | (Wire.read() << 8);
    return dist;
  }
  return -1;
}

// ======================== READ HEADING (BNO055) ==================
float readHeadingX() {
  // The BNO is on mux channel 4 – select it before reading
  selectMuxChannel(MUX_CHANNEL_BNO);
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  float angle = orientationData.orientation.x;
  angle = fmod(angle, 360.0f);
  if (angle > 180.0f) angle -= 360.0f;
  if (angle <= -180.0f) angle += 360.0f;
  return angle;
}

// ======================== NORMALISE ANGLE ========================
float normalizeAngle(float angle) {
  angle = fmod(angle, 360.0f);
  if (angle > 180.0f) angle -= 360.0f;
  if (angle <= -180.0f) angle += 360.0f;
  return angle;
}

// ======================== PID WITH SERVO MIRROR ==================
int calculatePID(float heading) {
  float error = heading;
  integral += error;
  float derivative = error - previousError;
  float adjustment = Kp * error + Ki * integral + Kd * derivative;
  previousError = error;

  int servoPos = STRAIGHT_ANGLE + (int)adjustment;
  servoPos = constrain(servoPos, MAX_RIGHT, MAX_LEFT);

  // Mirror the angle to compensate for an upside‑down servo
  servoPos = 2 * STRAIGHT_ANGLE - servoPos;
  return servoPos;
}

// ======================== TARGET HEADING UPDATES =================
void updateTargetHeadingLeft() {
  if (targetHeading == 0)      targetHeading = 270;
  else if (targetHeading == 270) targetHeading = 180;
  else if (targetHeading == 180) targetHeading = 90;
  else                         targetHeading = 0;
}

void updateTargetHeadingRight() {
  if (targetHeading == 0)      targetHeading = 90;
  else if (targetHeading == 90)  targetHeading = 180;
  else if (targetHeading == 180) targetHeading = 270;
  else                         targetHeading = 0;
}

// ======================== MOTOR CONTROL ==========================
void stopMotors() {
  digitalWrite(MOTOR_FWD, HIGH);
  digitalWrite(MOTOR_BWD, HIGH);
  digitalWrite(MOTOR_EN, LOW);
}

// ======================== DEBUG PRINT ============================
void printAllData(int left, int right, int front, float heading, int servoAngle) {
  Serial.print("TurnCount: "); Serial.print(turnCount);
  Serial.print(" | Left: "); Serial.print(left);
  Serial.print(" | Right: "); Serial.print(right);
  Serial.print(" | Front: "); Serial.print(front);
  Serial.print(" | Heading: "); Serial.print(heading);
  Serial.print(" | Target: "); Serial.print(targetHeading);
  Serial.print(" | Servo: "); Serial.println(servoAngle);
}

// ======================== SETUP ==================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  // ---- I2C init ----
  Wire.begin(21, 22);   // SDA=21, SCL=22 (default ESP32 pins)

  // ---- BNO055 on mux channel 4 ----
  selectMuxChannel(MUX_CHANNEL_BNO);
  if (!bno.begin()) {
    Serial.println("BNO055 not detected!");
    while (1);
  }
  bno.setExtCrystalUse(true);
  delay(1000);   // let sensor stabilise

  // ---- Servo ----
  steeringServo.attach(SERVO_PIN);
  steeringServo.write(STRAIGHT_ANGLE);

  // ---- Motor pins ----
  pinMode(MOTOR_FWD, OUTPUT);
  pinMode(MOTOR_BWD, OUTPUT);
  pinMode(MOTOR_EN, OUTPUT);
  digitalWrite(MOTOR_FWD, LOW);
  digitalWrite(MOTOR_BWD, LOW);
  analogWrite(MOTOR_EN, straightSpeed);   // use straightSpeed

  // ---- Start button (internal pull‑up) ----
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Serial.println("Setup complete. Press button to start...");

  // Wait for button press (LOW when pressed)
  // while (digitalRead(BUTTON_PIN) == HIGH) {
  //   delay(50);
  // }
  delay(500);
  Serial.println("Started!");
}

// ======================== LOOP ===================================
void loop() {
  // -------- 1. Sensor reading (with suppression) --------
  if (turning || millis() < suppressSensorsUntil) {
    leftDist = 500;
    rightDist = 500;
    frontDist = 500;
  } else {
    // Read each sensor from its mux channel
    int rawLeft  = readTFLunaI2C(MUX_CHANNEL_LEFT);
    int rawRight = readTFLunaI2C(MUX_CHANNEL_RIGHT);
    int rawFront = readTFLunaI2C(MUX_CHANNEL_FRONT);

    // Keep last valid reading in case of I2C errors
    static int lastLeft = 500, lastRight = 500, lastFront = 500;
    leftDist  = (rawLeft  > 0) ? rawLeft  : lastLeft;
    rightDist = (rawRight > 0) ? rawRight : lastRight;
    frontDist = (rawFront > 0) ? rawFront : lastFront;
    lastLeft  = leftDist;
    lastRight = rightDist;
    lastFront = frontDist;
  }

  // Read heading (the function selects the BNO mux channel internally)
  headingX = readHeadingX();

  // -------- 2. Finished turns? --------
  if (finishedTurns) {
    // if (frontDist > stopDistance) {
    //   steeringServo.write(STRAIGHT_ANGLE);
    //   digitalWrite(MOTOR_FWD, HIGH);
    //   digitalWrite(MOTOR_BWD, LOW);
    // } else {
      // stopMotors();
      digitalWrite(MOTOR_FWD, LOW);
      digitalWrite(MOTOR_BWD, LOW);
      digitalWrite(MOTOR_EN, LOW);
      Serial.println("Complete");
      while (1){Serial.println("completedddd");}
    // }
    return;
  }

  // -------- 3. Turning mode --------
  if (turning) {
    float diff = normalizeAngle(headingX - startHeading);

    if (turnLeft) {
      // Intended left turn – but servo is inverted, so write MAX_RIGHT
      steeringServo.write(MAX_RIGHT);
      analogWrite(MOTOR_EN, turnSpeed);   // use turnSpeed
      digitalWrite(MOTOR_FWD, HIGH);
      digitalWrite(MOTOR_BWD, LOW);
      if (fabs(diff) >= 70) {
        turning = false;
        updateTargetHeadingLeft();
        turnCount++;
        if (turnCount >= MAX_TURNS) finishedTurns = true;
        suppressSensorsUntil = millis() + SUPPRESSION_TIME;
        Serial.println("Left Mode");
      }
    } else {
      // Intended right turn – write MAX_LEFT due to inversion
      steeringServo.write(MAX_LEFT);
      analogWrite(MOTOR_EN, turnSpeed);   // use turnSpeed
      digitalWrite(MOTOR_FWD, HIGH);
      digitalWrite(MOTOR_BWD, LOW);
      if (fabs(diff) >= 70) {
        turning = false;
        updateTargetHeadingRight();
        turnCount++;
        if (turnCount >= MAX_TURNS) finishedTurns = true;
        suppressSensorsUntil = millis() + SUPPRESSION_TIME;
        Serial.println("Right Mode");
      }
    }
  }
  // -------- 4. Obstacle detection (start turn) --------
  else if (frontDist < FRONT_THRESHOLD) {
    startHeading = headingX;

    // Lock turn direction on first obstacle
    if (!turnModeLocked) {
      if (leftDist > SIDE_THRESHOLD) {
        leftMode = true;
        turnModeLocked = true;
      } else if (rightDist > SIDE_THRESHOLD) {
        leftMode = false;
        turnModeLocked = true;
      }
    }

    if (leftMode) {
      if (leftDist > SIDE_THRESHOLD) {
        turning = true;
        turnLeft = true;
      }
    } else {
      if (rightDist > SIDE_THRESHOLD) {
        turning = true;
        turnLeft = false;
      }
    }
  }
  // -------- 5. Straight line with PID --------
  else {
    float relativeHeading = normalizeAngle(headingX - targetHeading);
    int servoAngle = calculatePID(relativeHeading);
    steeringServo.write(servoAngle);
    // Set straight speed and direction
    analogWrite(MOTOR_EN, straightSpeed);
    digitalWrite(MOTOR_FWD, HIGH);
    digitalWrite(MOTOR_BWD, LOW);
  }

  // -------- 6. Debug output --------
  printAllData(leftDist, rightDist, frontDist, headingX, steeringServo.read());
}