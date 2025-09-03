#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <ESP32Servo.h>
#include <HardwareSerial.h>

HardwareSerial tfLunaL(1);
HardwareSerial tfLunaR(2);

#define RX1 18
#define TX1 19
#define RX2 17
#define TX2 16
#define TFLUNA_I2C_ADDR 0x10

Adafruit_BNO055 bno = Adafruit_BNO055(55);
Servo steeringServo;

const int SERVO_PIN = 12;
const int STRAIGHT_ANGLE = 90;
const int MAX_LEFT = 160;
const int MAX_RIGHT = 20;
const int mb = 14;
const int mf = 27;
const int ME = 32;  // Motor enable pin
int left, right, front;
float headingX;
float Kp = 4.0;
float Ki = 0.0;
float Kd = 1.0;
float previousError = 0;
float integral = 0;

int prevLeft = 0;
int prevRight = 0;
int prevFront = 0;

const int FRONT_THRESHOLD = 100;   // cm
const int SIDE_THRESHOLD = 120;    // cm

float targetHeading = 0;  // updated after each turn

bool turning = false;
bool turnLeft = false;
bool turnModeLocked = false; // true after first turn
bool leftMode = false;       // true if always turning left

float startHeading = 0;

// --- NEW: Turn counter ---
int turnCount = 0;
const int MAX_TURNS = 12;
bool finishedTurns = false;
const int stopDistance = 150; // Minimum distance 

// --- NEW: Sensor suppression timer ---
unsigned long suppressSensorsUntil = 0;
const int SUPPRESSION_TIME = 1000; // ms

// ---------- Helper Functions ----------
float normalizeAngle(float angle) {
  angle = fmod(angle, 360.0f);  // wrap to 0..360
  if (angle > 180.0f) angle -= 360.0f;
  if (angle <= -180.0f) angle += 360.0f;
  return angle;
}

int readTFLunaUART(HardwareSerial &serialPort, int &prevVal) {
  static uint8_t buf[9];
  if (serialPort.available()) {
    if (serialPort.read() == 0x59 && serialPort.read() == 0x59) {
      for (int i = 0; i < 7; i++) buf[i] = serialPort.read();
      int dist = buf[0] | (buf[1] << 8);
      prevVal = dist;
      return dist;
    }
  }
  return prevVal;
}

int readFrontTF() {
  Wire.beginTransmission(TFLUNA_I2C_ADDR);
  Wire.write(0x00);
  if (Wire.endTransmission(false) != 0) return prevFront;
  Wire.requestFrom(TFLUNA_I2C_ADDR, (uint8_t)2);
  if (Wire.available() == 2) {
    prevFront = Wire.read() | (Wire.read() << 8);
    return prevFront;
  }
  return prevFront;
}

float readHeadingX() {
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  return normalizeAngle(orientationData.orientation.x);
}

int calculatePID(float heading) {
  float error = heading;
  integral += error;
  float derivative = error - previousError;
  float adjustment = Kp * error + Ki * integral + Kd * derivative;
  previousError = error;

  int servoPos = STRAIGHT_ANGLE + (int)adjustment;
  return constrain(servoPos, MAX_RIGHT, MAX_LEFT);
}

void updateTargetHeadingLeft() {
  if (targetHeading == 0) targetHeading = 270;
  else if (targetHeading == 270) targetHeading = 180;
  else if (targetHeading == 180) targetHeading = 90;
  else targetHeading = 0;
}

void updateTargetHeadingRight() {
  if (targetHeading == 0) targetHeading = 90;
  else if (targetHeading == 90) targetHeading = 180;
  else if (targetHeading == 180) targetHeading = 270;
  else targetHeading = 0;
}

void printAllData(int left, int right, int front, float headingX, int servoAngle) {
  Serial.print("TurnCount: "); Serial.print(turnCount);
  Serial.print(" | Left: "); Serial.print(left);
  Serial.print(" | Right: "); Serial.print(right);
  Serial.print(" | Front: "); Serial.print(front);
  Serial.print(" | Heading: "); Serial.print(headingX);
  Serial.print(" | Target: "); Serial.print(targetHeading);
  Serial.print(" | Servo: "); Serial.println(servoAngle);
}

void stopMotors() {
  digitalWrite(mf, HIGH);
  digitalWrite(mb, HIGH);
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(1000);

  tfLunaL.begin(115200, SERIAL_8N1, RX1, TX1);
  tfLunaR.begin(115200, SERIAL_8N1, RX2, TX2);
  Wire.begin(21, 22);

  if (!bno.begin()) {
    Serial.println("BNO055 not detected!");
    while (1);
  }
  bno.setExtCrystalUse(true);
  delay(2000);

  steeringServo.attach(SERVO_PIN);
  steeringServo.write(STRAIGHT_ANGLE);

  // Motor pins
  pinMode(mf, OUTPUT);
  pinMode(mb, OUTPUT);
  pinMode(ME, OUTPUT);    // motor enable
  digitalWrite(mf, LOW);
  digitalWrite(mb, LOW);
  analogWrite(ME, 255); // keep motor enabled
  pinMode(25, INPUT_PULLUP);    // motor enable

  Serial.println("Setup complete");
  while (digitalRead(25) == HIGH) {
    //WAIT TILL BUTTON IS PRESSED
    left = readTFLunaUART(tfLunaL, prevLeft);
    right = readTFLunaUART(tfLunaR, prevRight);
    front = readFrontTF();
    headingX = readHeadingX();
  }
  delay(500);

}

// ---------- Loop ----------
void loop() {

  // --- Sensor suppression logic ---
  if (turning || millis() +1 < suppressSensorsUntil) {
    left = 500;
    right = 500;
    front = 500;
  } else {
    left = readTFLunaUART(tfLunaL, prevLeft);
    right = readTFLunaUART(tfLunaR, prevRight);
    front = readFrontTF();
  }

  headingX = readHeadingX();

  // --- After finishing 12 turns ---
  if (finishedTurns) {
    if (front > stopDistance) {
      steeringServo.write(STRAIGHT_ANGLE);
      digitalWrite(mf, HIGH);
      digitalWrite(mb, LOW);
    } else {
      stopMotors();
      Serial.println("Complete");
      while (1);
    }
    return;
  }

  // Turning mode
  if (turning) {
    float diff = normalizeAngle(headingX - startHeading);

    if (turnLeft) {
      steeringServo.write(MAX_LEFT);
      analogWrite(mf, 180);
      digitalWrite(mb, LOW);
      if (fabs(diff) >= 70) {
        turning = false;
        updateTargetHeadingLeft();
        turnCount++;
        if (turnCount >= MAX_TURNS) finishedTurns = true;

        // Suppress sensors for 1s
        suppressSensorsUntil = millis() + SUPPRESSION_TIME;
      }
    } else {
      steeringServo.write(MAX_RIGHT);
      digitalWrite(mf, 180);
      digitalWrite(mb, LOW);
      if (fabs(diff) >= 70) {
        turning = false;
        updateTargetHeadingRight();
        turnCount++;
        if (turnCount >= MAX_TURNS) finishedTurns = true;

        // Suppress sensors for 1s
        suppressSensorsUntil = millis() + SUPPRESSION_TIME;
      }
    }
  } 
  // Obstacle detection to start turn
  else if (front < FRONT_THRESHOLD) {
    startHeading = headingX;

    // Lock mode on first turn
    if (!turnModeLocked) {
      if (left > SIDE_THRESHOLD) {
        leftMode = true;
        turnModeLocked = true;
      } else if (right > SIDE_THRESHOLD) {
        leftMode = false;
        turnModeLocked = true;
      }
    }

    // Choose side based on mode
    if (leftMode) {
      if (left > SIDE_THRESHOLD) {
        turning = true;
        turnLeft = true;
      }
    } else {
      if (right > SIDE_THRESHOLD) {
        turning = true;
        turnLeft = false;
      }
    }
  } 
  // Straight running with PID correction
  else {
    float relativeHeading = normalizeAngle(headingX - targetHeading);
    int servoAngle = calculatePID(relativeHeading);
    steeringServo.write(servoAngle);
    digitalWrite(mf, HIGH);
    digitalWrite(mb, LOW);
  }

  printAllData(left, right, front, headingX, steeringServo.read());
}