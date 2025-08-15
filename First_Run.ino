#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Servo.h>

// ==== Ultrasonic Pins ====
const int trigLeft = A1, echoLeft = A0;
const int trigCenter = 8, echoCenter = 7;
const int trigRight = A3, echoRight = A2;

// ==== Motor Pins ====
const int MOTOR_EN = 3;
const int MOTOR_FWD = 4;
const int MOTOR_BWD = 5;
const int MOTOR_STBY = 6;

// ==== Servo and IMU ====
Servo steeringServo;
const int SERVO_PIN = 2;
const int MAX_LEFT = 150;
const int MAX_RIGHT = 30;

Adafruit_BNO055 bno = Adafruit_BNO055(55);
float headingOffset = 0;

// ==== Start Button ====
const int START_BUTTON = 7;

// ==== Constants ====
const int THRESHOLD = 45; 
bool hasTurned = false;
int lastTurn = 0;
unsigned long forceStraightUntil = 0;
int turn = 0;

// ==== Previous Distance Storage ====
int prevLeft = 0, prevCenter = 0, prevRight = 0;

// ==== PID constants for straight driving ====
float Kp = 3.0;
float Ki = 0.0;
float Kd = 1.5;

float prevError = 0;
float integral = 0;

// ==== Mode tracking ====
enum Mode { NONE, LEFT_MODE, RIGHT_MODE };
Mode mode = NONE;

// ==== Setup ====
void setup() {
  Serial.begin(9600);

  pinMode(trigLeft, OUTPUT);    pinMode(echoLeft, INPUT);
  pinMode(trigCenter, OUTPUT);  pinMode(echoCenter, INPUT);
  pinMode(trigRight, OUTPUT);   pinMode(echoRight, INPUT);

  pinMode(MOTOR_EN, OUTPUT);
  pinMode(MOTOR_FWD, OUTPUT);
  pinMode(MOTOR_BWD, OUTPUT);
  pinMode(MOTOR_STBY, OUTPUT);
  digitalWrite(MOTOR_STBY, HIGH);

  pinMode(START_BUTTON, INPUT_PULLUP);
  steeringServo.attach(SERVO_PIN);

  if (!bno.begin()) {
    Serial.println("IMU not detected!");
    while (1);
  }
  bno.setExtCrystalUse(true);
  delay(1000);

  Serial.println("Press button to start...");
  while (digitalRead(START_BUTTON) == HIGH);
  delay(500);
  resetHeadingOffset();
}

// ==== Main Loop ====
void loop() {
  goStraight();

  int center = readUltrasonic(trigCenter, echoCenter, prevCenter);
  if (center > 0) prevCenter = center;

  // If no mode chosen yet, scan all three when center < threshold
  if (mode == NONE && center < THRESHOLD) {
    int left = readUltrasonic(trigLeft, echoLeft, prevLeft);
    int right = readUltrasonic(trigRight, echoRight, prevRight);

    if (left > 0) prevLeft = left;
    if (right > 0) prevRight = right;

    if (left >= right) {
      mode = LEFT_MODE;
      Serial.println("Mode set to LEFT");
    } else {
      mode = RIGHT_MODE;
      Serial.println("Mode set to RIGHT");
    }
  }

  // If mode is set, only scan center + chosen side
  int sideDist = 0;
  if (mode == LEFT_MODE) {
    sideDist = readUltrasonic(trigLeft, echoLeft, prevLeft);
    if (sideDist > 0) prevLeft = sideDist;
  } else if (mode == RIGHT_MODE) {
    sideDist = readUltrasonic(trigRight, echoRight, prevRight);
    if (sideDist > 0) prevRight = sideDist;
  }

  Serial.print("Center: "); Serial.print(prevCenter);
  Serial.print(" | Side: "); Serial.println(sideDist);

  // === Turn decision ===
  if (turn >= 12) {
    digitalWrite(MOTOR_FWD, LOW);
    Serial.println("Turn limit reached. Stopping.");
    while (1);
  }

  if (mode == LEFT_MODE && center < THRESHOLD && sideDist > THRESHOLD) {
    turn++;
    turnByIMU(-90);
    
  }
  else if (mode == RIGHT_MODE && center < THRESHOLD && sideDist > THRESHOLD) {
    turn++;
    turnByIMU(90);
    
  }
}

// ==== Movement Functions ====
void goStraight() {
  float heading = getHeading();
  float error = -heading;

  integral += error;
  float derivative = error - prevError;
  float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
  prevError = error;

  if (abs(error) < 1.0) output = 0;

  int servoAngle = map(output, -90, 90, MAX_LEFT, MAX_RIGHT);
  servoAngle = constrain(servoAngle, MAX_RIGHT, MAX_LEFT);

  steeringServo.write(servoAngle);

  digitalWrite(MOTOR_FWD, HIGH);
  digitalWrite(MOTOR_BWD, LOW);
  analogWrite(MOTOR_EN, 100);
}

void goBackward() {
  steeringServo.write((MAX_LEFT + MAX_RIGHT) / 2);
  digitalWrite(MOTOR_FWD, LOW);
  digitalWrite(MOTOR_BWD, HIGH);
  analogWrite(MOTOR_EN, 180);
}

void turnByIMU(int angleDelta) {
  float startHeading = getHeading();
  float target = startHeading + angleDelta;

  if (target > 180) target -= 360;
  if (target < -180) target += 360;

  unsigned long startTime = millis();

  // --- Coarse turn ---
  while (true) {
    float current = getHeading();
    float error = target - current;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;

    int servoAngle = (error > 0) ? MAX_RIGHT : MAX_LEFT;
    steeringServo.write(servoAngle);

    digitalWrite(MOTOR_FWD, HIGH);
    digitalWrite(MOTOR_BWD, LOW);
    analogWrite(MOTOR_EN, 100);

    if (abs(error) <= 5) break;
    if (millis() - startTime > 5000) break;
    delay(20);
  }

  stopMotors();
  delay(200);

  // --- Fine correction ---
  for (int i = 0; i < 30; i++) {
    float current = getHeading();
    float error = target - current;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;

    if (abs(error) < 1.0) break;

    int servoAngle = map(error, -10, 10, MAX_LEFT, MAX_RIGHT);
    servoAngle = constrain(servoAngle, MAX_RIGHT, MAX_LEFT);
    steeringServo.write(servoAngle);

    digitalWrite(MOTOR_FWD, HIGH);
    digitalWrite(MOTOR_BWD, LOW);
    analogWrite(MOTOR_EN, 120);

    delay(20);
  }

  stopMotors();
  delay(300);

  resetHeadingOffset();
  forceStraightUntil = millis() + 1000;
}

// ==== Sensor Functions ====
int readUltrasonic(int trig, int echo, int prev) {
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH, 20000);
  int distance = duration * 0.034 / 2;
  return (distance == 0) ? prev : distance;
}

// ==== IMU Functions ====
float rawHeading() {
  sensors_event_t event;
  bno.getEvent(&event);
  float h = event.orientation.x;
  if (h > 180) h -= 360;
  return h;
}

float getHeading() {
  float h = rawHeading() - headingOffset;
  if (h > 180) h -= 360;
  if (h < -180) h += 360;
  return h;
}

void resetHeadingOffset() {
  headingOffset = rawHeading();
  prevError = 0;
  integral = 0;
}

void stopMotors() {
  digitalWrite(MOTOR_FWD, LOW);
  digitalWrite(MOTOR_BWD, LOW);
  analogWrite(MOTOR_EN, 0);
  steeringServo.write((MAX_LEFT + MAX_RIGHT) / 2);
}
