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
const int THRESHOLD = 40;
bool hasTurned = false;
int lastTurn = 0;
unsigned long forceStraightUntil = 0;

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
  headingOffset = rawHeading();
}

// ==== Main Loop ====
void loop() {
  int left = readUltrasonic(trigLeft, echoLeft);
  int center = readUltrasonic(trigCenter, echoCenter);
  int right = readUltrasonic(trigRight, echoRight);
  float heading = getHeading();

  Serial.print("Left: "); Serial.print(left);
  Serial.print("  Center: "); Serial.print(center);
  Serial.print("  Right: "); Serial.print(right);
  Serial.print("  | IMU: "); Serial.println(heading);

  // === Force straight movement after a turn ===
  if (millis() < forceStraightUntil) {
    goStraight();
    return;
  }

  // === Main Wall Following Logic ===
  if (left < THRESHOLD && right < THRESHOLD && center > THRESHOLD) {
    // Forward path is clear
    goStraight();
    hasTurned = false;
    lastTurn = 0;
  }
  else if (!hasTurned && left > THRESHOLD && right > THRESHOLD && center < THRESHOLD) {
    // Both sides open, prefer left once
    if(left>right){
       turnByIMU(-90);
      headingOffset = rawHeading();
      hasTurned = true;
      lastTurn = -90;
    }
    else{
        turnByIMU(90);
        headingOffset = rawHeading();
        hasTurned = true;
        lastTurn = 90;
    }
   
  }
  else if (!hasTurned && left > THRESHOLD && right < THRESHOLD && center < THRESHOLD) {
    // Left turn
    turnByIMU(-90);
    headingOffset = rawHeading();
    hasTurned = true;
    lastTurn = -90;
  }
  else if (!hasTurned && right > THRESHOLD && left < THRESHOLD && center < THRESHOLD) {
    // Right turn
    turnByIMU(90);
    headingOffset = rawHeading();
    hasTurned = true;
    lastTurn = 90;
  }
  else if (left < THRESHOLD && right < THRESHOLD && center < THRESHOLD) {
    // Dead end
    goBackward();
    hasTurned = false;
    lastTurn = 0;
  }
  else if (hasTurned && center < THRESHOLD) {
    // Retry last turn if still blocked
    turnByIMU(lastTurn);
    headingOffset = rawHeading();
  }
  else {
    goStraight();
  }

  delay(150);
}

// ==== Movement Functions ====
void goStraight() {
  float heading = getHeading();
  float correction = -heading;
  correction = constrain(correction, -90, 90);
  int servoAngle = map(correction, -90, 90, MAX_LEFT, MAX_RIGHT);
  servoAngle = constrain(servoAngle, MAX_RIGHT, MAX_LEFT);
  steeringServo.write(servoAngle);

  digitalWrite(MOTOR_FWD, HIGH);
  digitalWrite(MOTOR_BWD, LOW);
  analogWrite(MOTOR_EN, 180);
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

  bool rightTurn = angleDelta > 0;
  steeringServo.write(rightTurn ? MAX_RIGHT : MAX_LEFT);

  digitalWrite(MOTOR_FWD, HIGH);
  digitalWrite(MOTOR_BWD, LOW);
  analogWrite(MOTOR_EN, 180);

  while (true) {
    float current = getHeading();
    float error = target - current;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;

    if (abs(error) < 5) break;
    delay(50);
  }

  stopMotors();
  delay(300);
  forceStraightUntil = millis() + 1000;  // 1 second of straight movement after turn
}

// ==== Sensor Functions ====
int readUltrasonic(int trig, int echo) {
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH, 20000);
  return duration * 0.034 / 2;
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

void stopMotors() {
  digitalWrite(MOTOR_FWD, LOW);
  digitalWrite(MOTOR_BWD, LOW);
  analogWrite(MOTOR_EN, 0);
  steeringServo.write((MAX_LEFT + MAX_RIGHT) / 2);
}
