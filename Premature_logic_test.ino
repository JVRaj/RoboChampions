#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <ESP32Servo.h>

// ==== Ultrasonic Pins ====
const int trigLeft = 25, echoLeft = 26;
const int trigCenter = 27, echoCenter = 32;
const int trigRight = 33, echoRight = 19;

// ==== Servo and IMU ====
Servo steeringServo;
const int SERVO_PIN = 21;  // MUST be PWM-capable
const int MAX_LEFT = 150;
const int MAX_RIGHT = 30;

const int LED = 14;

Adafruit_BNO055 bno = Adafruit_BNO055(55);
float headingOffset = 0;

String state = "";

// ==== Constants ====
const int THRESHOLD = 4;
bool hasTurned = false;
int lastTurn = 0;
unsigned long forceStraightUntil = 0;

// ==== Setup ====
void setup() {
  Serial.begin(9600);

  pinMode(trigLeft, OUTPUT);    pinMode(echoLeft, INPUT);
  pinMode(trigCenter, OUTPUT);  pinMode(echoCenter, INPUT);
  pinMode(trigRight, OUTPUT);   pinMode(echoRight, INPUT);

  pinMode(LED, OUTPUT);

  steeringServo.attach(SERVO_PIN);

  if (!bno.begin()) {
    Serial.println("IMU not detected!");
    while (1);
  }
  bno.setExtCrystalUse(true);
  delay(1000);


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
  Serial.print("State: "); Serial.println(state);

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
  // float heading = getHeading();
  // float correction = -heading;
  // correction = constrain(correction, -90, 90);
  // int servoAngle = map(correction, -90, 90, MAX_LEFT, MAX_RIGHT);
  // servoAngle = constrain(servoAngle, MAX_RIGHT, MAX_LEFT);
  // steeringServo.write(servoAngle);
  state = "FORWARD";
  digitalWrite(LED, HIGH);
}

void goBackward() {
  steeringServo.write((MAX_LEFT + MAX_RIGHT) / 2);
  state = "BACKWARD";
  digitalWrite(LED, LOW);
}

void turnByIMU(int angleDelta) {
  float startHeading = getHeading();
  float target = startHeading + angleDelta;

  if (target > 180) target -= 360;
  if (target < -180) target += 360;

  bool rightTurn = angleDelta > 0;
  steeringServo.write(rightTurn ? MAX_RIGHT : MAX_LEFT);

  while (true) {
    float current = getHeading();
    float error = target - current;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;

    if (abs(error) < 5) break;
    delay(50);
  }
  state = "TURNING";
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