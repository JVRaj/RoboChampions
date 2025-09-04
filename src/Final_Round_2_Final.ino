#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// UART TF Luna Lidar for Left and Right sensors
HardwareSerial tfLunaL(1);
HardwareSerial tfLunaR(2);

// I2C for front lidar sensor
TwoWire myWire = TwoWire(0);
Adafruit_BNO055 bno = Adafruit_BNO055(55);

volatile int distanceLeft = 100000;
volatile int distanceRight = 100000;
volatile int distanceFront = 100000;

#define IN1 27
#define IN2 14
#define ENA 32

const int servoPin = 12;
const int servoCenter = 92;

const int button_pin = 25;

unsigned long lastServoWrite = 0;
int lastAngle = 0;

// I2C address for front LiDAR sensor
#define TFLUNA_I2C_ADDR 0x10


void moveServoAngle(int angle) {
  angle = constrain(angle, 20, 160);
  int pulsewidth = map(angle, 0, 180, 500, 2400);
  digitalWrite(servoPin, HIGH);
  delayMicroseconds(pulsewidth);
  digitalWrite(servoPin, LOW);
  delayMicroseconds(20000 - pulsewidth);
}

void steer(int direction) {
  int angle = constrain(servoCenter + direction, 20, 160);
  moveServoAngle(angle);
}

int readTFLunaUART(HardwareSerial &serialPort, volatile int &prevVal) {
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

int readFrontTF(volatile int &prevFront) {
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

// Handshake function - sends "OK" repeatedly and waits for "OK" response
void waitForOK() {
  bool okReceived = false;
  String input = "";
  unsigned long startTime = millis();

  while (!okReceived && (millis() - startTime < 5000)) { // 5 second timeout
    Serial.println("OK");  // Send "OK" every 500 ms
    delay(500);

    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\n') {
        input.trim();
        if (input.equalsIgnoreCase("OK")) {
          okReceived = true;
          break;
        }
        input = "";
      } else {
        input += c;
      }
    }
  }
  if (!okReceived) {
    Serial.println("Handshake Timeout");
  } else {
    Serial.println("Handshake Complete");
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(servoPin, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(button_pin, INPUT_PULLUP);

  // Initialize UART ports for Left and Right TF Luna LiDAR sensors
  tfLunaL.begin(115200, SERIAL_8N1, 18, 19);
  tfLunaR.begin(115200, SERIAL_8N1, 17, 16);

  if (!bno.begin()) {
    Serial.print("NO BNO");
    while (1);
  }

  delay(1000);
  bno.setExtCrystalUse(true);

  steer(0);
  analogWrite(ENA, 0);

  // Wait for button press to continue
  bool flag = true;
  while (flag) {
    if (digitalRead(button_pin) == LOW) {
      Serial.println("DEBUG: BUTTON CLICK");
      flag = false;
      break;
    }
  }

  waitForOK();  // Perform handshake before continuing

}

void loop() {
  distanceLeft = readTFLunaUART(tfLunaL, distanceLeft);
  distanceRight = readTFLunaUART(tfLunaR, distanceRight);
  distanceFront = readFrontTF(distanceFront);

  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  Serial.print((int)orientationData.orientation.x);
  Serial.print(",");
  Serial.print(distanceFront);
  Serial.print(",");
  Serial.print(distanceLeft);
  Serial.print(",");
  Serial.println(distanceRight);

  static String input = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      input.trim();
      if (input.length() > 0) {
        int parts[5] = {0};
        int idx = 0;
        char token = strtok((char)input.c_str(), ",");
        while (token != NULL && idx < 5) {
          parts[idx++] = atoi(token);
          token = strtok(NULL, ",");
        }
        int speed = parts[0];
        bool direction = parts[1];
        int steerRaw = parts[2];

        digitalWrite(IN1, direction == 0 ? HIGH : LOW);
        digitalWrite(IN2, direction == 0 ? LOW : HIGH);
        analogWrite(ENA, speed);
        steer(steerRaw);
        lastAngle = steerRaw;
      }
      input = "";
    } else {
      input += c;
    }
  }

  if (millis() - lastServoWrite > 20) {
    steer(lastAngle);
    lastServoWrite = millis();
  }

  delay(50);
}