// Library Robot - Arduino Mega firmware
// Motor control, three HC-SR04 sensors, serial protocol, and watchdog stop.

#include <AFMotor.h>
#include <string.h>
#include <Wire.h>

// Motor Shield V1 channels: left M1/M4, right M2/M3.
AF_DCMotor motorLeftFront(1);
AF_DCMotor motorLeftRear(4);
AF_DCMotor motorRightFront(2);
AF_DCMotor motorRightRear(3);

const uint8_t FORWARD_SPEED = 180;
const uint8_t ROTATE_SPEED = 120;
const uint8_t LEFT_SPEED_REDUCTION = 15;
const unsigned long COMMAND_TIMEOUT_MS = 2000;
const unsigned long IMU_TURN_TIMEOUT_MS = 5000;

// Confirmed Arduino Mega ultrasonic pins.
const uint8_t TRIG_LEFT = 25;
const uint8_t ECHO_LEFT = 24;
const uint8_t TRIG_CENTER = 23;
const uint8_t ECHO_CENTER = 22;
const uint8_t TRIG_RIGHT = 27;
const uint8_t ECHO_RIGHT = 26;

// One encoder pulse input per drivetrain side. Mega pins 18 and 19 support
// hardware interrupts. Change these two constants to match the final wiring.
const uint8_t ENCODER_LEFT_PIN = 18;
const uint8_t ENCODER_RIGHT_PIN = 19;
volatile long encoderLeftTicks = 0;
volatile long encoderRightTicks = 0;

// MPU6500 on the Mega I2C bus: SDA pin 20, SCL pin 21.
const uint8_t MPU_ADDR = 0x68;
float gyroZBias = 0.0;
bool imuTurnActive = false;
uint8_t imuTurnState = 0;  // 0=IDLE, 1=ACTIVE, 2=DONE, 3=ERROR
float imuTurnTargetDeg = 0.0;
float imuTurnAngleDeg = 0.0;
unsigned long imuTurnStartedMs = 0;
unsigned long lastGyroUs = 0;

const uint8_t COMMAND_BUFFER_SIZE = 32;
char commandBuffer[COMMAND_BUFFER_SIZE];
uint8_t commandLength = 0;
unsigned long lastCommandMs = 0;
bool motorsActive = false;

void onLeftEncoderPulse() {
  encoderLeftTicks++;
}

void onRightEncoderPulse() {
  encoderRightTicks++;
}

void setLeft(uint8_t direction, uint8_t speedValue) {
  motorLeftFront.setSpeed(speedValue);
  motorLeftRear.setSpeed(speedValue);
  motorLeftFront.run(direction);
  motorLeftRear.run(direction);
}

void setRight(uint8_t direction, uint8_t speedValue) {
  motorRightFront.setSpeed(speedValue);
  motorRightRear.setSpeed(speedValue);
  motorRightFront.run(direction);
  motorRightRear.run(direction);
}

void stopAll() {
  setLeft(RELEASE, 0);
  setRight(RELEASE, 0);
  motorsActive = false;
}

void cancelImuTurn() {
  imuTurnActive = false;
  imuTurnState = 0;
}

void moveStraight(uint8_t direction) {
  // Person 1 measured the left drivetrain as faster. Apply the same
  // mechanical compensation in both travel directions.
  uint8_t leftSpeed = max(0, FORWARD_SPEED - LEFT_SPEED_REDUCTION);
  uint8_t rightSpeed = min(255, FORWARD_SPEED + LEFT_SPEED_REDUCTION);
  setLeft(direction, leftSpeed);
  setRight(direction, rightSpeed);
  motorsActive = true;
}

void rotateLeft() {
  setLeft(BACKWARD, ROTATE_SPEED);
  setRight(FORWARD, ROTATE_SPEED);
  motorsActive = true;
}

void rotateRight() {
  setLeft(FORWARD, ROTATE_SPEED);
  setRight(BACKWARD, ROTATE_SPEED);
  motorsActive = true;
}

void wakeImu() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
}

float readGyroZ() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);
  if (Wire.endTransmission(false) != 0) {
    return 0.0;
  }
  if (Wire.requestFrom(MPU_ADDR, static_cast<uint8_t>(2)) != 2) {
    return 0.0;
  }
  int16_t raw = (static_cast<int16_t>(Wire.read()) << 8) | Wire.read();
  return raw / 131.0;
}

void calibrateGyro() {
  Serial.println("IMU:CALIBRATING");
  float sum = 0.0;
  const uint16_t samples = 100;
  for (uint16_t i = 0; i < samples; i++) {
    sum += readGyroZ();
    delay(10);
  }
  gyroZBias = sum / samples;
  Serial.print("IMU:READY,");
  Serial.println(gyroZBias, 3);
}

void startImuTurn(float targetDeg) {
  cancelImuTurn();
  if (targetDeg > 0) {
    rotateRight();
  } else {
    rotateLeft();
  }
  imuTurnTargetDeg = abs(targetDeg);
  imuTurnAngleDeg = 0.0;
  imuTurnStartedMs = millis();
  lastGyroUs = micros();
  imuTurnActive = true;
  imuTurnState = 1;
}

void updateImuTurn() {
  if (!imuTurnActive) {
    return;
  }
  unsigned long nowUs = micros();
  float dt = (nowUs - lastGyroUs) / 1000000.0;
  lastGyroUs = nowUs;
  float rate = readGyroZ() - gyroZBias;
  if (abs(rate) > 0.3) {
    imuTurnAngleDeg += abs(rate) * dt;
  }
  if (imuTurnAngleDeg >= imuTurnTargetDeg) {
    stopAll();
    imuTurnActive = false;
    imuTurnState = 2;
    Serial.print("TURN_DONE:");
    Serial.println(imuTurnAngleDeg, 1);
  } else if (millis() - imuTurnStartedMs >= IMU_TURN_TIMEOUT_MS) {
    stopAll();
    imuTurnActive = false;
    imuTurnState = 3;
    Serial.println("TURN_ERROR:TIMEOUT");
  }
}

void reportImuTurnStatus() {
  Serial.print("TURN:");
  if (imuTurnState == 1) {
    Serial.println("ACTIVE");
  } else if (imuTurnState == 2) {
    Serial.println("DONE");
  } else if (imuTurnState == 3) {
    Serial.println("ERROR");
  } else {
    Serial.println("IDLE");
  }
}

float readDistanceCm(uint8_t triggerPin, uint8_t echoPin) {
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, 30000UL);
  if (duration == 0) {
    return 999.0;
  }
  return duration * 0.0343 / 2.0;
}

void reportUltrasonic() {
  float left = readDistanceCm(TRIG_LEFT, ECHO_LEFT);
  delay(5);
  float center = readDistanceCm(TRIG_CENTER, ECHO_CENTER);
  delay(5);
  float right = readDistanceCm(TRIG_RIGHT, ECHO_RIGHT);

  Serial.print("US:");
  Serial.print(left, 1);
  Serial.print(",");
  Serial.print(center, 1);
  Serial.print(",");
  Serial.println(right, 1);
}

void resetEncoders() {
  noInterrupts();
  encoderLeftTicks = 0;
  encoderRightTicks = 0;
  interrupts();
  Serial.println("ENC_RESET:OK");
}

void reportEncoders() {
  noInterrupts();
  long leftSnapshot = encoderLeftTicks;
  long rightSnapshot = encoderRightTicks;
  interrupts();
  Serial.print("ENC:");
  Serial.print(leftSnapshot);
  Serial.print(",");
  Serial.println(rightSnapshot);
}

void handleCommand(const char *command) {
  bool recognised = true;

  if (strcmp(command, "FORWARD") == 0) {
    cancelImuTurn();
    moveStraight(FORWARD);
  } else if (strcmp(command, "BACKWARD") == 0) {
    cancelImuTurn();
    moveStraight(BACKWARD);
  } else if (strcmp(command, "ROTATE_LEFT") == 0) {
    cancelImuTurn();
    rotateLeft();
  } else if (strcmp(command, "ROTATE_RIGHT") == 0) {
    cancelImuTurn();
    rotateRight();
  } else if (strncmp(command, "TURN_LEFT", 9) == 0) {
    float target = -90.0;
    if (command[9] == ':') {
      target = -abs(atof(command + 10));
    }
    startImuTurn(target);
  } else if (strncmp(command, "TURN_RIGHT", 10) == 0) {
    float target = 90.0;
    if (command[10] == ':') {
      target = abs(atof(command + 11));
    }
    startImuTurn(target);
  } else if (strncmp(command, "TURN_UTURN", 10) == 0) {
    float target = -180.0;
    if (command[10] == ':') {
      target = -abs(atof(command + 11));
    }
    startImuTurn(target);
  } else if (strcmp(command, "TURN_STATUS") == 0) {
    reportImuTurnStatus();
  } else if (strcmp(command, "STOP") == 0) {
    cancelImuTurn();
    stopAll();
  } else if (strcmp(command, "CHECK") == 0) {
    reportUltrasonic();
  } else if (strcmp(command, "ENCODER") == 0) {
    reportEncoders();
  } else if (strcmp(command, "ENC_RESET") == 0) {
    resetEncoders();
  } else if (command[0] != '\0') {
    recognised = false;
    Serial.println("ERR:UNKNOWN_COMMAND");
  }

  if (recognised) {
    lastCommandMs = millis();
  }
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    char incoming = static_cast<char>(Serial.read());
    if (incoming == '\r') {
      continue;
    }
    if (incoming == '\n') {
      commandBuffer[commandLength] = '\0';
      handleCommand(commandBuffer);
      commandLength = 0;
      continue;
    }
    if (commandLength < COMMAND_BUFFER_SIZE - 1) {
      commandBuffer[commandLength++] = incoming;
    } else {
      commandLength = 0;
      Serial.println("ERR:COMMAND_TOO_LONG");
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_CENTER, OUTPUT);
  pinMode(ECHO_CENTER, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
  pinMode(ENCODER_LEFT_PIN, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), onLeftEncoderPulse, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), onRightEncoderPulse, RISING);
  Wire.begin();
  Wire.setClock(400000);
  wakeImu();
  delay(100);
  calibrateGyro();
  stopAll();
  lastCommandMs = millis();
  Serial.println("READY");
}

void loop() {
  readSerialCommands();
  updateImuTurn();
  // IMU turns use their own bounded timeout so a valid 180-degree turn is not
  // cut off by the normal serial-command watchdog.
  if (!imuTurnActive && motorsActive && millis() - lastCommandMs > COMMAND_TIMEOUT_MS) {
    stopAll();
    Serial.println("WATCHDOG:STOPPED");
  }
}
