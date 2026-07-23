// ============================================================
//  Library Robot — Arduino Firmware
//  Person 1: 电机 + 3×超声波 + IMU 90° 精确转向 + 前馈
// ============================================================

#include <AFMotor.h>
#include <Wire.h>

// ---- 电机 ----
AF_DCMotor motorL1(1);
AF_DCMotor motorL2(4);
AF_DCMotor motorR1(2);
AF_DCMotor motorR2(3);

#define FWD_SPEED      180
#define TURN_SPEED     120
#define TIMEOUT_MS     2000

int feedforward = 15;
unsigned long lastCmdMs = 0;

// ---- 超声波 ----
#define TRIG_L 25
#define ECHO_L 24
#define TRIG_C 23
#define ECHO_C 22
#define TRIG_R 27
#define ECHO_R 26

float readUS(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long dur = pulseIn(echo, HIGH, 30000);
  if (dur == 0) return 999.0;
  return dur * 0.0343 / 2.0;
}

// ---- IMU MPU6500 ----
#define MPU_ADDR 0x68
float gzBias = 0.5106;   // 预校准值，启动时会重测

void imuWake() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00);
  Wire.endTransmission();
}

float readGyroZ() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)2);
  int16_t raw = (Wire.read() << 8) | Wire.read();
  return raw / 131.0;
}

// ---- 电机控制 ----
void setLeft(uint8_t dir, int spd) {
  motorL1.setSpeed(spd); motorL1.run(dir);
  motorL2.setSpeed(spd); motorL2.run(dir);
}

void setRight(uint8_t dir, int spd) {
  motorR1.setSpeed(spd); motorR1.run(dir);
  motorR2.setSpeed(spd); motorR2.run(dir);
}

void setAll(uint8_t lDir, int lSpd, uint8_t rDir, int rSpd) {
  setLeft(lDir, lSpd);
  setRight(rDir, rSpd);
}

void stopAll() {
  setAll(RELEASE, 0, RELEASE, 0);
}

void moveStraight(uint8_t dir, int baseSpeed) {
  int ff = (dir == FORWARD) ? feedforward : -feedforward;
  int lSpd = constrain(baseSpeed - ff, 0, 255);
  int rSpd = constrain(baseSpeed + ff, 0, 255);
  setAll(dir, lSpd, dir, rSpd);
}

// ---- IMU 精确转向 ----
void turnByAngle(float targetDeg) {
  // targetDeg > 0 = 右转, < 0 = 左转
  if (targetDeg > 0) {
    setAll(FORWARD, TURN_SPEED, BACKWARD, TURN_SPEED);   // 右转
  } else {
    setAll(BACKWARD, TURN_SPEED, FORWARD, TURN_SPEED);    // 左转
  }

  float angle = 0.0;
  unsigned long lastUs = micros();

  while (abs(angle) < abs(targetDeg)) {
    float gz = readGyroZ() - gzBias;

    unsigned long now = micros();
    float dt = (now - lastUs) / 1000000.0;
    lastUs = now;

    // 死区过滤
    if (abs(gz) > 0.3) {
      angle += (targetDeg > 0) ? gz * dt : -gz * dt;
    }

    delay(5);  // ~200Hz 采样
  }

  stopAll();
  lastCmdMs = millis();  // 刷新超时计时器
  Serial.print("TURN_DONE:");
  Serial.println(angle, 1);
}

// ============================================================
void setup() {
  Serial.begin(115200);

  // 超声波
  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_C, OUTPUT); pinMode(ECHO_C, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);

  // IMU
  Wire.begin();
  Wire.setClock(400000);
  imuWake();
  delay(100);

  // 自动重校准陀螺仪 bias
  Serial.println("Calibrating gyro... keep still!");
  float sum = 0;
  for (int i = 0; i < 100; i++) {
    sum += readGyroZ();
    delay(10);
  }
  gzBias = sum / 100.0;
  Serial.print("Gyro bias: ");
  Serial.println(gzBias, 3);

  stopAll();
  Serial.println("READY");
}

// ============================================================
void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    lastCmdMs = millis();

    if (cmd == "FORWARD") {
      moveStraight(FORWARD, FWD_SPEED);
    }
    else if (cmd == "BACKWARD") {
      moveStraight(BACKWARD, FWD_SPEED);
    }
    else if (cmd == "TURN_LEFT") {
      turnByAngle(-90.0);
    }
    else if (cmd == "TURN_RIGHT") {
      turnByAngle(90.0);
    }
    else if (cmd == "STOP") {
      stopAll();
    }
    else if (cmd == "CHECK") {
      float l = readUS(TRIG_L, ECHO_L);
      delay(5);
      float c = readUS(TRIG_C, ECHO_C);
      delay(5);
      float r = readUS(TRIG_R, ECHO_R);
      Serial.print("US:");
      Serial.print(l, 1);
      Serial.print(",");
      Serial.print(c, 1);
      Serial.print(",");
      Serial.println(r, 1);
    }
  }

  if (millis() - lastCmdMs > TIMEOUT_MS) {
    stopAll();
  }
}
