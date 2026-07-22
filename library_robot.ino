// ============================================================
//  Library Robot — Arduino Firmware
//  Person 1: 电机驱动 + 3×超声波 + 串口指令 + 前馈补偿
// ============================================================

#include <AFMotor.h>

// ---- 电机 ----
// M1=左前 M4=左后, M2=右前 M3=右后
AF_DCMotor motorL1(1);
AF_DCMotor motorL2(4);
AF_DCMotor motorR1(2);
AF_DCMotor motorR2(3);

#define FWD_SPEED      180
#define ROTATE_SPEED   120
#define TIMEOUT_MS     2000

// ---- 前馈补偿 ----
// 左侧机械上更快 → 减速左、加速右使实际转速一致
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

// ---- 直行 (带前馈补偿) ----
void moveStraight(uint8_t dir, int baseSpeed) {
  int ff = (dir == FORWARD) ? feedforward : -feedforward;
  int lSpd = constrain(baseSpeed - ff, 0, 255);
  int rSpd = constrain(baseSpeed + ff, 0, 255);
  setAll(dir, lSpd, dir, rSpd);
}

// ---- 原地转向 (不加前馈) ----
void rotateInPlace(uint8_t lDir, uint8_t rDir, int speed) {
  setAll(lDir, speed, rDir, speed);
}

// ============================================================
void setup() {
  Serial.begin(115200);

  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_C, OUTPUT); pinMode(ECHO_C, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);

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
    else if (cmd == "ROTATE_LEFT") {
      rotateInPlace(BACKWARD, FORWARD, ROTATE_SPEED);
    }
    else if (cmd == "ROTATE_RIGHT") {
      rotateInPlace(FORWARD, BACKWARD, ROTATE_SPEED);
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

  // 安全超时
  if (millis() - lastCmdMs > TIMEOUT_MS) {
    stopAll();
  }
}
