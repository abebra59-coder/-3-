/*
  Task:
  Phase 1:
    1) Init sensors. Open claw
    2) Button press
    3) Turn +45 deg CCW
    4) Drive forward 9 cm, close claw
    5) Drive back 9 cm, turn back -45 deg CW to original heading
    6) Drive forward 14 cm, open claw (release)
    7) Drive back 14 cm to starting position
  
  Phase 2:
    8) Turn -45 deg CW
    9) Drive forward 9 cm, close claw
   10) Drive back 9 cm
   11) Turn -45 deg CW (another -45° turn, total -90° from start)
   12) Drive forward 14 cm, open claw (release)
   13) Drive back 14 cm to final position (full return)

  Phase 3 (MODIFIED - 13 cm movements):
   14) Turn -45 deg CW (clockwise)
   15) Drive forward 13 cm, close claw
   16) Drive back 13 cm
   17) Turn +45 deg CCW (counter-clockwise) - back to heading before phase 3
   18) Drive forward 14 cm, open claw (release)
   19) Drive back 14 cm to starting position

  Phase 4 (NEW):
   20) Turn -135 deg CW (clockwise)
   21) Drive forward 10 cm, close claw
   22) Drive back 10 cm
   23) Turn -45 deg CW (clockwise)
   24) Drive forward 14 cm, open claw (release)
   25) Drive back 14 cm to final position

  Board: Arduino Nano (ATmega328P Old Bootloader)
  Motors: DRV8833 (PWM only on IN2)
  IMU: MPU6050_light
  Encoder: D2 (single channel)
  Claw: Servo on D11 (VEX-style microseconds)
*/

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <Servo.h>

// ================== HW ==================
MPU6050 mpu(Wire);
Servo vex;

// Motor pins
const int L_IN1 = 8, L_IN2 = 6;   // IN2 PWM
const int R_IN1 = 7, R_IN2 = 3;   // IN2 PWM
const int BTN_PIN = 4;

// Encoder
const int ENC_PIN = 2;
const unsigned long MIN_PULSE_US = 180;
volatile long encTicks = 0;
volatile unsigned long lastPulseUs = 0;

// Claw
const int VEX_PIN = 11;
const int PWM_STOP  = 1500;
const int PWM_OPEN  = 1800;
const int PWM_CLOSE = 1300;
const unsigned long OPEN_MS  = 750;
const unsigned long CLOSE_MS = 600;

// ================== TURN PD ==================
const float KP = 2.9f;
const float KD = 0.12f;
const float TOL_DEG = 0.5f;

const int MIN_PWM_L = 55;
const int MIN_PWM_R = 53;
const int PWM_MAX = 180;
const unsigned long MAX_TURN_MS = 6000;

// ================== DRIVE ==================
const long TICKS_PER_METER = 180;
const int SPEED_FWD = 90;
const int SPEED_BWD = 90;
const int TRIM_L = +2;
const int TRIM_R = 0;

// ================== BUTTON ==================
bool buttonPressed() { return digitalRead(BTN_PIN) == LOW; }
void waitButtonPressRelease() {
  while (!buttonPressed()) delay(5);
  delay(30);
  while (buttonPressed()) delay(5);
  delay(80);
}

// ================== ENCODER ISR ==================
void isrEnc() {
  unsigned long now = micros();
  if (now - lastPulseUs >= MIN_PULSE_US) {
    encTicks++;
    lastPulseUs = now;
  }
}

void resetEnc() {
  noInterrupts();
  encTicks = 0;
  lastPulseUs = micros();
  interrupts();
}

long getEnc() {
  long t;
  noInterrupts();
  t = encTicks;
  interrupts();
  return t;
}

// ================== MOTORS ==================
void stopAll() {
  digitalWrite(L_IN1, LOW); digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN1, LOW); digitalWrite(R_IN2, LOW);
}

void motorSigned_IN2PWM(int in1, int in2_pwm, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed == 0) { digitalWrite(in1, LOW); digitalWrite(in2_pwm, LOW); return; }

  int pwm = abs(speed);
  if (speed > 0) {
    digitalWrite(in1, LOW);
    analogWrite(in2_pwm, pwm);
  } else {
    digitalWrite(in1, HIGH);
    analogWrite(in2_pwm, 255 - pwm);
  }
}

void setMotors(int leftSpeed, int rightSpeed) {
  motorSigned_IN2PWM(L_IN1, L_IN2, leftSpeed);
  motorSigned_IN2PWM(R_IN1, R_IN2, rightSpeed);
}

float wrapErrDeg(float e) {
  while (e > 180.0f) e -= 360.0f;
  while (e < -180.0f) e += 360.0f;
  return e;
}

int applyMin(int cmd, int minPwm) {
  if (cmd == 0) return 0;
  int s = (cmd > 0) ? 1 : -1;
  int a = abs(cmd);
  if (a < minPwm) a = minPwm;
  return s * a;
}

// ================== CLAW ==================
void gripperOpen() {
  vex.writeMicroseconds(PWM_STOP);
  delay(120);
  vex.writeMicroseconds(PWM_OPEN);
  delay(OPEN_MS);
  vex.writeMicroseconds(PWM_STOP);
  delay(200);
}

void gripperClose() {
  vex.writeMicroseconds(PWM_STOP);
  delay(120);
  vex.writeMicroseconds(PWM_CLOSE);
  delay(CLOSE_MS);
  vex.writeMicroseconds(PWM_STOP);
  delay(200);
}

// ================== TURN RELATIVE (PD) ==================
void turnRelative(float deg) {
  mpu.update();
  float startYaw = mpu.getAngleZ();
  float targetYaw = startYaw + deg;

  unsigned long t0 = millis();

  while (true) {
    mpu.update();

    float yaw = mpu.getAngleZ();
    float e = wrapErrDeg(targetYaw - yaw);
    float w = mpu.getGyroZ();

    if (abs(e) <= TOL_DEG) break;

    int u = (int)round(KP * e - KD * w);
    u = constrain(u, -PWM_MAX, PWM_MAX);

    int leftCmd  = applyMin(-u, MIN_PWM_L);
    int rightCmd = applyMin( u, MIN_PWM_R);

    setMotors(leftCmd, rightCmd);

    if (millis() - t0 > MAX_TURN_MS) break;
    delay(5);
  }

  stopAll();
  delay(200);
}

// ================== DRIVE BY ENCODER ==================
void driveCm(float cm, bool forward) {
  if (cm <= 0.1f) return;

  long targetTicks = (long)round(TICKS_PER_METER * (cm / 100.0f));
  long slowTicks   = (long)round(TICKS_PER_METER * 0.03f);
  if (slowTicks < 2) slowTicks = 2;

  resetEnc();

  int base = forward ? SPEED_FWD : -SPEED_BWD;

  while (true) {
    long t = getEnc();
    long remain = targetTicks - t;
    if (remain <= 0) break;

    int pwm = abs(base);

    if (remain < slowTicks) {
      pwm = (int)round((float)abs(base) * (float)remain / (float)slowTicks);
      if (pwm < 60) pwm = 60;
    }

    int l = (base > 0 ? +pwm : -pwm) + TRIM_L;
    int r = (base > 0 ? +pwm : -pwm) + TRIM_R;

    l = applyMin(l, MIN_PWM_L);
    r = applyMin(r, MIN_PWM_R);

    setMotors(l, r);
    delay(5);
  }

  stopAll();
  delay(200);
}

// ================== MAIN ==================
void setup() {
  pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT);
  pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT);
  stopAll();

  pinMode(BTN_PIN, INPUT_PULLUP);

  pinMode(ENC_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_PIN), isrEnc, RISING);

  vex.attach(VEX_PIN);
  vex.writeMicroseconds(PWM_STOP);
  delay(200);

  Wire.begin();
  if (mpu.begin() != 0) {
    while (true) { stopAll(); delay(50); }
  }
  delay(600);
  mpu.calcOffsets(true, true);

  // 1) Init done -> open claw
  gripperOpen();
}

void loop() {
  // 2) Button press to start
  waitButtonPressRelease();

  // ===== PHASE 1 =====
  // 3) Turn +45 deg CCW
  turnRelative(+45.0f);

  // 4) Drive forward 9 cm, close claw
  driveCm(9.0f, true);
  gripperClose();

  // 5) Drive back 9 cm, turn back -45 deg CW
  driveCm(9.0f, false);
  turnRelative(-45.0f);

  // 6) Drive forward 14 cm, open claw (release)
  driveCm(14.0f, true);
  gripperOpen();

  // 7) Drive back 14 cm to starting position
  driveCm(14.0f, false);

  // ===== PHASE 2 =====
  // 8) Turn -45 deg CW
  turnRelative(-45.0f);

  // 9) Drive forward 9 cm, close claw
  driveCm(9.0f, true);
  gripperClose();

  // 10) Drive back 9 cm
  driveCm(9.0f, false);

  // 11) Turn -45 deg CW (another clockwise turn, total -90° from start heading)
  turnRelative(-45.0f);

  // 12) Drive forward 14 cm, open claw (release)
  driveCm(14.0f, true);
  gripperOpen();

  // 13) Drive back 14 cm to final position
  driveCm(14.0f, false);

  // ===== PHASE 3 (MODIFIED - 13 cm movements) =====
  // 14) Turn -45 deg CW (clockwise)
  turnRelative(-45.0f);

  // 15) Drive forward 13 cm, close claw
  driveCm(13.0f, true);
  gripperClose();

  // 16) Drive back 13 cm
  driveCm(13.0f, false);

  // 17) Turn +45 deg CCW (counter-clockwise) - back to heading before phase 3 started
  turnRelative(+45.0f);

  // 18) Drive forward 14 cm, open claw (release)
  driveCm(14.0f, true);
  gripperOpen();

  // 19) Drive back 14 cm to starting position
  driveCm(14.0f, false);

  // ===== PHASE 4 (NEW SEQUENCE) =====
  // 20) Turn -135 deg CW (clockwise)
  turnRelative(-135.0f);

  // 21) Drive forward 10 cm, close claw
  driveCm(10.0f, true);
  gripperClose();

  // 22) Drive back 10 cm
  driveCm(10.0f, false);

  // 23) Turn -45 deg CW (clockwise)
  turnRelative(-45.0f);

  // 24) Drive forward 14 cm, open claw (release)
  driveCm(14.0f, true);
  gripperOpen();

  // 25) Drive back 14 cm to final position
  driveCm(14.0f, false);

  // Final stop - prevent restart
  stopAll();
  while (true) { delay(50); }
}