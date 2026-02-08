#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <Servo.h>


MPU6050 mpu(Wire);
Servo vex;

const int L_IN1 = 8, L_IN2 = 6;   
const int R_IN1 = 7, R_IN2 = 3;   
const int BTN_PIN = 4;


const int ENC_PIN = 2;
const unsigned long MIN_PULSE_US = 180;
volatile long encTicks = 0;
volatile unsigned long lastPulseUs = 0;


const int VEX_PIN = 11;
const int PWM_STOP  = 1500;
const int PWM_OPEN  = 1800;
const int PWM_CLOSE = 1300;
const unsigned long OPEN_MS  = 750;
const unsigned long CLOSE_MS = 600;


const float KP = 2.9f;
const float KD = 0.12f;
const float TOL_DEG = 0.5f;

const int MIN_PWM_L = 55;
const int MIN_PWM_R = 53;
const int PWM_MAX = 180;
const unsigned long MAX_TURN_MS = 6000;


const long TICKS_PER_METER = 180;
const int SPEED_FWD = 90;
const int SPEED_BWD = 90;
const int TRIM_L = +2;
const int TRIM_R = 0;

bool buttonPressed() { return digitalRead(BTN_PIN) == LOW; }
void waitButtonPressRelease() {
  while (!buttonPressed()) delay(5);
  delay(30);
  while (buttonPressed()) delay(5);
  delay(80);
}

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

  gripperOpen();
}

void loop() {

  waitButtonPressRelease();

  turnRelative(+45.0f);

  driveCm(9.0f, true);
  gripperClose();

  driveCm(9.0f, false);
  turnRelative(-45.0f);

  driveCm(14.0f, true);
  gripperOpen();

  driveCm(14.0f, false);

  turnRelative(-45.0f);

  driveCm(9.0f, true);
  gripperClose();

  driveCm(9.0f, false);

  turnRelative(-45.0f);


  driveCm(14.0f, true);
  gripperOpen();

  driveCm(14.0f, false);

  turnRelative(-45.0f);


  driveCm(13.0f, true);
  gripperClose();


  driveCm(13.0f, false);


  turnRelative(+45.0f);

  driveCm(14.0f, true);
  gripperOpen();

  driveCm(14.0f, false);

  turnRelative(-135.0f);


  driveCm(10.0f, true);
  gripperClose();

  driveCm(10.0f, false);

 
  turnRelative(-45.0f);

  driveCm(14.0f, true);
  gripperOpen();


  driveCm(14.0f, false);

  stopAll();
  while (true) { delay(50); }

}
