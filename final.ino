#include <Arduino.h>
#include <math.h>
#include <util/atomic.h>

// [상수/변수 선언부]
#define STEERING_PULSE_PIN 2
#define ACCEL_PULSE_PIN 3
#define ENCODER_A 18
#define ENCODER_B 19
#define MANUAL_MODE_PIN 20
#define AUTO_MODE_PIN 21

#define BREAK_MODE 200
#define MANUAL_MODE 1400
#define AUTO_MODE 1700

#define POT_MAX 900
#define POT_MIN 10
#define MAX_STEER_TIRE_DEG 24

#define KP 0.3
#define KI 0.0005
#define KD 0.004
#define PID_DEADBAND 0.07
// 조향 속도 미세 상향: 기존 실질 최대 50% -> 60%
#define STEER_PWM_GAIN 0.80
// 조향 각속도 로그용 저역통과필터(0~1, 클수록 반응 빠름)
#define STEER_RATE_LPF_ALPHA 0.25

#define ACCEL_CENTER_US 1500
#define STEER_CENTER_US 1500
#define ACCEL_DB_US 50
#define STEER_DB_US 25
#define ACCEL_FWD_MAX_US 1804
#define ACCEL_REV_MIN_US 1104
#define RC_THROTTLE_DB_NORM 0.03
#define RC_STEER_DB_NORM 0.1
#define STEER_DEADBAND_DEG 1.0

volatile long encoderCount = 0;
volatile uint32_t steer_rise_us = 0, accel_rise_us = 0, manual_rise_us = 0, auto_rise_us = 0;
volatile uint16_t Steering_us = 1500, Accel_us = 1500, Manual_us = 1000, Auto_us = 1000;
volatile uint32_t accel_last_us = 0;

int DIR1 = 10, PWM1 = 11, DIR2 = 6, PWM2 = 7, DIR3 = 8, PWM3 = 9;
int POTPin = A0;
#define MIN_DRIVE_PWM 0
#define MAX_DRIVE_PWM 250

#define SERIAL_BUFFER_SIZE 48
char serialBuffer[SERIAL_BUFFER_SIZE];
size_t bufferIndex = 0;

float throttle_cmd = 0.0f;
float steer_auto_deg = 0.0f;
bool throttleFresh = false;
bool steerFresh = false;
unsigned long lastThrottleMs = 0;
unsigned long lastSteerMs = 0;

#define THROTTLE_TIMEOUT_MS 500
#define STEER_TIMEOUT_MS 500

#define PULSE_MIN 500
#define PULSE_MAX 2500
#define SIGNAL_THRESHOLD 0.05 

volatile uint32_t steer_last_us = 0, manual_last_us = 0, auto_last_us = 0;

void Steer(double u) {
  u = constrain(u, -1.0, 1.0);

  if (fabs(u) < PID_DEADBAND) {
    analogWrite(PWM3, 0);
    digitalWrite(DIR3, LOW);
    return;
  }

  int pwm_val = (int)(fabs(u) * 255.0 * STEER_PWM_GAIN);
  pwm_val = constrain(pwm_val, 0, 255);

  if (u > 0) {
    digitalWrite(DIR3, HIGH);
    analogWrite(PWM3, pwm_val*0.6);
  } else {
    digitalWrite(DIR3, LOW);
    analogWrite(PWM3, pwm_val*0.6);
  }
}

void parseSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n') {
      serialBuffer[bufferIndex] = '\0';

      if (strncmp(serialBuffer, "TH", 2) == 0 || strncmp(serialBuffer, "th", 2) == 0) {
        char *p = serialBuffer + 2;
        while (*p == ' ' || *p == '\t') ++p;
        float v = atof(p);
        if (v > 1.0f) v = 1.0f;
        if (v < -1.0f) v = -1.0f;
        throttle_cmd = v;
        throttleFresh = true;
        lastThrottleMs = millis();
      }
      else if (strncmp(serialBuffer, "SA", 2) == 0 || strncmp(serialBuffer, "sa", 2) == 0) {
        char *p = serialBuffer + 2;
        while (*p == ' ' || *p == '\t') ++p;
        float a = atof(p);
        if (a > MAX_STEER_TIRE_DEG) a = MAX_STEER_TIRE_DEG;
        if (a < -MAX_STEER_TIRE_DEG) a = -MAX_STEER_TIRE_DEG;
        steer_auto_deg = a;
        steerFresh = true;
        lastSteerMs = millis();
      }
      bufferIndex = 0;
    }
    else if (c != '\r') {
      if (bufferIndex < SERIAL_BUFFER_SIZE - 1) {
        serialBuffer[bufferIndex++] = c;
      } else {
        bufferIndex = 0;
      }
    }
  }
}

static inline float Mapping(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static inline double applyDeadband(double x, double band) {
  return (fabs(x) < band) ? 0.0 : x;
}

const unsigned int DIR_DEADTIME_US = 200;
int last_dir_sign = 0;

void driveWithDeadtime(float cmd) {
  cmd = constrain(cmd, -1.0f, 1.0f);

  float mag = fabs(cmd);
  int dir_sign = (cmd > 0.0f) ? +1 : (cmd < 0.0f ? -1 : 0);

  if (mag < SIGNAL_THRESHOLD || dir_sign == 0) {
    analogWrite(PWM1, 0);
    analogWrite(PWM2, 0);
    last_dir_sign = 0;
    return;
  }

  if (last_dir_sign != 0 && dir_sign != last_dir_sign) {
    analogWrite(PWM1, 0);
    analogWrite(PWM2, 0);
    delayMicroseconds(DIR_DEADTIME_US);
  }

  if (dir_sign > 0) { // 전진
    digitalWrite(DIR1, HIGH);
    digitalWrite(DIR2, LOW);
  } else {            // 후진
    digitalWrite(DIR1, LOW);
    digitalWrite(DIR2, HIGH);
  }

  int in = (int)Mapping(mag, 0.0f, 1.0f, (float)MIN_DRIVE_PWM, (float)MAX_DRIVE_PWM);
  analogWrite(PWM1, in);
  analogWrite(PWM2, in);

  last_dir_sign = dir_sign;
}

void SteeringPulseInt() {
  uint32_t now = micros();
  if (digitalRead(STEERING_PULSE_PIN) == HIGH) {
    steer_rise_us = now;
  } else {
    uint32_t w = now - steer_rise_us;
    if (w >= PULSE_MIN && w <= PULSE_MAX) {
      Steering_us = (uint16_t)w;
      steer_last_us = now;
    }
  }
}

void AccelPulseInt() {
  uint32_t now = micros();
  if (digitalRead(ACCEL_PULSE_PIN) == HIGH) {
    accel_rise_us = now;
  } else {
    uint32_t w = now - accel_rise_us;
    if (w >= PULSE_MIN && w <= PULSE_MAX) {
      Accel_us = (uint16_t)w;
      accel_last_us = now;
    }
  }
}

void ManualPulseInt() {
  uint32_t now = micros();
  if (digitalRead(MANUAL_MODE_PIN) == HIGH) {
    manual_rise_us = now;
  } else {
    uint32_t w = now - manual_rise_us;
    if (w >= PULSE_MIN && w <= PULSE_MAX) {
      Manual_us = (uint16_t)w;
      manual_last_us = now;
    }
  }
}

void AutoPulseInt() {
  uint32_t now = micros();
  if (digitalRead(AUTO_MODE_PIN) == HIGH) {
    auto_rise_us = now;
  } else {
    uint32_t w = now - auto_rise_us;
    if (w >= PULSE_MIN && w <= PULSE_MAX) {
      Auto_us = (uint16_t)w;
      auto_last_us = now;
    }
  }
}

void encoderISR() {
  if (digitalRead(ENCODER_A) == digitalRead(ENCODER_B)) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

double PID(double ref, double sense, unsigned long dt_us) {
  static double prev_err = 0.0;
  static double integral = 0.0;

  double dt_s = dt_us * 1.0e-6;
  if (dt_s <= 0.0) dt_s = 1e-6;

  double err = ref - sense;
  integral += err * dt_s;

  double P = KP * err;
  double I = KI * integral;
  double D = KD * (err - prev_err) / dt_s;

  prev_err = err;
  return P + I + D;
}

void StopMotor() {
  digitalWrite(DIR1, HIGH);
  analogWrite(PWM1, 0);
  digitalWrite(DIR2, HIGH);
  analogWrite(PWM2, 0);
  digitalWrite(DIR3, LOW); // 조향 프리휠
  analogWrite(PWM3, 0);
}

void MoveForward(double throttle) {
  if (throttle > 1.0) throttle = 1.0;
  else if (throttle < 0.0) throttle = 0.0;

  int in = (int)(Mapping(throttle, 0.0, 1.0, MIN_DRIVE_PWM, MAX_DRIVE_PWM));
  if (throttle < 0.01) in = 0;
  
  digitalWrite(DIR1, HIGH);
  analogWrite(PWM1, in);
  digitalWrite(DIR2, LOW);
  analogWrite(PWM2, in);
}

void MoveBackward(double throttle) {
  if (throttle > 1.0) throttle = 1.0;
  else if (throttle < 0.0) throttle = 0.0;

  int in = (int)(Mapping(throttle, 0.0, 1.0, MIN_DRIVE_PWM, MAX_DRIVE_PWM));
  if (throttle < 0.01) in = 0;

  digitalWrite(DIR1, LOW);
  analogWrite(PWM1, in);
  digitalWrite(DIR2, HIGH);
  analogWrite(PWM2, in);
}
  

void CenterSteeringOnce() {
  const double CENTER_DEG = 0.0;      // 목표 센터 각도
  const double TOL_DEG = 3;         // 허용 오차
  const unsigned long TIMEOUT_MS = 1200;

  unsigned long t0 = millis();

  while (millis() - t0 < TIMEOUT_MS) {
    // 현재 각도 읽기
    int pot = analogRead(POTPin);
    double deg = Mapping(pot, POT_MIN, POT_MAX, +MAX_STEER_TIRE_DEG, -MAX_STEER_TIRE_DEG);

    // 에러 기반으로 간단히 조향 구동(센터 방향)
    double err = CENTER_DEG - deg;

    // 너무 미세하면 멈춤
    if (fabs(err) <= TOL_DEG) {
      digitalWrite(DIR3, LOW);
      analogWrite(PWM3, 0);;
      break;
    }

    // 센터로 당기는 힘(0.4는 적당한 값, 필요하면 0.2~0.6 조절)
    double u = constrain(err / MAX_STEER_TIRE_DEG, -1.0, 1.0);
    u = constrain(u * 0.6, -0.6, 0.6);
    if (fabs(u) < 0.12) u = (u > 0) ? 0.12 : -0.12;  // ✅ 최소 힘 보장
    Steer(u);
    delay(10);
  }

  Steer(0.0); // 마지막에 조향 정지
}

void setup() {
  Serial.begin(57600);

  pinMode(STEERING_PULSE_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(STEERING_PULSE_PIN), SteeringPulseInt, CHANGE);

  pinMode(ACCEL_PULSE_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ACCEL_PULSE_PIN), AccelPulseInt, CHANGE);

  pinMode(MANUAL_MODE_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(MANUAL_MODE_PIN), ManualPulseInt, CHANGE);

  pinMode(AUTO_MODE_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(AUTO_MODE_PIN), AutoPulseInt, CHANGE);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);

  pinMode(POTPin, INPUT);

  pinMode(DIR1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(DIR3, OUTPUT);
  pinMode(PWM3, OUTPUT);

  StopMotor();
  digitalWrite(DIR3, LOW);
  analogWrite(PWM3, 0);
  CenterSteeringOnce();
}

void loop() {
  // 루프 안정화를 위한 짧은 지연 (10ms)
  delay(10); 

  parseSerial();
  static unsigned long prev_t_us = 0;
  unsigned long t_us = micros();
  unsigned long dt_us = (prev_t_us == 0) ? 1000UL : (t_us - prev_t_us);
  prev_t_us = t_us;

  long Steering_us_local, Accel_us_local, Manual_us_local, Auto_us_local, encoder_local;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    Steering_us_local = Steering_us; 
    Accel_us_local = Accel_us;
    Manual_us_local = Manual_us; 
    Auto_us_local = Auto_us;
    encoder_local = encoderCount;
  }

  int Mode_val = BREAK_MODE;
  if (Manual_us_local > 1600) Mode_val = MANUAL_MODE;
  else if (Auto_us_local > 1600) Mode_val = AUTO_MODE;

  if (micros() - accel_last_us > 30000) Accel_us_local = ACCEL_CENTER_US;
  
  float Throttle_input = 0.0f;
  if (Accel_us_local > ACCEL_CENTER_US + ACCEL_DB_US)
    Throttle_input = (float)(Accel_us_local - ACCEL_CENTER_US) / (ACCEL_FWD_MAX_US - ACCEL_CENTER_US);
  else if (Accel_us_local < ACCEL_CENTER_US - ACCEL_DB_US)
    Throttle_input = (float)(Accel_us_local - ACCEL_CENTER_US) / (ACCEL_CENTER_US - ACCEL_REV_MIN_US);
  
  Throttle_input = constrain(Throttle_input, -1.0, 1.0);
  
  float Steer_rc = Mapping(Steering_us_local, 1280, 1792, -1.0, 1.0);
  Steer_rc = applyDeadband(Steer_rc, RC_STEER_DB_NORM);
  double ref_steer_deg_rc = Mapping(Steer_rc, -1.0, 1.0, +MAX_STEER_TIRE_DEG, -MAX_STEER_TIRE_DEG);

  int POTval = analogRead(POTPin);
  double raw_deg = Mapping(POTval, POT_MIN, POT_MAX, +MAX_STEER_TIRE_DEG, -MAX_STEER_TIRE_DEG);
  double deg = applyDeadband(raw_deg, STEER_DEADBAND_DEG);

  static bool steer_rate_init = false;
  static double prev_raw_deg = 0.0;
  static double steer_rate_dps = 0.0;
  double dt_s = dt_us * 1.0e-6;
  if (dt_s <= 0.0) dt_s = 1.0e-6;

  if (!steer_rate_init) {
    steer_rate_init = true;
    prev_raw_deg = raw_deg;
    steer_rate_dps = 0.0;
  } else {
    double rate_raw_dps = (raw_deg - prev_raw_deg) / dt_s;
    steer_rate_dps += STEER_RATE_LPF_ALPHA * (rate_raw_dps - steer_rate_dps);
    prev_raw_deg = raw_deg;
  }

  bool sa_ok = steerFresh && (millis() - lastSteerMs <= STEER_TIMEOUT_MS);
  double ref_steer_deg = (Mode_val == AUTO_MODE && sa_ok) ? (double)steer_auto_deg : ref_steer_deg_rc;
  
  bool th_ok = throttleFresh && (millis() - lastThrottleMs <= THROTTLE_TIMEOUT_MS);

// ===== 조향 PID: 모드에 따라 1번만 계산 (섞임 방지) =====
double u_rc = 0.0;
double u_auto = 0.0;

if (Mode_val == AUTO_MODE && sa_ok) {
  // AUTO: SA 기반 PID
  u_auto = PID((double)steer_auto_deg, deg, dt_us);
  u_auto = applyDeadband(u_auto, PID_DEADBAND);
  u_auto = constrain(u_auto, -1.0, 1.0);
} else {
  // MANUAL 또는 SA 끊김: RC 기반 PID
  u_rc = PID(ref_steer_deg_rc, deg, dt_us);
  u_rc = applyDeadband(u_rc, PID_DEADBAND);
  u_rc = constrain(u_rc, -1.0, 1.0);
}

// 디버그/제어에 쓸 현재 조향 출력
double u_used = (Mode_val == AUTO_MODE && sa_ok) ? u_auto : u_rc;

  if (Mode_val == BREAK_MODE) {
  StopMotor();
}
else if (Mode_val == MANUAL_MODE) {
  // ---- MANUAL은 1번 방식 유지 ----
  if (Throttle_input > 0.05f)       MoveForward(Throttle_input * 0.6f);
  else if (Throttle_input < -0.05f) MoveBackward((-Throttle_input) * 0.6f);
  else { analogWrite(PWM1, 0); analogWrite(PWM2, 0); }

  Steer(u_rc);   
}
else { // AUTO_MODE
  // ---- AUTO는 2번 방식 ----
  float th = th_ok ? throttle_cmd : 0.0f;  // 타임아웃이면 정지(안전)
  driveWithDeadtime(th);

  if (sa_ok) Steer(u_auto);  
  else       Steer(u_rc);    
}

  static unsigned long lp = 0;
  if (millis() - lp > 100) {
    static long prev_encoder = 0;
    long d_encoder = encoder_local - prev_encoder;
    prev_encoder = encoder_local;

    lp = millis();
    Serial.print("MODE:"); Serial.print(Mode_val);
Serial.print(" | Tgt:"); Serial.print(ref_steer_deg, 1);
Serial.print(" | Cur:"); Serial.print(deg, 1);
Serial.print(" | dDeg/s:"); Serial.print(steer_rate_dps, 1);
Serial.print(" | PID:"); Serial.print(u_used, 2);
Serial.print(" | POT:"); Serial.print(POTval);
Serial.print(" | ENC:"); Serial.print(encoder_local);
Serial.print(" | dENC:"); Serial.print(d_encoder);

Serial.print(" | TH:"); Serial.print(throttle_cmd, 2);
Serial.print(" ok:"); Serial.print((int)th_ok);
Serial.print(" | SA:"); Serial.print(steer_auto_deg, 1);
Serial.print(" ok:"); Serial.println((int)sa_ok);
  }
}
