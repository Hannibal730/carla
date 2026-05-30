#include <Arduino.h>
#include <math.h>
#include <util/atomic.h>

// =============================================================================
// 시리얼 통신 프로토콜 요약 (방향 구분 중요)
//
//   [ROS2 → 아두이노] 수신 명령어 (parseSerial에서 처리)
//     "TH <float>"  : 쓰로틀 명령 (-1.0 ~ +1.0). 양수=전진, 음수=후진
//     "SA <float>"  : 조향 목표각 명령 (도°, -24 ~ +24). MPPI가 AUTO 모드에서 전송
//
//   [아두이노 → ROS2] 송신 텔레메트리 (loop 100ms마다 출력)
//     "VX:<float>"  : 휠 엔코더로 계산한 전륜 선속도 (m/s)
//     "PS:<float>"  : POT으로 측정한 실제 조향각 (도°). POT Steering의 약자
//                     SA(명령값)와 달리 물리적으로 측정된 실제값임
//
//   ROS2(ros2_sensor.py)에서는 VX와 PS를 함께 수신하여
//   후륜축 실제 선속도를 계산한다:
//     v_rear = VX × cos(PS × π/180)
//   이 값을 /wheel/odom 의 twist.twist.linear.x 에 넣는다.
// =============================================================================


// =============================================================================
// RC 수신기 PWM 입력 핀 (외부 인터럽트 사용)
// RC 수신기는 채널별로 1000~2000 us 폭의 PWM 신호를 출력한다.
// =============================================================================
#define STEERING_PULSE_PIN 2   // RC 수신기 조향 채널 → 수동 조향 목표각 결정
#define ACCEL_PULSE_PIN    3   // RC 수신기 스로틀 채널 → 수동 주행 속도 결정
#define MANUAL_MODE_PIN   20   // RC 수신기 모드 채널A → HIGH이면 수동 모드
#define AUTO_MODE_PIN     21   // RC 수신기 모드 채널B → HIGH이면 자율주행 모드

// =============================================================================
// 쿼드러처 휠 엔코더 핀 (외부 인터럽트 사용)
// A채널이 변할 때 ISR이 호출되고, B채널과 비교해 방향(+/-)을 결정한다.
// A채널과 B채널은 하나의 인코더 센서에서 나오는 두 신호이다.
// 두 개의 바퀴를 따로 측정하는 것이 아님에 주의.
// =============================================================================
#define ENCODER_A 18   // 쿼드러처 인코더 채널 A (인터럽트 트리거)
#define ENCODER_B 19   // 쿼드러처 인코더 채널 B (방향 판별용)

// =============================================================================
// 동작 모드값 (Mode_val 변수에 할당되는 정수 코드)
// RC 수신기의 모드 채널 PWM 폭으로 결정된다.
// =============================================================================
#define BREAK_MODE   200   // 두 채널 모두 LOW → 정지/브레이크
#define MANUAL_MODE 1400   // MANUAL_MODE_PIN > 1600 us → 수동 RC 조종
#define AUTO_MODE   1700   // AUTO_MODE_PIN   > 1600 us → 자율주행 (ROS2 명령 수신)

// =============================================================================
// 조향각 POT(포텐셔미터) 보정값
// POT는 조향 축에 물리적으로 연결되어 실제 타이어 조향각을 측정한다.
// analogRead 결과(0~1023)를 각도(도°)로 변환할 때 쓰이는 캘리브레이션 값.
// POT_MIN/MAX는 실제 하드웨어 장착 상태에서 측정한 ADC 한계값이다.
// =============================================================================
#define POT_MAX          900   // 최대 조향각(+MAX_STEER_TIRE_DEG)에서의 ADC값
#define POT_MIN           10   // 최소 조향각(-MAX_STEER_TIRE_DEG)에서의 ADC값
#define MAX_STEER_TIRE_DEG 24  // 타이어 최대 조향각 (도°). 기구적 한계값

// =============================================================================
// 조향 PID 제어 파라미터
// POT로 읽은 실제 각도(sense)를 목표 각도(ref)로 추종하는 위치 제어 루프.
// =============================================================================
#define KP           0.3     // 비례 게인: 오차에 즉각 반응. 너무 크면 진동
#define KI           0.0005  // 적분 게인: 정상상태 오차 제거. 너무 크면 과적분
#define KD           0.004   // 미분 게인: 오차 변화율에 반응, 과도응답 감쇠
#define PID_DEADBAND 0.07    // PID 출력의 불감대. 이 이하는 0으로 처리(모터 진동 방지)

// 조향 PWM 게인: PID 출력(0~1)을 실제 모터 PWM(0~255)으로 스케일링할 때 사용
// 1.0이면 PID=1.0 → PWM=255(최대). 0.80이면 실질 최대 출력을 80%로 제한
#define STEER_PWM_GAIN       0.80

// 조향 각속도 계산용 저역통과필터 계수 (0~1)
// 1에 가까울수록 노이즈에 민감하게 반응, 0에 가까울수록 부드럽지만 지연 증가
#define STEER_RATE_LPF_ALPHA 0.25

// =============================================================================
// RC 수신기 PWM 중립/범위 정의 (마이크로초 단위)
// 일반적인 RC 서보/ESC 규격: 1000~2000 us, 중립 1500 us
// =============================================================================
#define ACCEL_CENTER_US  1500  // 스로틀 중립 (정지)
#define STEER_CENTER_US  1500  // 조향 중립 (직진)
#define ACCEL_DB_US        50  // 스로틀 불감대 (±50 us 이내는 정지로 처리)
#define STEER_DB_US        25  // 조향 불감대
#define ACCEL_FWD_MAX_US 1804  // 스로틀 최대 전진 PWM 폭
#define ACCEL_REV_MIN_US 1104  // 스로틀 최대 후진 PWM 폭

// RC 입력 정규화 후 불감대 (0.0~1.0 기준)
#define RC_THROTTLE_DB_NORM 0.03
#define RC_STEER_DB_NORM    0.1
#define STEER_DEADBAND_DEG  1.0  // 조향각 deadband (도°): 이 이하는 0으로 처리

// =============================================================================
// 바퀴/엔코더 파라미터 (odometry 속도 계산에 사용)
//
// ※ 실제 하드웨어 스펙에 맞게 반드시 수정할 것!
//
// ENCODER_PPR: 엔코더 분해능 (Pulses Per Revolution)
//   → 모터/엔코더 데이터시트에서 확인
//   → 쿼드러처이므로 실제 카운트는 PPR×4가 되기도 함 (ISR 방식에 따라 다름)
//   → 현재 ISR은 ENCODER_A CHANGE(상승+하강)만 카운트하므로 PPR×2에 해당
//
// WHEEL_RADIUS_M: 타이어 반지름 (미터)
//   → 직접 측정 또는 카탈로그 값 사용
// =============================================================================
#define ENCODER_PPR     360    // 엔코더 1회전당 펄스 수 (실제값으로 수정 필요)
#define WHEEL_RADIUS_M  0.135f  // 타이어 반지름 (m) (실제값으로 수정 필요)

// =============================================================================
// 구동 모터 핀 (모터드라이버 2채널)
// DIR1/DIR2는 반대 극성으로 설정: 두 모터가 물리적으로 반전 장착되어 있어
// 동일 방향 회전을 위해 반대 DIR 신호가 필요하다.
//   전진: DIR1=HIGH, DIR2=LOW,  PWM1=PWM2=in
//   후진: DIR1=LOW,  DIR2=HIGH, PWM1=PWM2=in
// DIR3/PWM3은 조향 모터 전용 채널
// =============================================================================
int DIR1 = 10, PWM1 = 11;  // 구동 모터 채널 A
int DIR2 =  6, PWM2 =  7;  // 구동 모터 채널 B (DIR은 A와 반대 극성)
int DIR3 =  8, PWM3 =  9;  // 조향 모터 채널
int POTPin = A0;            // 조향각 포텐셔미터 아날로그 입력

#define MIN_DRIVE_PWM   0    // 구동 모터 최소 PWM (0: 완전 정지)
#define MAX_DRIVE_PWM 250    // 구동 모터 최대 PWM (0~255 범위에서 상한 설정)

// =============================================================================
// 시리얼 수신 버퍼 (ROS2 → 아두이노 명령 파싱용)
// =============================================================================
#define SERIAL_BUFFER_SIZE 48
char serialBuffer[SERIAL_BUFFER_SIZE];
size_t bufferIndex = 0;

// =============================================================================
// AUTO 모드에서 ROS2로부터 수신한 명령값 저장 변수
// "타임아웃(500ms) 이내에 수신된 명령만 유효"로 처리해 통신 끊김 시 자동 정지
// =============================================================================
float throttle_cmd   = 0.0f;   // ROS2에서 수신한 쓰로틀 명령 (-1.0 ~ +1.0)
float steer_auto_deg = 0.0f;   // ROS2에서 수신한 조향 목표각 (도°, SA 명령)
bool  throttleFresh  = false;  // 쓰로틀 명령 수신 여부
bool  steerFresh     = false;  // 조향 명령 수신 여부
unsigned long lastThrottleMs = 0;  // 마지막 쓰로틀 명령 수신 시각
unsigned long lastSteerMs    = 0;  // 마지막 조향 명령 수신 시각

// 명령 타임아웃: 마지막 수신 후 이 시간(ms)이 지나면 명령을 무효 처리
#define THROTTLE_TIMEOUT_MS 500
#define STEER_TIMEOUT_MS    500

// RC 펄스 유효 범위 (us). 이 범위를 벗어난 펄스는 노이즈로 간주해 무시
#define PULSE_MIN  500
#define PULSE_MAX 2500

// 구동 모터 시동 최소 신호: 이 값 미만의 명령은 0으로 처리 (모터 떨림 방지)
#define SIGNAL_THRESHOLD 0.05

// =============================================================================
// ISR에서 기록하는 RC 펄스 타임스탬프 (마지막 수신 시각)
// 오래된 신호 감지(timeout)에 사용
// =============================================================================
volatile uint32_t steer_last_us  = 0;
volatile uint32_t manual_last_us = 0;
volatile uint32_t auto_last_us   = 0;

// =============================================================================
// ISR(인터럽트 서비스 루틴)에서 갱신되는 volatile 변수
// loop()에서 읽을 때는 반드시 ATOMIC_BLOCK으로 원자적 접근 필요
// =============================================================================
volatile long     encoderCount  = 0;      // 누적 엔코더 펄스 수 (정방향+, 역방향-)
volatile uint32_t steer_rise_us = 0, accel_rise_us  = 0;
volatile uint32_t manual_rise_us= 0, auto_rise_us   = 0;
volatile uint16_t Steering_us   = 1500, Accel_us = 1500;
volatile uint16_t Manual_us     = 1000,  Auto_us = 1000;
volatile uint32_t accel_last_us = 0;


// =============================================================================
// 조향 모터 출력 함수
// u: PID 출력값 (-1.0 ~ +1.0). 양수=한쪽 방향, 음수=반대 방향
// PID_DEADBAND 이하의 미세 출력은 모터 진동 방지를 위해 0으로 처리
// STEER_PWM_GAIN으로 최대 출력을 제한해 과격한 조향을 방지
// =============================================================================
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
    analogWrite(PWM3, pwm_val * 0.6);
  } else {
    digitalWrite(DIR3, LOW);
    analogWrite(PWM3, pwm_val * 0.6);
  }
}


// =============================================================================
// 시리얼 수신 파서 (ROS2 → 아두이노 방향)
//
// 지원 명령어:
//   "TH <float>\n"  : 쓰로틀 명령. 예) "TH 0.5\n" → throttle_cmd = 0.5
//   "SA <float>\n"  : 조향 목표각 명령 (도°). 예) "SA -10.0\n" → 우회전 10°
//
// 주의: "PS"는 아두이노→ROS2 방향 출력 키워드이므로 여기서 파싱하지 않는다.
//       "SA"(명령) vs "PS"(센서 출력) 혼동 주의.
// =============================================================================
void parseSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n') {
      serialBuffer[bufferIndex] = '\0';

      // --- 쓰로틀 명령 파싱 (TH 또는 th) ---
      if (strncmp(serialBuffer, "TH", 2) == 0 || strncmp(serialBuffer, "th", 2) == 0) {
        char *p = serialBuffer + 2;
        while (*p == ' ' || *p == '\t') ++p;
        float v = atof(p);
        if (v >  1.0f) v =  1.0f;
        if (v < -1.0f) v = -1.0f;
        throttle_cmd   = v;
        throttleFresh  = true;
        lastThrottleMs = millis();
      }
      // --- 조향 목표각 명령 파싱 (SA 또는 sa) ---
      // SA = Steering Angle command (MPPI → 아두이노 방향)
      // 이 값은 PID의 목표값(ref)으로 사용되며, 실제 측정값(PS)과 다르다.
      else if (strncmp(serialBuffer, "SA", 2) == 0 || strncmp(serialBuffer, "sa", 2) == 0) {
        char *p = serialBuffer + 2;
        while (*p == ' ' || *p == '\t') ++p;
        float a = atof(p);
        if (a >  MAX_STEER_TIRE_DEG) a =  MAX_STEER_TIRE_DEG;
        if (a < -MAX_STEER_TIRE_DEG) a = -MAX_STEER_TIRE_DEG;
        steer_auto_deg = a;
        steerFresh     = true;
        lastSteerMs    = millis();
      }
      bufferIndex = 0;
    }
    else if (c != '\r') {
      if (bufferIndex < SERIAL_BUFFER_SIZE - 1) {
        serialBuffer[bufferIndex++] = c;
      } else {
        bufferIndex = 0; // 버퍼 오버플로 방지: 버퍼 초기화
      }
    }
  }
}


// =============================================================================
// 선형 보간 (범위 변환)
// x를 [in_min, in_max] 범위에서 [out_min, out_max] 범위로 선형 변환
// =============================================================================
static inline float Mapping(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


// =============================================================================
// 불감대 적용 (deadband)
// |x| < band 이면 0 반환, 그 외 x 그대로 반환
// 센서 노이즈나 미세 진동으로 인한 작은 신호를 제거할 때 사용
// =============================================================================
static inline double applyDeadband(double x, double band) {
  return (fabs(x) < band) ? 0.0 : x;
}


// =============================================================================
// 구동 모터 제어 함수 (AUTO 모드용)
// driveWithDeadtime: 방향 전환 시 deadtime을 두어 모터 드라이버 파손을 방지
//
// cmd: 구동 명령 (-1.0~+1.0). 양수=전진, 음수=후진, 0=정지
// SIGNAL_THRESHOLD 미만은 정지로 처리 (모터 떨림 방지)
// 방향이 바뀔 때 DIR_DEADTIME_US 동안 PWM을 0으로 유지 (H-bridge 보호)
// =============================================================================
const unsigned int DIR_DEADTIME_US = 200;  // 방향 전환 시 대기 시간 (us)
int last_dir_sign = 0;                     // 이전 방향 기억 (방향 전환 감지용)

void driveWithDeadtime(float cmd) {
  cmd = constrain(cmd, -1.0f, 1.0f);

  float mag      = fabs(cmd);
  int   dir_sign = (cmd > 0.0f) ? +1 : (cmd < 0.0f ? -1 : 0);

  // 신호가 없거나 정지 명령이면 모터 출력 0
  if (mag < SIGNAL_THRESHOLD || dir_sign == 0) {
    analogWrite(PWM1, 0);
    analogWrite(PWM2, 0);
    last_dir_sign = 0;
    return;
  }

  // 방향 전환 감지: 이전 방향과 다르면 deadtime 후 방향 전환
  if (last_dir_sign != 0 && dir_sign != last_dir_sign) {
    analogWrite(PWM1, 0);
    analogWrite(PWM2, 0);
    delayMicroseconds(DIR_DEADTIME_US);
  }

  // 전진: DIR1=HIGH, DIR2=LOW (두 모터가 반전 장착되어 반대 DIR이 필요)
  if (dir_sign > 0) {
    digitalWrite(DIR1, HIGH);
    digitalWrite(DIR2, LOW);
  } else {
    // 후진: DIR1=LOW, DIR2=HIGH
    digitalWrite(DIR1, LOW);
    digitalWrite(DIR2, HIGH);
  }

  // 명령 크기(0~1)를 PWM 범위로 변환 후 동일하게 두 채널에 출력
  int in = (int)Mapping(mag, 0.0f, 1.0f, (float)MIN_DRIVE_PWM, (float)MAX_DRIVE_PWM);
  analogWrite(PWM1, in);
  analogWrite(PWM2, in);

  last_dir_sign = dir_sign;
}


// =============================================================================
// RC 수신기 PWM 펄스 측정 ISR (인터럽트 서비스 루틴)
//
// 동작 원리:
//   - RC 수신기는 각 채널마다 1000~2000 us 폭의 HIGH 펄스를 주기적으로 출력
//   - CHANGE 인터럽트: 핀 상태가 바뀔 때마다 ISR 호출
//   - HIGH가 되는 순간 타임스탬프 기록 → LOW가 되는 순간 폭(us) 계산
//   - PULSE_MIN/MAX 범위 내 값만 유효 (노이즈 필터링)
// =============================================================================
void SteeringPulseInt() {
  uint32_t now = micros();
  if (digitalRead(STEERING_PULSE_PIN) == HIGH) {
    steer_rise_us = now;  // 상승 엣지: 타임스탬프 저장
  } else {
    uint32_t w = now - steer_rise_us;  // 하강 엣지: 펄스 폭 계산
    if (w >= PULSE_MIN && w <= PULSE_MAX) {
      Steering_us   = (uint16_t)w;
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
      Accel_us      = (uint16_t)w;
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
      Manual_us      = (uint16_t)w;
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
      Auto_us      = (uint16_t)w;
      auto_last_us = now;
    }
  }
}


// =============================================================================
// 쿼드러처 엔코더 ISR
//
// ENCODER_A의 CHANGE(상승+하강 모두)에서 호출된다.
// ENCODER_A와 ENCODER_B를 비교해 회전 방향을 판별:
//   A == B → 정방향 (+1)
//   A != B → 역방향 (-1)
//
// 이는 하나의 엔코더 센서에서 나오는 두 채널(A, B)을 이용한 방향 감지이며,
// 두 개의 바퀴를 따로 측정하는 것이 아니다.
// =============================================================================
void encoderISR() {
  if (digitalRead(ENCODER_A) == digitalRead(ENCODER_B)) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}


// =============================================================================
// 조향 위치 PID 제어기
//
// ref   : 목표 조향각 (도°)
// sense : POT으로 측정한 현재 조향각 (도°)
// dt_us : 루프 주기 (마이크로초)
//
// 반환값: 모터 출력 (-1.0 ~ +1.0). Steer() 함수에 전달해 모터를 구동
// 주의: static 변수 사용으로 호출 간 상태(적분, 이전 오차)를 유지함
// =============================================================================
double PID(double ref, double sense, unsigned long dt_us) {
  static double prev_err  = 0.0;
  static double integral  = 0.0;

  double dt_s = dt_us * 1.0e-6;
  if (dt_s <= 0.0) dt_s = 1e-6;  // 0 나눗셈 방지

  double err = ref - sense;

  integral += err * dt_s;  // 적분 누적 (I 항)

  double P = KP * err;
  double I = KI * integral;
  double D = KD * (err - prev_err) / dt_s;  // 미분 (D 항)

  prev_err = err;
  return P + I + D;
}


// =============================================================================
// 전체 모터 정지
// 구동 모터: PWM=0으로 브레이크
// 조향 모터: PWM=0, DIR=LOW로 프리휠(free-wheel) 상태
// =============================================================================
void StopMotor() {
  digitalWrite(DIR1, HIGH);
  analogWrite(PWM1, 0);
  digitalWrite(DIR2, HIGH);
  analogWrite(PWM2, 0);
  digitalWrite(DIR3, LOW);
  analogWrite(PWM3, 0);
}


// =============================================================================
// 수동 전진/후진 함수 (MANUAL 모드 전용)
// throttle: 0.0 ~ 1.0 (크기만 받음. 방향은 함수 자체가 결정)
// =============================================================================
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


// =============================================================================
// 시작 시 조향 중립 복귀 (setup에서 1회 실행)
// POT 피드백을 보면서 조향각이 CENTER_DEG(0°) ±TOL_DEG 이내가 될 때까지
// 조향 모터를 구동한다. TIMEOUT_MS 초과 시 강제 종료.
// =============================================================================
void CenterSteeringOnce() {
  const double CENTER_DEG   = 0.0;   // 목표 중립 각도 (도°)
  const double TOL_DEG      = 3.0;   // 허용 오차 (도°)
  const unsigned long TIMEOUT_MS = 1200;

  unsigned long t0 = millis();

  while (millis() - t0 < TIMEOUT_MS) {
    int    pot = analogRead(POTPin);
    double deg = Mapping(pot, POT_MIN, POT_MAX, +MAX_STEER_TIRE_DEG, -MAX_STEER_TIRE_DEG);

    double err = CENTER_DEG - deg;

    if (fabs(err) <= TOL_DEG) {
      // 목표 범위 내 도달 → 정지
      digitalWrite(DIR3, LOW);
      analogWrite(PWM3, 0);
      break;
    }

    // 오차 비례 출력 (부드럽게 중립으로 이동)
    double u = constrain(err / MAX_STEER_TIRE_DEG, -1.0, 1.0);
    u = constrain(u * 0.6, -0.6, 0.6);
    if (fabs(u) < 0.12) u = (u > 0) ? 0.12 : -0.12;  // 최소 출력 보장
    Steer(u);
    delay(10);
  }

  Steer(0.0);  // 종료 시 조향 정지
}


// =============================================================================
// setup: 1회 초기화
// =============================================================================
void setup() {
  Serial.begin(57600);

  // RC 수신기 PWM 인터럽트 등록 (CHANGE: 상승+하강 엣지 모두 감지)
  pinMode(STEERING_PULSE_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(STEERING_PULSE_PIN), SteeringPulseInt, CHANGE);

  pinMode(ACCEL_PULSE_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ACCEL_PULSE_PIN), AccelPulseInt, CHANGE);

  pinMode(MANUAL_MODE_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(MANUAL_MODE_PIN), ManualPulseInt, CHANGE);

  pinMode(AUTO_MODE_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(AUTO_MODE_PIN), AutoPulseInt, CHANGE);

  // 쿼드러처 엔코더 인터럽트 등록 (A채널 CHANGE → 방향은 B채널로 판별)
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);

  pinMode(POTPin, INPUT);

  // 모터 드라이버 핀 출력 설정
  pinMode(DIR1, OUTPUT); pinMode(PWM1, OUTPUT);
  pinMode(DIR2, OUTPUT); pinMode(PWM2, OUTPUT);
  pinMode(DIR3, OUTPUT); pinMode(PWM3, OUTPUT);

  StopMotor();
  CenterSteeringOnce();  // 조향 중립 복귀 후 메인 루프 시작
}


// =============================================================================
// loop: 메인 제어 루프 (~10ms 주기)
// =============================================================================
void loop() {
  delay(10);  // 루프 최소 주기 보장 (10ms)

  // --- 1. 시리얼 수신 파싱 (ROS2로부터 TH/SA 명령 수신) ---
  parseSerial();

  // --- 2. 루프 주기(dt) 계산 ---
  static unsigned long prev_t_us = 0;
  unsigned long t_us  = micros();
  unsigned long dt_us = (prev_t_us == 0) ? 1000UL : (t_us - prev_t_us);
  prev_t_us = t_us;

  // --- 3. ISR 변수 원자적 읽기 (인터럽트와의 race condition 방지) ---
  long Steering_us_local, Accel_us_local, Manual_us_local, Auto_us_local, encoder_local;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    Steering_us_local = Steering_us;
    Accel_us_local    = Accel_us;
    Manual_us_local   = Manual_us;
    Auto_us_local     = Auto_us;
    encoder_local     = encoderCount;
  }

  // --- 4. 동작 모드 결정 (RC 수신기 채널 폭으로 판단) ---
  int Mode_val = BREAK_MODE;
  if      (Manual_us_local > 1600) Mode_val = MANUAL_MODE;
  else if (Auto_us_local   > 1600) Mode_val = AUTO_MODE;

  // 스로틀 신호 30ms 이상 없으면 중립(정지) 처리
  if (micros() - accel_last_us > 30000) Accel_us_local = ACCEL_CENTER_US;

  // --- 5. RC 스로틀 정규화 (-1.0 ~ +1.0) ---
  float Throttle_input = 0.0f;
  if (Accel_us_local > ACCEL_CENTER_US + ACCEL_DB_US)
    Throttle_input = (float)(Accel_us_local - ACCEL_CENTER_US) / (ACCEL_FWD_MAX_US - ACCEL_CENTER_US);
  else if (Accel_us_local < ACCEL_CENTER_US - ACCEL_DB_US)
    Throttle_input = (float)(Accel_us_local - ACCEL_CENTER_US) / (ACCEL_CENTER_US - ACCEL_REV_MIN_US);
  Throttle_input = constrain(Throttle_input, -1.0, 1.0);

  // --- 6. RC 조향 정규화 및 목표각 변환 ---
  float  Steer_rc         = Mapping(Steering_us_local, 1280, 1792, -1.0, 1.0);
  Steer_rc                = applyDeadband(Steer_rc, RC_STEER_DB_NORM);
  double ref_steer_deg_rc = Mapping(Steer_rc, -1.0, 1.0, +MAX_STEER_TIRE_DEG, -MAX_STEER_TIRE_DEG);

  // --- 7. POT으로 실제 조향각 측정 ---
  // raw_deg: deadband 미적용 값 → VX 보정 및 PS 출력에 사용 (deadband 적용 시 작은 각도가 사라져 오차 발생)
  // deg    : deadband 적용값  → PID 제어에 사용 (미세 떨림 제거)
  int    POTval  = analogRead(POTPin);
  double raw_deg = Mapping(POTval, POT_MIN, POT_MAX, +MAX_STEER_TIRE_DEG, -MAX_STEER_TIRE_DEG);
  double deg     = applyDeadband(raw_deg, STEER_DEADBAND_DEG);

  // --- 8. 조향 각속도 계산 (저역통과필터 적용) ---
  static bool   steer_rate_init = false;
  static double prev_raw_deg    = 0.0;
  static double steer_rate_dps  = 0.0;
  double dt_s = dt_us * 1.0e-6;
  if (dt_s <= 0.0) dt_s = 1.0e-6;

  if (!steer_rate_init) {
    steer_rate_init = true;
    prev_raw_deg    = raw_deg;
    steer_rate_dps  = 0.0;
  } else {
    double rate_raw_dps = (raw_deg - prev_raw_deg) / dt_s;
    steer_rate_dps += STEER_RATE_LPF_ALPHA * (rate_raw_dps - steer_rate_dps);
    prev_raw_deg = raw_deg;
  }

  // --- 9. 명령 유효성 확인 (타임아웃 체크) ---
  bool sa_ok = steerFresh    && (millis() - lastSteerMs    <= STEER_TIMEOUT_MS);
  bool th_ok = throttleFresh && (millis() - lastThrottleMs <= THROTTLE_TIMEOUT_MS);

  // --- 10. 조향 PID 계산 (모드별로 1회만 실행, 중복 적분 방지) ---
  double u_rc   = 0.0;
  double u_auto = 0.0;

  if (Mode_val == AUTO_MODE && sa_ok) {
    // AUTO: MPPI가 보낸 SA(목표각)를 POT(실제각)으로 추종
    u_auto = PID((double)steer_auto_deg, deg, dt_us);
    u_auto = applyDeadband(u_auto, PID_DEADBAND);
    u_auto = constrain(u_auto, -1.0, 1.0);
  } else {
    // MANUAL 또는 SA 타임아웃: RC 조종기 조향을 POT으로 추종
    u_rc = PID(ref_steer_deg_rc, deg, dt_us);
    u_rc = applyDeadband(u_rc, PID_DEADBAND);
    u_rc = constrain(u_rc, -1.0, 1.0);
  }

  double u_used = (Mode_val == AUTO_MODE && sa_ok) ? u_auto : u_rc;

  // --- 11. 모드별 모터 구동 ---
  if (Mode_val == BREAK_MODE) {
    StopMotor();
  }
  else if (Mode_val == MANUAL_MODE) {
    if      (Throttle_input >  0.05f) MoveForward (  Throttle_input  * 0.6f);
    else if (Throttle_input < -0.05f) MoveBackward((-Throttle_input) * 0.6f);
    else { analogWrite(PWM1, 0); analogWrite(PWM2, 0); }
    Steer(u_rc);
  }
  else {  // AUTO_MODE
    // 타임아웃이면 0을 넘겨 driveWithDeadtime이 정지 처리
    float th = th_ok ? throttle_cmd : 0.0f;
    driveWithDeadtime(th);
    if (sa_ok) Steer(u_auto);
    else       Steer(u_rc);
  }

  // ==========================================================================
  // 12. 시리얼 텔레메트리 출력 (100ms마다)
  //
  // ROS2(ros2_sensor.py)가 이 출력을 파싱하여 /wheel/odom을 구성한다.
  //
  // [odometry 관련 핵심 출력]
  //   VX  : 전륜 휠 엔코더 선속도 (m/s)
  //         = (d_encoder / ENCODER_PPR) × (2π × WHEEL_RADIUS_M) / dt
  //         ※ 전륜에 달린 값이므로 조향 시 후륜축 속도와 다를 수 있음
  //
  //   PS  : POT Steering — POT으로 측정한 실제 조향각 (도°)
  //         ROS2에서 v_rear = VX × cos(PS × π/180) 로 후륜축 속도 보정에 사용
  //         ※ SA(MPPI 명령값)와 달리 물리적으로 측정된 실제값임
  //
  // [디버그용 출력]
  //   MODE, Tgt, Cur, dDeg/s, PID, POT, ENC, dENC, TH, SA
  // ==========================================================================
  static unsigned long lp = 0;
  unsigned long now_ms = millis();

  if (now_ms - lp > 100) {
    // 실제 경과 시간 계산 (100ms보다 약간 길 수 있으므로 정확한 dt 사용)
    float dt_odom_s = (float)(now_ms - lp) / 1000.0f;
    lp = now_ms;

    static long prev_encoder = 0;
    long d_encoder = encoder_local - prev_encoder;
    prev_encoder   = encoder_local;

    // 휠 선속도 계산 (m/s)
    // 공식: v = (펄스 변화량 / 1회전당 펄스수) × 바퀴 둘레 / 경과 시간
    //      바퀴 둘레 = 2 × π × WHEEL_RADIUS_M
    float v_wheel_ms = (float)d_encoder
                       * (2.0f * PI * WHEEL_RADIUS_M)
                       / ((float)ENCODER_PPR * dt_odom_s);

    // --- 디버그 출력 ---
    Serial.print("MODE:");    Serial.print(Mode_val);
    Serial.print(" | Tgt:");  Serial.print(ref_steer_deg_rc, 1);
    Serial.print(" | Cur:");  Serial.print(deg, 1);
    Serial.print(" | dDeg/s:"); Serial.print(steer_rate_dps, 1);
    Serial.print(" | PID:");  Serial.print(u_used, 2);
    Serial.print(" | POT:");  Serial.print(POTval);
    Serial.print(" | ENC:");  Serial.print(encoder_local);
    Serial.print(" | dENC:"); Serial.print(d_encoder);
    Serial.print(" | TH:");   Serial.print(throttle_cmd, 2);
    Serial.print(" ok:");     Serial.print((int)th_ok);
    Serial.print(" | SA:");   Serial.print(steer_auto_deg, 1);
    Serial.print(" ok:");     Serial.print((int)sa_ok);

    // --- odometry용 핵심 출력 (ROS2가 파싱) ---
    // VX: 전륜 엔코더 선속도 (m/s) — 조향 보정 전 raw 값
    Serial.print(" | VX:");   Serial.print(v_wheel_ms, 4);
    // PS: POT Steering — 실제 측정 조향각 (도°)
    Serial.print(" | PS:");   Serial.println(raw_deg, 2);
  }
}
