#include <PID_v1.h>

// --- 엔코더 핀 ---
const byte ENC_LEFT_PIN_A = 2;
const byte ENC_LEFT_PIN_B = 12;

const byte ENC_RIGHT_PIN_A = 3;
const byte ENC_RIGHT_PIN_B = 13;

// --- 모터 핀 ---
const byte PWM_LEFT = 5;
const byte IN_LEFT_1 = 6;
const byte IN_LEFT_2 = 7;

const byte PWM_RIGHT = 9;
const byte IN_RIGHT_1 = 10;
const byte IN_RIGHT_2 = 11;

// --- 로봇 물리 변수 ---
const double trackWidth = 0.15;    // 로봇 바퀴 사이 간격 [m] (15cm)
const double wheelRadius = 0.03;   // 바퀴 반지름 [m] (3cm)
const int PPR = 960;               // 엔코더 펄스 수

const unsigned long pidSampleTime = 20; // PID 샘플 타임(ms)

// --- PID 변수 및 상태 ---
volatile long encoderCountLeft = 0;
volatile long encoderCountRight = 0;

double pwmOutputLeft = 0;
double pwmOutputRight = 0;

double setpointLeft = 0;      // 목표 펄스 수 (단위 : 펄스/샘플시간)
double setpointRight = 0;

double inputLeftForPID = 0;   // 현재 펄스수(절대값)
double inputRightForPID = 0;

// PID 튜닝값 (필요시 조정)
double Kp = 0.6, Ki = 5, Kd = 0;

PID pidLeft(&inputLeftForPID, &pwmOutputLeft, &setpointLeft, Kp, Ki, Kd, DIRECT);
PID pidRight(&inputRightForPID, &pwmOutputRight, &setpointRight, Kp, Ki, Kd, DIRECT);

// 이전 엔코더 A핀 상태 (방향 검출용)
volatile byte lastEncoderAStateLeft = LOW;
volatile byte lastEncoderAStateRight = LOW;

// 입력 명령 (사용자가 원하는 값, m/s, rad/s)
double linearVelocity = 0.0;    // m/s
double angularVelocity = 0.0;   // rad/s

unsigned long lastPidTime = 0;

void setup() {
  Serial.begin(9600);

  pinMode(IN_LEFT_1, OUTPUT);
  pinMode(IN_LEFT_2, OUTPUT);
  pinMode(PWM_LEFT, OUTPUT);

  pinMode(IN_RIGHT_1, OUTPUT);
  pinMode(IN_RIGHT_2, OUTPUT);
  pinMode(PWM_RIGHT, OUTPUT);

  pinMode(ENC_LEFT_PIN_A, INPUT);
  pinMode(ENC_LEFT_PIN_B, INPUT);
  pinMode(ENC_RIGHT_PIN_A, INPUT);
  pinMode(ENC_RIGHT_PIN_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_PIN_A), updateEncoderLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_PIN_A), updateEncoderRight, CHANGE);

  pidLeft.SetMode(AUTOMATIC);
  pidLeft.SetSampleTime(pidSampleTime);

  pidRight.SetMode(AUTOMATIC);
  pidRight.SetSampleTime(pidSampleTime);

  lastPidTime = millis();
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - lastPidTime >= pidSampleTime) {
    // 1. 목표 각 바퀴 선속도 계산 (m/s)
    double v_left = linearVelocity - angularVelocity * trackWidth / 2.0;
    double v_right = linearVelocity + angularVelocity * trackWidth / 2.0;

    // 2. 바퀴 각속도(rad/s) = 선속도 / 반지름
    double omega_left = v_left / wheelRadius;
    double omega_right = v_right / wheelRadius;

    // 3. 각속도(rad/s) -> 펄스수 변환
    // 1바퀴 회전당 펄스수 = PPR
    // 1초당 펄스수 = omega * (1회전 / 2pi rad) * PPR
    // 샘플타임 동안 펄스수 = 1초당 펄스수 * (샘플시간(s))
    double pulsesPerSecondLeft = omega_left * (PPR / (2.0 * PI));
    double pulsesPerSecondRight = omega_right * (PPR / (2.0 * PI));

    double deltaTime = pidSampleTime / 1000.0; // 샘플 타임 초 단위

    setpointLeft = abs(pulsesPerSecondLeft * deltaTime);
    setpointRight = abs(pulsesPerSecondRight * deltaTime);

    // 4. 현재 펄스 수 읽기 및 절대값으로 PID 입력 설정
    noInterrupts();
    double encoderCountLeftAbs = abs(encoderCountLeft);
    double encoderCountRightAbs = abs(encoderCountRight);
    // 카운트 초기화
    encoderCountLeft = 0;
    encoderCountRight = 0;
    interrupts();

    inputLeftForPID = encoderCountLeftAbs;
    inputRightForPID = encoderCountRightAbs;

    // 5. PID 계산
    pidLeft.Compute();
    pidRight.Compute();

    // 6. PWM 출력값 제한 (0~255)
    int pwmLeft = constrain((int)pwmOutputLeft, 0, 255);
    int pwmRight = constrain((int)pwmOutputRight, 0, 255);

    // 7. 모터 방향 제어 (목표 각속도 부호에 따라)
    if (pulsesPerSecondLeft < 0) moveBackwardLeft(pwmLeft);
    else if (pulsesPerSecondLeft > 0) moveForwardLeft(pwmLeft);
    else stopLeft();

    if (pulsesPerSecondRight < 0) moveBackwardRight(pwmRight);
    else if (pulsesPerSecondRight > 0) moveForwardRight(pwmRight);
    else stopRight();

    // 8. 플로터 출력 (setpoint, input, pwm) - setpoint와 엔코더 입력값에 방향에 따라 음수 붙이기
    Serial.print("setpointLeft:");
    Serial.print((pulsesPerSecondLeft < 0 ? -setpointLeft : setpointLeft), 3);
    Serial.print(",inputLeft:");
    Serial.print((pulsesPerSecondLeft < 0 ? -inputLeftForPID : inputLeftForPID), 3);
    Serial.print(",pwmLeft:");
    Serial.print(pwmLeft);

    Serial.print(",setpointRight:");
    Serial.print((pulsesPerSecondRight < 0 ? -setpointRight : setpointRight), 3);
    Serial.print(",inputRight:");
    Serial.print((pulsesPerSecondRight < 0 ? -inputRightForPID : inputRightForPID), 3);
    Serial.print(",pwmRight:");
    Serial.println(pwmRight);



    lastPidTime = currentTime;
  }
}

// --- 인터럽트 처리 (엔코더 펄스 카운트) ---
void updateEncoderLeft() {
  int currentA = digitalRead(ENC_LEFT_PIN_A);
  int valB = digitalRead(ENC_LEFT_PIN_B);

  if ((lastEncoderAStateLeft == LOW) && (currentA == HIGH)) {
    if (valB == LOW) {
      encoderCountLeft--;
    } else {
      encoderCountLeft++;
    }
  }
  lastEncoderAStateLeft = currentA;
}

void updateEncoderRight() {
  int currentA = digitalRead(ENC_RIGHT_PIN_A);
  int valB = digitalRead(ENC_RIGHT_PIN_B);

  if ((lastEncoderAStateRight == LOW) && (currentA == HIGH)) {
    if (valB == LOW) {
      encoderCountRight--;
    } else {
      encoderCountRight++;
    }
  }
  lastEncoderAStateRight = currentA;
}

// --- 모터 제어 함수 ---
void moveForwardLeft(int pwm) {
  digitalWrite(IN_LEFT_1, HIGH);
  digitalWrite(IN_LEFT_2, LOW);
  analogWrite(PWM_LEFT, pwm);
}

void moveBackwardLeft(int pwm) {
  digitalWrite(IN_LEFT_1, LOW);
  digitalWrite(IN_LEFT_2, HIGH);
  analogWrite(PWM_LEFT, pwm);
}

void stopLeft() {
  digitalWrite(IN_LEFT_1, LOW);
  digitalWrite(IN_LEFT_2, LOW);
  analogWrite(PWM_LEFT, 0);
}

void moveForwardRight(int pwm) {
  digitalWrite(IN_RIGHT_1, LOW);
  digitalWrite(IN_RIGHT_2, HIGH);
  analogWrite(PWM_RIGHT, pwm);
}

void moveBackwardRight(int pwm) {
  digitalWrite(IN_RIGHT_1, HIGH);
  digitalWrite(IN_RIGHT_2, LOW);
  analogWrite(PWM_RIGHT, pwm);
}

void stopRight() {
  digitalWrite(IN_RIGHT_1, LOW);
  digitalWrite(IN_RIGHT_2, LOW);
  analogWrite(PWM_RIGHT, 0);
}
