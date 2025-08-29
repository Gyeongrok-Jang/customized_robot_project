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
const double trackWidth = 0.15;    // [m]
const double wheelRadius = 0.03;   // [m]
const int PPR = 960;               // 펄스/회전

const unsigned long pidSampleTime = 100;   // PID 샘플 타임 (ms) 100ms -> 10Hz
const unsigned long serialInterval = 100;  // 시리얼 출력 주기 (ms) 500ms -> 2Hz

// --- PID 변수 및 상태 ---
volatile long encoderCountLeft = 0;
volatile long encoderCountRight = 0;

double pwmOutputLeft = 0;
double pwmOutputRight = 0;

double setpointLeft = 0;      // 목표 펄스 수 (펄스/샘플타임)
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

// 사용자 명령 (m/s, rad/s)
double linearVelocity = 0.0;    
double angularVelocity = 0.0;   

unsigned long lastPidTime = 0;
unsigned long lastSerialTime = 0;

// --- Odometry 변수 ---
double x_pos = 0.0;    // [m]
double y_pos = 0.0;    // [m]
double theta = 0.0;    // [rad]

// 엔코더 각속도 저장 (rad/s)
double leftRadPerSec = 0.0;
double rightRadPerSec = 0.0;

void setup() {
  Serial.begin(4800);

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
  lastSerialTime = millis();
}

void loop() {
  readSerialCommand();

  unsigned long currentTime = millis();

  // PID 제어 주기
  if (currentTime - lastPidTime >= pidSampleTime) {
    double deltaTime = pidSampleTime / 1000.0;

    // 1. 목표 각 바퀴 선속도 계산 (m/s)
    double v_left = linearVelocity - angularVelocity * trackWidth / 2.0;
    double v_right = linearVelocity + angularVelocity * trackWidth / 2.0;

    // 2. 바퀴 각속도(rad/s) = 선속도 / 반지름
    double omega_left = v_left / wheelRadius;
    double omega_right = v_right / wheelRadius;

    // 3. 각속도(rad/s) -> 펄스수 변환 (샘플타임 동안)
    double pulsesPerSecondLeft = omega_left * (PPR / (2.0 * PI));
    double pulsesPerSecondRight = omega_right * (PPR / (2.0 * PI));

    setpointLeft = abs(pulsesPerSecondLeft * deltaTime);
    setpointRight = abs(pulsesPerSecondRight * deltaTime);

    // 4. 현재 펄스 수 읽기 및 초기화 (절대값)
    noInterrupts();
    long encoderCountLeftCopy = encoderCountLeft;
    long encoderCountRightCopy = encoderCountRight;
    encoderCountLeft = 0;
    encoderCountRight = 0;
    interrupts();

    inputLeftForPID = abs(encoderCountLeftCopy);
    inputRightForPID = abs(encoderCountRightCopy);

    // 5. PID 계산
    pidLeft.Compute();
    pidRight.Compute();

    // 6. PWM 출력 제한 (0~255)
    int pwmLeft = constrain((int)pwmOutputLeft, 0, 255);
    int pwmRight = constrain((int)pwmOutputRight, 0, 255);

    // 7. 모터 방향 제어
    if (pulsesPerSecondLeft < 0) moveBackwardLeft(pwmLeft);
    else if (pulsesPerSecondLeft > 0) moveForwardLeft(pwmLeft);
    else stopLeft();

    if (pulsesPerSecondRight < 0) moveBackwardRight(pwmRight);
    else if (pulsesPerSecondRight > 0) moveForwardRight(pwmRight);
    else stopRight();

    // 8. 엔코더 각속도 계산 (rad/s)
    leftRadPerSec = -(encoderCountLeftCopy * 2.0 * PI / PPR) / deltaTime;
    rightRadPerSec = (encoderCountRightCopy * 2.0 * PI / PPR) / deltaTime;

    // --- Odometry 업데이트 ---
    double v_left_real = -leftRadPerSec * wheelRadius;
    double v_right_real = rightRadPerSec * wheelRadius;

    double v = (v_left_real + v_right_real) / 2.0;
    double omega = (v_right_real - v_left_real) / trackWidth;

    double delta_x = v * cos(theta) * deltaTime;
    double delta_y = v * sin(theta) * deltaTime;
    double delta_theta = omega * deltaTime;

    x_pos += delta_x;
    y_pos += delta_y;
    theta += delta_theta;

    if (theta > PI) theta -= 2.0 * PI;
    else if (theta < -PI) theta += 2.0 * PI;

    lastPidTime = currentTime;
  }

  // 시리얼 출력 주기
  if (currentTime - lastSerialTime >= serialInterval) {
    Serial.print("LRS:");
    Serial.print(leftRadPerSec, 4);
    Serial.print(",RRS:");
    Serial.println(rightRadPerSec, 4);

    lastSerialTime = currentTime;
  }
}

void readSerialCommand() {
  while (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    if (cmd.startsWith("L:") && cmd.indexOf(",A:") != -1) {
      int commaIndex = cmd.indexOf(',');
      String linStr = cmd.substring(2, commaIndex);
      String angStr = cmd.substring(commaIndex + 3);

      linearVelocity = linStr.toFloat();
      angularVelocity = angStr.toFloat();
    }
  }
}

// --- 인터럽트 처리 (엔코더 펄스 카운트) ---
void updateEncoderLeft() {
  int currentA = digitalRead(ENC_LEFT_PIN_A);
  int valB = digitalRead(ENC_LEFT_PIN_B);

  if ((lastEncoderAStateLeft == LOW) && (currentA == HIGH)) {
    if (valB == LOW) encoderCountLeft--;
    else encoderCountLeft++;
  }
  lastEncoderAStateLeft = currentA;
}

void updateEncoderRight() {
  int currentA = digitalRead(ENC_RIGHT_PIN_A);
  int valB = digitalRead(ENC_RIGHT_PIN_B);

  if ((lastEncoderAStateRight == LOW) && (currentA == HIGH)) {
    if (valB == LOW) encoderCountRight--;
    else encoderCountRight++;
  }
  lastEncoderAStateRight = currentA;
}

// --- 모터 제어 함수 ---
void moveForwardLeft(int pwm) {
  digitalWrite(IN_LEFT_1, LOW);
  digitalWrite(IN_LEFT_2, HIGH);
  analogWrite(PWM_LEFT, pwm);
}

void moveBackwardLeft(int pwm) {
  digitalWrite(IN_LEFT_1, HIGH);
  digitalWrite(IN_LEFT_2, LOW);
  analogWrite(PWM_LEFT, pwm);
}

void stopLeft() {
  digitalWrite(IN_LEFT_1, LOW);
  digitalWrite(IN_LEFT_2, LOW);
  analogWrite(PWM_LEFT, 0);
}

void moveForwardRight(int pwm) {
  digitalWrite(IN_RIGHT_1, HIGH);
  digitalWrite(IN_RIGHT_2, LOW);
  analogWrite(PWM_RIGHT, pwm);
}

void moveBackwardRight(int pwm) {
  digitalWrite(IN_RIGHT_1, LOW);
  digitalWrite(IN_RIGHT_2, HIGH);
  analogWrite(PWM_RIGHT, pwm);
}

void stopRight() {
  digitalWrite(IN_RIGHT_1, LOW);
  digitalWrite(IN_RIGHT_2, LOW);
  analogWrite(PWM_RIGHT, 0);
}
