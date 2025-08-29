#include <PID_v1.h>

// 포트 지정
const byte ENC_LEFT_PIN_A = 2;    // 왼쪽 모터 엔코더 A (인터럽트 핀)
const byte ENC_LEFT_PIN_B = 12;   // 왼쪽 모터 엔코더 B (일반 핀)

const byte ENC_RIGHT_PIN_A = 3;   // 오른쪽 모터 엔코더 A (인터럽트 핀)
const byte ENC_RIGHT_PIN_B = 13;  // 오른쪽 모터 엔코더 B (일반 핀)

const byte PWM_LEFT = 5;
const byte IN_LEFT_1 = 6;
const byte IN_LEFT_2 = 7;

const byte PWM_RIGHT = 9;
const byte IN_RIGHT_1 = 10;
const byte IN_RIGHT_2 = 11;

// 왼쪽 모터 변수
volatile long encoderCountLeft = 0;
double encoderCountLeftAbs = 0;
double pwmOutputLeft = 0;

double targetSpeedLeft = 0;
double setpointLeft = 0;
double inputLeftForPID = 0;

// PID 파라미터
double Kp = 0.6, Ki = 5, Kd = 0;

PID pidLeft(&inputLeftForPID, &pwmOutputLeft, &setpointLeft, Kp, Ki, Kd, DIRECT);

// 오른쪽 모터 변수
volatile long encoderCountRight = 0;
double encoderCountRightAbs = 0;
double pwmOutputRight = 0;

double targetSpeedRight = 0;
double setpointRight = 0;
double inputRightForPID = 0;

PID pidRight(&inputRightForPID, &pwmOutputRight, &setpointRight, Kp, Ki, Kd, DIRECT);

// 엔코더 A핀 이전 상태 저장 (방향 감지용)
volatile byte lastEncoderAStateLeft = LOW;
volatile byte lastEncoderAStateRight = LOW;

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
  pidLeft.SetSampleTime(100);

  pidRight.SetMode(AUTOMATIC);
  pidRight.SetSampleTime(100);
}

void loop() {
  // 여기에서 원하는 속도 지정
  targetSpeedLeft = 0;
  targetSpeedRight = 0;

  // 왼쪽 모터 PID 제어
  setpointLeft = abs(targetSpeedLeft);

  noInterrupts();
  encoderCountLeftAbs = abs(encoderCountLeft);
  interrupts();

  inputLeftForPID = encoderCountLeftAbs;
  pidLeft.Compute();

  noInterrupts();
  encoderCountLeft = 0;
  interrupts();

  int signedCountLeft = (targetSpeedLeft >= 0) ? encoderCountLeftAbs : -encoderCountLeftAbs;

  Serial.print("Left Encoder: ");
  Serial.print(signedCountLeft);
  Serial.print('\t');
  Serial.print("Setpoint: ");
  Serial.print(setpointLeft);
  Serial.print('\t');
  Serial.print("PWM Output: ");
  Serial.println(pwmOutputLeft);

  if (targetSpeedLeft < 0) {
    moveBackwardLeft((int)pwmOutputLeft);
  } else if (targetSpeedLeft > 0) {
    moveForwardLeft((int)pwmOutputLeft);
  } else {
    stopLeft();
  }

  // 오른쪽 모터 PID 제어
  setpointRight = abs(targetSpeedRight);

  noInterrupts();
  encoderCountRightAbs = abs(encoderCountRight);
  interrupts();

  inputRightForPID = encoderCountRightAbs;
  pidRight.Compute();

  noInterrupts();
  encoderCountRight = 0;
  interrupts();

  int signedCountRight = (targetSpeedRight >= 0) ? encoderCountRightAbs : -encoderCountRightAbs;

  Serial.print("Right Encoder: ");
  Serial.print(signedCountRight);
  Serial.print('\t');
  Serial.print("Setpoint: ");
  Serial.print(setpointRight);
  Serial.print('\t');
  Serial.print("PWM Output: ");
  Serial.println(pwmOutputRight);

  if (targetSpeedRight < 0) {
    moveBackwardRight((int)pwmOutputRight);
  } else if (targetSpeedRight > 0) {
    moveForwardRight((int)pwmOutputRight);
  } else {
    stopRight();
  }

  delay(100);
}

// 왼쪽 엔코더 인터럽트 함수 (A핀 변화 시 방향 판별)
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

// 오른쪽 엔코더 인터럽트 함수 (A핀 변화 시 방향 판별)
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

// 왼쪽 모터 제어 함수
void moveForwardLeft(int pwm) {
  digitalWrite(IN_LEFT_1, HIGH);
  digitalWrite(IN_LEFT_2, LOW);
  analogWrite(PWM_LEFT, constrain(pwm, 0, 255));
}

void moveBackwardLeft(int pwm) {
  digitalWrite(IN_LEFT_1, LOW);
  digitalWrite(IN_LEFT_2, HIGH);
  analogWrite(PWM_LEFT, constrain(pwm, 0, 255));
}

void stopLeft() {
  analogWrite(PWM_LEFT, 0);
  digitalWrite(IN_LEFT_1, LOW);
  digitalWrite(IN_LEFT_2, LOW);
}

// 오른쪽 모터 제어 함수
void moveForwardRight(int pwm) {
  digitalWrite(IN_RIGHT_1, HIGH);
  digitalWrite(IN_RIGHT_2, LOW);
  analogWrite(PWM_RIGHT, constrain(pwm, 0, 255));
}

void moveBackwardRight(int pwm) {
  digitalWrite(IN_RIGHT_1, LOW);
  digitalWrite(IN_RIGHT_2, HIGH);
  analogWrite(PWM_RIGHT, constrain(pwm, 0, 255));
}

void stopRight() {
  analogWrite(PWM_RIGHT, 0);
  digitalWrite(IN_RIGHT_1, LOW);
  digitalWrite(IN_RIGHT_2, LOW);
}
