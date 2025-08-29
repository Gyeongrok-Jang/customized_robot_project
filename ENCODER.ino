const byte ENC_LEFT_PIN_A = 2;
const byte ENC_LEFT_PIN_B = 12;

const byte ENC_RIGHT_PIN_A = 3;
const byte ENC_RIGHT_PIN_B = 13;

const int PPR = 960;              
const unsigned long sampleTime = 20; 

volatile long pulseCountLeft = 0;
volatile bool lastAStateLeft = LOW;

volatile long pulseCountRight = 0;
volatile bool lastAStateRight = LOW;

unsigned long lastSampleTime = 0;

double accumulatedAngleLeft = 0.0;
double accumulatedAngleRight = 0.0;

void setup() {
  Serial.begin(9600);

  pinMode(ENC_LEFT_PIN_A, INPUT);
  pinMode(ENC_LEFT_PIN_B, INPUT);
  pinMode(ENC_RIGHT_PIN_A, INPUT);
  pinMode(ENC_RIGHT_PIN_B, INPUT);

  lastAStateLeft = digitalRead(ENC_LEFT_PIN_A);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_PIN_A), countPulseLeft, CHANGE);

  lastAStateRight = digitalRead(ENC_RIGHT_PIN_A);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_PIN_A), countPulseRight, CHANGE);

  lastSampleTime = millis();
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - lastSampleTime >= sampleTime) {
    noInterrupts();
    long pulsesLeft = pulseCountLeft;
    pulseCountLeft = 0;
    long pulsesRight = pulseCountRight;
    pulseCountRight = 0;
    interrupts();

    double deltaTime = (currentTime - lastSampleTime) / 1000.0;

    double revsLeft = (double)pulsesLeft / PPR;
    double angularVelocityLeft = revsLeft * 2 * PI / deltaTime;

    double revsRight = (double)pulsesRight / PPR;
    double angularVelocityRight = revsRight * 2 * PI / deltaTime;

    accumulatedAngleLeft += angularVelocityLeft * deltaTime;
    accumulatedAngleRight += angularVelocityRight * deltaTime;

    double angleLeftDeg = accumulatedAngleLeft * 180.0 / PI;
    double angleRightDeg = accumulatedAngleRight * 180.0 / PI;


    Serial.print("L_Vel:");
    Serial.print(angularVelocityLeft, 3);
    Serial.print(", L_Angle:");
    Serial.print(angleLeftDeg, 2);
    Serial.print(", R_Vel:");
    Serial.print(angularVelocityRight, 3);
    Serial.print(", R_Angle:");
    Serial.println(angleRightDeg, 2);
    
    lastSampleTime = currentTime;
  }
}


void countPulseLeft() {
  bool currentA = digitalRead(ENC_LEFT_PIN_A);
  bool currentB = digitalRead(ENC_LEFT_PIN_B);
  if (lastAStateLeft == LOW && currentA == HIGH) {
    if (currentB == LOW) pulseCountLeft--;
    else pulseCountLeft++;
  }
  lastAStateLeft = currentA;
}

void countPulseRight() {
  bool currentA = digitalRead(ENC_RIGHT_PIN_A);
  bool currentB = digitalRead(ENC_RIGHT_PIN_B);
  if (lastAStateRight == LOW && currentA == HIGH) {
    if (currentB == LOW) pulseCountRight--;
    else pulseCountRight++;
  }
  lastAStateRight = currentA;
}
