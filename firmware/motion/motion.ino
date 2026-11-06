// L293D pins (agree with M1 before wiring)
const int ENA = 5, IN1 = 4, IN2 = 7;   // left motor  (ENA = PWM)
const int ENB = 6, IN3 = 8, IN4 = 12;  // right motor (ENB = PWM)

// Calibration constants (measure in Tinkercad, then fill in)
const float MM_PER_SEC_AT_200 = 0;     // forward speed at PWM 200
const float DEG_PER_SEC_PIVOT = 0;     // turn rate at PWM 200
const float RIGHT_TRIM = 1.00;         // drift compensation (e.g. 0.96)

void setMotor(int en, int a, int b, int speed) {
  digitalWrite(a, speed > 0);
  digitalWrite(b, speed < 0);
  analogWrite(en, abs(speed));
}
void drive(int left, int right) {
  setMotor(ENA, IN1, IN2, left);
  setMotor(ENB, IN3, IN4, right * RIGHT_TRIM);
}
void stopRover()      { drive(0, 0); }
void forward(int s)   { drive(s, s); }
void reverse(int s)   { drive(-s, -s); }
void pivotLeft(int s) { drive(0, s); }      // one wheel
void tankLeft(int s)  { drive(-s, s); }     // counter-rotate

// Timed, non-blocking action
unsigned long actionEnd = 0;
bool acting = false;
void startAction(unsigned long ms) { actionEnd = millis() + ms; acting = true; }
void updateMotion() {
  if (acting && millis() >= actionEnd) { stopRover(); acting = false; }
}

void setup() {
  int pins[] = {ENA, IN1, IN2, ENB, IN3, IN4};
  for (int p : pins) pinMode(p, OUTPUT);
}
void loop() { updateMotion(); }