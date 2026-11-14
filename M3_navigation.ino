/*
  M3 - Sensing and Navigation Engineer
  Project 1: Autonomous SLAM-Inspired Occupancy Mapping Rover (Tinkercad)

  Wiring (Tinkercad):
    Micro servo signal -> D9   (servo power from external 5V/9V rail, common GND)
    HC-SR04 TRIG       -> D7
    HC-SR04 ECHO       -> D2   (INT0, needed for the interrupt)
    HC-SR04 VCC/GND    -> 5V / GND

  Servo angle convention (assumption, tell team):
    90 deg = straight ahead, 0 deg = right, 180 deg = left

  Features:
    - Interrupt-driven ultrasonic ping (no pulseIn)
    - 15-degree scan, 13 positions (0..180), all timing with millis()
    - Edge detection (free <-> blocked) between neighbouring angles
    - FSM: IDLE, SCANNING, CALCULATING, MOVING, EMERGENCY_STOP
    - Motor stubs (replace with M2's real functions when merged)
    - SIM mode: virtual scan scenarios, because Tinkercad's HC-SR04 gives
      one distance regardless of servo angle

  Serial Monitor (9600 baud) commands:
    x     toggle SIM mode on/off
    0..4  choose SIM scenario
          0 open path, 1 wall ahead + left open, 2 dead end,
          3 narrow exit on far left, 4 obstacle too close (emergency)
*/

#include <Servo.h>

// ---------- Pins ----------
const uint8_t SERVO_PIN = 9;
const uint8_t TRIG_PIN  = 7;
const uint8_t ECHO_PIN  = 2;

// ---------- Scan settings ----------
const uint8_t NUM_POS   = 13;   // 0,15,...,180
const uint8_t STEP_DEG  = 15;
const uint8_t FRONT_IDX = 6;    // index of 90 degrees

// ---------- Distance thresholds (cm) ----------
const uint16_t BLOCK_CM     = 20;   // below this an angle is "blocked"
const uint16_t SAFE_CM      = 30;   // front must be at least this to go straight
const uint16_t EMERGENCY_CM = 10;   // below this -> EMERGENCY_STOP
const uint16_t MAX_CM       = 200;  // cap / no-echo value

// ---------- Timing (ms) ----------
const unsigned long SETTLE_MS          = 250;   // servo settle between steps
const unsigned long SETTLE_FIRST_MS    = 600;   // servo returning to 0
const unsigned long PING_TIMEOUT_MS    = 30;
const unsigned long IDLE_MS            = 1000;
const unsigned long FORWARD_MS         = 1000;  // one move segment
const unsigned long MS_PER_DEG         = 10;    // pivot time per degree (M2 calibrates)
const unsigned long CHECK_INTERVAL_MS  = 80;    // front check while moving
const unsigned long SERVO_RETURN_MS    = 300;   // ignore checks while servo returns to 90
const unsigned long EMERGENCY_BACK_MS  = 400;
const unsigned long EMERGENCY_WAIT_MS  = 500;

// ---------- Motor stubs: REPLACE with M2's functions ----------
void stop()       { Serial.println(F("[motor] stop")); }
void forward()    { Serial.println(F("[motor] forward")); }
void reverse()    { Serial.println(F("[motor] reverse")); }
void pivotLeft()  { Serial.println(F("[motor] pivotLeft")); }
void pivotRight() { Serial.println(F("[motor] pivotRight")); }

// ---------- Simulated scenarios (13 distances, 0..180 deg) ----------
const uint16_t SCENARIO[5][13] = {
  {200,200,200,200,200,200,200,200,200,200,200,200,200}, // 0 open path
  { 60, 60, 60, 60, 60, 15, 15, 15,150,150,150,150,150}, // 1 wall ahead, left open
  { 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12}, // 2 dead end
  { 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 40}, // 3 narrow exit far left
  {100,100,100,100,100, 50,  8, 50,100,100,100,100,100}  // 4 too close
};
bool simMode = false;
uint8_t scenario = 0;

// ---------- Interrupt ping ----------
volatile unsigned long echoStart = 0;
volatile unsigned long echoEnd   = 0;
volatile bool echoDone = false;
unsigned long pingStartedMs = 0;
uint16_t lastDistance = MAX_CM;

void echoISR() {
  if (digitalRead(ECHO_PIN) == HIGH) {
    echoStart = micros();
  } else {
    echoEnd = micros();
    echoDone = true;
  }
}

void startPing() {
  noInterrupts();
  echoDone = false;
  echoStart = 0;
  echoEnd = 0;
  interrupts();
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  pingStartedMs = millis();
}

// returns true when a result is ready (lastDistance updated)
bool pollPing() {
  bool done;
  unsigned long s, e;
  noInterrupts();
  done = echoDone;
  s = echoStart;
  e = echoEnd;
  interrupts();

  if (done) {
    unsigned long cm = (e - s) / 58UL;
    lastDistance = (cm > MAX_CM) ? MAX_CM : (uint16_t)cm;
    return true;
  }
  if (millis() - pingStartedMs >= PING_TIMEOUT_MS) {
    lastDistance = MAX_CM;   // no echo = nothing in range
    return true;
  }
  return false;
}

// ---------- State machine ----------
enum State { IDLE, SCANNING, CALCULATING, MOVING, EMERGENCY_STOP };
const char* STATE_NAMES[] = {"IDLE", "SCANNING", "CALCULATING", "MOVING", "EMERGENCY_STOP"};

State state = IDLE;
unsigned long stateTimer = 0;
uint8_t subPhase = 0;
bool pingActive = false;
unsigned long lastCheck = 0;

Servo scanServo;
uint16_t dist[13];
bool blocked[13];
uint8_t scanStep = 0;

int8_t pivotDir = 0;          // +1 left, -1 right, 0 none
unsigned long pivotMs = 0;

void changeState(State next) {
  state = next;
  stateTimer = millis();
  subPhase = 0;
  pingActive = false;

  Serial.print(F("STATE: "));
  Serial.println(STATE_NAMES[next]);

  switch (next) {
    case IDLE:
      stop();
      break;
    case SCANNING:
      scanStep = 0;
      scanServo.write(0);
      break;
    case CALCULATING:
      break;
    case MOVING:
      scanServo.write(90);
      lastCheck = millis();
      if (pivotMs > 0) {
        if (pivotDir > 0) pivotLeft(); else pivotRight();
        subPhase = 0;            // pivoting
      } else {
        forward();
        subPhase = 1;            // driving
      }
      break;
    case EMERGENCY_STOP:
      stop();
      reverse();
      break;
  }
}

void handleIdle() {
  if (millis() - stateTimer >= IDLE_MS) changeState(SCANNING);
}

void handleScanning() {
  if (subPhase == 0) {                       // waiting for servo to settle
    unsigned long settle = (scanStep == 0) ? SETTLE_FIRST_MS : SETTLE_MS;
    if (millis() - stateTimer >= settle) {
      startPing();
      subPhase = 1;
    }
    return;
  }

  if (!pollPing()) return;                   // waiting for echo interrupt

  uint16_t d = simMode ? SCENARIO[scenario][scanStep] : lastDistance;
  dist[scanStep] = d;
  blocked[scanStep] = (d < BLOCK_CM);

  Serial.print(scanStep * STEP_DEG);         // angle,distance
  Serial.print(',');
  Serial.println(d);

  if (scanStep > 0 && blocked[scanStep] != blocked[scanStep - 1]) {
    Serial.print(F("EDGE at "));
    Serial.print(scanStep * STEP_DEG);
    Serial.println(blocked[scanStep] ? F(" deg: FREE->BLOCKED") : F(" deg: BLOCKED->FREE"));
  }

  scanStep++;
  if (scanStep >= NUM_POS) {
    if (dist[FRONT_IDX] < EMERGENCY_CM) changeState(EMERGENCY_STOP);
    else changeState(CALCULATING);
  } else {
    scanServo.write(scanStep * STEP_DEG);
    stateTimer = millis();
    subPhase = 0;
  }
}

void handleCalculating() {
  bool frontClear = dist[FRONT_IDX - 1] >= SAFE_CM &&
                    dist[FRONT_IDX]     >= SAFE_CM &&
                    dist[FRONT_IDX + 1] >= SAFE_CM;

  if (frontClear) {
    pivotDir = 0;
    pivotMs = 0;
    Serial.println(F("DECISION: front clear, go forward"));
  } else {
    int best = FRONT_IDX;
    uint16_t bestD = 0;
    for (int i = 0; i < NUM_POS; i++) {
      if (dist[i] > bestD ||
          (dist[i] == bestD && abs(i - FRONT_IDX) < abs(best - FRONT_IDX))) {
        best = i;
        bestD = dist[i];
      }
    }

    if (bestD < BLOCK_CM) {                  // everything blocked: dead end
      pivotDir = 1;
      pivotMs = 180UL * MS_PER_DEG;
      Serial.println(F("DECISION: dead end, turn around 180 deg"));
    } else {
      int deg = (best - FRONT_IDX) * STEP_DEG;   // + = left, - = right
      pivotDir = (deg > 0) ? 1 : ((deg < 0) ? -1 : 0);
      pivotMs = (unsigned long)abs(deg) * MS_PER_DEG;
      Serial.print(F("DECISION: best angle "));
      Serial.print(best * STEP_DEG);
      Serial.print(F(" deg ("));
      Serial.print(bestD);
      Serial.print(F(" cm), pivot "));
      Serial.print(abs(deg));
      Serial.println(deg > 0 ? F(" deg left") : F(" deg right"));
    }
  }
  changeState(MOVING);
}

void handleMoving() {
  if (subPhase == 0) {                       // pivoting
    if (millis() - stateTimer >= pivotMs) {
      stop();
      forward();
      subPhase = 1;
      stateTimer = millis();
      lastCheck = millis();
    }
    return;
  }

  // driving forward: watch the front with the interrupt ping
  if (!simMode) {
    if (!pingActive &&
        millis() - stateTimer >= SERVO_RETURN_MS &&
        millis() - lastCheck >= CHECK_INTERVAL_MS) {
      startPing();
      pingActive = true;
    }
    if (pingActive && pollPing()) {
      pingActive = false;
      lastCheck = millis();
      if (lastDistance < EMERGENCY_CM) {
        changeState(EMERGENCY_STOP);
        return;
      }
    }
  }

  if (millis() - stateTimer >= FORWARD_MS) {
    stop();
    changeState(SCANNING);
  }
}

void handleEmergency() {
  if (subPhase == 0) {
    if (millis() - stateTimer >= EMERGENCY_BACK_MS) {
      stop();
      subPhase = 1;
      stateTimer = millis();
    }
  } else if (millis() - stateTimer >= EMERGENCY_WAIT_MS) {
    changeState(SCANNING);
  }
}

void handleSerialCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'x') {
      simMode = !simMode;
      Serial.print(F("SIM mode: "));
      Serial.println(simMode ? F("ON") : F("OFF"));
    } else if (c >= '0' && c <= '4') {
      scenario = c - '0';
      Serial.print(F("SIM scenario: "));
      Serial.println(scenario);
    }
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  scanServo.attach(SERVO_PIN);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoISR, CHANGE);
  Serial.println(F("M3 navigation ready. x = toggle SIM, 0-4 = scenario"));
  changeState(IDLE);
}

void loop() {
  handleSerialCommands();
  switch (state) {
    case IDLE:           handleIdle();       break;
    case SCANNING:       handleScanning();   break;
    case CALCULATING:    handleCalculating();break;
    case MOVING:         handleMoving();     break;
    case EMERGENCY_STOP: handleEmergency();  break;
  }
}
