const int ENA = 5, IN1 = 4, IN2 = 7;    
const int ENB = 6, IN3 = 8, IN4 = 12;   
const int POT_PIN = A0;                 

const float WHEEL_DIAMETER_MM = 65.0;
const float WHEEL_BASE_MM     = 120.0;  
const float MOTOR_RPM_MAX     = 100.0;  
const int   DRIVE_PWM         = 200;    
const int   MIN_TURN_PWM      = 60;
float LEFT_TRIM  = 1.00;              
float RIGHT_TRIM = 1.00;
#define USE_DISTURBANCE_POT 1

float KP = 250.0;     
float KI = 400.0;     
float KD = 12.0;      
const float U_MAX = 80.0;        
const float I_MAX = 40.0;        
bool pidEnabled = true;

const unsigned long CONTROL_MS = 20;

const byte MODE_IDLE = 0, MODE_TIMED = 1, MODE_STRAIGHT = 2, MODE_TURN = 3;
const byte M_STOP = 0, M_FWD = 1, M_REV = 2, M_PIV_L = 3,
           M_PIV_R = 4, M_TANK_L = 5, M_TANK_R = 6;

byte mode = MODE_IDLE;
byte currentMotion = M_STOP;
unsigned long motionStart = 0, motionEnd = 0, lastTick = 0, lastPrint = 0;
int driveSpeed = DRIVE_PWM;
bool plotMode = false;

float cmdL = 0, cmdR = 0;            
float rightFactor = 1.0;             
float poseX = 0, poseY = 0, poseTh = 0;   
float travelled = 0, targetDist = 0, targetTh = 0;
int straightDir = 1;
float pidInt = 0, lastTh = 0, lastU = 0;

#define SAY(s) do { if (!plotMode) Serial.println(F(s)); } while (0)

void setMotor(int en, int a, int b, int speed) {
  speed = constrain(speed, -255, 255);
  digitalWrite(a, speed > 0);
  digitalWrite(b, speed < 0);
  analogWrite(en, abs(speed));
}

void drive(int left, int right) {
  cmdL = constrain(left  * LEFT_TRIM,  -255, 255);
  cmdR = constrain(right * RIGHT_TRIM, -255, 255);
  setMotor(ENA, IN1, IN2, (int)cmdL);
  setMotor(ENB, IN3, IN4, (int)cmdR);
}

void stopRover()       { drive(0, 0); }
void forward(int s)    { drive(s, s); }
void reverse(int s)    { drive(-s, -s); }
void pivotLeft(int s)  { drive(0, s); }
void pivotRight(int s) { drive(s, 0); }
void tankLeft(int s)   { drive(-s, s); }
void tankRight(int s)  { drive(s, -s); }

float wrapPi(float a) {
  while (a >  PI) a -= 2.0 * PI;
  while (a < -PI) a += 2.0 * PI;
  return a;
}

float pwmToSpeed(float pwm) {   
  return MOTOR_RPM_MAX / 60.0 * PI * WHEEL_DIAMETER_MM * pwm / 255.0;
}

void readDisturbance() {
#if USE_DISTURBANCE_POT
  rightFactor = 0.85 + 0.30 * (analogRead(POT_PIN) / 1023.0);
#endif
}

void updatePose(float dt) {
  float vL = pwmToSpeed(cmdL);
  float vR = pwmToSpeed(cmdR) * rightFactor;
  float v = (vL + vR) / 2.0;
  float w = (vR - vL) / WHEEL_BASE_MM;
  poseTh = wrapPi(poseTh + w * dt);
  poseX += v * cos(poseTh) * dt;
  poseY += v * sin(poseTh) * dt;
  travelled += fabs(v) * dt;
}

void getPose(float &x, float &y, float &th) { x = poseX; y = poseY; th = poseTh; }
void resetPose() { poseX = 0; poseY = 0; poseTh = 0; }

void startMotion(byte m, int speed, unsigned long ms) {
  switch (m) {
    case M_FWD:    forward(speed);    break;
    case M_REV:    reverse(speed);    break;
    case M_PIV_L:  pivotLeft(speed);  break;
    case M_PIV_R:  pivotRight(speed); break;
    case M_TANK_L: tankLeft(speed);   break;
    case M_TANK_R: tankRight(speed);  break;
    default:       stopRover();       break;
  }
  currentMotion = m;
  motionStart = millis();
  motionEnd = motionStart + ms;
  mode = MODE_TIMED;
}

void moveMM(float mm) {          
  travelled = 0;
  targetDist = fabs(mm);
  straightDir = (mm >= 0) ? 1 : -1;
  targetTh = poseTh;
  pidInt = 0;
  lastTh = poseTh;
  mode = MODE_STRAIGHT;
}

void turnByDeg(float deg) {    
  targetTh = wrapPi(poseTh + deg * PI / 180.0);
  motionStart = millis();
  mode = MODE_TURN;
}

bool motionBusy() { return mode != MODE_IDLE; }
void abortMotion() { stopRover(); mode = MODE_IDLE; }

void controlStep(float dt) {
  updatePose(dt);                      

  if (mode == MODE_TIMED) {
    if ((long)(millis() - motionEnd) >= 0) {
      stopRover(); mode = MODE_IDLE;
      if (!plotMode && currentMotion != M_STOP) {
        Serial.print(F("Timed motion done, ms = "));
        Serial.println(millis() - motionStart);
      }
    }
  }
  else if (mode == MODE_STRAIGHT) {
    if (travelled >= targetDist) {
      stopRover(); mode = MODE_IDLE;
      if (!plotMode) {
        Serial.print(F("Straight done. x,y,heading(deg) = "));
        Serial.print(poseX); Serial.print(F(", "));
        Serial.print(poseY); Serial.print(F(", "));
        Serial.println(poseTh * 180.0 / PI);
      }
    } else {
      float u = 0;
      if (pidEnabled) {
        float err = wrapPi(targetTh - poseTh);
        pidInt += err * dt;
        float iTerm = constrain(KI * pidInt, -I_MAX, I_MAX);
        if (KI > 0) pidInt = iTerm / KI;                 
        float rate = wrapPi(poseTh - lastTh) / dt;       
        u = KP * err + iTerm - KD * rate;
        u = constrain(u, -U_MAX, U_MAX);
      }
      lastU = u;
      lastTh = poseTh;
      int base = straightDir * DRIVE_PWM;
      drive((int)(base - u), (int)(base + u));
    }
  }
  else if (mode == MODE_TURN) {
    float err = wrapPi(targetTh - poseTh);
    float errDeg = err * 180.0 / PI;
    if (fabs(errDeg) < 1.5 || millis() - motionStart > 5000UL) {
      stopRover(); mode = MODE_IDLE;
      if (!plotMode) {
        Serial.print(F("Turn done. heading(deg) = "));
        Serial.println(poseTh * 180.0 / PI);
      }
    } else {
      int s = (int)constrain(fabs(errDeg) * 2.5, MIN_TURN_PWM, DRIVE_PWM);
      if (err > 0) tankLeft(s); else tankRight(s);
    }
  }

  if (plotMode && mode != MODE_IDLE && millis() - lastPrint >= 100) {
    lastPrint = millis();
    Serial.print(targetTh * 180.0 / PI); Serial.print(',');
    Serial.print(poseTh * 180.0 / PI);   Serial.print(',');
    Serial.println(lastU);
  }
}

const byte demoMotion[] = { M_FWD, M_STOP, M_REV, M_STOP, M_PIV_L, M_STOP,
                            M_PIV_R, M_STOP, M_TANK_L, M_STOP, M_TANK_R, M_STOP };
const unsigned int demoMs[] = { 2000, 500, 2000, 500, 1000, 500,
                                1000, 500, 1000, 500, 1000, 1000 };
const byte DEMO_LEN = sizeof(demoMotion);
byte demoIdx = 0;
bool demoRunning = false;

void runDemo() {
  if (!demoRunning || motionBusy()) return;
  if (demoIdx >= DEMO_LEN) { demoRunning = false; SAY("Demo finished"); return; }
  startMotion(demoMotion[demoIdx], driveSpeed, demoMs[demoIdx]);
  demoIdx++;
}

void printPose() {
  Serial.print(F("POSE,")); Serial.print(poseX); Serial.print(',');
  Serial.print(poseY); Serial.print(','); Serial.println(poseTh * 180.0 / PI);
}

void handleSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();
  switch (c) {
    case 'f': startMotion(M_FWD,    driveSpeed, 2000); SAY("Forward 2 s");     break;
    case 'b': startMotion(M_REV,    driveSpeed, 2000); SAY("Reverse 2 s");     break;
    case 'l': startMotion(M_PIV_L,  driveSpeed, 1000); SAY("Pivot left 1 s");  break;
    case 'r': startMotion(M_PIV_R,  driveSpeed, 1000); SAY("Pivot right 1 s"); break;
    case 'a': startMotion(M_TANK_L, driveSpeed, 1000); SAY("Tank left 1 s");   break;
    case 'd': startMotion(M_TANK_R, driveSpeed, 1000); SAY("Tank right 1 s");  break;
    case 's': abortMotion(); demoRunning = false; SAY("Stopped"); break;
    case 'h': pidEnabled = true;  moveMM(500); SAY("Straight 500 mm, PID ON");  break;
    case 'k': pidEnabled = false; moveMM(500); SAY("Straight 500 mm, PID OFF"); break;
    case 't': turnByDeg(90);  SAY("Turn +90"); break;
    case 'u': turnByDeg(-90); SAY("Turn -90"); break;
    case 'g': demoIdx = 0; demoRunning = true; SAY("Demo started"); break;
    case 'q': printPose(); break;
    case 'z': resetPose(); SAY("Pose zeroed"); break;
    case 'p': plotMode = !plotMode; break;
    case '+': driveSpeed = min(255, driveSpeed + 25); if (!plotMode) { Serial.print(F("PWM ")); Serial.println(driveSpeed); } break;
    case '-': driveSpeed = max(50,  driveSpeed - 25); if (!plotMode) { Serial.print(F("PWM ")); Serial.println(driveSpeed); } break;
    default: break;
  }
}


void setup() {
  int pins[] = { ENA, IN1, IN2, ENB, IN3, IN4 };
  for (int i = 0; i < 6; i++) pinMode(pins[i], OUTPUT);
  stopRover();
  Serial.begin(9600);
  SAY("M2 motion ready: f b l r a d s h k t u g q z p + -");
}

void loop() {
  unsigned long now = millis();
  if (now - lastTick >= CONTROL_MS) {
    float dt = (now - lastTick) / 1000.0;
    lastTick = now;
    readDisturbance();
    controlStep(dt);
  }
  handleSerial();
  runDemo();
}
