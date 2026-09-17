//====================================================================
// ESP32 Line Follower — 17mm LINE | FINAL BUILD (Infinity 2K26)
// ─────────────────────────────────────────────────────────────────
// DUAL-ZONE TRACKING:
//   CENTER ZONE  → sensors 6,7,8,9   → gentle PID, full cruise speed
//   EDGE ZONE    → sensors 0-5 & 10-15 → aggressive PID, turn speed
//
// ROOT CAUSE OF SPINNING IN PLACE:
//   Motor A (LEFT) and Motor B (RIGHT) were spinning in OPPOSITE
//   directions — one forward, one backward — causing the spin.
//   Fix: Motor B FWD/REV pins are inverted relative to Motor A
//   because the RIGHT motor is physically mounted mirror-image.
//   Motor A wiring is correct as-is. Only Motor B is flipped.
//
// Sensor 0 = LEFT edge    Sensor 15 = RIGHT edge
// Motor A   = LEFT wheel  Motor B   = RIGHT wheel
//====================================================================
 
#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
 
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
 
// ─── Hardware Pins ────────────────────────────────────────────────
#define BTN_START_STOP   13
#define MOTOR_A_FWD      17    // LEFT  motor — FWD
#define MOTOR_A_REV      16    // LEFT  motor — REV
#define MOTOR_A_PWM       4
#define MOTOR_B_FWD       5    // RIGHT motor — FWD (INVERTED vs Motor A)
#define MOTOR_B_REV      18    // RIGHT motor — REV (INVERTED vs Motor A)
#define MOTOR_B_PWM      19
#define MUX_S0           25
#define MUX_S1           33
#define MUX_S2           26
#define MUX_S3           27
#define IR_ANALOG        34
 
// ─── Sensor Layout ────────────────────────────────────────────────
//  [0   1   2   3   4   5] [6   7 | 8   9] [10  11  12  13  14  15]
//   └────── EDGE LEFT ────┘  └─ CENTER ──┘   └───── EDGE RIGHT ────┘
//  LEFT                                                         RIGHT
//
//  Positive weight → line is LEFT of center  → steer RIGHT (slow left, speed right)
//  Negative weight → line is RIGHT of center → steer LEFT  (speed left, slow right)
const int SENSOR_WEIGHTS[16] = {
   38,  32,  27,  22,  18,  13,   //  0- 5  left edge   (strong positive)
    5,   2,  -2,  -5,             //  6- 9  center      (gentle)
  -13, -18, -22, -27, -32, -38   // 10-15  right edge  (strong negative)
};
 
inline bool isCenterSensor(int i) { return (i >= 6 && i <= 9);  }
inline bool isEdgeSensor(int i)   { return (i <= 5 || i >= 10); }
 
// ─── PWM ──────────────────────────────────────────────────────────
const uint32_t PWM_FREQ       = 1000;
const uint8_t  PWM_RESOLUTION = 16;
const int      MAX_PWM        = 65535;
 
// ─── Speed Settings ───────────────────────────────────────────────
const int MAX_BASE_SPEED  = 60000;  // straight cruise
const int TURN_BASE_SPEED = 40000;  // edge/turn mode base
const int MIN_BASE_SPEED  = 20000;  // absolute floor
const int SEARCH_SPEED    = 26000;  // lost-line recovery
const int LOOP_INTERVAL   = 2;      // ms per control tick
 
// ─── PID — CENTER ZONE (sensors 6-9 only) ────────────────────────
const float Kp_center = 4.80f;
const float Kd_center = 13.00f;
const float Ki_center =  0.001f;
 
// ─── PID — EDGE ZONE (any of sensors 0-5 or 10-15) ───────────────
const float Kp_edge   = 7.50f;
const float Kd_edge   = 18.00f;
const float Ki_edge   =  0.0005f;
 
const float I_LIMIT   = 3500.0f;
 
// ─── State & Globals ──────────────────────────────────────────────
enum State { STOPPED, CALIBRATING, READY, RUNNING };
State robotState = STOPPED;
 
int   minWhite[16], maxBlack[16];
int   sensorNorm[16];
int   activeCount  = 0;
bool  onLine       = false;
bool  edgeActive   = false;
bool  centerActive = false;
 
float positionError    = 0.0f;
float previousPIDError = 0.0f;
float integral         = 0.0f;
float filteredError    = 0.0f;
float filteredD        = 0.0f;
 
unsigned long lastLoopTime   = 0;
unsigned long lastButtonMs   = 0;
unsigned long lostStartTime  = 0;
unsigned long stateChangedMs = 0;   // lockout: ignore button for 600ms after any state change
volatile bool buttonFlag     = false;
 
// ─── Prototypes ───────────────────────────────────────────────────
void IRAM_ATTR buttonISR();
void motorA(int spd);
void motorB(int spd);
void setMotors(int leftSpd, int rightSpd);
int  readSensor(byte ch);
void readAllSensors();
void followLine();
void autoCalibrate();
void showOLED(const char* l1, const char* l2 = nullptr);
 
//====================================================================
// SETUP
//====================================================================
void setup() {
  Serial.begin(115200);
  pinMode(BTN_START_STOP, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTN_START_STOP), buttonISR, FALLING);
 
  Wire.begin(21, 22);
  u8g2.begin();
 
  pinMode(MOTOR_A_FWD, OUTPUT); pinMode(MOTOR_A_REV, OUTPUT);
  pinMode(MOTOR_B_FWD, OUTPUT); pinMode(MOTOR_B_REV, OUTPUT);
  ledcAttach(MOTOR_A_PWM, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(MOTOR_B_PWM, PWM_FREQ, PWM_RESOLUTION);
 
  pinMode(MUX_S0, OUTPUT); pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT); pinMode(MUX_S3, OUTPUT);
 
  setMotors(0, 0);
  showOLED("CALIBRATE", "Press BTN");
}
 
//====================================================================
// MAIN LOOP
//====================================================================
void loop() {
  // ── Button handler ─────────────────────────────────────────────
  // Guard 1: buttonFlag set by ISR (250ms hardware debounce)
  // Guard 2: stateChangedMs lockout (600ms after any state change)
  //   → prevents bounces accumulated during the 5s calibration sweep
  //     from immediately firing a second transition when READY is shown
  if (buttonFlag && (millis() - stateChangedMs >= 600)) {
    buttonFlag = false;
    stateChangedMs = millis();                  // start lockout window
 
    if      (robotState == STOPPED)  { robotState = CALIBRATING; }
    else if (robotState == READY)    { robotState = RUNNING;     }
    else    { robotState = STOPPED;  setMotors(0, 0);            }
  } else if (buttonFlag && (millis() - stateChangedMs < 600)) {
    buttonFlag = false;   // discard stale/bounce press inside lockout window
  }
 
  if (millis() - lastLoopTime >= LOOP_INTERVAL) {
    lastLoopTime = millis();
 
    switch (robotState) {
 
      case CALIBRATING:
        autoCalibrate();
        // ── CRITICAL: flush any button events that fired during the
        //    5s sweep. Without this, a bounce sets buttonFlag=true
        //    and the very next loop tick transitions READY → RUNNING
        //    before the user touches anything.
        buttonFlag     = false;
        lastButtonMs   = millis();
        stateChangedMs = millis();
        robotState     = READY;
        showOLED("READY!", "Press BTN");
        break;
 
      case RUNNING:
        readAllSensors();
 
        // All sensors active = intersection → drive straight through
        if (activeCount > 13) {
          setMotors(MAX_BASE_SPEED, MAX_BASE_SPEED);
          lostStartTime = 0;
          break;
        }
 
        if (onLine) {
          followLine();
          lostStartTime = 0;
        } else {
          if (lostStartTime == 0) lostStartTime = millis();
          if (millis() - lostStartTime > 800) {
            robotState = STOPPED;
            setMotors(0, 0);
            showOLED("LINE LOST", "Stopped");
          } else {
            // Spin toward last known line side
            if (positionError >= 0) setMotors(-SEARCH_SPEED,  SEARCH_SPEED);
            else                    setMotors( SEARCH_SPEED, -SEARCH_SPEED);
          }
        }
        break;
 
      case STOPPED:
      case READY:
        setMotors(0, 0);
        previousPIDError = 0;
        integral         = 0;
        filteredError    = 0;
        filteredD        = 0;
        break;
    }
  }
}
 
//====================================================================
// SENSOR PROCESSING
//====================================================================
void readAllSensors() {
  long sumError = 0;
  activeCount   = 0;
  onLine        = false;
  edgeActive    = false;
  centerActive  = false;
 
  for (int i = 0; i < 16; i++) {
    int raw       = readSensor(i);
    sensorNorm[i] = map(raw, minWhite[i], maxBlack[i], 1000, 0);
    sensorNorm[i] = constrain(sensorNorm[i], 0, 1000);
 
    if (sensorNorm[i] < 750) {           // 750 catches soft 17mm edge hits
      onLine = true;
      activeCount++;
      sumError += (long)SENSOR_WEIGHTS[i] * (1000 - sensorNorm[i]);
      if (isCenterSensor(i)) centerActive = true;
      if (isEdgeSensor(i))   edgeActive   = true;
    }
  }
 
  if (activeCount > 0) positionError = (float)sumError / activeCount;
}
 
//====================================================================
// DUAL-ZONE PID FOLLOW
//====================================================================
void followLine() {
  filteredError = 0.70f * filteredError + 0.30f * positionError;
  float P = filteredError;
 
  // ── Zone selection ────────────────────────────────────────────
  float Kp_use, Kd_use, Ki_use;
  int   baseSpeed;
  float deadband;
 
  if (edgeActive) {
    // TURN MODE — any edge sensor lit
    Kp_use    = Kp_edge;
    Kd_use    = Kd_edge;
    Ki_use    = Ki_edge;
    baseSpeed = TURN_BASE_SPEED;
    deadband  = 40.0f;
  } else {
    // STRAIGHT MODE — only center sensors
    Kp_use    = Kp_center;
    Kd_use    = Kd_center;
    Ki_use    = Ki_center;
    baseSpeed = MAX_BASE_SPEED;
    deadband  = 80.0f;
  }
 
  if (fabs(P) < deadband) P = 0.0f;
 
  // ── Dynamic speed drop (eases into turns) ─────────────────────
  float normFactor = edgeActive ? 10000.0f : 12666.0f;
  float errMag     = constrain(fabs(P) / normFactor, 0.0f, 1.0f);
  int   speedDrop  = (int)((errMag * errMag) * (float)(baseSpeed - MIN_BASE_SPEED));
  int   dynBase    = max(baseSpeed - speedDrop, MIN_BASE_SPEED);
 
  if (errMag > 0.55f) Kp_use *= 1.6f;   // extra push on sharp turns
 
  // ── Derivative (faster response in edge zone) ─────────────────
  float rawD   = P - previousPIDError;
  float dAlpha = edgeActive ? 0.45f : 0.50f;
  filteredD    = (1.0f - dAlpha) * filteredD + dAlpha * rawD;
 
  // ── Integral with anti-windup ──────────────────────────────────
  integral = constrain(integral + P, -I_LIMIT, I_LIMIT);
 
  // ── Correction ────────────────────────────────────────────────
  float correction = (Kp_use * P) + (Ki_use * integral) + (Kd_use * filteredD);
  previousPIDError = P;
 
  // Positive correction → line is LEFT → slow LEFT motor, speed RIGHT motor
  int leftSpeed  = dynBase - (int)correction;
  int rightSpeed = dynBase + (int)correction;
 
  if      (correction >  dynBase) setMotors(0,          rightSpeed); // hard left turn
  else if (correction < -dynBase) setMotors(leftSpeed,  0);          // hard right turn
  else                            setMotors(leftSpeed,  rightSpeed);
}
 
//====================================================================
// MOTOR DRIVERS
// ─────────────────────────────────────────────────────────────────
// WHY MOTORS ARE DIFFERENT:
//   Left and right motors face OPPOSITE directions on the chassis.
//   Turning the LEFT shaft clockwise moves the robot FORWARD.
//   Turning the RIGHT shaft clockwise moves the robot BACKWARD.
//   So Motor B (RIGHT) has its FWD/REV pins logically inverted
//   compared to Motor A (LEFT) to make both wheels roll forward
//   when given a positive speed value.
//
//   Motor A (LEFT):  FWD → pin HIGH, REV → pin LOW  = forward ✓
//   Motor B (RIGHT): FWD → pin LOW,  REV → pin HIGH = forward ✓  ← INVERTED
//
// If the bot STILL spins after flashing:
//   → Try swapping the physical wire pair on Motor B's driver terminals.
//====================================================================
 
// Motor A — LEFT wheel
void motorA(int spd) {
  spd = constrain(spd, -MAX_PWM, MAX_PWM);
  if (spd > 0) {
    digitalWrite(MOTOR_A_FWD, HIGH);
    digitalWrite(MOTOR_A_REV, LOW);
    ledcWrite(MOTOR_A_PWM, spd);
  } else if (spd < 0) {
    digitalWrite(MOTOR_A_FWD, LOW);
    digitalWrite(MOTOR_A_REV, HIGH);
    ledcWrite(MOTOR_A_PWM, -spd);
  } else {
    digitalWrite(MOTOR_A_FWD, HIGH);  // active brake
    digitalWrite(MOTOR_A_REV, HIGH);
    ledcWrite(MOTOR_A_PWM, MAX_PWM);
  }
}
 
// Motor B — RIGHT wheel (FWD/REV INVERTED — mirror-mounted motor)
void motorB(int spd) {
  spd = constrain(spd, -MAX_PWM, MAX_PWM);
  if (spd > 0) {
    digitalWrite(MOTOR_B_FWD, LOW);   // ← INVERTED: LOW = forward for right motor
    digitalWrite(MOTOR_B_REV, HIGH);
    ledcWrite(MOTOR_B_PWM, spd);
  } else if (spd < 0) {
    digitalWrite(MOTOR_B_FWD, HIGH);
    digitalWrite(MOTOR_B_REV, LOW);   // ← INVERTED
    ledcWrite(MOTOR_B_PWM, -spd);
  } else {
    digitalWrite(MOTOR_B_FWD, HIGH);  // active brake
    digitalWrite(MOTOR_B_REV, HIGH);
    ledcWrite(MOTOR_B_PWM, MAX_PWM);
  }
}
 
// Convenience wrapper — always use this to drive both wheels
void setMotors(int leftSpd, int rightSpd) {
  motorA(leftSpd);
  motorB(rightSpd);
}
 
//====================================================================
// CALIBRATION — 5 s sweep, contrast guard
//====================================================================
void autoCalibrate() {
  showOLED("SWEEP...", "Hold still");
  for (int i = 0; i < 16; i++) { minWhite[i] = 4095; maxBlack[i] = 0; }
 
  // Slow spin so every sensor crosses both white surface and black tape
  setMotors(20000, -20000);
  unsigned long start = millis();
  while (millis() - start < 5000) {
    for (int i = 0; i < 16; i++) {
      int v = readSensor(i);
      if (v < minWhite[i]) minWhite[i] = v;
      if (v > maxBlack[i]) maxBlack[i] = v;
    }
  }
  setMotors(0, 0);
  delay(200);
 
  // If a sensor saw < 200 ADC contrast, default to full range (safe fallback)
  for (int i = 0; i < 16; i++) {
    if ((maxBlack[i] - minWhite[i]) < 200) {
      minWhite[i] = 0;
      maxBlack[i] = 4095;
    }
  }
}
 
//====================================================================
// MUX READ
//====================================================================
int readSensor(byte ch) {
  digitalWrite(MUX_S0, (ch >> 0) & 1);
  digitalWrite(MUX_S1, (ch >> 1) & 1);
  digitalWrite(MUX_S2, (ch >> 2) & 1);
  digitalWrite(MUX_S3, (ch >> 3) & 1);
  delayMicroseconds(25);
  return analogRead(IR_ANALOG);
}
 
//====================================================================
// OLED HELPER
//====================================================================
void showOLED(const char* l1, const char* l2) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB14_tr);
  u8g2.drawStr(4, 28, l1);
  if (l2) {
    u8g2.setFont(u8g2_font_ncenB10_tr);
    u8g2.drawStr(4, 52, l2);
  }
  u8g2.sendBuffer();
}
 
//====================================================================
// BUTTON ISR
//====================================================================
void IRAM_ATTR buttonISR() {
  unsigned long now = millis();
  if (now - lastButtonMs >= 300) {   // 300ms hardware debounce (was 250ms)
    buttonFlag   = true;
    lastButtonMs = now;
  }
}
 