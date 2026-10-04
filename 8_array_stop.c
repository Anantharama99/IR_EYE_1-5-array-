// 8 ARRAY NEW BOT
// Professional Line Follower Robot - 8-Array IR Sensor Version
/*
 * 8-ARRAY LINE FOLLOWER ROBOT
 * D2 = Calibration | D3 = Start | Both = Emergency Stop
 * RATIO-BASED SPEED CONFIGURATION - Change ONLY lfSpeed
 * WITH CORRECTED 3-STAGE RECOVERY
 * Stage 1: 45 deg towards last line side
 * Stage 2: 180 deg opposite direction
 * Stage 3: 360 deg SAME direction as Stage 1
 *
 * === v6 FINAL (8-SENSOR PORT, GEOMETRY-FIXED) ===
 *  - All-black: creep forward (curve ride) -> stop after moving stopDistanceCm
 *  - All-white: falls through to line-lost recovery
 *
 * Sensor mapping (physical):
 *   A0 = RIGHTMOST  ... A7 = LEFTMOST
 * Sensor mapping (logical index i used in code):
 *   i = 0 -> A0 (rightmost)
 *   i = 1 -> A1
 *   i = 2 -> A2
 *   i = 3 -> A3
 *   i = 4 -> A4
 *   i = 5 -> A5
 *   i = 6 -> A6
 *   i = 7 -> A7 (leftmost)
 *
 * Weights therefore run NEGATIVE (right) -> POSITIVE (left).
 *
 * LED behaviour:
 *   - Calibrating : fast toggle (handled inside calibrate())
 *   - NOT calibrated : 3 blinks + long gap (repeating)
 *   - Calibrated idle: 2 blinks + gap (repeating)
 *   - Running        : solid ON
 */

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

//--------Pin definitions for the TB6612FNG Motor Driver----
#define AIN1 9
#define AIN2 8
#define PWMA 10
#define BIN1 7
#define BIN2 6
#define PWMB 5
#define STBY 11
#define LED_PIN 13
//------------------------------------------------------------

//--------Sensor Pin Definitions (8-Array)--------
// A0 = rightmost  ...  A7 = leftmost  (physical order)
const int sensorPins[8] = { A0, A1, A2, A3, A4, A5, A6, A7 };
//------------------------------------------------

//--------Enter Line Details here---------
bool isBlackLine = 1;
unsigned int lineThickness = 25;
unsigned int numSensors = 8;
//-----------------------------------------

// ============ MASTER SPEED CONTROL - CHANGE ONLY THIS ============
int lfSpeed = 120;
// ================================================================

// Dynamic timings (calculated in calculateSpeeds())
int TIME_45_DEG;
int TIME_180_DEG;
int TIME_360_DEG;

// Auto-calculated speed variables
int minTurnSpeed;
int calSpeed;
int recoverySpeed;
int recoveryTurnSpeed;
int accelerationStep;
int recoveryLeftTurn;
int recoveryRightTurn;
int loopDelay;
int calibrationSeconds;

// PID Variables
int P, D, I, previousError, PIDvalue;
double error;
int lsp, rsp;
int currentSpeed = 30;

// Weights for 8 sensors.
// Index 0 = A0 = RIGHTMOST  -> negative weight
// Index 7 = A7 = LEFTMOST   -> positive weight
int sensorWeight[8] = { -4, -3, -2, -1, 1, 2, 3, 4 };

int activeSensors;
float Kp = 0.06;
float Kd = 0.08;
float Ki = 0.0005;

// Sensor Variables
int onLine = 0;
int minValues[8], maxValues[8], threshold[8], sensorValue[8], sensorArray[8];

// Button pins
const int CAL_BTN = 2;
const int START_BTN = 3;

// State variables
bool isRunning = false;
bool calibrated = false;
bool emergencyStop = false;

// LED state variables
bool isCalibrating = false;
unsigned long ledTimer = 0;
int ledStep = 0;

// ============ 3-STAGE RECOVERY VARIABLES ============
int lastLineSide = 0;
bool inRecovery = false;

// Debounce variables
const int debounceDelay = 80;

// ============ DISTANCE-BASED BLACK-BOX STOP ============
const float wheelDiameterCm      = 2.6;                          // your wheel
const float wheelCircumferenceCm = 3.14159265 * wheelDiameterCm; // ~8.17 cm
// Empirical cm/s per PWM unit for this drivetrain.
//   - If bot stops too EARLY (before reaching middle of box) -> DECREASE this.
//   - If bot stops too LATE  (past the box)                  -> INCREASE this.
const float cmPerSecPerPwm       = 0.7;
// Distance to travel inside the all-black region before stopping.
const float stopDistanceCm       = 12.0;

unsigned long allBlackLastTime   = 0;
float allBlackDistanceCm         = 0.0;
int lastLeftCmd                  = 0;
int lastRightCmd                 = 0;
bool inAllBlackState             = false;
// ============================================================

void setup() {
  calculateSpeeds();

  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);

  pinMode(CAL_BTN, INPUT_PULLUP);
  pinMode(START_BTN, INPUT_PULLUP);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  digitalWrite(STBY, HIGH);

  lineThickness = constrain(lineThickness, 10, 35);

  delay(500);
}

// ============ PROFESSIONAL AUTO SCALING FUNCTION ============
void calculateSpeeds() {
  lfSpeed = constrain(lfSpeed, 60, 255);

  minTurnSpeed       = lfSpeed * 0.67;
  calSpeed           = lfSpeed * 0.65;
  recoverySpeed      = lfSpeed * 0.85;
  recoveryTurnSpeed  = lfSpeed * 0.60;

  recoveryLeftTurn   = -lfSpeed * 0.45;
  recoveryRightTurn  =  lfSpeed * 0.85;

  minTurnSpeed = constrain(minTurnSpeed, 40, 180);
  calSpeed = constrain(calSpeed, 50, 150);
  recoverySpeed = constrain(recoverySpeed, 60, 200);
  recoveryTurnSpeed = constrain(recoveryTurnSpeed, 40, 150);
  recoveryLeftTurn = constrain(recoveryLeftTurn, -120, -30);
  recoveryRightTurn = constrain(recoveryRightTurn, 60, 200);

  accelerationStep = max(1, lfSpeed / 80);
  loopDelay = constrain(20 - (lfSpeed / 15), 5, 20);
  calibrationSeconds = 4;

  float baseSpeed = 80.0;
  float scale = baseSpeed / lfSpeed;
  scale = constrain(scale, 0.4, 2.5);

  TIME_45_DEG  = 300  * scale;
  TIME_180_DEG = 780  * scale;
  TIME_360_DEG = 1560 * scale;
}

// ============ CORRECTED 3-STAGE RECOVERY FUNCTION ============
void recoverySequence() {
  inRecovery = true;

  int turnSpeed = recoveryTurnSpeed * 1.2;
  turnSpeed = constrain(turnSpeed, 60, 180);

  // Stage 1: 45 deg towards last side
  unsigned long startTime = millis();
  while (millis() - startTime < TIME_45_DEG) {
    readLine();
    if (onLine) return;
    if (lastLineSide == -1) { motor1run(-turnSpeed); motor2run(turnSpeed); }
    else                    { motor1run(turnSpeed);  motor2run(-turnSpeed); }
  }

  // Stage 2: 180 deg opposite direction
  startTime = millis();
  while (millis() - startTime < TIME_180_DEG) {
    readLine();
    if (onLine) return;
    if (lastLineSide == -1) { motor1run(turnSpeed);  motor2run(-turnSpeed); }
    else                    { motor1run(-turnSpeed); motor2run(turnSpeed); }
  }

  // Stage 3: 360 deg same as Stage 1
  startTime = millis();
  while (millis() - startTime < TIME_360_DEG) {
    readLine();
    if (onLine) return;
    if (lastLineSide == -1) { motor1run(-turnSpeed); motor2run(turnSpeed); }
    else                    { motor1run(turnSpeed);  motor2run(-turnSpeed); }
  }

  inRecovery = false;
}

// ================= SPECIAL-CONDITION LOGIC =================

bool allSensorsBlack() {
  for (int i = 0; i < 8; i++) {
    if (!sensorArray[i]) return false;
  }
  return true;
}

bool allSensorsWhite() {
  for (int i = 0; i < 8; i++) {
    if (sensorArray[i]) return false;
  }
  return true;
}

// ====== ALL-BLACK HANDLER (DISTANCE-BASED) ======
// While all sensors see black:
//   - creep forward, biased toward the last-seen line side (rides U/C curves)
//   - accumulate travelled distance from wheel circumference + PWM
//   - if travelled distance >= stopDistanceCm -> STOP (inside end box)
void handleAllBlack() {
  unsigned long now = millis();

  if (!inAllBlackState) {
    inAllBlackState    = true;
    allBlackLastTime   = now;
    allBlackDistanceCm = 0.0;
  }

  // Distance moved since last call
  unsigned long dt = now - allBlackLastTime;
  allBlackLastTime = now;
  float dtSec = dt / 1000.0;

  int avgCmd = (abs(lastLeftCmd) + abs(lastRightCmd)) / 2;
  allBlackDistanceCm += avgCmd * cmPerSecPerPwm * dtSec;

  // ---- STOP: box confirmed after moving >= stopDistanceCm ----
  if (allBlackDistanceCm >= stopDistanceCm) {
    isRunning = false;
    motor1run(0);
    motor2run(0);
    currentSpeed    = 30;
    inRecovery      = false;
    lastLineSide    = 0;
    inAllBlackState = false;
    allBlackDistanceCm = 0.0;
    lastLeftCmd  = 0;
    lastRightCmd = 0;
    return;
  }

  // ---- Otherwise creep forward (curve ride) ----
  int moveSpeed = minTurnSpeed;
  moveSpeed = constrain(moveSpeed, 50, 130);

  int leftSpd, rightSpd;
  if (lastLineSide == -1) {
    leftSpd  = moveSpeed * 0.35;
    rightSpd = moveSpeed;
  } else if (lastLineSide == +1) {
    leftSpd  = moveSpeed;
    rightSpd = moveSpeed * 0.35;
  } else {
    leftSpd  = moveSpeed;
    rightSpd = moveSpeed;
  }

  motor1run(leftSpd);
  motor2run(rightSpd);

  lastLeftCmd  = leftSpd;
  lastRightCmd = rightSpd;
}

// Reset all-black tracker whenever the bot is NOT all-black anymore.
void updateAllBlackState() {
  if (!allSensorsBlack()) {
    inAllBlackState    = false;
    allBlackDistanceCm = 0.0;
    lastLeftCmd        = 0;
    lastRightCmd       = 0;
  }
}

bool handleSpecialConditions() {
  updateAllBlackState();

  // Case 1: ALL sensors black -> curve-ride / end-box distance stop
  if (allSensorsBlack()) {
    handleAllBlack();
    return true;
  }

  // Case 2: ALL sensors white -> fall through to line-lost recovery
  if (allSensorsWhite()) {
    return false;
  }

  // No special condition -> normal behavior
  return false;
}
// ================= END SPECIAL-CONDITION LOGIC =================

// ============ LED UPDATE FUNCTION ============
void updateLED() {
  if (isCalibrating) return;

  unsigned long now = millis();
  static bool lastCalibrated = false;

  // If calibration status just changed, restart the blink sequence cleanly
  if (calibrated != lastCalibrated) {
    ledStep  = 0;
    ledTimer = now;
    lastCalibrated = calibrated;
  }

  // Running: solid ON
  if (isRunning && calibrated && !emergencyStop) {
    digitalWrite(LED_PIN, HIGH);
    ledStep = 0;
    ledTimer = now;
    return;
  }

  // ---------- Not calibrated: 3 blinks + longer gap (repeats) ----------
  if (!calibrated) {
    switch (ledStep) {
      case 0:
        digitalWrite(LED_PIN, HIGH);
        if (now - ledTimer >= 150) { ledStep = 1; ledTimer = now; }
        break;
      case 1:
        digitalWrite(LED_PIN, LOW);
        if (now - ledTimer >= 150) { ledStep = 2; ledTimer = now; }
        break;
      case 2:
        digitalWrite(LED_PIN, HIGH);
        if (now - ledTimer >= 150) { ledStep = 3; ledTimer = now; }
        break;
      case 3:
        digitalWrite(LED_PIN, LOW);
        if (now - ledTimer >= 150) { ledStep = 4; ledTimer = now; }
        break;
      case 4:
        digitalWrite(LED_PIN, HIGH);
        if (now - ledTimer >= 150) { ledStep = 5; ledTimer = now; }
        break;
      case 5:
        digitalWrite(LED_PIN, LOW);
        if (now - ledTimer >= 700) { ledStep = 0; ledTimer = now; }
        break;
      default:
        ledStep = 0;
        ledTimer = now;
        break;
    }
    return;
  }

  // ---------- Calibrated but idle (or emergency stop): 2 short + gap ----------
  switch (ledStep) {
    case 0:
      digitalWrite(LED_PIN, HIGH);
      if (now - ledTimer >= 100) { ledStep = 1; ledTimer = now; }
      break;
    case 1:
      digitalWrite(LED_PIN, LOW);
      if (now - ledTimer >= 100) { ledStep = 2; ledTimer = now; }
      break;
    case 2:
      digitalWrite(LED_PIN, HIGH);
      if (now - ledTimer >= 100) { ledStep = 3; ledTimer = now; }
      break;
    case 3:
      digitalWrite(LED_PIN, LOW);
      if (now - ledTimer >= 600) { ledStep = 0; ledTimer = now; }
      break;
    default:
      ledStep = 0;
      ledTimer = now;
      break;
  }
}

void loop() {
  updateLED();

  int calState   = digitalRead(CAL_BTN);
  int startState = digitalRead(START_BTN);

  // ============ EMERGENCY STOP ============
  if (calState == LOW && startState == LOW) {
    delay(debounceDelay);

    if (digitalRead(CAL_BTN) == LOW && digitalRead(START_BTN) == LOW) {
      emergencyStop = true;
      isRunning = false;
      inRecovery = false;
      motor1run(0);
      motor2run(0);

      while (digitalRead(CAL_BTN) == LOW || digitalRead(START_BTN) == LOW) {
        updateLED();
        delay(10);
      }
      delay(100);
      emergencyStop = false;
    }
  }

  // ============ CALIBRATION BUTTON (D2 alone) ============
  else if (calState == LOW && startState == HIGH && !emergencyStop) {
    delay(debounceDelay);

    if (digitalRead(CAL_BTN) == LOW && digitalRead(START_BTN) == HIGH) {
      calibrate();
      calibrated = true;

      while (digitalRead(CAL_BTN) == LOW) { delay(10); }
      delay(100);
    }
  }

  // ============ START BUTTON (D3 alone) ============
  else if (startState == LOW && calState == HIGH && calibrated && !emergencyStop) {
    delay(debounceDelay);

    if (digitalRead(START_BTN) == LOW && digitalRead(CAL_BTN) == HIGH) {
      isRunning = !isRunning;

      if (!isRunning) {
        motor1run(0);
        motor2run(0);
        currentSpeed = 30;
        inRecovery   = false;
        lastLineSide = 0;
        inAllBlackState    = false;
        allBlackDistanceCm = 0.0;
      } else {
        currentSpeed = minTurnSpeed;
        inRecovery   = false;
        lastLineSide = 0;
        inAllBlackState    = false;
        allBlackDistanceCm = 0.0;
      }

      while (digitalRead(START_BTN) == LOW) { delay(10); }
      delay(100);
    }
  }

  // ============ LINE FOLLOWING LOOP ============
  if (isRunning && calibrated && !emergencyStop) {
    readLine();

    // Line on LEFT  -> leftmost sensors (physical A7, A6, A5 = logical 7,6,5)
    if (sensorArray[5] || sensorArray[6] || sensorArray[7]) lastLineSide = -1;
    // Line on RIGHT -> rightmost sensors (physical A0, A1, A2 = logical 0,1,2)
    if (sensorArray[0] || sensorArray[1] || sensorArray[2]) lastLineSide = +1;

    if (currentSpeed < lfSpeed) {
      currentSpeed += accelerationStep;
    }

    if (handleSpecialConditions()) {
      // handled
    }
    else if (onLine == 1) {
      inRecovery = false;
      linefollow();
    } else {
      inRecovery = true;
      recoverySequence();
    }

    delay(loopDelay);
  }
}

void linefollow() {
  error = 0;
  activeSensors = 0;

  for (int i = 0; i < 8; i++) {
    if (sensorArray[i]) {
      error += sensorWeight[i] * sensorValue[i];
      activeSensors++;
    }
  }

  if (activeSensors > 0) error = error / activeSensors;

  P = error;
  I = I + error;
  I = constrain(I, -5000, 5000);
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = currentSpeed - PIDvalue;
  rsp = currentSpeed + PIDvalue;

  if (abs(PIDvalue) > 50) {
    if (lsp < minTurnSpeed && lsp > 0) lsp = minTurnSpeed;
    if (rsp < minTurnSpeed && rsp > 0) rsp = minTurnSpeed;
  }

  lsp = constrain(lsp, 0, 255);
  rsp = constrain(rsp, 0, 255);

  motor1run(lsp);
  motor2run(rsp);
}

void calibrate() {
  isCalibrating = true;
  unsigned long calLedTimer = 0;
  bool calLedState = false;
  digitalWrite(LED_PIN, LOW);

  for (int i = 0; i < 8; i++) {
    minValues[i] = 1023;
    maxValues[i] = 0;
  }

  unsigned long startTime = millis();
  while (millis() - startTime < (unsigned long)(calibrationSeconds * 1000)) {
    motor1run(calSpeed);
    motor2run(-calSpeed);

    if (millis() - calLedTimer >= 100) {
      calLedState = !calLedState;
      digitalWrite(LED_PIN, calLedState);
      calLedTimer = millis();
    }

    for (int i = 0; i < 8; i++) {
      int value = analogRead(sensorPins[i]);
      if (value < minValues[i]) minValues[i] = value;
      if (value > maxValues[i]) maxValues[i] = value;
    }
    delay(10);
  }

  motor1run(0);
  motor2run(0);
  digitalWrite(LED_PIN, LOW);
  isCalibrating = false;
  delay(100);

  for (int i = 0; i < 8; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
  }
}

void readLine() {
  onLine = 0;

  for (int i = 0; i < 8; i++) {
    int rawValue = analogRead(sensorPins[i]);

    if (isBlackLine) {
      sensorValue[i] = map(rawValue, minValues[i], maxValues[i], 0, 1000);
    } else {
      sensorValue[i] = map(rawValue, minValues[i], maxValues[i], 1000, 0);
    }
    sensorValue[i] = constrain(sensorValue[i], 0, 1000);

    sensorArray[i] = (sensorValue[i] > 500);

    if (sensorArray[i]) onLine = 1;
  }
}

//--------Function to run Motor 1 (Left motor)-----------------
void motor1run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, abs(motorSpeed));
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, 0);
  }
}

//--------Function to run Motor 2 (Right motor)-----------------
void motor2run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, abs(motorSpeed));
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, 0);
  }
}
