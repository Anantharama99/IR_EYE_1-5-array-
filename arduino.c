//5 array new bot 
//speed track
/*
 * PROFESSIONAL LINE FOLLOWER ROBOT - ROBOJUNKIES 5-ARRAY
 * D2 = Calibration | D3 = Start | Both = Emergency Stop
 * RATIO-BASED SPEED CONFIGURATION - Change ONLY lfSpeed
 * WITH CORRECTED 3-STAGE RECOVERY
 * Stage 1: 45° towards last line side
 * Stage 2: 180° opposite direction
 * Stage 3: 360° SAME direction as Stage 1
 * 
 * WITH PROFESSIONAL AUTO SCALING
 * All speeds AND rotation times depend ONLY on lfSpeed
 */

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

//--------Pin definitions for the TB6612FNG Motor Driver----
#define AIN1 9    // Motor A input 1 (Left motor)
#define AIN2 8    // Motor A input 2 (Left motor)
#define PWMA 10   // Motor A speed (Left motor)
#define BIN1 7    // Motor B input 1 (Right motor)
#define BIN2 6    // Motor B input 2 (Right motor)
#define PWMB 5    // Motor B speed (Right motor)
#define STBY 11   // Standby pin
//------------------------------------------------------------

//--------Sensor Pin Definitions (5-Array)--------
const int S1 = A0;  // Left most sensor (out1)
const int S2 = A1;  // Left middle sensor (out2)
const int S3 = A2;  // Center sensor (out3)
const int S4 = A3;  // Right middle sensor (out4)
const int S5 = A4;  // Right most sensor (out5)
//------------------------------------------------

//--------Enter Line Details here---------
bool isBlackLine = 1;             // 1 for black line, 0 for white line
unsigned int lineThickness = 25;  // Line thickness in mm
unsigned int numSensors = 5;      // Number of sensors
//-----------------------------------------

// ============ MASTER SPEED CONTROL - CHANGE ONLY THIS ============
int lfSpeed = 80;  // Maximum straight line speed (60-255)
// ALL OTHER SPEEDS AND TIMINGS WILL AUTO-ADJUST BASED ON THIS VALUE
// ================================================================

// Dynamic timings (will be calculated in calculateSpeeds())
int TIME_45_DEG;
int TIME_180_DEG;
int TIME_360_DEG;

// Auto-calculated speed variables (DO NOT CHANGE MANUALLY)
int minTurnSpeed;           // 67% of lfSpeed (for curves)
int calSpeed;               // 65% of lfSpeed (for calibration)
int recoverySpeed;          // 85% of lfSpeed (for recovery)
int recoveryTurnSpeed;      // 60% of lfSpeed (spin speed)
int accelerationStep;       // Auto-calculated (1-3)
int recoveryLeftTurn;       // Auto-calculated based on lfSpeed
int recoveryRightTurn;      // Auto-calculated based on lfSpeed
int loopDelay;              // Auto-adjusted based on speed
int calibrationSeconds;     // Fixed at 4 seconds

// PID Variables
int P, D, I, previousError, PIDvalue;
double error;
int lsp, rsp;
int currentSpeed = 30;
int sensorWeight[7] = { 4, 2, 1, 0, -1, -2, -4 };
int activeSensors;
float Kp = 0.06;
float Kd = 0.08;
float Ki = 0.0005;

// Sensor Variables
int onLine = 0;
int minValues[7], maxValues[7], threshold[7], sensorValue[7], sensorArray[7];

// Button pins
const int CAL_BTN = 2;    // Calibration button (D2)
const int START_BTN = 3;  // Start button (D3)

// State variables
bool isRunning = false;
bool calibrated = false;
bool emergencyStop = false;

// ============ 3-STAGE RECOVERY VARIABLES ============
int lastLineSide = 0;     // -1 = left, +1 = right
bool inRecovery = false;  // Flag to track if we're in recovery mode

// Debounce variables
const int debounceDelay = 50;

void setup() {
  // Calculate all speeds based on lfSpeed
  calculateSpeeds();
  
  // Speed up ADC
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);

  // Initialize motor pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  
  // Initialize button pins (with pull-up)
  pinMode(CAL_BTN, INPUT_PULLUP);
  pinMode(START_BTN, INPUT_PULLUP);

  // Enable motor driver
  digitalWrite(STBY, HIGH);

  // Configure for 5 sensors
  lineThickness = constrain(lineThickness, 10, 35);
  if (numSensors == 5) {
    // Adjust weights for 5 sensors
    sensorWeight[1] = 4;   // S2 (left middle)
    sensorWeight[2] = 2;   // S3 (center)
    sensorWeight[3] = 0;   // S4 (right middle)
    sensorWeight[4] = -2;  // S5 (right most)
    sensorWeight[5] = -4;  // Not used
  }
  
  // Brief delay for stabilization
  delay(500);
}

// ============ PROFESSIONAL AUTO SCALING FUNCTION ============
void calculateSpeeds() {
  // Constrain master speed
  lfSpeed = constrain(lfSpeed, 60, 255);

  // ================= SPEED SCALING =================
  minTurnSpeed       = lfSpeed * 0.67;
  calSpeed           = lfSpeed * 0.65;
  recoverySpeed      = lfSpeed * 0.85;
  recoveryTurnSpeed  = lfSpeed * 0.60;

  recoveryLeftTurn   = -lfSpeed * 0.45;
  recoveryRightTurn  =  lfSpeed * 0.85;

  // Constrain all speeds to valid ranges
  minTurnSpeed = constrain(minTurnSpeed, 40, 180);
  calSpeed = constrain(calSpeed, 50, 150);
  recoverySpeed = constrain(recoverySpeed, 60, 200);
  recoveryTurnSpeed = constrain(recoveryTurnSpeed, 40, 150);
  recoveryLeftTurn = constrain(recoveryLeftTurn, -120, -30);
  recoveryRightTurn = constrain(recoveryRightTurn, 60, 200);

  // Acceleration control
  accelerationStep = max(1, lfSpeed / 80);

  // Loop delay (faster bot → faster loop)
  loopDelay = constrain(20 - (lfSpeed / 15), 5, 20);

  calibrationSeconds = 4;

  // ================= ROTATION TIME SCALING =================
  /*
   * KEY IDEA:
   * Time ∝ 1 / Speed
   * So we scale using lfSpeed directly
   */

  float baseSpeed = 80.0;   // reference speed where you tuned timings

  float scale = baseSpeed / lfSpeed;

  // Clamp for safety
  scale = constrain(scale, 0.4, 2.5);

  TIME_45_DEG  = 300  * scale;
  TIME_180_DEG = 780  * scale;
  TIME_360_DEG = 1560 * scale;
}

// ============ CORRECTED 3-STAGE RECOVERY FUNCTION ============
// Stage 1: 45° towards last line side
// Stage 2: 180° opposite direction
// Stage 3: 360° SAME direction as Stage 1
void recoverySequence() {
  // HARD RESET - Always restart recovery from Stage 1
  inRecovery = true;
  
  // PERCENTAGE-BASED BOOST (not fixed offset)
  int turnSpeed = recoveryTurnSpeed * 1.2;
  turnSpeed = constrain(turnSpeed, 60, 180);

  // -------- STAGE 1: 45 DEGREE (TOWARDS LAST SIDE) --------
  unsigned long startTime = millis();
  while (millis() - startTime < TIME_45_DEG) {
    readLine();
    if (onLine) return;  // Line found, exit recovery IMMEDIATELY

    if (lastLineSide == -1) {
      // Turn LEFT (towards last line side)
      motor1run(-turnSpeed);
      motor2run(turnSpeed);
    } else {
      // Turn RIGHT (towards last line side)
      motor1run(turnSpeed);
      motor2run(-turnSpeed);
    }
  }

  // -------- STAGE 2: 180 DEGREE (OPPOSITE DIRECTION) --------
  startTime = millis();
  while (millis() - startTime < TIME_180_DEG) {
    readLine();
    if (onLine) return;  // Line found, exit recovery IMMEDIATELY

    if (lastLineSide == -1) {
      // Opposite direction = RIGHT
      motor1run(turnSpeed);
      motor2run(-turnSpeed);
    } else {
      // Opposite direction = LEFT
      motor1run(-turnSpeed);
      motor2run(turnSpeed);
    }
  }

  // -------- STAGE 3: 360 DEGREE (SAME AS STAGE 1 DIRECTION) --------
  startTime = millis();
  while (millis() - startTime < TIME_360_DEG) {
    readLine();
    if (onLine) return;  // Line found, exit recovery IMMEDIATELY

    if (lastLineSide == -1) {
      // SAME AS STAGE 1 (LEFT)
      motor1run(-turnSpeed);
      motor2run(turnSpeed);
    } else {
      // SAME AS STAGE 1 (RIGHT)
      motor1run(turnSpeed);
      motor2run(-turnSpeed);
    }
  }
  
  // RESET FLAG AFTER COMPLETING ALL STAGES
  inRecovery = false;
}

void loop() {
  // Read button states (LOW when pressed due to pull-up)
  int calState = digitalRead(CAL_BTN);
  int startState = digitalRead(START_BTN);
  
  // ============ EMERGENCY STOP ============
  if (calState == LOW && startState == LOW) {
    delay(debounceDelay);  // Debounce
    
    if (digitalRead(CAL_BTN) == LOW && digitalRead(START_BTN) == LOW) {
      // Emergency stop
      emergencyStop = true;
      isRunning = false;
      inRecovery = false;
      motor1run(0);
      motor2run(0);
      
      // Wait for buttons to be released
      while (digitalRead(CAL_BTN) == LOW || digitalRead(START_BTN) == LOW) {
        delay(10);
      }
      
      delay(100);
      emergencyStop = false;
    }
  }
  
  // ============ CALIBRATION BUTTON (D2 alone) ============
  else if (calState == LOW && startState == HIGH && !emergencyStop) {
    delay(debounceDelay);  // Debounce
    
    if (digitalRead(CAL_BTN) == LOW && digitalRead(START_BTN) == HIGH) {
      calibrate();  // Run calibration
      calibrated = true;
      
      // Wait for button release
      while (digitalRead(CAL_BTN) == LOW) {
        delay(10);
      }
      delay(100);
    }
  }
  
  // ============ START BUTTON (D3 alone) ============
  else if (startState == LOW && calState == HIGH && calibrated && !emergencyStop) {
    delay(debounceDelay);  // Debounce
    
    if (digitalRead(START_BTN) == LOW && digitalRead(CAL_BTN) == HIGH) {
      // Toggle running state
      isRunning = !isRunning;
      
      if (!isRunning) {
        motor1run(0);
        motor2run(0);
        currentSpeed = 30;     // Reset speed when stopped
        inRecovery = false;
        lastLineSide = 0;
      } else {
        currentSpeed = minTurnSpeed;  // Start at minimum speed
        inRecovery = false;
        lastLineSide = 0;
      }
      
      // Wait for button release
      while (digitalRead(START_BTN) == LOW) {
        delay(10);
      }
      delay(100);
    }
  }
  
  // ============ LINE FOLLOWING LOOP ============
  if (isRunning && calibrated && !emergencyStop) {
    readLine();
    
    // Detect last line side using edge sensors (S1,S2 = LEFT) (S4,S5 = RIGHT)
    if (sensorArray[1] || sensorArray[2]) {
      lastLineSide = -1;  // LEFT side detected
    }
    if (sensorArray[4] || sensorArray[5]) {
      lastLineSide = +1;  // RIGHT side detected
    }
    
    // Gradually increase speed with auto-adjusted acceleration
    if (currentSpeed < lfSpeed) {
      currentSpeed += accelerationStep;
    }
    
    if (onLine == 1) {
      // Line detected - normal PID following
      inRecovery = false;
      linefollow();
    } else {
      // Line lost - start fresh recovery (FORCED RESET)
      // Always start fresh recovery from beginning
      inRecovery = true;
      recoverySequence();  // CALL 3-STAGE RECOVERY
    }
    
    delay(loopDelay);  // Auto-adjusted loop delay
  }
}

void linefollow() {
  error = 0;
  activeSensors = 0;

  // Calculate weighted error using active sensors only
  for (int i = 1; i < 6; i++) {
    if (sensorArray[i]) {  // Only include sensors that detect the line
      error += sensorWeight[i] * sensorValue[i];
      activeSensors++;
    }
  }
  
  if (activeSensors > 0) {
    error = error / activeSensors;
  }

  // PID calculations
  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  // Calculate motor speeds with curve speed limiting
  lsp = currentSpeed - PIDvalue;
  rsp = currentSpeed + PIDvalue;
  
  // Apply minimum turn speed when making sharp turns
  if (abs(PIDvalue) > 50) {
    // Sharp turn detected - limit to minTurnSpeed
    if (lsp < minTurnSpeed && lsp > 0) lsp = minTurnSpeed;
    if (rsp < minTurnSpeed && rsp > 0) rsp = minTurnSpeed;
  }

  // Constrain speeds to 0-255 range
  lsp = constrain(lsp, 0, 255);
  rsp = constrain(rsp, 0, 255);

  // Apply to motors
  motor1run(lsp);
  motor2run(rsp);
}

void calibrate() {
  // Reset calibration values
  for (int i = 0; i < 7; i++) {
    minValues[i] = 1023;
    maxValues[i] = 0;
  }

  // Rotate clockwise during calibration (auto-adjusted speed)
  unsigned long startTime = millis();
  while (millis() - startTime < (unsigned long)(calibrationSeconds * 1000)) {
    motor1run(calSpeed);   // Left motor forward (auto-adjusted)
    motor2run(-calSpeed);  // Right motor backward (auto-adjusted)

    // Read all 5 sensors
    for (int i = 0; i < 5; i++) {
      int value = analogRead(i);  // Read A0-A4
      
      if (value < minValues[i]) {
        minValues[i] = value;
      }
      if (value > maxValues[i]) {
        maxValues[i] = value;
      }
    }
    delay(10);
  }

  // Stop motors
  motor1run(0);
  motor2run(0);
  delay(100);

  // Calculate thresholds
  for (int i = 0; i < 5; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
  }
}

void readLine() {
  onLine = 0;
  
  // Read sensors A0-A4
  for (int i = 0; i < 5; i++) {
    int rawValue = analogRead(i);
    
    // Map to 0-1000 based on calibration
    if (isBlackLine) {
      sensorValue[i] = map(rawValue, minValues[i], maxValues[i], 0, 1000);
    } else {
      sensorValue[i] = map(rawValue, minValues[i], maxValues[i], 1000, 0);
    }
    sensorValue[i] = constrain(sensorValue[i], 0, 1000);
    
    // Determine if sensor sees line (threshold 500)
    sensorArray[i] = (sensorValue[i] > 500);
    
    // Check if any sensor sees the line
    if (sensorArray[i]) {
      onLine = 1;
    }
  }
  
  // Shift arrays for compatibility with original code (indices 1-5)
  for (int i = 4; i >= 0; i--) {
    sensorValue[i+1] = sensorValue[i];
    sensorArray[i+1] = sensorArray[i];
    minValues[i+1] = minValues[i];
    maxValues[i+1] = maxValues[i];
    threshold[i+1] = threshold[i];
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
