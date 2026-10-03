/* ============================================================
   MESHMERIZE LINE FOLLOWER
   RP2040 + TB6612FNG + 8-Channel Digital IR Array
   Arduino IDE — select "Raspberry Pi Pico" (Earle Philhower core)
   ============================================================
   Wiring recap:
     Sensor VCC   -> 3.3V
     Sensor GND   -> GND
     Sensor IR    -> 3.3V       (keeps IR emitters ON)
     OUT1..OUT8   -> GP2..GP9
     TB6612 STBY  -> GP10
     TB6612 AIN1  -> GP11   (Motor A = LEFT)
     TB6612 AIN2  -> GP12
     TB6612 PWMA  -> GP13
     TB6612 BIN1  -> GP14   (Motor B = RIGHT)
     TB6612 BIN2  -> GP15
     TB6612 PWMB  -> GP16
     Red LED      -> GP17 -> 220R -> GND
   ============================================================ */

/* ================== USER TUNING ================== */
#define MAIN_SPEED        150    // <== CHANGE THIS (0..255). Everything scales.
#define LINE_IS_LOW         1    // 1 = sensor OUT is LOW on black line, else 0
#define KP               0.08    // PID gains (tuned at MAIN_SPEED = 150)
#define KI               0.0005
#define KD               0.30

#define JUNCTION_SENSORS    5    // >= sensors on line => junction
#define END_ZONE_SENSORS    8    // all sensors on line => end zone
#define END_ZONE_HOLD_MS  250    // must persist this long to count as end

#define TURN_90_REF_MS    250    // time to pivot ~90°  @ MAIN_SPEED=150
#define TURN_180_REF_MS   500    // time to pivot ~180° @ MAIN_SPEED=150

#define DRY_RUN_MAX_MS   170000UL  // 2m50s safety
#define ACTUAL_MAX_MS    150000UL  // 2m30s safety
#define RESET_PAUSE_MS     8000UL  // pause between Dry and Actual run
/* ================================================== */

/* ---------- Pins ---------- */
const int sensorPins[8] = {2, 3, 4, 5, 6, 7, 8, 9};
const int PWMA = 13, AIN1 = 11, AIN2 = 12;   // LEFT motor
const int PWMB = 16, BIN1 = 14, BIN2 = 15;   // RIGHT motor
const int STBY = 10;
const int LED_PIN = 17;

/* ---------- Auto-derived from MAIN_SPEED ---------- */
int   baseSpeed, maxCorrection, pivotSpeed, searchSpeed;
int   turn90ms, turn180ms;
float speedScale;

/* ---------- PID state ---------- */
int   lastError = 0;
float integral  = 0;

/* ---------- Path memory ---------- */
#define MAX_PATH 200
char path[MAX_PATH];
int  pathLength = 0;

/* ---------- Phase machine ---------- */
enum Phase { DRY_RUN, RESET_PAUSE, ACTUAL_RUN, DONE };
Phase phase = DRY_RUN;
unsigned long resetPauseStart = 0;

/* ============================================================ */
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);          // enable TB6612

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  for (int i = 0; i < 8; i++) pinMode(sensorPins[i], INPUT);

  // ---- Auto-derive everything from MAIN_SPEED ----
  baseSpeed     = MAIN_SPEED;
  maxCorrection = (int)(MAIN_SPEED * 0.85);
  pivotSpeed    = (int)(MAIN_SPEED * 0.95);
  searchSpeed   = (int)(MAIN_SPEED * 0.55);
  speedScale    = MAIN_SPEED / 150.0;
  turn90ms      = (int)(TURN_90_REF_MS  * 150.0 / MAIN_SPEED);
  turn180ms     = (int)(TURN_180_REF_MS * 150.0 / MAIN_SPEED);

  Serial.println(F("\n=== Meshmerize Line Follower ==="));
  Serial.print(F("MAIN_SPEED = ")); Serial.println(MAIN_SPEED);
  Serial.print(F("baseSpeed  = ")); Serial.println(baseSpeed);
  Serial.print(F("pivotSpeed = ")); Serial.println(pivotSpeed);
  Serial.print(F("turn90ms   = ")); Serial.println(turn90ms);
  Serial.println(F("Place bot on START. Dry run in 3s..."));
  delay(3000);
}

/* ============================================================ */
void loop() {
  switch (phase) {
    case DRY_RUN:
      Serial.println(F("--- DRY RUN ---"));
      doDryRun();
      printPath("Raw path       : ");
      simplifyPath();
      printPath("Simplified path: ");
      resetPauseStart = millis();
      phase = RESET_PAUSE;
      break;

    case RESET_PAUSE:
      stopMotors();
      digitalWrite(LED_PIN, (millis() / 250) & 1);   // blink LED
      if (millis() - resetPauseStart >= RESET_PAUSE_MS) {
        digitalWrite(LED_PIN, LOW);
        lastError = 0; integral = 0;
        Serial.println(F("--- ACTUAL RUN ---"));
        phase = ACTUAL_RUN;
      }
      break;

    case ACTUAL_RUN:
      doActualRun();
      stopMotors();
      digitalWrite(LED_PIN, HIGH);   // glow on finish
      Serial.println(F("Done."));
      phase = DONE;
      break;

    case DONE:
    default:
      stopMotors();
      digitalWrite(LED_PIN, HIGH);
      break;
  }
}

/* ============================================================
   DRY RUN — explore, record turns, stop at end zone
   ============================================================ */
void doDryRun() {
  pathLength = 0;
  lastError = 0; integral = 0;
  unsigned long startTime    = millis();
  unsigned long endZoneStart = 0;
  bool junctionReady = true;

  while (millis() - startTime < DRY_RUN_MAX_MS) {
    int onLine = countSensorsOnLine();

    /* --- END ZONE --- */
    if (onLine >= END_ZONE_SENSORS) {
      if (endZoneStart == 0) endZoneStart = millis();
      if (millis() - endZoneStart >= END_ZONE_HOLD_MS) {
        stopMotors();
        Serial.println(F("End zone reached."));
        return;
      }
    } else {
      endZoneStart = 0;
    }

    /* --- JUNCTION --- */
    if (onLine >= JUNCTION_SENSORS && junctionReady) {
      junctionReady = false;
      stopMotors();
      delay(40);

      char decision = decideAtJunction();
      if (pathLength < MAX_PATH) path[pathLength++] = decision;
      Serial.print(F("Junction -> ")); Serial.println(decision);

      executeTurn(decision);

      /* Wait until we are clearly off the junction */
      unsigned long t = millis();
      while (countSensorsOnLine() >= JUNCTION_SENSORS && millis() - t < 1200) {
        followLine();
        delay(2);
      }
    } else if (onLine < JUNCTION_SENSORS) {
      junctionReady = true;
    }

    followLine();
    delay(2);
  }
  stopMotors();
  Serial.println(F("Dry-run timeout."));
}

/* ============================================================
   ACTUAL RUN — replay stored path
   ============================================================ */
void doActualRun() {
  unsigned long startTime    = millis();
  unsigned long endZoneStart = 0;
  int  idx = 0;
  bool junctionReady = true;

  while (millis() - startTime < ACTUAL_MAX_MS) {
    int onLine = countSensorsOnLine();

    /* --- END ZONE --- */
    if (onLine >= END_ZONE_SENSORS) {
      if (endZoneStart == 0) endZoneStart = millis();
      if (millis() - endZoneStart >= END_ZONE_HOLD_MS) {
        stopMotors();
        Serial.println(F("End zone reached."));
        return;
      }
    } else {
      endZoneStart = 0;
    }

    /* --- JUNCTION --- */
    if (onLine >= JUNCTION_SENSORS && junctionReady && idx < pathLength) {
      junctionReady = false;
      stopMotors();
      delay(40);

      char dir = path[idx++];
      Serial.print(F("Replay ")); Serial.print(idx);
      Serial.print('/'); Serial.print(pathLength);
      Serial.print(F(" -> ")); Serial.println(dir);

      executeTurn(dir);

      unsigned long t = millis();
      while (countSensorsOnLine() >= JUNCTION_SENSORS && millis() - t < 1200) {
        followLine();
        delay(2);
      }
    } else if (onLine < JUNCTION_SENSORS) {
      junctionReady = true;
    }

    followLine();
    delay(2);
  }
  stopMotors();
  Serial.println(F("Actual-run timeout."));
}

/* ============================================================
   EXPLORATION STRATEGY
   Default = simple right-hand rule.
   Improve this if your maze has loops or dead-ends.
   ============================================================ */
char decideAtJunction() {
  // TODO: replace with a smarter DFS/wall-follower if needed.
  return 'R';
}

/* ============================================================
   TURN EXECUTION
   ============================================================ */
void executeTurn(char dir) {
  switch (dir) {
    case 'L': pivotTimed(-1,  1, turn90ms);  break;   // pivot left
    case 'R': pivotTimed( 1, -1, turn90ms);  break;   // pivot right
    case 'U': pivotTimed(-1,  1, turn180ms); break;   // U-turn (via left)
    case 'S': /* straight-through, do nothing */ break;
  }
  delay(30);
  lastError = 0; integral = 0;
}

void pivotTimed(int lSign, int rSign, int ms) {
  unsigned long t = millis();
  while (millis() - t < ms) {
    setMotorSpeeds(lSign * pivotSpeed, rSign * pivotSpeed);
  }
  stopMotors();
}

/* ============================================================
   PID LINE FOLLOWING
   ============================================================ */
void followLine() {
  if (!lineVisible()) {
    // Pivot towards the last seen direction of the line
    int dir = (lastError >= 0) ? 1 : -1;
    setMotorSpeeds(dir * searchSpeed, -dir * searchSpeed);
    return;
  }

  int error = getLineError();
  integral += error;
  integral = constrain(integral, -5000, 5000);
  int derivative = error - lastError;

  float correction = (KP * error + KI * integral + KD * derivative) * speedScale;
  correction = constrain(correction, -maxCorrection, maxCorrection);

  int left  = baseSpeed + correction;
  int right = baseSpeed - correction;
  left  = constrain(left,  -255, 255);
  right = constrain(right, -255, 255);

  setMotorSpeeds(left, right);
  lastError = error;
}

int getLineError() {
  long sum = 0; int count = 0;
  for (int i = 0; i < 8; i++) {
    if (sensorOnLine(i)) { sum += (i * 100); count++; }
  }
  if (count == 0) return 0;
  int avg = sum / count;
  return avg - 350;   // -350 (line far left) .. +350 (line far right)
}

/* ============================================================
   SENSOR HELPERS
   ============================================================ */
inline bool sensorOnLine(int i) {
  int v = digitalRead(sensorPins[i]);
#if LINE_IS_LOW
  return (v == LOW);
#else
  return (v == HIGH);
#endif
}

int countSensorsOnLine() {
  int c = 0;
  for (int i = 0; i < 8; i++) if (sensorOnLine(i)) c++;
  return c;
}

bool lineVisible() {
  for (int i = 0; i < 8; i++) if (sensorOnLine(i)) return true;
  return false;
}

/* ============================================================
   MOTOR CONTROL
   ============================================================ */
void setMotorSpeeds(int left, int right) {
  // LEFT motor (A)
  if (left >= 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);  analogWrite(PWMA,  left); }
  else           { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); analogWrite(PWMA, -left); }
  // RIGHT motor (B)
  if (right >= 0) { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);  analogWrite(PWMB,  right); }
  else            { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); analogWrite(PWMB, -right); }
}

void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

/* ============================================================
   PATH SIMPLIFICATION
   Rules: LR = S, RL = S, LL = U, RR = U, UU = S
   ============================================================ */
void simplifyPath() {
  bool changed = true;
  while (changed) {
    changed = false;
    for (int i = 0; i < pathLength - 1; i++) {
      char a = path[i], b = path[i + 1];
      char r = 0;
      if      ((a=='L' && b=='R') || (a=='R' && b=='L')) r = 'S';
      else if ((a=='L' && b=='L') || (a=='R' && b=='R')) r = 'U';
      else if  (a=='U' && b=='U')                        r = 'S';
      if (r) {
        path[i] = r;
        for (int j = i + 1; j < pathLength - 1; j++) path[j] = path[j + 1];
        pathLength--;
        changed = true;
        break;
      }
    }
  }
}

void printPath(const char* label) {
  Serial.print(label);
  for (int i = 0; i < pathLength; i++) Serial.print(path[i]);
  Serial.println();
}
