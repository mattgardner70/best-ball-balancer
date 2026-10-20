// Motor test: home, go to a start angle, then swing back and forth, a little faster each loop.
//
// Directions are as seen from the front (looking at the arm, encoder at the back).
//   1. Home: creep counter-clockwise until the arm stalls (no current sensing on the MDD3A, so a
//      stall seen on the encoder is the "resistance"). That angle is 0.
//   2. Go clockwise START_DEG to the start position.
//   3. Loop: clockwise SWING_DEG, back counter-clockwise SWING_DEG. Each loop is faster, from
//      SPEED_FIRST up to MAX_SAFE_SPEED.
//
// Serial Monitor at 115200: Enter = run, H = home all + go to start, r = one motor at a time,
// h1/h2/h3 = home one motor + go to start,
// s = stop, e = encoder check,
// 1/2/3 = flip that motor's direction.
//
// Motor   MDD3A terminals   Mega PWM (A, B)   Encoder (blue, purple)
//   1     #1 M1A / M1B      D6, D7            D2, D19
//   2     #1 M2A / M2B      D8, D44           D3, D20
//   3     #2 M1A / M1B      D45, D46          D18, D21
// MDD3A header GND -> Mega GND, 5V0 unconnected. Encoder red -> 5V Wago, black -> GND Wago.

// ---- Tuning ---------------------------------------------------------------------------------
const float MAX_SAFE_SPEED = 120;      // deg/s, top speed of the swing
const float SPEED_FIRST = 20;          // deg/s, first loop
const float SPEED_STEP = 10;           // deg/s added each loop
const float ACCEL = 600;               // deg/s^2
const float START_DEG = 45, SWING_DEG = 45;
const float HOME_SPEED = 10;           // deg/s, homing creep
const int   HOME_DUTY_PCT = 45;        // most it pushes against the stop while homing (motors 2 and 3 need ~25% to break free)
const int   DUTY_MAX_PCT = 75;         // most it ever drives
const float KP = 6.0, KI = 0.03;       // position loop: duty per count, duty per count per tick
const int   FRICTION_PCT = 18;         // fixed push in the direction of the error: gets past gearbox stiction
const int   INTEG_MAX_PCT = 10;        // integral can add at most this (no wind-up lurch)
const float RUNAWAY_DEG = 15;          // tracking error that stops everything (jam or lost encoder)

// ---- Hardware -------------------------------------------------------------------------------
const float COUNTS_PER_DEG = 1320.0 / 360;   // 11 PPR x 4 edges x 30:1
const int PWM_TOP = 799;                     // 16 MHz / 800 = 20 kHz (MDD3A limit)
const int DUTY_MAX = PWM_TOP * (long)DUTY_MAX_PCT / 100;     // long: 799 x 55 overflows a 16-bit int
const int HOME_DUTY = PWM_TOP * (long)HOME_DUTY_PCT / 100;
const unsigned long TICK_US = 2000;
const float DT = TICK_US / 1e6;

struct Motor {
  const char *name;
  volatile uint16_t *ocrA, *ocrB;   // PWM compare registers for inputs A and B
  uint8_t encA, encB;
  volatile long count;
  uint8_t prev;
  int ccw;                          // drive sign (+1 = input A) that turns this arm CCW ('1'/'2'/'3' flip it)
  int sign;                         // +1 if driving CCW makes the count go up
  bool on;                          // homed and under position control
  float sp, integ;                  // setpoint (counts, CCW positive), integral term
  float worst;                      // largest tracking error this loop (counts)
};

const int N = 3;
Motor motors[N] = {
  // name, PWM A, PWM B, encoder A, B, count, prev, ccw, sign. Signs from the drive test (input A ->
  // counts: M1 -, M2 -, M3 +): fixed, because with the plate linked one arm's move turns the others.
  {"Motor 1", &OCR4A, &OCR4B, 2, 19, 0, 0, -1, 1},    // Matt: input A turns it CW
  {"Motor 2", &OCR4C, &OCR5C, 3, 20, 0, 0, 1, -1},    // input B homed it CW (Matt)
  {"Motor 3", &OCR5B, &OCR5A, 18, 21, 0, 0, -1, -1},
};

// ---- Encoders -------------------------------------------------------------------------------
static const int8_t QUAD[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
void encEdge(Motor &m) {
  uint8_t s = (digitalRead(m.encA) << 1) | digitalRead(m.encB);
  m.count += QUAD[(m.prev << 2) | s];
  m.prev = s;
}
void enc0() { encEdge(motors[0]); }
void enc1() { encEdge(motors[1]); }
void enc2() { encEdge(motors[2]); }
long raw(Motor &m) { noInterrupts(); long c = m.count; interrupts(); return c; }
float pos(Motor &m) { return m.sign * raw(m); }   // counts, CCW positive

// ---- Drive ----------------------------------------------------------------------------------
void drive(Motor &m, float ccwDuty, int limit) {  // + = CCW
  int d = constrain((int)(m.ccw * ccwDuty), -limit, limit);
  *m.ocrA = d > 0 ? d : 0;
  *m.ocrB = d < 0 ? -d : 0;
}
const char *phase = "idle";
void allOff() { for (auto &m : motors) { drive(m, 0, 0); m.on = false; } phase = "idle"; }

bool stopRequested() {
  if (!Serial.available()) return false;
  if (Serial.read() != 's') return false;
  allOff();
  Serial.println("STOP");
  return true;
}

// Every STATUS_MS: per motor, angle from home (CW +, deg), setpoint, and drive (%, + = CCW).
const unsigned long STATUS_MS = 250;
void status() {
  static unsigned long last = 0;
  if (millis() - last < STATUS_MS) return;
  last = millis();
  Serial.print(millis() / 1000.0, 2); Serial.print("s "); Serial.print(phase);
  for (auto &m : motors) {
    int d = *m.ocrA ? *m.ocrA : -(int)*m.ocrB;              // + = input A
    Serial.print(" | "); Serial.print(m.name + 6);           // "1", "2", "3"
    Serial.print(": "); Serial.print(-pos(m) / COUNTS_PER_DEG, 1);
    if (m.on) { Serial.print(" sp "); Serial.print(-m.sp / COUNTS_PER_DEG, 1); }
    Serial.print(" pwm "); Serial.print(m.ccw * d * 100L / PWM_TOP); Serial.print("%");
    Serial.print(" [A "); Serial.print(*m.ocrA); Serial.print(" B "); Serial.print(*m.ocrB);
    Serial.print(" ccw "); Serial.print(m.ccw); Serial.print(" raw "); Serial.print(raw(m)); Serial.print("]");
  }
  Serial.println();
}

// Wait for the next control tick; false if 's' was sent.
bool nextTick(unsigned long &t) {
  while (micros() - t < TICK_US) if (stopRequested()) return false;
  t += TICK_US;
  status();
  return true;
}

int driveLimit = DUTY_MAX;                    // the raise to start uses full 12 V (PWM_TOP)

// Position loop on every homed motor. False = something jammed or lost its encoder.
bool control() {
  for (auto &m : motors) {
    if (!m.on) continue;
    float err = m.sp - pos(m);
    m.worst = max(m.worst, abs(err));
    if (abs(err) > RUNAWAY_DEG * COUNTS_PER_DEG) {
      allOff();
      Serial.print(m.name); Serial.print(": tracking error "); Serial.print(err / COUNTS_PER_DEG, 1);
      Serial.println(" deg, stopped (jammed, or encoder lost)");
      return false;
    }
    const float imax = PWM_TOP * INTEG_MAX_PCT / 100.0;
    if (abs(err) < 3 * COUNTS_PER_DEG) m.integ = constrain(m.integ + KI * err, -imax, imax);   // only near the target
    if (err * m.integ < 0) m.integ *= 0.9;   // bleed the integral once it overshoots
    float fric = abs(err) > 1 ? (err > 0 ? 1 : -1) * PWM_TOP * FRICTION_PCT / 100.0 : 0;
    float u = KP * err + m.integ + fric;
    if (driveLimit == PWM_TOP && abs(err) > 3 * COUNTS_PER_DEG) u = err > 0 ? PWM_TOP : -PWM_TOP;   // raise: full 12 V while behind
    drive(m, u, driveLimit);
  }
  return true;
}

// ---- Homing ---------------------------------------------------------------------------------
// All motors creep CCW together; each stops when it stalls, and that angle becomes 0.
// only = -1 homes every motor together; 0..2 homes just that one (the others stay off).
// Open loop in the CCW direction: the encoder only measures speed (either sign) to keep the creep slow,
// and spots the stall. So a wrong encoder sign can't make it run away; homing learns the sign instead.
bool homeAll(int only = -1) {
  bool creeping[N], any = false;
  float duty[N]; long last[N], start[N]; unsigned long still[N];
  for (auto &m : motors) m.on = false;
  for (int i = 0; i < N; i++) {
    creeping[i] = only < 0 || i == only;
    any |= creeping[i];
    duty[i] = 0;
    start[i] = last[i] = raw(motors[i]);
    still[i] = millis();
  }
  if (!any) return false;
  Serial.println("Homing: creeping CCW to the stop");
  phase = "homing";
  const float target = HOME_SPEED * COUNTS_PER_DEG * 0.05;   // counts per 50 ms
  unsigned long t0 = millis(), lastCheck = millis();
  while (true) {
    if (stopRequested()) return false;
    status();
    if (millis() - lastCheck < 50) continue;
    lastCheck = millis();
    bool busy = false;
    for (int i = 0; i < N; i++) {
      if (!creeping[i]) continue;
      Motor &m = motors[i];
      long r = raw(m), moved = abs(r - last[i]);
      last[i] = r;
      duty[i] += moved < target ? PWM_TOP * 0.01 : -PWM_TOP * 0.02;   // speed up gently, back off fast
      duty[i] = constrain(duty[i], 0, HOME_DUTY);
      drive(m, duty[i], HOME_DUTY);                                  // + = CCW
      if (moved > 0 || duty[i] < HOME_DUTY) still[i] = millis();
      if (millis() - still[i] > 500) {                               // stalled at full homing push
        drive(m, 0, 0);
        creeping[i] = false;
        long travelled = r - start[i];
        if (abs(travelled) > 10) m.sign = travelled > 0 ? 1 : -1;    // CCW drive moved the count this way
        noInterrupts(); m.count = 0; interrupts();
        m.sp = m.integ = 0; m.on = true;
        Serial.print("  "); Serial.print(m.name); Serial.print(" home after ");
        Serial.print(abs(travelled) / COUNTS_PER_DEG, 1); Serial.println(" deg");
      }
      busy |= creeping[i];
    }
    if (!busy) return true;
    if (millis() - t0 > 30000) { allOff(); Serial.println("  no stop found in 30 s: stopped."); return false; }
  }
}

// ---- Moves ----------------------------------------------------------------------------------
// Move every homed motor to `deg` (CW positive, from home) with a trapezoidal speed profile.
bool moveTo(float deg, float speed) {
  float target = -deg * COUNTS_PER_DEG;              // CW is negative in CCW-positive counts
  float from = 0;
  for (auto &m : motors) if (m.on) { from = m.sp; break; }
  float dist = abs(target - from), dir = target > from ? 1 : -1;
  float v = 0, s = 0, vmax = speed * COUNTS_PER_DEG, a = ACCEL * COUNTS_PER_DEG;
  unsigned long t = micros();
  while (s < dist) {
    if (!nextTick(t)) return false;
    float stopDist = v * v / (2 * a);
    v = (dist - s <= stopDist) ? max(v - a * DT, a * DT) : min(v + a * DT, vmax);
    s = min(s + v * DT, dist);
    for (auto &m : motors) if (m.on) m.sp = from + dir * s;
    if (!control()) return false;
  }
  for (int i = 0; i < 100; i++) {                    // settle 0.2 s
    if (!nextTick(t) || !control()) return false;
  }
  return true;
}

// One motor at a time: home it, take it to the start, swing it 3 times, then off. Watch each arm:
// homing should go CCW (from the front), the start move CW. Send 1/2/3 to flip a motor that's backwards.
// The raise from home to the start position, at full 12 V.
bool raiseToStart() {
  driveLimit = PWM_TOP;
  bool ok = moveTo(START_DEG, MAX_SAFE_SPEED);
  driveLimit = DUTY_MAX;
  return ok;
}

void runSeries() {
  for (int i = 0; i < N; i++) {
    Serial.print("--- "); Serial.print(motors[i].name); Serial.println(" alone ---");
    if (!homeAll(i)) { allOff(); continue; }
    Serial.print("To start: CW "); Serial.print(START_DEG, 0); Serial.println(" deg");
    phase = "to start";
    bool ok = raiseToStart();
    phase = "swing";
    for (int n = 1; ok && n <= 3; n++) {
      ok = moveTo(START_DEG + SWING_DEG, SPEED_FIRST) && moveTo(START_DEG, SPEED_FIRST);
      if (ok) { Serial.print("  swing "); Serial.println(n); }
    }
    allOff();
    if (!ok && Serial.available() == 0) delay(500);
    delay(1000);
  }
}

void run() {
  if (!homeAll()) return;
  Serial.print("To start: CW "); Serial.print(START_DEG, 0); Serial.println(" deg");
  phase = "to start";
  if (!raiseToStart()) return;
  phase = "swing";
  Serial.println("Swinging ('s' stops)");
  float speed = SPEED_FIRST;
  for (long n = 1;; n++) {
    for (auto &m : motors) m.worst = 0;
    if (!moveTo(START_DEG + SWING_DEG, speed) || !moveTo(START_DEG, speed)) return;
    Serial.print("Loop "); Serial.print(n); Serial.print("  "); Serial.print(speed, 0); Serial.print(" deg/s  worst error:");
    for (auto &m : motors) if (m.on) { Serial.print("  "); Serial.print(m.worst / COUNTS_PER_DEG, 1); }
    Serial.println(" deg");
    speed = min(speed + SPEED_STEP, MAX_SAFE_SPEED);
  }
}

// ---- Setup ----------------------------------------------------------------------------------
void setupPwm() {
  // Timer4: D6 (A), D7 (B), D8 (C). Timer5: D46 (A), D45 (B), D44 (C). Fast PWM mode 14, TOP = ICR.
  OCR4A = OCR4B = OCR4C = OCR5A = OCR5B = OCR5C = 0;
  const uint8_t pins[] = {6, 7, 8, 44, 45, 46};
  for (uint8_t p : pins) pinMode(p, OUTPUT);
  TCCR4A = _BV(COM4A1) | _BV(COM4B1) | _BV(COM4C1) | _BV(WGM41);
  TCCR4B = _BV(WGM43) | _BV(WGM42) | _BV(CS40);
  ICR4 = PWM_TOP;
  TCCR5A = _BV(COM5A1) | _BV(COM5B1) | _BV(COM5C1) | _BV(WGM51);
  TCCR5B = _BV(WGM53) | _BV(WGM52) | _BV(CS50);
  ICR5 = PWM_TOP;
}

void setup() {
  setupPwm();                                        // all motors off before anything else
  Serial.begin(115200);
  for (auto &m : motors) {
    pinMode(m.encA, INPUT_PULLUP); pinMode(m.encB, INPUT_PULLUP);
    m.prev = (digitalRead(m.encA) << 1) | digitalRead(m.encB);
  }
  void (*isr[N])() = {enc0, enc1, enc2};
  for (int i = 0; i < N; i++) {
    attachInterrupt(digitalPinToInterrupt(motors[i].encA), isr[i], CHANGE);
    attachInterrupt(digitalPinToInterrupt(motors[i].encB), isr[i], CHANGE);
  }
  Serial.println("Motor test. POWER on, then Enter = run, r = one at a time, s = stop, e = encoder check, 1/2/3 = flip a motor.");
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c >= '1' && c <= '3') {
    Motor &m = motors[c - '1'];
    m.ccw = -m.ccw;
    m.sign = -m.sign;                                // same encoder, opposite drive: keep counts CCW-positive
    Serial.print(m.name); Serial.print(": CCW is now input "); Serial.println(m.ccw > 0 ? "A" : "B");
  }
  if (c == 'e') {
    Serial.println("Encoder counts for 10 s: turn each arm by hand");
    for (int i = 0; i < 50; i++) {
      for (auto &m : motors) { Serial.print(m.name); Serial.print(" "); Serial.print(raw(m)); Serial.print("   "); }
      Serial.println();
      delay(200);
    }
  }
  if (c == '\n' || c == 'g') { run(); allOff(); Serial.println("Motors off. Enter to run again."); }
  if (c == 'h') {                                    // h1 / h2 / h3: home just that motor, then go to start
    unsigned long t0 = millis();
    while (!Serial.available() && millis() - t0 < 2000) {}
    int i = Serial.read() - '1';
    if (i >= 0 && i < N && homeAll(i)) {
      Serial.print("To start: CW "); Serial.print(START_DEG, 0); Serial.println(" deg");
      phase = "to start";
      if (raiseToStart()) Serial.println("  at start");
    }
    allOff(); Serial.println("Motors off.");
  }
  if (c == 'a' || c == 'b') {                       // a1 / b2 ...: drive that input of one motor, 30% for 0.12 s
    unsigned long t0 = millis();
    while (!Serial.available() && millis() - t0 < 2000) {}
    int i = Serial.read() - '1';
    if (i >= 0 && i < N) {
      Motor &m = motors[i];
      long c0 = raw(m);
      volatile uint16_t *ocr = c == 'a' ? m.ocrA : m.ocrB;
      *ocr = PWM_TOP * 30 / 100;
      delay(120);                                    // about 10-15 deg
      *ocr = 0;
      delay(200);
      Serial.print(m.name); Serial.print(" input "); Serial.print(c == 'a' ? "A" : "B");
      Serial.print(": encoder moved "); Serial.println(raw(m) - c0);
    }
  }
  if (c == 'H') {                                    // home all three together, then go to start (no swing)
    if (homeAll()) {
      Serial.print("To start: CW "); Serial.print(START_DEG, 0); Serial.println(" deg");
      phase = "to start";
      if (raiseToStart()) Serial.println("  at start");
    }
    allOff(); Serial.println("Motors off.");
  }
  if (c == 'r') { runSeries(); allOff(); Serial.println("Series done, motors off."); }
}
