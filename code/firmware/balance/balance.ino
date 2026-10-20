// Ball balancer: at power-on it homes, raises to the start pose, and balances the ball as soon as
// it is on the plate (level until then).
//
// Layers (fastest first):
//   motor loop   500 Hz   each arm follows its angle setpoint (encoder PI + friction push)
//   ball loop    100 Hz   panel -> ball position/speed -> PID -> plate tilt -> inverse kinematics -> arm angles
// Gains and geometry come from the simulation (sim/params.py, sim/controller.py, sim/kinematics.py).
//
// Serial at 115200. Keys are ignored until the console sends ~balancer-hello~ (so stray bytes on the
// line can never act as keys). code/tools/console.py sends it; in the Arduino Serial Monitor
// choose "No line ending" and send ~balancer-hello~ once (a newline is the re-home key).
//   Enter  home, raise to start, then level       b  balance the ball at the centre
//   l      level (stop balancing, hold flat)      s  stop: all motors off
//   x / X  tilt test: +5 / -5 deg pitch           y / Y  tilt test: +5 / -5 deg roll
//   ?      print the ball position once       P/p D/d I/i  gains up/down 15%, 0 = I off, v = show       c  calibrate the panel axes by rolling the ball
//   1/2/3  raise that leg's trim 0.5 deg       !/@/#  lower it
//   a / f  move the target centre -x / +x 3 mm     w / z  +y / -y
//   [ / ]  turn the tilt command -/+2 deg (cancels sideways drift along a pattern's sides)
//   > / <  feed-forward lead +-20 ms (how early the pattern's turns are anticipated)
//   n      next pattern: centre, circle, figure-8, square, wander (the ball follows a moving target)
//   R      100 Hz debug stream on/off              T  arm a 1.5 s trace for the next run without USB
//   k1/k2/k3  record the ball held over motor 1/2/3 (all three set the axes and centre; switches to level)
//
// Motor   MDD3A terminals   Mega PWM (A, B)   Encoder (blue, purple)   Leg
//   1     #1 M1A / M1B      D6, D7            D2, D19                  0 deg
//   2     #1 M2A / M2B      D8, D44           D3, D20                  120 deg
//   3     #2 M1A / M1B      D45, D46          D18, D21                 240 deg
// Panel: A0-A2 one layer (x), A1-A3 the other (y), found with touch_test.
// Arm directions as seen from the front (arm side): CCW = down (homing), CW = up.

#include <math.h>
#include <EEPROM.h>

// ---- Geometry (mm, deg), from sim/params.py --------------------------------------------------
const float MOTOR_R = 30.0, MOTOR_Z = 40.0, ARM_L = 35.0, ROD_L = 59.0, JOINT_R = 65.0;
const float LEG_DEG[3] = {0, 120, 240};
const float HOME_ARM_DEG = -25.0;      // arm angle at the low stop (sim's arm_min); CW from home adds to it
const float ARM_MIN_DEG = -20.0, ARM_MAX_DEG = 40.0;   // never command past these (stop + margin)
float LEG_OFFSET_DEG[3] = {8.5, 6.6, -5.1};             // per-leg trim, degrees CW (up), for a level plate (Matt)
                                                       // live: 1/2/3 raise that leg 0.5 deg, !/@/# lower it
const float PLATE_Z = MOTOR_Z + ROD_L; // rod-end plane height with arms level

// ---- Panel calibration (raw 0-1023 -> mm from centre), from the corner presses ---------------
// x raw ~870 at the left edge, ~135 at the right; y raw ~105 at the bottom, ~945 at the top.
const float PX0 = 528, PY0 = 543;                 // centre
const float PX_MM = -141.0 / (870 - 135);         // mm per count, + = right (the panel's SHORT side)
const float PY_MM = 187.0 / (945 - 105);          // mm per count, + = top (the LONG side, toward motor 1)
// Panel (right, top) -> plate frame (x toward leg 1, y 90 deg CCW from it): the plate's +x and +y
// directions as unit vectors in panel (right, top). 'c' measures them by rolling the ball.
// Measured with k1/k2/k3 (ball held over each motor; re-derived after swapping the panel scales).
// The earlier hand-set x axis was backwards, which is why motor 1's side never pushed back.
float EX_R = 0.16, EX_T = -0.99, EY_R = -1.00, EY_T = -0.07;
float CX = 16.3, CY = 5.7;                        // panel (right, top) mm of the plate centre (k sets it)
const int TOUCH_Z = 600;                          // touched if the pressure reading is below this (ball ~40)

// ---- Ball loop, from sim/params.py Control ---------------------------------------------------
const float LONG_GAIN = 1.0;                               // per-axis trim (the real fix was swapping the panel scales)
float BALL_KP = 6.08, BALL_KI = 9.23, BALL_KD = 3.02;   // live: P/p D/d I/i (15% steps), 0 = I off, v = show   // KI up: it has to cancel a steady lean   // m/s^2 per m, per m*s, per m/s (KD up from the sim's 3.2: it overshot)
const float MAX_TILT_DEG = 8, MAX_TILT_RATE = 120;         // deg, deg/s (first run: 12/40 was slow, then overshot)
const float TILT_SMOOTH = 0.6;                             // fraction of the way to the wanted tilt each 10 ms
const float G_ROLL = 5.0 / 7.0 * 9.81;
const float AB_ALPHA = 0.4, AB_BETA = 0.05;                // alpha-beta tracker: the real panel is clean (sim assumed noise)

// ---- Motor loop (proven in jog_test) ---------------------------------------------------------
const float HOME_SPEED = 10, RAISE_SPEED = 120, ACCEL = 600;
const int   HOME_DUTY_PCT = 45, DUTY_MAX_PCT = 100;   // full 12 V: motor 1 lifts its corner alone
const float KP = 10.0, KI = 0.05;      // stiffer than jog_test: it has to hold the plate up
float KD_M = 5.0;                      // damping: duty per count moved in 10 ms (100 deg/s -> ~23%); live M/m
const int   HOLD_PCT = 15;             // constant upward push: carries the plate's weight
const int   FRICTION_PCT = 18, INTEG_MAX_PCT = 100;   // integral can carry a leg's full load (up to the drive cap)
const float CW_MIN = 3, CW_MAX = 50;   // never command an arm outside this (deg up from home)
const float CW_CUT = 55;               // measured past this: drive cut (a bad home can't over-extend an arm)
const unsigned long DEAD_MS = 3000;    // 90%+ drive, >15 deg off target, no encoder change this long: rest 1 s
const float RUNAWAY_DEG = 25;          // this far behind: warn (and after 5 s, ease off)

const float COUNTS_PER_DEG = 1320.0 / 360;
const int PWM_TOP = 799;
const int DUTY_MAX = PWM_TOP * (long)DUTY_MAX_PCT / 100;   // long: 799 x 75 overflows a 16-bit int
const int HOME_DUTY = PWM_TOP * (long)HOME_DUTY_PCT / 100;
const unsigned long TICK_US = 2000;
const float DT = TICK_US / 1e6;
const int BALL_EVERY = 5;              // ball loop every 5 motor ticks = 100 Hz
const float BALL_DT = DT * BALL_EVERY;

struct Motor {
  const char *name;
  volatile uint16_t *ocrA, *ocrB;
  uint8_t encA, encB;
  volatile long count;
  uint8_t prev;
  int ccw;                             // drive sign (+1 = input A) that turns this arm CCW (down)
  int sign;                            // +1 if driving CCW makes the count go up (from the pulse tests)
  bool on;
  float sp, integ;                     // setpoint (counts, CCW positive), integral
};

const int N = 3;
Motor motors[N] = {
  // ccw, sign from the open-loop pulse tests (Matt watching):
  //   M1: input A = CW, counts down     -> CCW = B, counts up
  //   M2: input A = CCW, counts down
  //   M3: input A counts up, B down; CCW = B -> counts down
  {"Motor 1", &OCR4A, &OCR4B, 2, 19, 0, 0, -1, 1},
  {"Motor 2", &OCR4C, &OCR5C, 3, 20, 0, 0, 1, -1},
  {"Motor 3", &OCR5B, &OCR5A, 18, 21, 0, 0, -1, -1},
};

// ---- Encoders --------------------------------------------------------------------------------
static const int8_t QUAD[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
volatile unsigned int badEdges[3] = {0, 0, 0};   // both channels flipped at once = noise or a missed edge
void encEdge(Motor &m) {
  uint8_t s = (digitalRead(m.encA) << 1) | digitalRead(m.encB);
  if ((m.prev ^ s) == 3) badEdges[&m - motors]++;
  m.count += QUAD[(m.prev << 2) | s];
  m.prev = s;
}
void enc0() { encEdge(motors[0]); }
void enc1() { encEdge(motors[1]); }
void enc2() { encEdge(motors[2]); }
long raw(Motor &m) { noInterrupts(); long c = m.count; interrupts(); return c; }
float pos(Motor &m) { return m.sign * raw(m); }
float cwDeg(Motor &m) { return -pos(m) / COUNTS_PER_DEG; }   // degrees CW (up) from home

// ---- Drive -----------------------------------------------------------------------------------
void drive(Motor &m, float ccwDuty, int limit) {
  int d = constrain((int)(m.ccw * ccwDuty), -limit, limit);
  *m.ocrA = d > 0 ? d : 0;
  *m.ocrB = d < 0 ? -d : 0;
}
enum Mode { IDLE, LEVEL, BALANCE };
Mode mode = IDLE;
void allOff() { for (auto &m : motors) { drive(m, 0, 0); m.on = false; } mode = IDLE; }

char pending = 0;                      // a command read while running, handled by loop()
// Stray bytes must never act as keys (gain changes, tilts, re-homes),
// so keys are ignored until the console sends the exact hello phrase.
const char HELLO[] = "~balancer-hello~";
bool consoleOn = false;
unsigned int strayBytes = 0;
byte maxErrDeg[3] = {0, 0, 0};          // diagnostics while balancing
unsigned long maxGapUs = 0;            // bytes received before the hello (noise when unplugged)
void diagSave();
void printTrace();
const int TRACE_AT = 600, TRACE_N = 150;
int readKey() {
  static byte matched = 0;
  diagSave();
  while (Serial.available()) {
    char c = Serial.read();
    if (consoleOn) return c;
    strayBytes++;
    matched = c == HELLO[matched] ? matched + 1 : (c == HELLO[0] ? 1 : 0);
    if (!HELLO[matched]) {
      consoleOn = true; Serial.println("Console connected - keys on");
      if (EEPROM.read(TRACE_AT - 2) == TRACE_N) { printTrace(); EEPROM.update(TRACE_AT - 2, 0); }   // shown once
    }
  }
  return -1;
}
bool stopRequested() {
  if (pending) return false;             // one key at a time; the rest wait in the buffer
  int k = readKey();
  if (k < 0) return false;
  char c = k;
  if (c == 's') { allOff(); Serial.println("STOP"); return true; }
  if (c != '\n' && c != '\r') pending = c;
  return false;
}

int driveLimit = DUTY_MAX;
bool raising = false;                  // the start-up raise: full push and a duty ramp, never while balancing
const float SLEW = PWM_TOP / 25.0;     // raise: max duty rise per 2 ms tick: 0 -> 100% in 50 ms
bool brownedOut = false;
// Boot log in EEPROM (survives power-off): how many boots, and for the last 16 how far each got
// (0 booted, 1 homed, 2 raised, 3 still running 30 s after boot). Many boots stuck at 1 = it resets on the raise.
const int LOG_N = 16;
byte logSlot;
void logStage(byte st) { EEPROM.update(4 + logSlot, st); }
void bootLog(byte cause) {
  unsigned int n; EEPROM.get(0, n); if (n == 0xFFFF) n = 0;
  n++; EEPROM.put(0, n);
  byte last = EEPROM.read(2); if (last >= LOG_N) last = LOG_N - 1;
  logSlot = (last + 1) % LOG_N; EEPROM.update(2, logSlot);
  Serial.print("Boot #"); Serial.print(n); Serial.print("  reset flags 0x"); Serial.println(cause, HEX);
  Serial.print("Earlier boots, oldest first (0 booted 1 homed 2 raised 3 balancing): ");
  for (int k = 1; k < LOG_N; k++) {
    byte st = EEPROM.read(4 + (logSlot + k) % LOG_N);
    Serial.print(st == 0xFF ? '-' : char('0' + st)); Serial.print(' ');
  }
  Serial.println();
  for (int k = LOG_N - 3; k < LOG_N; k++) {          // the last 3 boots in detail
    int a = 40 + (logSlot + k) % LOG_N * 32;
    unsigned int f, sf, sb; int mz; float mr;
    EEPROM.get(a, f); EEPROM.get(a + 2, sf); EEPROM.get(a + 4, mz); EEPROM.get(a + 6, mr); EEPROM.get(a + 10, sb);
    Serial.print("  boot -"); Serial.print(LOG_N - k); Serial.print(": ran "); Serial.print(EEPROM.read(a + 13));
    Serial.print(" s, mode "); Serial.print(EEPROM.read(a + 12)); Serial.print(" (0 idle 1 level 2 balance), stray serial bytes "); Serial.print(sb);
    Serial.print(", balance frames "); Serial.print(f); Serial.print(", ball seen "); Serial.print(sf);
    Serial.print(", lowest z "); Serial.print(mz); Serial.print(", farthest ball mm "); Serial.println(mr, 0);
    unsigned long gap; EEPROM.get(a + 18, gap);
    Serial.print("      worst motor error deg (balancing) "); for (int i = 0; i < N; i++) { Serial.print(EEPROM.read(a + 14 + i)); Serial.print(' '); }
    Serial.print(", longest control gap us "); Serial.print(gap);
    Serial.print(", bad encoder edges "); for (int i = 0; i < N; i++) { unsigned int b; EEPROM.get(a + 22 + 2 * i, b); Serial.print(b); Serial.print(' '); }
    Serial.println();
  }
  logStage(0);
  for (int k = 0; k < 32; k++) EEPROM.update(40 + logSlot * 32 + k, 0);   // fresh record for this boot
}
// A motor that falls far behind keeps pushing toward its target; it only eases off to 40% after
// STUCK_MS stuck, so a jam can't cook it. Nothing is switched off.
const unsigned long STUCK_MS = 5000;
const int STUCK_DUTY = PWM_TOP * 40L / 100;
unsigned long behindSince[N] = {0, 0, 0}, lastWarn = 0;
bool control() {
  static unsigned long restUntil[N] = {0, 0, 0}, lastUs = 0;
  unsigned long nowUs = micros();
  if (mode == BALANCE && lastUs) maxGapUs = max(maxGapUs, nowUs - lastUs);
  lastUs = nowUs;
  for (int i = 0; i < N; i++) {
    Motor &m = motors[i];
    if (restUntil[i] && millis() > restUntil[i]) { restUntil[i] = 0; m.on = true; m.integ = 0; }   // back on after a rest
    if (!m.on) continue;
    float err = m.sp - pos(m);
    if (mode == BALANCE) maxErrDeg[i] = max(maxErrDeg[i], (byte)min(255.0, abs(err) / COUNTS_PER_DEG));
    int limit = driveLimit;
    // Dead encoder: pushed hard but the count never changes -> it can't see the arm, so stop it.
    static long lastRaw[N]; static unsigned long sinceMove[N];
    long r = raw(m);
    if (r != lastRaw[i] || (*m.ocrA + *m.ocrB) < PWM_TOP * 9L / 10 || abs(err) < 15 * COUNTS_PER_DEG) { lastRaw[i] = r; sinceMove[i] = millis(); }
    if (millis() - sinceMove[i] > DEAD_MS) {
      drive(m, 0, 0); m.on = false; restUntil[i] = millis() + 1000; sinceMove[i] = millis() + 1000;
      Serial.print(m.name); Serial.println(": full push, no encoder movement - resting 1 s (loose encoder wire?)");
      continue;
    }
    if (cwDeg(m) > CW_CUT) {               // over-extended: never push it further up
      drive(m, 0, 0);
      if (millis() - lastWarn > 1000) { lastWarn = millis(); Serial.print(m.name); Serial.println(": past 55 deg, drive cut (re-home with Enter)"); }
      continue;
    }
    if (abs(err) > RUNAWAY_DEG * COUNTS_PER_DEG) {
      if (!behindSince[i]) behindSince[i] = millis();
      if (millis() - behindSince[i] > STUCK_MS) limit = min(limit, STUCK_DUTY);
      if (millis() - lastWarn > 1000) {
        lastWarn = millis();
        Serial.print(m.name); Serial.print(": "); Serial.print(abs(err) / COUNTS_PER_DEG, 0);
        Serial.println(limit < driveLimit ? " deg behind for 5 s+, easing to 40%" : " deg behind, still pushing");
      }
    } else behindSince[i] = 0;
    const float imax = PWM_TOP * INTEG_MAX_PCT / 100.0;
    if (abs(err) < 6 * COUNTS_PER_DEG) m.integ = constrain(m.integ + KI * err, -imax, imax);
    if (err * m.integ < 0) m.integ *= 0.9;
    // friction kick fades in over the first 2 deg, so it doesn't flip +-18% around the target like a relay
    float fric = constrain(err / (2 * COUNTS_PER_DEG), -1.0, 1.0) * PWM_TOP * FRICTION_PCT / 100.0;
    static float hist[N][5]; static byte h = 0;            // position 10 ms ago, for the damping term
    float p = pos(m), moved = p - hist[i][h];
    hist[i][h] = p;
    if (i == N - 1) h = (h + 1) % 5;
    float u = KP * err + m.integ + fric - KD_M * moved - PWM_TOP * HOLD_PCT / 100.0;   // up (CW) is negative CCW duty
    if (raising && abs(err) > 3 * COUNTS_PER_DEG) u = err > 0 ? PWM_TOP : -PWM_TOP;
    if (raising) {                          // raise only: ramp the duty up, no instant 0->100% surge
      float cur = (*m.ocrA - (float)*m.ocrB) * m.ccw;
      if (u * cur < 0) cur = 0;
      if (abs(u) > abs(cur) + SLEW) u = cur + (u > 0 ? SLEW : -SLEW);
    }
    drive(m, u, limit);
  }
  return true;
}

void setCw(Motor &m, float deg) { m.sp = -constrain(deg, CW_MIN, CW_MAX) * COUNTS_PER_DEG; }

// ---- Homing and the raise (from jog_test) ----------------------------------------------------
bool homeAll() {
  bool creeping[N];
  float duty[N]; long last[N], start[N]; unsigned long still[N];
  for (int i = 0; i < N; i++) {
    motors[i].on = false; creeping[i] = true; duty[i] = 0;
    start[i] = last[i] = raw(motors[i]); still[i] = millis();
  }
  Serial.println("Homing");
  const float target = HOME_SPEED * COUNTS_PER_DEG * 0.05;
  unsigned long t0 = millis(), lastCheck = millis();
  while (true) {
    if (stopRequested()) return false;
    if (millis() - lastCheck < 50) continue;
    lastCheck = millis();
    bool busy = false;
    for (int i = 0; i < N; i++) {
      if (!creeping[i]) continue;
      Motor &m = motors[i];
      long r = raw(m), moved = abs(r - last[i]);
      last[i] = r;
      duty[i] = constrain(duty[i] + (moved < target ? PWM_TOP * 0.01 : -PWM_TOP * 0.02), 0, HOME_DUTY);
      drive(m, duty[i], HOME_DUTY);
      if (moved > 0 || duty[i] < HOME_DUTY) still[i] = millis();
      if (millis() - still[i] > 500) {
        drive(m, 0, 0);
        creeping[i] = false;
        long travelled = r - start[i];
        // Signs are fixed (pulse tests): with the plate linking the arms, the other motors can push
        // this one while it creeps, so its count direction during homing proves nothing.
        if (abs(travelled) > 30 && (travelled > 0) != (m.sign > 0)) {
          Serial.print("  "); Serial.print(m.name); Serial.println(": counted the other way while homing (pushed by the plate?)");
        }
        noInterrupts(); m.count = 0; interrupts();
        m.sp = m.integ = 0; m.on = true;
        Serial.print("  "); Serial.print(m.name); Serial.println(" home");
      }
      busy |= creeping[i];
    }
    if (!busy) break;
    if (millis() - t0 > 30000) { allOff(); Serial.println("  no stop found in 30 s"); return false; }
  }
  // Confirm: all three push down together so the plate seats and every arm reaches its real stop
  // (one arm can stall against the plate early while the others are still up), then zero them.
  for (auto &m : motors) { m.on = false; drive(m, HOME_DUTY, HOME_DUTY); }
  unsigned long c0 = millis();
  while (millis() - c0 < 800) if (stopRequested()) return false;
  for (auto &m : motors) { drive(m, 0, 0); }
  delay(150);
  for (auto &m : motors) {
    long extra = abs(raw(m));
    noInterrupts(); m.count = 0; interrupts();
    m.sp = m.integ = 0; m.on = true;
    if (extra > 3 * COUNTS_PER_DEG) { Serial.print("  "); Serial.print(m.name); Serial.print(" went "); Serial.print(extra / COUNTS_PER_DEG, 0); Serial.println(" deg further down on the confirm push"); }
  }
  Serial.println("  homed");
  return true;
}

// Move every arm to cwDeg (CW from home) together, trapezoidal speed profile.
bool moveAllTo(const float target[N], float speed) {
  float from[N], dist = 0;
  for (int i = 0; i < N; i++) { from[i] = cwDeg(motors[i]); dist = max(dist, abs(target[i] - from[i])); }
  float v = 0, s = 0;
  unsigned long t = micros();
  while (s < dist) {
    while (micros() - t < TICK_US) if (stopRequested()) return false;
    t += TICK_US;
    float stopDist = v * v / (2 * ACCEL);
    v = (dist - s <= stopDist) ? max(v - ACCEL * DT, ACCEL * DT) : min(v + ACCEL * DT, speed);
    s = min(s + v * DT, dist);
    for (int i = 0; i < N; i++) setCw(motors[i], from[i] + (target[i] - from[i]) * s / dist);
    if (!control()) return false;
  }
  return true;
}

// ---- Inverse kinematics (sim/kinematics.py) --------------------------------------------------
// Arm angles (deg) for plate roll (about x) and pitch (about y), rod-end plane at PLATE_Z.
// One leg's arm angle (deg) for a tilt given as cos/sin of roll and pitch; false if out of reach.
bool legAngle(int i, float cr, float sr, float cp, float sp, float &out) {
  static float legC[N], legS[N]; static bool init = false;   // leg directions never change: compute once
  if (!init) { for (int j = 0; j < N; j++) { legC[j] = cos(LEG_DEG[j] * DEG_TO_RAD); legS[j] = sin(LEG_DEG[j] * DEG_TO_RAD); } init = true; }
  float ux = legC[i], uy = legS[i];
  float px = JOINT_R * ux, py = JOINT_R * uy;
  // R = Ry(pitch) * Rx(roll) applied to (px, py, 0)
  float y1 = cr * py, z1 = sr * py;
  float jx = cp * px + sp * z1, jy = y1, jz = -sp * px + cp * z1 + PLATE_Z;
  float dx = jx - MOTOR_R * ux, dy = jy - MOTOR_R * uy, dz = jz - MOTOR_Z;
  float du = dx * ux + dy * uy;
  float k = (dx * dx + dy * dy + dz * dz + ARM_L * ARM_L - ROD_L * ROD_L) / (2 * ARM_L);
  float r = sqrt(du * du + dz * dz);
  float c = k / r;
  if (c < -1 || c > 1) return false;
  out = (atan2(dz, du) - acos(c)) * RAD_TO_DEG;
  return true;
}
bool armAngles(float roll, float pitch, float out[N]) {
  float cr = cos(roll), sr = sin(roll), cp = cos(pitch), sp = sin(pitch);
  for (int i = 0; i < N; i++) if (!legAngle(i, cr, sr, cp, sp, out[i])) return false;
  return true;
}

// Arm angle -> degrees CW from home for leg i, clamped to the safe range, plus that leg's trim.
float armToCw(float armDeg, int i) { return constrain(armDeg, ARM_MIN_DEG, ARM_MAX_DEG) - HOME_ARM_DEG + LEG_OFFSET_DEG[i]; }

// ---- Panel -----------------------------------------------------------------------------------
const uint8_t XA = A0, XB = A2, YA = A1, YB = A3;
void floatPanel() { const uint8_t p[4] = {XA, XB, YA, YB}; for (uint8_t q : p) { pinMode(q, INPUT); digitalWrite(q, LOW); } }
int readAxis(uint8_t hi, uint8_t lo, uint8_t sense) {
  floatPanel();
  pinMode(hi, OUTPUT); digitalWrite(hi, HIGH);
  pinMode(lo, OUTPUT); digitalWrite(lo, LOW);
  delayMicroseconds(30);
  analogRead(sense);
  int v = analogRead(sense);
  floatPanel();
  return v;
}
int minZ = 1023;                        // diagnostics: lowest touch reading, frames that saw a ball
unsigned int seenFrames = 0, panelFrames = 0;
float maxR = 0;
// Once a second for the first 90 s: what the panel saw, stray serial bytes, the mode. Shown next boot.
void diagSave() {
  static unsigned long last = 0;
  if (last || millis() < 20000) return;   // once, 20 s in (writing every second stalled the loops ~10 ms)
  last = millis();
  int a = 40 + logSlot * 32;             // one 16-byte record per boot, so a later boot can't overwrite it
  EEPROM.put(a, panelFrames); EEPROM.put(a + 2, seenFrames); EEPROM.put(a + 4, minZ); EEPROM.put(a + 6, maxR);
  EEPROM.put(a + 10, strayBytes); EEPROM.update(a + 12, (byte)mode); EEPROM.update(a + 13, (byte)(millis() / 1000));
  for (int i = 0; i < N; i++) EEPROM.update(a + 14 + i, maxErrDeg[i]);
  EEPROM.put(a + 18, maxGapUs);
  for (int i = 0; i < N; i++) { noInterrupts(); unsigned int b = badEdges[i]; interrupts(); EEPROM.put(a + 22 + 2 * i, b); }
}
bool readPanel(float &right, float &top) {
  floatPanel();
  pinMode(XA, OUTPUT); digitalWrite(XA, LOW);
  pinMode(YA, INPUT_PULLUP);
  delayMicroseconds(50);
  analogRead(YA);
  int z = analogRead(YA);
  floatPanel();
  if (z < minZ) minZ = z;
  if (z > TOUCH_Z) return false;
  right = (readAxis(XA, XB, YA) - PX0) * PX_MM;
  top = (readAxis(YA, YB, XA) - PY0) * PY_MM;
  return true;
}
bool readBall(float &xMm, float &yMm) {
  float r, t;
  if (!readPanel(r, t)) return false;
  r -= CX; t -= CY;
  xMm = r * EX_R + t * EX_T;
  yMm = r * EY_R + t * EY_T;
  return true;
}

// ---- Ball loop -------------------------------------------------------------------------------
float tiltRoll = 0, tiltPitch = 0;      // rad, smoothed command
float bx = 0, by = 0, bvx = 0, bvy = 0, ix = 0, iy = 0;
bool tracking = false;
float testRoll = 0, testPitch = 0;      // tilt test offsets (rad)

// Trace: the first 1.5 s of balancing (every 10 ms: each motor's angle, target, drive %), saved to
// EEPROM, then the motors stop. Off unless armed with the T key; records on the next run without the console.

int8_t trace[TRACE_N][9];
void traceStep() {
  static int n = 0;
  static int8_t armed = -1;
  if (armed < 0) armed = EEPROM.read(TRACE_AT - 2) == 1;   // 1 = armed (read once, EEPROM is slow-ish)
  if (consoleOn || n >= TRACE_N || !armed) return;
  for (int i = 0; i < N; i++) {
    Motor &m = motors[i];
    int d = *m.ocrA ? (int)*m.ocrA : -(int)*m.ocrB;
    trace[n][3 * i] = constrain(cwDeg(m), -127, 127);
    trace[n][3 * i + 1] = constrain(-m.sp / COUNTS_PER_DEG, -127, 127);
    trace[n][3 * i + 2] = -m.ccw * d * 100L / PWM_TOP;
  }
  if (++n < TRACE_N) return;
  allOff();
  for (int j = 0; j < TRACE_N; j++) for (int c = 0; c < 9; c++) EEPROM.update(TRACE_AT + j * 9 + c, trace[j][c]);
  EEPROM.update(TRACE_AT - 2, TRACE_N);
  Serial.println("Trace saved, motors off");
}
void printTrace() {
  if (EEPROM.read(TRACE_AT - 2) != TRACE_N) { Serial.println("No trace saved"); return; }
  Serial.println("Trace, first 1.5 s of balancing. ms | per motor: angle/target drive% (+ = up)");
  for (int j = 0; j < TRACE_N; j++) {
    Serial.print(j * 10);
    for (int i = 0; i < N; i++) {
      Serial.print(i ? "   " : " | ");
      Serial.print((int8_t)EEPROM.read(TRACE_AT + j * 9 + 3 * i)); Serial.print('/');
      Serial.print((int8_t)EEPROM.read(TRACE_AT + j * 9 + 3 * i + 1)); Serial.print(' ');
      Serial.print((int8_t)EEPROM.read(TRACE_AT + j * 9 + 3 * i + 2));
    }
    Serial.println();
  }
}
// ---- Patterns: where the ball is steered (sim/modes.py). Live: n = next pattern. -------------
const char *const PATTERNS[] = {"centre", "circle", "figure-8", "square", "wander"};
const int N_PATTERNS = 5;
int pattern = 1;                       // starts in circle (Matt)
float bdt = BALL_DT;                   // s: measured ball-step interval
float patternT = 0;                    // s; advanced exactly BALL_DT per ball step, so the target's
                                       // differenced speed and acceleration are smooth (millis() made them noisy)
float minJerk(float s) { s = constrain(s, 0, 1); return s * s * s * (10 - 15 * s + 6 * s * s); }
float AXIS_ROT_DEG = -8;                // live [ / ]: turns the tilt command to cancel sideways drift
float FF_LEAD = 0.08;                  // s: the feed-forward uses the target this far ahead (actuation lag); live > / <
void patternAt(float t, float &x, float &y);
float CENTRE_X = 0, CENTRE_Y = 0.015;       // m: where the plate's real centre reads on the panel; live a/f (x), w/z (y)
void patternTarget(float &x, float &y) {          // m, plate frame: the target now
  patternAt(patternT, x, y);
  x += CENTRE_X; y += CENTRE_Y;
  patternT += bdt;
}
void patternAt(float t, float &x, float &y) {
  float ease = minJerk(t / 2.0);                   // glide out from the centre over 2 s
  x = y = 0;
  switch (pattern) {
    case 1: { float w = TWO_PI / 3.5; x = 0.045 * cos(w * t); y = 0.045 * sin(w * t); } break;
    case 2: { float w = TWO_PI / 5.5; x = 0.060 * sin(w * t); y = 0.035 * sin(2 * w * t); } break;
    case 3: {                                     // square, equal sides, clear of the narrow edges: hold at each
      const float H = 0.045, MOVE = 1.0, HOLD = 1.5;   // corner HOLD s, then dash to the next in MOVE s
      static const float C[5][2] = {{-1, 1}, {1, 1}, {1, -1}, {-1, -1}, {-1, 1}};
      float u = fmod(t, 4 * (HOLD + MOVE));
      int i = u / (HOLD + MOVE);
      float k = minJerk((u - i * (HOLD + MOVE) - HOLD) / MOVE);      // 0 while holding
      x = H * (C[i][0] + (C[i + 1][0] - C[i][0]) * k); y = H * (C[i][1] + (C[i + 1][1] - C[i][1]) * k);
    } break;
    case 4:
      x = 0.035 * sin(0.75 * t) + 0.020 * sin(1.9 * t + 1.0);
      y = 0.022 * sin(1.1 * t + 0.5) + 0.012 * sin(2.6 * t + 2.0);
      break;
  }
  x *= ease; y *= ease;
}

unsigned long panelUs = 0, ikUs = 0;
bool streaming = false;
void streamLine(bool seen, float zx, float zy);
// The panel read (~1.2 ms) gets its own motor tick, so no tick has to fit the read and the math.
bool panelSeen = false; float panelX = 0, panelY = 0;
void panelStep() {
  unsigned long tp = micros();
  panelSeen = mode == BALANCE && readBall(panelX, panelY);
  panelUs = micros() - tp;
}
const unsigned long HOLD_MS = 150;      // ride through panel dropouts this long
const float GATE_M = 0.012;             // a reading this far (m) from the prediction is a glitch: the ball
                                       // moves < 5 mm a step, panel glitches jump 20-40 mm for one frame
unsigned long coasted = 0, rejected = 0;   // frames coasted / readings thrown out, since the last 'v'
float tcr = 1, tsr = 0, tcp = 1, tsp = 0;   // cos/sin of the current tilt, for legStep
void ballStep() {
  static unsigned long lastUs = 0;       // the real time since the last step (ticks run long: ~15 ms, not 10)
  unsigned long nowUs = micros();
  bdt = lastUs ? constrain((nowUs - lastUs) * 1e-6, 0.005, 0.04) : BALL_DT;
  lastUs = nowUs;
  // Target and feed-forward update every step, seen or not (skipping them on a dropped frame made a kick).
  float tx, ty;                        // target from the pattern, and its velocity
  static float ltx = 0, lty = 0;
  patternTarget(tx, ty);
  float tvx = (tx - ltx) / bdt, tvy = (ty - lty) / bdt;
  ltx = tx; lty = ty;
  // Feed-forward: the acceleration of the target FF_LEAD ahead, so the tilt arrives on time.
  float fx, fy;
  static float lfx = 0, lfy = 0, lfvx = 0, lfvy = 0;
  patternAt(patternT + FF_LEAD, fx, fy);
  float fvx = (fx - lfx) / bdt, fvy = (fy - lfy) / bdt;
  float tax = (fvx - lfvx) / bdt, tay = (fvy - lfvy) / bdt;
  lfx = fx; lfy = fy; lfvx = fvx; lfvy = fvy;
  float wantRoll = testRoll, wantPitch = testPitch;
  float zx, zy;
  bool seen = panelSeen; zx = panelX; zy = panelY;   // read on the tick before (panelStep)
  if (mode == BALANCE) { panelFrames++; if (seen) { seenFrames++; maxR = max(maxR, sqrt(zx * zx + zy * zy)); } }
  static bool logged3 = false;
  if (mode == BALANCE) traceStep();
  if (!logged3 && mode == BALANCE && millis() > 30000) { logStage(3); logged3 = true; }
  // Missed frames: near the panel edge a reading can drop out or jump. A gap under HOLD_MS coasts on
  // the tracker's prediction with the last tilt; a reading far from the prediction counts as a miss.
  static unsigned long lastSeen = 0;
  static float holdRoll = 0, holdPitch = 0;
  bool fresh = millis() - lastSeen < HOLD_MS;
  if (seen && tracking && fresh) {
    float px = bx + bvx * bdt - zx / 1000, py = by + bvy * bdt - zy / 1000;
    if (px * px + py * py > GATE_M * GATE_M) { seen = false; rejected++; }
  }
  if (!seen && tracking && fresh) {      // coast: predict the ball, keep the tilt
    bx += bvx * bdt; by += bvy * bdt;
    wantRoll = holdRoll; wantPitch = holdPitch;
    coasted++;
  } else if (seen) {
    zx /= 1000; zy /= 1000;              // m
    lastSeen = millis();
    if (!tracking) { bx = zx; by = zy; bvx = bvy = 0; ix = iy = 0; tracking = true; }
    float px = bx + bvx * bdt, py = by + bvy * bdt;
    float rx = zx - px, ry = zy - py;
    bx = px + AB_ALPHA * rx; by = py + AB_ALPHA * ry;
    bvx += AB_BETA / bdt * rx; bvy += AB_BETA / bdt * ry;
    float ex = tx - bx, ey = ty - by;
    ix = constrain(ix + ex * bdt, -0.12, 0.12); iy = constrain(iy + ey * bdt, -0.12, 0.12);   // up to ~1.5 deg of trim
    float ax = tax + BALL_KP * ex + BALL_KI * ix - BALL_KD * (bvx - tvx);
    float ay = tay + LONG_GAIN * (BALL_KP * ey + BALL_KI * iy - BALL_KD * (bvy - tvy));   // plate y = the panel's wide side
    {                                    // rotate the command: the ball's response sits AXIS_ROT_DEG off the command
      float c = cos(AXIS_ROT_DEG * DEG_TO_RAD), sn = sin(AXIS_ROT_DEG * DEG_TO_RAD);
      float rx = c * ax - sn * ay, ry = sn * ax + c * ay;
      ax = rx; ay = ry;
    }
    float smax = sin(MAX_TILT_DEG * DEG_TO_RAD);
    // Soft limit: eases into the cap instead of clipping flat (a clipped tilt lets go with a jerk).
    float sx = smax * tanh(ax / G_ROLL / smax), sy = smax * tanh(ay / G_ROLL / smax);
    // +pitch drops the +x edge (ball goes +x); +roll lifts the +y edge (ball goes -y).
    wantRoll += -asin(sy); wantPitch += asin(sx);
    holdRoll = wantRoll; holdPitch = wantPitch;
  } else {
    tracking = false;                    // no ball (or not balancing): level
  }
  float rate = MAX_TILT_RATE * DEG_TO_RAD * bdt;
  tiltRoll += constrain(TILT_SMOOTH * (wantRoll - tiltRoll), -rate, rate);
  tiltPitch += constrain(TILT_SMOOTH * (wantPitch - tiltPitch), -rate, rate);
  if (streaming) streamLine(seen, zx, zy);
}
// 100 Hz debug stream (R toggles): ms, panel state (1 seen, 2 coasting, 0 none), raw ball mm (x,y),
// filtered mm, speed mm/s, tilt command 0.01 deg (roll, pitch), then per motor: error 0.1 deg, drive %.
void streamLine(bool seen, float zx, float zy) {
  Serial.print('R'); Serial.print(millis()); Serial.print(' ');
  Serial.print(seen ? 1 : tracking ? 2 : 0); Serial.print(' ');
  Serial.print((int)(seen ? zx * 1000 : 0)); Serial.print(' '); Serial.print((int)(seen ? zy * 1000 : 0)); Serial.print(' ');
  Serial.print((int)(bx * 1000)); Serial.print(' '); Serial.print((int)(by * 1000)); Serial.print(' ');
  Serial.print((int)(bvx * 1000)); Serial.print(' '); Serial.print((int)(bvy * 1000)); Serial.print(' ');
  Serial.print((int)(tiltRoll * RAD_TO_DEG * 100)); Serial.print(' '); Serial.print((int)(tiltPitch * RAD_TO_DEG * 100));
  for (auto &m : motors) {
    int d = *m.ocrA ? (int)*m.ocrA : -(int)*m.ocrB;
    Serial.print(' '); Serial.print((int)((m.sp - pos(m)) / COUNTS_PER_DEG * 10));
    Serial.print(' '); Serial.print((int)(-m.ccw * d * 100L / PWM_TOP));
  }
  Serial.println();
}
// The arm math is slow on the Mega (~0.9 ms a leg), so each leg gets its own motor tick after the ball step.
void legStep(int i) {
  float a;
  unsigned long tk = micros();
  if (i == 0) { tcr = cos(tiltRoll); tsr = sin(tiltRoll); tcp = cos(tiltPitch); tsp = sin(tiltPitch); }   // here, not in the ball step: spreads the load
  if (legAngle(i, tcr, tsr, tcp, tsp, a)) setCw(motors[i], armToCw(a, i));
  ikUs = max(ikUs, micros() - tk);
}

void status() {
  static unsigned long last = 0;
  if (!consoleOn || streaming || millis() - last < 200) return;    // nobody listening (or streaming): don't print
  last = millis();
  Serial.print(mode == BALANCE ? "bal" : "lvl");
  if (pattern) { Serial.print(" ["); Serial.print(PATTERNS[pattern]); Serial.print("]"); }
  if (tracking) { Serial.print(" ball "); Serial.print(bx * 1000, 0); Serial.print(","); Serial.print(by * 1000, 0); Serial.print(" mm"); }
  else Serial.print(" no ball");
  Serial.print(" tilt "); Serial.print(tiltRoll * RAD_TO_DEG, 1); Serial.print(","); Serial.print(tiltPitch * RAD_TO_DEG, 1);
  Serial.print(" |");
  for (auto &m : motors) {                              // angle, target, drive % (+ = up)
    int d = *m.ocrA ? (int)*m.ocrA : -(int)*m.ocrB;     // + = input A
    Serial.print(" "); Serial.print(cwDeg(m), 1); Serial.print("/"); Serial.print(-m.sp / COUNTS_PER_DEG, 1);
    Serial.print(" "); Serial.print(-m.ccw * d * 100L / PWM_TOP); Serial.print("%");
  }
  Serial.println();
}

// Run the two loops until 's' or a fault; other keys come back through `pending`.
// Loop timing: worst ball step and worst tick (us), and ticks that ran late, since the last 'v'.
unsigned long ballUsMax = 0, tickUsMax = 0, lateTicks = 0, ballUsSum = 0, ballSteps = 0;
void runLoops() {
  unsigned long t = micros();
  int k = 0;
  while (mode != IDLE) {
    if (stopRequested()) return;         // every tick, even when there's no spare time
    while (micros() - t < TICK_US) if (stopRequested()) return;
    unsigned long t0 = micros();
    t += TICK_US;                         // t = this tick's scheduled start
    if (t0 - t > TICK_US) { lateTicks++; t = t0; }   // a whole tick behind: restart from now, don't burst to catch up
    if (++k >= BALL_EVERY) k = 0;
    if (k == 0) panelStep();
    else if (k == 1) { ballStep(); unsigned long d = micros() - t0; ballUsMax = max(ballUsMax, d); ballUsSum += d; ballSteps++; }
    else legStep(k - 2);                 // k = 2, 3, 4: one leg each
    if (!control()) return;
    status();
    tickUsMax = max(tickUsMax, micros() - t0);
    if (pending) return;                 // let loop() handle the key, then come back
  }
}

// Keep both loops running for `ms` (or until 's').
bool runFor(unsigned long ms) {
  unsigned long t0 = millis(), t = micros();
  int k = 0;
  while (millis() - t0 < ms) {
    while (micros() - t < TICK_US) if (stopRequested()) return false;
    t += TICK_US;
    if (++k >= BALL_EVERY) k = 0;
    if (k == 0) panelStep();
    else if (k == 1) ballStep();
    else legStep(k - 2);
    if (!control()) return false;
  }
  return true;
}

bool panelAvg(float &r, float &t) {
  float sr = 0, st = 0; int n = 0;
  for (int i = 0; i < 10; i++) { float a, b; if (readPanel(a, b)) { sr += a; st += b; n++; } delay(5); }
  if (n < 5) return false;
  r = sr / n; t = st / n;
  return true;
}

// Ball-roll calibration: tilt toward +x and see where the ball goes, then tilt toward -y.
void calibrate() {
  if (mode == IDLE) { Serial.println("Run Enter first"); return; }
  mode = LEVEL;
  const float T = 6 * DEG_TO_RAD;
  float r0, t0, r1, t1, r2, t2;
  Serial.println("Calibrating: ball near the centre please");
  if (!runFor(1500) || !panelAvg(r0, t0)) { Serial.println("  no ball"); return; }
  testPitch = T; testRoll = 0;                       // +pitch drops the +x edge: ball rolls +x
  if (!runFor(900) || !panelAvg(r1, t1)) { Serial.println("  lost the ball"); testPitch = 0; return; }
  testPitch = 0; testRoll = T;                       // +roll lifts the +y edge: ball rolls -y
  if (!runFor(900) || !panelAvg(r2, t2)) { Serial.println("  lost the ball"); testRoll = 0; return; }
  testRoll = 0;
  float dxr = r1 - r0, dxt = t1 - t0, dyr = -(r2 - r1), dyt = -(t2 - t1);
  float nx = sqrt(dxr * dxr + dxt * dxt), ny = sqrt(dyr * dyr + dyt * dyt);
  Serial.print("  start "); Serial.print(r0, 0); Serial.print(","); Serial.print(t0, 0);
  Serial.print("  after +x tilt "); Serial.print(r1, 0); Serial.print(","); Serial.print(t1, 0);
  Serial.print("  after +roll "); Serial.print(r2, 0); Serial.print(","); Serial.println(t2, 0);
  if (nx < 15 || ny < 15) { Serial.println("  ball barely moved; try again"); return; }
  EX_R = dxr / nx; EX_T = dxt / nx; EY_R = dyr / ny; EY_T = dyt / ny;
  Serial.print("  +x moved "); Serial.print(nx, 0); Serial.print(" mm, +y moved "); Serial.print(ny, 0);
  Serial.print(" mm. EX_R "); Serial.print(EX_R, 2); Serial.print(" EX_T "); Serial.print(EX_T, 2);
  Serial.print(" EY_R "); Serial.print(EY_R, 2); Serial.print(" EY_T "); Serial.println(EY_T, 2);
  Serial.print("  axes "); Serial.println(EX_R * EY_T - EX_T * EY_R > 0 ? "not mirrored" : "MIRRORED");
  runFor(1000);
}

// Hold-the-ball calibration, one point per command: 'k' + motor number records where the ball sits
// (held still over that motor); after all three, the axes and centre are computed.
// +x points at motor 1 (away from the midpoint of 2 and 3), +y from motor 3 toward motor 2.
float calR[3], calT[3]; bool calHave[3] = {false, false, false};
void holdPoint(int i) {
  float r, t;
  if (!panelAvg(r, t)) { Serial.println("  no ball on the panel"); return; }
  for (int j = 0; j < 3; j++)
    if (j != i && calHave[j] && abs(r - calR[j]) + abs(t - calT[j]) < 40) {
      Serial.print("  too close to motor "); Serial.print(j + 1); Serial.println("'s point: move the ball over this motor");
      return;
    }
  calR[i] = r; calT[i] = t; calHave[i] = true;
  Serial.print("  motor "); Serial.print(i + 1); Serial.print(" at "); Serial.print(r, 0); Serial.print(","); Serial.println(t, 0);
  if (!(calHave[0] && calHave[1] && calHave[2])) return;
  float xr = calR[0] - (calR[1] + calR[2]) / 2, xt = calT[0] - (calT[1] + calT[2]) / 2;
  float yr = calR[1] - calR[2], yt = calT[1] - calT[2];
  float nx = sqrt(xr * xr + xt * xt), ny = sqrt(yr * yr + yt * yt);
  EX_R = xr / nx; EX_T = xt / nx; EY_R = yr / ny; EY_T = yt / ny;
  CX = (calR[0] + calR[1] + calR[2]) / 3; CY = (calT[0] + calT[1] + calT[2]) / 3;
  Serial.print("Centre "); Serial.print(CX, 1); Serial.print(","); Serial.print(CY, 1);
  Serial.print("  EX_R "); Serial.print(EX_R, 2); Serial.print(" EX_T "); Serial.print(EX_T, 2);
  Serial.print(" EY_R "); Serial.print(EY_R, 2); Serial.print(" EY_T "); Serial.print(EY_T, 2);
  Serial.println(EX_R * EY_T - EX_T * EY_R > 0 ? "  (not mirrored)" : "  (MIRRORED)");
  for (bool &h : calHave) h = false;
}

// ---- Setup / commands ------------------------------------------------------------------------
void setupPwm() {
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
  byte cause = MCUSR;
  brownedOut = cause & (1 << BORF);
  MCUSR = 0;
  setupPwm();
  Serial.begin(115200);
  pinMode(0, INPUT_PULLUP);             // hold RX idle when nothing drives it
  for (auto &m : motors) {
    pinMode(m.encA, INPUT_PULLUP); pinMode(m.encB, INPUT_PULLUP);
    m.prev = (digitalRead(m.encA) << 1) | digitalRead(m.encB);
  }
  void (*isr[N])() = {enc0, enc1, enc2};
  for (int i = 0; i < N; i++) {
    attachInterrupt(digitalPinToInterrupt(motors[i].encA), isr[i], CHANGE);
    attachInterrupt(digitalPinToInterrupt(motors[i].encB), isr[i], CHANGE);
  }
  floatPanel();
  Serial.println("Balancer. Homing at power-on, then balancing. s = stop, Enter = home again, l = level only, b = balance.");
  bootLog(cause);
  pending = '\n';                       // power-on: home and start straight away
}

// Home, then raise straight to the start position: the level pose with each leg's trim
// (motor 1/2/3 at about 33/26/25 deg from home).
void startUp() {
  if (!homeAll()) { allOff(); return; }
  logStage(1);
  float lvl[N], start[N];
  armAngles(0, 0, lvl);
  for (int i = 0; i < N; i++) start[i] = armToCw(lvl[i], i);
  driveLimit = brownedOut ? PWM_TOP * 6L / 10 : PWM_TOP;   // full 12 V, unless that just sagged the supply
  raising = true;
  if (brownedOut) Serial.println("Last reset was a power sag - raising at 60%");
  bool ok = moveAllTo(start, RAISE_SPEED);
  driveLimit = DUTY_MAX;
  raising = false;
  if (!ok || !runFor(500)) { allOff(); return; }   // settle before balancing
  tiltRoll = tiltPitch = testRoll = testPitch = 0;
  mode = BALANCE;                        // level until it sees the ball, then balance it
  logStage(2);
  Serial.println("At start. Balancing as soon as the ball is on the plate.");
}

void handle(char c) {
  const float T = 5 * DEG_TO_RAD;
  switch (c) {
    case '\n': case 'g': startUp(); break;
    case 'b': if (mode != IDLE) { testRoll = testPitch = 0; mode = BALANCE; Serial.println("Balancing"); } break;
    case 'l': if (mode != IDLE) { testRoll = testPitch = 0; mode = LEVEL; Serial.println("Level"); } break;
    case 'x': testPitch = T; testRoll = 0; Serial.println("Tilt: +5 pitch (+x edge should drop)"); break;
    case 'X': testPitch = -T; testRoll = 0; Serial.println("Tilt: -5 pitch"); break;
    case 'y': testRoll = T; testPitch = 0; Serial.println("Tilt: +5 roll (+y edge should rise)"); break;
    case 'Y': testRoll = -T; testPitch = 0; Serial.println("Tilt: -5 roll"); break;
    case 'c': calibrate(); break;
    case 'k': {                                          // k1 / k2 / k3: ball held over that motor
      unsigned long t0 = millis();
      while (!Serial.available() && millis() - t0 < 2000) {}
      int i = Serial.read() - '1';
      if (i >= 0 && i < 3) { if (mode == BALANCE) mode = LEVEL; holdPoint(i); }
    } break;
    case '1': case '2': case '3': case '!': case '@': case '#': {
      int i = (c == '1' || c == '!') ? 0 : (c == '2' || c == '@') ? 1 : 2;
      LEG_OFFSET_DEG[i] += (c >= '1' && c <= '3') ? 0.5 : -0.5;
      Serial.print("Trims (deg up): "); for (float o : LEG_OFFSET_DEG) { Serial.print(o, 1); Serial.print(" "); } Serial.println();
    } break;
    case 'P': case 'p': case 'D': case 'd': case 'I': case 'i': case '0': case 'v': case 'M': case 'm': {
      float f = (c >= 'A' && c <= 'Z') ? 1.15 : 1 / 1.15;
      if (c == 'P' || c == 'p') BALL_KP *= f;
      if (c == 'D' || c == 'd') BALL_KD *= f;
      if (c == 'I' || c == 'i') BALL_KI = BALL_KI < 0.05 && c == 'I' ? 0.3 : BALL_KI * f;
      if (c == '0') BALL_KI = 0;
      if (c == 'M' || c == 'm') KD_M *= f;
      Serial.print("Gains  P "); Serial.print(BALL_KP, 2); Serial.print("  I "); Serial.print(BALL_KI, 2);
      Serial.print("  D "); Serial.print(BALL_KD, 2); Serial.print("   motor damping M "); Serial.println(KD_M, 2);
      Serial.print("Timing: ball step avg "); Serial.print(ballSteps ? ballUsSum / ballSteps : 0); Serial.print(" us, worst "); Serial.print(ballUsMax); Serial.print(" us, worst tick "); Serial.print(tickUsMax);
      Serial.print(" us (budget 2000), late ticks "); Serial.print(lateTicks);
      Serial.print("   last: panel "); Serial.print(panelUs); Serial.print(" us, worst leg IK "); Serial.print(ikUs); Serial.println(" us"); ikUs = 0;
      Serial.print("Panel: coasted "); Serial.print(coasted); Serial.print(" frames, rejected "); Serial.print(rejected); Serial.println(" jumps");
      coasted = rejected = 0;
      ballUsMax = tickUsMax = lateTicks = ballUsSum = ballSteps = 0;
    } break;
    case 'T': EEPROM.update(TRACE_AT - 2, 1); Serial.println("Trace armed: the next run without the console records 1.5 s, then stops"); break;
    case 'R': streaming = !streaming; Serial.println(streaming ? "Stream on" : "Stream off"); break;
    case 'a': case 'f': case 'w': case 'z':
      if (c == 'a') CENTRE_X -= 0.003; if (c == 'f') CENTRE_X += 0.003;
      if (c == 'z') CENTRE_Y -= 0.003; if (c == 'w') CENTRE_Y += 0.003;
      Serial.print("Centre "); Serial.print(CENTRE_X * 1000, 0); Serial.print(", "); Serial.print(CENTRE_Y * 1000, 0); Serial.println(" mm");
      break;
    case '[': case ']':
      AXIS_ROT_DEG += c == ']' ? 2 : -2;
      Serial.print("Axis rotation "); Serial.print(AXIS_ROT_DEG, 0); Serial.println(" deg");
      break;
    case '>': case '<':
      FF_LEAD = constrain(FF_LEAD + (c == '>' ? 0.02 : -0.02), 0.0, 0.3);
      Serial.print("Feed-forward lead "); Serial.print(FF_LEAD * 1000, 0); Serial.println(" ms");
      break;
    case 'n':
      pattern = (pattern + 1) % N_PATTERNS; patternT = 0;
      Serial.print("Pattern: "); Serial.println(PATTERNS[pattern]);
      break;
    case '?': { float x, y; if (readBall(x, y)) { Serial.print("ball "); Serial.print(x, 0); Serial.print(", "); Serial.print(y, 0); Serial.println(" mm"); } else Serial.println("no ball"); } break;
  }
}

void loop() {
  if (pending) { char c = pending; pending = 0; handle(c); }
  else { int k = readKey(); if (k >= 0) handle(k); }
  if (mode != IDLE) runLoops();
}
