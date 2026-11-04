// Ball balancer: at power-on it homes, raises to the start pose, and balances the ball as soon as
// it is on the plate (level until then).
//
// Layers (fastest first):
//   motor loop   500 Hz   each arm follows its angle setpoint (encoder PI + friction push)
//   ball loop    100 Hz   panel -> ball position/speed -> PID -> plate tilt -> inverse kinematics -> arm angles
// Gains and geometry come from the simulation (sim/params.py, sim/controller.py, sim/kinematics.py).
//
// The ball does NOT follow a pattern's path exactly, however the gains are set: it cuts inside (the circle
// comes out a smaller rounded triangle). PID tuning only goes so far, and learning (:ilc) makes the plate
// loud. The quiet fix is a tuned path: the target runs a reshaped path so the ball traces the wanted one
// (circle: ax/ay + h1c..h4s; others: the shape table). See tuning.md.
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
//   e      pin and encoder report now              h  heartbeat log on/off (1 s, 5 s while balancing)
//
// Typed commands: a line starting with ':' (ended by Enter), e.g. ":cx 21". Each answers "= key value".
//   :get              every setting on one line, then each pattern's size and period
//   :cx / :cy <mm>    target centre (where the plate's real centre reads on the panel)
//   :kp :ki :kd       ball gains       :md  motor damping     :lead <ms>  feed-forward lead     :rot <deg>
//   :t1 :t2 :t3 <deg> leg trims (up)
//   :mode <name|0-4>  centre, circle, figure-8, square, wander
//   :ax / :ay <mm>    the current pattern's half-size in x / y     :per <s>  its period (square: per corner)
//   :h1c :h1s ... :h4s  circle shape: radius change (mm) as k cos / k sin harmonics (h3 = three lobes)
//   :save             keep all of the above through a reset (EEPROM)      :defaults  forget them (firmware values)
//   :ilc 1/0          learn the target's shape so the ball follows the path (repeating patterns)
//   :ilcg <0-1>       how much of each lap's error it learns   :ilclead <bins>  how far ahead (100 bins a lap)
//   :ilcclear         zero the shape table (learned or loaded)
//   :off <0-99> <dx> <dy>  one point of the shape table (mm, 100 per lap)    :offs  print the table
//   :park             lower the arms slowly to their stops, motors off (before flashing); home to restart
//   :viz 1 / :viz 0   50 Hz stream for tools/viz.py: "V <ms> <seen 1, coasting 2, none 0> <ball x y> <target x y>" (mm)
//
// Logs (always on, no hello needed): a pin report at boot (motors off), each encoder's check nudge
// (counts, edges per wire, PWM seen on the pin), homing per motor, and a heartbeat line:
//   HB <s> <phase> | M1 <deg> <drive%> A<pin level>B<level> <edges/s A/B> bad <n> [FAULT] | ... | panel z <raw>
//
// Motor   MDD3A terminals   Mega PWM (A, B)   Encoder (blue, purple)   Leg
//   1     #1 M1A / M1B      D6, D7            D2, D19                  0 deg
//   2     #2 M1A / M1B      D45, D46          D3, D20                  120 deg
//   3     #1 M2A / M2B      D8, D44           D18, D21                 240 deg
// Panel: tail pins 1-4 (Y-, X-, Y+, X+) to A0-A3: X layer A3/A1, Y layer A2/A0 (wiring/wiring.pdf).
// MODE button: NO contact D30 to GND (pull-up). Starts in circle; each press = next pattern (like n).
// POWER button: its NO turns the Pololu switch on (on-only); its NC holds D31 low, so a press reads high.
// A press parks the arms slowly, then D32 pulses the Pololu's OFF pin to cut the 12 V.
// Arm directions as seen from the front (arm side): CCW = down (homing), CW = up.

#include <math.h>
#include <EEPROM.h>

// ---- Geometry (mm, deg), from sim/params.py --------------------------------------------------
const float MOTOR_R = 30.0, MOTOR_Z = 40.0, ARM_L = 35.0, ROD_L = 59.0, JOINT_R = 65.0;
const float LEG_DEG[3] = {0, 120, 240};
const float HOME_ARM_DEG = -25.0;      // arm angle at the low stop (sim's arm_min); CW from home adds to it
const float ARM_MIN_DEG = -20.0, ARM_MAX_DEG = 40.0;   // never command past these (stop + margin)
float LEG_OFFSET_DEG[3] = {10.5, 0, 13.5};             // per-leg trim, degrees CW (up), for a level plate (new legs, inclinometer)
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
float EX_R = -0.16, EX_T = -0.99, EY_R = 1.00, EY_T = -0.07;   // narrow axis (right) flipped after the ribbon mirror: the plate leaned into the ball
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
const unsigned long DEAD_MS = 400;     // 50%+ drive, >5 deg off target, no encoder change this long: that motor off
const float WRONG_WAY_DEG = 15;        // moved this far against a 50%+ drive: that motor off (coasting past a reversal stays well under)
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
  //   M2: input A = CW (up), counts up; CCW = B -> counts down (drive test on D45/D46, arm watched)
  //   M3: input A counts up, B down; CCW = B -> counts down (re-checked on D8/D44: A +501, B -466)
  // Motor 2 and 3 drive pins swapped to match the build: motor 3 is on D8/D44, motor 2 on D45/D46.
  {"Motor 1", &OCR4A, &OCR4B, 2, 19, 0, 0, -1, 1},
  {"Motor 2", &OCR5B, &OCR5A, 3, 20, 0, 0, -1, -1},
  {"Motor 3", &OCR4C, &OCR5C, 18, 21, 0, 0, -1, -1},
};

// ---- Encoders --------------------------------------------------------------------------------
const uint8_t PWM_PINS[N][2] = {{6, 7}, {45, 46}, {8, 44}};   // input A, B pins (same order as ocrA, ocrB)
static const int8_t QUAD[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
volatile unsigned int badEdges[3] = {0, 0, 0};   // both channels flipped at once = noise or a missed edge
volatile unsigned long edgesA[3], edgesB[3];     // per wire: one stuck at 0 while the arm turns = that wire
// Edge flood: an unpowered or floating encoder picks up the PWM and fires the interrupt so often the
// board locks up (seen with the encoder 5V off). Past STORM_EDGES in one millisecond (the fastest arm
// makes ~7) that encoder's interrupts are detached and its motor is cut, right here in the ISR.
const byte STORM_EDGES = 30;
volatile bool stormed[3];
volatile unsigned long stormMs[3]; volatile byte stormN[3];
void encEdge(Motor &m) {
  uint8_t i = &m - motors;
  uint8_t s = (digitalRead(m.encA) << 1) | digitalRead(m.encB), ch = m.prev ^ s;
  if (ch == 3) badEdges[i]++;
  if (ch & 2) edgesA[i]++;
  if (ch & 1) edgesB[i]++;
  m.count += QUAD[(m.prev << 2) | s];
  m.prev = s;
  unsigned long ms = millis();
  if (ms != stormMs[i]) { stormMs[i] = ms; stormN[i] = 0; }
  if (++stormN[i] > STORM_EDGES && !stormed[i]) {
    detachInterrupt(digitalPinToInterrupt(m.encA)); detachInterrupt(digitalPinToInterrupt(m.encB));
    *m.ocrA = *m.ocrB = 0;
    stormed[i] = true;
  }
}
void enc0() { encEdge(motors[0]); }
void enc1() { encEdge(motors[1]); }
void enc2() { encEdge(motors[2]); }
void (*const ENC_ISR[N])() = {enc0, enc1, enc2};
void attachEncoder(int i) {
  Motor &m = motors[i];
  noInterrupts(); stormed[i] = false; stormN[i] = 0; interrupts();
  m.prev = (digitalRead(m.encA) << 1) | digitalRead(m.encB);
  attachInterrupt(digitalPinToInterrupt(m.encA), ENC_ISR[i], CHANGE);
  attachInterrupt(digitalPinToInterrupt(m.encB), ENC_ISR[i], CHANGE);
}
// digitalRead() turns a pin's PWM off, so the logs read the port register instead.
int pinHigh(uint8_t pin) { return (*portInputRegister(digitalPinToPort(pin)) & digitalPinToBitMask(pin)) ? 1 : 0; }
unsigned long edges(volatile unsigned long *e, int i) { noInterrupts(); unsigned long v = e[i]; interrupts(); return v; }
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
void heartbeat();
void buttonLog();
void lineCommand(char *s);
const uint8_t MODE_PIN = 30;           // MODE button, to GND
const uint8_t POWER_PIN = 31, OFF_PIN = 32;   // POWER button NC (low at rest), Pololu OFF (pulse high)
bool offRequested = false, parkRequested = false, shuttingDown = false;   // POWER press / :park
const char *phase = "boot";            // for the logs: boot, check, homing, raising, idle, running, stopped
const int TRACE_AT = 600, TRACE_N = 150;
int readKey() {
  static byte matched = 0;
  diagSave();
  heartbeat();
  buttonLog();
  while (Serial.available()) {
    char c = Serial.read();
    if (consoleOn) {                   // ':' starts a typed command, ended by Enter (so Enter doesn't re-home)
      static char line[40]; static int8_t len = -1;
      if (len >= 0) {
        if (c == '\n' || c == '\r') { line[len] = 0; len = -1; lineCommand(line); }
        else if (len < 39) line[len++] = c;
        continue;
      }
      if (c == ':') { len = 0; continue; }
      return c;
    }
    strayBytes++;
    matched = c == HELLO[matched] ? matched + 1 : (c == HELLO[0] ? 1 : 0);
    if (!HELLO[matched]) {
      consoleOn = true; Serial.println(F("Console connected - keys on"));
      if (EEPROM.read(TRACE_AT - 2) == TRACE_N) { printTrace(); EEPROM.update(TRACE_AT - 2, 0); }   // shown once
    }
  }
  return -1;
}
bool stopRequested() {
  buttonLog();
  if ((offRequested || parkRequested) && !shuttingDown) return true;   // POWER / :park: drop out, loop() parks
  if (pending) return false;             // one key at a time; the rest wait in the buffer
  int k = readKey();
  if (k < 0) return false;
  char c = k;
  if (c == 's') { allOff(); phase = "stopped"; Serial.println(F("STOP")); return true; }
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
  Serial.print(F("Boot #")); Serial.print(n); Serial.print(F("  reset flags 0x")); Serial.println(cause, HEX);
  Serial.print(F("Earlier boots, oldest first (0 booted 1 homed 2 raised 3 balancing): "));
  for (int k = 1; k < LOG_N; k++) {
    byte st = EEPROM.read(4 + (logSlot + k) % LOG_N);
    Serial.print(st == 0xFF ? '-' : char('0' + st)); Serial.print(' ');
  }
  Serial.println();
  for (int k = LOG_N - 3; k < LOG_N; k++) {          // the last 3 boots in detail
    int a = 40 + (logSlot + k) % LOG_N * 32;
    unsigned int f, sf, sb; int mz; float mr;
    EEPROM.get(a, f); EEPROM.get(a + 2, sf); EEPROM.get(a + 4, mz); EEPROM.get(a + 6, mr); EEPROM.get(a + 10, sb);
    Serial.print(F("  boot -")); Serial.print(LOG_N - k); Serial.print(F(": ran ")); Serial.print(EEPROM.read(a + 13));
    Serial.print(F(" s, mode ")); Serial.print(EEPROM.read(a + 12)); Serial.print(F(" (0 idle 1 level 2 balance), stray serial bytes ")); Serial.print(sb);
    Serial.print(F(", balance frames ")); Serial.print(f); Serial.print(F(", ball seen ")); Serial.print(sf);
    Serial.print(F(", lowest z ")); Serial.print(mz); Serial.print(F(", farthest ball mm ")); Serial.println(mr, 0);
    unsigned long gap; EEPROM.get(a + 18, gap);
    Serial.print(F("      worst motor error deg (balancing) ")); for (int i = 0; i < N; i++) { Serial.print(EEPROM.read(a + 14 + i)); Serial.print(' '); }
    Serial.print(F(", longest control gap us ")); Serial.print(gap);
    Serial.print(F(", bad encoder edges ")); for (int i = 0; i < N; i++) { unsigned int b; EEPROM.get(a + 22 + 2 * i, b); Serial.print(b); Serial.print(' '); }
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
// Encoder fault: that motor alone is switched off until a re-home (Enter) re-checks it; the
// others carry on. A motor that can't see its arm would run it flat out into the stops.
bool encFault[N] = {false, false, false};
void printEncoder(int i) {                // pin levels now, edges per wire so far, bad edges
  Motor &m = motors[i];
  Serial.print(F(" pins A D")); Serial.print(m.encA); Serial.print('='); Serial.print(pinHigh(m.encA));
  Serial.print(F(" B D")); Serial.print(m.encB); Serial.print('='); Serial.print(pinHigh(m.encB));
  Serial.print(F(", edges A ")); Serial.print(edges(edgesA, i)); Serial.print(F(" B ")); Serial.print(edges(edgesB, i));
  Serial.print(F(", bad ")); Serial.print(badEdges[i]); Serial.print(F(", count ")); Serial.print(raw(m));
  if (stormed[i]) Serial.print(F(", FLOODED (interrupts off)"));
}
void encoderFault(int i, const char *why) {
  encFault[i] = true;
  drive(motors[i], 0, 0); motors[i].on = false;
  Serial.print(motors[i].name); Serial.print(F(" off - encoder fault: ")); Serial.println(why);
  Serial.print(F("  ")); Serial.print(motors[i].name); Serial.print(':'); printEncoder(i); Serial.println();
}
const char *FLOOD_WHY = "edge flood on its pins (encoder unpowered or a wire floating near the motor leads?)";

bool control() {
  static unsigned long lastUs = 0;
  static long lastRaw[N]; static unsigned long sinceMove[N]; static float wrongWay[N];
  // Position history for the damping term, per motor (a shared index stalled whenever motor 3 was
  // off, so the others damped over 2 ms instead of 10: about 5x too weak).
  static float hist[N][5]; static byte hIdx[N];
  unsigned long nowUs = micros();
  if (!lastUs || nowUs - lastUs > 50000UL)   // first tick of a run: start the encoder checks fresh
    for (int i = 0; i < N; i++) {
      lastRaw[i] = raw(motors[i]); sinceMove[i] = millis(); wrongWay[i] = 0;
      for (float &v : hist[i]) v = pos(motors[i]);
    }
  if (mode == BALANCE && lastUs) maxGapUs = max(maxGapUs, nowUs - lastUs);
  lastUs = nowUs;
  for (int i = 0; i < N; i++) {
    Motor &m = motors[i];
    if (stormed[i] && !encFault[i]) encoderFault(i, FLOOD_WHY);
    if (encFault[i]) { drive(m, 0, 0); continue; }
    if (!m.on) continue;
    float err = m.sp - pos(m);
    if (mode == BALANCE) maxErrDeg[i] = max(maxErrDeg[i], (byte)min(255.0, abs(err) / COUNTS_PER_DEG));
    int limit = driveLimit;
    // Encoder checks: pushed at 50%+ toward a target 5+ deg away, the count must change within
    // DEAD_MS, and must not run against the push (A/B or motor leads swapped = runaway).
    long r = raw(m);
    float applied = (*m.ocrA - (float)*m.ocrB) * m.ccw;   // CCW duty from the last tick
    float dpos = m.sign * (float)(r - lastRaw[i]);
    lastRaw[i] = r;
    bool pushing = abs(applied) >= PWM_TOP / 2 && abs(err) > 5 * COUNTS_PER_DEG;
    if (dpos != 0 || !pushing) sinceMove[i] = millis();
    if (pushing && dpos * applied < 0) wrongWay[i] += abs(dpos);
    else if (dpos * applied > 0) wrongWay[i] = 0;
    if (millis() - sinceMove[i] > DEAD_MS) { encoderFault(i, "driven hard, count never changed (encoder unplugged or no 5V?)"); continue; }
    if (wrongWay[i] > WRONG_WAY_DEG * COUNTS_PER_DEG) { encoderFault(i, "arm moving against its drive (encoder A/B or motor leads swapped?)"); continue; }
    if (cwDeg(m) > CW_CUT) {               // over-extended: never push it further up
      drive(m, 0, 0);
      if (millis() - lastWarn > 1000) { lastWarn = millis(); Serial.print(m.name); Serial.println(F(": past 55 deg, drive cut (re-home with Enter)")); }
      continue;
    }
    if (abs(err) > RUNAWAY_DEG * COUNTS_PER_DEG) {
      if (!behindSince[i]) behindSince[i] = millis();
      if (millis() - behindSince[i] > STUCK_MS) limit = min(limit, STUCK_DUTY);
      if (millis() - lastWarn > 1000) {
        lastWarn = millis();
        Serial.print(m.name); Serial.print(F(": ")); Serial.print(abs(err) / COUNTS_PER_DEG, 0);
        Serial.println(limit < driveLimit ? " deg behind for 5 s+, easing to 40%" : " deg behind, still pushing");
      }
    } else behindSince[i] = 0;
    const float imax = PWM_TOP * INTEG_MAX_PCT / 100.0;
    if (abs(err) < 6 * COUNTS_PER_DEG) m.integ = constrain(m.integ + KI * err, -imax, imax);
    if (err * m.integ < 0) m.integ *= 0.9;
    // friction kick fades in over the first 2 deg, so it doesn't flip +-18% around the target like a relay
    float fric = constrain(err / (2 * COUNTS_PER_DEG), -1.0, 1.0) * PWM_TOP * FRICTION_PCT / 100.0;
    float p = pos(m), moved = p - hist[i][hIdx[i]];        // counts moved in the last 10 ms
    hist[i][hIdx[i]] = p;
    hIdx[i] = (hIdx[i] + 1) % 5;
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
// Before homing, nudge each arm up a couple of degrees on its own (the others idle): its encoder
// must count, and count the right way. A motor that fails is switched off; the others carry on.
// Logged per motor: counts, time, drive reached, edges on each wire, and how often the driven PWM pin
// read high (should be about the duty: 0 = the pin isn't switching, or is shorted to GND).
bool checkEncoders() {
  phase = "check";
  Serial.println(F("Encoder check: each arm nudged up 2 deg on its own"));
  for (int i = 0; i < N; i++) { encFault[i] = false; attachEncoder(i); }
  delay(50);
  for (int i = 0; i < N; i++) {
    Motor &m = motors[i];
    if (stormed[i]) {                                   // flooding with the motor off: don't drive it at all
      Serial.print(F("  ")); Serial.print(m.name); Serial.println(F(": flooding with the motor off - not nudged"));
      encoderFault(i, FLOOD_WHY);
      continue;
    }
    long r0 = raw(m);
    unsigned long a0 = edges(edgesA, i), b0 = edges(edgesB, i); unsigned int bad0 = badEdges[i];
    float duty = 0, upCounts = 0;
    long highs = 0, samples = 0;
    unsigned long t0 = millis(), last = 0;
    while (millis() - t0 < 1500 && !stormed[i]) {
      if (stopRequested()) { drive(m, 0, 0); return false; }
      if (millis() - last < 10) continue;
      last = millis();
      upCounts = -m.sign * (float)(raw(m) - r0);       // CW (up) is negative pos
      if (abs(upCounts) >= 2 * COUNTS_PER_DEG) break;
      duty = min(duty + PWM_TOP * 0.01, (float)PWM_TOP * 0.4);
      drive(m, -duty, HOME_DUTY);                       // negative CCW duty = up
      uint8_t pin = *m.ocrA ? PWM_PINS[i][0] : PWM_PINS[i][1];
      if (duty > PWM_TOP * 0.1) for (int k = 0; k < 20; k++) { highs += pinHigh(pin); samples++; }
    }
    unsigned long ms = millis() - t0;
    drive(m, 0, 0);
    delay(100);
    unsigned long ea = edges(edgesA, i) - a0, eb = edges(edgesB, i) - b0;
    unsigned int bad = badEdges[i] - bad0;
    uint8_t pin = PWM_PINS[i][m.ccw < 0 ? 0 : 1];       // up = input A when ccw is -1
    Serial.print(F("  ")); Serial.print(m.name); Serial.print(F(": ")); Serial.print(upCounts, 0); Serial.print(F(" counts up in "));
    Serial.print(ms); Serial.print(F(" ms at up to ")); Serial.print(duty * 100 / PWM_TOP, 0); Serial.print(F("% drive on D")); Serial.print(pin);
    Serial.print(F(" (pin high ")); if (samples) { Serial.print(highs * 100 / samples); Serial.print('%'); } else Serial.print(F("n/a"));
    Serial.print(F(" of samples), edges A "));
    Serial.print(ea); Serial.print(F(" B ")); Serial.print(eb); Serial.print(F(", bad ")); Serial.println(bad);
    if (stormed[i]) encoderFault(i, FLOOD_WHY);
    else if (samples && highs == 0) encoderFault(i, "its PWM pin never went high (pin shorted to GND, or the timer isn't driving it)");
    else if (ea == 0 && eb == 0) encoderFault(i, "no edges on either wire: encoder 5V/GND or plug (or the arm didn't turn: motor leads / MDD3A 12V)");
    else if (ea == 0) encoderFault(i, "no edges on wire A: that wire or its pin");
    else if (eb == 0) encoderFault(i, "no edges on wire B: that wire or its pin");
    else if (abs(upCounts) < 2) encoderFault(i, "edges but no net count (noise, not movement)");
    else if (upCounts < 0) encoderFault(i, "counted down while driven up (encoder A/B or motor leads swapped?)");
    else {
      if (bad > 2) { Serial.print(F("  ")); Serial.print(m.name); Serial.println(F(": encoder OK but noisy (bad edges)")); }
      else { Serial.print(F("  ")); Serial.print(m.name); Serial.println(F(" encoder OK")); }
    }
  }
  int ok = 0; for (bool f : encFault) ok += !f;
  Serial.print(F("Encoder check done: ")); Serial.print(ok); Serial.println(F(" of 3 OK"));
  if (!ok) { Serial.println(F("No working encoder - not homing. Fix, then Enter (or reset)")); phase = "stopped"; return false; }
  return true;
}

bool homedOk = false;                  // set once homing finishes: the arms' angles are known (power-off parks)
bool homeAll() {
  homedOk = false;
  if (!checkEncoders()) return false;
  bool creeping[N];
  float duty[N]; long last[N], start[N]; unsigned long still[N];
  for (int i = 0; i < N; i++) {
    motors[i].on = false; creeping[i] = !encFault[i]; duty[i] = 0;
    start[i] = last[i] = raw(motors[i]); still[i] = millis();
  }
  phase = "homing";
  Serial.println(F("Homing"));
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
          Serial.print(F("  ")); Serial.print(m.name); Serial.println(F(": counted the other way while homing (pushed by the plate?)"));
        }
        noInterrupts(); m.count = 0; interrupts();
        m.sp = m.integ = 0; m.on = true;
        Serial.print(F("  ")); Serial.print(m.name); Serial.print(F(" home: travelled ")); Serial.print(abs(travelled) / COUNTS_PER_DEG, 1);
        Serial.print(F(" deg in ")); Serial.print(millis() - t0); Serial.print(F(" ms, at ")); Serial.print(duty[i] * 100 / PWM_TOP, 0); Serial.println(F("% drive"));
      }
      busy |= creeping[i];
    }
    if (!busy) break;
    for (int i = 0; i < N; i++) if (creeping[i] && stormed[i]) { creeping[i] = false; encoderFault(i, FLOOD_WHY); }
    if (millis() - t0 > 30000) {
      allOff(); phase = "stopped";
      Serial.print(F("  no stop found in 30 s:"));
      for (int i = 0; i < N; i++) if (creeping[i]) { Serial.print(' '); Serial.print(motors[i].name); }
      Serial.println(F(" still creeping (counting the whole time = never reached its stop)"));
      for (int i = 0; i < N; i++) if (creeping[i]) { Serial.print(F("  ")); Serial.print(motors[i].name); Serial.print(':'); printEncoder(i); Serial.println(); }
      return false;
    }
  }
  // Confirm: all three push down together so the plate seats and every arm reaches its real stop
  // (one arm can stall against the plate early while the others are still up), then zero them.
  for (int i = 0; i < N; i++) { motors[i].on = false; drive(motors[i], encFault[i] ? 0 : HOME_DUTY, HOME_DUTY); }
  unsigned long c0 = millis();
  while (millis() - c0 < 800) if (stopRequested()) return false;
  for (auto &m : motors) { drive(m, 0, 0); }
  delay(150);
  for (auto &m : motors) {
    long extra = abs(raw(m));
    noInterrupts(); m.count = 0; interrupts();
    m.sp = m.integ = 0; m.on = !encFault[&m - motors];
    if (extra > 3 * COUNTS_PER_DEG) { Serial.print(F("  ")); Serial.print(m.name); Serial.print(F(" went ")); Serial.print(extra / COUNTS_PER_DEG, 0); Serial.println(F(" deg further down on the confirm push")); }
  }
  Serial.println(F("  homed"));
  homedOk = true;
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
const uint8_t XA = A3, XB = A1, YA = A2, YB = A0;   // ribbon mirrored: A0<->A3, A1<->A2
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
int panelZ() {                          // touch pressure: ~1023 untouched, low when pressed
  floatPanel();
  pinMode(XA, OUTPUT); digitalWrite(XA, LOW);
  pinMode(YA, INPUT_PULLUP);
  delayMicroseconds(50);
  analogRead(YA);
  int z = analogRead(YA);
  floatPanel();
  return z;
}
bool readPanel(float &right, float &top) {
  int z = panelZ();
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
  Serial.println(F("Trace saved, motors off"));
}
void printTrace() {
  if (EEPROM.read(TRACE_AT - 2) != TRACE_N) { Serial.println(F("No trace saved")); return; }
  Serial.println(F("Trace, first 1.5 s of balancing. ms | per motor: angle/target drive% (+ = up)"));
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
float CENTRE_X = 0.021, CENTRE_Y = -0.015;  // m: where the plate's real centre reads on the panel (ball held there); live a/f (x), w/z (y), :cx :cy
void patternTarget(float &x, float &y);
// Each pattern's half-size in x and y (mm) and period (s; square: per corner, 60% hold then 40% dash).
// Live: :ax :ay :per act on the current pattern.
float PAT_AX[N_PATTERNS] = {0, 56.86, 60, 45, 35}, PAT_AY[N_PATTERNS] = {0, 56.86, 35, 45, 22};
float PAT_T[N_PATTERNS] = {0, 3.5, 5.5, 2.5, 8.4};
// Circle shape: radius change (mm) with the angle round the circle, as harmonics: k cos / k sin for
// k = 1-4 (3 = the three-lobe shape the legs give). Tuned by hand (tools/paths/circle_paths_test2.csv):
// average radius 56.86 (ax/ay) so that the ball traces a 45 mm circle. Live: :h1c :h1s ... :h4s.
float SHAPE[8] = {-0.08, -1.32, -0.20, 0.89, -0.76, -0.11, 0.06, 0.47};
void patternAt(float t, float &x, float &y) {
  float ease = minJerk(t / 2.0);                   // glide out from the centre over 2 s
  float a = PAT_AX[pattern] / 1000, b = PAT_AY[pattern] / 1000, T = max(PAT_T[pattern], 0.5f), w = TWO_PI / T;
  x = y = 0;
  switch (pattern) {
    case 1: {
      float th = w * t, dr = 0;
      for (int k = 1; k <= 4; k++) dr += SHAPE[2 * k - 2] * cos(k * th) + SHAPE[2 * k - 1] * sin(k * th);
      x = (a + dr / 1000) * cos(th); y = (b + dr / 1000) * sin(th);
    } break;
    case 2: x = a * sin(w * t); y = b * sin(2 * w * t); break;
    case 3: {                                     // square: hold at each corner, then dash to the next
      const float MOVE = 0.4 * T, HOLD = 0.6 * T;
      static const float C[5][2] = {{-1, 1}, {1, 1}, {1, -1}, {-1, -1}, {-1, 1}};
      float u = fmod(t, 4 * T);
      int i = u / T;
      float k = minJerk((u - i * T - HOLD) / MOVE);      // 0 while holding
      x = a * (C[i][0] + (C[i + 1][0] - C[i][0]) * k); y = b * (C[i][1] + (C[i + 1][1] - C[i][1]) * k);
    } break;
    case 4: {                                     // two sines per axis; per = the slow one's period
      float u = t * 8.4 / T;
      x = a * (sin(0.75 * u) + 0.571 * sin(1.9 * u + 1.0));
      y = b * (sin(1.1 * u + 0.5) + 0.545 * sin(2.6 * u + 2.0));
    } break;
  }
  x *= ease; y *= ease;
}

// ---- Learned target shape (iterative learning) -----------------------------------------------
// The ball doesn't trace the pattern (a circle comes out a smaller, rounded triangle: it cuts in over
// each leg). For the repeating patterns, each lap's error between the path and the ball is fed back
// into a correction added to the target, ILC_LEAD bins earlier (the ball answers late). After a few
// laps the target takes whatever shape makes the ball follow the path. One correction per lap phase,
// smoothed once a lap; cleared when the pattern, its size or period changes. Live: :ilc :ilcg :ilclead.
const int ILC_N = 100;                 // bins per lap
float ilcX[ILC_N], ilcY[ILC_N];        // m
float ILC_ON = 0, ILC_GAIN = 0.3, ILC_LEAD = 3;   // on/off, fraction of a lap's error learned per lap, bins
const float ILC_MAX = 0.015;           // m: never correct further than this (a learned drop-in swing reached 22)
const float ILC_SKIP = 0.02;           // m: a sample this far off the path is a disturbance, not the shape
float lapT() { return pattern == 3 ? 4 * PAT_T[3] : (pattern == 1 || pattern == 2) ? PAT_T[pattern] : 0; }
// The same table holds a shape loaded from a file (:off, tools/shape.py) and is what :save keeps, one
// table per repeating pattern (circle, figure-8, square) at EEPROM 2400 (0.1 mm steps).
const int OFF_AT = 2400, OFF_SLOT = 2 + 4 * ILC_N; const uint16_t OFF_MAGIC = 0x5AFE;
int offSlot(int p) { return p >= 1 && p <= 3 ? OFF_AT + (p - 1) * OFF_SLOT : -1; }
void offSave() {
  int a = offSlot(pattern); if (a < 0) return;
  for (int i = 0; i < ILC_N; i++) { EEPROM.put(a + 2 + 4 * i, (int16_t)round(ilcX[i] * 1e4)); EEPROM.put(a + 4 + 4 * i, (int16_t)round(ilcY[i] * 1e4)); }
  EEPROM.put(a, OFF_MAGIC);
}
// The figure-8's tuned table (tools/paths/figure8_target_test8_revert_test5.csv), 0.1 mm per lap bin:
// used until a table is saved for it.
const int16_t FIG8_TABLE[ILC_N][2] PROGMEM = {
  {50, -70}, {49, -59}, {48, -47}, {48, -35}, {48, -24}, {49, -14}, {50, -5}, {51, 1}, {52, 5}, {53, 7},
  {51, 12}, {47, 18}, {42, 22}, {37, 26}, {31, 30}, {24, 33}, {17, 35}, {10, 36}, {3, 37}, {-4, 37},
  {-11, 36}, {-17, 34}, {-22, 32}, {-27, 29}, {-30, 25}, {-33, 21}, {-34, 17}, {-34, 14}, {-33, 13}, {-32, 12},
  {-29, 13}, {-26, 14}, {-22, 15}, {-17, 16}, {-13, 16}, {-8, 16}, {-2, 15}, {3, 13}, {7, 10}, {12, 5},
  {16, -1}, {20, -11}, {26, -30}, {31, -47}, {35, -61}, {38, -72}, {41, -79}, {44, -82}, {46, -81}, {48, -77},
  {50, -70}, {51, -59}, {52, -48}, {51, -38}, {48, -29}, {44, -22}, {36, -18}, {27, -17}, {15, -20}, {0, -26},
  {-6, -25}, {-8, -21}, {-10, -18}, {-11, -15}, {-11, -13}, {-10, -10}, {-8, -8}, {-5, -6}, {-1, -4}, {4, -2},
  {10, -1}, {16, -1}, {22, -2}, {28, -3}, {33, -6}, {38, -9}, {42, -13}, {45, -15}, {48, -16}, {50, -17},
  {51, -18}, {53, -18}, {54, -19}, {55, -20}, {56, -21}, {58, -22}, {59, -24}, {60, -27}, {62, -30}, {63, -35},
  {64, -40}, {64, -46}, {64, -56}, {63, -65}, {62, -72}, {60, -78}, {58, -82}, {56, -83}, {54, -82}, {52, -77},
};
bool offLoad() {                       // this pattern's saved table, else its built-in one, else zeros
  int a = offSlot(pattern); uint16_t m = 0;
  if (a >= 0) EEPROM.get(a, m);
  for (int i = 0; i < ILC_N; i++) {
    int16_t x = 0, y = 0;
    if (m == OFF_MAGIC) { EEPROM.get(a + 2 + 4 * i, x); EEPROM.get(a + 4 + 4 * i, y); }
    else if (pattern == 2) { x = pgm_read_word(&FIG8_TABLE[i][0]); y = pgm_read_word(&FIG8_TABLE[i][1]); }
    ilcX[i] = x * 1e-4; ilcY[i] = y * 1e-4;
  }
  return m == OFF_MAGIC;
}
int ilcLaps = 0; float ilcBest = 1;   // laps since the shape was cleared, and the best lap's error (m)
void ilcClear() { for (int i = 0; i < ILC_N; i++) ilcX[i] = ilcY[i] = 0; ilcLaps = 0; ilcBest = 1; }
void ilcAt(float t, float &x, float &y) {          // the correction at pattern time t (interpolated)
  float L = lapT();
  x = y = 0;
  if (L <= 0) return;
  float u = fmod(t, L) / L * ILC_N;
  int i = (int)u % ILC_N, j = (i + 1) % ILC_N;
  float f = u - (int)u;
  x = ilcX[i] + (ilcX[j] - ilcX[i]) * f; y = ilcY[i] + (ilcY[j] - ilcY[i]) * f;
}
// Called each ball step with the ball seen: t = pattern time of this step's target.
void ilcLearn(float t, float bxM, float byM) {
  static float lastPhase = 0, sumE2 = 0; static int nE = 0;
  float L = lapT();
  if (!ILC_ON || L <= 0 || t < 2.0) return;        // only repeating patterns, after the 2 s glide-in
  static unsigned long lastCall = 0, since = 0;    // learn only once the ball has followed a whole lap
  if (millis() - lastCall > 200) since = millis();  // (a gap = the ball was off or just put down)
  lastCall = millis();
  if (millis() - since < L * 1000) return;
  float px, py;
  patternAt(t, px, py);
  float ex = px + CENTRE_X - bxM, ey = py + CENTRE_Y - byM;
  if (ex * ex + ey * ey > ILC_SKIP * ILC_SKIP) return;
  float phase = fmod(t, L) / L;
  int b = ((int)(phase * ILC_N) - (int)ILC_LEAD + ILC_N) % ILC_N;
  float g = ILC_GAIN * ILC_N * bdt / L;            // spread one lap's worth of learning over the bin's samples
  ilcX[b] = constrain(ilcX[b] + g * ex, -ILC_MAX, ILC_MAX);
  ilcY[b] = constrain(ilcY[b] + g * ey, -ILC_MAX, ILC_MAX);
  sumE2 += ex * ex + ey * ey; nE++;
  if (phase < lastPhase) {                         // a lap done: smooth ([1 2 1]/4 twice, around the loop), log
    for (int pass = 0; pass < 2; pass++) {
    float fx = ilcX[0], fy = ilcY[0], px0 = ilcX[ILC_N - 1], py0 = ilcY[ILC_N - 1];
    for (int i = 0; i < ILC_N; i++) {
      float nx = i < ILC_N - 1 ? ilcX[i + 1] : fx, ny = i < ILC_N - 1 ? ilcY[i + 1] : fy;
      float cx = ilcX[i], cy = ilcY[i];
      ilcX[i] = (px0 + 2 * cx + nx) / 4; ilcY[i] = (py0 + 2 * cy + ny) / 4;
      px0 = cx; py0 = cy;
    }
    }
    float mx = 0; for (int i = 0; i < ILC_N; i++) mx = max(mx, sqrt(ilcX[i] * ilcX[i] + ilcY[i] * ilcY[i]));
    int &laps = ilcLaps; float &best = ilcBest;
    laps++;
    float rms = nE ? sqrt(sumE2 / nE) : 0;
    Serial.print(F("ILC lap ")); Serial.print(laps); Serial.print(F(": ball off the path by ")); Serial.print(rms * 1000, 1);
    Serial.print(F(" mm rms, largest correction ")); Serial.print(mx * 1000, 1); Serial.println(F(" mm"));
    if (laps == 1) best = 1;                       // the first lap includes putting the ball down
    else best = min(best, rms);
    if (laps > 3 && rms > 0.008 && rms > 2 * best) {   // getting worse lap on lap: forget it, start again
      float was = best;
      ilcClear();
      Serial.print(F("ILC diverging (best ")); Serial.print(was * 1000, 1); Serial.println(F(" mm) - shape cleared, learning again"));
    }
    sumE2 = 0; nE = 0;
  }
  lastPhase = phase;
}
void patternTarget(float &x, float &y) {          // m, plate frame: the target now (path + shape table)
  static int lastP = -1;
  if (pattern != lastP) {                          // a new pattern: its saved shape table, or none
    lastP = pattern; ilcLaps = 0; ilcBest = 1;
    if (offLoad()) { Serial.print(F("Shape table loaded for ")); Serial.println(PATTERNS[pattern]); }
  }
  float cx, cy;
  patternAt(patternT, x, y);
  ilcAt(patternT, cx, cy);
  x += CENTRE_X + cx; y += CENTRE_Y + cy;
  patternT += bdt;
}

unsigned long panelUs = 0, ikUs = 0;
bool streaming = false;
bool vizOn = false;                    // :viz 1 - ball and target for tools/viz.py
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
  float tPat = patternT;               // this step's pattern time (patternTarget advances it)
  patternTarget(tx, ty);
  // Target speed includes the learned correction: taking it from the smooth path alone delayed the
  // ball's answer to the correction and the learning diverged (laps 5-11: 6 -> 15 mm).
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
    if (mode == BALANCE) ilcLearn(tPat, bx, by);
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
  static byte vizN = 0;
  if (vizOn && (++vizN & 1)) {           // every other step: 50 Hz, short enough not to stall the loops
    Serial.print(F("V ")); Serial.print(millis()); Serial.print(' '); Serial.print(seen ? 1 : tracking ? 2 : 0);
    Serial.print(' '); Serial.print((int)(bx * 1000)); Serial.print(' '); Serial.print((int)(by * 1000));
    Serial.print(' '); Serial.print((int)(tx * 1000)); Serial.print(' '); Serial.println((int)(ty * 1000));
  }
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

// Boot pin report, motors off. Each encoder wire with the Mega's pull-up on and off (a powered encoder
// drives the line, so it reads the same both ways), edges at rest (should be 0), the PWM pins (should
// be low), and the panel's raw readings.
void pinReport() {
  Serial.println(F("Pin report (motors off):"));
  int allLow = 0;
  for (int i = 0; i < N; i++) {
    Motor &m = motors[i];
    unsigned long a0 = edges(edgesA, i), b0 = edges(edgesB, i);
    delay(300);
    unsigned long ra = edges(edgesA, i) - a0, rb = edges(edgesB, i) - b0;
    Serial.print(F("  ")); Serial.print(m.name); Serial.print(F(" encoder"));
    bool low = true;
    const uint8_t pins[2] = {m.encA, m.encB};
    for (uint8_t pin : pins) {
      int up = pinHigh(pin);
      pinMode(pin, INPUT); delay(2); int down = pinHigh(pin); pinMode(pin, INPUT_PULLUP); delayMicroseconds(50);
      Serial.print(pin == m.encA ? "  A D" : "  B D"); Serial.print(pin); Serial.print(' ');
      Serial.print(!up ? "low" : down ? "high" : "high on the pull-up only (not driven: wire open or encoder off?)");
      low &= !up;
    }
    allLow += low;
    Serial.print(F(", edges in 300 ms at rest A ")); Serial.print(ra); Serial.print(F(" B ")); Serial.print(rb);
    if (ra + rb > 10) Serial.print(F(" - NOISY (floating or unpowered encoder?)"));
    if (stormed[i]) Serial.print(F(" - FLOODED, interrupts off"));
    Serial.print(F("   PWM D")); Serial.print(PWM_PINS[i][0]); Serial.print('='); Serial.print(pinHigh(PWM_PINS[i][0]));
    Serial.print(F(" D")); Serial.print(PWM_PINS[i][1]); Serial.print('='); Serial.println(pinHigh(PWM_PINS[i][1]));
  }
  if (allLow == N) Serial.println(F("  All six encoder wires read low: encoder 5V likely off (5V Wago / buck)"));
  int z = panelZ();
  Serial.print(F("  Panel z ")); Serial.print(z); Serial.print(z > TOUCH_Z ? " (untouched)" : " (pressed, or shorted)");
  Serial.print(F(", raw x ")); Serial.print(readAxis(XA, XB, YA)); Serial.print(F(" y ")); Serial.print(readAxis(YA, YB, XA));
  Serial.println(F(" (A0-A3)"));
  Serial.print(F("  MODE button D30 ")); Serial.println(pinHigh(MODE_PIN) ? F("high (not pressed)") : F("LOW: pressed, or shorted to GND"));
  Serial.print(F("  POWER button D31 ")); Serial.println(pinHigh(POWER_PIN) ? F("HIGH: pressed, or its NC wire not connected (ignored until it reads low)") : F("low (not pressed)"));
  Serial.print(F("  Pololu OFF D32 ")); Serial.println(pinHigh(OFF_PIN) ? F("HIGH (should be low!)") : F("low"));
}

// MODE button: each press moves to the next pattern (like n); press and release logged (20 ms debounce).
unsigned int modePresses = 0;
// POWER button: acts only once D31 has read low (at rest) since boot, so an unplugged NC wire (reads
// high) or a press held through power-on can't power it straight back off.
bool powerArmed = false;
void powerButton() {
  static bool down = false; static unsigned long changed = 0, pressedAt = 0;
  bool now = pinHigh(POWER_PIN);
  if (!powerArmed && !now && !down) { powerArmed = true; Serial.println(F("POWER button D31 reads low (at rest): armed")); }
  if (now == down) { changed = millis(); return; }
  if (millis() - changed < 20) return;
  down = now;
  if (!down) {
    if (!powerArmed) { powerArmed = true; Serial.println(F("POWER button D31 reads low (at rest): armed")); return; }
    Serial.print(F("POWER button released after ")); Serial.print(millis() - pressedAt); Serial.println(F(" ms"));
    return;
  }
  pressedAt = millis();
  if (!powerArmed) { Serial.println(F("POWER button D31 high, not armed yet (held at power-on, or NC wire off) - ignored")); return; }
  bool no12V = stormed[0] && stormed[1] && stormed[2];         // every encoder flooding = no 5 V = no 12 V
  if (!strcmp(phase, "off") || (!strcmp(phase, "stopped") && no12V)) {   // 12 V was off: this press turned
    Serial.println(F("POWER button pressed while off: 12 V back on - homing"));    // it on, so home
    phase = "idle"; pending = '\n';
    return;
  }
  Serial.println(F("POWER button pressed - parking, then power off"));
  offRequested = true;
}
void buttonLog() {
  powerButton();
  static bool down = false; static unsigned long changed = 0, pressedAt = 0;
  bool now = !pinHigh(MODE_PIN);
  if (now == down) { changed = millis(); return; }
  if (millis() - changed < 20) return;   // has to stay changed 20 ms
  down = now;
  if (down) {
    pressedAt = millis(); modePresses++;
    pattern = (pattern + 1) % N_PATTERNS; patternT = 0;
    Serial.print(F("MODE button pressed (#")); Serial.print(modePresses); Serial.print(F(") - pattern: ")); Serial.println(PATTERNS[pattern]);
  }
  else { Serial.print(F("MODE button released after ")); Serial.print(millis() - pressedAt); Serial.println(F(" ms")); }
}

// One line a second (5 s while balancing, none during the raise): phase, then per motor angle, drive,
// encoder pin levels, edges per second on each wire, bad edges, and the panel's touch reading.
bool heartbeatOn = true;
void heartbeat() {
  static unsigned long last = 0, lastE[N][2];
  unsigned long now = millis(), every = mode == BALANCE ? 5000 : 1000;
  if (!heartbeatOn || streaming || raising || now - last < every) return;
  float secs = (now - last) / 1000.0;
  last = now;
  Serial.print(F("HB ")); Serial.print(now / 1000.0, 1); Serial.print(' '); Serial.print(phase);
  for (int i = 0; i < N; i++) {
    Motor &m = motors[i];
    unsigned long a = edges(edgesA, i), b = edges(edgesB, i);
    int d = *m.ocrA ? (int)*m.ocrA : -(int)*m.ocrB;
    Serial.print(F(" | M")); Serial.print(i + 1); Serial.print(' '); Serial.print(cwDeg(m), 1); Serial.print(F("deg "));
    Serial.print(-m.ccw * d * 100L / PWM_TOP); Serial.print(F("% A")); Serial.print(pinHigh(m.encA)); Serial.print('B'); Serial.print(pinHigh(m.encB));
    Serial.print(' '); Serial.print((a - lastE[i][0]) / secs, 0); Serial.print('/'); Serial.print((b - lastE[i][1]) / secs, 0);
    Serial.print(F("e/s bad ")); Serial.print(badEdges[i]);
    if (stormed[i]) Serial.print(F(" FLOOD")); else if (encFault[i]) Serial.print(F(" FAULT")); else if (!m.on) Serial.print(F(" off"));
    lastE[i][0] = a; lastE[i][1] = b;
  }
  if (mode != BALANCE) { Serial.print(F(" | panel z ")); Serial.print(panelZ()); }
  Serial.print(F(" | MODE ")); Serial.print(pinHigh(MODE_PIN) ? F("up") : F("DOWN")); Serial.print(F(" presses ")); Serial.print(modePresses);
  Serial.print(F(" | POWER ")); Serial.print(pinHigh(POWER_PIN) ? F("DOWN") : F("up")); if (!powerArmed) Serial.print(F(" (not armed)"));
  Serial.println();
}

void status() {
  static unsigned long last = 0;
  if (!consoleOn || streaming || millis() - last < 200) return;    // nobody listening (or streaming): don't print
  last = millis();
  Serial.print(mode == BALANCE ? "bal" : "lvl");
  if (pattern) { Serial.print(F(" [")); Serial.print(PATTERNS[pattern]); Serial.print(F("]")); }
  if (tracking) { Serial.print(F(" ball ")); Serial.print(bx * 1000, 0); Serial.print(F(",")); Serial.print(by * 1000, 0); Serial.print(F(" mm")); }
  else Serial.print(F(" no ball"));
  Serial.print(F(" tilt ")); Serial.print(tiltRoll * RAD_TO_DEG, 1); Serial.print(F(",")); Serial.print(tiltPitch * RAD_TO_DEG, 1);
  Serial.print(F(" |"));
  for (auto &m : motors) {                              // angle, target, drive % (+ = up)
    int d = *m.ocrA ? (int)*m.ocrA : -(int)*m.ocrB;     // + = input A
    Serial.print(F(" ")); Serial.print(cwDeg(m), 1); Serial.print(F("/")); Serial.print(-m.sp / COUNTS_PER_DEG, 1);
    Serial.print(F(" ")); Serial.print(-m.ccw * d * 100L / PWM_TOP); Serial.print(F("%"));
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
  if (mode == IDLE) { Serial.println(F("Run Enter first")); return; }
  mode = LEVEL;
  const float T = 6 * DEG_TO_RAD;
  float r0, t0, r1, t1, r2, t2;
  Serial.println(F("Calibrating: ball near the centre please"));
  if (!runFor(1500) || !panelAvg(r0, t0)) { Serial.println(F("  no ball")); return; }
  testPitch = T; testRoll = 0;                       // +pitch drops the +x edge: ball rolls +x
  if (!runFor(900) || !panelAvg(r1, t1)) { Serial.println(F("  lost the ball")); testPitch = 0; return; }
  testPitch = 0; testRoll = T;                       // +roll lifts the +y edge: ball rolls -y
  if (!runFor(900) || !panelAvg(r2, t2)) { Serial.println(F("  lost the ball")); testRoll = 0; return; }
  testRoll = 0;
  float dxr = r1 - r0, dxt = t1 - t0, dyr = -(r2 - r1), dyt = -(t2 - t1);
  float nx = sqrt(dxr * dxr + dxt * dxt), ny = sqrt(dyr * dyr + dyt * dyt);
  Serial.print(F("  start ")); Serial.print(r0, 0); Serial.print(F(",")); Serial.print(t0, 0);
  Serial.print(F("  after +x tilt ")); Serial.print(r1, 0); Serial.print(F(",")); Serial.print(t1, 0);
  Serial.print(F("  after +roll ")); Serial.print(r2, 0); Serial.print(F(",")); Serial.println(t2, 0);
  if (nx < 15 || ny < 15) { Serial.println(F("  ball barely moved; try again")); return; }
  EX_R = dxr / nx; EX_T = dxt / nx; EY_R = dyr / ny; EY_T = dyt / ny;
  Serial.print(F("  +x moved ")); Serial.print(nx, 0); Serial.print(F(" mm, +y moved ")); Serial.print(ny, 0);
  Serial.print(F(" mm. EX_R ")); Serial.print(EX_R, 2); Serial.print(F(" EX_T ")); Serial.print(EX_T, 2);
  Serial.print(F(" EY_R ")); Serial.print(EY_R, 2); Serial.print(F(" EY_T ")); Serial.println(EY_T, 2);
  Serial.print(F("  axes ")); Serial.println(EX_R * EY_T - EX_T * EY_R > 0 ? "not mirrored" : "MIRRORED");
  runFor(1000);
}

// Hold-the-ball calibration, one point per command: 'k' + motor number records where the ball sits
// (held still over that motor); after all three, the axes and centre are computed.
// +x points at motor 1 (away from the midpoint of 2 and 3), +y from motor 3 toward motor 2.
float calR[3], calT[3]; bool calHave[3] = {false, false, false};
void holdPoint(int i) {
  float r, t;
  if (!panelAvg(r, t)) { Serial.println(F("  no ball on the panel")); return; }
  for (int j = 0; j < 3; j++)
    if (j != i && calHave[j] && abs(r - calR[j]) + abs(t - calT[j]) < 40) {
      Serial.print(F("  too close to motor ")); Serial.print(j + 1); Serial.println(F("'s point: move the ball over this motor"));
      return;
    }
  calR[i] = r; calT[i] = t; calHave[i] = true;
  Serial.print(F("  motor ")); Serial.print(i + 1); Serial.print(F(" at ")); Serial.print(r, 0); Serial.print(F(",")); Serial.println(t, 0);
  if (!(calHave[0] && calHave[1] && calHave[2])) return;
  float xr = calR[0] - (calR[1] + calR[2]) / 2, xt = calT[0] - (calT[1] + calT[2]) / 2;
  float yr = calR[1] - calR[2], yt = calT[1] - calT[2];
  float nx = sqrt(xr * xr + xt * xt), ny = sqrt(yr * yr + yt * yt);
  EX_R = xr / nx; EX_T = xt / nx; EY_R = yr / ny; EY_T = yt / ny;
  CX = (calR[0] + calR[1] + calR[2]) / 3; CY = (calT[0] + calT[1] + calT[2]) / 3;
  Serial.print(F("Centre ")); Serial.print(CX, 1); Serial.print(F(",")); Serial.print(CY, 1);
  Serial.print(F("  EX_R ")); Serial.print(EX_R, 2); Serial.print(F(" EX_T ")); Serial.print(EX_T, 2);
  Serial.print(F(" EY_R ")); Serial.print(EY_R, 2); Serial.print(F(" EY_T ")); Serial.print(EY_T, 2);
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
  for (int i = 0; i < N; i++) {
    pinMode(motors[i].encA, INPUT_PULLUP); pinMode(motors[i].encB, INPUT_PULLUP);
    attachEncoder(i);
  }
  floatPanel();
  pinMode(MODE_PIN, INPUT_PULLUP);
  pinMode(POWER_PIN, INPUT_PULLUP);
  digitalWrite(OFF_PIN, LOW); pinMode(OFF_PIN, OUTPUT);
  Serial.println(F("Balancer. Homing at power-on, then balancing. s = stop, Enter = home again, l = level only, b = balance."));
  bootLog(cause);
  if (cfgLoad()) { Serial.println(F("Saved settings loaded from EEPROM (:defaults to clear):")); printSettings(); }
  Serial.print(F("Pins:"));
  for (int i = 0; i < N; i++) {
    Serial.print(i ? " |" : ""); Serial.print(' '); Serial.print(motors[i].name); Serial.print(F(" PWM D")); Serial.print(PWM_PINS[i][0]);
    Serial.print(F("/D")); Serial.print(PWM_PINS[i][1]); Serial.print(F(" enc D")); Serial.print(motors[i].encA); Serial.print(F("/D")); Serial.print(motors[i].encB);
  }
  Serial.println(F(" | panel A0-A3"));
  pinReport();
  phase = "idle";
  pending = '\n';                       // power-on: home and start straight away
}

// Home, then raise straight to the start position: the level pose with each leg's trim
// (motor 1/2/3 at about 33/26/25 deg from home).
void startUp() {
  if (!homeAll()) { allOff(); phase = "stopped"; Serial.println(F("Homing stopped - motors off")); return; }
  logStage(1);
  float lvl[N], start[N];
  armAngles(0, 0, lvl);
  for (int i = 0; i < N; i++) start[i] = armToCw(lvl[i], i);
  driveLimit = brownedOut ? PWM_TOP * 6L / 10 : PWM_TOP;   // full 12 V, unless that just sagged the supply
  raising = true; phase = "raising";
  Serial.print(F("Raising to start (deg up):")); for (float d : start) { Serial.print(' '); Serial.print(d, 1); } Serial.println();
  if (brownedOut) Serial.println(F("Last reset was a power sag - raising at 60%"));
  bool ok = moveAllTo(start, RAISE_SPEED);
  driveLimit = DUTY_MAX;
  raising = false;
  if (!ok || !runFor(500)) { allOff(); phase = "stopped"; Serial.println(F("Raise stopped")); return; }   // settle before balancing
  Serial.print(F("Raised, arms at (deg up):")); for (auto &m : motors) { Serial.print(' '); Serial.print(cwDeg(m), 1); } Serial.println();
  tiltRoll = tiltPitch = testRoll = testPitch = 0;
  mode = BALANCE; phase = "running";     // level until it sees the ball, then balance it
  logStage(2);
  Serial.println(F("At start. Balancing as soon as the ball is on the plate."));
}

void handle(char c) {
  const float T = 5 * DEG_TO_RAD;
  switch (c) {
    case '\n': case 'g': startUp(); break;
    case 'b': if (mode != IDLE) { testRoll = testPitch = 0; mode = BALANCE; Serial.println(F("Balancing")); } break;
    case 'l': if (mode != IDLE) { testRoll = testPitch = 0; mode = LEVEL; Serial.println(F("Level")); } break;
    case 'x': testPitch = T; testRoll = 0; Serial.println(F("Tilt: +5 pitch (+x edge should drop)")); break;
    case 'X': testPitch = -T; testRoll = 0; Serial.println(F("Tilt: -5 pitch")); break;
    case 'y': testRoll = T; testPitch = 0; Serial.println(F("Tilt: +5 roll (+y edge should rise)")); break;
    case 'Y': testRoll = -T; testPitch = 0; Serial.println(F("Tilt: -5 roll")); break;
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
      Serial.print(F("Trims (deg up): ")); for (float o : LEG_OFFSET_DEG) { Serial.print(o, 1); Serial.print(F(" ")); } Serial.println();
    } break;
    case 'P': case 'p': case 'D': case 'd': case 'I': case 'i': case '0': case 'v': case 'M': case 'm': {
      float f = (c >= 'A' && c <= 'Z') ? 1.15 : 1 / 1.15;
      if (c == 'P' || c == 'p') BALL_KP *= f;
      if (c == 'D' || c == 'd') BALL_KD *= f;
      if (c == 'I' || c == 'i') BALL_KI = BALL_KI < 0.05 && c == 'I' ? 0.3 : BALL_KI * f;
      if (c == '0') BALL_KI = 0;
      if (c == 'M' || c == 'm') KD_M *= f;
      Serial.print(F("Gains  P ")); Serial.print(BALL_KP, 2); Serial.print(F("  I ")); Serial.print(BALL_KI, 2);
      Serial.print(F("  D ")); Serial.print(BALL_KD, 2); Serial.print(F("   motor damping M ")); Serial.println(KD_M, 2);
      Serial.print(F("Timing: ball step avg ")); Serial.print(ballSteps ? ballUsSum / ballSteps : 0); Serial.print(F(" us, worst ")); Serial.print(ballUsMax); Serial.print(F(" us, worst tick ")); Serial.print(tickUsMax);
      Serial.print(F(" us (budget 2000), late ticks ")); Serial.print(lateTicks);
      Serial.print(F("   last: panel ")); Serial.print(panelUs); Serial.print(F(" us, worst leg IK ")); Serial.print(ikUs); Serial.println(F(" us")); ikUs = 0;
      Serial.print(F("Panel: coasted ")); Serial.print(coasted); Serial.print(F(" frames, rejected ")); Serial.print(rejected); Serial.println(F(" jumps"));
      coasted = rejected = 0;
      ballUsMax = tickUsMax = lateTicks = ballUsSum = ballSteps = 0;
    } break;
    case 'T': EEPROM.update(TRACE_AT - 2, 1); Serial.println(F("Trace armed: the next run without the console records 1.5 s, then stops")); break;
    case 'R': streaming = !streaming; Serial.println(streaming ? "Stream on" : "Stream off"); break;
    case 'a': case 'f': case 'w': case 'z':
      if (c == 'a') CENTRE_X -= 0.003; if (c == 'f') CENTRE_X += 0.003;
      if (c == 'z') CENTRE_Y -= 0.003; if (c == 'w') CENTRE_Y += 0.003;
      Serial.print(F("Centre ")); Serial.print(CENTRE_X * 1000, 0); Serial.print(F(", ")); Serial.print(CENTRE_Y * 1000, 0); Serial.println(F(" mm"));
      break;
    case '[': case ']':
      AXIS_ROT_DEG += c == ']' ? 2 : -2;
      Serial.print(F("Axis rotation ")); Serial.print(AXIS_ROT_DEG, 0); Serial.println(F(" deg"));
      break;
    case '>': case '<':
      FF_LEAD = constrain(FF_LEAD + (c == '>' ? 0.02 : -0.02), 0.0, 0.3);
      Serial.print(F("Feed-forward lead ")); Serial.print(FF_LEAD * 1000, 0); Serial.println(F(" ms"));
      break;
    case 'n':
      pattern = (pattern + 1) % N_PATTERNS; patternT = 0;
      Serial.print(F("Pattern: ")); Serial.println(PATTERNS[pattern]);
      break;
    case 'e': if (mode == IDLE) pinReport(); else Serial.println(F("Pin report only with the motors off (s first)")); break;
    case 'h': heartbeatOn = !heartbeatOn; Serial.println(heartbeatOn ? "Heartbeat on" : "Heartbeat off"); break;
    case '?': { float x, y; if (readBall(x, y)) { Serial.print(F("ball ")); Serial.print(x, 0); Serial.print(F(", ")); Serial.print(y, 0); Serial.println(F(" mm")); } else Serial.println(F("no ball")); } break;
  }
}

// ---- Typed commands and saved settings -------------------------------------------------------
// Saved block at EEPROM 2048 (boot log 0-552, trace 598-1950): magic, then these floats in order.
const int CFG_AT = 2048; const uint16_t CFG_MAGIC = 0xBA11;
float *const CFG[] = {&CENTRE_X, &CENTRE_Y, &BALL_KP, &BALL_KI, &BALL_KD, &KD_M, &FF_LEAD, &AXIS_ROT_DEG,
                      &LEG_OFFSET_DEG[0], &LEG_OFFSET_DEG[1], &LEG_OFFSET_DEG[2], &ILC_ON, &ILC_GAIN, &ILC_LEAD,
                      &SHAPE[0], &SHAPE[1], &SHAPE[2], &SHAPE[3], &SHAPE[4], &SHAPE[5], &SHAPE[6], &SHAPE[7]};
const int N_CFG = sizeof(CFG) / sizeof(CFG[0]);
void cfgSave() {
  int a = CFG_AT + 2;
  for (float *v : CFG) { EEPROM.put(a, *v); a += 4; }
  for (int i = 0; i < N_PATTERNS; i++) { EEPROM.put(a, PAT_AX[i]); EEPROM.put(a + 4, PAT_AY[i]); EEPROM.put(a + 8, PAT_T[i]); a += 12; }
  EEPROM.put(CFG_AT, CFG_MAGIC);
}
bool cfgLoad() {
  uint16_t m; EEPROM.get(CFG_AT, m);
  if (m != CFG_MAGIC) return false;
  int a = CFG_AT + 2;
  for (float *v : CFG) { EEPROM.get(a, *v); a += 4; }
  for (int i = 0; i < N_PATTERNS; i++) { EEPROM.get(a, PAT_AX[i]); EEPROM.get(a + 4, PAT_AY[i]); EEPROM.get(a + 8, PAT_T[i]); a += 12; }
  return true;
}
// Name, value, and how a typed number maps to it (stored = typed * scale).
struct Setting { const char *key; float *v; float scale; };
const Setting SETTINGS[] = {
  {"cx", &CENTRE_X, 0.001}, {"cy", &CENTRE_Y, 0.001}, {"kp", &BALL_KP, 1}, {"ki", &BALL_KI, 1}, {"kd", &BALL_KD, 1},
  {"md", &KD_M, 1}, {"lead", &FF_LEAD, 0.001}, {"rot", &AXIS_ROT_DEG, 1},
  {"t1", &LEG_OFFSET_DEG[0], 1}, {"t2", &LEG_OFFSET_DEG[1], 1}, {"t3", &LEG_OFFSET_DEG[2], 1},
  {"ilc", &ILC_ON, 1}, {"ilcg", &ILC_GAIN, 1}, {"ilclead", &ILC_LEAD, 1},
  {"h1c", &SHAPE[0], 1}, {"h1s", &SHAPE[1], 1}, {"h2c", &SHAPE[2], 1}, {"h2s", &SHAPE[3], 1},
  {"h3c", &SHAPE[4], 1}, {"h3s", &SHAPE[5], 1}, {"h4c", &SHAPE[6], 1}, {"h4s", &SHAPE[7], 1},
};
void printSettings() {
  Serial.print(F("cfg mode=")); Serial.print(PATTERNS[pattern]);
  for (const Setting &p : SETTINGS) { Serial.print(' '); Serial.print(p.key); Serial.print('='); Serial.print(*p.v / p.scale, 2); }
  Serial.println();
  for (int i = 1; i < N_PATTERNS; i++) {
    Serial.print(F("pattern ")); Serial.print(PATTERNS[i]); Serial.print(F(" ax=")); Serial.print(PAT_AX[i], 1);
    Serial.print(F(" ay=")); Serial.print(PAT_AY[i], 1); Serial.print(F(" per=")); Serial.println(PAT_T[i], 2);
  }
}
void lineCommand(char *s) {
  char *key = strtok(s, " =\t"), *arg = strtok(NULL, " =\t");
  if (!key) return;
  if (!strcmp(key, "get")) { printSettings(); return; }
  if (!strcmp(key, "ilcclear")) { ilcClear(); Serial.println(F("= ilc cleared")); return; }
  if (!strcmp(key, "park")) { parkRequested = true; Serial.println(F("= park")); return; }
  if (!strcmp(key, "off")) {           // :off <bin 0-99> <dx mm> <dy mm> - one point of the shape table
    char *a2 = strtok(NULL, " =\t"), *a3 = strtok(NULL, " =\t");
    int i = arg ? atoi(arg) : -1;
    if (i < 0 || i >= ILC_N || !a2 || !a3) { Serial.println(F("? off <0-99> <dx mm> <dy mm>")); return; }
    ilcX[i] = constrain(atof(a2) / 1000, -0.03, 0.03); ilcY[i] = constrain(atof(a3) / 1000, -0.03, 0.03);
    if (i == ILC_N - 1) { Serial.print(F("= off table set for ")); Serial.println(PATTERNS[pattern]); }
    return;
  }
  if (!strcmp(key, "offs")) {          // the table on one line: dx,dy per bin in 0.1 mm
    Serial.print(F("offs ")); Serial.print(PATTERNS[pattern]);
    for (int i = 0; i < ILC_N; i++) { Serial.print(' '); Serial.print((int)round(ilcX[i] * 1e4)); Serial.print(','); Serial.print((int)round(ilcY[i] * 1e4)); }
    Serial.println();
    return;
  }
  if (!strcmp(key, "viz")) { vizOn = arg && atoi(arg); Serial.print(F("= viz ")); Serial.println(vizOn); return; }
  if (!strcmp(key, "save")) { cfgSave(); offSave(); Serial.println(F("= saved (kept through a reset)")); return; }
  if (!strcmp(key, "defaults")) {
    EEPROM.put(CFG_AT, (uint16_t)0);
    for (int p = 1; p <= 3; p++) EEPROM.put(offSlot(p), (uint16_t)0);
    Serial.println(F("= saved settings cleared: reset for the firmware values"));
    return;
  }
  if (!strcmp(key, "mode")) {
    if (!arg) { Serial.print(F("= mode ")); Serial.println(PATTERNS[pattern]); return; }
    int m = -1;
    for (int i = 0; i < N_PATTERNS; i++) if (!strcmp(arg, PATTERNS[i])) m = i;
    if (m < 0 && isdigit(arg[0])) m = atoi(arg);
    if (m < 0 || m >= N_PATTERNS) { Serial.println(F("? mode: centre circle figure-8 square wander (or 0-4)")); return; }
    pattern = m; patternT = 0;
    Serial.print(F("= mode ")); Serial.println(PATTERNS[pattern]);
    return;
  }
  float *v = NULL, scale = 1;
  if (!strcmp(key, "ax")) v = &PAT_AX[pattern];
  else if (!strcmp(key, "ay")) v = &PAT_AY[pattern];
  else if (!strcmp(key, "per")) v = &PAT_T[pattern];
  else for (const Setting &p : SETTINGS) if (!strcmp(key, p.key)) { v = p.v; scale = p.scale; }
  if (!v) { Serial.print(F("? unknown: ")); Serial.println(key); return; }
  if (arg) *v = atof(arg) * scale;
  if (v == &PAT_T[pattern] && *v < 0.5) *v = 0.5;
  Serial.print(F("= ")); Serial.print(key);
  if (v == &PAT_AX[pattern] || v == &PAT_AY[pattern] || v == &PAT_T[pattern]) { Serial.print(F(" (")); Serial.print(PATTERNS[pattern]); Serial.print(')'); }
  Serial.print(' '); Serial.println(*v / scale, 2);
}

// Lower every working arm to its low stop slowly, then motors off. Not homed (angles unknown): no
// park, the arms just come down under gravity.
const float PARK_SPEED = 15;           // deg/s
void parkArms() {
  shuttingDown = true; phase = "parking";
  if (homedOk) {
    for (int i = 0; i < N; i++) if (!encFault[i]) { motors[i].on = true; motors[i].integ = 0; motors[i].sp = pos(motors[i]); }
    float park[N], from[N];
    for (int i = 0; i < N; i++) { park[i] = CW_MIN; from[i] = cwDeg(motors[i]); }
    Serial.print(F("Parking from (deg up):")); for (float d : from) { Serial.print(' '); Serial.print(d, 1); }
    Serial.print(F(" to ")); Serial.print(CW_MIN, 0); Serial.print(F(" at ")); Serial.print(PARK_SPEED, 0); Serial.println(F(" deg/s"));
    bool ok = moveAllTo(park, PARK_SPEED);
    Serial.print(ok ? F("  parked at (deg up):") : F("  park interrupted at (deg up):"));
    for (auto &m : motors) { Serial.print(' '); Serial.print(cwDeg(m), 1); } Serial.println();
  } else Serial.println(F("Not homed - no park, motors off"));
  allOff();
  delay(300);
  shuttingDown = false;
}
// :park - park and stay put, motors off, 12 V on (e.g. before flashing). Enter (home) starts again.
void park() {
  parkArms();
  parkRequested = false; phase = "parked";
  Serial.println(F("Parked, motors off. Safe to flash. 'home' (Enter) to start again"));
}
// POWER press: park, then pulse the Pololu OFF.
void powerOff() {
  parkArms();
  shuttingDown = true;
  Serial.println(F("Power off: D32 pulse to the Pololu OFF")); Serial.flush();
  digitalWrite(OFF_PIN, HIGH); delay(100); digitalWrite(OFF_PIN, LOW);
  delay(200);
  // Still here = the Mega runs on USB. The 12 V is off; a POWER press turns it back on and re-homes.
  Serial.println(F("Still running (on USB): 12 V should be off now. POWER press = on again and home"));
  offRequested = shuttingDown = false; homedOk = false; phase = "off";
}

void loop() {
  if (offRequested) { powerOff(); return; }
  if (parkRequested) { park(); return; }
  if (pending) { char c = pending; pending = 0; handle(c); }
  else { int k = readKey(); if (k >= 0) handle(k); }
  if (mode != IDLE) runLoops();
}
