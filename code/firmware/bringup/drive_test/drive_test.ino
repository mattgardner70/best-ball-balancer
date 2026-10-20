// Drive test: on Enter, pulses each MDD3A input in turn (25% for 0.3 s) and reports how far that
// motor's encoder moved. Finds which drive path (Mega pin -> MDD3A -> motor) is broken.
//
// Motor   MDD3A          Mega PWM (A, B)   Encoder
//   1     #1 M1A/M1B     D6, D7            D2, D19
//   2     #1 M2A/M2B     D8, D44           D3, D20
//   3     #2 M1A/M1B     D45, D46          D18, D21

const int PWM_TOP = 799;                       // 20 kHz
struct Path { const char *name; volatile uint16_t *ocr; uint8_t pin; };
Path paths[6] = {
  {"M1 A (D6)", &OCR4A, 6}, {"M1 B (D7)", &OCR4B, 7},
  {"M2 A (D8)", &OCR4C, 8}, {"M2 B (D44)", &OCR5C, 44},
  {"M3 A (D45)", &OCR5B, 45}, {"M3 B (D46)", &OCR5A, 46},
};
struct Enc { uint8_t a, b; volatile long count; uint8_t prev; };
Enc enc[3] = {{2, 19}, {3, 20}, {18, 21}};
static const int8_t QUAD[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
void edge(Enc &e) {
  uint8_t s = (digitalRead(e.a) << 1) | digitalRead(e.b);
  e.count += QUAD[(e.prev << 2) | s];
  e.prev = s;
}
void e0() { edge(enc[0]); }
void e1() { edge(enc[1]); }
void e2() { edge(enc[2]); }
long cnt(int i) { noInterrupts(); long c = enc[i].count; interrupts(); return c; }

void setup() {
  OCR4A = OCR4B = OCR4C = OCR5A = OCR5B = OCR5C = 0;
  for (auto &p : paths) pinMode(p.pin, OUTPUT);
  TCCR4A = _BV(COM4A1) | _BV(COM4B1) | _BV(COM4C1) | _BV(WGM41);
  TCCR4B = _BV(WGM43) | _BV(WGM42) | _BV(CS40);
  ICR4 = PWM_TOP;
  TCCR5A = _BV(COM5A1) | _BV(COM5B1) | _BV(COM5C1) | _BV(WGM51);
  TCCR5B = _BV(WGM53) | _BV(WGM52) | _BV(CS50);
  ICR5 = PWM_TOP;
  Serial.begin(115200);
  for (auto &e : enc) {
    pinMode(e.a, INPUT_PULLUP); pinMode(e.b, INPUT_PULLUP);
    e.prev = (digitalRead(e.a) << 1) | digitalRead(e.b);
  }
  void (*isr[3])() = {e0, e1, e2};
  for (int i = 0; i < 3; i++) {
    attachInterrupt(digitalPinToInterrupt(enc[i].a), isr[i], CHANGE);
    attachInterrupt(digitalPinToInterrupt(enc[i].b), isr[i], CHANGE);
  }
  Serial.println("Drive test. POWER on, then Enter: pulses each input 25% for 0.3 s.");
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c != '\n') return;
  for (int i = 0; i < 6; i++) {
    long before[3];
    for (int k = 0; k < 3; k++) before[k] = cnt(k);
    *paths[i].ocr = PWM_TOP * 25 / 100;
    delay(300);
    *paths[i].ocr = 0;
    delay(300);
    Serial.print(paths[i].name); Serial.print(" -> encoder counts moved: M1 ");
    Serial.print(cnt(0) - before[0]); Serial.print("  M2 "); Serial.print(cnt(1) - before[1]);
    Serial.print("  M3 "); Serial.println(cnt(2) - before[2]);
  }
  Serial.println("Done. Enter to run again.");
}
