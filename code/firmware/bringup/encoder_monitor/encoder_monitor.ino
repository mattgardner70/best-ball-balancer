// Encoder monitor: motors held off; prints each encoder's count and raw A/B pin levels every 200 ms.
// Turn the arms by hand: the count should change and A/B should flicker between 0 and 1.
//
// Motor   Encoder (blue A, purple B)
//   1     D2, D19
//   2     D3, D20
//   3     D18, D21

struct Enc { const char *name; uint8_t a, b; volatile long count; uint8_t prev; };
Enc enc[3] = {{"M1", 2, 19}, {"M2", 3, 20}, {"M3", 18, 21}};

static const int8_t QUAD[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
void edge(Enc &e) {
  uint8_t s = (digitalRead(e.a) << 1) | digitalRead(e.b);
  e.count += QUAD[(e.prev << 2) | s];
  e.prev = s;
}
void e0() { edge(enc[0]); }
void e1() { edge(enc[1]); }
void e2() { edge(enc[2]); }

void setup() {
  const uint8_t motorPins[] = {6, 7, 8, 44, 45, 46};      // MDD3A inputs: hold low (motors off)
  for (uint8_t p : motorPins) { digitalWrite(p, LOW); pinMode(p, OUTPUT); }
  Serial.begin(115200);
  for (auto &e : enc) {
    pinMode(e.a, INPUT_PULLUP); pinMode(e.b, INPUT_PULLUP);
    e.prev = (digitalRead(e.a) << 1) | digitalRead(e.b);
  }
  attachInterrupt(digitalPinToInterrupt(2), e0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(19), e0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), e1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(20), e1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(18), e2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(21), e2, CHANGE);
  Serial.println("Encoder monitor (motors off). count [A B]");
}

void loop() {
  Serial.print(millis() / 1000.0, 1); Serial.print("s");
  for (auto &e : enc) {
    noInterrupts(); long c = e.count; interrupts();
    Serial.print("   "); Serial.print(e.name); Serial.print(" "); Serial.print(c);
    Serial.print(" ["); Serial.print(digitalRead(e.a)); Serial.print(digitalRead(e.b)); Serial.print("]");
  }
  Serial.println();
  delay(200);
}
