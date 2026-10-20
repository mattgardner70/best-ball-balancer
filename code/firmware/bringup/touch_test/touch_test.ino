// Touch-panel test: motors held off. At boot it works out which of A0-A3 belong to the same layer
// (don't touch the panel then), then prints raw X/Y (0-1023) about 20 times a second, or "no touch".
//
// Finding the layers is safe: one pin is pulled low, the others are only pulled up by the Mega's
// ~35 kohm internal resistors, so at most ~0.15 mA flows. A pin on the same resistive layer (a few
// hundred ohms) gets dragged low; pins on the other layer stay high (the layers only touch when pressed).

const uint8_t PINS[4] = {A0, A1, A2, A3};
uint8_t xa, xb, ya, yb;                 // layer 1 ends (called X), layer 2 ends (called Y)

void floatAll() { for (uint8_t p : PINS) { pinMode(p, INPUT); digitalWrite(p, LOW); } }

bool findLayers() {
  floatAll();
  pinMode(PINS[0], OUTPUT); digitalWrite(PINS[0], LOW);
  int partner = -1;
  for (int i = 1; i < 4; i++) {
    pinMode(PINS[i], INPUT_PULLUP);
    delay(2);
    analogRead(PINS[i]);
    int v = analogRead(PINS[i]);
    Serial.print("  A0 low -> A"); Serial.print(i); Serial.print(" reads "); Serial.println(v);
    if (v < 200) partner = partner < 0 ? i : 99;       // 99 = more than one: someone's touching
    pinMode(PINS[i], INPUT); digitalWrite(PINS[i], LOW);
  }
  floatAll();
  if (partner < 1 || partner > 3) return false;
  xa = PINS[0]; xb = PINS[partner];
  int k = 0; uint8_t rest[2];
  for (int i = 1; i < 4; i++) if (i != partner) rest[k++] = PINS[i];
  ya = rest[0]; yb = rest[1];
  return true;
}

// Drive one layer as a voltage divider and read the other layer's voltage at the touch point.
int readAxis(uint8_t hi, uint8_t lo, uint8_t sense, uint8_t idle) {
  floatAll();
  pinMode(hi, OUTPUT); digitalWrite(hi, HIGH);
  pinMode(lo, OUTPUT); digitalWrite(lo, LOW);
  delayMicroseconds(50);
  analogRead(sense);
  int v = analogRead(sense);
  floatAll();
  return v;
}

// Touched if the pulled-up Y layer is dragged down by the grounded X layer.
int zRaw = 1023;
bool touched() {
  floatAll();
  pinMode(xa, OUTPUT); digitalWrite(xa, LOW);
  pinMode(ya, INPUT_PULLUP);
  delayMicroseconds(200);
  analogRead(ya);
  zRaw = analogRead(ya);
  floatAll();
  return zRaw < 900;
}

bool ok = false;

void setup() {
  const uint8_t motorPins[] = {6, 7, 8, 44, 45, 46};    // MDD3A inputs: hold low (motors off)
  for (uint8_t p : motorPins) { digitalWrite(p, LOW); pinMode(p, OUTPUT); }
  Serial.begin(115200);
  Serial.println("Touch test (motors off). Finding the panel layers - don't touch it...");
  ok = findLayers();
  if (!ok) { Serial.println("Couldn't pair the pins (panel touched, or not connected). Send r to retry."); return; }
  Serial.print("Layer X: A"); Serial.print(xa - A0); Serial.print("-A"); Serial.print(xb - A0);
  Serial.print("   Layer Y: A"); Serial.print(ya - A0); Serial.print("-A"); Serial.println(yb - A0);
}

void loop() {
  if (Serial.available() && Serial.read() == 'r') { ok = findLayers(); setup(); return; }
  if (!ok) return;
  if (touched()) {
    int x = readAxis(xa, xb, ya, yb);                  // X layer driven, read on the Y layer
    int y = readAxis(ya, yb, xa, xb);                  // Y layer driven, read on the X layer
    Serial.print("x "); Serial.print(x); Serial.print("   y "); Serial.print(y);
    Serial.print("   z "); Serial.println(zRaw);
  } else {
    Serial.print("no touch (z "); Serial.print(zRaw); Serial.println(")");
  }
  delay(50);
}
