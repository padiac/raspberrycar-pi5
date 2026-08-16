// Why is the HC-SR04 silent? Looks at the ECHO line directly instead of
// through pulseIn, which only ever says yes or no.
//
// Reading it three ways tells the causes apart:
//   idle level      what ECHO sits at with nothing happening
//   after a trigger whether the sensor answers at all
//   wiggle          whether the line is driven or just floating

#define TRIG 2
#define ECHO 34

void setup() {
  Serial.begin(115200);
  delay(600);
  Serial.println();
  Serial.println("=== echo line diagnostic ===");
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  digitalWrite(TRIG, LOW);
  delay(100);
}

void loop() {
  // 1. Idle level, sampled over 20ms with no trigger sent.
  int high = 0;
  for (int i = 0; i < 200; i++) { if (digitalRead(ECHO)) high++; delayMicroseconds(100); }

  // 2. Send a trigger, then watch ECHO for 30ms at ~10us resolution.
  digitalWrite(TRIG, LOW);  delayMicroseconds(4);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  unsigned long t0 = micros();
  int wentHigh = 0;
  unsigned long rose = 0, fell = 0;
  int last = digitalRead(ECHO);
  while (micros() - t0 < 30000) {
    int now = digitalRead(ECHO);
    if (!last && now) { wentHigh++; if (!rose) rose = micros() - t0; }
    if (last && !now && rose && !fell) fell = micros() - t0;
    last = now;
  }

  Serial.printf("idle %3d/200 high | edges after trigger: %d", high, wentHigh);
  if (rose) {
    Serial.printf(" | rose at %luus", rose);
    if (fell > rose) Serial.printf(", width %luus (~%.1f cm)", fell - rose, (fell - rose) * 0.0343 / 2.0);
    else Serial.print(", still high at 30ms");
  }
  Serial.println();

  if (high == 0 && wentHigh == 0)
    Serial.println("   -> ECHO never moves. Sensor unpowered, ECHO not connected, or divider shorting it to ground.");
  else if (high >= 190)
    Serial.println("   -> ECHO stuck high. Divider probably wired wrong, or ECHO shorted to 3V3.");
  else if (high > 5 && high < 190)
    Serial.println("   -> ECHO floating and noisy. Likely not actually connected to the sensor.");

  delay(700);
}
