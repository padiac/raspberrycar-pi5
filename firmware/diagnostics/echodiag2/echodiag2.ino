// HC-SR04 diagnostic, round 2.
//
// Round 1 established one thing: ECHO never rises after a trigger. Note that
// ECHO sitting at 0V when idle is NORMAL -- the 2k leg of the divider holds it
// there. The fault is only that it never goes high.
//
// So the sensor is not answering. This narrows down why:
//
//   PHASE 1  blinks the onboard LED on GPIO2, which is also TRIG on this
//            wiring. If the LED does not blink, TRIG is not being driven and
//            nothing else matters.
//   PHASE 2  tries several trigger widths and a much longer echo window, in
//            case a clone module wants more than the datasheet 10us.
//   PHASE 3  watches ECHO for a whole second to see if it EVER moves.
//
// Upload, open Serial Monitor at 115200, and watch the board while phase 1
// runs.

#define TRIG 2
#define ECHO 34

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println();
  Serial.println("========================================");
  Serial.println(" HC-SR04 diagnostic");
  Serial.println("========================================");
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  digitalWrite(TRIG, LOW);
  delay(200);

  // ---- PHASE 1: is TRIG actually being driven? ----
  // GPIO2 has the onboard LED on most ESP32 DevKits. Watch the board.
  Serial.println("\n[1] Blinking GPIO2 (TRIG) 6 times, ~0.5s each.");
  Serial.println("    WATCH THE BOARD. Does the small blue LED blink?");
  Serial.println("      blinking  -> TRIG is being driven, wiring/sensor is the problem");
  Serial.println("      dark      -> this board has no LED on GPIO2, inconclusive");
  for (int i = 0; i < 6; i++) {
    digitalWrite(TRIG, HIGH); delay(250);
    digitalWrite(TRIG, LOW);  delay(250);
    Serial.printf("    blink %d\n", i + 1);
  }
  delay(500);
}

// Fire one trigger of the given width, then watch ECHO for up to 60ms.
// Returns the echo width in microseconds, 0 if it never rose.
unsigned long tryTrigger(int triggerUs) {
  digitalWrite(TRIG, LOW);  delayMicroseconds(5);
  digitalWrite(TRIG, HIGH); delayMicroseconds(triggerUs);
  digitalWrite(TRIG, LOW);

  unsigned long start = micros();
  while (digitalRead(ECHO) == LOW) {
    if (micros() - start > 60000) return 0;      // never rose
  }
  unsigned long rose = micros();
  while (digitalRead(ECHO) == HIGH) {
    if (micros() - rose > 60000) break;          // stuck high
  }
  return micros() - rose;
}

void loop() {
  // ---- PHASE 2: several trigger widths ----
  Serial.println("\n[2] Trigger widths:");
  bool any = false;
  int widths[] = {10, 20, 50, 100};
  for (int w : widths) {
    unsigned long echo = tryTrigger(w);
    if (echo) {
      any = true;
      Serial.printf("    %3dus -> echo %lu us  (%.1f cm)\n", w, echo, echo * 0.0343 / 2.0);
    } else {
      Serial.printf("    %3dus -> no echo\n", w);
    }
    delay(100);
  }

  // ---- PHASE 3: does ECHO ever move on its own? ----
  int high = 0, changes = 0, last = digitalRead(ECHO);
  unsigned long t0 = millis();
  while (millis() - t0 < 1000) {
    int now = digitalRead(ECHO);
    if (now) high++;
    if (now != last) changes++;
    last = now;
  }
  Serial.printf("[3] ECHO over 1s: %d transitions, %s\n",
                changes, high ? "sometimes high" : "always low");

  Serial.println("\n---- what this means ----");
  if (any) {
    Serial.println("  WORKING. The sensor answers -- use whichever trigger width above worked.");
  } else if (changes > 0) {
    Serial.println("  ECHO moves but not in response to TRIG.");
    Serial.println("  Likely floating: check the ECHO wire actually reaches the divider.");
  } else {
    Serial.println("  Sensor is not responding at all. In order of likelihood:");
    Serial.println("   1. GND not shared. The sensor's GND must connect to the ESP32's GND,");
    Serial.println("      even when the sensor is powered from the buck module. This is the");
    Serial.println("      most common cause when a separate supply is involved.");
    Serial.println("   2. Sensor VCC is not on 5V. It will not fire on 3.3V.");
    Serial.println("      Measure between the sensor's VCC and GND pins -- expect ~5V.");
    Serial.println("   3. TRIG is not reaching the sensor (check phase 1's LED, then the wire).");
    Serial.println("   4. ECHO is not reaching the divider, or the divider's tap is wrong:");
    Serial.println("        ECHO --[1k]--+--[2k]-- GND, with GPIO34 on the +");
    Serial.println("      Swapped resistors give 1.67V, under the ~2.5V needed to read high.");
    Serial.println("   5. Dead sensor. Try another if you have one.");
  }
  Serial.println("\n(repeating in 3s)");
  delay(3000);
}
