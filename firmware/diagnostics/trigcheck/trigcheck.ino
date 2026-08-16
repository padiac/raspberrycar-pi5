// Does the trigger signal actually reach the sensor?
//
// Fourteen candidate pins showed no echo, so this is no longer about which
// pin ECHO is in -- the sensor is not answering at all. VCC reads 5V and the
// grounds are bonded, which leaves the trigger not arriving, or a dead sensor.
//
// PHASE A  toggles GPIO2 at 1Hz for 15 seconds. Put a multimeter on the
//          SENSOR'S OWN TRIG pin, black lead on ground. It should swing
//          between 0V and about 3.3V once a second. This is the measurement
//          that settles it: if the pin at the sensor never moves, the signal
//          is not getting there and nothing else matters.
//
// PHASE B  tries the trigger on other pins in case the board's silkscreen
//          does not map the way it looks. Motor pins are deliberately left
//          out -- driving a PWM pin high could spin a wheel.

const int TRIG_CANDIDATES[] = {2, 12, 13, 15, 16, 17};
const int NT = sizeof(TRIG_CANDIDATES) / sizeof(TRIG_CANDIDATES[0]);

const int ECHO_CANDIDATES[] = {34, 35, 36, 39};
const int NE = sizeof(ECHO_CANDIDATES) / sizeof(ECHO_CANDIDATES[0]);

void setup() {
  Serial.begin(115200);
  delay(900);
  Serial.println("\n\n=== trigger check ===");

  for (int i = 0; i < NT; i++) { pinMode(TRIG_CANDIDATES[i], OUTPUT); digitalWrite(TRIG_CANDIDATES[i], LOW); }
  for (int i = 0; i < NE; i++) pinMode(ECHO_CANDIDATES[i], INPUT);
  delay(200);

  // ---- PHASE A ----
  Serial.println("\n[A] GPIO2 toggling at 1Hz for 15 seconds.");
  Serial.println("    Multimeter on the SENSOR'S TRIG pin, black lead to ground.");
  Serial.println("    Expect it to swing 0V <-> ~3.3V once a second.");
  Serial.println("    If it never moves, the trigger is not reaching the sensor.\n");
  for (int i = 0; i < 15; i++) {
    digitalWrite(2, HIGH); Serial.printf("    %2d  HIGH (expect ~3.3V)\n", i + 1); delay(500);
    digitalWrite(2, LOW);  Serial.printf("        LOW  (expect 0V)\n");           delay(500);
  }
  digitalWrite(2, LOW);
}

// Pulse one pin, watch all echo candidates for 30ms.
int probe(int trigPin) {
  int found = -1;
  int last[NE];
  for (int i = 0; i < NE; i++) last[i] = digitalRead(ECHO_CANDIDATES[i]);

  digitalWrite(trigPin, LOW);  delayMicroseconds(4);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long t0 = micros();
  while (micros() - t0 < 30000) {
    for (int i = 0; i < NE; i++) {
      int now = digitalRead(ECHO_CANDIDATES[i]);
      if (!last[i] && now) { found = ECHO_CANDIDATES[i]; return found; }
      last[i] = now;
    }
  }
  return found;
}

void loop() {
  Serial.println("\n[B] trying the trigger on each safe pin:");
  bool any = false;
  for (int i = 0; i < NT; i++) {
    int hit = -1;
    for (int shot = 0; shot < 3 && hit < 0; shot++) { hit = probe(TRIG_CANDIDATES[i]); delay(80); }
    if (hit > 0) {
      any = true;
      Serial.printf("    TRIG on GPIO%-3d -> ECHO answered on GPIO%d   <<<< THIS PAIR WORKS\n",
                    TRIG_CANDIDATES[i], hit);
    } else {
      Serial.printf("    TRIG on GPIO%-3d -> nothing\n", TRIG_CANDIDATES[i]);
    }
  }

  if (!any) {
    Serial.println("\n  No pin combination gets a response.");
    Serial.println("  With 5V present and grounds bonded, that points at the sensor itself.");
    Serial.println("  Phase A settles it: if GPIO2 measures 0/3.3V at the sensor's TRIG pin");
    Serial.println("  and it still says nothing, the sensor is dead. Swap it if you have another.");
  }
  Serial.println("\n(repeating in 4s)");
  delay(4000);
}
