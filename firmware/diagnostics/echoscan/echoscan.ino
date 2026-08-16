// Where is ECHO actually connected?
//
// Rather than guessing which header pin the wire is in, fire one trigger and
// watch every plausible input at once. Whichever pin carries a pulse is the
// one the sensor is wired to.
//
// The 34/35/36/39 block is the usual suspect: they sit next to each other on
// the header, and 36 and 39 are silkscreened VP and VN rather than by number,
// which is exactly how a wire ends up one position off.

#define TRIG 2

const int CANDIDATES[] = {34, 35, 36, 39, 32, 33, 25, 26, 27, 14, 12, 13, 15, 16};
const int N = sizeof(CANDIDATES) / sizeof(CANDIDATES[0]);

void setup() {
  Serial.begin(115200);
  delay(900);
  Serial.println("\n\n=== ECHO pin scanner ===");
  Serial.println("TRIG on GPIO2, watching every candidate at once.");
  Serial.println("Aim the sensor at a wall roughly 30cm away.\n");
  pinMode(TRIG, OUTPUT);
  digitalWrite(TRIG, LOW);
  for (int i = 0; i < N; i++) pinMode(CANDIDATES[i], INPUT);
  delay(200);
}

void loop() {
  int  hits[N]  = {0};
  long width[N] = {0};

  for (int shot = 0; shot < 5; shot++) {
    int last[N], rose[N];
    unsigned long tRise[N];
    for (int i = 0; i < N; i++) {
      last[i] = digitalRead(CANDIDATES[i]); rose[i] = 0; tRise[i] = 0;
    }

    digitalWrite(TRIG, LOW);  delayMicroseconds(4);
    digitalWrite(TRIG, HIGH); delayMicroseconds(10);
    digitalWrite(TRIG, LOW);

    unsigned long t0 = micros();
    while (micros() - t0 < 30000) {
      for (int i = 0; i < N; i++) {
        int now = digitalRead(CANDIDATES[i]);
        if (!last[i] && now && !rose[i]) { rose[i] = 1; tRise[i] = micros(); }
        if (last[i] && !now && rose[i] && !width[i]) width[i] = micros() - tRise[i];
        last[i] = now;
      }
    }
    for (int i = 0; i < N; i++) if (rose[i]) hits[i]++;
    delay(120);
  }

  bool found = false;
  for (int i = 0; i < N; i++) {
    if (hits[i]) {
      found = true;
      Serial.printf("GPIO%-3d  %d/5 pulses  %6ld us  ->  %.1f cm   <<<< ECHO IS HERE\n",
                    CANDIDATES[i], hits[i], width[i], width[i] * 0.0343 / 2.0);
    }
  }
  if (!found) {
    Serial.println("nothing responded on any of the 14 pins.");
    Serial.println("  -> the sensor is not answering the trigger at all, so this is");
    Serial.println("     not about which pin ECHO is in. Check, in order:");
    Serial.println("       5V across the sensor's own VCC and GND pins");
    Serial.println("       its GND tied to the ESP32 GND");
    Serial.println("       the TRIG wire really on GPIO2");
  }
  Serial.println();
  delay(1500);
}
