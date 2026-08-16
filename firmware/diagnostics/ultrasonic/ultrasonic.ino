// Ultrasonic range test. No libraries needed -- run this one first, it proves
// the toolchain, the upload path and the serial monitor all work before any
// motor is involved.
//
// ECHO is on GPIO34, which is input-only on the ESP32 and has no pull-up. That
// is fine here: the HC-SR04 drives the line, and the 1k/2k divider brings its
// 5V output down to something the 3.3V pin can take.

#define TRIG 2
#define ECHO 34

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== ultrasonic test ===");
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  digitalWrite(TRIG, LOW);
}

float readDistance() {
  digitalWrite(TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  // 30ms cap: about 5m of flight time. Without a timeout a missing echo would
  // block the loop for a second.
  long dur = pulseIn(ECHO, HIGH, 30000);
  if (dur == 0) return -1;
  return dur * 0.0343 / 2.0;
}

void loop() {
  float d = readDistance();
  if (d < 0) Serial.println("no echo");
  else       Serial.printf("dist: %.1f cm\n", d);
  delay(300);
}
