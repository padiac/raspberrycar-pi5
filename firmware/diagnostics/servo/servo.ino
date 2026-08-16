// Servo sweep for the ultrasonic pan mount.
//
// 500-2400us rather than the library default: SG90 clones vary, and the wider
// window gets closer to a true 0-180 range. If the horn buzzes at an end stop
// the pulse is asking for further than it can go -- narrow this.

#include <ESP32Servo.h>

#define SERVO_PIN 4

Servo cam;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== servo sweep ===");
  cam.setPeriodHertz(50);              // analogue servos want 50Hz
  cam.attach(SERVO_PIN, 500, 2400);
}

void loop() {
  Serial.println("45");  cam.write(45);  delay(1000);
  Serial.println("90");  cam.write(90);  delay(1000);
  Serial.println("135"); cam.write(135); delay(1000);
}
