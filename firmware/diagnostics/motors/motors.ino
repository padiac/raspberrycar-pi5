// Four-wheel test: spins each motor on its own so wiring and direction can be
// checked one at a time.
//
// PUT THE CAR ON A STAND FIRST. The wheels must be off the ground -- this runs
// each motor at 70% for two seconds with no way to stop it from the outside.
//
// Two TB6612FNGs, one per axle. Each drives two motors (A and B channels), so
// A/B are the front pair and C/D the rear.
//
// ledcAttach(pin, freq, bits) is Arduino-ESP32 3.x. On a 2.x core use:
//     ledcSetup(channel, 20000, 8); ledcAttachPin(pin, channel);
// and pass the channel, not the pin, to ledcWrite.

#define PWMA 32   // left front
#define AIN2 33
#define AIN1 25
#define BIN1 26   // right front
#define BIN2 27
#define PWMB 14

#define PWMC 23   // left rear
#define CIN2 22
#define CIN1 21
#define DIN1 19   // right rear
#define DIN2 18
#define PWMD 5

// Flip an entry to -1 if that wheel turns the wrong way. Doing it here means
// the mecanum mixing below never has to know about miswired motors.
const int DIR[4] = {1, 1, 1, 1};   // left front, right front, left rear, right rear

// 20kHz: above hearing, so the motors do not whine. 8-bit gives 0-255.
const int PWM_HZ = 20000;
const int PWM_BITS = 8;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== four motor test -- wheels must be off the ground ===");

  int outs[] = {AIN1, AIN2, BIN1, BIN2, CIN1, CIN2, DIN1, DIN2};
  for (int p : outs) pinMode(p, OUTPUT);

  ledcAttach(PWMA, PWM_HZ, PWM_BITS);
  ledcAttach(PWMB, PWM_HZ, PWM_BITS);
  ledcAttach(PWMC, PWM_HZ, PWM_BITS);
  ledcAttach(PWMD, PWM_HZ, PWM_BITS);
}

void drive(int pwmPin, int in1, int in2, int spd) {
  spd = constrain(spd, -255, 255);
  digitalWrite(in1, spd >= 0);
  digitalWrite(in2, spd <  0);
  ledcWrite(pwmPin, abs(spd));
}

void setWheels(int lf, int rf, int lb, int rb) {
  drive(PWMA, AIN1, AIN2, lf * DIR[0]);
  drive(PWMB, BIN1, BIN2, rf * DIR[1]);
  drive(PWMC, CIN1, CIN2, lb * DIR[2]);
  drive(PWMD, DIN1, DIN2, rb * DIR[3]);
}

// Mecanum inverse kinematics: vx forward, vy sideways, w rotation.
// Not used by the test loop, but this is the shape the real driving code takes.
void mecanum(int vx, int vy, int w) {
  setWheels(vx - vy - w, vx + vy + w, vx + vy - w, vx - vy + w);
}

void spin(const char *name, int lf, int rf, int lb, int rb) {
  Serial.println(name);
  setWheels(lf, rf, lb, rb);
  delay(2000);
  setWheels(0, 0, 0, 0);
  delay(1000);
}

void loop() {
  spin("left front",  180, 0, 0, 0);
  spin("right front", 0, 180, 0, 0);
  spin("left rear",   0, 0, 180, 0);
  spin("right rear",  0, 0, 0, 180);
  Serial.println("--- cycle done ---");
  delay(1000);
}
