// Everything at once: servo, ultrasonic, four wheels. One sketch, runs forever.
//
// WHEELS OFF THE GROUND. The motor phase starts on its own, ten seconds in.
//
// Nothing here needs the serial monitor -- flash it, unplug USB, run it on the
// battery. It prints if something is listening, and the servo doubles as a
// status indicator so you can read the phase off the hardware:
//
//     sweeping 30-150   servo phase        (proves the 5V rail carries a load)
//     parked at 90      ultrasonic phase
//     steps 45/90/135   motor phase, one step per wheel
//
// Whatever is not powered simply does nothing -- if the 7.4V is not connected
// the wheels stay still and the rest still runs.

#include <ESP32Servo.h>

// --- pins -------------------------------------------------------------
#define TRIG 2
#define ECHO 34
#define SERVO_PIN 4

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

// Flip to -1 for any wheel that turns the wrong way.
const int DIR[4] = {1, 1, 1, 1};   // LF, RF, LB, RB

const int PWM_HZ = 20000;
const int PWM_BITS = 8;
const int TEST_SPEED = 170;        // out of 255

// ESP32Servo and the motor PWM both drive the LEDC peripheral, and channels
// there share timers. Left to allocate themselves the servo can land on a
// timer the motors have already set to 20kHz, and then it silently receives
// 20kHz instead of 50Hz and never moves.
//
// On the classic ESP32, channels 0-7 and 8-15 sit in different timer groups.
// Pinning the motors to 8-11 keeps them physically clear of whatever the
// servo library takes, and the servo is attached first so it gets first pick.
const int MOTOR_CH[4] = {8, 9, 10, 11};

Servo cam;

// --- motors -----------------------------------------------------------
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

void allStop() { setWheels(0, 0, 0, 0); }

// Mecanum mixing: vx forward, vy sideways, w rotation. Not used by the test
// below, but this is the shape the real driving code takes.
void mecanum(int vx, int vy, int w) {
  setWheels(vx - vy - w, vx + vy + w, vx + vy - w, vx - vy + w);
}

// --- ultrasonic -------------------------------------------------------
float readDistance() {
  digitalWrite(TRIG, LOW);  delayMicroseconds(4);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long dur = pulseIn(ECHO, HIGH, 30000);
  return dur ? dur * 0.0343 / 2.0 : -1;
}

// --- setup ------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  digitalWrite(TRIG, LOW);

  // Servo first, so it claims its LEDC timer before the motors take any.
  ESP32PWM::allocateTimer(0);
  cam.setPeriodHertz(50);
  cam.attach(SERVO_PIN, 500, 2400);
  cam.write(90);
  delay(400);

  int outs[] = {AIN1, AIN2, BIN1, BIN2, CIN1, CIN2, DIN1, DIN2};
  for (int p : outs) pinMode(p, OUTPUT);
  ledcAttachChannel(PWMA, PWM_HZ, PWM_BITS, MOTOR_CH[0]);
  ledcAttachChannel(PWMB, PWM_HZ, PWM_BITS, MOTOR_CH[1]);
  ledcAttachChannel(PWMC, PWM_HZ, PWM_BITS, MOTOR_CH[2]);
  ledcAttachChannel(PWMD, PWM_HZ, PWM_BITS, MOTOR_CH[3]);
  allStop();

  delay(400);
  Serial.println("\n\n=== full test: servo, ultrasonic, wheels ===");
  Serial.println("wheels must be off the ground");

  // Prove the servo still works now the motor PWM is configured. If it moves
  // here but not later, the two are still fighting over LEDC.
  Serial.println("\n[check] servo, after motor PWM is set up");
  for (int a = 60; a <= 120; a += 10) { cam.write(a); delay(200); }
  for (int a = 120; a >= 60; a -= 10) { cam.write(a); delay(200); }
  cam.write(90);
  Serial.println("[check] if that did not move, LEDC is still clashing\n");
  delay(600);
}

// --- phases -----------------------------------------------------------
void phaseServo() {
  Serial.println("[servo] sweeping 30-150");
  for (int a = 30; a <= 150; a += 3) { cam.write(a); delay(12); }
  for (int a = 150; a >= 30; a -= 3) { cam.write(a); delay(12); }
  cam.write(90);
  delay(300);
}

void phaseUltrasonic() {
  Serial.println("[ultrasonic]");
  int ok = 0;
  for (int i = 0; i < 8; i++) {
    float d = readDistance();
    if (d > 1 && d < 400) { ok++; Serial.printf("   %.1f cm\n", d); }
    else                  { Serial.println("   no echo"); }
    delay(200);
  }
  Serial.printf("   %d/8 good\n", ok);
}

void oneWheel(const char *name, int idx, int servoAngle) {
  cam.write(servoAngle);                       // visual marker for this wheel
  int s[4] = {0, 0, 0, 0};

  Serial.printf("[motor] %s forward\n", name);
  s[idx] = TEST_SPEED;
  setWheels(s[0], s[1], s[2], s[3]);
  delay(1200);
  allStop(); delay(400);

  Serial.printf("[motor] %s reverse\n", name);
  s[idx] = -TEST_SPEED;
  setWheels(s[0], s[1], s[2], s[3]);
  delay(1200);
  allStop(); delay(600);
}

void phaseMotors() {
  oneWheel("left front",  0, 45);
  oneWheel("right front", 1, 75);
  oneWheel("left rear",   2, 105);
  oneWheel("right rear",  3, 135);

  // All four together: the current peak. If the supply is going to sag, here
  // is where it does it.
  Serial.println("[motor] all four, ramping up");
  cam.write(90);
  for (int s = 70; s <= 255; s += 15) {
    setWheels(s, s, s, s);
    delay(200);
  }
  delay(600);
  allStop();
  Serial.println("[motor] done\n");
}

void loop() {
  phaseServo();
  phaseUltrasonic();
  phaseMotors();
  Serial.println("--- cycle complete, repeating in 3s ---\n");
  delay(3000);
}
