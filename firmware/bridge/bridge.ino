// ESP32 side of the Pi <-> ESP32 link.
//
// This firmware holds no opinions about where the car should go. It receives a
// velocity, works out what each wheel must do, drives the motors, reads the
// sensors and reports back. Everything about *intent* -- following, patrolling,
// behaviour trees -- lives in Python on the Pi.
//
// The split is by what each side is good at. Linux cannot hold microsecond
// timing (a scheduling hiccup ruins an ultrasonic echo measurement), and an
// ESP32 cannot run a neural network. So: reflexes here, thinking there.
//
// Flash this once. After that you change behaviour by editing Python on the
// Pi and restarting a service, which takes two seconds instead of a ten
// second reflash with a USB cable attached.
//
// ---- protocol -------------------------------------------------------------
// Text lines, NMEA-style, with an XOR checksum over everything between $ and *.
// Text rather than a packed struct on purpose: at 20Hz these messages are a
// few hundred bytes a second, so efficiency is irrelevant, while being able to
// open a serial monitor and *read* the conversation is worth a great deal when
// something misbehaves.
//
//   Pi  -> ESP32   $V,<vx>,<vy>,<omega>*CS   drive, each -1.00 .. 1.00
//                  $S,<angle>*CS             servo, 0 .. 180
//                  $H*CS                     heartbeat, keeps the watchdog fed
//                  $E*CS                     stop now
//                  $W,<fl>,<fr>,<rl>,<rr>*CS raw wheels, calibration only
//
//   ESP32 -> Pi    $T,<range_cm>,<fl>,<fr>,<rl>,<rr>,<stopped>*CS
//                  $!,<message>*CS           something went wrong
//
// range_cm is -1 when the sensor gave no echo.

#include <ESP32Servo.h>

// ---- pins -----------------------------------------------------------------
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

// Flip an entry to -1 for a wheel that turns the wrong way. Keeping this here
// means the Pi never has to know about miswired motors.
// Measured with calibrate_wheels.py: the rear right motor's leads are
// swapped, so it is corrected here rather than by rewiring it.
const int DIR[4] = {1, 1, 1, -1};   // LF, RF, LB, RB

const int PWM_HZ = 20000;
const int PWM_BITS = 8;

// ESP32Servo and the motor PWM share the LEDC peripheral, and channels there
// share timers. Left to allocate themselves the servo can land on a timer the
// motors have set to 20kHz, and then it silently gets 20kHz instead of 50Hz
// and never moves. Channels 0-7 and 8-15 are in different timer groups, so
// pinning the motors to 8-11 keeps them clear of the servo, which is attached
// first so it gets first pick.
const int MOTOR_CH[4] = {8, 9, 10, 11};

// If the Pi goes quiet for this long, stop. A crashed process or an unplugged
// cable must not leave the car driving away at its last commanded speed.
const unsigned long COMMAND_TIMEOUT_MS = 500;

const unsigned long TELEMETRY_PERIOD_MS = 50;    // 20Hz
const unsigned long RANGE_PERIOD_MS = 60;        // HC-SR04 needs ~60ms between pings

Servo cam;

float gVx = 0, gVy = 0, gW = 0;
float gWheel[4] = {0, 0, 0, 0};
bool gStopped = true;
//: Last velocity command. Only this feeds the motion watchdog.
unsigned long gLastCommand = 0;
//: Last heartbeat, kept only so link liveness can be reported separately.
unsigned long gLastHeartbeat = 0;
float gRangeCm = -1;

char gLine[96];
uint8_t gLen = 0;

// ---- motors ---------------------------------------------------------------
void driveMotor(int pwmPin, int in1, int in2, float speed) {
  speed = constrain(speed, -1.0f, 1.0f);
  digitalWrite(in1, speed >= 0);
  digitalWrite(in2, speed < 0);
  ledcWrite(pwmPin, (int)(fabs(speed) * 255));
}

// Mecanum inverse kinematics. The rollers sit at 45 degrees so each wheel
// pushes diagonally; summing four diagonal pushes gives motion in any
// direction without the chassis turning to face it.
void applyVelocity(float vx, float vy, float w) {
  float wheel[4] = {
    vx - vy - w,   // front left
    vx + vy + w,   // front right
    vx + vy - w,   // rear left
    vx - vy + w    // rear right
  };

  // Scale down together if any wheel exceeds full power. Clipping each one on
  // its own would quietly change the direction of travel.
  float peak = 1.0f;
  for (int i = 0; i < 4; i++) peak = max(peak, (float)fabs(wheel[i]));
  for (int i = 0; i < 4; i++) wheel[i] /= peak;

  driveMotor(PWMA, AIN1, AIN2, wheel[0] * DIR[0]);
  driveMotor(PWMB, BIN1, BIN2, wheel[1] * DIR[1]);
  driveMotor(PWMC, CIN1, CIN2, wheel[2] * DIR[2]);
  driveMotor(PWMD, DIN1, DIN2, wheel[3] * DIR[3]);

  for (int i = 0; i < 4; i++) gWheel[i] = wheel[i];
  gStopped = (vx == 0 && vy == 0 && w == 0);
}

void stopNow() {
  gVx = gVy = gW = 0;
  applyVelocity(0, 0, 0);
}

// ---- ultrasonic -----------------------------------------------------------
float readRange() {
  digitalWrite(TRIG, LOW);  delayMicroseconds(4);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long dur = pulseIn(ECHO, HIGH, 30000);
  return dur ? dur * 0.0343f / 2.0f : -1.0f;
}

// ---- protocol -------------------------------------------------------------
uint8_t checksum(const char *body) {
  uint8_t cs = 0;
  for (const char *p = body; *p; p++) cs ^= (uint8_t)*p;
  return cs;
}

void sendLine(const char *body) {
  Serial.printf("$%s*%02X\n", body, checksum(body));
}

void sendTelemetry() {
  char body[96];
  snprintf(body, sizeof(body), "T,%.1f,%.3f,%.3f,%.3f,%.3f,%d",
           gRangeCm, gWheel[0], gWheel[1], gWheel[2], gWheel[3], gStopped ? 1 : 0);
  sendLine(body);
}

// Returns true if the line was well formed. A garbled byte on the wire must
// not turn into a wild command, so anything failing the checksum is dropped.
// A return of false means "complain about this". Pure noise -- which arrives
// on the RX line when opening the port resets the board -- is silently
// dropped, so a complaint always means something real got mangled.
bool parseLine(char *line) {
  if (line[0] != '$') return true;
  char *star = strrchr(line, '*');
  if (!star) return false;
  *star = '\0';
  uint8_t want = (uint8_t)strtol(star + 1, nullptr, 16);
  if (checksum(line + 1) != want) return false;

  char *body = line + 1;
  switch (body[0]) {
    case 'V': {
      float vx = 0, vy = 0, w = 0;
      if (sscanf(body, "V,%f,%f,%f", &vx, &vy, &w) == 3) {
        gVx = constrain(vx, -1.0f, 1.0f);
        gVy = constrain(vy, -1.0f, 1.0f);
        gW  = constrain(w,  -1.0f, 1.0f);
        gLastCommand = millis();
      }
      return true;
    }
    case 'S': {
      int angle = 90;
      if (sscanf(body, "S,%d", &angle) == 1) cam.write(constrain(angle, 0, 180));
      return true;
    }
    case 'H':
      // Deliberately does NOT touch gLastCommand. A heartbeat only says the
      // link is up; it must not keep the wheels turning. On the Pi the
      // heartbeat runs on its own thread, so if the control loop hung while
      // that thread lived on, feeding the watchdog here would let the car
      // drive away under a robot that had stopped thinking. Only a velocity
      // command counts as "still being driven".
      gLastHeartbeat = millis();
      return true;
    case 'E':
      stopNow();
      gLastCommand = millis();
      return true;

    case 'W': {
      // Raw per-wheel drive, bypassing the mixer. Only for calibration: the
      // mixer can isolate a diagonal pair at best, so working out which way an
      // individual motor turns needs a way to address it directly. DIR[] is
      // still applied, so a wheel already marked reversed stays reversed.
      float w0 = 0, w1 = 0, w2 = 0, w3 = 0;
      if (sscanf(body, "W,%f,%f,%f,%f", &w0, &w1, &w2, &w3) == 4) {
        driveMotor(PWMA, AIN1, AIN2, constrain(w0, -1.0f, 1.0f) * DIR[0]);
        driveMotor(PWMB, BIN1, BIN2, constrain(w1, -1.0f, 1.0f) * DIR[1]);
        driveMotor(PWMC, CIN1, CIN2, constrain(w2, -1.0f, 1.0f) * DIR[2]);
        driveMotor(PWMD, DIN1, DIN2, constrain(w3, -1.0f, 1.0f) * DIR[3]);
        gWheel[0] = w0; gWheel[1] = w1; gWheel[2] = w2; gWheel[3] = w3;
        gStopped = !(w0 || w1 || w2 || w3);
        gVx = gVy = gW = 0;      // the mixer must not fight this next tick
        gLastCommand = millis();
      }
      return true;
    }
  }
  return false;
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (gLen) {
        gLine[gLen] = '\0';
        if (!parseLine(gLine)) sendLine("!,bad line");
        gLen = 0;
      }
    } else if (gLen < sizeof(gLine) - 1) {
      gLine[gLen++] = c;
    } else {
      gLen = 0;                       // overlong, throw it away
      sendLine("!,line too long");
    }
  }
}

// ---- setup / loop ---------------------------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  digitalWrite(TRIG, LOW);

  ESP32PWM::allocateTimer(0);
  cam.setPeriodHertz(50);
  cam.attach(SERVO_PIN, 500, 2400);
  cam.write(90);

  int outs[] = {AIN1, AIN2, BIN1, BIN2, CIN1, CIN2, DIN1, DIN2};
  for (int p : outs) pinMode(p, OUTPUT);
  ledcAttachChannel(PWMA, PWM_HZ, PWM_BITS, MOTOR_CH[0]);
  ledcAttachChannel(PWMB, PWM_HZ, PWM_BITS, MOTOR_CH[1]);
  ledcAttachChannel(PWMC, PWM_HZ, PWM_BITS, MOTOR_CH[2]);
  ledcAttachChannel(PWMD, PWM_HZ, PWM_BITS, MOTOR_CH[3]);
  stopNow();

  delay(300);
  sendLine("!,bridge ready");
}

void loop() {
  readSerial();

  // Watchdog: silence means stop.
  if (!gStopped && millis() - gLastCommand > COMMAND_TIMEOUT_MS) {
    stopNow();
    sendLine("!,command timeout, stopped");
  } else if (gVx || gVy || gW) {
    // Only re-apply the mixer when a velocity is what is being commanded. A
    // raw $W sets the motors directly, and mixing zeroes over the top of it
    // would cancel it out on the very next tick.
    applyVelocity(gVx, gVy, gW);
  }

  static unsigned long lastRange = 0;
  if (millis() - lastRange >= RANGE_PERIOD_MS) {
    lastRange = millis();
    gRangeCm = readRange();
  }

  static unsigned long lastTel = 0;
  if (millis() - lastTel >= TELEMETRY_PERIOD_MS) {
    lastTel = millis();
    sendTelemetry();
  }
}
