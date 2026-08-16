// One-shot self test: run this on battery, read the verdict, then take it apart.
//
// The point is to exercise the paths that have NOT been tested yet. Servo and
// ultrasonic working over USB proves very little about running on battery:
//
//   - that 5V came from USB, not from the buck module. On battery the buck
//     feeds them instead, and that path has never carried current.
//   - the motors, both TB6612s, and the 7.4V rail are completely untested.
//   - nothing so far has drawn more than a few tens of milliamps. Four motors
//     starting together pull amps, and that is when brownouts, weak grounds
//     and thin wiring show themselves.
//
// So: RUN THIS ON BATTERY, USB unplugged if you can (or at least battery on),
// WITH THE WHEELS OFF THE GROUND.
//
// The motor phase waits for you to type 'g' first, so nothing moves by
// surprise. The brownout check is the genuinely useful automatic one: the
// ESP32 remembers why it last reset, so if the motors crash the supply, the
// next boot says so outright.

#include <ESP32Servo.h>
#include <esp_system.h>

#define TRIG 2
#define ECHO 34
#define SERVO_PIN 4

#define PWMA 32
#define AIN2 33
#define AIN1 25
#define BIN1 26
#define BIN2 27
#define PWMB 14
#define PWMC 23
#define CIN2 22
#define CIN1 21
#define DIN1 19
#define DIN2 18
#define PWMD 5

const int PWM_HZ = 20000, PWM_BITS = 8;
Servo cam;

void drive(int pwmPin, int in1, int in2, int spd) {
  spd = constrain(spd, -255, 255);
  digitalWrite(in1, spd >= 0);
  digitalWrite(in2, spd < 0);
  ledcWrite(pwmPin, abs(spd));
}
void setWheels(int lf, int rf, int lb, int rb) {
  drive(PWMA, AIN1, AIN2, lf);
  drive(PWMB, BIN1, BIN2, rf);
  drive(PWMC, CIN1, CIN2, lb);
  drive(PWMD, DIN1, DIN2, rb);
}
void allStop() { setWheels(0, 0, 0, 0); }

float readDistance() {
  digitalWrite(TRIG, LOW);  delayMicroseconds(4);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long dur = pulseIn(ECHO, HIGH, 30000);
  return dur ? dur * 0.0343 / 2.0 : -1;
}

void banner(const char *s) {
  Serial.println();
  Serial.println("----------------------------------------");
  Serial.println(s);
  Serial.println("----------------------------------------");
}

void setup() {
  Serial.begin(115200);
  delay(900);
  Serial.println("\n\n========================================");
  Serial.println("  SELF TEST");
  Serial.println("========================================");

  // ---- why did we last reset? -------------------------------------------
  // This is the one check that needs no eyes. If a previous run browned out
  // under motor load, it shows up here.
  esp_reset_reason_t why = esp_reset_reason();
  Serial.print("last reset: ");
  switch (why) {
    case ESP_RST_POWERON:  Serial.println("power on (normal)"); break;
    case ESP_RST_SW:       Serial.println("software / upload (normal)"); break;
    case ESP_RST_BROWNOUT:
      Serial.println("*** BROWNOUT ***");
      Serial.println("  The supply collapsed last run. If that was during the motor");
      Serial.println("  phase, the battery or wiring cannot deliver the current --");
      Serial.println("  fix that before trusting anything else here.");
      break;
    case ESP_RST_PANIC:    Serial.println("crash (panic)"); break;
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:      Serial.println("watchdog"); break;
    default:               Serial.printf("code %d\n", (int)why); break;
  }

  // ---- ultrasonic --------------------------------------------------------
  banner("[1/3] ULTRASONIC  (fully automatic)");
  pinMode(TRIG, OUTPUT); pinMode(ECHO, INPUT); digitalWrite(TRIG, LOW);
  delay(150);
  int ok = 0;
  float sum = 0;
  for (int i = 0; i < 10; i++) {
    float d = readDistance();
    if (d > 1 && d < 400) { ok++; sum += d; Serial.printf("   %5.1f cm\n", d); }
    else                  { Serial.println("   no echo"); }
    delay(120);
  }
  if (ok >= 7)      Serial.printf("  PASS  %d/10 good, average %.1f cm\n", ok, sum / ok);
  else if (ok > 0)  Serial.printf("  FLAKY %d/10 good -- check wiring, or it is aimed at something soft\n", ok);
  else              Serial.println("  FAIL  no echoes at all");

  // ---- servo -------------------------------------------------------------
  banner("[2/3] SERVO  (watch the mount)");
  cam.setPeriodHertz(50);
  cam.attach(SERVO_PIN, 500, 2400);
  for (int a = 45; a <= 135; a += 45) {
    Serial.printf("   -> %d deg\n", a);
    cam.write(a);
    delay(700);
  }
  cam.write(90);
  Serial.println("  Did it move to three positions? If yes, the servo path is fine on this supply.");

  // ---- motors ------------------------------------------------------------
  banner("[3/3] MOTORS  --  WHEELS MUST BE OFF THE GROUND");
  int outs[] = {AIN1, AIN2, BIN1, BIN2, CIN1, CIN2, DIN1, DIN2};
  for (int p : outs) pinMode(p, OUTPUT);
  ledcAttach(PWMA, PWM_HZ, PWM_BITS);
  ledcAttach(PWMB, PWM_HZ, PWM_BITS);
  ledcAttach(PWMC, PWM_HZ, PWM_BITS);
  ledcAttach(PWMD, PWM_HZ, PWM_BITS);
  allStop();

  Serial.println("  Type  g  then Enter to start the motor test.");
  Serial.println("  (anything else skips it)");
  while (!Serial.available()) delay(50);
  if (Serial.read() != 'g') {
    Serial.println("  skipped.");
    Serial.println("\n=== done ===");
    return;
  }

  const char *names[] = {"left front", "right front", "left rear", "right rear"};
  for (int i = 0; i < 4; i++) {
    Serial.printf("\n   %s -- forward\n", names[i]);
    setWheels(i == 0 ? 170 : 0, i == 1 ? 170 : 0, i == 2 ? 170 : 0, i == 3 ? 170 : 0);
    delay(1500);
    allStop(); delay(400);
    Serial.printf("   %s -- reverse\n", names[i]);
    setWheels(i == 0 ? -170 : 0, i == 1 ? -170 : 0, i == 2 ? -170 : 0, i == 3 ? -170 : 0);
    delay(1500);
    allStop(); delay(700);
  }

  // The real test: everything at once, which is the current peak. If the
  // supply is going to fold, it folds here -- and the next boot will say
  // BROWNOUT above.
  Serial.println("\n   ALL FOUR together, ramping to full -- this is the current peak");
  for (int s = 60; s <= 255; s += 15) {
    setWheels(s, s, s, s);
    Serial.printf("     pwm %d\n", s);
    delay(220);
  }
  delay(700);
  allStop();

  Serial.println("\n========================================");
  Serial.println("  If you are reading this, the board survived full load.");
  Serial.println("  Note which wheels turned the wrong way -- flip those in DIR[].");
  Serial.println("  Press EN/reset once: 'last reset' at the top should say");
  Serial.println("  power on or software, NOT brownout.");
  Serial.println("========================================");
}

void loop() {}
