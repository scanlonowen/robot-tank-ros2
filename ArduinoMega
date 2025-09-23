#include <AccelStepper.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

// ---- DRIVER PINS ----
#define EN_PIN1   7
#define DIR_PIN1  6
#define STEP_PIN1 5

#define EN_PIN2   10
#define DIR_PIN2  9
#define STEP_PIN2 8

// --- serial ingest state ---
static char   rx_buf[64];
static uint8_t rx_idx = 0;

static unsigned long last_cmd_ms = 0;
static bool   cmd_present = false;

// caps (tune to your robot)
const float V_MAX = 8.0f;   // inches/s
const float W_MAX = 3.0f;   // rad/s

// drivetrain params
float track_width = 9.0f;               // inches between wheel centers
float steps_per_rev = 200.0f;           // motor native steps/rev
float microsteps   = 8.0f;              // microstep setting on driver
float gear_ratio   = 54.0f / 16.0f;     // output/input
float steps_per_wheel_rev = steps_per_rev * microsteps * gear_ratio;

// wheel circumference (inches). measure yours if this is off.
float c_wheel = 5.498f;
float steps_per_inch = steps_per_wheel_rev / c_wheel;

// command state
float v = 0.0f;    // inches/s
float w = 0.0f;    // rad/s

// slew state
float prev_left_sps  = 0.0f;
float prev_right_sps = 0.0f;

AccelStepper stepper_left (AccelStepper::DRIVER, STEP_PIN1, DIR_PIN1);
AccelStepper stepper_right(AccelStepper::DRIVER, STEP_PIN2, DIR_PIN2);

// ---- fwd decls ----
void stepperSetup();
void driveMotors(float Val, float Ang);
bool parseVWLine(const char* s, float* outV, float* outW);

void setup() {
  stepperSetup();
  Serial.begin(115200);
}

void loop() {
  const unsigned long now = millis();

  // Non-blocking line reader. Expect "V:<num>;W:<num>\n"
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      rx_buf[min(rx_idx, (uint8_t)(sizeof(rx_buf)-1))] = '\0';
      float nv, nw;
      if (parseVWLine(rx_buf, &nv, &nw)) {
        // clamp to sane limits
        if (nv >  V_MAX) nv =  V_MAX; if (nv < -V_MAX) nv = -V_MAX;
        if (nw >  W_MAX) nw =  W_MAX; if (nw < -W_MAX) nw = -W_MAX;
        v = nv; w = nw;
        last_cmd_ms = now;
        cmd_present = true;
      }
      rx_idx = 0;
    } else if (rx_idx < sizeof(rx_buf)-1) {
      rx_buf[rx_idx++] = c;
    } else {
      // overflow: drop buffer and resync on next newline
      rx_idx = 0;
    }
  }

  // watchdog: if we stop hearing commands, stop
  float v_cmd = v;
  float w_cmd = w;
  if (!cmd_present || (now - last_cmd_ms) > 300) {
    v_cmd = 0.0f;
    w_cmd = 0.0f;
  }

  driveMotors(v_cmd, w_cmd);

  // Debug print @4 Hz
  static unsigned long lastPrint = 0;
  if (now - lastPrint >= 250) {
    float left_sps  = (v_cmd + w_cmd * (track_width * 0.5f)) * steps_per_inch;
    float right_sps = (v_cmd - w_cmd * (track_width * 0.5f)) * steps_per_inch;

    Serial.print("v=");
    Serial.print(v_cmd, 2);
    Serial.print(" in/s  w=");
    Serial.print(w_cmd, 2);
    Serial.print(" rad/s  | L=");
    Serial.print(left_sps, 0);
    Serial.print(" sps  R=");
    Serial.print(right_sps, 0);
    Serial.println(" sps");
    lastPrint = now;
  }
}

void driveMotors(float Val, float Ang) {
  float target_left  = (Val + Ang * (track_width * 0.5f)) * steps_per_inch;
  float target_right = (Val - Ang * (track_width * 0.5f)) * steps_per_inch;

  // Give turns some bite (optional)
  const float MIN_TURN_SPS = 120.0f;
  if (fabs(Ang) > 0.25f) {
    if (fabs(target_left)  > 0 && fabs(target_left)  < MIN_TURN_SPS)
      target_left  = (target_left  > 0 ?  MIN_TURN_SPS : -MIN_TURN_SPS);
    if (fabs(target_right) > 0 && fabs(target_right) < MIN_TURN_SPS)
      target_right = (target_right > 0 ?  MIN_TURN_SPS : -MIN_TURN_SPS);
  }

  // *** Hard cap that preserves the commanded v/w mix ***
  const float MAX_SPS = 8000.0f;            // pick what your rig can do
  float maxmag = max(fabs(target_left), fabs(target_right));
  if (maxmag > MAX_SPS) {
    float scale = MAX_SPS / maxmag;
    target_left  *= scale;
    target_right *= scale;
  }

  // Slew
  const float MAX_DELTA = 300.0f;
  float dL = target_left  - prev_left_sps;
  if (dL >  MAX_DELTA) dL =  MAX_DELTA;
  if (dL < -MAX_DELTA) dL = -MAX_DELTA;
  prev_left_sps += dL;

  float dR = target_right - prev_right_sps;
  if (dR >  MAX_DELTA) dR =  MAX_DELTA;
  if (dR < -MAX_DELTA) dR = -MAX_DELTA;
  prev_right_sps += dR;

  stepper_left.setSpeed(prev_left_sps);
  stepper_right.setSpeed(prev_right_sps);
  stepper_left.runSpeed();
  stepper_right.runSpeed();
}


void stepperSetup() {
  stepper_left.setEnablePin(EN_PIN1);
  stepper_right.setEnablePin(EN_PIN2);

  // Invert enable if your driver is active-low on EN
  stepper_left.setPinsInverted(true, false, true);
  stepper_right.setPinsInverted(false, false, true);

  stepper_left.enableOutputs();
  stepper_right.enableOutputs();

  // TB6560 likes >= 2-5us; 10us is safe
  stepper_left.setMinPulseWidth(10);
  stepper_right.setMinPulseWidth(10);

  // runSpeed() ignores acceleration; keep for future run() use
  stepper_left.setMaxSpeed(7000);
  stepper_right.setMaxSpeed(7000);
  stepper_left.setAcceleration(400);
  stepper_right.setAcceleration(400);

  stepper_left.setCurrentPosition(0);
  stepper_right.setCurrentPosition(0);
}

// Parse lines like "V:+3.25;W:-0.80"
bool parseVWLine(const char* s, float* outV, float* outW) {
  if (!s || !outV || !outW) return false;

  // tolerant scan
  const char* vtag = strstr(s, "V:");
  const char* wtag = strstr(s, "W:");
  if (!vtag || !wtag) return false;

  char* endp = nullptr;
  float v_parsed = strtod(vtag + 2, &endp);
  if (endp == vtag + 2) return false;

  float w_parsed = strtod(wtag + 2, &endp);
  if (endp == wtag + 2) return false;

  if (!isfinite(v_parsed) || !isfinite(w_parsed)) return false;

  *outV = v_parsed;
  *outW = w_parsed;
  return true;
}
