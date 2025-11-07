#include "motor.h"
#include "hardware/pwm.h"
#include "encoder.h"
#include "pico/stdlib.h"
#include "hardware/timer.h"

static float targetLeft = 0, targetRight = 0;
static bool pid_active = false;

volatile float rpmL_meas = 0, rpmR_meas = 0;
volatile float rpm_target_L = 0, rpm_target_R = 0;

// PID state
static float Kp = 0.2f, Ki = 0.00f, Kd = 0.0f;
static float integralL = 0, integralR = 0;
static float lastErrL = 0, lastErrR = 0;
static int32_t lastTicksL = 0, lastTicksR = 0;

// Timer handle
static repeating_timer_t pid_timer;

// --- Internal: periodic PID update ---
bool pid_callback(repeating_timer_t *t) {
    if (!pid_active) return true; // keep timer alive but skip
    
    const float dt = 0.01f; // 10 ms
    const float ticks_per_rev = 360.0f;

    int32_t ticksL = encoder_left_ticks();
    int32_t ticksR = encoder_right_ticks();
    float dL = (ticksL - lastTicksL) / dt;
    float dR = (ticksR - lastTicksR) / dt;
    lastTicksL = ticksL;
    lastTicksR = ticksR;

    float rpmL = (dL / ticks_per_rev) * 60.0f;
    float rpmR = (dR / ticks_per_rev) * 60.0f;

    rpmL_meas = rpmL;
    rpmR_meas = rpmR;
    rpm_target_L = targetLeft;
    rpm_target_R = targetRight;

    // PID calc
    float errL = targetLeft - rpmL;
    float errR = targetRight - rpmR;

    integralL += errL * dt;
    integralR += errR * dt;

    float dErrL = (errL - lastErrL) / dt;
    float dErrR = (errR - lastErrR) / dt;

    lastErrL = errL;
    lastErrR = errR;

    float outL = Kp * errL + Ki * integralL + Kd * dErrL;
    float outR = Kp * errR + Ki * integralR + Kd * dErrR;

    // Limit output
    if (outL > 100) outL = 100;
    if (outL < -100) outL = -100;
    if (outR > 100) outR = 100;
    if (outR < -100) outR = -100;

    motor_set_pwm(outL, outR);
    Serial.printf("%.1f,%.1f,%.1f,%.1f\n",
       rpm_target_L, rpmL_meas,
       rpm_target_R, rpmR_meas);
    return true; // repeat
}

void motor_init() {
  gpio_init(LEFT_DIR);  gpio_set_dir(LEFT_DIR, GPIO_OUT);
  gpio_init(RIGHT_DIR); gpio_set_dir(RIGHT_DIR, GPIO_OUT);

  // PWM initialisieren (1 kHz)
  gpio_set_function(LEFT_PWM, GPIO_FUNC_PWM);
  gpio_set_function(RIGHT_PWM, GPIO_FUNC_PWM);

  uint sliceL = pwm_gpio_to_slice_num(LEFT_PWM);
  uint sliceR = pwm_gpio_to_slice_num(RIGHT_PWM);

  pwm_config cfg = pwm_get_default_config();
  pwm_config_set_clkdiv(&cfg, 1.0f);
  uint32_t wrap = (uint32_t)(125000000.0f / PWM_FREQ) - 1;  // = 6249 bei 20 kHz
  pwm_config_set_wrap(&cfg, wrap);

  pwm_init(sliceL, &cfg, true);
  pwm_init(sliceR, &cfg, true);

  // PID timer
  add_repeating_timer_ms(-10, pid_callback, NULL, &pid_timer);  // 10 ms
}

void motor_set_pwm(float left_percent, float right_percent) {
  if (left_percent  >  PWM_MAX) left_percent  =  PWM_MAX;
  if (left_percent  < -PWM_MAX) left_percent  = -PWM_MAX;
  if (right_percent >  PWM_MAX) right_percent =  PWM_MAX;
  if (right_percent < -PWM_MAX) right_percent = -PWM_MAX;

  // Richtung
  gpio_put(LEFT_DIR,  (left_percent  < 0));
  gpio_put(RIGHT_DIR, (right_percent < 0));

  // PWM-Level (0–PWM_MAX → 0–100 %)
  float left_duty  = fabs(left_percent);
  float right_duty = fabs(right_percent);

  pwm_set_gpio_level(LEFT_PWM,  left_duty  * 62.49);
  pwm_set_gpio_level(RIGHT_PWM, right_duty * 62.49);
}

void motor_set_pid(float left_rpm, float right_rpm) {
    targetLeft = left_rpm;
    targetRight = right_rpm;
    pid_active = true;
}

void motor_stop_pid() {
    pid_active = false;
    motor_set_pwm(0, 0);
}

// --- PID gain setters/getters ---
void motor_set_pid_gains(float p, float i, float d) {
    Kp = p;
    Ki = i;
    Kd = d;
}

void motor_get_pid_gains(float *p, float *i, float *d) {
    if (p) *p = Kp;
    if (i) *i = Ki;
    if (d) *d = Kd;
}