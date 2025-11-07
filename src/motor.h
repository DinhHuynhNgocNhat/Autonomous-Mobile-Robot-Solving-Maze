#pragma once
#include <Arduino.h>
#include "pins.h"

// PWM
#define PWM_FREQ   20000   // 20 kHz
#define PWM_MAX    80.0f   // End of linear curve

extern volatile float rpmL_meas, rpmR_meas;
extern volatile float rpm_target_L, rpm_target_R;

void motor_init();
void motor_set_pwm(float left, float right);
void motor_set_pid(float left_rpm, float right_rpm);
void motor_stop_pid();
void motor_set_pid_gains(float p, float i, float d);
void motor_get_pid_gains(float *p, float *i, float *d);