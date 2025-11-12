#include <Arduino.h>
#include "hardware/pwm.h"
#include "pico/time.h"
#include "oled.h"
#include "encoder.h"
#include "motor.h"
#include "sensors.h"
#include "lab.h"

void splash_screen() {
  oled_clear();
  oled_draw_logo();
  oled_text(0, 7, "AMR Lab (C) h_da 2025");
  oled_update();
  delay(2000);   // 2 s warten
}

void show_main_screen() {
  oled_clear();
  char line[32];

  // === Title bar ===
  snprintf(line, sizeof(line), "AMR Lab %s.%s - %s", LAB_ID, TASK_ID, STUDENT_ID);
  oled_text(0, 0, line);
  oled_hline(0, 10, 128);

  // === Encoder tick counts ===
  snprintf(line, sizeof(line), "L: %6ld  R: %6ld",
           encoder_left_ticks(), encoder_right_ticks());
  oled_text(0, 2, line);

  // === Range Sensor data ===
  snprintf(line, sizeof(line), "L: %u F: %u R: %u",
           tof_get_left(), tof_get_front(), tof_get_right());
  oled_text(0, 3, line);

  // === Current PID parameters ===
  float p, i, d;
  motor_get_pid_gains(&p, &i, &d);
  snprintf(line, sizeof(line), "P:%.2f I:%.2f D:%.3f", p, i, d);
  oled_text(0, 6, line);
  oled_update();
}

bool oled_update_callback(repeating_timer_t *t) {
    show_main_screen();
    return true; // wiederholen
}

void setup() {
  static repeating_timer_t oled_timer;
  Serial.begin(115200);
  delay(100);
  oled_init();
  encoder_init();
  motor_init();
  buttons_init();  // Buttons
  tof_init();   // Range Sensors

  splash_screen(); // shows splash screen for 2 secs
  add_repeating_timer_ms(100, oled_update_callback, NULL, &oled_timer);
}

void loop() {
  lab_main(); // call main lab function
  sleep_ms(10);  // lightweight idle delay
}