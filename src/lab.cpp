#include "lab.h"

// ======================================================
//  Lab Dispatcher
// ======================================================
// Called from main.cpp -> loop()
// Select the task you want to execute by editing lab_main()
void lab_main() {
    lab1_task1();   // <- Change this to lab1_task2(), etc.
}

// ======================================================
//  LAB 1 – TASK 1
//  Open-loop motor test
// ======================================================
// Goal: Drive a simple trajectory using PWM (no feedback).
// Use sleep_ms() for timing. Observe motion symmetry and timing accuracy.
// The OLED and encoder run automatically in the background.

void lab1_task1() {
    if (buttonA_pressed()) {
        motor_set_pwm(30, 30);   // forward 30% PWM
        sleep_ms(1000);

        motor_set_pwm(0, 0);     // stop for 1s
        sleep_ms(1000);

        motor_set_pwm(-30, -30); // reverse 30% PWM
        sleep_ms(1000);

        motor_set_pwm(0, 0);     // final stop
    }
}

// ======================================================
//  LAB 1 – TASK 2
//  Closed-loop PID speed control
// ======================================================
// Goal: Tune Kp, Ki, Kd for stable RPM control.
// Use Arduino Serial Plotter to visualize step responses.

void lab1_task2() {
    // PID parameters – to be tuned experimentally
    float Kp = 0.20f;
    float Ki = 0.00f;
    float Kd = 0.00f;

    const float target = 300.0f; // Target speed in RPM

    if (buttonA_pressed()) {
        motor_set_pid_gains(Kp, Ki, Kd);

        // Step response test: forward / stop / reverse
        motor_set_pid(0, 0);
        sleep_ms(50);

        motor_set_pid(target, target);
        sleep_ms(1000);

        motor_set_pid(0, 0);
        sleep_ms(500);

        motor_set_pid(-target, -target);
        sleep_ms(1000);

        motor_set_pid(0, 0);
        sleep_ms(500);

        motor_stop_pid();
    }
}

// ======================================================
//  LAB 1 – TASK 3
//  Trajectory Control and Calibration
// ======================================================
// Goal: Use calibrated PID speeds (mm/s) to move 1 m straight.
// Adjust for left/right asymmetry using encoder feedback.

void lab1_task3() {
    const float max_speed = 300.0f;  // target RPM
    const float step = 10.0f;        // acceleration step
    const int hold_time = 20;        // delay between updates [ms]

    if (buttonA_pressed()) {
        // Accelerate gradually
        for (float s = 0; s <= max_speed; s += step) {
            motor_set_pid(s, s);
            sleep_ms(hold_time);
        }

        sleep_ms(2000); // cruise

        // Decelerate
        for (float s = max_speed; s >= 0; s -= step) {
            motor_set_pid(s, s);
            sleep_ms(hold_time);
        }

        motor_stop_pid();
    }
}

// ======================================================
//  LAB 1 – TASK 4
//  Maze Trajectory
// ======================================================
// Goal: Follow the full Micromouse-style path:
// 4 cells straight → 90° right → 2 cells → 90° left → 2 cells → goal → 360° turn.

void lab1_task4() {
    // TODO: Implement full maze trajectory
    // Combine calibrated distance and turn sequences
}