#pragma once
#include "motor.h"
#include "sensors.h"
#include "oled.h"
#include "encoder.h"
#include "pico/stdlib.h"

// ======================================================
//  Student Information
// ======================================================
#define STUDENT_ID  "1234567"   // <-- Replace with your student ID
#define LAB_ID      "1"         // <-- Set to the current lab number
#define TASK_ID     "1"         // <-- Set to the current task number

// ======================================================
//  Function Declarations
// ======================================================

// Main entry point for the current lab
void lab_main();   // Called from loop() in main.cpp

// === Lab 1 ===
void lab1_task1(); // Open-loop motion control (no feedback)
void lab1_task2(); // PID tuning using closed-loop control
void lab1_task3(); // Trajectory with PID control and calibration
void lab1_task4(); // Extended maze trajectory (optional)

// === Lab 2 ===
// Add new tasks here when available