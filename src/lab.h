#pragma once
#include "motor.h"
#include "sensors.h"
#include "oled.h"
#include "encoder.h"
#include "pico/stdlib.h"

// ======================================================
//  Student Information
// ======================================================
#define STUDENT_ID  "1134320"   // <-- Replace with your student ID
#define LAB_ID      "3"         // <-- Set to the current lab number
#define TASK_ID     ""         // <-- Set to the current task number

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
void lab2_task1(); // Perception 1 (front sensor + stop)
void lab2_task2(); // Perception 2 (front sensor + trajectory)
void lab2_task3(); // Perception 3 (side sensor + wall following straight)
void lab2_task4(); // Perception 4 (side sensor + wall following maze)

// === Lab 3 ===
void lab3(); // Perception (maze solving)