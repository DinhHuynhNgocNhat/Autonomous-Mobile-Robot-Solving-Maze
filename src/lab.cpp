#include "lab.h"
#include <algorithm>
#include <iostream>
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
//  LAB 2 – TASK 1
//  Perception 1 
// ======================================================
// Goal: Drive straight until front sensor shows < 50 mm then stop

#define Size 3
uint16_t frontBuffer[Size] = {0}, leftBuffer[Size] = {0}, rightBuffer[Size] = {0};
int idxFront = 0, idxLeft = 0, idxRight = 0;
uint32_t sumFront = 0, sumLeft = 0, sumRight = 0;
bool filledFront = false, filledLeft = false, filledRight = false;

const float speed = 50;           // constant speed
const uint16_t threshold = 100;     // determined from plot


uint16_t moving_average_filter(uint16_t newValue, uint16_t *buffer, uint32_t &sum, int &idx, bool &filled) {
    if (newValue == 0 && sum != 0) {                 
        return sum / (filled ? Size : (idx == 0 ? 1 : idx));
    }

    sum -= buffer[idx];
    buffer[idx] = newValue;
    sum += newValue;

    idx = (idx + 1) % Size;
    if (!filled && idx == 0) filled = true;

    int n = filled ? Size : (idx == 0 ? 1 : idx);
    return sum / n;
}

void read_sensors(uint16_t &front, uint16_t &left, uint16_t &right) {
    tof_update();
    front = moving_average_filter(tof_get_front(), frontBuffer, sumFront, idxFront, filledFront);
    left  = moving_average_filter(tof_get_left(),  leftBuffer,  sumLeft,  idxLeft,  filledLeft);
    right = moving_average_filter(tof_get_right(), rightBuffer, sumRight, idxRight, filledRight);
}

void drive_forward(){
    while (true) {
        uint16_t front,left,right;
        read_sensors(front,left,right);
        int error = left - right;
        float correction = 0.2 * error;
        motor_set_pid(speed - correction, speed + correction);

        if (front<=threshold && front!=0) {
            motor_stop_pid();
            break;
        }
        sleep_ms(200);
    }
}

void lab2_task1() {
    // tof_update();
    // uint16_t raw = tof_get_front();
    // uint16_t filt = median_filter(raw);
    // //Serial.printf("L: %u mm  F: %u mm  R: %u\n", tof_get_left(), tof_get_front(), tof_get_right());
    // Serial.printf("Raw: %u Filter: %u\n", raw, filt);
    // sleep_ms(200);
    if (buttonA_pressed()) {
        sleep_ms(200);
        //drive_forward();
    }
}

// ======================================================
//  LAB 2 – TASK 2
//  Perception 2
// ======================================================
// Goal: Turn 180° and Return to the Opposite Wall 

const int encoderTick = 540;    // read encoder when rotate 180

void lab2_task2() {
    if (buttonA_pressed()) {
        //drive_forward();
        sleep_ms(2000);
        //turn(180);
        //reset_filter(frontBuffer, sumFront, idxFront, filledFront);
        sleep_ms(2000);
        //drive_forward();
    }
}

// ======================================================
//  LAB 2 – TASK 3
//  Perception 3
// ======================================================
// Goal: Wall Following Using a Side Sensor
void lab2_task3(){
    // tof_update();
    // uint16_t front,left,right;
    // read_sensors(front,left,right);
    float Kp = 0.2f;
    uint16_t setpoint = 50; // read sensor when next to wall

    if (buttonA_pressed()) {
        //motor_set_pid_gains(Kp, 0,0);
        while (true) {
            tof_update();
            uint16_t frontSensor, leftSensor, rightSensor;
            read_sensors(frontSensor, leftSensor, rightSensor);

            if (frontSensor < 30 && frontSensor!=0) {
                motor_stop_pid();
                break;
            }

            int error = leftSensor - setpoint;
            float correction = Kp * error;

            if (correction > 50) correction = 50;
            if (correction < -50) correction = -50;

            float leftSpeed  = speed - correction;
            float rightSpeed = speed + correction;
            motor_set_pid(leftSpeed*0.8, rightSpeed*0.8);
            Serial.printf("error:%d,leftSpeed:%.1f,rightSpeed:%.1f\n",error,leftSpeed,rightSpeed);

            sleep_ms(10);
        }
    }
}

// ======================================================
//  LAB 3
//  Perception 
// ======================================================
// Goal: 
// const int cell_ticks = 100;
// void step_forward_cell() {          // đi 1 ô sau khi rẽ
//     encoder_reset();
//     motor_set_pid(speed*scaleL, speed*scaleR);
//     while (true) {
//         int avg = (abs(encoder_left_ticks()) + abs(encoder_right_ticks())) / 2;
//         if (avg >= cell_ticks) break;   // CELL_TICKS: số tick cho 1 ô
//     }
//     motor_stop_pid();
//     sleep_ms(100);
// }
const uint16_t open_wall = 200;
const uint16_t tol = 40;         // tolerance encoder
float scaleL = 1.0, scaleR = 1.0066;
float TRACK_W = 96.0f;
float WHEEL_D = 32.0f;
float CPR = 360.0f;
bool checkFree(uint16_t sensor){
    if (sensor >= open_wall){
        return true;
    }
    return false;
}

void reset_filter(uint16_t *buffer, uint32_t &sum, int &idx, bool &filled)
{
    for (int i = 0; i < Size; i++)
        buffer[i] = 0;
    sum = 0;
    idx = 0;
    filled = false;
}

void reset_all_filters(){
    reset_filter(frontBuffer, sumFront, idxFront, filledFront);
    reset_filter(leftBuffer,  sumLeft,  idxLeft,  filledLeft);
    reset_filter(rightBuffer, sumRight, idxRight, filledRight);
}

void turn(int degree) {
    bool leftTurn = (degree > 0);
    encoder_reset();
    float rad = abs(degree) * PI / 180.0f;
    float mmPerTick = (PI * WHEEL_D) / CPR;
    float arc = (TRACK_W / 2.0f) * rad;
    int tick = (int)(arc / mmPerTick + 0.5f);
    if (leftTurn) {
        motor_set_pid(-speed*scaleL, speed*scaleR);
    } else {
        motor_set_pid(speed*scaleL, -speed*scaleR);
    }

    while (true) {
        int L = abs(encoder_left_ticks());
        int R = abs(encoder_right_ticks());
        if (leftTurn) {
            if (L <= -tick || R >= tick) break;
        } else {
            if (L >= tick || R <= -tick) break;
        }
        sleep_ms(5);
    }
    motor_stop_pid();
    sleep_ms(200);
    reset_all_filters();
}

const int CELL_TICKS = 644;
void drive_one_cell() {
    encoder_reset();
    while (true) {
        uint16_t front,left,right;
        read_sensors(front,left,right);
        if (front < 40) { motor_stop_pid(); break; }

        float corr = 0;
        if (left < 60)
            corr = std::clamp(0.5f * (left - 60), -50.0f, 50.0f);

        motor_set_pwm(
            std::clamp(speed - corr, 0.0f, 50.0f),
            std::clamp(speed + corr, 0.0f, 50.0f)
        );

        int avg = (abs(encoder_left_ticks()) + abs(encoder_right_ticks())) / 2;
        if (avg >= CELL_TICKS) {
            motor_stop_pid();
            break;
        }
        sleep_ms(15);
    }
    reset_all_filters();
}

void turn_left(){
    turn(80);
    drive_one_cell();
    sleep_ms(20);
}

void turn_right(){
    turn(-80);
    drive_one_cell();
    sleep_ms(20);
}

void u_turn(){
    turn(160);
    drive_one_cell();
    sleep_ms(20);
}

void lab3(){
    //tof_update();
    uint16_t front,left,right;
    
    if (buttonA_pressed()) {
        while(true){
        // uint16_t front,left,right;
        read_sensors(front,left,right);

        // Priority 1: Left side free-> turn left
        if(checkFree(left)){
            turn_left();
            continue;
        }

        // Priority 2: Forward path free-> drive forward
        if(checkFree(front)){
            drive_one_cell();
            continue;
        }

        // Priority 3: Right side free-> turn right
        if(checkFree(right)){
            turn_right();
            continue;
        }

        // Priority 4: Dead end-> 180° turn
        u_turn();
        }
    }
}

// ======================================================
//  Lab Dispatcher
// ======================================================
// Called from main.cpp -> loop()
// Select the task you want to execute by editing lab_main()
void lab_main() {
    lab3();   // <- Change this to lab1_task2(), etc.
}