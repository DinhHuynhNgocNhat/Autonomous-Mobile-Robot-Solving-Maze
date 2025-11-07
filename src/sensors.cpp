#include "sensors.h"

// === Initialization ===
void sensors_init(void) {
    gpio_init(BTN_A);
    gpio_set_dir(BTN_A, GPIO_IN);
    gpio_pull_up(BTN_A);
}

// === Button read functions ===
bool buttonA_pressed(void) {
    return gpio_get(BTN_A) == 0;  // aktiv LOW
}

bool buttonC_pressed(void) {
    return gpio_get(BTN_C) == 0;
}

// === Debounce helper ===
bool wait_for_button_release(uint pin) {
    while (gpio_get(pin) == 0) {
        sleep_ms(10);
    }
    sleep_ms(50);  // Entprellzeit
    return true;
}