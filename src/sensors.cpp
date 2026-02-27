#include "sensors.h"

// ======================
//       BUTTONS
// ======================

// === Initialization ===
void buttons_init(void) {
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

// ======================
//     RANGE SENSORS
// ======================

// interne Werte
static uint16_t dist_left  = 0;
static uint16_t dist_front = 0;
static uint16_t dist_right = 0;

// interne Hilfsfunktion: Register schreiben
static void vl6180x_write8(uint8_t addr, uint16_t reg, uint8_t value) {
    uint8_t buf[3];
    buf[0] = (reg >> 8) & 0xFF;
    buf[1] = reg & 0xFF;
    buf[2] = value;
    i2c_write_blocking(i2c0, addr, buf, 3, false);
}

// internes Lesen eines 8-Bit-Registers
static uint8_t vl6180x_read8(uint8_t addr, uint16_t reg) {
    uint8_t buf[2];
    buf[0] = (reg >> 8) & 0xFF;
    buf[1] = reg & 0xFF;
    i2c_write_blocking(i2c0, addr, buf, 2, true);
    uint8_t value = 0;
    i2c_read_blocking(i2c0, addr, &value, 1, false);
    return value;
}

// Adresse ändern
static void vl6180x_set_address(uint8_t new_addr) {
    vl6180x_write8(VL6180X_BASE_ADDRESS, 0x212, new_addr & 0x7F);
    sleep_ms(2);
}

// Distanzmessung starten und lesen
static uint16_t vl6180x_read_range(uint8_t addr) {
    vl6180x_write8(addr, 0x018, 0x01);   // SYSRANGE_START
    sleep_ms(10);
    return vl6180x_read8(addr, 0x062);   // RESULT_RANGE_VAL
}

// === Minimal-Setup für VL6180X ===
static void vl6180x_init(uint8_t addr) {
    // Mandatory init sequence (aus Pololu-Beispiel)
    vl6180x_write8(addr, 0x0207, 0x01);
    vl6180x_write8(addr, 0x0208, 0x01);
    vl6180x_write8(addr, 0x0096, 0x00);
    vl6180x_write8(addr, 0x0097, 0xfd);
    vl6180x_write8(addr, 0x00e3, 0x00);
    vl6180x_write8(addr, 0x00e4, 0x04);
    vl6180x_write8(addr, 0x00e5, 0x02);
    vl6180x_write8(addr, 0x00e6, 0x01);
    vl6180x_write8(addr, 0x00e7, 0x03);
    vl6180x_write8(addr, 0x00f5, 0x02);
    vl6180x_write8(addr, 0x00d9, 0x05);
    vl6180x_write8(addr, 0x00db, 0xce);
    vl6180x_write8(addr, 0x00dc, 0x03);
    vl6180x_write8(addr, 0x00dd, 0xf8);
    vl6180x_write8(addr, 0x009f, 0x00);
    vl6180x_write8(addr, 0x00a3, 0x3c);
    vl6180x_write8(addr, 0x00b7, 0x00);
    vl6180x_write8(addr, 0x00bb, 0x3c);
    vl6180x_write8(addr, 0x00b2, 0x09);
    vl6180x_write8(addr, 0x00ca, 0x09);
    vl6180x_write8(addr, 0x0198, 0x01);
    vl6180x_write8(addr, 0x01b0, 0x17);
    vl6180x_write8(addr, 0x01ad, 0x00);
    vl6180x_write8(addr, 0x00ff, 0x05);
    vl6180x_write8(addr, 0x0100, 0x05);
    vl6180x_write8(addr, 0x0199, 0x05);
    vl6180x_write8(addr, 0x01a6, 0x1b);
    vl6180x_write8(addr, 0x01ac, 0x3e);
    vl6180x_write8(addr, 0x01a7, 0x1f);
    vl6180x_write8(addr, 0x0030, 0x00);
}

// === Initialisierung ===
void tof_init() {
    // Pins für I2C (angepasst an dein Board)
    i2c_init(i2c0, 400 * 1000);
    gpio_set_function(4, GPIO_FUNC_I2C);  // SDA
    gpio_set_function(5, GPIO_FUNC_I2C);  // SCL
    gpio_pull_up(4);
    gpio_pull_up(5);

    // Enable-Pins
    gpio_init(TOF_FRONT);
    gpio_init(TOF_RIGHT);
    gpio_set_dir(TOF_FRONT, GPIO_OUT);
    gpio_set_dir(TOF_RIGHT, GPIO_OUT);

    // alle aus
    gpio_put(TOF_FRONT, 0);
    gpio_put(TOF_RIGHT, 0);
    sleep_ms(10);

    vl6180x_set_address(VL6180X_LEFT_ADDRESS);

    // FRONT aktivieren und umadressieren
    gpio_put(TOF_FRONT, 1);
    sleep_ms(100);
    vl6180x_set_address(VL6180X_FRONT_ADDRESS);

    // RIGHT aktivieren und umadressieren
    gpio_put(TOF_RIGHT, 1);
    sleep_ms(100);
    vl6180x_set_address(VL6180X_RIGHT_ADDRESS);

    vl6180x_init(VL6180X_LEFT_ADDRESS);
    vl6180x_init(VL6180X_FRONT_ADDRESS);
    vl6180x_init(VL6180X_RIGHT_ADDRESS);
    
    sleep_ms(10);
}

// === Update ===
// void tof_update() {
//     dist_left  = vl6180x_read_range(VL6180X_LEFT_ADDRESS);
//     dist_front = vl6180x_read_range(VL6180X_FRONT_ADDRESS);
//     dist_right = vl6180x_read_range(VL6180X_RIGHT_ADDRESS);
// }

void tof_update(void)
{
    uint16_t d;
    static uint16_t last_left  = 0;
    static uint16_t last_front = 0;
    static uint16_t last_right = 0;

    d = vl6180x_read_range(VL6180X_LEFT_ADDRESS);
    dist_left = (d == last_left) ? 255 : d;
    last_left = d;

    d = vl6180x_read_range(VL6180X_FRONT_ADDRESS);
    dist_front = (d == last_front) ? 255 : d;
    last_front = d;

    d = vl6180x_read_range(VL6180X_RIGHT_ADDRESS);
    dist_right = (d == last_right) ? 255 : d;
    last_right = d;
}

// === Getter ===
uint16_t tof_get_left(void)  { return dist_left;  }
uint16_t tof_get_front(void) { return dist_front; }
uint16_t tof_get_right(void) { return dist_right; }