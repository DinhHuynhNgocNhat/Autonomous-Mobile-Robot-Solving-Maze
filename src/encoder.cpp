#include <Arduino.h>
#include "hardware/gpio.h"
#include "encoder.h"

#define ENC_L_A 12
#define ENC_L_B 13
#define ENC_R_A 8
#define ENC_R_B 9

// Quadratur-State: 2 Bits (A<<1 | B)
static volatile uint8_t lastL = 0;
static volatile uint8_t lastR = 0;
static volatile int32_t cntL = 0;
static volatile int32_t cntR = 0;

// Übergangstabelle: Index = (alt<<2) | neu
// +1 = vorwärts, -1 = rückwärts, 0 = ungültig/kein Schritt
static const int8_t quad_table[16] = {
   0, -1, +1,  0,
  +1,  0,  0, -1,
  -1,  0,  0, +1,
   0, +1, -1,  0
};

static inline uint8_t readAB(uint pinA, uint pinB) {
  return (gpio_get(pinA) << 1) | gpio_get(pinB);
}

static void gpio_irq_handler(uint gpio, uint32_t events) {
  // LED-Feedback
  gpio_xor_mask(1u << LED_BUILTIN);

  if (gpio == ENC_L_A || gpio == ENC_L_B) {
    uint8_t newL = readAB(ENC_L_A, ENC_L_B);
    uint8_t idx  = ((lastL << 2) | newL) & 0x0F;
    cntL += quad_table[idx];
    lastL = newL;
  }
  else if (gpio == ENC_R_A || gpio == ENC_R_B) {
    uint8_t newR = readAB(ENC_R_A, ENC_R_B);
    uint8_t idx  = ((lastR << 2) | newR) & 0x0F;
    cntR += quad_table[idx];
    lastR = newR;
  }
}

void encoder_init() {
  // Eingänge (externe Pull-ups)
  gpio_init(ENC_L_A); gpio_set_dir(ENC_L_A, false);
  gpio_init(ENC_L_B); gpio_set_dir(ENC_L_B, false);
  gpio_init(ENC_R_A); gpio_set_dir(ENC_R_A, false);
  gpio_init(ENC_R_B); gpio_set_dir(ENC_R_B, false);

  gpio_set_input_hysteresis_enabled(ENC_L_A, true);
  gpio_set_input_hysteresis_enabled(ENC_L_B, true);
  gpio_set_input_hysteresis_enabled(ENC_R_A, true);
  gpio_set_input_hysteresis_enabled(ENC_R_B, true);

  // LED zur Kontrolle
  gpio_init(LED_BUILTIN);
  gpio_set_dir(LED_BUILTIN, true);
  gpio_put(LED_BUILTIN, 0);

  // Anfangszustände speichern
  lastL = readAB(ENC_L_A, ENC_L_B);
  lastR = readAB(ENC_R_A, ENC_R_B);

  // Erster Callback aktiviert globalen IRQ
  gpio_set_irq_enabled_with_callback(
      ENC_L_A,
      GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
      true,
      &gpio_irq_handler);

  // alle Flanken auf allen A/B aktivieren
  gpio_set_irq_enabled(ENC_L_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
  gpio_set_irq_enabled(ENC_R_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
  gpio_set_irq_enabled(ENC_R_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

int32_t encoder_left_ticks()  { return cntL; }
int32_t encoder_right_ticks() { return cntR; }
void encoder_reset()          { cntL = cntR = 0; }