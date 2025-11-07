#include "oled.h"
#include "logo.h"
#include <stdint.h>
#include "font5x7.h"

// optional Offset for JMD1.3C / 3pi+ Display
static const uint8_t OLED_X_OFFSET = 2;

// === Local framebuffer: 128 x 64 / 8 = 1024 bytes ===
static uint8_t oled_buffer[128 * 8];

// === Low-level helpers ===
static inline void oled_cmd(uint8_t c) {
  gpio_put(OLED_DC, 0);
  spi_write_blocking(spi0, &c, 1);
}

static inline void oled_data(uint8_t d) {
  gpio_put(OLED_DC, 1);
  spi_write_blocking(spi0, &d, 1);
}

void oled_init() {
  gpio_init(OLED_DC); gpio_set_dir(OLED_DC, GPIO_OUT);
  gpio_init(OLED_RST); gpio_set_dir(OLED_RST, GPIO_OUT);
  spi_init(spi0, 8000000);
  gpio_set_function(OLED_SCK, GPIO_FUNC_SPI);
  gpio_set_function(OLED_MOSI, GPIO_FUNC_SPI);

  gpio_put(OLED_RST, 0); sleep_ms(20);
  gpio_put(OLED_RST, 1); sleep_ms(20);

  uint8_t initseq[] = {
    0xAE,0xD5,0x80,0xA8,0x3F,0xD3,0x00,0x40,
    0xAD,0x8B,0xA1,0xC8,0xDA,0x12,0x81,0xAF,
    0xD9,0x22,0xDB,0x35,0xA4,0xA6,0xAF
  };
  for (uint8_t c : initseq) oled_cmd(c);
  oled_clear();
  oled_update();
}

// === Framebuffer operations ===
void oled_clear() {
  memset(oled_buffer, 0x00, sizeof(oled_buffer));
}

void oled_update() {
  for (uint8_t page = 0; page < 8; page++) {
    oled_cmd(0xB0 + page);
    oled_cmd(OLED_X_OFFSET & 0x0F);
    oled_cmd(0x10 | (OLED_X_OFFSET >> 4));

    gpio_put(OLED_DC, 1);
    spi_write_blocking(spi0, &oled_buffer[page * 128], 128);
  }
}

// === Drawing primitives ===
void oled_char(uint8_t x, uint8_t page, char c) {
  if (c < 32 || c > 127) c = '?';
  const uint8_t *p = font5x7[c - 32];
  uint8_t xpos = 2 + x * 6;

  for (int i = 0; i < 5; i++) {
    if (xpos + i < 128)
      oled_buffer[page * 128 + xpos + i] = p[i];
  }
  if (xpos + 5 < 128)
    oled_buffer[page * 128 + xpos + 5] = 0x00;
}

void oled_text(uint8_t x, uint8_t page, const char *s) {
  while (*s) oled_char(x++, page, *s++);
}

void oled_hline(uint8_t x, uint8_t y, uint8_t length) {
  if (y >= 64) return;
  uint8_t page = y / 8;
  uint8_t bit  = 1 << (y % 8);
  for (uint8_t i = 0; i < length && (x + i) < 128; i++) {
    oled_buffer[page * 128 + x + i] |= bit;
  }
}

void oled_vline(uint8_t x, uint8_t y, uint8_t length) {
  if (x >= 128) return;
  for (uint8_t i = 0; i < length && (y + i) < 64; i++) {
    uint8_t page = (y + i) / 8;
    uint8_t bit  = 1 << ((y + i) % 8);
    oled_buffer[page * 128 + x] |= bit;
  }
}

// === Logo drawing (direct push, optional) ===
void oled_draw_logo() {
  const uint8_t *ptr = amr_logo;
  for (uint8_t page = 0; page < LOGO_HEIGHT / 8; page++) {
    memcpy(&oled_buffer[page * 128], ptr, LOGO_WIDTH);
    ptr += LOGO_WIDTH;
  }
}  