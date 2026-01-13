#ifndef _SSD1306_H_
#define _SSD1306_H_

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdbool.h>
#include <stdint.h>

#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64

typedef struct {
    uint16_t width;
    uint16_t height;
    bool external_vcc;
    uint8_t address;
    i2c_inst_t *i2c_instance;
    uint8_t buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];
} ssd1306_t;

// Initialisation de l’écran
void ssd1306_init(ssd1306_t *p, uint16_t width, uint16_t height, bool external_vcc, uint8_t address, i2c_inst_t *i2c_instance);

// Efface le tampon (RAM)
void ssd1306_clear(ssd1306_t *p);

// Affiche le contenu du tampon sur l’écran
void ssd1306_show(ssd1306_t *p);

// Dessine un pixel
void ssd1306_draw_pixel(ssd1306_t *p, int x, int y, bool color);

// Dessine un caractère ASCII
void ssd1306_draw_char(ssd1306_t *p, int x, int y, char c, bool color);

// Dessine une chaîne de texte
void ssd1306_draw_string(ssd1306_t *p, int x, int y, bool color, const char *str);

#endif
