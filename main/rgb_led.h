// #pragma once
// #include <stdint.h>
// #include "esp_err.h"


// typedef enum { MODE_OFF=0, MODE_ON=1, MODE_AUTO=2 } light_mode_t;

// esp_err_t rgb_init(void);
// void rgb_set_rgb(uint8_t r, uint8_t g, uint8_t b);
// static inline void rgb_set_black(void){ rgb_set_rgb(0,0,0); }
// void rgb_set_mode_color(light_mode_t m);   // OFF=red, AUTO=green, ON=blue


// #pragma once
// #include "esp_err.h"
// #include <stdint.h>

// typedef enum { MODE_OFF=0, MODE_ON=1, MODE_AUTO=2 } light_mode_t;

// // Единый адресный RGB-пиксель WS2812 на GPIO PIN_RGB
// #ifndef PIN_RGB
// #define PIN_RGB 8        // DevKitC-1: встроенный светодиод сидит на GPIO8
// #endif

// // Режимы (используются и main.c, и rgb_led.c)
// typedef enum {
//     MODE_OFF  = 0,
//     MODE_ON   = 1,
//     MODE_AUTO = 2
// } light_mode_t;

// // Инициализация драйвера (RMTv2 + WS2812)
// esp_err_t rgb_init(void);

// // Установить цвет напрямую (0..255)
// void rgb_set_rgb(uint8_t r, uint8_t g, uint8_t b);

// // Индикация режима (OFF=красный, AUTO=зелёный, ON=синий)
// void rgb_set_mode_color(light_mode_t m);


#pragma once
#include <stdint.h>
#include "esp_err.h"

// Единый тип режима для всего проекта
typedef enum {
    MODE_OFF  = 0,
    MODE_ON   = 1,
    MODE_AUTO = 2
} light_mode_t;

// Адресный RGB WS2812 на GPIO PIN_RGB
#ifndef PIN_RGB
#define PIN_RGB 8
#endif

esp_err_t rgb_init(void);
void rgb_set_rgb(uint8_t r, uint8_t g, uint8_t b);
void rgb_set_mode_color(light_mode_t m);  // OFF=red, AUTO=green, ON=blue
