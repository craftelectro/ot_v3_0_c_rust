// #include "rgb_led.h"
// #include "config.h"
// #include "esp_log.h"
// #include "driver/gpio.h"

// #if LED_TYPE_WS2812
// #include "led_strip.h"
// static led_strip_handle_t s_strip;
// #endif

// static const char *TAG = "rgb";

// esp_err_t rgb_init(void)
// {
// #if LED_TYPE_WS2812
//     led_strip_config_t strip_cfg = {
//         .strip_gpio_num = LED_WS_PIN,
//         .max_leds = LED_WS_LENGTH,
//         .led_pixel_format = LED_PIXEL_FORMAT_GRB,
//         .led_model = LED_MODEL_WS2812,
//         .flags.invert_out = false,
//     };
//     led_strip_rmt_config_t rmt_cfg = {
//         .clk_src = RMT_CLK_SRC_DEFAULT,
//         .resolution_hz = 10 * 1000 * 1000,
//         .mem_block_symbols = 0,
//     };
//     ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &s_strip));
//     led_strip_clear(s_strip);
//     ESP_LOGI(TAG, "WS2812 on GPIO%d", LED_WS_PIN);
// #else
//     gpio_config_t io = {
//         .pin_bit_mask = (1ULL<<PIN_LED_R)|(1ULL<<PIN_LED_G)|(1ULL<<PIN_LED_B),
//         .mode = GPIO_MODE_OUTPUT
//     };
//     ESP_ERROR_CHECK(gpio_config(&io));
//     gpio_set_level(PIN_LED_R, 0);
//     gpio_set_level(PIN_LED_G, 0);
//     gpio_set_level(PIN_LED_B, 0);
// #endif
//     return ESP_OK;
// }

// void rgb_set_rgb(uint8_t r, uint8_t g, uint8_t b)
// {
// #if LED_TYPE_WS2812
//     led_strip_set_pixel(s_strip, 0, r, g, b);
//     led_strip_refresh(s_strip);
// #else
//     gpio_set_level(PIN_LED_R, r>0);
//     gpio_set_level(PIN_LED_G, g>0);
//     gpio_set_level(PIN_LED_B, b>0);
// #endif
// }

// void rgb_set_mode_color(light_mode_t m)
// {
//     switch (m) {
//         case MODE_OFF:  rgb_set_rgb(64, 0, 0); break;
//         case MODE_AUTO: rgb_set_rgb(0, 64, 0); break;
//         case MODE_ON:   rgb_set_rgb(0, 0, 64); break;
//         default:        rgb_set_black(); break;
//     }
// }




// #include "rgb_led.h"
// #include "config.h"
// #include "esp_log.h"
// #include "driver/gpio.h"

// #if LED_TYPE_WS2812
// #include "led_strip.h"
// static led_strip_handle_t s_strip;
// #endif

// static const char *TAG = "rgb";

// esp_err_t rgb_init(void)
// {
// #if LED_TYPE_WS2812
//     led_strip_config_t strip_cfg = {
//         .strip_gpio_num = LED_WS_PIN,
//         .max_leds = LED_WS_LENGTH,
//         .led_pixel_format = LED_PIXEL_FORMAT_GRB,
//         .led_model = LED_MODEL_WS2812,
//         .flags.invert_out = false,
//     };
//     led_strip_rmt_config_t rmt_cfg = {
//         .clk_src = RMT_CLK_SRC_DEFAULT,
//         .resolution_hz = 10 * 1000 * 1000,
//         .mem_block_symbols = 0,
//     };
//     ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &s_strip));
//     led_strip_clear(s_strip);
//     ESP_LOGI(TAG, "WS2812 on GPIO%d", LED_WS_PIN);
// #else
//     gpio_config_t io = {
//         .pin_bit_mask = (1ULL<<PIN_LED_R)|(1ULL<<PIN_LED_G)|(1ULL<<PIN_LED_B),
//         .mode = GPIO_MODE_OUTPUT
//     };
//     ESP_ERROR_CHECK(gpio_config(&io));
//     gpio_set_level(PIN_LED_R, 0);
//     gpio_set_level(PIN_LED_G, 0);
//     gpio_set_level(PIN_LED_B, 0);
// #endif
//     return ESP_OK;
// }

// void rgb_set_rgb(uint8_t r, uint8_t g, uint8_t b)
// {
// #if LED_TYPE_WS2812
//     led_strip_set_pixel(s_strip, 0, r, g, b);
//     led_strip_refresh(s_strip);
// #else
//     gpio_set_level(PIN_LED_R, r>0);
//     gpio_set_level(PIN_LED_G, g>0);
//     gpio_set_level(PIN_LED_B, b>0);
// #endif
// }

// void rgb_set_mode_color(light_mode_t m)
// {
//     // OFF = красный, AUTO = зелёный, ON = синий
//     switch (m) {
//         case MODE_OFF:  rgb_set_rgb(64, 0, 0); break;   // красный
//         case MODE_AUTO: rgb_set_rgb(0, 64, 0); break;   // зелёный
//         case MODE_ON:   rgb_set_rgb(0, 0, 64); break;   // синий
//         default:        rgb_set_rgb(0, 0, 0);  break;   // чёрный
//     }
// }


#include "rgb_led.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include <string.h>

static const char *TAG = "rgb";

// ===================== WS2812 через RMT v2 =====================
// Тайминги WS2812 (типичные) — возьмем частоту RMT 10 MHz (1 тик = 100 нс)
#define RMT_RES_HZ         10000000  // 10 MHz
#define T0H_TICKS          4         // 0.4 us ~ 4 * 100 ns
#define T0L_TICKS          8         // 0.8 us
#define T1H_TICKS          8         // 0.8 us
#define T1L_TICKS          4         // 0.4 us
#define TRESET_TICKS       800       // 80 us (низкий уровень для "reset")

// Буфер на 24 бита (GRB), каждый бит — один rmt_symbol_word_t
static rmt_channel_handle_t   s_rmt_ch   = NULL;
static rmt_encoder_handle_t   s_copy_enc = NULL;
static rmt_transmit_config_t  s_tx_cfg   = {
    .loop_count = 0,
};

// Формируем символ по биту: '1' или '0'
static inline rmt_symbol_word_t ws2812_sym_one(void)
{
    rmt_symbol_word_t s = { 0 };
    s.level0    = 1;
    s.duration0 = T1H_TICKS;
    s.level1    = 0;
    s.duration1 = T1L_TICKS;
    return s;
}
static inline rmt_symbol_word_t ws2812_sym_zero(void)
{
    rmt_symbol_word_t s = { 0 };
    s.level0    = 1;
    s.duration0 = T0H_TICKS;
    s.level1    = 0;
    s.duration1 = T0L_TICKS;
    return s;
}

// Кодируем один байт (MSB first) в массив символов начиная с индекса *idx
static inline void ws2812_encode_byte(uint8_t byte, rmt_symbol_word_t *syms, int *idx)
{
    for (int i = 7; i >= 0; --i) {
        if (byte & (1 << i)) syms[(*idx)++] = ws2812_sym_one();
        else                  syms[(*idx)++] = ws2812_sym_zero();
    }
}

// Отправка одного пикселя (GRB)
static void ws2812_send_pixel(uint8_t g, uint8_t r, uint8_t b)
{
    // 24 бита данных + 2 символа на Treset (низкий уровень)
    rmt_symbol_word_t frame[24 + 2] = {0};
    int idx = 0;

    ws2812_encode_byte(g, frame, &idx);
    ws2812_encode_byte(r, frame, &idx);
    ws2812_encode_byte(b, frame, &idx);

    // Добавим «reset» — длинный низкий уровень (двумя символами подряд)
    frame[idx].level0    = 0;
    frame[idx].duration0 = TRESET_TICKS;
    frame[idx].level1    = 0;
    frame[idx].duration1 = 0;
    idx++;

    frame[idx].level0    = 0;
    frame[idx].duration0 = TRESET_TICKS;
    frame[idx].level1    = 0;
    frame[idx].duration1 = 0;
    idx++;

    // Передать копирующим энкодером
    esp_err_t err = rmt_transmit(s_rmt_ch, s_copy_enc, frame, sizeof(frame), &s_tx_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_transmit failed (%d)", err);
        return;
    }
    rmt_tx_wait_all_done(s_rmt_ch, -1);
}

// ===================== Публичные функции =====================

esp_err_t rgb_init(void)
{
    // Создаем TX-канал RMT
    rmt_tx_channel_config_t tx_cfg = {
        .gpio_num            = PIN_RGB,
        .clk_src             = RMT_CLK_SRC_DEFAULT,
        .resolution_hz       = RMT_RES_HZ,
        .mem_block_symbols   = 96,   // с запасом (несколько десятков символов)
        .trans_queue_depth   = 4,
        .flags.with_dma      = false,
        .flags.invert_out    = false,
        .flags.io_loop_back  = false,
        .flags.io_od_mode    = false,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_cfg, &s_rmt_ch));
    ESP_ERROR_CHECK(rmt_enable(s_rmt_ch));

    // Копирующий энкодер (передаем уже готовые символы)
    rmt_copy_encoder_config_t cec = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&cec, &s_copy_enc));

    ESP_LOGI(TAG, "WS2812 on GPIO%d", PIN_RGB);

    // Гасим при старте
    rgb_set_rgb(0,0,0);
    return ESP_OK;
}

void rgb_set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    // Прямая установка цвета 0..255
    ws2812_send_pixel(g, r, b); // передаем в порядке GRB
}

void rgb_set_mode_color(light_mode_t m)
{
    // OFF = красный, AUTO = зелёный, ON = синий
    switch (m) {
        case MODE_OFF:  rgb_set_rgb(255, 0,   0);   break;
        case MODE_AUTO: rgb_set_rgb(0,   255, 0);   break;
        case MODE_ON:   rgb_set_rgb(0,   0,   255); break;
        default:        rgb_set_rgb(0,   0,   0);   break;
    }
}
