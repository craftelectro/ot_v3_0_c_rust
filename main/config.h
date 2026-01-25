// #pragma once
// #include "driver/uart.h"

// // ===== зона/логика =====
// #define ZONE_ID                    1
// #define AUTO_HOLD_MS               (10*60*1000)   // 10 минут

// // ===== SSR =====
// #define PIN_RELAY                  4

// // ===== кулачковый переключатель (2 бита) =====
// // 00=OFF, 01=ON, 10/11=AUTO
// #define PIN_SW_A                   6
// #define PIN_SW_B                   7

// // ===== TFmini (UART1, только RX) =====
// #define UART_PORT                  UART_NUM_1
// #define UART_TFMINI_RX             20
// #define TFMINI_DISTANCE_CM_MAX     200   // «движение» ближе 2 м

// // ===== RGB LED =====
// // 1 — адресный (WS2812/NeoPixel) на плате; 0 — три дискретных GPIO
// #define LED_TYPE_WS2812            1
// #define LED_WS_PIN                 8
// #define LED_WS_LENGTH              1
// #define PIN_LED_R                  8   // если LED_TYPE_WS2812=0
// #define PIN_LED_G                  9
// #define PIN_LED_B                  10

// // ===== преднастроенный Dataset (можно менять) =====
// #define OT_CHANNEL                 15
// #define OT_PANID                   0x1234
// #define OT_NETWORK_NAME            "WAREHOUSE"
// static const uint8_t OT_EXT_PANID[8]  = {0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88};
// static const uint8_t OT_NETWORK_KEY[16]={
//     0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff
// };



#pragma once

// ========== РОЛЬ УЗЛА ==========
#define ROLE_CONTROLLER      0     // 0 = исполнитель (этот файл), 1 = контроллер с кулачком
#define HAS_TFMINI  1   // 1 = есть датчик, 0 = relay-only node


// ========== ПИНЫ ПЛАТЫ ==========
#define PIN_RELAY            4     // вход SSR (GPIO → оптодиод SSR)
#define PIN_SW_A             6     // пины тумблера - на исполнителе не используются
#define PIN_SW_B             7
#define PIN_RGB              8     // встроенный WS2812 на DevKitC-1 обычно GPIO8

// ========== TFmini ==========
#define UART_PORT            UART_NUM_1
#define UART_TFMINI_RX       5     // GPIO для RX TFmini (проверь свой)
#define UART_TFMINI_TX       -1  
// #define TFMINI_DISTANCE_CM_MAX  3000
#define TFMINI_TRIGGER_CM  165  // включить при <= 1м
#define TFMINI_RELEASE_CM  170  // считать “ушёл” при > 1.2м


// ========== ЛОГИКА ==========
#define ZONE_ID              1
#define AUTO_HOLD_MS         300000    // 10 минут удержания при AUTO

// ========== DATASET Thread ==========
#define OT_CHANNEL           15
#define OT_PANID             0x1234
#define OT_EXT_PANID         {0xDE,0xAD,0xBE,0xEF,0x00,0x00,0x00,0x01}
#define OT_NETWORK_KEY       {0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xAA,0xBB,0xCC,0xDD,0xEE,0xFF,0x00}
#define OT_NETWORK_NAME      "WAREHOUSE"
