#pragma once

#include "driver/uart.h"

// ========== РОЛЬ УЗЛА ==========
#define ROLE_CONTROLLER      0     // 0 = исполнитель, 1 = контроллер с кулачком
#define HAS_TFMINI           1     // 1 = есть датчик, 0 = relay-only node

// ========== ПИНЫ ПЛАТЫ ==========
#define PIN_RELAY            4     // вход SSR (GPIO → оптодиод SSR)
#define PIN_SW_A             6     // пины тумблера - на исполнителе не используются
#define PIN_SW_B             7
#define PIN_RGB              8     // встроенный WS2812 на DevKitC-1 обычно GPIO8

// ========== TFmini ==========
#define UART_PORT            UART_NUM_1
#define UART_TFMINI_RX       5     // GPIO для RX TFmini (проверь свой)
#define UART_TFMINI_TX       -1

// ========== ЛОГИКА ==========
#define ZONE_ID              1
#define AUTO_HOLD_MS         300000    // 10 минут удержания при AUTO

// ========== DATASET Thread (дефолты) ==========
#define OT_CHANNEL           15
#define OT_PANID             0x1234
#define OT_NETWORK_NAME      "WAREHOUSE"

static const uint8_t OT_EXT_PANID_DEFAULT[8] = {
    0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x00, 0x00, 0x01,
};
static const uint8_t OT_NETWORK_KEY_DEFAULT[16] = {
    0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
    0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00,
};

// ========== TFmini пороги (дефолты) ==========
#define TFMINI_TRIGGER_CM    165  // включить при <= 1м
#define TFMINI_RELEASE_CM    170  // считать “ушёл” при > 1.2м
