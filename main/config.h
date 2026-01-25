#pragma once

#include "sdkconfig.h"

// ========== РОЛЬ УЗЛА ==========
#define ROLE_CONTROLLER      CONFIG_OT_ROLE_CONTROLLER // 0 = исполнитель, 1 = контроллер с кулачком
#define HAS_TFMINI           CONFIG_OT_HAS_TFMINI       // 1 = есть датчик, 0 = relay-only node


// ========== ПИНЫ ПЛАТЫ ==========
#define PIN_RELAY            CONFIG_OT_PIN_RELAY
#define PIN_SW_A             CONFIG_OT_PIN_SW_A
#define PIN_SW_B             CONFIG_OT_PIN_SW_B
#define PIN_RGB              CONFIG_OT_PIN_RGB

// ========== TFmini ==========
#define UART_PORT            UART_NUM_1
#define UART_TFMINI_RX       CONFIG_OT_UART_TFMINI_RX
#define UART_TFMINI_TX       CONFIG_OT_UART_TFMINI_TX
#define TFMINI_TRIGGER_CM    CONFIG_OT_TFMINI_TRIGGER_CM
#define TFMINI_RELEASE_CM    CONFIG_OT_TFMINI_RELEASE_CM


// ========== ЛОГИКА ==========
#define ZONE_ID              CONFIG_OT_ZONE_ID
#define AUTO_HOLD_MS         CONFIG_OT_AUTO_HOLD_MS

// ========== DATASET Thread ==========
#define OT_CHANNEL           CONFIG_OT_CHANNEL
#define OT_PANID             CONFIG_OT_PANID
#define OT_EXT_PANID_STR     CONFIG_OT_EXT_PANID
#define OT_NETWORK_KEY_STR   CONFIG_OT_NETWORK_KEY
#define OT_NETWORK_NAME      CONFIG_OT_NETWORK_NAME
