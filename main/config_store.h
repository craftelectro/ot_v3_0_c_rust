#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CONFIG_STORE_NS "cfg"
#define CONFIG_STORE_MAGIC 0x43464701u
#define CONFIG_STORE_VERSION 1u
#define CONFIG_STORE_NET_NAME_MAX 16

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint8_t zone_id;
    uint32_t auto_hold_ms;
    uint16_t tfmini_trigger_cm;
    uint16_t tfmini_release_cm;
    uint16_t ot_channel;
    uint16_t ot_panid;
    uint8_t ot_ext_panid[8];
    uint8_t ot_network_key[16];
    char ot_network_name[CONFIG_STORE_NET_NAME_MAX];
} app_config_t;

void config_store_init(void);
const app_config_t *config_store_get(void);
bool config_store_is_configured(void);
esp_err_t config_store_save(const app_config_t *cfg);
void config_store_reset_to_defaults(void);

#ifdef __cplusplus
}
#endif
