#include "config_store.h"
#include "config.h"

#include "esp_log.h"
#include "nvs.h"
#include <string.h>

static const char *TAG = "config_store";
static app_config_t s_cfg;
static bool s_configured = false;

static void config_store_apply_defaults(app_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->magic = CONFIG_STORE_MAGIC;
    cfg->version = CONFIG_STORE_VERSION;
    cfg->size = sizeof(*cfg);
    cfg->zone_id = ZONE_ID;
    cfg->auto_hold_ms = AUTO_HOLD_MS;
    cfg->tfmini_trigger_cm = TFMINI_TRIGGER_CM;
    cfg->tfmini_release_cm = TFMINI_RELEASE_CM;
    cfg->ot_channel = OT_CHANNEL;
    cfg->ot_panid = OT_PANID;
    memcpy(cfg->ot_ext_panid, OT_EXT_PANID_DEFAULT, sizeof(cfg->ot_ext_panid));
    memcpy(cfg->ot_network_key, OT_NETWORK_KEY_DEFAULT, sizeof(cfg->ot_network_key));
    strlcpy(cfg->ot_network_name, OT_NETWORK_NAME, sizeof(cfg->ot_network_name));
}

static bool config_store_is_valid(const app_config_t *cfg)
{
    if (!cfg) {
        return false;
    }
    if (cfg->magic != CONFIG_STORE_MAGIC || cfg->version != CONFIG_STORE_VERSION) {
        return false;
    }
    if (cfg->size != sizeof(*cfg)) {
        return false;
    }
    if (cfg->ot_channel < 11 || cfg->ot_channel > 26) {
        return false;
    }
    if (cfg->ot_network_name[0] == '\0') {
        return false;
    }
    return true;
}

static esp_err_t config_store_load(app_config_t *cfg)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(CONFIG_STORE_NS, NVS_READONLY, &h);
    if (err != ESP_OK) {
        return err;
    }

    size_t len = sizeof(*cfg);
    err = nvs_get_blob(h, "cfg", cfg, &len);
    nvs_close(h);
    if (err != ESP_OK || len != sizeof(*cfg)) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

void config_store_init(void)
{
    config_store_apply_defaults(&s_cfg);
    app_config_t stored;
    if (config_store_load(&stored) == ESP_OK && config_store_is_valid(&stored)) {
        s_cfg = stored;
        s_configured = true;
        ESP_LOGI(TAG, "Loaded config from NVS");
    } else {
        s_configured = false;
        ESP_LOGI(TAG, "Using default config");
    }
}

const app_config_t *config_store_get(void)
{
    return &s_cfg;
}

bool config_store_is_configured(void)
{
    return s_configured;
}

esp_err_t config_store_save(const app_config_t *cfg)
{
    if (!cfg || !config_store_is_valid(cfg)) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t h;
    esp_err_t err = nvs_open(CONFIG_STORE_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_blob(h, "cfg", cfg, sizeof(*cfg));
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    if (err == ESP_OK) {
        s_cfg = *cfg;
        s_configured = true;
    }
    return err;
}

void config_store_reset_to_defaults(void)
{
    app_config_t cfg;
    config_store_apply_defaults(&cfg);
    config_store_save(&cfg);
}
