#include "ot_app.h"
#include "coap_if.h"
#include "logic.h"
#include "config.h"

#include "esp_openthread.h"
#include "esp_openthread_lock.h"
#include "esp_openthread_netif_glue.h"
#include "esp_openthread_cli.h"   // <-- ДОБАВИТЬ
#include "esp_ot_config.h"        // <-- ДОБАВИТЬ
#include "esp_log.h"
#include "esp_netif.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <openthread/dataset.h>
#include <openthread/thread.h>
#include <openthread/ip6.h>
#include <openthread/logging.h>   // <-- ДОБАВИТЬ
#include <ctype.h>
#include <string.h>

static const char *TAG = "ot_app";

static int hex_value(char c)
{
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
        return 10 + (c - 'a');
    }
    if (c >= 'A' && c <= 'F') {
        return 10 + (c - 'A');
    }
    return -1;
}

static bool parse_hex_string(const char *str, uint8_t *out, size_t out_len)
{
    if (!str || !out || out_len == 0) {
        return false;
    }

    size_t idx = 0;
    int high = -1;

    for (const char *p = str; *p != '\0'; ++p) {
        if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) {
            ++p;
            continue;
        }

        if (isxdigit((unsigned char)*p)) {
            int val = hex_value(*p);
            if (val < 0) {
                return false;
            }
            if (high < 0) {
                high = val;
            } else {
                if (idx >= out_len) {
                    return false;
                }
                out[idx++] = (uint8_t)((high << 4) | val);
                high = -1;
            }
            continue;
        }

        if (*p == ':' || *p == '-' || *p == ' ') {
            continue;
        }

        return false;
    }

    return (high < 0) && (idx == out_len);
}

static bool dataset_has_required_fields(const otOperationalDataset *dataset)
{
    if (!dataset) {
        return false;
    }

    return dataset->mComponents.mIsActiveTimestampPresent &&
           dataset->mComponents.mIsChannelPresent &&
           dataset->mComponents.mIsPanIdPresent &&
           dataset->mComponents.mIsExtendedPanIdPresent &&
           dataset->mComponents.mIsNetworkKeyPresent &&
           dataset->mComponents.mIsNetworkNamePresent;
}

static esp_netif_t *init_openthread_netif(const esp_openthread_platform_config_t *config)
{
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_OPENTHREAD();
    esp_netif_t *netif = esp_netif_new(&cfg);
    ESP_ERROR_CHECK(esp_netif_attach(netif, esp_openthread_netif_glue_init(config)));
    return netif;
}

static void ensure_active_dataset(otInstance *ot)
{
    otOperationalDataset current;
    memset(&current, 0, sizeof(current));
    if (otDatasetGetActive(ot, &current) == OT_ERROR_NONE &&
        dataset_has_required_fields(&current)) {
        ESP_LOGI(TAG, "Active Dataset already present");
        return;
    }

    ESP_LOGW(TAG, "Active Dataset missing or incomplete, applying defaults");

    otOperationalDataset d;
    memset(&d, 0, sizeof(d));

    d.mActiveTimestamp = (otTimestamp){ .mSeconds = 1 };
    d.mChannel = OT_CHANNEL;
    d.mPanId   = OT_PANID;

    uint8_t xpanid[8] = {0};
    uint8_t nkey[16] = {0};

    bool ok = parse_hex_string(OT_EXT_PANID_STR, xpanid, sizeof(xpanid));
    if (!ok) {
        ESP_LOGE(TAG, "Invalid OT_EXT_PANID: %s", OT_EXT_PANID_STR);
    }
    ESP_ERROR_CHECK(ok ? ESP_OK : ESP_FAIL);

    ok = parse_hex_string(OT_NETWORK_KEY_STR, nkey, sizeof(nkey));
    if (!ok) {
        ESP_LOGE(TAG, "Invalid OT_NETWORK_KEY: %s", OT_NETWORK_KEY_STR);
    }
    ESP_ERROR_CHECK(ok ? ESP_OK : ESP_FAIL);

    memcpy(d.mExtendedPanId.m8, xpanid, sizeof(xpanid));
    memcpy(d.mNetworkKey.m8,     nkey,  sizeof(nkey));
    strcpy((char*)d.mNetworkName.m8, OT_NETWORK_NAME);

    d.mComponents.mIsActiveTimestampPresent = true;
    d.mComponents.mIsChannelPresent         = true;
    d.mComponents.mIsPanIdPresent           = true;
    d.mComponents.mIsExtendedPanIdPresent   = true;
    d.mComponents.mIsNetworkKeyPresent      = true;
    d.mComponents.mIsNetworkNamePresent     = true;

    otError e = otDatasetSetActive(ot, &d);
    ESP_LOGI(TAG, "otDatasetSetActive -> %d", e);
    ESP_ERROR_CHECK(e == OT_ERROR_NONE ? ESP_OK : ESP_FAIL);
    ESP_LOGI(TAG, "Active Dataset written to NVS");
}

static void ot_task_worker(void *ctx)
{
    esp_openthread_platform_config_t cfg = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config  = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config  = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };

    ESP_ERROR_CHECK(esp_openthread_init(&cfg));

#if CONFIG_OPENTHREAD_LOG_LEVEL_DYNAMIC
    (void)otLoggingSetLevel(CONFIG_LOG_DEFAULT_LEVEL);
#endif
#if CONFIG_OPENTHREAD_CLI
    esp_openthread_cli_init();
#endif

    esp_netif_t *ot_netif = init_openthread_netif(&cfg);
    esp_netif_set_default_netif(ot_netif);

    esp_openthread_lock_acquire(portMAX_DELAY);
    otInstance *ot = esp_openthread_get_instance();

    ensure_active_dataset(ot);

    otError e = otIp6SetEnabled(ot, true);
    if (e != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "otIp6SetEnabled(true) -> %d", e);
    }

    e = otThreadSetEnabled(ot, true);
    if (e != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "otThreadSetEnabled(true) -> %d", e);
    } else {
        ESP_LOGI(TAG, "Thread explicitly started");
    }
    esp_openthread_lock_release();

#if CONFIG_OPENTHREAD_CLI
    esp_openthread_cli_create_task();
#endif

    // стартуем логику и CoAP
    coap_if_register(ot);
    logic_start();

    esp_openthread_launch_mainloop();

    esp_openthread_netif_glue_deinit();
    esp_netif_destroy(ot_netif);
    vTaskDelete(NULL);
}

void ot_app_start(void)
{
    xTaskCreate(ot_task_worker, "ot_main", 10240, NULL, 5, NULL);
}
