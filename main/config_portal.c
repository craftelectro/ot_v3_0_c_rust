#include "config_portal.h"
#include "config_store.h"
#include "config.h"

#include "esp_err.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs.h"

#include "driver/gpio.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "config_portal";
static httpd_handle_t s_server;
static bool s_running = false;
static bool s_force_portal = false;
static esp_timer_handle_t s_clear_timer = NULL;

#define RESET_NS "portal"
#define RESET_KEY "rst_count"
#define RESET_CLEAR_DELAY_US (5 * 1000 * 1000)

static int reset_sequence_load(void)
{
    nvs_handle_t h;
    int count = 0;
    if (nvs_open(RESET_NS, NVS_READONLY, &h) != ESP_OK) {
        return 0;
    }
    (void)nvs_get_i32(h, RESET_KEY, &count);
    nvs_close(h);
    if (count < 0) {
        count = 0;
    }
    return count;
}

static void reset_sequence_save(int count)
{
    nvs_handle_t h;
    if (nvs_open(RESET_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    (void)nvs_set_i32(h, RESET_KEY, count);
    (void)nvs_commit(h);
    nvs_close(h);
}

static void reset_sequence_clear_cb(void *arg)
{
    (void)arg;
    reset_sequence_save(0);
}

static void reset_sequence_schedule_clear(void)
{
    if (!s_clear_timer) {
        esp_timer_create_args_t args = {
            .callback = reset_sequence_clear_cb,
            .name = "portal_clear",
        };
        if (esp_timer_create(&args, &s_clear_timer) != ESP_OK) {
            return;
        }
    }
    (void)esp_timer_stop(s_clear_timer);
    (void)esp_timer_start_once(s_clear_timer, RESET_CLEAR_DELAY_US);
}

static bool detect_reset_sequence(void)
{
    int count = reset_sequence_load();
    count++;

    if (count >= 3) {
        reset_sequence_save(0);
        return true;
    }

    reset_sequence_save(count);
    reset_sequence_schedule_clear();
    return false;
}

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

static void url_decode(char *dst, const char *src, size_t dst_len)
{
    size_t di = 0;
    for (size_t si = 0; src[si] != '\0' && di + 1 < dst_len; ++si) {
        if (src[si] == '+') {
            dst[di++] = ' ';
            continue;
        }
        if (src[si] == '%' && isxdigit((unsigned char)src[si + 1]) &&
            isxdigit((unsigned char)src[si + 2])) {
            int hi = hex_value(src[si + 1]);
            int lo = hex_value(src[si + 2]);
            dst[di++] = (char)((hi << 4) | lo);
            si += 2;
            continue;
        }
        dst[di++] = src[si];
    }
    dst[di] = '\0';
}

static const char *form_value(const char *body, const char *key, char *out, size_t out_len)
{
    size_t key_len = strlen(key);
    const char *p = body;
    while (p && *p) {
        const char *eq = strchr(p, '=');
        if (!eq) {
            return NULL;
        }
        size_t len = (size_t)(eq - p);
        if (len == key_len && strncmp(p, key, key_len) == 0) {
            const char *end = strchr(eq + 1, '&');
            size_t val_len = end ? (size_t)(end - (eq + 1)) : strlen(eq + 1);
            char tmp[128];
            if (val_len >= sizeof(tmp)) {
                val_len = sizeof(tmp) - 1;
            }
            memcpy(tmp, eq + 1, val_len);
            tmp[val_len] = '\0';
            url_decode(out, tmp, out_len);
            return out;
        }
        p = strchr(eq, '&');
        if (p) {
            ++p;
        }
    }
    return NULL;
}

static esp_err_t handle_root(httpd_req_t *req)
{
    const app_config_t *cfg = config_store_get();
    char xpanid_hex[17];
    char netkey_hex[33];
    for (size_t i = 0; i < sizeof(cfg->ot_ext_panid); ++i) {
        snprintf(&xpanid_hex[i * 2], 3, "%02X", cfg->ot_ext_panid[i]);
    }
    for (size_t i = 0; i < sizeof(cfg->ot_network_key); ++i) {
        snprintf(&netkey_hex[i * 2], 3, "%02X", cfg->ot_network_key[i]);
    }
    char html[1536];
    int len = snprintf(
        html,
        sizeof(html),
        "<!doctype html><html><head><meta charset='utf-8'>"
        "<title>OT Light Config</title></head><body>"
        "<h2>OpenThread Config</h2>"
        "<form method='POST' action='/save'>"
        "Network name:<br><input name='net_name' value='%s'><br>"
        "Channel (11-26):<br><input name='channel' value='%u'><br>"
        "PAN ID (hex, e.g. 0x1234):<br><input name='panid' value='0x%04x'><br>"
        "XPANID (16 hex chars):<br><input name='xpanid' value='%s'><br>"
        "Network key (32 hex chars):<br><input name='netkey' value='%s'><br>"
        "Zone ID:<br><input name='zone_id' value='%u'><br>"
        "Auto-hold ms:<br><input name='auto_hold_ms' value='%u'><br>"
        "TFmini trigger cm:<br><input name='tf_trigger' value='%u'><br>"
        "TFmini release cm:<br><input name='tf_release' value='%u'><br><br>"
        "<button type='submit'>Save & Reboot</button>"
        "</form>"
        "</body></html>",
        cfg->ot_network_name,
        cfg->ot_channel,
        cfg->ot_panid,
        xpanid_hex,
        netkey_hex,
        cfg->zone_id,
        (unsigned)cfg->auto_hold_ms,
        cfg->tfmini_trigger_cm,
        cfg->tfmini_release_cm);

    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, html, len);
}

static bool parse_uint(const char *s, uint32_t *out)
{
    if (!s || !*s) {
        return false;
    }
    char *end = NULL;
    unsigned long v = strtoul(s, &end, 0);
    if (end == s || *end != '\0') {
        return false;
    }
    *out = (uint32_t)v;
    return true;
}

static esp_err_t handle_save(httpd_req_t *req)
{
    char body[512];
    int received = httpd_req_recv(req, body, sizeof(body) - 1);
    if (received <= 0) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty body");
    }
    body[received] = '\0';

    app_config_t cfg = *config_store_get();
    char tmp[128];
    uint32_t val = 0;

    if (form_value(body, "net_name", tmp, sizeof(tmp))) {
        if (tmp[0] != '\0') {
            strlcpy(cfg.ot_network_name, tmp, sizeof(cfg.ot_network_name));
        }
    }
    if (form_value(body, "channel", tmp, sizeof(tmp)) && parse_uint(tmp, &val)) {
        cfg.ot_channel = (uint16_t)val;
    }
    if (form_value(body, "panid", tmp, sizeof(tmp)) && parse_uint(tmp, &val)) {
        cfg.ot_panid = (uint16_t)val;
    }
    if (form_value(body, "zone_id", tmp, sizeof(tmp)) && parse_uint(tmp, &val)) {
        cfg.zone_id = (uint8_t)val;
    }
    if (form_value(body, "auto_hold_ms", tmp, sizeof(tmp)) && parse_uint(tmp, &val)) {
        cfg.auto_hold_ms = val;
    }
    if (form_value(body, "tf_trigger", tmp, sizeof(tmp)) && parse_uint(tmp, &val)) {
        cfg.tfmini_trigger_cm = (uint16_t)val;
    }
    if (form_value(body, "tf_release", tmp, sizeof(tmp)) && parse_uint(tmp, &val)) {
        cfg.tfmini_release_cm = (uint16_t)val;
    }
    if (form_value(body, "xpanid", tmp, sizeof(tmp)) && tmp[0] != '\0') {
        if (!parse_hex_string(tmp, cfg.ot_ext_panid, sizeof(cfg.ot_ext_panid))) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad XPANID");
        }
    }
    if (form_value(body, "netkey", tmp, sizeof(tmp)) && tmp[0] != '\0') {
        if (!parse_hex_string(tmp, cfg.ot_network_key, sizeof(cfg.ot_network_key))) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad network key");
        }
    }

    esp_err_t err = config_store_save(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Config save failed: %s", esp_err_to_name(err));
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Save failed");
    }

    httpd_resp_sendstr(req, "Saved. Rebooting...");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

static void start_http_server(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.uri_match_fn = httpd_uri_match_wildcard;

    if (httpd_start(&s_server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start http server");
        return;
    }

    httpd_uri_t root = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = handle_root,
    };
    httpd_uri_t save = {
        .uri = "/save",
        .method = HTTP_POST,
        .handler = handle_save,
    };
    httpd_register_uri_handler(s_server, &root);
    httpd_register_uri_handler(s_server, &save);
}

static void start_wifi_ap(void)
{
    esp_netif_t *netif = esp_netif_create_default_wifi_ap();
    if (!netif) {
        ESP_LOGE(TAG, "Failed to create default Wi-Fi AP netif");
        return;
    }
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t ap_cfg = {0};
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    snprintf((char *)ap_cfg.ap.ssid, sizeof(ap_cfg.ap.ssid),
             "ot-light-setup-%02X%02X", mac[4], mac[5]);
    ap_cfg.ap.ssid_len = strlen((char *)ap_cfg.ap.ssid);
    ap_cfg.ap.max_connection = 4;
    ap_cfg.ap.authmode = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "AP started: %s", ap_cfg.ap.ssid);
}

static bool should_start_portal(void)
{
    if (s_force_portal) {
        return true;
    }

    if (!config_store_is_configured()) {
        return true;
    }

#if ROLE_CONTROLLER
    int a = gpio_get_level(PIN_SW_A);
    int b = gpio_get_level(PIN_SW_B);
    return (a == 1 && b == 1);
#else
    return false;
#endif
}

void config_portal_start_if_needed(void)
{
    s_force_portal = detect_reset_sequence();
    if (!should_start_portal()) {
        return;
    }
    start_wifi_ap();
    start_http_server();
    s_running = (s_server != NULL);
}

bool config_portal_is_running(void)
{
    return s_running;
}
