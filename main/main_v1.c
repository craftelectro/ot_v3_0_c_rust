#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_vfs_eventfd.h"
#include "nvs_flash.h"

#include "esp_openthread.h"
#include "esp_openthread_cli.h"
#include "esp_openthread_netif_glue.h"
#include "esp_openthread_lock.h"
#include "esp_ot_config.h"

#include "openthread/instance.h"
#include "openthread/cli.h"
#include "openthread/logging.h"
#include "openthread/coap.h"
#include "openthread/dataset.h"
#include "openthread/dataset_ftd.h"
#include "openthread/thread.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_timer.h"

#include "config.h"
#include "rgb_led.h"

static const char *TAG = "warehouse";

// ===== состояния =====
// typedef enum { MODE_OFF=0, MODE_ON=1, MODE_AUTO=2 } light_mode_t;   // дублируем тип если в rgb_led.h нет
typedef struct {
    light_mode_t mode;
    bool         relay_on;
    uint16_t     dist_cm;
    int64_t      last_motion_us;
} state_t;
static state_t st;

// ===== пины / функции =====
static inline void relay_apply(bool on) {
    st.relay_on = on;
    gpio_set_level(PIN_RELAY, on ? 1 : 0);
}

static light_mode_t read_switch() {
#if ROLE_CONTROLLER
    int a = gpio_get_level(PIN_SW_A);
    int b = gpio_get_level(PIN_SW_B);
    if(a==0 && b==0) return MODE_OFF;
    if(a==0 && b==1) return MODE_ON;
    return MODE_AUTO;
#else
    return st.mode; // на исполнителе «тумблер» не используется
#endif
}

static void leds_set(light_mode_t m){ rgb_set_mode_color(m); }

// ===== TFmini =====
// static void tfmini_init(void){
//     uart_config_t cfg = {
//         .baud_rate = 115200,
//         .data_bits = UART_DATA_8_BITS,
//         .parity    = UART_PARITY_DISABLE,
//         .stop_bits = UART_STOP_BITS_1,
//         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
//     };
//     ESP_ERROR_CHECK(uart_driver_install(UART_PORT, 256, 0, 0, NULL, 0));
//     ESP_ERROR_CHECK(uart_param_config(UART_PORT, &cfg));
//     ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_PIN_NO_CHANGE, UART_TFMINI_RX,
//                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
// }

static void tfmini_init(void){
    uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    // UART1 для TFmini — отдельный драйвер, чтобы не мешать OT CLI на UART0
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, 512, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &cfg));
    // ВАЖНО: порядок — TX, RX, RTS, CTS
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT,
                                 UART_TFMINI_TX,  // -1 (не используем TX)
                                 UART_TFMINI_RX,  // ваш пин RX, например GPIO5
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));
}


static bool tfmini_poll_once(){
    uint8_t b[32];
    int n = uart_read_bytes(UART_PORT, b, sizeof(b), 10/portTICK_PERIOD_MS);
    if(n < 9) return false;
    for(int i=0;i<=n-9;i++){
        if(b[i]==0x59 && b[i+1]==0x59){
            uint16_t dist = b[i+2] | (b[i+3]<<8);
            uint8_t sum=0; for(int k=0;k<8;k++) sum+=b[i+k];
            if(sum == b[i+8]){ st.dist_cm = dist; return true; }
        }
    }
    return false;
}

// ===== OpenThread =====
static otInstance *s_ot = NULL;

// ===== CoAP ресурсы =====
static otCoapResource res_status, res_mode;

static const char* mode_str(light_mode_t m){
    return (m==MODE_OFF)?"off":(m==MODE_ON)?"on":"auto";
}

static void coap_send_text(otMessage *request, const otMessageInfo *info, const char *payload){
    otMessage *rsp = otCoapNewMessage(s_ot, NULL);
    otCoapMessageInitResponse(rsp, request, OT_COAP_TYPE_NON_CONFIRMABLE, OT_COAP_CODE_CONTENT);
    otCoapMessageSetPayloadMarker(rsp);
    otMessageAppend(rsp, payload, strlen(payload));
    otCoapSendResponse(s_ot, rsp, info);
}

static void on_status(void *aContext, otMessage *aMessage, const otMessageInfo *aInfo){
    char json[128];
    snprintf(json, sizeof(json),
        "{\"zone\":%d,\"mode\":\"%s\",\"relay\":%s,\"dist_cm\":%u}",
        ZONE_ID, mode_str(st.mode), st.relay_on?"true":"false", st.dist_cm);
    coap_send_text(aMessage, aInfo, json);
}

static void on_mode(void *aContext, otMessage *aMessage, const otMessageInfo *aInfo){
    char buf[32] = {0};
    uint16_t payload_len = otMessageGetLength(aMessage) - otMessageGetOffset(aMessage);
    if (payload_len > 0 && payload_len < sizeof(buf)) {
        otMessageRead(aMessage, otMessageGetOffset(aMessage), buf, payload_len);
        buf[payload_len] = '\0';
        ESP_LOGI(TAG, "CoAP /mode payload: '%s'", buf);

        if      (strstr(buf, "off"))  st.mode = MODE_OFF;
        else if (strstr(buf, "on"))   st.mode = MODE_ON;
        else                          st.mode = MODE_AUTO;

        leds_set(st.mode);
        ESP_LOGI(TAG, "Mode via CoAP -> %s", mode_str(st.mode));
    } else {
        ESP_LOGW(TAG, "CoAP /mode empty or too long payload (%u)", payload_len);
    }
    coap_send_text(aMessage, aInfo, mode_str(st.mode));
}

static void register_coap(void){
    esp_openthread_lock_acquire(portMAX_DELAY);
    s_ot = esp_openthread_get_instance();
    otCoapStart(s_ot, OT_DEFAULT_COAP_PORT);

    static char path_status[24], path_mode[24];
    snprintf(path_status,sizeof(path_status), "zone/%d/status", ZONE_ID);
    snprintf(path_mode,  sizeof(path_mode),   "zone/%d/mode",   ZONE_ID);

    res_status.mUriPath = path_status;
    res_status.mHandler = on_status;

    res_mode.mUriPath   = path_mode;
    res_mode.mHandler   = on_mode;

    otCoapAddResource(s_ot, &res_status);
    otCoapAddResource(s_ot, &res_mode);
    esp_openthread_lock_release();
    ESP_LOGI(TAG, "CoAP resources: /%s, /%s", path_status, path_mode);
}

// ===== Dataset (если пуст) =====
static void ensure_active_dataset(void){
    esp_openthread_lock_acquire(portMAX_DELAY);
    otInstance *ot = esp_openthread_get_instance();
    otOperationalDatasetTlvs tlv;
    otError e = otDatasetGetActiveTlvs(ot, &tlv);
    if(e == OT_ERROR_NONE){
        ESP_LOGI(TAG, "Active Dataset already present");
        esp_openthread_lock_release();
        return;
    }

    otOperationalDataset d; memset(&d, 0, sizeof(d));
    d.mActiveTimestamp = (otTimestamp){.mSeconds=1};
    d.mChannel = OT_CHANNEL;
    d.mPanId   = OT_PANID;
    static const uint8_t xpanid[8] = OT_EXT_PANID;
    static const uint8_t nkey[16]  = OT_NETWORK_KEY;
    memcpy(d.mExtendedPanId.m8, xpanid, sizeof(xpanid));
    memcpy(d.mNetworkKey.m8,     nkey,  sizeof(nkey));
    strcpy((char*)d.mNetworkName.m8, OT_NETWORK_NAME);

    d.mComponents.mIsActiveTimestampPresent = true;
    d.mComponents.mIsChannelPresent         = true;
    d.mComponents.mIsPanIdPresent           = true;
    d.mComponents.mIsExtendedPanIdPresent   = true;
    d.mComponents.mIsNetworkKeyPresent      = true;
    d.mComponents.mIsNetworkNamePresent     = true;

    otError oe = otDatasetSetActive(ot, &d);
    ESP_LOGI(TAG, "otDatasetSetActive -> %d", oe);
    ESP_ERROR_CHECK(oe == OT_ERROR_NONE ? ESP_OK : ESP_FAIL);
    esp_openthread_lock_release();
    ESP_LOGI(TAG, "Active Dataset written to NVS");
}

// ===== автоматика =====
static void io_init(void){
    gpio_config_t out = { .pin_bit_mask = (1ULL<<PIN_RELAY), .mode=GPIO_MODE_OUTPUT };
    gpio_config(&out);
#if ROLE_CONTROLLER
    gpio_config_t inp = { .pin_bit_mask = (1ULL<<PIN_SW_A)|(1ULL<<PIN_SW_B),
                          .mode=GPIO_MODE_INPUT, .pull_up_en=1 };
    gpio_config(&inp);
#endif
}

static void logic_task(void *arg){
#if ROLE_CONTROLLER
    st.mode = read_switch();
#else
    st.mode = MODE_AUTO;  // исполнитель стартует в AUTO, пока не придёт CoAP
#endif
    leds_set(st.mode);
    relay_apply(false);
    int64_t deadline = 0;

    for(;;){
#if ROLE_CONTROLLER
        light_mode_t m = read_switch();
        if(m != st.mode){ st.mode = m; leds_set(st.mode); }
#endif
        if(tfmini_poll_once()){
            if(st.mode==MODE_AUTO && st.dist_cm>0 && st.dist_cm<=TFMINI_DISTANCE_CM_MAX){
                relay_apply(true);
                st.last_motion_us = esp_timer_get_time();
                deadline = st.last_motion_us + (int64_t)AUTO_HOLD_MS*1000;
            }
        }

        if(st.mode==MODE_OFF)  relay_apply(false);
        if(st.mode==MODE_ON)   relay_apply(true);
        if(st.mode==MODE_AUTO && st.relay_on && deadline && esp_timer_get_time()>deadline){
            relay_apply(false);
            deadline = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ===== ot_cli «каркас» =====
static esp_netif_t *init_openthread_netif(const esp_openthread_platform_config_t *config)
{
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_OPENTHREAD();
    esp_netif_t *netif = esp_netif_new(&cfg);
    ESP_ERROR_CHECK(esp_netif_attach(netif, esp_openthread_netif_glue_init(config)));
    return netif;
}

// static void ot_task_worker(void *ctx)
// {
//     esp_openthread_platform_config_t cfg = {
//         .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
//         .host_config  = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
//         .port_config  = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
//     };

//     ESP_ERROR_CHECK(esp_openthread_init(&cfg));
// #if CONFIG_OPENTHREAD_LOG_LEVEL_DYNAMIC
//     (void)otLoggingSetLevel(CONFIG_LOG_DEFAULT_LEVEL);
// #endif

// #if CONFIG_OPENTHREAD_CLI
//     esp_openthread_cli_init();
// #endif

//     esp_netif_t *ot_netif = init_openthread_netif(&cfg);
//     esp_netif_set_default_netif(ot_netif);

//     ensure_active_dataset();

// #if CONFIG_OPENTHREAD_CLI
//     esp_openthread_cli_create_task();
// #endif

//     // ЯВНО стартуем Thread (без AUTO_START)
//     esp_openthread_lock_acquire(portMAX_DELAY);
//     otInstance *ot = esp_openthread_get_instance();
//     otError e = otThreadSetEnabled(ot, true);
//     esp_openthread_lock_release();
//     ESP_LOGI(TAG, "Thread explicitly started (err=%d)", e);

//     register_coap();
//     xTaskCreate(logic_task, "logic", 4096, NULL, 5, NULL);

//     esp_openthread_launch_mainloop();

//     // обычно сюда не доходит
//     esp_openthread_netif_glue_deinit();
//     esp_netif_destroy(ot_netif);
//     vTaskDelete(NULL);
// }

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
    esp_openthread_cli_init();   // CLI на UART0
#endif

    esp_netif_t *ot_netif = init_openthread_netif(&cfg);
    esp_netif_set_default_netif(ot_netif);

    // 1) Dataset, если пуст
    ensure_active_dataset();

    // 2) Включаем IPv6 стек и сам Thread (раньше мы включали только Thread)
    esp_openthread_lock_acquire(portMAX_DELAY);
    otInstance *ot = esp_openthread_get_instance();

    otError e = otIp6SetEnabled(ot, true);     // <-- ВКЛ IPv6
    if (e != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "otIp6SetEnabled(true) -> %d", e);
    }

    e = otThreadSetEnabled(ot, true);          // <-- Стартуем Thread
    if (e != OT_ERROR_NONE) {
        ESP_LOGE(TAG, "otThreadSetEnabled(true) -> %d", e);
    } else {
        ESP_LOGI(TAG, "Thread explicitly started");
    }
    esp_openthread_lock_release();

#if CONFIG_OPENTHREAD_CLI
    esp_openthread_cli_create_task();
#endif

    // 3) Ждём, пока роль станет не disabled (необязательно, но удобно для логики ниже)
    for (int i = 0; i < 100; ++i) { // ~5 секунд
        esp_openthread_lock_acquire(portMAX_DELAY);
        otDeviceRole role = otThreadGetDeviceRole(ot);
        esp_openthread_lock_release();
        if (role != OT_DEVICE_ROLE_DISABLED) break;
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    // 4) Поднимаем наши сервисы и автоматику
    register_coap();
    xTaskCreate(logic_task, "logic", 4096, NULL, 5, NULL);

    // 5) Основной цикл OT
    esp_openthread_launch_mainloop();

    // cleanup (обычно недостижимо)
    esp_openthread_netif_glue_deinit();
    esp_netif_destroy(ot_netif);
    vTaskDelete(NULL);
}


void app_main(void)
{
    esp_vfs_eventfd_config_t ev = {.max_fds = 3};

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_vfs_eventfd_register(&ev));

    ESP_ERROR_CHECK(rgb_init());
    io_init();
    tfmini_init();

    xTaskCreate(ot_task_worker, "ot_cli_main", 10240, NULL, 5, NULL);
}


// #include <string.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_event.h"
// #include "esp_log.h"
// #include "esp_netif.h"
// #include "esp_vfs_eventfd.h"
// #include "nvs_flash.h"

// #include "esp_openthread.h"
// #include "esp_openthread_cli.h"
// #include "esp_openthread_netif_glue.h"
// #include "esp_openthread_lock.h"
// #include "esp_ot_config.h"

// #include "openthread/instance.h"
// #include "openthread/cli.h"
// #include "openthread/logging.h"
// #include "openthread/coap.h"
// #include "openthread/dataset.h"
// #include "openthread/dataset_ftd.h"
// #include "openthread/thread.h"

// #include "driver/gpio.h"
// #include "driver/uart.h"
// #include "esp_timer.h"

// #include "config.h"
// #include "rgb_led.h"

// static volatile bool g_boot_demo_done = false;


// static const char *TAG = "warehouse";

// // ====== состояние узла ======
// typedef struct {
//     light_mode_t mode;
//     bool         relay_on;
//     uint16_t     dist_cm;
//     int64_t      last_motion_us;
// } state_t;
// static state_t st;

// // ====== OpenThread instance ======
// static otInstance *s_ot = NULL;

// // ---------------- Relay / Switch / LED ----------------
// static inline void relay_apply(bool on) {
//     st.relay_on = on;
//     gpio_set_level(PIN_RELAY, on ? 1 : 0);
// }
// static light_mode_t read_switch(void) {
//     int a = gpio_get_level(PIN_SW_A);
//     int b = gpio_get_level(PIN_SW_B);
//     if (a==0 && b==0) return MODE_OFF;
//     if (a==0 && b==1) return MODE_ON;
//     return MODE_AUTO;
// }
// static void leds_set(light_mode_t m) { rgb_set_mode_color(m); }

// // ---------------- TFmini ----------------
// static void tfmini_init(void){
//     uart_config_t cfg = {
//         .baud_rate = 115200,
//         .data_bits = UART_DATA_8_BITS,
//         .parity    = UART_PARITY_DISABLE,
//         .stop_bits = UART_STOP_BITS_1,
//         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
//     };
//     ESP_ERROR_CHECK(uart_driver_install(UART_PORT, 256, 0, 0, NULL, 0));
//     ESP_ERROR_CHECK(uart_param_config(UART_PORT, &cfg));
//     ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_PIN_NO_CHANGE, UART_TFMINI_RX,
//                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
// }
// static bool tfmini_poll_once(){
//     uint8_t b[32];
//     int n = uart_read_bytes(UART_PORT, b, sizeof(b), 10/portTICK_PERIOD_MS);
//     if(n < 9) return false;
//     for(int i=0;i<=n-9;i++){
//         if(b[i]==0x59 && b[i+1]==0x59){
//             uint16_t dist = b[i+2] | (b[i+3]<<8);
//             uint8_t sum=0; for(int k=0;k<8;k++) sum+=b[i+k];
//             if(sum == b[i+8]){ st.dist_cm = dist; return true; }
//         }
//     }
//     return false;
// }

// // ---------------- CoAP ----------------
// static otCoapResource res_status, res_mode;

// static const char* mode_str(light_mode_t m){
//     return (m==MODE_OFF)?"off":(m==MODE_ON)?"on":"auto";
// }

// // корректный ответ на входящий запрос (ACK/CONTENT), чтобы CLI показывал payload
// static void coap_send_text(otMessage *request, const otMessageInfo *info, const char *payload){
//     otMessage *resp = otCoapNewMessage(s_ot, NULL);
//     otCoapMessageInitResponse(resp, request, OT_COAP_TYPE_ACKNOWLEDGMENT, OT_COAP_CODE_CONTENT);
//     otCoapMessageSetPayloadMarker(resp);
//     otMessageAppend(resp, payload, strlen(payload));
//     otCoapSendResponse(s_ot, resp, info);
// }

// static void on_status(void *aContext, otMessage *aMessage, const otMessageInfo *aInfo){
//     char json[128];
//     snprintf(json, sizeof(json),
//         "{\"zone\":%d,\"mode\":\"%s\",\"relay\":%s,\"dist_cm\":%u}",
//         ZONE_ID, mode_str(st.mode), st.relay_on?"true":"false", st.dist_cm);
//     coap_send_text(aMessage, aInfo, json);
// }

// // static void on_mode(void *aContext, otMessage *aMessage, const otMessageInfo *aInfo){
// //     char buf[16]={0};
// //     int len = otMessageRead(aMessage, otMessageGetOffset(aMessage), buf, sizeof(buf)-1);
// //     if(len>0){
// //         if      (strstr(buf,"off"))  st.mode = MODE_OFF;
// //         else if (strstr(buf,"on"))   st.mode = MODE_ON;
// //         else                         st.mode = MODE_AUTO;
// //         leds_set(st.mode);
// //         ESP_LOGI(TAG, "Mode via CoAP -> %s", mode_str(st.mode));
// //     }
// //     coap_send_text(aMessage, aInfo, mode_str(st.mode));
// // }


// static void on_mode(void *aContext, otMessage *aMessage, const otMessageInfo *aInfo)
// {
//     OT_UNUSED_VARIABLE(aContext);
//     char buf[32] = {0};

//     // Получаем payload через API CoAP, а не через otMessageGetOffset
//     otCoapCode code = otCoapMessageGetCode(aMessage);
//     if (code == OT_COAP_CODE_PUT || code == OT_COAP_CODE_POST) {
//         uint16_t payloadLength = otMessageGetLength(aMessage) - otMessageGetOffset(aMessage);
//         if (payloadLength > 0 && payloadLength < sizeof(buf)) {
//             otMessageRead(aMessage, otMessageGetOffset(aMessage), buf, payloadLength);
//             buf[payloadLength] = '\0';
//             ESP_LOGI(TAG, "CoAP /mode payload: '%s'", buf);

//             if      (strstr(buf, "off"))  st.mode = MODE_OFF;
//             else if (strstr(buf, "on"))   st.mode = MODE_ON;
//             else                          st.mode = MODE_AUTO;

//             leds_set(st.mode);
//             ESP_LOGI(TAG, "Mode via CoAP -> %s", mode_str(st.mode));
//         } else {
//             ESP_LOGW(TAG, "CoAP /mode empty or too long payload (%u)", payloadLength);
//         }
//     }

//     // отправляем подтверждение
//     coap_send_text(aMessage, aInfo, mode_str(st.mode));
// }



// static void register_coap(void){
//     esp_openthread_lock_acquire(portMAX_DELAY);
//     s_ot = esp_openthread_get_instance();
//     otCoapStart(s_ot, OT_DEFAULT_COAP_PORT);

//     static char path_status[24], path_mode[24];
//     snprintf(path_status,sizeof(path_status), "zone/%d/status", ZONE_ID);
//     snprintf(path_mode,  sizeof(path_mode),   "zone/%d/mode",   ZONE_ID);

//     res_status.mUriPath = path_status;
//     res_status.mHandler = on_status;

//     res_mode.mUriPath   = path_mode;
//     res_mode.mHandler   = on_mode;

//     otCoapAddResource(s_ot, &res_status);
//     otCoapAddResource(s_ot, &res_mode);
//     esp_openthread_lock_release();
//     ESP_LOGI(TAG, "CoAP resources: /%s, /%s", path_status, path_mode);
// }

// // ---------------- Dataset ----------------
// static void ensure_active_dataset(void){
//     esp_openthread_lock_acquire(portMAX_DELAY);
//     otInstance *ot = esp_openthread_get_instance();
//     otOperationalDatasetTlvs tlv;
//     otError e = otDatasetGetActiveTlvs(ot, &tlv);
//     if(e == OT_ERROR_NONE){
//         ESP_LOGI(TAG, "Active Dataset already present");
//         esp_openthread_lock_release();
//         return;
//     }

//     otOperationalDataset d; memset(&d, 0, sizeof(d));
//     d.mActiveTimestamp = (otTimestamp){.mSeconds=1};
//     d.mChannel = OT_CHANNEL;
//     d.mPanId   = OT_PANID;
//     memcpy(d.mExtendedPanId.m8, OT_EXT_PANID, 8);
//     memcpy(d.mNetworkKey.m8, OT_NETWORK_KEY, 16);
//     strcpy((char*)d.mNetworkName.m8, OT_NETWORK_NAME);

//     d.mComponents.mIsActiveTimestampPresent = true;
//     d.mComponents.mIsChannelPresent = true;
//     d.mComponents.mIsPanIdPresent = true;
//     d.mComponents.mIsExtendedPanIdPresent = true;
//     d.mComponents.mIsNetworkKeyPresent = true;
//     d.mComponents.mIsNetworkNamePresent = true;

//     ESP_ERROR_CHECK(otDatasetSetActive(ot, &d) == OT_ERROR_NONE ? ESP_OK : ESP_FAIL);
//     esp_openthread_lock_release();
//     ESP_LOGI(TAG, "Active Dataset written to NVS");
// }

// // ---------------- автоматика ----------------
// static void io_init(void){
//     gpio_config_t out = { .pin_bit_mask = (1ULL<<PIN_RELAY), .mode=GPIO_MODE_OUTPUT };
//     gpio_config(&out);
//     gpio_config_t inp = { .pin_bit_mask = (1ULL<<PIN_SW_A)|(1ULL<<PIN_SW_B),
//                           .mode=GPIO_MODE_INPUT, .pull_up_en=1 };
//     gpio_config(&inp);
// }

// // static void logic_task(void *arg){
// //     st.mode = read_switch();
// //     leds_set(st.mode);
// //     relay_apply(false);
// //     int64_t deadline = 0;

// //     for(;;){
// //         light_mode_t m = read_switch();
// //         if(m != st.mode){ st.mode = m; leds_set(st.mode); }

// //         if(tfmini_poll_once()){
// //             if(st.mode==MODE_AUTO && st.dist_cm>0 && st.dist_cm<=TFMINI_DISTANCE_CM_MAX){
// //                 relay_apply(true);
// //                 st.last_motion_us = esp_timer_get_time();
// //                 deadline = st.last_motion_us + (int64_t)AUTO_HOLD_MS*1000;
// //             }
// //         }

// //         if(st.mode==MODE_OFF)  relay_apply(false);
// //         if(st.mode==MODE_ON)   relay_apply(true);
// //         if(st.mode==MODE_AUTO && st.relay_on && deadline && esp_timer_get_time()>deadline){
// //             relay_apply(false);
// //             deadline = 0;
// //         }
// //         vTaskDelay(pdMS_TO_TICKS(20));
// //     }
// // }


// static void logic_task(void *arg)
// {
//     // дождаться завершения стартового RGB-демо
//     while (!g_boot_demo_done) {
//         vTaskDelay(pdMS_TO_TICKS(50));
//     }

//     // начальное состояние
//     light_mode_t sw = read_switch();
//     st.mode = sw;
//     leds_set(st.mode);
//     relay_apply(false);
//     int64_t deadline = 0;

//     // простая антидребезговая логика тумблера
//     const int DEBOUNCE_MS = 50;
//     light_mode_t last_sw = sw;
//     int64_t      last_sw_change = esp_timer_get_time();

//     for (;;) {
//         // --- чтение тумблера + дебаунс ---
//         light_mode_t cur_sw = read_switch();
//         if (cur_sw != last_sw) {
//             last_sw = cur_sw;
//             last_sw_change = esp_timer_get_time();
//         }
//         if (st.mode != cur_sw &&
//             (esp_timer_get_time() - last_sw_change) >= (int64_t)DEBOUNCE_MS * 1000) {
//             st.mode = cur_sw;
//             leds_set(st.mode);
//         }

//         // --- обработка TFmini ---
//         if (tfmini_poll_once()) {
//             if (st.mode == MODE_AUTO &&
//                 st.dist_cm > 0 && st.dist_cm <= TFMINI_DISTANCE_CM_MAX) {
//                 relay_apply(true);
//                 st.last_motion_us = esp_timer_get_time();
//                 deadline = st.last_motion_us + (int64_t)AUTO_HOLD_MS * 1000;
//             }
//         }

//         // --- применение режима к реле ---
//         if (st.mode == MODE_OFF) {
//             relay_apply(false);
//         } else if (st.mode == MODE_ON) {
//             relay_apply(true);
//         } else { // MODE_AUTO
//             if (st.relay_on && deadline && esp_timer_get_time() > deadline) {
//                 relay_apply(false);
//                 deadline = 0;
//             }
//         }

//         vTaskDelay(pdMS_TO_TICKS(20));
//     }
// }


// // ---------------- ot_cli каркас + запуск Thread ----------------
// static esp_netif_t *init_openthread_netif(const esp_openthread_platform_config_t *config)
// {
//     esp_netif_config_t cfg = ESP_NETIF_DEFAULT_OPENTHREAD();
//     esp_netif_t *netif = esp_netif_new(&cfg);
//     ESP_ERROR_CHECK(esp_netif_attach(netif, esp_openthread_netif_glue_init(config)));
//     return netif;
// }

// static void ot_task_worker(void *ctx)
// {
//     esp_openthread_platform_config_t cfg = {
//         .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
//         .host_config  = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
//         .port_config  = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
//     };

//     ESP_ERROR_CHECK(esp_openthread_init(&cfg));

// #if CONFIG_OPENTHREAD_LOG_LEVEL_DYNAMIC
//     (void)otLoggingSetLevel(CONFIG_LOG_DEFAULT_LEVEL);
// #endif

// #if CONFIG_OPENTHREAD_CLI
//     esp_openthread_cli_init();   // CLI backend выбирается в menuconfig (UART/Console)
// #endif

//     esp_netif_t *ot_netif = init_openthread_netif(&cfg);
//     esp_netif_set_default_netif(ot_netif);

//     ensure_active_dataset();

// #if CONFIG_OPENTHREAD_CLI
//     esp_openthread_cli_create_task();
// #endif

// #if CONFIG_OPENTHREAD_AUTO_START
//     otOperationalDatasetTlvs ds;
//     otError err = otDatasetGetActiveTlvs(esp_openthread_get_instance(), &ds);
//     ESP_ERROR_CHECK(esp_openthread_auto_start((err==OT_ERROR_NONE)? &ds : NULL));
// #else
//     // явный старт, если AUTO_START выключен
//     esp_openthread_lock_acquire(portMAX_DELAY);
//     otInstance *ot = esp_openthread_get_instance();
//     otIp6SetEnabled(ot, true);
//     otThreadSetEnabled(ot, true);
//     esp_openthread_lock_release();
//     ESP_LOGI(TAG, "Thread explicitly started (no AUTO_START)");
// #endif

//     register_coap();
//     xTaskCreate(logic_task, "logic", 4096, NULL, 5, NULL);

//     esp_openthread_launch_mainloop();

//     // (обычно сюда не доходит)
//     esp_openthread_netif_glue_deinit();
//     esp_netif_destroy(ot_netif);
//     vTaskDelete(NULL);
// }

// // void app_main(void)
// // {
// //     esp_vfs_eventfd_config_t ev = {.max_fds = 3};

// //     ESP_ERROR_CHECK(nvs_flash_init());
// //     ESP_ERROR_CHECK(esp_event_loop_create_default());
// //     ESP_ERROR_CHECK(esp_netif_init());
// //     ESP_ERROR_CHECK(esp_vfs_eventfd_register(&ev));

// //     ESP_ERROR_CHECK(rgb_init());

// //     io_init();
// //     tfmini_init();

// //     xTaskCreate(ot_task_worker, "ot_cli_main", 10240, NULL, 5, NULL);
// // }


// void app_main(void)
// {
//     esp_vfs_eventfd_config_t ev = {.max_fds = 3};

//     ESP_ERROR_CHECK(nvs_flash_init());
//     ESP_ERROR_CHECK(esp_event_loop_create_default());
//     ESP_ERROR_CHECK(esp_netif_init());
//     ESP_ERROR_CHECK(esp_vfs_eventfd_register(&ev));

//     // RGB и короткое демо (чтобы глазами увидеть цвета)
//     ESP_ERROR_CHECK(rgb_init());
//     rgb_set_rgb(255, 0,   0); vTaskDelay(pdMS_TO_TICKS(600));
//     rgb_set_rgb(0,   255, 0); vTaskDelay(pdMS_TO_TICKS(600));
//     rgb_set_rgb(0,   0, 255); vTaskDelay(pdMS_TO_TICKS(600));
//     rgb_set_rgb(0,   0,   0); vTaskDelay(pdMS_TO_TICKS(200));
//     g_boot_demo_done = true;  // <- сообщаем логике, что демо закончено

//     // остальное железо
//     io_init();
//     tfmini_init();

//     // старт OpenThread/CLI
//     xTaskCreate(ot_task_worker, "ot_cli_main", 10240, NULL, 5, NULL);
// }
