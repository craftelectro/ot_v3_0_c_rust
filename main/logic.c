#include "logic.h"
#include "config.h"
#include "io_board.h"
#include "rgb_led.h"
#include "tfmini.h"
#include "coap_if.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "nvs.h"

#include <string.h>

#include "esp_system.h"   // esp_reset_reason()

#include "freertos/queue.h"


static int64_t s_last_local_trigger_us = 0;
static const char *TAG = "logic";
static zone_state_t s_state;

static bool s_global_mode_valid = false;
static light_mode_t s_global_mode = MODE_AUTO;

static bool s_zone_mode_valid = false;
static uint8_t s_zone_mode_zone = 0;
static light_mode_t s_zone_mode = MODE_AUTO;

static bool s_node_mode_valid = false;
static light_mode_t s_node_mode = MODE_AUTO;




typedef enum {
    EVT_STATE_RSP,
    EVT_TRIGGER_RX,
    EVT_OFF_RX,

    EVT_MODE_SET_GLOBAL,
    EVT_MODE_SET_ZONE,
    EVT_MODE_SET_NODE,
    EVT_MODE_CLR_GLOBAL,
    EVT_MODE_CLR_ZONE,
    EVT_MODE_CLR_NODE,

} logic_evt_type_t;

typedef struct {
    logic_evt_type_t type;
    uint32_t epoch;
    otIp6Address addr;
    uint32_t u32;
    bool b;
} logic_evt_t;

static QueueHandle_t s_logic_q;


// ===== NVS keys =====
#define NVS_NS           "app"
#define NVS_K_MODE       "mode"
#define NVS_K_EPOCH      "epoch"
#define NVS_K_ACTIVE     "active"
#define NVS_K_DEADLINE   "deadline_us"
#define NVS_K_OWNER_OK   "owner_ok"
#define NVS_K_OWNER_ADDR "owner_addr"

#define RESTORE_WAIT_MS  1200  // ждать state_rsp после ребута (strict)

// ===== NVS keys for MODE overrides (persistent) =====
#define NVS_K_GMODE_VALID  "g_valid"
#define NVS_K_GMODE        "g_mode"

#define NVS_K_ZMODE_VALID  "z_valid"
#define NVS_K_ZMODE_ZONE   "z_zone"
#define NVS_K_ZMODE        "z_mode"

#define NVS_K_NMODE_VALID  "n_valid"
#define NVS_K_NMODE        "n_mode"


static void set_relay(bool on)
{
    io_board_set_relay(on);
    s_state.relay_on = on;
}

static void clear_active(void)
{
    s_state.active = false;
    s_state.deadline_us = 0;
    s_state.owner_valid = false;
    memset(&s_state.owner_addr, 0, sizeof(s_state.owner_addr));
    s_state.pending_restore = false;
}

static bool addr_eq(const otIp6Address *a, const otIp6Address *b)
{
    return memcmp(a->mFields.m8, b->mFields.m8, 16) == 0;
}

bool logic_is_owner(void)
{
    if (!s_state.owner_valid) return false;
    otIp6Address me;
    if (!coap_if_get_my_meshlocal_eid(&me)) return false;
    return addr_eq(&me, &s_state.owner_addr);
}

static void nvs_save_all(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;

    nvs_set_u8(h, NVS_K_MODE, (uint8_t)s_state.mode);
    nvs_set_u32(h, NVS_K_EPOCH, (uint32_t)s_state.epoch);
    nvs_set_u8(h, NVS_K_ACTIVE, (uint8_t)(s_state.active ? 1 : 0));
    nvs_set_i64(h, NVS_K_DEADLINE, (int64_t)s_state.deadline_us);
    nvs_set_u8(h, NVS_K_OWNER_OK, (uint8_t)(s_state.owner_valid ? 1 : 0));
    if (s_state.owner_valid) {
        nvs_set_blob(h, NVS_K_OWNER_ADDR, s_state.owner_addr.mFields.m8, 16);
    } else {
        uint8_t z[16] = {0};
        nvs_set_blob(h, NVS_K_OWNER_ADDR, z, 16);
    }

        // --- persist overrides ---
    nvs_set_u8(h, NVS_K_GMODE_VALID, (uint8_t)(s_global_mode_valid ? 1 : 0));
    nvs_set_u8(h, NVS_K_GMODE,       (uint8_t)s_global_mode);

    nvs_set_u8(h, NVS_K_ZMODE_VALID, (uint8_t)(s_zone_mode_valid ? 1 : 0));
    nvs_set_u8(h, NVS_K_ZMODE_ZONE,  (uint8_t)s_zone_mode_zone);
    nvs_set_u8(h, NVS_K_ZMODE,       (uint8_t)s_zone_mode);

    nvs_set_u8(h, NVS_K_NMODE_VALID, (uint8_t)(s_node_mode_valid ? 1 : 0));
    nvs_set_u8(h, NVS_K_NMODE,       (uint8_t)s_node_mode);


    nvs_commit(h);
    nvs_close(h);
}

static light_mode_t effective_mode(void)
{
#if ROLE_CONTROLLER
    return io_board_read_mode_switch();   // контроллер главный
#else
    if (s_node_mode_valid) return s_node_mode;
    if (s_zone_mode_valid && s_zone_mode_zone == ZONE_ID) return s_zone_mode;
    if (s_global_mode_valid) return s_global_mode;
    return s_state.mode;                  // локальный режим (NVS/CLI)
#endif
}


// static void apply_effective_mode(light_mode_t m)
// {
//     if (m == MODE_OFF) {
//         clear_active();
//         set_relay(false);
//         return;
//     }
//     if (m == MODE_ON) {
//         clear_active();
//         set_relay(true);
//         return;
//     }
//     // MODE_AUTO: ничего сразу не включаем; дальше работает active/deadline
// }

static void apply_effective_mode(light_mode_t m)
{
    if (m == MODE_OFF || m == MODE_ON) {
        // OFF/ON — это “ручные” режимы, зона/таймеры не должны жить
        clear_active();
        // реле не трогаем тут!
        return;
    }
    // MODE_AUTO: ничего не делаем
}



void logic_post_mode_cmd_global(light_mode_t mode)
{
    if (!s_logic_q) return;
    logic_evt_t e = { .type = EVT_MODE_SET_GLOBAL, .u32 = (uint32_t)mode };
    xQueueSend(s_logic_q, &e, 0);
}

void logic_post_mode_cmd_zone(uint8_t zone_id, light_mode_t mode)
{
    if (!s_logic_q) return;
    logic_evt_t e = { .type = EVT_MODE_SET_ZONE, .u32 = ((uint32_t)zone_id << 8) | (uint32_t)mode };
    xQueueSend(s_logic_q, &e, 0);
}

void logic_post_mode_cmd_node(light_mode_t mode)
{
    if (!s_logic_q) return;
    logic_evt_t e = { .type = EVT_MODE_SET_NODE, .u32 = (uint32_t)mode };
    xQueueSend(s_logic_q, &e, 0);
}

void logic_post_mode_clear_global(void)
{
    if (!s_logic_q) return;
    logic_evt_t e = { .type = EVT_MODE_CLR_GLOBAL };
    xQueueSend(s_logic_q, &e, 0);
}

void logic_post_mode_clear_zone(uint8_t zone_id)
{
    if (!s_logic_q) return;
    logic_evt_t e = { .type = EVT_MODE_CLR_ZONE, .u32 = (uint32_t)zone_id };
    xQueueSend(s_logic_q, &e, 0);
}

void logic_post_mode_clear_node(void)
{
    if (!s_logic_q) return;
    logic_evt_t e = { .type = EVT_MODE_CLR_NODE };
    xQueueSend(s_logic_q, &e, 0);
}



// static void nvs_load_all(light_mode_t def_mode)
// {
//     nvs_handle_t h;
//     if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
//         // defaults
//         s_state.mode = def_mode;
//         s_state.epoch = 0;
//         clear_active();
//         ESP_LOGI(TAG, "NVS empty -> defaults");
//         return;
//     }

//     uint8_t mode = (uint8_t)def_mode;
//     uint32_t epoch = 0;
//     uint8_t active = 0;
//     int64_t deadline_us = 0;
//     uint8_t owner_ok = 0;
//     uint8_t owner[16] = {0};
//     size_t sz = sizeof(owner);

//     (void)nvs_get_u8(h, NVS_K_MODE, &mode);
//     (void)nvs_get_u32(h, NVS_K_EPOCH, &epoch);
//     (void)nvs_get_u8(h, NVS_K_ACTIVE, &active);
//     (void)nvs_get_i64(h, NVS_K_DEADLINE, &deadline_us);
//     (void)nvs_get_u8(h, NVS_K_OWNER_OK, &owner_ok);
//     (void)nvs_get_blob(h, NVS_K_OWNER_ADDR, owner, &sz);

//     nvs_close(h);

//     if (mode > MODE_AUTO) mode = (uint8_t)def_mode;

//     s_state.mode = (light_mode_t)mode;
//     s_state.epoch = epoch;
//     s_state.active = (active != 0);
//     s_state.deadline_us = deadline_us;
//     s_state.owner_valid = (owner_ok != 0);
//     memcpy(s_state.owner_addr.mFields.m8, owner, 16);
//     s_state.pending_restore = false;

//     ESP_LOGI(TAG, "NVS: mode=%u active=%u deadline_us=%lld owner_ok=%u epoch=%lu",
//              (unsigned)mode,
//              (unsigned)active,
//              (long long)deadline_us,
//              (unsigned)owner_ok,
//              (unsigned long)epoch);
// }

static void nvs_load_all(light_mode_t def_mode)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        // defaults
        s_state.mode = def_mode;
        s_state.epoch = 0;
        clear_active();
        ESP_LOGI(TAG, "NVS empty -> defaults");
        return;
    }

    uint8_t mode = (uint8_t)def_mode;
    uint32_t epoch = 0;
    uint8_t active = 0;
    int64_t deadline_us = 0;
    uint8_t owner_ok = 0;
    uint8_t owner[16] = {0};
    size_t sz = sizeof(owner);

    uint8_t g_valid = 0, g_mode = (uint8_t)MODE_AUTO;
    uint8_t z_valid = 0, z_zone = 0, z_mode = (uint8_t)MODE_AUTO;
    uint8_t n_valid = 0, n_mode = (uint8_t)MODE_AUTO;


    (void)nvs_get_u8(h, NVS_K_MODE, &mode);
    (void)nvs_get_u32(h, NVS_K_EPOCH, &epoch);
    (void)nvs_get_u8(h, NVS_K_ACTIVE, &active);
    (void)nvs_get_i64(h, NVS_K_DEADLINE, &deadline_us);
    (void)nvs_get_u8(h, NVS_K_OWNER_OK, &owner_ok);
    (void)nvs_get_blob(h, NVS_K_OWNER_ADDR, owner, &sz);

    (void)nvs_get_u8(h, NVS_K_GMODE_VALID, &g_valid);
    (void)nvs_get_u8(h, NVS_K_GMODE, &g_mode);

    (void)nvs_get_u8(h, NVS_K_ZMODE_VALID, &z_valid);
    (void)nvs_get_u8(h, NVS_K_ZMODE_ZONE,  &z_zone);
    (void)nvs_get_u8(h, NVS_K_ZMODE, &z_mode);

    (void)nvs_get_u8(h, NVS_K_NMODE_VALID, &n_valid);
    (void)nvs_get_u8(h, NVS_K_NMODE, &n_mode);


    nvs_close(h);

    // apply loaded overrides (with sanity)
    s_global_mode_valid = (g_valid != 0);
    s_global_mode = (light_mode_t)g_mode;
    if (s_global_mode > MODE_AUTO) { s_global_mode = MODE_AUTO; s_global_mode_valid = false; }

    s_zone_mode_valid = (z_valid != 0);
    s_zone_mode_zone  = z_zone;
    s_zone_mode = (light_mode_t)z_mode;
    if (s_zone_mode > MODE_AUTO) { s_zone_mode = MODE_AUTO; s_zone_mode_valid = false; }

    s_node_mode_valid = (n_valid != 0);
    s_node_mode = (light_mode_t)n_mode;
    if (s_node_mode > MODE_AUTO) { s_node_mode = MODE_AUTO; s_node_mode_valid = false; }


    if (mode > MODE_AUTO) mode = (uint8_t)def_mode;

    s_state.mode = (light_mode_t)mode;
    s_state.epoch = epoch;
    s_state.active = (active != 0);
    s_state.deadline_us = deadline_us;
    s_state.owner_valid = (owner_ok != 0);
    memcpy(s_state.owner_addr.mFields.m8, owner, 16);
    s_state.pending_restore = false;

    ESP_LOGI(TAG, "NVS: mode=%u active=%u deadline_us=%lld owner_ok=%u epoch=%lu",
             (unsigned)mode,
             (unsigned)active,
             (long long)deadline_us,
             (unsigned)owner_ok,
             (unsigned long)epoch);

    // <<< ВОТ ЭТО ДОБАВЬ >>>
    esp_reset_reason_t rr = esp_reset_reason();
    if (rr == ESP_RST_POWERON || rr == ESP_RST_BROWNOUT) {
        ESP_LOGW(TAG, "cold boot (%d) -> ignore stored active/owner/timer", (int)rr);

        // Сбрасываем "воскрешаемое" состояние
        clear_active();                 // active=false, relay off, deadline=0 и т.п. (как у тебя реализовано)
        s_state.owner_valid = false;
        s_state.epoch = 0;
        s_state.pending_restore = false;

        // ВАЖНО: сразу перезаписать NVS, иначе другой узел может ответить state_rsp из старого active=1
        nvs_save_all();
    }
}


// void logic_build_state(uint32_t *epoch,
//                        otIp6Address *owner,
//                        uint32_t *remaining_ms,
//                        bool *active)
// {
//     if (epoch) *epoch = s_state.epoch;

//     if (owner) {
//         if (s_state.owner_valid) {
//             *owner = s_state.owner_addr;
//         } else {
//             memset(owner, 0, sizeof(*owner));
//         }
//     }

//     int64_t now = esp_timer_get_time();
//     uint32_t rem = 0;
//     if (s_state.active && s_state.deadline_us > now) {
//         rem = (uint32_t)((s_state.deadline_us - now) / 1000);
//     }

//     if (remaining_ms) *remaining_ms = rem;
//     if (active) *active = (s_state.active && rem > 0);
// }

void logic_build_state(uint32_t *epoch, otIp6Address *owner, uint32_t *rem_ms, bool *active)
{
    int64_t now = esp_timer_get_time();

    *epoch = s_state.epoch;
    *owner = s_state.owner_addr;

    // если мы в strict-restore режиме — не утверждаем active наружу
    if (s_state.pending_restore) {
        *active = false;
        *rem_ms = 0;
        return;
    }

    *active = s_state.active;

    if (s_state.active && s_state.deadline_us > now) {
        int64_t left_us = s_state.deadline_us - now;
        *rem_ms = (uint32_t)(left_us / 1000);
    } else {
        *rem_ms = 0;
    }
}


void logic_on_state_response(uint32_t epoch,
                             const otIp6Address *owner,
                             uint32_t remaining_ms,
                             bool active)
{
    // 1) Старое поколение игнорируем
    if (epoch < s_state.epoch) {
        return;
    }

    // 2) При том же epoch не даём "переизбрать owner"
    if (epoch == s_state.epoch && s_state.owner_valid) {
        if (!addr_eq(owner, &s_state.owner_addr)) {
            ESP_LOGW(TAG, "state_rsp ignored: same epoch from different owner");
            return;
        }
    }

    int64_t now = esp_timer_get_time();
    int64_t cand_deadline_us = 0;

    if (active && remaining_ms > 0) {
        cand_deadline_us = now + (int64_t)remaining_ms * 1000;
    }

    // 3) Решение accept
    bool accept = false;

    if (epoch > s_state.epoch) {
        // новое поколение — принимаем
        accept = true;
    } else { // epoch == s_state.epoch
        if (s_state.pending_restore) {
            // после ребута/сна принимаем первый нормальный ответ, чтобы ожить
            accept = true;
        } else {
            // ВАЖНО: при том же epoch НИКОГДА не продлеваем дедлайн
            if (active && remaining_ms > 0) {
                if (!s_state.active) {
                    // мы были OFF, а сеть говорит ON — включаемся
                    accept = true;
                } else {
                    // мы уже ON — разрешаем только заметное сокращение (анти-джиттер)
                    if (cand_deadline_us < s_state.deadline_us - 300 * 1000) {
                        accept = true;
                    } else {
                        ESP_LOGI(TAG, "state_rsp duplicate/extend -> ignore (epoch=%lu rem_ms=%lu)",
                                 (unsigned long)epoch, (unsigned long)remaining_ms);
                        return;
                    }
                }
            } else {
                // active=0: при том же epoch не выключаем по чужому мнению,
                // пусть выключение идёт по собственному дедлайну/команде off
                return;
            }
        }
    }

    if (!accept) {
        return;
    }

    // 4) Применяем состояние
    s_state.epoch = epoch;
    s_state.owner_addr = *owner;
    s_state.owner_valid = true;

    if (active && remaining_ms > 0) {
        s_state.active = true;
        s_state.deadline_us = cand_deadline_us;
        s_state.pending_restore = false;

        // if (s_state.mode == MODE_AUTO) {
        //     set_relay(true);
        // }

        ESP_LOGI(TAG, "state_rsp accepted: active=1 epoch=%lu remain_ms=%lu",
                 (unsigned long)epoch, (unsigned long)remaining_ms);
    } else {
        ESP_LOGI(TAG, "state_rsp accepted: active=0 epoch=%lu",
                 (unsigned long)epoch);
        clear_active();
        // epoch уже обновлён выше
    }

    nvs_save_all();
}


// void logic_on_trigger_rx(uint32_t epoch,
//                          const otIp6Address *src,
//                          uint32_t hold_ms)
// {
//     // игнорируем старое
//     if (epoch < s_state.epoch) return;

//     // принимаем (новый или обновление текущего)
//     s_state.epoch = epoch;
//     s_state.owner_addr = *src;
//     s_state.owner_valid = true;

//     if (s_state.mode == MODE_AUTO) {
//         s_state.active = true;
//         int64_t now = esp_timer_get_time();
//         s_state.deadline_us = now + (int64_t)hold_ms * 1000;
//         s_state.pending_restore = false;
//         set_relay(true);
//         ESP_LOGI(TAG, "remote trigger accepted -> ON, epoch=%lu until %lld",
//                  (unsigned long)epoch, (long long)s_state.deadline_us);
//     }

//     nvs_save_all();
// }

// void logic_on_trigger_rx(uint32_t epoch,
//                          const otIp6Address *src,
//                          uint32_t hold_ms)   // по смыслу: rem_ms
// {
//     // игнорируем старое поколение
//     if (epoch < s_state.epoch) {
//         return;
//     }

//     int64_t now = esp_timer_get_time();
//     int64_t new_deadline_us = now + (int64_t)hold_ms * 1000; // hold_ms == rem_ms

//     // если это дубль того же epoch (CoAP-ретрай/повтор) — НЕ продлеваем таймер
//     if (epoch == s_state.epoch && s_state.active) {
//         // если новый дедлайн не дальше текущего (с допуском 200мс) — игнор
//         if (new_deadline_us <= s_state.deadline_us + 200 * 1000) {
//             ESP_LOGI(TAG, "remote trigger duplicate -> ignore (epoch=%lu rem_ms=%lu)",
//                      (unsigned long)epoch, (unsigned long)hold_ms);
//             return;
//         }
//     }

//     // принимаем (новый epoch или реально более дальний дедлайн)
//     s_state.epoch = epoch;
//     s_state.owner_addr = *src;
//     s_state.owner_valid = true;

//     if (s_state.mode == MODE_AUTO) {
//         s_state.active = true;
//         s_state.deadline_us = new_deadline_us;
//         s_state.pending_restore = false;
//         set_relay(true);

//         ESP_LOGI(TAG, "remote trigger accepted -> ON, epoch=%lu until %lld",
//                  (unsigned long)epoch, (long long)s_state.deadline_us);
//     }

//     nvs_save_all();
// }

void logic_on_trigger_rx(uint32_t epoch,
                         const otIp6Address *src,
                         uint32_t hold_ms)   // по смыслу: rem_ms
{
    // игнорируем старое поколение
    if (epoch < s_state.epoch) {
        return;
    }

    // если epoch тот же — принимать только от текущего owner (если он известен)
    if (epoch == s_state.epoch && s_state.owner_valid) {
        if (memcmp(src, &s_state.owner_addr, sizeof(*src)) != 0) {
            ESP_LOGW(TAG, "remote trigger ignored: same epoch from non-owner");
            return;
        }
    }

    int64_t now = esp_timer_get_time();
    int64_t new_deadline_us = now + (int64_t)hold_ms * 1000; // hold_ms == rem_ms

    // дубль того же epoch (CoAP-ретрай) — не продлеваем
    // if (epoch == s_state.epoch && s_state.active) {
    //     if (new_deadline_us <= s_state.deadline_us + 200 * 1000) {
    //         ESP_LOGI(TAG, "remote trigger duplicate -> ignore (epoch=%lu rem_ms=%lu)",
    //                  (unsigned long)epoch, (unsigned long)hold_ms);
    //         return;
    //     }
    // }

    // если тот же epoch — НИКОГДА не продлеваем дедлайн
    if (epoch == s_state.epoch && s_state.active) {
        if (new_deadline_us >= s_state.deadline_us) {
            ESP_LOGI(TAG, "remote trigger duplicate -> ignore (epoch=%lu rem_ms=%lu)",
                    (unsigned long)epoch, (unsigned long)hold_ms);
            return;
        }

        // разрешаем сокращение только если заметное (>300 мс)
        if ((s_state.deadline_us - new_deadline_us) < 300 * 1000) {
            ESP_LOGI(TAG, "remote trigger tiny shorten -> ignore");
            return;
        }
        // допускаем только сокращение дедлайна
    }


    // принимаем (новый epoch или реально более дальний дедлайн от owner)
    s_state.epoch = epoch;
    s_state.owner_addr = *src;
    s_state.owner_valid = true;

    // if (s_state.mode == MODE_AUTO) {
    s_state.active = true;
    s_state.deadline_us = new_deadline_us;
    s_state.pending_restore = false;
        // set_relay(true);

    ESP_LOGI(TAG, "remote trigger accepted -> ON, epoch=%lu until %lld",
        (unsigned long)epoch, (long long)s_state.deadline_us);
    // }

    nvs_save_all();
}


void logic_on_off_rx(uint32_t epoch)
{
    if (epoch != s_state.epoch) return; // только текущий epoch выключает
    ESP_LOGI(TAG, "OFF accepted epoch=%lu", (unsigned long)epoch);
    clear_active();
    // set_relay(false);
    nvs_save_all();
}

static void local_become_owner_or_refresh(bool force_new_owner)
{
    otIp6Address me;
    if (!coap_if_get_my_meshlocal_eid(&me)) return;

    int64_t now = esp_timer_get_time();

        // ---- антидребезг локального trigger ----
    static int64_t s_last_local_trigger_us = 0;
    if (now - s_last_local_trigger_us < 800 * 1000) { // 800 мс
        return;
    }
    s_last_local_trigger_us = now;


    if (force_new_owner || !s_state.active || !s_state.owner_valid || !addr_eq(&me, &s_state.owner_addr)) {
        // новый owner
        s_state.epoch += 1;
        s_state.owner_addr = me;
        s_state.owner_valid = true;
        ESP_LOGI(TAG, "LOCAL trigger -> new owner, epoch=%lu", (unsigned long)s_state.epoch);
    } else {
        // owner=self, просто refresh
        ESP_LOGI(TAG, "LOCAL trigger -> refresh epoch=%lu", (unsigned long)s_state.epoch);
    }

    s_state.active = true;
    s_state.pending_restore = false;
    s_state.last_motion_us = now;
    s_state.deadline_us = now + (int64_t)AUTO_HOLD_MS * 1000;

    // set_relay(true);
    nvs_save_all();

    // всем в зону обновление таймера
    // coap_if_send_trigger(s_state.epoch, (uint32_t)AUTO_HOLD_MS);
    uint32_t rem_ms = 0;
    if (s_state.active && s_state.deadline_us > now) {
        rem_ms = (uint32_t)((s_state.deadline_us - now) / 1000);
    }
    coap_if_send_trigger(s_state.epoch, rem_ms);

}

// static void logic_task(void *arg)
// {
//     (void)arg;

// #if ROLE_CONTROLLER
//     light_mode_t def_mode = io_board_read_mode_switch();
// #else
//     light_mode_t def_mode = MODE_AUTO;
// #endif

//     // init defaults
//     memset(&s_state, 0, sizeof(s_state));
//     nvs_load_all(def_mode);

//     rgb_set_mode_color(s_state.mode);
//     set_relay(false);

//     // ВСЕГДА просим состояние зоны (для восстановления после сна/ребута)
//     // coap_if_send_state_req();

//     // Ждём пока Thread реально "в сети" (иначе multicast часто дропается)
//     int64_t wait_until = esp_timer_get_time() + 5000 * 1000; // 5 сек
//     while (!coap_if_thread_ready() && esp_timer_get_time() < wait_until) {
//         vTaskDelay(pdMS_TO_TICKS(100));
//     }

//     // Первый запрос состояния зоны
//     // ESP_LOGI(TAG, "boot: send state_req (thread_ready=%d)", coap_if_thread_ready());
//     // coap_if_send_state_req();

//     ESP_LOGI(TAG, "boot: send state_req (thread_ready=%d)", coap_if_thread_ready());
//     if (coap_if_thread_ready()) {
//         coap_if_send_state_req();
//     } else {
//         ESP_LOGW(TAG, "boot: thread not ready -> defer state_req");
//     }



//     int64_t now = esp_timer_get_time();
//     int64_t restore_deadline_us = 0;
//     int64_t next_state_req_us = 0;


//     // strict restore: если думали что active — не включаем, ждём state_rsp
//     if (s_state.mode == MODE_AUTO &&
//         s_state.active &&
//         s_state.deadline_us > now) {

//         s_state.pending_restore = true;
//         set_relay(false);
//         restore_deadline_us = now + (int64_t)RESTORE_WAIT_MS * 1000;

//         ESP_LOGI(TAG, "restore(strict): state_req sent, stay OFF until state_rsp");
//     } else {
//         if (s_state.active) {
//             ESP_LOGI(TAG, "restore: invalid stored active -> clear");
//             clear_active();
//             nvs_save_all();
//         }
//     }

//     for (;;) {



//         logic_evt_t e;
//         while (s_logic_q && xQueueReceive(s_logic_q, &e, 0) == pdTRUE) {
//             switch (e.type) {

//                 case EVT_MODE_SET_GLOBAL: {
//                     s_global_mode_valid = true;
//                     s_global_mode = (light_mode_t)(e.u32 & 0xFF);
//                     apply_effective_mode(effective_mode());
//                     nvs_save_all();
//                 } break;

//                 case EVT_MODE_SET_ZONE: {
//                     uint8_t zone = (uint8_t)((e.u32 >> 8) & 0xFF);
//                     light_mode_t mode = (light_mode_t)(e.u32 & 0xFF);
//                     s_zone_mode_valid = true;
//                     s_zone_mode_zone = zone;
//                     s_zone_mode = mode;
//                     apply_effective_mode(effective_mode());
//                     nvs_save_all();
//                 } break;

//                 case EVT_MODE_SET_NODE: {
//                     s_node_mode_valid = true;
//                     s_node_mode = (light_mode_t)(e.u32 & 0xFF);
//                     apply_effective_mode(effective_mode());
//                     nvs_save_all();
//                 } break;

//                 case EVT_MODE_CLR_GLOBAL:
//                     s_global_mode_valid = false;
//                     apply_effective_mode(effective_mode());
//                     nvs_save_all();
//                     break;

//                 case EVT_MODE_CLR_ZONE: {
//                     uint8_t zone = (uint8_t)(e.u32 & 0xFF);
//                     if (s_zone_mode_valid && s_zone_mode_zone == zone) {
//                         s_zone_mode_valid = false;
//                         apply_effective_mode(effective_mode());
//                         nvs_save_all();
//                     }
//                 } break;

//                 case EVT_MODE_CLR_NODE:
//                     s_node_mode_valid = false;
//                     apply_effective_mode(effective_mode());
//                     nvs_save_all();
//                     break;



//                 case EVT_STATE_RSP:
//                     logic_on_state_response(e.epoch, &e.addr, e.u32, e.b);
//                     break;
//                 case EVT_TRIGGER_RX:
//                     logic_on_trigger_rx(e.epoch, &e.addr, e.u32);
//                     break;
//                 case EVT_OFF_RX:
//                     logic_on_off_rx(e.epoch);
//                     break;
//             }
//         }


// #if ROLE_CONTROLLER
//         light_mode_t sw = io_board_read_mode_switch();
//         if (sw != s_state.mode) {
//             s_state.mode = sw;
//             rgb_set_mode_color(sw);
//             if (s_state.mode != MODE_AUTO) {
//                 clear_active();
//                 set_relay(sw == MODE_ON);
//             }
//             nvs_save_all();
//         }
// #endif
//         // ----- retry state_req while pending_restore -----
//         if (s_state.pending_restore) {
//             int64_t now = esp_timer_get_time();

//             // каждые 1 сек пытаемся снова запросить состояние (но только если Thread ready)
//             if (now >= next_state_req_us) {
//                 if (coap_if_thread_ready()) {
//                     ESP_LOGI(TAG, "restore: retry state_req");
//                     coap_if_send_state_req();
//                 } else {
//                     ESP_LOGW(TAG, "restore: thread not ready yet, skip state_req");
//                 }
//                 next_state_req_us = now + 1000 * 1000; // 1 сек
//             }
//         }

//         // strict restore timeout
//         // if (s_state.pending_restore && restore_deadline_us) {
//         //     if (esp_timer_get_time() > restore_deadline_us) {
//         //         ESP_LOGW(TAG, "restore(strict): no state_rsp -> clear, stay OFF");
//         //         clear_active();
//         //         nvs_save_all();
//         //         restore_deadline_us = 0;
//         //     }
//         // }

//         if (s_state.pending_restore && restore_deadline_us) {
//             if (esp_timer_get_time() > restore_deadline_us) {
//                 ESP_LOGW(TAG, "restore(strict): no state_rsp -> stay OFF, but keep NVS (will retry)");
//                 // НЕ clear_active() — остаёмся OFF и продолжаем ретраи state_req
//                 // просто продлеваем окно ожидания ещё раз
//                 restore_deadline_us = esp_timer_get_time() + (int64_t)RESTORE_WAIT_MS * 1000;
//             }
//         }


//         // local sensor
// #if HAS_TFMINI
//         if (s_state.mode == MODE_AUTO) {
//             uint16_t dist = 0;
//             if (tfmini_poll_once(&dist)) {
//                 s_state.dist_cm = dist;

//                 ESP_LOGI(TAG, "TFMINI: dist=%u cm (thr=%u) pending_restore=%d active=%d",
//                     dist, TFMINI_TRIGGER_CM, s_state.pending_restore, s_state.active); 

//                 if (dist > 0 && dist <= TFMINI_TRIGGER_CM) {
//                     // если мы были в pending_restore — всё равно локальный сенсор "истина": становимся owner
//                     local_become_owner_or_refresh(false);
//                 }
//             }
//         }
// #endif
//         light_mode_t m = effective_mode();
//         // apply modes
//         // if (s_state.mode == MODE_OFF) {
//         if (m == MODE_OFF) {
//             set_relay(false);
//         // } else if (s_state.mode == MODE_ON) {
//         } else if (m == MODE_ON) {
//             set_relay(true);
//         } else { // MODE_AUTO
//             if (s_state.pending_restore) {
//                 set_relay(false);
//             } else {
//                 // таймаут
//                 if (s_state.active && s_state.deadline_us && esp_timer_get_time() > s_state.deadline_us) {
//                     ESP_LOGI(TAG, "AUTO deadline expired epoch=%lu", (unsigned long)s_state.epoch);

//                     // OFF по зоне отправляет owner
//                     if (logic_is_owner()) {
//                         ESP_LOGI(TAG, "I am owner -> multicast OFF");
//                         coap_if_send_off(s_state.epoch);
//                     }

//                     clear_active();
//                     set_relay(false);
//                     nvs_save_all();
//                 } else {
//                     set_relay(s_state.active);
//                 }
//             }
//         }

//         vTaskDelay(pdMS_TO_TICKS(50));
//     }
// }


static void apply_mode_and_led(void)
{
    light_mode_t m = effective_mode();
    apply_effective_mode(m);
    rgb_set_mode_color(m);
}


static void logic_task(void *arg)
{
    (void)arg;

#if ROLE_CONTROLLER
    light_mode_t def_mode = io_board_read_mode_switch();
#else
    light_mode_t def_mode = MODE_AUTO;
#endif

    // init defaults
    memset(&s_state, 0, sizeof(s_state));
    nvs_load_all(def_mode);

    apply_mode_and_led();   // применит effective_mode(): relay + цвет (через apply_effective_mode + rgb_set_mode_color)

    // rgb_set_mode_color(s_state.mode);

    // Базово при старте держим OFF до первого apply в цикле
    // set_relay(false);

    // Ждём пока Thread реально "в сети" (иначе multicast часто дропается)
    int64_t wait_until = esp_timer_get_time() + 5000 * 1000; // 5 сек
    while (!coap_if_thread_ready() && esp_timer_get_time() < wait_until) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    ESP_LOGI(TAG, "boot: send state_req (thread_ready=%d)", coap_if_thread_ready());
    if (coap_if_thread_ready()) {
        coap_if_send_state_req();
    } else {
        ESP_LOGW(TAG, "boot: thread not ready -> defer state_req");
    }

    int64_t now = esp_timer_get_time();
    int64_t restore_deadline_us = 0;
    int64_t next_state_req_us = 0;

    // strict restore: если думали что active — не включаем, ждём state_rsp
    // ВАЖНО: проверяем по effective_mode(), а не по s_state.mode
    if (effective_mode() == MODE_AUTO &&
        s_state.active &&
        s_state.deadline_us > now) {

        s_state.pending_restore = true;
        restore_deadline_us = now + (int64_t)RESTORE_WAIT_MS * 1000;

        ESP_LOGI(TAG, "restore(strict): state_req sent, stay OFF until state_rsp");
    } else {
        if (s_state.active) {
            ESP_LOGI(TAG, "restore: invalid stored active -> clear");
            clear_active();
            nvs_save_all();
        }
    }

    for (;;) {

        // 1) Drain logic events queue (CoAP -> logic)
        logic_evt_t e;
        while (s_logic_q && xQueueReceive(s_logic_q, &e, 0) == pdTRUE) {
            switch (e.type) {

                case EVT_MODE_SET_GLOBAL: {
                    s_global_mode_valid = true;
                    s_global_mode = (light_mode_t)(e.u32 & 0xFF);
                    apply_mode_and_led();
                    nvs_save_all();
                } break;

                case EVT_MODE_SET_ZONE: {
                    uint8_t zone = (uint8_t)((e.u32 >> 8) & 0xFF);
                    light_mode_t mode = (light_mode_t)(e.u32 & 0xFF);
                    s_zone_mode_valid = true;
                    s_zone_mode_zone = zone;
                    s_zone_mode = mode;
                    apply_mode_and_led();
                    nvs_save_all();

                } break;

                case EVT_MODE_SET_NODE: {
                    s_node_mode_valid = true;
                    s_node_mode = (light_mode_t)(e.u32 & 0xFF);
                    apply_mode_and_led();
                    nvs_save_all();
                } break;

                case EVT_MODE_CLR_GLOBAL:
                    s_global_mode_valid = false;
                    apply_mode_and_led();
                    nvs_save_all();
                    break;

                case EVT_MODE_CLR_ZONE: {
                    uint8_t zone = (uint8_t)(e.u32 & 0xFF);
                    if (s_zone_mode_valid && s_zone_mode_zone == zone) {
                        s_zone_mode_valid = false;
                        apply_mode_and_led();
                        nvs_save_all();

                    }
                } break;

                case EVT_MODE_CLR_NODE:
                    s_node_mode_valid = false;
                    apply_mode_and_led();
                    nvs_save_all();
                    break;

                case EVT_STATE_RSP:
                    logic_on_state_response(e.epoch, &e.addr, e.u32, e.b);
                    break;

                case EVT_TRIGGER_RX:
                    logic_on_trigger_rx(e.epoch, &e.addr, e.u32);
                    break;

                case EVT_OFF_RX:
                    logic_on_off_rx(e.epoch);
                    break;
            }
        }

#if ROLE_CONTROLLER
        // 2) Controller switch updates local mode (controller strongest anyway via effective_mode())
        light_mode_t sw = io_board_read_mode_switch();
        if (sw != s_state.mode) {
            s_state.mode = sw;
            // rgb_set_mode_color(sw);
            apply_mode_and_led();


            // Если ушли из AUTO — выключаем “зонную активность”, чтобы при возвращении не было старых таймеров
            if (sw != MODE_AUTO) {
                clear_active();
            }

            nvs_save_all();
        }
#endif

        // 3) Retry state_req while pending_restore
        if (s_state.pending_restore) {
            int64_t tnow = esp_timer_get_time();

            // раз в 1 сек повторяем state_req (если Thread ready)
            if (tnow >= next_state_req_us) {
                if (coap_if_thread_ready()) {
                    ESP_LOGI(TAG, "restore: retry state_req");
                    coap_if_send_state_req();
                } else {
                    ESP_LOGW(TAG, "restore: thread not ready yet, skip state_req");
                }
                next_state_req_us = tnow + 1000 * 1000;
            }

            // таймаут окна — просто продлеваем (fail-safe: остаёмся OFF)
            if (restore_deadline_us && tnow > restore_deadline_us) {
                ESP_LOGW(TAG, "restore(strict): no state_rsp -> stay OFF, keep retry");
                restore_deadline_us = tnow + (int64_t)RESTORE_WAIT_MS * 1000;
            }
        }

        // 4) Local sensor (only in AUTO effective mode, and NOT during pending_restore)
#if HAS_TFMINI
        if (effective_mode() == MODE_AUTO && !s_state.pending_restore) {
            uint16_t dist = 0;
            if (tfmini_poll_once(&dist)) {
                s_state.dist_cm = dist;

                // (опционально) лог раз в N, иначе заспамит
                // ESP_LOGI(TAG, "TFMINI: dist=%u cm thr=%u", dist, TFMINI_TRIGGER_CM);

                if (dist > 0 && dist <= TFMINI_TRIGGER_CM) {
                    local_become_owner_or_refresh(false);
                }
            }
        }
#endif

        // 5) Apply effective mode (THE ONLY place that drives relay)
        light_mode_t m = effective_mode();

        if (m == MODE_OFF) {
            // режим OFF сильнее любого active/trigger
            set_relay(false);

        } else if (m == MODE_ON) {
            // режим ON сильнее таймеров
            set_relay(true);

        } else { // MODE_AUTO
            if (s_state.pending_restore) {
                set_relay(false);
            } else {
                int64_t tnow = esp_timer_get_time();

                // deadline
                if (s_state.active && s_state.deadline_us && tnow > s_state.deadline_us) {
                    ESP_LOGI(TAG, "AUTO deadline expired epoch=%lu", (unsigned long)s_state.epoch);

                    if (logic_is_owner()) {
                        ESP_LOGI(TAG, "I am owner -> multicast OFF");
                        coap_if_send_off(s_state.epoch);
                    }

                    clear_active();
                    set_relay(false);
                    nvs_save_all();
                } else {
                    set_relay(s_state.active);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


void logic_start(void)
{
    // xTaskCreate(logic_task, "logic", 4096, NULL, 5, NULL);
    s_logic_q = xQueueCreate(16, sizeof(logic_evt_t));
    xTaskCreate(logic_task, "logic", 4096, NULL, 5, NULL);
}


void logic_post_state_response(uint32_t epoch, const otIp6Address *owner, uint32_t remaining_ms, bool active)
{
    if (!s_logic_q) return;
    logic_evt_t e = {.type=EVT_STATE_RSP, .epoch=epoch, .addr=*owner, .u32=remaining_ms, .b=active};
    xQueueSend(s_logic_q, &e, 0);
}

void logic_post_trigger_rx(uint32_t epoch, const otIp6Address *src, uint32_t hold_ms)
{
    if (!s_logic_q) return;
    logic_evt_t e = {.type=EVT_TRIGGER_RX, .epoch=epoch, .addr=*src, .u32=hold_ms};
    xQueueSend(s_logic_q, &e, 0);
}

void logic_post_off_rx(uint32_t epoch)
{
    if (!s_logic_q) return;
    logic_evt_t e = {.type=EVT_OFF_RX, .epoch=epoch};
    xQueueSend(s_logic_q, &e, 0);
}


const zone_state_t *logic_get_state(void)
{
    return &s_state;
}

void logic_set_mode(light_mode_t mode)
{
    if (mode > MODE_AUTO) mode = MODE_AUTO;
    if (s_state.mode == mode) return;

    s_state.mode = mode;
    rgb_set_mode_color(mode);

    if (mode != MODE_AUTO) {
        clear_active();
        // set_relay(mode == MODE_ON);
    } else {
        // в AUTO при входе — запросим состояние зоны
        coap_if_send_state_req();
        // set_relay(false);
    }

    nvs_save_all();
}
