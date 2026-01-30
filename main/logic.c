#include "logic.h"
#include "config.h"
#include "io_board.h"
#include "rgb_led.h"
#include "tfmini.h"
#include "coap_if.h"
#include "config_store.h"
#include "rust_payload.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "nvs.h"

#include <string.h>

#include "esp_system.h"   // esp_reset_reason()

#include "freertos/queue.h"
#include "openthread/cli.h"


static const char *TAG = "logic";
typedef enum {
    FSM_AUTO_IDLE = 0,
    FSM_AUTO_ACTIVE,
    FSM_MANUAL_ON,
    FSM_MANUAL_OFF,
    FSM_PENDING_RESTORE,
} fsm_state_t;

typedef struct {
    zone_state_t zone;
    fsm_state_t fsm;

    bool global_mode_valid;
    light_mode_t global_mode;

    bool zone_mode_valid;
    uint8_t zone_mode_zone;
    light_mode_t zone_mode;

    bool node_mode_valid;
    light_mode_t node_mode;

    int64_t restore_deadline_us;
    int64_t next_state_req_us;
    int64_t last_local_trigger_us;
    bool nvs_dirty;
    uint64_t nvs_next_flush_us;
    bool last_trigger_valid;
    uint32_t last_trigger_epoch;
    otIp6Address last_trigger_addr;
    uint32_t last_trigger_rem_ms;
    int64_t last_trigger_time_us;
    bool last_state_rsp_valid;
    uint32_t last_state_rsp_epoch;
    otIp6Address last_state_rsp_addr;
    uint32_t last_state_rsp_rem_ms;
    int64_t last_state_rsp_time_us;
} logic_state_t;

static logic_state_t s_state;




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
    EVT_LOCAL_MODE_SET,
    EVT_LOCAL_TRIGGER,
    EVT_TICK,
    EVT_ENTER_PENDING_RESTORE,
    EVT_COLD_BOOT,

} logic_evt_type_t;

typedef struct {
    logic_evt_type_t type;
    uint32_t epoch;
    otIp6Address addr;
    uint32_t u32;
    bool b;
} logic_evt_t;

static QueueHandle_t s_logic_q;

static void logic_queue_send(const logic_evt_t *e)
{
    if (!s_logic_q || !e) {
        return;
    }
    if (xQueueSend(s_logic_q, e, 0) != pdTRUE) {
        ESP_LOGW(TAG, "logic queue full, drop evt=%d", (int)e->type);
    }
}


// ===== NVS keys =====
#define NVS_NS           "app"
#define NVS_K_MODE       "mode"
#define NVS_K_EPOCH      "epoch"
#define NVS_K_ACTIVE     "active"
#define NVS_K_DEADLINE   "deadline_us"
#define NVS_K_OWNER_OK   "owner_ok"
#define NVS_K_OWNER_ADDR "owner_addr"

#define RESTORE_WAIT_MS  1200  // ждать state_rsp после ребута (strict)
#define RESTORE_RETRY_INTERVAL_US (30 * 1000 * 1000)
#define RESTORE_COLD_BOOT_TIMEOUT_US (3 * 60 * 1000 * 1000)
#define NVS_DEBOUNCE_US  (5 * 1000 * 1000)
#define RX_DEDUP_WINDOW_US (2 * 1000 * 1000)
#define RX_DEDUP_MIN_DIFF_MS 300

// ===== NVS keys for MODE overrides (persistent) =====
#define NVS_K_GMODE_VALID  "g_valid"
#define NVS_K_GMODE        "g_mode"

#define NVS_K_ZMODE_VALID  "z_valid"
#define NVS_K_ZMODE_ZONE   "z_zone"
#define NVS_K_ZMODE        "z_mode"

#define NVS_K_NMODE_VALID  "n_valid"
#define NVS_K_NMODE        "n_mode"


static void set_relay(logic_state_t *state, bool on)
{
    io_board_set_relay(on);
    state->zone.relay_on = on;
}

static void state_clear_active(logic_state_t *state)
{
    state->zone.active = false;
    state->zone.deadline_us = 0;
    state->zone.owner_valid = false;
    memset(&state->zone.owner_addr, 0, sizeof(state->zone.owner_addr));
    state->zone.pending_restore = false;
}

static void clear_active(void)
{
    state_clear_active(&s_state);
}

static bool addr_eq(const otIp6Address *a, const otIp6Address *b)
{
    return memcmp(a->mFields.m8, b->mFields.m8, 16) == 0;
}

bool logic_is_owner(void)
{
    if (!s_state.zone.owner_valid) return false;
    otIp6Address me;
    if (!coap_if_get_my_meshlocal_eid(&me)) return false;
    return addr_eq(&me, &s_state.zone.owner_addr);
}

static void nvs_save_all(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;

    nvs_set_u8(h, NVS_K_MODE, (uint8_t)s_state.zone.mode);
    nvs_set_u32(h, NVS_K_EPOCH, (uint32_t)s_state.zone.epoch);
    nvs_set_u8(h, NVS_K_ACTIVE, (uint8_t)(s_state.zone.active ? 1 : 0));
    nvs_set_i64(h, NVS_K_DEADLINE, (int64_t)s_state.zone.deadline_us);
    nvs_set_u8(h, NVS_K_OWNER_OK, (uint8_t)(s_state.zone.owner_valid ? 1 : 0));
    if (s_state.zone.owner_valid) {
        nvs_set_blob(h, NVS_K_OWNER_ADDR, s_state.zone.owner_addr.mFields.m8, 16);
    } else {
        uint8_t z[16] = {0};
        nvs_set_blob(h, NVS_K_OWNER_ADDR, z, 16);
    }

    // --- persist overrides ---
    nvs_set_u8(h, NVS_K_GMODE_VALID, (uint8_t)(s_state.global_mode_valid ? 1 : 0));
    nvs_set_u8(h, NVS_K_GMODE,       (uint8_t)s_state.global_mode);

    nvs_set_u8(h, NVS_K_ZMODE_VALID, (uint8_t)(s_state.zone_mode_valid ? 1 : 0));
    nvs_set_u8(h, NVS_K_ZMODE_ZONE,  (uint8_t)s_state.zone_mode_zone);
    nvs_set_u8(h, NVS_K_ZMODE,       (uint8_t)s_state.zone_mode);

    nvs_set_u8(h, NVS_K_NMODE_VALID, (uint8_t)(s_state.node_mode_valid ? 1 : 0));
    nvs_set_u8(h, NVS_K_NMODE,       (uint8_t)s_state.node_mode);


    nvs_commit(h);
    nvs_close(h);
}

static light_mode_t effective_mode(const logic_state_t *state)
{
#if ROLE_CONTROLLER
    return io_board_read_mode_switch();   // контроллер главный
#else
    if (state->node_mode_valid) return state->node_mode;
    if (state->zone_mode_valid && state->zone_mode_zone == config_store_get()->zone_id) return state->zone_mode;
    if (state->global_mode_valid) return state->global_mode;
    return state->zone.mode;                  // локальный режим (NVS/CLI)
#endif
}

typedef struct {
    bool set_relay;
    bool relay_on;
    bool update_led;
    bool save_nvs;
    bool send_state_req;
    bool send_trigger;
    uint32_t trigger_rem_ms;
    bool send_off;
    uint32_t off_epoch;
    bool log_transition;
    bool flush_nvs_now;
    fsm_state_t from_state;
    fsm_state_t to_state;
    logic_evt_type_t event;
} fsm_actions_t;

static const char *fsm_state_name(fsm_state_t state)
{
    switch (state) {
        case FSM_AUTO_IDLE: return "AutoIdle";
        case FSM_AUTO_ACTIVE: return "AutoActive";
        case FSM_MANUAL_ON: return "ManualOn";
        case FSM_MANUAL_OFF: return "ManualOff";
        case FSM_PENDING_RESTORE: return "PendingRestore";
        default: return "Unknown";
    }
}

static const char *event_name(logic_evt_type_t event)
{
    switch (event) {
        case EVT_STATE_RSP: return "STATE_RSP";
        case EVT_TRIGGER_RX: return "TRIGGER_RX";
        case EVT_OFF_RX: return "OFF_RX";
        case EVT_MODE_SET_GLOBAL: return "MODE_SET_GLOBAL";
        case EVT_MODE_SET_ZONE: return "MODE_SET_ZONE";
        case EVT_MODE_SET_NODE: return "MODE_SET_NODE";
        case EVT_MODE_CLR_GLOBAL: return "MODE_CLR_GLOBAL";
        case EVT_MODE_CLR_ZONE: return "MODE_CLR_ZONE";
        case EVT_MODE_CLR_NODE: return "MODE_CLR_NODE";
        case EVT_LOCAL_MODE_SET: return "LOCAL_MODE_SET";
        case EVT_LOCAL_TRIGGER: return "LOCAL_TRIGGER";
        case EVT_TICK: return "TICK";
        case EVT_ENTER_PENDING_RESTORE: return "ENTER_PENDING_RESTORE";
        case EVT_COLD_BOOT: return "COLD_BOOT";
        default: return "UNKNOWN";
    }
}

void logic_cli_print_state(void)
{
    uint32_t epoch = 0;
    uint32_t rem_ms = 0;
    bool active = false;
    otIp6Address owner;
    logic_build_state(&epoch, &owner, &rem_ms, &active);

    char owner_str[OT_IP6_ADDRESS_STRING_SIZE];
    otIp6AddressToString(&owner, owner_str, sizeof(owner_str));

    light_mode_t mode = effective_mode(&s_state);
    const char *fsm = fsm_state_name(s_state.fsm);

    otCliOutputFormat("epoch=%lu active=%u rem_ms=%lu fsm=%s mode=%u owner=%s\r\n",
                      (unsigned long)epoch,
                      active ? 1u : 0u,
                      (unsigned long)rem_ms,
                      fsm,
                      (unsigned)mode,
                      owner_str);
}

bool logic_post_parsed(logic_parsed_kind_t kind,
                       const rust_parsed_t *parsed,
                       const otIp6Address *peer_addr,
                       bool is_multicast)
{
    if (!parsed) {
        return false;
    }

    int epoch = parsed->has_epoch ? (int)parsed->epoch : -1;
    int rem_ms = parsed->has_rem_ms ? (int)parsed->rem_ms : -1;
    int active = parsed->has_active ? (int)parsed->active : -1;
    int mode = parsed->has_m ? (int)parsed->m : (parsed->has_mode ? (int)parsed->mode : -1);

    ESP_LOGI(TAG, "parsed: epoch=%d rem_ms=%d active=%d mode=%d",
             epoch, rem_ms, active, mode);

    switch (kind) {
        case LOGIC_PARSED_STATE_RSP: {
            if (!parsed->has_epoch || !parsed->has_active) {
                return false;
            }
            uint32_t remaining_ms = parsed->has_rem_ms ? parsed->rem_ms : 0;
            bool is_active = (parsed->active != 0);
            otIp6Address owner = {0};
            if (peer_addr) {
                owner = *peer_addr;
            }
            logic_post_state_response(parsed->epoch, &owner, remaining_ms, is_active);
            return true;
        }
        case LOGIC_PARSED_TRIGGER: {
            if (!parsed->has_epoch) {
                return false;
            }
            uint32_t rem = parsed->has_rem_ms ? parsed->rem_ms : config_store_get()->auto_hold_ms;
            otIp6Address src = {0};
            if (peer_addr) {
                src = *peer_addr;
            }
            logic_post_trigger_rx(parsed->epoch, &src, rem);
            return true;
        }
        case LOGIC_PARSED_OFF: {
            if (!parsed->has_epoch) {
                return false;
            }
            logic_post_off_rx(parsed->epoch);
            return true;
        }
        case LOGIC_PARSED_MODE: {
            if (parsed->has_clr) {
                if (!is_multicast) {
                    logic_post_mode_clear_node();
                } else if (parsed->has_z) {
                    logic_post_mode_clear_zone((uint8_t)parsed->z);
                } else {
                    logic_post_mode_clear_global();
                }
                return true;
            }
            if (parsed->has_m || parsed->has_mode) {
                uint32_t m = parsed->has_m ? parsed->m : parsed->mode;
                if (m > 2) {
                    return false;
                }
                light_mode_t mode_val = (light_mode_t)m;
                if (!is_multicast) {
                    logic_post_mode_cmd_node(mode_val);
                } else if (parsed->has_z) {
                    logic_post_mode_cmd_zone((uint8_t)parsed->z, mode_val);
                } else {
                    logic_post_mode_cmd_global(mode_val);
                }
                return true;
            }
            return false;
        }
        default:
            return false;
    }
}

static fsm_state_t fsm_from_state(const logic_state_t *state, int64_t now)
{
    light_mode_t mode = effective_mode(state);
    if (mode == MODE_OFF) {
        return FSM_MANUAL_OFF;
    }
    if (mode == MODE_ON) {
        return FSM_MANUAL_ON;
    }
    if (state->zone.pending_restore) {
        return FSM_PENDING_RESTORE;
    }
    if (state->zone.active && state->zone.deadline_us > now) {
        return FSM_AUTO_ACTIVE;
    }
    return FSM_AUTO_IDLE;
}

static void fsm_sync(logic_state_t *state, int64_t now)
{
    light_mode_t mode = effective_mode(state);
    if (mode == MODE_OFF || mode == MODE_ON) {
        state_clear_active(state);
    }
    state->fsm = fsm_from_state(state, now);
}

static void set_transition_action(fsm_actions_t *actions,
                                  fsm_state_t from_state,
                                  fsm_state_t to_state,
                                  logic_evt_type_t event)
{
    if (from_state == to_state) {
        return;
    }
    actions->log_transition = true;
    actions->from_state = from_state;
    actions->to_state = to_state;
    actions->event = event;
}

static fsm_actions_t step(logic_state_t *state, const logic_evt_t *event, int64_t now)
{
    fsm_actions_t actions = {0};
    fsm_state_t prev_state = state->fsm;

    switch (event->type) {
        case EVT_MODE_SET_GLOBAL:
            state->global_mode_valid = true;
            state->global_mode = (light_mode_t)(event->u32 & 0xFF);
            actions.update_led = true;
            actions.save_nvs = true;
            fsm_sync(state, now);
            break;

        case EVT_MODE_SET_ZONE: {
            uint8_t zone = (uint8_t)((event->u32 >> 8) & 0xFF);
            light_mode_t mode = (light_mode_t)(event->u32 & 0xFF);
            state->zone_mode_valid = true;
            state->zone_mode_zone = zone;
            state->zone_mode = mode;
            actions.update_led = true;
            actions.save_nvs = true;
            fsm_sync(state, now);
        } break;

        case EVT_MODE_SET_NODE:
            state->node_mode_valid = true;
            state->node_mode = (light_mode_t)(event->u32 & 0xFF);
            actions.update_led = true;
            actions.save_nvs = true;
            fsm_sync(state, now);
            break;

        case EVT_MODE_CLR_GLOBAL:
            state->global_mode_valid = false;
            actions.update_led = true;
            actions.save_nvs = true;
            fsm_sync(state, now);
            break;

        case EVT_MODE_CLR_ZONE: {
            uint8_t zone = (uint8_t)(event->u32 & 0xFF);
            if (state->zone_mode_valid && state->zone_mode_zone == zone) {
                state->zone_mode_valid = false;
                actions.update_led = true;
                actions.save_nvs = true;
                fsm_sync(state, now);
            }
        } break;

        case EVT_MODE_CLR_NODE:
            state->node_mode_valid = false;
            actions.update_led = true;
            actions.save_nvs = true;
            fsm_sync(state, now);
            break;

        case EVT_LOCAL_MODE_SET: {
            light_mode_t mode = (light_mode_t)(event->u32 & 0xFF);
            if (mode > MODE_AUTO) {
                mode = MODE_AUTO;
            }
            if (state->zone.mode == mode) {
                break;
            }
            state->zone.mode = mode;
            actions.update_led = true;
            actions.save_nvs = true;
            if (mode != MODE_AUTO) {
                state_clear_active(state);
            } else {
                actions.send_state_req = true;
            }
            fsm_sync(state, now);
        } break;

        case EVT_STATE_RSP: {
            int64_t delta_us = now - state->last_state_rsp_time_us;
            if (state->last_state_rsp_valid &&
                event->epoch == state->last_state_rsp_epoch &&
                addr_eq(&event->addr, &state->last_state_rsp_addr) &&
                delta_us >= 0 && delta_us < RX_DEDUP_WINDOW_US) {
                uint32_t last_rem = state->last_state_rsp_rem_ms;
                uint32_t rem = event->u32;
                uint32_t diff = (last_rem > rem) ? (last_rem - rem) : (rem - last_rem);
                if (diff < RX_DEDUP_MIN_DIFF_MS) {
                    ESP_LOGD(TAG, "RX state_rsp duplicate ignored epoch=%lu rem_ms=%lu",
                             (unsigned long)event->epoch, (unsigned long)event->u32);
                    return actions;
                }
            }

            if (event->epoch < state->zone.epoch) {
                break;
            }
            if (event->epoch == state->zone.epoch && state->zone.owner_valid) {
                if (!addr_eq(&event->addr, &state->zone.owner_addr)) {
                    break;
                }
            }

            int64_t cand_deadline_us = 0;
            if (event->b && event->u32 > 0) {
                cand_deadline_us = now + (int64_t)event->u32 * 1000;
            }

            bool accept = false;
            if (event->epoch > state->zone.epoch) {
                accept = true;
            } else {
                if (state->zone.pending_restore) {
                    accept = true;
                } else {
                    if (event->b && event->u32 > 0) {
                        if (!state->zone.active) {
                            accept = true;
                        } else {
                            if (cand_deadline_us < state->zone.deadline_us - 300 * 1000) {
                                accept = true;
                            } else {
                                break;
                            }
                        }
                    } else {
                        break;
                    }
                }
            }

            if (!accept) {
                break;
            }

            state->zone.epoch = event->epoch;
            state->zone.owner_addr = event->addr;
            state->zone.owner_valid = true;

            if (event->b && event->u32 > 0) {
                state->zone.active = true;
                state->zone.deadline_us = cand_deadline_us;
                state->zone.pending_restore = false;
            } else {
                state_clear_active(state);
            }

            actions.save_nvs = true;
            state->last_state_rsp_valid = true;
            state->last_state_rsp_epoch = event->epoch;
            state->last_state_rsp_addr = event->addr;
            state->last_state_rsp_rem_ms = event->u32;
            state->last_state_rsp_time_us = now;
            fsm_sync(state, now);
        } break;

        case EVT_TRIGGER_RX: {
            int64_t delta_us = now - state->last_trigger_time_us;
            if (state->last_trigger_valid &&
                event->epoch == state->last_trigger_epoch &&
                addr_eq(&event->addr, &state->last_trigger_addr) &&
                delta_us >= 0 && delta_us < RX_DEDUP_WINDOW_US) {
                uint32_t last_rem = state->last_trigger_rem_ms;
                uint32_t rem = event->u32;
                uint32_t diff = (last_rem > rem) ? (last_rem - rem) : (rem - last_rem);
                if (diff < RX_DEDUP_MIN_DIFF_MS) {
                    ESP_LOGD(TAG, "RX trigger duplicate ignored epoch=%lu rem_ms=%lu",
                             (unsigned long)event->epoch, (unsigned long)event->u32);
                    return actions;
                }
            }

            if (event->epoch < state->zone.epoch) {
                break;
            }
            if (event->epoch == state->zone.epoch && state->zone.owner_valid) {
                if (memcmp(&event->addr, &state->zone.owner_addr, sizeof(event->addr)) != 0) {
                    break;
                }
            }

            int64_t new_deadline_us = now + (int64_t)event->u32 * 1000;
            if (event->epoch == state->zone.epoch && state->zone.active) {
                if (new_deadline_us >= state->zone.deadline_us) {
                    break;
                }
                if ((state->zone.deadline_us - new_deadline_us) < 300 * 1000) {
                    break;
                }
            }

            state->zone.epoch = event->epoch;
            state->zone.owner_addr = event->addr;
            state->zone.owner_valid = true;
            state->zone.active = true;
            state->zone.deadline_us = new_deadline_us;
            state->zone.pending_restore = false;
            actions.save_nvs = true;
            state->last_trigger_valid = true;
            state->last_trigger_epoch = event->epoch;
            state->last_trigger_addr = event->addr;
            state->last_trigger_rem_ms = event->u32;
            state->last_trigger_time_us = now;
            fsm_sync(state, now);
        } break;

        case EVT_OFF_RX:
            if (event->epoch != state->zone.epoch) {
                break;
            }
            state_clear_active(state);
            actions.save_nvs = true;
            fsm_sync(state, now);
            break;

        case EVT_LOCAL_TRIGGER: {
            if (now - state->last_local_trigger_us < 800 * 1000) {
                break;
            }
            state->last_local_trigger_us = now;

            otIp6Address me;
            if (!coap_if_get_my_meshlocal_eid(&me)) {
                break;
            }

            bool force_new_owner = event->b;
            if (force_new_owner || !state->zone.active || !state->zone.owner_valid ||
                !addr_eq(&me, &state->zone.owner_addr)) {
                state->zone.epoch += 1;
                state->zone.owner_addr = me;
                state->zone.owner_valid = true;
            }

            state->zone.active = true;
            state->zone.pending_restore = false;
            state->zone.last_motion_us = now;
            state->zone.deadline_us = now + (int64_t)config_store_get()->auto_hold_ms * 1000;
            actions.save_nvs = true;

            if (state->zone.deadline_us > now) {
                actions.send_trigger = true;
                actions.trigger_rem_ms = (uint32_t)((state->zone.deadline_us - now) / 1000);
            }

            fsm_sync(state, now);
        } break;

        case EVT_ENTER_PENDING_RESTORE:
            state->zone.pending_restore = true;
            state->restore_deadline_us = now + (int64_t)RESTORE_WAIT_MS * 1000;
            state->next_state_req_us = now;
            fsm_sync(state, now);
            break;

        case EVT_COLD_BOOT:
            state_clear_active(state);
            state->zone.owner_valid = false;
            state->zone.pending_restore = true;
            state->restore_deadline_us = now + RESTORE_COLD_BOOT_TIMEOUT_US;
            state->next_state_req_us = now;
            actions.flush_nvs_now = true;
            fsm_sync(state, now);
            break;

        case EVT_TICK: {
            if (state->zone.pending_restore) {
                if (now >= state->next_state_req_us) {
                    actions.send_state_req = true;
                    state->next_state_req_us = now + RESTORE_RETRY_INTERVAL_US;
                }
                if (state->restore_deadline_us && now > state->restore_deadline_us) {
                    state_clear_active(state);
                    state->zone.pending_restore = false;
                    actions.flush_nvs_now = true;
                    fsm_sync(state, now);
                }
            }

            if (state->fsm == FSM_AUTO_ACTIVE && state->zone.active &&
                state->zone.deadline_us && now > state->zone.deadline_us) {
                if (logic_is_owner()) {
                    actions.send_off = true;
                    actions.off_epoch = state->zone.epoch;
                }
                state_clear_active(state);
                actions.save_nvs = true;
                fsm_sync(state, now);
            }

            if (state->nvs_dirty && state->nvs_next_flush_us &&
                (uint64_t)now >= state->nvs_next_flush_us) {
                actions.flush_nvs_now = true;
            }

            actions.set_relay = true;
            switch (state->fsm) {
                case FSM_MANUAL_OFF:
                case FSM_AUTO_IDLE:
                case FSM_PENDING_RESTORE:
                    actions.relay_on = false;
                    break;
                case FSM_MANUAL_ON:
                case FSM_AUTO_ACTIVE:
                    actions.relay_on = true;
                    break;
                default:
                    actions.relay_on = false;
                    break;
            }
        } break;
    }

    set_transition_action(&actions, prev_state, state->fsm, event->type);
    return actions;
}

static void apply_actions(logic_state_t *state, const fsm_actions_t *actions)
{
    uint64_t now_us = (uint64_t)esp_timer_get_time();

    if (actions->update_led) {
        rgb_set_mode_color(effective_mode(state));
    }
    if (actions->set_relay) {
        set_relay(state, actions->relay_on);
    }
    if (actions->send_state_req) {
        if (coap_if_thread_ready()) {
            coap_if_send_state_req();
        }
    }
    if (actions->send_trigger) {
        coap_if_send_trigger(state->zone.epoch, actions->trigger_rem_ms);
    }
    if (actions->send_off) {
        coap_if_send_off(actions->off_epoch);
    }
    if (actions->flush_nvs_now) {
        ESP_LOGI(TAG, "NVS flush");
        nvs_save_all();
        state->nvs_dirty = false;
        state->nvs_next_flush_us = 0;
    } else if (actions->save_nvs) {
        if (!state->nvs_dirty) {
            state->nvs_dirty = true;
            state->nvs_next_flush_us = now_us + NVS_DEBOUNCE_US;
            ESP_LOGI(TAG, "NVS dirty, schedule flush in %lu ms",
                     (unsigned long)(NVS_DEBOUNCE_US / 1000));
        } else {
            state->nvs_next_flush_us = now_us + NVS_DEBOUNCE_US;
        }
    }
    if (actions->log_transition) {
        ESP_LOGI(TAG, "FSM %s -> %s on %s",
                 fsm_state_name(actions->from_state),
                 fsm_state_name(actions->to_state),
                 event_name(actions->event));
    }
}



void logic_post_mode_cmd_global(light_mode_t mode)
{
    logic_evt_t e = { .type = EVT_MODE_SET_GLOBAL, .u32 = (uint32_t)mode };
    logic_queue_send(&e);
}

void logic_post_mode_cmd_zone(uint8_t zone_id, light_mode_t mode)
{
    logic_evt_t e = { .type = EVT_MODE_SET_ZONE, .u32 = ((uint32_t)zone_id << 8) | (uint32_t)mode };
    logic_queue_send(&e);
}

void logic_post_mode_cmd_node(light_mode_t mode)
{
    logic_evt_t e = { .type = EVT_MODE_SET_NODE, .u32 = (uint32_t)mode };
    logic_queue_send(&e);
}

void logic_post_mode_clear_global(void)
{
    logic_evt_t e = { .type = EVT_MODE_CLR_GLOBAL };
    logic_queue_send(&e);
}

void logic_post_mode_clear_zone(uint8_t zone_id)
{
    logic_evt_t e = { .type = EVT_MODE_CLR_ZONE, .u32 = (uint32_t)zone_id };
    logic_queue_send(&e);
}

void logic_post_mode_clear_node(void)
{
    logic_evt_t e = { .type = EVT_MODE_CLR_NODE };
    logic_queue_send(&e);
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
        s_state.zone.mode = def_mode;
        s_state.zone.epoch = 0;
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
    s_state.global_mode_valid = (g_valid != 0);
    s_state.global_mode = (light_mode_t)g_mode;
    if (s_state.global_mode > MODE_AUTO) { s_state.global_mode = MODE_AUTO; s_state.global_mode_valid = false; }

    s_state.zone_mode_valid = (z_valid != 0);
    s_state.zone_mode_zone  = z_zone;
    s_state.zone_mode = (light_mode_t)z_mode;
    if (s_state.zone_mode > MODE_AUTO) { s_state.zone_mode = MODE_AUTO; s_state.zone_mode_valid = false; }

    s_state.node_mode_valid = (n_valid != 0);
    s_state.node_mode = (light_mode_t)n_mode;
    if (s_state.node_mode > MODE_AUTO) { s_state.node_mode = MODE_AUTO; s_state.node_mode_valid = false; }


    if (mode > MODE_AUTO) mode = (uint8_t)def_mode;

    s_state.zone.mode = (light_mode_t)mode;
    s_state.zone.epoch = epoch;
    s_state.zone.active = (active != 0);
    s_state.zone.deadline_us = deadline_us;
    s_state.zone.owner_valid = (owner_ok != 0);
    memcpy(s_state.zone.owner_addr.mFields.m8, owner, 16);
    s_state.zone.pending_restore = false;

    ESP_LOGI(TAG, "NVS: mode=%u active=%u deadline_us=%lld owner_ok=%u epoch=%lu",
             (unsigned)mode,
             (unsigned)active,
             (long long)deadline_us,
             (unsigned)owner_ok,
             (unsigned long)epoch);

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

    *epoch = s_state.zone.epoch;
    if (s_state.zone.owner_valid) {
        *owner = s_state.zone.owner_addr;
    } else {
        memset(owner, 0, sizeof(*owner));
    }

    // если мы в strict-restore режиме — не утверждаем active наружу
    if (s_state.zone.pending_restore) {
        *active = false;
        *rem_ms = 0;
        return;
    }

    *active = s_state.zone.active;

    if (s_state.zone.active && s_state.zone.deadline_us > now) {
        int64_t left_us = s_state.zone.deadline_us - now;
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
    logic_post_state_response(epoch, owner, remaining_ms, active);
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
    logic_post_trigger_rx(epoch, src, hold_ms);
}


void logic_on_off_rx(uint32_t epoch)
{
    logic_post_off_rx(epoch);
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

    s_state.fsm = FSM_AUTO_IDLE;

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

    // cold boot handling (do not reset epoch)
    int64_t now = esp_timer_get_time();
    esp_reset_reason_t rr = esp_reset_reason();
    if (rr == ESP_RST_POWERON || rr == ESP_RST_BROWNOUT) {
        logic_evt_t cold = {.type = EVT_COLD_BOOT};
        fsm_actions_t cold_actions = step(&s_state, &cold, now);
        apply_actions(&s_state, &cold_actions);
    }

    // strict restore: если думали что active — не включаем, ждём state_rsp
    // ВАЖНО: проверяем по effective_mode(), а не по s_state.mode
    if (effective_mode(&s_state) == MODE_AUTO &&
        s_state.zone.active &&
        s_state.zone.deadline_us > now) {

        logic_evt_t enter = {.type = EVT_ENTER_PENDING_RESTORE};
        fsm_actions_t enter_actions = step(&s_state, &enter, now);
        apply_actions(&s_state, &enter_actions);
        ESP_LOGI(TAG, "restore(strict): state_req sent, stay OFF until state_rsp");
    } else {
        if (s_state.zone.active) {
            ESP_LOGI(TAG, "restore: invalid stored active -> clear");
            clear_active();
            fsm_actions_t flush_actions = {.flush_nvs_now = true};
            apply_actions(&s_state, &flush_actions);
        }
    }

    s_state.fsm = fsm_from_state(&s_state, now);
    {
        fsm_actions_t init_actions = {.update_led = true};
        apply_actions(&s_state, &init_actions);
    }

    for (;;) {
        now = esp_timer_get_time();

        // 1) Drain logic events queue (CoAP -> logic)
        logic_evt_t e;
        while (s_logic_q && xQueueReceive(s_logic_q, &e, 0) == pdTRUE) {
            fsm_actions_t actions = step(&s_state, &e, now);
            apply_actions(&s_state, &actions);
        }

#if ROLE_CONTROLLER
        // 2) Controller switch updates local mode (controller strongest anyway via effective_mode())
        light_mode_t sw = io_board_read_mode_switch();
        if (sw != s_state.zone.mode) {
            logic_evt_t ev = {.type = EVT_LOCAL_MODE_SET, .u32 = (uint32_t)sw};
            fsm_actions_t actions = step(&s_state, &ev, now);
            apply_actions(&s_state, &actions);
        }
#endif

        // 4) Local sensor (only in AUTO effective mode, and NOT during pending_restore)
#if HAS_TFMINI
        if (effective_mode(&s_state) == MODE_AUTO && !s_state.zone.pending_restore) {
            uint16_t dist = 0;
            if (tfmini_poll_once(&dist)) {
                s_state.zone.dist_cm = dist;

                // (опционально) лог раз в N, иначе заспамит
                // ESP_LOGI(TAG, "TFMINI: dist=%u cm thr=%u", dist, TFMINI_TRIGGER_CM);

                if (dist > 0 && dist <= config_store_get()->tfmini_trigger_cm) {
                    logic_evt_t ev = {.type = EVT_LOCAL_TRIGGER, .b = false};
                    fsm_actions_t actions = step(&s_state, &ev, now);
                    apply_actions(&s_state, &actions);
                }
            }
        }
#endif

        // 5) Tick: deadlines, pending restore, relay
        logic_evt_t tick = {.type = EVT_TICK};
        fsm_actions_t tick_actions = step(&s_state, &tick, now);
        apply_actions(&s_state, &tick_actions);

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
    logic_evt_t e = {.type=EVT_STATE_RSP, .epoch=epoch, .addr=*owner, .u32=remaining_ms, .b=active};
    logic_queue_send(&e);
}

void logic_post_trigger_rx(uint32_t epoch, const otIp6Address *src, uint32_t hold_ms)
{
    logic_evt_t e = {.type=EVT_TRIGGER_RX, .epoch=epoch, .addr=*src, .u32=hold_ms};
    logic_queue_send(&e);
}

void logic_post_off_rx(uint32_t epoch)
{
    logic_evt_t e = {.type=EVT_OFF_RX, .epoch=epoch};
    logic_queue_send(&e);
}


const zone_state_t *logic_get_state(void)
{
    return &s_state.zone;
}

void logic_set_mode(light_mode_t mode)
{
    logic_evt_t ev = {.type = EVT_LOCAL_MODE_SET, .u32 = (uint32_t)mode};
    logic_queue_send(&ev);
}
