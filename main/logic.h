#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "rgb_led.h"          // light_mode_t
#include <openthread/ip6.h>   // otIp6Address

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // старое
    light_mode_t mode;
    bool         relay_on;
    uint16_t     dist_cm;
    int64_t      last_motion_us;

    // новое (зона-состояние для синхронизации)
    uint32_t     epoch;            // версия события в зоне (монотонно растёт)
    bool         active;           // зона "включена" по AUTO-триггеру
    bool         pending_restore;  // strict-restore: ждём state_rsp
    int64_t      deadline_us;      // локальный дедлайн (esp_timer_get_time(), us)

    otIp6Address owner_addr;       // кто owner (mesh-local EID отправителя trigger)
    bool         owner_valid;
} zone_state_t;

// старт логики
void logic_start(void);

// состояние для /status
const zone_state_t *logic_get_state(void);

// смена режима OFF/ON/AUTO (через CoAP /mode или тумблер)
void logic_set_mode(light_mode_t mode);

// true если текущий узел = owner (по owner_addr)
bool logic_is_owner(void);

void logic_cli_print_state(void);

// собрать состояние зоны для ответов state_rsp
void logic_build_state(uint32_t *epoch,
                       otIp6Address *owner,
                       uint32_t *remaining_ms,
                       bool *active);

// принять state_rsp (multicast)
void logic_on_state_response(uint32_t epoch,
                             const otIp6Address *owner,
                             uint32_t remaining_ms,
                             bool active);

// принять trigger (multicast)
// void logic_on_trigger_rx(uint32_t epoch,
//                          const otIp6Address *src,
//                          uint32_t hold_ms);

void logic_on_trigger_rx(uint32_t epoch,
                         const otIp6Address *from,
                         uint32_t rem_ms);


// принять off (multicast)
void logic_on_off_rx(uint32_t epoch);


void logic_post_state_response(uint32_t epoch, const otIp6Address *owner, uint32_t remaining_ms, bool active);

void logic_post_trigger_rx(uint32_t epoch, const otIp6Address *src, uint32_t hold_ms);

void logic_post_off_rx(uint32_t epoch);


void logic_post_mode_cmd_global(light_mode_t mode);
void logic_post_mode_cmd_zone(uint8_t zone_id, light_mode_t mode);
void logic_post_mode_cmd_node(light_mode_t mode);

void logic_post_mode_clear_global(void);
void logic_post_mode_clear_zone(uint8_t zone_id);
void logic_post_mode_clear_node(void);


#ifdef __cplusplus
}
#endif
