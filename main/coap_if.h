#pragma once
#include <stdbool.h>
#include <stdint.h>

#include <openthread/instance.h>
#include <openthread/ip6.h>

#include "rgb_led.h"   // light_mode_t


#ifdef __cplusplus
extern "C" {
#endif

void coap_if_register(otInstance *ot);

// multicast SEND
void coap_if_send_state_req(void);
void coap_if_send_state_rsp(uint32_t epoch,
                            const otIp6Address *owner,
                            uint32_t remaining_ms,
                            bool active);

// void coap_if_send_trigger(uint32_t epoch, uint32_t hold_ms);
void coap_if_send_trigger(uint32_t epoch, uint32_t rem_ms);

void coap_if_send_off(uint32_t epoch);

// утилита: получить свой Mesh-Local EID
bool coap_if_get_my_meshlocal_eid(otIp6Address *out);

bool coap_if_thread_ready(void);


void coap_if_send_mode_global(light_mode_t mode);
void coap_if_send_mode_zone(uint8_t zone_id, light_mode_t mode);
void coap_if_send_mode_unicast(const otIp6Address *dst, light_mode_t mode);

void coap_if_send_mode_clear_global(void);
void coap_if_send_mode_clear_zone(uint8_t zone_id);
void coap_if_send_mode_clear_unicast(const otIp6Address *dst);



#ifdef __cplusplus
}
#endif
