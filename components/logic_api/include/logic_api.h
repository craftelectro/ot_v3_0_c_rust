#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <openthread/ip6.h>

#ifdef __cplusplus
extern "C" {
#endif

void logic_build_state(uint32_t *epoch, otIp6Address *owner, uint32_t *rem_ms, bool *active);

#ifdef __cplusplus
}
#endif
