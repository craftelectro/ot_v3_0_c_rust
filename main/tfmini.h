#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void tfmini_init(void);
bool tfmini_poll_once(uint16_t *dist_cm);

#ifdef __cplusplus
}
#endif
