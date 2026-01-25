#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t tfmini_init(void);
bool tfmini_poll_once(uint16_t *dist_cm);

#ifdef __cplusplus
}
#endif
