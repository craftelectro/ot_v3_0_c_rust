#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "config.h"
#include "rgb_led.h"   // light_mode_t
#include "logic.h"     // zone_state_t, logic_get_state

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t io_board_init(void);

void io_board_set_relay(bool on);
bool io_board_get_relay(void);

light_mode_t io_board_read_mode_switch(void);

#ifdef __cplusplus
}
#endif
