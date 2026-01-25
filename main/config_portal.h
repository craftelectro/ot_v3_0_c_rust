#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void config_portal_start_if_needed(void);
bool config_portal_is_running(void);

#ifdef __cplusplus
}
#endif
