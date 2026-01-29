#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t has_epoch;
    uint8_t has_rem_ms;
    uint8_t has_active;
    uint8_t has_mode;
    uint8_t has_clr;
    uint8_t has_z;
    uint8_t has_m;
    uint8_t _reserved;
    uint32_t epoch;
    uint32_t rem_ms;
    uint32_t active;
    uint32_t mode;
    uint32_t clr;
    uint32_t z;
    uint32_t m;
} rust_parsed_t;

uint32_t rust_parse_payload(const uint8_t *buf, uint32_t len, rust_parsed_t *out);

#ifdef __cplusplus
}
#endif
