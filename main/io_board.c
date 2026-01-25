
#include "config.h"
#include "io_board.h"
#include "driver/gpio.h"

static bool s_relay_on = false;

void io_board_init(void)
{
    gpio_config_t out = {
        .pin_bit_mask = (1ULL << PIN_RELAY),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&out);

#if ROLE_CONTROLLER
    gpio_config_t inp = {
        .pin_bit_mask = (1ULL << PIN_SW_A) | (1ULL << PIN_SW_B),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
    };
    gpio_config(&inp);
#endif
}

void io_board_set_relay(bool on)
{
    s_relay_on = on;
    gpio_set_level(PIN_RELAY, on ? 1 : 0);
}

bool io_board_get_relay(void)
{
    return s_relay_on;
}

light_mode_t io_board_read_mode_switch(void)
{
#if ROLE_CONTROLLER
    int a = gpio_get_level(PIN_SW_A);
    int b = gpio_get_level(PIN_SW_B);

    if (a == 0 && b == 0) return MODE_OFF;
    if (a == 0 && b == 1) return MODE_ON;
    return MODE_AUTO;
#else
    // на исполнителе физического переключателя нет
    extern const zone_state_t *logic_get_state(void);
    return logic_get_state()->mode;
#endif
}
