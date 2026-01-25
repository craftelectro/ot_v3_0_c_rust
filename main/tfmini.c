#include "tfmini.h"
#include "driver/uart.h"
#include "config.h"
#include "esp_log.h"

static const char *TAG = "tfmini";

esp_err_t tfmini_init(void)
{
    uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(UART_PORT, 512, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        return err;
    }

    err = uart_param_config(UART_PORT, &cfg);
    if (err != ESP_OK) {
        return err;
    }

    err = uart_set_pin(UART_PORT,
                       UART_TFMINI_TX,
                       UART_TFMINI_RX,
                       UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        return err;
    }

    return ESP_OK;
}

bool tfmini_poll_once(uint16_t *out_dist_cm)
{
    uint8_t b[32];
    int n = uart_read_bytes(UART_PORT, b, sizeof(b), 10 / portTICK_PERIOD_MS);
    if (n < 9) return false;

    for (int i = 0; i <= n - 9; i++) {
        if (b[i] == 0x59 && b[i + 1] == 0x59) {
            uint16_t dist = b[i + 2] | (b[i + 3] << 8);
            uint8_t sum = 0;
            for (int k = 0; k < 8; k++) sum += b[i + k];
            if (sum == b[i + 8]) {
                if (out_dist_cm) *out_dist_cm = dist;
                return true;
            }
        }
    }
    return false;
}
