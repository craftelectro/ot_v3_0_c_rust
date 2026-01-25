#include "freertos/FreeRTOS.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_vfs_eventfd.h"
#include "nvs_flash.h"

#include "rgb_led.h"
#include "io_board.h"
#include "tfmini.h"
#include "ot_app.h"

void app_main(void)
{
    esp_vfs_eventfd_config_t ev = {.max_fds = 3};

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_vfs_eventfd_register(&ev));

    ESP_ERROR_CHECK(rgb_init());
    io_board_init();
    tfmini_init();

    ot_app_start();
}
