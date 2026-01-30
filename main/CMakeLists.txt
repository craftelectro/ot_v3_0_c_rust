# idf_component_register(SRCS "ot_app.c" "logic.c" "io_board.c" "tfmini.c" "coap_if.c" "main_v1.c" "esp_ot_cli.c"
#                        INCLUDE_DIRS ".")


# idf_component_register(
#     SRCS "rgb_led.c"
#     INCLUDE_DIRS "."
#     REQUIRES openthread nvs_flash driver esp_timer led_strip esp_netif
# )
# idf_component_register(
#     SRCS "main.c" "rgb_led.c"
#     INCLUDE_DIRS "."
#     REQUIRES openthread nvs_flash driver esp_timer led_strip esp_netif vfs
# )

idf_component_register(
    SRCS
        "main.c"
        "esp_ot_cli.c"
        "rgb_led.c"
        "io_board.c"
        "tfmini.c"
        "logic.c"
        "logic_cli.c"
        "coap_if.c"
        "ot_app.c"
        "config_store.c"
        "config_portal.c"
    INCLUDE_DIRS "."
    REQUIRES openthread nvs_flash driver esp_timer led_strip esp_netif vfs esp_wifi esp_http_server rust_payload logic_api logic_api
)
