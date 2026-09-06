idf_component_register(
    SRCS "main.cpp" "kb.cpp" "otter.cpp" "settings.cpp" "serial_config.cpp"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash esp_wifi esp_driver_gpio esp_event esp_netif log freertos espressif__cjson esp_driver_uart esp_timer esp_adc
)
