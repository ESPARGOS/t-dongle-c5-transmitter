idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    EMBED_FILES
        "espargos_logo.rgb565"
        "home_logo_mask.gray"
    REQUIRES driver esp_event esp_lcd esp_timer esp_wifi lvgl nvs_flash
)
