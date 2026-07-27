#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_st7735.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "csi_tx";
static const char *NVS_NAMESPACE = "csi_tx";
static const char *NVS_CONFIG_KEY = "config";

#define CSI_TX_LCD_HOST SPI2_HOST
#define CSI_TX_LCD_H_RES 80
#define CSI_TX_LCD_V_RES 160
#define CSI_TX_LCD_DRAW_BUF_LINES 20
#define CSI_TX_LCD_PIXEL_CLOCK_HZ (20 * 1000 * 1000)
#define CSI_TX_LCD_CMD_BITS 8
#define CSI_TX_LCD_PARAM_BITS 8
#define CSI_TX_LCD_BL_ON_LEVEL 0
#define CSI_TX_LCD_BL_OFF_LEVEL !CSI_TX_LCD_BL_ON_LEVEL
#define CSI_TX_LCD_PIN_BL GPIO_NUM_0
#define CSI_TX_LCD_PIN_DC GPIO_NUM_3
#define CSI_TX_LCD_PIN_RST GPIO_NUM_1
#define CSI_TX_LCD_PIN_CS GPIO_NUM_10
#define CSI_TX_LCD_PIN_SCLK GPIO_NUM_6
#define CSI_TX_LCD_PIN_MOSI GPIO_NUM_2

#define CSI_TX_BUTTON_PIN GPIO_NUM_28
#define CSI_TX_BUTTON_ACTIVE_LEVEL 0
#define CSI_TX_BUTTON_DEBOUNCE_US 30000
#define CSI_TX_BUTTON_LONG_PRESS_US 600000

#define CSI_TX_LED_SPI_CLOCK_HZ (500 * 1000)
#define CSI_TX_LED_BRIGHTNESS 31

#define CSI_TX_SPLASH_MS 1500
#define CSI_TX_UI_TICK_PERIOD_MS 5
#define CSI_TX_UI_REFRESH_MS 40
#define CSI_TX_STATUS_PERIOD_MS 1000
#define CSI_TX_HOME_LOGO_WIDTH 136
#define CSI_TX_HOME_LOGO_HEIGHT 28
#define CSI_TX_HOME_LOGO_OFFSET_Y 16

typedef enum {
    CSI_TX_MOD_11B,
    CSI_TX_MOD_11G,
    CSI_TX_MOD_HT20,
    CSI_TX_MOD_HT40,
    CSI_TX_MOD_HE20,
} csi_tx_modulation_t;

typedef enum {
    CSI_TX_UI_HOME,
    CSI_TX_UI_MENU,
    CSI_TX_UI_EDIT,
} csi_tx_ui_mode_t;

typedef enum {
    CSI_TX_MENU_CHANNEL,
    CSI_TX_MENU_TX_INDEX,
    CSI_TX_MENU_MODULATION,
    CSI_TX_MENU_RATE,
    CSI_TX_MENU_SECONDARY,
    CSI_TX_MENU_INTERVAL,
    CSI_TX_MENU_TX_POWER,
} csi_tx_menu_item_t;

typedef struct {
    uint8_t frame_control[2];
    uint8_t duration[2];
    uint8_t destination[6];
    uint8_t source[6];
    uint8_t bssid[6];
    uint8_t sequence_control[2];
    uint8_t payload[8];
} __attribute__((packed)) wifi_frame_t;

typedef struct {
    csi_tx_modulation_t modulation;
    uint8_t protocol_bitmap;
    uint8_t channel;
    uint8_t min_channel;
    uint8_t max_channel;
    uint8_t tx_index;
    wifi_bandwidth_t bandwidth;
    wifi_second_chan_t secondary_channel;
    wifi_phy_mode_t phy_mode;
    wifi_phy_rate_t phy_rate;
    uint32_t interval_ms;
    int8_t max_tx_power_qdbm;
    int rate_index;
    uint32_t accent_rgb;
    const char *modulation_name;
    const char *rate_name;
} csi_tx_config_t;

typedef struct {
    csi_tx_config_t active_config;
    csi_tx_config_t edit_config;
    csi_tx_ui_mode_t ui_mode;
    csi_tx_menu_item_t menu_item;
    bool long_press_preview;
    uint8_t current_channel;
    uint32_t tx_count;
    uint32_t tx_failures;
} csi_tx_state_t;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint8_t modulation;
    uint8_t channel;
    uint8_t tx_index;
    int8_t secondary_channel;
    uint32_t interval_ms;
    int8_t max_tx_power_qdbm;
    int32_t rate_index;
} csi_tx_persisted_config_t;

#define CSI_TX_CONFIG_MAGIC 0x43534954UL
#define CSI_TX_CONFIG_VERSION 1U

static const uint32_t s_interval_options_ms[] = {
    1, 2, 5, 7, 10, 11, 13, 15, 16, 20, 25, 50, 100, 200, 500, 1000,
};

static const int8_t s_tx_power_options_qdbm[] = {
    8, 16, 24, 32, 40, 48, 56, 64, 72, 80, 84,
};

static const wifi_phy_rate_t s_rates_11b[] = {
    WIFI_PHY_RATE_1M_L,
    WIFI_PHY_RATE_2M_L,
    WIFI_PHY_RATE_5M_L,
    WIFI_PHY_RATE_11M_L,
};

static const char *const s_rate_names_11b[] = {
    "1M", "2M", "5.5M", "11M",
};

static const wifi_phy_rate_t s_rates_11g[] = {
    WIFI_PHY_RATE_6M,
    WIFI_PHY_RATE_9M,
    WIFI_PHY_RATE_12M,
    WIFI_PHY_RATE_18M,
    WIFI_PHY_RATE_24M,
    WIFI_PHY_RATE_36M,
    WIFI_PHY_RATE_48M,
    WIFI_PHY_RATE_54M,
};

static const char *const s_rate_names_11g[] = {
    "6M", "9M", "12M", "18M", "24M", "36M", "48M", "54M",
};

static const wifi_phy_rate_t s_rates_mcs[] = {
    WIFI_PHY_RATE_MCS0_LGI,
    WIFI_PHY_RATE_MCS1_LGI,
    WIFI_PHY_RATE_MCS2_LGI,
    WIFI_PHY_RATE_MCS3_LGI,
    WIFI_PHY_RATE_MCS4_LGI,
    WIFI_PHY_RATE_MCS5_LGI,
    WIFI_PHY_RATE_MCS6_LGI,
    WIFI_PHY_RATE_MCS7_LGI,
#if CONFIG_SOC_WIFI_HE_SUPPORT
    WIFI_PHY_RATE_MCS8_LGI,
    WIFI_PHY_RATE_MCS9_LGI,
#endif
};

static const char *const s_rate_names_mcs[] = {
    "MCS0", "MCS1", "MCS2", "MCS3", "MCS4",
    "MCS5", "MCS6", "MCS7",
#if CONFIG_SOC_WIFI_HE_SUPPORT
    "MCS8", "MCS9",
#endif
};

static const wifi_frame_t s_frame_template = {
    .frame_control = {0x08, 0x00},
    .duration = {0x00, 0x00},
    .destination = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
    .payload = {0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef},
};

static wifi_frame_t s_frame;
static csi_tx_state_t s_state;
static uint8_t s_base_mac[6];
static bool s_wifi_ready;
static bool s_led_ready;
static portMUX_TYPE s_state_lock = portMUX_INITIALIZER_UNLOCKED;

static esp_lcd_panel_io_handle_t s_lcd_io;
static esp_lcd_panel_handle_t s_lcd_panel;
static lv_display_t *s_lv_display;
static lv_obj_t *s_home_screen;
static lv_obj_t *s_home_logo;
static lv_obj_t *s_home_status;
static lv_obj_t *s_menu_screen;
static lv_obj_t *s_menu_header;
static lv_obj_t *s_menu_body;
static esp_timer_handle_t s_lvgl_tick_timer;
static esp_timer_handle_t s_tx_timer;
static spi_device_handle_t s_led_strip;
static uint32_t s_tx_timer_interval_ms;

extern const uint8_t _binary_espargos_logo_rgb565_start[] asm("_binary_espargos_logo_rgb565_start");
extern const uint8_t _binary_espargos_logo_rgb565_end[] asm("_binary_espargos_logo_rgb565_end");
extern const uint8_t _binary_home_logo_mask_gray_start[] asm("_binary_home_logo_mask_gray_start");
extern const uint8_t _binary_home_logo_mask_gray_end[] asm("_binary_home_logo_mask_gray_end");

static uint8_t s_home_logo_pixels[CSI_TX_HOME_LOGO_WIDTH * CSI_TX_HOME_LOGO_HEIGHT * 3];
static const lv_image_dsc_t s_home_logo_image = {
    .header.cf = LV_COLOR_FORMAT_RGB565A8,
    .header.magic = LV_IMAGE_HEADER_MAGIC,
    .header.w = CSI_TX_HOME_LOGO_WIDTH,
    .header.h = CSI_TX_HOME_LOGO_HEIGHT,
    .header.stride = CSI_TX_HOME_LOGO_WIDTH * 2,
    .data_size = sizeof(s_home_logo_pixels),
    .data = (const uint8_t *)s_home_logo_pixels,
};

static uint32_t csi_tx_get_tx_color(uint8_t tx_index)
{
    static const uint32_t colors[] = {
        0x1f77b4, 0xff7f0e, 0x2ca02c, 0xd62728, 0x9467bd,
    };

    if (tx_index < 1 || tx_index > sizeof(colors) / sizeof(colors[0])) {
        return colors[0];
    }

    return colors[tx_index - 1U];
}

static void csi_tx_prepare_home_logo(void)
{
    const uint8_t *mask = _binary_home_logo_mask_gray_start;
    size_t mask_size = (size_t)(_binary_home_logo_mask_gray_end - _binary_home_logo_mask_gray_start);
    size_t pixel_count = CSI_TX_HOME_LOGO_WIDTH * CSI_TX_HOME_LOGO_HEIGHT;
    uint8_t *color_plane = s_home_logo_pixels;
    uint8_t *alpha_plane = s_home_logo_pixels + (pixel_count * 2);

    if (mask_size != pixel_count) {
        ESP_LOGW(TAG, "unexpected home logo mask size: %u", (unsigned)mask_size);
        return;
    }

    for (size_t i = 0; i < pixel_count; ++i) {
        color_plane[(i * 2) + 0] = 0xff;
        color_plane[(i * 2) + 1] = 0xff;
    }

    memcpy(alpha_plane, mask, pixel_count);
}

static bool csi_tx_has_secondary_channel(csi_tx_modulation_t modulation)
{
    return modulation == CSI_TX_MOD_HT40;
}

static const char *csi_tx_get_modulation_name(csi_tx_modulation_t modulation)
{
    switch (modulation) {
    case CSI_TX_MOD_11B:
        return "11b";
    case CSI_TX_MOD_11G:
        return "11g";
    case CSI_TX_MOD_HT20:
        return "11n HT20";
    case CSI_TX_MOD_HT40:
        return "11n HT40";
    case CSI_TX_MOD_HE20:
        return "11ax HE20";
    default:
        return "11n HT20";
    }
}

static void csi_tx_get_rate_table(csi_tx_modulation_t modulation,
                                  const wifi_phy_rate_t **rates,
                                  const char *const **names,
                                  size_t *count)
{
    const wifi_phy_rate_t *selected_rates;
    const char *const *selected_names;
    size_t selected_count;

    switch (modulation) {
    case CSI_TX_MOD_11B:
        selected_rates = s_rates_11b;
        selected_names = s_rate_names_11b;
        selected_count = sizeof(s_rates_11b) / sizeof(s_rates_11b[0]);
        break;
    case CSI_TX_MOD_11G:
        selected_rates = s_rates_11g;
        selected_names = s_rate_names_11g;
        selected_count = sizeof(s_rates_11g) / sizeof(s_rates_11g[0]);
        break;
    case CSI_TX_MOD_HT20:
    case CSI_TX_MOD_HT40:
    case CSI_TX_MOD_HE20:
    default:
        selected_rates = s_rates_mcs;
        selected_names = s_rate_names_mcs;
        selected_count = sizeof(s_rates_mcs) / sizeof(s_rates_mcs[0]);
        break;
    }

    if (rates != NULL) {
        *rates = selected_rates;
    }
    if (names != NULL) {
        *names = selected_names;
    }
    if (count != NULL) {
        *count = selected_count;
    }
}

static uint8_t csi_tx_clamp_channel(uint8_t channel, uint8_t min_channel, uint8_t max_channel)
{
    if (channel < min_channel) {
        return min_channel;
    }
    if (channel > max_channel) {
        return max_channel;
    }
    return channel;
}

static void csi_tx_resolve_config(csi_tx_config_t *config)
{
    const wifi_phy_rate_t *rate_table = NULL;
    const char *const *rate_names = NULL;
    size_t rate_count = 0;

    csi_tx_get_rate_table(config->modulation, &rate_table, &rate_names, &rate_count);

    config->protocol_bitmap = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N;
    config->bandwidth = WIFI_BW20;
    config->secondary_channel = csi_tx_has_secondary_channel(config->modulation)
        ? config->secondary_channel
        : WIFI_SECOND_CHAN_NONE;
    config->phy_mode = WIFI_PHY_MODE_HT20;
    config->min_channel = 1;
    config->max_channel = 13;

    if (config->tx_index < 1 || config->tx_index > 5) {
        config->tx_index = 1;
    }
    if (config->interval_ms == 0) {
        config->interval_ms = 1;
    }
    if (rate_count == 0) {
        rate_count = 1;
    }
    if (config->rate_index < 0 || (size_t)config->rate_index >= rate_count) {
        config->rate_index = 0;
    }

    switch (config->modulation) {
    case CSI_TX_MOD_11B:
        config->protocol_bitmap = WIFI_PROTOCOL_11B;
        config->phy_mode = WIFI_PHY_MODE_11B;
        break;
    case CSI_TX_MOD_11G:
        config->protocol_bitmap = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G;
        config->phy_mode = WIFI_PHY_MODE_11G;
        break;
    case CSI_TX_MOD_HT40:
        config->bandwidth = WIFI_BW40;
        config->phy_mode = WIFI_PHY_MODE_HT40;
        if (config->secondary_channel != WIFI_SECOND_CHAN_BELOW) {
            config->secondary_channel = WIFI_SECOND_CHAN_ABOVE;
        }
        config->min_channel = config->secondary_channel == WIFI_SECOND_CHAN_ABOVE ? 1 : 5;
        config->max_channel = config->secondary_channel == WIFI_SECOND_CHAN_ABOVE ? 9 : 13;
        break;
    case CSI_TX_MOD_HE20:
        config->protocol_bitmap |= WIFI_PROTOCOL_11AX;
        config->phy_mode = WIFI_PHY_MODE_HE20;
        break;
    case CSI_TX_MOD_HT20:
    default:
        config->phy_mode = WIFI_PHY_MODE_HT20;
        break;
    }

    config->channel = csi_tx_clamp_channel(config->channel, config->min_channel, config->max_channel);

    config->phy_rate = rate_table[config->rate_index];
    config->modulation_name = csi_tx_get_modulation_name(config->modulation);
    config->rate_name = rate_names[config->rate_index];
    config->accent_rgb = csi_tx_get_tx_color(config->tx_index);
}

static csi_tx_modulation_t csi_tx_get_default_modulation(void)
{
#if CONFIG_CSI_TX_MODULATION_11B
    return CSI_TX_MOD_11B;
#elif CONFIG_CSI_TX_MODULATION_11G
    return CSI_TX_MOD_11G;
#elif CONFIG_CSI_TX_MODULATION_HT40
    return CSI_TX_MOD_HT40;
#elif CONFIG_CSI_TX_MODULATION_HE20
    return CSI_TX_MOD_HE20;
#else
    return CSI_TX_MOD_HT20;
#endif
}

static csi_tx_config_t csi_tx_get_default_config(void)
{
    csi_tx_config_t config = {
        .modulation = csi_tx_get_default_modulation(),
        .channel = CONFIG_CSI_TX_WIFI_CHANNEL,
        .tx_index = CONFIG_CSI_TX_INDEX,
        .secondary_channel = WIFI_SECOND_CHAN_NONE,
        .interval_ms = CONFIG_CSI_TX_INTERVAL_MS,
        .max_tx_power_qdbm = CONFIG_CSI_TX_MAX_TX_POWER_QDBM,
        .rate_index = CONFIG_CSI_TX_RATE_INDEX,
    };

#if CONFIG_CSI_TX_MODULATION_HT40
    config.secondary_channel = CONFIG_CSI_TX_WIFI_SECONDARY_CHANNEL_ABOVE
        ? WIFI_SECOND_CHAN_ABOVE
        : WIFI_SECOND_CHAN_BELOW;
#endif

    csi_tx_resolve_config(&config);
    return config;
}

static csi_tx_persisted_config_t csi_tx_to_persisted_config(const csi_tx_config_t *config)
{
    csi_tx_persisted_config_t persisted = {
        .magic = CSI_TX_CONFIG_MAGIC,
        .version = CSI_TX_CONFIG_VERSION,
        .modulation = (uint8_t)config->modulation,
        .channel = config->channel,
        .tx_index = config->tx_index,
        .secondary_channel = (int8_t)config->secondary_channel,
        .interval_ms = config->interval_ms,
        .max_tx_power_qdbm = config->max_tx_power_qdbm,
        .rate_index = config->rate_index,
    };

    return persisted;
}

static void csi_tx_apply_persisted_config(csi_tx_config_t *config, const csi_tx_persisted_config_t *persisted)
{
    config->modulation = (csi_tx_modulation_t)persisted->modulation;
    config->channel = persisted->channel;
    config->tx_index = persisted->tx_index;
    config->secondary_channel = (wifi_second_chan_t)persisted->secondary_channel;
    config->interval_ms = persisted->interval_ms;
    config->max_tx_power_qdbm = persisted->max_tx_power_qdbm;
    config->rate_index = persisted->rate_index;
}

static esp_err_t csi_tx_load_config_from_nvs(csi_tx_config_t *config)
{
    nvs_handle_t nvs_handle;
    csi_tx_persisted_config_t persisted;
    size_t size = sizeof(persisted);
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return err;
    }
    ESP_RETURN_ON_ERROR(err, TAG, "failed to open NVS namespace");

    err = nvs_get_blob(nvs_handle, NVS_CONFIG_KEY, &persisted, &size);
    nvs_close(nvs_handle);

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return err;
    }
    ESP_RETURN_ON_ERROR(err, TAG, "failed to read config from NVS");

    if (size != sizeof(persisted) ||
        persisted.magic != CSI_TX_CONFIG_MAGIC ||
        persisted.version != CSI_TX_CONFIG_VERSION) {
        ESP_LOGW(TAG, "ignoring invalid persisted config");
        return ESP_ERR_INVALID_VERSION;
    }

    csi_tx_apply_persisted_config(config, &persisted);
    csi_tx_resolve_config(config);
    ESP_LOGI(TAG, "loaded config from NVS");
    return ESP_OK;
}

static esp_err_t csi_tx_save_config_to_nvs(const csi_tx_config_t *config)
{
    nvs_handle_t nvs_handle;
    csi_tx_persisted_config_t persisted = csi_tx_to_persisted_config(config);

    ESP_RETURN_ON_ERROR(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle), TAG, "failed to open NVS namespace");
    esp_err_t err = nvs_set_blob(nvs_handle, NVS_CONFIG_KEY, &persisted, sizeof(persisted));
    if (err == ESP_OK) {
        err = nvs_commit(nvs_handle);
    }
    nvs_close(nvs_handle);
    ESP_RETURN_ON_ERROR(err, TAG, "failed to save config to NVS");

    return ESP_OK;
}

static csi_tx_state_t csi_tx_get_state_copy(void)
{
    csi_tx_state_t snapshot;

    taskENTER_CRITICAL(&s_state_lock);
    snapshot = s_state;
    taskEXIT_CRITICAL(&s_state_lock);

    return snapshot;
}

static csi_tx_config_t csi_tx_get_active_config(void)
{
    csi_tx_config_t config;

    taskENTER_CRITICAL(&s_state_lock);
    config = s_state.active_config;
    taskEXIT_CRITICAL(&s_state_lock);

    return config;
}

static void csi_tx_set_runtime_stats(uint32_t tx_count, uint32_t tx_failures, uint8_t channel)
{
    taskENTER_CRITICAL(&s_state_lock);
    s_state.tx_count = tx_count;
    s_state.tx_failures = tx_failures;
    s_state.current_channel = channel;
    taskEXIT_CRITICAL(&s_state_lock);
}

static esp_err_t csi_tx_init_nvs(void)
{
    esp_err_t err = nvs_flash_init();

    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "failed to erase NVS");
        err = nvs_flash_init();
    }

    return err;
}

static void csi_tx_prepare_frame(void)
{
    s_frame = s_frame_template;
    ESP_ERROR_CHECK(esp_read_mac(s_base_mac, ESP_MAC_WIFI_STA));
}

static void csi_tx_update_frame_identity(uint8_t tx_index)
{
    memcpy(s_frame.source, s_base_mac, sizeof(s_frame.source));
    memcpy(s_frame.bssid, s_base_mac, sizeof(s_frame.bssid));
    s_frame.source[5] = tx_index;
    s_frame.bssid[5] = tx_index;
}

static esp_err_t csi_tx_apply_channel(uint8_t channel, wifi_second_chan_t secondary_channel)
{
    ESP_RETURN_ON_ERROR(esp_wifi_set_channel(channel, secondary_channel), TAG, "failed to set channel");
    csi_tx_set_runtime_stats(s_state.tx_count, s_state.tx_failures, channel);
    return ESP_OK;
}

static esp_err_t csi_tx_apply_wifi_config(const csi_tx_config_t *config)
{
    wifi_tx_rate_config_t rate_config = {
        .phymode = config->phy_mode,
        .rate = config->phy_rate,
        .ersu = false,
        .dcm = false,
    };

    ESP_RETURN_ON_ERROR(esp_wifi_set_ps(WIFI_PS_NONE), TAG, "failed to disable power save");
    ESP_RETURN_ON_ERROR(esp_wifi_set_max_tx_power(config->max_tx_power_qdbm), TAG, "failed to set TX power");
    ESP_RETURN_ON_ERROR(esp_wifi_set_protocol(WIFI_IF_STA, config->protocol_bitmap), TAG, "failed to set protocol");
    ESP_RETURN_ON_ERROR(esp_wifi_set_bandwidth(WIFI_IF_STA, config->bandwidth), TAG, "failed to set bandwidth");
    ESP_RETURN_ON_ERROR(csi_tx_apply_channel(config->channel, config->secondary_channel), TAG, "failed to set channel");
    ESP_RETURN_ON_ERROR(esp_wifi_config_80211_tx(WIFI_IF_STA, &rate_config), TAG, "failed to set fixed TX config");
    return ESP_OK;
}

static esp_err_t csi_tx_init_wifi(void)
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    wifi_country_t country = {
        .cc = "DE",
        .schan = 1,
        .nchan = 13,
        .policy = WIFI_COUNTRY_POLICY_MANUAL,
    };
    wifi_config_t wifi_config = {0};

    cfg.ampdu_tx_enable = false;
    cfg.sta_disconnected_pm = false;

    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "failed to create event loop");
    ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), TAG, "failed to init Wi-Fi");
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "failed to set storage");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "failed to set station mode");
    ESP_RETURN_ON_ERROR(esp_wifi_set_country(&country), TAG, "failed to set country");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &wifi_config), TAG, "failed to set STA config");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "failed to start Wi-Fi");
    ESP_RETURN_ON_ERROR(esp_wifi_set_band_mode(WIFI_BAND_MODE_2G_ONLY), TAG, "failed to force 2.4 GHz band");
    ESP_RETURN_ON_ERROR(csi_tx_apply_wifi_config(&s_state.active_config), TAG, "failed to apply Wi-Fi config");

    s_wifi_ready = true;
    return ESP_OK;
}

static void csi_tx_log_config(const csi_tx_config_t *config)
{
    ESP_LOGI(TAG,
             "ready: mac=%02x:%02x:%02x:%02x:%02x:%02x tx=%u channel=%u modulation=%s rate=%s interval=%" PRIu32 "ms",
             s_frame.source[0], s_frame.source[1], s_frame.source[2],
             s_frame.source[3], s_frame.source[4], s_frame.source[5],
             config->tx_index,
             config->channel,
             config->modulation_name,
             config->rate_name,
             config->interval_ms);
}

static esp_err_t csi_tx_init_led(void)
{
    spi_device_interface_config_t led_config = {
        .clock_speed_hz = CSI_TX_LED_SPI_CLOCK_HZ,
        .mode = 0,
        .spics_io_num = -1,
        .queue_size = 1,
    };

    ESP_RETURN_ON_ERROR(spi_bus_add_device(CSI_TX_LCD_HOST, &led_config, &s_led_strip), TAG, "failed to add LED device");
    s_led_ready = true;
    return ESP_OK;
}

static void csi_tx_set_led_rgb(uint32_t rgb)
{
    uint8_t buffer[12] = {
        0x00, 0x00, 0x00, 0x00,
        0xff, 0x00, 0x00, 0x00,
        0xff, 0xff, 0xff, 0xff,
    };
    spi_transaction_t transaction = {
        .length = sizeof(buffer) * 8,
        .tx_buffer = buffer,
    };

    if (!s_led_ready) {
        return;
    }

    buffer[4] = 0b11100000 | (CSI_TX_LED_BRIGHTNESS & 0x1f);
    buffer[5] = (uint8_t)(rgb & 0xff);
    buffer[6] = (uint8_t)((rgb >> 8) & 0xff);
    buffer[7] = (uint8_t)((rgb >> 16) & 0xff);

    if (spi_device_transmit(s_led_strip, &transaction) != ESP_OK) {
        ESP_LOGW(TAG, "failed to update APA102 LED");
    }
}

static esp_err_t csi_tx_apply_runtime_config(csi_tx_config_t config)
{
    csi_tx_config_t previous_config;
    esp_err_t persist_err;

    csi_tx_resolve_config(&config);

    if (s_wifi_ready) {
        previous_config = csi_tx_get_active_config();
        esp_err_t err = csi_tx_apply_wifi_config(&config);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "failed to apply Wi-Fi config: %s; restoring previous config",
                     esp_err_to_name(err));
            esp_err_t rollback_err = csi_tx_apply_wifi_config(&previous_config);
            if (rollback_err != ESP_OK) {
                ESP_LOGE(TAG, "failed to restore previous Wi-Fi config: %s",
                         esp_err_to_name(rollback_err));
            }
            return err;
        }
    } else {
        csi_tx_set_runtime_stats(s_state.tx_count, s_state.tx_failures, config.channel);
    }

    csi_tx_update_frame_identity(config.tx_index);
    csi_tx_set_led_rgb(config.accent_rgb);

    taskENTER_CRITICAL(&s_state_lock);
    s_state.active_config = config;
    s_state.edit_config = config;
    taskEXIT_CRITICAL(&s_state_lock);

    persist_err = csi_tx_save_config_to_nvs(&config);
    if (persist_err != ESP_OK) {
        ESP_LOGW(TAG, "applied config but failed to persist it: %s", esp_err_to_name(persist_err));
    }

    ESP_LOGI(TAG,
             "applied: tx=%u channel=%u modulation=%s rate=%s interval=%" PRIu32 "ms",
             config.tx_index,
             config.channel,
             config.modulation_name,
             config.rate_name,
             config.interval_ms);
    return ESP_OK;
}

static bool csi_tx_button_pressed(void)
{
    return gpio_get_level(CSI_TX_BUTTON_PIN) == CSI_TX_BUTTON_ACTIVE_LEVEL;
}

static esp_err_t csi_tx_init_button(void)
{
    gpio_config_t io_config = {
        .pin_bit_mask = 1ULL << CSI_TX_BUTTON_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    return gpio_config(&io_config);
}

static csi_tx_menu_item_t csi_tx_get_next_menu_item(csi_tx_menu_item_t item, const csi_tx_config_t *config)
{
    csi_tx_menu_item_t next = item;

    do {
        next = (csi_tx_menu_item_t)(((int)next + 1) % ((int)CSI_TX_MENU_TX_POWER + 1));
    } while (next == CSI_TX_MENU_SECONDARY && !csi_tx_has_secondary_channel(config->modulation));

    return next;
}

static uint8_t csi_tx_get_menu_item_count(const csi_tx_config_t *config)
{
    return csi_tx_has_secondary_channel(config->modulation) ? 7 : 6;
}

static uint8_t csi_tx_get_menu_item_position(csi_tx_menu_item_t item, const csi_tx_config_t *config)
{
    csi_tx_menu_item_t cursor = CSI_TX_MENU_CHANNEL;
    uint8_t index = 1;

    while (cursor != item && index < 7) {
        cursor = csi_tx_get_next_menu_item(cursor, config);
        index++;
    }

    return index;
}

static csi_tx_modulation_t csi_tx_get_next_modulation(csi_tx_modulation_t modulation)
{
    static const csi_tx_modulation_t modulations[] = {
        CSI_TX_MOD_11B,
        CSI_TX_MOD_11G,
        CSI_TX_MOD_HT20,
        CSI_TX_MOD_HT40,
#if CONFIG_SOC_WIFI_HE_SUPPORT
        CSI_TX_MOD_HE20,
#endif
    };
    size_t count = sizeof(modulations) / sizeof(modulations[0]);

    for (size_t i = 0; i < count; ++i) {
        if (modulations[i] == modulation) {
            return modulations[(i + 1U) % count];
        }
    }

    return modulations[0];
}

static uint32_t csi_tx_get_next_interval_ms(uint32_t value)
{
    size_t count = sizeof(s_interval_options_ms) / sizeof(s_interval_options_ms[0]);

    for (size_t i = 0; i < count; ++i) {
        if (s_interval_options_ms[i] >= value) {
            return s_interval_options_ms[(i + 1U) % count];
        }
    }

    return s_interval_options_ms[0];
}

static int8_t csi_tx_get_next_tx_power_qdbm(int8_t value)
{
    size_t count = sizeof(s_tx_power_options_qdbm) / sizeof(s_tx_power_options_qdbm[0]);

    for (size_t i = 0; i < count; ++i) {
        if (s_tx_power_options_qdbm[i] >= value) {
            return s_tx_power_options_qdbm[(i + 1U) % count];
        }
    }

    return s_tx_power_options_qdbm[0];
}

static void csi_tx_cycle_menu_value(csi_tx_config_t *config, csi_tx_menu_item_t item)
{
    size_t rate_count = 0;

    switch (item) {
    case CSI_TX_MENU_CHANNEL:
        config->channel++;
        if (config->channel > config->max_channel || config->channel < config->min_channel) {
            config->channel = config->min_channel;
        }
        break;
    case CSI_TX_MENU_TX_INDEX:
        config->tx_index = (uint8_t)((config->tx_index % 5U) + 1U);
        break;
    case CSI_TX_MENU_MODULATION:
        config->modulation = csi_tx_get_next_modulation(config->modulation);
        config->rate_index = 0;
        if (config->modulation == CSI_TX_MOD_HT40 && config->secondary_channel == WIFI_SECOND_CHAN_NONE) {
            config->secondary_channel = config->channel > 9
                ? WIFI_SECOND_CHAN_BELOW
                : WIFI_SECOND_CHAN_ABOVE;
        }
        break;
    case CSI_TX_MENU_RATE:
        csi_tx_get_rate_table(config->modulation, NULL, NULL, &rate_count);
        config->rate_index = (config->rate_index + 1) % (int)rate_count;
        break;
    case CSI_TX_MENU_SECONDARY:
        if (csi_tx_has_secondary_channel(config->modulation)) {
            config->secondary_channel = config->secondary_channel == WIFI_SECOND_CHAN_BELOW
                ? WIFI_SECOND_CHAN_ABOVE
                : WIFI_SECOND_CHAN_BELOW;
        }
        break;
    case CSI_TX_MENU_INTERVAL:
        config->interval_ms = csi_tx_get_next_interval_ms(config->interval_ms);
        break;
    case CSI_TX_MENU_TX_POWER:
        config->max_tx_power_qdbm = csi_tx_get_next_tx_power_qdbm(config->max_tx_power_qdbm);
        break;
    }

    csi_tx_resolve_config(config);
}

static bool csi_tx_menu_item_is_last(csi_tx_menu_item_t item, const csi_tx_config_t *config)
{
    return item == CSI_TX_MENU_TX_POWER ||
           (item == CSI_TX_MENU_INTERVAL && !csi_tx_has_secondary_channel(config->modulation));
}

static const char *csi_tx_get_menu_title(csi_tx_menu_item_t item)
{
    switch (item) {
    case CSI_TX_MENU_CHANNEL:
        return "Channel";
    case CSI_TX_MENU_TX_INDEX:
        return "TX Index";
    case CSI_TX_MENU_MODULATION:
        return "Modulation";
    case CSI_TX_MENU_RATE:
        return "Rate";
    case CSI_TX_MENU_SECONDARY:
        return "Secondary";
    case CSI_TX_MENU_INTERVAL:
        return "Interval";
    case CSI_TX_MENU_TX_POWER:
        return "TX Power";
    default:
        return "Setup";
    }
}

static void csi_tx_format_tx_power(char *buffer, size_t size, int8_t power_qdbm)
{
    int whole_dbm = power_qdbm / 4;
    int frac_dbm = (power_qdbm % 4) * 25;

    snprintf(buffer, size, "%d.%02ddBm", whole_dbm, frac_dbm);
}

static void csi_tx_format_status(char *buffer, size_t size, const csi_tx_config_t *config)
{
    char power[16];

    csi_tx_format_tx_power(power, sizeof(power), config->max_tx_power_qdbm);

    if (config->bandwidth == WIFI_BW40) {
        snprintf(buffer,
                 size,
                 "TX %u | CH %u | %s | %s | SEC %s | INT %" PRIu32 "ms | PWR %s",
                 config->tx_index,
                 config->channel,
                 config->modulation_name,
                 config->rate_name,
                 config->secondary_channel == WIFI_SECOND_CHAN_ABOVE ? "ABOVE" : "BELOW",
                 config->interval_ms,
                 power);
        return;
    }

    snprintf(buffer,
             size,
             "TX %u | CH %u | %s | %s | INT %" PRIu32 "ms | PWR %s",
             config->tx_index,
             config->channel,
             config->modulation_name,
             config->rate_name,
             config->interval_ms,
             power);
}

static void csi_tx_format_menu_value(char *buffer,
                                     size_t size,
                                     csi_tx_menu_item_t item,
                                     const csi_tx_config_t *config)
{
    switch (item) {
    case CSI_TX_MENU_CHANNEL:
        snprintf(buffer, size, "%u", config->channel);
        break;
    case CSI_TX_MENU_TX_INDEX:
        snprintf(buffer, size, "%u", config->tx_index);
        break;
    case CSI_TX_MENU_MODULATION:
        snprintf(buffer, size, "%s", config->modulation_name);
        break;
    case CSI_TX_MENU_RATE:
        snprintf(buffer, size, "%s", config->rate_name);
        break;
    case CSI_TX_MENU_SECONDARY:
        snprintf(buffer, size, "%s",
                 config->secondary_channel == WIFI_SECOND_CHAN_ABOVE ? "ABOVE" : "BELOW");
        break;
    case CSI_TX_MENU_INTERVAL:
        snprintf(buffer, size, "%" PRIu32 "ms", config->interval_ms);
        break;
    case CSI_TX_MENU_TX_POWER:
        csi_tx_format_tx_power(buffer, size, config->max_tx_power_qdbm);
        break;
    default:
        buffer[0] = '\0';
        break;
    }
}

static bool csi_tx_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io,
                                    esp_lcd_panel_io_event_data_t *edata,
                                    void *user_ctx)
{
    LV_UNUSED(panel_io);
    LV_UNUSED(edata);
    lv_display_flush_ready(user_ctx);
    return false;
}

static void csi_tx_lvgl_flush_cb(lv_display_t *display, const lv_area_t *area, uint8_t *px_map)
{
    LV_UNUSED(display);
    lv_draw_sw_rgb565_swap(px_map, lv_area_get_width(area) * lv_area_get_height(area));
    esp_lcd_panel_draw_bitmap(s_lcd_panel, area->x1, area->y1, area->x2 + 1, area->y2 + 1, px_map);
}

static void csi_tx_lvgl_tick_cb(void *arg)
{
    LV_UNUSED(arg);
    lv_tick_inc(CSI_TX_UI_TICK_PERIOD_MS);
}

static void csi_tx_tx_timer_cb(void *arg)
{
    TaskHandle_t task_handle = (TaskHandle_t)arg;

    if (task_handle != NULL) {
        xTaskNotifyGive(task_handle);
    }
}

static esp_err_t csi_tx_init_tx_timer(TaskHandle_t task_handle, uint32_t interval_ms)
{
    const esp_timer_create_args_t timer_args = {
        .callback = csi_tx_tx_timer_cb,
        .arg = task_handle,
        .name = "tx_tick",
    };

    if (s_tx_timer != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(esp_timer_create(&timer_args, &s_tx_timer), TAG, "failed to create TX timer");
    ESP_RETURN_ON_ERROR(esp_timer_start_periodic(s_tx_timer, interval_ms * 1000ULL), TAG, "failed to start TX timer");
    s_tx_timer_interval_ms = interval_ms;
    return ESP_OK;
}

static esp_err_t csi_tx_update_tx_timer_interval(uint32_t interval_ms)
{
    if (s_tx_timer == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_tx_timer_interval_ms == interval_ms && esp_timer_is_active(s_tx_timer)) {
        return ESP_OK;
    }

    if (esp_timer_is_active(s_tx_timer)) {
        ESP_RETURN_ON_ERROR(esp_timer_stop(s_tx_timer), TAG, "failed to stop TX timer");
    }

    ESP_RETURN_ON_ERROR(esp_timer_start_periodic(s_tx_timer, interval_ms * 1000ULL),
                        TAG,
                        "failed to start TX timer");
    s_tx_timer_interval_ms = interval_ms;
    return ESP_OK;
}

static void csi_tx_show_splash(void)
{
    size_t image_size = (size_t)(_binary_espargos_logo_rgb565_end - _binary_espargos_logo_rgb565_start);

    if (image_size != 160U * 80U * 2U) {
        ESP_LOGW(TAG, "unexpected splash size: %u", (unsigned)image_size);
        return;
    }

    esp_lcd_panel_draw_bitmap(s_lcd_panel, 0, 0, CSI_TX_LCD_V_RES, CSI_TX_LCD_H_RES, _binary_espargos_logo_rgb565_start);
    vTaskDelay(pdMS_TO_TICKS(CSI_TX_SPLASH_MS));
}

static esp_err_t csi_tx_init_display(void)
{
    const spi_bus_config_t buscfg = {
        .sclk_io_num = CSI_TX_LCD_PIN_SCLK,
        .mosi_io_num = CSI_TX_LCD_PIN_MOSI,
        .miso_io_num = GPIO_NUM_NC,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = CSI_TX_LCD_H_RES * CSI_TX_LCD_DRAW_BUF_LINES * sizeof(lv_color16_t),
    };
    const gpio_config_t backlight_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << CSI_TX_LCD_PIN_BL,
    };
    const esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = CSI_TX_LCD_PIN_DC,
        .cs_gpio_num = CSI_TX_LCD_PIN_CS,
        .pclk_hz = CSI_TX_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = CSI_TX_LCD_CMD_BITS,
        .lcd_param_bits = CSI_TX_LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = CSI_TX_LCD_PIN_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    esp_lcd_panel_io_callbacks_t io_callbacks = {
        .on_color_trans_done = csi_tx_lvgl_flush_ready,
    };
    esp_timer_create_args_t tick_timer_args = {
        .callback = csi_tx_lvgl_tick_cb,
        .name = "lvgl_tick",
    };
    size_t draw_buffer_sz = CSI_TX_LCD_H_RES * CSI_TX_LCD_DRAW_BUF_LINES * sizeof(lv_color16_t);
    void *buf1 = NULL;
    void *buf2 = NULL;

    ESP_RETURN_ON_ERROR(gpio_config(&backlight_config), TAG, "failed to configure backlight");
    ESP_RETURN_ON_ERROR(gpio_set_level(CSI_TX_LCD_PIN_BL, CSI_TX_LCD_BL_OFF_LEVEL), TAG, "failed to disable backlight");
    ESP_RETURN_ON_ERROR(spi_bus_initialize(CSI_TX_LCD_HOST, &buscfg, SPI_DMA_CH_AUTO), TAG, "failed to init SPI bus");
    ESP_RETURN_ON_ERROR(csi_tx_init_led(), TAG, "failed to init LED");
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)CSI_TX_LCD_HOST, &io_config, &s_lcd_io),
                        TAG, "failed to create panel IO");
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_st7735(s_lcd_io, &panel_config, &s_lcd_panel),
                        TAG, "failed to create ST7735 panel");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(s_lcd_panel), TAG, "failed to reset panel");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(s_lcd_panel), TAG, "failed to init panel");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_invert_color(s_lcd_panel, true), TAG, "failed to invert panel colors");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_swap_xy(s_lcd_panel, true), TAG, "failed to rotate panel axes");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_mirror(s_lcd_panel, true, false), TAG, "failed to mirror panel");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_set_gap(s_lcd_panel, 1, 26), TAG, "failed to set panel gap");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(s_lcd_panel, true), TAG, "failed to enable panel");

    lv_init();
    s_lv_display = lv_display_create(CSI_TX_LCD_H_RES, CSI_TX_LCD_V_RES);

    buf1 = spi_bus_dma_memory_alloc(CSI_TX_LCD_HOST, draw_buffer_sz, 0);
    buf2 = spi_bus_dma_memory_alloc(CSI_TX_LCD_HOST, draw_buffer_sz, 0);
    if (buf1 == NULL || buf2 == NULL) {
        return ESP_ERR_NO_MEM;
    }

    lv_display_set_buffers(s_lv_display, buf1, buf2, draw_buffer_sz, LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_color_format(s_lv_display, LV_COLOR_FORMAT_RGB565);
    lv_display_set_rotation(s_lv_display, LV_DISPLAY_ROTATION_90);
    lv_display_set_flush_cb(s_lv_display, csi_tx_lvgl_flush_cb);
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_register_event_callbacks(s_lcd_io, &io_callbacks, s_lv_display),
                        TAG, "failed to register panel callbacks");
    ESP_RETURN_ON_ERROR(esp_timer_create(&tick_timer_args, &s_lvgl_tick_timer), TAG, "failed to create LVGL tick timer");
    ESP_RETURN_ON_ERROR(esp_timer_start_periodic(s_lvgl_tick_timer, CSI_TX_UI_TICK_PERIOD_MS * 1000ULL),
                        TAG, "failed to start LVGL tick timer");

    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_RETURN_ON_ERROR(gpio_set_level(CSI_TX_LCD_PIN_BL, CSI_TX_LCD_BL_ON_LEVEL), TAG, "failed to enable backlight");
    csi_tx_show_splash();
    return ESP_OK;
}

static void csi_tx_init_ui(void)
{
    lv_obj_t *screen = lv_display_get_screen_active(s_lv_display);

    lv_obj_set_style_bg_color(screen, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);

    s_home_screen = lv_obj_create(screen);
    lv_obj_remove_style_all(s_home_screen);
    lv_obj_set_size(s_home_screen, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_opa(s_home_screen, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(s_home_screen, lv_color_hex(csi_tx_get_tx_color(1)), 0);

    csi_tx_prepare_home_logo();

    s_home_logo = lv_image_create(s_home_screen);
    lv_image_set_src(s_home_logo, &s_home_logo_image);
    lv_obj_align(s_home_logo, LV_ALIGN_TOP_MID, 0, CSI_TX_HOME_LOGO_OFFSET_Y);
    lv_obj_set_style_bg_opa(s_home_logo, LV_OPA_TRANSP, 0);

    s_home_status = lv_label_create(s_home_screen);
    lv_obj_set_width(s_home_status, 148);
    lv_label_set_long_mode(s_home_status, LV_LABEL_LONG_MODE_SCROLL_CIRCULAR);
    lv_obj_set_style_anim_duration(s_home_status, 12000, 0);
    lv_obj_set_style_text_color(s_home_status, lv_color_white(), 0);
    lv_obj_align(s_home_status, LV_ALIGN_BOTTOM_LEFT, 6, -4);

    s_menu_screen = lv_obj_create(screen);
    lv_obj_remove_style_all(s_menu_screen);
    lv_obj_set_size(s_menu_screen, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_opa(s_menu_screen, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(s_menu_screen, lv_color_black(), 0);
    lv_obj_add_flag(s_menu_screen, LV_OBJ_FLAG_HIDDEN);

    s_menu_header = lv_label_create(s_menu_screen);
    lv_obj_set_width(s_menu_header, 148);
    lv_obj_set_style_text_color(s_menu_header, lv_color_white(), 0);
    lv_obj_align(s_menu_header, LV_ALIGN_TOP_LEFT, 6, 6);

    s_menu_body = lv_label_create(s_menu_screen);
    lv_obj_set_width(s_menu_body, 148);
    lv_obj_set_style_text_color(s_menu_body, lv_color_white(), 0);
    lv_obj_align(s_menu_body, LV_ALIGN_TOP_LEFT, 6, 34);
}

static void csi_tx_ui_task(void *arg)
{
    uint32_t last_color = UINT32_MAX;
    csi_tx_ui_mode_t last_mode = (csi_tx_ui_mode_t)-1;
    char last_status[192] = "";
    char last_header[24] = "";
    char last_body[96] = "";

    LV_UNUSED(arg);
    csi_tx_init_ui();

    while (true) {
        csi_tx_state_t snapshot = csi_tx_get_state_copy();
        csi_tx_config_t view_config = snapshot.ui_mode == CSI_TX_UI_EDIT ? snapshot.edit_config : snapshot.active_config;
        char status[192];
        char header[24];
        char value[48];
        char body[96];

        csi_tx_format_status(status, sizeof(status), &view_config);

        if (view_config.accent_rgb != last_color) {
            lv_obj_set_style_bg_color(s_home_screen, lv_color_hex(view_config.accent_rgb), 0);
            last_color = view_config.accent_rgb;
        }

        if (snapshot.ui_mode == CSI_TX_UI_HOME) {
            if (last_mode != CSI_TX_UI_HOME) {
                lv_obj_clear_flag(s_home_screen, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(s_menu_screen, LV_OBJ_FLAG_HIDDEN);
                last_mode = CSI_TX_UI_HOME;
            }
            if (strcmp(status, last_status) != 0) {
                lv_label_set_text(s_home_status, status);
                snprintf(last_status, sizeof(last_status), "%s", status);
            }
        } else {
            uint8_t position = csi_tx_get_menu_item_position(snapshot.menu_item, &view_config);
            uint8_t count = csi_tx_get_menu_item_count(&view_config);

            if (last_mode != snapshot.ui_mode) {
                lv_obj_add_flag(s_home_screen, LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(s_menu_screen, LV_OBJ_FLAG_HIDDEN);
                last_mode = snapshot.ui_mode;
            }

            snprintf(header,
                     sizeof(header),
                     "%s %u/%u",
                     snapshot.long_press_preview
                         ? (snapshot.ui_mode == CSI_TX_UI_EDIT ? "APPLY" : "EDIT")
                         : (snapshot.ui_mode == CSI_TX_UI_EDIT ? "EDIT" : "SETUP"),
                     position,
                     count);
            csi_tx_format_menu_value(value, sizeof(value), snapshot.menu_item, &view_config);
            snprintf(body, sizeof(body), "%s\n\n%s", csi_tx_get_menu_title(snapshot.menu_item), value);

            if (strcmp(header, last_header) != 0) {
                lv_label_set_text(s_menu_header, header);
                snprintf(last_header, sizeof(last_header), "%s", header);
            }
            if (strcmp(body, last_body) != 0) {
                lv_label_set_text(s_menu_body, body);
                snprintf(last_body, sizeof(last_body), "%s", body);
            }
        }

        lv_timer_handler();
        csi_tx_set_led_rgb(view_config.accent_rgb);

        vTaskDelay(pdMS_TO_TICKS(CSI_TX_UI_REFRESH_MS));
    }
}

static void csi_tx_enter_menu(void)
{
    taskENTER_CRITICAL(&s_state_lock);
    s_state.ui_mode = CSI_TX_UI_MENU;
    s_state.menu_item = CSI_TX_MENU_CHANNEL;
    s_state.edit_config = s_state.active_config;
    s_state.long_press_preview = false;
    taskEXIT_CRITICAL(&s_state_lock);
}

static void csi_tx_exit_menu(void)
{
    taskENTER_CRITICAL(&s_state_lock);
    s_state.ui_mode = CSI_TX_UI_HOME;
    s_state.edit_config = s_state.active_config;
    s_state.long_press_preview = false;
    taskEXIT_CRITICAL(&s_state_lock);
}

static void csi_tx_begin_edit(void)
{
    taskENTER_CRITICAL(&s_state_lock);
    s_state.ui_mode = CSI_TX_UI_EDIT;
    s_state.edit_config = s_state.active_config;
    s_state.long_press_preview = false;
    taskEXIT_CRITICAL(&s_state_lock);
}

static void csi_tx_commit_edit(void)
{
    csi_tx_config_t config;

    taskENTER_CRITICAL(&s_state_lock);
    config = s_state.edit_config;
    taskEXIT_CRITICAL(&s_state_lock);

    if (csi_tx_apply_runtime_config(config) == ESP_OK) {
        taskENTER_CRITICAL(&s_state_lock);
        s_state.ui_mode = CSI_TX_UI_MENU;
        s_state.long_press_preview = false;
        taskEXIT_CRITICAL(&s_state_lock);
    }
}

static void csi_tx_handle_button_event(bool long_press)
{
    csi_tx_state_t snapshot = csi_tx_get_state_copy();

    switch (snapshot.ui_mode) {
    case CSI_TX_UI_HOME:
        if (!long_press) {
            csi_tx_enter_menu();
        }
        break;
    case CSI_TX_UI_MENU:
        if (long_press) {
            csi_tx_begin_edit();
        } else if (csi_tx_menu_item_is_last(snapshot.menu_item, &snapshot.active_config)) {
            csi_tx_exit_menu();
        } else {
            taskENTER_CRITICAL(&s_state_lock);
            s_state.menu_item = csi_tx_get_next_menu_item(s_state.menu_item, &s_state.active_config);
            s_state.long_press_preview = false;
            taskEXIT_CRITICAL(&s_state_lock);
        }
        break;
    case CSI_TX_UI_EDIT:
        if (long_press) {
            csi_tx_commit_edit();
        } else {
            taskENTER_CRITICAL(&s_state_lock);
            csi_tx_cycle_menu_value(&s_state.edit_config, s_state.menu_item);
            s_state.long_press_preview = false;
            taskEXIT_CRITICAL(&s_state_lock);
        }
        break;
    }

    taskENTER_CRITICAL(&s_state_lock);
    s_state.long_press_preview = false;
    taskEXIT_CRITICAL(&s_state_lock);
}

static void csi_tx_poll_button(void)
{
    static bool last_raw_pressed;
    static bool stable_pressed;
    static int64_t last_change_us;
    static int64_t press_started_us;
    static bool long_press_triggered;

    bool raw_pressed = csi_tx_button_pressed();
    int64_t now_us = esp_timer_get_time();

    if (raw_pressed != last_raw_pressed) {
        last_raw_pressed = raw_pressed;
        last_change_us = now_us;
        return;
    }

    if ((now_us - last_change_us) < CSI_TX_BUTTON_DEBOUNCE_US || raw_pressed == stable_pressed) {
        if (stable_pressed && !long_press_triggered && (now_us - press_started_us) >= CSI_TX_BUTTON_LONG_PRESS_US) {
            taskENTER_CRITICAL(&s_state_lock);
            s_state.long_press_preview = true;
            taskEXIT_CRITICAL(&s_state_lock);
            long_press_triggered = true;
        }
        return;
    }

    stable_pressed = raw_pressed;
    if (stable_pressed) {
        press_started_us = now_us;
        long_press_triggered = false;
        taskENTER_CRITICAL(&s_state_lock);
        s_state.long_press_preview = false;
        taskEXIT_CRITICAL(&s_state_lock);
        return;
    }

    csi_tx_handle_button_event(long_press_triggered);
    long_press_triggered = false;
}

void app_main(void)
{
    uint16_t sequence = 0;
    uint32_t tx_count = 0;
    uint32_t tx_failures = 0;
    csi_tx_config_t boot_config;
    esp_err_t load_err;
    int64_t last_status_us;

    memset(&s_state, 0, sizeof(s_state));
    boot_config = csi_tx_get_default_config();

    esp_log_level_set("*", ESP_LOG_INFO);
    ESP_ERROR_CHECK(csi_tx_init_nvs());

    load_err = csi_tx_load_config_from_nvs(&boot_config);
    if (load_err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "no persisted config found, using menuconfig defaults");
    } else if (load_err != ESP_OK) {
        ESP_LOGW(TAG, "using menuconfig defaults after NVS load failure: %s", esp_err_to_name(load_err));
    }

    s_state.active_config = boot_config;
    s_state.edit_config = s_state.active_config;
    s_state.ui_mode = CSI_TX_UI_HOME;
    s_state.menu_item = CSI_TX_MENU_CHANNEL;
    s_state.current_channel = s_state.active_config.channel;

    ESP_ERROR_CHECK(csi_tx_init_button());

    if (csi_tx_init_display() == ESP_OK) {
        xTaskCreate(csi_tx_ui_task, "csi_tx_ui", 8192, NULL, 2, NULL);
    } else {
        ESP_LOGW(TAG, "display init failed, continuing without TFT output");
    }

    csi_tx_prepare_frame();
    csi_tx_update_frame_identity(s_state.active_config.tx_index);
    csi_tx_set_led_rgb(s_state.active_config.accent_rgb);
    ESP_ERROR_CHECK(csi_tx_init_wifi());
    csi_tx_log_config(&s_state.active_config);
    ESP_ERROR_CHECK(csi_tx_init_tx_timer(xTaskGetCurrentTaskHandle(), s_state.active_config.interval_ms));

    last_status_us = esp_timer_get_time();

    while (true) {
        csi_tx_config_t config = csi_tx_get_active_config();
        esp_err_t err;
        uint32_t pending_intervals;

        ESP_ERROR_CHECK(csi_tx_update_tx_timer_interval(config.interval_ms));
        csi_tx_poll_button();
        config = csi_tx_get_active_config();
        ESP_ERROR_CHECK(csi_tx_update_tx_timer_interval(config.interval_ms));

        pending_intervals = ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(5));
        if (pending_intervals == 0) {
            int64_t now_us = esp_timer_get_time();

            if ((now_us - last_status_us) >= (CSI_TX_STATUS_PERIOD_MS * 1000LL)) {
                csi_tx_state_t snapshot = csi_tx_get_state_copy();
                ESP_LOGI(TAG,
                         "tx_count=%" PRIu32 " tx_failures=%" PRIu32 " channel=%u tx=%u",
                         snapshot.tx_count,
                         snapshot.tx_failures,
                         snapshot.current_channel,
                         snapshot.active_config.tx_index);
                last_status_us = now_us;
            }
            continue;
        }

        s_frame.sequence_control[0] = (uint8_t)((sequence & 0x0fU) << 4);
        s_frame.sequence_control[1] = (uint8_t)((sequence >> 4) & 0xffU);
        err = esp_wifi_80211_tx(WIFI_IF_STA, &s_frame, sizeof(s_frame), false);
        if (err != ESP_OK) {
            tx_failures++;
        }

        sequence++;
        tx_count++;
        csi_tx_set_runtime_stats(tx_count, tx_failures, config.channel);

        {
            int64_t now_us = esp_timer_get_time();
            if ((now_us - last_status_us) >= (CSI_TX_STATUS_PERIOD_MS * 1000LL)) {
                csi_tx_state_t snapshot = csi_tx_get_state_copy();
                ESP_LOGI(TAG,
                         "tx_count=%" PRIu32 " tx_failures=%" PRIu32 " channel=%u tx=%u",
                         snapshot.tx_count,
                         snapshot.tx_failures,
                         snapshot.current_channel,
                         snapshot.active_config.tx_index);
                last_status_us = now_us;
            }
        }
    }
}
