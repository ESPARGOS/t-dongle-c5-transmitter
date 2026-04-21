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
#include "esp_lcd_st7735.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"
#include "nvs_flash.h"

static const char *TAG = "csi_tx";

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
#define CSI_TX_UI_TICK_PERIOD_MS 5
#define CSI_TX_UI_REFRESH_MS 40
#define CSI_TX_STATUS_PERIOD_MS 1000
#define CSI_TX_SPLASH_MS 1500
#define CSI_TX_LED_PIN_CI CSI_TX_LCD_PIN_SCLK
#define CSI_TX_LED_PIN_DI CSI_TX_LCD_PIN_MOSI
#define CSI_TX_LED_SPI_CLOCK_HZ (500 * 1000)
#define CSI_TX_LED_BRIGHTNESS 31
#define CSI_TX_HOME_LOGO_WIDTH 136
#define CSI_TX_HOME_LOGO_HEIGHT 28
#define CSI_TX_HOME_LOGO_OFFSET_X ((CSI_TX_LCD_V_RES - CSI_TX_HOME_LOGO_WIDTH) / 2)
#define CSI_TX_HOME_LOGO_OFFSET_Y 16

typedef enum {
    CSI_TX_MOD_11B,
    CSI_TX_MOD_11G,
    CSI_TX_MOD_HT20,
    CSI_TX_MOD_HT40,
    CSI_TX_MOD_HE20,
} csi_tx_modulation_t;

typedef enum {
    CSI_TX_UI_MODE_HOME,
    CSI_TX_UI_MODE_MENU_NAV,
    CSI_TX_UI_MODE_MENU_EDIT,
} csi_tx_ui_mode_t;

typedef enum {
    CSI_TX_MENU_ITEM_CHANNEL,
    CSI_TX_MENU_ITEM_TX_INDEX,
    CSI_TX_MENU_ITEM_MODULATION,
    CSI_TX_MENU_ITEM_RATE,
    CSI_TX_MENU_ITEM_SECONDARY,
    CSI_TX_MENU_ITEM_INTERVAL,
    CSI_TX_MENU_ITEM_TX_POWER,
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
    TickType_t interval_ticks;
    uint32_t interval_ms;
    int8_t max_tx_power_qdbm;
    int rate_index;
    uint32_t accent_rgb;
    const char *modulation_name;
    const char *rate_name;
} csi_tx_config_t;

typedef struct {
    uint8_t current_channel;
    uint32_t tx_count;
    uint32_t tx_failures;
} csi_tx_runtime_t;

typedef struct {
    csi_tx_ui_mode_t mode;
    csi_tx_menu_item_t item;
    csi_tx_config_t edit_config;
    bool long_press_preview;
} csi_tx_ui_state_t;

typedef struct {
    csi_tx_runtime_t runtime;
    csi_tx_config_t active_config;
    csi_tx_config_t preview_config;
    csi_tx_ui_state_t ui_state;
} csi_tx_ui_snapshot_t;

static const uint32_t s_interval_options_ms[] = {
    1, 2, 5, 7, 10, 11, 13, 15, 16, 20, 25, 50, 100, 200, 500, 1000,
};
static const int8_t s_tx_power_options_qdbm[] = {
    8, 16, 24, 32, 40, 48, 56, 64, 72, 80, 84,
};

static wifi_frame_t s_frame = {
    .frame_control = {0x08, 0x00},
    .duration = {0x00, 0x00},
    .destination = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
    .payload = {0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef},
};

static csi_tx_config_t s_config;
static csi_tx_runtime_t s_runtime;
static csi_tx_ui_state_t s_ui_state;
static uint8_t s_base_mac[6];
static bool s_wifi_ready;
static bool s_led_ready;
static portMUX_TYPE s_state_lock = portMUX_INITIALIZER_UNLOCKED;

static esp_lcd_panel_io_handle_t s_lcd_io;
static esp_lcd_panel_handle_t s_lcd_panel;
static lv_display_t *s_lv_display;
static lv_obj_t *s_home_container;
static lv_obj_t *s_menu_container;
static lv_obj_t *s_home_logo;
static lv_obj_t *s_marquee_label;
static lv_obj_t *s_menu_header_label;
static lv_obj_t *s_menu_body_label;
static lv_obj_t *s_menu_hint_label;
static esp_timer_handle_t s_lvgl_tick_timer;
static spi_device_handle_t s_led_strip;

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

static TickType_t csi_tx_ms_to_ticks(uint32_t interval_ms)
{
    return (interval_ms + portTICK_PERIOD_MS - 1U) / portTICK_PERIOD_MS;
}

static uint32_t csi_tx_get_tx_color(uint8_t tx_index)
{
    switch (tx_index) {
    case 1: return 0x1f77b4;
    case 2: return 0xff7f0e;
    case 3: return 0x2ca02c;
    case 4: return 0xd62728;
    case 5: return 0x9467bd;
    default: return 0x1f77b4;
    }
}

static void csi_tx_prepare_home_logo(void)
{
    const uint8_t *mask = _binary_home_logo_mask_gray_start;
    size_t mask_size = (size_t)(_binary_home_logo_mask_gray_end - _binary_home_logo_mask_gray_start);
    size_t pixel_count = CSI_TX_HOME_LOGO_WIDTH * CSI_TX_HOME_LOGO_HEIGHT;
    uint8_t *color_plane = s_home_logo_pixels;
    uint8_t *alpha_plane = s_home_logo_pixels + (pixel_count * 2);

    if (mask_size != pixel_count) {
        return;
    }

    for (size_t i = 0; i < pixel_count; ++i) {
        color_plane[(i * 2) + 0] = 0xff;
        color_plane[(i * 2) + 1] = 0xff;
    }
    memcpy(alpha_plane, mask, pixel_count);
}

static bool csi_tx_modulation_supports_secondary(csi_tx_modulation_t modulation)
{
    return modulation == CSI_TX_MOD_HT40;
}

static uint8_t csi_tx_get_rate_count(csi_tx_modulation_t modulation)
{
    switch (modulation) {
    case CSI_TX_MOD_11B:
        return 4;
    case CSI_TX_MOD_11G:
        return 8;
    case CSI_TX_MOD_HE20:
#if CONFIG_SOC_WIFI_HE_SUPPORT
        return 10;
#else
        return 8;
#endif
    case CSI_TX_MOD_HT20:
    case CSI_TX_MOD_HT40:
    default:
        return 8;
    }
}

static wifi_phy_rate_t csi_tx_get_rate_11b(int rate_index)
{
    switch (rate_index) {
    case 0: return WIFI_PHY_RATE_1M_L;
    case 1: return WIFI_PHY_RATE_2M_L;
    case 2: return WIFI_PHY_RATE_5M_L;
    case 3: return WIFI_PHY_RATE_11M_L;
    default: return WIFI_PHY_RATE_1M_L;
    }
}

static wifi_phy_rate_t csi_tx_get_rate_11g(int rate_index)
{
    switch (rate_index) {
    case 0: return WIFI_PHY_RATE_6M;
    case 1: return WIFI_PHY_RATE_9M;
    case 2: return WIFI_PHY_RATE_12M;
    case 3: return WIFI_PHY_RATE_18M;
    case 4: return WIFI_PHY_RATE_24M;
    case 5: return WIFI_PHY_RATE_36M;
    case 6: return WIFI_PHY_RATE_48M;
    case 7: return WIFI_PHY_RATE_54M;
    default: return WIFI_PHY_RATE_6M;
    }
}

static wifi_phy_rate_t csi_tx_get_rate_mcs(int rate_index)
{
    switch (rate_index) {
    case 0: return WIFI_PHY_RATE_MCS0_LGI;
    case 1: return WIFI_PHY_RATE_MCS1_LGI;
    case 2: return WIFI_PHY_RATE_MCS2_LGI;
    case 3: return WIFI_PHY_RATE_MCS3_LGI;
    case 4: return WIFI_PHY_RATE_MCS4_LGI;
    case 5: return WIFI_PHY_RATE_MCS5_LGI;
    case 6: return WIFI_PHY_RATE_MCS6_LGI;
    case 7: return WIFI_PHY_RATE_MCS7_LGI;
#if CONFIG_SOC_WIFI_HE_SUPPORT
    case 8: return WIFI_PHY_RATE_MCS8_LGI;
    case 9: return WIFI_PHY_RATE_MCS9_LGI;
#endif
    default: return WIFI_PHY_RATE_MCS0_LGI;
    }
}

static const char *csi_tx_get_modulation_name(csi_tx_modulation_t modulation)
{
    switch (modulation) {
    case CSI_TX_MOD_11B: return "11b";
    case CSI_TX_MOD_11G: return "11g";
    case CSI_TX_MOD_HT20: return "11n HT20";
    case CSI_TX_MOD_HT40: return "11n HT40";
    case CSI_TX_MOD_HE20: return "11ax HE20";
    default: return "11n HT20";
    }
}

static const char *csi_tx_get_rate_name(csi_tx_modulation_t modulation, int rate_index)
{
    static const char *const names_11b[] = {"1M", "2M", "5.5M", "11M"};
    static const char *const names_11g[] = {"6M", "9M", "12M", "18M", "24M", "36M", "48M", "54M"};
    static const char *const names_mcs[] = {
        "MCS0", "MCS1", "MCS2", "MCS3", "MCS4",
        "MCS5", "MCS6", "MCS7", "MCS8", "MCS9",
    };

    switch (modulation) {
    case CSI_TX_MOD_11B:
        return names_11b[rate_index >= 0 && rate_index < 4 ? rate_index : 0];
    case CSI_TX_MOD_11G:
        return names_11g[rate_index >= 0 && rate_index < 8 ? rate_index : 0];
    case CSI_TX_MOD_HT20:
    case CSI_TX_MOD_HT40:
    case CSI_TX_MOD_HE20:
    default:
        return names_mcs[rate_index >= 0 && rate_index < 10 ? rate_index : 0];
    }
}

static void csi_tx_resolve_config(csi_tx_config_t *config)
{
    uint8_t rate_count = csi_tx_get_rate_count(config->modulation);

    config->protocol_bitmap = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N;
    config->bandwidth = WIFI_BW20;
    config->secondary_channel = WIFI_SECOND_CHAN_NONE;
    config->phy_mode = WIFI_PHY_MODE_HT20;
    config->phy_rate = WIFI_PHY_RATE_MCS0_LGI;
    config->min_channel = 1;
    config->max_channel = 13;

    if (config->rate_index < 0 || config->rate_index >= rate_count) {
        config->rate_index = 0;
    }
    if (config->tx_index < 1 || config->tx_index > 5) {
        config->tx_index = 1;
    }
    if (config->interval_ms == 0) {
        config->interval_ms = 1;
    }

    switch (config->modulation) {
    case CSI_TX_MOD_11B:
        config->protocol_bitmap = WIFI_PROTOCOL_11B;
        config->phy_mode = WIFI_PHY_MODE_11B;
        config->phy_rate = csi_tx_get_rate_11b(config->rate_index);
        break;
    case CSI_TX_MOD_11G:
        config->protocol_bitmap = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G;
        config->phy_mode = WIFI_PHY_MODE_11G;
        config->phy_rate = csi_tx_get_rate_11g(config->rate_index);
        break;
    case CSI_TX_MOD_HT40:
        config->protocol_bitmap = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N;
        config->bandwidth = WIFI_BW40;
        config->phy_mode = WIFI_PHY_MODE_HT40;
        config->phy_rate = csi_tx_get_rate_mcs(config->rate_index);
        if (config->secondary_channel != WIFI_SECOND_CHAN_BELOW) {
            config->secondary_channel = WIFI_SECOND_CHAN_ABOVE;
        }
        config->min_channel = config->secondary_channel == WIFI_SECOND_CHAN_ABOVE ? 1 : 5;
        config->max_channel = config->secondary_channel == WIFI_SECOND_CHAN_ABOVE ? 9 : 13;
        break;
    case CSI_TX_MOD_HE20:
        config->protocol_bitmap = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_11AX;
        config->phy_mode = WIFI_PHY_MODE_HE20;
        config->phy_rate = csi_tx_get_rate_mcs(config->rate_index);
        break;
    case CSI_TX_MOD_HT20:
    default:
        config->protocol_bitmap = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N;
        config->phy_mode = WIFI_PHY_MODE_HT20;
        config->phy_rate = csi_tx_get_rate_mcs(config->rate_index);
        break;
    }

    if (!csi_tx_modulation_supports_secondary(config->modulation)) {
        config->secondary_channel = WIFI_SECOND_CHAN_NONE;
    }
    if (config->channel < config->min_channel || config->channel > config->max_channel) {
        config->channel = config->min_channel;
    }

    config->modulation_name = csi_tx_get_modulation_name(config->modulation);
    config->rate_name = csi_tx_get_rate_name(config->modulation, config->rate_index);
    config->interval_ticks = csi_tx_ms_to_ticks(config->interval_ms);
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
        .interval_ms = CONFIG_CSI_TX_INTERVAL_MS,
        .max_tx_power_qdbm = CONFIG_CSI_TX_MAX_TX_POWER_QDBM,
        .rate_index = CONFIG_CSI_TX_RATE_INDEX,
        .secondary_channel = WIFI_SECOND_CHAN_NONE,
    };

#if CONFIG_CSI_TX_MODULATION_HT40
    config.secondary_channel = CONFIG_CSI_TX_WIFI_SECONDARY_CHANNEL_ABOVE ? WIFI_SECOND_CHAN_ABOVE : WIFI_SECOND_CHAN_BELOW;
#endif

    csi_tx_resolve_config(&config);
    return config;
}

static csi_tx_config_t csi_tx_get_config_snapshot(void)
{
    csi_tx_config_t config;

    taskENTER_CRITICAL(&s_state_lock);
    config = s_config;
    taskEXIT_CRITICAL(&s_state_lock);

    return config;
}

static void csi_tx_set_runtime_channel(uint8_t channel)
{
    taskENTER_CRITICAL(&s_state_lock);
    s_runtime.current_channel = channel;
    taskEXIT_CRITICAL(&s_state_lock);
}

static void csi_tx_note_tx_result(uint32_t tx_count, uint32_t tx_failures)
{
    taskENTER_CRITICAL(&s_state_lock);
    s_runtime.tx_count = tx_count;
    s_runtime.tx_failures = tx_failures;
    taskEXIT_CRITICAL(&s_state_lock);
}

static csi_tx_runtime_t csi_tx_get_runtime_snapshot(void)
{
    csi_tx_runtime_t snapshot;

    taskENTER_CRITICAL(&s_state_lock);
    snapshot = s_runtime;
    taskEXIT_CRITICAL(&s_state_lock);

    return snapshot;
}

static csi_tx_ui_snapshot_t csi_tx_get_ui_snapshot(void)
{
    csi_tx_ui_snapshot_t snapshot;

    taskENTER_CRITICAL(&s_state_lock);
    snapshot.runtime = s_runtime;
    snapshot.active_config = s_config;
    snapshot.ui_state = s_ui_state;
    snapshot.preview_config = s_ui_state.mode == CSI_TX_UI_MODE_MENU_EDIT ? s_ui_state.edit_config : s_config;
    taskEXIT_CRITICAL(&s_state_lock);

    return snapshot;
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
    csi_tx_set_runtime_channel(channel);
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
    ESP_RETURN_ON_ERROR(csi_tx_apply_wifi_config(&s_config), TAG, "failed to apply runtime Wi-Fi config");

    s_wifi_ready = true;
    return ESP_OK;
}

static void csi_tx_log_config(void)
{
    ESP_LOGI(TAG,
             "ready: mac=%02x:%02x:%02x:%02x:%02x:%02x tx=%u channel=%u modulation=%s rate=%s interval=%" PRIu32 "ms",
             s_frame.source[0], s_frame.source[1], s_frame.source[2],
             s_frame.source[3], s_frame.source[4], s_frame.source[5],
             s_config.tx_index,
             s_config.channel,
             s_config.modulation_name,
             s_config.rate_name,
             s_config.interval_ms);
}

static void csi_tx_console_warmup(void)
{
    esp_rom_printf("csi_tx boot\r\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
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
    uint8_t red = (uint8_t)((rgb >> 16) & 0xff);
    uint8_t green = (uint8_t)((rgb >> 8) & 0xff);
    uint8_t blue = (uint8_t)(rgb & 0xff);

    if (!s_led_ready) {
        return;
    }

    // APA102 uses a 4-byte start frame, then [brightness | blue | green | red],
    // followed by trailing 1 bits to latch the last LED.
    buffer[4] = 0b11100000 | (CSI_TX_LED_BRIGHTNESS & 0x1f);
    buffer[5] = blue;
    buffer[6] = green;
    buffer[7] = red;

    if (spi_device_transmit(s_led_strip, &transaction) != ESP_OK) {
        ESP_LOGW(TAG, "failed to update APA102 LED");
    }
}

static esp_err_t csi_tx_apply_runtime_config(csi_tx_config_t config)
{
    esp_err_t err = ESP_OK;

    csi_tx_resolve_config(&config);

    if (s_wifi_ready) {
        err = csi_tx_apply_wifi_config(&config);
        if (err != ESP_OK) {
            return err;
        }
    } else {
        csi_tx_set_runtime_channel(config.channel);
    }

    csi_tx_update_frame_identity(config.tx_index);
    csi_tx_set_led_rgb(config.accent_rgb);

    taskENTER_CRITICAL(&s_state_lock);
    s_config = config;
    taskEXIT_CRITICAL(&s_state_lock);

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

static csi_tx_menu_item_t csi_tx_menu_item_next(csi_tx_menu_item_t item, const csi_tx_config_t *config)
{
    csi_tx_menu_item_t next = item;

    do {
        next = (csi_tx_menu_item_t)(((int)next + 1) % ((int)CSI_TX_MENU_ITEM_TX_POWER + 1));
    } while (next == CSI_TX_MENU_ITEM_SECONDARY && !csi_tx_modulation_supports_secondary(config->modulation));

    return next;
}

static csi_tx_modulation_t csi_tx_next_modulation(csi_tx_modulation_t modulation)
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

static uint32_t csi_tx_next_interval_ms(uint32_t value)
{
    for (size_t i = 0; i < sizeof(s_interval_options_ms) / sizeof(s_interval_options_ms[0]); ++i) {
        if (s_interval_options_ms[i] >= value) {
            return s_interval_options_ms[(i + 1U) % (sizeof(s_interval_options_ms) / sizeof(s_interval_options_ms[0]))];
        }
    }
    return s_interval_options_ms[0];
}

static int8_t csi_tx_next_tx_power_qdbm(int8_t value)
{
    for (size_t i = 0; i < sizeof(s_tx_power_options_qdbm) / sizeof(s_tx_power_options_qdbm[0]); ++i) {
        if (s_tx_power_options_qdbm[i] >= value) {
            return s_tx_power_options_qdbm[(i + 1U) % (sizeof(s_tx_power_options_qdbm) / sizeof(s_tx_power_options_qdbm[0]))];
        }
    }
    return s_tx_power_options_qdbm[0];
}

static void csi_tx_cycle_menu_value(csi_tx_config_t *config, csi_tx_menu_item_t item)
{
    switch (item) {
    case CSI_TX_MENU_ITEM_CHANNEL:
        config->channel++;
        if (config->channel > config->max_channel || config->channel < config->min_channel) {
            config->channel = config->min_channel;
        }
        break;
    case CSI_TX_MENU_ITEM_TX_INDEX:
        config->tx_index = (uint8_t)((config->tx_index % 5U) + 1U);
        break;
    case CSI_TX_MENU_ITEM_MODULATION:
        config->modulation = csi_tx_next_modulation(config->modulation);
        config->rate_index = 0;
        if (config->modulation == CSI_TX_MOD_HT40 && config->secondary_channel == WIFI_SECOND_CHAN_NONE) {
            config->secondary_channel = WIFI_SECOND_CHAN_ABOVE;
        }
        break;
    case CSI_TX_MENU_ITEM_RATE:
        config->rate_index = (config->rate_index + 1) % csi_tx_get_rate_count(config->modulation);
        break;
    case CSI_TX_MENU_ITEM_SECONDARY:
        if (csi_tx_modulation_supports_secondary(config->modulation)) {
            config->secondary_channel = config->secondary_channel == WIFI_SECOND_CHAN_BELOW ? WIFI_SECOND_CHAN_ABOVE : WIFI_SECOND_CHAN_BELOW;
        }
        break;
    case CSI_TX_MENU_ITEM_INTERVAL:
        config->interval_ms = csi_tx_next_interval_ms(config->interval_ms);
        break;
    case CSI_TX_MENU_ITEM_TX_POWER:
        config->max_tx_power_qdbm = csi_tx_next_tx_power_qdbm(config->max_tx_power_qdbm);
        break;
    default:
        break;
    }

    csi_tx_resolve_config(config);
}

static void csi_tx_enter_menu(void)
{
    taskENTER_CRITICAL(&s_state_lock);
    s_ui_state.mode = CSI_TX_UI_MODE_MENU_NAV;
    s_ui_state.item = CSI_TX_MENU_ITEM_CHANNEL;
    s_ui_state.edit_config = s_config;
    s_ui_state.long_press_preview = false;
    taskEXIT_CRITICAL(&s_state_lock);
}

static void csi_tx_exit_menu(void)
{
    taskENTER_CRITICAL(&s_state_lock);
    s_ui_state.mode = CSI_TX_UI_MODE_HOME;
    s_ui_state.edit_config = s_config;
    s_ui_state.long_press_preview = false;
    taskEXIT_CRITICAL(&s_state_lock);
}

static void csi_tx_begin_edit(void)
{
    taskENTER_CRITICAL(&s_state_lock);
    s_ui_state.mode = CSI_TX_UI_MODE_MENU_EDIT;
    s_ui_state.edit_config = s_config;
    s_ui_state.long_press_preview = false;
    taskEXIT_CRITICAL(&s_state_lock);
}

static void csi_tx_commit_edit(void)
{
    csi_tx_config_t config;

    taskENTER_CRITICAL(&s_state_lock);
    config = s_ui_state.edit_config;
    taskEXIT_CRITICAL(&s_state_lock);

    if (csi_tx_apply_runtime_config(config) == ESP_OK) {
        taskENTER_CRITICAL(&s_state_lock);
        s_ui_state.mode = CSI_TX_UI_MODE_MENU_NAV;
        s_ui_state.edit_config = s_config;
        s_ui_state.long_press_preview = false;
        taskEXIT_CRITICAL(&s_state_lock);
    }
}

static bool csi_tx_menu_item_is_last(csi_tx_menu_item_t item, const csi_tx_config_t *config)
{
    return item == CSI_TX_MENU_ITEM_TX_POWER ||
           (item == CSI_TX_MENU_ITEM_INTERVAL && !csi_tx_modulation_supports_secondary(config->modulation));
}

static void csi_tx_handle_button_event(bool long_press)
{
    csi_tx_ui_state_t ui_state;
    csi_tx_config_t config;

    taskENTER_CRITICAL(&s_state_lock);
    ui_state = s_ui_state;
    config = s_config;
    taskEXIT_CRITICAL(&s_state_lock);

    switch (ui_state.mode) {
    case CSI_TX_UI_MODE_HOME:
        if (!long_press) {
            csi_tx_enter_menu();
        }
        break;
    case CSI_TX_UI_MODE_MENU_NAV:
        if (long_press) {
            csi_tx_begin_edit();
        } else {
            if (csi_tx_menu_item_is_last(ui_state.item, &config)) {
                csi_tx_exit_menu();
            } else {
                taskENTER_CRITICAL(&s_state_lock);
                s_ui_state.item = csi_tx_menu_item_next(s_ui_state.item, &s_config);
                s_ui_state.long_press_preview = false;
                taskEXIT_CRITICAL(&s_state_lock);
            }
        }
        break;
    case CSI_TX_UI_MODE_MENU_EDIT:
        if (long_press) {
            csi_tx_commit_edit();
        } else {
            taskENTER_CRITICAL(&s_state_lock);
            csi_tx_cycle_menu_value(&s_ui_state.edit_config, s_ui_state.item);
            taskEXIT_CRITICAL(&s_state_lock);
        }
        break;
    default:
        break;
    }

    taskENTER_CRITICAL(&s_state_lock);
    s_ui_state.long_press_preview = false;
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
            s_ui_state.long_press_preview = true;
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
        s_ui_state.long_press_preview = false;
        taskEXIT_CRITICAL(&s_state_lock);
        return;
    }

    csi_tx_handle_button_event(long_press_triggered);
    long_press_triggered = false;
}

static bool csi_tx_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io,
                                    esp_lcd_panel_io_event_data_t *edata,
                                    void *user_ctx)
{
    lv_display_t *display = user_ctx;

    LV_UNUSED(panel_io);
    LV_UNUSED(edata);
    lv_display_flush_ready(display);
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
    ESP_RETURN_ON_ERROR(spi_bus_initialize(CSI_TX_LCD_HOST, &buscfg, SPI_DMA_CH_AUTO), TAG, "failed to init LCD SPI bus");
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

static void csi_tx_build_marquee(char *buffer, size_t buffer_size, const csi_tx_config_t *config)
{
    int whole_dbm = config->max_tx_power_qdbm / 4;
    int frac_dbm = (config->max_tx_power_qdbm % 4) * 25;

    if (config->bandwidth == WIFI_BW40) {
        snprintf(buffer,
                 buffer_size,
                 "TX %u | CH %u | %s | %s | SEC %s | INT %" PRIu32 "ms | PWR %d.%02ddBm",
                 config->tx_index,
                 config->channel,
                 config->modulation_name,
                 config->rate_name,
                 config->secondary_channel == WIFI_SECOND_CHAN_ABOVE ? "ABOVE" : "BELOW",
                 config->interval_ms,
                 whole_dbm,
                 frac_dbm);
        return;
    }

    snprintf(buffer,
             buffer_size,
             "TX %u | CH %u | %s | %s | INT %" PRIu32 "ms | PWR %d.%02ddBm",
             config->tx_index,
             config->channel,
             config->modulation_name,
             config->rate_name,
             config->interval_ms,
             whole_dbm,
             frac_dbm);
}

static const char *csi_tx_get_menu_title(csi_tx_menu_item_t item)
{
    switch (item) {
    case CSI_TX_MENU_ITEM_CHANNEL: return "Channel";
    case CSI_TX_MENU_ITEM_TX_INDEX: return "TX Index";
    case CSI_TX_MENU_ITEM_MODULATION: return "Modulation";
    case CSI_TX_MENU_ITEM_RATE: return "Rate";
    case CSI_TX_MENU_ITEM_SECONDARY: return "Secondary";
    case CSI_TX_MENU_ITEM_INTERVAL: return "Interval";
    case CSI_TX_MENU_ITEM_TX_POWER: return "TX Power";
    default: return "Setup";
    }
}

static void csi_tx_get_menu_value(char *buffer, size_t buffer_size, csi_tx_menu_item_t item, const csi_tx_config_t *config)
{
    int whole_dbm = config->max_tx_power_qdbm / 4;
    int frac_dbm = (config->max_tx_power_qdbm % 4) * 25;

    switch (item) {
    case CSI_TX_MENU_ITEM_CHANNEL:
        snprintf(buffer, buffer_size, "%u", config->channel);
        break;
    case CSI_TX_MENU_ITEM_TX_INDEX:
        snprintf(buffer, buffer_size, "%u", config->tx_index);
        break;
    case CSI_TX_MENU_ITEM_MODULATION:
        snprintf(buffer, buffer_size, "%s", config->modulation_name);
        break;
    case CSI_TX_MENU_ITEM_RATE:
        snprintf(buffer, buffer_size, "%s", config->rate_name);
        break;
    case CSI_TX_MENU_ITEM_SECONDARY:
        snprintf(buffer, buffer_size, "%s",
                 config->secondary_channel == WIFI_SECOND_CHAN_ABOVE ? "ABOVE" : "BELOW");
        break;
    case CSI_TX_MENU_ITEM_INTERVAL:
        snprintf(buffer, buffer_size, "%" PRIu32 "ms", config->interval_ms);
        break;
    case CSI_TX_MENU_ITEM_TX_POWER:
        snprintf(buffer, buffer_size, "%d.%02ddBm", whole_dbm, frac_dbm);
        break;
    default:
        buffer[0] = '\0';
        break;
    }
}

static uint8_t csi_tx_get_menu_item_position(csi_tx_menu_item_t item, const csi_tx_config_t *config)
{
    csi_tx_menu_item_t cursor = CSI_TX_MENU_ITEM_CHANNEL;
    uint8_t position = 1;

    while (cursor != item) {
        cursor = csi_tx_menu_item_next(cursor, config);
        position++;
        if (position > 7) {
            break;
        }
    }

    return position;
}

static uint8_t csi_tx_get_menu_item_count(const csi_tx_config_t *config)
{
    return csi_tx_modulation_supports_secondary(config->modulation) ? 7 : 6;
}

static void csi_tx_init_ui(void)
{
    lv_obj_t *screen = lv_display_get_screen_active(s_lv_display);

    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(screen, lv_color_black(), 0);

    s_home_container = lv_obj_create(screen);
    lv_obj_remove_style_all(s_home_container);
    lv_obj_set_size(s_home_container, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_opa(s_home_container, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(s_home_container, lv_color_hex(csi_tx_get_tx_color(1)), 0);

    s_menu_container = lv_obj_create(screen);
    lv_obj_remove_style_all(s_menu_container);
    lv_obj_set_size(s_menu_container, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(s_menu_container, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_menu_container, LV_OPA_COVER, 0);
    lv_obj_add_flag(s_menu_container, LV_OBJ_FLAG_HIDDEN);

    csi_tx_prepare_home_logo();

    s_home_logo = lv_image_create(s_home_container);
    lv_image_set_src(s_home_logo, &s_home_logo_image);
    lv_obj_align(s_home_logo, LV_ALIGN_TOP_MID, 0, CSI_TX_HOME_LOGO_OFFSET_Y);
    lv_obj_set_style_bg_opa(s_home_logo, LV_OPA_TRANSP, 0);

    s_marquee_label = lv_label_create(s_home_container);
    lv_obj_set_width(s_marquee_label, 148);
    lv_label_set_long_mode(s_marquee_label, LV_LABEL_LONG_MODE_SCROLL_CIRCULAR);
    lv_obj_set_style_anim_duration(s_marquee_label, 12000, 0);
    lv_obj_align(s_marquee_label, LV_ALIGN_BOTTOM_LEFT, 6, -4);
    lv_obj_set_style_text_color(s_marquee_label, lv_color_white(), 0);

    s_menu_header_label = lv_label_create(s_menu_container);
    lv_obj_set_width(s_menu_header_label, 148);
    lv_obj_set_style_text_align(s_menu_header_label, LV_TEXT_ALIGN_LEFT, 0);
    lv_obj_align(s_menu_header_label, LV_ALIGN_TOP_LEFT, 6, 4);
    lv_obj_set_style_text_color(s_menu_header_label, lv_color_white(), 0);

    s_menu_body_label = lv_label_create(s_menu_container);
    lv_obj_set_width(s_menu_body_label, 148);
    lv_label_set_long_mode(s_menu_body_label, LV_LABEL_LONG_MODE_CLIP);
    lv_obj_set_style_text_align(s_menu_body_label, LV_TEXT_ALIGN_LEFT, 0);
    lv_obj_align(s_menu_body_label, LV_ALIGN_TOP_LEFT, 6, 24);
    lv_obj_set_style_text_color(s_menu_body_label, lv_color_white(), 0);

    s_menu_hint_label = lv_label_create(s_menu_container);
    lv_obj_set_width(s_menu_hint_label, 148);
    lv_obj_set_style_text_align(s_menu_hint_label, LV_TEXT_ALIGN_LEFT, 0);
    lv_obj_align(s_menu_hint_label, LV_ALIGN_BOTTOM_LEFT, 6, -4);
    lv_obj_set_style_text_color(s_menu_hint_label, lv_color_white(), 0);
    lv_label_set_text(s_menu_hint_label, "");
}

static void csi_tx_ui_task(void *arg)
{
    uint32_t last_home_color = 0;
    uint32_t last_led_color = 0;
    csi_tx_ui_mode_t last_mode = (csi_tx_ui_mode_t)-1;
    char last_marquee[192] = "";
    char last_header[24] = "";
    char last_menu_body[96] = "";

    LV_UNUSED(arg);
    csi_tx_init_ui();

    while (true) {
        csi_tx_ui_snapshot_t snapshot = csi_tx_get_ui_snapshot();
        char marquee[192];
        char menu_value[48];
        char menu_body[96];
        char header[24];
        uint8_t item_position = csi_tx_get_menu_item_position(snapshot.ui_state.item, &snapshot.preview_config);
        uint8_t item_count = csi_tx_get_menu_item_count(&snapshot.preview_config);
        bool ui_changed = false;

        if (snapshot.active_config.accent_rgb != last_home_color) {
            lv_obj_set_style_bg_color(s_home_container, lv_color_hex(snapshot.active_config.accent_rgb), 0);
            last_home_color = snapshot.active_config.accent_rgb;
            ui_changed = true;
        }

        csi_tx_build_marquee(marquee, sizeof(marquee), &snapshot.preview_config);

        if (snapshot.ui_state.mode == CSI_TX_UI_MODE_HOME) {
            if (last_mode != CSI_TX_UI_MODE_HOME) {
                lv_obj_clear_flag(s_home_container, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(s_menu_container, LV_OBJ_FLAG_HIDDEN);
                last_mode = CSI_TX_UI_MODE_HOME;
                ui_changed = true;
            }
            if (strcmp(last_marquee, marquee) != 0) {
                lv_label_set_text(s_marquee_label, marquee);
                snprintf(last_marquee, sizeof(last_marquee), "%s", marquee);
                ui_changed = true;
            }
        } else {
            if (last_mode != snapshot.ui_state.mode) {
                lv_obj_add_flag(s_home_container, LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(s_menu_container, LV_OBJ_FLAG_HIDDEN);
                last_mode = snapshot.ui_state.mode;
                ui_changed = true;
            }
            snprintf(header,
                     sizeof(header),
                     "%s %u/%u",
                     snapshot.ui_state.long_press_preview
                         ? (snapshot.ui_state.mode == CSI_TX_UI_MODE_MENU_EDIT ? "SETUP" : "EDIT")
                         : (snapshot.ui_state.mode == CSI_TX_UI_MODE_MENU_EDIT ? "EDIT" : "SETUP"),
                     item_position,
                     item_count);
            csi_tx_get_menu_value(menu_value, sizeof(menu_value), snapshot.ui_state.item, &snapshot.preview_config);
            snprintf(menu_body,
                     sizeof(menu_body),
                     "%s\n\n%s",
                     csi_tx_get_menu_title(snapshot.ui_state.item),
                     menu_value);
            if (strcmp(last_header, header) != 0) {
                lv_label_set_text(s_menu_header_label, header);
                snprintf(last_header, sizeof(last_header), "%s", header);
                ui_changed = true;
            }
            if (strcmp(last_menu_body, menu_body) != 0) {
                lv_label_set_text(s_menu_body_label, menu_body);
                snprintf(last_menu_body, sizeof(last_menu_body), "%s", menu_body);
                ui_changed = true;
            }
        }

        lv_timer_handler();
        if (snapshot.ui_state.mode == CSI_TX_UI_MODE_HOME) {
            csi_tx_set_led_rgb(snapshot.preview_config.accent_rgb);
            last_led_color = snapshot.preview_config.accent_rgb;
        } else if (ui_changed || snapshot.preview_config.accent_rgb != last_led_color) {
            csi_tx_set_led_rgb(snapshot.preview_config.accent_rgb);
            last_led_color = snapshot.preview_config.accent_rgb;
        }
        vTaskDelay(pdMS_TO_TICKS(CSI_TX_UI_REFRESH_MS));
    }
}

void app_main(void)
{
    uint16_t sequence = 0;
    uint32_t tx_failures = 0;
    TickType_t last_wake_tick;
    TickType_t last_status_tick;
    TickType_t status_period_ticks = pdMS_TO_TICKS(CSI_TX_STATUS_PERIOD_MS);

    s_config = csi_tx_get_default_config();
    s_runtime.current_channel = s_config.channel;
    s_ui_state.mode = CSI_TX_UI_MODE_HOME;
    s_ui_state.item = CSI_TX_MENU_ITEM_CHANNEL;
    s_ui_state.edit_config = s_config;

    esp_log_level_set("*", ESP_LOG_INFO);
    csi_tx_console_warmup();
    ESP_ERROR_CHECK(csi_tx_init_nvs());
    ESP_ERROR_CHECK(csi_tx_init_button());

    if (csi_tx_init_display() != ESP_OK) {
        ESP_LOGW(TAG, "display init failed, continuing without TFT output");
    } else {
        xTaskCreate(csi_tx_ui_task, "csi_tx_ui", 8192, NULL, 2, NULL);
    }

    csi_tx_prepare_frame();
    csi_tx_update_frame_identity(s_config.tx_index);
    csi_tx_set_led_rgb(s_config.accent_rgb);
    ESP_ERROR_CHECK(csi_tx_init_wifi());
    csi_tx_log_config();

    last_wake_tick = xTaskGetTickCount();
    last_status_tick = last_wake_tick;

    while (true) {
        csi_tx_config_t config = csi_tx_get_config_snapshot();
        esp_err_t err;

        csi_tx_poll_button();

        s_frame.sequence_control[0] = (uint8_t)((sequence & 0x0fU) << 4);
        s_frame.sequence_control[1] = (uint8_t)((sequence >> 4) & 0xffU);
        err = esp_wifi_80211_tx(WIFI_IF_STA, &s_frame, sizeof(s_frame), false);
        if (err != ESP_OK) {
            tx_failures++;
            if (err == ESP_ERR_NO_MEM) {
                vTaskDelay(config.interval_ticks);
            }
        }

        sequence++;
        csi_tx_note_tx_result(sequence, tx_failures);

        {
            TickType_t now_tick = xTaskGetTickCount();
            if ((now_tick - last_status_tick) >= status_period_ticks) {
                csi_tx_runtime_t snapshot = csi_tx_get_runtime_snapshot();
                ESP_LOGI(TAG,
                         "tx_count=%" PRIu32 " tx_failures=%" PRIu32 " channel=%u tx=%u",
                         snapshot.tx_count,
                         snapshot.tx_failures,
                         snapshot.current_channel,
                         config.tx_index);
                last_status_tick = now_tick;
            }
        }

        vTaskDelayUntil(&last_wake_tick, config.interval_ticks);
    }
}
