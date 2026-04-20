#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_rom_sys.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

static const char *TAG = "csi_tx";

#if CONFIG_CSI_TX_WIFI_BANDWIDTH_HT20
#define CSI_TX_WIFI_BANDWIDTH WIFI_BW20
#else
#define CSI_TX_WIFI_BANDWIDTH WIFI_BW40
#endif

#if CONFIG_CSI_TX_WIFI_SECONDARY_CHANNEL_ABOVE
#define CSI_TX_SECONDARY_CHANNEL WIFI_SECOND_CHAN_ABOVE
#else
#define CSI_TX_SECONDARY_CHANNEL WIFI_SECOND_CHAN_BELOW
#endif

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
    uint8_t channel;
    wifi_bandwidth_t bandwidth;
    wifi_second_chan_t secondary_channel;
    wifi_phy_mode_t phy_mode;
    wifi_phy_rate_t phy_rate;
    TickType_t interval_ticks;
    uint32_t interval_ms;
    int8_t max_tx_power_qdbm;
} csi_tx_config_t;

static wifi_frame_t s_frame = {
    .frame_control = {0x08, 0x00},
    .duration = {0x00, 0x00},
    .destination = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
    .payload = {0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef},
};

static TickType_t csi_tx_ms_to_ticks(uint32_t interval_ms)
{
    return (interval_ms + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS;
}

static wifi_phy_rate_t csi_tx_get_rate(int mcs_index)
{
    switch (mcs_index) {
    case 0: return WIFI_PHY_RATE_MCS0_LGI;
    case 1: return WIFI_PHY_RATE_MCS1_LGI;
    case 2: return WIFI_PHY_RATE_MCS2_LGI;
    case 3: return WIFI_PHY_RATE_MCS3_LGI;
    case 4: return WIFI_PHY_RATE_MCS4_LGI;
    case 5: return WIFI_PHY_RATE_MCS5_LGI;
    case 6: return WIFI_PHY_RATE_MCS6_LGI;
    case 7: return WIFI_PHY_RATE_MCS7_LGI;
    default: return WIFI_PHY_RATE_MCS0_LGI;
    }
}

static csi_tx_config_t csi_tx_get_config(void)
{
    const wifi_bandwidth_t bandwidth = CSI_TX_WIFI_BANDWIDTH;
    const uint32_t interval_ms = CONFIG_CSI_TX_INTERVAL_MS;

    return (csi_tx_config_t) {
        .channel = CONFIG_CSI_TX_WIFI_CHANNEL,
        .bandwidth = bandwidth,
        .secondary_channel = CSI_TX_SECONDARY_CHANNEL,
        .phy_mode = bandwidth == WIFI_BW20 ? WIFI_PHY_MODE_HT20 : WIFI_PHY_MODE_HT40,
        .phy_rate = csi_tx_get_rate(CONFIG_CSI_TX_MCS_INDEX),
        .interval_ticks = csi_tx_ms_to_ticks(interval_ms),
        .interval_ms = interval_ms,
        .max_tx_power_qdbm = CONFIG_CSI_TX_MAX_TX_POWER_QDBM,
    };
}

static bool csi_tx_channel_config_is_valid(const csi_tx_config_t *config)
{
    if (config->bandwidth == WIFI_BW20) {
        return true;
    }

    if (config->secondary_channel == WIFI_SECOND_CHAN_ABOVE) {
        return config->channel >= 1 && config->channel <= 9;
    }

    return config->channel >= 5 && config->channel <= 13;
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

static esp_err_t csi_tx_init_wifi(const csi_tx_config_t *config)
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    cfg.ampdu_tx_enable = false;
    cfg.sta_disconnected_pm = false;
    wifi_tx_rate_config_t rate_config = {
        .phymode = config->phy_mode,
        .rate = config->phy_rate,
        .ersu = false,
        .dcm = false,
    };

    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "failed to create event loop");
    ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), TAG, "failed to init Wi-Fi");
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "failed to set storage");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "failed to set station mode");

    wifi_country_t country = {
        .cc = "DE",
        .schan = 1,
        .nchan = 13,
        .policy = WIFI_COUNTRY_POLICY_MANUAL,
    };

    wifi_config_t wifi_config = { 0 };

    ESP_RETURN_ON_ERROR(esp_wifi_set_country(&country), TAG, "failed to set country");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &wifi_config), TAG, "failed to set STA config");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "failed to start Wi-Fi");
    ESP_RETURN_ON_ERROR(esp_wifi_set_band_mode(WIFI_BAND_MODE_2G_ONLY), TAG, "failed to force 2.4 GHz band");
    ESP_RETURN_ON_ERROR(esp_wifi_set_ps(WIFI_PS_NONE), TAG, "failed to disable power save");
    ESP_RETURN_ON_ERROR(esp_wifi_set_max_tx_power(config->max_tx_power_qdbm), TAG, "failed to set TX power");
    ESP_RETURN_ON_ERROR(esp_wifi_set_protocol(WIFI_IF_STA,
                                              WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N),
                        TAG, "failed to set 2.4 GHz protocol");
    ESP_RETURN_ON_ERROR(esp_wifi_set_bandwidth(WIFI_IF_STA, config->bandwidth), TAG, "failed to set bandwidth");
    ESP_RETURN_ON_ERROR(esp_wifi_set_channel(config->channel, config->secondary_channel),
                        TAG, "failed to set channel");
    ESP_RETURN_ON_ERROR(esp_wifi_config_80211_tx(WIFI_IF_STA, &rate_config),
                        TAG, "failed to set fixed TX config");

    return ESP_OK;
}

static void csi_tx_prepare_frame(void)
{
    uint8_t mac[6] = {0};
    ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_WIFI_STA));

    memcpy(s_frame.source, mac, sizeof(s_frame.source));
    memcpy(s_frame.bssid, mac, sizeof(s_frame.bssid));
}

static void csi_tx_log_config(const csi_tx_config_t *config)
{
    ESP_LOGI(TAG,
             "ready: mac=%02x:%02x:%02x:%02x:%02x:%02x channel=%u bw=%s mcs=%d secondary=%s interval=%" PRIu32 "ms",
             s_frame.source[0], s_frame.source[1], s_frame.source[2],
             s_frame.source[3], s_frame.source[4], s_frame.source[5],
             config->channel,
             config->bandwidth == WIFI_BW20 ? "HT20" : "HT40",
             CONFIG_CSI_TX_MCS_INDEX,
             config->secondary_channel == WIFI_SECOND_CHAN_ABOVE ? "above" : "below",
             config->interval_ms);
}

static void csi_tx_console_warmup(void)
{
    esp_rom_printf("csi_tx boot\r\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
}

void app_main(void)
{
    const csi_tx_config_t config = csi_tx_get_config();

    esp_log_level_set("*", ESP_LOG_INFO);
    csi_tx_console_warmup();
    ESP_ERROR_CHECK(csi_tx_init_nvs());

    if (!csi_tx_channel_config_is_valid(&config)) {
        ESP_LOGE(TAG,
                 "invalid HT40 config: channel=%d secondary=%s",
                 config.channel,
                 config.secondary_channel == WIFI_SECOND_CHAN_ABOVE ? "above" : "below");
        ESP_LOGE(TAG, "use channels 1-9 for HT40 above or 5-13 for HT40 below");
        return;
    }

    csi_tx_prepare_frame();
    ESP_ERROR_CHECK(csi_tx_init_wifi(&config));
    csi_tx_log_config(&config);

    uint16_t sequence = 0;
    uint32_t tx_failures = 0;
    TickType_t last_wake_tick = xTaskGetTickCount();
    TickType_t last_status_tick = last_wake_tick;
    const TickType_t status_period_ticks = pdMS_TO_TICKS(1000);

    while (true) {
        s_frame.sequence_control[0] = (sequence & 0x0f) << 4;
        s_frame.sequence_control[1] = (sequence >> 4) & 0xff;

        esp_err_t err = esp_wifi_80211_tx(WIFI_IF_STA, &s_frame, sizeof(s_frame), false);
        if (err != ESP_OK) {
            tx_failures++;
            if (err == ESP_ERR_NO_MEM) {
                vTaskDelay(config.interval_ticks);
            }
        }

        sequence++;
        TickType_t now_tick = xTaskGetTickCount();
        if ((now_tick - last_status_tick) >= status_period_ticks) {
            ESP_LOGI(TAG, "tx_count=%u tx_failures=%" PRIu32, sequence, tx_failures);
            last_status_tick = now_tick;
        }

        vTaskDelayUntil(&last_wake_tick, config.interval_ticks);
    }
}
