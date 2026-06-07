#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "pin_config.h"
#include "actuator_control.h"
#include "lcd_status.h"
#include "setup_button.h"
#include "web_portal.h"
#include "wifi_manager.h"
#include "runtime_tasks.h"
#include "syslog.h"

static const char *TAG = "main";

void app_main(void)
{
#if ENABLE_ACTUATORS
    /* Drive all actuator GPIOs LOW immediately — prevents relay chatter
     * caused by floating pins before actuator_control_init() is called. */
    actuator_control_early_gpio_init();
#endif

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

#if ENABLE_LCD
    esp_err_t lcd_ret = lcd_status_init();
    if (lcd_ret == ESP_OK) {
        lcd_status_show_booting();
    } else {
        ESP_LOGW(TAG, "LCD init failed: %s — continuing without display", esp_err_to_name(lcd_ret));
    }
#endif

    runtime_tasks_record_boot_faults();
    wifi_manager_init();
    esp_log_level_set("ph_serial", ESP_LOG_WARN);
#if ENABLE_SETUP_BUTTON
    setup_button_init();
#endif

    /* ------------------------------------------------------------------ *
     * Try to resume the last known configuration without user interaction.
     * If it succeeds the AP is never started — the device goes straight
     * into runtime mode.  A background task keeps watching the button so
     * the user can still trigger reconfiguration at any time.
     * ------------------------------------------------------------------ */
    bool auto_started = false;
    esp_err_t auto_ret = web_portal_try_autostart_from_nvs(&auto_started);
    if (auto_started) {
        ESP_LOGI(TAG, "Autostart succeeded. Runtime running. AP not started.");
#if ENABLE_SETUP_BUTTON
        ESP_LOGI(TAG, "Hold GPIO%d for 3 s at any time to reconfigure.", 7);
        setup_button_start_monitor();
#endif
        return; /* app_main exits; runtime + monitor tasks keep running */
    }

    /* ------------------------------------------------------------------ *
     * Autostart failed (WiFi/MQTT error or no saved config) —
     * automatically start AP and web portal for reconfiguration.
     * Button monitor runs in background so user can reconfigure at any time.
     * ------------------------------------------------------------------ */
    if (auto_ret != ESP_OK) {
        ESP_LOGW(TAG, "Autostart failed: %s — starting AP automatically", esp_err_to_name(auto_ret));
    } else {
        ESP_LOGI(TAG, "No saved configuration found — starting AP automatically");
    }

    ESP_LOGI(TAG, "Starting setup AP and web portal...");
    ESP_ERROR_CHECK(wifi_manager_start_ap("ESP32S3-Updater", NULL));
#if ENABLE_LCD
    lcd_status_show_setup("192.168.4.1");
#endif
    ESP_ERROR_CHECK(web_portal_start());
    ESP_LOGI(TAG, "Setup portal running. Connect to ESP32S3-Updater and open http://192.168.4.1/");
#if ENABLE_SETUP_BUTTON
    ESP_LOGI(TAG, "Hold GPIO%d for 3 s at any time to reconfigure.", 7);
    setup_button_start_monitor();
#endif
    /* app_main returns; HTTP server and button monitor tasks keep running */
}
