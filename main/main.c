#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "setup_button.h"
#include "web_portal.h"
#include "wifi_manager.h"
#include "runtime_tasks.h"

static const char *TAG = "main";

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    runtime_tasks_record_boot_faults();
    wifi_manager_init();
    setup_button_init();

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
        ESP_LOGI(TAG, "Hold GPIO%d for 3 s at any time to reconfigure.", 46);
        setup_button_start_monitor();
        return; /* app_main exits; runtime + monitor tasks keep running */
    }

    if (auto_ret != ESP_OK) {
        ESP_LOGW(TAG, "Autostart failed: %s", esp_err_to_name(auto_ret));
    } else {
        ESP_LOGI(TAG, "No saved configuration found.");
    }

    /* ------------------------------------------------------------------ *
     * No valid saved config (or autostart failed) — wait for the user to
     * hold the setup button before exposing the AP and web portal.
     * ------------------------------------------------------------------ */
    ESP_LOGI(TAG, "Waiting for setup button hold (3 s) on GPIO%d...", 46);
    setup_button_wait_for_hold(3000);

    ESP_LOGI(TAG, "Starting setup AP and web portal...");
    ESP_ERROR_CHECK(wifi_manager_start_ap("ESP32S3-Updater", NULL));
    ESP_ERROR_CHECK(web_portal_start());
    ESP_LOGI(TAG, "Setup portal running. Connect to ESP32S3-Updater and open http://192.168.4.1/");
    /* app_main returns; HTTP server task keeps running */
}
