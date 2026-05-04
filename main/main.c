#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"

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
	ESP_ERROR_CHECK(wifi_manager_start_ap("ESP32S3-Updater", NULL));
	ESP_ERROR_CHECK(web_portal_start());

	bool auto_started = false;
	esp_err_t auto_ret = web_portal_try_autostart_from_nvs(&auto_started);
	if (auto_ret != ESP_OK) {
		ESP_LOGW(TAG, "Saved setup autostart failed: %s", esp_err_to_name(auto_ret));
	} else if (auto_started) {
		ESP_LOGI(TAG, "Saved setup applied. Runtime started automatically.");
	}

	ESP_LOGI(TAG, "Setup complete. Connect to AP and open http://192.168.4.1/");

}
