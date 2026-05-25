Firmware berjalan di atas ESP32-S3 menggunakan ESP-IDF dengan FreeRTOS. Firmware bertanggung jawab untuk: pembacaan sensor secara periodik, pengiriman data ke broker MQTT, penerimaan dan eksekusi perintah aktuator, manajemen koneksi WiFi/MQTT (reconnect otomatis), sistem safety runtime, serta proses OTA update firmware secara aman.
Firmware menggunakan arsitektur multi-task FreeRTOS dengan 4 task yang berjalan secara paralel. Setiap task memiliki stack, prioritas, dan tanggung jawab yang terpisah:
•	SensorTask (prioritas 2, stack 6144 bytes, interval 1000 ms): 
Membaca nilai dari seluruh sensor (pH, TDS, Water Level, Water Temp DS18B20) via sensor_telemetry_sample() dan memperbarui snapshot dengan rolling average 30 sampel. Setiap siklus, task mengirim heartbeat ke SafetyTask via runtime_safety_heartbeat_sensor() dan melakukan kick pada ESP Task Watchdog (WDT). Task menggunakan vTaskDelayUntil() untuk menjaga interval sampling tetap presisi 1000 ms terlepas dari durasi eksekusi.
•	CommTask (prioritas 1, stack 8192 bytes, interval 1000 ms): 
Setiap siklus mengambil snapshot sensor terbaru; jika snapshot tidak tersedia, menggunakan snapshot terakhir yang valid (last-known-good). Data dipublikasikan ke topik MQTT per-sensor: water_level, water_temp, ph, ph_raw, ph_valid, tds, tds_raw, tds_valid. Nilai pH dan TDS hanya dipublikasikan jika kalibrasi valid (ph_valid/tds_valid = true), sedangkan flag valid selalu dipublikasikan dengan retain=1. LCD diperbarui hanya saat ada perubahan perintah aktuator, menampilkan channel, state (ON/OFF/PULSE), dan nilai sensor relevan (pH untuk dosing pH, TDS untuk nutrisi, water level untuk valve). Heartbeat versi firmware dikirim ke <zone>/version setiap 30 detik.
•	DosingTask (prioritas 3, stack 4096 bytes): 
Memblokir pada queue dengan timeout 200 ms per iterasi (kapasitas queue 16 item). Setiap perintah diperiksa dua kondisi sebelum dieksekusi: (1) jika safety gate tertutup (safe mode aktif), perintah langsung dibuang; (2) jika terjadi pH interlock — pH Up diperintah saat pH Down sedang ON atau sebaliknya — fault SAFETY_FAULT_PH_INTERLOCK di-set dan perintah dibuang. Eksekusi aksi: ON/OFF memanggil actuator_control_apply_state(), PULSE memanggil actuator_control_apply_pulse() dengan durasi ms. Setelah eksekusi berhasil, task melaporkan ke safety runtime: dose watchdog diperbarui, valve action dicatat untuk state machine auto-fill, pH dose dicatat untuk monitoring respons pH, TDS dose dicatat untuk monitoring respons TDS.
•	SafetyTask (prioritas 4, stack 6144 bytes, tertinggi): 
Berjalan dengan interval SAFETY_INTERVAL_MS menggunakan ulTaskNotifyTake() sebagai mekanisme sleep. Setiap siklus memantau heartbeat ketiga task lain (Dosing, Sensor, Comm) — jika heartbeat tidak berubah melebihi batas waktu, task terkait dianggap stall dan fault di-set. Pemeriksaan sensor aktif meliputi: suhu air (warning jika > SAFETY_TEMP_HIGH, fault jika > SAFETY_TEMP_CRITICAL atau < SAFETY_TEMP_LOW), deteksi sensor beku (nilai tidak berubah selama N sampel), pH out-of-range, pH critical high/low, laju perubahan pH terlalu cepat (rate > SAFETY_PH_RATE_MAX_PER_MIN), serta pemeriksaan TDS serupa. Fault yang terdeteksi dipublikasikan ke <zone>/safety/fault dengan kode fault (F-001 s/d F-013). Safety gate dikontrol via portMUX untuk akses atomik — gate tertutup saat ada fault aktif, terbuka kembali hanya setelah semua fault di-clear via <zone>/safety/clear.
•	OTA: 
Firmware subscribe ke <zone>/ota/latest_version (retained) untuk menerima versi terbaru. Ketika topik <zone>/ota/trigger diterima dengan payload “1”, firmware mendownload binary dari URL http://<broker_ip>:8123/local/firmware/lorong_node.bin dan melakukan flash OTA.
•	Safety Clear: 
Operator dapat mereset fault via topik <zone>/safety/clear dengan payload berisi bitmask fault yang ingin dihapus. Jika semua fault berhasil dihapus, safety gate akan dibuka kembali dan dosing dapat dilanjutkan.

Konfigurasi Zone disimpan di NVS (namespace zone_cfg) berisi zone_id dan zone_name. Nilai ini digunakan sebagai prefix seluruh topik MQTT (contoh: lorong_1/sensor, lorong_1/safety/fault). Zone dapat diubah melalui web portal tanpa reflash firmware.

Web Portal berjalan sebagai HTTP server di atas ESP-IDF (esp_http_server). Portal aktif saat perangkat belum terkonfigurasi atau saat mode AP aktif. Halaman utama (/) menyediakan form untuk mengisi SSID, password WiFi, zone ID, zone name, broker IP, dan broker port. Halaman /debug menampilkan status runtime, daftar topik aktif, perintah terakhir, data sensor terakhir, dan kesehatan koneksi MQTT.

Kalibrasi Sensor disimpan secara persisten di NVS (namespace sensor_cali). Tersedia kalibrasi untuk sensor pH (slope + offset) dan TDS (slope + offset), masing-masing dilengkapi flag valid dan timestamp pembaruan. Nilai kalibrasi diterapkan saat pembacaan sensor sebelum data dipublikasikan ke MQTT.

Manajemen Koneksi WiFi menggunakan mode STA dengan auto-reconnect. Saat koneksi awal, firmware mencoba hingga 10 kali (MAXIMUM_RETRY). Setelah koneksi berhasil, auto-reconnect aktif secara permanen — setiap event WIFI_EVENT_STA_DISCONNECTED akan memicu esp_wifi_connect() ulang tanpa batas. MQTT juga melakukan resubscribe otomatis ke seluruh topik yang terdaftar setiap kali koneksi MQTT pulih.

Ringkasan Topik MQTT:
Publish (outbound):
•	<zone>/sensor/pH/raw — nilai ADC raw pH (uint16)
•	<zone>/sensor/pH/state — nilai pH terkalibrasi (float, hanya jika kalibrasi valid)
•	<zone>/sensor/pH/valid — "true" | "false" (retain=1)
•	<zone>/sensor/TDS/raw — nilai ADC raw TDS
•	<zone>/sensor/TDS/state — nilai TDS terkalibrasi (float, hanya jika kalibrasi valid)
•	<zone>/sensor/TDS/valid — "true" | "false" (retain=1)
•	<zone>/sensor/WaterLevel/state — level air
•	<zone>/sensor/WaterTemp/state — suhu air (°C)
•	<zone>/calibration/pH/state — koefisien kalibrasi pH (JSON)
•	<zone>/calibration/TDS/state — koefisien kalibrasi TDS (JSON)
•	<zone>/version — versi firmware (retain=1, dikirim setiap 30 detik)
•	<zone>/safety/fault — notifikasi fault aktif (SafetyTask)
•	<zone>/<channel>/status — status ON/OFF per channel aktuator

Subscribe (inbound):
•	<zone>/command — perintah zone: format "<Channel> ON|OFF|PULSE <ms>"
•	<zone>/<channel>/command — perintah per channel: payload "ON" | "OFF" | "PULSE:<ms>"
•	<zone>/calibration/pH/set — set kalibrasi pH (JSON)
•	<zone>/calibration/TDS/set — set kalibrasi TDS (JSON)
•	<zone>/safety/clear — reset fault: payload "CLEAR", "ALL", atau "MASK:<hex>"
•	<zone>/ota/latest_version — versi firmware terbaru (retained)
•	<zone>/ota/trigger — trigger download OTA (payload "1")
•	<zone>/emergency — emergency stop semua aktuator (payload "1")

Stack Size per Task:
•	SensorTask: 6144 bytes
•	CommTask: 8192 bytes
•	DosingTask: 4096 bytes
•	SafetyTask: 6144 bytes
