# Pertanyaan Sidang & Jawaban - Firmware Hidroponik (FreeRTOS)

Dokumen ini berisi daftar potensi pertanyaan teknis mendalam terkait implementasi firmware pada ESP32 yang digunakan untuk sistem kontrol hidroponik, beserta panduan jawabannya.

---

## 1. Arsitektur Firmware & FreeRTOS

**Q1: Mengapa menggunakan FreeRTOS dibandingkan super-loop (seperti `loop()` biasa pada Arduino) pada ESP32?**
**Jawaban:**
FreeRTOS digunakan untuk mencapai sifat deterministik, modularitas, dan pemisahan *concern* antar subsistem. Dalam hidroponik otomatis, ada tugas-tugas kritis seperti memonitor sensor, mengontrol aktuator (pompa dosing), berkomunikasi via MQTT, dan *safety watchdog*. 
Dengan *super-loop*, jika komunikasi jaringan *blocking* (misal timeout koneksi MQTT), maka pembacaan sensor atau penghentian pompa dosing bisa tertunda, yang sangat berbahaya (bisa *overdosing* nutrisi atau pH). Dengan FreeRTOS, setiap sistem berjalan pada *Task* independen dengan prioritasnya masing-masing.

**Q2: Bagaimana pembagian prioritas Task pada sistem ini, dan apa alasannya?**
**Jawaban:**
Terdapat 4 task utama (didefinisikan di `runtime_tasks.c`):
1. **SafetyTask (Prioritas 4 - Tertinggi):** Memantau status *heartbeat* dari task lain, mengecek limit batas waktu pompa menyala (*dose duration limit*), dan kondisi pembekuan sensor (*sensor freeze*). Wajib memiliki prioritas tertinggi agar jika sistem *hang*, ia bisa segera masuk *Safe Mode* dan mematikan aktuator.
2. **DosingTask (Prioritas 3):** Memproses perintah aktuator dari antrian (queue). Harus memiliki prioritas tinggi agar pompa dapat merespon perintah nyala/mati tanpa jeda (terutama untuk mode PULSE).
3. **SensorTask (Prioritas 2):** Membaca data ADC (pH, TDS) dan Onewire (Suhu). Pembacaan sensor tidak membutuhkan respon waktu nyata yang sangat presisi, sehingga prioritasnya di bawah kontrol fisik.
4. **CommTask (Prioritas 1 - Terendah):** Menangani *publish* data ke MQTT broker. Proses komunikasi memiliki latensi yang tidak dapat diprediksi (network I/O), sehingga ditempatkan di prioritas terendah agar tidak mengganggu *timing* kontrol aktuator dan safety.

**Q3: Bagaimana cara antar-Task saling berkomunikasi dengan aman tanpa *Data Race*?**
**Jawaban:**
- **Queue:** Untuk perintah aktuator (dari MQTT ke DosingTask), digunakan `xQueueSend` / `xQueueReceive`.
- **Mutex (Semaphore):** Digunakan untuk melindungi akses ke data bersama, seperti `s_snapshot_mutex` untuk melindungi struct `sensor_telemetry_snapshot_t` saat SensorTask menulis data baru dan CommTask membacanya untuk dikirim ke MQTT.
- **Atomics:** Untuk status-status sederhana (*flags*) seperti *Heartbeat Counters*, indikator status *Safe Mode*, dan *Fault Masks*, digunakan tipe data `<stdatomic.h>` (`atomic_load`, `atomic_store`, `atomic_fetch_add`) yang aman diakses secara *lock-free* antar core.
- **Critical Sections (portENTER_CRITICAL):** Digunakan pada *state* internal yang singkat (misal update status channel aktuator), menonaktifkan *interrupt* sementara pada *core* tersebut untuk memastikan pembacaan/penulisan secara utuh (atomic).

---

## 2. Sistem Keamanan (Safety & Interlock)

**Q4: Jika ESP32 mengalami *hang* pada salah satu Task (misalnya karena *deadlock*), bagaimana sistem mencegah pompa terus menyala?**
**Jawaban:**
Terdapat dua tingkat pengamanan *Watchdog*:
1. **Software Task Watchdog (Heartbeat):** Setiap Task (Dosing, Sensor, Comm) akan menambah *counter heartbeat* (menggunakan atomic operations) setiap putaran loop-nya. *SafetyTask* akan mengecek setiap 500ms apakah *heartbeat* Task lain bertambah. Jika stagnan melebihi batas waktu (misal 15 detik), SafetyTask akan men-trigger *Safety Fault* (SAFETY_FAULT_WDT) dan mengaktifkan *Safe Mode* (mematikan semua aktuator).
2. **Hardware/Task WDT (ESP-IDF):** Firmware menginisialisasi `esp_task_wdt`. Setiap Task me-register dirinya dan men-trigger (kick) WDT secara berkala. Jika ada task yang benar-benar *stuck* sehingga gagal *kick* WDT selama 8 detik, sistem operasi RTOS (ESP-IDF) akan langsung melakukan *Hardware Panic* / *Reset*. Kondisi *reset* karena WDT ini dicatat (`runtime_tasks_record_boot_faults`) sehingga setelah *reboot*, alat langsung masuk ke *Safe Mode* (Boot fault) dan melaporkannya ke server, tidak melanjutkan operasinya secara membabi buta.

**Q5: Apa itu "Peristaltic Interlock" atau *Chemical Hazard Interlock* pada kode Anda?**
**Jawaban:**
Interlock ini ada di dalam `dosing_task`. Hanya satu pompa peristaltik (NutA, NutB, pH Up, pH Down) yang diizinkan menyala dalam satu waktu. Hal paling krusial adalah mencegah cairan asam pekat (pH Down) dan basa pekat (pH Up) dicampur secara bersamaan, karena bisa menyebabkan reaksi kimia berbahaya dan merusak komponen pipa. Jika alat mendeteksi ada permintaan menyalakan pH Up saat pH Down sedang menyala (atau sebaliknya), sistem men-trigger `SAFETY_FAULT_PH_INTERLOCK` dan langsung masuk ke *Safe Mode*.

**Q6: Bagaimana mekanisme mencegah kolam meluap (overfill) atau pompa menyala terus tanpa batas?**
**Jawaban:**
Modul `runtime_safety` memantau *Dose Duration*. Setiap kali aktuator menyala, dicatat `on_tick`-nya. Pada setiap putaran, `SafetyTask` menghitung selisih waktu dari awal aktuator menyala sampai waktu sekarang. Jika durasi menyala melebihi parameter yang disetup (seperti `SAFETY_MAX_DOSE_A_MS` yaitu 2 menit, atau `SAFETY_FILL_TIMEOUT_MS` yaitu 10 menit untuk katup air), maka sistem men-trigger batas *timeout fault*, dan langsung masuk ke *Safe Mode*.

**Q7: Jika alat mendeteksi masalah (Safe Mode aktif), apa yang terjadi dengan relay dan aktuator?**
**Jawaban:**
Fungsi `safety_enter_safe_state()` akan dipanggil. Fungsi ini akan:
1. Menurunkan status internal `s_gate_open` menjadi *false*.
2. Mematikan (OFF) ke-10 channel aktuator dengan memanggil `actuator_control_apply_state`.
3. Mengosongkan sisa antrian perintah yang mungkin ada di `dosing_queue` (`flush_dosing_queue`).
4. Mengubah tampilan LCD menjadi mode *Emergency*.
5. Sistem tidak akan bisa menyalakan aktuator kembali sampai menerima perintah MQTT *Safety Clear* dari user yang bertanggung jawab.

---

## 3. Sensor & Kalibrasi (NVS)

**Q8: Bagaimana arsitektur pembacaan sensor pH dan TDS?**
**Jawaban:**
Untuk sensor pH dan TDS analog, pembacaan tegangan menggunakan eksternal ADC ADS1115 via I2C (16-bit resolusi) dibanding menggunakan internal ADC ESP32 yang non-linear dan noisy. 
1. `sensor_telemetry.c` akan membaca *raw value* (satuan milivolt/count ADC) secara rutin (misalnya 1Hz).
2. Data *raw* akan dimasukkan ke *Rolling History Buffer* untuk dirata-rata (*Moving Average*) guna meminimalisir fluktuasi transien (*noise*).
3. Hasil rata-rata dikonversikan ke besaran fisik (nilai pH atau ppm TDS) berdasarkan *kalibrasi linear* (`y = mx + c` / `value = slope * raw + offset`).
4. Jika nilai hasil konversi keluar dari batas rasional (misal pH < 0 atau > 14), maka data tersebut ditandai dengan flag `valid = false`.

**Q9: Bagaimana cara menyimpan dan me-*load* kalibrasi agar tidak hilang saat ESP32 mati/reset?**
**Jawaban:**
Kami menggunakan subsistem **NVS (Non-Volatile Storage)** dari ESP-IDF. Modul `sensor_calibration_nvs.c` menyimpan data *slope* (float) dan *offset* (float) langsung ke *flash memory* pada *namespace* "sensor_cali". 
Setiap kali alat dihidupkan (boot), fungsi `sensor_telemetry_init` akan memuat data kalibrasi dari NVS. Jika NVS kosong atau terjadi kegagalan pembacaan, nilai *valid flag* dari sensor akan menjadi *false*, yang mencegah otomatisasi berjalan menggunakan data yang tidak terkalibrasi.

**Q10: Mengapa perlu fitur "Sensor Frozen Fault"?**
**Jawaban:**
Fitur `SAFETY_PH_FROZEN_SAMPLES` pada `runtime_safety.c` mendeteksi anomali perangkat keras. Jika pembacaan sensor (misal TDS atau pH) menghasilkan nilai yang *sama persis* (selisih di bawah `FROZEN_EPSILON`) selama beberapa menit berturut-turut pada massa air yang besar, ini secara fisik tidak mungkin dan mengindikasikan modul sensor/I2C *hang* atau kabel terputus. Sistem akan men-*trigger* `SAFETY_FAULT_PH_SENSOR` untuk mencegah *dosing* buta.

---

## 4. Pengendalian Aktuator

**Q11: Di sisi aktuator, kenapa firmware mendefinisikan *state* `ACTUATOR_ACTION_PULSE` alih-alih `ON` biasa untuk *dosing*?**
**Jawaban:**
Mode `PULSE` diciptakan untuk keamanan *dosing* mikro. Jika server mengirimkan perintah `ON` dan kemudian server atau jaringan mati (sehingga perintah `OFF` gagal sampai), pompa akan menyala terus dan menyebabkan kolam hancur. 
Dengan perintah `PULSE <durasi_ms>`, firmware bertugas mematikan pompa secara otomatis setelah *timer* habis (*delay blocking* di *DosingTask*). Ini membuat *dosing* bergantung pada *hardware timer* di alat lokal, bukan pada *networking*.

**Q12: Bagaimana masalah *Relay Chatter* (relay bergetar saat *boot*) diselesaikan?**
**Jawaban:**
Seringkali GPIO pada ESP32 memiliki status *floating* atau menjadi *HIGH* sesaat saat mikrokontroler menyala (terutama *boot strapping pins*). Untuk mengatasinya, kami memanggil `actuator_control_early_gpio_init()` di *baris paling pertama* pada fungsi `app_main()`. Fungsi ini mengatur pin secara eksplisit ke mode Output dan bernilai `LOW`, sebelum *WiFi*, *NVS*, atau *Task* lain mulai dijalankan, memastikan semua aktuator mati dengan aman tanpa kedipan mekanis.

---

## 5. Komunikasi Jaringan (MQTT & WiFi)

**Q13: Apa itu MQTT Retain dan QoS 1, dan mengapa digunakan?**
**Jawaban:**
- **QoS 1 (Quality of Service 1 - At least once):** Memastikan pesan dari ESP32 benar-benar diterima oleh *Broker*. Broker harus membalas dengan `PUBACK`. Jika tidak, ESP32 akan terus mencoba mengirimkan ulang pesan tersebut. Sangat krusial untuk laporan *Safety Faults*.
- **Retain = True:** Digunakan pada topik seperti konfigurasi alat (`/version`) dan validasi sensor (`/pH/valid`). Ketika diset ke true, Broker menyimpan pesan terakhir. Sehingga jika ada UI (Dashboard Frontend) yang baru terkoneksi ke broker, ia bisa langsung mendapatkan *state* terakhir dari firmware tanpa harus menunggu siklus *publish* berikutnya.

**Q14: Jika koneksi WiFi terputus saat mesin sedang beroperasi, apa yang terjadi?**
**Jawaban:**
Operasional *DosingTask*, *SensorTask*, dan *SafetyTask* berjalan sepenuhnya secara otonom dan mandiri di RTOS lokal tanpa koneksi internet. 
Jika *disconnect*, `wifi_manager.c` mendeteksinya via `WIFI_EVENT_STA_DISCONNECTED` dan akan secara asinkron mencoba terkoneksi ulang (Auto-reconnect). Selama *offline*, sistem hidroponik tetap terlindungi oleh seluruh lapis keamanan lokal (Watchdog, sensor checks, interlock), tetapi pesan telemetry ke broker akan numpuk/gagal sampai koneksi pulih kembali.

---

## 6. Over-The-Air (OTA) Updates

**Q15: Bagaimana alur firmware memproses pembaruan perangkat lunak via OTA tanpa kabel USB?**
**Jawaban:**
Alur OTA berada di `ota_update.c`:
1. Menerima notifikasi *trigger* (payload `1`) via MQTT ke topik `ota/trigger`.
2. Firmware mengambil URL *firmware server* dari konfigurasi setup (HTTP Server di jaringan lokal).
3. Melalui `esp_http_client`, alat mendownload *binary* (`.bin`) terbaru secara bertahap (chunking menggunakan *buffer* 1024 bytes).
4. `esp_ota_begin` dan `esp_ota_write` menulis potongan-potongan tersebut langsung ke partisi `ota_0` atau `ota_1` yang sedang tidak aktif (*Ping-Pong Partition*).
5. Setelah *download* utuh, `esp_ota_end` memvalidasi *checksum* *binary*, mengatur partisi tersebut menjadi partisi utama saat *boot* selanjutnya (`esp_ota_set_boot_partition`), kemudian mikrokontroler direstart otomatis (esp_restart).

**Q16: Apa fungsi partisi `ota_0` dan `ota_1` pada tabel partisi (partitions.csv)?**
**Jawaban:**
Ini adalah mekanisme pengamanan "A/B Partitioning". ESP32 tidak menimpa partisi OS (*app*) yang sedang berjalan (`ota_0`). Saat *update* OTA, data *firmware* baru ditulis ke partisi sekunder (`ota_1`). Jika proses pengunduhan terputus di tengah jalan (misal listrik mati), partisi yang sedang berjalan tidak rusak (*uncorrupted*), karena alat masih *boot* dari `ota_0`. Partisi mana yang aktif dikelola oleh blok data *otadata*.

---

## 7. Boot Sequence & Provisioning (Web Portal)

**Q17: Bagaimana alur pertama kali alat dioperasikan — dari pertama dinyalakan hingga bisa berkomunikasi dengan server?**
**Jawaban:**
Alur *First-Time Setup* dikendalikan oleh fungsi `app_main()` di `main.c`:
1. **Early GPIO Init:** Langkah pertama adalah `actuator_control_early_gpio_init()` untuk memaksa semua pin aktuator ke LOW sebelum hal lain dilakukan.
2. **NVS Init:** `nvs_flash_init()` menginisialisasi Flash Storage. Jika Flash terdeteksi rusak atau versi berbeda, flash dihapus dan diinisialisasi ulang.
3. **LCD Boot Screen:** `lcd_status_show_booting()` ditampilkan sebagai umpan balik visual saat *hardware* sedang diinisialisasi.
4. **Auto-Start dari NVS:** Sistem mencoba membaca konfigurasi WiFi & MQTT yang sudah tersimpan dari NVS sebelumnya (`web_portal_try_autostart_from_nvs`). Jika berhasil, alat langsung masuk ke *runtime mode* tanpa memunculkan AP.
5. **Mode Setup (AP):** Jika autostart gagal (tidak ada konfigurasi NVS, atau sudah 5× gagal konek), alat menunggu tombol Setup ditekan selama 3 detik. Setelah itu, ESP32 membuka *Access Point* bernama `ESP32S3-Updater` dan menjalankan *Web Server* di `192.168.4.1` agar pengguna bisa memasukkan konfigurasi via browser.

**Q17b: Jika alat sudah pernah dikonfigurasi (ada NVS) tetapi WiFi/server terus gagal dikoneksi, apakah AP akan terbuka otomatis tanpa perlu menekan tombol?**
**Jawaban:**
**Ya, bisa — setelah 5 kali gagal berturut-turut.** Mekanisme ini diimplementasikan melalui counter `wifi_tries` yang disimpan di NVS (namespace `"storage"`, key `"wifi_tries"`).

Alurnya di `web_portal_try_autostart_from_nvs()`:
1. Setiap kali koneksi WiFi atau MQTT **gagal** saat autostart, firmware menambah `wifi_tries` lalu memanggil `esp_restart()` (reboot).
2. Di boot berikutnya, autostart dicoba lagi — jika gagal lagi, `wifi_tries++` dan reboot lagi.
3. Setelah **`wifi_tries >= 5`**, autostart langsung mengembalikan `ESP_FAIL` **tanpa mencoba koneksi** sekali.
4. `main.c` menerima return value gagal, jatuh ke `setup_button_wait_for_hold(3000)`, dan setelah tombol ditahan 3 detik, **AP dibuka**.

Catatan penting: Tombol **tetap harus ditekan** untuk membuka AP — device tidak membuka AP sepenuhnya otomatis. Namun device sudah berada di layar tunggu button, jadi pengguna hanya perlu tahan tombol 3 detik. Jika koneksi berhasil kembali dari Portal, `wifi_tries` direset ke 0 secara otomatis.

**Q18: Apa yang dimaksud dengan "Zone" dan bagaimana perangkat mengetahui `zone_id`-nya?**
**Jawaban:**
`zone_id` adalah pengidentifikasi unik untuk setiap unit alat (misalnya `zone_1`, `zone_2`). Ini digunakan sebagai *prefix* dari semua topik MQTT sehingga satu MQTT Broker bisa melayani banyak unit secara bersamaan tanpa konflik. Konfigurasi ini dimasukkan oleh pengguna melalui *Web Portal* saat proses *setup*, lalu disimpan ke NVS. Setiap kali alat dinyalakan ulang, ia membaca `zone_id` dari NVS dan langsung menggunakannya tanpa perlu konfigurasi ulang.

**Q19: Bagaimana alat bisa dikonfigurasi ulang (re-konfigurasi) jika sudah dalam runtime mode?**
**Jawaban:**
Ada dua mekanisme:
1. **Tombol Setup Fisik (GPIO 6):** Jika pengguna menekan dan menahan tombol selama 3 detik saat alat sedang berjalan, `setup_button_start_monitor()` akan mendeteksinya dan memicu penghentian *runtime task* serta membuka kembali Web Portal AP.
2. **MQTT Safety Clear + Restart:** Mengirim payload `CLEAR` ke topik `<zone_id>/safety/clear` akan mereset *reset counter* dan memulai ulang perangkat (`esp_restart()`). Setelah *restart*, alur auto-start dari NVS akan berjalan kembali.

---

## 8. Manajemen Bus I2C & Hardware Sharing

**Q20: Sensor pH/TDS (ADS1115) dan LCD sama-sama menggunakan bus I2C. Bagaimana konflik akses antar Task dicegah?**
**Jawaban:**
Modul `i2c_bus.c` mengimplementasikan sebuah **Shared Mutex** (`s_i2c0_mutex`) yang menjadi "gatekeeper" tunggal untuk seluruh akses ke bus `I2C_NUM_0`. Sebelum setiap task (SensorTask untuk ADS1115, atau CommTask untuk LCD) mengakses bus, ia harus terlebih dahulu memanggil `i2c_bus_lock()`. Jika task lain sedang memegang *mutex*, fungsi ini akan memblokir task peminta hingga bus tersedia. Setelah selesai, `i2c_bus_unlock()` dipanggil. Ini mencegah dua task mengirimkan sinyal I2C secara bersamaan yang akan menyebabkan data korup atau *bus error*.

**Q21: Mengapa menggunakan ADS1115 eksternal, bukan ADC internal ESP32, untuk membaca tegangan sensor pH dan TDS?**
**Jawaban:**
ADC internal ESP32 memiliki beberapa kelemahan signifikan untuk akuisisi data presisi:
1. **Non-Linearitas:** ADC internal ESP32 terkenal memiliki kurva *linearity* yang buruk, terutama di dekat ujung bawah (0V) dan atas (VCC) rentangnya. Kesalahan bisa mencapai ratusan mV tanpa kalibrasi khusus.
2. **Resolusi:** ADC internal 12-bit. ADS1115 adalah 16-bit, menghasilkan resolusi ~0.188mV/LSB pada rentang ±6.144V, jauh lebih sensitif untuk mendeteksi perubahan pH yang kecil.
3. **Interferensi WiFi:** ADC internal saluran 2 (`ADC2`) pada ESP32 tidak bisa digunakan bersamaan dengan WiFi aktif karena berbagi hardware. ADS1115 menggunakan I2C sehingga sama sekali tidak terpengaruh oleh WiFi.

---

## 9. Detail Sensor Task & Arsitektur Sampling

**Q22: Apa perbedaan `vTaskDelay` dan `vTaskDelayUntil` yang dipakai di SensorTask, dan mengapa `vTaskDelayUntil` lebih tepat?**
**Jawaban:**
- **`vTaskDelay(N)`:** Menunda task selama tepat N tick **dihitung mulai dari saat fungsi dipanggil**. Jika eksekusi isi loop memakan waktu T ms, maka periode aktual siklus menjadi `T + N`, yang menyebabkan *drift* kumulatif.
- **`vTaskDelayUntil(&last_wake, N)`:** Menunda task hingga waktu absolut `last_wake + N`. Variabel `last_wake` otomatis diperbarui setelah setiap panggilan. Jika eksekusi loop memakan waktu T ms, sistem secara otomatis hanya menunda selama `N - T` ms, sehingga periode sampling tetap konsisten `N` ms tanpa drift. Ini sangat penting untuk akurasi data *time-series* sensor.

**Q23: Bagaimana firmware menangani kegagalan sensor suhu DS18B20 yang terhubung lewat 1-Wire?**
**Jawaban:**
Terdapat mekanisme berlapis di `sensor_telemetry.c`:
1. **Fail Counter:** Setiap kali baca DS18B20 gagal, `s_temp_fail_count` bertambah. Setiap keberhasilan mereset counter ini ke nol.
2. **Threshold:** Jika `s_temp_fail_count` mencapai `SENSOR_TEMP_FAIL_THRESHOLD` (100 kali), sensor dinyatakan "dead" dan flag `water_temp_sensor_dead` diset `true`. Data suhu tidak lagi digunakan untuk pengecekan keamanan.
3. **Bus Reinit (Backoff):** Setelah sejumlah kegagalan berturut-turut (`OW_REINIT_AFTER_N_FAILURES` = 5 kali), sistem mencoba *reinit* bus 1-Wire tanpa harus restart perangkat.
4. **Dead Retry:** Setelah sensor dinyatakan mati, sistem tetap mencoba reinit secara berkala setiap `OW_DEAD_RETRY_INTERVAL` (15 sampel) untuk pulih secara mandiri jika kabel tersambung kembali.

**Q24: Apa yang dimaksud dengan "Rolling Average" pada pembacaan sensor dan mengapa digunakan?**
**Jawaban:**
*Rolling Average* (atau *Moving Average*) adalah teknik *filtering* sinyal. Alih-alih langsung menggunakan satu pembacaan ADC tunggal (yang mungkin mengandung *noise* transien akibat interferensi elektromagnetik), firmware menyimpan N sampel terakhir dalam sebuah buffer circular (`s_ph_history[SENSOR_SAMPLE_WINDOW]` dengan N=30). Nilai yang digunakan adalah rata-rata dari seluruh buffer. Efeknya adalah kurva respons yang lebih mulus, jauh lebih tahan terhadap *spike* tegangan sesaat, sebelum nilai akhir dipublikasikan ke MQTT.

---

## 10. Safety Task — Pengecekan Lanjutan

**Q25: Mengapa `SafetyTask` menggunakan `ulTaskNotifyTake` (Task Notification), bukan `vTaskDelayUntil` seperti task lainnya?**
**Jawaban:**
`ulTaskNotifyTake` berfungsi ganda: sebagai *periodic timer* sekaligus sebagai *event listener*. Jika tidak ada notifikasi yang diterima, task akan "tidur" selama `SAFETY_INTERVAL_MS` (500ms) dan otomatis bangun untuk menjalankan pengecekan rutin. Namun, jika ada bagian kode lain yang memanggil `xTaskNotify` ke *Safety Task* handle (misalnya saat ada kejadian darurat), task bisa **langsung terbangun** tanpa harus menunggu siklus 500ms berikutnya. Ini memungkinkan respons yang jauh lebih cepat terhadap kondisi kritis.

**Q26: Apa yang dimaksud dengan "pH Rate of Change Check" dan bagaimana cara kerjanya?**
**Jawaban:**
Pengecekan ini ada di `runtime_safety.c` dan bertujuan mendeteksi kondisi berbahaya seperti pompa pH yang "macet menyala" atau kebocoran kimia. Cara kerjanya menggunakan rumus **kecepatan perubahan** (turunan):

```
rate (pH/menit) = (pH_sekarang - pH_sebelumnya) / selisih_waktu_dalam_menit
```

Jika `rate` melebihi `SAFETY_PH_RATE_MAX_PER_MIN` (1.0 pH/menit ke atas), artinya pH naik terlalu cepat → kemungkinan pompa pH Up macet → trigger `SAFETY_FAULT_PH_UP`. Sebaliknya jika rate turun lebih dari 1.0 pH/menit. Penghitungan hanya dijalankan jika minimal 1 menit telah berlalu (`rate_dt_min >= 1.0f`) untuk menghindari *false positive* dari noise sesaat. Selain itu, ada **warmup period** selama 2 menit (`SAFETY_PH_WARMUP_MS`) sejak sistem nyala — selama periode ini semua pengecekan pH dinonaktifkan karena sensor kimia membutuhkan waktu stabilisasi.

**Q27: Apa itu "pH/TDS Response Timeout Fault" dan mengapa ini dianggap kondisi berbahaya?**
**Jawaban:**
Setiap kali pompa pH (Up atau Down) atau pompa nutrisi (NutA, NutB) dinyalakan, sistem mencatat waktu mulai dan nilai sensor awal. Sistem kemudian mengharapkan sensor pH atau TDS berubah dalam periode tertentu (`SAFETY_PH_RESPONSE_TIMEOUT_MS` = 90 detik, `SAFETY_TDS_RESPONSE_TIMEOUT_MS` = 120 detik).
- Jika perubahan minimal (`SAFETY_PH_RESPONSE_MIN_DELTA` = 0.03 pH, `SAFETY_TDS_RESPONSE_MIN_DELTA` = 5 ppm) tidak terjadi dalam batas waktu → trigger fault.
- Alasannya: Ini mengindikasikan pompa bermasalah (selang mampet/putus), atau volume larutan sangat besar sehingga dosis tidak efektif, yang bisa berujung pada *overdosing* tanpa efek terukur. Pengecekan ini hanya aktif jika sensor yang bersangkutan memiliki kalibrasi yang valid (`ph_valid` / `tds_valid` = true) untuk mencegah *false positive*.

---

## 11. CommTask & Resiliensi Data

**Q28: Apa yang terjadi jika SensorTask gagal membaca sensor (misal I2C *timeout*) namun CommTask tetap perlu mengirim data?**
**Jawaban:**
CommTask memiliki mekanisme **"Last Known Good Snapshot"**. Setiap kali CommTask berhasil mendapatkan data sensor yang valid dari `sensor_telemetry_get_snapshot()`, data tersebut langsung disimpan ke variabel global `s_last_good_snapshot` (dilindungi oleh `s_snapshot_mutex`). Jika pada siklus berikutnya pembacaan sensor gagal, CommTask akan mengambil data dari cache ini dan tetap mempublikasikannya ke MQTT, sambil mencatat log `"Using last known-good sensor snapshot"`. Ini mencegah dashboard di sisi server menampilkan data kosong/null karena *glitch* sementara pada hardware.

---

## 12. Desain Topik MQTT

**Q29: Mengapa topik MQTT dipisah menjadi `/raw`, `/state`, dan `/valid` untuk sensor pH dan TDS?**
**Jawaban:**
Pemisahan ini mengikuti prinsip *single responsibility* dalam desain API:
- **`/raw`** (uint16 ADC counts): Data mentah dari ADC, berguna untuk debugging, kalibrasi, atau verifikasi hardware oleh teknisi. Dikirim *selalu*, terlepas dari status kalibrasi.
- **`/state`** (float nilai fisik): Nilai yang sudah dikonversi ke satuan pH atau ppm. **Hanya dikirim jika kalibrasi valid** (`ph_valid = true`). Subscriber (Backend/Node-RED) hanya perlu subscribe topik ini untuk mendapatkan nilai siap pakai.
- **`/valid`** (true/false, **retained**): Flag boolean yang menunjukkan apakah data dapat dipercaya. Dikirim dengan `retain=1` sehingga setiap subscriber baru langsung tahu kondisi validitas sensor tanpa menunggu siklus publish berikutnya.

**Q30: Bagaimana topik `<zone_id>/command` (multi-channel) berbeda dengan topik `<zone_id>/<channel>/command` (single-channel)?**
**Jawaban:**
- **Single-channel** (misal `zone_1/PerNutA/command`): Payload langsung adalah perintah (`ON`, `OFF`, `PULSE:5000`). Server MQTT harus mengirim satu pesan per aktuator.
- **Multi-channel** (misal `zone_1/command`): Payload berisi nama channel dan perintah dalam satu string (`PerNutA PULSE 5000`). Format ini memungkinkan server mengirim *compound command* dalam satu pesan MQTT. Firmware mem-parse payload menggunakan `strtok_r` (thread-safe) untuk memisahkan nama channel, jenis aksi, dan durasi (jika PULSE), kemudian memasukkannya ke dosing queue.

---

## 13. DEV_MODE, Override & Pengujian

**Q31: Apa itu `DEV_MODE` pada `safety_config.h` dan kapan harus digunakan?**
**Jawaban:**
`DEV_MODE` adalah *compile-time flag* (didefinisikan di `safety_config.h` sebagai `#define DEV_MODE 0`). Jika diset ke `1`:
- **Semua Safety Fault dinonaktifkan** secara global (fungsi `safety_fault_set` menjadi no-op).
- **Safe Mode tidak pernah aktif** (gate selalu terbuka).
- **Boot Fault recording dilewati**.
- **Emergency Stop** tidak memicu fault nyata (hanya membuat LED berkedip).

Ini digunakan selama **pengembangan dan pengujian di laboratorium** agar pengembang bisa menguji fungsionalitas aktuator tanpa risiko sistem terus-menerus masuk *Safe Mode* karena sensor belum terkalibrasi. `safety_config.h` bahkan memuat peringatan eksplisit: `WARNING: NEVER deploy to production with DEV_MODE = 1`.

**Q32: Apa fungsi fitur "Override Mode" yang bisa diaktifkan dari MQTT?**
**Jawaban:**
Override Mode diaktifkan dengan mengirim payload `ON` ke topik `<zone_id>/override/command`. Ketika aktif:
- Pengecekan safety untuk **sensor** (pH OOR, TDS OOR, rate-of-change, dose response timeout) **ditangguhkan sementara**.
- Pengecekan *dose duration limit* dan *fill timeout* juga **ditangguhkan**.
- Petugas dapat mengoperasikan pompa secara manual untuk keperluan maintenance (misal flushing pipa, kalibrasi manual) tanpa sistem langsung masuk Safe Mode.
- **Tidak menonaktifkan** proteksi keras seperti WDT atau interlock kimia pH Up+Down.

Dinonaktifkan dengan mengirim `OFF` ke topik yang sama.

---

## 14. Manajemen Fault & Safe Mode Recovery

**Q33: Bagaimana cara "keluar" dari Safe Mode setelah fault terjadi?**
**Jawaban:**
Ada dua mekanisme yang berbeda, dikirim ke topik `<zone_id>/safety/clear`:
1. **Payload `CLEAR` atau `ALL`:** Me-reset *reset counter* (`s_reset_count`, `s_wdt_reset_count`) lalu langsung memanggil `esp_restart()` untuk merestart ulang perangkat sepenuhnya. Cara paling bersih dan aman.
2. **Payload `MASK:<hex>`:** Membersihkan fault spesifik berdasarkan bitmask hexadecimal tanpa restart. Jika semua fault berhasil dihapus (`new_mask == 0`), `safe_mode` diset `false` dan gate kembali dibuka, sehingga aktuator bisa dioperasikan lagi. Berguna jika fault sudah teratasi (misal sensor sudah terkalibrasi) dan operator tidak ingin merestart alat.

**Q34: Bagaimana sistem mendeteksi dan menangani kondisi *brownout* (tegangan drop) pada power supply?**
**Jawaban:**
ESP32 memiliki detektor *brownout* bawaan. Jika tegangan VCC turun di bawah ambang batas aman, chip secara otomatis melakukan reset (`ESP_RST_BROWNOUT`). Firmware mendeteksi ini pada saat boot berikutnya melalui `esp_reset_reason()` di fungsi `runtime_tasks_record_boot_faults()`. Jika terdeteksi brownout reset, flag `SAFETY_FAULT_POWER` langsung ditambahkan ke `boot_faults`. Ketika runtime task dimulai, fault ini langsung diterapkan (`runtime_apply_boot_faults`), sehingga sistem masuk Safe Mode dan melaporkan `SAFETY_FAULT_POWER` ke server via MQTT, mencegah aktuator beroperasi pada kondisi supply yang tidak stabil.

Selain itu, jika total jumlah reset tidak normal dalam periode `SAFETY_RESET_WINDOW_SEC` (300 detik) melebihi `SAFETY_RESET_MAX_COUNT` (5 kali), sistem juga memicu `SAFETY_FAULT_POWER` — mengindikasikan instabilitas power supply yang lebih serius.

---

## 15. Stack & Memory Management

**Q35: Bagaimana firmware memantau penggunaan *stack* setiap Task untuk mendeteksi potensi *stack overflow*?**
**Jawaban:**
Setiap Task memanggil `uxTaskGetStackHighWaterMark(NULL)` di awal inisialisasi dan secara berkala setiap 100 iterasi loop (khusus SensorTask). Fungsi ini mengembalikan jumlah minimum byte *stack* yang tersisa sejak task dimulai (FreeRTOS mengisi stack dengan pola `0xA5` dan menghitung berapa yang belum tertulis). Log hasilnya menggunakan `ESP_LOGI`. Jika angka ini mendekati 0, ukuran stack saat pembuatan task (`xTaskCreate`) harus ditingkatkan. Contoh: SensorTask memiliki stack 8192 byte (`SENSOR_TASK_STACK 8192`) karena membutuhkan lebih banyak untuk operasi ADS1115 + DS18B20 + logging.

**Q36: Mengapa ada komentar di kode bahwa stack SensorTask dinaikkan dari 6144 menjadi 8192?**
**Jawaban:**
Komentar dalam kode menyebutkan: *"Increased from 6144 — ADS1115 + DS18B20 + logging needs more headroom."* Ini adalah contoh nyata *iterative debugging* menggunakan stack watermark monitoring. Ketika SensorTask diimplementasikan pertama kali dengan stack 6144, pengembang memantau watermark dan menemukan sisa stack terlalu sedikit. Daripada mengambil risiko *stack overflow* yang menyebabkan crash acak yang sulit di-debug, ukuran dinaikkan ke 8192 sebagai margin aman.

---

## 16. Pertanyaan Konseptual & Desain

**Q37: Apa perbedaan antara `portENTER_CRITICAL` dan `xSemaphoreTake`, dan kapan masing-masing digunakan?**
**Jawaban:**
- **`portENTER_CRITICAL` / `portEXIT_CRITICAL`:** Menonaktifkan *interrupt* dan preemption pada core yang sedang berjalan. Harus **sangat singkat** (hanya beberapa instruksi). Digunakan untuk melindungi variabel tunggal atau operasi atomik kecil, seperti membaca/menulis satu flag atau pointer. Tidak bisa digunakan untuk operasi yang memblokir.
- **`xSemaphoreTake` (Mutex):** Operasi *blocking* yang aman. Task yang gagal mendapatkan mutex akan masuk ke kondisi *suspended* dan dibangunkan kembali oleh scheduler ketika mutex tersedia. Digunakan untuk melindungi blok kode yang lebih panjang dan bisa memblokir seperti akses I2C, akses ke struct sensor snapshot, atau operasi MQTT.

**Q38: Mengapa sistem ini menggunakan dua mekanisme Watchdog sekaligus (Software Heartbeat + Hardware ESP Task WDT)?**
**Jawaban:**
Kedua mekanisme melindungi dari kondisi yang berbeda:
- **Hardware Task WDT (8 detik):** Menangkap kondisi *hard hang* — ketika sebuah task benar-benar terjebak dalam *infinite loop* atau menunggu sesuatu selamanya sehingga tidak pernah memanggil `esp_task_wdt_reset()`. ESP-IDF langsung melakukan Panic dan Reset.
- **Software Heartbeat (15 detik):** Menangkap kondisi yang lebih subtle — ketika task masih *hidup* dan me-*kick* WDT hardware, tetapi tidak lagi memproses pekerjaan dengan benar (misal DosingTask berhenti memproses queue karena resource deadlock namun masih dalam loop-nya). SafetyTask mendeteksi ini dan memicu Safe Mode tanpa perlu restart keras.

Kombinasi keduanya memberikan lapisan perlindungan yang jauh lebih komprehensif dibandingkan hanya satu mekanisme saja.

**Q39: Apa yang terjadi pada antrian dosing jika queue penuh dan perintah baru masuk?**
**Jawaban:**
Firmware menerapkan strategi **"latest command wins"** (perintah terbaru yang menang). Jika `s_dosing_queue` (kapasitas 64 item) penuh saat perintah baru tiba, sistem akan:
1. Menghitung jumlah item lama yang antri (`uxQueueMessagesWaiting`).
2. Membuang (flush) **semua** item lama dari antrian dengan `xQueueReset`.
3. Segera memasukkan perintah terbaru yang baru datang.
4. Mencatat log peringatan berapa item yang dibuang.

Strategi ini memastikan bahwa perintah paling baru dari server selalu dieksekusi, menghindari eksekusi "perintah basi" yang mungkin sudah tidak relevan. Namun, jika dipanggil dari konteks ISR, perintah baru langsung dibuang (karena `xQueueReset` tidak aman dipanggil dari ISR).

**Q40: Bagaimana firmware memastikan CirculationPump selalu menyala dan tidak bisa dimatikan?**
**Jawaban:**
Pompa sirkulasi (`ACTUATOR_CHANNEL_CIRCULATION_PUMP`, GPIO 13) mendapat perlakuan khusus di beberapa level:
1. Pada `init_gpios_once()` di `actuator_control.c`, pin 13 diinisialisasi sebagai OUTPUT dan langsung diset `HIGH` (pompa ON).
2. Pada `actuator_control_apply_state()`, ada guard eksplisit: `if (channel == ACTUATOR_CHANNEL_CIRCULATION_PUMP) return ESP_OK;` — instruksi untuk mematikan pompa langsung diabaikan.
3. Pada `actuator_control_deinit()`, semua pin aktuator lain diset LOW, tetapi pompa sirkulasi tetap diset `HIGH`: `gpio_set_level((gpio_num_t)PIN_CIRCULATION_PUMP, 1);`.
4. Pompa ini juga **dikecualikan** dari daftar `s_gpio_list[]` yang diinisialisasi ke LOW di awal, karena ia punya logika inisialisasi sendiri yang berbeda.

---

## 17. Design Choices — Mengapa dan Bagaimana Membela Keputusan Desain

> Bagian ini khusus membahas pertanyaan-pertanyaan kritis tentang *trade-off* desain yang telah dibuat, beserta argumentasi teknisnya.

---

### 17A. Arsitektur Sistem

**Q41: Mengapa sistem dibuat dengan 4 task, bukan lebih sedikit (misalnya 2 task saja)?**
**Jawaban & Defense:**
Desain 4 task bukan kesewenangan, melainkan mengikuti prinsip *Separation of Concerns* berdasarkan **sifat timing** masing-masing subsistem:

| Task | Sifat Timing | Alasan Dipisah |
|---|---|---|
| Safety | Event-driven + periodik cepat | Harus bisa preempt kapan saja — tidak boleh menunggu I/O selesai |
| Dosing | Event-driven (queue) | Tidak boleh blocking sambil menjalankan pompa |
| Sensor | Periodik tetap (2 detik) | Sampling rate konstan — tidak boleh terganggu network latency |
| Comm | Periodik lambat (3 detik) | Network I/O bisa latency tinggi — tidak boleh block task kritis |

Jika digabung menjadi 2 task saja, misal "control" dan "comm", maka ketika MQTT publish membutuhkan 1 detik timeout karena jaringan lambat, seluruh sensor sampling dan safety check akan tertunda, yang langsung membahayakan sistem fisik. FreeRTOS scheduler memisahkan ini secara hardware (preemptive scheduling).

---

**Q42: Mengapa tidak menggunakan `esp_event` loop sebagai pengganti FreeRTOS Queue untuk komunikasi antar-komponen?**
**Jawaban & Defense:**
`esp_event` adalah event loop *single-threaded* berbasis callback — semua callback antri di satu thread yang sama, sehingga **tidak ada prioritas eksekusi yang bisa diatur** antar callback. Jika callback MQTT (terima perintah) dan callback safety (sensor OOR) tiba bersamaan, keduanya berlomba di satu antrian event tanpa bisa dikendalikan.

Dengan FreeRTOS Queue, perintah dari MQTT callback dikirim sebagai data *non-blocking* ke dosing queue, sementara safety task tetap berjalan di core dan prioritasnya sendiri secara benar-benar paralel di ESP32 dual-core. Prioritas eksekusi terjamin oleh scheduler FreeRTOS.

---

**Q43: Mengapa MQTT dipilih sebagai protokol komunikasi, bukan HTTP REST atau WebSocket langsung?**
**Jawaban & Defense:**
Tiga alasan utama:

1. **Publish-Subscribe vs. Request-Response:** HTTP REST mengharuskan device terus-menerus polling atau server aktif memicu request. MQTT memungkinkan server *push* perintah kapan saja tanpa device harus memintanya — esensial untuk perintah darurat (`emergency stop`) yang tidak bisa menunggu siklus polling berikutnya.

2. **Overhead rendah pada embedded:** MQTT frame seminimal 2 byte header. HTTP minimal ratusan byte untuk header saja. Pada ESP32 dengan RAM terbatas, ini berpengaruh pada memori dan CPU untuk parsing.

3. **Broker sebagai mediator resilient:** Jika device sementara offline, broker MQTT menyimpan pesan (retain/QoS 1). Ketika device reconnect, state dan perintah masih bisa diterima. HTTP REST tidak memiliki mekanisme ini secara built-in.

Kelemahan yang diterima: membutuhkan broker di infrastruktur. Defense: broker MQTT (Mosquitto) ringan dan bisa berjalan bahkan di Raspberry Pi atau PC lab yang sudah ada.

---

### 17B. Keputusan Parameter Safety

**Q44: Mengapa batas waktu warmup pH dibuat 2 menit (`SAFETY_PH_WARMUP_MS = 120000`)? Bukankah lebih aman jika lebih pendek?**
**Jawaban & Defense:**
Sensor pH elektroda membutuhkan waktu stabilisasi setelah power-on karena elektroda referensi harus mencapai keseimbangan elektrokimia dengan larutan. Jika pengecekan langsung diaktifkan pada detik pertama, sensor mungkin masih membaca nilai drift — misalnya pH terbaca 3.2 padahal sebenarnya 6.5 — dan sistem akan langsung masuk Safe Mode karena `SAFETY_PH_MIN = 4.5`. Ini *false positive* yang merepotkan.

2 menit dipilih berdasarkan datasheet umum elektroda pH yang menyarankan 1-3 menit stabilisasi. Selama 2 menit pertama, sistem baru saja boot dan dosing belum diperintahkan server, sehingga risiko overdosing pada phase warmup sangat minimal.

---

**Q45: Mengapa `SAFETY_MAX_DOSE_A_MS` dan `SAFETY_MAX_DOSE_B_MS` dibuat 2 menit (120.000ms), sedangkan `SAFETY_MAX_DOSE_PH_UP/DOWN_MS` hanya 1 menit (60.000ms)?**
**Jawaban & Defense:**
Perbedaan ini mencerminkan **perbedaan dampak kimia** jika dosis berlebihan:

- **Nutrisi (NutA, NutB):** Kelebihan nutrisi berbahaya bagi tanaman secara kumulatif, namun prosesnya lambat. Volume larutan besar juga membutuhkan waktu sirkulasi agar TDS berubah terukur — lebih banyak toleransi waktu dibutuhkan.

- **pH (Up/Down):** Perubahan pH bisa terjadi sangat cepat. pH di bawah 4 atau di atas 9 merusak akar tanaman dalam hitungan jam, dan interaksi kimia langsung (korosi, presipitasi mineral) terjadi segera. Batas lebih ketat (1 menit) proporsional dengan risikonya yang lebih tinggi.

---

**Q46: Mengapa `SAFETY_HEARTBEAT_TIMEOUT_MS = 15000` (15 detik)? Bukankah lebih cepat lebih baik?**
**Jawaban & Defense:**
15 detik bukan angka sembarangan. Setiap task memiliki periode loop masing-masing — SensorTask (2 detik), CommTask (3 detik). CommTask yang sedang dalam MQTT publish dengan network latency tinggi (TCP timeout bisa 3-5 detik) bisa dianggap dead jika threshold terlalu rendah. Ini *false positive* watchdog yang berbahaya — sistem masuk Safe Mode padahal task sebenarnya masih sehat.

15 detik = buffer 3-5× periode terlama, memberikan ruang untuk keadaan network lambat tanpa mengorbankan responsivitas terhadap task yang benar-benar hang. Hardware TWDT (8 detik) tetap ada sebagai lapis pertama — software heartbeat 15 detik adalah lapis kedua untuk kondisi lebih subtle.

---

**Q47: Mengapa threshold `SAFETY_PH_RATE_MAX_PER_MIN = 1.0` (1.0 pH/menit)?**
**Jawaban & Defense:**
Angka ini didasarkan pada karakteristik fisik sistem:

- Pompa peristaltik pH dengan laju dosis normal menghasilkan perubahan ~0.1-0.3 pH/menit pada volume larutan standar (100-200 liter).
- Perubahan >1.0 pH/menit mengindikasikan kondisi abnormal: pompa terlalu besar, volume larutan sangat kecil, atau pompa stuck ON.
- 1.0 dipilih cukup jauh dari operasi normal (0.3) untuk tidak menghasilkan false positive, namun cukup sensitif untuk mendeteksi anomali nyata.

Nilai ini bisa disesuaikan di `safety_config.h` tanpa mengubah logika kode jika profil sistem berubah.

---

### 17C. Keputusan Hardware & Peripheral

**Q48: Mengapa menggunakan ADS1115 dengan 16-bit resolusi? Bukankah 12-bit dari ADC internal sudah cukup untuk pH (0-14)?**
**Jawaban & Defense:**
Mari hitung secara konkret:

- **ADC internal 12-bit pada ±3.3V:** ~0.8 mV/LSB → ~0.08 pH/LSB (untuk sensor pH dengan output 0-3.3V pada range pH 0-14).
- **ADS1115 16-bit pada ±4.096V:** ~0.125 mV/LSB → ~0.013 pH/LSB → resolusi 6× lebih tinggi.

Lebih penting: ADC internal ESP32 memiliki non-linearity yang diketahui menyebabkan error ±5-10% dan ADC2 tidak bisa digunakan bersamaan dengan WiFi aktif (berbagi hardware). ADS1115 menggunakan I2C dan sama sekali tidak terpengaruh WiFi — krusial untuk sistem yang selalu terhubung.

---

**Q49: Mengapa bus I2C yang sama (I2C_NUM_0) digunakan untuk LCD dan ADS1115? Bukankah bus terpisah lebih aman?**
**Jawaban & Defense:**
Secara ideal bus terpisah lebih bersih. Namun ESP32-S3 memiliki keterbatasan alokasi GPIO — dalam desain PCB yang ada, GPIO untuk I2C kedua sudah dialokasikan untuk keperluan lain. Berbagi bus adalah keputusan *pragmatis*.

Risikonya diminimalkan dengan mutex `s_i2c0_mutex` di `i2c_bus.c`. Satu-satunya downside nyata adalah potensi latency tambahan pada sensor read jika LCD sedang update. Karena frekuensi LCD update (event-driven, jarang) dan sensor read (2 detik) berbeda, kemungkinan konflik ada tapi ditangani dengan aman oleh mutex.

---

**Q50: Mengapa DS18B20 (1-Wire) dipilih untuk sensor suhu air, bukan NTC thermistor analog yang lebih murah?**
**Jawaban & Defense:**
Tiga keunggulan DS18B20 untuk lingkungan hidroponik:

1. **Waterproof out-of-the-box:** DS18B20 tersedia dalam enkapsulasi stainless steel kedap air, dirancang untuk immersion dalam cairan. NTC thermistor butuh enclosure khusus.
2. **Self-calibrated:** DS18B20 dikalibrasi pabrik dengan akurasi ±0.5°C tanpa kalibrasi pengguna. NTC thermistor membutuhkan karakterisasi kurva Steinhart-Hart dan kalibrasi manual.
3. **1-Wire protocol:** Hanya butuh 1 GPIO, dan mendukung multiple sensor pada satu bus untuk redundansi masa depan.

Biaya lebih tinggi dari NTC, namun untuk sistem yang berjalan tanpa pengawasan, keandalan lebih penting dari penghematan kecil pada BOM.

---

### 17D. Keputusan Software & Protokol

**Q51: Mengapa kalibrasi sensor menggunakan model linear (`value = slope * raw + offset`), bukan polinomial yang lebih akurat?**
**Jawaban & Defense:**
Model linear dipilih atas dasar trade-off akurasi vs. kompleksitas yang disengaja:

1. **Cukup akurat untuk range operasi:** Sensor pH dan TDS berperilaku mendekati linear di *range operasi hidroponik* (pH 5.5-7.5, TDS 200-1500 ppm). Nonlinearitas signifikan hanya di ujung ekstrem skala.
2. **Mudah diinterpretasi operator:** Slope dan offset memiliki makna fisik. Model polinomial orde-3 membutuhkan 4 parameter yang tidak intuitif.
3. **Kalibrasi 2-titik mudah dilakukan di lapangan:** Operator cukup memasukkan dua cairan referensi. Kalibrasi polinomial butuh minimal 3-4 titik.
4. **Komputasi minimal:** Float multiply-add vs. polynomial evaluation — perbedaannya signifikan pada MCU.

---

**Q52: Mengapa validasi kalibrasi dilakukan di device (firmware), bukan hanya di server (Node-RED)?**
**Jawaban & Defense:**
Validasi di device adalah *defense-in-depth* (pertahanan berlapis). Server memvalidasi untuk UI, namun firmware harus melindungi dirinya sendiri karena:

1. **Koneksi bisa terputus:** Jika kalibrasi korup masuk ke NVS (bit-flip saat flash write), server tidak bisa mencegah device menggunakannya saat boot offline.
2. **NVS integrity:** Device memvalidasi *range plausibility* (slope/offset dalam batas fisik masuk akal) saat load dari NVS. Ini mendeteksi bit-flip atau partial write di flash.
3. **Single Responsibility yang independen:** Device bertanggung jawab atas integritas datanya sendiri, server bertanggung jawab atas logika bisnis. Keduanya memvalidasi secara independen — tidak ada single point of failure.

---

**Q53: Mengapa `portENTER_CRITICAL` digunakan alih-alih mutex untuk melindungi akses ke state aktuator (`s_channel_state_on[]`)?**
**Jawaban & Defense:**
`s_channel_state_on[]` adalah array boolean yang dibaca dan ditulis dengan operasi sangat singkat (satu assignment atau satu read). Karakteristik ini cocok untuk critical section:

1. **Operasi sangat singkat:** Tidak ada I/O, tidak ada pemanggilan fungsi di dalam critical section — hanya satu operasi array. Durasi interrupt disabled < 1 µs.
2. **Tidak ada blocking:** Mutex bisa membuat task suspended. Critical section tidak blocking — hanya menonaktifkan interrupt sesaat. Untuk operasi 1-instruksi, ini lebih efisien.
3. **Konteks ISR-safe:** Jika read diperlukan dari timer callback (ISR context), mutex tidak bisa digunakan dari ISR. Critical section adalah satu-satunya pilihan.

---

**Q54: Mengapa strategi "latest wins" diterapkan saat dosing queue penuh, bukan FIFO murni?**
**Jawaban & Defense:**
Jika queue penuh dengan 64 perintah lama yang belum dieksekusi, kondisi ini mengindikasikan **perintah lama sudah stale**:

- Server mungkin sudah mengirim `ON` kemudian `OFF` secara cepat dalam burst. Jika FIFO murni, device akan mengeksekusi 64 perintah lama sebelum sampai ke perintah `OFF` yang kritis.
- Dengan "latest wins": semua perintah lama dibuang, perintah terbaru (mencerminkan *intent* operator saat ini) langsung dieksekusi.

Dalam konteks safety: jika operator mengirim "emergency stop" (semua OFF), perintah ini harus segera dieksekusi, tidak antri di belakang 64 perintah lama. Queue reset + enqueue latest memastikan ini.

---

**Q55: Mengapa `wifi_tries` dibatasi hanya 5 kali sebelum jatuh ke setup mode, bukan lebih banyak?**
**Jawaban & Defense:**
Trade-off antara uptime dan serviceability:

- **Terlalu sedikit (misal 2):** Gangguan jaringan sementara (router reboot 30 detik) menyebabkan device ke setup mode, mengganggu operasi yang tidak perlu.
- **Terlalu banyak atau infinite:** Jika WiFi SSID atau password berubah, device tidak pernah bisa dikonfigurasi ulang secara mandiri — operator harus menunggu sangat lama.
- **5 kali + reboot dengan timeout 2 menit:** Total ~10 menit. Cukup untuk menunggu router reboot, tapi tidak terlalu lama untuk operator yang sadar ada masalah konfigurasi.

Setelah 5 kali, konfigurasi NVS lama tetap tersimpan — device hanya menunggu tombol untuk buka AP. Operator tidak perlu memasukkan zone ID ulang dari awal.

---

**Q56: Mengapa `esp_restart()` dipanggil setelah gagal connect, bukan hanya retry WiFi di tempat?**
**Jawaban & Defense:**
ESP32 WiFi stack pada kondisi DHCP timeout atau AP hilang bisa meninggalkan state internal yang korup atau memory leak kecil. `esp_restart()` memberikan *clean slate* — semua state di-reset ke kondisi awal yang diketahui bersih. Ini praktik umum pada embedded system untuk menghadapi kegagalan yang tidak dapat diatasi secara graceful di level software.

Alternatif (retry in-place) memerlukan urutan `esp_wifi_disconnect()` → `esp_wifi_stop()` → `esp_wifi_start()` yang bisa tidak deterministik pada WiFi stack dan berpotensi menyebabkan state lebih korup. Reboot bersih lebih aman dan lebih mudah di-debug.

---

**Q57: Mengapa konfigurasi (ssid, password, broker) baru disimpan ke NVS setelah runtime berhasil start, bukan langsung saat terima dari form?**
**Jawaban & Defense:**
Ini adalah prinsip **transactional commitment** — hanya commit jika seluruh operasi berhasil:

1. Coba connect WiFi → jika gagal, kembalikan error ke browser (**tidak simpan NVS**)
2. Coba connect MQTT → jika gagal, kembalikan error ke browser (**tidak simpan NVS**)
3. Coba start runtime → jika gagal, kembalikan error ke browser (**tidak simpan NVS**)
4. **Baru simpan ke NVS setelah semua berhasil**

Jika disimpan di awal kemudian MQTT gagal, device di boot berikutnya akan autostart menggunakan konfigurasi WiFi yang benar tapi MQTT yang salah, looping selama `wifi_tries` belum habis. Dengan menyimpan hanya setelah semua berhasil, NVS selalu berisi konfigurasi yang terverifikasi end-to-end.

---

**Q58: Mengapa LCD event drain dilakukan di CommTask, bukan di DosingTask yang berdekatan dengan eksekusi aktuator?**
**Jawaban & Defense:**
DosingTask (prioritas 3) bertanggung jawab atas eksekusi hardware yang time-critical. LCD driver menggunakan I2C yang bisa membutuhkan beberapa milidetik dan berbagi bus dengan ADS1115 (locked via mutex).

Jika DosingTask langsung melakukan I2C LCD write setelah setiap perintah, variabilitas latency LCD bisa mempengaruhi timing pulse aktuator (khususnya `ACTUATOR_ACTION_PULSE` yang bergantung pada `vTaskDelay` presisi). Misalnya, `PULSE 2000ms` bisa menjadi `PULSE 2050ms`.

CommTask (prioritas 2) menangani LCD drain secara non-blocking (`xQueueReceive` timeout 0) setiap siklus 3 detiknya — timing tidak kritis. Ini memisahkan concern hardware real-time (DosingTask) dari concern display informatif (CommTask).

---

**Q59: Mengapa DS18B20 menggunakan pola "trigger-then-read" antar siklus, bukan "trigger-and-wait" dalam satu siklus?**
**Jawaban & Defense:**
DS18B20 membutuhkan 750ms untuk menyelesaikan konversi suhu 12-bit. Jika pola "trigger-and-wait" digunakan, setiap siklus sampling akan memakan 750ms hanya untuk menunggu sensor — membuang 37.5% waktu pada siklus 2 detik.

Pola "trigger-then-read" memisahkan dua operasi ke siklus berbeda:
- **Akhir siklus N:** Trigger conversion (non-blocking, <1ms)
- **Awal siklus N+1 (2 detik kemudian):** Read hasil konversi (sudah ready)

SensorTask tidur 2 detik, DS18B20 bekerja selama itu. Tidak ada waktu yang terbuang. Ini contoh klasik *overlap* I/O dan computation pada embedded systems.

---

**Q60: Mengapa `SAFETY_PH_FROZEN_EPSILON = 0.005` digunakan untuk deteksi frozen pH, bukan equality check (`==`) seperti TDS?**
**Jawaban & Defense:**
Ini perbedaan fundamental antara sensor floating-point (pH) dan integer (TDS):

- **TDS:** Hasil akhir adalah integer dari rolling average integer — sangat mungkin benar-benar identik antar sample jika kondisi tidak berubah. Equality check (`==`) valid.
- **pH:** Dihitung dari ADC float dengan slope/offset float, melalui rolling average float. Floating-point arithmetic menghasilkan micro-variation di bit terakhir (machine epsilon ~1e-7) bahkan pada input identik. Menggunakan `==` pada float akan **tidak pernah** mendeteksi frozen karena selalu ada floating-point rounding noise.

`SAFETY_PH_FROZEN_EPSILON = 0.005` dipilih:
- Jauh di atas machine epsilon (~1e-7) → floating-point noise tidak memicu false positive.
- Jauh di bawah perubahan pH nyata (~0.05 untuk kondisi larutan tenang) → sensor yang benar-benar tidak berubah terdeteksi setelah 300 sample (10 menit).
