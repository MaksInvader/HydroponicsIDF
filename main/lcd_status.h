#ifndef LCD_STATUS_H
#define LCD_STATUS_H

#include "esp_err.h"

esp_err_t lcd_status_init(void);
void lcd_status_show_wifi_and_broker(const char *ssid, const char *broker_ip);
void lcd_status_show_mqtt_connected(void);
void lcd_status_show_zone_overview(const char *zone_id, const char *zone_name, const char *last_actuator);
void lcd_status_show_actuator_event(const char *zone_id, const char *zone_name, const char *actuator_name, const char *sensor_text, const char *state_text);

#endif