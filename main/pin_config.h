#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

#include "driver/i2c.h"
#include "esp_adc/adc_oneshot.h"

/* Actuator output pins */
#define PIN_ACTUATOR_VALVE        12
#define PIN_ACTUATOR_PER_NUTA     13
#define PIN_ACTUATOR_PER_NUTB     14
#define PIN_ACTUATOR_PER_PH_UP    17
#define PIN_ACTUATOR_PER_PH_DOWN  18

/* Sensor pins */
#define PIN_SENSOR_WATER_LEVEL             1
#define PIN_SENSOR_WATER_TEMP_ADC_UNIT     ADC_UNIT_1
#define PIN_SENSOR_WATER_TEMP_ADC_CHANNEL  ADC_CHANNEL_3 /* GPIO4 on ESP32-S3 */

/* LCD I2C pins */
#define PIN_LCD_I2C_PORT     I2C_NUM_0
#define PIN_LCD_I2C_SDA      15
#define PIN_LCD_I2C_SCL      16
#define PIN_LCD_I2C_FREQ_HZ  100000

#endif