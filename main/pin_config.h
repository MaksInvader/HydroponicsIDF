#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

#include "driver/i2c.h"

/* Actuator output pins */
#define PIN_ACTUATOR_VALVE        12
#define PIN_ACTUATOR_PER_NUTA     13
#define PIN_ACTUATOR_PER_NUTB     14
#define PIN_ACTUATOR_PER_PH_UP    17
#define PIN_ACTUATOR_PER_PH_DOWN  18

/* Relay output pins (generic, server-controlled) */
#define PIN_RELAY_1               19
#define PIN_RELAY_2               20
#define PIN_RELAY_3               21
#define PIN_RELAY_4               38

/* Status / indicator LEDs */
#define PIN_LED_CONNECTION        39   /* HIGH = broker connected */
#define PIN_LED_FAULT             40   /* HIGH = fault or emergency active */

/* Reserved binary output */
#define PIN_RESERVE_BINARY        41

/* Sensor pins */
#define PIN_SENSOR_WATER_LEVEL             1
#define PIN_SENSOR_WATER_TEMP_ONEWIRE      4   /* GPIO4 — DS18B20 1-Wire data line */

/* pH and TDS sensors — direct GPIO digital output (not ADC).
 * Set these to the GPIO numbers your modules are wired to. */
#define PIN_SENSOR_PH   5   /* TODO: replace with your actual GPIO number */
#define PIN_SENSOR_TDS  6   /* TODO: replace with your actual GPIO number */

/* LCD I2C pins */
#define PIN_LCD_I2C_PORT     I2C_NUM_0
#define PIN_LCD_I2C_SDA      15
#define PIN_LCD_I2C_SCL      16
#define PIN_LCD_I2C_FREQ_HZ  100000

#endif