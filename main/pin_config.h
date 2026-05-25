#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

#include "driver/i2c.h"

/* Actuator output pins */
#define PIN_ACTUATOR_VALVE        12
#define PIN_ACTUATOR_PER_NUTA     41
#define PIN_ACTUATOR_PER_NUTB     40
#define PIN_ACTUATOR_PER_PH_UP    39
#define PIN_ACTUATOR_PER_PH_DOWN  38

/* Circulation pump — always HIGH, never driven LOW by firmware */
#define PIN_CIRCULATION_PUMP      45

/* Relay output pins (generic, server-controlled) */
#define PIN_RELAY_1               13
#define PIN_RELAY_2               14
#define PIN_RELAY_3               21
#define PIN_RELAY_4               47

/* Status / indicator LEDs */
//#pilot1
#define PIN_LED_CONNECTION        2   /* HIGH = broker connected */
#define PIN_LED_FAULT             42   /* HIGH = fault or emergency active */

/* Reserved binary output */
#define PIN_RESERVE_BINARY        41

/* Sensor pins */
#define PIN_SENSOR_WATER_LEVEL             5
#define PIN_SENSOR_WATER_TEMP_ONEWIRE      8   /* GPIO4 — DS18B20 1-Wire data line */

/* pH and TDS sensors — ADS1115 16-bit ADC over I2C.
 * AIN0 = pH,  AIN1 = TDS.
 * ADDR pin tied to GND → I2C address 0x48. */
#define PIN_ADS1115_I2C_PORT   I2C_NUM_0
#define PIN_ADS1115_I2C_SDA    16
#define PIN_ADS1115_I2C_SCL    15
#define PIN_ADS1115_I2C_ADDR   0x48

/* LCD I2C pins */
#define PIN_LCD_I2C_PORT     I2C_NUM_0
#define PIN_LCD_I2C_SDA      16
#define PIN_LCD_I2C_SCL      15
#define PIN_LCD_I2C_FREQ_HZ  100000

#endif