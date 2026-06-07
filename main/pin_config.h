#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

#include "driver/i2c.h"
#include "driver/uart.h"   /* needed for UART_NUM_1 when PH_SOURCE_USE_SERIAL = 1 */

/* --------------------------------------------------------------------------
 * Hardware Enable Flags
 * Set to 1 to enable, 0 to disable at compile time
 * -------------------------------------------------------------------------- */
#define ENABLE_LCD              1
#define ENABLE_TDS_SENSOR       1
#define ENABLE_WATER_TEMP       1
#define ENABLE_WATER_LEVEL      1
#define ENABLE_ACTUATORS        1
#define ENABLE_INDICATOR_LEDS   1
#define ENABLE_SETUP_BUTTON     0

/* Actuator output pins */
#define PIN_ACTUATOR_VALVE        12
#define PIN_ACTUATOR_PER_NUTA     41
#define PIN_ACTUATOR_PER_NUTB     40
#define PIN_ACTUATOR_PER_PH_UP    39
#define PIN_ACTUATOR_PER_PH_DOWN  38

/* Circulation pump — always HIGH, never driven LOW by firmware */
#define PIN_CIRCULATION_PUMP      13

/* Relay output pins (generic, server-controlled) */
#define PIN_RELAY_1               47
#define PIN_RELAY_2               14
#define PIN_RELAY_3               21
#define PIN_RELAY_4               45

/* Status / indicator LEDs */
//#pilot1
#define PIN_LED_CONNECTION        1
#define PIN_LED_FAULT             2

/* Reserved binary output */
#define PIN_RESERVE_BINARY        42

/* Sensor pins */
#define PIN_SENSOR_WATER_LEVEL             5
#define PIN_SENSOR_WATER_TEMP_ONEWIRE      8

/* pH and TDS sensors — ADS1115 16-bit ADC over I2C.
 * AIN1 = TDS (moved from AIN0), AIN0 = unused.
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

/* Setup / reconfiguration button */
#define PIN_SETUP_BUTTON     6

/* --------------------------------------------------------------------------
 * pH input source selector
 *   0 = Unused/unplugged (pH sensor disabled)
 *   1 = UART serial module (e.g. Atlas EZO-pH, DFRobot gravity serial)
 *   2 = ADS1115 AIN1 (I2C analog)
 *
 * Change this value to switch pH source at compile time.
 * TDS (ADS1115 AIN0) is unaffected by this setting.
 * -------------------------------------------------------------------------- */
#define PH_SOURCE_USE_SERIAL  0

#if (PH_SOURCE_USE_SERIAL == 1)
#  define PIN_PH_SERIAL_UART       UART_NUM_1    // Changed from UART_NUM_0
#  define PIN_PH_SERIAL_TX         17            // UART1 default TX
#  define PIN_PH_SERIAL_RX         18            // UART1 default RX
#  define PIN_PH_SERIAL_BAUD       115200
#  define PIN_PH_SERIAL_DEBUG_LED  10
#endif

#endif /* PIN_CONFIG_H */