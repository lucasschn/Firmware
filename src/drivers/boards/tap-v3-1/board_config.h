#pragma once

#undef BOARD_HAS_LTC4417

#define PX4_FMUV5_RC01

#define GPIO_EEPROM_WP  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN2)
#define BOARD_EEPROM_WP_CTRL(_on_true)    px4_arch_gpiowrite(GPIO_EEPROM_WP, (_on_true))
#define BOARD_HAS_MTD_PARTITION_OVERRIDE {"/fs/mtd_caldata"}

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that
 * can be used by the Px4 Firmware in the adc driver
 */
#define ADC_CHANNELS (1 << 0)

/* Define Battery 1 Voltage Divider and A per V
 */
#define BOARD_BATTERY1_V_DIV (9.0f)

// ADC defines to be used in sensors.cpp to read from a particular channel
#define ADC_BATTERY_VOLTAGE_CHANNEL		0
#define ADC_BATTERY_CURRENT_CHANNEL ((uint8_t)(-1))

#include <drivers/boards/px4fmu-v5/board_config.h>

#undef BOARD_NAME
#define	BOARD_NAME "TAP_V3_1"

// undefine the serial port of the RC not used on OB rc input over mavlink
#undef RC_SERIAL_PORT

#define TAP_ESC_NO_VERIFY_CONFIG
#define BOARD_TAP_ESC_MODE 2 // select closed-loop control mode for the esc

#undef PX4_I2C_BUS_EXPANSION3 // undefine expansion3 bus since it is used by the baro
#define PX4_I2C_BUS_ONBOARD 4 // wrong workaround to get the baro working

#define BOARD_MAX_LEDS 4 // Define the number of led this board has

// UI LED are active high
#undef BOARD_UI_LED_PWM_DRIVE_ACTIVE_LOW

// Set correct string for hardware detection
#undef HW_INFO_INIT
#define HW_INFO_INIT {'V','3','x','x',0}
//      HW_INFO_INIT_REV       2
//      HW_INFO_INIT_VER           3
