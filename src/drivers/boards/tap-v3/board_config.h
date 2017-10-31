#pragma once

#undef BOARD_HAS_LTC4417
/**
 * Definition for the prototype of H520S V01 hardware
 * Schematics directory: H520S---000-R1
 * Pin chart table column: BR
 * Silkscreen desciptor: TYPHOON H520S V01 20170801
 * This needs to be removed once a hardware with the correct pinning is configured
 */
#define PX4_FMUV5_RC00

#define GPIO_EEPROM_WP  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN2)
#define BOARD_EEPROM_WP_CTRL(_on_true)    px4_arch_gpiowrite(GPIO_EEPROM_WP, (_on_true))
#define BOARD_HAS_MTD_PARTITION_OVERRIDE {"/fs/mtd_caldata"}

#define BOARD_HAS_POWER_CONTROL	1

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
#define	BOARD_NAME "TAP_V3"

// define the serial port of the RC to be UART 6 since UART 5 is enabled
#undef RC_SERIAL_PORT
#define RC_SERIAL_PORT "/dev/ttyS5"

#define TAP_ESC_NO_VERIFY_CONFIG
#define BOARD_TAP_ESC_MODE 2 // select closed-loop control mode for the esc

#define PX4_I2C_BUS_ONBOARD 4 // wrong workaround to get the baro working

#define BOARD_MAX_LEDS 6 // Define the number of led this board has

// Set correct string for the Hardware detection
#undef HW_INFO_INIT
#define HW_INFO_INIT {'V','3','x', 'x',0}
//      HW_INFO_INIT_REV       2
//      HW_INFO_INIT_VER            3
