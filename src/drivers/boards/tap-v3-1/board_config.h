#pragma once

#undef BOARD_HAS_LTC4417
/**
 * From the pin chart it should be PX4_FMUV5_RC01 for OB
 */
#define PX4_FMUV5_RC00

#define GPIO_EEPROM_WP  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN2)
#define BOARD_EEPROM_WP_CTRL(_on_true)    px4_arch_gpiowrite(GPIO_EEPROM_WP, (_on_true))
#define BOARD_HAS_MTD_PARTITION_OVERRIDE {"/fs/mtd_caldata"}

#include <drivers/boards/px4fmu-v5/board_config.h>

#undef BOARD_NAME
#define	BOARD_NAME "TAP_V3_1"

// undefine the serial port of the RC not used on OB rc input over mavlink
#undef RC_SERIAL_PORT

#define TAP_ESC_NO_VERIFY_CONFIG
#define BOARD_TAP_ESC_MODE 2 // select closed-loop control mode for the esc

#define PX4_I2C_BUS_ONBOARD 4 // wrong workaround to get the baro working

#define BOARD_MAX_LEDS 4 // Define the number of led this board has

// Set correct string for hardware detection
#undef HW_INFO_INIT
#define HW_INFO_INIT {'V','3','x', 'x',0}
//      HW_INFO_INIT_REV       2
//      HW_INFO_INIT_VER            3
