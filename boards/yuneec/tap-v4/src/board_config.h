#pragma once

#undef BOARD_HAS_LTC4417

#define PX4_FMUV5_RC01

#define GPIO_EEPROM_WP  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN2)
#define BOARD_EEPROM_WP_CTRL(_on_true)    px4_arch_gpiowrite(GPIO_EEPROM_WP, (_on_true))
#define BOARD_HAS_MTD_PARTITION_OVERRIDE {"/fs/mtd_caldata"}

#define BOARD_HAS_POWER_CONTROL	1

/*power on/off*/
#define MS_PWR_BUTTON_DOWN 1500
#define KEY_AD_GPIO    (GPIO_INPUT|GPIO_PULLUP|GPIO_EXTI|GPIO_PORTC|GPIO_PIN4)
#define POWER_ON_GPIO  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN5)
#define POWER_OFF_GPIO (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTC|GPIO_PIN5)

#include <../../../px4/fmu-v5/src/board_config.h>

/* Define Battery 1 Voltage Divider
 */
#undef BOARD_BATTERY1_V_DIV
#define BOARD_BATTERY1_V_DIV (9.0f)

/*
 * ESC configuration
 *
 * Physical / Logical ESC mapping
 * The index corresponds to the physical ESC, the value to the logical ESC
 * Phy Log
 * 0   0
 * 1   1
 * 2   2
 * 3   3
 * 4   4
 * 5   5
 *  ....
 *
 */
// Circular from back right in CCW direction
#define BOARD_MAP_ESC_PHYS_TO_LOG {0, 1, 2, 3, 4, 5, 6, 7}
// 0 is CW, 1 is CCW
#define BOARD_MAP_ESC_TO_PX4_DIR  {0, 1, 0, 1, 0, 1, 0, 1}
// output remap table
#define BOARD_MAP_ESC_TO_PX4_OUT  {2, 1, 3, 0, 4, 5, 6, 7}

#undef BOARD_NAME
#define	BOARD_NAME "TAP_V4"

// undefine the serial port of the RC not used on OB rc input over mavlink
#undef RC_SERIAL_PORT
#define RC_SERIAL_PORT "/dev/ttyS5"

#define BOARD_TAP_ESC_NO_VERIFY_CONFIG
#define BOARD_TAP_ESC_MODE 2 // select closed-loop control mode for the esc

#undef PX4_I2C_BUS_EXPANSION3 // undefine expansion3 bus since it is used by the baro

#undef PX4_I2C_BUS_ONBOARD
#define PX4_I2C_BUS_ONBOARD 4 // wrong workaround to get the baro working

#define BOARD_MAX_LEDS 4 // Define the number of led this board has

// UI LED are active high
#undef BOARD_UI_LED_PWM_DRIVE_ACTIVE_LOW

// Set correct string for hardware detection
#undef HW_INFO_INIT
#define HW_INFO_INIT {'V','4','x','x',0}
//      HW_INFO_INIT_REV       2
//      HW_INFO_INIT_VER           3

// No safety switch on our boards
#undef GPIO_BTN_SAFETY

// LED mapping
#define BOARD_FRONT_LED_MASK (1 << 0) | (1 << 3)
#define BOARD_REAR_LED_MASK  (1 << 1) | (1 << 2)
#define BOARD_LEFT_LED_MASK  (0)
#define BOARD_RIGHT_LED_MASK (0)

// In HITL, we can use the usual voltage measurement.
#define BOARD_HAS_VOLTAGE_IN_HITL

/************************************************************************************
 * Name: board_pwr_init()
 *
 * Description:
 *   Called to configure power control for the tap-v2 board.
 *
 * Input Parameters:
 *   stage- 0 for boot, 1 for board init
 *
 ************************************************************************************/

void board_pwr_init(int stage);

/****************************************************************************
 * Name: board_pwr_button_down
 *
 * Description:
 *   Called to Read the logical state of the power button
 ****************************************************************************/

bool board_pwr_button_down(void);
