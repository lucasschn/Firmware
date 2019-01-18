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

/*radio*/
#define GPIO_PCON_RADIO  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN8)
#define RF_RADIO_POWER_CONTROL(_on_true)    px4_arch_gpiowrite(GPIO_PCON_RADIO, !(_on_true))

/*power on/off*/
#define MS_PWR_BUTTON_DOWN 1500
#define KEY_AD_GPIO    (GPIO_INPUT|GPIO_PULLUP|GPIO_EXTI|GPIO_PORTC|GPIO_PIN4)
#define POWER_ON_GPIO  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN5)
#define POWER_OFF_GPIO (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTC|GPIO_PIN5)

#include <../../../px4/fmu-v5/src/board_config.h>

/* Define Battery 1 Voltage Divider and A per V
 */
#undef BOARD_BATTERY1_V_DIV
#define BOARD_BATTERY1_V_DIV (9.0f)


#undef BOARD_NAME
#define	BOARD_NAME "TAP_V3"

/*
 * TIM2_CH4    PB11    Sonar Echo
 * TIM2_CH2    PB3     Sonar Trig
 * */
#define GPIO_TIM2_CH2_OUT    /* PB3   T22C2  FMU_CAP2 */ GPIO_TIM2_CH2OUT_2
#define GPIO_TIM2_CH4_OUT    /* PB11  T22C4  FMU_CAP3 */ GPIO_TIM2_CH4OUT_2

#if !defined(ENABLE_RC_HELPER)
// define the serial port of the RC to be UART 6 since UART 5 is enabled
#undef RC_SERIAL_PORT
#define RC_SERIAL_PORT "/dev/ttyS5"
#endif

#define BOARD_TAP_ESC_MODE 2 // select closed-loop control mode for the esc
#define BOARD_USE_ESC_CURRENT_REPORT // each ESC reports its current estimate
#define BOARD_SUPPORTS_FTC // Board supports fault tolerant control. Set param FTC_ENABLE to 1 to enable it.

#undef PX4_I2C_BUS_ONBOARD
#define PX4_I2C_BUS_ONBOARD 4 // wrong workaround to get the baro working

#define BOARD_MAX_LEDS 6 // Define the number of led this board has

// Set correct string for the Hardware detection
#undef HW_INFO_INIT
#define HW_INFO_INIT {'V','3','x', 'x',0}
//      HW_INFO_INIT_REV       2
//      HW_INFO_INIT_VER            3

// No safety switch on our boards
#undef GPIO_BTN_SAFETY

// LED mapping
#define BOARD_FRONT_LED_MASK (1 << 2) | (1 << 3)
#define BOARD_REAR_LED_MASK  (1 << 0) | (1 << 5)
#define BOARD_LEFT_LED_MASK  (1 << 4)
#define BOARD_RIGHT_LED_MASK (1 << 1)

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

#include "drivers/boards/common/board_common.h"
