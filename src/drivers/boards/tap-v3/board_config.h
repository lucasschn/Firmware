#pragma once

#include <drivers/boards/px4fmu-v5/board_config.h>

#undef BOARD_HAS_LTC4417
#define PX4_FMUV5_RC01

#undef BOARD_NAME
#define	BOARD_NAME "TAP_V3"

#define TAP_ESC_NO_VERIFY_CONFIG

#define PX4_I2C_BUS_ONBOARD 4 // wrong workaround to get the baro working
