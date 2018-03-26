/**
 * @file tap_esc_params.c
 * Parameters for esc version
 *
 */

/**
 * Required esc firmware version.
 *
 * @group ESC
 */
PARAM_DEFINE_INT32(ESC_FIRM_VER, 0);

/**
 * Required esc bootloader version.
 *
 * @group ESC
 */
PARAM_DEFINE_INT32(ESC_BOOT_VER, 0);

/**
 * Required esc hardware version
 *
 * @group ESC
 */
PARAM_DEFINE_INT32(ESC_HARD_VER, 0);
