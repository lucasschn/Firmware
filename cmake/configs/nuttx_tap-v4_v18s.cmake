include(configs/nuttx_tap-v4_default)

px4_nuttx_configure(HWCLASS m7 CONFIG nsh ROMFS y ROMFSROOT tap_common TAP_ESC V18S)
# created to enable specific module to be included or excluded from the build

list(REMOVE_ITEM config_module_list
	driver/realsense
	)
