include(configs/nuttx_tap-v4_default)

# created to enable specific module to be included or excluded from the build

list(REMOVE_ITEM config_module_list
	driver/realsense
	)
