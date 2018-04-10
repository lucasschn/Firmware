px4_nuttx_configure(HWCLASS m7 CONFIG nsh ROMFS y ROMFSROOT tap_common TAP_ESC H520)

set(config_module_list
	#
	# Board support modules
	#
	drivers/device
	drivers/stm32
	drivers/stm32/adc
	drivers/stm32/tone_alarm
	drivers/led
	drivers/px4fmu
	drivers/boards
	drivers/rgbled_pwm
	drivers/tap_esc
	drivers/imu/mpu6000
	drivers/barometer/ms5611
	drivers/barometer/mpc2520
	drivers/magnetometer/hmc5883
	drivers/magnetometer/ist8310
	drivers/distance_sensor/hc_sr04
	drivers/gps
	drivers/airspeed
	drivers/realsense
	modules/sensors
	drivers/vmount
	drivers/camera_trigger
	drivers/gimbal_protocol_splitter
	drivers/mavlink_dup
	drivers/pwm_out_sim

	#
	# System commands
	#
	systemcmds/bl_update
	systemcmds/led_control
	systemcmds/mixer
	systemcmds/param
	systemcmds/perf
	systemcmds/pwm
	systemcmds/hardfault_log
	systemcmds/motor_test
	systemcmds/reboot
	systemcmds/top
	systemcmds/config
	systemcmds/nshterm
	systemcmds/mtd
	systemcmds/dumpfile
	systemcmds/ver
	systemcmds/sd_bench
	systemcmds/tap_esc_config
	systemcmds/topic_listener
	systemcmds/tune_control

	#
	# General system control
	#
	modules/commander
	modules/events
	modules/load_mon
	modules/navigator
	modules/mavlink
	modules/land_detector

	#
	# Estimation modules (EKF/ SO3 / other filters)
	#
	modules/ekf2

	#
	# Vehicle Control
	#
	modules/mc_att_control
	modules/mc_pos_control
	modules/vtol_att_control

	#
	# Logging
	#
	modules/logger
	modules/sdlog2

	#
	# Library modules
	#
	modules/systemlib/param
	modules/systemlib
	modules/uORB
	modules/dataman

	#
	# Libraries
	#
	lib/controllib
	lib/mathlib
	lib/ecl
	lib/conversion
	lib/led
	lib/mixer
	lib/terrain_estimation
	lib/version
	lib/DriverFramework/framework
	lib/rc
	lib/tunes
	lib/FlightTasks
)
