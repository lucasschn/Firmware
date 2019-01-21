add_definitions("-DENABLE_RC_HELPER")

px4_add_board(
	PLATFORM nuttx
	VENDOR yuneec
	MODEL tap-v3
	LABEL ofdm
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m4
	ROMFSROOT tap_common
	# IO px4_io-v2_default  # TODO: Do we need this for copying the ESC firmware bin?
	# TAP_ESC H520  # TODO:  Copy ESC firmware binary
	TESTING
	# UAVCAN_INTERFACES 2  # TODO: Doesn't have any UAVCAN interfaces, right?

	SERIAL_PORTS
		# GPS1:/dev/ttyS0
		# TEL1:/dev/ttyS1
		# TEL2:/dev/ttyS2
		# TEL4:/dev/ttyS3

	DRIVERS
		# barometer # all available barometer drivers
		barometer/ms5611
		barometer/mpc2520
		# batt_smbus
		camera_trigger
		# differential_pressure # all available differential pressure drivers
		# distance_sensor # all available distance sensor drivers
		distance_sensor/hc_sr04
		gps
		#heater
		#imu # all available imu drivers
		# imu/adis16448
		# imu/l3gd20
		# imu/lsm303d
		imu/mpu6000
		# imu/mpu9250
		# irlock
		#lights/blinkm
		#lights/oreoled
		#lights/rgbled
		lights/rgbled_pwm
		# magnetometer # all available magnetometer drivers
		magnetometer/hmc5883
		magnetometer/ist8310
		#md25
		mavlink_dup
		# mkblctrl
		# pca8574
		# pca9685
		#pmw3901
		# protocol_splitter
		gimbal_protocol_splitter
		# pwm_input
		# pwm_out_sim
		# px4flow
		px4fmu
		# px4io
		rc_input
		realsense
		# rgbled_ncp5623c
		# roboclaw
		stm32
		stm32/adc
		# stm32/tone_alarm
		tap_esc
		# telemetry # all available telemetry drivers
		# test_ppm
		#uavcan

	MODULES
		# attitude_estimator_q
		# camera_feedback
		commander
		dataman
		ekf2
		events
		# fw_att_control
		# fw_pos_control_l1
		# gnd_att_control
		# gnd_pos_control
		land_detector
		# landing_target_estimator
		load_mon
		# local_position_estimator
		logger
		mavlink
		mc_att_control
		mc_pos_control
		navigator
		# position_estimator_inav
		sensors
		vmount
		vtol_att_control
		# wind_estimator

	SYSTEMCMDS
		bl_update
		config
		dumpfile
		# esc_calib
		hardfault_log
		led_control
		mixer
		# motor_ramp
		motor_test
		mtd
		nshterm
		param
		perf
		pwm
		reboot
		# reflect
		sd_bench
		# shutdown
		tap_esc_config
		# tests # tests and test runner
		top
		topic_listener
		tune_control
		# usb_connected
		ver

	EXAMPLES
		# bottle_drop # OBC challenge
		# fixedwing_control # Tutorial code from https://px4.io/dev/example_fixedwing_control
		# hwtest # Hardware test
		#matlab_csv_serial
		#publisher
		# px4_mavlink_debug # Tutorial code from https://px4.io/dev/debug_values
		# px4_simple_app # Tutorial code from https://px4.io/dev/px4_simple_app
		# rover_steering_control # Rover example app
		# segway
		#subscriber
		# uuv_example_app

	)
