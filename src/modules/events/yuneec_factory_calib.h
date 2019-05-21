/****************************************************************************
 *
 *   Copyright (c) 2019 YUNEEC. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file yuneec_factory_calib.h
 *
 */

#pragma once

#include "subscriber_handler.h"

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status_flags.h>

#define LEVEL_ANG_THRESHOLD_DEG			5.0f
#define ENTER_MIN_NEGATIVE_ATTITUDE		-25.0f
#define ENTER_MAX_NEGATIVE_ATTITUDE		-75.0f
#define NUMBER_SHAKES_REQUIRED			2
#define CYCLES_FOR_SHAKE_DETECTION_RESET 	60  // 2 senconds (60 cycles @ 30Hz)
#define CYCLES_BETWEEN_CALIBRATIONS 		90 // 3 seconds (90 cycles @ 30 Hz)
#define CYCLES_BEFORE_POWER_OFF 		90 // 3 seconds (90 cycles @ 30 Hz)


namespace events
{
namespace yuneec_factory_calib
{

enum Stage {
	DETECTING_SHAKES,
	CONTINUE,
	CALIBRATING,
	POWEROFF,
	DO_NOTHING
};


class ShakeCalibration
{
public:

	ShakeCalibration(const events::SubscriberHandler &subscriber_handler);

	/** regularily called to handle state updates */
	void process();

private:
	/**
	 * check for topic updates
	 * @return true if one or more topics got updated
	 */
	bool check_for_updates();

	/** Send a vehicle_command */
	inline void send_vehicle_command(const vehicle_command_s &cmd);

	const events::SubscriberHandler &_subscriber_handler;

	bool _mag_calibration_done = false;
	bool _gyro_calibration_done = false;
	bool _calibration_detected = false;
	bool _drone_is_level = false;
	unsigned _thread_cycle_counter = 0;
	unsigned _pitch_zero_counter = 0;
	unsigned _pitch_negative_counter = 0;
	struct vehicle_attitude_s _v_att = {};
	struct vehicle_command_s _cmd = {};
	struct vehicle_command_ack_s _command_ack = {};
	struct vehicle_status_flags_s _status_flags = {};
	orb_advert_t _vehicle_command_pub = nullptr;
	Stage _stage = Stage::DETECTING_SHAKES;
};

} /* namespace yuneec_factory_calib */
} /* namespace events */
