/****************************************************************************
 *
 *   Copyright (c) 2019 YUNEEC. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file yuneec_factory_calib.cpp
 *
 */

#include "yuneec_factory_calib.h"

#include <px4_defines.h>
#include <px4_log.h>

#include <matrix/math.hpp>


namespace events
{
namespace yuneec_factory_calib
{

ShakeCalibration::ShakeCalibration(const events::SubscriberHandler &subscriber_handler)
	: _subscriber_handler(subscriber_handler)
{
}

bool ShakeCalibration::check_for_updates()
{
	bool ret = false;

	if (_subscriber_handler.vehicle_attitude_updated()) {
		orb_copy(ORB_ID(vehicle_attitude), _subscriber_handler.get_vehicle_attitude_sub(), &_v_att);
		ret = true;
	}

	if (_subscriber_handler.vehicle_command_ack_updated()) {
		orb_copy(ORB_ID(vehicle_command_ack), _subscriber_handler.get_vehicle_command_ack_sub(), &_command_ack);
		ret = true;
	}

	if (_subscriber_handler.vehicle_status_flags_updated()) {
		orb_copy(ORB_ID(vehicle_status_flags), _subscriber_handler.get_vehicle_status_flags_sub(), &_status_flags);
		ret = true;
	}

	return ret;
}

void ShakeCalibration::send_vehicle_command(const vehicle_command_s &cmd)
{
	if (_vehicle_command_pub != nullptr) {
		orb_publish(ORB_ID(vehicle_command), _vehicle_command_pub, &cmd);

	} else	{
		_vehicle_command_pub = orb_advertise_queue(ORB_ID(vehicle_command), &cmd, vehicle_command_s::ORB_QUEUE_LENGTH);
	}
}

void ShakeCalibration::process()
{
	if (!check_for_updates()) {
		return;
	}

	switch (_stage) {
	case Stage::DETECTING_SHAKES: {
			// TODO: Use Hysteresis to detect shakes

			/* get pitch angle of the vehicle, trigger calibration by nodding two times*/
			matrix::Quatf q;

			q(0) = _v_att.q[0];
			q(1) = _v_att.q[1];
			q(2) = _v_att.q[2];
			q(3) = _v_att.q[3];

			matrix::Eulerf euler(q);

			float pitch_angle = euler(1) * 57.3f;  // TODO: Use px4 method for conversion

			/* back to level and status flag to true */
			if (fabsf(pitch_angle) < LEVEL_ANG_THRESHOLD_DEG) {
				_drone_is_level = true;
				_pitch_zero_counter++;

				/* enter factory calibration */
				if (_pitch_negative_counter >= NUMBER_SHAKES_REQUIRED) {
					_pitch_negative_counter = 0;

					_stage = Stage::CONTINUE;
					break;
				}

				/* trigger factory calibration within 2 senconds (60 / 30Hz), otherwise restart the count */
				if (_pitch_zero_counter > CYCLES_FOR_SHAKE_DETECTION_RESET) {
					_pitch_zero_counter = 0;
					_pitch_negative_counter = 0;
				}
			}

			/* at a negative angle and previous status flag is at level */
			if (pitch_angle < ENTER_MIN_NEGATIVE_ATTITUDE && pitch_angle > ENTER_MAX_NEGATIVE_ATTITUDE && _drone_is_level) {
				_drone_is_level = false;
				_pitch_zero_counter = 0;
				_pitch_negative_counter++;
			}

			break;
		}

	case Stage::CONTINUE:

		// Keep spamming calibration requests until commander accepts
		if (_command_ack.command == vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION &&
		    _command_ack.result == vehicle_command_ack_s::VEHICLE_RESULT_ACCEPTED) {

			_command_ack = vehicle_command_ack_s{};  // Clear any previously received ACKs
			_calibration_detected = false;
			_stage = Stage::CALIBRATING;
			break;

		} else {

			// Start with mag calibration
			if (!_mag_calibration_done) {
				PX4_INFO("Requesting mag calibration");
				_cmd = vehicle_command_s{};
				_cmd.command = vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION;
				_cmd.param1 = 0; // Disable Gyro calibration
				_cmd.param2 = 1; // Enable Mag calibration
				send_vehicle_command(_cmd);

			} else if (_mag_calibration_done && !_gyro_calibration_done) {
				// Commence gyro calibration after a pause
				if (_thread_cycle_counter++ >= CYCLES_BETWEEN_CALIBRATIONS) {
					PX4_INFO("Requesting Gyro calibration");
					_cmd = vehicle_command_s{};
					_cmd.command = vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION;
					_cmd.param1 = 1; // Enable Gyro calibration
					_cmd.param2 = 0; // Disable Mag calibration
					send_vehicle_command(_cmd);
				}

			} else {
				// Shutdown system after calibration
				_thread_cycle_counter = 0;
				_stage = Stage::POWEROFF;
			}
		}

		break;

	case Stage::CALIBRATING:

		// Require at least one vehicle_status message with calibration flag enabled
		if (_status_flags.condition_calibration_enabled && !_calibration_detected) {
			_calibration_detected = true;

		} else if (!_status_flags.condition_calibration_enabled && _calibration_detected) {
			// Mark the corresponding calibration as completed
			_calibration_detected = false;

			if ((int)_cmd.param1) {
				_gyro_calibration_done = true;

			} else if ((int)_cmd.param2) {
				_mag_calibration_done = true;
			}

			_stage = Stage::CONTINUE;
		}

		break;

	case Stage::POWEROFF:

		// Power off after a pause since calibration completed
		if (_thread_cycle_counter++ >= CYCLES_BEFORE_POWER_OFF) {
			_cmd = vehicle_command_s{};
			_cmd.command = vehicle_command_s::VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
			_cmd.param1 = 2;

			send_vehicle_command(_cmd);
		}

		break;
	}
}

} /* namespace yuneec_factory_calib */
} /* namespace events */
