#pragma once

#include <uORB/uORB.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/cpuload.h>
#include <uORB/topics/smart_heading.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_status_flags.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/manual_control_setpoint.h>

namespace events
{

/**
 * @class SubscriberHandler
 * Contains a list of uORB subscriptions and maintains their update state.
 */
class SubscriberHandler
{
public:
	void subscribe();
	void unsubscribe();
	void check_for_updates();

	int get_battery_status_sub() const { return _battery_status_sub; }
	int get_cpuload_sub() const { return _cpuload_sub; }
	int get_vehicle_command_sub() const { return _vehicle_command_sub; }
	int get_vehicle_command_ack_sub() const { return _vehicle_command_ack_sub; }
	int get_vehicle_status_sub() const { return _vehicle_status_sub; }
	int get_vehicle_status_flags_sub() const { return _vehicle_status_flags_sub; }
	int get_vehicle_attitude_sub() const { return _vehicle_attitude_sub; }
	int get_smart_heading_sub() const { return _smart_heading_sub; }
	int get_manual_control_setpoint_sub() const { return _manual_control_setpoint_sub; }
	int get_vehicle_land_detected_sub() const { return _vehicle_land_detected_sub; }
	// TODO: incorporate an add_topic method, this will push back the sub handler
	// in the subscriber vector

	/* update checking methods */
	bool battery_status_updated() const { return _update_bitfield & (uint32_t)StatusMask::BatteryStatus; }
	bool cpuload_updated() const { return _update_bitfield & (uint32_t)StatusMask::CpuLoad; }
	bool vehicle_command_updated() const { return _update_bitfield & (uint32_t)StatusMask::VehicleCommand; }
	bool vehicle_command_ack_updated() const { return _update_bitfield & (uint32_t)StatusMask::VehicleCommandAck; }
	bool vehicle_status_updated() const { return _update_bitfield & (uint32_t)StatusMask::VehicleStatus; }
	bool vehicle_status_flags_updated() const { return _update_bitfield & (uint32_t)StatusMask::VehicleStatusFlags; }
	bool vehicle_attitude_updated() const { return _update_bitfield & (uint32_t)StatusMask::VehicleAttitude; }
	bool smart_heading_updated() const { return _update_bitfield & (uint32_t)StatusMask::SmartHeading; }
	bool manual_control_sp_updated() const { return _update_bitfield & (uint32_t)StatusMask::ManualControlSP; }
	bool vehicle_land_detcted_updated() const { return _update_bitfield & (uint32_t)StatusMask::VehicleLandDetected; }


private:
	enum class StatusMask : uint32_t {
		VehicleCommand = (0x01 << 0),
		VehicleStatus = (0x01 << 1),
		VehicleStatusFlags = (0x01 << 2),
		BatteryStatus = (0x01 << 3),
		CpuLoad = (0x01 << 4),
		VehicleAttitude = (0x01 << 5),
		SmartHeading = (0x01 << 6),
		ManualControlSP = (0x01 << 7),
		VehicleLandDetected = (0x01 << 8),
		VehicleCommandAck = (0x01 << 9)
	};

	// TODO: incorporate the subscriber into a vector of int
	int _battery_status_sub = -1;
	int _cpuload_sub = -1;
	int _smart_heading_sub = -1;
	int _vehicle_command_sub = -1;
	int _vehicle_command_ack_sub = -1;
	int _vehicle_status_sub = -1;
	int _vehicle_status_flags_sub = -1;
	int _vehicle_attitude_sub = -1;
	int _manual_control_setpoint_sub = -1;
	int _vehicle_land_detected_sub = -1;

	uint32_t _update_bitfield = 0;
};

} /* events */
