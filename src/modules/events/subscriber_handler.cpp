
#include "subscriber_handler.h"

#include <px4_log.h>

namespace events
{

void SubscriberHandler::subscribe()
{
	if (_battery_status_sub < 0) {
		_battery_status_sub = orb_subscribe(ORB_ID(battery_status));
	}

	if (_cpuload_sub < 0) {
		_cpuload_sub = orb_subscribe(ORB_ID(cpuload));
	}

	if (_vehicle_command_sub < 0) {
		_vehicle_command_sub = orb_subscribe(ORB_ID(vehicle_command));
	}

	if (_vehicle_command_ack_sub < 0) {
		_vehicle_command_ack_sub = orb_subscribe(ORB_ID(vehicle_command_ack));
	}

	if (_vehicle_status_sub < 0) {
		_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	}

	if (_vehicle_status_flags_sub < 0) {
		_vehicle_status_flags_sub = orb_subscribe(ORB_ID(vehicle_status_flags));
	}

	if (_vehicle_attitude_sub < 0) {
		_vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	}

	if (_smart_heading_sub < 0) {
		_smart_heading_sub = orb_subscribe(ORB_ID(smart_heading));
	}

	if (_manual_control_setpoint_sub < 0) {
		_manual_control_setpoint_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	}

	if (_vehicle_land_detected_sub < 0) {
		_vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
	}
}

void SubscriberHandler::unsubscribe()
{
	if (_battery_status_sub >= 0) {
		orb_unsubscribe(_battery_status_sub);
		_battery_status_sub = -1;
	}

	if (_cpuload_sub >= 0) {
		orb_unsubscribe(_cpuload_sub);
		_cpuload_sub = -1;
	}

	if (_vehicle_command_sub >= 0) {
		orb_unsubscribe(_vehicle_command_sub);
		_vehicle_command_sub = -1;
	}

	if (_vehicle_command_ack_sub >= 0) {
		orb_unsubscribe(_vehicle_command_ack_sub);
		_vehicle_command_ack_sub = -1;
	}

	if (_vehicle_status_sub >= 0) {
		orb_unsubscribe(_vehicle_status_sub);
		_vehicle_status_sub = -1;
	}

	if (_vehicle_status_flags_sub >= 0) {
		orb_unsubscribe(_vehicle_status_flags_sub);
		_vehicle_status_flags_sub = -1;
	}

	if (_vehicle_attitude_sub >= 0) {
		orb_unsubscribe(_vehicle_attitude_sub);
		_vehicle_attitude_sub = -1;
	}

	if (_smart_heading_sub >= 0) {
		orb_unsubscribe(_smart_heading_sub);
		_smart_heading_sub = -1;
	}

	if (_manual_control_setpoint_sub >= 0) {
		orb_unsubscribe(_manual_control_setpoint_sub);
		_manual_control_setpoint_sub = -1;
	}

	if (_vehicle_land_detected_sub >= 0) {
		orb_unsubscribe(_vehicle_land_detected_sub);
		_vehicle_land_detected_sub = -1;
	}
}

void SubscriberHandler::check_for_updates()
{
	bool updated = false;
	_update_bitfield = 0;

	orb_check(_vehicle_command_sub, &updated);

	if (updated) {
		_update_bitfield |= (uint32_t)StatusMask::VehicleCommand;
	}

	orb_check(_vehicle_command_ack_sub, &updated);

	if (updated) {
		_update_bitfield |= (uint32_t)StatusMask::VehicleCommandAck;
	}

	orb_check(_vehicle_status_sub, &updated);

	if (updated) {
		_update_bitfield |= (uint32_t)StatusMask::VehicleStatus;
	}

	orb_check(_vehicle_status_flags_sub, &updated);

	if (updated) {
		_update_bitfield |= (uint32_t)StatusMask::VehicleStatusFlags;
	}

	orb_check(_battery_status_sub, &updated);

	if (updated) {
		_update_bitfield |= (uint32_t)StatusMask::BatteryStatus;
	}

	orb_check(_cpuload_sub, &updated);

	if (updated) {
		_update_bitfield |= (uint32_t)StatusMask::CpuLoad;
	}

	orb_check(_vehicle_attitude_sub, &updated);

	if (updated) {
		_update_bitfield |= (uint32_t)StatusMask::VehicleAttitude;
	}

	orb_check(_smart_heading_sub, &updated);

	if (updated) {
		_update_bitfield |= (uint32_t)StatusMask::SmartHeading;
	}

	orb_check(_manual_control_setpoint_sub, &updated);

	if (updated) {
		_update_bitfield |= (uint32_t)StatusMask::ManualControlSP;
	}

	orb_check(_vehicle_land_detected_sub, &updated);

	if (updated) {
		_update_bitfield |= (uint32_t)StatusMask::VehicleLandDetected;
	}
}

} /* namespace events */
