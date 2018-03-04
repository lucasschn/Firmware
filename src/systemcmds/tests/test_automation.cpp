
#include <vector>

#include <unit_test.h>

#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/battery_status.h>

#include <commander/state_machine_helper.h>

#include <platforms/px4_tasks.h>

#undef PX4_DEBUG
#define PX4_DEBUG PX4_INFO

extern transition_result_t arm_disarm(bool arm, orb_advert_t *mavlink_log_pub_local, const char *armedBy);

class uORB_topic
{
public:
	uORB_topic(const struct orb_metadata *topic_meta, void *topic_status):
		meta(topic_meta), subscribe(-1), status(topic_status), is_updated(false) {}
	virtual ~uORB_topic() {}

public:
	const char *name;
	const struct orb_metadata *meta;
	int subscribe;
	void *status;
	bool is_updated;

	virtual void dump() = 0;
};

static std::vector<uORB_topic *> uORB_topic_list;

#define DECLARE_UORB_SUB(TOPIC) \
	static TOPIC##_s TOPIC##_status; \
	class TOPIC##_uORB_topic : public uORB_topic { \
	public: \
		TOPIC##_uORB_topic(const struct orb_metadata *topic_meta, void *topic_status): \
			uORB_topic(topic_meta, topic_status) { \
			name = #TOPIC; \
		} \
		virtual void dump();\
	}; \
	static TOPIC##_uORB_topic TOPIC##_uORB_topic_obj(ORB_ID(TOPIC), &TOPIC##_status);

#define APPEND_UORB_SUB(TOPIC) \
	uORB_topic_list.push_back(&TOPIC##_uORB_topic_obj);

//declare subscribed topics
DECLARE_UORB_SUB(vehicle_local_position)
void vehicle_local_position_uORB_topic::dump()
{
	PX4_DEBUG("x = %f, y = %f, z = %f",
		  vehicle_local_position_status.x,
		  vehicle_local_position_status.y,
		  vehicle_local_position_status.z);
	PX4_DEBUG("vx = %f, vy = %f, vz = %f",
		  vehicle_local_position_status.vx,
		  vehicle_local_position_status.vy,
		  vehicle_local_position_status.vz);
}

DECLARE_UORB_SUB(vehicle_global_position)
void vehicle_global_position_uORB_topic::dump()
{
	PX4_DEBUG("lat = %f, lon = %f, alt =%f, eph = %f, epv = %f, timestamp = %llu",
		  vehicle_global_position_status.lat,
		  vehicle_global_position_status.lon,
		  vehicle_global_position_status.alt,
		  vehicle_global_position_status.eph,
		  vehicle_global_position_status.epv,
		  vehicle_global_position_status.timestamp);
}

DECLARE_UORB_SUB(vehicle_land_detected)
void vehicle_land_detected_uORB_topic::dump()
{
	PX4_DEBUG("land detected = %d", vehicle_land_detected_status.landed);
}

//task control
static bool task_should_exit = false;
static int daemon_task = -1;

//advertised topics
static orb_advert_t vehicle_command_pub = nullptr;
static orb_advert_t mavlink_log_pub = nullptr;
static orb_advert_t manual_control_setpoint_pub = nullptr;
static orb_advert_t battery_status_pub = nullptr;

static const int mavlink_system_id = 1;
static const int mavlink_component_id = 1;

typedef enum {
	AUTOMATION_AUTO_LAND = 0,
	AUTOMATION_AUTO_RTL  = 1
} automation_land_t ;

class AutomationTest : public UnitTest
{
public:
	AutomationTest();
	virtual ~AutomationTest();
	virtual bool run_tests();

protected:
	bool enableFailSafe(bool);

	//internal functions
	bool _arm();
	bool _disarm();
	bool _takeOff(float, int);
	bool _land(int);
	bool _manualControl(float, float, float, float, int);
	bool _checkLandedStatus(int, automation_land_t);
	bool _rtlForFailSafe(int);
	bool _isGPSAvaiable();
	void _saveLandingPosition();

	//tests
	bool takeOffTest();
	bool landTest();
	bool manualTest();
	bool failSafeTest();

protected:
	vehicle_local_position_s _takeoff_vehicle_local_position_status; //local position when take off
	vehicle_local_position_s _landing_vehicle_local_position_status; //local position when start landing
};

static void updateTopicTask(int argc, char *argv[])
{
	while (!task_should_exit) {

		for (auto it : uORB_topic_list) {
			bool updated;
			orb_check(it->subscribe, &updated);

			if (updated) {
				PX4_DEBUG("Topic updated: %s", it->name);
				orb_copy(it->meta, it->subscribe, it->status);
				it->is_updated = true;
				it->dump();
			}
		}

		sleep(1);
	}
}

AutomationTest::AutomationTest()
{
	APPEND_UORB_SUB(vehicle_local_position);
	APPEND_UORB_SUB(vehicle_global_position);
	APPEND_UORB_SUB(vehicle_land_detected);

	for (auto it : uORB_topic_list) {
		PX4_DEBUG("Subscribe %s", it->name);

		if (it->subscribe < 0) {
			it->subscribe = orb_subscribe(it->meta);
		}
	}

	if (daemon_task < 0) {
		task_should_exit = false;
		daemon_task = px4_task_spawn_cmd("automation",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT + 40,
						 2048,
						 (px4_main_t)&updateTopicTask,
						 nullptr);
	}

}

AutomationTest::~AutomationTest()
{
	task_should_exit = true;

	if (daemon_task > -1) {
		px4_task_delete(daemon_task);
		daemon_task = -1;
	}

	//unadvertise topics
	if (vehicle_command_pub != nullptr) {
		orb_unadvertise(vehicle_command_pub);
		vehicle_command_pub = nullptr;
	}

	if (mavlink_log_pub != nullptr) {
		orb_unadvertise(mavlink_log_pub);
		mavlink_log_pub = nullptr;
	}

	if (manual_control_setpoint_pub != nullptr) {
		orb_unadvertise(manual_control_setpoint_pub);
		manual_control_setpoint_pub = nullptr;
	}

	if (battery_status_pub != nullptr) {
		orb_unadvertise(battery_status_pub);
		battery_status_pub = nullptr;
	}

	for (auto it : uORB_topic_list) {
		PX4_DEBUG("Unsubscribe %s", it->name);

		if (it->subscribe >= 0) {
			orb_unsubscribe(it->subscribe);
			it->subscribe = -1;
		}
	}

	uORB_topic_list.clear();
}

bool AutomationTest::enableFailSafe(bool enabled)
{
	//Disable datalink lost and rc lost action to avoid RTL,
	//since the automation test is done without RC or datalink.
	int navigator_datalinklost_action = enabled ? 2 : 0;
	ut_assert_true(param_set(param_find("NAV_DLL_ACT"), &navigator_datalinklost_action) == OK);

	int navigator_rclost_action = enabled ? 2 : 0;
	ut_assert_true(param_set(param_find("NAV_RCL_ACT"), &navigator_rclost_action) == OK);

	float minimal_battery_percentage = 50.f;
	ut_assert_true(param_set(param_find("SIM_BAT_MIN_PCT"), &minimal_battery_percentage) == OK);

	return true;
}

bool AutomationTest::_arm()
{
	return (TRANSITION_CHANGED == arm_disarm(true, &mavlink_log_pub, "Automation"));
}

bool AutomationTest::_disarm()
{
	return (TRANSITION_CHANGED == arm_disarm(false, &mavlink_log_pub, "Automation"));
}

bool AutomationTest::_takeOff(float altitude, int waitTimeOutInSecond)
{
	const int max_wait_gps_seconds = 120;
	const float min_vertical_speed = 0.02;
	const float min_altitude_delta = 0.5;

	float mis_takeoff_alt = altitude;

	ut_assert_true(altitude > 0);
	ut_assert_true(param_set(param_find("MIS_TAKEOFF_ALT"), &mis_takeoff_alt) == OK);

	//wait for GPS
	for (int i = 0; i < max_wait_gps_seconds; i++) {
		if (_isGPSAvaiable()) {
			//Wait a while for status stable after GPS available
			sleep(5);
			break;
		}

		sleep(1);
	}

	//Save local position status when taking off
	memcpy(&_takeoff_vehicle_local_position_status, &vehicle_local_position_status, sizeof(vehicle_local_position_s));

	ut_assert_true(_isGPSAvaiable());
	ut_assert_true(_arm());

	struct vehicle_command_s cmd = {
		.timestamp = hrt_absolute_time(),
		.param5 = NAN,
		.param6 = NAN,
		.param1 = NAN,
		.param2 = NAN,
		.param3 = NAN,
		.param4 = NAN,
		.param7 = NAN,
		.command = vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF,
		.target_system = mavlink_system_id,
		.target_component = mavlink_component_id
	};

	if (vehicle_command_pub == nullptr) {
		vehicle_command_pub = orb_advertise_queue(ORB_ID(vehicle_command), &cmd, vehicle_command_s::ORB_QUEUE_LENGTH);

	} else {
		orb_publish(ORB_ID(vehicle_command), vehicle_command_pub, &cmd);
	}

	ut_assert_true(param_get(param_find("MIS_TAKEOFF_ALT"), &mis_takeoff_alt) == OK);
	PX4_DEBUG("mis_takeoff_alt = %f", mis_takeoff_alt);

	int runningSeconds;

	for (runningSeconds = 0 ; runningSeconds < waitTimeOutInSecond; runningSeconds++) {
		if (std::fabs(mis_takeoff_alt + vehicle_local_position_status.z) < min_altitude_delta) {
			break;
		}

		sleep(1);
	}

	//Assert not time out
	ut_assert_true(runningSeconds < waitTimeOutInSecond);

	//Assert vertical speed is zero. Sleep a while to let it stable.
	sleep(15);
	ut_assert_true((std::fabs(vehicle_local_position_status.vz) < min_vertical_speed));

	//Assert that current position is same as takeoff altitude.
	//The local_pos.z direction is downward, so the difference = mis_takeoff_alt - ( - local_pos.z)
	ut_assert_true((std::fabs(mis_takeoff_alt + vehicle_local_position_status.z) < min_altitude_delta));

	PX4_DEBUG("diff = %f", mis_takeoff_alt + vehicle_local_position_status.z);

	return true;
}

bool AutomationTest::_checkLandedStatus(int waitTimeOutInSecond, automation_land_t land_type)
{
	int runningSeconds;

	for (runningSeconds = 0 ; runningSeconds < waitTimeOutInSecond; runningSeconds++) {
		if (vehicle_land_detected_status.landed) {
			break;
		}

		sleep(1);
	}

	//Assert not time out
	ut_assert_true(runningSeconds < waitTimeOutInSecond);

	const float max_local_position_diff = (land_type == AUTOMATION_AUTO_RTL) ? 0.06 : 0.15;

	PX4_DEBUG("vehicle_local_position_status.z = %f", vehicle_local_position_status.z);

	//Assert landed is detected
	ut_assert_true(vehicle_land_detected_status.landed);

	//Assert the altitude is less than 0.5m after landing
	ut_assert_true(vehicle_local_position_status.z > -0.5);

	PX4_INFO("land_type = %d", land_type);
	const vehicle_local_position_s &expected_landing_local_position = (land_type == AUTOMATION_AUTO_RTL) ?
			_takeoff_vehicle_local_position_status : _landing_vehicle_local_position_status;

	PX4_DEBUG("Vehicle local position difference from expected landing location: x = %f, y = %f, z = %f",
		  std::fabs(expected_landing_local_position.x - vehicle_local_position_status.x),
		  std::fabs(expected_landing_local_position.y - vehicle_local_position_status.y),
		  std::fabs(expected_landing_local_position.z - vehicle_local_position_status.z));
	ut_assert_true((std::fabs(expected_landing_local_position.x - vehicle_local_position_status.x) <
			max_local_position_diff));
	ut_assert_true((std::fabs(expected_landing_local_position.y - vehicle_local_position_status.y) <
			max_local_position_diff));

	return true;
}

bool AutomationTest::_land(int waitTimeOutInSecond)
{
	_saveLandingPosition();

	struct vehicle_command_s cmd = {
		.timestamp = hrt_absolute_time(),
		.param5 = NAN,
		.param6 = NAN,
		.param1 = NAN,
		.param2 = NAN,
		.param3 = NAN,
		.param4 = NAN,
		.param7 = NAN,
		.command = vehicle_command_s::VEHICLE_CMD_NAV_LAND,
		.target_system = mavlink_system_id,
		.target_component = mavlink_component_id
	};

	if (vehicle_command_pub == nullptr) {
		vehicle_command_pub = orb_advertise_queue(ORB_ID(vehicle_command), &cmd, vehicle_command_s::ORB_QUEUE_LENGTH);

	} else {
		orb_publish(ORB_ID(vehicle_command), vehicle_command_pub, &cmd);
	}

	ut_assert_true(_checkLandedStatus(waitTimeOutInSecond, AUTOMATION_AUTO_LAND));

	//Sometimes it is disarmed automatically already,
	//so it cannot assert true here
	_disarm();

	return true;
}

bool AutomationTest::_rtlForFailSafe(int waitTimeOutInSecond)
{
	ut_assert_true(enableFailSafe(true));

	ut_assert_true(_checkLandedStatus(waitTimeOutInSecond, AUTOMATION_AUTO_RTL));

	_disarm();

	return true;
}

bool AutomationTest::_isGPSAvaiable()
{
	return vehicle_global_position_uORB_topic_obj.is_updated;
}

void AutomationTest::_saveLandingPosition()
{
	memcpy(&_landing_vehicle_local_position_status, &vehicle_local_position_status, sizeof(vehicle_local_position_s));
}

bool AutomationTest::_manualControl(float x, float y, float z, float r, int waitTimeOutInSecond)
{
	struct manual_control_setpoint_s manual = {};

	for (int i = 0; i < waitTimeOutInSecond; i++) {
		manual.timestamp = hrt_absolute_time();
		manual.x = x;
		manual.y = y;
		manual.z = z;
		manual.r = r;

		if (manual_control_setpoint_pub == nullptr) {
			manual_control_setpoint_pub = orb_advertise(ORB_ID(manual_control_setpoint), &manual);

		} else {
			orb_publish(ORB_ID(manual_control_setpoint), manual_control_setpoint_pub, &manual);
		}

		sleep(1);
	}

	return true;
}

bool AutomationTest::takeOffTest()
{
	//Takeoff to specified altitude, wait for some time till it climb up,
	//and then land.

	ut_assert_true(_takeOff(2.5, 10));
	ut_assert_true(_land(10));

	ut_assert_true(_takeOff(100, 60));
	ut_assert_true(_land(180));

	return true;
}

bool AutomationTest::manualTest()
{
	ut_assert_true(_takeOff(2.5, 10));

	//Move drone up in z direction
	_manualControl(0, 0, 1, 0, 10);

	//Move drone to north in x direction
	_manualControl(0.5, 0, 0.5, 0, 10);

	//Move drone to east in y direction
	_manualControl(0, 0.5, 0.5, 0, 10);

	//stop
	_manualControl(0, 0, 0.5, 0, 2);

	ut_assert_true(_land(60));

	return true;
}

bool AutomationTest::failSafeTest()
{
	ut_assert_true(_takeOff(5, 10));

	//Move drone to north in x direction
	_manualControl(0.5, 0, 0.5, 0, 5);

	//Move drone to east in y direction
	_manualControl(0, 0.5, 0.5, 0, 5);

	ut_assert_true(_rtlForFailSafe(120));

	//Disable fail safe for other tests
	ut_assert_true(enableFailSafe(false));

	return true;
}

bool AutomationTest::run_tests()
{
	ut_assert_true(enableFailSafe(false));
	ut_run_test(takeOffTest);
	ut_run_test(manualTest);
	ut_run_test(failSafeTest);

	return (_tests_failed == 0);
}

class BatteryTest : public AutomationTest
{
public:
	BatteryTest() {};
	virtual ~BatteryTest() {};
	virtual bool run_tests();

private:
	//internal functions
	bool _batteryWarning(int, int);

	//tests
	bool batteryTest();

};

bool BatteryTest::_batteryWarning(int warningLevel, int waitTimeOutInSecond)
{
	ut_assert_true(warningLevel == battery_status_s::BATTERY_WARNING_CRITICAL ||
		       warningLevel == battery_status_s::BATTERY_WARNING_EMERGENCY);

	//Limit battery voltage to fly normally
	float minimal_battery_percentage = 50.0f;
	ut_assert_true(param_set(param_find("SIM_BAT_MIN_PCT"), &minimal_battery_percentage) == OK);

	ut_assert_true(_takeOff(5, 10));

	//Move drone to north in x direction
	_manualControl(0.5, 0, 0.5, 0, 5);

	//Move drone to east in y direction
	_manualControl(0, 0.5, 0.5, 0, 5);

	//Stop
	_manualControl(0, 0, 0.5, 0, 2);

	sleep(10);
	_saveLandingPosition();

	//Simulate battery level
	minimal_battery_percentage = (warningLevel == battery_status_s::BATTERY_WARNING_CRITICAL) ? 94.0f : 97.0f;
	ut_assert_true(param_set(param_find("SIM_BAT_MIN_PCT"), &minimal_battery_percentage) == OK);

	//Auto RTL for critical battery level and auto land for emergency battery level
	ut_assert_true(_checkLandedStatus(waitTimeOutInSecond,
					  warningLevel == battery_status_s::BATTERY_WARNING_CRITICAL ? AUTOMATION_AUTO_RTL : AUTOMATION_AUTO_LAND));

	_disarm();

	//Low battery and cannot arm
	ut_assert_false(_arm());

	return true;
}

bool BatteryTest::batteryTest()
{
	ut_assert_true(argc == 2);

	uint8_t batteryLevel = battery_status_s::BATTERY_WARNING_NONE;

	if (!strcmp(argv[1], "critical")) {
		batteryLevel = battery_status_s::BATTERY_WARNING_CRITICAL;

	} else if (!strcmp(argv[1], "emergency")) {
		batteryLevel = battery_status_s::BATTERY_WARNING_EMERGENCY;
	}

	ut_assert_true(batteryLevel != battery_status_s::BATTERY_WARNING_NONE);

	//Land at current position when battery low
	int low_battery_action = 3;
	ut_assert_true(param_set(param_find("COM_LOW_BAT_ACT"), &low_battery_action) == OK);

	//Emergency battery level test
	ut_assert_true(_batteryWarning(batteryLevel, 120));

	return true;
}

bool BatteryTest::run_tests()
{
	ut_assert_true(enableFailSafe(false));
	ut_run_test(batteryTest);

	return (_tests_failed == 0);
}

ut_declare_test_c(test_automation, AutomationTest)
ut_declare_test_c(test_battery, BatteryTest)
