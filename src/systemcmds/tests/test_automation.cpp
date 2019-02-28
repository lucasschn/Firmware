
#include <unit_test.h>

#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/battery_status.h>

#include <commander/state_machine_helper.h>

#include <containers/List.hpp>
#include <platforms/px4_tasks.h>
#include <string.h>
#include <unistd.h>

#undef PX4_DEBUG
#define PX4_DEBUG PX4_INFO

extern transition_result_t arm_disarm(bool arm, orb_advert_t *mavlink_log_pub_local, const char *armedBy);

class uORB_topic : public ListNode<uORB_topic *>
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
	uORB_topic_list.add(&TOPIC##_uORB_topic_obj);

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
		  (long long unsigned) vehicle_global_position_status.timestamp);
}

DECLARE_UORB_SUB(vehicle_land_detected)
void vehicle_land_detected_uORB_topic::dump()
{
	PX4_DEBUG("land detected = %d", vehicle_land_detected_status.landed);
}

DECLARE_UORB_SUB(commander_state)
void commander_state_uORB_topic::dump()
{
	PX4_DEBUG("main_state = %d", commander_state_status.main_state);
}

DECLARE_UORB_SUB(vehicle_status)
void vehicle_status_uORB_topic::dump()
{
	PX4_DEBUG("nav_state = %d", vehicle_status_status.nav_state);
}

#define PUBLISH_TOPIC(topic, data) \
	if (topic##_pub == nullptr) { \
		topic##_pub = orb_advertise(ORB_ID(topic), &data); \
	} else { \
		orb_publish(ORB_ID(topic), topic##_pub, &data); \
	}

#define WAIT_FOR_CONDITION(condition, timeout) \
	for (int i = 0; i < timeout; i++) { \
		if (condition) { \
			break; \
		} \
		px4_usleep(1000000); \
		ut_assert_true(i<timeout); \
	}

class AutomationTest : public UnitTest
{
public:
	AutomationTest();
	virtual ~AutomationTest();
	virtual bool run_tests();

	bool task_should_exit;

	friend int updateTopicTask(int argc, char *argv[]);
	friend int manualControlTask(int argc, char *argv[]);

	typedef enum {
		AUTOMATION_AUTO_LAND = 0,
		AUTOMATION_AUTO_RTL  = 1
	} automation_land_t ;
protected:
	//internal functions
	bool _arm();
	bool _disarm();
	bool _takeOff(float, int);
	bool _land(int);
	void _sendManualControl();
	void _setManualControl(float, float, float, float);
	void _setManualControlAndWait(float, float, float, float, int);
	bool _assertLandedStatus(int, automation_land_t);
	bool _assertState(int, int); //Assert main state and navigate state
	bool _rtlForRCLost(int);
	bool _isGPSAvailable();
	void _saveLandingPosition();

	//tests
	bool takeOffTest();
	bool landTest();
	bool manualTest();
	bool rcLostTest();

protected:
	vehicle_local_position_s _takeoff_vehicle_local_position_status; //local position when take off
	vehicle_local_position_s _landing_vehicle_local_position_status; //local position when start landing
	manual_control_setpoint_s _manual_control_setpoint_data; //manual control setpoint data

private:
	//advertised topics
	orb_advert_t vehicle_command_pub = nullptr;
	orb_advert_t mavlink_log_pub = nullptr;
	orb_advert_t manual_control_setpoint_pub = nullptr;
	orb_advert_t battery_status_pub = nullptr;

	const uint8_t mavlink_system_id = 1;
	const uint8_t mavlink_component_id = 1;

	int update_topic_task;
	int manual_control_task;
	pthread_mutex_t	manual_control_mutex;
	bool publish_manual_control;

	List<uORB_topic *> uORB_topic_list;
};

//Only one instance is created for each test session. We use this instance
//to access test memembers, instead of using static variables.
static AutomationTest *automationTest = nullptr;

int updateTopicTask(int argc, char *argv[])
{
	while (!automationTest->task_should_exit) {

		for (uORB_topic *it = automationTest->uORB_topic_list.getHead(); it; it = it->getSibling()) {
			bool updated;
			orb_check(it->subscribe, &updated);

			if (updated) {
				PX4_DEBUG("Topic updated: %s", it->name);
				orb_copy(it->meta, it->subscribe, it->status);
				it->is_updated = true;
				it->dump();
			}
		}

		px4_usleep(1000000);
	}

	return 0;
}

//The task is reponsible for sending out manual control message at background.
//Add a dedicate task is to avoid loops in other logic.
int manualControlTask(int argc, char *argv[])
{
	while (!automationTest->task_should_exit) {
		if (automationTest->publish_manual_control) {
			automationTest->_sendManualControl();
		}

		px4_usleep(100000);
	}

	return 0;
}

AutomationTest::AutomationTest()
{
	automationTest = this;

	APPEND_UORB_SUB(vehicle_local_position);
	APPEND_UORB_SUB(vehicle_global_position);
	APPEND_UORB_SUB(vehicle_land_detected);
	APPEND_UORB_SUB(commander_state);
	APPEND_UORB_SUB(vehicle_status);

	for (uORB_topic *it = automationTest->uORB_topic_list.getHead(); it; it = it->getSibling()) {
		PX4_DEBUG("Subscribe %s", it->name);

		if (it->subscribe < 0) {
			it->subscribe = orb_subscribe(it->meta);
		}
	}

	//Set initial RC state
	_setManualControl(0, 0, 0.5, 0);

	//Reset parameters which could be updated in previous test
	float minimal_battery_percentage = 50.f;
	param_set(param_find("SIM_BAT_MIN_PCT"), &minimal_battery_percentage);

	int navigator_datalinklost_action =  0;
	param_set(param_find("NAV_DLL_ACT"), &navigator_datalinklost_action);

	task_should_exit = false;
	update_topic_task = px4_task_spawn_cmd("update topic task",
					       SCHED_DEFAULT,
					       SCHED_PRIORITY_DEFAULT + 40,
					       2048,
					       (px4_main_t)&updateTopicTask,
					       nullptr);

	//By default, manul control message is published
	publish_manual_control = true;
	pthread_mutex_init(&manual_control_mutex, nullptr);
	manual_control_task = px4_task_spawn_cmd("manual control task",
			      SCHED_DEFAULT,
			      SCHED_PRIORITY_DEFAULT + 40,
			      2048,
			      (px4_main_t)&manualControlTask,
			      nullptr);
}

AutomationTest::~AutomationTest()
{
	task_should_exit = true;

	px4_task_delete(update_topic_task);
	px4_task_delete(manual_control_task);
	pthread_mutex_destroy(&manual_control_mutex);

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

	for (uORB_topic *it = automationTest->uORB_topic_list.getHead(); it; it = it->getSibling()) {
		PX4_DEBUG("Unsubscribe %s", it->name);

		if (it->subscribe >= 0) {
			orb_unsubscribe(it->subscribe);
			it->subscribe = -1;
		}
	}
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

	WAIT_FOR_CONDITION(_isGPSAvailable(), max_wait_gps_seconds);

	//Wait a while for status stable after GPS available
	px4_usleep(5000000);

	//Save local position status when taking off
	memcpy(&_takeoff_vehicle_local_position_status, &vehicle_local_position_status, sizeof(vehicle_local_position_s));

	ut_assert_true(_isGPSAvailable());
	ut_assert_true(_arm());

	struct vehicle_command_s cmd = {};
	cmd.timestamp = hrt_absolute_time();
	cmd.param5 = NAN;
	cmd.param6 = NAN;
	cmd.param1 = NAN;
	cmd.param2 = NAN;
	cmd.param3 = NAN;
	cmd.param4 = NAN;
	cmd.param7 = NAN;
	cmd.command = vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF;
	cmd.target_system = mavlink_system_id;
	cmd.target_component = mavlink_component_id;
	cmd.source_system = 0;
	cmd.source_component = 0;
	cmd.confirmation = 0;
	cmd.from_external = false;

	PUBLISH_TOPIC(vehicle_command, cmd);

	ut_assert_true(param_get(param_find("MIS_TAKEOFF_ALT"), &mis_takeoff_alt) == OK);
	PX4_DEBUG("mis_takeoff_alt = %f", mis_takeoff_alt);

	//Wait for state change
	px4_usleep(1000000);
	ut_assert_true(_assertState(commander_state_s::MAIN_STATE_AUTO_TAKEOFF,
				    vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF));

	WAIT_FOR_CONDITION((std::fabs(mis_takeoff_alt + vehicle_local_position_status.z) < min_altitude_delta),
			   waitTimeOutInSecond);

	//Assert vertical speed is zero. Sleep a while to let it stable.
	px4_usleep(15000000);
	ut_assert_true((std::fabs(vehicle_local_position_status.vz) < min_vertical_speed));

	//Assert that current position is same as takeoff altitude.
	//The local_pos.z direction is downward, so the difference = mis_takeoff_alt - ( - local_pos.z)
	PX4_DEBUG("Take off altitude difference = %f", mis_takeoff_alt + vehicle_local_position_status.z);
	ut_assert_true((std::fabs(mis_takeoff_alt + vehicle_local_position_status.z) < min_altitude_delta));

	//Assert drone is in loiter state
	ut_assert_true(_assertState(commander_state_s::MAIN_STATE_AUTO_LOITER,
				    vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER));

	return true;
}

bool AutomationTest::_assertLandedStatus(int waitTimeOutInSecond, automation_land_t land_type)
{
	const float max_local_position_diff = 0.2;

	WAIT_FOR_CONDITION(vehicle_land_detected_status.landed, waitTimeOutInSecond);

	PX4_DEBUG("vehicle_local_position_status.z = %f", vehicle_local_position_status.z);

	//Assert landed is detected
	ut_assert_true(vehicle_land_detected_status.landed);

	//Assert the altitude is less than 0.5m after landing
	ut_assert_true(vehicle_local_position_status.z > -0.5);

	PX4_DEBUG("land_type = %d", land_type);
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

bool AutomationTest::_assertState(int main_state, int nav_state)
{
	ut_assert_true(commander_state_status.main_state == main_state);
	ut_assert_true(vehicle_status_status.nav_state == nav_state);

	return true;
}

bool AutomationTest::_land(int waitTimeOutInSecond)
{
	_saveLandingPosition();

	struct vehicle_command_s cmd = {};
	cmd.timestamp = hrt_absolute_time();
	cmd.param5 = NAN;
	cmd.param6 = NAN;
	cmd.param1 = NAN;
	cmd.param2 = NAN;
	cmd.param3 = NAN;
	cmd.param4 = NAN;
	cmd.param7 = NAN;
	cmd.command = vehicle_command_s::VEHICLE_CMD_NAV_LAND;
	cmd.target_system = mavlink_system_id;
	cmd.target_component = mavlink_component_id;
	cmd.source_system = 0;
	cmd.source_component = 0;
	cmd.confirmation = 0;
	cmd.from_external = false;

	PUBLISH_TOPIC(vehicle_command, cmd);

	px4_usleep(2000000);
	ut_assert_true(_assertState(commander_state_s::MAIN_STATE_AUTO_LAND,
				    vehicle_status_s::NAVIGATION_STATE_AUTO_LAND));

	ut_assert_true(_assertLandedStatus(waitTimeOutInSecond, AUTOMATION_AUTO_LAND));

	//Sometimes it is disarmed automatically already,
	//so it cannot assert true here
	_disarm();

	return true;
}

bool AutomationTest::_rtlForRCLost(int waitTimeOutInSecond)
{
	int main_state = commander_state_status.main_state;
	int nav_state = vehicle_status_status.nav_state;

	PX4_INFO("%s: RC lost", __FUNCTION__);
	publish_manual_control = false;
	px4_usleep(5000000);

	//Assert drone is still in original main state, and in auto RTL navigate state
	ut_assert_true(_assertState(main_state, vehicle_status_s::NAVIGATION_STATE_AUTO_RTL));

	PX4_INFO("%s: RC regain", __FUNCTION__);
	publish_manual_control = true;
	px4_usleep(5000000);

	//Assert the drone is changed to original state after RC resume
	ut_assert_true(_assertState(main_state, nav_state));

	PX4_INFO("%s: RC lost", __FUNCTION__);
	publish_manual_control = false;
	px4_usleep(2000000);

	//Assert drone is still in original main state, and in auto RTL navigate state
	ut_assert_true(_assertState(main_state, vehicle_status_s::NAVIGATION_STATE_AUTO_RTL));

	//Assert drone landed at take off position
	ut_assert_true(_assertLandedStatus(waitTimeOutInSecond, AUTOMATION_AUTO_RTL));

	_disarm();

	PX4_INFO("%s: RC regain", __FUNCTION__);
	publish_manual_control = true;

	return true;
}

bool AutomationTest::_isGPSAvailable()
{
	return vehicle_global_position_uORB_topic_obj.is_updated;
}

void AutomationTest::_saveLandingPosition()
{
	memcpy(&_landing_vehicle_local_position_status, &vehicle_local_position_status, sizeof(vehicle_local_position_s));
}

void AutomationTest::_setManualControl(float x, float y, float z, float r)
{
	pthread_mutex_lock(&manual_control_mutex);

	_manual_control_setpoint_data.x = x;
	_manual_control_setpoint_data.y = y;
	_manual_control_setpoint_data.z = z;
	_manual_control_setpoint_data.r = r;
	_manual_control_setpoint_data.mode_slot = manual_control_setpoint_s::MODE_SLOT_3; //position control

	pthread_mutex_unlock(&manual_control_mutex);
}

void AutomationTest::_setManualControlAndWait(float x, float y, float z, float r, int interval)
{
	_setManualControl(x, y, z, r);
	px4_usleep(interval * 1000000);
}

void AutomationTest::_sendManualControl()
{
	pthread_mutex_lock(&manual_control_mutex);

	_manual_control_setpoint_data.timestamp = hrt_absolute_time();
	PUBLISH_TOPIC(manual_control_setpoint, _manual_control_setpoint_data);

	pthread_mutex_unlock(&manual_control_mutex);
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
	_setManualControlAndWait(0, 0, 1, 0, 10);

	//Move drone to north in x direction
	_setManualControlAndWait(0.5, 0, 0.5, 0, 10);

	//Move drone to east in y direction
	_setManualControlAndWait(0, 0.5, 0.5, 0, 10);

	//stop
	_setManualControlAndWait(0, 0, 0.5, 0, 2);

	ut_assert_true(_land(60));

	return true;
}

bool AutomationTest::rcLostTest()
{
	//Test in auto loiter state
	ut_assert_true(_takeOff(20, 60));
	ut_assert_true(_rtlForRCLost(120));

	//Test in position control state
	ut_assert_true(_takeOff(5, 10));

	//Move drone to north in x direction
	_setManualControlAndWait(0.5, 0, 0.5, 0, 5);

	//Move drone to east in y direction
	_setManualControlAndWait(0, 0.5, 0.5, 0, 5);

	ut_assert_true(_rtlForRCLost(120));

	return true;
}

bool AutomationTest::run_tests()
{
	ut_run_test(takeOffTest);
	ut_run_test(manualTest);
	ut_run_test(rcLostTest);

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
	const int wait_for_battery_drain_timeout = 30;

	ut_assert_true(warningLevel == battery_status_s::BATTERY_WARNING_CRITICAL ||
		       warningLevel == battery_status_s::BATTERY_WARNING_EMERGENCY);

	//Limit battery voltage to fly normally
	float minimal_battery_percentage = 50.0f;
	ut_assert_true(param_set(param_find("SIM_BAT_MIN_PCT"), &minimal_battery_percentage) == OK);

	ut_assert_true(_takeOff(5, 10));

	//Move drone to north in x direction
	_setManualControlAndWait(0.5, 0, 0.5, 0, 5);

	//Move drone to east in y direction
	_setManualControlAndWait(0, 0.5, 0.5, 0, 5);

	//Stop
	_setManualControlAndWait(0, 0, 0.5, 0, 2);

	px4_usleep(10000000);
	_saveLandingPosition();

	//Simulate battery level
	minimal_battery_percentage = (warningLevel == battery_status_s::BATTERY_WARNING_CRITICAL) ? 94.0f : 97.0f;
	ut_assert_true(param_set(param_find("SIM_BAT_MIN_PCT"), &minimal_battery_percentage) == OK);

	if (warningLevel == battery_status_s::BATTERY_WARNING_CRITICAL) {
		WAIT_FOR_CONDITION(commander_state_status.main_state == commander_state_s::MAIN_STATE_AUTO_RTL,
				   wait_for_battery_drain_timeout);

		//Assert drone is in auto RTL main state and auto RTL nagivate state
		ut_assert_true(_assertState(commander_state_s::MAIN_STATE_AUTO_RTL, vehicle_status_s::NAVIGATION_STATE_AUTO_RTL));

	} else {
		WAIT_FOR_CONDITION(commander_state_status.main_state == commander_state_s::MAIN_STATE_AUTO_LAND,
				   wait_for_battery_drain_timeout);

		//Assert drone is in auto RTL main state and auto land nagivate state
		ut_assert_true(_assertState(commander_state_s::MAIN_STATE_AUTO_LAND, vehicle_status_s::NAVIGATION_STATE_AUTO_LAND));
	}

	//Auto RTL for critical battery level and auto land for emergency battery level
	ut_assert_true(_assertLandedStatus(waitTimeOutInSecond,
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
	ut_run_test(batteryTest);

	return (_tests_failed == 0);
}

ut_declare_test_c(test_automation, AutomationTest)
ut_declare_test_c(test_battery, BatteryTest)
