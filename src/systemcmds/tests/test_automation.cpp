
#include <vector>
#include <string>

#include <unit_test.h>

#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_global_position.h>

#include <commander/state_machine_helper.h>

#include <platforms/px4_tasks.h>

//#undef PX4_DEBUG
//#define PX4_DEBUG PX4_INFO

extern transition_result_t arm_disarm(bool arm, orb_advert_t *mavlink_log_pub_local, const char *armedBy);

class uORB_topic
{
public:
	uORB_topic() {}
	virtual ~uORB_topic() {}

public:
	std::string name;
	const struct orb_metadata *meta;
	int *subscribe;
	void *status;
	bool *is_updated;

	virtual void dump() = 0;
};

static std::vector<uORB_topic *> uORB_topic_list;

#define DECLARE_UORB_SUB(TOPIC) \
	static int TOPIC##_sub = -1; \
	static TOPIC##_s TOPIC##_status; \
	static bool is_##TOPIC##_updated = false;

#define APPEND_UORB_SUB(TOPIC) \
	{ \
		uORB_topic *topic = new TOPIC##_uORB_topic; \
		topic->name = #TOPIC; \
		topic->meta = ORB_ID(TOPIC); \
		topic->subscribe = &TOPIC##_sub; \
		topic->status = &TOPIC##_status; \
		topic->is_updated = &is_##TOPIC##_updated; \
		uORB_topic_list.push_back(topic); \
	}

//declare subscribed topics
DECLARE_UORB_SUB(vehicle_local_position)
class vehicle_local_position_uORB_topic : public uORB_topic
{
public:
	void dump()
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
};

DECLARE_UORB_SUB(vehicle_global_position)
class vehicle_global_position_uORB_topic : public uORB_topic
{
public:
	void dump()
	{
		PX4_DEBUG("lat = %f, lon = %f, alt =%f, eph = %f, epv = %f, timestamp = %llu",
			  vehicle_global_position_status.lat,
			  vehicle_global_position_status.lon,
			  vehicle_global_position_status.alt,
			  vehicle_global_position_status.eph,
			  vehicle_global_position_status.epv,
			  vehicle_global_position_status.timestamp);
	}
};

DECLARE_UORB_SUB(vehicle_land_detected)
class vehicle_land_detected_uORB_topic : public uORB_topic
{
public:
	void dump()
	{
		PX4_DEBUG("land detected = %d", vehicle_land_detected_status.landed);
	}
};

//task control
static bool task_should_exit = false;
static int daemon_task = -1;

//advertised topics
static orb_advert_t vehicle_command_pub = nullptr;
static orb_advert_t mavlink_log_pub = nullptr;
static orb_advert_t manual_control_setpoint_pub = nullptr;

static const int mavlink_system_id = 1;
static const int mavlink_component_id = 1;

class AutomationTest : public UnitTest
{
public:
	AutomationTest();
	~AutomationTest();
	virtual bool run_tests();

private:
	bool enableFailSafe(bool);

	//internal functions
	bool _arm();
	bool _disarm();
	bool _takeOff(float, int);
	bool _land(int);
	bool _manualControl(float, float, float, float, int);
	bool _checkLandedStatus();
	bool _rtlForFailSafe(int);
	bool _isGPSAvaiable();

	//tests
	bool takeOffTest();
	bool landTest();
	bool manualTest();
	bool failSafeTest();

private:
	vehicle_local_position_s _takeoff_vehicle_local_position_status;
};

static void updateTopicTask(int argc, char *argv[])
{
	while (!task_should_exit) {
		std::vector<uORB_topic *>::iterator it;

		for (it = uORB_topic_list.begin(); it != uORB_topic_list.end(); it++) {
			bool updated;
			orb_check(*((*it)->subscribe), &updated);

			if (updated) {
				PX4_DEBUG("Topic updated: %s", (*it)->name.c_str());
				orb_copy((*it)->meta, *((*it)->subscribe), (*it)->status);
				*((*it)->is_updated) = true;
				(*it)->dump();
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

	std::vector<uORB_topic *>::iterator it;

	for (it = uORB_topic_list.begin(); it != uORB_topic_list.end(); it++) {
		PX4_DEBUG("Subscribe %s", (*it)->name.c_str());

		if (*((*it)->subscribe) < 0) {
			*((*it)->subscribe) = orb_subscribe((*it)->meta);
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

	std::vector<uORB_topic *>::iterator it;

	for (it = uORB_topic_list.begin(); it != uORB_topic_list.end();) {
		PX4_DEBUG("Unsubscribe %s", (*it)->name.c_str());

		if (*((*it)->subscribe) >= 0) {
			orb_unsubscribe(*((*it)->subscribe));
			*((*it)->subscribe) = -1;
		}

		delete (*it);
		uORB_topic_list.erase(it);
	}
}

bool AutomationTest::enableFailSafe(bool enabled)
{
	//Disable datalink lost and rc lost action to avoid RTL,
	//since the automation test is done without RC or datalink.
	int navigator_datalinklost_action = enabled ? 2 : 0;
	ut_assert_true(param_set(param_find("NAV_DLL_ACT"), &navigator_datalinklost_action) == OK);

	int navigator_rclost_action = enabled ? 2 : 0;
	ut_assert_true(param_set(param_find("NAV_RCL_ACT"), &navigator_rclost_action) == OK);

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

bool AutomationTest::_checkLandedStatus()
{
	PX4_DEBUG("vehicle_local_position_status.z = %f", vehicle_local_position_status.z);

	//Assert landed is detected
	ut_assert_true(vehicle_land_detected_status.landed);

	//Assert the altitude is less than 0.5m after landing
	ut_assert_true(vehicle_local_position_status.z > -0.5);

	return true;
}

bool AutomationTest::_land(int waitTimeOutInSecond)
{
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

	int runningSeconds;

	for (runningSeconds = 0 ; runningSeconds < waitTimeOutInSecond; runningSeconds++) {
		if (vehicle_land_detected_status.landed) {
			break;
		}

		sleep(1);
	}

	//Assert not time out
	ut_assert_true(runningSeconds < waitTimeOutInSecond);

	_checkLandedStatus();

	//Sometimes it is disarmed automatically already,
	//so it cannot assert true here
	_disarm();

	return true;
}

bool AutomationTest::_rtlForFailSafe(int waitTimeOutInSecond)
{
	const float max_local_position_diff = 0.06;

	ut_assert_true(enableFailSafe(true));

	int runningSeconds;

	for (runningSeconds = 0 ; runningSeconds < waitTimeOutInSecond; runningSeconds++) {
		if (vehicle_land_detected_status.landed) {
			break;
		}

		sleep(1);
	}

	//Assert not time out
	ut_assert_true(runningSeconds < waitTimeOutInSecond);

	_checkLandedStatus();

	PX4_DEBUG("Vehicle local position difference from take off: x = %f, y = %f, z = %f",
		  std::fabs(_takeoff_vehicle_local_position_status.x - vehicle_local_position_status.x),
		  std::fabs(_takeoff_vehicle_local_position_status.y - vehicle_local_position_status.y),
		  std::fabs(_takeoff_vehicle_local_position_status.z - vehicle_local_position_status.z));
	ut_assert_true((std::fabs(_takeoff_vehicle_local_position_status.x - vehicle_local_position_status.x) <
			max_local_position_diff));
	ut_assert_true((std::fabs(_takeoff_vehicle_local_position_status.y - vehicle_local_position_status.y) <
			max_local_position_diff));

	_disarm();

	return true;
}

bool AutomationTest::_isGPSAvaiable()
{
	return is_vehicle_global_position_updated;
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

	ut_assert_true(_rtlForFailSafe(60));

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

ut_declare_test_c(test_automation, AutomationTest)
