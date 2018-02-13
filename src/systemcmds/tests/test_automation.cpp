
#include <unit_test.h>

#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_land_detected.h>

#include <commander/state_machine_helper.h>

#include <platforms/px4_tasks.h>

//#undef PX4_DEBUG
//#define PX4_DEBUG PX4_INFO

extern transition_result_t arm_disarm(bool arm, orb_advert_t *mavlink_log_pub_local, const char *armedBy);

//subscribed topics
static int vehicle_local_position_sub = -1;
static int vehicle_land_detected_sub = -1;

//task control
static bool task_should_exit = false;
static int daemon_task = -1;

//advertised topics
static orb_advert_t vehicle_command_pub = nullptr;
static orb_advert_t mavlink_log_pub = nullptr;

//subscribed status
static vehicle_local_position_s vehicle_local_position_status;
static vehicle_land_detected_s vehicle_land_detected_status;

class AutomationTest : public UnitTest
{
public:
	AutomationTest();
	~AutomationTest();
	virtual bool run_tests();

private:
	bool preset();

	//internal functions
	bool _arm();
	bool _disarm();
	bool _takeOff(float altitude, int waitTimeSecond);
	bool _land(int waitTimeSecond);

	//tests
	bool takeOffTest();
	bool landTest();
};

static void updateTopicTask(int argc, char *argv[])
{
	while (!task_should_exit) {
		bool updated;

		orb_check(vehicle_local_position_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(vehicle_local_position), vehicle_local_position_sub, &vehicle_local_position_status);
			PX4_DEBUG("x = %f, y = %f, z = %f",
				  vehicle_local_position_status.x, vehicle_local_position_status.y, vehicle_local_position_status.z);

		} else {
			PX4_DEBUG("vehicle_local_position_sub is not updated");
		}

		orb_check(vehicle_land_detected_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(vehicle_land_detected), vehicle_land_detected_sub, &vehicle_land_detected_status);
			PX4_DEBUG("land detected = %d", vehicle_land_detected_status.landed);

		} else {
			PX4_DEBUG("vehicle_land_detected_sub is not updated");
		}

		sleep(1);
	}
}

AutomationTest::AutomationTest()
{
	if (vehicle_local_position_sub < 0) {
		vehicle_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	}

	if (vehicle_land_detected_sub < 0) {
		vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
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

	//unsubscribe topics
	if (vehicle_local_position_sub >= 0) {
		orb_unsubscribe(vehicle_local_position_sub);
		vehicle_local_position_sub = -1;
	}

	if (vehicle_land_detected_sub >= 0) {
		orb_unsubscribe(vehicle_land_detected_sub);
		vehicle_land_detected_sub = -1;
	}
}

bool AutomationTest::preset()
{
	//Disable datalink lost and rc lost action to avoid RTL,
	//since the automation test is done without RC or datalink.
	int navigator_datalinklost_action = 0;
	ut_assert_true(param_set(param_find("NAV_DLL_ACT"), &navigator_datalinklost_action) == OK);

	int navigator_rclost_action = 0;
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

bool AutomationTest::_takeOff(float altitude, int waitTimeSecond)
{
	float mis_takeoff_alt = altitude;

	ut_assert_true(altitude > 0);
	ut_assert_true(param_set(param_find("MIS_TAKEOFF_ALT"), &mis_takeoff_alt) == OK);

	struct vehicle_status_s status = {};
	status.system_id = 1; //MAV_SYS_ID
	status.component_id = 1; //MAV_COMP_ID

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
		.target_system = status.system_id,
		.target_component = status.component_id
	};

	if (vehicle_command_pub == nullptr) {
		vehicle_command_pub = orb_advertise_queue(ORB_ID(vehicle_command), &cmd, vehicle_command_s::ORB_QUEUE_LENGTH);

	} else {
		orb_publish(ORB_ID(vehicle_command), vehicle_command_pub, &cmd);
	}

	ut_assert_true(param_get(param_find("MIS_TAKEOFF_ALT"), &mis_takeoff_alt) == OK);
	PX4_DEBUG("mis_takeoff_alt = %f", mis_takeoff_alt);

	sleep(waitTimeSecond);

	//Assert that current position is same as takeoff altitude.
	//The local_pos.z direction is downward, so the difference = mis_takeoff_alt - ( - local_pos.z)
	ut_assert_true((std::fabs(mis_takeoff_alt + vehicle_local_position_status.z) < 0.5));

	PX4_DEBUG("diff = %f", mis_takeoff_alt + vehicle_local_position_status.z);

	return true;
}

bool AutomationTest::_land(int waitTimeSecond)
{
	struct vehicle_status_s status = {};
	status.system_id = 1; //MAV_SYS_ID
	status.component_id = 1; //MAV_COMP_ID

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
		.target_system = status.system_id,
		.target_component = status.component_id
	};

	if (vehicle_command_pub == nullptr) {
		vehicle_command_pub = orb_advertise_queue(ORB_ID(vehicle_command), &cmd, vehicle_command_s::ORB_QUEUE_LENGTH);

	} else {
		orb_publish(ORB_ID(vehicle_command), vehicle_command_pub, &cmd);
	}

	sleep(waitTimeSecond);

	PX4_DEBUG("vehicle_local_position_status.z = %f", vehicle_local_position_status.z);

	//Assert landed is detected
	ut_assert_true(vehicle_land_detected_status.landed);

	//Assert the altitude is less than 0.5m after landing
	ut_assert_true(vehicle_local_position_status.z > -0.5);

	//Sometimes it is disarmed automatically already,
	//so it cannot assert true here
	_disarm();

	return true;
}

bool AutomationTest::takeOffTest()
{
	//Takeoff to specified altitude, wait for some time till it climb up,
	//and then land.
	ut_assert_true(_takeOff(2.5, 10));
	ut_assert_true(_land(10));

	ut_assert_true(_takeOff(100, 45));
	ut_assert_true(_land(110));

	return true;
}

bool AutomationTest::run_tests()
{
	ut_assert_true(preset());
	ut_run_test(takeOffTest);

	return (_tests_failed == 0);
}

ut_declare_test_c(test_automation, AutomationTest)
