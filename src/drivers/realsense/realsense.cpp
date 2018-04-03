/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <px4_getopt.h>
#include <px4_workqueue.h>
#include <termios.h>
#include <lib/mathlib/mathlib.h>
#include <drivers/device/device.h>
#include <drivers/drv_range_finder.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <systemlib/param/param.h>
#include <lib/rc/st24.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/realsense_avoidance_setpoint.h>
#include <uORB/topics/realsense_distance_info.h>
#include <uORB/topics/obstacle_avoidance.h>
#include <uORB/topics/obstacle_distance.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/home_position.h>

#include <lib/FlightTasks/FlightTasks.hpp>

#include "realsense.h"

#define B460800 460800
#define MAX_YPRC_PORT_NUM			2
#define ST24_LENGTH_VALUE			130
#define REALSENSE_FRAME_SIZE		50
#define REALSENSE_BUFFER_SIZE		(REALSENSE_FRAME_SIZE + REALSENSE_FRAME_SIZE / 2)
#define PACKET_LENGTH_HEARTBEAT						 	sizeof(Heartbeat)
#define PACKET_LENGTH_REALSENSE_OBSTACLE_DATA_INPUT  	sizeof(ObstacleAvoidanceInput)
#define PACKET_LENGTH_REALSENSE_OBSTACLE_DATA_OUTPUT 	sizeof(ObstacleAvoidanceOutput)
#define PACKET_LENGTH_REALSENSE_OPTFLOW				 	sizeof(OpticalFlowDataOutput)
#define PACKET_LENGTH_REALSENSE_DISTANCE_INFO			sizeof(ObstacleDistanceInfo)
#define PACKET_LENGTH_REALSENSE_DISTANCE_360			sizeof(ObstacleDistance360)
#define PACKET_LENGTH_REALSNESE_CHT_COMMAND				sizeof(ChtCommand)
#define _YP_PACKET_DATA(msg) ( (const char *)( &((msg)->st24_data[0])))

/* Measurement rate is 100Hz */
#define CONVERSION_INTERVAL_REALSENSE	(1000000 / 100)	/* microseconds */
#define REALSENSE_CONVERSION_INTERVAL    10000
#define REALSENSE_DEVICE_PATH "/dev/realsense"

#define DELAY_FLOW_US 38000 // delay of pixel flow measurements compared to gyro from auto pilot
#define DISTANCE_SENSOR_INSTANCES 2

#define TIME_WAIT_SHUTDOWN_US 60000000 // 1min

#define MAX_DIST 2000 //20 m

#define SIGMA_NORM			0.001f

class REALSENSE: public device::CDev
{
public:
	REALSENSE(const char *port);
	~REALSENSE();

	void start();
	void stop();
	virtual int			ioctl(struct file *filp, int cmd, unsigned long arg);
	virtual int			init_dev();
	/**
	 * @return true if this task is currently running.
	 */
	inline bool is_running() const
	{
		return _taskIsRunning;
	}
private:

	bool	_realsense_is_present;
	bool	_initialized;
	bool 	_taskShouldExit;
	bool 	_taskIsRunning;
	char 	_device_realsense[20];
	uint16_t  _read_write_count;
	uint64_t 	_init_time;
	int		_measure_ticks;
	int		_vehicle_local_position_sub;
	int		_avoidance_input_sub;
	int		_sensor_combined_sub;
	int 	_vehicle_attitude_sub;
	int 	_distance_sensor_subs[DISTANCE_SENSOR_INSTANCES];
	int 	_manual_sub;
	int 	_vehicle_status_sub;
	int 	_home_pos_sub;
	float _current_distance;
	struct vehicle_local_position_s _local_pos;
	struct vehicle_local_position_s _local_pos_prev;
	struct realsense_avoidance_setpoint_s _avoidance_input;
	struct vehicle_attitude_s _attitude;
	struct manual_control_setpoint_s _manual;
	struct vehicle_status_s _vehicle_status;
	struct home_position_s _home_pos;
	struct work_s	_work;
	ringbuffer::RingBuffer	*_rb_gyro;
	int _uart_fd = -1;
	float _yaw;
	float _nav_rad;

	FlightTasks _flight_tasks;

	orb_advert_t _obstacle_avoidance_pub;
	orb_advert_t _optical_flow_pub;
	orb_advert_t _distance_sensor_pub;
	orb_advert_t _realsense_distance_info_pub;
	orb_advert_t _obstacle_distance_pub;
	static void _cycle_trampoline(void *arg);
	void _init_realsense();  							 // init - initialise the sensor
	void update();     					 // update - check input and send out data
	void poll_subscriptions();				 // update all msg
	void _cycle_realsense();
	int _initialise_uart(const char *device);
	void _send_shutdown_command();									 // RealSense Command shutdow
	void _send_heartbeat();
	void _send_obstacle_avoidance_data();						     // send data to RealSense 100Hz
	int _realsense_parse_frame_sync(uint8_t index, uint8_t byte, ReceiverFcPacket *returnPacket);
	void _read_obstacle_avoidance_data();
};

namespace realsense
{

// driver 'main' command
/**
 * realsense start / stop handling function
 * This makes the realsense drivers accessible from the nuttx shell
 * @ingroup apps
 */
extern "C" __EXPORT int realsense_main(int argc, char *argv[]);

// Private variables
static REALSENSE *realsense_drv_task = nullptr;
static char _device[20] = {};

void usage();
static void realsense_stop(void);
}

REALSENSE::REALSENSE(const char *port):
	CDev("REALSENSE", REALSENSE_DEVICE_PATH),
	_realsense_is_present(false),
	_initialized(false),
	_taskShouldExit(false),
	_taskIsRunning(false),
	_read_write_count(0),
	_init_time(0),
	_measure_ticks(0),
	_vehicle_local_position_sub(-1),
	_avoidance_input_sub(-1),
	_sensor_combined_sub(-1),
	_vehicle_attitude_sub(-1),
	_distance_sensor_subs{},
	_manual_sub(-1),
	_vehicle_status_sub(-1),
	_current_distance(0.0f),
	_local_pos{},
	_local_pos_prev{},
	_avoidance_input{},
	_attitude {},
	_manual{},
	_vehicle_status{},
	_home_pos{},
	_work{},
	_rb_gyro(nullptr),
	_yaw(0.0f),
	_nav_rad(0.0f),
	_flight_tasks(),
	_obstacle_avoidance_pub(nullptr),
	_optical_flow_pub(nullptr),
	_distance_sensor_pub(nullptr),
	_realsense_distance_info_pub(nullptr),
	_obstacle_distance_pub(nullptr)
{
	/* store port name */
	strncpy(_device_realsense, port, sizeof(_device_realsense));
	/* enforce null termination */
	_device_realsense[sizeof(_device_realsense) - 1] = '\0';
	param_get(param_find("NAV_ACC_RAD"), &_nav_rad);

}

REALSENSE::~REALSENSE()
{
	if (_initialized) {
		/* tell the task we want it to go away */
		_initialized = false;
	}

	if (_rb_gyro != nullptr) {
		delete _rb_gyro;
	}

	work_cancel(HPWORK, &_work);

	_taskShouldExit = true;
	::close(_uart_fd);
}


void
REALSENSE::start()
{
	/* reset the report ring and state machine */
	_rb_gyro->flush();
	_taskShouldExit = false;
	usleep(1000000);
	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&REALSENSE::_cycle_trampoline, this, 0);
}
int
REALSENSE::init_dev()
{
	int ret = PX4_ERROR;

	ret = CDev::init();

	if (ret != OK) {
		PX4_ERR("CDev init failed");
		goto out;
	}

	// allocate gyro buffer, size -> ~5samples per flow message, 38ms delay -> 4 extra samples, 3 room for error
	_rb_gyro = new ringbuffer::RingBuffer(12, sizeof(vector3f_timestamped));

	if (_rb_gyro == nullptr) {
		PX4_ERR("could not allocate gyro ringbuffer");
		return PX4_ERROR;
	}

	ret = OK;
out:
	return ret;
}
void
REALSENSE::stop()
{
	_taskShouldExit = true;
}
void
REALSENSE::_cycle_trampoline(void *arg)
{
	REALSENSE *dev = reinterpret_cast<REALSENSE *>(arg);
	dev->_cycle_realsense();
}

void
REALSENSE::poll_subscriptions()				 // update all msg
{
	// check and copy the msg
	bool updated;

	orb_check(_vehicle_local_position_sub, &updated);

	if (updated) {
		_local_pos_prev = _local_pos;
		orb_copy(ORB_ID(vehicle_local_position), _vehicle_local_position_sub, &_local_pos);
	}

	orb_check(_avoidance_input_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(realsense_avoidance_setpoint_input), _avoidance_input_sub, &_avoidance_input);
	}

	orb_check(_sensor_combined_sub, &updated);

	if (updated) {
		struct sensor_combined_s sensor_combined;
		vector3f_timestamped gyro_stamped;
		orb_copy(ORB_ID(sensor_combined), _sensor_combined_sub, &sensor_combined);
		// write gyro values into ring buffer for optical flow
		gyro_stamped.time_usec = sensor_combined.timestamp;
		gyro_stamped.x = sensor_combined.gyro_rad[0];
		gyro_stamped.y = sensor_combined.gyro_rad[1];
		gyro_stamped.z = sensor_combined.gyro_rad[2];

		_rb_gyro->force(&gyro_stamped);

	}

	orb_check(_vehicle_attitude_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &_attitude);

		/* get current rotation matrix and euler angles from control state quaternions */
		math::Quaternion q_att(_attitude.q[0], _attitude.q[1], _attitude.q[2], _attitude.q[3]);
		math::Matrix<3, 3> R = q_att.to_dcm();
		math::Vector<3> euler_angles;
		euler_angles = R.to_euler();
		_yaw = euler_angles(2);
	}

	for (unsigned i = 0; i < DISTANCE_SENSOR_INSTANCES; i++) {

		orb_check(_distance_sensor_subs[i], &updated);

		if (updated) {
			struct distance_sensor_s distance_sensor;
			orb_copy(ORB_ID(distance_sensor), _distance_sensor_subs[i], &distance_sensor);

			if (distance_sensor.orientation == distance_sensor_s::ROTATION_FORWARD_FACING) {
				_current_distance = distance_sensor.current_distance;        // to save RAM
			}
		}
	}

	orb_check(_manual_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual);
	}

	orb_check(_vehicle_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);
	}

	orb_check(_home_pos_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(home_position), _home_pos_sub, &_home_pos);
	}

}

void
REALSENSE::_send_heartbeat()
{
	Heartbeat tx = {};
	ReceiverFcPacket _txpacket = {};
	tx.componentType = ComponentType::FlightController;
	tx.versionMajor = FC_CHT_COMM_VERSION_MAJOR;
	tx.versionMinor = FC_CHT_COMM_VERSION_MINOR;

	// pack packet and send
	_txpacket.header1 = ST24_STX1;
	_txpacket.header2 = ST24_STX2;
	_txpacket.length = PACKET_LENGTH_HEARTBEAT + 2;
	_txpacket.type = PacketId::Heartbeat;
	memcpy(_txpacket.st24_data, (const uint8_t *)&tx, PACKET_LENGTH_HEARTBEAT);
	_txpacket.st24_data[PACKET_LENGTH_HEARTBEAT] = st24_common_crc8((uint8_t *) & (_txpacket.length), _txpacket.length);
	int ret = ::write(_uart_fd, (uint8_t *)&_txpacket, (_txpacket.length + 3));

	if (ret < 0 && _realsense_is_present) {
		PX4_ERR("failed to write heart data to realsense (%i)", ret);
	}
}

void
REALSENSE::_send_obstacle_avoidance_data()
{
	ReceiverFcPacket _txpacket = {};
	ObstacleAvoidanceInput tx = {};

	// desired velocity - ENU
	if (!PX4_ISFINITE(_avoidance_input.vx) || !PX4_ISFINITE(_avoidance_input.vy) || !PX4_ISFINITE(_avoidance_input.vz)) {
		return;
	}

	const float avoidance_input_z = _avoidance_input.vz;
	const float diff_xy = matrix::Vector2f((_local_pos_prev.x - _local_pos.x), (_local_pos_prev.y - _local_pos.y)).length();
	const bool xy_lock = diff_xy < SIGMA_NORM;
	const float diff_z = fabsf(_local_pos_prev.z - _local_pos.z);
	const bool z_lock = diff_z < SIGMA_NORM;

	/* get velocity setpoint in heading frame */
	matrix::Quatf q_yaw = matrix::AxisAnglef(matrix::Vector3f(0.0f, 0.0f, 1.0f), _yaw);
	matrix::Vector3f vel_sp_heading = q_yaw.conjugate_inversed(matrix::Vector3f(_avoidance_input.vx, _avoidance_input.vy,
					  0.0f));

	if (!xy_lock && vel_sp_heading(0) < 0.0f && (fabsf(vel_sp_heading(1)) <  0.1f)) {

		/* we adjust the y desired velocity setpoint such that 180 turns are possible. */

		vel_sp_heading(1) = fabsf(vel_sp_heading(0)) * 0.1f + 0.001f;

	} else if (xy_lock && !z_lock) {

		/* we adjust the desired velocity setpoint such that the vehicle can fly upwards
		 * and downwards if only z-direction maneuvers are requested. This is rather a hack to
		 * fix realsense output for straight up and down maneuvers. */

		vel_sp_heading(0) = 0.001f + 0.1f * fabsf(_avoidance_input.vz);
	}

	matrix::Vector3f vel_sp_local = q_yaw.conjugate(vel_sp_heading);
	_avoidance_input.vx = vel_sp_local(0);
	_avoidance_input.vy = vel_sp_local(1);
	_avoidance_input.vz = avoidance_input_z;

	tx.desiredSpeed.x   = _avoidance_input.vy; // E
	tx.desiredSpeed.y   = _avoidance_input.vx; // N
	tx.desiredSpeed.z   = -_avoidance_input.vz; // U

	// current attitude - NED the  just one of the four elements is be check is enough
	if (!PX4_ISFINITE(_attitude.q[0])) {
		return;
	}

	tx.attitude.w       = _attitude.q[0];  // wxyz format
	tx.attitude.x       = _attitude.q[1];  // wxyz format
	tx.attitude.y       = _attitude.q[2];  // wxyz format
	tx.attitude.z       = _attitude.q[3];  // wxyz format

	// current relative position - ENU
	if (!PX4_ISFINITE(_local_pos.x) || !PX4_ISFINITE(_local_pos.y) || !PX4_ISFINITE(_local_pos.z)) {
		return;
	}

	tx.position.x       = _local_pos.y; // position in relative coordinate system in m E
	tx.position.y       = _local_pos.x; // position in relative coordinate system in m N
	tx.position.z       = -_local_pos.z; // position in relative coordinate system in m U

	// sonar front in mm
	if (!PX4_ISFINITE(_current_distance)) {
		return;
	}

	tx.sonarFrontMm     = _current_distance * 1000.0f; // in mm

	// timestamp
	tx.timestamp        = hrt_absolute_time() * 1.0e-3;// in ms

	// unused fields
	tx.unused1 = {};
	tx.unused2 = {};
	tx.unused3 = {};

	// flags
	uint32_t flag = 0;

	if (_local_pos.xy_valid) {
		flag |= ObstacleAvoidanceInputFlags::OK_TO_FLY;
	}

	// set flag for Sense&Stop
	if ((_manual.obsavoid_switch == manual_control_setpoint_s::SWITCH_POS_ON)
	    && (_vehicle_status.nav_state == _vehicle_status.NAVIGATION_STATE_POSCTL) && (!_flight_tasks.isAnyTaskActive())) {
		flag |= ObstacleAvoidanceInputFlags::USE_SMALL_VEHICLE_SIZE;
	}

	tx.flags            = flag;

	// pack packet and send
	_txpacket.header1 = ST24_STX1;
	_txpacket.header2 = ST24_STX2;
	_txpacket.length = PACKET_LENGTH_REALSENSE_OBSTACLE_DATA_INPUT + 2;
	_txpacket.type = PacketId::ObstacleAvoidanceInput;
	memcpy(_txpacket.st24_data, (const uint8_t *)&tx, PACKET_LENGTH_REALSENSE_OBSTACLE_DATA_INPUT);
	_txpacket.st24_data[PACKET_LENGTH_REALSENSE_OBSTACLE_DATA_INPUT] = st24_common_crc8((uint8_t *) & (_txpacket.length),
			_txpacket.length);
	int ret = ::write(_uart_fd, (uint8_t *)&_txpacket, (_txpacket.length + 3));

	if (ret < 0 && _realsense_is_present) {
		PX4_ERR("failed to write send data to realsense (%i)", ret);
	}
}

void
REALSENSE::_send_shutdown_command()
{
	ChtCommand tx = {};
	tx.command = CommandTypes::CMD_SHUTDOWN;
	ReceiverFcPacket _txpacket = {};
	// pack packet and send
	_txpacket.header1 = ST24_STX1;
	_txpacket.header2 = ST24_STX2;
	_txpacket.length = PACKET_LENGTH_REALSNESE_CHT_COMMAND + 2;
	_txpacket.type = PacketId::Command;
	memcpy(_txpacket.st24_data, (const uint8_t *)&tx, PACKET_LENGTH_REALSNESE_CHT_COMMAND);
	_txpacket.st24_data[PACKET_LENGTH_REALSNESE_CHT_COMMAND] = st24_common_crc8((uint8_t *) & (_txpacket.length),
			_txpacket.length);
	int ret = ::write(_uart_fd, (uint8_t *)&_txpacket, (_txpacket.length + 3));

	if (ret < 0 && _realsense_is_present) {
		PX4_ERR("failed to write shutdown command to realsense (%i)", ret);
	}
}


/********************handle yuneec protocol*******************
* frame sync for YP
* param in byte : new char received
* param out returnPacket :synced packet
* return 0:success 1:syncing 3:unsync 4:CRC err
*/
int
REALSENSE::_realsense_parse_frame_sync(uint8_t index, uint8_t byte, ReceiverFcPacket *returnPacket)
{
	static volatile ST24_DECODE_STATE decodeState[MAX_YPRC_PORT_NUM] = { ST24_DECODE_STATE_UNSYNCED };
	static volatile uint8_t rxLen[MAX_YPRC_PORT_NUM] = {0};
	static ReceiverFcPacket rxpacket[MAX_YPRC_PORT_NUM] = {0};

	int ret = SyncStatus::SYNC_UNSYNC;
	uint8_t data_index = 0;

	if (index >= MAX_YPRC_PORT_NUM) {
		return ret;
	}

	switch (decodeState[index]) {
	case ST24_DECODE_STATE_UNSYNCED:
		if (byte == ST24_STX1) {
			decodeState[index] = ST24_DECODE_STATE_GOT_STX1;

		} else {
			ret = SyncStatus::SYNC_UNSYNC;
		}

		break;

	case ST24_DECODE_STATE_GOT_STX1:
		if (byte == ST24_STX2) {
			decodeState[index] = ST24_DECODE_STATE_GOT_STX2;

		} else {
			decodeState[index] = ST24_DECODE_STATE_UNSYNCED;
		}

		ret = SyncStatus::SYNC_RUNNING;
		break;

	case ST24_DECODE_STATE_GOT_STX2:

		/* ensure no data overflow failure or hack is possible */
		if (byte <= ST24_LENGTH_VALUE) {
			rxpacket[index].length = byte;
			rxLen[index] = 0;
			decodeState[index] = ST24_DECODE_STATE_GOT_LEN;

		} else {
			decodeState[index] = ST24_DECODE_STATE_UNSYNCED;
		}

		ret = SyncStatus::SYNC_RUNNING;
		break;

	case ST24_DECODE_STATE_GOT_LEN:
		rxpacket[index].type = byte;
		++rxLen[index];
		decodeState[index] = ST24_DECODE_STATE_GOT_TYPE;
		ret = SyncStatus::SYNC_RUNNING;
		break;

	case ST24_DECODE_STATE_GOT_TYPE:
		data_index = rxLen[index] - 1;

		if (data_index < ST24_DATA_LEN_MAX) {
			rxpacket[index].st24_data[data_index] = byte;

		} else {
			decodeState[index] = ST24_DECODE_STATE_UNSYNCED;
			ret = SyncStatus::SYNC_UNSYNC;
		}

		++rxLen[index];

		if (rxLen[index] == (rxpacket[index].length - 1)) {
			decodeState[index] = ST24_DECODE_STATE_GOT_DATA;
		}

		ret = SyncStatus::SYNC_RUNNING;
		break;

	case ST24_DECODE_STATE_GOT_DATA:
		rxpacket[index].crc8 = byte;
		++rxLen[index];

		if (st24_common_crc8((uint8_t *) & (rxpacket[index].length), rxLen[index]) == rxpacket[index].crc8) {
			/* get full frame */
			ret = SyncStatus::SYNC_SUCCESS;
			memcpy(returnPacket, &rxpacket[index], sizeof(ReceiverFcPacket));

		} else {
			/* CRC failed */
			ret = SyncStatus::SYNC_CRC_ERR;
		}

		decodeState[index] = ST24_DECODE_STATE_UNSYNCED;
		break;
	}

	return ret;
}


void
REALSENSE::_read_obstacle_avoidance_data()
{
	uint8_t _rcs_buf[REALSENSE_BUFFER_SIZE] = {};
	static ReceiverFcPacket _rxpacket_realsense[MAX_YPRC_PORT_NUM] = { 0 };
	// read all available data from the serial realsense through UART4
	int newBytes = ::read(_uart_fd, &_rcs_buf[0], REALSENSE_BUFFER_SIZE);

	for (int i = 0; i < newBytes; i++) {
		if (_realsense_parse_frame_sync(1, _rcs_buf[i], _rxpacket_realsense) == SyncStatus::SYNC_SUCCESS) {

			switch (_rxpacket_realsense->type) {

			case PacketId::ObstacleAvoidanceOutput: {//Type ID is 0x11

					ObstacleAvoidanceOutput packet_data_output;
					memcpy((char *)&packet_data_output, _YP_PACKET_DATA(_rxpacket_realsense), PACKET_LENGTH_REALSENSE_OBSTACLE_DATA_OUTPUT);

					struct obstacle_avoidance_s obstacle_avoidance = {};

					obstacle_avoidance.timestamp = hrt_absolute_time();

					obstacle_avoidance.point_1[0] = NAN; // position sp
					obstacle_avoidance.point_1[1] = NAN;
					obstacle_avoidance.point_1[2] = NAN;
					obstacle_avoidance.point_1[3] = packet_data_output.obstacleAvoidanceSpeed.y;	 //realsense E  -> UAV N
					obstacle_avoidance.point_1[4] = packet_data_output.obstacleAvoidanceSpeed.x;  //realsense N  -> UAV E
					obstacle_avoidance.point_1[5] = -packet_data_output.obstacleAvoidanceSpeed.z; //realsense U  -> UAV D
					obstacle_avoidance.point_1[6] = NAN; // acceleration sp
					obstacle_avoidance.point_1[7] = NAN;
					obstacle_avoidance.point_1[8] = NAN;
					obstacle_avoidance.point_1[9] = NAN; // yaw
					obstacle_avoidance.point_1[10] = math::constrain(packet_data_output.obstacleAvoidanceYawSpeed, -M_PI_F,
									 M_PI_F); //yawspeed

					obstacle_avoidance.point_valid[0] = 1;
					obstacle_avoidance.point_valid[1] = 0;
					obstacle_avoidance.point_valid[2] = 0;
					obstacle_avoidance.point_valid[3] = 0;
					obstacle_avoidance.point_valid[4] = 0;

					obstacle_avoidance.field_of_view[0] = 180;
					obstacle_avoidance.field_of_view[1] = 0;
					obstacle_avoidance.field_of_view[2] = -1;
					obstacle_avoidance.field_of_view[3] = 15;

					/* Don't use realsense if we are above home within acceptance radius */
					const float dist_to_home_xy =  matrix::Vector2f(_home_pos.x - _local_pos.x, _home_pos.y - _local_pos.y).length();
					const bool close_to_home = dist_to_home_xy < _nav_rad;

					const bool use_realsense = (_manual.obsavoid_switch == manual_control_setpoint_s::SWITCH_POS_ON)
								   && (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL)
								   && !close_to_home;

					if (use_realsense) {
						if (_obstacle_avoidance_pub == nullptr) {
							_obstacle_avoidance_pub = orb_advertise(ORB_ID(obstacle_avoidance), &obstacle_avoidance);

						} else {
							orb_publish(ORB_ID(obstacle_avoidance), _obstacle_avoidance_pub, &obstacle_avoidance);
						}

					}

					break;
				}

			case PacketId::OpticalFlowDataOutput: {//Type ID is 0x12

					// check if we get optical flow msgs and a RealSense module is present (stop driver otherwise)
					if (!_realsense_is_present) {
						PX4_WARN("Found RealSense module");
						_realsense_is_present = true;
					}

					OpticalFlowDataOutput packet_optical;
					memcpy((char *)&packet_optical, _YP_PACKET_DATA(_rxpacket_realsense), PACKET_LENGTH_REALSENSE_OPTFLOW);
					// integrate gyro values
					vector3f gyro_integrated{};
					vector3f_timestamped gyro;
					uint32_t time_now_us = hrt_absolute_time();
					static uint32_t last_imu_time = 0;
					float dt = 0.0f;

					// integrate and account for flow delay
					do {
						if (!_rb_gyro->get(&gyro)) {
							break;
						}

						dt = float(gyro.time_usec - last_imu_time) / 1000000.0f;
						last_imu_time = gyro.time_usec;

						if (last_imu_time == 0) {
							PX4_WARN("RealSense Flow: gyro time stamp equals 0");

						} else {
							gyro_integrated.x += gyro.x * dt;
							gyro_integrated.y += gyro.y * dt;
							gyro_integrated.z += gyro.z * dt;
						}
					} while (gyro.time_usec < (time_now_us - DELAY_FLOW_US));

					struct optical_flow_s 	optical_flow;

					// swap directions to match PX4Flow and SENS_FLOW_ROT 6 (270 degrees)
					optical_flow.pixel_flow_x_integral = packet_optical.integration_y;
					optical_flow.pixel_flow_y_integral = -packet_optical.integration_x;
					optical_flow.gyro_x_rate_integral = gyro_integrated.x;
					optical_flow.gyro_y_rate_integral = gyro_integrated.y;
					optical_flow.gyro_z_rate_integral = gyro_integrated.z;
					optical_flow.ground_distance_m = packet_optical.distance;
					optical_flow.integration_timespan = packet_optical.integration_time_us;
					optical_flow.time_since_last_sonar_update = packet_optical.time_delta_distance_us;
					optical_flow.frame_count_since_last_readout = 0; //? TODO: do not know what this msg is.
					optical_flow.quality = packet_optical.quality;
					optical_flow.sensor_id = 0;
					optical_flow.timestamp = time_now_us;

					if (_optical_flow_pub == nullptr) {
						_optical_flow_pub = orb_advertise(ORB_ID(optical_flow), &optical_flow);

					} else {
						orb_publish(ORB_ID(optical_flow), _optical_flow_pub, &optical_flow);
					}

					//publish the distance we get to the distance_sensor topic
					struct distance_sensor_s report = {};

					report.timestamp = time_now_us;
					report.min_distance = 0.4f; // [m] according to camera team
					report.max_distance = 4.5f;  // [m] according to camera team
					report.current_distance = packet_optical.distance; // [m]
					report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND;
					report.orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING; // downward facing

					// There are 2 sonars -> use multi
					int _orb_class_instance = -1;

					if (_distance_sensor_pub != nullptr) {
						orb_publish(ORB_ID(distance_sensor), _distance_sensor_pub, &report);

					} else {
						_distance_sensor_pub = orb_advertise_multi(ORB_ID(distance_sensor), &report,
								       &_orb_class_instance, ORB_PRIO_LOW);
					}

					break;
				}

			case PacketId::ObstacleDistanceInfo: {

					ObstacleDistanceInfo packet_distance_info;
					memcpy((char *)&packet_distance_info, _YP_PACKET_DATA(_rxpacket_realsense), PACKET_LENGTH_REALSENSE_DISTANCE_INFO);

					struct realsense_distance_info_s realsense_distance_info;
					memcpy(realsense_distance_info.distances, packet_distance_info.front_distance,
					       sizeof(packet_distance_info.front_distance));
					realsense_distance_info.timestamp = hrt_absolute_time();

					if (_realsense_distance_info_pub == nullptr) {
						_realsense_distance_info_pub = orb_advertise(ORB_ID(realsense_distance_info), &realsense_distance_info);

					} else {
						orb_publish(ORB_ID(realsense_distance_info), _realsense_distance_info_pub, &realsense_distance_info);
					}


					break;
				}

			case PacketId::ObstacleDistance360: {

					ObstacleDistance360 packet_distance_360;
					memcpy((char *)&packet_distance_360, _YP_PACKET_DATA(_rxpacket_realsense), PACKET_LENGTH_REALSENSE_DISTANCE_360);

					struct obstacle_distance_s obstacle_distance;
					obstacle_distance.timestamp = hrt_absolute_time();
					obstacle_distance.increment = 5;
					obstacle_distance.min_distance = 20;
					obstacle_distance.max_distance = MAX_DIST;

					for (int k = 0; k < ObstacleDistance360::nAzimuthBlocks; ++k) {
						obstacle_distance.distances[k] = packet_distance_360.distances[k] / 10; // covert from 1u=10cm to 1u=1cm

						/* change convention for no obstacle in front of the sensor */
						if (packet_distance_360.distances[k] == 0) {
							obstacle_distance.distances[k] = MAX_DIST + 1;
						}
					}

					if (_obstacle_distance_pub == nullptr) {
						_obstacle_distance_pub = orb_advertise(ORB_ID(obstacle_distance), &obstacle_distance);

					} else {
						orb_publish(ORB_ID(obstacle_distance), _obstacle_distance_pub, &obstacle_distance);
					}

					break;

				}

			default:
				break;
			}
		}
	}
}

void
REALSENSE::_init_realsense() 							 // init - initialise the sensor
{
	if (!_initialized) {
		_vehicle_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));
		_avoidance_input_sub = orb_subscribe(ORB_ID(realsense_avoidance_setpoint_input));
		_sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
		_vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));

		// because we have 2 distance sensors instances (forward- and downward-facing)
		for (unsigned i = 0; i < DISTANCE_SENSOR_INSTANCES; i++) {
			_distance_sensor_subs[i] = orb_subscribe_multi(ORB_ID(distance_sensor), i);
		}

		_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
		_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
		_home_pos_sub = orb_subscribe(ORB_ID(home_position));

		_initialized = true;
	}
}

void
REALSENSE::update()     					 // update - check input and send out data
{

	if (_uart_fd < 0) {
		// We need to initialise the UART here because the file descriptor needs to be initialized in the correct task context.
		int ret = _initialise_uart(_device_realsense);

		if (ret < 0) {
			PX4_ERR("initialise uart failed");
		}
	}

	if (_read_write_count >= 100) {
		_send_heartbeat(); //1 hz
		_read_write_count = 0;
	}

	_send_obstacle_avoidance_data(); // 100 hz
	_read_write_count ++;
	_read_obstacle_avoidance_data(); // 100 hz

}

void
REALSENSE::_cycle_realsense()
{
	if (!_initialized) {
		_init_realsense();
		_taskIsRunning = true;
		_init_time = hrt_absolute_time();
	}

	poll_subscriptions();
	update();


	if (!_taskShouldExit) {
		// Schedule next cycle.
		work_queue(HPWORK, &_work, (worker_t)&REALSENSE::_cycle_trampoline, this,
			   USEC2TICK(CONVERSION_INTERVAL_REALSENSE));

	} else {
		_taskIsRunning = false;
		_send_shutdown_command();
	}

	// shut down driver if we don't get any msgs after 1min
	if (!_realsense_is_present && (hrt_absolute_time() - _init_time) > TIME_WAIT_SHUTDOWN_US) {
		delete realsense::realsense_drv_task;
		realsense::realsense_drv_task = nullptr;
		PX4_WARN("No RealSense present - stopping driver");
	}
}


int
REALSENSE::_initialise_uart(const char *device)
{
	// open uart
	_uart_fd = ::open(device, O_RDWR | O_NONBLOCK | O_NOCTTY);

	if (_uart_fd < 0) {
		PX4_ERR("failed to open uart device!");
		return PX4_ERROR;
	}

	struct termios uart_config;

	int termios_state;

	/* fill the struct for the new configuration */
	if (tcgetattr(_uart_fd, &uart_config)) {
		PX4_ERR("error getting uart configuration");
		return PX4_ERROR;
	}

	/* properly configure the terminal (see also https://en.wikibooks.org/wiki/Serial_Programming/termios ) */

	// clear ONLCR flag (which appends a CR for every LF)
	uart_config.c_oflag &= ~ONLCR;

	// clear CRTSCTS flag (which enables flow-control)
	// This fixes the issue where we are no longer able to write to the realsense.
	uart_config.c_cflag &= ~CRTSCTS;

	/* set baud rate */
	if ((termios_state = cfsetispeed(&uart_config, B460800)) < 0) {
		PX4_ERR("ERR: %d (cfsetispeed)", termios_state);
		::close(_uart_fd);
		return PX4_ERROR;
	}

	if ((termios_state = cfsetospeed(&uart_config, B460800)) < 0) {
		PX4_ERR("ERR: %d (cfsetospeed)", termios_state);
		::close(_uart_fd);
		return PX4_ERROR;
	}

	if ((termios_state = tcsetattr(_uart_fd, TCSANOW, &uart_config)) < 0) {
		PX4_ERR("ERR: %d (tcsetattr)", termios_state);
		::close(_uart_fd);
		return PX4_ERROR;
	}

	return PX4_OK;
}

int
REALSENSE::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_measure_ticks = 0;
				return OK;

			/* external signalling (DRDY) not supported */
			case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
			case 0:
				return -EINVAL;

			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(REALSENSE_CONVERSION_INTERVAL);

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();

					}

					return OK;
				}

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* convert hz to tick interval via microseconds */
					int ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(REALSENSE_CONVERSION_INTERVAL)) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_measure_ticks == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}

		return (1000 / _measure_ticks);

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			irqstate_t flags = px4_enter_critical_section();

			px4_leave_critical_section(flags);

			return OK;
		}

	case SENSORIOCRESET:
		/* XXX implement this */
		return -EINVAL;

	default:
		/* give it to the superclass */
		return CDev::ioctl(filp, cmd, arg);
	}

	return 1;
}

static void realsense::realsense_stop()
{
	if (realsense_drv_task == nullptr) {
		PX4_WARN("not running");
		return;
	}

	realsense_drv_task->stop();


	// Wait for task to die
	int i = 0;

	do {
		// wait 2000ms at a time
		usleep(2000000);

	} while (realsense_drv_task->is_running() && ++i < 50);


	delete realsense_drv_task;
	realsense_drv_task = nullptr;
	PX4_WARN("realsense has been stopped");

}

void realsense::usage()
{
	PX4_INFO("usage: realsense start -d /dev/ttyS3");
	PX4_INFO("       realsense stop");
	PX4_INFO("       realsense status");
}

int realsense::realsense_main(int argc, char *argv[])
{
	const char *device = nullptr;
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	char *verb = nullptr;

	if (argc >= 2) {
		verb = argv[1];
	}

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device = myoptarg;
			strncpy(_device, device, strlen(device));
			break;
		}
	}

	// Start/load the driver.
	if (!strcmp(verb, "start")) {
		if (realsense_drv_task != nullptr) {
			PX4_WARN("realsense already running");
			return 1;
		}

		// Check on required arguments
		if (device == nullptr || strlen(device) == 0) {
			usage();
			return 1;
		}

		if (realsense_drv_task == nullptr) {
			realsense_drv_task = new REALSENSE(_device);

			if (realsense_drv_task == nullptr) {
				PX4_ERR("failed to start realsense.");
				return -1;
			}
		}

		realsense_drv_task->init_dev();
		realsense_drv_task->start();

	} else if (!strcmp(verb, "stop")) {
		if (realsense_drv_task == nullptr) {
			PX4_WARN("realsense is not running");
			return 1;
		}

		realsense_stop();

	} else if (!strcmp(verb, "status")) {
		PX4_WARN("realsense is %s", realsense_drv_task != nullptr ? "running" : "not running");
		return 0;

	} else {
		usage();
		return 1;
	}

	return 0;
}
