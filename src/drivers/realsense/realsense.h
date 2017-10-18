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

#ifndef REALSENSE_H_
#define REALSENSE_H_

#include <stdint.h>
#define FC_CHT_COMM_VERSION_MAJOR 1 // Increase for API breaḱage, i.e. code needs to be rewritten.
#define FC_CHT_COMM_VERSION_MINOR 0 // Increase for ABI breakage, i.e. code needs to be recompiled.
#define FC_CHT_COMM_VERSION_PATCH 3 // Increase for patches, i.e. no recompilation etc. necessary. Also applies for new messages added.

/*
 * == Yuneec message format ==
 * uint8_t delimiter0 = 0x55;
 * uint8_t delimiter1 = 0x55;
 * uint8_t length;  // length includes type, data, and crc --> sizeof(type)+sizeof(data[payload_len])+sizeof(crc8)
 * uint8_t type;
 * uint8_t data; // up to 128 bytes
 * uint8_t crc8; // crc8 checksum, computed over fields length, type and data
 */

/// Packet id definitions.
struct PacketId {
	enum {
		Heartbeat = 0x0f,
		ObstacleAvoidanceInput = 0x10,
		ObstacleAvoidanceOutput = 0x11,
		OpticalFlowDataOutput = 0x12,
		ObstacleDistanceInfo = 0x13,
		ReservedDoNotUse = 0x14,
		ObstacleDistance360 = 0x15,
		Command = 0x16
	};
};

/// Heartbeat packet to be sent @ ~1 Hz. Definition must NOT change!
typedef struct __attribute__((packed))
{
	uint8_t versionMajor;
	uint8_t versionMinor;
	uint32_t componentType; ///< Indicates which component sent the heartbeat. See ComponentType.
} Heartbeat;

/// Component types.
struct ComponentType {
	enum {
		FlightController = 0x01,
		ObstacleAvoidance = 0x02,
		OpticalFlow = 0x03,
	};
};

struct SyncStatus {
	enum {
		SYNC_SUCCESS = 0,
		SYNC_RUNNING = 1,
		SYNC_UNSYNC = 2,
		SYNC_CRC_ERR = 3
	};
};

struct CommandTypes {
	enum {
		CMD_SHUTDOWN = 0x01
	};
};

typedef union __attribute__((packed))
{
	uint32_t command;
} ChtCommand;

/// Single precision floating point 3-element vector.
typedef union __attribute__((packed))
{
	struct {
		float x, y, z;
	};
	float elem[3];
} vector3f;

/// Single precision floating point 3-element vector with time stamp.
typedef union __attribute__((packed))
{
	struct {
		float x, y, z;
		uint32_t time_usec;
	};
	float elem[4];
} vector3f_timestamped;

/// Single precision floating point 4-element vector.
typedef union __attribute__((packed))
{
	struct {
		float x, y, z, c;
	};
	float data[4];
} vector4f;

/// Single precision floating point quaternion.
typedef union __attribute__((packed))
{
	struct {
		float q1, q2, q3, q4;
	};
	struct {
		float w, x, y, z;
	};
	float elem[4];
} quaternion;

/// Obstacle avoidance data structure sent from the flight controller to the CHT.
typedef struct __attribute__((packed))
{
	/// Desired speed in [m/s] in ENU frame.
	vector3f desiredSpeed;

	float unused1;

	/**
	 * \brief Attitude state information in NED frame.
	 * The quaternion is in wxyz format. Active transform of the body frame w.r.t. inertial frame, or in other words, this transforms coordinates expressed in body frame to NED frame (passive).
	 */
	quaternion attitude;

	/// Position state information in [m] in local ENU frame.
	vector3f position;

	/// Linear velocity state information in [m/s] in ENU frame.
	vector3f velocity;

	vector3f unused2;

	vector3f unused3;

	/// Front sonar measurement in [mm]. Zero if invalid. Not used.
	uint16_t sonarFrontMm;

	/// Flight control timestamp in [ms]
	uint32_t timestamp;

	/// Flags, see definitions in ObstacleAvoidanceInputFlags
	uint32_t flags;
} ObstacleAvoidanceInput;

/// Flags used in ObstacleAvoidanceInput.flags
struct ObstacleAvoidanceInputFlags {
	enum {
		OK_TO_FLY = 0x01, ///< Set if GPS has lock and state estimation provides reliable data.
		USE_SMALL_VEHICLE_SIZE = 0x08 ///< shrinks safety distance around vehicle. Was used in Typhoon H sense & stop functionality during "Angle Mode"
	};
};

/// Obstacle avoidance data structure sent from the CHT to the flight controller.
typedef struct __attribute__((packed))
{
	/// New desired speed from the obstacle avoidance algorithm. In [m/s] and expressed in the ENU frame.
	vector3f obstacleAvoidanceSpeed;

	/// New desired yaw speed from the obstacle avoidance algorithm. In [rad/s] and expressed in the NED frame.
	float obstacleAvoidanceYawSpeed;

	/// Timestamp of the CHT board in [ms]
	uint32_t timestamp;

	/// Flags, see definitions in ObstacleAvoidanceOutputFlags
	uint32_t flags;
} ObstacleAvoidanceOutput;

/// Flags used in ObstacleAvoidanceOutputflags.flags
struct ObstacleAvoidanceOutputFlags {
	enum {
		/// Camera started and producing frames.
		CAMERA_RUNNING = 0x01,
	};
};


// defined for optical flow, but not used in Obstacle Avoidance
typedef struct __attribute((packed))
{
	uint64_t   shutter_time;            ///< time elapse between the current optical flow calculation and message send out
	uint32_t   integration_time_us;     ///< time elapse between two message sending time
	float      integration_x;           ///< flow in radians around X axis (RH rotation definition)
	float      integration_y;           ///< flow in radians around Y axis (RH ratation definition)
	float      gyro_x;                  ///< RH rotation around X axis in rad
	float      gyro_y;                  ///< RH rotation around Y axis in rad
	float      gyro_z;                  ///< RH rotation around Z axis in rad
	uint8_t    quality;                 ///< optical flow quality
	uint32_t   time_delta_distance_us;  ///< time elapse between the distance measurement and message send out
	float      distance;                ///< distance to the center of the flow field in meters
} OpticalFlowDataOutput;              ///< send from the CHT board to the autopilot. Packgetype 0x12

typedef struct __attribute((packed))
{
	/**
	 * Distance of obstacles in front of the camera. Units in 10cm, starting -30 deg from the left, in 5deg increments to the right.
	 * Expressed in camera coordinates. It is the slice of zero elevation Angle seen from the camera. A value of 0 means no obstacle.
	 */
	uint8_t   front_distance[12];
} ObstacleDistanceInfo; // send from the CHT board to the autopilot with 5Hz. packet-type 0x13

/// 360 deg distance info sent from the CHT to the flight controller. Packet ID 0x15
typedef struct __attribute__((packed))
{
	enum {nAzimuthBlocks = 72, nElevationBlocks = 1};
	/// Timestamp of the CHT board in [ms]
	uint32_t timestampMs;
	/**
	 * Distance between vehicle and obstacles 360 deg around the vehicle. Elements contain the minimal distance of an obstacle
	 * within the respective sector. Coordinates are expressed in the world coordinate system. Zero azimuth is north, increasing clockwise.
	 * One unit is 10 cm.
	 * Data layout: starting from the bottom layer, increasing azimuth cw.
	 */
	uint8_t distances[nAzimuthBlocks * nElevationBlocks];
} ObstacleDistance360;

#endif /* REALSENSE_H_ */