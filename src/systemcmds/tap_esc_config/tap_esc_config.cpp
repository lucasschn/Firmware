/****************************************************************************
 *
 *   Copyright (c) 2012-2015, 2017 PX4 Development Team. All rights reserved.
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

/**
 * @file tap_esc_config.cpp
 *
 * ESC configurator for tap
 */

#include <unistd.h>

#include <px4_getopt.h>
#include <px4_module.h>
#include <px4_log.h>
#include <drivers/tap_esc/tap_esc_uploader.h>
#include <drivers/tap_esc/drv_tap_esc.h>  // ESC_UART_BUF
#include <drivers/tap_esc/tap_esc_common.h>

/**
 *  Print command line usage
 */
static void	print_usage(const char *reason = nullptr);

/**
 *  Uploads a firmware binary to all connected ESCs
 *  @param fw_paths Firmware paths to search for the binary file. Must be terminated with a nullptr entry.
 *  @param device Unix path of UART device where ESCs are connected to
 *  @param num_escs Number of ESCs that are currently connected to the board
 *  @return 0 on success, 1 on error instantiating the uploader and othwerwise errno (linux man)
 */
static int upload_firmware(const char *fw_paths[], const char *device, uint8_t num_escs);

/**
 *  Check CRC of ESC's currently loaded firmware. If one or more are faulty, firmware will be re-uploaded
 *  @param fw_paths Firmware paths to search for the binary file. Must be terminated with a nullptr entry.
 *  @param device Unix path of UART device where ESCs are connected to
 *  @param num_escs Number of ESCs that are currently connected to the board
 *  @return 0 on success, -1 on error
 */
static int check_crc(const char *fw_paths[], const char *device, uint8_t num_escs);

/**
 *  Specify the ESC ID of connected ESCs by manually touching and turning the
 *  motors.
 *  @param device Unix path of UART device where ESCs are connected to
 *  @param id ID that should be assigned to an ESC, starting from 0
 *  @param num_escs Number of ESCs that are currently connected to the board
 *  @return 0 on success, -1 on error
 */
static int configure_esc_id(const char *device, int8_t id, uint8_t num_escs);

extern "C" {
	__EXPORT int tap_esc_config_main(int argc, char *argv[]);
}

static void print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Command-line tool to configure, flash and check ESCs.

To use it make sure no esc driver is running and using the same serial device'.

### Examples
Flash firmware to board with six ESC channels connected to /dev/ttyS2
$ tap_esc_config upload /dev/ttyS2 -n 6

Configure ID of all ESCs of a quadrotor
tap_esc_config identify -d /dev/ttyS4 -n 4

Check firmware CRC of al ESCs and re-flash on mismatch
tap_esc_config checkcrc -d /dev/ttyS2 -n 6

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("tap_esc_config", "command");

	PRINT_MODULE_USAGE_COMMAND_DESCR("identify", "Configure ESC id");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<device>", "Device used to talk to ESCs", false);
	PRINT_MODULE_USAGE_PARAM_INT('n', 0, 4, 8, "Number of ESCs", false);
	PRINT_MODULE_USAGE_PARAM_INT('t', 0, 0, 8, "Target ESC of which to configure ID. If not specified all ESCs will be configured", true);

	PRINT_MODULE_USAGE_COMMAND_DESCR("checkcrc", "Check CRC of firmware and re-flash on mismatch");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<device>", "Device used to talk to ESCs", false);
	PRINT_MODULE_USAGE_PARAM_INT('n', 0, 6, 8, "Number of ESCs", false);
	PRINT_MODULE_USAGE_PARAM_STRING('f', nullptr, "<file>", "Firmware used to flash", true);


	PRINT_MODULE_USAGE_COMMAND_DESCR("upload", "Upload new firmware to ESCs");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<device>", "Device used to talk to ESCs", false);
	PRINT_MODULE_USAGE_PARAM_INT('n', 0, 6, 8, "Number of ESCs", false);
	PRINT_MODULE_USAGE_PARAM_STRING('f', nullptr, "<file>", "Firmware used to check CRC against", true);
}

static int upload_firmware(const char * fw_paths[], const char * device, uint8_t num_escs)
{
	// TODO: Make use of device!
	TAP_ESC_UPLOADER *uploader = nullptr;
	uploader = new TAP_ESC_UPLOADER(num_escs);

	if (uploader==nullptr)
	{
		PX4_ERR("failed to initialize firmware uploader");
		delete uploader;
		return 1;
	}

	PX4_INFO("flashing firmware %s", fw_paths[0]);
	int ret = uploader->upload(&fw_paths[0]);
	delete uploader;

	switch (ret) {
	case OK:
		PX4_INFO("success! Uploaded ESC firmware");
		break;

	case -ENOENT:
		PX4_ERR("firmware file not found");

	case -EEXIST:
	case -EIO:
		PX4_ERR("error updating ESCs - check that bootloader mode is enabled");

	case -EINVAL:
		PX4_ERR("firmware verification failed - try again");

	case -ETIMEDOUT:
		PX4_ERR("timed out waiting for bootloader - power-cycle and try again");

	default:
		PX4_ERR("unexpected error %d", ret);
	}

	return ret;
}

static int check_crc(const char * fw_paths[], const char * device, uint8_t num_escs)
{
	TAP_ESC_UPLOADER *uploader = nullptr;
	uploader = new TAP_ESC_UPLOADER(num_escs);

	if (uploader==nullptr)
	{
		PX4_ERR("failed to initialize firmware uploader");
		delete uploader;
		return -1;
	}

	int ret = uploader->checkcrc(&fw_paths[0]);
	delete uploader;

	if (ret != OK) {
		PX4_ERR("TAP_ESC firmware auto check crc and upload fail error %d", ret);
	}

	return ret;
}

static int configure_esc_id(const char * device, int8_t id, uint8_t num_escs)
{
	int uart_fd;
	int ret = tap_esc_common::initialise_uart(device, uart_fd);
	if (ret)
	{
		PX4_ERR("failed to open UART device %s (error code %d)", device, uart_fd);
		return ret;
	}

	// Send basic config to ESCs
	usleep(500000);
	EscPacket packet = {PACKET_HEAD, sizeof(ConfigInfoBasicRequest), ESCBUS_MSG_ID_CONFIG_BASIC};
	ConfigInfoBasicRequest   &config = packet.d.reqConfigInfoBasic;
	memset(&config, 0, sizeof(ConfigInfoBasicRequest));
	config.maxChannelInUse = num_escs;
	config.controlMode = BOARD_TAP_ESC_MODE;
	config.maxChannelValue = RPMMAX;
	config.minChannelValue = RPMMIN;

	ret = tap_esc_common::send_packet(uart_fd, packet, 0);
	if (ret < 0) {
		PX4_ERR("Error sending basic config packet to ESCs");
		return
		 ret;
	}

	usleep(30000);

	// To Unlock the ESC from the Power up state we need to issue 10
	// ESCBUS_MSG_ID_RUN request with all the values 0;
	EscPacket unlock_packet = {PACKET_HEAD, num_escs, ESCBUS_MSG_ID_RUN};
	unlock_packet.len *= sizeof(unlock_packet.d.reqRun.rpm_flags[0]);
	memset(unlock_packet.d.bytes, 0, sizeof(packet.d.bytes));

	int unlock_times = 10;

	while (unlock_times--) {

		ret = tap_esc_common::send_packet(uart_fd, unlock_packet, -1);
		if (ret < 0) {
			PX4_ERR("failed sending the ESC basic configuration packet");
			return ret;
		}

		// Min Packet to Packet time is 1 Ms so use 2
		usleep(2000);
	}

	// Configure specific ESC or all of them
	uint8_t first_esc, last_esc;
	if (id<0){
		PX4_INFO("starting configuration of all ESCs");
		first_esc = 0;
		last_esc = num_escs-1;
	}else{
		first_esc = last_esc = id;
	}

	for (uint8_t i_esc = first_esc; i_esc <= last_esc; i_esc++)
	{
		EscPacket id_config = {PACKET_HEAD, sizeof(EscbusConfigidPacket), ESCBUS_MSG_ID_DO_CMD};
		id_config.d.configidPacket.id_mask = PACKET_ID_MASK;
		id_config.d.configidPacket.child_cmd = DO_ID_ASSIGNMENT;
		id_config.d.configidPacket.id = i_esc;
		tap_esc_common::send_packet(uart_fd, id_config, -1);
		PX4_INFO("touch and turn motor number %u now", i_esc);

		// Give UART time to write
		usleep(10000);

		// Wait for response
		ESC_UART_BUF uart_buf = {};
		EscPacket response;
		bool valid = false;

		while (true) {
			tap_esc_common::read_data_from_uart(uart_fd, &uart_buf);
			ret = tap_esc_common::parse_tap_esc_feedback(&uart_buf, &response);
			if (ret==0) {
				valid = (response.msg_id == ESCBUS_MSG_ID_ASSIGNED_ID
					 && response.d.rspAssignedId.escID == i_esc);

				if (valid) {
					PX4_INFO("success!");
				}else{
					PX4_WARN("failed: ESC %u configuration invalid", i_esc);
					break;
				}

				break;
			}

			usleep(100000);
		}
		if (valid)
		{
			usleep(500000); // Give time for ESC confirmation beep
		}
	}

	tap_esc_common::deinitialise_uart(uart_fd);
	return 0;
}

int tap_esc_config_main(int argc, char *argv[]) {
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	const char *device = nullptr;
	uint8_t num_escs = 0;
	int8_t id_config_num = -1;
	const char *firmware_paths[3] = TAP_ESC_FW_SEARCH_PATHS;

	while ((ch = px4_getopt(argc, argv, "d:n:t:f:", &myoptind, &myoptarg)) != EOF) {
		if(myoptarg==nullptr)
		{
			continue;
		}
		switch (ch) {
		case 'd':
			device = myoptarg;
			break;

		case 'n':
			num_escs = atoi(myoptarg);
			break;

		case 't':
			id_config_num = atoi(myoptarg);
			break;

		case 'f':
			firmware_paths[0] = myoptarg;
			firmware_paths[1] = nullptr;
			break;
		}
	}

	if (myoptind >= argc) {
		print_usage();
		return 1;
	}

	if (device == nullptr || strlen(device) == 0){
		print_usage("device not specified");
		return 1;
	}
	if (num_escs==0)
	{
		print_usage("number of ESCs must be positive");
		return 1;
	}


	if (!strcmp(argv[myoptind], "checkcrc")) {
		return check_crc(&firmware_paths[0], device, num_escs);

	} else if (!strcmp(argv[myoptind], "identify")) {
		if (id_config_num>=num_escs)
		{
			print_usage("ID mut be smaller than the number of channels");
			return 1;
		}
		return configure_esc_id(device, id_config_num, num_escs);

	} else if (!strcmp(argv[myoptind], "upload")) {
		return upload_firmware(&firmware_paths[0], device, num_escs);

	} else {
		print_usage("Command not recognised");
		return 1;
	}

	return 0;
}
