/* @file UnicoreComm protocol definitions */

#ifndef UNICORE_H_
#define UNICORE_H_

#include "../devices/src/gps_helper.h"
#include "../definitions.h"

#define UNICORE_CONFIG_PPS_EN           "CONFIG PPS ENABLE GPS POSITIVE 500000 1000 0 0\r\n"
#define UNICORE_CONFIG_GPGSA            "LOG COM2 GPGSA ONTIME 0.2\r\n"
#define UNICORE_CONFIG_GPZDA            "LOG COM2 GPZDA ONTIME 0.2\r\n"
#define UNICORE_CONFIG_BESTPOSA         "LOG COM2 BESTPOSA ONTIME 0.2\r\n"
#define UNICORE_CONFIG_BESTVELA         "LOG COM2 BESTVELA ONTIME 0.2\r\n"
#define UNICORE_CONFIG_GPGGA            "LOG COM2 GPGGA ONTIME 0.2\r\n"
#define UNICORE_CONFIG_GPHDT            "LOG COM2 GPHDT ONTIME 0.2\r\n"
#define UNICORE_CONFIG_EVENT_EN         "CONFIG EVENT ENABLE POSITIVE 10\r\n"
#define UNICORE_CONFIG_EVENT_LOG        "LOG COM1 EVENTALLB ONCHANGED\r\n"
#define UNICORE_CONFIG_EVENT_RX         "LOG EVENTALLA ONCHANGED\r\n"

#define UNICORE_RESPONSE_TAIL           ",response: OK"

#define UNICORE_WAIT_BOOTUP             "devicename"
#define UNICORE_WAIT_GPGGA              "GPGGA"

#define UNICORE_TIMEOUT_RESPONSE        1000    //ms
#define UNICORE_TIMEOUT_GPGGA           15000   //ms

#define UNICORE_TIMEOUT_READ            10      //ms

#define UNICORE_BAUDRATE                115200

#define UNICORE_RECV_BUFFER_SIZE        512
#define UNICORE_READ_BUFFER_SIZE        300

#define START_GPS_UTC_TIME              315936000


typedef struct {
	uint8_t     buffer[UNICORE_RECV_BUFFER_SIZE];
	uint16_t    count;
	uint16_t    head;
	uint16_t    tail;
} unicore_rx_buffer_t;

class GPSDriverUnicore : public GPSHelper
{
public:
	GPSDriverUnicore(GPSCallbackPtr callback, void *callback_user,
			 struct vehicle_gps_position_s *gps_position);

	virtual ~GPSDriverUnicore() = default;

	int receive(unsigned timeout) override;
	int configure(unsigned &baudrate, OutputMode output_mode) override;

private:

	int handle_message(int len);

	void buffer_init();

	int buffer_check();

	int buffer_push(uint8_t *buf, uint16_t len);

	int buffer_popMsg(uint8_t *buf, uint16_t size);

	int sendconfig(const char *config, unsigned timeout);

	int waitformsg(unsigned timeout);

	int waitforack(char *response, unsigned timeout);

	char *skipComma(char *buf, int num);

	struct vehicle_gps_position_s *_gps_position {nullptr};
	uint64_t        _last_gps_timestamp{0};
	unicore_rx_buffer_t _rx_buffer{};
	uint8_t _buf[UNICORE_READ_BUFFER_SIZE];
	bool            _pos_valid{false};
	bool            _vel_valid{false};
	bool            _event_log_valid{false};
};

#endif /* UNICORE_H */
