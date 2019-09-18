/****************************************************************************
 *
 *   Copyright (c) 2019 YUNEEC Development Team. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file unicore.cpp
 *
 */

#include "unicore.h"

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <ctime>

GPSDriverUnicore::GPSDriverUnicore(GPSCallbackPtr callback, void *callback_user,
				   struct vehicle_gps_position_s *gps_position) :
	GPSHelper(callback, callback_user),
	_gps_position(gps_position)
{
	buffer_init();
}

int
GPSDriverUnicore::configure(unsigned &baudrate, OutputMode output_mode)
{
	if (output_mode != OutputMode::GPS) {
		GPS_WARN("Unicore: Unsupported Output Mode %i", (int)output_mode);
		return -1;
	}

	/* set baudrate first */
	if (GPSHelper::setBaudrate(UNICORE_BAUDRATE) != 0) {
		return -1;
	}

	baudrate = UNICORE_BAUDRATE;

	buffer_init();

	/* wait board bootup */
	waitforack((char *)UNICORE_WAIT_BOOTUP, UNICORE_TIMEOUT_RESPONSE);

	if (sendconfig(UNICORE_CONFIG_GPGGA, UNICORE_TIMEOUT_RESPONSE)) {
		goto errout;
	}

	if (sendconfig(UNICORE_CONFIG_GPGSA, UNICORE_TIMEOUT_RESPONSE)) {
		goto errout;
	}

	if (sendconfig(UNICORE_CONFIG_GPZDA, UNICORE_TIMEOUT_RESPONSE)) {
		goto errout;
	}

	if (sendconfig(UNICORE_CONFIG_BESTPOSA, UNICORE_TIMEOUT_RESPONSE)) {
		goto errout;
	}

	if (sendconfig(UNICORE_CONFIG_BESTVELA, UNICORE_TIMEOUT_RESPONSE)) {
		goto errout;
	}

	if (sendconfig(UNICORE_CONFIG_GPHDT, UNICORE_TIMEOUT_RESPONSE)) {
		goto errout;
	}

	if (sendconfig(UNICORE_CONFIG_PPS_EN, UNICORE_TIMEOUT_RESPONSE)) {
		goto errout;
	}

	if (sendconfig(UNICORE_CONFIG_EVENT_EN, UNICORE_TIMEOUT_RESPONSE)) {
		goto errout;
	}

	if (sendconfig(UNICORE_CONFIG_EVENT_LOG, UNICORE_TIMEOUT_RESPONSE)) {
		goto errout;
	}

	if (sendconfig(UNICORE_CONFIG_EVENT_RX, UNICORE_TIMEOUT_RESPONSE)) {
		goto errout;
	}

	/* wait first GPGGA message */
	waitforack((char *)UNICORE_WAIT_GPGGA, UNICORE_TIMEOUT_GPGGA);

	return 0;

errout:
	GPS_WARN("unicore: config write failed");
	return -1;
}

int // 1(bit:0001) = gps message handled, 2(bit:0010) = satellite info
GPSDriverUnicore::receive(unsigned timeout)
{
	gps_abstime time_started = gps_absolute_time();
	int len, ret;

	ret = -1;

	while (true) {

		len = waitformsg(timeout);

		if (len > 0) {
			handle_message(len);

			if ((_pos_valid && _vel_valid) || _event_log_valid) {
				if (_event_log_valid) {
					_event_log_valid = false;

				} else {
					_pos_valid = false;
					_vel_valid = false;
				}

				return 1;

			} else {
				/*
				message received but not valid. Maybe in room or bad signal.
				*/
				ret = -1;
			}
		}

		/* in case we get crap from GPS or time out */
		if (time_started + timeout * 1000  < gps_absolute_time()) {
			return ret;
		}
	}
}

int // -1 = NAK, timeout; 0 = ACK
GPSDriverUnicore::sendconfig(const char *config, unsigned timeout)
{
	char response_char[100];

	if (strlen(config) != (unsigned)write(config, strlen(config))) {
		return -1;
	}

	memcpy(response_char, config, strlen(config));
	memcpy(response_char + strlen(config) - 2, UNICORE_RESPONSE_TAIL, strlen(UNICORE_RESPONSE_TAIL) + 1);

	return waitforack(response_char, timeout);
}

int // -1 = NAK, error or timeout, else return message length
GPSDriverUnicore::waitformsg(unsigned timeout)
{
	gps_abstime time_started = gps_absolute_time();

	int count = buffer_popMsg(_buf, UNICORE_READ_BUFFER_SIZE);

	/* message already in ring buffer */
	if (count > 0) {
		return count;
	}

	while (true) {

		/* poll or read for new data */
		count = read(_buf, sizeof(_buf), UNICORE_TIMEOUT_READ);

		if (count < 0) {
			/* something went wrong when polling */
			return -1;

		} else if (count == 0) {
			/* Timeout while polling or just nothing read if reading, let's
			 * stay here, and use timeout below. */
		} else if (count > 0) {
			/* if we have new data from GPS, save it to ring buffers */
			if (buffer_push(_buf, count) == count) {
				/* check message received */
				count = buffer_popMsg(_buf, UNICORE_READ_BUFFER_SIZE);

				if (count > 0) {
					return count;
				}

			} else {
				/* ring buffer full */
				buffer_init();
			}
		}

		/* in case we get crap from GPS or time out */
		if (time_started + timeout * 1000  < gps_absolute_time()) {
			return -1;
		}
	}
}


int // -1 = NAK, timeout; 0 = ACK
GPSDriverUnicore::waitforack(char *response, unsigned timeout)
{
	gps_abstime time_started = gps_absolute_time();
	int len = waitformsg(timeout);
	char *ptr;

	while (time_started + timeout * 1000  > gps_absolute_time()) {
		if (len) {
			ptr = strstr((char *)_buf, response);

			if (ptr) {
				return 0;
			}
		}

		len = waitformsg(timeout);
	}

	return -1;
}

char * // nullptr = not found comma; return address contain comma if not same as buf
GPSDriverUnicore::skipComma(char *buf, int num)
{
	if (num <= 0) {
		return buf;
	}

	for (int i = 0 ; i < UNICORE_READ_BUFFER_SIZE; i++) {
		if ((buf[i] == ',') || (buf[i] == ';')) {
			num--;

			if (num == 0) {
				return &buf[i];
			}
		}
	}

	return nullptr;
}

/*
 * All NMEA descriptions are taken from
 * http://www.trimble.com/OEM_ReceiverHelp/V4.44/en/NMEA-0183messages_MessageOverview.html
 */

int
GPSDriverUnicore::handle_message(int len)
{
	char *endp;
	int ret = 0;

	if (len <= 0) {
		return ret;
	}

	char *bufptr = (char *)(_buf + 6);

	if (memcmp(_buf + 3, "ZDA", 3) == 0) {
		/*
		UTC day, month, and year, and local time zone offset
		An example of the ZDA message string is:
		$GPZDA,172809.456,12,07,1996,00,00*45
		ZDA message fields
		Field   Meaning
		0   Message ID $GPZDA
		1   UTC
		2   Day, ranging between 01 and 31
		3   Month, ranging between 01 and 12
		4   Year
		5   Local time zone offset from GMT, ranging from 00 through 13 hours
		6   Local time zone offset from GMT, ranging from 00 through 59 minutes
		7   The checksum data, always begins with *
		Fields 5 and 6 together yield the total offset. For example, if field 5 is -5
		and field 6 is +15, local time is 5 hours and 15 minutes earlier than GMT.
		*/

		double unicore_time = 0.0;
		int day = 0, month = 0, year = 0;

		if (bufptr && *(++bufptr) != ',') { unicore_time = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { day = strtol(bufptr, &endp, 10); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { month = strtol(bufptr, &endp, 10); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { year = strtol(bufptr, &endp, 10); bufptr = endp; }

		/* time is not valid */
		if (year == 0) {
			return ret;
		}

		int unicore_hour = static_cast<int>(unicore_time / 10000);
		int unicore_minute = static_cast<int>((unicore_time - unicore_hour * 10000) / 100);
		double unicore_sec = static_cast<double>(unicore_time - unicore_hour * 10000 - unicore_minute * 100);

		/*
		 * convert to unix timestamp
		 */
		struct tm timeinfo = {};
		timeinfo.tm_year = year - 1900;
		timeinfo.tm_mon = month - 1;
		timeinfo.tm_mday = day;
		timeinfo.tm_hour = unicore_hour;
		timeinfo.tm_min = unicore_minute;
		timeinfo.tm_sec = int(unicore_sec);
		timeinfo.tm_isdst = 0;

#ifndef NO_MKTIME
		time_t epoch = mktime(&timeinfo);

		if (epoch > GPS_EPOCH_SECS) {
			uint64_t usecs = static_cast<uint64_t>((unicore_sec - static_cast<uint64_t>(unicore_sec))) * 1000000;

			// FMUv2+ boards have a hardware RTC, but GPS helps us to configure it
			// and control its drift. Since we rely on the HRT for our monotonic
			// clock, updating it from time to time is safe.

			timespec ts{};
			ts.tv_sec = epoch;
			ts.tv_nsec = usecs * 1000;

			setClock(ts);

			_gps_position->time_utc_usec = static_cast<uint64_t>(epoch) * 1000000ULL;
			_gps_position->time_utc_usec += usecs;

		} else {
			_gps_position->time_utc_usec = 0;
		}

#else
		_gps_position->time_utc_usec = 0;
#endif

		_last_gps_timestamp = gps_absolute_time();

	} else if (memcmp(_buf + 3, "GGA", 3) == 0) {
		/*
		  Time, position, and fix related data
		  An example of the GBS message string is:
		  $GPGGA,172814.0,3723.46587704,N,12202.26957864,W,2,6,1.2,18.893,M,-25.669,M,2.0,0031*4F
		  GGA message fields
		  Field   Meaning
		  0   Message ID $GPGGA
		  1   UTC of position fix
		  2   Latitude
		  3   Direction of latitude:
		  N: North
		  S: South
		  4   Longitude
		  5   Direction of longitude:
		  E: East
		  W: West
		  6   GPS Quality indicator:
		      0: Fix not valid
		      1: GPS fix
		      2: Differential GPS fix, OmniSTAR VBS
		      4: Real-Time Kinematic, fixed integers
		      5: Real-Time Kinematic, float integers, OmniSTAR XP/HP or Location RTK
		  7   Number of SVs in use, range from 00 through to 24+
		  8   HDOP
		  9   Orthometric height (MSL reference)
		  10  M: unit of measure for orthometric height is meters
		  11  Geoid separation
		  12  M: geoid separation measured in meters
		  13  Age of differential GPS data record, Type 1 or Type 9. Null field when DGPS is not used.
		  14  Reference station ID, range 0000-4095. A null field when any reference station ID is
		      selected and no corrections are received1.
		  15
		  The checksum data, always begins with *
		  Note - If a user-defined geoid model, or an inclined
		*/

		int fix_quality = 0;

		bufptr = skipComma(++bufptr, 5);

		if (bufptr && *(++bufptr) != ',') { fix_quality = strtol(bufptr, &endp, 10); bufptr = endp; }

		if (fix_quality <= 0) {
			_gps_position->fix_type = 0;

		} else {
			/*
			 * in this NMEA message float integers (value 5) mode has higher value than fixed integers (value 4), whereas it provides lower quality,
			 * and since value 3 is not being used, I "moved" value 5 to 3 to add it to _gps_position->fix_type
			 */
			if (fix_quality == 5) { fix_quality = 3; }

			/*
			 * fix quality 1 means just a normal 3D fix, so I'm subtracting 1 here. This way we'll have 3 for auto, 4 for DGPS, 5 for floats, 6 for fixed.
			 */
			_gps_position->fix_type = 3 + fix_quality - 1;
		}

	} else if (memcmp(_buf + 3, "GSA", 3) == 0) {
		/*
		  GPS DOP and active satellites
		  $GPGSA,A,3,01,20,19,13,,,,,,,,,40.4,24.4,32.2*0A
		  GSA message fields
		  Field   Meaning
		  0   Message ID $GPGSA
		  1   Mode 1, M = manual, A = automatic
		  2   Mode 2, Fix type, 1 = not available, 2 = 2D, 3 = 3D
		  3   PRN number, 01 through 32 for GPS, 33 through 64 for SBAS, 64+ for GLONASS
		  4   PDOP: 0.5 through 99.9
		  5   HDOP: 0.5 through 99.9
		  6   VDOP: 0.5 through 99.9
		  7   The checksum data, always begins with *
		*/

		double hdop = 0.0, vdop = 0.0;

		bufptr = skipComma(++bufptr, 15);

		if (bufptr && *(++bufptr) != ',') { hdop = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { vdop = strtod(bufptr, &endp); bufptr = endp; }


		_gps_position->hdop = static_cast<float>(hdop);
		_gps_position->vdop = static_cast<float>(vdop);

	} else if (memcmp(_buf, "#EVENTALLA", strlen("#EVENTALLA")) == 0) {
		/*
		  Unicore event marka
		  #EVENTALL,COM3,0,64.0,FINE,2044,285920.900,655376,1,18;1,43,0,0,2044(week),285616(second),2573639(nanosecond),,,,,31.323232,120.42342,,,,,,,,,,879797*f64646
		  EVENTALL
		  Field   Meaning
		  0   event ID
		  1   status
		  2   reserved0
		  3   reserved1
		  4   week
		  5   second
		  6   nanosecond
		  7   reserved2
		  8   offest second
		  9   offest subsecond
		  10  sol status
		  11  pos type
		  12  lat:degree
		  13  lon:degree
		  14  hgt:m
		  15  undulation
		  16  datum id#
		  17  lat σ
		  18  lon σ
		  19  hgt σ
		  .
		  .
		  .
		  30  32 bit CRC
		  31   The checksum data, always begins with *
		*/

		int64_t week = 0, second = 0, nanosecond = 0;
		double lat = 0.0, lon = 0.0, hgt = 0.0;
		bufptr += 3; //goto first comma
		bufptr = skipComma(++bufptr, 14);

		if (bufptr && *(++bufptr) != ',') { week = strtol(bufptr, &endp, 10); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { second = strtol(bufptr, &endp, 10); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { nanosecond = strtol(bufptr, &endp, 10); bufptr = endp; }

		bufptr = skipComma(++bufptr, 5);

		if (bufptr && *(++bufptr) != ',') { lat = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { lon = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { hgt = strtod(bufptr, &endp); bufptr = endp; }

		_gps_position->time_utc_usec = (uint64_t)((week * 7 * 24 * 3600 + second + START_GPS_UTC_TIME) * 1e6 +
					       (nanosecond / 1e3));  //conversion to us
		_gps_position->lat = lat;
		_gps_position->lon = lon;
		_gps_position->alt = hgt;
		_gps_position->fix_type = 7;

		_event_log_valid = true;


	} else if (memcmp(_buf + 3, "HDT", 3) == 0) {
		/*
		  heading
		  $GPHDT,178.7236,T*15
		  HDT message fields
		  Field   Meaning
		  0   Message ID $GPHDT
		  1   Heading : 0-360.0 deg, main antenna -> vice antenna and true north angle in anticlockwise
		  2   True North: T
		  3   The checksum data, always begins with *
		*/

		//float heading = 0.0;

		//if (bufptr && *(++bufptr) != ',') { heading = strtof(bufptr, &endp); bufptr = endp; }


	} else if (memcmp(_buf, "#BESTPOSA", strlen("#BESTPOSA")) == 0) {
		/*
		  Best position
		  #BESTPOSA,ICOM4,0,92.0,FINE,1859,293874.000,00000000,14,0;SOL_COMPUTED,SINGLE,40.07895545155,
		    116.23650516491,67.0768,-9.7850,WGS84,1.4355,0.9639,2.0576,"0",0.000,0.000,23,22,0,0,0,06,0,7b*d144c4c6
		  Best positions message fields
		  Field   Meaning
		  0    Message ID #BESTPOSA
		  1    BESTPOS headers
		  2    sol status
		  3    pos type
		  4    Latitude
		  5    Longitude
		  6    height
		  7    undulation
		  8    datum id#
		  9    Latitude σ
		  10   Longitude σ
		  11   height σ
		  12   station id
		  13   diff_age
		  14   sol_age
		  15   #SVs
		  16   #solnSVs
		  17   Reserved
		  18   Reserved
		  19   Reserveds
		  20   ext sol stat
		  21   Reserved
		  22   sig mask
		  23   The checksum data, always begins with *
		*/

		double lat = 0.0, lon = 0.0, alt = 0.0, und = 0.0;
		double lat_err = 0.0, lon_err = 0.0, alt_err = 0.0;
		int num_of_sv = 0;

		bufptr += 3; //goto first comma
		bufptr = skipComma(++bufptr, 9);

		if (memcmp(++bufptr, "SOL_COMPUTED", strlen("SOL_COMPUTED")) != 0) {
			_pos_valid = false;
			return ret;
		}

		_pos_valid = true;

		bufptr = skipComma(++bufptr, 2);

		if (bufptr && *(++bufptr) != ',') { lat = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { lon = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { alt = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { und = strtod(bufptr, &endp); bufptr = endp; }

		bufptr = skipComma(++bufptr, 1);

		if (bufptr && *(++bufptr) != ',') { lat_err = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { lon_err = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { alt_err = strtod(bufptr, &endp); bufptr = endp; }

		bufptr = skipComma(++bufptr, 3);

		if (bufptr && *(++bufptr) != ',') { num_of_sv = strtol(bufptr, &endp, 10); bufptr = endp; }

		_gps_position->lat = static_cast<int>(lat * 10000000);

		_gps_position->lon = static_cast<int>(lon * 10000000);

		_gps_position->alt = static_cast<int>(alt * 1000);

		_gps_position->alt_ellipsoid = _gps_position->alt + static_cast<int>(und * 1000);

		_rate_count_lat_lon++;

		_gps_position->eph = sqrtf(static_cast<float>(lat_err) * static_cast<float>(lat_err)
					   + static_cast<float>(lon_err) * static_cast<float>(lon_err));
		_gps_position->epv = static_cast<float>(alt_err);

		_gps_position->s_variance_m_s = 0;

		_gps_position->satellites_used = num_of_sv;

		_gps_position->timestamp = gps_absolute_time();

	} else if ((memcmp(_buf, "#BESTVELA", strlen("#BESTVELA")) == 0)) {
		/*
		  Best velocity
		  #BESTVELA,COM1,0,61.0,FINE,1337,334167.000,00000000,827B,1984;SOL_COMPUTED,PSRDIFF,0250,4.000,
		  0.0206,227.712486,0.0493,0.0*0E68BF05
		  Best velocity message fields
		  Field   Meaning
		  0    Message ID #BESTVEL
		  1    BESTVEL header
		  2    sol status
		  3    vel type
		  4    latency
		  5    age
		  6    horizental speed
		  7    track ground
		  8    vertical speed
		  9    Reserved
		  10   The checksum data, always begins with *
		*/

		double ground_speed = 0.0, track_true = 0.0, vertical_speed = 0.0;

		bufptr += 3; //goto first comma
		bufptr = skipComma(++bufptr, 9);

		if (memcmp(++bufptr, "SOL_COMPUTED", strlen("SOL_COMPUTED")) != 0) {
			_vel_valid = false;
			return ret;
		}

		_vel_valid = true;

		bufptr = skipComma(++bufptr, 4);

		if (bufptr && *(++bufptr) != ',') { ground_speed = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { track_true = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { vertical_speed = strtod(bufptr, &endp); bufptr = endp; }

		float track_rad = static_cast<float>(track_true) * M_PI_F / 180.0f;
		float velocity_north = static_cast<float>(ground_speed) * cosf(track_rad);
		float velocity_east  = static_cast<float>(ground_speed) * sinf(track_rad);

		_gps_position->vel_m_s = static_cast<float>(ground_speed);
		_gps_position->vel_n_m_s = velocity_north;
		_gps_position->vel_e_m_s = velocity_east;
		_gps_position->vel_d_m_s = static_cast<float>(-vertical_speed);
		_gps_position->cog_rad = track_rad;
		_gps_position->vel_ned_valid = true;                /** Flag to indicate if NED speed is valid */
		_rate_count_vel++;
	}

	if (_pos_valid && _vel_valid) {
		_gps_position->timestamp_time_relative = (int32_t)(_last_gps_timestamp - _gps_position->timestamp);
	}

	return ret;
}


void
GPSDriverUnicore::buffer_init()
{
	_rx_buffer.count = _rx_buffer.head = _rx_buffer.tail = 0;
}

int
GPSDriverUnicore::buffer_check()
{
	uint16_t i, tail;

	/* find message header */
	while (_rx_buffer.count) {
		if ((_rx_buffer.buffer[_rx_buffer.tail] != '$')
		    && (_rx_buffer.buffer[_rx_buffer.tail] != '#')) {

			if (++_rx_buffer.tail == UNICORE_RECV_BUFFER_SIZE) {
				_rx_buffer.tail = 0;
			}

			_rx_buffer.count--;
			continue;
		}

		break;
	}

	/* find message tail */
	tail = _rx_buffer.tail;

	for (i = 0; i < _rx_buffer.count; i++) {
		if (_rx_buffer.buffer[tail] == '\n') {
			return i + 1;
		}

		if (++tail == UNICORE_RECV_BUFFER_SIZE) {
			tail = 0;
		}
	}

	return 0;
}

int
GPSDriverUnicore::buffer_push(uint8_t *buf, uint16_t len)
{
	uint16_t i;

	if (UNICORE_RECV_BUFFER_SIZE - _rx_buffer.count < len) {
		GPS_ERR("Unicore: receive buf full");
		return -1;
	}

	for (i = 0; i < len; i++) {
		_rx_buffer.buffer[_rx_buffer.head] = *buf;

		buf++;

		if (++_rx_buffer.head == UNICORE_RECV_BUFFER_SIZE) {
			_rx_buffer.head = 0;
		}

		_rx_buffer.count++;
	}

	return len;
}

int
GPSDriverUnicore::buffer_popMsg(uint8_t *buf, uint16_t size)
{
	uint16_t i, len;

	len = buffer_check();

	if (len == 0) {
		return 0;
	}

	if (len > size) {
		buffer_init();
		return 0;
	}

	for (i = 0; i < len; i++) {
		*buf = _rx_buffer.buffer[_rx_buffer.tail];

		buf++;

		if (++_rx_buffer.tail == UNICORE_RECV_BUFFER_SIZE) {
			_rx_buffer.tail = 0;
		}

		_rx_buffer.count--;
	}

	return len;
}
