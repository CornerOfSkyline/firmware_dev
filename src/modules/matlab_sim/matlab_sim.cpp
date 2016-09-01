/****************************************************************************
 *
 *   Copyright (c) 2014-2015 PX4 Development Team. All rights reserved.
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
 * @file matlab_sim_main.cpp
 *
 * Matlab sim serial interface at 115200 baud, 8 data bits,
 * 1 stop bit, no parity
 *
 * @author L3YZ <l3yz@126.com>
 */

#include <px4_config.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <fcntl.h>
#include <float.h>
#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <termios.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <uORB/uORB.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <poll.h>
#include "matlab_sim.h"

#define SIM_WAIT_BEFORE_READ	20		// ms, wait before reading to save read() calls
#define SIM_READ_BUFFER_SIZE 128
#define SIM_PACKET_TIMEOUT	2		// ms, if now data during this delay assume that full update received

__EXPORT int matlab_sim_main(int argc, char *argv[]);
static bool thread_should_exit = false;		/**< Daemon exit flag */
static bool thread_running = false;		/**< Daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */

int matlab_sim_thread_main(int argc, char *argv[]);
static void usage(const char *reason);

static uint8_t buffer[24];

static void usage(const char *reason)
{
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}

	fprintf(stderr, "usage: daemon {start|stop|status} [-p <additional params>]\n\n");
	exit(1);
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to px4_task_spawn_cmd().
 */
int matlab_sim_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
	}

	if (!strcmp(argv[1], "start")) {
		if (thread_running) {
			warnx("already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
        daemon_task = px4_task_spawn_cmd("matlab_sim",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_MAX - 5,
						 2000,
                         matlab_sim_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("running");

		} else {
			warnx("stopped");
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

MatlabSim::MatlabSim() :
{
    decodeInit();
}

MatlabSim::~MatlabSim()
{

}

void
MatlabSim::decodeInit(void)
{
    _decode_state = SIM_DECODE_SYNC1;
    _rx_ck_a = 0;
    _rx_ck_b = 0;
    _rx_payload_length = 0;
    _rx_payload_index = 0;
}

void
MatlabSim::addByteToChecksum(const uint8_t b)
{
    _rx_ck_a = _rx_ck_a + b;
    _rx_ck_b = _rx_ck_b + _rx_ck_a;
}

void
MatlabSim::calcChecksum(const uint8_t *buffer, const uint16_t length, sim_checksum_t *checksum)
{
    for (uint16_t i = 0; i < length; i++) {
        checksum->ck_a = checksum->ck_a + buffer[i];
        checksum->ck_b = checksum->ck_b + checksum->ck_a;
    }
}

bool
MatlabSim::sendMessage(const uint16_t msg, const uint8_t *payload, const uint16_t length)
{
    sim_header_t   header = {SIM_SYNC1, SIM_SYNC2, 0, 0};
    sim_checksum_t checksum = {0, 0};

    // Populate header
    header.msg	= msg;
    header.length	= length;

    // Calculate checksum
    calcChecksum(((uint8_t *)&header) + 2, sizeof(header) - 2, &checksum); // skip 2 sync bytes

    if (payload != nullptr) {
        calcChecksum(payload, length, &checksum);
    }

    // Send message
    if (write(_serial_fd, (void *)&header, sizeof(header)) != sizeof(header)) {
        return false;
    }

    if (payload && write(_serial_fd, (void *)payload, length) != length) {
        return false;
    }

    if (write(_serial_fd, (void *)&checksum, sizeof(checksum)) != sizeof(checksum)) {
        return false;
    }

    return true;
}

int	// -1 = error, 0 = no message handled, 1 = message handled, 2 = sat info message handled
MatlabSim::receive(unsigned timeout)
{
    uint8_t buf[SIM_READ_BUFFER_SIZE];

    /* timeout additional to poll */
    hrt_abstime time_started = hrt_absolute_time();

    int handled = 0;

    while (true) {

        /* Wait for only SIM_PACKET_TIMEOUT if something already received. */
        int ret = read(buf, sizeof(buf), ready_to_return ? SIM_PACKET_TIMEOUT : timeout);

        if (ret < 0) {
            /* something went wrong when polling or reading */
            warnx("sim poll_or_read err");
            return -1;

        } else if (ret == 0) {

            }

        } else {

            /* pass received bytes to the packet decoder */
            for (int i = 0; i < ret; i++) {
                handled |= parseChar(buf[i]);
            }
        }

        /* abort after timeout if no useful packets received */
        if (time_started + timeout * 1000 < hrt_absolute_time()) {
            warnx("timed out, returning");
            return -1;
        }
    }
}

int	// 0 = decoding, 1 = message handled, 2 = sat info message handled
MatlabSim::parseChar(const uint8_t b)
{
    int ret = 0;

    switch (_decode_state) {

    /* Expecting Sync1 */
    case SIM_DECODE_SYNC1:
        if (b == SIM_SYNC1) {	// Sync1 found --> expecting Sync2
            _decode_state = SIM_DECODE_SYNC2;
        }
        break;

    /* Expecting Sync2 */
    case SIM_DECODE_SYNC2:
        if (b == SIM_SYNC2) {	// Sync2 found --> expecting Class
            _decode_state = SIM_DECODE_CLASS;

        } else {		// Sync1 not followed by Sync2: reset parser
            decodeInit();
        }

        break;

    /* Expecting Class */
    case SIM_DECODE_CLASS:
        addByteToChecksum(b);  // checksum is calculated for everything except Sync and Checksum bytes
        _rx_msg = b;
        _decode_state = SIM_DECODE_ID;
        break;

    /* Expecting ID */
    case SIM_DECODE_ID:
        addByteToChecksum(b);
        _rx_msg |= b << 8;
        _decode_state = SIM_DECODE_LENGTH1;
        break;

    /* Expecting first length byte */
    case SIM_DECODE_LENGTH1:
        addByteToChecksum(b);
        _rx_payload_length = b;
        _decode_state = SIM_DECODE_LENGTH2;
        break;

    /* Expecting second length byte */
    case SIM_DECODE_LENGTH2:
        addByteToChecksum(b);
        _rx_payload_length |= b << 8;	// calculate payload size

        if (payloadRxInit() != 0) {	// start payload reception
            // payload will not be handled, discard message
            decodeInit();

        } else {
            _decode_state = (_rx_payload_length > 0) ? SIM_DECODE_PAYLOAD : SIM_DECODE_CHKSUM1;
        }

        break;

    /* Expecting payload */
    case SIM_DECODE_PAYLOAD:
        addByteToChecksum(b);

        switch (_rx_msg) {
        case SIM_MSG_RAW_ACCEL:
            ret = payloadRxAddRawAccel(b);	// add a RAW-ACCEL payload byte
            break;

        case SIM_MSG_RAW_MAG:
            ret = payloadRxAddRawMag(b);	// add a RAW-MAG payload byte
            break;

        case SIM_MSG_RAW_MPU:
            ret = payloadRxAddRawGyro(b); // add a RAW-GYRO payload byte
            break;

        case SIM_MSG_RAW_BARO:
            ret = payloadRxAddRawBaro(b); // add a RAW-BARO payload byte
            break;

        case SIM_MSG_RAW_AIRSPEED:
            ret = payloadRxAddRawAirspeed(b); // add a RAW-AIRSPEED payload byte
            break;

        case SIM_MSG_RAW_GPS:
            ret = payloadRxAddRawGps(b); // add a RAW-GPS payload byte
            break;

        default:
            ret = payloadRxAdd(b);		// add a payload byte
            break;
        }

        if (ret < 0) {
            // payload not handled, discard message
            decodeInit();

        } else if (ret > 0) {
            // payload complete, expecting checksum
            _decode_state = SIM_DECODE_CHKSUM1;

        } else {
            // expecting more payload, stay in state UBX_DECODE_PAYLOAD
        }

        ret = 0;
        break;

    /* Expecting first checksum byte */
    case SIM_DECODE_CHKSUM1:
        if (_rx_ck_a != b) {
            decodeInit();

        } else {
            _decode_state = SIM_DECODE_CHKSUM2;
        }

        break;

    /* Expecting second checksum byte */
    case SIM_DECODE_CHKSUM2:
        if (_rx_ck_b != b) {

        } else {
            ret = payloadRxDone();	// finish payload processing
        }

        decodeInit();
        break;

    default:
        break;
    }

    return ret;
}

/**
 * Start payload rx
 */
int	// -1 = abort, 0 = continue
MatlabSim::payloadRxInit()
{
    int ret = 0;

    _rx_state = SIM_RXMSG_HANDLE;	// handle by default

    switch (_rx_msg) {
    case SIM_MSG_RAW_ACCEL:
        if (_rx_payload_length != sizeof(sim_payload_rx_raw_accel_t)) {
            _rx_state = SIM_RXMSG_ERROR_LENGTH;

        }
        break;

    case SIM_MSG_RAW_MAG:
        if (_rx_payload_length != sizeof(sim_payload_rx_raw_mag_t)) {
            _rx_state = SIM_RXMSG_ERROR_LENGTH;

        }
        break;

    case SIM_MSG_RAW_GYRO:
        if (_rx_payload_length != sizeof(sim_payload_rx_raw_gyro_t)) {
            _rx_state = SIM_RXMSG_ERROR_LENGTH;

        }
        break;

    case SIM_MSG_RAW_BARO:
        if (_rx_payload_length != sizeof(sim_payload_rx_raw_baro_t)) {
            _rx_state = SIM_RXMSG_ERROR_LENGTH;

        }
        break;

    case SIM_MSG_RAW_AIRSPEED:
        if (_rx_payload_length != sizeof(sim_payload_rx_raw_airspeed_t)) {
            _rx_state = SIM_RXMSG_ERROR_LENGTH;

        }
        break;

    case SIM_MSG_RAW_GPS:
        if (_rx_payload_length != sizeof(sim_payload_rx_raw_gps_t)) {
            _rx_state = SIM_RXMSG_ERROR_LENGTH;

        }
        break;

    default:
        _rx_state = UBX_RXMSG_DISABLE;	// disable all other messages
        break;
    }

    switch (_rx_state) {
    case SIM_RXMSG_HANDLE:	// handle message
    case SIM_RXMSG_IGNORE:	// ignore message but don't report error
        ret = 0;
        break;

    case SIM_RXMSG_DISABLE:	// disable unexpected messages
        {

        }

        ret = -1;	// return error, abort handling this message
        break;

    case SIM_RXMSG_ERROR_LENGTH:	// error: invalid length
        warnx("sim msg 0x%04x invalid len %u", SWAP16((unsigned)_rx_msg), (unsigned)_rx_payload_length);
        ret = -1;	// return error, abort handling this message
        break;

    default:	// invalid message state
        warnx("sim internal err1");
        ret = -1;	// return error, abort handling this message
        break;
    }

    return ret;
}

/**
 * Add payload rx byte
 */
int	// -1 = error, 0 = ok, 1 = payload completed
MatlabSim::payloadRxAdd(const uint8_t b)
{
    int ret = 0;
    uint8_t *p_buf = (uint8_t *)&_buf;

    p_buf[_rx_payload_index] = b;

    if (++_rx_payload_index >= _rx_payload_length) {
        ret = 1;	// payload received completely
    }

    return ret;
}

/**
 * Add RAW-ACCEL payload rx byte
 */
int	// -1 = error, 0 = ok, 1 = payload completed
MatlabSim::payloadRxAddRawAccel(const uint8_t b)
{
    int ret = 0;
    uint8_t *p_buf = (uint8_t *)&_buf;

    if (_rx_payload_index < sizeof(sim_payload_rx_raw_accel_t)) {
        // Fill Part 1 buffer
        p_buf[_rx_payload_index] = b;

    } else {
        if (_rx_payload_index == sizeof(sim_payload_rx_raw_accel_t)) {
            _sensor_accel.timestamp = hrt_absolute_time();
            _sensor_accel.x = _buf.payload_rx_raw_accel.x;
            _sensor_accel.y = _buf.payload_rx_raw_accel.y;
            _sensor_accel.z = _buf.payload_rx_raw_accel.z;
        }
    }

    if (++_rx_payload_index >= _rx_payload_length) {
        ret = 1;	// payload received completely
    }

    return ret;
}

/**
 * Add RAW-MAG payload rx byte
 */
int	// -1 = error, 0 = ok, 1 = payload completed
MatlabSim::payloadRxAddRawMag(const uint8_t b)
{
    int ret = 0;
    uint8_t *p_buf = (uint8_t *)&_buf;

    if (_rx_payload_index < sizeof(sim_payload_rx_raw_mag_t)) {
        // Fill Part 1 buffer
        p_buf[_rx_payload_index] = b;

    } else {
        if (_rx_payload_index == sizeof(sim_payload_rx_raw_mag_t)) {
            _sensor_mag.timestamp = hrt_absolute_time();
            _sensor_mag.x = _buf.payload_rx_raw_mag.x;
            _sensor_mag.y = _buf.payload_rx_raw_mag.y;
            _sensor_mag.z = _buf.payload_rx_raw_mag.z;
        }
    }

    if (++_rx_payload_index >= _rx_payload_length) {
        ret = 1;	// payload received completely
    }

    return ret;
}

/**
 * Add RAW-GYRO payload rx byte
 */
int	// -1 = error, 0 = ok, 1 = payload completed
MatlabSim::payloadRxAddRawGyro(const uint8_t b)
{
    int ret = 0;
    uint8_t *p_buf = (uint8_t *)&_buf;

    if (_rx_payload_index < sizeof(sim_payload_rx_raw_gyro_t)) {
        // Fill Part 1 buffer
        p_buf[_rx_payload_index] = b;

    } else {
        if (_rx_payload_index == sizeof(sim_payload_rx_raw_gyro_t)) {
            _sensor_gyro.timestamp = hrt_absolute_time();
            _sensor_gyro.x = _buf.payload_rx_raw_gyro.x;
            _sensor_gyro.y = _buf.payload_rx_raw_gyro.y;
            _sensor_gyro.z = _buf.payload_rx_raw_gyro.z;
        }
    }

    if (++_rx_payload_index >= _rx_payload_length) {
        ret = 1;	// payload received completely
    }

    return ret;
}

/**
 * Add RAW-BARO payload rx byte
 */
int	// -1 = error, 0 = ok, 1 = payload completed
MatlabSim::payloadRxAddRawBaro(const uint8_t b)
{
    int ret = 0;
    uint8_t *p_buf = (uint8_t *)&_buf;

    if (_rx_payload_index < sizeof(sim_payload_rx_raw_baro_t)) {
        // Fill Part 1 buffer
        p_buf[_rx_payload_index] = b;

    } else {
        if (_rx_payload_index == sizeof(sim_payload_rx_raw_baro_t)) {
            _sensor_baro.timestamp = hrt_absolute_time();
            _sensor_baro.pressure = _buf.payload_rx_raw_baro.pressure;
            _sensor_baro.altitude = _buf.payload_rx_raw_baro.altitude;
        }
    }

    if (++_rx_payload_index >= _rx_payload_length) {
        ret = 1;	// payload received completely
    }

    return ret;
}

/**
 * Add RAW-AIRSPEED payload rx byte
 */
int	// -1 = error, 0 = ok, 1 = payload completed
MatlabSim::payloadRxAddRawAirspeed(const uint8_t b)
{
    int ret = 0;
    uint8_t *p_buf = (uint8_t *)&_buf;

    if (_rx_payload_index < sizeof(sim_payload_rx_raw_airspeed_t)) {
        // Fill Part 1 buffer
        p_buf[_rx_payload_index] = b;

    } else {
        if (_rx_payload_index == sizeof(sim_payload_rx_raw_airspeed_t)) {
            _airspeed.timestamp = hrt_absolute_time();
            _airspeed.indicated_airspeed_m_s = _buf.payload_rx_raw_airspeed.indicated_airspeed_m_s;
            _airspeed.true_airspeed_m_s = _buf.payload_rx_raw_airspeed.true_airspeed_m_s;
            _airspeed.true_airspeed_unfiltered_m_s = _buf.payload_rx_raw_airspeed.true_airspeed_m_s;
        }
    }

    if (++_rx_payload_index >= _rx_payload_length) {
        ret = 1;	// payload received completely
    }

    return ret;
}

/**
 * Add RAW-GPS payload rx byte
 */
int	// -1 = error, 0 = ok, 1 = payload completed
MatlabSim::payloadRxAddRawGps(const uint8_t b)
{
    int ret = 0;
    uint8_t *p_buf = (uint8_t *)&_buf;

    if (_rx_payload_index < sizeof(sim_payload_rx_raw_gps_t)) {
        // Fill Part 1 buffer
        p_buf[_rx_payload_index] = b;

    } else {
        if (_rx_payload_index == sizeof(sim_payload_rx_raw_gps_t)) {
            vehicle_gps_position.timestamp = hrt_absolute_time();
            vehicle_gps_position.lat = _buf.payload_rx_raw_gps.lat;
            vehicle_gps_position.lon = _buf.payload_rx_raw_gps.lon;
            vehicle_gps_position.alt = _buf.payload_rx_raw_gps.alt;
            vehicle_gps_position.eph = _buf.payload_rx_raw_gps.eph;
            vehicle_gps_position.epv = _buf.payload_rx_raw_gps.epv;
            vehicle_gps_position.vel_n_m_s = _buf.payload_rx_raw_gps.vn;
            vehicle_gps_position.vel_e_m_s = _buf.payload_rx_raw_gps.ve;
            vehicle_gps_position.vel_d_m_s = _buf.payload_rx_raw_gps.vd;
            vehicle_gps_position.vel_m_s = _buf.payload_rx_raw_gps.vel;
            vehicle_gps_position.fix_type = _buf.payload_rx_raw_gps.fix_type;
            vehicle_gps_position.satellites_used = _buf.payload_rx_raw_gps.satellites_visible;

        }
    }

    if (++_rx_payload_index >= _rx_payload_length) {
        ret = 1;	// payload received completely
    }

    return ret;
}

int matlab_sim_thread_main(int argc, char *argv[])
{

	if (argc < 2) {
		errx(1, "need a serial port name as argument");
	}

	const char *uart_name = argv[1];

	warnx("opening port %s", uart_name);

	int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);

    unsigned speed = 115200;

	if (serial_fd < 0) {
		err(1, "failed to open port: %s", uart_name);
	}

	/* Try to set baud rate */
	struct termios uart_config;
	int termios_state;

	/* Back up the original uart configuration to restore it after exit */
	if ((termios_state = tcgetattr(serial_fd, &uart_config)) < 0) {
		warnx("ERR GET CONF %s: %d\n", uart_name, termios_state);
		close(serial_fd);
		return -1;
	}

	/* Clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;

	/* USB serial is indicated by /dev/ttyACM0*/
	if (strcmp(uart_name, "/dev/ttyACM0") != OK && strcmp(uart_name, "/dev/ttyACM1") != OK) {

		/* Set baud rate */
		if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
			warnx("ERR SET BAUD %s: %d\n", uart_name, termios_state);
			close(serial_fd);
			return -1;
		}

	}

	if ((termios_state = tcsetattr(serial_fd, TCSANOW, &uart_config)) < 0) {
		warnx("ERR SET CONF %s\n", uart_name);
		close(serial_fd);
		return -1;
	}

	/* subscribe to vehicle status, attitude, sensors and flow*/
	struct accel_report accel0;
	struct accel_report accel1;
	struct gyro_report gyro0;
	struct gyro_report gyro1;

	/* subscribe to parameter changes */
	int accel0_sub = orb_subscribe_multi(ORB_ID(sensor_accel), 0);
	int accel1_sub = orb_subscribe_multi(ORB_ID(sensor_accel), 1);
	int gyro0_sub = orb_subscribe_multi(ORB_ID(sensor_gyro), 0);
	int gyro1_sub = orb_subscribe_multi(ORB_ID(sensor_gyro), 1);

	thread_running = true;

	while (!thread_should_exit) {

        int ret = receive_sim_data(buffer,100,20);

        if(ret < 0){
            /*poll error, ignore*/

        }else if(ret == 0){
            /*no return value, ignore*/
            warnx("no sim data");

        }else{
            sim_data_decode_publish(ret);
        }

		/*This runs at the rate of the sensors */
		struct pollfd fds[] = {
			{ .fd = accel0_sub, .events = POLLIN }
		};

		/* wait for a sensor update, check for exit condition every 500 ms */
		int ret = poll(fds, sizeof(fds) / sizeof(fds[0]), 500);

		if (ret < 0) {
			/* poll error, ignore */

		} else if (ret == 0) {
			/* no return value, ignore */
			warnx("no sensor data");

		} else {

			/* accel0 update available? */
			if (fds[0].revents & POLLIN) {
				orb_copy(ORB_ID(sensor_accel), accel0_sub, &accel0);
				orb_copy(ORB_ID(sensor_accel), accel1_sub, &accel1);
				orb_copy(ORB_ID(sensor_gyro), gyro0_sub, &gyro0);
				orb_copy(ORB_ID(sensor_gyro), gyro1_sub, &gyro1);

				// write out on accel 0, but collect for all other sensors as they have updates
                //dprintf(serial_fd, "%llu,%d,%d,%d,%d,%d,%d\n", accel0.timestamp, (int)accel0.x_raw, (int)accel0.y_raw,
                    //(int)accel0.z_raw,
                    //(int)accel1.x_raw, (int)accel1.y_raw, (int)accel1.z_raw);

                write(serial_fd,buffer,24);
			}
                read(serial_fd,buffer,24);
		}
	}

	warnx("exiting");
	thread_running = false;

	fflush(stdout);
	return 0;
}

int receive_sim_data(uint8_t *buf, size_t buf_length, int timeout)
{
        const int max_timeout = 50;

        pollfd fds[1];
        fds[0].fd = _serial_fd;
        fds[0].events = POLLIN;

        int ret = poll(fds, sizeof(fds) / sizeof(fds[0]), math::min(max_timeout, timeout));

        if (ret > 0) {
            /* if we have new data from GPS, go handle it */
            if (fds[0].revents & POLLIN) {
                /*
                 * We are here because poll says there is some data, so this
                 * won't block even on a blocking device. But don't read immediately
                 * by 1-2 bytes, wait for some more data to save expensive read() calls.
                 * If more bytes are available, we'll go back to poll() again.
                 */
                usleep(SIM_WAIT_BEFORE_READ * 1000);
                ret = ::read(_serial_fd, buf, buf_length);

            } else {
                ret = -1;
            }
        }

        return ret;
}

void sim_data_decode_publish(int num)
{
    for(int i = 0; i < num; i++)
    {
        parseChar(buffer[i]);
    }
}

void parseChar(uint8_t byte)
{
    switch(decodeState)
    {
        case SYNC:

    }
}

