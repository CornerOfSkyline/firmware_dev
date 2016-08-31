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

    struct RawAccelData {
        float temperature;
        float x;
        float y;
        float z;
    };

    struct RawMagData {
        float temperature;
        float x;
        float y;
        float z;
    };

    struct RawMPUData {
        float	accel_x;
        float	accel_y;
        float	accel_z;
        float	temp;
        float	gyro_x;
        float	gyro_y;
        float	gyro_z;
    };

    struct RawBaroData {
        float pressure;
        float altitude;
        float temperature;
    };

    struct RawAirspeedData {
        float temperature;
        float diff_pressure;
    };

    struct RawGPSData {
        int32_t lat;
        int32_t lon;
        int32_t alt;
        uint16_t eph;
        uint16_t epv;
        uint16_t vel;
        int16_t vn;
        int16_t ve;
        int16_t vd;
        uint16_t cog;
        uint8_t fix_type;
        uint8_t satellites_visible;
    };

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

