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
#include <geo/geo.h>
#include <mathlib/mathlib.h>
#include "matlab_sim.h"

#define SWAP16(X)	((((X) >>  8) & 0x00ff) | (((X) << 8) & 0xff00))

#define SIM_WAIT_BEFORE_READ	20		// ms, wait before reading to save read() calls
#define SIM_READ_buf_receiveFER_SIZE 128
#define SIM_PACKET_TIMEOUT	2		// ms, if now data during this delay assume that full update received

uint8_t buffert[24];

static const float mg2ms2 = CONSTANTS_ONE_G / 1000.0f;

namespace Matlab_Sim
{
    MatlabSim *sim_control;
}

MatlabSim::MatlabSim():
    _task_should_exit(false),
    _sim_task(-1)
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

int	// -1 = error, 0 = no message handled, 1 = message handled, 2 = sat info message handled
MatlabSim::receive(unsigned timeout)
{
    uint8_t buf[SIM_READ_buf_receiveFER_SIZE];

    /* timeout additional to poll */
    hrt_abstime time_started = hrt_absolute_time();

    while (true) {

        /* Wait for only SIM_PACKET_TIMEOUT if something already received. */
        int ret = read(buf, sizeof(buf), timeout);

        if (ret < 0) {
            /* something went wrong when polling or reading */
            warnx("sim poll_or_read err");
            return -1;

        } else if (ret == 0) {


        } else {

            /* pass received bytes to the packet decoder */
            for (int i = 0; i < ret; i++) {
                parseChar(buf[i]);
            }
        }

        /* abort after timeout if no useful packets received */
        if (time_started + timeout * 1000 < hrt_absolute_time()) {
            //warnx("timed out, returning");
            return -1;
        }
    }
}

int
MatlabSim::read(uint8_t *buf, int buf_length, int timeout)
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
            //usleep(SIM_WAIT_BEFORE_READ * 1000);
            ret = ::read(_serial_fd, buf, buf_length);

        } else {
            ret = -1;
        }
    }

    return ret;
}

int	// 0 = decoding, 1 = message handled, 2 = sat info message handled
MatlabSim::parseChar(const uint8_t b)
{
    int ret = 0;

    //warnx("parseChar %x",b);

    switch (_decode_state) {

    /* Expecting Sync1 */
    case SIM_DECODE_SYNC1:
        if (b == SIM_SYNC1) {	// Sync1 found --> expecting Sync2
            _decode_state = SIM_DECODE_SYNC2;
            //warnx("sync1");
        }
        break;

    /* Expecting Sync2 */
    case SIM_DECODE_SYNC2:
        if (b == SIM_SYNC2) {	// Sync2 found --> expecting Class
            _decode_state = SIM_DECODE_CLASS;
            //warnx("sync2");

        } else {		// Sync1 not followed by Sync2: reset parser
            decodeInit();
        }

        break;

    /* Expecting Class */
    case SIM_DECODE_CLASS:
        addByteToChecksum(b);  // checksum is calculated for everything except Sync and Checksum bytes
        _rx_msg = b;
        _decode_state = SIM_DECODE_ID;
        //warnx("class = %x",b);
        break;

    /* Expecting ID */
    case SIM_DECODE_ID:
        addByteToChecksum(b);
        _rx_msg |= b << 8;
        _decode_state = SIM_DECODE_LENGTH1;
        //warnx("id = %x",b);
        break;

    /* Expecting first length byte */
    case SIM_DECODE_LENGTH1:
        addByteToChecksum(b);
        _rx_payload_length = b;
        _decode_state = SIM_DECODE_LENGTH2;
        //warnx("length1 = %x",b);
        break;

    /* Expecting second length byte */
    case SIM_DECODE_LENGTH2:
        addByteToChecksum(b);
        _rx_payload_length |= b << 8;	// calculate payload size
        //warnx("length2 = %x",b);

        //warnx("sizeof = %d",sizeof(sim_payload_rx_raw_from_matlab_t));

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

//        switch(_rx_msg){
//        case SIM_MSG_RAW_FROM_MATLAB:
            //ret = payloadRxAddFromMatlab(b);	// add a NAV-SVINFO payload byte
//            break;
//        default:
            ret = payloadRxAdd(b);		// add a payload byte
//            break;
//        }

        if (ret < 0) {
            // payload not handled, discard message
            decodeInit();
            warnx("payload rx add error");

        } else if (ret > 0) {
            switch(_rx_msg){
            case SIM_MSG_RAW_FROM_MATLAB:
                _decode_state = SIM_DECODE_CHECKTERMIN;
                break;
            default:
                // payload complete, expecting checksum
                _decode_state = SIM_DECODE_CHKSUM1;
                break;
                warnx("payload rx add ret > 0");
            }

        } else {
            // expecting more payload, stay in state UBX_DECODE_PAYLOAD
        }

        ret = 0;
        break;

    /* Expecting first checksum byte */
    case SIM_DECODE_CHKSUM1:
        //warnx("checksum1");
        if (_rx_ck_a != b) {
            decodeInit();
            //warnx("checksum1 error");

        } else {
            _decode_state = SIM_DECODE_CHKSUM2;
            //warnx("checksum1 ok");
        }

        break;

    /* Expecting second checksum byte */
    case SIM_DECODE_CHKSUM2:
        //warnx("checksum2");
        if (_rx_ck_b != b) {
            //warnx("checksum2 error");
        } else {
            ret = payloadRxDone();	// finish payload processing
            //warnx("checksum2 ok");
        }

        decodeInit();
        break;

    case SIM_DECODE_CHECKTERMIN:
        if('\r' != b){
            warnx("check terminator error");
        }
        else{
            ret = payloadRxDone();
            //warnx("check terminator ok");
        }

        decodeInit();

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
            warnx("accel length error");

        }
        break;

    case SIM_MSG_RAW_MAG:
        if (_rx_payload_length != sizeof(sim_payload_rx_raw_mag_t)) {
            _rx_state = SIM_RXMSG_ERROR_LENGTH;
            warnx("mag length error");

        }
        break;

    case SIM_MSG_RAW_GYRO:
        if (_rx_payload_length != sizeof(sim_payload_rx_raw_gyro_t)) {
            _rx_state = SIM_RXMSG_ERROR_LENGTH;
            warnx("gyro length error");

        }
        break;

    case SIM_MSG_RAW_BARO:
        if (_rx_payload_length != sizeof(sim_payload_rx_raw_baro_t)) {
            _rx_state = SIM_RXMSG_ERROR_LENGTH;
            warnx("baro length error");

        }
        break;

    case SIM_MSG_RAW_AIRSPEED:
        if (_rx_payload_length != sizeof(sim_payload_rx_raw_airspeed_t)) {
            _rx_state = SIM_RXMSG_ERROR_LENGTH;
            warnx("airspeed length error");

        }
        break;

    case SIM_MSG_RAW_GPS:
        if (_rx_payload_length != sizeof(sim_payload_rx_raw_gps_t)) {
            _rx_state = SIM_RXMSG_ERROR_LENGTH;
            warnx("gps length error");

        }
        break;

    case SIM_MSG_RAW_FROM_MATLAB:
        //warnx("payload decode init from matlab");
        if (_rx_payload_length != sizeof(sim_payload_rx_raw_from_matlab_t)) {
            _rx_state = SIM_RXMSG_ERROR_LENGTH;
            warnx("rx from matlab length error");

        }
        break;

    default:
        _rx_state = SIM_RXMSG_DISABLE;	// disable all other messages
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
    uint8_t *p_buf_receive = (uint8_t *)&_buf_receive;

    p_buf_receive[_rx_payload_index] = b;

    //warnx("rx_payload_length = %d",_rx_payload_length);
    //warnx("_rx_payload_index = %d",_rx_payload_index);

    if (++_rx_payload_index >= _rx_payload_length) {
        ret = 1;	// payload received completely
    }

    return ret;
}

/**
 * Add payload rx from matlab byte
 */
int	// -1 = error, 0 = ok, 1 = payload completed
MatlabSim::payloadRxAddFromMatlab(const uint8_t b)
{
    int ret = 0;
    uint8_t *p_buf = (uint8_t *)&_buf_receive;

    if (_rx_payload_index < sizeof(sim_payload_rx_raw_from_matlab_t)) {
        // Fill Part 1 buffer
        p_buf[_rx_payload_index] = b;

    } else {
        if (_rx_payload_index == sizeof(sim_payload_rx_raw_from_matlab_t)) {
            memset(&_sensor_accel, 0, sizeof(_sensor_accel));
            _sensor_accel.timestamp = hrt_absolute_time();
            _sensor_accel.x_raw = _buf_receive.payload_rx_raw_from_matlab.accel_x / mg2ms2;
            _sensor_accel.y_raw = _buf_receive.payload_rx_raw_from_matlab.accel_y / mg2ms2;
            _sensor_accel.z_raw = _buf_receive.payload_rx_raw_from_matlab.accel_z / mg2ms2;
            _sensor_accel.x = _buf_receive.payload_rx_raw_from_matlab.accel_x;
            _sensor_accel.y = _buf_receive.payload_rx_raw_from_matlab.accel_y;
            _sensor_accel.z = _buf_receive.payload_rx_raw_from_matlab.accel_z;
            _sensor_accel.integral_dt = 0;
            if(_sensor_accel.x > 10)
            {
                warnx("accel x = %f",(double)_sensor_accel.x);
            }
            //ret = orb_publish_auto(ORB_ID(sensor_accel), &_accel_pub, &_sensor_accel, &accel_multi, ORB_PRIO_HIGH);
            ret = orb_publish(ORB_ID(sensor_accel),_accel_pub,&_sensor_accel);

            memset(&_vehicle_attitude, 0, sizeof(_vehicle_attitude));
            _vehicle_attitude.pitch = _buf_receive.payload_rx_raw_from_matlab.mag_y;
            _vehicle_attitude.roll = _buf_receive.payload_rx_raw_from_matlab.mag_x;
            _vehicle_attitude.yaw = _buf_receive.payload_rx_raw_from_matlab.mag_z;

            memset(&_sensor_gyro, 0, sizeof(_sensor_gyro));
            _sensor_gyro.timestamp = hrt_absolute_time();
            _sensor_gyro.x_raw = _buf_receive.payload_rx_raw_from_matlab.gyro_x * 1000.0f;
            _sensor_gyro.y_raw = _buf_receive.payload_rx_raw_from_matlab.gyro_y * 1000.0f;
            _sensor_gyro.z_raw = _buf_receive.payload_rx_raw_from_matlab.gyro_z * 1000.0f;
            _sensor_gyro.x = _buf_receive.payload_rx_raw_from_matlab.gyro_x;
            _sensor_gyro.y = _buf_receive.payload_rx_raw_from_matlab.gyro_y;
            _sensor_gyro.z = _buf_receive.payload_rx_raw_from_matlab.gyro_z;
            _sensor_gyro.integral_dt = 0;
            //orb_publish_auto(ORB_ID(sensor_gyro), &_gyro_pub, &_sensor_gyro, &gyro_multi, ORB_PRIO_HIGH);
            ret = orb_publish(ORB_ID(sensor_gyro),_gyro_pub,&_sensor_gyro);
            //warnx("publish gyro");
            _vehicle_attitude.pitchspeed = _buf_receive.payload_rx_raw_from_matlab.gyro_y;
            _vehicle_attitude.rollspeed = _buf_receive.payload_rx_raw_from_matlab.gyro_x;
            _vehicle_attitude.yawspeed = _buf_receive.payload_rx_raw_from_matlab.gyro_z;


            q.from_euler(_vehicle_attitude.roll,_vehicle_attitude.pitch,_vehicle_attitude.yaw);
            _vehicle_attitude.q[0] = q(0);
            _vehicle_attitude.q[1] = q(1);
            _vehicle_attitude.q[2] = q(2);
            _vehicle_attitude.q[3] = q(3);
            _vehicle_attitude.q_valid = true;

            R.from_euler(_vehicle_attitude.roll,_vehicle_attitude.pitch,_vehicle_attitude.yaw);
            _vehicle_attitude.R[0] = R(0,0);
            _vehicle_attitude.R[1] = R(0,1);
            _vehicle_attitude.R[2] = R(0,2);
            _vehicle_attitude.R[3] = R(1,0);
            _vehicle_attitude.R[4] = R(1,1);
            _vehicle_attitude.R[5] = R(1,2);
            _vehicle_attitude.R[6] = R(2,0);
            _vehicle_attitude.R[7] = R(2,1);
            _vehicle_attitude.R[8] = R(2,2);
            _vehicle_attitude.R_valid = true;

            _vehicle_attitude.timestamp = hrt_absolute_time();

            ret = orb_publish(ORB_ID(vehicle_attitude),_attitude_pub,&_vehicle_attitude);

            _ctrl_state.timestamp = _vehicle_attitude.timestamp;

            /* attitude quaternions for control state */
            _ctrl_state.q[0] = _vehicle_attitude.q[0];
            _ctrl_state.q[1] = _vehicle_attitude.q[1];
            _ctrl_state.q[2] = _vehicle_attitude.q[2];
            _ctrl_state.q[3] = _vehicle_attitude.q[3];

            _ctrl_state.x_acc = _vehicle_attitude.pitchacc;
            _ctrl_state.y_acc = _vehicle_attitude.rollacc;
            _ctrl_state.z_acc = _vehicle_attitude.yawacc;

            /* attitude rates for control state */
            _ctrl_state.roll_rate = _vehicle_attitude.rollspeed;

            _ctrl_state.pitch_rate = _vehicle_attitude.pitchspeed;

            _ctrl_state.yaw_rate = _vehicle_attitude.yawspeed;

            _ctrl_state.airspeed = _airspeed.indicated_airspeed_m_s;
            _ctrl_state.airspeed_valid = true;


            ret = orb_publish(ORB_ID(control_state), _ctrl_state_pub, &_ctrl_state);

            memset(&_vehicle_gps_position, 0, sizeof(_vehicle_gps_position));
            _vehicle_gps_position.timestamp = hrt_absolute_time();
            _vehicle_gps_position.lat = _buf_receive.payload_rx_raw_from_matlab.lat;
            _vehicle_gps_position.lon = _buf_receive.payload_rx_raw_from_matlab.lon;
            _vehicle_gps_position.alt = _buf_receive.payload_rx_raw_from_matlab.alt;
            _vehicle_gps_position.eph = _buf_receive.payload_rx_raw_from_matlab.eph;
            _vehicle_gps_position.epv = _buf_receive.payload_rx_raw_from_matlab.epv;
            _vehicle_gps_position.vel_n_m_s = _buf_receive.payload_rx_raw_from_matlab.vn;
            _vehicle_gps_position.vel_e_m_s = _buf_receive.payload_rx_raw_from_matlab.ve;
            _vehicle_gps_position.vel_d_m_s = _buf_receive.payload_rx_raw_from_matlab.vd;
            _vehicle_gps_position.vel_m_s = _buf_receive.payload_rx_raw_from_matlab.vel;
            _vehicle_gps_position.fix_type = _buf_receive.payload_rx_raw_from_matlab.fix_type;
            _vehicle_gps_position.satellites_used = _buf_receive.payload_rx_raw_from_matlab.satellites_visible;

            //warnx("lat = %f,lon = %f",(double)_vehicle_gps_position.lat,(double)_vehicle_gps_position.lon);

            //orb_publish_auto(ORB_ID(vehicle_gps_position), &_gps_pub, &_airspeed, &gps_multi, ORB_PRIO_HIGH);
            ret = orb_publish(ORB_ID(vehicle_gps_position),_gps_pub,&_vehicle_gps_position);
            //warnx("publish gps");
        }
    }

    if (++_rx_payload_index >= _rx_payload_length) {
        ret = 1;	// payload received completely
    }

    return ret;
}

/**
 * Finish payload rx
 */
int	// 0 = no message handled, 1 = message handled, 2 = sat info message handled
MatlabSim::payloadRxDone(void)
{
    int ret = 0;

    // return if no message handled
    if (_rx_state != SIM_RXMSG_HANDLE) {
        return ret;
    }

    // handle message
    switch (_rx_msg) {

    case SIM_MSG_RAW_ACCEL:
        //int accel_multi;
        memset(&_sensor_accel, 0, sizeof(_sensor_accel));
        _sensor_accel.timestamp = hrt_absolute_time();
        _sensor_accel.x_raw = _buf_receive.payload_rx_raw_accel.x / mg2ms2;
        _sensor_accel.y_raw = _buf_receive.payload_rx_raw_accel.y / mg2ms2;
        _sensor_accel.z_raw = _buf_receive.payload_rx_raw_accel.z / mg2ms2;
        _sensor_accel.x = _buf_receive.payload_rx_raw_accel.x;
        _sensor_accel.y = _buf_receive.payload_rx_raw_accel.y;
        _sensor_accel.z = _buf_receive.payload_rx_raw_accel.z;
        _sensor_accel.integral_dt = 0;
        //ret = orb_publish_auto(ORB_ID(sensor_accel), &_accel_pub, &_sensor_accel, &accel_multi, ORB_PRIO_HIGH);
        ret = orb_publish(ORB_ID(sensor_accel),_accel_pub,&_sensor_accel);
        warnx("publish accel ret = %X",ret);
        _vehicle_attitude.pitchacc = _buf_receive.payload_rx_raw_accel.x;
        _vehicle_attitude.rollacc = _buf_receive.payload_rx_raw_accel.y;
        _vehicle_attitude.yawacc = _buf_receive.payload_rx_raw_accel.z;
        ret = 1;
        break;

    case SIM_MSG_RAW_MAG:
        //int mag_multi;
        memset(&_sensor_mag, 0, sizeof(_sensor_mag));
        //_sensor_mag.timestamp = hrt_absolute_time();
        //_sensor_mag.x_raw = _buf_receive.payload_rx_raw_mag.x * 1000.0f;
        //_sensor_mag.y_raw = _buf_receive.payload_rx_raw_mag.y * 1000.0f;
        //_sensor_mag.z_raw = _buf_receive.payload_rx_raw_mag.z * 1000.0f;
        //_sensor_mag.x = _buf_receive.payload_rx_raw_mag.x;
        //_sensor_mag.y = _buf_receive.payload_rx_raw_mag.y;
        //_sensor_mag.z = _buf_receive.payload_rx_raw_mag.z;
        //orb_publish_auto(ORB_ID(sensor_mag), &_mag_pub, &_sensor_mag, &mag_multi, ORB_PRIO_HIGH);
        ret = orb_publish(ORB_ID(sensor_mag),_mag_pub,&_sensor_mag);
        warnx("publish mag");
        _vehicle_attitude.pitch = _buf_receive.payload_rx_raw_mag.y;
        _vehicle_attitude.roll = _buf_receive.payload_rx_raw_mag.x;
        _vehicle_attitude.yaw = _buf_receive.payload_rx_raw_mag.z;
        ret = 1;
        break;

    case SIM_MSG_RAW_GYRO:
        //int gyro_multi;
        memset(&_sensor_gyro, 0, sizeof(_sensor_gyro));
        _sensor_gyro.timestamp = hrt_absolute_time();
        _sensor_gyro.x_raw = _buf_receive.payload_rx_raw_gyro.x * 1000.0f;
        _sensor_gyro.y_raw = _buf_receive.payload_rx_raw_gyro.y * 1000.0f;
        _sensor_gyro.z_raw = _buf_receive.payload_rx_raw_gyro.z * 1000.0f;
        _sensor_gyro.x = _buf_receive.payload_rx_raw_gyro.x;
        _sensor_gyro.y = _buf_receive.payload_rx_raw_gyro.y;
        _sensor_gyro.z = _buf_receive.payload_rx_raw_gyro.z;
        _sensor_gyro.integral_dt = 0;
        //orb_publish_auto(ORB_ID(sensor_gyro), &_gyro_pub, &_sensor_gyro, &gyro_multi, ORB_PRIO_HIGH);
        ret = orb_publish(ORB_ID(sensor_gyro),_gyro_pub,&_sensor_gyro);
        warnx("publish gyro");
        _vehicle_attitude.pitchspeed = _buf_receive.payload_rx_raw_gyro.y;
        _vehicle_attitude.rollspeed = _buf_receive.payload_rx_raw_gyro.x;
        _vehicle_attitude.yawspeed = _buf_receive.payload_rx_raw_gyro.z;


        q.from_euler(_vehicle_attitude.roll,_vehicle_attitude.pitch,_vehicle_attitude.yaw);
        _vehicle_attitude.q[0] = q(0);
        _vehicle_attitude.q[1] = q(1);
        _vehicle_attitude.q[2] = q(2);
        _vehicle_attitude.q[3] = q(3);
        _vehicle_attitude.q_valid = true;

        R.from_euler(_vehicle_attitude.roll,_vehicle_attitude.pitch,_vehicle_attitude.yaw);
        _vehicle_attitude.R[0] = R(0,0);
        _vehicle_attitude.R[1] = R(0,1);
        _vehicle_attitude.R[2] = R(0,2);
        _vehicle_attitude.R[3] = R(1,0);
        _vehicle_attitude.R[4] = R(1,1);
        _vehicle_attitude.R[5] = R(1,2);
        _vehicle_attitude.R[6] = R(2,0);
        _vehicle_attitude.R[7] = R(2,1);
        _vehicle_attitude.R[8] = R(2,2);
        _vehicle_attitude.R_valid = true;

        _vehicle_attitude.timestamp = hrt_absolute_time();

        ret = orb_publish(ORB_ID(vehicle_attitude),_attitude_pub,&_vehicle_attitude);
        warnx("publish vehicle attitude");

        _ctrl_state.timestamp = _vehicle_attitude.timestamp;

        /* attitude quaternions for control state */
        _ctrl_state.q[0] = _vehicle_attitude.q[0];
        _ctrl_state.q[1] = _vehicle_attitude.q[1];
        _ctrl_state.q[2] = _vehicle_attitude.q[2];
        _ctrl_state.q[3] = _vehicle_attitude.q[3];

        _ctrl_state.x_acc = _vehicle_attitude.pitchacc;
        _ctrl_state.y_acc = _vehicle_attitude.rollacc;
        _ctrl_state.z_acc = _vehicle_attitude.yawacc;

        /* attitude rates for control state */
        _ctrl_state.roll_rate = _vehicle_attitude.rollspeed;

        _ctrl_state.pitch_rate = _vehicle_attitude.pitchspeed;

        _ctrl_state.yaw_rate = _vehicle_attitude.yawspeed;

        _ctrl_state.airspeed = _airspeed.indicated_airspeed_m_s;
        _ctrl_state.airspeed_valid = true;


        ret = orb_publish(ORB_ID(control_state), _ctrl_state_pub, &_ctrl_state);
        warnx("publish control state");

        //ret = 1;
        break;

    case SIM_MSG_RAW_BARO:
        //int baro_multi;
        memset(&_sensor_baro, 0, sizeof(_sensor_baro));
        _sensor_baro.timestamp = hrt_absolute_time();
        _sensor_baro.pressure = _buf_receive.payload_rx_raw_baro.pressure;
        _sensor_baro.altitude = _buf_receive.payload_rx_raw_baro.altitude;
        //orb_publish_auto(ORB_ID(sensor_baro), &_baro_pub, &_sensor_baro, &baro_multi, ORB_PRIO_HIGH);
        ret = orb_publish(ORB_ID(sensor_baro),_baro_pub,&_sensor_baro);
        warnx("publish baro");
        break;

    case SIM_MSG_RAW_AIRSPEED:
        //int airspeed_multi;
        memset(&_airspeed, 0, sizeof(_airspeed));
        _airspeed.timestamp = hrt_absolute_time();
        _airspeed.indicated_airspeed_m_s = _buf_receive.payload_rx_raw_airspeed.indicated_airspeed_m_s;
        _airspeed.true_airspeed_m_s = _buf_receive.payload_rx_raw_airspeed.true_airspeed_m_s;
        _airspeed.true_airspeed_unfiltered_m_s = _buf_receive.payload_rx_raw_airspeed.true_airspeed_m_s;
        //orb_publish_auto(ORB_ID(airspeed), &_airspeed_pub, &_airspeed, &airspeed_multi, ORB_PRIO_HIGH);
        ret = orb_publish(ORB_ID(airspeed),_airspeed_pub,&_airspeed);
        warnx("publish airspeed");

        ret = 1;
        break;

    case SIM_MSG_RAW_GPS:
        //int gps_multi;
        memset(&_vehicle_gps_position, 0, sizeof(_vehicle_gps_position));
        _vehicle_gps_position.timestamp = hrt_absolute_time();
        _vehicle_gps_position.lat = _buf_receive.payload_rx_raw_gps.lat;
        _vehicle_gps_position.lon = _buf_receive.payload_rx_raw_gps.lon;
        _vehicle_gps_position.alt = _buf_receive.payload_rx_raw_gps.alt;
        _vehicle_gps_position.eph = _buf_receive.payload_rx_raw_gps.eph;
        _vehicle_gps_position.epv = _buf_receive.payload_rx_raw_gps.epv;
        _vehicle_gps_position.vel_n_m_s = _buf_receive.payload_rx_raw_gps.vn;
        _vehicle_gps_position.vel_e_m_s = _buf_receive.payload_rx_raw_gps.ve;
        _vehicle_gps_position.vel_d_m_s = _buf_receive.payload_rx_raw_gps.vd;
        _vehicle_gps_position.vel_m_s = _buf_receive.payload_rx_raw_gps.vel;
        _vehicle_gps_position.fix_type = _buf_receive.payload_rx_raw_gps.fix_type;
        _vehicle_gps_position.satellites_used = _buf_receive.payload_rx_raw_gps.satellites_visible;

        //warnx("lat = %f,lon = %f",(double)_vehicle_gps_position.lat,(double)_vehicle_gps_position.lon);

        //orb_publish_auto(ORB_ID(vehicle_gps_position), &_gps_pub, &_airspeed, &gps_multi, ORB_PRIO_HIGH);
        ret = orb_publish(ORB_ID(vehicle_gps_position),_gps_pub,&_vehicle_gps_position);
        warnx("publish gps");

        ret = 1;
        break;

    case SIM_MSG_RAW_FROM_MATLAB:
        memset(&_sensor_accel, 0, sizeof(_sensor_accel));
        _sensor_accel.timestamp = hrt_absolute_time();
        _sensor_accel.x_raw = _buf_receive.payload_rx_raw_from_matlab.accel_x / mg2ms2;
        _sensor_accel.y_raw = _buf_receive.payload_rx_raw_from_matlab.accel_y / mg2ms2;
        _sensor_accel.z_raw = _buf_receive.payload_rx_raw_from_matlab.accel_z / mg2ms2;
        _sensor_accel.x = _buf_receive.payload_rx_raw_from_matlab.accel_x;
        _sensor_accel.y = _buf_receive.payload_rx_raw_from_matlab.accel_y;
        _sensor_accel.z = _buf_receive.payload_rx_raw_from_matlab.accel_z;
        _sensor_accel.integral_dt = 0;
        //ret = orb_publish_auto(ORB_ID(sensor_accel), &_accel_pub, &_sensor_accel, &accel_multi, ORB_PRIO_HIGH);
        ret = orb_publish(ORB_ID(sensor_accel),_accel_pub,&_sensor_accel);
        //warnx("accel");

        memset(&_vehicle_attitude, 0, sizeof(_vehicle_attitude));
        _vehicle_attitude.pitch = _buf_receive.payload_rx_raw_from_matlab.mag_y;
        _vehicle_attitude.roll = _buf_receive.payload_rx_raw_from_matlab.mag_x;
        _vehicle_attitude.yaw = _buf_receive.payload_rx_raw_from_matlab.mag_z;

        memset(&_sensor_gyro, 0, sizeof(_sensor_gyro));
        _sensor_gyro.timestamp = hrt_absolute_time();
        _sensor_gyro.x_raw = _buf_receive.payload_rx_raw_from_matlab.gyro_x * 1000.0f;
        _sensor_gyro.y_raw = _buf_receive.payload_rx_raw_from_matlab.gyro_y * 1000.0f;
        _sensor_gyro.z_raw = _buf_receive.payload_rx_raw_from_matlab.gyro_z * 1000.0f;
        _sensor_gyro.x = _buf_receive.payload_rx_raw_from_matlab.gyro_x;
        _sensor_gyro.y = _buf_receive.payload_rx_raw_from_matlab.gyro_y;
        _sensor_gyro.z = _buf_receive.payload_rx_raw_from_matlab.gyro_z;
        _sensor_gyro.integral_dt = 0;
        //orb_publish_auto(ORB_ID(sensor_gyro), &_gyro_pub, &_sensor_gyro, &gyro_multi, ORB_PRIO_HIGH);
        ret = orb_publish(ORB_ID(sensor_gyro),_gyro_pub,&_sensor_gyro);
        //warnx("publish gyro");

        _sensor_baro.timestamp = hrt_absolute_time();
        _sensor_baro.pressure = _buf_receive.payload_rx_raw_from_matlab.pressure;
        _sensor_baro.altitude = _buf_receive.payload_rx_raw_from_matlab.altitude;
        //orb_publish_auto(ORB_ID(sensor_baro), &_baro_pub, &_sensor_baro, &baro_multi, ORB_PRIO_HIGH);
        ret = orb_publish(ORB_ID(sensor_baro),_baro_pub,&_sensor_baro);
        //warnx("baro alt = %f",(double)_sensor_baro.altitude);

        _vehicle_attitude.pitchspeed = _buf_receive.payload_rx_raw_from_matlab.gyro_y;
        _vehicle_attitude.rollspeed = _buf_receive.payload_rx_raw_from_matlab.gyro_x;
        _vehicle_attitude.yawspeed = _buf_receive.payload_rx_raw_from_matlab.gyro_z;


        q.from_euler(_vehicle_attitude.roll,_vehicle_attitude.pitch,_vehicle_attitude.yaw);
        _vehicle_attitude.q[0] = q(0);
        _vehicle_attitude.q[1] = q(1);
        _vehicle_attitude.q[2] = q(2);
        _vehicle_attitude.q[3] = q(3);
        _vehicle_attitude.q_valid = true;

        R.from_euler(_vehicle_attitude.roll,_vehicle_attitude.pitch,_vehicle_attitude.yaw);
        _vehicle_attitude.R[0] = R(0,0);
        _vehicle_attitude.R[1] = R(0,1);
        _vehicle_attitude.R[2] = R(0,2);
        _vehicle_attitude.R[3] = R(1,0);
        _vehicle_attitude.R[4] = R(1,1);
        _vehicle_attitude.R[5] = R(1,2);
        _vehicle_attitude.R[6] = R(2,0);
        _vehicle_attitude.R[7] = R(2,1);
        _vehicle_attitude.R[8] = R(2,2);
        _vehicle_attitude.R_valid = true;

        _vehicle_attitude.timestamp = hrt_absolute_time();

        ret = orb_publish(ORB_ID(vehicle_attitude),_attitude_pub,&_vehicle_attitude);
        //warnx("vehicle attitude");

        _ctrl_state.timestamp = _vehicle_attitude.timestamp;

        /* attitude quaternions for control state */
        _ctrl_state.q[0] = _vehicle_attitude.q[0];
        _ctrl_state.q[1] = _vehicle_attitude.q[1];
        _ctrl_state.q[2] = _vehicle_attitude.q[2];
        _ctrl_state.q[3] = _vehicle_attitude.q[3];

        _ctrl_state.x_acc = _vehicle_attitude.pitchacc;
        _ctrl_state.y_acc = _vehicle_attitude.rollacc;
        _ctrl_state.z_acc = _vehicle_attitude.yawacc;

        /* attitude rates for control state */
        _ctrl_state.roll_rate = _vehicle_attitude.rollspeed;

        _ctrl_state.pitch_rate = _vehicle_attitude.pitchspeed;

        _ctrl_state.yaw_rate = _vehicle_attitude.yawspeed;

        _ctrl_state.airspeed = _airspeed.indicated_airspeed_m_s;
        _ctrl_state.airspeed_valid = true;


        ret = orb_publish(ORB_ID(control_state), _ctrl_state_pub, &_ctrl_state);
        //warnx("control state");

        memset(&_vehicle_gps_position, 0, sizeof(_vehicle_gps_position));
        _vehicle_gps_position.timestamp = hrt_absolute_time();
        _vehicle_gps_position.lat = _buf_receive.payload_rx_raw_from_matlab.lat;
        _vehicle_gps_position.lon = _buf_receive.payload_rx_raw_from_matlab.lon;
        _vehicle_gps_position.alt = _buf_receive.payload_rx_raw_from_matlab.alt;
        _vehicle_gps_position.eph = _buf_receive.payload_rx_raw_from_matlab.eph;
        _vehicle_gps_position.epv = _buf_receive.payload_rx_raw_from_matlab.epv;
        _vehicle_gps_position.vel_n_m_s = _buf_receive.payload_rx_raw_from_matlab.vn;
        _vehicle_gps_position.vel_e_m_s = _buf_receive.payload_rx_raw_from_matlab.ve;
        _vehicle_gps_position.vel_d_m_s = _buf_receive.payload_rx_raw_from_matlab.vd;
        _vehicle_gps_position.vel_m_s = _buf_receive.payload_rx_raw_from_matlab.vel;
        _vehicle_gps_position.fix_type = _buf_receive.payload_rx_raw_from_matlab.fix_type;
        _vehicle_gps_position.satellites_used = _buf_receive.payload_rx_raw_from_matlab.satellites_visible;

        //warnx("lat = %f,lon = %f",(double)_vehicle_gps_position.lat,(double)_vehicle_gps_position.lon);

        //orb_publish_auto(ORB_ID(vehicle_gps_position), &_gps_pub, &_airspeed, &gps_multi, ORB_PRIO_HIGH);
        ret = orb_publish(ORB_ID(vehicle_gps_position),_gps_pub,&_vehicle_gps_position);
        //warnx("publish gps");

    default:
        break;
    }

    return ret;
}

bool
MatlabSim::sendMessage(const uint16_t msg, const uint8_t *payload, const uint16_t length)
{
    sim_header_t   header = {SIM_SYNC1, SIM_SYNC2, 0, 0};
    sim_checksum_t checksum = {0, 0};

    // Populate header
    header.msg	= msg;
    header.length	= length;

    //warnx("length %d",length);

    // Calculate checksum
    calcChecksum(((uint8_t *)&header) + 2, sizeof(header) - 2, &checksum); // skip 2 sync bytes

    if (payload != nullptr) {
        calcChecksum(payload, length, &checksum);
        //warnx("checksum");
    }

    // Send message
    if (write((void *)&header, sizeof(header)) != sizeof(header)) {
        //warnx("send header error.");
        return false;
    }

    if (payload && write((void *)payload, length) != length) {
        //warnx("send payload error.");
        return false;
    }

    if (write((void *)&checksum, sizeof(checksum)) != sizeof(checksum)) {
        //warnx("send checksum error.");
        return false;
    }

    return true;
}

int
MatlabSim::write(const void *buf, int buf_length)
{
    return ::write(_serial_fd, buf, (size_t)buf_length);
}

bool
MatlabSim::sendtoMatlab(const uint8_t *payload, const uint16_t length)
{
    char header = 'A';
    char term = '\r';

    // Send message
    if (write((void *)&header, 1) != 1) {
        //warnx("send header error.");
        return false;
    }

    if (payload && write((void *)payload, length) != length) {
        //warnx("send payload error.");
        return false;
    }

    if (write((void *)&term, 1) != 1) {
        //warnx("send checksum error.");
        return false;
    }

    return true;
}

bool
MatlabSim::copy_if_updated_multi(orb_id_t topic, int multi_instance, int *handle, void *buffer)
{
    bool updated = false;

    if (*handle < 0) {

        if (OK == orb_exists(topic, multi_instance))

        {
            *handle = orb_subscribe_multi(topic, multi_instance);
            /* copy first data */
            if (*handle >= 0) {

                /* but only if it has really been updated */
                orb_check(*handle, &updated);

                if (updated) {
                    orb_copy(topic, *handle, buffer);
                }
            }
        }
    } else {
        orb_check(*handle, &updated);

        if (updated) {
            orb_copy(topic, *handle, buffer);
        }
    }

    return updated;
}

int
MatlabSim::task_main(int argc, char *argv[])
{
    if (argc < 2) {
        errx(1, "need a serial port name as argument");
    }

    const char *uart_name = argv[1];

    warnx("opening port %s", uart_name);

    _serial_fd = open(uart_name, O_RDWR | O_NOCTTY);

    unsigned speed = 115200;

    if (_serial_fd < 0) {
        err(1, "failed to open port: %s", uart_name);
    }

    /* Try to set baud rate */
    struct termios uart_config;
    int termios_state;

    /* Back up the original uart configuration to restore it after exit */
    if ((termios_state = tcgetattr(_serial_fd, &uart_config)) < 0) {
        warnx("ERR GET CONF %s: %d\n", uart_name, termios_state);
        close(_serial_fd);
        return -1;
    }

    /* Clear ONLCR flag (which appends a CR for every LF) */
    uart_config.c_oflag &= ~ONLCR;

    /* USB serial is indicated by /dev/ttyACM0*/
    if (strcmp(uart_name, "/dev/ttyACM0") != OK && strcmp(uart_name, "/dev/ttyACM1") != OK) {

        /* Set baud rate */
        if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
            warnx("ERR SET BAUD %s: %d\n", uart_name, termios_state);
            close(_serial_fd);
            return -1;
        }

    }

    if ((termios_state = tcsetattr(_serial_fd, TCSANOW, &uart_config)) < 0) {
        warnx("ERR SET CONF %s\n", uart_name);
        close(_serial_fd);
        return -1;
    }

    _accel_pub = orb_advertise_multi(ORB_ID(sensor_accel), &_sensor_accel, &accel_multi, ORB_PRIO_HIGH);
    _mag_pub = orb_advertise_multi(ORB_ID(sensor_mag), &_sensor_mag, &mag_multi, ORB_PRIO_HIGH);
    _gyro_pub = orb_advertise_multi(ORB_ID(sensor_gyro), &_sensor_gyro, &gyro_multi, ORB_PRIO_HIGH);
    _baro_pub = orb_advertise_multi(ORB_ID(sensor_baro), &_sensor_baro, &baro_multi, ORB_PRIO_HIGH);
    _airspeed_pub = orb_advertise_multi(ORB_ID(airspeed), &_airspeed, &airspeed_multi, ORB_PRIO_HIGH);
    _gps_pub = orb_advertise_multi(ORB_ID(vehicle_gps_position), &_vehicle_gps_position, &gps_multi, ORB_PRIO_HIGH);
    _attitude_pub = orb_advertise_multi(ORB_ID(vehicle_attitude), &_vehicle_attitude, &att_inst, ORB_PRIO_HIGH);
    _ctrl_state_pub = orb_advertise_multi(ORB_ID(control_state), &_ctrl_state, &ctrl_inst, ORB_PRIO_HIGH);

    memset(&_to_matlab, 0, sizeof(_to_matlab));
    _to_matlab.main_pwm1 = 900;
    _to_matlab.main_pwm2 = 900;
    _to_matlab.main_pwm3 = 900;
    _to_matlab.main_pwm4 = 900;
    _to_matlab.main_pwm5 = 900;
    _to_matlab.aux_pwm1 = 1500;
    _to_matlab.aux_pwm2 = 1500;
    _to_matlab.aux_pwm3 = 1500;
    _to_matlab.aux_pwm4 = 1500;


    while(!_task_should_exit)
    {
        receive(1);

        /* --- ACTUATOR OUTPUTS --- */
        if (copy_if_updated_multi(ORB_ID(actuator_outputs), 0, &act_outputs_sub, &_actuator_outputs)) {

//            memset(&_buf_send.payload_tx_raw_pwm_m, 0, sizeof(_buf_send.payload_tx_raw_pwm_m));

//            _buf_send.payload_tx_raw_pwm_m.pwm1 = _actuator_outputs.output[0];
//            _buf_send.payload_tx_raw_pwm_m.pwm2 = _actuator_outputs.output[1];
//            _buf_send.payload_tx_raw_pwm_m.pwm3 = _actuator_outputs.output[2];
//            _buf_send.payload_tx_raw_pwm_m.pwm4 = _actuator_outputs.output[3];
//            _buf_send.payload_tx_raw_pwm_m.pwm5 = _actuator_outputs.output[4];

//            //warnx("pwm1 = %f",(double)_buf_receive.payload_tx_raw_pwm_m.pwm1);

//            sendMessage(SIM_MSG_RAW_PWM_M,(uint8_t *)&_buf_send, sizeof(_buf_send.payload_tx_raw_pwm_m));

            _to_matlab.main_pwm1 = _actuator_outputs.output[0];
            _to_matlab.main_pwm2 = _actuator_outputs.output[1];
            _to_matlab.main_pwm3 = _actuator_outputs.output[2];
            _to_matlab.main_pwm4 = _actuator_outputs.output[3];
            _to_matlab.main_pwm5 = _actuator_outputs.output[4];

            sendtoMatlab((uint8_t *)&_to_matlab, sizeof(_to_matlab));

        }

        if (copy_if_updated_multi(ORB_ID(actuator_outputs), 1, &act_outputs_1_sub, &_actuator_outputs_1)) {

//            memset(&_buf_send.payload_tx_raw_pwm_a, 0, sizeof(_buf_send.payload_tx_raw_pwm_a));

//            _buf_send.payload_tx_raw_pwm_a.pwm1 = _actuator_outputs_1.output[0];
//            _buf_send.payload_tx_raw_pwm_a.pwm2 = _actuator_outputs_1.output[1];
//            _buf_send.payload_tx_raw_pwm_a.pwm3 = _actuator_outputs_1.output[2];
//            _buf_send.payload_tx_raw_pwm_a.pwm4 = _actuator_outputs_1.output[3];

//            sendMessage(SIM_MSG_RAW_PWM_A,(uint8_t *)&_buf_send, sizeof(_buf_send.payload_tx_raw_pwm_a));

//            //warnx("pwm6 = %f",(double)_buf_receive.payload_tx_raw_pwm_a.pwm1);

            _to_matlab.aux_pwm1 = _actuator_outputs_1.output[0];
            _to_matlab.aux_pwm2 = _actuator_outputs_1.output[1];
            _to_matlab.aux_pwm3 = _actuator_outputs_1.output[2];
            _to_matlab.aux_pwm4 = _actuator_outputs_1.output[3];

            sendtoMatlab((uint8_t *)&_to_matlab, sizeof(_to_matlab));
        }

        //usleep(1e4);
    }

    return 0;


}

int
MatlabSim::start(int argc, char *argv[])
{
        ASSERT(_sim_task == -1);

        /* start the task */
        _sim_task = px4_task_spawn_cmd("matlab_sim",
                           SCHED_DEFAULT,
                           SCHED_PRIORITY_MAX - 10,
                           1100,
                           (px4_main_t)&MatlabSim::task_main_trampoline,
                           (argv) ? (char * const *)&argv[2] : (char * const *)NULL);

        if (_sim_task < 0) {
            PX4_WARN("task start failed");
            return -errno;
        }

        return OK;
}

int
MatlabSim::stop()
{


        return OK;
}

void
MatlabSim::task_main_trampoline(int argc, char *argv[])
{
    Matlab_Sim::sim_control->task_main(argc,argv);
}


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

        if(Matlab_Sim::sim_control != nullptr)
        {
            warnx("already running\n");
            /* this is not an error */
            return 0;
        }

        Matlab_Sim::sim_control = new MatlabSim;

        if(Matlab_Sim::sim_control == nullptr)
        {
            warnx("alloc failed");
            return 1;
        }

        if(OK != Matlab_Sim::sim_control->start(argc,argv))
        {
            delete Matlab_Sim::sim_control;
            Matlab_Sim::sim_control = nullptr;
            warnx("start failed");
            return 1;
        }

        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        if(Matlab_Sim::sim_control == nullptr){
            warnx("not running");
            return 0;
        }

        Matlab_Sim::sim_control->stop();
        delete Matlab_Sim::sim_control;
        return 0;
    }

    if (!strcmp(argv[1], "status")) {
        if (Matlab_Sim::sim_control) {
            warnx("running");

        } else {
            warnx("stopped");
        }

        exit(0);
    }

    usage("unrecognized command");
    exit(1);
}

