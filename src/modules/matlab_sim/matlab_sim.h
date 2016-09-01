#ifndef MATLAB_SIM_H_
#define MATLAB_SIM_H_

#include <uORB/topics/hil_sensor.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/battery_status.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_baro.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_rc_input.h>
#include <systemlib/perf_counter.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/distance_sensor.h>

#define SIM_SYNC1 0xEB
#define SIM_SYNC2 0x90

/* Message Classes */
#define SIM_CLASS_RAW		0x01

/* Message IDs */
#define SIM_ID_RAW_ACCEL	0x01
#define SIM_ID_RAW_MAG		0x02
#define SIM_ID_RAW_MPU		0x03
#define SIM_ID_RAW_BARO		0x04
#define SIM_ID_RAW_AIRSPEED	0x12
#define SIM_ID_RAW_GPS      	0x21

/* Message Classes & IDs */
#define SIM_MSG_RAW_ACCEL              ((SIM_CLASS_RAW) | SIM_ID_RAW_ACCEL << 8)
#define SIM_MSG_RAW_MAG                 ((SIM_CLASS_RAW) | SIM_ID_RAW_MAG << 8)
#define SIM_MSG_RAW_MPU                 ((SIM_CLASS_RAW) | SIM_ID_RAW_MPU << 8)
#define SIM_MSG_RAW_BARO                ((SIM_CLASS_RAW) | SIM_ID_RAW_BARO << 8)
#define SIM_MSG_RAW_AIRSPEED         ((SIM_CLASS_RAW) | SIM_ID_RAW_AIRSPEED << 8)
#define SIM_MSG_RAW_GPS                   ((SIM_CLASS_RAW) | SIM_ID_RAW_GPS << 8)

/*** sim protocol binary message and payload definitions ***/
#pragma pack(push, 1)

/* General: Header */
typedef struct {
    uint8_t		sync1;
    uint8_t		sync2;
    uint16_t	msg;
    uint16_t	length;
} sim_header_t;

/* General: Checksum */
typedef struct {
    uint8_t		ck_a;
    uint8_t		ck_b;
} sim_checksum_t ;

/* Rx raw accel */
typedef struct {
    float temperature;
    float x;
    float y;
    float z;
}sim_payload_rx_raw_accel_t;

/* Rx raw mag */
typedef struct {
    float temperature;
    float x;
    float y;
    float z;
}sim_payload_rx_raw_mag_t;

/* Rx raw mpu */
typedef struct {
    float	accel_x;
    float	accel_y;
    float	accel_z;
    float	temp;
    float	gyro_x;
    float	gyro_y;
    float	gyro_z;
}sim_payload_rx_raw_mpu_t;

/* Rx raw baro */
typedef struct {
    float pressure;
    float altitude;
    float temperature;
}sim_payload_rx_raw_baro_t;

/* Rx raw airspeed */
typedef struct {
    float temperature;
    float diff_pressure;
}sim_payload_rx_raw_airspeed_t;

/* Rx raw gps */
typedef struct {
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
}sim_payload_rx_raw_gps_t;

/* General message and payload buffer union */
typedef union {
    sim_payload_rx_raw_accel_t		payload_rx_raw_accel;
    sim_payload_rx_raw_mpu_t        payload_rx_raw_mpu;
    sim_payload_rx_raw_mag_t        payload_rx_raw_mag;
    sim_payload_rx_raw_baro_t       payload_rx_raw_baro;
    sim_payload_rx_raw_airspeed_t   payload_rx_raw_airspeed;
    sim_payload_rx_raw_gps_t        payload_rx_raw_gps;
} sim_buf_t;

#pragma pack(pop)
/*** END OF sim protocol binary message and payload definitions ***/

/* Decoder state */
typedef enum {
    SIM_DECODE_SYNC1 = 0,
    SIM_DECODE_SYNC2,
    SIM_DECODE_CLASS,
    SIM_DECODE_ID,
    SIM_DECODE_LENGTH1,
    SIM_DECODE_LENGTH2,
    SIM_DECODE_PAYLOAD,
    SIM_DECODE_CHKSUM1,
    SIM_DECODE_CHKSUM2
} sim_decode_state_t;

/* Rx message state */
typedef enum {
    SIM_RXMSG_IGNORE = 0,
    SIM_RXMSG_HANDLE,
    SIM_RXMSG_DISABLE,
    SIM_RXMSG_ERROR_LENGTH
} sim_rxmsg_state_t;


class MatlabSim
{
public:
    MatlabSim();
	static int start(int argc, char *argv[]);
    virtual ~MatlabSim();
    int receive(unsigned timeout);

private:

    /**
     * Parse the binary sim packet
     */
    int parseChar(const uint8_t b);

    /**
     * Start payload rx
     */
    int payloadRxInit(void);

    /**
     * Add payload rx byte
     */
    int payloadRxAdd(const uint8_t b);

    /**
     * Finish payload rx
     */
    int payloadRxDone(void);

    /**
     * Reset the parse state machine for a fresh start
     */
    void decodeInit(void);

    /**
     * While parsing add every byte (except the sync bytes) to the checksum
     */
    void addByteToChecksum(const uint8_t);

    /**
     * Send a message
     * @return true on success, false on write error (errno set)
     */
    bool sendMessage(const uint16_t msg, const uint8_t *payload, const uint16_t length);

    int             _serial_fd;
    uint16_t		_rx_payload_length;
    uint16_t		_rx_payload_index;
    uint8_t			_rx_ck_a;
    uint8_t			_rx_ck_b;
    sim_buf_t		_buf;
    sim_decode_state_t decode_state;
	
}

#endif /* MATLAB_SIM_H_ */
