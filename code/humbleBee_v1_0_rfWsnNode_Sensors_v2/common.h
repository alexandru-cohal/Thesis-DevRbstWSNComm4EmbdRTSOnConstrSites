/*
 * common.h
 *
 *  Created on: 9 Jul 2018
 *      Author: Alexandru Cohal
 */

#ifndef COMMON_H_
#define COMMON_H_

/***** Type declarations *****/
typedef struct
{
    uint32_t sec;
    uint32_t subsec;
} timestamp_t;

typedef struct
{
    uint8_t hwMaj, hwMin;
    uint8_t swMaj, swMin;
} HwSwVersions;

typedef struct
{
    uint8_t degrees;
    uint16_t minutes;
    uint8_t azmuth;
} locationToSend_t;

/***** Defines *****/
#define HARDWARE_VER_MAJOR    1
#define HARDWARE_VER_MINOR    1
/*
 * 1.0: humbleBee v1.0 with accelerometer + gyroscope, temperature + pressure sensors
 * 1.1: In addition to 1.0: GNSS module
 */

#define SOFTWARE_VER_MAJOR    1
#define SOFTWARE_VER_MINOR    6
/*
 * 1.0: Accelerometer, gyroscope, temperature, pressure, battery voltage, location, UTC time, timestamp, packet number, address sent in one packet;
 *      Random backoff time before every retransmission
 * 1.1: In addition to 1.0: hardware and software versions also sent in the same packet
 * 1.2: In addition to 1.1: The address of the node is the last 8 bits of the MAC address (it is not generated randomly as before);
 *      TO DO: Use the last 32 bits of the MAC address because the last 8 bits are not necessarily unique
 * 1.3: In addition to 1.2: Magnetometer data is read and sent
 * 1.4: In addition to 1.3: IAA packets are sent periodically. The hardware and the software versions are send in this packet, not in the ones containing the data from the sensors.
 * 1.5: In addition to 1.4: The values of the sensors can be sent all in one packet or in 4 smaller packets.
 * 1.6: In addition to 1.5: A watchdog timer resets the MCU if it gets stuck.
 */

/* Accelerometer, Gyrosocope, Magnetometer - number of axes */
#define NMB_AXIS_IMU        3

/* Short or Long message length used */
#define SHORT_MESSAGES

/***** Constants *****/
static const HwSwVersions SELF_HW_SW_VERSIONS = {HARDWARE_VER_MAJOR, HARDWARE_VER_MINOR, SOFTWARE_VER_MAJOR, SOFTWARE_VER_MINOR};

/***** Variables *****/
uint8_t radio_packet_type_current;

#endif /* COMMON_H_ */
