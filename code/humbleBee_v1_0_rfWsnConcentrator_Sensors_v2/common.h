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
 * 1.0: LaunchPad CC1350
 * 1.1: LaunchPad CC1350 with external antenna
 * 1.2: humbleBee v1.0
 */

#define SOFTWARE_VER_MAJOR    1
#define SOFTWARE_VER_MINOR    4
/*
 * 1.0: Accelerometer, gyroscope, temperature, pressure, battery voltage, location, UTC time, timestamp, packet number, address received in one packet;
 *      Received values and own timestamp sent via UART using JSON format;
 *      Error messages sent via UART using JSON format;
 *      Acknowledgment message sent after every successful packet received;
 * 1.1: In addition to 1.0: sensor's hardware and software versions received in the same packet;
 *                          gateway's own hardware and software versions sent also via UART using JSON format;
 * 1.2: In addition to 1.1: magnetometer data is received
 * 1.3: In addition to 1.2: IAA packets are received periodically from the sensor nodes. Print them also via UART using JSON format
 * 1.4: In addition to 1.4: No matter if the sensors' values are received all in one packet or in 4 smaller packets, the content is received and send over UART
 */

/* Accelerometer, Gyrosocope, Magnetometer - number of axes */
#define NMB_AXIS_IMU        3

/***** Constants *****/
static const HwSwVersions SELF_HW_SW_VERSIONS = {HARDWARE_VER_MAJOR, HARDWARE_VER_MINOR, SOFTWARE_VER_MAJOR, SOFTWARE_VER_MINOR};

#endif /* COMMON_H_ */
