/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RADIOPROTOCOL_H_
#define RADIOPROTOCOL_H_

#include "stdint.h"
#include "easylink/EasyLink.h"

/* Application Header files */
#include "common.h"

/* GNSS parsing library */
#include "GNSS/gnss_parser.h"

#define RADIO_CONCENTRATOR_ADDRESS     0x00
#define RADIO_EASYLINK_MODULATION     EasyLink_Phy_625bpsLrm

#define RADIO_PACKET_TYPE_ACK_PACKET               0
#define RADIO_PACKET_TYPE_IAA_PACKET               1
#define RADIO_PACKET_TYPE_SENSORS_DATA_PACKET      2

#define RADIO_PACKET_TYPE_SENSORS_DATA_PACKET_SUBTYPE_ALLIN              0
#define RADIO_PACKET_TYPE_SENSORS_DATA_PACKET_SUBTYPE_ACCGYRO            1
#define RADIO_PACKET_TYPE_SENSORS_DATA_PACKET_SUBTYPE_MAG                2
#define RADIO_PACKET_TYPE_SENSORS_DATA_PACKET_SUBTYPE_BATTEMPPRESSTIME   3
#define RADIO_PACKET_TYPE_SENSORS_DATA_PACKET_SUBTYPE_LOC                4


struct PacketHeader
{
    uint8_t sourceAddress;
    uint8_t packetType;
};

struct AckPacket
{
    struct PacketHeader header;
};

struct SensorsDataPacket
{
    struct PacketHeader header;
    uint8_t packetSubType;
    uint16_t packetNumber;

    uint16_t battVolt;
    int16_t acc[3], gyro[3];
    int32_t mag[3];
    int16_t temp;
    uint32_t press;
    locationToSend_t lat, lon;
    int32_t alt;
    uint8_t satCount;
    utc_time_t timeUTC;
    timestamp_t timestamp;
};

struct IAAPacket
{
    struct PacketHeader header;

    HwSwVersions HwSwVer;
    timestamp_t timestamp;
};

#endif /* RADIOPROTOCOL_H_ */
