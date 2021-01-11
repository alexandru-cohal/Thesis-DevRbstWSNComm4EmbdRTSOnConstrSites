/*
 * Copyright (c) 2015-2017, Texas Instruments Incorporated
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

/***** Includes *****/

#include <stdio.h>
/* XDCtools Header files */ 
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */ 
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Clock.h>

/* TI-RTOS Header files */
#include <ti/drivers/UART.h>

/* Board Header files */
#include "Board.h"

/* Application Header files */
#include "common.h"
#include "ConcentratorRadioTask.h"
#include "ConcentratorTask.h"
#include "RadioProtocol.h"

/* GNSS parsing library */
#include "GNSS/gnss_parser.h"

/* Real Time Clock library */
#include <ti/devices/cc13x0/driverlib/aon_rtc.h>

/***** Defines *****/
#define CONCENTRATOR_TASK_STACK_SIZE 1024
#define CONCENTRATOR_TASK_PRIORITY   3
#define IAAMSG_TASK_STACK_SIZE       1024
#define IAAMSG_TASK_PRIORITY         5
#define IAA_TASK_PERIOD              3000000

#define CONCENTRATOR_EVENT_ALL                         0xFFFFFFFF
#define CONCENTRATOR_EVENT_NEW_IAA                     (uint32_t)(1 << 1)
#define CONCENTRATOR_EVENT_NEW_SENSORS_DATA            (uint32_t)(1 << 2)

/***** Type declarations *****/
struct SensorsDataPacketContent
{
    // Payload
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

    // Not Payload, but needed
    uint8_t address;
    int8_t rssi;
};

struct IAAPacketContent
{
    // Payload
    HwSwVersions HwSwVer;
    timestamp_t timestamp;

    // Not Payload, but needed
    uint8_t address;
    int8_t rssi;
};

/***** Variable declarations *****/
static Task_Params concentratorTaskParams, IAAMsgTaskParams;
Task_Struct concentratorTask, IAAMsgTask;    /* not static so you can see in ROV */
static uint8_t concentratorTaskStack[CONCENTRATOR_TASK_STACK_SIZE], IAAMsgTaskStack[IAAMSG_TASK_STACK_SIZE];

Event_Struct concentratorEvent;  /* not static so you can see in ROV */
static Event_Handle concentratorEventHandle;

static struct SensorsDataPacketContent latestSensorsDataPacketContent;
static struct IAAPacketContent latestIAAPacketContent;

extern UART_Handle uart;

/***** Prototypes *****/
static void concentratorTaskFunction(UArg arg0, UArg arg1);
static void IAAMsgTaskFunction(UArg arg0, UArg arg1);
static void packetReceivedCallback(union ConcentratorPacket* packet, int8_t rssi);

/***** Function definitions *****/
void ConcentratorTask_init(void) {

    /* Create event used internally for state changes */
    Event_Params eventParam;
    Event_Params_init(&eventParam);
    Event_construct(&concentratorEvent, &eventParam);
    concentratorEventHandle = Event_handle(&concentratorEvent);

    /* Create the concentrator radio protocol task */
    Task_Params_init(&concentratorTaskParams);
    concentratorTaskParams.stackSize = CONCENTRATOR_TASK_STACK_SIZE;
    concentratorTaskParams.priority = CONCENTRATOR_TASK_PRIORITY;
    concentratorTaskParams.stack = &concentratorTaskStack;
    Task_construct(&concentratorTask, concentratorTaskFunction, &concentratorTaskParams, NULL);

    /* Create the concentrator radio protocol task */
    Task_Params_init(&IAAMsgTaskParams);
    IAAMsgTaskParams.stackSize = IAAMSG_TASK_STACK_SIZE;
    IAAMsgTaskParams.priority = IAAMSG_TASK_PRIORITY;
    IAAMsgTaskParams.stack = &IAAMsgTaskStack;
    Task_construct(&IAAMsgTask, IAAMsgTaskFunction, &IAAMsgTaskParams, NULL);
}

static void concentratorTaskFunction(UArg arg0, UArg arg1)
{
    char uartMessage[45] = "";
    uint8_t status;

    /* Initialize Real Time Clock */
    AONRTCEnable();
    status = AONRTCActive();

    /* Register a packet received callback with the radio task */
    ConcentratorRadioTask_registerPacketReceivedCallback(packetReceivedCallback);

    /* Enter main task loop */
    while(1)
    {
        /* Wait for event */
        uint32_t events = Event_pend(concentratorEventHandle, 0, CONCENTRATOR_EVENT_ALL, BIOS_WAIT_FOREVER);

        /* If we got a new packet containing sensors' values */
        if(events & CONCENTRATOR_EVENT_NEW_SENSORS_DATA)
        {
            memset(&uartMessage[0], 0, sizeof(uartMessage));
            sprintf(uartMessage, "{");
            UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);

            memset(&uartMessage[0], 0, sizeof(uartMessage));
            sprintf(uartMessage, "\"addressSender\":%d", latestSensorsDataPacketContent.address);
            UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);

            memset(&uartMessage[0], 0, sizeof(uartMessage));
            sprintf(uartMessage, ",\"message\":\"Sensor_Data\"");
            UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);

            memset(&uartMessage[0], 0, sizeof(uartMessage));
            sprintf(uartMessage, ",\"rssi\":%d", latestSensorsDataPacketContent.rssi);
            UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);

            memset(&uartMessage[0], 0, sizeof(uartMessage));
            sprintf(uartMessage, ",\"packetNumber\":%d", latestSensorsDataPacketContent.packetNumber);
            UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);

            if (latestSensorsDataPacketContent.packetSubType == 1 || latestSensorsDataPacketContent.packetSubType == 0)
            {
                memset(&uartMessage[0], 0, sizeof(uartMessage));
                sprintf(uartMessage, ",\"accX\":%d,\"accY\":%d,\"accZ\":%d", latestSensorsDataPacketContent.acc[0], latestSensorsDataPacketContent.acc[1], latestSensorsDataPacketContent.acc[2]);
                UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);

                memset(&uartMessage[0], 0, sizeof(uartMessage));
                sprintf(uartMessage, ",\"gyroX\":%d,\"gyroY\":%d,\"gyroZ\":%d", latestSensorsDataPacketContent.gyro[0], latestSensorsDataPacketContent.gyro[1], latestSensorsDataPacketContent.gyro[2]);
                UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);
            }

            if (latestSensorsDataPacketContent.packetSubType == 2 || latestSensorsDataPacketContent.packetSubType == 0)
            {
                memset(&uartMessage[0], 0, sizeof(uartMessage));
                sprintf(uartMessage, ",\"magX\":%d,\"magY\":%d,\"magZ\":%d", latestSensorsDataPacketContent.mag[0], latestSensorsDataPacketContent.mag[1], latestSensorsDataPacketContent.mag[2]);
                UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);
            }

            if (latestSensorsDataPacketContent.packetSubType == 3 || latestSensorsDataPacketContent.packetSubType == 0)
            {
                memset(&uartMessage[0], 0, sizeof(uartMessage));
                sprintf(uartMessage, ",\"battVolt\":%d", latestSensorsDataPacketContent.battVolt);
                UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);

                memset(&uartMessage[0], 0, sizeof(uartMessage));
                sprintf(uartMessage, ",\"temp\":%d", latestSensorsDataPacketContent.temp);
                UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);

                memset(&uartMessage[0], 0, sizeof(uartMessage));
                sprintf(uartMessage, ",\"press\":%d", latestSensorsDataPacketContent.press);
                UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);

                memset(&uartMessage[0], 0, sizeof(uartMessage));
                sprintf(uartMessage, ",\"satCount\":%d", latestSensorsDataPacketContent.satCount);
                UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);

                memset(&uartMessage[0], 0, sizeof(uartMessage));
                sprintf(uartMessage, ",\"UTChour\":%d,\"UTCmin\":%d,\"UTCsec\":%d", latestSensorsDataPacketContent.timeUTC.hour, latestSensorsDataPacketContent.timeUTC.minute, latestSensorsDataPacketContent.timeUTC.second);
                UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);
            }

            if (latestSensorsDataPacketContent.packetSubType == 4 || latestSensorsDataPacketContent.packetSubType == 0)
            {
                memset(&uartMessage[0], 0, sizeof(uartMessage));
                sprintf(uartMessage, ",\"latDeg\":%d,\"latMin\":%d,\"latAzm\":%d", latestSensorsDataPacketContent.lat.degrees, latestSensorsDataPacketContent.lat.minutes, latestSensorsDataPacketContent.lat.azmuth);
                UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);

                memset(&uartMessage[0], 0, sizeof(uartMessage));
                sprintf(uartMessage, ",\"lonDeg\":%d,\"lonMin\":%d,\"lonAzm\":%d", latestSensorsDataPacketContent.lon.degrees, latestSensorsDataPacketContent.lon.minutes, latestSensorsDataPacketContent.lon.azmuth);
                UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);

                memset(&uartMessage[0], 0, sizeof(uartMessage));
                sprintf(uartMessage, ",\"altitude\":%d", latestSensorsDataPacketContent.alt);
                UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);
            }

            memset(&uartMessage[0], 0, sizeof(uartMessage));
            sprintf(uartMessage, ",\"sensorTimestampSec\":%d", latestSensorsDataPacketContent.timestamp.sec);
            UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);

            memset(&uartMessage[0], 0, sizeof(uartMessage));
            sprintf(uartMessage, ",\"gatewayTimestampSec\":%d", AONRTCSecGet());
            UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);

            memset(&uartMessage[0], 0, sizeof(uartMessage));
            sprintf(uartMessage, "}\r\n");
            UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);
        }

        if(events & CONCENTRATOR_EVENT_NEW_IAA)
        {
            memset(&uartMessage[0], 0, sizeof(uartMessage));
            sprintf(uartMessage, "{");
            UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);

            memset(&uartMessage[0], 0, sizeof(uartMessage));
            sprintf(uartMessage, "\"addressSender\":%d", latestIAAPacketContent.address);
            UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);

            memset(&uartMessage[0], 0, sizeof(uartMessage));
            sprintf(uartMessage, ",\"message\":\"IAA\"");
            UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);

            memset(&uartMessage[0], 0, sizeof(uartMessage));
            sprintf(uartMessage, ",\"rssi\":%d", latestIAAPacketContent.rssi);
            UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);

            memset(&uartMessage[0], 0, sizeof(uartMessage));
            sprintf(uartMessage, ",\"sensorTimestampSec\":%d", latestIAAPacketContent.timestamp.sec);
            UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);

            memset(&uartMessage[0], 0, sizeof(uartMessage));
            sprintf(uartMessage, ",\"sensorHwVer\":\"%d.%d\"", latestIAAPacketContent.HwSwVer.hwMaj, latestIAAPacketContent.HwSwVer.hwMin);
            UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);

            memset(&uartMessage[0], 0, sizeof(uartMessage));
            sprintf(uartMessage, ",\"sensorSwVer\":\"%d.%d\"", latestIAAPacketContent.HwSwVer.swMaj, latestIAAPacketContent.HwSwVer.swMin);
            UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);

            memset(&uartMessage[0], 0, sizeof(uartMessage));
            sprintf(uartMessage, ",\"gatewayTimestampSec\":%d", AONRTCSecGet());
            UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);

            memset(&uartMessage[0], 0, sizeof(uartMessage));
            sprintf(uartMessage, ",\"gatewayHwVer\":\"%d.%d\"", SELF_HW_SW_VERSIONS.hwMaj, SELF_HW_SW_VERSIONS.hwMin);
            UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);

            memset(&uartMessage[0], 0, sizeof(uartMessage));
            sprintf(uartMessage, ",\"gatewaySwVer\":\"%d.%d\"", SELF_HW_SW_VERSIONS.swMaj, SELF_HW_SW_VERSIONS.swMin);
            UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);

            memset(&uartMessage[0], 0, sizeof(uartMessage));
            sprintf(uartMessage, "}\r\n");
            UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);
        }
    }
}

static void IAAMsgTaskFunction(UArg arg0, UArg arg1)
{
    char uartMessage[45] = "";

    while (1)
    {
        memset(&uartMessage[0], 0, sizeof(uartMessage));
        sprintf(uartMessage, "{");
        UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);

        memset(&uartMessage[0], 0, sizeof(uartMessage));
        sprintf(uartMessage, "\"addressSender\":%d", RADIO_CONCENTRATOR_ADDRESS);
        UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);

        memset(&uartMessage[0], 0, sizeof(uartMessage));
        sprintf(uartMessage, ",\"message\":\"IAA\"");
        UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);

        memset(&uartMessage[0], 0, sizeof(uartMessage));
        sprintf(uartMessage, ",\"gatewayTimestampSec\":%d", AONRTCSecGet());
        UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);

        memset(&uartMessage[0], 0, sizeof(uartMessage));
        sprintf(uartMessage, "}\r\n");
        UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);

        Task_sleep(IAA_TASK_PERIOD);
    }
}

static void packetReceivedCallback(union ConcentratorPacket* packet, int8_t rssi)
{
    uint8_t indexAxis;

    if (packet->header.packetType == RADIO_PACKET_TYPE_SENSORS_DATA_PACKET)
    {
        latestSensorsDataPacketContent.address = packet->header.sourceAddress;
        latestSensorsDataPacketContent.packetSubType = packet->sensorsDataPacket.packetSubType;
        latestSensorsDataPacketContent.packetNumber = packet->sensorsDataPacket.packetNumber;
        latestSensorsDataPacketContent.battVolt = packet->sensorsDataPacket.battVolt;
        for (indexAxis = 0; indexAxis < NMB_AXIS_IMU; ++indexAxis)
        {
            latestSensorsDataPacketContent.acc[indexAxis] = packet->sensorsDataPacket.acc[indexAxis];
            latestSensorsDataPacketContent.gyro[indexAxis] = packet->sensorsDataPacket.gyro[indexAxis];
            latestSensorsDataPacketContent.mag[indexAxis] = packet->sensorsDataPacket.mag[indexAxis];
        }
        latestSensorsDataPacketContent.temp = packet->sensorsDataPacket.temp;
        latestSensorsDataPacketContent.press = packet->sensorsDataPacket.press;
        latestSensorsDataPacketContent.lat = packet->sensorsDataPacket.lat;
        latestSensorsDataPacketContent.lon = packet->sensorsDataPacket.lon;
        latestSensorsDataPacketContent.alt = packet->sensorsDataPacket.alt;
        latestSensorsDataPacketContent.satCount = packet->sensorsDataPacket.satCount;
        latestSensorsDataPacketContent.timeUTC = packet->sensorsDataPacket.timeUTC;
        latestSensorsDataPacketContent.timestamp = packet->sensorsDataPacket.timestamp;

        latestSensorsDataPacketContent.rssi = rssi;

        Event_post(concentratorEventHandle, CONCENTRATOR_EVENT_NEW_SENSORS_DATA);
    }

    if (packet->header.packetType == RADIO_PACKET_TYPE_IAA_PACKET)
    {
        latestIAAPacketContent.address = packet->header.sourceAddress;

        latestIAAPacketContent.HwSwVer = packet->IAAPacket.HwSwVer;

        latestIAAPacketContent.timestamp = packet->IAAPacket.timestamp;

        latestIAAPacketContent.rssi = rssi;

        Event_post(concentratorEventHandle, CONCENTRATOR_EVENT_NEW_IAA);
    }
}
