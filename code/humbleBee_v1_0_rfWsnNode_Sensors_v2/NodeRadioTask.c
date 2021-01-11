/*
 * Copyright (c) 2016-2018, Texas Instruments Incorporated
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
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/UART.h>

/* Board Header files */
#include "Board.h"

/* Standard C Libraries */
#include <stdlib.h>

/* EasyLink API Header files */ 
#include "easylink/EasyLink.h"

/* Application Header files */ 
#include "common.h"
#include "RadioProtocol.h"
#include "NodeRadioTask.h"
#include "NodeTask.h"

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/aon_batmon.h)
#include DeviceFamily_constructPath(driverlib/trng.h)

#include "ble_adv/BleAdv.h"

/* GNSS parsing library */
#include "GNSS/gnss_parser.h"

/* Register reading library (for the MAC address) */
#include <hw_fcfg1.h>

/***** Type declarations *****/
struct RadioOperation
{
    EasyLink_TxPacket easyLinkTxPacket;
    uint8_t retriesDone;
    uint8_t maxNumberOfRetries;
    uint32_t ackTimeoutMs;
    enum NodeRadioOperationStatus result;
};

/***** Defines *****/
/* Task Stack Size */
#define NODERADIO_TASK_STACK_SIZE 1024

/* Task Priorities */
#define NODERADIO_TASK_PRIORITY   3

/* Events */
#define RADIO_EVENT_ALL                             0xFFFFFFFF
#define RADIO_EVENT_DATA_ACK_RECEIVED               (uint32_t)(1 << 1)
#define RADIO_EVENT_ACK_TIMEOUT                     (uint32_t)(1 << 2)
#define RADIO_EVENT_SEND_FAIL                       (uint32_t)(1 << 4)
#define RADIO_EVENT_SEND_SENSORS_DATA               (uint32_t)(1 << 8)
#define RADIO_EVENT_SEND_IAA                        (uint32_t)(1 << 16)

/* Packet Sending Retrial */
#define NODERADIO_MAX_RETRIES               0
#define NORERADIO_ACK_TIMEOUT_TIME_MS       500

/* Resending backoff time */
#define MIN_BACKOFF_TIME                    2000     //2000 usec  =  2 msec
#define MAX_BACKOFF_TIME                    10000    //10000 usec = 10 msec

/***** Variable declarations *****/
static Task_Params nodeRadioTaskParams;
Task_Struct nodeRadioTask;        /* not static so you can see in ROV */
static uint8_t nodeRadioTaskStack[NODERADIO_TASK_STACK_SIZE];

Semaphore_Struct radioAccessSem;  /* not static so you can see in ROV */
static Semaphore_Handle radioAccessSemHandle;

Event_Struct radioOperationEvent; /* not static so you can see in ROV */
static Event_Handle radioOperationEventHandle;

Semaphore_Struct radioResultSem;  /* not static so you can see in ROV */
static Semaphore_Handle radioResultSemHandle;

static struct RadioOperation currentRadioOperation;

static uint16_t packetNumberToSend;
static uint16_t battVoltToSend;
static int16_t accToSend[3];
static int16_t gyroToSend[3];
static int32_t magToSend[3];
static int16_t tempToSend;
static uint32_t pressToSend;
static locationToSend_t latToSend, lonToSend;
static uint32_t altToSend;
static uint8_t satCountToSend;
utc_time_t timeUTCToSend;
timestamp_t timestampToSend;

static uint8_t nodeAddress = 0;

/* UART handle */
extern UART_Handle uart;

extern uint8_t radio_packet_type_current;

/***** Prototypes *****/
static void nodeRadioTaskFunction(UArg arg0, UArg arg1);
static void returnRadioOperationStatus(enum NodeRadioOperationStatus status);
static void sendSensorsDataPacket(struct SensorsDataPacket sensorsDataPacketToSend, uint8_t maxNumberOfRetries, uint32_t ackTimeoutMs);
static void sendIAAPacket(struct IAAPacket IAAPacketToSend, uint8_t maxNumberOfRetries, uint32_t ackTimeoutMs);
static void resendPacket(void);
static void rxDoneCallback(EasyLink_RxPacket * rxPacket, EasyLink_Status status);

/***** Function definitions *****/
void NodeRadioTask_init(void) {

    /* Create semaphore used for exclusive radio access */
    Semaphore_Params semParam;
    Semaphore_Params_init(&semParam);
    Semaphore_construct(&radioAccessSem, 1, &semParam);
    radioAccessSemHandle = Semaphore_handle(&radioAccessSem);

    /* Create semaphore used for callers to wait for result */
    Semaphore_construct(&radioResultSem, 0, &semParam);
    radioResultSemHandle = Semaphore_handle(&radioResultSem);

    /* Create event used internally for state changes */
    Event_Params eventParam;
    Event_Params_init(&eventParam);
    Event_construct(&radioOperationEvent, &eventParam);
    radioOperationEventHandle = Event_handle(&radioOperationEvent);

    /* Create the radio protocol task */
    Task_Params_init(&nodeRadioTaskParams);
    nodeRadioTaskParams.stackSize = NODERADIO_TASK_STACK_SIZE;
    nodeRadioTaskParams.priority = NODERADIO_TASK_PRIORITY;
    nodeRadioTaskParams.stack = &nodeRadioTaskStack;
    Task_construct(&nodeRadioTask, nodeRadioTaskFunction, &nodeRadioTaskParams, NULL);
}

uint8_t nodeRadioTask_getNodeAddr(void)
{
    return nodeAddress;
}

static void nodeRadioTaskFunction(UArg arg0, UArg arg1)
{
    uint8_t indexAxis;
    uint32_t backoffTime;
    EasyLink_Params easyLink_params;
    EasyLink_Params_init(&easyLink_params);
    //char uartMessage[45] = "";

    easyLink_params.ui32ModType = RADIO_EASYLINK_MODULATION;
    /* Initialize EasyLink */
    if(EasyLink_init(&easyLink_params) != EasyLink_Status_Success){
        System_abort("EasyLink_init failed");
    }

    /* If you wich to use a frequency other than the default use
     * the below API
     * EasyLink_setFrequency(868000000);
     */

    /* Generate the sensor node address */
    // Based on the MAC address
    nodeAddress = (uint8_t)(HWREG(FCFG1_BASE + FCFG1_O_MAC_15_4_0) & 0xFF);

    // Use the True Random Number Generator to generate sensor node address randomly
    /*
    Power_setDependency(PowerCC26XX_PERIPH_TRNG);
    TRNGEnable();
    // Do not accept the same address as the concentrator, in that case get a new random value
    do
    {
        while (!(TRNGStatusGet() & TRNG_NUMBER_READY))
        {
            //wait for random number generator
        }
        nodeAddress = (uint8_t)TRNGNumberGet(TRNG_LOW_WORD);
    } while (nodeAddress == RADIO_CONCENTRATOR_ADDRESS);
    TRNGDisable();
    Power_releaseDependency(PowerCC26XX_PERIPH_TRNG);
    */

    /* Set the filter to the generated random address */
    if (EasyLink_enableRxAddrFilter(&nodeAddress, 1, 1) != EasyLink_Status_Success)
    {
        System_abort("EasyLink_enableRxAddrFilter failed");
    }

    /* Enter main task loop */
    while (1)
    {
        /* Wait for an event */
        uint32_t events = Event_pend(radioOperationEventHandle, 0, RADIO_EVENT_ALL, BIOS_WAIT_FOREVER);

        if (events & RADIO_EVENT_SEND_SENSORS_DATA)
        {
            static struct SensorsDataPacket sensorsDataPacketToSend;

            sensorsDataPacketToSend.header.sourceAddress = nodeAddress;
            sensorsDataPacketToSend.header.packetType = RADIO_PACKET_TYPE_SENSORS_DATA_PACKET;
            sensorsDataPacketToSend.packetSubType = radio_packet_type_current;
            sensorsDataPacketToSend.packetNumber = packetNumberToSend;
            sensorsDataPacketToSend.timestamp = timestampToSend;

            if (radio_packet_type_current == 1 || radio_packet_type_current == 0)
            {
                for (indexAxis = 0; indexAxis < NMB_AXIS_IMU; ++indexAxis)
                {
                    sensorsDataPacketToSend.acc[indexAxis] = accToSend[indexAxis];
                    sensorsDataPacketToSend.gyro[indexAxis] = gyroToSend[indexAxis];
                }
            }

            if (radio_packet_type_current == 2 || radio_packet_type_current == 0)
            {
                for (indexAxis = 0; indexAxis < NMB_AXIS_IMU; ++indexAxis)
                {
                    sensorsDataPacketToSend.mag[indexAxis] = magToSend[indexAxis];
                }
            }

            if (radio_packet_type_current == 3 || radio_packet_type_current == 0)
            {
                sensorsDataPacketToSend.battVolt = battVoltToSend;
                sensorsDataPacketToSend.temp = tempToSend;
                sensorsDataPacketToSend.press = pressToSend;
                sensorsDataPacketToSend.satCount = satCountToSend;
                sensorsDataPacketToSend.timeUTC = timeUTCToSend;
            }

            if (radio_packet_type_current == 4 || radio_packet_type_current == 0)
            {
                sensorsDataPacketToSend.lat = latToSend;
                sensorsDataPacketToSend.lon = lonToSend;
                sensorsDataPacketToSend.alt = altToSend;
            }

            sendSensorsDataPacket(sensorsDataPacketToSend, NODERADIO_MAX_RETRIES, NORERADIO_ACK_TIMEOUT_TIME_MS);
        }

        if (events & RADIO_EVENT_SEND_IAA)
        {
            static struct IAAPacket IAAPacketToSend;

            IAAPacketToSend.header.sourceAddress = nodeAddress;
            IAAPacketToSend.header.packetType = RADIO_PACKET_TYPE_IAA_PACKET;

            IAAPacketToSend.HwSwVer = SELF_HW_SW_VERSIONS;

            IAAPacketToSend.timestamp = timestampToSend;

            sendIAAPacket(IAAPacketToSend, NODERADIO_MAX_RETRIES, NORERADIO_ACK_TIMEOUT_TIME_MS);
        }

         /* If we get an ACK from the concentrator */
        if (events & RADIO_EVENT_DATA_ACK_RECEIVED)
        {
            returnRadioOperationStatus(NodeRadioStatus_Success);

            /*
            memset(&uartMessage[0], 0, sizeof(uartMessage));
            sprintf(uartMessage, "ACK received %d!\r\n", packetNumberToSend);
            UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);
            */
        }

        /* If we get an ACK timeout */
        if (events & RADIO_EVENT_ACK_TIMEOUT)
        {
            /* If we haven't resent it the maximum number of times yet, then resend packet */
            if (currentRadioOperation.retriesDone < currentRadioOperation.maxNumberOfRetries)
            {
                /*
                memset(&uartMessage[0], 0, sizeof(uartMessage));
                sprintf(uartMessage, "Resend %d!\r\n", packetNumberToSend);
                UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);
                */

                // Wait a random Backoff Time before resending
                Power_setDependency(PowerCC26XX_PERIPH_TRNG);
                TRNGEnable();
                while (!(TRNGStatusGet() & TRNG_NUMBER_READY))
                {
                    //wait for random number generator
                }
                backoffTime = (TRNGNumberGet(TRNG_LOW_WORD) % (MAX_BACKOFF_TIME - MIN_BACKOFF_TIME)) + MIN_BACKOFF_TIME;
                TRNGDisable();
                Power_releaseDependency(PowerCC26XX_PERIPH_TRNG);

                Task_sleep(backoffTime);

                resendPacket();
            }
            else
            {
                /* Else return send fail */
                Event_post(radioOperationEventHandle, RADIO_EVENT_SEND_FAIL);
            }
        }

        /* If send fail */
        if (events & RADIO_EVENT_SEND_FAIL)
        {
            returnRadioOperationStatus(NodeRadioStatus_Failed);
        }
    }
}

enum NodeRadioOperationStatus NodeRadioTask_sendSensorsData(uint8_t packet_type, uint16_t packetNumber, uint16_t battVolt, int16_t acc[], int16_t gyro[], int32_t mag[], int16_t temp, uint32_t press, locationToSend_t lat, locationToSend_t lon, int32_t alt, uint8_t satCount, utc_time_t timeUTC, timestamp_t timestamp)
{
    enum NodeRadioOperationStatus status;
    uint8_t indexAxis;

    /* Get radio access semaphore */
    Semaphore_pend(radioAccessSemHandle, BIOS_WAIT_FOREVER);

    radio_packet_type_current = packet_type;

    /* Save data to send */
    packetNumberToSend = packetNumber;
    timestampToSend = timestamp;

    if (radio_packet_type_current == 1 || radio_packet_type_current == 0)
    {
        for (indexAxis = 0; indexAxis < NMB_AXIS_IMU; ++indexAxis)
        {
            accToSend[indexAxis] = acc[indexAxis];
            gyroToSend[indexAxis] = gyro[indexAxis];
        }
    }

    if (radio_packet_type_current == 2 || radio_packet_type_current == 0)
    {
        for (indexAxis = 0; indexAxis < NMB_AXIS_IMU; ++indexAxis)
        {
            magToSend[indexAxis] = mag[indexAxis];
        }
    }

    if (radio_packet_type_current == 3 || radio_packet_type_current == 0)
    {
        battVoltToSend = battVolt;
        tempToSend = temp;
        pressToSend = press;
        satCountToSend = satCount;
        timeUTCToSend = timeUTC;
    }

    if (radio_packet_type_current == 4 || radio_packet_type_current == 0)
    {
        latToSend = lat;
        lonToSend = lon;
        altToSend = alt;
    }

    /* Raise RADIO_EVENT_SEND_BATTVOLT_DATA event */
    Event_post(radioOperationEventHandle, RADIO_EVENT_SEND_SENSORS_DATA);

    /* Wait for result */
    Semaphore_pend(radioResultSemHandle, BIOS_WAIT_FOREVER);

    /* Get result */
    status = currentRadioOperation.result;

    /* Return radio access semaphore */
    Semaphore_post(radioAccessSemHandle);

    return status;
}

enum NodeRadioOperationStatus NodeRadioTask_sendIAA(timestamp_t timestamp)
{
    enum NodeRadioOperationStatus status;

    /* Get radio access semaphore */
    Semaphore_pend(radioAccessSemHandle, BIOS_WAIT_FOREVER);

    /* Save data to send */
    timestampToSend = timestamp;

    /* Raise RADIO_EVENT_SEND_BATTVOLT_DATA event */
    Event_post(radioOperationEventHandle, RADIO_EVENT_SEND_IAA);

    /* Wait for result */
    Semaphore_pend(radioResultSemHandle, BIOS_WAIT_FOREVER);

    /* Get result */
    status = currentRadioOperation.result;

    /* Return radio access semaphore */
    Semaphore_post(radioAccessSemHandle);

    return status;
}


static void returnRadioOperationStatus(enum NodeRadioOperationStatus result)
{
    /* Save result */
    currentRadioOperation.result = result;

    /* Post result semaphore */
    Semaphore_post(radioResultSemHandle);
}

static void sendSensorsDataPacket(struct SensorsDataPacket sensorsDataPacketToSend, uint8_t maxNumberOfRetries, uint32_t ackTimeoutMs)
{
    uint8_t indexAxis;

    /* Set destination address in EasyLink API */
     currentRadioOperation.easyLinkTxPacket.dstAddr[0] = RADIO_CONCENTRATOR_ADDRESS;

     /* Copy packet to payload
      * Note that the EasyLink API will implcitily both add the length byte and the destination address byte. */
     currentRadioOperation.easyLinkTxPacket.payload[0] = sensorsDataPacketToSend.header.sourceAddress;
     currentRadioOperation.easyLinkTxPacket.payload[1] = sensorsDataPacketToSend.header.packetType;
     currentRadioOperation.easyLinkTxPacket.payload[2] = sensorsDataPacketToSend.packetSubType;

     currentRadioOperation.easyLinkTxPacket.payload[3] = (sensorsDataPacketToSend.packetNumber & 0xFF00) >> 8;
     currentRadioOperation.easyLinkTxPacket.payload[4] = (sensorsDataPacketToSend.packetNumber & 0xFF);

     currentRadioOperation.easyLinkTxPacket.payload[5] = (sensorsDataPacketToSend.timestamp.sec & 0xFF000000) >> 24;
     currentRadioOperation.easyLinkTxPacket.payload[6] = (sensorsDataPacketToSend.timestamp.sec & 0x00FF0000) >> 16;
     currentRadioOperation.easyLinkTxPacket.payload[7] = (sensorsDataPacketToSend.timestamp.sec & 0x0000FF00) >> 8;
     currentRadioOperation.easyLinkTxPacket.payload[8] = (sensorsDataPacketToSend.timestamp.sec & 0x000000FF);

     currentRadioOperation.easyLinkTxPacket.len = sizeof(sensorsDataPacketToSend.header) + sizeof(sensorsDataPacketToSend.packetSubType) + sizeof(sensorsDataPacketToSend.packetNumber) + sizeof(sensorsDataPacketToSend.timestamp);

     if (sensorsDataPacketToSend.packetSubType == 1)
     {
         for (indexAxis = 0; indexAxis < NMB_AXIS_IMU; ++indexAxis)
         {
            currentRadioOperation.easyLinkTxPacket.payload[9 + indexAxis * 4] = (sensorsDataPacketToSend.acc[indexAxis] & 0xFF00) >> 8;
            currentRadioOperation.easyLinkTxPacket.payload[10 + indexAxis * 4] = (sensorsDataPacketToSend.acc[indexAxis] & 0xFF);

            currentRadioOperation.easyLinkTxPacket.payload[11 + indexAxis * 4] = (sensorsDataPacketToSend.gyro[indexAxis] & 0xFF00) >> 8;
            currentRadioOperation.easyLinkTxPacket.payload[12 + indexAxis * 4] = (sensorsDataPacketToSend.gyro[indexAxis] & 0xFF);
         }

         currentRadioOperation.easyLinkTxPacket.len += 12;
     }

     if (sensorsDataPacketToSend.packetSubType == 2)
     {
         for (indexAxis = 0; indexAxis < NMB_AXIS_IMU; ++indexAxis)
         {
            currentRadioOperation.easyLinkTxPacket.payload[9 + indexAxis * 4] = (sensorsDataPacketToSend.mag[indexAxis] & 0xFF000000) >> 24;
            currentRadioOperation.easyLinkTxPacket.payload[10 + indexAxis * 4] = (sensorsDataPacketToSend.mag[indexAxis] & 0x00FF0000) >> 16;
            currentRadioOperation.easyLinkTxPacket.payload[11 + indexAxis * 4] = (sensorsDataPacketToSend.mag[indexAxis] & 0x0000FF00) >> 8;
            currentRadioOperation.easyLinkTxPacket.payload[12 + indexAxis * 4] = (sensorsDataPacketToSend.mag[indexAxis] & 0x000000FF);
         }

         currentRadioOperation.easyLinkTxPacket.len += 12;
     }

     if (sensorsDataPacketToSend.packetSubType == 3)
     {
         currentRadioOperation.easyLinkTxPacket.payload[9] = (sensorsDataPacketToSend.battVolt & 0xFF00) >> 8;
         currentRadioOperation.easyLinkTxPacket.payload[10] = (sensorsDataPacketToSend.battVolt & 0xFF);

         currentRadioOperation.easyLinkTxPacket.payload[11] = (sensorsDataPacketToSend.temp & 0xFF00) >> 8;
         currentRadioOperation.easyLinkTxPacket.payload[12] = (sensorsDataPacketToSend.temp & 0xFF);

         currentRadioOperation.easyLinkTxPacket.payload[13] = (sensorsDataPacketToSend.press & 0xFF000000) >> 24;
         currentRadioOperation.easyLinkTxPacket.payload[14] = (sensorsDataPacketToSend.press & 0x00FF0000) >> 16;
         currentRadioOperation.easyLinkTxPacket.payload[15] = (sensorsDataPacketToSend.press & 0x0000FF00) >> 8;
         currentRadioOperation.easyLinkTxPacket.payload[16] = (sensorsDataPacketToSend.press & 0x000000FF);

         currentRadioOperation.easyLinkTxPacket.payload[17] = sensorsDataPacketToSend.satCount;

         currentRadioOperation.easyLinkTxPacket.payload[18] = sensorsDataPacketToSend.timeUTC.hour;
         currentRadioOperation.easyLinkTxPacket.payload[19] = sensorsDataPacketToSend.timeUTC.minute;
         currentRadioOperation.easyLinkTxPacket.payload[20] = sensorsDataPacketToSend.timeUTC.second;

         currentRadioOperation.easyLinkTxPacket.len += 12;
     }

     if (sensorsDataPacketToSend.packetSubType == 4)
     {
         currentRadioOperation.easyLinkTxPacket.payload[9] = sensorsDataPacketToSend.lat.degrees;
         currentRadioOperation.easyLinkTxPacket.payload[10] = (sensorsDataPacketToSend.lat.minutes & 0xFF00) >> 8;
         currentRadioOperation.easyLinkTxPacket.payload[11] = (sensorsDataPacketToSend.lat.minutes & 0xFF);
         currentRadioOperation.easyLinkTxPacket.payload[12] = sensorsDataPacketToSend.lat.azmuth;

         currentRadioOperation.easyLinkTxPacket.payload[13] = sensorsDataPacketToSend.lon.degrees;
         currentRadioOperation.easyLinkTxPacket.payload[14] = (sensorsDataPacketToSend.lon.minutes & 0xFF00) >> 8;
         currentRadioOperation.easyLinkTxPacket.payload[15] = (sensorsDataPacketToSend.lon.minutes & 0xFF);
         currentRadioOperation.easyLinkTxPacket.payload[16] = sensorsDataPacketToSend.lon.azmuth;

         currentRadioOperation.easyLinkTxPacket.payload[17] = (sensorsDataPacketToSend.alt & 0xFF000000) >> 24;
         currentRadioOperation.easyLinkTxPacket.payload[18] = (sensorsDataPacketToSend.alt & 0x00FF0000) >> 16;
         currentRadioOperation.easyLinkTxPacket.payload[19] = (sensorsDataPacketToSend.alt & 0x0000FF00) >> 8;
         currentRadioOperation.easyLinkTxPacket.payload[20] = (sensorsDataPacketToSend.alt & 0x000000FF);

         currentRadioOperation.easyLinkTxPacket.len += 12;
     }

     if (sensorsDataPacketToSend.packetSubType == 0)
     {
         currentRadioOperation.easyLinkTxPacket.payload[9] = (sensorsDataPacketToSend.battVolt & 0xFF00) >> 8;
         currentRadioOperation.easyLinkTxPacket.payload[10] = (sensorsDataPacketToSend.battVolt & 0xFF);

         for (indexAxis = 0; indexAxis < NMB_AXIS_IMU; ++indexAxis)
         {
            currentRadioOperation.easyLinkTxPacket.payload[11 + indexAxis * 8] = (sensorsDataPacketToSend.acc[indexAxis] & 0xFF00) >> 8;
            currentRadioOperation.easyLinkTxPacket.payload[12 + indexAxis * 8] = (sensorsDataPacketToSend.acc[indexAxis] & 0xFF);

            currentRadioOperation.easyLinkTxPacket.payload[13 + indexAxis * 8] = (sensorsDataPacketToSend.gyro[indexAxis] & 0xFF00) >> 8;
            currentRadioOperation.easyLinkTxPacket.payload[14 + indexAxis * 8] = (sensorsDataPacketToSend.gyro[indexAxis] & 0xFF);

            currentRadioOperation.easyLinkTxPacket.payload[15 + indexAxis * 8] = (sensorsDataPacketToSend.mag[indexAxis] & 0xFF000000) >> 24;
            currentRadioOperation.easyLinkTxPacket.payload[16 + indexAxis * 8] = (sensorsDataPacketToSend.mag[indexAxis] & 0x00FF0000) >> 16;
            currentRadioOperation.easyLinkTxPacket.payload[17 + indexAxis * 8] = (sensorsDataPacketToSend.mag[indexAxis] & 0x0000FF00) >> 8;
            currentRadioOperation.easyLinkTxPacket.payload[18 + indexAxis * 8] = (sensorsDataPacketToSend.mag[indexAxis] & 0x000000FF);
         }

         currentRadioOperation.easyLinkTxPacket.payload[35] = (sensorsDataPacketToSend.temp & 0xFF00) >> 8;
         currentRadioOperation.easyLinkTxPacket.payload[36] = (sensorsDataPacketToSend.temp & 0xFF);

         currentRadioOperation.easyLinkTxPacket.payload[37] = (sensorsDataPacketToSend.press & 0xFF000000) >> 24;
         currentRadioOperation.easyLinkTxPacket.payload[38] = (sensorsDataPacketToSend.press & 0x00FF0000) >> 16;
         currentRadioOperation.easyLinkTxPacket.payload[39] = (sensorsDataPacketToSend.press & 0x0000FF00) >> 8;
         currentRadioOperation.easyLinkTxPacket.payload[40] = (sensorsDataPacketToSend.press & 0x000000FF);

         currentRadioOperation.easyLinkTxPacket.payload[41] = sensorsDataPacketToSend.lat.degrees;
         currentRadioOperation.easyLinkTxPacket.payload[42] = (sensorsDataPacketToSend.lat.minutes & 0xFF00) >> 8;
         currentRadioOperation.easyLinkTxPacket.payload[43] = (sensorsDataPacketToSend.lat.minutes & 0xFF);
         currentRadioOperation.easyLinkTxPacket.payload[44] = sensorsDataPacketToSend.lat.azmuth;

         currentRadioOperation.easyLinkTxPacket.payload[45] = sensorsDataPacketToSend.lon.degrees;
         currentRadioOperation.easyLinkTxPacket.payload[46] = (sensorsDataPacketToSend.lon.minutes & 0xFF00) >> 8;
         currentRadioOperation.easyLinkTxPacket.payload[47] = (sensorsDataPacketToSend.lon.minutes & 0xFF);
         currentRadioOperation.easyLinkTxPacket.payload[48] = sensorsDataPacketToSend.lon.azmuth;

         currentRadioOperation.easyLinkTxPacket.payload[49] = (sensorsDataPacketToSend.alt & 0xFF000000) >> 24;
         currentRadioOperation.easyLinkTxPacket.payload[50] = (sensorsDataPacketToSend.alt & 0x00FF0000) >> 16;
         currentRadioOperation.easyLinkTxPacket.payload[51] = (sensorsDataPacketToSend.alt & 0x0000FF00) >> 8;
         currentRadioOperation.easyLinkTxPacket.payload[52] = (sensorsDataPacketToSend.alt & 0x000000FF);

         currentRadioOperation.easyLinkTxPacket.payload[53] = sensorsDataPacketToSend.satCount;

         currentRadioOperation.easyLinkTxPacket.payload[54] = sensorsDataPacketToSend.timeUTC.hour;
         currentRadioOperation.easyLinkTxPacket.payload[55] = sensorsDataPacketToSend.timeUTC.minute;
         currentRadioOperation.easyLinkTxPacket.payload[56] = sensorsDataPacketToSend.timeUTC.second;

         currentRadioOperation.easyLinkTxPacket.len = sizeof(struct SensorsDataPacket);
     }

     /* Setup retries */
     currentRadioOperation.maxNumberOfRetries = maxNumberOfRetries;
     currentRadioOperation.ackTimeoutMs = ackTimeoutMs;
     currentRadioOperation.retriesDone = 0;
     EasyLink_setCtrl(EasyLink_Ctrl_AsyncRx_TimeOut, EasyLink_ms_To_RadioTime(ackTimeoutMs));

     /* Send packet  */
     if (EasyLink_transmit(&currentRadioOperation.easyLinkTxPacket) != EasyLink_Status_Success)
     {
         System_abort("EasyLink_transmit failed");
     }
 #if defined(Board_DIO30_SWPWR)
     /* this was a blocking call, so Tx is now complete. Turn off the RF switch power */
     PIN_setOutputValue(blePinHandle, Board_DIO30_SWPWR, 0);
 #endif

     /* Enter RX */
     if (EasyLink_receiveAsync(rxDoneCallback, 0) != EasyLink_Status_Success)
     {
         System_abort("EasyLink_receiveAsync failed");
     }
}

static void sendIAAPacket(struct IAAPacket IAAPacketToSend, uint8_t maxNumberOfRetries, uint32_t ackTimeoutMs)
{
    /* Set destination address in EasyLink API */
     currentRadioOperation.easyLinkTxPacket.dstAddr[0] = RADIO_CONCENTRATOR_ADDRESS;

     /* Copy packet to payload
      * Note that the EasyLink API will implcitily both add the length byte and the destination address byte. */
     currentRadioOperation.easyLinkTxPacket.payload[0] = IAAPacketToSend.header.sourceAddress;
     currentRadioOperation.easyLinkTxPacket.payload[1] = IAAPacketToSend.header.packetType;

     currentRadioOperation.easyLinkTxPacket.payload[2] = IAAPacketToSend.HwSwVer.hwMaj;
     currentRadioOperation.easyLinkTxPacket.payload[3] = IAAPacketToSend.HwSwVer.hwMin;
     currentRadioOperation.easyLinkTxPacket.payload[4] = IAAPacketToSend.HwSwVer.swMaj;
     currentRadioOperation.easyLinkTxPacket.payload[5] = IAAPacketToSend.HwSwVer.swMin;

     currentRadioOperation.easyLinkTxPacket.payload[6] = (IAAPacketToSend.timestamp.sec & 0xFF000000) >> 24;
     currentRadioOperation.easyLinkTxPacket.payload[7] = (IAAPacketToSend.timestamp.sec & 0x00FF0000) >> 16;
     currentRadioOperation.easyLinkTxPacket.payload[8] = (IAAPacketToSend.timestamp.sec & 0x0000FF00) >> 8;
     currentRadioOperation.easyLinkTxPacket.payload[9] = (IAAPacketToSend.timestamp.sec & 0x000000FF);

     currentRadioOperation.easyLinkTxPacket.len = sizeof(struct IAAPacket);

     /* Setup retries */
     currentRadioOperation.maxNumberOfRetries = maxNumberOfRetries;
     currentRadioOperation.ackTimeoutMs = ackTimeoutMs;
     currentRadioOperation.retriesDone = 0;
     EasyLink_setCtrl(EasyLink_Ctrl_AsyncRx_TimeOut, EasyLink_ms_To_RadioTime(ackTimeoutMs));

     /* Send packet  */
     if (EasyLink_transmit(&currentRadioOperation.easyLinkTxPacket) != EasyLink_Status_Success)
     {
         System_abort("EasyLink_transmit failed");
     }
 #if defined(Board_DIO30_SWPWR)
     /* this was a blocking call, so Tx is now complete. Turn off the RF switch power */
     PIN_setOutputValue(blePinHandle, Board_DIO30_SWPWR, 0);
 #endif

     /* Enter RX */
     if (EasyLink_receiveAsync(rxDoneCallback, 0) != EasyLink_Status_Success)
     {
         System_abort("EasyLink_receiveAsync failed");
     }
}

static void resendPacket(void)
{
    /* Send packet  */
    if (EasyLink_transmit(&currentRadioOperation.easyLinkTxPacket) != EasyLink_Status_Success)
    {
        System_abort("EasyLink_transmit failed");
    }
#if defined(Board_DIO30_SWPWR)
    /* this was a blocking call, so Tx is now complete. Turn off the RF switch power */
    PIN_setOutputValue(blePinHandle, Board_DIO30_SWPWR, 0);
#endif

    /* Enter RX and wait for ACK with timeout */
    if (EasyLink_receiveAsync(rxDoneCallback, 0) != EasyLink_Status_Success)
    {
        System_abort("EasyLink_receiveAsync failed");
    }

    /* Increase retries by one */
    currentRadioOperation.retriesDone++;
}

static void rxDoneCallback(EasyLink_RxPacket * rxPacket, EasyLink_Status status)
{
    struct PacketHeader* packetHeader;

#if defined(Board_DIO30_SWPWR)
    /* Rx is now complete. Turn off the RF switch power */
    PIN_setOutputValue(blePinHandle, Board_DIO30_SWPWR, 0);
#endif

    /* If this callback is called because of a packet received */
    if (status == EasyLink_Status_Success)
    {
        /* Check the payload header */
        packetHeader = (struct PacketHeader*)rxPacket->payload;

        /* Check if this is an ACK packet */
        if (packetHeader->packetType == RADIO_PACKET_TYPE_ACK_PACKET)
        {
            /* Signal ACK packet received */
            Event_post(radioOperationEventHandle, RADIO_EVENT_DATA_ACK_RECEIVED);
        }
        else
        {
            /* Packet Error, treat as a Timeout and Post a RADIO_EVENT_ACK_TIMEOUT
               event */
            Event_post(radioOperationEventHandle, RADIO_EVENT_ACK_TIMEOUT);
        }
    }
    /* did the Rx timeout */
    else if(status == EasyLink_Status_Rx_Timeout)
    {
        /* Post a RADIO_EVENT_ACK_TIMEOUT event */
        Event_post(radioOperationEventHandle, RADIO_EVENT_ACK_TIMEOUT);
    }
    else
    {
        /* The Ack receiption may have been corrupted causing an error.
         * Treat this as a timeout
         */
        Event_post(radioOperationEventHandle, RADIO_EVENT_ACK_TIMEOUT);
    }
}
