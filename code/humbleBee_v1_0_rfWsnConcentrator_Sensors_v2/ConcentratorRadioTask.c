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

#include "ConcentratorRadioTask.h"

/* BIOS Header files */ 
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>

/* Drivers */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>

/* Board Header files */
#include "Board.h"

/* EasyLink API Header files */ 
#include "easylink/EasyLink.h"

/* Application Header files */ 
#include "RadioProtocol.h"
#include "ConcentratorTask.h"

/* GNSS parsing library */
#include "GNSS/gnss_parser.h"

/* Real Time Clock library */
#include <ti/devices/cc13x0/driverlib/aon_rtc.h>

/***** Defines *****/
#define CONCENTRATORRADIO_TASK_STACK_SIZE 1024
#define CONCENTRATORRADIO_TASK_PRIORITY   3

#define RADIO_EVENT_ALL                         0xFFFFFFFF
#define RADIO_EVENT_VALID_PACKET_RECEIVED       (uint32_t)(1 << 0)
#define RADIO_EVENT_INVALID_PACKET_RECEIVED     (uint32_t)(1 << 1)
#define RADIO_EVENT_UNKNOWN_PACKET_RECEIVED     (uint32_t)(1 << 2)

#define CONCENTRATORRADIO_MAX_RETRIES           2
#define NORERADIO_ACK_TIMEOUT_TIME_MS           16 //(160)


/***** Variable declarations *****/
static Task_Params concentratorRadioTaskParams;
Task_Struct concentratorRadioTask; /* not static so you can see in ROV */
static uint8_t concentratorRadioTaskStack[CONCENTRATORRADIO_TASK_STACK_SIZE];
Event_Struct radioOperationEvent;  /* not static so you can see in ROV */
static Event_Handle radioOperationEventHandle;


static ConcentratorRadio_PacketReceivedCallback packetReceivedCallback;
static union ConcentratorPacket latestRxPacket;
static EasyLink_TxPacket txPacket;
static struct AckPacket ackPacket;
static uint8_t concentratorAddress;
static int8_t latestRssi;

/* UART handle */
extern UART_Handle uart;


/***** Prototypes *****/
static void concentratorRadioTaskFunction(UArg arg0, UArg arg1);
static void rxDoneCallback(EasyLink_RxPacket * rxPacket, EasyLink_Status status);
static void notifyPacketReceived(union ConcentratorPacket* latestRxPacket);
static void sendAck(uint8_t latestSourceAddress);


/***** Function definitions *****/
void ConcentratorRadioTask_init(void)
{
    /* Create event used internally for state changes */
    Event_Params eventParam;
    Event_Params_init(&eventParam);
    Event_construct(&radioOperationEvent, &eventParam);
    radioOperationEventHandle = Event_handle(&radioOperationEvent);

    /* Create the concentrator radio protocol task */
    Task_Params_init(&concentratorRadioTaskParams);
    concentratorRadioTaskParams.stackSize = CONCENTRATORRADIO_TASK_STACK_SIZE;
    concentratorRadioTaskParams.priority = CONCENTRATORRADIO_TASK_PRIORITY;
    concentratorRadioTaskParams.stack = &concentratorRadioTaskStack;
    Task_construct(&concentratorRadioTask, concentratorRadioTaskFunction, &concentratorRadioTaskParams, NULL);
}

void ConcentratorRadioTask_registerPacketReceivedCallback(ConcentratorRadio_PacketReceivedCallback callback)
{
    packetReceivedCallback = callback;
}

static void concentratorRadioTaskFunction(UArg arg0, UArg arg1)
{
    char uartMessage[45] = "";

    /* Initialize EasyLink */
	EasyLink_Params easyLink_params;
    EasyLink_Params_init(&easyLink_params);
	
	easyLink_params.ui32ModType = RADIO_EASYLINK_MODULATION;
	
	if(EasyLink_init(&easyLink_params) != EasyLink_Status_Success)
	{
		System_abort("EasyLink_init failed");
	}	

    /* If you wich to use a frequency other than the default use
     * the below API
     * EasyLink_setFrequency(868000000);
     */

    /* Set concentrator address */;
    concentratorAddress = RADIO_CONCENTRATOR_ADDRESS;
    EasyLink_enableRxAddrFilter(&concentratorAddress, 1, 1);

    /* Set up Ack packet */
    ackPacket.header.sourceAddress = concentratorAddress;
    ackPacket.header.packetType = RADIO_PACKET_TYPE_ACK_PACKET;

    /* Enter receive */
    if(EasyLink_receiveAsync(rxDoneCallback, 0) != EasyLink_Status_Success)
    {
        System_abort("EasyLink_receiveAsync failed");
    }

    while (1)
    {
        uint32_t events = Event_pend(radioOperationEventHandle, 0, RADIO_EVENT_ALL, BIOS_WAIT_FOREVER);

        /* If valid packet received */
        if(events & RADIO_EVENT_VALID_PACKET_RECEIVED)
        {
            /* Send ack packet */
            sendAck(latestRxPacket.header.sourceAddress);

            /* Call packet received callback */
            notifyPacketReceived(&latestRxPacket);

            /* Go back to RX */
            if(EasyLink_receiveAsync(rxDoneCallback, 0) != EasyLink_Status_Success)
            {
                System_abort("EasyLink_receiveAsync failed");
            }

            /* toggle Activity LED */
            GPIO_toggle(Board_GPIO_LED0);
        }

        /* If invalid packet received */
        if(events & RADIO_EVENT_INVALID_PACKET_RECEIVED)
        {
            memset(&uartMessage[0], 0, sizeof(uartMessage));
            sprintf(uartMessage, "{");
            UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);

            memset(&uartMessage[0], 0, sizeof(uartMessage));
            sprintf(uartMessage, "\"addressSender\":%d", RADIO_CONCENTRATOR_ADDRESS);
            UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);

            memset(&uartMessage[0], 0, sizeof(uartMessage));
            sprintf(uartMessage, ",\"message\":\"Invalid_Packet_Received\"");
            UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);

            memset(&uartMessage[0], 0, sizeof(uartMessage));
            sprintf(uartMessage, ",\"gatewayTimestampSec\":%d", AONRTCSecGet());
            UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);

            memset(&uartMessage[0], 0, sizeof(uartMessage));
            sprintf(uartMessage, "}\r\n");
            UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);

            /* toggle Invalid Packet LED */
            GPIO_toggle(Board_GPIO_LED1);

            /* Go back to RX */
            if(EasyLink_receiveAsync(rxDoneCallback, 0) != EasyLink_Status_Success)
            {
                System_abort("EasyLink_receiveAsync failed");
            }
        }

        if (events & RADIO_EVENT_UNKNOWN_PACKET_RECEIVED)
        {
            memset(&uartMessage[0], 0, sizeof(uartMessage));
            sprintf(uartMessage, "{");
            UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);

            memset(&uartMessage[0], 0, sizeof(uartMessage));
            sprintf(uartMessage, "\"addressSender\":%d", RADIO_CONCENTRATOR_ADDRESS);
            UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);

            memset(&uartMessage[0], 0, sizeof(uartMessage));
            sprintf(uartMessage, ",\"message\":\"Unknown_Packet_Received\"");
            UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);

            memset(&uartMessage[0], 0, sizeof(uartMessage));
            sprintf(uartMessage, ",\"gatewayTimestampSec\":%d", AONRTCSecGet());
            UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);

            memset(&uartMessage[0], 0, sizeof(uartMessage));
            sprintf(uartMessage, "}\r\n");
            UART_write(uart, uartMessage, strchr(uartMessage, 0) - uartMessage);

            /* toggle Invalid Packet LED */
            GPIO_toggle(Board_GPIO_LED1);

            /* Go back to RX */
            if(EasyLink_receiveAsync(rxDoneCallback, 0) != EasyLink_Status_Success)
            {
                System_abort("EasyLink_receiveAsync failed");
            }
        }
    }
}

static void sendAck(uint8_t latestSourceAddress) {

    /* Set destinationAdress, but use EasyLink layers destination adress capability */
    txPacket.dstAddr[0] = latestSourceAddress;

    /* Copy ACK packet to payload, skipping the destination adress byte.
     * Note that the EasyLink API will implcitily both add the length byte and the destination address byte. */
    memcpy(txPacket.payload, &ackPacket.header, sizeof(ackPacket));
    txPacket.len = sizeof(ackPacket);

    /* Send packet  */
    if (EasyLink_transmit(&txPacket) != EasyLink_Status_Success)
    {
        System_abort("EasyLink_transmit failed");
    }
}

static void notifyPacketReceived(union ConcentratorPacket* latestRxPacket)
{
    if (packetReceivedCallback)
    {
        packetReceivedCallback(latestRxPacket, latestRssi);
    }
}

static void rxDoneCallback(EasyLink_RxPacket * rxPacket, EasyLink_Status status)
{
    uint8_t indexAxis;
    union ConcentratorPacket* tmpRxPacket;

    /* If we received a packet successfully */
    if (status == EasyLink_Status_Success)
    {
        /* Save the latest RSSI, which is later sent to the receive callback */
        latestRssi = (int8_t)rxPacket->rssi;

        /* Check that this is a valid packet */
        tmpRxPacket = (union ConcentratorPacket*)(rxPacket->payload);

        /* If this is a known packet */
        if (tmpRxPacket->header.packetType == RADIO_PACKET_TYPE_SENSORS_DATA_PACKET)
        {
            /* Save packet */
           latestRxPacket.header.sourceAddress = rxPacket->payload[0];
           latestRxPacket.header.packetType = rxPacket->payload[1];

           latestRxPacket.sensorsDataPacket.packetSubType = rxPacket->payload[2];

           latestRxPacket.sensorsDataPacket.packetNumber = (rxPacket->payload[3] << 8) | (rxPacket->payload[4]);

           latestRxPacket.sensorsDataPacket.timestamp.sec = (rxPacket->payload[5] << 24) | (rxPacket->payload[6] << 16) | (rxPacket->payload[7] << 8) | (rxPacket->payload[8]);

           if (latestRxPacket.sensorsDataPacket.packetSubType == 1)
           {
               for (indexAxis = 0; indexAxis < NMB_AXIS_IMU; ++indexAxis)
               {
                   latestRxPacket.sensorsDataPacket.acc[indexAxis] =  (rxPacket->payload[9 + indexAxis * 4] << 8) | (rxPacket->payload[10 + indexAxis * 4]);
                   latestRxPacket.sensorsDataPacket.gyro[indexAxis] = (rxPacket->payload[11 + indexAxis * 4] << 8) | (rxPacket->payload[12 + indexAxis * 4]);
               }
           }

           if (latestRxPacket.sensorsDataPacket.packetSubType == 2)
           {
               for (indexAxis = 0; indexAxis < NMB_AXIS_IMU; ++indexAxis)
               {
                   latestRxPacket.sensorsDataPacket.mag[indexAxis] = (rxPacket->payload[9 + indexAxis * 4] << 24) | (rxPacket->payload[10 + indexAxis * 4] << 16) | (rxPacket->payload[11 + indexAxis * 4] << 8) | (rxPacket->payload[12 + indexAxis * 4]);
               }
           }

           if (latestRxPacket.sensorsDataPacket.packetSubType == 3)
           {
               latestRxPacket.sensorsDataPacket.battVolt = (rxPacket->payload[9] << 8) | (rxPacket->payload[10]);
               latestRxPacket.sensorsDataPacket.temp = (rxPacket->payload[11] << 8) | (rxPacket->payload[12]);
               latestRxPacket.sensorsDataPacket.press = (rxPacket->payload[13] << 24) | (rxPacket->payload[14] << 16) | (rxPacket->payload[15] << 8) | (rxPacket->payload[16]);
               latestRxPacket.sensorsDataPacket.satCount = rxPacket->payload[17];
               latestRxPacket.sensorsDataPacket.timeUTC.hour = rxPacket->payload[18];
               latestRxPacket.sensorsDataPacket.timeUTC.minute = rxPacket->payload[19];
               latestRxPacket.sensorsDataPacket.timeUTC.second = rxPacket->payload[20];
           }

           if (latestRxPacket.sensorsDataPacket.packetSubType == 4)
           {
               latestRxPacket.sensorsDataPacket.lat.degrees = rxPacket->payload[9];
               latestRxPacket.sensorsDataPacket.lat.minutes = (rxPacket->payload[10] << 8) | (rxPacket->payload[11]);
               latestRxPacket.sensorsDataPacket.lat.azmuth = rxPacket->payload[12];

               latestRxPacket.sensorsDataPacket.lon.degrees = rxPacket->payload[13];
               latestRxPacket.sensorsDataPacket.lon.minutes = (rxPacket->payload[14] << 8) | (rxPacket->payload[15]);
               latestRxPacket.sensorsDataPacket.lon.azmuth = rxPacket->payload[16];

               latestRxPacket.sensorsDataPacket.alt = (rxPacket->payload[17] << 24) | (rxPacket->payload[18] << 16) | (rxPacket->payload[19] << 8) | (rxPacket->payload[20]);
           }

           if (latestRxPacket.sensorsDataPacket.packetSubType == 0)
           {
               latestRxPacket.sensorsDataPacket.battVolt = (rxPacket->payload[9] << 8) | (rxPacket->payload[10]);

               for (indexAxis = 0; indexAxis < NMB_AXIS_IMU; ++indexAxis)
               {
                   latestRxPacket.sensorsDataPacket.acc[indexAxis] = (rxPacket->payload[11 + indexAxis * 8] << 8) | (rxPacket->payload[12 + indexAxis * 8]);
                   latestRxPacket.sensorsDataPacket.gyro[indexAxis] = (rxPacket->payload[13 + indexAxis * 8] << 8) | (rxPacket->payload[14 + indexAxis * 8]);
                   latestRxPacket.sensorsDataPacket.mag[indexAxis] = (rxPacket->payload[15 + indexAxis * 8] << 24) | (rxPacket->payload[16 + indexAxis * 8] << 16) | (rxPacket->payload[17 + indexAxis * 8] << 8) | (rxPacket->payload[18 + indexAxis * 8]);
               }

               latestRxPacket.sensorsDataPacket.temp = (rxPacket->payload[35] << 8) | (rxPacket->payload[36]);

               latestRxPacket.sensorsDataPacket.press = (rxPacket->payload[37] << 24) | (rxPacket->payload[38] << 16) | (rxPacket->payload[39] << 8) | (rxPacket->payload[40]);

               latestRxPacket.sensorsDataPacket.lat.degrees = rxPacket->payload[41];
               latestRxPacket.sensorsDataPacket.lat.minutes = (rxPacket->payload[42] << 8) | (rxPacket->payload[43]);
               latestRxPacket.sensorsDataPacket.lat.azmuth = rxPacket->payload[44];

               latestRxPacket.sensorsDataPacket.lon.degrees = rxPacket->payload[45];
               latestRxPacket.sensorsDataPacket.lon.minutes = (rxPacket->payload[46] << 8) | (rxPacket->payload[47]);
               latestRxPacket.sensorsDataPacket.lon.azmuth = rxPacket->payload[48];

               latestRxPacket.sensorsDataPacket.alt = (rxPacket->payload[49] << 24) | (rxPacket->payload[50] << 16) | (rxPacket->payload[51] << 8) | (rxPacket->payload[52]);

               latestRxPacket.sensorsDataPacket.satCount = rxPacket->payload[55];

               latestRxPacket.sensorsDataPacket.timeUTC.hour = rxPacket->payload[54];
               latestRxPacket.sensorsDataPacket.timeUTC.minute = rxPacket->payload[54];
               latestRxPacket.sensorsDataPacket.timeUTC.second = rxPacket->payload[56];
           }

           /* Signal packet received */
           Event_post(radioOperationEventHandle, RADIO_EVENT_VALID_PACKET_RECEIVED);
        }
        else
            if (tmpRxPacket->header.packetType == RADIO_PACKET_TYPE_IAA_PACKET)
                {
                    /* Save packet */
                   latestRxPacket.header.sourceAddress = rxPacket->payload[0];
                   latestRxPacket.header.packetType = rxPacket->payload[1];

                   latestRxPacket.IAAPacket.HwSwVer.hwMaj = rxPacket->payload[2];
                   latestRxPacket.IAAPacket.HwSwVer.hwMin = rxPacket->payload[3];
                   latestRxPacket.IAAPacket.HwSwVer.swMaj = rxPacket->payload[4];
                   latestRxPacket.IAAPacket.HwSwVer.swMin = rxPacket->payload[5];

                   latestRxPacket.IAAPacket.timestamp.sec = (rxPacket->payload[6] << 24) | (rxPacket->payload[7] << 16) | (rxPacket->payload[8] << 8) | (rxPacket->payload[9]);

                   /* Signal packet received */
                   Event_post(radioOperationEventHandle, RADIO_EVENT_VALID_PACKET_RECEIVED);
                }
                else
                    {
                        /* Signal invalid packet received */
                        Event_post(radioOperationEventHandle, RADIO_EVENT_UNKNOWN_PACKET_RECEIVED);
                    }
    }
    else
    {
        /* Signal invalid packet received */
        Event_post(radioOperationEventHandle, RADIO_EVENT_INVALID_PACKET_RECEIVED);
    }
}
