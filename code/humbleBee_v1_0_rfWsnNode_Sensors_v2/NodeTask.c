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
#include <stdlib.h>
#include <math.h>

/* XDCtools Header files */ 
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */ 
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>

/* TI-RTOS Header files */ 
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/ADC.h>
#include <ti/drivers/Watchdog.h>

/* Board Header files */
#include "Board.h"

/* Application Header files */ 
#include "common.h"
#include "NodeTask.h"
#include "NodeRadioTask.h"

/* GNSS parsing library */
#include "GNSS/gnss_parser.h"

/* Sensor libraries */
#include "Sensors/SensorI2C.h"
#include "Sensors/SensorBmp280.h"
#include "Sensors/SensorMpu9250.h"

/* Register reading library (for the MAC address) */
#include <hw_fcfg1.h>

/* Real Time Clock library */
#include <ti/devices/cc13x0/driverlib/aon_rtc.h>

/***** Defines *****/
/* A change mask of 0xFF0 means that changes in the lower 4 bits does not trigger a wakeup. */
#define NODE_ADCTASK_CHANGE_MASK        0xFF0

/* Task Stack Size */
#define TASK_STACK_SIZE 512

/* Task Priorities */
#define GETSENSORDATA_TASK_PRIORITY     5
#define BATTVOLT_TASK_PRIORITY          1
#define IMU_TASK_PRIORITY               1
#define TEMPPRESS_TASK_PRIORITY         1
#define GNSS_TASK_PRIORITY              2
#define IAA_TASK_PRIORITY               3

/* Task Periods (in ticks; 1 tick = 10 us) */
#define GETSENSORDATA_TASK_PERIOD    50000
#define BATTVOLT_TASK_PERIOD         50000
#define IMU_TASK_PERIOD              50000
#define TEMPPRESS_TASK_PERIOD        50000
#define GNSS_TASK_PERIOD             50000
#define IAA_TASK_PERIOD              3000000

/* GNSS */
#define GNSS_RESET_IMPULSE_DURATION         30000   // ticks
#define GNSS_NO_PARSED_MESSAGES_FOR_UPDATE  5

/* Watchdog */
#define WATCHDOG_TIMEOUT_MS         15000 // the MCU is reset after 2 timeouts

/***** Variable declarations *****/
static Task_Params GetSensorDataTaskParams, IAATaskParams, BattVoltTaskParams, IMUTaskParams, TempPressTaskParams, GNSSTaskParams;
Task_Struct GetSensorDataTask, IAATask, BattVoltTask, IMUTask, TempPressTask, GNSSTask;    /* Not static so you can see in ROV */
static uint8_t GetSensorDataTaskStack[TASK_STACK_SIZE], IAATaskStack[TASK_STACK_SIZE], BattVoltTaskStack[TASK_STACK_SIZE], IMUTaskStack[TASK_STACK_SIZE], TempPressTaskStack[TASK_STACK_SIZE], GNSSTaskStack[TASK_STACK_SIZE];

Semaphore_Struct semReadStruct, semBattVoltStruct, semIMUStruct, semTempPressStruct, semGNSSStruct;
Semaphore_Handle semReadHandle, semBattVoltHandle, semIMUHandle, semTempPressHandle, semGNSSHandle;

Watchdog_Handle watchdogHandle;

UART_Handle uart;

uint16_t packetNumber;
ADC_Handle adcVBat1;
uint16_t VBat1;
int16_t acc[3], gyro[3];
int32_t mag[3];
int16_t temp;
uint32_t press;
locationToSend_t lat, lon;
int32_t altitude;
uint8_t satCount;
utc_time_t timeUTC;

/***** Prototypes *****/
static void GetSensorsDataFunction(UArg arg0, UArg arg1);
static void IAAFunction(UArg arg0, UArg arg1);
static void readBatteryVoltage(UArg arg0, UArg arg1);
static void readIMU(UArg arg0, UArg arg1);
static void readTempPress(UArg arg0, UArg arg1);
static void readGNSS(UArg arg0, UArg arg1);
timestamp_t  readTimestamp(void);

/* Callback functions definitions */
void UARTReadCallback(UART_Handle handle, void *buffer, size_t num)
{
    Semaphore_post(semReadHandle);
}


/***** Function definitions *****/
void MainTaskFunction(UArg arg0, UArg arg1)
{
    Semaphore_Params semParams;

    /* Initialize GPIO */
        GPIO_init();

    /* Initialize LED */
        GPIO_setConfig(Board_GPIO_LED0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
        GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_ON);

    /* Initialize the Packet Counter */
        packetNumber = 0;

    /* Initialize Battery Voltage Reader */
        // Initialize the READ_VBAT pin
        GPIO_setConfig(Board_READ_VBAT, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

        // Initialize ADC
        ADC_Params params;

        ADC_init();
        ADC_Params_init(&params);
        adcVBat1 = ADC_open(Board_ADC1, &params);
        if (adcVBat1 == NULL)
        {
            //TO DO
        }

    /* Initialize I2C */
        bool status;

        status = SensorI2C_open();

        // Initialize MPU9250 sensor (accelerometer, gyroscope, magnetometer)
        status = SensorMpu9250_init();
        status = SensorMpu9250_powerIsOn();
        status = SensorMpu9250_test();
        SensorMpu9250_enable(0x7F);
        status = SensorMpu9250_accSetRange(ACC_RANGE_2G);
        //status = SensorMpu9250_magTest();

        // Initialize BMP280 sensor (temperature, pressure)
        status = SensorBmp280_init();
        status = SensorBmp280_test();
        SensorBmp280_enable(1);

    /* Initialize GNSS */
        // Initialize the Reset pin
        GPIO_setConfig(Board_MIKROBUS_RESET, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH);

        // Initialize UART (for GNSS)
        UART_Params uartParams;

        UART_init();
        UART_Params_init(&uartParams);
        uartParams.readMode = UART_MODE_CALLBACK;
        uartParams.readCallback = &UARTReadCallback;
        uartParams.writeDataMode = UART_DATA_BINARY;
        uartParams.readDataMode = UART_DATA_BINARY;
        uartParams.readReturnMode = UART_RETURN_FULL;
        uartParams.readEcho = UART_ECHO_OFF;
        uartParams.baudRate = 9600;

        uart = UART_open(Board_UART0, &uartParams);

        // Reset the GNSS sensor
        //GPIO_write(Board_MIKROBUS_RESET, 0);
        Task_sleep(GNSS_RESET_IMPULSE_DURATION);
        GPIO_write(Board_MIKROBUS_RESET, 1);

        /* Construct the UART Reading Semaphore*/
        Semaphore_Params_init(&semParams);
        Semaphore_construct(&semReadStruct, 1, &semParams);
        semReadHandle = Semaphore_handle(&semReadStruct);

    /* Create the Sensor Data Reading task */
        Task_Params_init(&GetSensorDataTaskParams);
        GetSensorDataTaskParams.stackSize = TASK_STACK_SIZE;
        GetSensorDataTaskParams.priority = GETSENSORDATA_TASK_PRIORITY;
        GetSensorDataTaskParams.stack = &GetSensorDataTaskStack;
        Task_construct(&GetSensorDataTask, GetSensorsDataFunction, &GetSensorDataTaskParams, NULL);

    /* Create the IAA message sending task */
        Task_Params_init(&IAATaskParams);
        IAATaskParams.stackSize = TASK_STACK_SIZE;
        IAATaskParams.priority = IAA_TASK_PRIORITY;
        IAATaskParams.stack = &IAATaskStack;
        Task_construct(&IAATask, IAAFunction, &IAATaskParams, NULL);

    /* Create the Battery Voltage Reading task */
        Task_Params_init(&BattVoltTaskParams);
        BattVoltTaskParams.stackSize = TASK_STACK_SIZE;
        BattVoltTaskParams.priority = BATTVOLT_TASK_PRIORITY;
        BattVoltTaskParams.stack = &BattVoltTaskStack;
        Task_construct(&BattVoltTask, readBatteryVoltage, &BattVoltTaskParams, NULL);

    /* Construct the Semaphore for Battery Voltage, initial count 1 */
        Semaphore_Params_init(&semParams);
        Semaphore_construct(&semBattVoltStruct, 1, &semParams);
        semBattVoltHandle = Semaphore_handle(&semBattVoltStruct);

    /* Create the IMU (Accelerometer, Gyroscope, Magnetometer) Reading task */
        Task_Params_init(&IMUTaskParams);
        IMUTaskParams.stackSize = TASK_STACK_SIZE;
        IMUTaskParams.priority = IMU_TASK_PRIORITY;
        IMUTaskParams.stack = &IMUTaskStack;
        Task_construct(&IMUTask, readIMU, &IMUTaskParams, NULL);

    /* Construct the Semaphore for the IMU (Accelerometer, Gyroscope, Magnetometer), initial count 1 */
        Semaphore_Params_init(&semParams);
        Semaphore_construct(&semIMUStruct, 1, &semParams);
        semIMUHandle = Semaphore_handle(&semIMUStruct);

    /* Create the Temperature + Pressure Reading task */
        Task_Params_init(&TempPressTaskParams);
        TempPressTaskParams.stackSize = TASK_STACK_SIZE;
        TempPressTaskParams.priority = TEMPPRESS_TASK_PRIORITY;
        TempPressTaskParams.stack = &TempPressTaskStack;
        Task_construct(&TempPressTask, readTempPress, &TempPressTaskParams, NULL);

    /* Construct the Semaphore for Temperature + Pressure, initial count 1 */
        Semaphore_Params_init(&semParams);
        Semaphore_construct(&semTempPressStruct, 1, &semParams);
        semTempPressHandle = Semaphore_handle(&semTempPressStruct);

    /* Create the GNSS Reading task */
        Task_Params_init(&GNSSTaskParams);
        GNSSTaskParams.stackSize = TASK_STACK_SIZE;
        GNSSTaskParams.priority = GNSS_TASK_PRIORITY;
        GNSSTaskParams.stack = &GNSSTaskStack;
        Task_construct(&GNSSTask, readGNSS, &GNSSTaskParams, NULL);

    /* Construct the Semaphore for GNSS, initial count 1 */
        Semaphore_Params_init(&semParams);
        Semaphore_construct(&semGNSSStruct, 1, &semParams);
        semGNSSHandle = Semaphore_handle(&semGNSSStruct);

    /* Construct the Watchdog Timer */
        Watchdog_Params watchdogParams;
        uint32_t tickValue;

        Watchdog_init();
        Watchdog_Params_init(&watchdogParams);

        watchdogParams.resetMode = Watchdog_RESET_ON;

        watchdogHandle = Watchdog_open(Board_WATCHDOG0, &watchdogParams);
        if (watchdogHandle == NULL)
            status = 0;
        else
            status = 1;

        tickValue = Watchdog_convertMsToTicks(watchdogHandle, WATCHDOG_TIMEOUT_MS);
        Watchdog_setReload(watchdogHandle, tickValue);
}

static void GetSensorsDataFunction(UArg arg0, UArg arg1)
{
    uint8_t indexAxis;
    uint16_t battVoltToSend;
    int16_t accToSend[3], gyroToSend[3];
    int32_t magToSend[3];
    int16_t tempToSend;
    uint32_t pressToSend;
    locationToSend_t latToSend, lonToSend;
    int32_t altitudeToSend;
    uint8_t satCountToSend;
    utc_time_t timeUTCToSend;
    timestamp_t timestampToSend;

    /* Infinite loop */
    while (1)
    {
        // Clear the Watchdog
        Watchdog_clear(watchdogHandle);

        // Toggle the activity LED (Red LED)
        GPIO_toggle(Board_GPIO_LED0);

        /* Timestamp */
        timestampToSend = readTimestamp();

        /* Packet Number */
        packetNumber++;

        /* Battery Voltage */
        Semaphore_pend(semBattVoltHandle, BIOS_WAIT_FOREVER);
        battVoltToSend = VBat1;
        Semaphore_post(semBattVoltHandle);

        /* Accelerometer, Gyroscope */
        Semaphore_pend(semIMUHandle, BIOS_WAIT_FOREVER);
        for (indexAxis = 0; indexAxis < NMB_AXIS_IMU; ++indexAxis)
        {
            accToSend[indexAxis] = acc[indexAxis];
            gyroToSend[indexAxis] = gyro[indexAxis];
            magToSend[indexAxis] = mag[indexAxis];
        }
        Semaphore_post(semIMUHandle);

        /* Temperature, Pressure */
        Semaphore_pend(semTempPressHandle, BIOS_WAIT_FOREVER);
        tempToSend = temp;
        pressToSend = press;
        Semaphore_post(semTempPressHandle);

        /* GNSS */
        Semaphore_pend(semGNSSHandle, BIOS_WAIT_FOREVER);
        latToSend = lat;
        lonToSend = lon;
        altitudeToSend = altitude;
        satCountToSend = satCount;
        timeUTCToSend = timeUTC;
        Semaphore_post(semGNSSHandle);

        /* Timestamp */
        timestampToSend = readTimestamp();

        // Send the measured value to concentrator
#ifdef SHORT_MESSAGES
        NodeRadioTask_sendSensorsData(1, packetNumber, battVoltToSend, accToSend, gyroToSend, magToSend, tempToSend, pressToSend, latToSend, lonToSend, altitudeToSend, satCountToSend, timeUTCToSend, timestampToSend);
        //NodeRadioTask_sendSensorsData(2, packetNumber, battVoltToSend, accToSend, gyroToSend, magToSend, tempToSend, pressToSend, latToSend, lonToSend, altitudeToSend, satCountToSend, timeUTCToSend, timestampToSend);
        NodeRadioTask_sendSensorsData(3, packetNumber, battVoltToSend, accToSend, gyroToSend, magToSend, tempToSend, pressToSend, latToSend, lonToSend, altitudeToSend, satCountToSend, timeUTCToSend, timestampToSend);
        NodeRadioTask_sendSensorsData(4, packetNumber, battVoltToSend, accToSend, gyroToSend, magToSend, tempToSend, pressToSend, latToSend, lonToSend, altitudeToSend, satCountToSend, timeUTCToSend, timestampToSend);
#else
        NodeRadioTask_sendSensorsData(0, packetNumber, battVoltToSend, accToSend, gyroToSend, magToSend, tempToSend, pressToSend, latToSend, lonToSend, altitudeToSend, satCountToSend, timeUTCToSend, timestampToSend);
#endif

        // Sleep
        Task_sleep(GETSENSORDATA_TASK_PERIOD);
    }
}

static void IAAFunction(UArg arg0, UArg arg1)
{
    timestamp_t timestampToSend;

    while (1)
    {
        /* Timestamp */
        timestampToSend = readTimestamp();

        // Send the IAA message
        NodeRadioTask_sendIAA(timestampToSend);

        // Sleep
        Task_sleep(IAA_TASK_PERIOD);
    }
}

/* ************* Battery Voltage ************* */
static void readBatteryVoltage(UArg arg0, UArg arg1)
{
    int_fast16_t status;
    uint16_t adcValue;
    uint16_t VBat1Converted;

    while (1)
    {
        GPIO_write(Board_READ_VBAT, 1);
        Task_sleep(2000);

        status = ADC_convert(adcVBat1, &adcValue);

        GPIO_write(Board_READ_VBAT, 0);
        //Task_sleep(2000);

        if (status == ADC_STATUS_SUCCESS)
        {
            VBat1Converted = floor(((adcValue * 4.175) / 1960) * 1000); // mV
        }
        else
        {
            VBat1Converted = 0;
        }

        Semaphore_pend(semBattVoltHandle, BIOS_WAIT_FOREVER);
        VBat1 = VBat1Converted;
        Semaphore_post(semBattVoltHandle);

        Task_sleep(BATTVOLT_TASK_PERIOD);
    }
}

/* ************* Accelerometer, Gyroscope ************* */
static void readIMU(UArg arg0, UArg arg1)
{
    bool status;
    uint16_t accRaw[3], gyroRaw[3];
    int16_t magRaw[3];
    uint8_t indexAxis;

    while (1)
    {
        status = SensorMpu9250_accRead(accRaw);
        if (!status)
        {
            for (indexAxis = 0; indexAxis < NMB_AXIS_IMU; ++indexAxis)
            {
                accRaw[indexAxis] = 0;
            }
        }

        status = SensorMpu9250_gyroRead(gyroRaw);
        if (!status)
        {
            for (indexAxis = 0; indexAxis < NMB_AXIS_IMU; ++indexAxis)
            {
                gyroRaw[indexAxis] = 0;
            }
        }

        /*
        status = SensorMpu9250_magRead(magRaw); // for magnetometer, status is 0 if the reading is successful
        if (status)
        {
            for (indexAxis = 0; indexAxis < NMB_AXIS_IMU; ++indexAxis)
            {
                magRaw[indexAxis] = 0;
            }
        }
        */

        Semaphore_pend(semIMUHandle, BIOS_WAIT_FOREVER);
        for (indexAxis = 0; indexAxis < NMB_AXIS_IMU; ++indexAxis)
        {
            acc[indexAxis] = floor(SensorMpu9250_accConvert(accRaw[indexAxis]) * 1000);
            gyro[indexAxis] = floor(SensorMpu9250_gyroConvert(gyroRaw[indexAxis]) * 100);
            //mag[indexAxis] = floor(SensorMpu9250_magConvert(magRaw[indexAxis]) * 1000);
        }
        Semaphore_post(semIMUHandle);

        Task_sleep(IMU_TASK_PERIOD);
    }
}

/* ************* Temperature, Pressure ************* */
static void readTempPress(UArg arg0, UArg arg1)
{
    bool status;
    uint8_t data[6];
    int32_t tempRaw;
    uint32_t pressRaw;

    while (1)
    {
        status = SensorBmp280_read(data);
        if (status)
        {
            SensorBmp280_convert(data, &tempRaw, &pressRaw);
        }
        else
        {
            tempRaw = 0;
            pressRaw = 0;
        }

        Semaphore_pend(semTempPressHandle, BIOS_WAIT_FOREVER);
        temp = (int16_t)tempRaw;
        press = pressRaw;
        Semaphore_post(semTempPressHandle);

        Task_sleep(TEMPPRESS_TASK_PERIOD);
    }
}

/* ************* GNSS ************* */
static void readGNSS(UArg arg0, UArg arg1)
{
    bool doneReading = 0;
    uint8_t noParsedMessages = 0;
    char input;
    location_t *latRaw, *lonRaw;
    double altitudeRaw;
    uint8_t satCountRaw;
    utc_time_t *timeRaw;

    while (1)
    {
        doneReading = 0;

        while (!doneReading)
        {
            // Read one character from UART
            UART_read(uart, &input, 1);
            Semaphore_pend(semReadHandle, BIOS_WAIT_FOREVER);

            // Add the character to a buffer
            gnss_put(input);

            // Parse the buffer (if the buffer is ready to be parsed)
            if (gnss_parse())
            {
                noParsedMessages++;

                if (noParsedMessages == GNSS_NO_PARSED_MESSAGES_FOR_UPDATE)
                {
                    latRaw = gnss_current_lat();
                    lonRaw = gnss_current_lon();
                    altitudeRaw = gnss_gga_altitude();
                    satCountRaw = gnss_gga_satcount();
                    timeRaw = gnss_current_fix();

                    noParsedMessages = 0;
                    doneReading = 1;
                }
            }
        }

        Semaphore_pend(semGNSSHandle, BIOS_WAIT_FOREVER);
        lat.azmuth = (uint8_t)latRaw->azmuth;
        lat.degrees = latRaw->degrees;
        lat.minutes = floor(latRaw->minutes * 1000);
        lon.azmuth = (uint8_t)lonRaw->azmuth;
        lon.degrees = lonRaw->degrees;
        lon.minutes = floor(lonRaw->minutes * 1000);
        altitude = floor(altitudeRaw * 1000);
        satCount = satCountRaw;
        timeUTC.hour = timeRaw->hour;
        timeUTC.minute = timeRaw->minute;
        timeUTC.second = timeRaw->second;
        timeUTC.ms = timeRaw->ms;
        Semaphore_post(semGNSSHandle);

        Task_sleep(GNSS_TASK_PERIOD);
    }
}

/* ************* Timestamp ************* */
timestamp_t readTimestamp(void)
{
    uint8_t status;
    timestamp_t timestamp;

    AONRTCEnable();
    status = AONRTCActive();
    timestamp.sec = AONRTCSecGet();
    timestamp.subsec = AONRTCFractionGet();

    return timestamp;
}
