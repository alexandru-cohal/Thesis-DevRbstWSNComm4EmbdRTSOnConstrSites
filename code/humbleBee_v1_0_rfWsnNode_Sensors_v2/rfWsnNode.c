/*
 * Copyright (c) 2015-2018, Texas Instruments Incorporated
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

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* Board Header files */
#include "Board.h"

/* Application Header files */ 
#include "NodeRadioTask.h"
#include "NodeTask.h"

/***** Defines *****/
/* Task Stack Size */
#define TASK_STACK_SIZE        1024

/* Task Priorities */
#define MAIN_TASK_PRIORITY     5

/***** Variable declarations *****/
static Task_Params MainTaskParams;
Task_Struct MainTask;    /* Not static so you can see in ROV */
static uint8_t MainTaskStack[TASK_STACK_SIZE];

/***** Prototypes *****/
extern void MainTaskFunction(UArg arg0, UArg arg1);

/*
 *  ======== main ========
 */
int main(void)
{
    /* Call driver init functions. */
    Board_initGeneral();

    /* Create the main Task */
    Task_Params_init(&MainTaskParams);
    MainTaskParams.stackSize = TASK_STACK_SIZE;
    MainTaskParams.priority = MAIN_TASK_PRIORITY;
    MainTaskParams.stack = &MainTaskStack;
    Task_construct(&MainTask, MainTaskFunction, &MainTaskParams, NULL);

    /* Initialize sensor node tasks */
    NodeRadioTask_init();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
