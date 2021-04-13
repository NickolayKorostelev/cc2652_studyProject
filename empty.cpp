/*
 * Copyright (c) 2015-2019, Texas Instruments Incorporated
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
 *    its contributors macceleration.y be used to endorse or promote products derived
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
 * OTHERWISE) ARISING IN ANY Wacceleration.y OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== empty.c ========
 */
#include <unistd.h>
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/apps/Button.h>
#include <ti/drivers/apps/LED.h>

/* Driver configuration */
#include "ti_drivers_config.h"

#include "I2Cdev.h"
#include "kalman.hpp"
#include "MPU6050.h"
#include <Cmath>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
/*
 * struct for acceleration and rotation values
 */
struct Aiss
{
    int16_t x;
    int16_t y;
    int16_t z;
};
Semaphore_Struct semStruct;
Semaphore_Handle semHandle;

Button_Params buttonParams;
Button_Handle configHandle = NULL;
LED_Params ledParams;
LED_Handle ledHandle = NULL;

/*
 *  ======== mainThread ========
 */

extern "C" {
void bntConfigCallback(Button_Handle buttonHandle,
                       Button_EventMask buttonEvents)
{
    switch (buttonEvents)
    {
    case Button_EV_PRESSED:
        break;
    case Button_EV_DOUBLECLICKED:
        LED_setOn(ledHandle, 1);
        for (uint8_t i = 0; i < 255; i++)
        {
            LED_setBrightnessLevel(ledHandle, i);
            usleep(10000);
        }
        Semaphore_post(semHandle);
        break;
    case Button_EV_LONGCLICKED:
        break;
    case Button_EV_LONGPRESSED:
        break;
    case Button_EV_CLICKED:
        break;
    case Button_EV_RELEASED:
        break;
    default:
        break;
    }
    return;
}

void* mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();
    Button_init();
    LED_init();



    /* Construct a Semaphore object to be use as a resource lock, inital count 1 */
    Semaphore_Params semParams;
    semParams.mode = ti_sysbios_knl_Semaphore_Mode_BINARY;
    Semaphore_Params_init(&semParams);
    Semaphore_construct(&semStruct, 0, &semParams);

    /* Obtain instance handle */
    semHandle = Semaphore_handle(&semStruct);

    buttonParams.debounceDuration = 50;
    buttonParams.buttonEventMask = 0xFF;
    buttonParams.doublePressDetectiontimeout = 300;
    buttonParams.longPressDuration = 300;

    Button_Params_init(&buttonParams);
    configHandle = Button_open(CONFIG_BUTTON_0, bntConfigCallback,
                               &buttonParams);

    if(configHandle == NULL)
    {
        while(1);
    }

    ledParams.setState = LED_STATE_OFF;
    ledParams.brightness = 75;
    ledParams.blinkPeriod = 500;
    LED_Params_init(&ledParams);
    ledHandle = LED_open(CONFIG_LED_0, &ledParams);

    Aiss acceleration = { 0 };
    Aiss rotation = { 0 };
    Aiss finalPosition = { 0 };

    MPU6050 *accelerometer = new MPU6050();

    Kalman *filter = new Kalman();

    while (1)
    {
        Semaphore_pend(semHandle, BIOS_WAIT_FOREVER);
        accelerometer->getRotation(&rotation.x, &rotation.y, &rotation.z);/*read rotation from gyroscope to local struct*/
        accelerometer->getAcceleration(&acceleration.x, &acceleration.y,
                                       &acceleration.z);/*read acceleration from gyroscope to local struct*/

        finalPosition.x = filter->getResult(
                (180 / 3.141592)
                        * atanf(acceleration.x
                                / sqrt((acceleration.y * acceleration.y)
                                        + (acceleration.z * acceleration.z))));
        finalPosition.y = filter->getResult(
                (180 / 3.141592)
                        * atanf(acceleration.y
                                / sqrt((acceleration.x * acceleration.x)
                                        + (acceleration.z * acceleration.z))));
        finalPosition.z = filter->getResult(
                (180 / 3.141592)
                        * atanf(sqrt(
                                (acceleration.z * acceleration.z)
                                        + (acceleration.x * acceleration.x))
                                / acceleration.z));

        for (uint8_t i = 100; i > 0; i--)
        {
            LED_setBrightnessLevel(ledHandle, i);
            usleep(10000);
        }
        LED_setOff(ledHandle);
    }
}
}
