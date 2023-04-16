/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
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

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <strings.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>

// Driver for GPIO.
#include <ti/drivers/GPIO.h>

// Driver configuration.
#include "ti_drivers_config.h"

// Library to use timer.
#include <ti/drivers/Timer.h>

// Size of the buffer used for UART logging, and server output.
#define UART_BUFFER_SIZE 64

// I2C Global Variables
I2C_Handle i2c;

// List of possible temperature sensors depending on board being used.
static const struct
{
    char *id;
    uint8_t address;
    uint8_t resultReg;

} sensors[3] = {
    { .address = 0x48, .resultReg = 0x00, .id = "11X" },
    { .address = 0x49, .resultReg = 0x00, .id = "116" },
    { .address = 0x41, .resultReg = 0x01, .id = "006" }
};
I2C_Transaction i2cTransaction;
uint8_t txBuffer[1];
uint8_t rxBuffer[2];

// UART Global Variables
UART_Handle uart;
int bytesToSend;
char output[UART_BUFFER_SIZE];

// Macro to write a buffer over UART.
#define DISPLAY(x) UART_write(uart, &output, x)

// Macro to simplify use of display macro by providing the output buffer and size of the buffer.
#define LOG(args...) DISPLAY(snprintf(output, sizeof(output), args))

// Timer period in milliseconds.
const uint32_t gTimerPeriodMs = 100;

// Flag to identify at least one task is ready to run.
volatile bool gRunnableTasks = false;

// Indicator to alert that the button has been pushed to raise the target temperature.
volatile bool gRaiseTargetTemp = false;

// indicator to alert the button has been pushed to lower the target temperature.
volatile bool gLowerTargetTemp = false;

// Number of times the timer callback has been called.
volatile uint32_t gTimerCount = 0;

// Target temperature in celsius.
uint32_t gTargetTempCelsius = 0;

// Current temperature in celsius
int16_t gCurrentTempCelsius = 0;

// Type for functions that are used by the task scheduler to execute tasks.
typedef void (*TaskHandler)(void);

// Task configuration to represent a single task.
typedef struct {
    bool isReady;         // Ready to perform task.
    uint16_t interval;    // Period between events.
    uint16_t elapsed;     // Amount of time since last triggered.
    TaskHandler handler;  // Function to be called when task must be executed.
} Task;

void checkButtons();      // Prototype of the handler to detect button presses.
void checkTemperature();  // Prototype of the handler to check temperature.
void updateServer();      // Prototype of the handler to update the server.

// List of tasks configurations to perform tasks of the thermostat.
volatile Task gTaskList[] = {
                    { .isReady = false, .interval = 200, .elapsed = 200, .handler = &checkButtons},
                    { .isReady = false, .interval = 500, .elapsed = 500, .handler = &checkTemperature},
                    { .isReady = false, .interval = 1000, .elapsed = 1000, .handler = &updateServer}

};

// Total number of tasks in the gTaskList.
const int gNumTasks = sizeof(gTaskList) / sizeof(gTaskList[0]);

// Function to read temperature. Read temperature from I2C sensor.
int16_t readTemp(void)
{
    int16_t temperature = 0;
    i2cTransaction.readCount = 2;
    if (I2C_transfer(i2c, &i2cTransaction))
    {
        /*
         * Extract degrees C from the received data;
         * see TMP sensor datasheet
         */
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;
        /*
         * If the MSB is set '1', then we have a 2's complement
         * negative value which needs to be sign extended
         */
        if (rxBuffer[0] & 0x80)
        {
            temperature |= 0xF000;
        }
    }
    else
    {
        LOG("Error reading temperature sensor (%d)\n\r", i2cTransaction.status);
        LOG("Please power cycle your board by unplugging USB and plugging it back in.\n\r");
    }
    return temperature;
}

// Function to check if either button has been pressed.
void checkButtons() {
    if (gRaiseTargetTemp) {
        gTargetTempCelsius++;
        gRaiseTargetTemp = false;
    } else if (gLowerTargetTemp) {
        gTargetTempCelsius--;
        gLowerTargetTemp = false;
    }
}
// Function to read current temperature. If temp is lower than target temperature, heat(LED) goes ON.
void checkTemperature() {
    gCurrentTempCelsius = readTemp();
    if (gCurrentTempCelsius < gTargetTempCelsius) {
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
    } else {
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
    }
}
// Function to simulate data being sent to server.
void updateServer() {
    const uint32_t msPerSec = 1000;
    uint32_t seconds = gTimerCount * gTimerPeriodMs / msPerSec;
    LOG("<%02d,%02d,%d,%04d>\n\r", gCurrentTempCelsius, gTargetTempCelsius, (gTargetTempCelsius > gCurrentTempCelsius) ? 1:0, seconds);
}


/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    gLowerTargetTemp = true;
}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *  This may not be used for all boards.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn1(uint_least8_t index)
{
    gRaiseTargetTemp = true;
}

// Function that gets called by timer when the period has elapsed.
void timerCallback(Timer_Handle myHandle, int_fast16_t status) {
    int i = 0;
    gTimerCount++;
    for (i = 0; i < gNumTasks; i++) {
       gTaskList[i].elapsed += gTimerPeriodMs;
       if (gTaskList[i].elapsed >= gTaskList[i].interval) {
           gTaskList[i].isReady = true;
           gRunnableTasks = true;
       }
    }
}

// Function to initialize I2C.
void initI2C(void)
{
    int8_t i, found;
    I2C_Params i2cParams;
    LOG("Initializing I2C Driver -");

    // Init the driver
    I2C_init();

    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL)
    {
        LOG("Failed\n\r");
        while (1)
            ;
    }
    LOG("Passed\n\r");
    // Boards were shipped with different sensors.
    // Welcome to the world of embedded systems.
    // Try to determine which sensor we have.
    // Scan through the possible sensor addresses
    /* Common I2C transaction setup */
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;
    found = false;
    for (i = 0; i < 3; ++i)
    {
        i2cTransaction.slaveAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;
        LOG("Is this %s? ", sensors[i].id);
        if (I2C_transfer(i2c, &i2cTransaction))
        {
            LOG("Found\n\r");
            found = true;
            break;
        }
        LOG("No\n\r");
    }
    if (found)
    {
        LOG("Detected TMP%s I2C address: %x\n\r",
            sensors[i].id, i2cTransaction.slaveAddress);
    }
    else
    {
        LOG("Temperature sensor not found, contact professor\n\r");
    }
}

// Function to initialize UART.
void initUART(void)
{
    UART_Params uartParams;
    // Init the driver
    UART_init();

    // Configure the driver
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;

    // Open the driver
    uart = UART_open(CONFIG_UART_0, &uartParams);
    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }
}

// Function to initialize timer.
void initTimer(void)
{
    const int msToUs = 1000; // Number of microseconds in milliseconds.
    Timer_Handle timer0;
    Timer_Params params;
    Timer_init();
    Timer_Params_init(&params);
    params.period = gTimerPeriodMs * msToUs;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    timer0 = Timer_open(CONFIG_TIMER_0, &params);
    if (timer0 == NULL)
    {
        /* Failed to initialized timer */
        while (1)
        {
        }
    }
    if (Timer_start(timer0) == Timer_STATUS_ERROR)
    {
        /* Failed to start timer */
        while (1)
        {
        }
    }
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on CONFIG_GPIO_BUTTON1.
     */
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1) {
        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }

    initUART();  // Initialize UART.
    initTimer(); // Initialize timer.
    initI2C();   // Initialize I2C.

    int i = 0;

    while (1) {
        while (!gRunnableTasks) { /* Iterate until timer called */ }
        for (i = 0; i < gNumTasks; i++) {
            if (gTaskList[i].isReady) {
                gTaskList[i].handler();
                gTaskList[i].isReady = false;
                gTaskList[i].elapsed = 0;
            }
        }
        gRunnableTasks = false;
    }

}
