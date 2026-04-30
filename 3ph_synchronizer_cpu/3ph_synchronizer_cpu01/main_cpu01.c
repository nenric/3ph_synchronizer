//
// Included Files
//
#include "F28x_Project.h"
//
// Defines
//
#define BLINKY_LED_GPIO_BLUE 31
#define BLINKY_LED_GPIO_RED 34
void main(void)
{
//
// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2837xD_SysCtrl.c file.
//
    InitSysCtrl();
//
// Step 2. Initialize GPIO:
// This example function is found in the F2837xD_Gpio.c file and
// illustrates how to set the GPIO to it’s default state.
//
    InitGpio();
    GPIO_SetupPinMux(BLINKY_LED_GPIO_BLUE, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinMux(BLINKY_LED_GPIO_RED, GPIO_MUX_CPU2, 0);
    GPIO_SetupPinOptions(BLINKY_LED_GPIO_BLUE, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinOptions(BLINKY_LED_GPIO_RED, GPIO_OUTPUT, GPIO_PUSHPULL);

//
// Step 3. Loop to blink LED
//
    for(;;)
    {
        //
        // Turn on LED
        //
        GPIO_WritePin(BLINKY_LED_GPIO_BLUE, 0);
        //
        // Delay for a bit.
        //
        DELAY_US(1000*500);
        //
        // Turn off LED
        //
        GPIO_WritePin(BLINKY_LED_GPIO_BLUE, 1);
        //
        // Delay for a bit.
        //
        DELAY_US(1000*500);
    }
}
