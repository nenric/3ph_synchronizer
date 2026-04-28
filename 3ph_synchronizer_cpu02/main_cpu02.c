//
// Included Files
//
#include "F28x_Project.h"
//
// Defines
//
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
// Step 2. Loop to blink LED
//
    for(;;)
    {
        //
        // Turn on LED
        //
        GPIO_WritePin(BLINKY_LED_GPIO_RED, 0);
        //
        // Delay for a bit.
        //
        DELAY_US(500*500);
        //
        // Turn off LED
        //
        GPIO_WritePin(BLINKY_LED_GPIO_RED, 1);
        //
        // Delay for a bit.
        //
        DELAY_US(500*500);
    }
}
