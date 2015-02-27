#include "board.h"

#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"

void board_init(void)
{
    cpu_init();

	/* enable ports for leds and uart */
    ROM_SysCtlPeripheralDisable(SYSCTL_PERIPH_GPION);
   	ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_GPION);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    
    ROM_SysCtlPeripheralDisable(SYSCTL_PERIPH_GPIOF);
   	ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOF);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    
    ROM_SysCtlPeripheralDisable(SYSCTL_PERIPH_GPIOA);
   	ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOA);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    
    ROM_SysCtlPeripheralDisable(SYSCTL_PERIPH_GPIOC);
   	ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOC);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    /* Enable the GPIO pins for the LED (PF1, PF2 & PF3) */
    ROM_GPIOPinTypeGPIOOutput(GPION_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    ROM_GPIOPinTypeGPIOOutput(GPIOF_AHB_BASE, GPIO_PIN_0);

    /* Configure GPIO Pins for UART mode */
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIOA_AHB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    
    ROM_GPIOPinConfigure(GPIO_PA6_U2RX);
    ROM_GPIOPinConfigure(GPIO_PA7_U2TX);
    ROM_GPIOPinConfigure(GPIO_PN3_U2CTS);
    ROM_GPIOPinConfigure(GPIO_PN2_U2RTS);
    ROM_GPIOPinTypeUART(GPIOA_AHB_BASE, GPIO_PIN_6 | GPIO_PIN_7);
    ROM_GPIOPinTypeUART(GPION_BASE, GPIO_PIN_2 | GPIO_PIN_3);
    
    ROM_GPIOPinConfigure(GPIO_PC4_U7RX);
    ROM_GPIOPinConfigure(GPIO_PC5_U7TX);
    ROM_GPIOPinTypeUART(GPIOC_AHB_BASE, GPIO_PIN_4 | GPIO_PIN_5);

}
