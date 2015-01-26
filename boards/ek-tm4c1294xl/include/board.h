#ifndef __BOARD_H
#define __BOARD_H

#include "cpu.h"
#include "driverlib/gpio.h"

/**
 * Define the nominal CPU core clock in this board
 */
#define F_CPU               (120000000UL)

/**
 * @name Assign the hardware timer
 */
#define HW_TIMER            TIMER_0

/**
 * @name Define the UART used for stdio
 */
#define STDIO               UART_0
#define STDIO_BAUDRATE      (115200U)

#define HWREG(x)            (*((volatile uint32_t *)(x)))

#define _RED_LED            HWREG(GPION_BASE + (GPIO_PIN_0 << 2))
#define _BLUE_LED           HWREG(GPION_BASE + (GPIO_PIN_1 << 2))
#define _GREEN_LED          HWREG(GPIOF_AHB_BASE + (GPIO_PIN_0 << 2))

#define RED_LED_ON          (_RED_LED |=  GPIO_PIN_0)
#define RED_LED_OFF         (_RED_LED &= ~GPIO_PIN_0)
#define RED_LED_TOGGLE      (_RED_LED ^=  GPIO_PIN_0)

#define BLUE_LED_ON         (_BLUE_LED |=  GPIO_PIN_1)
#define BLUE_LED_OFF        (_BLUE_LED &= ~GPIO_PIN_1)
#define BLUE_LED_TOGGLE     (_BLUE_LED ^=  GPIO_PIN_1)

#define GREEN_LED_ON        (_GREEN_LED |= 0x01)
#define GREEN_LED_OFF       (_GREEN_LED &= 0xfe)
#define GREEN_LED_TOGGLE    (_GREEN_LED ^= 0x01)

/**
 * @brief Initialize board specific hardware, including clock, LEDs and std-IO
 */
void board_init(void);

#endif /** __BOARD_H */
/** @} */
