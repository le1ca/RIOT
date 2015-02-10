#ifndef __PERIPH_CONF_H
#define __PERIPH_CONF_H

#define UART_IRQ_PRIO       1
#define UART_NUMOF          (7U)

#define UART_0_EN           1
#define UART_0_ISR          isr_uart0

#define UART_1_EN           0
#define UART_1_ISR          isr_uart1

#define UART_7_EN			1
#define UART_7_ISR			isr_uart7

#define UART_0_IRQ_CHAN     UART0_IRQn
#define UART_1_IRQ_CHAN     UART1_IRQn
#define UART_7_IRQ_CHAN		UART7_IRQn

#define UART_0_DEV          UART0
#define UART_1_DEV          UART1
#define UART_7_DEV			UART7

#define TIMER_NUMOF         6

#define TIMER_0_EN          1
#define TIMER_1_EN          1
#define TIMER_2_EN          1
#define TIMER_3_EN          1
#define TIMER_4_EN          1
#define TIMER_5_EN          1

#define TIMER_0_DEV         TIMER0
#define TIMER_1_DEV         TIMER1
#define TIMER_2_DEV         TIMER2
#define TIMER_3_DEV         TIMER3
#define TIMER_4_DEV         TIMER4
#define TIMER_5_DEV         TIMER5

#define TIMER_0_ISR         isr_tim0a
#define TIMER_1_ISR         isr_tim1a
#define TIMER_2_ISR         isr_tim2a
#define TIMER_3_ISR         isr_tim3a
#define TIMER_4_ISR         isr_tim4a
#define TIMER_5_ISR         isr_tim5a

#include "xbee-config.h"

#endif /* __PERIPH_CONF_H */
