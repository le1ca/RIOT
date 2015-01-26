/*
 * Copyright (C) 2014 volatiles UG (haftungsbeschränkt)
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     cpu_tm4c123
 * @{
 *
 * @file
 * @brief       Implementation of the CPU initialization
 *
 * @author      Benjamin Valentin <benjamin.valentin@volatiles.de>
 * @}
 */

#include <stdint.h>
#include "cpu.h"

#include "driverlib/rom.h"
#include "driverlib/sysctl.h"

/**
 * @brief Initialize the CPU, set IRQ priorities
 */
void cpu_init(void)
{
    /* Set the clocking to run directly from the crystal */
    SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000UL);

    /* set pendSV interrupt to lowest possible priority */
    NVIC_SetPriority(PendSV_IRQn, 0xff);

    /* Enable the FPU */
    ROM_FPUEnable();    
    
}
