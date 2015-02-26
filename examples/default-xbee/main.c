/*
 * Copyright (C) 2008, 2009, 2010  Kaspar Schleiser <kaspar@schleiser.de>
 * Copyright (C) 2013 INRIA
 * Copyright (C) 2013 Ludwig Ortmann <ludwig.ortmann@fu-berlin.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       Default application that shows a lot of functionality of RIOT
 *
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 * @author      Oliver Hahm <oliver.hahm@inria.fr>
 * @author      Ludwig Ortmann <ludwig.ortmann@fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>
#include <string.h>

#include "thread.h"
#include "posix_io.h"
#include "shell.h"
#include "shell_commands.h"
#include "board_uart0.h"
#include "radio_driver.h"

#include "xbee.h"
#include "joseta.h"

uint16_t mypan;

static int shell_readc(void){
    char c = 0;
    (void) posix_read(uart0_handler_pid, &c, 1);
    return c;
}

static void shell_putchar(int c){
    (void) putchar(c);
}

void xbee_rx(void *buf, unsigned int len, int8_t rssi, uint8_t lqi, bool crc_ok){
	static uint8_t seq = 0;
	char* cb = (char*) buf;

	// print incoming packet
	printf("[xbee] got packet, rsi %d, lqi %d, crc %s, length is %d\n", rssi, lqi, crc_ok? "good" : "bad", len);
	for(int i = 0; i < len; i++){
		printf("%02x ", cb[i]);
	}
	printf("\n\n");
	
	// send ping reply
	ieee802154_node_addr_t dest;
	char rbuf[] = "\0\0RX\r";
	rbuf[0] = ++seq;
	dest.pan.id   = mypan;
	dest.pan.addr = 0xffff;
	
	xbee_radio_driver.send(PACKET_KIND_DATA,
						   dest,
						   false,
						   true,
						   (void*) &rbuf,
						   5
	);
}

int main(void)
{
    shell_t shell;
    (void) posix_open(uart0_handler_pid, 0);

	xbee_radio_driver.init();
	xbee_radio_driver.set_receive_callback(xbee_rx);
	
	mypan = xbee_radio_driver.get_pan_id();
	printf("[xbee] pan id is 0x%04x\n", mypan);
	printf("[xbee] channel is 0x%08x\n", xbee_radio_driver.get_channel());
	
	joseta_init(0);

    (void) puts("Welcome to RIOT!");

    shell_init(&shell, NULL, UART0_BUFSIZE, shell_readc, shell_putchar);

    shell_run(&shell);
    return 0;
}
