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

#if FEATURE_PERIPH_RTC
#include "periph/rtc.h"
#endif

#ifdef MODULE_LTC4150
#include "ltc4150.h"
#endif

#if MODULE_AT86RF231 || MODULE_CC2420 || MODULE_MC1322X
#include "ieee802154_frame.h"
#endif

#ifdef MODULE_TRANSCEIVER
#include "transceiver.h"
#endif

#define SND_BUFFER_SIZE     (100)
#define RCV_BUFFER_SIZE     (64)
#define RADIO_STACK_SIZE    (8* KERNEL_CONF_STACKSIZE_DEFAULT)

#ifdef MODULE_TRANSCEIVER

char radio_stack_buffer[RADIO_STACK_SIZE];
msg_t msg_q[RCV_BUFFER_SIZE];

void *radio(void *arg)
{
    (void) arg;

    msg_t m;

#if MODULE_AT86RF231 || MODULE_CC2420 || MODULE_MC1322X
    ieee802154_packet_t *p;
    uint16_t i;
#elif MODULE_TI_EMAC
	ethernet_frame *p;
	uint16_t i;
#else
    radio_packet_t *p;
    radio_packet_length_t i;
#endif

    msg_init_queue(msg_q, RCV_BUFFER_SIZE);

    while (1) {
        msg_receive(&m);

        if (m.type == PKT_PENDING) {
#if MODULE_AT86RF231 || MODULE_CC2420 || MODULE_MC1322X

            p = (ieee802154_packet_t*) m.content.ptr;
            printf("Got radio packet:\n");
            printf("\tLength:\t%u\n", p->length);
            printf("\tSrc:\t%u\n", (p->frame.src_addr[0])|(p->frame.src_addr[1]<<8));
            printf("\tDst:\t%u\n", (p->frame.dest_addr[0])|(p->frame.dest_addr[1]<<8));
            printf("\tSrc:\t%u\n", (p->frame.src_addr[0])|(p->frame.src_addr[1]<<8));
            printf("\tLQI:\t%u\n", p->lqi);
            printf("\tRSSI:\t%u\n", p->rssi);

            printf("Payload Length:%u\n", p->frame.payload_len);
            printf("Payload: \n");
            
            for (i = 0; i < p->frame.payload_len; i++) {
                printf("%02X ", p->frame.payload[i]);
                if((i+1) % 16 == 0)
                	printf("\n");
            }
            if((i+1) % 16 != 0)
            	printf("\n");

            p->processing--;
            
            puts("done in app layer");
            puts("\n");
            
#elif MODULE_TI_EMAC
			p = (ethernet_frame *) m.content.ptr;
			
			/* ipv4 */
			if(p->hdr.type == 0x800){
				uint32_t* ip_hdr  = (uint32_t *) p->data;		 			 // ip header as words
				uint8_t   ip_hlen = (uint8_t)((ip_hdr[0] & 0x0f000000)>>24); // ip header len
				uint8_t   ip_prot = (uint8_t)((ip_hdr[2] & 0x00ff0000)>>16); // ip protocol
				uint8_t*  ip_sadd = (uint8_t*)&ip_hdr[3];					 // ip source
				uint8_t*  ip_dadd = (uint8_t*)&ip_hdr[4];					 // ip dest
				uint32_t* ip_pld  = &ip_hdr[ip_hlen];					     // ip payload
				
				printf("IPv4 packet\n");
				printf("Srce: %d.%d.%d.%d\n", ip_sadd[0], ip_sadd[1], ip_sadd[2], ip_sadd[3]);
				printf("Dest: %d.%d.%d.%d\n", ip_dadd[0], ip_dadd[1], ip_dadd[2], ip_dadd[3]);
				printf("Prot: 0x%02x\n", ip_prot);
			}
			else{
			
				printf("Ethernet frame\n");
			
				if(p->hdr.type <= 1500){
					printf("\tLen:\t%u\n", p->hdr.type);
				}
				else{
					printf("\tType:\t0x%04x\n", p->hdr.type);
				}
            
	            printf("\tSrc:\t%02x:%02x:%02x:%02x:%02x:%02x\n",
	            	p->hdr.src[0],
	            	p->hdr.src[1],
	            	p->hdr.src[2],
	            	p->hdr.src[3],
	            	p->hdr.src[4],
	            	p->hdr.src[5]
	            );
            
	             printf("\tDst:\t%02x:%02x:%02x:%02x:%02x:%02x\n",
	            	p->hdr.dest[0],
	            	p->hdr.dest[1],
	            	p->hdr.dest[2],
	            	p->hdr.dest[3],
	            	p->hdr.dest[4],
	            	p->hdr.dest[5]
	            );
	            
    	        printf("Payload Length:%u\n", p->plen);
    	        
    	        /*
	            printf("Payload: \n");
            
    	        for (i = 0; i < p->plen; i++) {
	                printf("%02X ", p->data[i]);
	                if((i+1) % 16 == 0)
	                	printf("\n");
	            }
	            if((i+1) % 16 != 0)
	            	printf("\n");
	            */
			}
			
    	    p->processing--;
	        puts("\n");
            
#else
            p = (radio_packet_t *) m.content.ptr;

            printf("Got radio packet:\n");
            printf("\tLength:\t%u\n", p->length);
            printf("\tSrc:\t%u\n", p->src);
            printf("\tDst:\t%u\n", p->dst);
            printf("\tLQI:\t%u\n", p->lqi);
            printf("\tRSSI:\t%u\n", p->rssi);

            for (i = 0; i < p->length; i++) {
                printf("%02X ", p->data[i]);
            }

            p->processing--;

#endif

        }
        else if (m.type == ENOBUFFER) {
            puts("Transceiver buffer full");
        }
        else {
            puts("Unknown packet received");
        }
    }
}

void init_transceiver(void)
{
    kernel_pid_t radio_pid = thread_create(
                        radio_stack_buffer,
                        sizeof(radio_stack_buffer),
                        PRIORITY_MAIN - 2,
                        CREATE_STACKTEST,
                        radio,
                        NULL,
                        "radio");

    uint16_t transceivers = TRANSCEIVER_DEFAULT;

    transceiver_init(transceivers);
    (void) transceiver_start();
    transceiver_register(transceivers, radio_pid);
}
#endif /* MODULE_TRANSCEIVER */

static int shell_readc(void)
{
    char c = 0;
    (void) posix_read(uart0_handler_pid, &c, 1);
    return c;
}

static void shell_putchar(int c)
{
    (void) putchar(c);
}

int main(void)
{
    shell_t shell;
    (void) posix_open(uart0_handler_pid, 0);

#ifdef MODULE_LTC4150
    ltc4150_start();
#endif

#ifdef MODULE_TRANSCEIVER
	puts("init transceiver");
    init_transceiver();
#else
	puts("no transceiver");
#endif

#ifdef FEATURE_PERIPH_RTC
    rtc_init();
#endif

    (void) puts("Welcome to RIOT!");

    shell_init(&shell, NULL, UART0_BUFSIZE, shell_readc, shell_putchar);

    shell_run(&shell);
    return 0;
}
