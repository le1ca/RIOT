#include <stdio.h>
#include <string.h>

#include "thread.h"
#include "posix_io.h"
#include "shell.h"
#include "shell_commands.h"
#include "board_uart0.h"
#include "radio_driver.h"
#include "radio_trans.h"
#include "radio_app.h"
#include "xbee.h"
#include "joseta.h"

#define APP_PKT_BUFFER_SIZE 2048
uint16_t mypan, myaddr;
joseta_df_t joseta_last_minute[60];
char abuf1[APP_PKT_BUFFER_SIZE], abuf2[APP_PKT_BUFFER_SIZE];
bool joseta_fresh = false, master_set = false;

char     *ja_names[] = {"occupan", "relay", "voltage", "current", "phase","tempera","timesta"};
uint8_t   ja_types[] = {RA_FMT_u8, RA_FMT_u8, RA_FMT_u16, RA_FMT_u16, RA_FMT_u16, RA_FMT_u8, RA_FMT_u32};

static int shell_readc(void){
    char c = 0;
    (void) posix_read(uart0_handler_pid, &c, 1);
    return c;
}

static void shell_putchar(int c){
    (void) putchar(c);
}

void xbee_tx(char *buf, unsigned int len, uint16_t addr){
	ieee802154_node_addr_t dest;
	dest.pan.id   = mypan;
	dest.pan.addr = addr;
	
	char temp[100];
	memcpy(temp, buf, len);
	
	printf("[xbee] calling send()\n");

	radio_tx_status_t status = xbee_radio_driver.send(PACKET_KIND_DATA,
						   dest,
						   false,
						   true,
						   (void*) temp,
						   len
	);
	
	printf("[xbee] tx %d bytes to %04x\n", len, dest);
	printf("       result: %d\n", status);
}

void xbee_rx(void *buf, unsigned int len, int8_t rssi, uint8_t lqi, bool crc_ok){

	char temp[100];
	memcpy(temp, buf, len);
	
	printf("[xbee] rx %d bytes (rssi %d)\n", len, rssi);
	
	// pass to driver
	rt_incoming(temp, len);

}

void j_callback(unsigned int type, joseta_df_t *buf, unsigned int num_records){
    if(type == JOSETA_CB_PURGE){
        for(unsigned i = 0; i < num_records; i++)
            printf("[joseta] time=%lu, occ=%u, rel=%u, voltage=%u, current=%u, phase=%u, temp=%u\n",
                    (unsigned long) buf[i].time,
                    (unsigned) buf[i].occupancy,
                    (unsigned) buf[i].relay,
                    (unsigned) buf[i].voltage,
                    (unsigned) buf[i].current,
                    (unsigned) buf[i].phase,
                    (unsigned) buf[i].temp
            );
        printf("[joseta] purged %d records\n", num_records);
        memcpy(joseta_last_minute, buf, sizeof(joseta_df_t) * num_records);
        joseta_fresh = true;
    }
}	

void incoming_packet(radio_trans_pkt* packet){
    switch(packet->hdr.type){
        case RTRANS_TYPE_PROBE:{
            printf("[rt] incoming probe from %04x\n", packet->hdr.master);
            if(!master_set){
                    rt_set_master(packet->hdr.master);
                    rt_transmit(RTRANS_TYPE_JOIN, 0, 0);
                    master_set = true;
            }
            break;
        }
        case RTRANS_TYPE_POLL:{
            printf("[rt] incoming poll from %04x\n", packet->hdr.master);
            if(joseta_fresh){
                void *next = abuf1;
                uint32_t offset = 0;
                int i;
                for(i = 0; i < 60; i++){
                    offset += app_append_sample(next + offset, APP_PKT_BUFFER_SIZE - offset, ja_types[0], &joseta_last_minute[i].occupancy);
                    offset += app_append_sample(next + offset, APP_PKT_BUFFER_SIZE - offset, ja_types[1], &joseta_last_minute[i].relay);
                    offset += app_append_sample(next + offset, APP_PKT_BUFFER_SIZE - offset, ja_types[2], &joseta_last_minute[i].voltage);
                    offset += app_append_sample(next + offset, APP_PKT_BUFFER_SIZE - offset, ja_types[3], &joseta_last_minute[i].current);
                    offset += app_append_sample(next + offset, APP_PKT_BUFFER_SIZE - offset, ja_types[4], &joseta_last_minute[i].phase);
                    offset += app_append_sample(next + offset, APP_PKT_BUFFER_SIZE - offset, ja_types[5], &joseta_last_minute[i].temp);
                    offset += app_append_sample(next + offset, APP_PKT_BUFFER_SIZE - offset, ja_types[6], &joseta_last_minute[i].time);
                }
                uint16_t plen = app_build_pkt(abuf2, APP_PKT_BUFFER_SIZE, 60, 7, ja_names, (void *) abuf1, offset);
                printf("[ra] constructed packet of size %u\n", plen);
                rt_transmit(RTRANS_TYPE_DATA, abuf2, plen);
                //joseta_fresh = false;
            }
            else{
                rt_transmit(RTRANS_TYPE_DATA, 0, 0);
            }
            break;
        }
        default:{
            printf("[rt] skipping incoming msg of type %02x\n", packet->hdr.type);
        }
    }
}

int main(void)
{
    shell_t shell;
    (void) posix_open(uart0_handler_pid, 0);

	xbee_radio_driver.init();
	xbee_radio_driver.set_receive_callback(xbee_rx);
	
	mypan  = xbee_radio_driver.get_pan_id();
	myaddr = (uint16_t) xbee_radio_driver.get_long_address();
	xbee_radio_driver.set_address(myaddr); // set short addr to last 2 bytes of long addr
	
	printf("[xbee] pan id  is 0x%04x\n", mypan);
	printf("[xbee] address is 0x%04x\n", myaddr);
	printf("[xbee] channel is 0x%08x\n", xbee_radio_driver.get_channel());
	
	joseta_init(0);
	joseta_setcallback(j_callback, JOSETA_CB_PURGE);

    (void) puts("Welcome to RIOT!");
    rt_init_driver(myaddr, xbee_tx, incoming_packet);

    shell_init(&shell, NULL, UART0_BUFSIZE, shell_readc, shell_putchar);

    shell_run(&shell);
    return 0;
}
