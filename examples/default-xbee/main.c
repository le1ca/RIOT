#include <stdio.h>
#include <string.h>

#include "thread.h"
#include "posix_io.h"
#include "shell.h"
#include "shell_commands.h"
#include "board_uart0.h"
#include "radio_driver.h"
#include "radio_trans.h"

#include "xbee.h"
#include "joseta.h"

uint16_t mypan, myaddr;

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

	xbee_radio_driver.send(PACKET_KIND_DATA,
						   dest,
						   false,
						   true,
						   (void*) temp,
						   len
	);
}

void xbee_rx(void *buf, unsigned int len, int8_t rssi, uint8_t lqi, bool crc_ok){

	char temp[100];
	memcpy(temp, buf, len);
	
	// pass to driver
	rt_incoming(temp, len);

}

void j_callback(unsigned int type, joseta_df_t *buf, unsigned int num_records){
    if(type == JOSETA_CB_PURGE){
        for(unsigned i = 0; i < num_records; i++)
            printf("[joseta] time=%lu, voltage=%u, curent=%u, temp=%d\n",
                    (unsigned long) buf[i].time,
                    (unsigned) buf[i].voltage,
                    (unsigned) buf[i].current,
                    (unsigned) buf[i].temp
            );
    }
}	

void incoming_packet(radio_trans_pkt* packet){
    static char test[1002] = "Lorem ipsum dolor sit amet, consectetur adipiscing elit. In vitae ex eleifend, aliquam est ut, varius sapien. Nunc a vulputate diam. Class aptent taciti sociosqu ad litora torquent per conubia nostra, per inceptos himenaeos. Nam efficitur, sapien vel fermentum fermentum, dolor diam aliquet erat, vitae rhoncus lorem arcu in risus. Donec id hendrerit nibh. Suspendisse sit amet lacinia purus, at gravida eros. Nam lobortis nunc vel ornare ultrices. Etiam suscipit odio a varius hendrerit. Vestibulum porta magna et suscipit vestibulum. Nullam dignissim nisi eget rutrum vulputate. Donec in risus quis metus rhoncus fermentum. Phasellus sodales magna non purus gravida tincidunt. Fusce et eleifend urna. Nunc convallis erat ac enim pellentesque egestas. Aenean vel volutpat tortor. Duis sagittis odio ultricies eros posuere, tristique efficitur turpis venenatis. Morbi condimentum imperdiet diam, vitae viverra ante pharetra id. Nunc consectetur mi id auctor ornare. Phasellus consectetur, sem sed sed.";
    switch(packet->hdr.type){
        case RTRANS_TYPE_PROBE:
            printf("[rt] incoming probe from %04x\n", packet->hdr.master);
            rt_set_master(packet->hdr.master);
            rt_transmit(RTRANS_TYPE_JOIN, 0, 0);
            break;
        case RTRANS_TYPE_POLL:
            printf("[rt] incoming poll from %04x\n", packet->hdr.master);
            rt_transmit(RTRANS_TYPE_DATA, test, 1001);
            break;
        default:
            printf("[rt] skipping incoming msg of type %02x\n", packet->hdr.type);
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
	
	//joseta_init(0);
	//joseta_setcallback(j_callback, JOSETA_CB_PURGE);

    (void) puts("Welcome to RIOT!");
    rt_init_driver(myaddr, xbee_tx, incoming_packet);

    shell_init(&shell, NULL, UART0_BUFSIZE, shell_readc, shell_putchar);

    shell_run(&shell);
    return 0;
}
