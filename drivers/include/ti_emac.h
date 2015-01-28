#ifndef TI_EMAC_H
#define TI_EMAC_H

#ifdef MODULE_TI_EMAC

#include <stdint.h>
#include "kernel.h"
#include "netdev/base.h"
#include "radio/types.h"
#include "driverlib/emac.h"
#include "driverlib/hw_emac.h"

#define TI_EMAC_NUM_RX_DESCRIPTORS 4
#define TI_EMAC_NUM_TX_DESCRIPTORS 8
#define TI_EMAC_NUM_RX_BUFFERS 10
#define TI_EMAC_NUM_TX_BUFFERS 10
#define TI_EMAC_BUFFER_SIZE 512
#define TI_EMAC_PHY_PHYS_ADDR 0
#define TI_EMAC_MAX_DATA_LENGTH 1500

#define RX_BUF_SIZE (10)
#define TRANSCEIVER_BUFFER_SIZE 10

#define TI_EMAC_PHY_CONFIG (EMAC_PHY_TYPE_INTERNAL | EMAC_PHY_INT_MDIX_EN | EMAC_PHY_AN_100B_T_FULL_DUPLEX)

struct rx_buffer_s {
    radio_packet_t packet;
    char data[TI_EMAC_MAX_DATA_LENGTH];
};

extern struct rx_buffer_s _ti_emac_rx_buffer[RX_BUF_SIZE];

void ti_emac_init(kernel_pid_t tpid);
int ti_emac_initialize(netdev_t *dev);
int ti_emac_send(radio_packet_t *p);
int ti_emac_set_channel(uint8_t c);
uint8_t ti_emac_get_channel(void);
int ti_emac_set_address(radio_address_t addr);
radio_address_t ti_emac_get_address(void);
void ti_emac_set_monitor(uint8_t mode);
void ti_emac_powerdown(void);
void ti_emac_switch_to_rx(void);

#endif

#endif
