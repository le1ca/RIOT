/*
 * Copyright (C) 2014 INRIA
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @defgroup    driver_xbee XBee driver
 * @ingroup     drivers
 * @brief       High-level driver for the XBee 802.15.4 modem
 * @{
 *
 * @file
 * @brief       High-level driver for the XBee 802.15.4 modem
 *
 * @author      Kévin Roussel <kevin.roussel@inria.fr>
 */

#ifndef __XBEE_H
#define __XBEE_H

#include "radio_driver.h"
#include "ieee802154_frame.h"
#include "kernel_types.h"
#include "ringbuffer.h"

#define XBEE_MAX_PKT_DATA_LENGTH  100
#define XBEE_RECV_BUF_SIZE    128
#define TRANSCEIVER_BUFFER_SIZE 128


/**
 * XBee low-level radio driver definition.
 */
extern const ieee802154_radio_driver_t xbee_radio_driver;
extern ringbuffer_t xbee_pkt_ringbuffer;

/* data for an incoming packet */
// Would be defined in xbee.c but transceiver.c needs access to this type.
typedef struct xbee_incoming_packet {
	char buf[XBEE_RECV_BUF_SIZE];
	unsigned int len;
	int8_t rssi; 
	uint8_t lqi;
	bool crc_ok;
    uint8_t processing;
    ieee802154_node_addr_t src;
} xbee_incoming_packet_t; 

// All the public functions
void xbee_init(kernel_pid_t tpid);
int8_t xbee_send(ieee802154_packet_t *pkt);
int32_t xbee_set_channel(unsigned int chan);
unsigned int xbee_get_channel(void);
uint16_t xbee_get_pan_id(void);
uint16_t xbee_set_pan_id(uint16_t pan);
uint16_t xbee_get_address(void);
uint16_t xbee_set_address(uint16_t addr);
uint64_t xbee_get_long_address(void);
uint64_t xbee_set_long_address(uint64_t addr);
void xbee_set_monitor(bool monitor);
void xbee_off(void);
void xbee_switch_to_rx(void);

#endif /* __XBEE_H */
/** @} */
