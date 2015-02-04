#ifndef ti_emac_interface_h
#define ti_emac_interface_h

#ifndef PART_TM4C1294NCPDT
#define PART_TM4C1294NCPDT
#endif

#include "driverlib/pin_map.h"

/* PHY LED config */
#define LINK_LED      GPIO_PF0_EN0LED0
#define LINK_LED_BASE GPIOF_AHB_BASE
#define LINK_LED_PIN  GPIO_PIN_0
#define ACT_LED       GPIO_PF4_EN0LED1
#define ACT_LED_BASE  GPIOF_AHB_BASE
#define ACT_LED_PIN   GPIO_PIN_4

/* DMA descriptor management structure */
typedef struct {
  tEMACDMADescriptor Desc;
  struct tpbuffer *pBuf;
} tDescriptor;

/* DMA descriptor list */
typedef struct {
    tDescriptor *pDescriptors;
    uint32_t ui32NumDescs;
    uint32_t ui32Read;
    uint32_t ui32Write;
} tDescriptorList;

/* Packet buffer structure */
typedef struct tpbuffer {
	struct tpbuffer *next;
	uint8_t  p[TI_EMAC_BUFFER_SIZE];
	uint16_t len;
	uint8_t  free;
	uint16_t tot_len;
} tpbuffer;


/* Buffer management functions */
tpbuffer *ti_emac_alloc_buf(tpbuffer *pool);
void ti_emac_free_buf(tpbuffer *buf);
void ti_emac_buf_cat(tpbuffer *a, tpbuffer *b);
void ti_emac_init_bufs(void);

/* ISR helpers */
void process_phy_interrupt(void);
void process_tx_interrupt(void);
void process_rx_interrupt(void);

/* Clean up a packet and pass it up to the transceiver thread */
void handle_packet(tpbuffer* p);

/* Send a packet out of the interface */
uint32_t ti_emac_transmit(tpbuffer *p);

#endif
