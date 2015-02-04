#include <string.h>
#include "ti_emac.h"
#include "transceiver.h"
#include "ti_emac-interface.h"
#include "hwtimer.h"
#include "config.h"
#include "cpu.h"
#include "netdev/base.h"
#include "driverlib/rom.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"

/* our mac address */
uint8_t  ti_emac_pmac[8];

/* Rx buffer */
ethernet_frame _ti_emac_rx_buffer[RX_BUF_SIZE];
static volatile uint8_t rx_buffer_next;

/* DMA descriptors */
tDescriptor g_pTxDescriptors[TI_EMAC_NUM_TX_DESCRIPTORS];
tDescriptor g_pRxDescriptors[TI_EMAC_NUM_RX_DESCRIPTORS];

/* DMA descriptor lists */
tDescriptorList g_TxDescList = {
    g_pTxDescriptors, TI_EMAC_NUM_TX_DESCRIPTORS, 0, 0
};
tDescriptorList g_RxDescList = {
    g_pRxDescriptors, TI_EMAC_NUM_RX_DESCRIPTORS, 0, 0
};

/* Packet buffers */
tpbuffer rxbuffers[TI_EMAC_NUM_RX_BUFFERS];
tpbuffer txbuffers[TI_EMAC_NUM_TX_BUFFERS];

/* Prints headers and content of a packet */
void ti_emac_print_pkt_debug(ethernet_frame *p){
	uint16_t i;
	
	printf("Got ethernet frame:\n");
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
	printf("Payload: \n");

	for (i = 0; i < p->plen; i++) {
		printf("%02X ", p->data[i]);
		if((i+1) % 16 == 0)
			printf("\n");
	}
	
	if((i+1) % 16 != 0)
		printf("\n");
	printf("\n");
}

/* Initialize buffers (mark all as free) */
inline void ti_emac_init_bufs(void){
	int i;
	for(i = 0; i < TI_EMAC_NUM_RX_BUFFERS; i++){
		rxbuffers[i].len     = TI_EMAC_BUFFER_SIZE;
		rxbuffers[i].free    = 1;
		rxbuffers[i].next    = 0;
		rxbuffers[i].tot_len = TI_EMAC_BUFFER_SIZE;
	}
	for(i = 0; i < TI_EMAC_NUM_TX_BUFFERS; i++){
		txbuffers[i].len     = TI_EMAC_BUFFER_SIZE;
		txbuffers[i].free    = 1;
		txbuffers[i].next    = 0;
		txbuffers[i].tot_len = TI_EMAC_BUFFER_SIZE;
	}
}

/* Find a free buffer and clear its free flag */
tpbuffer *ti_emac_alloc_buf(tpbuffer pool[]){
	tpbuffer *r = 0;
	uint8_t i;
	for(i = 0; i < TI_EMAC_NUM_RX_BUFFERS; i++){
		if(pool[i].free){
			r = &pool[i];
			r->len = TI_EMAC_BUFFER_SIZE;
			r->tot_len = TI_EMAC_BUFFER_SIZE;
			r->next = 0;
			r->free = 0;
			break;
		}
	}
	return r;
}

/* Restore the free flag of a buffer */
void ti_emac_free_buf(tpbuffer *buf){
	if(buf->next)
		ti_emac_free_buf(buf->next);

	buf->len = TI_EMAC_BUFFER_SIZE;
	buf->tot_len = TI_EMAC_BUFFER_SIZE;
	buf->next = 0;
	buf->free = 1;
	
}

/* Chain two buffers together */
void ti_emac_buf_cat(tpbuffer *h, tpbuffer *t){
	tpbuffer *p;
	for (p = h; p->next != NULL; p = p->next) {
		p->tot_len += t->tot_len;
  	}
	p->tot_len += t->tot_len;
  	p->next = t;
}

/* Pass a packet up to the transceiver thread */
inline void handle_packet(tpbuffer *p){
	struct eth_hdr *ethhdr = (struct eth_hdr *)p->p;
	ethhdr->type = ntohs(ethhdr->type);

	/* copy header */
    memcpy(&_ti_emac_rx_buffer[rx_buffer_next].hdr, ethhdr, sizeof(struct eth_hdr));
    
    /* copy data */
    uint16_t bsf = 0;	// bytes read so far (position in destination buffer)
    tpbuffer *q = p;    // pointer to current chunk of frame
    
    while(q){
    	uint16_t foffset = 0, eoffset = 0;
    	
    	// if this is the first chunk of the frame, skip past the header
    	if(bsf == 0)
    		foffset = sizeof(struct eth_hdr);
    		
    	// if this is the last chunk of the frame, skip the crc
    	if(!q->next)
    		eoffset = 4;
    		
    	if(foffset > q->len){
    		printf("[emac] frame chunk smaller than header (len %d vs. %d)\n", q->len, foffset);
    	}
    		
		// pointer to start of payload
		uint8_t *payload = (uint8_t *) (q->p + foffset);
    	uint16_t len = q->len - (foffset + eoffset);
    	
    	if(bsf + len > 1500){
    		printf("[emac] got oversized frame (at least %d bytes)\n", bsf + len);
    		break;
    	}
    		
	    memcpy(&_ti_emac_rx_buffer[rx_buffer_next].data[bsf], payload, len);
	    
	    bsf += len;
	    q = q->next;
	}
    
    _ti_emac_rx_buffer[rx_buffer_next].plen = bsf;
    _ti_emac_rx_buffer[rx_buffer_next].processing = 0;

	/* send frame to transceiver thread */
    if (transceiver_pid != KERNEL_PID_UNDEF) {
        msg_t m;
        m.type = (uint16_t) RCV_PKT_TI_EMAC;
        m.content.value = rx_buffer_next;
        msg_send_int(&m, transceiver_pid);
        //notified = 1;
    }
    
    //ti_emac_print_pkt_debug(&_ti_emac_rx_buffer[rx_buffer_next]);
    
    if (++rx_buffer_next == RX_BUF_SIZE) {
        rx_buffer_next = 0;
    }
}

#ifdef MODULE_TRANSCEIVER
/* Primary init entry point (when transceiver module is present)*/
void ti_emac_init(kernel_pid_t tpid){
    transceiver_pid = tpid;
    rx_buffer_next = 0;
    ti_emac_initialize(NULL);
}
#endif

/* Initialize DMA descriptors and pass them to the hardware */
inline void ti_emac_dma_init(void){
	uint32_t ui32Loop;
	
	ti_emac_init_bufs();

    /* Transmit list -  mark all descriptors as not owned by the hardware */
   for(ui32Loop = 0; ui32Loop < TI_EMAC_NUM_TX_DESCRIPTORS; ui32Loop++){
       g_pTxDescriptors[ui32Loop].pBuf = (struct tpbuffer *)0;
       g_pTxDescriptors[ui32Loop].Desc.ui32Count = 0;
       g_pTxDescriptors[ui32Loop].Desc.pvBuffer1 = 0;
       g_pTxDescriptors[ui32Loop].Desc.DES3.pLink =
               ((ui32Loop == (TI_EMAC_NUM_TX_DESCRIPTORS - 1)) ?
               &g_pTxDescriptors[0].Desc : &g_pTxDescriptors[ui32Loop + 1].Desc);
       g_pTxDescriptors[ui32Loop].Desc.ui32CtrlStatus = DES0_TX_CTRL_INTERRUPT |
               DES0_TX_CTRL_CHAINED | DES0_TX_CTRL_IP_ALL_CKHSUMS;

   }

   g_TxDescList.ui32Read = 0;
   g_TxDescList.ui32Write = 0;

   /* Receive list -  tag each descriptor with a pbuf and set all fields to
    * allow packets to be received.
    */
  for(ui32Loop = 0; ui32Loop < TI_EMAC_NUM_RX_DESCRIPTORS; ui32Loop++){
      g_pRxDescriptors[ui32Loop].pBuf = ti_emac_alloc_buf(rxbuffers);      
      g_pRxDescriptors[ui32Loop].Desc.ui32Count = DES1_RX_CTRL_CHAINED;
      if(g_pRxDescriptors[ui32Loop].pBuf){
          /* Set the DMA to write directly into the pbuf payload. */
          g_pRxDescriptors[ui32Loop].Desc.pvBuffer1  = g_pRxDescriptors[ui32Loop].pBuf->p;
          g_pRxDescriptors[ui32Loop].Desc.ui32Count |= (g_pRxDescriptors[ui32Loop].pBuf->len << DES1_RX_CTRL_BUFF1_SIZE_S);
          g_pRxDescriptors[ui32Loop].Desc.ui32CtrlStatus = DES0_RX_CTRL_OWN;
      }
      else{
          g_pRxDescriptors[ui32Loop].Desc.pvBuffer1 = 0;
          g_pRxDescriptors[ui32Loop].Desc.ui32CtrlStatus = 0;
      }
      g_pRxDescriptors[ui32Loop].Desc.DES3.pLink =
              ((ui32Loop == (TI_EMAC_NUM_RX_DESCRIPTORS - 1)) ?
              &g_pRxDescriptors[0].Desc : &g_pRxDescriptors[ui32Loop + 1].Desc);
  }

  //g_RxDescList.ui32Read = 0;
  //g_RxDescList.ui32Write = 0;

  //
  // Set the descriptor pointers in the hardware.
  //
  ROM_EMACRxDMADescriptorListSet(EMAC0_BASE, &g_pRxDescriptors[0].Desc);
  ROM_EMACTxDMADescriptorListSet(EMAC0_BASE, &g_pTxDescriptors[0].Desc);
}

/* Initialize the EMAC hardware */
int ti_emac_initialize(netdev_t *dev){
	uint32_t umac0, umac1;
	
	// get mac and convert to byte array
	ROM_FlashUserGet(&umac0, &umac1);
    if((umac0 == 0xffffffff) || (umac1 == 0xffffffff)){
        puts("[emac] no MAC configured in user registers - not enabling emac0");
        return -1;
    }
	ti_emac_pmac[0] = ((umac0 >>  0) & 0xff);
	ti_emac_pmac[1] = ((umac0 >>  8) & 0xff);
	ti_emac_pmac[2] = ((umac0 >> 16) & 0xff);
	ti_emac_pmac[3] = ((umac1 >>  0) & 0xff);
	ti_emac_pmac[4] = ((umac1 >>  8) & 0xff);
	ti_emac_pmac[5] = ((umac1 >> 16) & 0xff);
    	
	printf("[emac] mac addr %02x:%02x:%02x:%02x:%02x:%02x\n", ti_emac_pmac[0], ti_emac_pmac[1], ti_emac_pmac[2], ti_emac_pmac[3], ti_emac_pmac[4], ti_emac_pmac[5]);
    
    // enable peripherals
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_EMAC0);
    ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_EMAC0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_EPHY0);
	ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_EPHY0);
	
	// wait for peripherals to be ready
	while(!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_EMAC0));
	
	// configure LEDs
	ROM_GPIOPinConfigure(LINK_LED);
	GPIOPinTypeEthernetLED(LINK_LED_BASE, LINK_LED_PIN);
	ROM_GPIOPinConfigure(ACT_LED);
	GPIOPinTypeEthernetLED(ACT_LED_BASE, ACT_LED_PIN);
    
    // configure internal emac
    ROM_EMACPHYConfigSet(EMAC0_BASE, TI_EMAC_PHY_CONFIG);
    ROM_EMACInit(EMAC0_BASE, ti_clock_hz,
                 EMAC_BCONFIG_MIXED_BURST | EMAC_BCONFIG_PRIORITY_FIXED,
                 4, 4, 0);
	ROM_EMACConfigSet(EMAC0_BASE, (EMAC_CONFIG_FULL_DUPLEX |
                                   EMAC_CONFIG_CHECKSUM_OFFLOAD |
                                   EMAC_CONFIG_7BYTE_PREAMBLE |
                                   EMAC_CONFIG_IF_GAP_96BITS |
                                   EMAC_CONFIG_USE_MACADDR0 |
                                   EMAC_CONFIG_SA_FROM_DESCRIPTOR |
                                   EMAC_CONFIG_BO_LIMIT_1024),
                      (EMAC_MODE_RX_STORE_FORWARD |
                       EMAC_MODE_TX_STORE_FORWARD |
                       EMAC_MODE_TX_THRESHOLD_64_BYTES |
                       EMAC_MODE_RX_THRESHOLD_64_BYTES), 0);
	ROM_EMACAddrSet(EMAC0_BASE, 0, (uint8_t *) ti_emac_pmac);

	// set up DMA
	ti_emac_dma_init();
	
	// clear stray interrupts
	ROM_EMACPHYRead(EMAC0_BASE, TI_EMAC_PHY_PHYS_ADDR, EPHY_MISR1);
  	ROM_EMACPHYRead(EMAC0_BASE, TI_EMAC_PHY_PHYS_ADDR, EPHY_MISR2);

	// configure interrupts for status change
	uint16_t status;
  	status = ROM_EMACPHYRead(EMAC0_BASE, TI_EMAC_PHY_PHYS_ADDR, EPHY_SCR);
  	status |= (EPHY_SCR_INTEN_EXT | EPHY_SCR_INTOE_EXT);
	ROM_EMACPHYWrite(EMAC0_BASE, TI_EMAC_PHY_PHYS_ADDR, EPHY_SCR, status);
	ROM_EMACPHYWrite(EMAC0_BASE, TI_EMAC_PHY_PHYS_ADDR, EPHY_MISR1, (EPHY_MISR1_LINKSTATEN |
               EPHY_MISR1_SPEEDEN | EPHY_MISR1_DUPLEXMEN | EPHY_MISR1_ANCEN));

	// clear stray interrupts
  	ROM_EMACPHYRead(EMAC0_BASE, TI_EMAC_PHY_PHYS_ADDR, EPHY_MISR1);

	// enable mac filtering - allow broadcast, multicast, and unicast for us
	ROM_EMACFrameFilterSet(EMAC0_BASE, (EMAC_FRMFILTER_HASH_AND_PERFECT |
                     EMAC_FRMFILTER_PASS_MULTICAST));
                     
	// TODO: Precision time protocol
	//ROM_EMACTimestampConfigSet(EMAC0_BASE, (EMAC_TS_ALL_RX_FRAMES |
    //                     EMAC_TS_DIGITAL_ROLLOVER |
    //                     EMAC_TS_PROCESS_IPV4_UDP | EMAC_TS_ALL |
    //                     EMAC_TS_PTP_VERSION_1 | EMAC_TS_UPDATE_FINE),
    //                     (1000000000 / (25000000 / 2)));
	//ROM_EMACTimestampAddendSet(EMAC0_BASE, 0x80000000);
	//ROM_EMACTimestampEnable(EMAC0_BASE);
                     
	// clear interrupts 
	ROM_EMACIntClear(EMAC0_BASE, ROM_EMACIntStatus(EMAC0_BASE, false));
                     
	// enable tx and rx
	ROM_EMACTxEnable(EMAC0_BASE);
	ROM_EMACRxEnable(EMAC0_BASE);

	// enable tx, rx interrupts
	ROM_EMACIntEnable(EMAC0_BASE, (EMAC_INT_RECEIVE | EMAC_INT_TRANSMIT |
                EMAC_INT_TX_STOPPED | EMAC_INT_RX_NO_BUFFER |
                EMAC_INT_RX_STOPPED | EMAC_INT_PHY));
	NVIC_SetPriority(EMAC0_IRQn, 0xc0);
    NVIC_EnableIRQ(EMAC0_IRQn);

	// negotiate link
	ROM_EMACPHYWrite(EMAC0_BASE, TI_EMAC_PHY_PHYS_ADDR, EPHY_BMCR, (EPHY_BMCR_ANEN |
               EPHY_BMCR_RESTARTAN));
	
	printf("[emac] init finished\n");

    return 0;
}

/* TODO: place packet in tx buffers */
int ti_emac_send(ethernet_frame *p){
	tpbuffer *a = ti_emac_alloc_buf(txbuffers);
	uint32_t r;
	struct eth_hdr *aa = (struct eth_hdr *) a->p;
	
	
	a->len     = p->plen + sizeof(struct eth_hdr);
	a->tot_len = a->len;
	memcpy(a->p+sizeof(struct eth_hdr), p->data, p->plen);
	memcpy(aa, &p->hdr, sizeof(struct eth_hdr));
	memcpy(aa->src, ti_emac_pmac, 6);
	
	ti_emac_print_pkt_debug(a);
	r = ti_emac_transmit(a);
	return 1;
}

/* Does nothing */
int ti_emac_set_channel(uint8_t c){
	return 0;
}

/* Does nothing */
uint8_t ti_emac_get_channel(void){
	return 0;
}

/* Does nothing */
int ti_emac_set_address(radio_address_t addr){
	return 0;
}

/* Does nothing */
radio_address_t ti_emac_get_address(void){
	return 0;
}

/* Does nothing */
void ti_emac_set_monitor(uint8_t mode){
	(void) mode;
}

/* TODO */
void ti_emac_powerdown(void){

}

/* TODO */
void ti_emac_switch_to_rx(void){
	
}

/* Main ISR for emac interrupt */
void isr_emac0(void){

	uint32_t state = disableIRQ();

	uint32_t status = ROM_EMACIntStatus(EMAC0_BASE, true);
	if(status){
		ROM_EMACIntClear(EMAC0_BASE, status);
	}
	
	if(status & EMAC_INT_NORMAL_INT){
		//printf("[emac0] normal int\n");
	}
	
	if(status & EMAC_INT_ABNORMAL_INT){
		printf("[emac0] abnormal int\n");
	}
	
	if(status & EMAC_INT_PHY){
      process_phy_interrupt();
	}

	if(status & EMAC_INT_TRANSMIT){
        process_tx_interrupt();
	}

	if(status & (EMAC_INT_RECEIVE | EMAC_INT_RX_NO_BUFFER | EMAC_INT_RX_STOPPED)){
        process_rx_interrupt();
	}
	
	restoreIRQ(state);	
}

/* PHY interrupt: renegotiate link parameters */
inline void process_phy_interrupt(void){
    uint16_t ui16Val, ui16Status;
    uint32_t ui32Config, ui32Mode, ui32RxMaxFrameSize;

    /* Read the PHY interrupt status.  This clears all interrupt sources.
     * Note that we are only enabling sources in EPHY_MISR1 so we don't
     * read EPHY_MISR2.
     */
    ui16Val = ROM_EMACPHYRead(EMAC0_BASE, TI_EMAC_PHY_PHYS_ADDR, EPHY_MISR1);

    /* Read the current PHY status. */
    ui16Status = ROM_EMACPHYRead(EMAC0_BASE, TI_EMAC_PHY_PHYS_ADDR, EPHY_STS);

    /* Has the link status changed? */
    if(ui16Val & EPHY_MISR1_LINKSTAT){
        /* Is link up or down now? */
        if(ui16Status & EPHY_STS_LINK){
            // netif_set_link_up(psNetif);
        }
        else{
            // netif_set_link_down(psNetif);
        }
    }

    /* Has the speed or duplex status changed? */
    if(ui16Val & (EPHY_MISR1_SPEED | EPHY_MISR1_SPEED | EPHY_MISR1_ANC)){
        /* Get the current MAC configuration. */
        ROM_EMACConfigGet(EMAC0_BASE, &ui32Config, &ui32Mode,
                        &ui32RxMaxFrameSize);

        /* What speed is the interface running at now?
         */
        if(ui16Status & EPHY_STS_SPEED){
            /* 10Mbps is selected */
            ui32Config &= ~EMAC_CONFIG_100MBPS;
        }
        else{
            /* 100Mbps is selected */
            ui32Config |= EMAC_CONFIG_100MBPS;
        }

        /* Are we in fui32l- or half-duplex mode? */
        if(ui16Status & EPHY_STS_DUPLEX){
            /* Fui32l duplex. */
            ui32Config |= EMAC_CONFIG_FULL_DUPLEX;
        }
        else{
            /* Half duplex. */
            ui32Config &= ~EMAC_CONFIG_FULL_DUPLEX;
        }

        /* Reconfigure the MAC */
        ROM_EMACConfigSet(EMAC0_BASE, ui32Config, ui32Mode, ui32RxMaxFrameSize);
    }
}

/* Tx interrupt: reclaim buffers that we gave to the DMA for transmission */
inline void process_tx_interrupt(void){
    tDescriptorList *pDescList;
    uint32_t ui32NumDescs;

    /* Get a pointer to the transmit descriptor list. */
    pDescList = &g_TxDescList;

    /* Walk the list until we have checked all descriptors or we reach the
     * write pointer or find a descriptor that the hardware is still working
     * on.
     */
    for(ui32NumDescs = 0; ui32NumDescs < pDescList->ui32NumDescs; ui32NumDescs++){
    
        /* Has the buffer attached to this descriptor been transmitted? */
        if(pDescList->pDescriptors[pDescList->ui32Read].Desc.ui32CtrlStatus & DES0_TX_CTRL_OWN){
            /* No - we're finished. */
            break;
        }

        /* Does this descriptor have a buffer attached to it? */
        if(pDescList->pDescriptors[pDescList->ui32Read].pBuf) {
            /* Yes - free it if it's not marked as an intermediate pbuf */
            if(!((uint32_t)(pDescList->pDescriptors[pDescList->ui32Read].pBuf) & 1))
            {
                ti_emac_free_buf(pDescList->pDescriptors[pDescList->ui32Read].pBuf);
                pDescList->pDescriptors[pDescList->ui32Read].pBuf = NULL;
                //DRIVER_STATS_INC(TXBufFreedCount);
            }
        }
        else{
            /* If the descriptor has no buffer, we are finished. */
            break;
        }

        /* Move on to the next descriptor. */
        pDescList->ui32Read++;
        if(pDescList->ui32Read == pDescList->ui32NumDescs){
            pDescList->ui32Read = 0;
        }
    }
}

/* Rx interrupt: process buffers that the DMA gave to us */
inline void process_rx_interrupt(void){
  tDescriptorList *pDescList;
  struct tpbuffer *pBuf;
  uint32_t ui32DescEnd;

  /* Get a pointer to the receive descriptor list. */
  pDescList = &g_RxDescList;

  /* Start with a NULL pbuf so that we don't try to link chain the first
   * time round.
   */
  pBuf = NULL;

  /* Determine where we start and end our walk of the descriptor list */
  ui32DescEnd = pDescList->ui32Read ? (pDescList->ui32Read - 1) : (pDescList->ui32NumDescs - 1);

  /* Step through the descriptors that are marked for CPU attention. */
  while(pDescList->ui32Read != ui32DescEnd){
  
      /* Does the current descriptor have a buffer attached to it? */
      if(pDescList->pDescriptors[pDescList->ui32Read].pBuf){
      
          /* Yes - determine if the host has filled it yet. */
          if(pDescList->pDescriptors[pDescList->ui32Read].Desc.ui32CtrlStatus & DES0_RX_CTRL_OWN){
              /* The DMA engine still owns the descriptor so we are finished */
              break;
          }

          //DRIVER_STATS_INC(RXBufReadCount);

          /* If this descriptor contains the end of the packet, fix up the
           * buffer size accordingly.
           */
          if(pDescList->pDescriptors[pDescList->ui32Read].Desc.ui32CtrlStatus & DES0_RX_STAT_LAST_DESC){
              /* This is the last descriptor for the frame so fix up the
               * length.  It is safe for us to modify the internal fields
               * directly here (rather than calling pbuf_realloc) since we
               * know each of these pbufs is never chained.
               */
              //printf("[emac] rx last_desc len %d\n",  pDescList->pDescriptors[pDescList->ui32Read].pBuf->len);
              pDescList->pDescriptors[pDescList->ui32Read].pBuf->len =
                       (pDescList->pDescriptors[pDescList->ui32Read].Desc.ui32CtrlStatus &
                        DES0_RX_STAT_FRAME_LENGTH_M) >>
                        DES0_RX_STAT_FRAME_LENGTH_S;
              pDescList->pDescriptors[pDescList->ui32Read].pBuf->tot_len =
                        pDescList->pDescriptors[pDescList->ui32Read].pBuf->len;
          }

          if(pBuf){
              /* Link this pbuf to the last one we looked at since this buffer
               * is a continuation of an existing frame (split across mui32tiple
               * pbufs).  Note that we use pbuf_cat() here rather than
               * pbuf_chain() since we don't want to increase the reference
               * count of either pbuf - we only want to link them together.
               */
               // TODO: fix this because it's totally screwed
               // then we can make the buffers smaller, too
               //printf("[emac] linking buffer %x (len %d) to %x (len %d)\n", pBuf, pBuf->len,
               //	pDescList->pDescriptors[pDescList->ui32Read].pBuf, pDescList->pDescriptors[pDescList->ui32Read].pBuf->len
               //);
              ti_emac_buf_cat(pBuf, pDescList->pDescriptors[pDescList->ui32Read].pBuf);
              pDescList->pDescriptors[pDescList->ui32Read].pBuf = pBuf;
          }

          /* Remember the buffer associated with this descriptor. */
          pBuf = pDescList->pDescriptors[pDescList->ui32Read].pBuf;

          /* Is this the last descriptor for the current frame? */
          if(pDescList->pDescriptors[pDescList->ui32Read].Desc.ui32CtrlStatus & DES0_RX_STAT_LAST_DESC){
              /* Yes - does the frame contain errors? */
              if(pDescList->pDescriptors[pDescList->ui32Read].Desc.ui32CtrlStatus & DES0_RX_STAT_ERR){
                  /* This is a bad frame so discard it and update the relevant
                   * statistics.
                   */
                  //LWIP_DEBUGF(NETIF_DEBUG, ("tivaif_receive: packet error\n"));
                  ti_emac_free_buf(pBuf);
                  //LINK_STATS_INC(link.drop);
                  //DRIVER_STATS_INC(RXPacketErrCount);
                  puts("[emac] rx bad frame");
              }
              else
              {
                  /* This is a good frame so pass it up the stack. */
                  //LINK_STATS_INC(link.recv);
                  //DRIVER_STATS_INC(RXPacketReadCount);
                  handle_packet(pBuf);
                  ti_emac_free_buf(pBuf);

				
                  /* Place the timestamp in the PBUF if PTPD is enabled */
                  //pBuf->time_s =
                  //     pDescList->pDescriptors[pDescList->ui32Read].Desc.ui32IEEE1588TimeHi;
                  //pBuf->time_ns =
                  //     pDescList->pDescriptors[pDescList->ui32Read].Desc.ui32IEEE1588TimeLo;
				

                  //if(ethernet_input(pBuf, psNetif) != ERR_OK){

                      /* drop the packet */
                      //LWIP_DEBUGF(NETIF_DEBUG, ("tivaif_input: input error\n"));
                      //pbuf_free(pBuf);

                      /* Adjust the link statistics */
                      //LINK_STATS_INC(link.memerr);
                      //LINK_STATS_INC(link.drop);
                      //DRIVER_STATS_INC(RXPacketCBErrCount);
                  //}

                  /* We're finished with this packet so make sure we don't try
                   * to link the next buffer to it.
                   */
                  pBuf = NULL;
              }
          }
      }

      /* Allocate a new buffer for this descriptor */
      pDescList->pDescriptors[pDescList->ui32Read].pBuf = ti_emac_alloc_buf(rxbuffers);
      pDescList->pDescriptors[pDescList->ui32Read].Desc.ui32Count = DES1_RX_CTRL_CHAINED;
      if(pDescList->pDescriptors[pDescList->ui32Read].pBuf)
      {
          /* We got a buffer so fill in the payload pointer and size. */
          pDescList->pDescriptors[pDescList->ui32Read].Desc.pvBuffer1 =
                              pDescList->pDescriptors[pDescList->ui32Read].pBuf->p;
          pDescList->pDescriptors[pDescList->ui32Read].Desc.ui32Count |=
                              (pDescList->pDescriptors[pDescList->ui32Read].pBuf->len <<
                               DES1_RX_CTRL_BUFF1_SIZE_S);

          /* Give this descriptor back to the hardware */
          pDescList->pDescriptors[pDescList->ui32Read].Desc.ui32CtrlStatus =
                              DES0_RX_CTRL_OWN;
                              
      }
      else
      {
          //LWIP_DEBUGF(NETIF_DEBUG, ("tivaif_receive: pbuf_alloc error\n"));
          pDescList->pDescriptors[pDescList->ui32Read].Desc.pvBuffer1 = 0;

          /* Update the stats to show we coui32dn't allocate a pbuf. */
          //DRIVER_STATS_INC(RXNoBufCount);
          //LINK_STATS_INC(link.memerr);

          /* Stop parsing here since we can't leave a broken descriptor in
           * the chain.
           */
          break;
      }

      /* Move on to the next descriptor in the chain, taking care to wrap. */
      pDescList->ui32Read++;
      if(pDescList->ui32Read == pDescList->ui32NumDescs)
      {
          pDescList->ui32Read = 0;
      }
  }
}

/* Low-level transmit function */
uint32_t ti_emac_transmit(tpbuffer *p){
  tDescriptor *pDesc;
  tpbuffer *pBuf;
  uint32_t ui32NumChained;
  bool bFirst;
  uint32_t irq_state = disableIRQ();

  //LWIP_DEBUGF(NETIF_DEBUG, ("tivaif_transmit 0x%08x, len %d\n", p,
  //            p->tot_len));
  /* Update our transmit attempt counter. */

  /* Make sure that the transmit descriptors are not all in use */
  pDesc = &(g_TxDescList.pDescriptors[g_TxDescList.ui32Write]);
  if(pDesc->pBuf){
      /**
       * The current write descriptor has a pbuf attached to it so this
       * implies that the ring is fui32l. Reject this transmit request with a
       * memory error since we can't satisfy it just now.
       */
      printf("[emac] tx failed - no available buffers\n");
      ti_emac_free_buf(p);
      restoreIRQ(irq_state);
      return 1;
  }

  /* How many pbufs are in the chain passed? */
  // TODO: fix chaining. right now we are just making our buffers big enough for the whole frame
  //ui32NumChained = (uint32_t)pbuf_clen(p);
  ui32NumChained = 1;

  /* How many free transmit descriptors do we have? */
  //ui32NumDescs = (pIF->pTxDescList->ui32Read > pIF->pTxDescList->ui32Write) ?
  //        (pIF->pTxDescList->ui32Read - pIF->pTxDescList->ui32Write) :
  //        ((NUM_TX_DESCRIPTORS - pIF->pTxDescList->ui32Write) +
  //         pIF->pTxDescList->ui32Read);

  /* Do we have enough free descriptors to send the whole packet? */
  //if(ui32NumDescs < ui32NumChained)
  //{
      /* No - we can't transmit this whole packet so return an error. */
  //    pbuf_free(p);
  //    LINK_STATS_INC(link.memerr);
  //    DRIVER_STATS_INC(TXNoDescCount);
  //    SYS_ARCH_UNPROTECT(lev);
  //    return (ERR_MEM);
  //}

  /* Tag the first descriptor as the start of the packet. */
  bFirst = true;
  pDesc->Desc.ui32CtrlStatus = DES0_TX_CTRL_FIRST_SEG;

  /* Here, we know we can send the packet so write it to the descriptors */
  pBuf = p;

  while(ui32NumChained)
  {
  
      /* Get a pointer to the descriptor we will write next. */
      pDesc = &(g_TxDescList.pDescriptors[g_TxDescList.ui32Write]);

      /* Fill in the buffer pointer and length */
      pDesc->Desc.ui32Count = (uint32_t)pBuf->len;
      pDesc->Desc.pvBuffer1 = pBuf->p;

      /* Tag the first descriptor as the start of the packet. */
      if(bFirst)
      {
          bFirst = false;
          pDesc->Desc.ui32CtrlStatus = DES0_TX_CTRL_FIRST_SEG;
      }
      else
      {
          pDesc->Desc.ui32CtrlStatus = 0;
      }

      pDesc->Desc.ui32CtrlStatus |= (DES0_TX_CTRL_IP_ALL_CKHSUMS |
                                     DES0_TX_CTRL_CHAINED);

      /* Decrement our descriptor counter, move on to the next buffer in the
       * pbuf chain. */
      ui32NumChained--;
      pBuf = pBuf->next;

      /* Update the descriptor list write index. */
      g_TxDescList.ui32Write++;
      if(g_TxDescList.ui32Write == TI_EMAC_NUM_TX_DESCRIPTORS)
      {
          g_TxDescList.ui32Write = 0;
      }

      /* If this is the last descriptor, mark it as the end of the packet. */
      if(!ui32NumChained)
      {
          pDesc->Desc.ui32CtrlStatus |= (DES0_TX_CTRL_LAST_SEG |
                                         DES0_TX_CTRL_INTERRUPT);
		  
          /* Tag the descriptor with the original pbuf pointer. */
          pDesc->pBuf = p;
      }
      else
      {
          /* Set the lsb of the pbuf pointer.  We use this as a signal that
           * we should not free the pbuf when we are walking the descriptor
           * list while processing the transmit interrupt.  We only free the
           * pbuf when processing the last descriptor used to transmit its
           * chain.
           */
          //pDesc->pBuf = (tpbuffer *)((uint32_t)p + 1);
          //printf("[emac] pbuf addr now %08lx\n", pDesc->pBuf);
      }

      /* Hand the descriptor over to the hardware. */
      pDesc->Desc.ui32CtrlStatus |= DES0_TX_CTRL_OWN;
  }

  /* Tell the transmitter to start (in case it had stopped). */
  ROM_EMACTxDMAPollDemand(EMAC0_BASE);

  restoreIRQ(irq_state);

  return 0;
}
