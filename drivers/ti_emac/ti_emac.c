
#include "ti_emac.h"

/*

#include "cc110x-internal.h"

#include "periph/gpio.h"
#include "periph/spi.h"

*/

#include "transceiver.h"
#include "hwtimer.h"
#include "config.h"
#include "cpu.h"
#include "netdev/base.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

/* Internal function prototypes */
/*
static int rd_set_mode(int mode);
static void reset(void);
static void power_up_reset(void);
static void write_register(uint8_t r, uint8_t value)
*/;

/* External variables */

//extern uint8_t pa_table[];                      /* PATABLE with available output powers */
//extern uint8_t pa_table_index;                  /* Current PATABLE Index */

/* Global variables */
//cc110x_statistic_t cc110x_statistic;            /* Statistic values for debugging */

//volatile cc110x_flags rflags;                   /* Radio control flags */
//volatile uint8_t radio_state = RADIO_UNKNOWN;   /* Radio state */

//static radio_address_t radio_address;
//static uint8_t radio_channel;                   /* Radio channel */

struct rx_buffer_s _ti_emac_rx_buffer[RX_BUF_SIZE];

#ifndef NUM_RX_DESCRIPTORS
#define NUM_RX_DESCRIPTORS 4
#endif

#ifndef NUM_TX_DESCRIPTORS
#define NUM_TX_DESCRIPTORS 8
#endif

#define NUM_RX_BUFFERS 10
#define NUM_TX_BUFFERS 10

#define BUFFER_SIZE 512

#define PHY_PHYS_ADDR 0

typedef struct {
  tEMACDMADescriptor Desc;
  struct tpbuffer *pBuf;
} tDescriptor;

typedef struct {
    tDescriptor *pDescriptors;
    uint32_t ui32NumDescs;
    uint32_t ui32Read;
    uint32_t ui32Write;
} tDescriptorList;

typedef struct tpbuffer {
	uint16_t len;
	uint8_t  p[BUFFER_SIZE];
	uint8_t  free;
	struct tpbuffer *next;
	uint16_t tot_len;
} tpbuffer;

tDescriptor g_pTxDescriptors[NUM_TX_DESCRIPTORS];
tDescriptor g_pRxDescriptors[NUM_RX_DESCRIPTORS];

tpbuffer rxbuffers[NUM_RX_BUFFERS];
tpbuffer txbuffers[NUM_TX_BUFFERS];

tDescriptorList g_TxDescList = {
    g_pTxDescriptors, NUM_TX_DESCRIPTORS, 0, 0
};
tDescriptorList g_RxDescList = {
    g_pRxDescriptors, NUM_RX_DESCRIPTORS, 0, 0
};

tpbuffer *alloc_buf(tpbuffer *pool);
void free_buf(tpbuffer *buf);
void ti_emac_buf_cat(tpbuffer *a, tpbuffer *b);

tpbuffer *alloc_buf(tpbuffer pool[]){
	tpbuffer *r = 0;
	uint8_t i;
	for(i = 0; i < NUM_RX_BUFFERS; i++){
		if(pool[i].free){
			r = &pool[i];
			r->len = BUFFER_SIZE;
			r->tot_len = BUFFER_SIZE;
			r->next = 0;
			r->free = 0;
			break;
		}
	}
	return r;
}

void free_buf(tpbuffer *buf){
	buf->len = BUFFER_SIZE;
	buf->tot_len = BUFFER_SIZE;
	buf->next = 0;
	buf->free = 0;
}

void ti_emac_buf_cat(tpbuffer *h, tpbuffer *t){
	tpbuffer *p;
	for (p = h; p->next != NULL; p = p->next) {
		p->tot_len += t->tot_len;
  	}
	p->tot_len += t->tot_len;
  	p->next = t;
}

/*---------------------------------------------------------------------------*
 *                           Radio Driver API                                *
 *---------------------------------------------------------------------------*/
#ifdef MODULE_TRANSCEIVER
void ti_emac_init(kernel_pid_t tpid)
{
	puts("init emac");
    transceiver_pid = tpid;
    DEBUG("Transceiver PID: %" PRIkernel_pid "\n", transceiver_pid);
    ti_emac_initialize(NULL); /* TODO */
}
#endif

inline void ti_emac_dma_init(void){
	uint32_t ui32Loop;

    /* Transmit list -  mark all descriptors as not owned by the hardware */
   for(ui32Loop = 0; ui32Loop < NUM_TX_DESCRIPTORS; ui32Loop++)
   {
       g_pTxDescriptors[ui32Loop].pBuf = (struct tpbuffer *)0;
       g_pTxDescriptors[ui32Loop].Desc.ui32Count = 0;
       g_pTxDescriptors[ui32Loop].Desc.pvBuffer1 = 0;
       g_pTxDescriptors[ui32Loop].Desc.DES3.pLink =
               ((ui32Loop == (NUM_TX_DESCRIPTORS - 1)) ?
               &g_pTxDescriptors[0].Desc : &g_pTxDescriptors[ui32Loop + 1].Desc);
       g_pTxDescriptors[ui32Loop].Desc.ui32CtrlStatus = DES0_TX_CTRL_INTERRUPT |
               DES0_TX_CTRL_CHAINED | DES0_TX_CTRL_IP_ALL_CKHSUMS;

   }

   g_TxDescList.ui32Read = 0;
   g_TxDescList.ui32Write = 0;


   /* Receive list -  tag each descriptor with a pbuf and set all fields to
    * allow packets to be received.
    */
  for(ui32Loop = 0; ui32Loop < NUM_RX_DESCRIPTORS; ui32Loop++)
  {
      
      g_pRxDescriptors[ui32Loop].pBuf = alloc_buf(rxbuffers);
      /*
      g_pRxDescriptors[ui32Loop].pBuf = &rxbuffers[ui32Loop];
      g_pRxDescriptors[ui32Loop].pBuf->len = 512;
      g_pRxDescriptors[ui32Loop].pBuf->tot_len = 512;
      g_pRxDescriptors[ui32Loop].pBuf->next = 0;
      g_pRxDescriptors[ui32Loop].pBuf->free = 1;
      */
      
      
      g_pRxDescriptors[ui32Loop].Desc.ui32Count = DES1_RX_CTRL_CHAINED;
      if(g_pRxDescriptors[ui32Loop].pBuf)
      {
          /* Set the DMA to write directly into the pbuf payload. */
          g_pRxDescriptors[ui32Loop].Desc.pvBuffer1 =
                  g_pRxDescriptors[ui32Loop].pBuf->p;
          g_pRxDescriptors[ui32Loop].Desc.ui32Count |=
             (g_pRxDescriptors[ui32Loop].pBuf->len << DES1_RX_CTRL_BUFF1_SIZE_S);
          g_pRxDescriptors[ui32Loop].Desc.ui32CtrlStatus = DES0_RX_CTRL_OWN;
      }
      else
      {
          g_pRxDescriptors[ui32Loop].Desc.pvBuffer1 = 0;
          g_pRxDescriptors[ui32Loop].Desc.ui32CtrlStatus = 0;
      }
      g_pRxDescriptors[ui32Loop].Desc.DES3.pLink =
              ((ui32Loop == (NUM_RX_DESCRIPTORS - 1)) ?
              &g_pRxDescriptors[0].Desc : &g_pRxDescriptors[ui32Loop + 1].Desc);
  }

  g_TxDescList.ui32Read = 0;
  g_TxDescList.ui32Write = 0;

  //
  // Set the descriptor pointers in the hardware.
  //
  ROM_EMACRxDMADescriptorListSet(EMAC0_BASE, &g_pRxDescriptors[0].Desc);
  ROM_EMACTxDMADescriptorListSet(EMAC0_BASE, &g_pTxDescriptors[0].Desc);
}

int ti_emac_initialize(netdev_t *dev)
{
	uint32_t umac0, umac1;
	uint8_t  pmac[8];
	
	// get mac and convert to byte array
	ROM_FlashUserGet(&umac0, &umac1);
    if((umac0 == 0xffffffff) || (umac1 == 0xffffffff)){
        puts("[emac] no MAC configured in user registers");
        return -1;
    }
	pmac[0] = ((umac0 >>  0) & 0xff);
	pmac[1] = ((umac0 >>  8) & 0xff);
	pmac[2] = ((umac0 >> 16) & 0xff);
	pmac[3] = ((umac1 >>  0) & 0xff);
	pmac[4] = ((umac1 >>  8) & 0xff);
	pmac[5] = ((umac1 >> 16) & 0xff);
    	
	printf("[emac] mac addr %02x:%02x:%02x:%02x:%02x:%02x\n", pmac[0], pmac[1], pmac[2], pmac[3], pmac[4], pmac[5]);
    
    // enable peripherals
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_EMAC0);
    ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_EMAC0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_EPHY0);
	ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_EPHY0);
	
	// wait for peripherals to be ready
	while(!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_EMAC0));
    
    // configure internal emac
    ROM_EMACPHYConfigSet(EMAC0_BASE, EMAC_PHY_CONFIG);
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
	ROM_EMACAddrSet(EMAC0_BASE, 0, (uint8_t *) pmac);

	// set up DMA
	ti_emac_dma_init();
	
	// clear stray interrupts
	ROM_EMACPHYRead(EMAC0_BASE, PHY_PHYS_ADDR, EPHY_MISR1);
  	ROM_EMACPHYRead(EMAC0_BASE, PHY_PHYS_ADDR, EPHY_MISR2);

	// configure interrupts for status change
	uint16_t status;
  	status = ROM_EMACPHYRead(EMAC0_BASE, PHY_PHYS_ADDR, EPHY_SCR);
  	status |= (EPHY_SCR_INTEN_EXT | EPHY_SCR_INTOE_EXT);
	ROM_EMACPHYWrite(EMAC0_BASE, PHY_PHYS_ADDR, EPHY_SCR, status);
	ROM_EMACPHYWrite(EMAC0_BASE, PHY_PHYS_ADDR, EPHY_MISR1, (EPHY_MISR1_LINKSTATEN |
               EPHY_MISR1_SPEEDEN | EPHY_MISR1_DUPLEXMEN | EPHY_MISR1_ANCEN));

	// clear stray interrupts
  	ROM_EMACPHYRead(EMAC0_BASE, PHY_PHYS_ADDR, EPHY_MISR1);

	// enable mac filtering - allow broadcast, multicast, and unicast for us
	ROM_EMACFrameFilterSet(EMAC0_BASE, (EMAC_FRMFILTER_HASH_AND_PERFECT |
                     EMAC_FRMFILTER_PASS_MULTICAST));
                     
	ROM_EMACTimestampConfigSet(EMAC0_BASE, (EMAC_TS_ALL_RX_FRAMES |
                         EMAC_TS_DIGITAL_ROLLOVER |
                         EMAC_TS_PROCESS_IPV4_UDP | EMAC_TS_ALL |
                         EMAC_TS_PTP_VERSION_1 | EMAC_TS_UPDATE_FINE),
                         (1000000000 / (25000000 / 2)));
	ROM_EMACTimestampAddendSet(EMAC0_BASE, 0x80000000);
	ROM_EMACTimestampEnable(EMAC0_BASE);
                     
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
	//ROM_IntEnable(EMAC0_IRQn);
	//ROM_IntMasterEnable();

	// negotiate link
	ROM_EMACPHYWrite(EMAC0_BASE, PHY_PHYS_ADDR, EPHY_BMCR, (EPHY_BMCR_ANEN |
               EPHY_BMCR_RESTARTAN));
	
	printf("[emac] init finished\n");

    return 0;
}

int ti_emac_send(radio_packet_t *p){
	return 1;
}

int ti_emac_set_channel(uint8_t c){
	return 0;
}

uint8_t ti_emac_get_channel(void){
	return 0;
}

int ti_emac_set_address(radio_address_t addr){
	return 0;
}

radio_address_t ti_emac_get_address(void){
	return 0;
}

void ti_emac_set_monitor(uint8_t mode){
	(void) mode;
}

void ti_emac_powerdown(void){

}

void ti_emac_switch_to_rx(void){
	
}

void process_phy_interrupt(void);
void process_tx_interrupt(void);
void process_rx_interrupt(void);

void isr_emac0(void){

	//puts("ethernet interrupt");

	uint32_t status = ROM_EMACIntStatus(EMAC0_BASE, true);
	if(status){
		ROM_EMACIntClear(EMAC0_BASE, status);
	}
	
	if(status & EMAC_INT_NORMAL_INT){
		printf("emac0 normal int\n");
	}
	
	if(status & EMAC_INT_ABNORMAL_INT){
		printf("emac0 abnormal int\n");
	}
	
	if(status & EMAC_INT_PHY){
      process_phy_interrupt();
	}

	if(status & EMAC_INT_TRANSMIT){
		//printf("emac0 tx int\n");
        process_tx_interrupt();
	}

	if(status & (EMAC_INT_RECEIVE | EMAC_INT_RX_NO_BUFFER | EMAC_INT_RX_STOPPED)){
		//printf("emac0 rx int\n");
        process_rx_interrupt();
	}
}

inline void process_phy_interrupt(void){
    uint16_t ui16Val, ui16Status;
    uint32_t ui32Config, ui32Mode, ui32RxMaxFrameSize;

    /* Read the PHY interrupt status.  This clears all interrupt sources.
     * Note that we are only enabling sources in EPHY_MISR1 so we don't
     * read EPHY_MISR2.
     */
    ui16Val = ROM_EMACPHYRead(EMAC0_BASE, PHY_PHYS_ADDR, EPHY_MISR1);

    /* Read the current PHY status. */
    ui16Status = ROM_EMACPHYRead(EMAC0_BASE, PHY_PHYS_ADDR, EPHY_STS);

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
        if(ui16Status & EPHY_STS_SPEED)
        {
            /* 10Mbps is selected */
            ui32Config &= ~EMAC_CONFIG_100MBPS;
        }
        else
        {
            /* 100Mbps is selected */
            ui32Config |= EMAC_CONFIG_100MBPS;
        }

        /* Are we in fui32l- or half-duplex mode? */
        if(ui16Status & EPHY_STS_DUPLEX)
        {
            /* Fui32l duplex. */
            ui32Config |= EMAC_CONFIG_FULL_DUPLEX;
        }
        else
        {
            /* Half duplex. */
            ui32Config &= ~EMAC_CONFIG_FULL_DUPLEX;
        }

        /* Reconfigure the MAC */
        ROM_EMACConfigSet(EMAC0_BASE, ui32Config, ui32Mode, ui32RxMaxFrameSize);
    }
}

inline void process_tx_interrupt(void){
    tDescriptorList *pDescList;
    uint32_t ui32NumDescs;

    /* Get a pointer to the transmit descriptor list. */
    pDescList = &g_TxDescList;

    /* Walk the list until we have checked all descriptors or we reach the
     * write pointer or find a descriptor that the hardware is still working
     * on.
     */
    for(ui32NumDescs = 0; ui32NumDescs < pDescList->ui32NumDescs; ui32NumDescs++)
    {
        /* Has the buffer attached to this descriptor been transmitted? */
        if(pDescList->pDescriptors[pDescList->ui32Read].Desc.ui32CtrlStatus &
           DES0_TX_CTRL_OWN)
        {
            /* No - we're finished. */
            break;
        }

        /* Does this descriptor have a buffer attached to it? */
        if(pDescList->pDescriptors[pDescList->ui32Read].pBuf)
        {
            /* Yes - free it if it's not marked as an intermediate pbuf */
            //if(!((uint32_t)(pDescList->pDescriptors[pDescList->ui32Read].pBuf) & 1))
            //{
                //pbuf_free(pDescList->pDescriptors[pDescList->ui32Read].pBuf);
                //pDescList->pDescriptors[pDescList->ui32Read].pBuf->free = 1;
                //DRIVER_STATS_INC(TXBufFreedCount);
            //}
            free_buf(pDescList->pDescriptors[pDescList->ui32Read].pBuf);
            //pDescList->pDescriptors[pDescList->ui32Read].pBuf = NULL;
        }
        else
        {
            /* If the descriptor has no buffer, we are finished. */
            break;
        }

        /* Move on to the next descriptor. */
        pDescList->ui32Read++;
        if(pDescList->ui32Read == pDescList->ui32NumDescs)
        {
            pDescList->ui32Read = 0;
        }
    }
}

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
  while(pDescList->ui32Read != ui32DescEnd)
  {
      /* Does the current descriptor have a buffer attached to it? */
      if(pDescList->pDescriptors[pDescList->ui32Read].pBuf)
      {
          /* Yes - determine if the host has filled it yet. */
          if(pDescList->pDescriptors[pDescList->ui32Read].Desc.ui32CtrlStatus &
             DES0_RX_CTRL_OWN)
          {
              /* The DMA engine still owns the descriptor so we are finished */
              break;
          }

          //DRIVER_STATS_INC(RXBufReadCount);

          /* If this descriptor contains the end of the packet, fix up the
           * buffer size accordingly.
           */
          if(pDescList->pDescriptors[pDescList->ui32Read].Desc.ui32CtrlStatus &
             DES0_RX_STAT_LAST_DESC)
          {
              /* This is the last descriptor for the frame so fix up the
               * length.  It is safe for us to modify the internal fields
               * directly here (rather than calling pbuf_realloc) since we
               * know each of these pbufs is never chained.
               */
              pDescList->pDescriptors[pDescList->ui32Read].pBuf->len =
                       (pDescList->pDescriptors[pDescList->ui32Read].Desc.ui32CtrlStatus &
                        DES0_RX_STAT_FRAME_LENGTH_M) >>
                        DES0_RX_STAT_FRAME_LENGTH_S;
              pDescList->pDescriptors[pDescList->ui32Read].pBuf->tot_len =
                        pDescList->pDescriptors[pDescList->ui32Read].pBuf->len;
          }

          if(pBuf)
          {
              /* Link this pbuf to the last one we looked at since this buffer
               * is a continuation of an existing frame (split across mui32tiple
               * pbufs).  Note that we use pbuf_cat() here rather than
               * pbuf_chain() since we don't want to increase the reference
               * count of either pbuf - we only want to link them together.
               */
              ti_emac_buf_cat(pBuf, pDescList->pDescriptors[pDescList->ui32Read].pBuf);
              pDescList->pDescriptors[pDescList->ui32Read].pBuf = pBuf;
          }

          /* Remember the buffer associated with this descriptor. */
          pBuf = pDescList->pDescriptors[pDescList->ui32Read].pBuf;

          /* Is this the last descriptor for the current frame? */
          if(pDescList->pDescriptors[pDescList->ui32Read].Desc.ui32CtrlStatus &
             DES0_RX_STAT_LAST_DESC)
          {
              /* Yes - does the frame contain errors? */
              if(pDescList->pDescriptors[pDescList->ui32Read].Desc.ui32CtrlStatus &
                 DES0_RX_STAT_ERR)
              {
                  /* This is a bad frame so discard it and update the relevant
                   * statistics.
                   */
                  //LWIP_DEBUGF(NETIF_DEBUG, ("tivaif_receive: packet error\n"));
                  free_buf(pBuf);
                  //LINK_STATS_INC(link.drop);
                  //DRIVER_STATS_INC(RXPacketErrCount);
                  puts("[emac] rx bad frame");
              }
              else
              {
                  /* This is a good frame so pass it up the stack. */
                  //LINK_STATS_INC(link.recv);
                  //DRIVER_STATS_INC(RXPacketReadCount);
                  puts("[emac] good frame");

				/*
                  / * Place the timestamp in the PBUF if PTPD is enabled * /
                  pBuf->time_s =
                       pDescList->pDescriptors[pDescList->ui32Read].Desc.ui32IEEE1588TimeHi;
                  pBuf->time_ns =
                       pDescList->pDescriptors[pDescList->ui32Read].Desc.ui32IEEE1588TimeLo;
				*/

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
      pDescList->pDescriptors[pDescList->ui32Read].pBuf = alloc_buf(rxbuffers);
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

/*

uint8_t cc110x_get_buffer_pos(void)
{
    return (rx_buffer_next - 1);
}

radio_address_t cc110x_get_address(void)
{
    return radio_address;
}

radio_address_t cc110x_set_address(radio_address_t address)
{
    if ((address < MIN_UID) || (address > MAX_UID)) {
        return 0;
    }

    uint8_t id = (uint8_t) address;

    if (radio_state != RADIO_UNKNOWN) {
        write_register(CC1100_ADDR, id);
    }

    radio_address = id;
    return radio_address;
}

#ifdef MODULE_CONFIG
radio_address_t cc110x_set_config_address(radio_address_t address)
{
    radio_address_t a = cc110x_set_address(address);

    if (a) {
        sysconfig.radio_address = a;
    }

    config_save();
    return a;
}
#endif

void cc110x_set_monitor(uint8_t mode)
{
    if (mode) {
        write_register(CC1100_PKTCTRL1, (0x04));
    }
    else {
        write_register(CC1100_PKTCTRL1, (0x06));
    }
}

void cc110x_setup_rx_mode(void)
{
   / * Stay in RX mode until end of packet * /
    cc110x_write_reg(CC1100_MCSM2, 0x07);
    cc110x_switch_to_rx();
}


void cc110x_switch_to_rx(void)
{
    radio_state = RADIO_RX;
    cc110x_strobe(CC1100_SRX);
}

void cc110x_wakeup_from_rx(void)
{
    if (radio_state != RADIO_RX) {
        return;
    }

    DEBUG("CC110x going to idle\n");
    cc110x_strobe(CC1100_SIDLE);
    radio_state = RADIO_IDLE;
}

char *cc110x_get_marc_state(void)
{
    uint8_t state;

    / * Save old radio state * /
    uint8_t old_state = radio_state;

    / * Read content of status register * /
    state = cc110x_read_status(CC1100_MARCSTATE) & MARC_STATE;

    / * Make sure in IDLE state.
     * Only goes to IDLE if state was RX * /
    cc110x_wakeup_from_rx();

    / * Have to put radio back to RX if old radio state
     * was RX, otherwise no action is necessary * /
    if (old_state == RADIO_RX) {
        cc110x_switch_to_rx();
    }

    switch(state) {
            / * Note: it is not possible to read back the SLEEP or XOFF state numbers
             * because setting CSn low will make the chip enter the IDLE mode from the
             * SLEEP (0) or XOFF (2) states. * /
        case 1:
            return "IDLE";

        case 3:
        case 4:
        case 5:
            return "MANCAL";

        case 6:
        case 7:
            return "FS_WAKEUP";

        case 8:
        case 12:
            return "CALIBRATE";

        case 9:
        case 10:
        case 11:
            return "SETTLING";

        case 13:
        case 14:
        case 15:
            return "RX";

        case 16:
            return "TXRX_SETTLING";

        case 17:
            return "RXFIFO_OVERFLOW";

        case 18:
            return "FSTXON";

        case 19:
        case 20:
            return "TX";

        case 21:
            return "RXTX_SETTLING";

        case 22:
            return "TXFIFO_UNDERFLOW";

        default:
            return "UNKNOWN";
    }
}

char *cc110x_state_to_text(uint8_t state)
{
    switch(state) {
        case RADIO_UNKNOWN:
            return "Unknown";

        case RADIO_IDLE:
            return "IDLE";

        case RADIO_SEND_BURST:
            return "TX BURST";

        case RADIO_RX:
            return "RX";

        case RADIO_PWD:
            return "PWD";

        default:
            return "unknown";
    }
}

void cc110x_print_config(void)
{
    printf("Current radio state:          %s\r\n", cc110x_state_to_text(radio_state));
    printf("Current MARC state:           %s\r\n", cc110x_get_marc_state());
    printf("Current channel number:       %u\r\n", radio_channel);
}

void cc110x_switch_to_pwd(void)
{
    DEBUG("[cc110x] switching to powerdown\n");
    cc110x_wakeup_from_rx();
    cc110x_strobe(CC1100_SPWD);
    radio_state = RADIO_PWD;
}

/ *---------------------------------------------------------------------------* /
int16_t cc110x_set_channel(uint8_t channr)
{
    if (channr > MAX_CHANNR) {
        return -1;
    }

    write_register(CC1100_CHANNR, channr * 10);
    radio_channel = channr;
    return radio_channel;
}

#ifdef MODULE_CONFIG
int16_t cc110x_set_config_channel(uint8_t channr)
{
    int16_t c = cc110x_set_channel(channr);

    if (c) {
        sysconfig.radio_channel = c;
    }

    config_save();
    return c;
}
#endif

int16_t cc110x_get_channel(void)
{
    return radio_channel;
}


/ *---------------------------------------------------------------------------
 *                          CC1100 reset functionality
 *---------------------------------------------------------------------------* /

static void reset(void)
{
    cc110x_wakeup_from_rx();
    cc110x_cs();
    cc110x_strobe(CC1100_SRES);
    hwtimer_wait(RTIMER_TICKS(100));
}

static void power_up_reset(void)
{
    gpio_set(CC110X_CS);
    gpio_clear(CC110X_CS);
    gpio_set(CC110X_CS);
    hwtimer_wait(RESET_WAIT_TIME);
    reset();
    radio_state = RADIO_IDLE;
}

static void write_register(uint8_t r, uint8_t value)
{
    / * Save old radio state * /
    uint8_t old_state = radio_state;

    / * Wake up from RX (no effect if in other mode) * /
    cc110x_wakeup_from_rx();
    cc110x_write_reg(r, value);

    / * Have to put radio back to RX if old radio state
     * was RX, otherwise no action is necessary * /
    if (old_state == RADIO_RX) {
        cc110x_switch_to_rx();
    }
}

static int rd_set_mode(int mode)
{
    int result;

    / * Get current radio mode * /
    if ((radio_state == RADIO_UNKNOWN) || (radio_state == RADIO_PWD)) {
        result = RADIO_MODE_OFF;
    }
    else {
        result = RADIO_MODE_ON;
    }

    switch(mode) {
        case RADIO_MODE_ON:
            DEBUG("Enabling rx mode\n");
            gpio_irq_enable(CC110X_GDO2);
            cc110x_setup_rx_mode();                 / * Set chip to desired mode * /
            break;

        case RADIO_MODE_OFF:
            gpio_irq_disable(CC110X_GDO2);          / * Disable interrupts * /
            cc110x_switch_to_pwd();                 / * Set chip to power down mode * /
            break;

        case RADIO_MODE_GET:
            / * do nothing, just return current mode * /
        default:
            / * do nothing * /
            break;
    }

    / * Return previous mode * /
    return result;
}

*/
