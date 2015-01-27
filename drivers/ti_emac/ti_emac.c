
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
#include "driverlib/emac.h"
#include "driverlib/sysctl.h"
#include "driverlib/hw_emac.h"

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
	uint8_t  p[512];
} tpbuffer;

tDescriptor g_pTxDescriptors[NUM_TX_DESCRIPTORS];
tDescriptor g_pRxDescriptors[NUM_RX_DESCRIPTORS];

tpbuffer rxbuffers[NUM_RX_DESCRIPTORS];

tDescriptorList g_TxDescList = {
    g_pTxDescriptors, NUM_TX_DESCRIPTORS, 0, 0
};
tDescriptorList g_RxDescList = {
    g_pRxDescriptors, NUM_RX_DESCRIPTORS, 0, 0
};


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
      g_pRxDescriptors[ui32Loop].pBuf = &rxbuffers[ui32Loop];
      g_pRxDescriptors[ui32Loop].pBuf->len = 512;
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
    ROM_EMACPHYConfigSet(EMAC0_BASE, EMAC_PHY_TYPE_INTERNAL);
    ROM_EMACInit(EMAC0_BASE, F_CPU,
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
	
	// enable tx and rx
	ROM_EMACTxEnable(EMAC0_BASE);
	ROM_EMACRxEnable(EMAC0_BASE);

	// enable tx, rx interrupts
	ROM_EMACIntEnable(EMAC0_BASE, (EMAC_INT_RECEIVE | EMAC_INT_TRANSMIT |
                EMAC_INT_TX_STOPPED | EMAC_INT_RX_NO_BUFFER |
                EMAC_INT_RX_STOPPED | EMAC_INT_PHY));
	ROM_IntEnable(EMAC0_IRQn);
	
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

void isr_emac0(void){
	uint32_t status = ROM_EMACIntStatus(EMAC0_BASE, true);
	if(status){
		ROM_EMACIntClear(EMAC0_BASE, status);
	}
	printf("[emac] interrupt (status = %x)\n", status);
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
