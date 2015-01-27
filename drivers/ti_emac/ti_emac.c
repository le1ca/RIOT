
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

//static radio_address_t radio_address;           /* Radio address */
//static uint8_t radio_channel;                   /* Radio channel */

struct rx_buffer_s _ti_emac_rx_buffer[RX_BUF_SIZE];


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

int ti_emac_initialize(netdev_t *dev)
{

	uint32_t umac0, umac1;
	
	ROM_FlashUserGet(&umac0, &umac1);
    if((umac0 == 0xffffffff) || (umac1 == 0xffffffff)){
        puts("[emac] no MAC configured in user registers");

        return -1;
    }
    else{
    	printf("[emac] mac addr %x %x\n", (unsigned) umac0, (unsigned) umac1);
    }

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
