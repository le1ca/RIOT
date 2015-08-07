#ifndef radio_trans_h
#define radio_trans_h

#include <stdint.h>
#include <stdbool.h>

#include "ringbuffer.h"
#include "mutex.h"
#include "thread.h"

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* timer config */
#define RTRANS_TIMER_NUM        TIMER_4
#define RTRANS_TIMER_BASE       TIMER4_BASE
#define RTRANS_TIMER_HZ         (uint32_t) 16000000L
#define RTRANS_TIMER_MAX        0xffffffff
#define RTRANS_TIMER_PRESCALE   (uint32_t) 125

/* stack size */
#define RT_THREAD_STACK_SIZE 1024

/* special uuid for no master configured */
#define RT_NO_MASTER 0xffff

/* packet types */
#define RTRANS_TYPE_PROBE 0 // from master to slave only - broadcast message to detect slaves
#define RTRANS_TYPE_JOIN  1 // from slave to master only - response to 
#define RTRANS_TYPE_POLL  2 // from master to slave only - request for data
#define RTRANS_TYPE_DATA  3 // from slave to master only - response containing sensing data
#define RTRANS_TYPE_SET   4 // from master to slave only - set control params
#define RTRANS_TYPE_ERR   5 // from slave to master only - report high-priority hardware error
#define RTRANS_TYPE_ACK   254 // general acknowledgment pkt - confirm join, ack data, ack set
#define RTRANS_TYPE_NAK   255 // negative acknowledgment - refuse join, retx request, error

/* driver states */
#define RTRANS_STATE_INIT 0 // waiting for probe
#define RTRANS_STATE_JOIN 1 // waiting for join-ack
#define RTRANS_STATE_IDLE 2 // waiting for poll or set
#define RTRANS_STATE_DATA 3 // waiting for ack

#define RTRANS_TX_WAITING 0 // waiting
#define RTRANS_TX_SUCCESS 1 // got ack
#define RTRANS_TX_FAILURE 2 // got nack or timed out

/* max number of retx */
#define RTRANS_MAX_RETRY 8

/* total packet size */
#define RTRANS_MAX_PKT_LEN (100)

/* header size (10b) */
#define RTRANS_HDR_LEN (sizeof(struct radio_trans_hdr_s))

/* max payload size (85b) */
#define RTRANS_MAX_PAYLOAD (RTRANS_MAX_PKT_LEN - RTRANS_HDR_LEN - 1)

/* max number of segments - each package can be 1360 bytes */
#define RTRANS_MAX_SEGS (16)

/* radio address length in bytes */
#define RTRANS_ADDR_LEN (2)

/* window size in bytes */
#define RTRANS_WINDOW (RTRANS_MAX_PKT_LEN*RTRANS_MAX_SEGS)

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* FORWARD DEF */
typedef struct radio_trans_pkt_s radio_trans_pkt;

/* FUNCTION TYPES */
typedef void (*rt_package_cb)(radio_trans_pkt* packet);
typedef void (*rt_radio_tx)(char *buf, unsigned int len, uint16_t addr);

/* DRIVER STATE */
typedef struct radio_trans_state_s {

    uint16_t slave_mac;
    uint16_t master_mac;
    uint8_t  driver_state;
    
    rt_radio_tx   tx_func;
    rt_package_cb rx_func;
    
    kernel_pid_t tx_pid;
    ringbuffer_t tx_ringbuffer;
    
    kernel_pid_t cb_pid;
    ringbuffer_t cb_ringbuffer;    
    char         cb_window[RTRANS_WINDOW];
    
    mutex_t      tx_mutex, retx_mutex;
    uint8_t      tx_wait_pkg, tx_wait_seq;
    uint8_t      tx_retries;
    uint8_t      tx_success;
    char         tx_window[RTRANS_WINDOW];
    uint8_t      tx_pkg_no;
    
    //kernel_pid_t rx_pid;
    //ringbuffer_t rx_ringbuffer;
    //char         rx_window[RTRANS_WINDOW];
    //uint8_t      rx_window_seq;
    
} radio_trans_state;

/* RADIO PACKET HEADER - 10 bytes */
typedef struct __attribute__ ((__packed__)) radio_trans_hdr_s {
    uint16_t master;    // master mac
    uint16_t slave;     // slave mac
    uint16_t pkg_no;    // package number
    uint8_t  type;      // message type
    uint8_t  pkt_ct;    // number of segments in package
    uint8_t  seg_no;    // segment number
    uint8_t  len;       // payload length
} radio_trans_hdr;

/* RADIO PACKET - 96 bytes */
typedef struct __attribute__ ((__packed__)) radio_trans_pkt_s {
    radio_trans_hdr hdr;
    uint8_t payload[RTRANS_MAX_PAYLOAD]; // 85 bytes
    uint8_t checksum;
} radio_trans_pkt;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* initialize the driver with given uuid, tx function, and rx callback */
void rt_init_driver(uint16_t uuid, rt_radio_tx tx_func, rt_package_cb rx_func);

/* segments a payload into packets. each packet (except perhaps the last) will
   be exactly RTRANS_MAX_PKT_LEN bytes long. these packets are then transmitted;
   this call may block if the transmission buffer is full.

   params -
    dest:   destination node uuid
    type:   packet type
    input:  payload
    plen:   length of input
    
*/
void rt_transmit(uint8_t type, char *payload, uint16_t len);

/* handle incoming packet */
void rt_incoming(char *payload, uint8_t len);

/* change master */
void rt_set_master(uint16_t master);

#endif
