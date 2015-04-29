#include "radio_trans.h"

#include "string.h"
#include "periph/timer.h"
#include "driverlib/rom.h"
#include "driverlib/timer.h"
//#include "driverlib/hw_memmap.h"

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* utility macros */
#define RB_FREE_SPACE(rb) (rb.size - rb.avail)

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* global state */
radio_trans_state rt_state;

/* thread stacks */
char rt_tx_thread_stack[RT_THREAD_STACK_SIZE];
char rt_cb_thread_stack[RT_THREAD_STACK_SIZE];

/* forward decls */
void rt_timer_init(void);
void rt_timer_cb(int arg);
void rt_timer_start(void);
void rt_timer_stop(void);
void rt_tx_thread_init(void);
void rt_cb_thread_init(void);
bool rt_checksum_good(char *payload, uint8_t len);
void *rt_tx_loop(void *arg);
void *rt_cb_loop(void *arg);
void rt_add_pkt(uint8_t type, uint8_t pkg_no, uint8_t seq_no, uint8_t seg_ct, char *buff, uint8_t len);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* initialization functions */

void rt_init_driver(uint16_t uuid, rt_radio_tx tx_func, rt_package_cb rx_func){
    /* parameters */
    rt_state.slave_mac  = uuid;
    rt_state.master_mac = RT_NO_MASTER;
    rt_state.tx_func = tx_func;
    rt_state.rx_func = rx_func;
    
    /* ringbuffers */
    //ringbuffer_init(&rt_state.rx_ringbuffer, rt_state.rx_window, RTRANS_WINDOW);
    ringbuffer_init(&rt_state.cb_ringbuffer, rt_state.cb_window, RTRANS_WINDOW);
    ringbuffer_init(&rt_state.tx_ringbuffer, rt_state.tx_window, RTRANS_WINDOW);
    
    /* mutexes */
    mutex_init(&rt_state.tx_mutex);
    mutex_init(&rt_state.retx_mutex);
    
    /* retx timer */
    rt_timer_init();
    
    /* tx thread */
    rt_tx_thread_init();
    
    /* cb thrad */
    rt_cb_thread_init();
}

void rt_timer_init(void){
    timer_init(RTRANS_TIMER_NUM, 1, rt_timer_cb);
    ROM_TimerDisable(RTRANS_TIMER_BASE, TIMER_A);
    ROM_TimerClockSourceSet(RTRANS_TIMER_BASE, TIMER_CLOCK_PIOSC);
    ROM_TimerConfigure(RTRANS_TIMER_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_ONE_SHOT | TIMER_CFG_A_ACT_TOINTD);
    ROM_TimerPrescaleSet(RTRANS_TIMER_BASE, TIMER_A, RTRANS_TIMER_PRESCALE);
    ROM_TimerLoadSet(RTRANS_TIMER_BASE, TIMER_A, RTRANS_TIMER_MAX);
    ROM_TimerIntEnable(RTRANS_TIMER_BASE, TIMER_TIMA_MATCH);
    ROM_TimerIntEnable(RTRANS_TIMER_BASE, TIMER_TIMA_TIMEOUT);
    ROM_TimerDisable(RTRANS_TIMER_BASE, TIMER_A);   
}

void rt_tx_thread_init(void){
    kernel_pid_t pid = thread_create(rt_tx_thread_stack,
                                     sizeof(rt_tx_thread_stack),
                                     PRIORITY_MAIN - 1,
                                     CREATE_STACKTEST | CREATE_SLEEPING,
                                     rt_tx_loop,
                                     &rt_state.tx_ringbuffer,
                                     "rt_tx"
    );
    rt_state.tx_pid = pid;
    thread_wakeup(pid);
}

void rt_cb_thread_init(void){
    kernel_pid_t pid = thread_create(rt_cb_thread_stack,
                                     sizeof(rt_cb_thread_stack),
                                     PRIORITY_MAIN - 1,
                                     CREATE_STACKTEST | CREATE_SLEEPING,
                                     rt_cb_loop,
                                     &rt_state.cb_ringbuffer,
                                     "rt_cb"
    );
    rt_state.cb_pid = pid;
    thread_wakeup(pid);
}

void rt_set_master(uint16_t master){
    rt_state.master_mac = master;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* transmit package */

void rt_transmit(uint8_t type, char *payload, uint16_t len){
    uint8_t  pkg_no = rt_state.tx_pkg_no++;
    uint8_t  seq_no = 0, num_segs = 0;
    
    // calculate number of segments
    num_segs = len / RTRANS_MAX_PAYLOAD;
    if(num_segs * RTRANS_MAX_PAYLOAD < len || len == 0)
        num_segs++;
    
    // construct segment packets
    do{
       uint8_t plen = (len > RTRANS_MAX_PAYLOAD) ? RTRANS_MAX_PAYLOAD : len;
       rt_add_pkt(type, pkg_no, seq_no++, num_segs, payload, plen);
       payload += plen;
       len -= plen;
    } while(len > 0);
    
}

void rt_add_pkt(uint8_t type, uint8_t pkg_no, uint8_t seq_no, uint8_t seg_ct, char *buff, uint8_t len){
     
     unsigned char buffer[RTRANS_MAX_PKT_LEN];
     radio_trans_pkt *pbuf = (radio_trans_pkt *) buffer;
     msg_t m;
     m.type = 0;
     
     // construct header
     pbuf->hdr.master = rt_state.master_mac;
     pbuf->hdr.slave  = rt_state.slave_mac;
     pbuf->hdr.pkg_no = pkg_no;
     pbuf->hdr.type   = type;
     pbuf->hdr.pkt_ct = seg_ct;
     pbuf->hdr.seg_no = seq_no;
     pbuf->hdr.len    = len;
     
     // copy payload
     memcpy(pbuf->payload, buff, len);
     
     // calculate checksum
     pbuf->payload[len] = 0xff;
     for(unsigned i = 0; i < len + RTRANS_HDR_LEN; i++){
        pbuf->payload[len] -= buffer[i];
     }
     
     // wait for sufficient space in ringbuffer: payload + header + checksum
     while(RB_FREE_SPACE(rt_state.tx_ringbuffer) < len + RTRANS_HDR_LEN + 1){
        //printf("[rt] waiting for buffer to clear\n");
        mutex_lock(&rt_state.tx_mutex);
        mutex_lock(&rt_state.tx_mutex);
        mutex_unlock(&rt_state.tx_mutex);
        //printf("[rt] proceeding\n");
     }
     
     // add to ringbuffer
     if(ringbuffer_add(&rt_state.tx_ringbuffer, (char*) buffer, len + RTRANS_HDR_LEN + 1) != len + RTRANS_HDR_LEN + 1){
        printf("[rt] error: tx buffer overflow\n");
     }
     
     // wake tx thread
     msg_send_int(&m, rt_state.tx_pid);
}

void *rt_tx_loop(void *arg){
    char buffer[RTRANS_MAX_PKT_LEN];
    radio_trans_pkt *pbuf = (radio_trans_pkt *) buffer;
    ringbuffer_t *rb = (ringbuffer_t *) arg;
    msg_t m;
    
    while(1){
        msg_receive(&m);
        
        while (rb->avail) {
        
            // begin CS
            unsigned state = disableIRQ();
            
            // peek a header
            ringbuffer_peek(rb, buffer, RTRANS_HDR_LEN);
            
            // remove the packet from rb
            ringbuffer_get(rb, buffer, RTRANS_HDR_LEN + pbuf->hdr.len + 1);
            
            // end CS
            restoreIRQ(state);
            
            // notify other threads that the ringbuffer is no longer full
            mutex_unlock(&rt_state.tx_mutex);
            
            // begin attempts to transmit
            rt_state.tx_retries = 0;
            rt_state.tx_success = RTRANS_TX_WAITING;
            rt_state.tx_wait_pkg = pbuf->hdr.pkg_no;
            rt_state.tx_wait_seq = pbuf->hdr.seg_no;
            
            while(rt_state.tx_success == RTRANS_TX_WAITING){
            
                if(rt_state.tx_retries++ == RTRANS_MAX_RETRY){
                    rt_state.tx_success = RTRANS_TX_FAILURE;
                    break;
                }
            
                // transmit packet
                rt_state.tx_func(buffer, RTRANS_HDR_LEN + pbuf->hdr.len + 1, pbuf->hdr.master);
            
                // start timer
                rt_timer_start();
            
                // wait for ACK or timeout
                mutex_lock(&rt_state.retx_mutex);
                mutex_lock(&rt_state.retx_mutex);
                mutex_unlock(&rt_state.retx_mutex);
                
            }
            
            // cancel the package if tx failed
            if(rt_state.tx_success == RTRANS_TX_FAILURE){
                uint8_t count = 0;
                printf("[rt] pkg %d seg %d failed to transmit\n", rt_state.tx_wait_pkg, rt_state.tx_wait_seq);
                ringbuffer_peek(rb, buffer, RTRANS_HDR_LEN);
                while(rb->avail > RTRANS_HDR_LEN && pbuf->hdr.pkg_no == rt_state.tx_wait_pkg){
                    state = disableIRQ();
                    ringbuffer_get(rb, buffer, RTRANS_HDR_LEN + pbuf->hdr.len + 1);
                    ringbuffer_peek(rb, buffer, RTRANS_HDR_LEN);
                    restoreIRQ(state);
                    mutex_unlock(&rt_state.tx_mutex);
                    count++;
                }
                printf("[rt] removed %d additional segments\n", count);
            }
            
        }
         
    }
    return 0;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* incoming package */

void rt_incoming(char *payload, uint8_t len){

    radio_trans_pkt *pbuf = (radio_trans_pkt *) payload;

    if(!rt_checksum_good(payload, len)){
        printf("[rt] ignoring incoming segment with bad checksum\n");
        return;
    }

    switch(pbuf->hdr.type){
    
        case RTRANS_TYPE_PROBE:
        case RTRANS_TYPE_POLL:
        case RTRANS_TYPE_SET:   {
            msg_t m;
            m.type = 0;
            if(ringbuffer_add(&rt_state.cb_ringbuffer, (char*) pbuf, pbuf->hdr.len + RTRANS_HDR_LEN + 1) != pbuf->hdr.len + RTRANS_HDR_LEN + 1){
                printf("[rt] error: rx buffer overflow\n");
            }
            msg_send_int(&m, rt_state.cb_pid);
            break;
        }
        
        /* ACK: if this is the ACK we were waiting for, stop the timeout timer and advance the window */
        case RTRANS_TYPE_ACK:   {
            if(pbuf->hdr.pkg_no == rt_state.tx_wait_pkg && pbuf->hdr.seg_no == rt_state.tx_wait_seq){
                rt_state.tx_success = RTRANS_TX_SUCCESS;
                rt_timer_stop();
                rt_timer_cb(0);
            }
            break;
        }
        
        /* NACK: cancel the timeout and clear the window */
        case RTRANS_TYPE_NAK:   {
            if(pbuf->hdr.pkg_no == rt_state.tx_wait_pkg && pbuf->hdr.seg_no == rt_state.tx_wait_seq){
                rt_state.tx_success = RTRANS_TX_FAILURE;
                rt_timer_stop();
                rt_timer_cb(0);
            }
            break;
        }
        
        default:                {
            printf("[rt] ignoring incoming segment with undefined type %02x\n", pbuf->hdr.type);
        }
    
    }
}

bool rt_checksum_good(char *payload, uint8_t len){

    radio_trans_pkt *pbuf = (radio_trans_pkt *) payload;
    uint8_t acc = 0;
    
    /* check that there is at least a valid header */
    if(len < RTRANS_HDR_LEN)
        return false;

    /* check that the length declared in the header matches the buffer length */
    if(len != pbuf->hdr.len + RTRANS_HDR_LEN + 1)
        return false;
        
    /* calculate the checksum */
    for(int i = 0; i < len; i++){
        acc += payload[i];
    }
    
    /* verify correctnes */
    return (acc == 0xff);
}

void *rt_cb_loop(void *arg){
    char buffer[RTRANS_MAX_PKT_LEN];
    radio_trans_pkt *pbuf = (radio_trans_pkt *) buffer;
    ringbuffer_t *rb = (ringbuffer_t *) arg;
    msg_t m;
    while(1){
        msg_receive(&m);
        while (rb->avail) {
            unsigned state = disableIRQ();
            ringbuffer_peek(rb, buffer, RTRANS_HDR_LEN);
            ringbuffer_get(rb, buffer, RTRANS_HDR_LEN + pbuf->hdr.len + 1);
            restoreIRQ(state);
            rt_state.rx_func(pbuf);
        }
    }
    return 0;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* timer manipulation functions */

void rt_timer_start(void){
    ROM_TimerLoadSet(RTRANS_TIMER_BASE, TIMER_A, RTRANS_TIMER_MAX);
    ROM_TimerEnable(RTRANS_TIMER_BASE, TIMER_A);  
}

void rt_timer_stop(void){
    ROM_TimerDisable(RTRANS_TIMER_BASE, TIMER_A);
}

void rt_timer_cb(int arg){
    mutex_unlock(&rt_state.retx_mutex);
}
