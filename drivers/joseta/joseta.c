#include "joseta_internal.h"

////////////////////////////////////////////////////////////////////////////////
// GLOBAL VARIABLES ////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/* buffers */
static char joseta_serial_buffer[JOSETA_UART_BUF];
static char joseta_frame_buffer[JOSETA_BUFFER_SIZE];

/* driver state */
joseta_state_t joseta_state; 

/* thread stacks */
static char joseta_serial_thread_stack[JOSETA_SERIAL_STACK];
static char joseta_callback_thread_stack[JOSETA_CALLBACK_STACK];

////////////////////////////////////////////////////////////////////////////////
// INITIALIZATION FUNCTIONS ////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/* initialize board and start polling thread */
void joseta_init(uint64_t rtc){
	joseta_state_init(rtc);
	joseta_timer_init();
	joseta_serial_thread_init();
	joseta_callback_thread_init();
	joseta_uart_init();
	joseta_board_init();
}

/* initialize uart device */
void joseta_uart_init(void){
	uart_init(JOSETA_UART, 9600U, joseta_serial_recv, 0, 0);
}

/* initialize timers */
void joseta_timer_init(void){

	/* set callback */
	timer_init(JOSETA_TIMER_NUM, 1, joseta_timer_cb);
	
	/* custom timer configuration */
	ROM_TimerDisable(JOSETA_TIMER_BASE, TIMER_A);
	ROM_TimerClockSourceSet(JOSETA_TIMER_BASE, TIMER_CLOCK_PIOSC);
	ROM_TimerConfigure(JOSETA_TIMER_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC | TIMER_CFG_A_ACT_TOINTD);
	ROM_TimerPrescaleSet(JOSETA_TIMER_BASE, TIMER_A, JOSETA_TIMER_PRESCALE);
	ROM_TimerLoadSet(JOSETA_TIMER_BASE, TIMER_A, JOSETA_TIMER_MAX);
    ROM_TimerIntEnable(JOSETA_TIMER_BASE, TIMER_TIMA_MATCH);
    ROM_TimerIntEnable(JOSETA_TIMER_BASE, TIMER_TIMA_TIMEOUT);
	ROM_TimerEnable(JOSETA_TIMER_BASE, TIMER_A);
}

/* start chardev thread */
void joseta_serial_thread_init(void){
	
	ringbuffer_init(&joseta_state.serial_ringbuffer, joseta_serial_buffer,
					JOSETA_UART_BUF
	);
    
    kernel_pid_t pid = thread_create(joseta_serial_thread_stack,
                  					 sizeof(joseta_serial_thread_stack),
									 PRIORITY_MAIN - 1,
									 CREATE_STACKTEST | CREATE_SLEEPING,
									 joseta_serial_loop,
									 &joseta_state.serial_ringbuffer,
									 "joseta_uart"
	);
	
    joseta_state.serial_pid = pid;
    thread_wakeup(pid);
}

/* start callback message-handling thread */
void joseta_callback_thread_init(void){
	
	ringbuffer_init(&joseta_state.frame_ringbuffer, joseta_frame_buffer,
					JOSETA_BUFFER_SIZE
	);
	
    kernel_pid_t pid = thread_create(joseta_callback_thread_stack,
                  					 sizeof(joseta_callback_thread_stack),
									 PRIORITY_MAIN - 1,
									 CREATE_STACKTEST | CREATE_SLEEPING,
									 joseta_callback_loop,
									 &joseta_state.frame_ringbuffer,
									 "joseta_callback"
	);
	
    joseta_state.callback_pid = pid;
    thread_wakeup(pid);
}

/* initialize driver state */
void joseta_state_init(uint64_t rtc){
	
	/* clock */
	joseta_state.rtc = rtc;
	
	/* thread ids */
	joseta_state.serial_pid = KERNEL_PID_UNDEF;
	joseta_state.callback_pid = KERNEL_PID_UNDEF;
	
	/* default settings */
	joseta_state.purgethresh = JOSETA_DEFAULT_PURGETHRESH;
	joseta_state.callback = 0;
	
	/* read/write counters */
	joseta_state.current_frame_idx = 0;
	joseta_state.expected_frames = 0;
	joseta_state.frame_count = 0;
	
}

/* send initialization commands to board */
void joseta_board_init(void){
	joseta_state.fsm = JOSETA_FSM_INIT;
	joseta_send_reset();
}

/* set new epoch */
void joseta_finish_init(void){
	joseta_state.epoch = joseta_state.rtc;
	joseta_send_time(0);
}

/* set cb function */
void joseta_setcallback(joseta_cb_t fun, uint8_t mask){
	joseta_state.callback = fun;
	joseta_state.callback_mask = mask;
}

////////////////////////////////////////////////////////////////////////////////
// THREAD LOOPS  ///////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/* process serial ringbuffer */
void *joseta_serial_loop(void *arg){
	ringbuffer_t *rb = (ringbuffer_t *) arg;
	msg_t m;
	while(1){
        msg_receive(&m);
      	while (rb->avail) {
      		char c = 0;
            unsigned state = disableIRQ();
            ringbuffer_get(rb, &c, 1);
            restoreIRQ(state);
			joseta_uart_byte(c);
        }  
    }
	return 0;
}

/* process callback events */
void *joseta_callback_loop(void *arg){
	ringbuffer_t *rb = (ringbuffer_t *) arg;
	msg_t m;
	
	while(1){
    
        msg_receive(&m);
        
        switch(m.type){
        
        	case JOSETA_CB_TIMER: {
        		joseta_request_minute();
        		break;
        	}
        	
        	case JOSETA_CB_RESET: {
        		joseta_state.pending_reset = true;
        		joseta_request_minute();
        		break;
        	}
        
        	case JOSETA_CB_FRAME: {
        		if(joseta_state.callback &&
        		   joseta_state.callback_mask & JOSETA_CB_FRAME
        		){
        			joseta_df_t p;
        			unsigned state = disableIRQ();
        			ringbuffer_peek(rb, (char*) &p, sizeof(joseta_df_t));
            		restoreIRQ(state);
            		joseta_state.callback(m.type, &p, 1);
        		}
        		break;
			}
			
			case JOSETA_CB_PURGE: {
			
				joseta_df_t p[JOSETA_BUFFER_COUNT];
				unsigned count = rb->avail / sizeof(joseta_df_t);
				
				/* purge */
				unsigned state = disableIRQ();
				ringbuffer_get(rb, (char*) p, count * sizeof(joseta_df_t));
				joseta_state.frame_count = 0;
				restoreIRQ(state);
			
				/* run callback if it exists and isn't masked */
				if(joseta_state.callback &&
        		   joseta_state.callback_mask & JOSETA_CB_PURGE
        		){
					joseta_state.callback(m.type, p, count);
				}
				
				break;
				
			}
			
			case JOSETA_CB_ERROR: {
				if(joseta_state.callback &&
        		   joseta_state.callback_mask & JOSETA_CB_ERROR
        		){
        			joseta_df_t p;
        			unsigned state = disableIRQ();
        			ringbuffer_get(rb, (char*) &p, sizeof(joseta_df_t));
            		restoreIRQ(state);
            		joseta_state.callback(m.type, &p, 1);
        		}
        		break;
			}
			
			default: {
				printf("[joseta] unknown event type %d\n", m.type);
			}
        
        }
        
    }
	return 0;
}

////////////////////////////////////////////////////////////////////////////////
// INPUT-PROCESSING FUNCTIONS  /////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/* handle incoming character from uart */
void joseta_serial_recv(void *arg, char c){
    msg_t m;
    m.type = 0;
    ringbuffer_add_one(&joseta_state.serial_ringbuffer, c);
    msg_send_int(&m, joseta_state.serial_pid);
}

/* process buffered character */
void joseta_uart_byte(char c){

	joseta_state.current_frame[joseta_state.current_frame_idx++] = c;
			
	/* if frame complete */
	if(joseta_state.current_frame_idx == JOSETA_RAW_FRAME_SIZE){
		
		/* reset counter */
		joseta_state.current_frame_idx = 0;
		
		switch(joseta_state.fsm){
			
			/* if we are idle, we were not expecting data */
			case JOSETA_FSM_IDLE: {
				printf("[joseta] received unexpected data frame, discarding\n");
				break;
			}
			
			/* data frame was expected */
			case JOSETA_FSM_READ: {
				
				/* process this frame */
				joseta_process_frame();
				
				/* decrement number of frames we are expecting, check if done */
				if((--joseta_state.expected_frames) == 0){
					if(joseta_state.pending_reset){
						joseta_state.fsm = JOSETA_FSM_INIT;
						joseta_send_reset();
						joseta_state.pending_reset = false;
					}
					else{
						joseta_state.fsm = JOSETA_FSM_IDLE;
					}
				}
				
				break;
			}
			
			/* data frame was a request for epoch */
			case JOSETA_FSM_INIT: {
				printf("[joseta] board has reset, setting new epoch\n");
				joseta_finish_init();
				joseta_state.fsm = JOSETA_FSM_IDLE;
				break;
			}
			
			/* undefined fsm state */
			default: {
				printf("[joseta] driver in bad state (%d)\n", joseta_state.fsm);
				break;
			}
			
		}		
	}
	
}

/* check crc from buffered data */
bool joseta_verify_crc(void){
	/* TODO */
	return true;
}

/* process current buffered frame */
void joseta_process_frame(void){
	joseta_raw_frame_t *frame = (joseta_raw_frame_t*)joseta_state.current_frame;
	joseta_df_t parsed;
	msg_t m1, m2;
	
	if(!joseta_verify_crc()){
		printf("[joseta] discarding frame with bad crc\n");
	}
	else{
		/* parse frame */
		parsed.occupancy = frame->flags & JOSETA_FLAG_OCCUPANCY;
		parsed.relay     = frame->flags * JOSETA_FLAG_RELAY;
		memcpy((char*) &parsed.voltage, (char*) &frame->voltage,
		       JOSETA_COMMON_FRAME_LEN
		);
		
		/* pass to handler thread */
		ringbuffer_add(&joseta_state.frame_ringbuffer,
		               (const char *) &parsed,
					   sizeof(joseta_df_t)
		);
		m1.type = JOSETA_CB_FRAME;
		msg_send(&m1, joseta_state.callback_pid);
		
		/* increment frame counter, purge buffer if necessary */
		if((++joseta_state.frame_count) >= joseta_state.purgethresh){
			m2.type = JOSETA_CB_PURGE;
			msg_send(&m2, joseta_state.callback_pid);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////
// REQUEST CONTROL  ////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/* send serial frame with given type and payload */
void joseta_send_frame(uint8_t type, uint8_t payload){
	/* calc checksum */
	uint8_t cs = 0x00;
	cs = (cs + (type << 4)) & 0xff;
	cs = (cs + payload    ) & 0xff;
	cs = 0xff - cs;
	
	/* send frame */
	uart_write_blocking(JOSETA_UART, type << 4);
	uart_write_blocking(JOSETA_UART, payload);
	uart_write_blocking(JOSETA_UART, cs);
}

/* send request for data frame(s) */
void joseta_send_dreq(uint8_t addr){
	joseta_send_frame(0x1, addr << 4);
}

/* request one minute worth of data */
void joseta_request_minute(void){
	joseta_state.expected_frames = 60;
	joseta_send_dreq(0);
}

/* send time */
void joseta_send_time(uint8_t time){
	joseta_send_frame(0x04, 0x80 | time);
}

/* reset the sensor device */
void joseta_send_reset(void){
	joseta_send_frame(0x4, 0);
}

/* tick rtc and trigger request every minute */
void joseta_timer_cb(int arg){
	static uint32_t ticks = 0;
	static bool drift = false;
	if((ticks = (ticks + 1) % JOSETA_TIMER_INTERVAL) == 0){
		joseta_state.rtc++;
		if(drift){
			drift = false;
		}
		else{
			msg_t m;
			if(joseta_state.rtc % JOSETA_TIMER_DRIFT == 0){
				joseta_state.rtc--;
				drift = true;
			}
			if(joseta_state.rtc % 86400 == 0){
				msg_send(&m, joseta_state.callback_pid);
				m.type = JOSETA_CB_RESET;
			}
			else if(joseta_state.rtc % 60 == 0){
				m.type = JOSETA_CB_TIMER;
				msg_send(&m, joseta_state.callback_pid);
			}
		}
	}
}
