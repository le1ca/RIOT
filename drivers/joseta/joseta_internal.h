#ifndef joseta_internal_h
#define joseta_internal_h

#include "joseta.h"
#include "thread.h"
#include "ringbuffer.h"
#include "string.h"

#include "hwtimer_cpu.h"
#include "periph_conf.h"
#include "periph/uart.h"
#include "periph/timer.h"
#include "driverlib/timer.h"
#include "driverlib/rom.h"

#define JOSETA_RAW_FRAME_SIZE 16
#define JOSETA_NORMAL_FRAMES  60
#define JOSETA_RETX_FRAMES     4

#define JOSETA_COMMON_FRAME_LEN (sizeof(joseta_df_t) - 2*sizeof(bool))

#define JOSETA_TIMER_NUM        TIMER_3
#define JOSETA_TIMER_BASE       TIMER3_BASE
#define JOSETA_TIMER_HZ         (uint32_t) 16000000L
#define JOSETA_TIMER_MAX        ((JOSETA_TIMER_HZ/1000)+94)
#define JOSETA_TIMER_PRESCALE   (uint32_t) 99
#define JOSETA_TIMER_INTERVAL   10
#define JOSETA_TIMER_DRIFT      3636

#define JOSETA_FLAG_OCCUPANCY 0x01
#define JOSETA_FLAG_RELAY     0x02

#define JOSETA_ERROR_RESET    0x80
#define JOSETA_ERROR_RELAY    0x40
#define JOSETA_ERROR_BUS      0x20

typedef enum {
	JOSETA_FSM_IDLE,
	JOSETA_FSM_READ,
	JOSETA_FSM_INIT
} joseta_fsm_state_t;

typedef struct joseta_state_t {

	ringbuffer_t frame_ringbuffer;
	ringbuffer_t serial_ringbuffer;
	unsigned int frame_count;
	
	uint64_t     epoch;
	uint64_t     rtc;
	unsigned int purgethresh;
	joseta_cb_t  callback;
	unsigned int callback_mask;

	kernel_pid_t serial_pid;
	kernel_pid_t callback_pid;
	
	joseta_fsm_state_t fsm;
	
	char current_frame[JOSETA_RAW_FRAME_SIZE];
	unsigned int current_frame_idx;
	unsigned int expected_frames;
	bool pending_reset;
	
} joseta_state_t;

typedef struct joseta_raw_frame_t {
	uint8_t  flags;
	uint16_t voltage;
	uint16_t current;
	uint16_t phase;
	uint8_t  temperature;
	uint32_t timestamp;
	uint8_t  id;
	uint8_t  error;
	uint16_t crc;
} joseta_raw_frame_t;

void joseta_state_init(uint64_t rtc);
void joseta_uart_init(void);
void joseta_serial_thread_init(void);
void joseta_callback_thread_init(void);
void joseta_board_init(void);
void joseta_timer_init(void);
void joseta_timer_cb(int arg);
void joseta_finish_init(void);
void joseta_request_minute(void);
void joseta_process_frame(void);
void joseta_send_time(uint8_t time);
void joseta_send_dreq(uint8_t addr);
void joseta_send_reset(void);
void joseta_serial_recv(void *arg, char c);
void joseta_uart_byte(char c);
void *joseta_serial_loop(void *arg);
void *joseta_callback_loop(void *arg);
bool joseta_verify_crc(void);

#endif
