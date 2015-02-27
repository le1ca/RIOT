#ifndef joseta_h
#define joseta_h

#include <stdint.h>
#include <stdbool.h>

/////////////////////////////////////////////////////
// public interface to jose's sensor/control board //
/////////////////////////////////////////////////////

/* max number of records to buffer */
#define JOSETA_BUFFER_COUNT (60*2) /* 2 minutes of data */

/* overall size of buffer */
#define JOSETA_BUFFER_SIZE  (JOSETA_BUFFER_COUNT * sizeof(joseta_df_t))

/* uart config */
#define JOSETA_UART 	UART_7
#define JOSETA_UART_BUF	128

/* default settings */
#define JOSETA_DEFAULT_PURGETHRESH (JOSETA_BUFFER_COUNT / 2)
#define JOSETA_SERIAL_STACK   512
#define JOSETA_CALLBACK_STACK 3072

/* callback masking */
#define JOSETA_CB_TIMER 0
#define JOSETA_CB_FRAME 1
#define JOSETA_CB_PURGE 2
#define JOSETA_CB_ERROR 4
#define JOSETA_CB_RESET 8

/* specification for parsed data frame */
typedef struct joseta_df {
	bool occupancy;
	bool relay;
	uint16_t voltage;
	uint16_t current;
	uint16_t phase;
	uint8_t  temp;
	uint64_t time;
	uint8_t  error;
} joseta_df_t;

/* callback function for data acquisition */
typedef void (*joseta_cb_t)(unsigned int type, joseta_df_t *buf, 
							unsigned int num_records
);

/* initialize board and start polling thread */
void joseta_init(uint64_t rtc);

/* set threshold at which to run purge callback and empty buffer */
void joseta_setpurgethresh(unsigned int num_records);

/* set function to call when events occur */
void joseta_setcallback(joseta_cb_t fun, uint8_t mask);

#endif
