#ifndef __MAIN_H
#define __MAIN_H


#include "deca_types.h"
#include "usmart.h"
#include "stdint.h"
#include "rf24l01.h"
#include "delay.h"
#include "math.h"

/* Length of the common part of the message (up to and including the function code, see NOTE 2 below). */
#define ALL_MSG_COMMON_LEN 10
/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX 2
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define FINAL_MSG_TS_LEN 4
#define POLL_TX_TO_RESP_RX_DLY_UUS 150
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.66 ms with above configuration. */
#define RESP_RX_TO_FINAL_TX_DLY_UUS 3100
/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 2700


//#inlcude "USART.h"
#define ACK_FC_0 0x02
#define ACK_FC_1 0x00
#define RX_BUF_LEN 35
#define FRAME_IDX 2
#define DESTADD 5
#define SOURADD 7
#define FUNCODE_IDX 9
#define XTAL_FREQ_HZ 38400000
#define SLEEP_TIME_MS 500
#define UUS_TO_DWT_TIME 65536
#define q30 1073741824.0f
#define TX_ANT_DLY 16495
#define RX_ANT_DLY 16495
#define TAG_ID 1u|0x8000u
#if TAG_ID>0x8000u
#else
#define TIMEBASE
#define SYNCIDX 6
#endif
//#define USINGMPU
#ifndef USINGMPU
#define PLLMSGLEN 14
#else
#define PLLMSGLEN 126
#endif
#define WLIDX 10
#define WRIDX 11
#define QUANTITY_ANCHOR 3
#define TOA_MSG_LEN 12+4*QUANTITY_ANCHOR
#define TOA_DATA_IDX 10
//#define FLASHPROTECT
//#define MAXRDPLEVEL
typedef unsigned long long uint64;
typedef struct 
{
	uint16_t RC :1;
	uint16_t RD :1;
	uint16_t count :14;//16383 bytes
	
}usart_bitfield;
extern uint8_t nrf_Tx_Buffer[33] ; // 无线传输发送数据
extern uint8_t nrf_Rx_Buffer[33] ; // 无线传输接收数据
extern uint8_t USART_tmp;
extern uint8_t usart_rx_buff[64];
extern usart_bitfield USART_STA;
extern unsigned int localtime;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;

extern TIM_HandleTypeDef htim14;

extern UART_HandleTypeDef huart1;
extern char newdata;
extern uint8 dw_txseq_num;
extern uint8 dwiswake;

extern volatile uint8 isframe_sent;
extern volatile uint8 istxframe_acked;
extern volatile uint8 isreceive_To;
extern volatile uint8 isframe_rec;
extern volatile uint8 isack_sent;
extern uint8 rx_buffer[];
extern volatile uint8 tim14_int;
#endif
