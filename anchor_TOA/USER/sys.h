
#ifndef __SYS_H
#define __SYS_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"
#include "stdio.h"
#include "deca_types.h"
#include "deca_regs.h"
#include "deca_device_api.h"
typedef unsigned long long uint64;
/* Default antenna delay values for 64 MHz PRF. See NOTE 2 below. */
#define TX_ANT_DLY 16495
#define RX_ANT_DLY 16495

#define SEND_ORDER 1 //max 3
/* Index to access some of the fields in the frames involved in the process. */
#define ACK_FC_0 0x02
#define ACK_FC_1 0x00
#define ANCHTYPE 1
#define TAGTYPE	0
#define FRAME_SN_IDX 2
#define DESTADD 5
#define SOURADD 7
#define FUNCODE_IDX 9
#define TIMSTAMPS_OWNERID 10
#define RXBUFFTS_IDX 12
#define PAYLOADTS_IDX 2
#define FINAL_MSG_TS_LEN 5
#define WLIDX 10
#define WRIDX 11
#define PAYLOOAD_WLIDX 7
#define PAYLOOAD_WRIDX 8
#define RXBUFF_WLIDX 17
#define RXBUFF_WRIDX 18
#define FCTRL_ACK_REQ_MASK 0x20
/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 ? and 1 ? = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536


/* Delay between frames, in UWB microseconds. See NOTE 1 below. */

/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 0//这是init第一次延迟打开接受器的时间，和POLL_RX_TO_RESP_TX_DLY_UUS相关
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately  ms with above configuration. */
#define RESP_RX_TO_FINAL_TX_DLY_UUS 500//30000//延迟发射的时间
/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 0//3500 超时时间

/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
* frame length of approximately  ns with above configuration. */
#define POLL_RX_TO_RESP_TX_DLY_UUS 200//20000//延迟发射的时间
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define RESP_TX_TO_FINAL_RX_DLY_UUS 0//這是resp接受到第一次信息之後，返回發送后接收器的延遲打開時間，和RESP_RX_TO_FINAL_TX_DLY_UUS有關
/* Receive final timeout. See NOTE 5 below. */
#define FINAL_RX_TIMEOUT_UUS 0//3300
/* Preamble timeout, in multiple of PAC size. See NOTE 6 below. */
#define PRE_TIMEOUT 8


/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547
/* Length of the common part of the message (up to and including the function code, see NOTE 3 below). */
#define ALL_MSG_COMMON_LEN 10

#define RX_BUF_LEN 127
#define USAMRTCMD 63//==/?
#define ANCHORCNT 4
#define ANCHOR_NUM 1
#define TXDELAYTIME_US (ANCHOR_NUM-1)*1500UL
//#define MPUUSING
//#define TIMEBASE
#ifndef TIMEBASE
	#if ANCHOR_NUM==1
		#define MAIN_ANCHOR
	#else
		#define SLAVE_ANCHOR
	#endif
#else
	#define SYNCIDX 6
#endif
#define NRFQUELen 50

extern  __IO uint32_t msec;

extern uint8_t nrf_Tx_Buffer[33] ; // 无线传输发送数据
extern uint8_t nrf_Rx_Buffer[33] ; // 无线传输接收数据
extern uint8 rx_buffer[RX_BUF_LEN];
extern uint8 frame_seq_nb;
extern uint8 dw_payloadbuff[];
void reset_DW1000(void);
void dw_setARER(int enable);
extern uint8 DMA_transing;
extern uint8 ACKframe[];
extern uint8_t NrfQue[NRFQUELen][32];
extern uint8_t NrfQcnt;
extern uint8_t NrfQfront;
extern uint8_t NrfQrear;
#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
