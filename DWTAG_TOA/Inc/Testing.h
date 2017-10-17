#ifndef __TESTING_H
#define __TESTING_H

#include "main.h"
#include "dw1000.h"
void NRF_Test(uint8_t stat);
void unlockflash(unsigned int passwd);
void going(void);
void Read_status(void);
void Start_dwrx(void);
void SET_Tpoint(void);
void GET_Time2Tpoint(void);
extern uint32 time_stack[];
extern uint16 timestack_cnt;
extern uint32 time_record;
extern uint8 triggle;

#endif
