#ifndef __TESTING_H
#define __TESTING_H

#include "main.h"
#include "dw1000.h"
extern uint8 triggle;
void NRF_Test(uint8_t stat);
void unlockflash(unsigned int passwd);
void going(void);
void Read_status(void);
void Start_dwrx(void);
#endif
