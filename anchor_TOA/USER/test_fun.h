#ifndef _TEST_FUN
#define _TEST_FUN
#include "sys.h"
#include "deca_regs.h"
#include "deca_device_api.h"
void System_GetClocks(void);
void getSYSstatus(unsigned char index);
void getIDs(unsigned char index);
void read_test(unsigned char add);
void Unset_start_AR(void);
void Set_start_AR(void);
void Prt_anchnum(void);
void unlockflash(unsigned int passwd);
void testfun1(void);
void testfun2(void);
void getloctime(void);
void DMA_test(void);
#endif
