#include "usmart.h"
#include "usmart_str.h" 
#include "main.h"
#include "stm32f0xx_hal.h"
#include "Testing.h"
////////////////////////////用户配置区///////////////////////////////////////////////
//这下面要包含所用到的函数所申明的头文件(用户自己添加) 


												 
extern void test_fun(void(*ledset)(uint8_t),uint8_t sta);
extern void * test1(void);
//函数名列表初始化(用户自己添加)
//用户直接在这里输入要执行的函数名及其查找串
struct _m_usmart_nametab usmart_nametab[]=
{
#if USMART_USE_WRFUNS==1 	//如果使能了读写操作
	(void*)read_addr,"uint32_t read_addr(uint32_t addr)",
	(void*)write_addr,"void write_addr(uint32_t addr,uint32_t val)",	 
#endif
	(void*)NRF_Test,"void NRF_Test(uint8_t stat)",
	(void*)unlockflash,"void unlockflash(unsigned int passwd)",
	(void*)going,"void going(void)",
	(void*)Read_status,"void Read_status(void)",
	(void *)Start_dwrx,"void Start_dwrx(void)",
	(void*)ShowTimeStack,"void ShowTimeStack(void)",
#ifdef SetDelay
		(void*)Set_AtennaDelay,"void Set_AtennaDelay(uint16 delay)",

#endif

											 
};						  
///////////////////////////////////END///////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
//函数控制管理器初始化
//得到各个受控函数的名字
//得到函数总数量
struct _m_usmart_dev usmart_dev=
{
	usmart_nametab,
	usmart_init,
	usmart_cmd_rec,
	usmart_exe,
	usmart_scan,
	sizeof(usmart_nametab)/sizeof(struct _m_usmart_nametab),//函数数量
	0,	  	//参数数量
	0,	 	//函数ID
	1,		//参数显示类型,0,10进制;1,16进制
	0,		//参数类型.bitx:,0,数字;1,字符串	    
	0,	  	//每个参数的长度暂存表,需要MAX_PARM个0初始化
	0,		//函数的参数,需要PARM_LEN个0初始化
};   



















