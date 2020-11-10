#include "stark.h"
#include "ztt.h"
#include "stdint.h"
#include "uart1.h"
#include "main.h"

char portForSTARKBatteryIsInitiated=0;
char sTARKBatteryHndlPhase;
char sTARKRequestPhase;
short sTARKSilentCnt[3];
char sTARKButteryCnter=0;
char sTARKBatteryHndlCmnd;

//-----------------------------------------------
void stark_bat_hndl(void)
{

if(portForSTARKBatteryIsInitiated==0)
	{
	//UARTInit(1, 9600);	/* baud rate setting */
	uart1_init(9600);
	portForSTARKBatteryIsInitiated=1;
	}

sTARKBatteryHndlPhase++;

if((sTARKSilentCnt[0])&&(sTARKSilentCnt[0]<=10))sTARKSilentCnt[0]--;
if((sTARKSilentCnt[1])&&(sTARKSilentCnt[1]<=10))sTARKSilentCnt[1]--;
if((sTARKSilentCnt[2])&&(sTARKSilentCnt[2]<=10))sTARKSilentCnt[2]--;

if(sTARKBatteryHndlPhase==4)	//7E 32 36 30 30 34 36 34 32 45 30 30 32 30 31 46 44 33 30 0D
	{
	if(sTARKButteryCnter==0)
		{
		char sTARKButteryOutBuff[]={0x7e,0x32,0x30,0x30,0x30,0x34,0x36,0x34,0x32,0x45,0x30,0x30,0x32,0x30,0x30,0x46,0x44,0x33,0x37,0x0d};
		uart_out_buff1(sTARKButteryOutBuff,20);
		sTARKSilentCnt[0]=10;
		}
	else if(sTARKButteryCnter==1)
		{
		char sTARKButteryOutBuff[]={0x7e,0x32,0x36,0x30,0x30,0x34,0x36,0x34,0x32,0x45,0x30,0x30,0x32,0x30,0x32,0x46,0x44,0x32,0x46,0x0d};
		uart_out_buff1(sTARKButteryOutBuff,20);
		sTARKSilentCnt[1]=10;
		}
	else if(sTARKButteryCnter==2)
		{
		char sTARKButteryOutBuff[]={0x7e,0x32,0x36,0x30,0x30,0x34,0x36,0x34,0x32,0x45,0x30,0x30,0x32,0x30,0x33,0x46,0x44,0x32,0x45,0x0d};
		uart_out_buff1(sTARKButteryOutBuff,20);
		sTARKSilentCnt[2]=10;
		}

	sTARKRequestPhase=0;

	}
else if(sTARKBatteryHndlPhase==8)
	{
	if(sTARKButteryCnter==0)
		{
		char sTARKButteryOutBuff[]={0x7e,0x32,0x30,0x30,0x30,0x34,0x36,0x34,0x34,0x45,0x30,0x30,0x32,0x30,0x30,0x46,0x44,0x33,0x35,0x0d};
		uart_out_buff1(sTARKButteryOutBuff,20);
		sTARKSilentCnt[0]=10;
		}
/*	else if(sTARKButteryCnter==1)
		{
		char sTARKButteryOutBuff[]={0x7e,0x32,0x36,0x30,0x30,0x34,0x36,0x34,0x34,0x45,0x30,0x30,0x32,0x30,0x32,0x46,0x44,0x32,0x44,0x0d};
		uart_out_buff1(sTARKButteryOutBuff,20);
		sTARKSilentCnt[1]=10;
		}
	else if(sTARKButteryCnter==2)
		{
		char sTARKButteryOutBuff[]={0x7e,0x32,0x36,0x30,0x30,0x34,0x36,0x34,0x34,0x45,0x30,0x30,0x32,0x30,0x33,0x46,0x44,0x32,0x45,0x0d};
		uart_out_buff1(sTARKButteryOutBuff,20);
		sTARKSilentCnt[2]=10;
		} */

	sTARKRequestPhase=1;

	sTARKBatteryHndlPhase=0;
	
	sTARKButteryCnter++;
	if(sTARKButteryCnter>=NUMBAT_FSO)sTARKButteryCnter=0;
	}
}

