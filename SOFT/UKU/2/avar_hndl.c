#include "avar_hndl.h"
#include "eeprom_map.h"
#include "full_can.h"
#include "25lc640.h"
#include <LPC17xx.H>
#include "main.h"
#include "control.h"

     
//***********************************************
//Аварии
unsigned avar_stat;	 	//"Отображение" всех аварийных в данный момент устройств в одном месте
unsigned avar_ind_stat; 	//"Отображение" всех не просмотренных аварийных устройств в одном месте
unsigned avar_stat_old;
unsigned avar_stat_new,avar_stat_offed;
//Структура переменных
//1бит  - питающая сеть
//2бита - батареи
//12бит - БПСы
//5бит  - инверторы
//4бита - внешние датчики температуры
//4бита - внешние сухие контакты

char /*av_net*//*,av_bat[2]*//*av_bps[12],*/av_inv[6];//,/*av_dt[4],av_sk[4]*/;

extern char bOUT_FREE2;	






//**********************************************
//**********************************************
//**********************************************
//**********************************************
//**********************************************
//**********************************************
//**********************************************
//**********************************************
//**********************************************
//Внешние


//***********************************************
//Состояние первичной сети
//char u_net_av_stat,u_net_av_stat_;
extern signed short net_U,net_Ustore;
extern char bFF,bFF_;
extern signed short net_F,hz_out,hz_out_cnt;
extern signed char unet_drv_cnt;
extern char net_av;



//-----------------------------------------------
void avar_hndl(void)
{
//static unsigned avar_stat_old;
char i;
//unsigned avar_stat_new,avar_stat_offed;


}

//-----------------------------------------------
void reload_hndl(void)
{
char data[4];
unsigned int event_ptr,lc640_adr/*,event_ptr_find*/,event_cnt;

	
}

//-----------------------------------------------
void avar_unet_hndl(char in)
{

char data[4];
unsigned int event_ptr,lc640_adr,event_ptr_find,event_cnt;


	__nop();		
}


//-----------------------------------------------
void avar_bps_hndl(char dev, char v, char in)
{
char data[4];
unsigned short event_ptr,lc640_adr,event_ptr_find,event_cnt;
char avar_simbol;

__nop();		
}

//-----------------------------------------------
void wrk_mem_hndl(char b)
{
char data[4];
unsigned short event_ptr,lc640_adr/*,event_ptr_find*/,event_cnt;

signed short temp_temp;
	
__nop();
}  

#ifdef UKU_220_IPS_TERMOKOMPENSAT
//-----------------------------------------------
void avar_bat_ips_hndl(char in)
{
char data[4];
unsigned short event_ptr,lc640_adr,event_ptr_find,event_cnt;


__nop();		
}

#endif

//-----------------------------------------------
void avar_bat_hndl(char b, char in)
{
char data[4];
unsigned short event_ptr,lc640_adr,event_ptr_find,event_cnt;

__nop();		
}

//-----------------------------------------------
void avar_bat_as_hndl(char b, char in)
{
char data[4];
unsigned short event_ptr,lc640_adr,event_ptr_find,event_cnt;

__nop();		
}


//-----------------------------------------------
void ke_mem_hndl(char b,unsigned short in)
{
char data[4];
unsigned int event_ptr=0,lc640_adr/*,event_ptr_find*/,event_cnt;
//unsigned char temp,temp_;
//unsigned int tempUI;
//unsigned long tempUL; 
signed temp_temp;


 
}


//-----------------------------------------------
void vz_mem_hndl(unsigned short in)
{
char data[4];
unsigned int event_ptr=0,lc640_adr,event_ptr_find=0,event_cnt;
//char avar_simbol;

__nop(); 

}
  



