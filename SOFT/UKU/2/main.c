//Ветка для CAN на MCP2515 короткая


#include "lcd_AGM1232_uku207_3.h"
#include "rtl.h"
#include "type.h"
#include "main.h"
#include "simbol.h"
#include "25lc640.h"
#include "Timer.h"
#include "gran.h"
#include "uart0.h"
#include "uart1.h"
#include "uart2.h"
#include "cmd.h"
#include "ret.h"
#include "eeprom_map.h"
#include "common_func.h"
#include "control.h"
#include "mess.h"
#include "full_can.h"
#include "watchdog.h"
#include "ad7705.h"
#include "beep.h"
#include "avar_hndl.h"
#include "memo.h"
#include "simbols.h"
#include "graphic.h"
#include "snmp_data_file.h" 
#include "net_config.h"
#include "uart0.h"
#include <rtl.h>
#include "modbus.h"
#include "mcp2515.h"

BOOL tick;
//extern LOCALM localm[];
//#define MY_IP localm[NETIF_ETH].IpAdr
#define DHCP_TOUT   50

//***********************************************
//Таймер
char b10000Hz,b1000Hz,b2000Hz,b100Hz,b50Hz,b10Hz,b5Hz,b2Hz,b1Hz,b1min;
short t0cnt,t0cnt0,t0cnt1,t0cnt2,t0cnt3,t0cnt4,t0cnt5,t0cnt6,t0_cnt7,t0_cnt_min;
char bFL5,bFL2,bFL,bFL_,bTPS;
signed short main_10Hz_cnt=0;
signed short main_1Hz_cnt=0;

 
//***********************************************
//Структура ИБЭПа
char cnt_of_slave=3;
//char cnt_of_wrks;   //колличество работающих источников , для индикации



//**********************************************
//Коэффициенты, отображаемые из EEPROM
signed short Ktsrc[2];
signed short Kusrc[2];
signed short Kisrc[2];
signed short Ki0src[2];
signed short Kubat[2];
signed short Kubatm[2];
unsigned short Kibat0[2];
signed short Kibat1[2];
signed short Ktbat[2];
signed short Kunet;
signed short Kunet_ext[3];
signed short Ktext[3];
signed short Kuload;
signed short KunetA;
signed short KunetB;
signed short KunetC;

signed short MAIN_IST;
signed short UMAX;
signed short UB0;
signed short UB20;
signed short TMAX;
signed short TSIGN;
signed short AV_OFF_AVT;
signed short USIGN;
signed short UMN;
signed short ZV_ON;
signed short IKB;
//signed short KVZ;
signed short UVZ;
signed short IMAX;
signed short IMIN;
signed short APV_ON;
signed short IZMAX;
signed short U0B;
signed short TZAS;
signed short VZ_HR;
signed short TBAT;
signed short U_AVT;
signed short DU;
signed short PAR;
signed short TBATMAX;
signed short TBATSIGN;
signed short UBM_AV;
signed short RELE_LOG;
signed short TBOXMAX;
signed short TBOXREG;
signed short TBOXVENTMAX;
signed short TLOADDISABLE;
signed short TLOADENABLE;
signed short TBATDISABLE;
signed short TBATENABLE;
signed short TVENTON;
signed short TVENTOFF;
signed short TWARMON;
signed short TWARMOFF;
enum_releventsign RELEVENTSIGN;
signed short TZNPN;
signed short UONPN;
signed short UVNPN;
enum_npn_out NPN_OUT;
enum_npn_sign NPN_SIGN;
signed short TERMOKOMPENS;
signed short TBOXVENTON; 
signed short TBOXVENTOFF;
signed short TBOXWARMON; 
signed short TBOXWARMOFF;
signed short BAT_TYPE;

signed short NUMBAT;
signed short NUMIST;
signed short NUMINV;
signed short NUMDT;
signed short NUMSK;
signed short NUMEXT;
signed short NUMAVT;
signed short NUMMAKB;
signed short NUMBYPASS;

enum_apv_on APV_ON1,APV_ON2;
signed short APV_ON2_TIME;

enum_bat_is_on BAT_IS_ON[2];
signed short BAT_DAY_OF_ON[2];
signed short BAT_MONTH_OF_ON[2];
signed short BAT_YEAR_OF_ON[2];
signed short BAT_C_NOM[2];
signed short BAT_RESURS[2];
signed short BAT_C_REAL[2];
//signed short BAT_TYPE[2];

unsigned short AUSW_MAIN;
unsigned long 	AUSW_MAIN_NUMBER;
unsigned short AUSW_DAY;
unsigned short AUSW_MONTH;
unsigned short AUSW_YEAR;
unsigned short AUSW_UKU;
unsigned short AUSW_UKU_SUB;
unsigned long AUSW_UKU_NUMBER;
unsigned long	AUSW_BPS1_NUMBER;
unsigned long  AUSW_BPS2_NUMBER;
unsigned short AUSW_RS232;
unsigned short AUSW_PDH;
unsigned short AUSW_SDH;
unsigned short AUSW_ETH;

signed short TMAX_EXT_EN[3];
signed short TMAX_EXT[3];
signed short TMIN_EXT_EN[3];
signed short TMIN_EXT[3];
signed short T_EXT_REL_EN[3];
signed short T_EXT_ZVUK_EN[3];
signed short T_EXT_LCD_EN[3];
signed short T_EXT_RS_EN[3];

signed short SK_SIGN[4];
signed short SK_REL_EN[4];
signed short SK_ZVUK_EN[4];
signed short SK_LCD_EN[4];
signed short SK_RS_EN[4];

enum_avz AVZ;

unsigned short HOUR_AVZ;
unsigned short MIN_AVZ;
unsigned short SEC_AVZ;
unsigned short DATE_AVZ;
unsigned short MONTH_AVZ;
unsigned short YEAR_AVZ;
unsigned short AVZ_TIME;

enum_mnemo_on MNEMO_ON;
unsigned short MNEMO_TIME;

signed short POWER_CNT_ADRESS;

signed short ETH_IS_ON;
signed short ETH_DHCP_ON;
signed short ETH_IP_1;
signed short ETH_IP_2;
signed short ETH_IP_3;
signed short ETH_IP_4;
signed short ETH_MASK_1;
signed short ETH_MASK_2;
signed short ETH_MASK_3;
signed short ETH_MASK_4;
signed short ETH_TRAP1_IP_1;
signed short ETH_TRAP1_IP_2;
signed short ETH_TRAP1_IP_3;
signed short ETH_TRAP1_IP_4;
signed short ETH_TRAP2_IP_1;
signed short ETH_TRAP2_IP_2;
signed short ETH_TRAP2_IP_3;
signed short ETH_TRAP2_IP_4;
signed short ETH_TRAP3_IP_1;
signed short ETH_TRAP3_IP_2;
signed short ETH_TRAP3_IP_3;
signed short ETH_TRAP3_IP_4;
signed short ETH_TRAP4_IP_1;
signed short ETH_TRAP4_IP_2;
signed short ETH_TRAP4_IP_3;
signed short ETH_TRAP4_IP_4;
signed short ETH_TRAP5_IP_1;
signed short ETH_TRAP5_IP_2;
signed short ETH_TRAP5_IP_3;
signed short ETH_TRAP5_IP_4;

signed short ETH_SNMP_PORT_READ;
signed short ETH_SNMP_PORT_WRITE;

signed short ETH_GW_1;
signed short ETH_GW_2;
signed short ETH_GW_3;
signed short ETH_GW_4;

signed short RELE_VENT_LOGIC;

signed short MODBUS_ADRESS;
signed short MODBUS_BAUDRATE;
signed short BAT_LINK;


//***********************************************
//Состояние батарей
BAT_STAT bat[2],bat_ips;
signed short		bat_u_old_cnt;
signed short 		Ib_ips_termokompensat;

//***********************************************
//Мониторы АКБ
MAKB_STAT makb[4];

//***********************************************
//Литиевые АКБ
LAKB_STAT lakb[2];

//***********************************************
//Телеметрия по внутренней шине
char can_slot[12][16];


//***********************************************
//Состояние источников
BPS_STAT bps[40];

//***********************************************
//Состояние инверторов
INV_STAT inv[20];
char first_inv_slot=MINIM_INV_ADRESS;

//***********************************************
//Состояние байпаса
BYPS_STAT byps;

//***********************************************
//Состояние нагрузки
signed short load_U;
signed short load_I;


//***********************************************
//Индикация

char lcd_buffer[LCD_SIZE+100]={"Hello World"};
signed char parol[3];
char phase;
char lcd_bitmap[1024];
char dig[5];
char dumm_ind[20];
stuct_ind a_ind,b_ind[10],c_ind;
char dumm_ind_[20];
char zero_on;
char mnemo_cnt=50;
char simax;
short av_j_si_max;
const char ABCDEF[]={"0123456789ABCDEF"};
const char sm_mont[13][4]={"   ","янв","фев","мар","апр","май","июн","июл","авг","сен","окт","ноя","дек"}; //
signed short ptr_ind=0;

signed short ind_pointer=0;

//***********************************************
//Состояние первичной сети
signed short net_U,net_Ustore,net_Ua,net_Ub,net_Uc;
char bFF,bFF_;
signed short net_F,hz_out,hz_out_cnt,net_F3;
signed char unet_drv_cnt;
char net_av;

//***********************************************
//Состояние внешних датчиков
//signed short tout[4];
char tout_max_cnt[4],tout_min_cnt[4];
enum_tout_stat tout_stat[4];
signed short t_ext[3];
BOOL ND_EXT[3];
signed char sk_cnt_dumm[4],sk_cnt[4],sk_av_cnt[4];
enum_sk_stat sk_stat[4]={ssOFF,ssOFF,ssOFF,ssOFF};
enum_sk_av_stat sk_av_stat[4]={sasOFF,sasOFF,sasOFF,sasOFF},sk_av_stat_old[4];
signed short t_box;

//***********************************************
//Звуки
extern char beep_cnt;
BOOL bSILENT;








signed short u_necc,u_necc_,u_necc_up,u_necc_dn;
signed short main_cnt_5Hz;
signed short num_necc;
signed short num_necc_Imax;
signed short num_necc_Imin;
signed short cnt_num_necc;
//char bSAME_IST_ON;
signed mat_temper;

//***********************************************
//АПВ
unsigned main_apv_cnt,hour_apv_cnt[2],reset_apv_cnt[2];
unsigned short apv_cnt_sec[2],apv_cnt[2];

//***********************************************
//Текстовые константы
const char sm_[]	={"                    "};
const char sm_exit[]={" Выход              "};
const char sm_time[]={" 0%:0^:0& 0</>  /0{ "};





//**********************************************
//Работа с кнопками 
char but;                            
unsigned long but_n,but_s;
char but0_cnt;
char but1_cnt;
char but_onL_temp;

//***********************************************
//Межблоковая связь
char cnt_net_drv;

//***********************************************
//КАН 
extern char ptr_can1_tx_wr,ptr_can1_tx_rd;
extern char ptr_can2_tx_wr,ptr_can2_tx_rd;
extern unsigned short rotor_can[6];
extern char RXBUFF[40],TXBUFF[40];





//***********************************************
//Работа с кнопками
char speed,l_but,n_but;

//***********************************************
//Неразобранное
enum {wrkON=0x55,wrkOFF=0xAA}wrk;
char cnt_wrk;
signed short ibat_integr;
unsigned short av_beep,av_rele,av_stat;
char default_temp;
char ND_out[3];

//***********************************************
//Тест
enum_tst_state tst_state[15];

//***********************************************
//АЦП
//extern short adc_buff[16][16],adc_buff_[16];
extern char adc_cnt,adc_cnt1,adc_ch;

//***********************************************

char flag=0;


extern signed short bat_ver_cnt;
signed short Isumm;
signed short Isumm_;

#include <LPC17xx.H>                        /* LPC21xx definitions */



/*
extern void lcd_init(void);
extern void lcd_on(void);
extern void lcd_clear(void);
*/

extern short plazma_adc_cnt;
extern char net_buff_cnt;
extern unsigned short net_buff[32],net_buff_;
extern char rele_stat/*,rele_stat_*/;
extern char bRXIN0;


char cntrl_plazma;
extern char bOUT_FREE2;
extern char /*av_net,*//*av_bat[2],*/av_bps[12],av_inv[6],av_dt[4],av_sk[4];

char content[63];

//const short ptr_bat_zar_cnt[2]={EE_ZAR1_CNT,EE_ZAR2_CNT};


//unsigned short YEAR_AVZ,MONTH_AVZ,DATE_AVZ,HOUR_AVZ,MIN_AVZ,SEC_AVZ;


//**********************************************
//Самокалиброввка
extern signed short samokalibr_cnt;

//**********************************************
//Сообщения
extern char mess[MESS_DEEP],mess_old[MESS_DEEP],mess_cnt[MESS_DEEP];
extern short mess_par0[MESS_DEEP],mess_par1[MESS_DEEP],mess_data[2];


//**********************************************
//Контроль наличия батарей
extern signed short 	main_kb_cnt;
extern signed short 	kb_cnt_1lev;
extern signed short 	kb_cnt_2lev;
extern char 			kb_full_ver;
extern char 			kb_start[2],kb_start_ips;

//***********************************************
//Управление ШИМом
extern signed short cntrl_stat;
extern signed short cntrl_stat_old;
extern signed short cntrl_stat_new;
extern signed short Ibmax;


//-----------------------------------------------
//Контроль заряда
char sign_U[2],sign_I[2];
char superviser_cnt;


char plazma_plazma_plazma;

char bRESET=0;
char bRESET_EXT=0;
char ext_can_cnt;
//-----------------------------------------------
//Состояние вводов
signed short vvod_pos;

//-----------------------------------------------
//Плата расширения
unsigned short adc_buff_ext_[3];
unsigned short Uvv[3];
unsigned short Uvv0;
short pos_vent;
short t_ext_can;
char t_ext_can_nd;


//-----------------------------------------------
//Плата расширения 2
char eb2_data[30];
short eb2_data_short[10];
short Uvv_eb2[3],Upes_eb2[3];
short Kvv_eb2[3],Kpes_eb2[3];
//-----------------------------------------------
//Работа со щетчиком
signed long power_summary;
signed short power_current;

//-----------------------------------------------
//Климатконтроль и вентиляторы
signed short main_vent_pos;
signed char t_box_cnt=0;
enum_mixer_vent_stat mixer_vent_stat=mvsOFF;
INT_BOX_TEMPER ibt;
enum_tbatdisable_stat tbatdisable_stat=tbdsON;
enum_tloaddisable_stat tloaddisable_stat=tldsON;
enum_av_tbox_stat av_tbox_stat=atsOFF;
signed short av_tbox_cnt;
char tbatdisable_cmnd=20,tloaddisable_cmnd=22;
short tbatdisable_cnt,tloaddisable_cnt;
#ifdef UKU_KONTUR
short t_box_vent_on_cnt;
short t_box_warm_on_cnt;
enum_vent_stat vent_stat_k=vsON;
enum_warm_stat warm_stat_k=wsON;
#endif

//-----------------------------------------------
//Состояние контролируемых автоматов нагрузки 
enum_avt_stat avt_stat[12],avt_stat_old[12];

//short sys_plazma,sys_plazma1;

char snmp_plazma;


short plazma_but_an;

char bCAN_OFF;


char max_net_slot;

//-----------------------------------------------
//Показания АЦП на плате измерения тока батареи
signed long ibat_metr_buff_[2];
short bIBAT_SMKLBR;

//-----------------------------------------------
//Управление низкоприоритетной нагрузкой
signed short npn_tz_cnt;
enum_npn_stat npn_stat=npnsON;


char ips_bat_av_vzvod=0;
char ips_bat_av_stat=0;

char rel_warm_plazma;
char can_byps_plazma0,can_byps_plazma1;

char bCAN_INV;
char plazma_can_inv[3];

unsigned short bat_drv_rx_cnt;
char bat_drv_rx_buff[512];
char bat_drv_rx_in;

short plazma_bat_drv0,plazma_bat_drv1,bat_drv_cnt_cnt;
short can_plazma;

//-----------------------------------------------
void rtc_init (void) 
{
LPC_RTC->CCR=0x11;
}

//-----------------------------------------------
static void timer_poll () 
{

}

//-----------------------------------------------
void inv_search(void)
{
char i;

first_inv_slot=8;
for(i=0;i<12;i++)
	{
	if(bps[i]._device==dINV)
		{
		first_inv_slot=i;
		break;
		}
	}
}

//-----------------------------------------------
signed short abs_pal(signed short in)
{
if(in<0)return -in;
else return in;
}




//-----------------------------------------------
void ADC_IRQHandler(void) {
LPC_ADC->ADCR &=  ~(7<<24);



adc_self_ch_buff[adc_self_ch_cnt]=(LPC_ADC->ADGDR>>4) & 0xFFF;/* Read Conversion Result             */
adc_self_ch_cnt++;
if(adc_self_ch_cnt<3)
	{
	LPC_ADC->ADCR |=  (1<<24);
	}
else
	{

 
	//SET_REG(LPC_ADC->ADCR,1,24,3);
	}

/*			adc_buff_[0]=AD_last;
			if(AD_last<adc_buff_min[adc_ch])adc_buff_min[adc_ch]=AD_last;
			if(AD_last>adc_buff_max[adc_ch])adc_buff_max[adc_ch]=AD_last;*/
}

//-----------------------------------------------
void def_set(int umax__,int ub0__,int ub20__,int usign__,int imax__,int uob__,int numi,int _uvz)
{
;
lc640_write_int(EE_NUMIST,numi);
lc640_write_int(EE_NUMINV,0);
//lc640_write_int(EE_NUMDT,0);
//lc640_write_int(EE_NUMSK,0);
lc640_write_int(EE_MAIN_IST,0);
lc640_write_int(EE_PAR,1);
lc640_write_int(EE_TBAT,60);
lc640_write_int(EE_UMAX,umax__);
lc640_write_int(EE_DU,ub20__/2);
lc640_write_int(EE_UB0,ub0__);
lc640_write_int(EE_UB20,ub20__);
lc640_write_int(EE_TSIGN,70);
lc640_write_int(EE_TMAX,80);
//lc640_write_int(EE_C_BAT,180);
lc640_write_int(EE_USIGN,usign__);
lc640_write_int(EE_UMN,187);
lc640_write_int(EE_ZV_ON,0);
lc640_write_int(EE_IKB,10);
//lc640_write_int(EE_KVZ,1030);
lc640_write_int(EE_UVZ,_uvz);
lc640_write_int(EE_IMAX,imax__);
lc640_write_int(EE_IMIN,(imax__*8)/10);
//lc640_write_int(EE_APV_ON,apvON);
lc640_write_int(EE_APV_ON1,apvON);
lc640_write_int(EE_APV_ON2,apvON);
lc640_write_int(EE_APV_ON2_TIME,1);
lc640_write_int(EE_IZMAX,160);
lc640_write_int(EE_U0B,uob__);
lc640_write_int(EE_TZAS,3);
lc640_write_int(EE_TBATMAX,50);  
lc640_write_int(EE_TBATSIGN,40);
lc640_write_int(EE_MNEMO_ON,mnON);
lc640_write_int(EE_MNEMO_TIME,30);	
lc640_write_int(EE_AV_OFF_AVT,1);
//lc640_write_int(EE_APV_ON1,apvOFF);



lc640_write_int(EE_TBOXMAX,70);
lc640_write_int(EE_TBOXVENTMAX,60);
lc640_write_int(EE_TBOXREG,25);
lc640_write_int(EE_TLOADDISABLE,80);
lc640_write_int(EE_TLOADENABLE,70);
lc640_write_int(EE_TBATDISABLE,91);
lc640_write_int(EE_TBATENABLE,80);

lc640_write_int(ADR_SK_SIGN[0],0);
lc640_write_int(ADR_SK_REL_EN[0],0);
lc640_write_int(ADR_SK_LCD_EN[0],0xffff);

lc640_write_int(ADR_SK_SIGN[1],0);
lc640_write_int(ADR_SK_REL_EN[1],0);
lc640_write_int(ADR_SK_LCD_EN[1],0xffff);

lc640_write_int(ADR_SK_SIGN[2],0);
lc640_write_int(ADR_SK_REL_EN[2],0);
lc640_write_int(ADR_SK_LCD_EN[2],0xffff);

lc640_write_int(ADR_SK_SIGN[3],0);
lc640_write_int(ADR_SK_REL_EN[3],0);
lc640_write_int(ADR_SK_LCD_EN[3],0xffff);

lc640_write_int(EE_UBM_AV,10);

lc640_write_int(EE_POS_VENT,11);
}


//-----------------------------------------------
void def_ips_set(short voltage)
{
if(voltage==24)
	{
	def_set(300,voltage,voltage,22,150,240,7,0);
	}
if(voltage==48)
	{
	def_set(600,voltage,voltage,44,100,480,7,0);
	}
if(voltage==60)
	{
	def_set(750,voltage,voltage,55,100,600,7,0);
	}

if(voltage==220)
	{
	def_set(2450,2366,2315,187,100,2200,2,2346);
	lc640_write_int(EE_DU,2315-1870);
	lc640_write_int(EE_U_AVT,2200);
	lc640_write_int(EE_IZMAX,20);
	lc640_write_int(EE_AUSW_MAIN,22033);
	lc640_write_int(EE_PAR,1);
	lc640_write_int(EE_MNEMO_ON,mnOFF);
	}

lc640_write_int(ADR_EE_BAT_IS_ON[0],bisOFF);
lc640_write_int(ADR_EE_BAT_DAY_OF_ON[0],LPC_RTC->DOM);
lc640_write_int(ADR_EE_BAT_MONTH_OF_ON[0],LPC_RTC->MONTH);
lc640_write_int(ADR_EE_BAT_YEAR_OF_ON[0],LPC_RTC->YEAR);
lc640_write_int(ADR_EE_BAT_C_NOM[0],0);
lc640_write_int(ADR_EE_BAT_RESURS[0],0);

lc640_write_int(ADR_EE_BAT_IS_ON[1],bisOFF);
lc640_write_int(ADR_EE_BAT_DAY_OF_ON[1],LPC_RTC->DOM);
lc640_write_int(ADR_EE_BAT_MONTH_OF_ON[1],LPC_RTC->MONTH);
lc640_write_int(ADR_EE_BAT_YEAR_OF_ON[1],LPC_RTC->YEAR);
lc640_write_int(ADR_EE_BAT_C_NOM[1],0);
lc640_write_int(ADR_EE_BAT_RESURS[1],0);
}

//-----------------------------------------------
void can_reset_hndl(void)
{


}

//-----------------------------------------------
void net_drv(void)
{ 
//char temp_;    



}

//-----------------------------------------------
void net_drv_mcp2515(void)
{ 
//char temp_;    



max_net_slot=MINIM_INV_ADRESS+NUMINV+8;
//if(NUMINV) max_net_slot=MINIM_INV_ADRESS+NUMINV;
//gran_char(&max_net_slot,0,MAX_NET_ADRESS);

if(++cnt_net_drv>max_net_slot) 
	{
	cnt_net_drv=0;
	//LPC_GPIO2->FIODIR|=(1UL<<7);
	//LPC_GPIO2->FIOPIN^=(1UL<<7);
	if(bCAN_INV)bCAN_INV=0;
	else bCAN_INV=1;

	} 


#ifndef UKU_KONTUR
if(cnt_net_drv<=11) // с 1 по 12 посылки адресные
#endif
#ifdef UKU_KONTUR
if(cnt_net_drv<=7) // с 1 по 12 посылки адресные
#endif
	{
	//cnt_net_drv=2; 
	if(mess_find_unvol(MESS2NET_DRV))
		{
		if(mess_data[0]==PARAM_BPS_NET_OFF)
			{
			//mess_data[1]=1;
			if(sub_ind1==cnt_net_drv)
				{
				return;
				}
			}
		}
			   
	if(!bCAN_OFF)mcp2515_transmit(cnt_net_drv,cnt_net_drv,0xed,bps[cnt_net_drv]._flags_tu,*((char*)(&bps[cnt_net_drv]._vol_u)),*((char*)((&bps[cnt_net_drv]._vol_u))+1),*((char*)(&bps[cnt_net_drv]._vol_i)),*((char*)((&bps[cnt_net_drv]._vol_i))+1));
     
	if(cnt_net_drv<=11)
	     {
	     if(bps[cnt_net_drv]._cnt<CNT_SRC_MAX)
   	 		{    
   	 		bps[cnt_net_drv]._cnt++;
   	 		if( (bps[cnt_net_drv]._cnt>=CNT_SRC_MAX) && (!net_av) && (!(bps[cnt_net_drv]._av&0x08)) && (cnt_net_drv<NUMIST) ) 
   	 			{
   	 			avar_bps_hndl(cnt_net_drv,3,1);
   	 			}
   	 		}
		else bps[cnt_net_drv]._cnt=CNT_SRC_MAX;
						
		if((bps[cnt_net_drv]._cnt>=3)&&(bps[cnt_net_drv]._cnt_old<3))bps[cnt_net_drv]._cnt_more2++;
		bps[cnt_net_drv]._cnt_old=bps[cnt_net_drv]._cnt;
	     }
	}

else if(cnt_net_drv==12)
	{
     if(!bCAN_OFF)mcp2515_transmit(0xff,0xff,MEM_KF,*((char*)(&UMAX)),*((char*)((&UMAX))+1),*((char*)(&DU)),*((char*)((&DU))+1),0);
     } 
     
else if(cnt_net_drv==13)
	{
     if(!bCAN_OFF)mcp2515_transmit(0xff,0xff,MEM_KF1,*((char*)(&TMAX)),*((char*)((&TMAX))+1),*((char*)(&TSIGN)),*((char*)((&TSIGN))+1),(char)TZAS);
     byps._cnt++;
	} 
else if(cnt_net_drv==14)
	{                 
	static char makb_cnt;
	makb_cnt++;
	if(makb_cnt>=4)makb_cnt=0;
     if(!bCAN_OFF)mcp2515_transmit(14,14,GET_MAKB,makb_cnt,makb_cnt,0,0,0);
	makb[makb_cnt]._cnt++;
	if(makb[makb_cnt]._cnt>20)makb[makb_cnt]._cnt=20;
	}
	
	
else if(cnt_net_drv==15)
	{
     if(!bCAN_OFF)mcp2515_transmit(0xff,0xff,MEM_KF1,*((char*)(&TMAX)),*((char*)((&TMAX))+1),*((char*)(&TSIGN)),*((char*)((&TSIGN))+1),(char)TZAS);
     }

/*
else if((cnt_net_drv>=MINIM_INV_ADRESS)&&(cnt_net_drv<(MINIM_INV_ADRESS+NUMINV))&&(NUMINV))
	{
    if(!bCAN_OFF) can1_out(cnt_net_drv,cnt_net_drv,GETTM,bps[cnt_net_drv]._flags_tu,*((char*)(&bps[cnt_net_drv]._vol_u)),*((char*)((&bps[cnt_net_drv]._vol_u))+1),*((char*)(&bps[cnt_net_drv]._vol_i)),*((char*)((&bps[cnt_net_drv]._vol_i))+1));

	if(bps[cnt_net_drv]._cnt<CNT_SRC_MAX)
   	 		{    
   	 		bps[cnt_net_drv]._cnt++;
   	 		if( (bps[cnt_net_drv]._cnt>=CNT_SRC_MAX) && (!net_av) && (!(bps[cnt_net_drv]._av&0x08)) && (cnt_net_drv<NUMIST) ) 
   	 			{
   	 			avar_bps_hndl(cnt_net_drv,3,1);
   	 			}
   	 		}
		else bps[cnt_net_drv]._cnt=CNT_SRC_MAX;
						
		if((bps[cnt_net_drv]._cnt>=3)&&(bps[cnt_net_drv]._cnt_old<3))bps[cnt_net_drv]._cnt_more2++;
		bps[cnt_net_drv]._cnt_old=bps[cnt_net_drv]._cnt;
	} 
*/	
	

else if(cnt_net_drv==19)
	{
     if(!bCAN_OFF)
		{
		mcp2515_transmit(cnt_net_drv,cnt_net_drv,GETTM,0,0,0,0,0);
		lakb[0]._cnt++;
		if(lakb[0]._cnt>20)lakb[0]._cnt=20;
		lakb[1]._cnt++;
		if(lakb[1]._cnt>20)lakb[1]._cnt=20;
		}
     }
	
	
else if((cnt_net_drv>=MINIM_INV_ADRESS)&&(cnt_net_drv<MINIM_INV_ADRESS+15))
	{
	if(!bCAN_OFF)
		{
		if(bCAN_INV)
			{
			
			mcp2515_transmit(cnt_net_drv,cnt_net_drv,GETTM,bps[cnt_net_drv]._flags_tu,*((char*)(&bps[cnt_net_drv]._vol_u)),*((char*)((&bps[cnt_net_drv]._vol_u))+1),*((char*)(&bps[cnt_net_drv]._vol_i)),*((char*)((&bps[cnt_net_drv]._vol_i))+1));
     		}
		else
			{
			
			mcp2515_transmit(cnt_net_drv,cnt_net_drv,GETTM_INV,bps[cnt_net_drv]._flags_tu,*((char*)(&bps[cnt_net_drv]._vol_u)),*((char*)((&bps[cnt_net_drv]._vol_u))+1),*((char*)(&bps[cnt_net_drv]._vol_i)),*((char*)((&bps[cnt_net_drv]._vol_i))+1));
     		} 
		}
	//if(cnt_net_drv<=11)
	     {
	     if(bps[cnt_net_drv]._cnt<CNT_SRC_MAX)
   	 		{    
   	 		bps[cnt_net_drv]._cnt++;
   	 	/*	if( (bps[cnt_net_drv]._cnt>=CNT_SRC_MAX) && (!net_av) && (!(bps[cnt_net_drv]._av&0x08)) && (cnt_net_drv<NUMIST) ) 
   	 			{
   	 			avar_bps_hndl(cnt_net_drv,3,1);
   	 			}*/
   	 		}
		else bps[cnt_net_drv]._cnt=CNT_SRC_MAX;
						
		if((bps[cnt_net_drv]._cnt>=3)&&(bps[cnt_net_drv]._cnt_old<3))bps[cnt_net_drv]._cnt_more2++;
		bps[cnt_net_drv]._cnt_old=bps[cnt_net_drv]._cnt;
	     }
	}


/*

else if((cnt_net_drv==MINIM_INV_ADRESS+2)&&(NUMINV))
	{
    if(!bCAN_OFF) can1_out(cnt_net_drv,cnt_net_drv,0,0,0,0,0,0);
	}*/


}

//-----------------------------------------------
void parol_init(void)
{
parol[0]=0;
parol[1]=0;
parol[2]=0;
sub_ind=0;
}

//-----------------------------------------------
void bitmap_hndl(void)
{
short x,ii,i;
unsigned int ptr_bitmap;
static char ptr_cnt,ptr_cnt1,ptr_cnt2,ptr_cnt3,ptr_cnt4;

for(ii=0;ii<488;ii++)
	{
	lcd_bitmap[ii]=0x00;
	}

mnemo_cnt=10;

if((!mnemo_cnt)&&((NUMBAT==0)||((NUMBAT==1)&&(BAT_IS_ON[0]==bisON))))
	{
	if(avar_stat&0x0001)
		{
		if(bFL2)
			{
			graphic_print(3,3,50,24,50,3,sAVNET,0);
			graphic_print(3,3,50,24,50,3,sAVNET1,0);
			}
		}
	else
		{

		if(NUMIST>=1)
			{
/*

	if(bps[sub_ind1]._state==bsWRK)
		{
		ptr[0]=		"      в работе      ";
		if((bps[sub_ind1]._flags_tm&0x08)&&(bFL2))
		ptr[0]=		"  СИЛЬНЫЙ НАГРЕВ!!! ";	      
		}
 	 else if(bps[sub_ind1]._state==bsRDY)
	 	{
		ptr[0]=		"      в резерве     ";	
		}

 	 else if(bps[sub_ind1]._state==bsBL)
	 	{
		ptr[0]=		" заблокирован извне ";	
		}

	 else if(bps[sub_ind1]._state==bsAPV)
	 	{
		ptr[0]=		"    Работает АПВ    ";
		}
	 
	 else if(bps[sub_ind1]._state==bsAV)
	 	{
		if(bps[sub_ind1]._av&(1<<0))
		ptr[0]=		" Авария - перегрев! ";
		else if(bps[sub_ind1]._av&(1<<1))
		ptr[0]=		"Авария - завыш.Uвых!";
		else if(bps[sub_ind1]._av&(1<<2))	 
		ptr[0]=		"Авария - заниж.Uвых!";
		else if(bps[sub_ind1]._av&(1<<3))
			{
			ptr[0]=	"  Авария - потеряна ";
			ptr[1]=	"      связь!!!      ";
			ptr[2]=	"                    ";
			simax=0;
			}
		}

	 else if(bps[sub_ind1]._state==bsOFF_AV_NET)
	 	{
		ptr[0]=		"      ВЫКЛЮЧЕН      ";
		ptr[1]=		"     Отсутствует    ";
		ptr[2]=		" первичное питание! ";
		simax=0;
		}

	bgnd_par(			"       БПС N&       ",
					ptr[index_set],
					ptr[index_set+1],
					ptr[index_set+2]);

	if(sub_ind-index_set>2)index_set=sub_ind-2;
	else if (sub_ind<index_set)index_set=sub_ind;

	if(sub_ind>=4)	pointer_set(1);*/


			draw_rectangle(0,0,20,20,0,0);
			draw_rectangle(1,1,18,18,0,0);
			if(bps[0]._state!=bsAV)
				{
				graphic_print(3,2,15,15,15,2,sBPS1,0);
				}
			else if(bps[0]._av&(1<<0))
				{
				if(bFL2)graphic_print(3,2,15,15,15,2,sAVT,0);
				}
			else if( (bps[0]._av&(1<<1)) || (bps[0]._av&(1<<2)))
				{
				if(bFL2)graphic_print(2,2,15,15,15,2,sAVU,0);
				}	
			
			if(bps[0]._state==bsWRK)
				{
				draw(9,20,0,11,0);
				draw(9,31,91,0,0);
				draw_ptr(9,19+ptr_cnt1,0,4);
				}				
			}
		if(NUMIST>=2)
			{
			draw_rectangle(23,0,20,20,0,0);
			draw_rectangle(24,1,18,18,0,0);
			if(bps[1]._state!=bsAV)
				{
				graphic_print(25,2,15,15,15,2,sBPS2,0);
				}
			else if(bps[1]._av&(1<<0))
				{
				if(bFL2)graphic_print(25,2,15,15,15,2,sAVT,0);
				}
			else if( (bps[1]._av&(1<<1)) || (bps[1]._av&(1<<2)))
				{
				if(bFL2)graphic_print(25,2,15,15,15,2,sAVU,0);
				}	
			
			if(bps[1]._state==bsWRK)
				{
				draw(32,20,0,11,0);
				draw(32,31,68,0,0);
				draw_ptr(32,19+ptr_cnt1,0,4);
				}				
			}			
		}
	if(NUMBAT)
		{
		draw_rectangle(50,0,35,20,0,0);
		draw_rectangle(53,20,3,2,0,0);
		draw_rectangle(79,20,3,2,0,0);
		if(bat[0]._av&0x01)
			{
			if(bFL2)graphic_print(43,0,50,24,50,3,sAVNET1,0);
			}
		else 
			{
			draw(66,20,0,11,0);
			draw(66,31,34,0,0);
			if(bat[0]._Ib<0)draw_ptr(66,19+ptr_cnt1,0,4);
			else if(bat[0]._Ib>=0)draw_ptr(66,34-ptr_cnt1,2,4);
			
			if(ptr_cnt4<15)
				{
				if(BAT_C_REAL[0]!=0x5555)
					{
					signed short u;
					u=(((signed short)bat[0]._zar/5));
					gran(&u,0,20);
					draw_rectangle(51,0,32,u,1,0);
					//zar_percent=100;
					if(bat[0]._zar<10)
						{
						draw_rectangle(61,5,12,9,1,2);
						graphic_print_text(61,5," %",2,bat[0]._zar,0,1,1);
						}
					else if(bat[0]._zar<100)
						{
						draw_rectangle(58,5,18,9,1,2);
						graphic_print_text(58,5,"  %",3,bat[0]._zar,0,2,1);
						}		
					else 
						{
						draw_rectangle(55,5,24,9,1,2);
						graphic_print_text(55,5,"   %",4,bat[0]._zar,0,3,1);
						}									
					//draw_rectangle(59,3,18,9,1,2);
					//graphic_print_text(53,3,"   %",4,zar_percent,0,3,1);
					}

				}				
			else if(ptr_cnt4<30)
				{
				graphic_print_text(58,5,"   A",4,bat[0]._Ib/10,1,3,1);
				}
			else
				{
				graphic_print_text_text(53,5,"ACCИМ",5,bat[0]._Ib/10,1,3,1);
				}
			//graphic_print_text_text(53,5,"ACCИМ",5,bat[0]._Ib/10,1,3,1);
					
			}

		}	
		

	draw_rectangle(92,4,27,14,0,0);
	draw(92,10,-4,0,0);
	draw(118,10,4,0,0);
	draw(67,31,39,0,0);
	draw(105,31,0,-14,0);	
	draw_ptr(105,34-ptr_cnt3,2,4);
	
	graphic_print_text(70,22,"    B",5,/*ind_reset_cnt*/load_U/10,0,4,1);
	if(load_I<100)graphic_print_text(93,7,"   A",4,load_I,1,3,1);
	else graphic_print_text(90,7,"   A",4,load_I/10,0,3,1);
			
	ptr_cnt++;
	if(ptr_cnt>=3)
		{
		ptr_cnt=0;
		ptr_cnt1++;
		if(ptr_cnt1>=13)
			{
			ptr_cnt1=0;
			}
	
		ptr_cnt2++;
		if(ptr_cnt2>=32)
			{
			ptr_cnt2=0;
			}
				
		ptr_cnt3++;
		if(ptr_cnt3>=15)
			{
			ptr_cnt3=0;
			}

		ptr_cnt4++;
		if(bat[0]._av&0x02)
			{
			if(ptr_cnt4>=45)
				{
				ptr_cnt4=0;
				}
			}
		else
			{
			if(ptr_cnt4>=30)
				{
				ptr_cnt4=0;
				}					
			}
		}			
	}

else
	{
	for(i=0;i<4;i++)
		{
		ptr_bitmap=122*(unsigned)i;
		for(x=(20*i);x<((20*i)+20);x++)
	 		{
			lcd_bitmap[ptr_bitmap++]=caracter[(unsigned)lcd_buffer[x]*6];
			lcd_bitmap[ptr_bitmap++]=caracter[((unsigned)lcd_buffer[x]*6)+1];
			lcd_bitmap[ptr_bitmap++]=caracter[((unsigned)lcd_buffer[x]*6)+2];
			lcd_bitmap[ptr_bitmap++]=caracter[((unsigned)lcd_buffer[x]*6)+3];
			lcd_bitmap[ptr_bitmap++]=caracter[((unsigned)lcd_buffer[x]*6)+4];
			lcd_bitmap[ptr_bitmap++]=caracter[((unsigned)lcd_buffer[x]*6)+5];
			} 
		}
	}	
}

//-----------------------------------------------
void ind_hndl(void)
{
//const char* ptr;
const char* ptrs[50];
const char* sub_ptrs[50];
static char sub_cnt,sub_cnt1;
char i,sub_cnt_max;
char ii_;
static char ii_cnt,cnt_ind_bat;


	   
sub_cnt_max=5;
i=0;
	      
if(spc_stat==spcVZ)
	{
	sub_ptrs[i++]=		" Выравн.заряд  X:0x ";
	sub_cnt_max++;
	}
if(spc_stat==spcKE)
	{
	if(spc_bat==0)		sub_ptrs[i++]=		"Контроль емк. бат №1";
	else if(spc_bat==1)	sub_ptrs[i++]=		"Контроль емк. бат №2";
	sub_cnt_max++;
	}	
if(avar_stat&0x0001)
	{
	sub_ptrs[i++]=		"   Авария сети!!!   ";
	sub_cnt_max++;	
	}


if(avar_stat&0x0002)
	{
	sub_ptrs[i++]=	" Авария батареи №1  ";
	sub_cnt_max++;	
	}

if(avar_stat&0x0004)
	{
	sub_ptrs[i++]=	" Авария батареи №2  ";
	sub_cnt_max++;	
	}

if(ips_bat_av_stat)
	{
	sub_ptrs[i++]=	"  Авария батареи    ";
	sub_cnt_max++;	
	}
if(avar_stat&(1<<(3+0)))
	{
	sub_ptrs[i++]=	"   Авария БПС №1    ";
	sub_cnt_max++;	
	}
if(avar_stat&(1<<(3+1)))
	{
	sub_ptrs[i++]=	"   Авария БПС №2    ";
	sub_cnt_max++;	
	}
if(avar_stat&(1<<(3+2)))
	{
	sub_ptrs[i++]=	"   Авария БПС №3    ";
	sub_cnt_max++;	
	}
if(avar_stat&(1<<(3+3)))
	{
	sub_ptrs[i++]=	"   Авария БПС №4    ";
	sub_cnt_max++;	
	}
if(avar_stat&(1<<(3+4)))
	{
	sub_ptrs[i++]=	"   Авария БПС №5    ";
	sub_cnt_max++;	
	}
if(avar_stat&(1<<(3+5)))
	{
	sub_ptrs[i++]=	"   Авария БПС №6    ";
	sub_cnt_max++;	
	}
if(avar_stat&(1<<(3+6)))
	{
	sub_ptrs[i++]=	"   Авария БПС №7    ";
	sub_cnt_max++;	
	}
if(avar_stat&(1<<(3+7)))
	{
	sub_ptrs[i++]=	"   Авария БПС №8    ";
	sub_cnt_max++;	
	}

#ifdef UKU_KONTUR
if((avar_stat&(1<<(25)))&&(SK_LCD_EN[0]))
	{
	sub_ptrs[i++]=	" Открыта дверь!!    ";
	sub_cnt_max++;	
	}

if((avar_stat&(1<<(26)))&&(SK_LCD_EN[1]))
	{
	sub_ptrs[i++]=	"Сработал датч. дыма ";
	sub_cnt_max++;	
	}

if((avar_stat&(1<<(27)))&&(SK_LCD_EN[2]))
	{
	sub_ptrs[i++]=	"Сработал датч. удара";
	sub_cnt_max++;	
	}
#endif

//#ifdef UKU_GLONASS
if((sk_av_stat[0]==sasON)&&(NUMSK)&&(!SK_LCD_EN[0]))
	{
	sub_ptrs[i++]=	"   Сработал СК№1    ";
	sub_cnt_max++;	
	}
if((sk_av_stat[1]==sasON)&&(NUMSK>1)&&(!SK_LCD_EN[1]))
	{
	sub_ptrs[i++]=	"   Сработал СК№2    ";
	sub_cnt_max++;	
	}
if((sk_av_stat[2]==sasON)&&(NUMSK>2)&&(!SK_LCD_EN[2]))
	{
	sub_ptrs[i++]=	"   Сработал СК№3    ";
	sub_cnt_max++;	
	}
if((sk_av_stat[3]==sasON)&&(NUMSK>3)&&(!SK_LCD_EN[3]))
	{
	sub_ptrs[i++]=	"   Сработал СК№4    ";
	sub_cnt_max++;	
	}
//#endif
/*if((avar_stat&(1<<(28)))&&(SK_LCD_EN[3]))
	{
	sub_ptrs[i++]=	"   Авария СК №4     ";
	sub_cnt_max++;	
	} */
cnt_of_slave=NUMIST+NUMINV;


//cnt_of_wrks=0;
//for(i=0;i<NUMIST;i++)
 //    {
//     if(bps[i]._state==bsWRK)cnt_of_wrks++;
  //   }


sub_cnt1++;	
if(sub_cnt1>=20)
	{
	sub_cnt1=0;
	sub_cnt++;
	if(sub_cnt>=sub_cnt_max)
		{
		sub_cnt=0;
		}
	}


if(ind==iMn_6U)
	{
	ptrs[0]	=	"                    ";
	if(num_of_wrks_bps)	
		{
		ptrs[0]	=	"  В работе    rист. ";
		}
	else if((NUMBAT>1)&&(!bat[0]._av)&&(!bat[1]._av))
		{
		ptrs[0]	=	" Работа от батарей  ";
		}
	else if((BAT_IS_ON[0]==bisON)&&(!bat[0]._av)/*&&(!(()&&()))*/)
		{
		ptrs[0]	=	"Работа от батареи №1";
		}
	else if((BAT_IS_ON[1]==bisON)&&(!bat[1]._av))
		{
		ptrs[0]	=	"Работа от батареи №2";
		}

	 
     i=0;
 	
 	ptrs[1]="Uбz=   ]В Iбz=    @А";
     ptrs[2]="Uн=    #В Iн=     $А";
     ptrs[3]=" 0%:0^:0& 0</>  /0{ ";
     
 	ptrs[4]=										" Батарея N1         ";
     ptrs[5]=										" Батарея N2         ";
     if((NUMBAT==1)&&(BAT_IS_ON[0]!=bisON))ptrs[4]=		" Батарея N2         ";
								
	ptrs[4+NUMBAT]=  								" БПС N1             ";
     ptrs[5+NUMBAT]=  								" БПС N2             ";
     ptrs[6+NUMBAT]=  								" БПС N3             ";
     ptrs[7+NUMBAT]=  								" БПС N4             ";
     ptrs[8+NUMBAT]= 								" БПС N5             ";
     ptrs[9+NUMBAT]= 								" БПС N6             ";
     ptrs[10+NUMBAT]= 								" БПС N7             ";
     ptrs[11+NUMBAT]= 								" БПС N8             ";
     ptrs[12+NUMBAT]= 								" БПС N9             ";
     ptrs[13+NUMBAT]= 								" БПС N10            ";
     ptrs[14+NUMBAT]= 								" БПС N11            ";
     ptrs[15+NUMBAT]= 								" БПС N12            ";

	ptrs[4+NUMIST+NUMBAT]=  							" Байпас            ";     

     ptrs[4+NUMIST+NUMBAT+NUMBYPASS]=  					" Инвертор N1        ";
     ptrs[5+NUMIST+NUMBAT+NUMBYPASS]=  					" Инвертор N2        ";
     ptrs[6+NUMIST+NUMBAT+NUMBYPASS]=  					" Инвертор N3        ";
     ptrs[7+NUMIST+NUMBAT+NUMBYPASS]=  					" Инвертор N4        ";
     ptrs[8+NUMIST+NUMBAT+NUMBYPASS]=  					" Инвертор N5        ";
     ptrs[9+NUMIST+NUMBAT+NUMBYPASS]=  					" Инвертор N6        ";
     ptrs[10+NUMIST+NUMBAT+NUMBYPASS]=  				" Инвертор N7        ";
     ptrs[11+NUMIST+NUMBAT+NUMBYPASS]=  				" Инвертор N8        ";
     ptrs[12+NUMIST+NUMBAT+NUMBYPASS]=  				" Инвертор N9        ";
     ptrs[13+NUMIST+NUMBAT+NUMBYPASS]=  				" Инвертор N10       ";
     ptrs[14+NUMIST+NUMBAT+NUMBYPASS]=  				" Инвертор N11       ";
     ptrs[15+NUMIST+NUMBAT+NUMBYPASS]=  				" Инвертор N12       ";
     ptrs[16+NUMIST+NUMBAT+NUMBYPASS]=  				" Инвертор N13       ";
     ptrs[17+NUMIST+NUMBAT+NUMBYPASS]=  				" Инвертор N14       ";
     ptrs[18+NUMIST+NUMBAT+NUMBYPASS]=  				" Инвертор N15       ";

	ptrs[4+NUMIST+NUMBAT+NUMBYPASS+NUMINV]= 			" Таблица инверторов ";

     ptrs[4+NUMIST+NUMBAT+NUMINV+NUMBYPASS+(NUMINV!=0)]=				" Монитор АКБ N1     ";
     ptrs[5+NUMIST+NUMBAT+NUMINV+NUMBYPASS+(NUMINV!=0)]=				" Монитор АКБ N2     ";
     ptrs[6+NUMIST+NUMBAT+NUMINV+NUMBYPASS+(NUMINV!=0)]=				" Монитор АКБ N3     ";
     ptrs[7+NUMIST+NUMBAT+NUMINV+NUMBYPASS+(NUMINV!=0)]=				" Монитор АКБ N4     ";


     ptrs[4+NUMIST+NUMBAT+NUMINV+NUMMAKB+NUMBYPASS+(NUMINV!=0)]= 				" Сеть               "; 
     ptrs[5+NUMIST+NUMBAT+NUMINV+NUMMAKB+NUMBYPASS+(NUMINV!=0)]= 				" Нагрузка           "; 
     ptrs[6+NUMIST+NUMBAT+NUMINV+NUMMAKB+NUMBYPASS+(NUMINV!=0)]= 				" Внешние датчики    "; 
 	ptrs[6+NUMIST+NUMBAT+NUMINV+NUMMAKB+NUMBYPASS+(NUMEXT!=0)+(NUMINV!=0)]= 	" Спецфункции    	 ";
     ptrs[7+NUMIST+NUMBAT+NUMINV+NUMMAKB+NUMBYPASS+(NUMEXT!=0)+(NUMINV!=0)]= 	" Установки          "; 
     ptrs[8+NUMIST+NUMBAT+NUMINV+NUMMAKB+NUMBYPASS+(NUMEXT!=0)+(NUMINV!=0)]= 	" Журнал событий     "; 
     ptrs[9+NUMIST+NUMBAT+NUMINV+NUMMAKB+NUMBYPASS+(NUMEXT!=0)+(NUMINV!=0)]= 	" Выход              "; 
     ptrs[10+NUMIST+NUMBAT+NUMINV+NUMMAKB+NUMBYPASS+(NUMEXT!=0)+(NUMINV!=0)]= 	" Журнал батареи N1  "; 
     ptrs[11+NUMIST+NUMBAT+NUMINV+NUMMAKB+NUMBYPASS+(NUMEXT!=0)+(NUMINV!=0)]= 	" Журнал батареи N2  "; 
	ptrs[12+NUMIST+NUMBAT+NUMINV+NUMMAKB+NUMBYPASS+(NUMEXT!=0)+(NUMINV!=0)]=	" Тест               ";

     if(sub_ind==0)index_set=0;
	else if((index_set-sub_ind)>2)index_set=sub_ind+2;
	else if(sub_ind>index_set)index_set=sub_ind;
	
	if(sub_cnt<5)bgnd_par(ptrs[0],ptrs[index_set+1],ptrs[index_set+2],ptrs[index_set+3]);
	else bgnd_par(sub_ptrs[sub_cnt-5],ptrs[index_set+1],ptrs[index_set+2],ptrs[index_set+3]);
	
	if((ii_cnt<=80)||((spc_stat!=spcVZ)&&(spc_stat!=spcKE)/*&&(spc_stat!=spcKE2p1)*/)) 
		{
	if((ii_!=139)&&(/*(src_state[0]==ssWRK)||(src_state[1]==ssWRK)||(src_state[2]==ssWRK))*/num_of_wrks_bps!=0))
		{
		
		if(sub_cnt<5)int2lcdyx(num_of_wrks_bps,0,14,0);

          }
     }

	if((!NUMBAT)&&(!NUMINV)) {
		int2lcd(byps._Uout,'#',1);
     	int2lcd(byps._Iout,'$',1); 
	} else {
		int2lcd(load_U,'#',1);
 		int2lcd(load_I,'$',1);
	}
 	
	int2lcd(LPC_RTC->HOUR,'%',0);
	int2lcd(LPC_RTC->MIN,'^',0);
	int2lcd(LPC_RTC->SEC,'&',0);
	int2lcd(LPC_RTC->DOM,'<',0);
	int2lcd(LPC_RTC->YEAR,'{',0); 
	sub_bgnd(sm_mont[LPC_RTC->MONTH],'>',0);

 	if(bFL2)
 	     {
 	     lcd_buffer[find(':')]=' ';
 	     lcd_buffer[find(':')]=' ';
 	     lcd_buffer[find(':')]=' ';
 	     }

	if((index_set)&&(sub_ind))
	     {
	     if(index_set==sub_ind)lcd_buffer[60]=1;
	     else if((index_set-sub_ind)==1)lcd_buffer[40]=1;
	     else if((index_set-sub_ind)==2)lcd_buffer[20]=1;
	     }	
		
	cnt_ind_bat++;
	if(cnt_ind_bat>=(NUMBAT*20)) cnt_ind_bat=0;
	
	if((AUSW_MAIN==2400)||(AUSW_MAIN==4800)||(AUSW_MAIN==6000)||
		(AUSW_MAIN==2403)||(AUSW_MAIN==4803)||(AUSW_MAIN==6003))sub_bgnd("                    ",'z',-2);
	else if(NUMBAT==0)sub_bgnd(" Работа без батарей ",'z',-2);
	else
		{
		int2lcd((cnt_ind_bat/20)+1+(!(BAT_IS_ON[0]==bisON)),'z',0);
		int2lcd((cnt_ind_bat/20)+1+(!(BAT_IS_ON[0]==bisON)),'z',0);
		if((NUMBAT==1)&&(!(BAT_IS_ON[0]==bisON)))
			{
			int2lcd(bat[1]._Ub,']',1);
			if((mess_find_unvol(MESS2IND_HNDL))&&(mess_data[0]==PARAM_SAMOKALIBR)) sub_bgnd("КЛБР. ",'@',-4);
			else 
				{
				if((AUSW_MAIN==24120)||(AUSW_MAIN==24210)||(AUSW_MAIN==24123)||(AUSW_MAIN==48140)) int2lcd_mmm(bat[1]._Ib/10,'@',1);
				else int2lcd_mmm(bat[1]._Ib,'@',2);
				}
			}
		else
			{
			int2lcd(bat[cnt_ind_bat/20]._Ub,']',1);
			if((mess_find_unvol(MESS2IND_HNDL))&&(mess_data[0]==PARAM_SAMOKALIBR)) sub_bgnd("КЛБР. ",'@',-4);
			else 
				{
				if((AUSW_MAIN==24120)||(AUSW_MAIN==24210)||(AUSW_MAIN==24123)||(AUSW_MAIN==48140)) int2lcd_mmm(bat[cnt_ind_bat/20]._Ib/10,'@',1);
				else 
					{
					if((bat[cnt_ind_bat/20]._Ib<=9999)&&(bat[cnt_ind_bat/20]._Ib>=-9999))int2lcd_mmm(bat[cnt_ind_bat/20]._Ib,'@',2);
					else int2lcd_mmm(bat[cnt_ind_bat/20]._Ib/10,'@',1);
					}
			 	}
			}		
		}

	int2lcd(vz_cnt_s_/60,'x',0);
	int2lcd(vz_cnt_h_,'X',0); 

	//int2lcdyx(plazma_bat_drv0,0,4,0);
	//int2lcdyx(plazma_bat_drv1,0,10,0);
	//int2lcdyx(bat[0]._min_cell_volt,0,15,0); 
	//int2lcdyx(bat_drv_rx_cnt,0,19,0);
//	int2lcdyx(bps[i+20]._buff[14],0,19,0);
/*	int2lcdyx(makb[2]._cnt,0,10,0);*/
	//int2lcdyx(first_inv_slot,0,19,0);	
 	//int2lcdyx(npn_tz_cnt,0,7,0);
	//char2lcdbyx(GET_REG(LPC_GPIO0->FIOPIN,4,8),0,19);
	//int2lcdyx(lc640_read_int(ADR_EE_BAT_IS_ON[0]),0,4,0);
	//int2lcdyx(lc640_read_int(ADR_EE_BAT_IS_ON[1]),0,9,0);
	//int2lcdyx(BAT_IS_ON[0],0,14,0);
	int2lcdyx(mcp2515_can_st,0,9,0);
	int2lcdyx(mcp2515_buff_wr_ptr,0,3,0);
	int2lcdyx(mcp2515_buff_rd_ptr,0,6,0);

	int2lcdyx(can_plazma,0,13,0);
	}

	 
			
if(ind==iDeb)
     {
     if(sub_ind==0)
     	{


         	bgnd_par("*0000*000000*       ",
     	         "                    ",
     	         "                    ",
     	         "      ********      ");

	int2lcdyx(SOFT_NUM,0,4,0);
	long2lcdyx_mmm(SOFT_DATE,0,11,0);

/*	int2lcdyx(a_ind.i,0,2,0);
	int2lcdyx(a_ind.s_i,0,6,0);
	int2lcdyx(a_ind.s_i1,0,10,0);
	int2lcdyx(a_ind.s_i2,0,14,0);
	int2lcdyx(a_ind.i_s,0,16,0);




	int2lcdyx(c_ind.i,1,2,0);
	int2lcdyx(c_ind.s_i,1,6,0);
	int2lcdyx(c_ind.s_i1,1,10,0);
	int2lcdyx(c_ind.s_i2,1,14,0);
	int2lcdyx(c_ind.i_s,1,16,0);*/


	int2lcdyx(t_box,2,4,0);
	int2lcdyx(t_ext_can,3,5,0);
	//int2lcdyx(t_ext_can_nd,3,6,0);
	int2lcdyx(t_ext[1],2,10,0);
	int2lcdyx(t_ext[2],3,10,0);

	int2lcdyx(cntrl_stat,0,19,0);

	//int2lcdyx(adc_buff_[14],3,14,0);
	int2lcdyx(net_metr_buff_[0],1,5,0);
	int2lcdyx(net_metr_buff_[1],1,11,0);
	int2lcdyx(net_metr_buff_[2],1,17,0);

	/*int2lcdyx(adc_buff_[12],2,14,0);
	
	int2lcdyx(adc_buff_[5],2,19,0);
	int2lcdyx(adc_buff_[6],3,19,0);*/

	//int2lcdyx(tloaddisable_cmnd,2,14,0);
	//int2lcdyx(tloaddisable_stat,3,14,0);

/*		int2lcdyx(load_U,0,15,0);
		int2lcdyx(u_necc,0,19,0);
		
		

		int2lcdyx(sub_ind1+1,1,0,0);
		int2lcdyx(sub_ind1+2,2,0,0);
		int2lcdyx(sub_ind1+3,3,0,0);
		
		
		int2lcdyx(bps[sub_ind1  ]._cnt,1,2,0);
		int2lcdyx(bps[sub_ind1+1]._cnt,2,2,0);
		int2lcdyx(bps[sub_ind1+2]._cnt,3,2,0);*/		
		
	/*	int2lcdyx(bps[sub_ind1  ]._ist_blok_cnt,1,5,0);
		int2lcdyx(bps[sub_ind1+1]._ist_blok_cnt,2,5,0);
		int2lcdyx(bps[sub_ind1+2]._ist_blok_cnt,3,5,0);*/			
		
	/*	char2lcdhyx(bps[sub_ind1  ]._flags_tu,1,8);
		char2lcdhyx(bps[sub_ind1+1]._flags_tu,2,8);
		char2lcdhyx(bps[sub_ind1+2]._flags_tu,3,8);

		int2lcdyx(bps[sub_ind1  ]._vol_u,1,12,0);
		int2lcdyx(bps[sub_ind1+1]._vol_u,2,12,0);
		int2lcdyx(bps[sub_ind1+2]._vol_u,3,12,0);		


		char2lcdhyx(bps[sub_ind1]._flags_tm,1,15);
		char2lcdhyx(bps[sub_ind1+1]._flags_tm,2,15);
		char2lcdhyx(bps[sub_ind1+2]._flags_tm,3,15);	

		char2lcdhyx(bps[sub_ind1]._Ii,1,19);
		char2lcdhyx(bps[sub_ind1+1]._Ii,2,19);
		char2lcdhyx(bps[sub_ind1+2]._Ii,3,19);*/
	/*
		char2lcdhyx(bps[sub_ind1]._rotor>>8,1,15);
		char2lcdhyx(bps[sub_ind1+1]._rotor>>8,2,15);
		char2lcdhyx(bps[sub_ind1+2]._rotor>>8,3,15);		
		
		char2lcdhyx((char)bps[sub_ind1]._rotor,1,17);
		char2lcdhyx((char)bps[sub_ind1+1]._rotor,2,17);
		char2lcdhyx((char)bps[sub_ind1+2]._rotor,3,17);*/



     	
 /*    	bgnd_par("                    ",
     	         "                    ",
     	         "                    ",
     	         "%                   ");


		int2lcdyx(main_kb_cnt,0,3,0);
		int2lcdyx(cntrl_stat,1,3,0);
   		
		
		int2lcdyx(u_necc_up,0,7,0);
		int2lcdyx(u_necc,1,7,0);
		int2lcdyx(u_necc_dn,2,7,0);
		int2lcdyx(bat[0]._Ub,3,7,0);

		int2lcdyx(sign_U,0,10,0);
		int2lcdyx(sign_I,1,10,0);
		int2lcdyx(superviser_cnt,2,10,0);	


		int2lcdyx(bat[0]._zar,0,19,0);
		int2lcdyx(BAT_C_REAL[0],1,19,0);
		int2lcdyx(BAT_C_NOM[0],2,19,0);
		int2lcdyx(lc640_read_int(EE_BAT1_ZAR_CNT),3,19,0);  */

		  //bat_ver_cnt



	/*	int2lcdyx(tlv_buff[1][1],0,9,0);
		int2lcdyx(tlv_buff[1][2],0,14,0);
		int2lcdyx(tlv_buff[1][3],0,19,0);

   		int2lcdyx(tlv_buff[1][4],1,4,0);
		int2lcdyx(tlv_buff[1][5],1,9,0);
		int2lcdyx(tlv_buff[1][6],1,14,0);
		int2lcdyx(tlv_buff[1][7],1,19,0);

   		int2lcdyx(tlv_buff[1][8],2,4,0);
		int2lcdyx(tlv_buff[1][9],2,9,0);
		int2lcdyx(tlv_buff[1][10],2,14,0);
		int2lcdyx(tlv_buff[1][11],2,19,0);

   		int2lcdyx(tlv_buff[1][12],3,4,0);
		int2lcdyx(tlv_buff[1][13],3,9,0);
		int2lcdyx(tlv_buff[1][14],3,14,0);
		int2lcdyx(tlv_buff[1][15],3,19,0);	*/
      

     	}     

    	else if(sub_ind==1) 
     	{
     	bgnd_par("Б                   ",
     	         "                    ",
     	         "                    ",
     	         "                    ");

		//int2lcdyx(bAVG,0,0,0);
		//int2lcdyx(LPC_CAN1->GSR,0,6,0);
		//int2lcdyx((LPC_CAN1->GSR)>>16,0,16,0);
		//int2lcdyx(avg,0,19,0);

				int2lcdyx((((LPC_CAN1->GSR)&(0xff000000))>>24),0,19,0);
		int2lcdyx((((LPC_CAN1->GSR)&(0x00ff0000))>>16),0,15,0);

 /*         int2lcdyx(bat[0]._Ubm,1,7,0); 	int2lcdyx(bat[0]._av,1,10,0);
		int2lcdyx(bat[0]._dUbm,2,7,0);
		int2lcdyx(bat[0]._cnt_as,3,7,0);
		
 
		int2lcdyx(bat[1]._Ub,0,14,0);
          int2lcdyx(bat[1]._Ubm,1,14,0);	int2lcdyx(bat[1]._av,1,17,0);
		int2lcdyx(bat[1]._dUbm,2,14,0);
		int2lcdyx(bat[1]._cnt_as,3,14,0);*/

		int2lcdyx(sub_ind1+0,1,0,0);
		int2lcdyx(sub_ind1+1,2,0,0);
		int2lcdyx(sub_ind1+2,3,0,0);
		
		
		int2lcdyx(bps[sub_ind1  ]._cnt,1,2,0);
		int2lcdyx(bps[sub_ind1+1]._cnt,2,2,0);
		int2lcdyx(bps[sub_ind1+2]._cnt,3,2,0);		
		
	/*	int2lcdyx(bps[sub_ind1  ]._ist_blok_cnt,1,5,0);
		int2lcdyx(bps[sub_ind1+1]._ist_blok_cnt,2,5,0);
		int2lcdyx(bps[sub_ind1+2]._ist_blok_cnt,3,5,0);*/			
		
	/*	char2lcdhyx(bps[sub_ind1  ]._flags_tu,1,8);
		char2lcdhyx(bps[sub_ind1+1]._flags_tu,2,8);
		char2lcdhyx(bps[sub_ind1+2]._flags_tu,3,8);

		int2lcdyx(bps[sub_ind1  ]._vol_u,1,12,0);
		int2lcdyx(bps[sub_ind1+1]._vol_u,2,12,0);
		int2lcdyx(bps[sub_ind1+2]._vol_u,3,12,0);		


		char2lcdhyx(bps[sub_ind1]._flags_tm,1,15);
		char2lcdhyx(bps[sub_ind1+1]._flags_tm,2,15);
		char2lcdhyx(bps[sub_ind1+2]._flags_tm,3,15);	
		*/
		int2lcdyx(bps[sub_ind1]._Ii,1,15,0);
		int2lcdyx(bps[sub_ind1+1]._Ii,2,15,0);
		int2lcdyx(bps[sub_ind1+2]._Ii,3,15,0);
	/*
		char2lcdhyx(bps[sub_ind1]._rotor>>8,1,15);
		char2lcdhyx(bps[sub_ind1+1]._rotor>>8,2,15);
		char2lcdhyx(bps[sub_ind1+2]._rotor>>8,3,15);		
		*/
		
		int2lcdyx(bps[sub_ind1]._rotor,1,19,0);
		int2lcdyx(bps[sub_ind1+1]._rotor,2,19,0);
		int2lcdyx(bps[sub_ind1+2]._rotor,3,19,0);


 		}

 

    else if(sub_ind==2)
     	{
     	bgnd_par(	"КБ                  ",
     		    	"                    ",
     		    	"                    ",
     		    	"                    ");

		int2lcdyx(main_kb_cnt,1,4,0);
		int2lcdyx(TBAT,2,4,0);
		int2lcdyx(cntrl_stat,3,4,0); 

		int2lcdyx(kb_start[0],1,7,0);
		int2lcdyx(kb_start[1],2,7,0);
		int2lcdyx(kb_start_ips,3,7,0); 

		int2lcdyx(kb_cnt_1lev,1,10,0);
		int2lcdyx(kb_cnt_2lev,2,10,0);
		int2lcdyx(kb_full_ver,3,10,0);


		int2lcdyx(ips_bat_av_vzvod,0,10,0);
		int2lcdyx(ips_bat_av_stat,0,7,0);

/*		int2lcdhyx(avar_ind_stat,0,7);
		char2lcdbyx(rele_stat,1,7);
		
		int2lcdyx(sk_av_stat[0],0,19,0);
		int2lcdyx(sk_av_stat[1],1,19,0);
		int2lcdyx(sk_av_stat[2],2,19,0);

		int2lcdyx(SK_REL_EN[0]&0x000f,0,16,0);
		int2lcdyx(SK_REL_EN[1]&0x000f,1,16,0);
		int2lcdyx(SK_REL_EN[2]&0x000f,2,16,0);

		int2lcdyx(SK_SIGN[0]&0x000f,0,10,0);
		int2lcdyx(SK_SIGN[1]&0x000f,1,10,0);
		int2lcdyx(SK_SIGN[2]&0x000f,2,10,0);

		int2lcdyx(SK_LCD_EN[0]&0x000f,0,13,0);
		int2lcdyx(SK_LCD_EN[1]&0x000f,1,13,0);
		int2lcdyx(SK_LCD_EN[2]&0x000f,2,13,0);
		//int2lcdyx(adc_buff_ext_[1],1,19,0);
		//int2lcdyx(plazma_suz[2],2,4,0);
		//int2lcdyx(plazma_suz[3],3,4,0); */
		}  

	else if(sub_ind==3)
     	{
     	bgnd_par("КЕ                  ",
     	         "                    ",
     	         "                   ^",
     	         "                   &");

	int2lcdyx(spc_stat,0,5,0);
	int2lcdyx(__ee_spc_stat,0,9,0);
	int2lcdyx(lc640_read_int(EE_SPC_STAT),0,13,0);

	int2lcdyx(spc_bat,1,5,0);
	int2lcdyx(__ee_spc_bat,1,9,0);
	int2lcdyx(lc640_read_int(EE_SPC_BAT),1,13,0);

	int2lcdyx(bat_u_old_cnt,0,19,0);
	
	
	int2lcdyx(bat[0]._zar_cnt_ke,2,5,0);
	int2lcdyx(lc640_read_int(ADR_EE_BAT_ZAR_CNT_KE[0]),2,10,0);	
	int2lcdyx(bat[0]._u_old[0],2,14,0);
	int2lcd_mmm(bat[0]._Ib,'^',2);

	int2lcdyx(bat[1]._zar_cnt_ke,3,5,0);
	int2lcdyx(lc640_read_int(ADR_EE_BAT_ZAR_CNT_KE[1]),3,10,0);
	int2lcdyx(bat[1]._Ub,3,14,0);
	int2lcd_mmm(bat[1]._Ib,'&',2);	

	int2lcdyx(spc_phase,1,15,0);
	int2lcdyx(__ee_spc_phase,1,17,0);
	int2lcdyx(lc640_read_int(EE_SPC_PHASE),1,19,0);
	
		

/*	    		int2lcdyx(adc_net_buff_cnt,0,4,0);

		    	int2lcdyx((short)(main_power_buffer[0]>>12),0,19,0);
			int2lcdyx((short)(main_power_buffer[1]>>12),1,19,0);
			int2lcdyx((short)(main_power_buffer[2]>>12),2,19,0);
			int2lcdyx((short)(main_power_buffer[3]>>12),3,19,0);

		    	int2lcdyx((net_buff_),2,5,0); */


		   
		    


/*		int2lcdyx(load_U,0,4,0);
		int2lcdyx(load_I,1,4,0);
		lcd_buffer[44]='a';
		int2lcd_mmm((bat[0]._Ib)/10,'a',1);
		lcd_buffer[64]='a';
		int2lcd_mmm((bat[1]._Ib)/10,'a',1);

 		int2lcdyx(u_necc,0,8,0);

		
		
		lcd_buffer[14]='.';
		lcd_buffer[34]='.';
		int2lcdyx(Isumm,0,15,1);		
		int2lcdyx(Isumm_,1,15,1);


		int2lcdyx(cntrl_stat,0,19,0);
		int2lcdyx(num_necc,1,19,0);
		
		
		  
//		int2lcdyx(cntrl_stat,0,15,0);
		 
		//int2lcdyx(cntrl_plazma,1,3,0);
		//lcd_buffer[30]='a';
		int2lcd_mmm(Ibmax,'a',0);
		int2lcdyx(IZMAX,1,14,0);

		lcd_buffer[65]='a';
		int2lcd_mmm(bat[0]._Ib,'a',0);

		lcd_buffer[70]='a';
		int2lcd_mmm(bat[1]._Ib,'a',0); 

		lcd_buffer[75]='a';
		int2lcd_mmm(Ibmax,'a',0); 

	//	int2lcdyx(IMAX,2,3,0);
		
		

	//	int2lcdyx(IZMAX,3,19,0);

		//int2lcdyx(num_necc_Imax,3,6,0);
		//int2lcdyx(num_necc_Imin,3,12,0);


 //    	lcd_buffer[4]='a';            
 //    	int2lcd_mmm(Ibat,'a',1);   int2lcdyx(cntrl_stat,0,9,0);          int2lcdyx(hour_apv_cnt,0,13,0);                             char2lcdhyx(St_[0],0,19);  
 //    	int2lcdyx(Ubat,1,4,0);     int2lcdyx(main_apv_cnt,1,9,0);        int2lcdyx(lc640_read_int(bps1_AVAR_PTR),1,13,0);            char2lcdhyx(St_[1],1,19);
 //    	int2lcdyx(Us[0],2,4,0);  int2lcdyx(apv_cnt_1,2,9,0);           int2lcdyx(lc640_read_int(SRC1_AVAR_CNT),2,13,0);                                     int2lcdhyx(av_stat,2,19);
 //    	int2lcdyx(Us[1],3,4,0);  int2lcdyx(reset_apv_cnt,3,9,0);                                            int2lcdyx(plazma,3,19,0);
     	//int2lcd(plazma,'(',0);

     	//int2lcd(Us[0],'#',1);
     	//int2lcd(Us[1],'$',1);
     	//int2lcd(Is[0],'%',1);
     	//int2lcd(Is[1],'^',1);
    // 	int2lcd(bat[0]._Ub,'<',1);
    // 	int2lcd_mmm(bat[0]._Ib,'>',2);
 //    	char2lcdhyx(St_[0],3,13);
 //    	char2lcdhyx(St_[1],3,19);
 //    	char2lcdhyx(St,3,5);  */
		}

	else if(sub_ind==4)
     	{
     	bgnd_par(" АВАРИИ             ",
     	         "                    ",
     	         "                    ",
     	         "                    ");

		int2lcdyx(main_10Hz_cnt,0,7,0);
		int2lcdyx(bat[0]._av,0,10,0);
		int2lcdyx(bat[1]._av,0,12,0);
		char2lcdhyx(rele_stat,0,19);

 		long2lcdhyx(avar_stat,1,7);
		long2lcdhyx(avar_stat_old,2,7);
		long2lcdhyx(avar_ind_stat,3,7);

		long2lcdhyx(avar_stat_new,2,19);
		long2lcdhyx(avar_stat_offed,3,19);



	//	int2lcdyx(bat[0]._Ub,1,15,0);
	//	int2lcdyx(bat[1]._rel_stat,2,15,0);


/*		int2lcdyx(mat_temper,0,7,0);
		int2lcdyx(load_U,0,11,0);  
		int2lcdyx(cntrl_stat,0,15,0);
		int2lcdyx(cntrl_stat_old,0,19,0); 
		int2lcdyx(cntrl_plazma,1,3,0);
		lcd_buffer[30]='a';
		int2lcd_mmm(Ibmax,'a',0);
		int2lcdyx(IZMAX,1,14,0);*/

		}
   else if(sub_ind==5)
     	{
     	bgnd_par("**                  ",
     		    "                    ",
     		    "                    ",
     		    "                    ");
     		    


     	
     	int2lcdyx(vz_cnt_s_,0,6,0);
		int2lcdyx(vz_cnt_s,1,6,0);
		int2lcdyx(vz_cnt_h,2,6,0);
		int2lcdyx(vz_cnt_h_,3,6,0);

     	int2lcdyx(__ee_vz_cnt,3,12,0);
		int2lcdyx(lc640_read_int(EE_VZ_CNT),1,10,0);
	/*		int2lcdyx(eb2_data[10],2,10,0);
/*		int2lcdyx(eb2_data[11],3,10,0);

     	int2lcdyx(eb2_data[12],0,14,0);
		int2lcdyx(eb2_data[13],1,14,0);
		int2lcdyx(eb2_data[14],2,14,0);
		int2lcdyx(eb2_data[15],3,14,0);
     	
     	int2lcdyx(eb2_data[16],0,18,0);
		int2lcdyx(eb2_data[17],1,18,0);
		int2lcdyx(eb2_data[18],2,18,0);
		int2lcdyx(eb2_data[19],3,18,0);*/

	/*	int2lcdyx(eb2_data_short[0],0,13,0);
		int2lcdyx(eb2_data_short[1],1,13,0);
		int2lcdyx(eb2_data_short[2],2,13,0);

		int2lcdyx(eb2_data_short[3],0,19,0);
		int2lcdyx(eb2_data_short[4],1,19,0);
		int2lcdyx(eb2_data_short[5],2,19,0);  */

     	/*int2lcdyx(eb2_data[20],0,10,0);
		int2lcdyx(eb2_data[21],1,10,0);
		int2lcdyx(eb2_data[22],2,10,0);
		int2lcdyx(eb2_data[23],3,10,0);*/

    		}
    else if(sub_ind==6)
     	{
     	bgnd_par("                    ",
     		    "                    ",
     		    "                    ",
     		    "                    ");
     		    
     	/*int2lcdyx(ad7705_buff[0][0],0,4,0);
     	int2lcdyx(ad7705_buff[0][1],0,9,0);
     	int2lcdyx(ad7705_buff[0][2],0,14,0);
     	int2lcdyx(ad7705_buff[0][3],0,19,0);
     	int2lcdyx(ad7705_buff[0][4],1,4,0);
     	int2lcdyx(ad7705_buff[0][5],1,9,0);
     	int2lcdyx(ad7705_buff[0][6],1,14,0);
     	int2lcdyx(ad7705_buff[0][7],1,19,0);
     	int2lcdyx(ad7705_buff[0][8],2,4,0);
     	int2lcdyx(ad7705_buff[0][9],2,9,0);
     	int2lcdyx(ad7705_buff[0][10],2,14,0);
     	int2lcdyx(ad7705_buff[0][11],2,19,0);
     	int2lcdyx(ad7705_buff[0][12],3,4,0);
     	int2lcdyx(ad7705_buff[0][13],3,9,0);
     	int2lcdyx(ad7705_buff[0][14],3,14,0);
     	int2lcdyx(ad7705_buff[0][15],3,19,0);*/

	/*	int2lcdyx(adc_buff_[0],0,4,0);
    		int2lcdyx(adc_buff_[1],0,9,0);
     	int2lcdyx(adc_buff_[2],0,14,0);
     	int2lcdyx(adc_buff_[3],0,19,0); 
     	int2lcdyx(adc_buff_[4],1,4,0);	
     	int2lcdyx(adc_buff_[5],1,9,0);
     	int2lcdyx(adc_buff_[6],1,14,0);
     	int2lcdyx(adc_buff_[7],1,19,0); 
     	int2lcdyx(adc_buff_[8],2,4,0);
     	int2lcdyx(adc_buff_[9],2,9,0);
     	int2lcdyx(adc_buff_[10],2,14,0);
     	int2lcdyx(adc_buff_[11],2,19,0);
     	int2lcdyx(adc_buff_[12],3,4,0);
     	int2lcdyx(adc_buff_[13],3,9,0);
     	int2lcdyx(adc_buff_[14],3,14,0);
     	int2lcdyx(adc_buff_[15],3,19,0);*/

		char2lcdhyx(bat_drv_rx_buff[0],0,2);
    		char2lcdhyx(bat_drv_rx_buff[1],0,5);
     	char2lcdhyx(bat_drv_rx_buff[2],0,8);
     	char2lcdhyx(bat_drv_rx_buff[3],0,11); 
     	char2lcdhyx(bat_drv_rx_buff[4],0,14);	
     	char2lcdhyx(bat_drv_rx_buff[5],0,17);
		char2lcdhyx(bat_drv_rx_buff[6],1,2);
    		char2lcdhyx(bat_drv_rx_buff[7],1,5);
     	char2lcdhyx(bat_drv_rx_buff[8],1,8);
     	char2lcdhyx(bat_drv_rx_buff[9],1,11); 
     	char2lcdhyx(bat_drv_rx_buff[10],1,14);	
     	char2lcdhyx(bat_drv_rx_buff[11],1,17);
		char2lcdhyx(bat_drv_rx_buff[12],2,2);
    		char2lcdhyx(bat_drv_rx_buff[13],2,5);
     	char2lcdhyx(bat_drv_rx_buff[14],2,8);
     	char2lcdhyx(bat_drv_rx_buff[15],2,11); 
     	char2lcdhyx(bat_drv_rx_buff[16],2,14);	
     	char2lcdhyx(bat_drv_rx_buff[17],2,17);
     	/*int2lcdhyx(bat_drv_rx_buff[12],3,4);
     	int2lcdhyx(bat_drv_rx_buff[13],3,9);
     	int2lcdhyx(bat_drv_rx_buff[14],3,14);*/
     	
		int2lcdyx(bat_drv_rx_cnt,3,15,0);
		int2lcdyx(plazma_bat_drv0,3,4,0);
		int2lcdyx(plazma_bat_drv1,3,10,0);
		int2lcdyx(bat_drv_cnt_cnt,3,19,0);

    	}  		  		

   else if(sub_ind==7)
     	{
     	bgnd_par("7                   ",
     		    "                    ",
     		    "                    ",
     		    "                    ");
     		    
		int2lcdyx(main_vent_pos,0,19,0);
		int2lcdyx(TBOXMAX,1,2,0);
		int2lcdyx(TBOXREG,2,2,0);
		int2lcdyx(t_box,3,2,0);

		int2lcdyx(adc_buff_ext_[0],1,10,0);
		int2lcdyx(adc_buff_ext_[1],2,10,0);
		int2lcdyx(adc_buff_ext_[2],3,10,0);
    		}
    else if(sub_ind==8)
     	{
     	bgnd_par("                    ",
     		    "                    ",
     		    "                    ",
     		    "                    ");
     	int2lcdyx(ibt._T[0],0,2,0);
		int2lcdyx(ibt._T[1],1,2,0);
     	int2lcdyx(ibt._T[2],2,2,0);
		int2lcdyx(ibt._T[3],3,2,0);
		
     	int2lcdyx(ibt._nd[0],0,4,0);
		int2lcdyx(ibt._nd[1],1,4,0);
     	int2lcdyx(ibt._nd[2],2,4,0);
		int2lcdyx(ibt._nd[3],3,4,0);	    

     	int2lcdyx(ibt._T_dispers[0],0,7,0);
		int2lcdyx(ibt._T_dispers[1],1,7,0);
     	int2lcdyx(ibt._T_dispers[2],2,7,0);
		int2lcdyx(ibt._T_dispers[3],3,7,0);
			    
		int2lcdyx(ibt._avg1,0,19,0);
		int2lcdyx(ibt._max_dispers_num,1,19,0);
		int2lcdyx(t_box,3,19,0);
     	}		     	

    else if(sub_ind==10)
     	{
     	bgnd_par("LB                  ",
     		    "                    ",
     		    "                    ",
     		    "                    ");

     	int2lcdyx(sub_ind1+1,0,3,0);
		int2lcdyx(lakb[sub_ind1]._cnt,0,6,0);

		int2lcdyx(lakb[sub_ind1]._max_cell_temp,0,14,0);
		int2lcdyx(lakb[sub_ind1]._min_cell_temp,0,19,0);

		int2lcdyx(lakb[sub_ind1]._max_cell_volt,1,4,0);
		int2lcdyx(lakb[sub_ind1]._min_cell_volt,1,9,0);
		int2lcdyx(lakb[sub_ind1]._tot_bat_volt,1,14,0);
		int2lcdyx(lakb[sub_ind1]._s_o_h,1,19,0);

		int2lcdyx(lakb[sub_ind1]._ch_curr,2,4,0);
		int2lcdyx(lakb[sub_ind1]._dsch_curr,2,9,0);
		int2lcdyx(lakb[sub_ind1]._rat_cap,2,14,0);
		int2lcdyx(lakb[sub_ind1]._s_o_c,2,19,0);

		int2lcdyx(lakb[sub_ind1]._c_c_l_v,3,4,0);
		int2lcdyx(lakb[sub_ind1]._r_b_t,3,9,0);
		int2lcdyx(lakb[sub_ind1]._b_p_ser_num,3,14,0);
		int2lcdyx(lakb[sub_ind1]._bRS485ERR,3,16,0);
		int2lcdyx(lakb[sub_ind1]._rs485_cnt,3,19,0); 
		
     	}	
     			
     }


}							    


#define BUT0	16
#define BUT1	17
#define BUT2	18
#define BUT3	19
#define BUT4	20   
#define BUT_MASK (1UL<<BUT0)|(1UL<<BUT1)|(1UL<<BUT2)|(1UL<<BUT3)|(1UL<<BUT4)

#define BUT_ON 4
#define BUT_ONL 20 

#define butLUR_  101
#define butU   253
#define butU_  125
#define butD   251
#define butD_  123
#define butL   247
#define butL_  119
#define butR   239
#define butR_  111
#define butE   254
#define butE_  126
#define butEL_  118
#define butUD  249
#define butUD_  121
#define butLR   231
#define butLR_   103
#define butED_  122
#define butDR_  107

#define BUT_ON 4
#define BUT_ONL 20 
//-----------------------------------------------
void but_drv(void)
{
char i;
LPC_GPIO1->FIODIR|=(1<<21);
LPC_GPIO1->FIOPIN&=~(1<<21);
LPC_GPIO1->FIODIR&=~((1<<22)|(1<<23)|(1<<24)|(1<<25)|(1<<26));
LPC_PINCON->PINMODE3&=~((1<<12)|(1<<13)|(1<<14)|(1<<15)|(1<<16)|(1<<17)|(1<<18)|(1<<19)|(1<<20)|(1<<21));

LPC_GPIO2->FIODIR|=(1<<8);
LPC_GPIO2->FIOPIN&=~(1<<8);
for(i=0;i<200;i++)
{
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
__nop();
}

			LPC_GPIO2->FIODIR|=(1<<8);
			LPC_GPIO2->FIOPIN|=(1<<8);

but_n=((LPC_GPIO1->FIOPIN|(~((1<<22)|(1<<23)|(1<<24)|(1<<25)|(1<<26))))>>22)/*&0x0000001f*/;



if((but_n==1023UL)||(but_n!=but_s))
 	{
	speed=0;
 
   	if (((but0_cnt>=BUT_ON)||(but1_cnt!=0))&&(!l_but))
  		{
   	     n_but=1;
          but=but_s;

          }
   	if (but1_cnt>=but_onL_temp)
  		{
   	     n_but=1;
 
          but=but_s&0x7f;
          }
    	l_but=0;
   	but_onL_temp=BUT_ONL;
    	but0_cnt=0;
  	but1_cnt=0;          
     goto but_drv_out;
  	}
else if(but_n==but_s)
 	{
  	but0_cnt++;
  	if(but0_cnt>=BUT_ON)
  		{
   		but0_cnt=0;
   		but1_cnt++;
   		if(but1_cnt>=but_onL_temp)
   			{              
    			but=but_s&0x7f;
    			but1_cnt=0;
    			n_but=1;
    			     
    			l_but=1;
			if(speed)
				{
    				but_onL_temp=but_onL_temp>>1;
        			if(but_onL_temp<=2) but_onL_temp=2;
				}    
   			}
  		}
 	}
but_drv_out: 
but_s=but_n; 
   
}

//-----------------------------------------------
void but_an(void)
{
signed short temp_SS;
signed short deep,i,cap,ptr;
char av_head[4];
if(!n_but)goto but_an_end;
/*else  					
	{
	plazma_but_an++;
	goto but_an_end;
	}*/
av_beep=0x0000;
av_rele=0x0000;
mnemo_cnt=MNEMO_TIME;
ips_bat_av_stat=0;
//bat_ips._av&=~1;

if((main_1Hz_cnt<10)&&((but==butU)||(but==butU_)||(but==butD)||(but==butD_)||(but==butL)||(but==butL_)||(but==butR)||(but==butR_)||(but==butE)||(but==butE_)))
	{
	__ee_spc_stat=spcOFF;
	spc_stat=spcOFF;
	}
if(but==butUD)
     {
     if(ind!=iDeb)
          {
		c_ind=a_ind;
		tree_up(iDeb,10,0,0);
		
          }
     else 
          {
		tree_down(0,0);
          }
		
		     
     }
else if(but==butLR)
	{
	bSILENT=1;
	beep_init(0x00000000,'S');
	}
else if(but==butUD_)
     {
	avar_bat_as_hndl(0,1);
	}

else if(but==butED_)
     {
	if(!bCAN_OFF)bCAN_OFF=1;
	else bCAN_OFF=0;
	speed=0;
	}

else if(ind==iDeb)
	{
	if(but==butR)
		{
		sub_ind++;
		index_set=0;
		gran_ring_char(&sub_ind,0,10);
		}
	else if(but==butL)
		{
		sub_ind--;
		index_set=0;
		gran_ring_char(&sub_ind,0,10);
		}
		
	else if(sub_ind==1)
		{
		if(but==butU)
	     	{
	     	sub_ind1--;
	     	gran_char(&sub_ind1,0,30);
	     	}
		if(but==butD)
	     	{
	     	sub_ind1++;
	     	gran_char(&sub_ind1,0,30);
	     	}
	     
		if(but==butE)
	     	{
	     	/*SET_REG(C2GSR,3,24,8);
			C2MOD=0;
			 bOUT_FREE2=1;*/

			 // CAN interface 1, use IRQVec7, at 125kbit
//can2_init(7,8,CANBitrate250k_60MHz);

// Receive message with ID 102h on CAN 1
//FullCAN_SetFilter(2,0x18e);
			 }

		if(but==butE)
	     	{
			//lc640_write_int(EE_BAT1_ZAR_CNT,10);
			ind_pointer=0;
			ind=(i_enum)0;
			sub_ind=0;
			sub_ind1=0;
			sub_ind2=0;
			index_set=0;
			}
	     
			
		}
		
	 else if(sub_ind==5)
	 	{
		if(but==butE_)
		{
		//can1_init(BITRATE62_5K6_25MHZ);
		//FullCAN_SetFilter(0,0x18e);
		LPC_CAN1->MOD&=~(1<<0);
		}
		}

	else if(sub_ind==1)
		{
		if(but==butU)
	     	{
	     	sub_ind1--;
	     	gran_char(&sub_ind1,0,1);
	     	}
		if(but==butD)
	     	{
	     	sub_ind1++;
	     	gran_char(&sub_ind1,0,1);
	     	}
		}		
		
		
			
     else if(but==butU)
	     {
	     index_set--;
	     gran_char(&index_set,0,4);
	     //lc640_write_int(ptr_ki_src[0],lc640_read_int(ptr_ki_src[0])+10);
	     }	
     else if(but==butD)
	     {
	     index_set++;
	     gran_char(&index_set,0,4); 
	     //lc640_write_int(ptr_ki_src[0],lc640_read_int(ptr_ki_src[0])-10);
	     }	
     else if(but==butE)
         	{
          //a=b[--ptr_ind];
          }   
          
     else if(but==butE_)
         	{
          //a=b[--ptr_ind];
          
          }                      				
	}


else if(ind==iMn_6U)
	{
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,9+NUMBAT+NUMIST+NUMINV+NUMMAKB+NUMBYPASS+(NUMEXT!=0));
		//can1_init(BITRATE62_5K25MHZ);
		}
		
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,9+NUMBAT+NUMIST+NUMINV+NUMMAKB+NUMBYPASS+(NUMEXT!=0));
		//LPC_CAN1->CMR=0x00000022;
		}	

	else if(but==butE_)
		{
		//can1_init(BITRATE62_5K25MHZ);
		//FullCAN_SetFilter(0,0x18e);
		}
	else if(but==butDR_)
		{
		tree_up(iK_6U,0,0,0);
		}
	else if(but==butL)
		{
		//ind=iMn;
		sub_ind=0;
		}
		
	else if(but==butE)
		{
		if(sub_ind==0)
			{
			if(avar_ind_stat)
				{
				//ind=iAv_view;
				//sub_ind=0;
				tree_up(iAv_view,0,0,0);
				while(!(avar_ind_stat&(1<<sub_ind)))
					{
					sub_ind++;
					if(sub_ind>=32)
						{
						tree_down(0,0);
						avar_ind_stat=0;
						}
					}
				}																							
			}
		else if((sub_ind>0)&&(sub_ind<=NUMBAT))
		    	{
		    	if(BAT_IS_ON[0]!=bisON)
				{
				if(BAT_TYPE==0)tree_up(iBat_simple,0,0,1);
				else if(BAT_TYPE==1) tree_up(iBat_li,0,0,1);
				}
		    	else 
				{
				if(BAT_TYPE==0)tree_up(iBat_simple,0,0,sub_ind-1);
				else if(BAT_TYPE==1) tree_up(iBat_li,0,0,sub_ind-1);
				}
		    	}
		else if((sub_ind>NUMBAT)&&(sub_ind<=(NUMBAT+NUMIST)))
		    	{
		    	tree_up(iBps,0,0,sub_ind-(1+NUMBAT));
		    	}
		else if((sub_ind>(NUMBAT+NUMIST))&&(sub_ind<=(NUMBAT+NUMIST+NUMBYPASS)))
		    	{
		    	tree_up(iByps,0,0,0);
		    	}
		else if((sub_ind>(NUMBAT+NUMIST+NUMBYPASS))&&(sub_ind<=(NUMBAT+NUMIST+NUMBYPASS+NUMINV)))
		    	{
		    	tree_up(iInv_v2,0,0,sub_ind-(1+NUMBAT+NUMIST+NUMBYPASS));
		    	}
		else if((sub_ind==(1+NUMBAT+NUMIST+NUMBYPASS+NUMINV))&&(NUMINV))
			{
			tree_up(iInv_tabl,0,0,0);
		     //ret(500);
		     }
		else if((sub_ind>(NUMBAT+NUMIST+NUMBYPASS+NUMINV+(NUMINV!=0)))&&(sub_ind<=(NUMBAT+NUMIST+NUMBYPASS+NUMINV+NUMMAKB+(NUMINV!=0))))
		    	{
		    	tree_up(iMakb,0,0,sub_ind-(1+NUMBAT+NUMIST+NUMBYPASS+NUMINV+(NUMINV!=0)));
		    	}
		else if(sub_ind==(1+NUMBAT+NUMIST+NUMBYPASS+NUMINV+NUMMAKB+(NUMINV!=0)))
			{
			if(AUSW_MAIN%10)
				{
				tree_up(iNet3,0,0,0);
		     	ret(1000);
				}
			else 
				{
				tree_up(iNet,0,0,0);
		     	ret(1000);
				}
			}
		else if(sub_ind==(2+NUMBAT+NUMIST+NUMBYPASS+NUMINV+NUMMAKB+(NUMINV!=0)))
			{
			tree_up(iLoad,0,0,0);
		     ret(1000);
			}
		else if((sub_ind==(3+NUMBAT+NUMIST+NUMBYPASS+NUMINV+NUMMAKB)+(NUMINV!=0))&&(NUMEXT))
			{
			tree_up(iExtern_6U,0,0,0);
		     ret(1000);
			}

		else if(sub_ind==(3+NUMBAT+NUMIST+NUMBYPASS+NUMINV+NUMMAKB+(NUMINV!=0)+(NUMEXT!=0)))
			{
			tree_up(iSpc,0,0,0);
		     ret(1000);
			}

		else if(sub_ind==(4+NUMBAT+NUMIST+NUMBYPASS+NUMINV+NUMMAKB+(NUMINV!=0)+(NUMEXT!=0)))
			{
			tree_up(iSet_prl,0,0,0);
		     ret(50);
		     parol_init();
			}
		else if(sub_ind==(5+NUMBAT+NUMIST+NUMBYPASS+NUMINV+NUMMAKB+(NUMINV!=0)+(NUMEXT!=0)))
			{
			tree_up(iLog,0,0,0);
		     ret(1000);
			}
		else if(sub_ind==(6+NUMBAT+NUMIST+NUMBYPASS+NUMINV+NUMMAKB+(NUMINV!=0)+(NUMEXT!=0)))
			{
			sub_ind=0;
			}
		else if(sub_ind==(7+NUMBAT+NUMIST+NUMBYPASS+NUMINV+NUMMAKB+(NUMINV!=0)+(NUMEXT!=0)))
			{
			tree_up(iBatLog,0,0,0);
		     ret(1000);
			}
		else if(sub_ind==(8+NUMBAT+NUMIST+NUMBYPASS+NUMINV+NUMMAKB+(NUMINV!=0)+(NUMEXT!=0)))
			{
			tree_up(iBatLog,0,0,1);
		     ret(1000);
			}
		else if(sub_ind==(9+NUMBAT+NUMIST+NUMBYPASS+NUMINV+NUMMAKB+(NUMINV!=0)+(NUMEXT!=0)))
			{
			if(but==butE)
		     	{
		     	tree_up(iPrltst,0,0,0);
		     	parol_init();
		     	}
			}
		}
    	}

	
but_an_end:
n_but=0;
}

//-----------------------------------------------
void watchdog_enable (void) 
{
LPC_WDT->WDTC=2000000;
LPC_WDT->WDCLKSEL=0;
LPC_WDT->WDMOD=3;
LPC_WDT->WDFEED=0xaa;
LPC_WDT->WDFEED=0x55;
}

//-----------------------------------------------
void watchdog_reset (void) 
{
LPC_WDT->WDFEED=0xaa;
LPC_WDT->WDFEED=0x55;
}


//***********************************************
//***********************************************
//***********************************************
//***********************************************
//***********************************************
void SysTick_Handler (void) 	 /* SysTick Interrupt Handler (1ms)    */
{
//sys_plazma++;
b2000Hz=1;

if(bTPS)
	{
	LPC_GPIO1->FIODIR|=(1UL<<26);
	LPC_GPIO1->FIOPIN^=(1UL<<26);
	}

if(++t0cnt4>=2)
	{
t0cnt4=0;
b1000Hz=1;

	bFF=(char)(GET_REG(LPC_GPIO0->FIOPIN, 27, 1));
	if(bFF!=bFF_) hz_out++;
	bFF_=bFF;


if(++t0cnt5>=20)
     {
     t0cnt5=0;
     b50Hz=1;
     }
     
if(++t0cnt>=10)
     {
     t0cnt=0;
     b100Hz=1;

     hz_out_cnt++;
     if(hz_out_cnt>=500)
	     {	
	     hz_out_cnt=0;
	     net_F=hz_out;
	     hz_out=0;
	     }

     if(++t0cnt0>=10)
	     {
	     t0cnt0=0;
	     b10Hz=1;
		beep_drv();
		if(main_10Hz_cnt<10000) main_10Hz_cnt++;
	     }

     if(t0cnt0==5)
	     {
		//beep_drv();
	     }

     if(++t0cnt1>=20)
	     {
	     t0cnt1=0;
	     b5Hz=1;
		if(bFL5)bFL5=0;
		else bFL5=1;     
	     }

     if(++t0cnt2>=50)
	     {
	     t0cnt2=0;
	     b2Hz=1;
		if(bFL2)bFL2=0;
		else bFL2=1;

	     }         

     if(++t0cnt3>=100)
	     {
	     t0cnt3=0;
	     b1Hz=1;
		if(main_1Hz_cnt<10000) main_1Hz_cnt++;
		if(bFL)bFL=0;
		else bFL=1;
	     }
     }

	}


if(modbus_timeout_cnt<6)
	{
	modbus_timeout_cnt++;
	if(modbus_timeout_cnt>=6)
		{
		bMODBUS_TIMEOUT=1;
		}
	}
else if (modbus_timeout_cnt>6)
	{
	modbus_timeout_cnt=0;
	bMODBUS_TIMEOUT=0;
	}

//LPC_GPIO0->FIOCLR|=0x00000001;
  return;          



//LPC_GPIO0->FIOCLR|=0x00000001;
}


//***********************************************
__irq void timer0_interrupt(void) 
{	
/*if(BPS1_spa_leave)T0EMR_bit.EM1=0; 
else T0EMR_bit.EM1=1;
if(BPS2_spa_leave)T0EMR_bit.EM3=0; 
else T0EMR_bit.EM3=1;
T0IR = 0xff;*/
}

//===============================================
//===============================================
//===============================================
//===============================================
int main (void) 
{
char ind_reset_cnt=0;
//long i;
char mac_adr[6] = { 0x00,0x73,0x04,50,60,70 };

//i=200000;
//while(--i){};

SystemInit();

bTPS=1;

SysTick->LOAD = (SystemFrequency / 2000) - 1;
SysTick->CTRL = 0x07;

//init_timer( 0,SystemFrequency/2000/4 - 1 ); // 1ms	
//enable_timer( 0 );

//rs232_data_out_1();

bps[0]._state=bsOFF_AV_NET;
bps[1]._state=bsOFF_AV_NET;
bps[2]._state=bsOFF_AV_NET;
bps[3]._state=bsOFF_AV_NET;
bps[4]._state=bsOFF_AV_NET;
bps[5]._state=bsOFF_AV_NET;
bps[6]._state=bsOFF_AV_NET;

SET_REG(LPC_GPIO0->FIODIR, 0, 27, 1);
SET_REG(LPC_GPIO2->FIODIR, 1, 7, 1);
SET_REG(LPC_GPIO2->FIODIR, 1, 8, 1);
//LPC_GPIO1->FIODIR  |= 1<<27;                
	;
//FIO1MASK = 0x00000000;	 
//LPC_GPIO0->FIODIR  |= 1<<27;
//LPC_GPIO0->FIOSET  |= 1<<27;

///SET_REG(LPC_GPIO0->FIODIR,0,10,1); //вход частоты 

SET_REG(LPC_GPIO3->FIODIR,1,SHIFT_REL_AV_NET,1);
SET_REG(LPC_GPIO3->FIOSET,1,SHIFT_REL_AV_NET,1);  // реле аварии сети под ток


/* 2515
ad7705_reset();
delay_ms(20);

ad7705_write(0x21);
ad7705_write(BIN8(1101)); 
ad7705_write(0x11);
ad7705_write(0x44);
*/

ad7705_buff[0][1]=0x7fff;
ad7705_buff[0][2]=0x7fff;
ad7705_buff[0][3]=0x7fff;
ad7705_buff[0][4]=0x7fff;
ad7705_buff[0][5]=0x7fff;
ad7705_buff[0][6]=0x7fff;
ad7705_buff[0][7]=0x7fff;
ad7705_buff[0][8]=0x7fff;
ad7705_buff[0][9]=0x7fff;
ad7705_buff[0][10]=0x7fff;
ad7705_buff[0][11]=0x7fff;
ad7705_buff[0][12]=0x7fff;
ad7705_buff[0][13]=0x7fff;
ad7705_buff[0][14]=0x7fff;
ad7705_buff[0][15]=0x7fff;
ad7705_buff[1][1]=0x7fff;
ad7705_buff[1][2]=0x7fff;
ad7705_buff[1][3]=0x7fff;
ad7705_buff[1][4]=0x7fff;
ad7705_buff[1][5]=0x7fff;
ad7705_buff[1][6]=0x7fff;
ad7705_buff[1][7]=0x7fff;
ad7705_buff[1][8]=0x7fff;
ad7705_buff[1][9]=0x7fff;
ad7705_buff[1][10]=0x7fff;
ad7705_buff[1][11]=0x7fff;
ad7705_buff[1][12]=0x7fff;
ad7705_buff[1][13]=0x7fff;
ad7705_buff[1][14]=0x7fff;
ad7705_buff[1][15]=0x7fff;

ad7705_buff_[0]=0x7fff;
ad7705_buff_[1]=0x7fff;

/*
ad7705_reset();
delay_ms(20);

ad7705_write(0x20);
ad7705_write(BIN8(1101)); 
ad7705_write(0x10);
ad7705_write(0x44);

ad7705_reset();
delay_ms(20);  

ad7705_write(0x20);
ad7705_write(BIN8(1101)); 
ad7705_write(0x10);
ad7705_write(0x44); 

delay_ms(20); */




lcd_init();  
lcd_on();
lcd_clear();
		
///LPC_GPIO4->FIODIR |= (1<<29);           /* LEDs on PORT2 defined as Output    */
rtc_init();
///pwm_init();
ind=iMn;
#ifdef UKU_GLONASS
ind=iMn_GLONASS;
#endif
#ifdef UKU_3U
ind=iMn_3U;
#endif
#ifdef UKU_RSTKM
ind=iMn_RSTKM;
#endif 
#ifdef UKU_KONTUR
ind=iMn_KONTUR;
#endif 
#ifdef UKU_6U
ind=iMn_6U;
#endif
#ifdef UKU_220
ind=iMn_220;
#endif
#ifdef UKU_220_IPS_TERMOKOMPENSAT
ind=iMn_220_IPS_TERMOKOMPENSAT;
#endif
#ifdef UKU_220_V2
ind=iMn_220_V2;
#endif

//snmp_plazma=15;


//#ifdef ETHISON
//mac_adr[5]=*((char*)&AUSW_MAIN_NUMBER);
//mac_adr[4]=*(((char*)&AUSW_MAIN_NUMBER)+1);
//mac_adr[3]=*(((char*)&AUSW_MAIN_NUMBER)+2);
//mem_copy (own_hw_adr, mac_adr, 6);


//if(lc640_read_int(EE_ETH_IS_ON)==1)
	//{
	bgnd_par(		"                    ",
     		"    Инициализация   ",
     		"      Ethernet      ",
     		"                    ");
	//bitmap_hndl();
	//lcd_out(lcd_bitmap);
	//init_TcpNet ();

	//init_ETH();
	//mem_copy (&localm[NETIF_ETH], &ip_config, sizeof(ip_config));

//	}
//#endif
//event2snmp(2);

reload_hndl();
//LPC_GPIO0->FIODIR |= (0x60000000);

adc_init();

LPC_GPIO0->FIODIR|=(1<<11);
LPC_GPIO0->FIOSET|=(1<<11);

/* 2515
lc640_write_int(100,134);

can1_init(BITRATE62_5K25MHZ); 
can2_init(BITRATE125K25MHZ);
FullCAN_SetFilter(1,0x0e9);
FullCAN_SetFilter(0,0x18e);*/



//2515 memo_read();

#ifndef UKU_220 
UARTInit(0, (uint32_t)MODBUS_BAUDRATE*10UL);	/* baud rate setting */
#endif

#ifdef UKU_220 
UART_2_Init((uint32_t)MODBUS_BAUDRATE*10UL);	/* baud rate setting */
UARTInit(0, (uint32_t)MODBUS_BAUDRATE*10UL);	/* baud rate setting */
#endif


//mac_adr[5]=*((char*)&AUSW_MAIN_NUMBER);
//mac_adr[4]=*(((char*)&AUSW_MAIN_NUMBER)+1);
//mac_adr[3]=*(((char*)&AUSW_MAIN_NUMBER)+2);
//mem_copy (own_hw_adr, mac_adr, 6);




if(__ee_spc_stat==spcVZ)
	{
	if(__ee_vz_cnt)
		{
		spc_stat=spcVZ;  
		vz_cnt_h=__ee_vz_cnt/60;
		vz_cnt_h_=(lc640_read_int(EE_SPC_VZ_LENGT)-__ee_vz_cnt)/60;
		if(vz_cnt_h_<0)vz_cnt_h_=0;
		vz_cnt_s_=(short)(((lc640_read_int(EE_SPC_VZ_LENGT)-__ee_vz_cnt)*60)%3600UL);

		vz_cnt_s=0;
		}
	}
else if(__ee_spc_stat==spcKE)
	{
	spc_stat=spcKE;
	spc_bat=__ee_spc_bat;
	bat[spc_bat]._zar_cnt_ke=0;
	spc_phase=__ee_spc_phase;
	}
//watchdog_enable();
if((AUSW_MAIN==2400)||(AUSW_MAIN==4800)||(AUSW_MAIN==6000)||(BAT_TYPE==1))
	{
	cntrl_stat=350;
	cntrl_stat_old=350;
	}
kb_init();
//ind=iDeb;
//sub_ind=6;

can_mcp2515_init();
		
while (1)  
	{
	bTPS=0; 
     //timer_poll ();
//2515     main_TcpNet ();

	//watchdog_reset();

/* 2515	if(bMODBUS_TIMEOUT)
		{
		bMODBUS_TIMEOUT=0;
		//modbus_plazma++;;
		modbus_in();
		} */

	if(bRXIN0) 
		{
		bRXIN0=0;
	
//2515		uart_in0();
		} 
	/*
	if(bRXIN1) 
		{
		bRXIN1=0;
	
		uart_in1();
		}*/ 
     if(b10000Hz)
		{
		b10000Hz=0; 
		

		}

     if(b2000Hz)
		{

		if(adc_window_cnt<200)adc_window_cnt++;

		b2000Hz=0; 
	//2515	adc_drv7();
		
		}

	if(b1000Hz)
		{
		b1000Hz=0;
		
		can_mcp2515_hndl();
		//mcp2515_read_status();
		}
	
	if(b100Hz)
		{
		b100Hz=0;

		//LPC_GPIO2->FIODIR|=(1<<7);
		//LPC_GPIO2->FIOPIN^=(1<<7);		

		if(!bRESET)but_drv();
		but_an();
		}
		 
	if(b50Hz)
		{
		b50Hz=0;
		net_drv_mcp2515();
		//mcp2515_transmit(2,2,GETTM,bps[cnt_net_drv]._flags_tu,*((char*)(&bps[cnt_net_drv]._vol_u)),*((char*)((&bps[cnt_net_drv]._vol_u))+1),*((char*)(&bps[cnt_net_drv]._vol_i)),*((char*)((&bps[cnt_net_drv]._vol_i))+1));
     	//mcp2515_transmit(1,2,3,0,0,0,0,0);
		}

	if(b10Hz)
		{
		char i;

     
     tick = __TRUE;

		b10Hz=0;
				
		u_necc_hndl();
		
		for(i=0;i<NUMIST;i++)bps_drv(i);
		bps_hndl();

		//inv_search();
		
		//if(NUMINV) {for(i=0;i<NUMINV;i++)inv_drv(i);}		  
		
		//nv[0]._Uii=123;


		if(BAT_IS_ON[0]==bisON)bat_drv(0);
		if(BAT_IS_ON[1]==bisON)bat_drv(1);
		bat_hndl();

#ifdef UKU_220_IPS_TERMOKOMPENSAT		
		if(main_10Hz_cnt>200)
			{
			if(abs(Ib_ips_termokompensat)>IKB) 
				{
				if((bat_ips._av&1))avar_bat_ips_hndl(0);
				}
			}
#endif				
		unet_drv();

		
		
		ind_hndl(); 
		#ifndef SIMULATOR
		bitmap_hndl();
		if(!bRESET)
			{
			lcd_out(lcd_bitmap);
			}
		#endif
		//ad7705_drv();
		//ad7705_write(0x20);

		adc_window_cnt=0;  

		ret_hndl();  
		mess_hndl();
		cntrl_hndl();
		ret_hndl();
		ext_drv();
		avt_hndl();
		}

	if(b5Hz)
		{
		b5Hz=0;

		if(!bRESET)
			{
			//ad7705_drv();
			}
		if(!bRESET)
			{
			//memo_read();
			}
		LPC_GPIO1->FIODIR|=(1UL<<26);
		matemat();
		
		rele_hndl();
//2515		if(!bRESET)avar_hndl();
		zar_superviser_drv();
///		snmp_data();
		//LPC_GPIO1->FIODIR|=(1UL<<31);
		//LPC_GPIO1->FIOPIN^=(1UL<<31);


  		}

	if(b2Hz)
		{
		b2Hz=0;

				//uart_out_adr1(dig,150);
		
  		}

	if(b1Hz)
		{
		//can_plazma++;
		//mcp2515_transmit(0,0,0,0,0,0,0,0);

		b1Hz=0;
/* 2515		if(!bRESET)
			{
			watchdog_reset();
			}
		//can1_out_adr((char*)&net_U,21);

		samokalibr_hndl();
		num_necc_hndl();
		//zar_drv();
		ubat_old_drv();
		kb_hndl();
		beep_hndl();
		avg_hndl();
		vz_drv();	 
		avz_drv();
		ke_drv();
		mnemo_hndl();
		vent_hndl(); */

/*2515		plazma_plazma_plazma++;

		if(++ind_reset_cnt>=60)
			{
			ind_reset_cnt=0;
			lcd_init();
			lcd_on();
			lcd_clear();
			}*/
               
/*  2515        vent_hndl();
		klimat_hndl();
		
		if(t_ext_can_nd<10) t_ext_can_nd++;
		
		//if(main_1Hz_cnt<200)main_1Hz_cnt++;


		can_reset_hndl();
		npn_hndl(); */
/*		#ifdef UKU_220_IPS_TERMOKOMPENSAT
		if((AUSW_MAIN==22063)||(AUSW_MAIN==22023)||(AUSW_MAIN==22043))rs232_data_out_tki();
		else if(AUSW_MAIN==22010)rs232_data_out_1();
		else rs232_data_out();
		#endif */	
 
		//modbus_registers_transmit(MODBUS_ADRESS,4,0,5);
		
	/*	putchar2(0x56);
		putchar2(0x57);
		putchar2(0x58);
		putchar2(0x59);
		putchar2(0x5a);*/

		}
	if(b1min)
		{
		b1min=0;

/* 2515		if((tloaddisable_cmnd)&&(tloaddisable_cmnd<=10))
			{
			tloaddisable_cmnd--;
			if(!tloaddisable_cmnd)tloaddisable_cmnd=20;
			}
		if((tbatdisable_cmnd)&&(tbatdisable_cmnd<=10))
			{
			if(!tbatdisable_cmnd)tbatdisable_cmnd=20;
			} */

		
		}
	}
}
