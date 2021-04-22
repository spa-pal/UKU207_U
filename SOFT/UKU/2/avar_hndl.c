#include "avar_hndl.h"
#include "eeprom_map.h"	   
#include "full_can.h"
#include "25lc640.h"
#include <LPC17xx.H>
#include "main.h"
#include "control.h"
#include "snmp_data_file.h" 

     
//***********************************************
//������
unsigned avar_stat;	 	//"�����������" ���� ��������� � ������ ������ ��������� � ����� �����
unsigned avar_ind_stat; 	//"�����������" ���� �� ������������� ��������� ��������� � ����� �����
unsigned avar_stat_old;
unsigned avar_stat_new,avar_stat_offed;
//��������� ����������
//1���  - �������� ����
//2���� - �������
//12��� - ����
//5���  - ���������
//4���� - ������� ������� �����������
//4���� - ������� ����� ��������
char sk_avar_stat;	 	//"�����������" ���� ��������� � ������ ������ �� � ����� �����
char sk_avar_ind_stat; 	//"�����������" ���� �� ������������� ��������� �� � ����� �����
char sk_avar_stat_old;
char sk_avar_stat_new,sk_avar_stat_offed;

char /*av_net*//*,av_bat[2]*//*av_bps[12],*/av_inv[6];//,/*av_dt[4],av_sk[4]*/;

// oleg_start
//������
unsigned rki_avar1_stat;	 	//"�����������" ���� ��������� � ������ ������ ��������� � ����� �����
unsigned rki_avar1_ind_stat; 	//"�����������" ���� �� ������������� ��������� ��������� � ����� �����
unsigned rki_avar1_stat_old;
unsigned rki_avar1_stat_new, rki_avar1_stat_offed;

/*
unsigned rki_avarI1_stat;	 	//"�����������" ���� ��������� � ������ ������ ��������� � ����� �����
unsigned rki_avarI1_ind_stat; 	//"�����������" ���� �� ������������� ��������� ��������� � ����� �����
unsigned rki_avarI1_stat_old;
unsigned rki_avarI1_stat_new, rki_avarI1_stat_offed;

unsigned rki_avarI2_stat;	 	//"�����������" ���� ��������� � ������ ������ ��������� � ����� �����
unsigned rki_avarI2_ind_stat; 	//"�����������" ���� �� ������������� ��������� ��������� � ����� �����
unsigned rki_avarI2_stat_old;
unsigned rki_avarI2_stat_new, rki_avarI2_stat_offed;
*/
// oleg_end

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
//�������


//***********************************************
//��������� ��������� ����
//char u_net_av_stat,u_net_av_stat_;
extern signed short net_U,net_Ustore, net_Umax, net_Ustore_max; //o_10
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

if(net_av)		SET_REG(avar_stat,1,0,1);
else	   			SET_REG(avar_stat,0,0,1);
#ifndef UKU_220_IPS_TERMOKOMPENSAT //o_10
for(i=0;i<2;i++)
	{
	if(bat[i]._av&3)	SET_REG(avar_stat,1,1+i,1);
	else	   			SET_REG(avar_stat,0,1+i,1);
	}
#endif //o_10
for(i=0;i<18;i++)
	{
	if(bps[i]._av&0xef)	SET_REG(avar_stat,1,3+i,1);
	else	   			SET_REG(avar_stat,0,3+i,1);
	}

/*for(i=0;i<6;i++)
	{
	if(av_inv[i])	SET_REG(avar_stat,1,15+i,1);
	else	   		SET_REG(avar_stat,0,15+i,1);
	}*/

/*for(i=0;i<4;i++)
	{
	if(av_dt[i])	SET_REG(avar_stat,1,21+i,1);
	else	   		SET_REG(avar_stat,0,21+i,1);
	}  */
#ifndef UKU_ZVU
for(i=0;i<4;i++)
	{
	if(sk_av_stat[i]==sasON)	SET_REG(sk_avar_stat,1,i,1);
	else	   					SET_REG(sk_avar_stat,0,i,1);
	}
#endif //UKU_ZVU
#ifndef UKU_FSO
for(i=0;i<4;i++)
	{
//	if(sk_av_stat[i]==sasON)	SET_REG(avar_stat,1,24+i,1);
//	else	   		SET_REG(avar_stat,0,24+i,1);
	}
#endif //UKU_FSO
//if(uout_av)			SET_REG(avar_stat,1,28,1);
//else	   			SET_REG(avar_stat,0,28,1);


avar_stat_new=(avar_stat^avar_stat_old)&avar_stat;

avar_ind_stat|=avar_stat_new;

avar_stat_offed=~((avar_stat^avar_stat_old)&avar_stat_old);

if(!AV_OFF_AVT)avar_stat_offed|=0xeffffffe;

avar_ind_stat&=avar_stat_offed; 

avar_stat_old=avar_stat;


sk_avar_stat_new=(sk_avar_stat^sk_avar_stat_old)&sk_avar_stat;

sk_avar_ind_stat|=sk_avar_stat_new;

sk_avar_stat_offed=~((sk_avar_stat^sk_avar_stat_old)&sk_avar_stat_old);


if((SK_ZVUK_EN[0])) sk_avar_ind_stat&=(~(1UL<<0));
if((SK_ZVUK_EN[1])) sk_avar_ind_stat&=(~(1UL<<1));
if((SK_ZVUK_EN[2])) sk_avar_ind_stat&=(~(1UL<<2));
if((SK_ZVUK_EN[3])) sk_avar_ind_stat&=(~(1UL<<3));	

if(!AV_OFF_AVT)sk_avar_stat_offed|=0x0f;

sk_avar_ind_stat&=sk_avar_stat_offed; 

sk_avar_stat_old=sk_avar_stat;

//oleg_start



rki_avar1_stat=(sk_alarm<<8)|(status_izm_r&0x3F);
rki_avar1_stat_new=(rki_avar1_stat^rki_avar1_stat_old)&rki_avar1_stat;
rki_avar1_ind_stat|=rki_avar1_stat_new;
rki_avar1_stat_offed=~((rki_avar1_stat^rki_avar1_stat_old)&rki_avar1_stat_old);
rki_avar1_ind_stat&=rki_avar1_stat_offed;
rki_avar1_stat_old=rki_avar1_stat;
/*
rki_avarI1_stat=status_di1;
rki_avarI1_stat_new=(rki_avarI1_stat^rki_avarI1_stat_old)&rki_avarI1_stat;
rki_avarI1_ind_stat|=rki_avarI1_stat_new;
rki_avarI1_stat_offed=~((rki_avarI1_stat^rki_avarI1_stat_old)&rki_avarI1_stat_old);
rki_avarI1_ind_stat&=rki_avarI1_stat_offed;
rki_avarI1_stat_old=rki_avarI1_stat;

rki_avarI2_stat=status_di2;
rki_avarI2_stat_new=(rki_avarI2_stat^rki_avarI2_stat_old)&rki_avarI2_stat;
rki_avarI2_ind_stat|=rki_avarI2_stat_new;
rki_avarI2_stat_offed=~((rki_avarI2_stat^rki_avarI2_stat_old)&rki_avarI2_stat_old);
rki_avarI2_ind_stat&=rki_avarI2_stat_offed;
rki_avarI2_stat_old=rki_avarI2_stat;
*/
//oleg_end
}

//-----------------------------------------------
void reload_hndl(void)
{

char data[4];
unsigned int event_ptr,lc640_adr/*,event_ptr_find*/,event_cnt;

event_ptr=lc640_read_int(PTR_EVENT_LOG);
event_ptr++;	
if(event_ptr>63)event_ptr=0;	
lc640_write_int(PTR_EVENT_LOG,event_ptr);	
	
event_cnt=lc640_read_int(CNT_EVENT_LOG);
if(event_cnt!=63)event_cnt=event_ptr;
lc640_write_int(CNT_EVENT_LOG,event_cnt); 
	
lc640_adr=EVENT_LOG+(lc640_read_int(PTR_EVENT_LOG)*32);
	
data[0]='U';
data[1]=0;
data[2]='R';
data[3]=0;
lc640_write_long_ptr(lc640_adr,data);

data[0]=0;
data[1]=0;
data[2]=0;
data[3]=0;
lc640_write_long_ptr(lc640_adr+4,data);

data[0]=LPC_RTC->YEAR;
data[1]=LPC_RTC->MONTH;
data[2]=LPC_RTC->DOM;
data[3]=0;
lc640_write_long_ptr(lc640_adr+8,data);

data[0]=LPC_RTC->HOUR;
data[1]=LPC_RTC->MIN;
data[2]=LPC_RTC->SEC;
data[3]=LPC_SC->RSID;

lc640_write_long_ptr(lc640_adr+12,data);
	
data[0]='A';
data[1]='A';
data[2]='A';
data[3]='A';
lc640_write_long_ptr(lc640_adr+16,data);
	
data[0]='A';
data[1]='A';
data[2]='A';
data[3]='A';
lc640_write_long_ptr(lc640_adr+20,data);
	
data[0]='A';
data[1]='A';
data[2]='A';
data[3]='A';
lc640_write_long_ptr(lc640_adr+24,data);
	
data[0]='A';
data[1]='A';
data[2]='A';
data[3]='A';
lc640_write_long_ptr(lc640_adr+28,data);				
	
	
}

//-----------------------------------------------
void avar_unet_hndl(char in)
{

char data[4];
unsigned int event_ptr,lc640_adr,event_ptr_find,event_cnt;


if(in==1)
	{
	net_av|=1; //o_10
	net_av_2min_timer=1200;

	//beep_init(0x01L,'O');
	//a.av.bAN=1;


	//beep_init(0x33333333,'A');
	 
	event_ptr=lc640_read_int(PTR_EVENT_LOG);
	event_ptr++;	
	if(event_ptr>63)event_ptr=0;	
	lc640_write_int(PTR_EVENT_LOG,event_ptr);	
	
     event_cnt=lc640_read_int(CNT_EVENT_LOG);
	if(event_cnt!=63)event_cnt=event_ptr;
	lc640_write_int(CNT_EVENT_LOG,event_cnt); 
	
	lc640_adr=EVENT_LOG+(lc640_read_int(PTR_EVENT_LOG)*32);
	
	data[0]='P';
	data[1]=0;
	data[2]='A';
	data[3]=0;
	lc640_write_long_ptr(lc640_adr,data);

	data[0]=0;//*((char*)&Unet_store);
	data[1]=0;//*(((char*)&Unet_store)+1);
	data[2]=0;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+4,data);

	data[0]=LPC_RTC->YEAR;
	data[1]=LPC_RTC->MONTH;
	data[2]=LPC_RTC->DOM;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+8,data);

	data[0]=LPC_RTC->HOUR;
	data[1]=LPC_RTC->MIN;
	data[2]=LPC_RTC->SEC;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+12,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+16,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+28,data);				
	
/*	memo_out0[0]=0x55;
     memo_out0[1]=0x00+2;
     memo_out0[2]=1;
     memo_out0[3]=0x0b;
     memo_out0[4]=0x55;
     memo_out0[5]=0x55; 
     	
     memo_out0[6]=crc_87(memo_out0,6);
	memo_out0[7]=crc_95(memo_out0,6);
     usart_out_adr0(memo_out0,8); 
	*/
	snmp_trap_send("Main power alarm, voltage reduced",2,1,0);
	}

else if(in==2)
	{
	net_av|=2;	  //o_10

	//beep_init(0x01L,'O');
	//a.av.bAN=1;


	//beep_init(0x33333333,'A');
	 
	event_ptr=lc640_read_int(PTR_EVENT_LOG);
	event_ptr++;	
	if(event_ptr>63)event_ptr=0;	
	lc640_write_int(PTR_EVENT_LOG,event_ptr);	
	
     event_cnt=lc640_read_int(CNT_EVENT_LOG);
	if(event_cnt!=63)event_cnt=event_ptr;
	lc640_write_int(CNT_EVENT_LOG,event_cnt); 
	
	lc640_adr=EVENT_LOG+(lc640_read_int(PTR_EVENT_LOG)*32);
	
	data[0]='P';
	data[1]=0;
	data[2]='B';
	data[3]=0;
	lc640_write_long_ptr(lc640_adr,data);

	data[0]=0;//*((char*)&Unet_store);
	data[1]=0;//*(((char*)&Unet_store)+1);
	data[2]=0;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+4,data);

	data[0]=LPC_RTC->YEAR;
	data[1]=LPC_RTC->MONTH;
	data[2]=LPC_RTC->DOM;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+8,data);

	data[0]=LPC_RTC->HOUR;
	data[1]=LPC_RTC->MIN;
	data[2]=LPC_RTC->SEC;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+12,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+16,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+28,data);				
	
	snmp_trap_send("Main power alarm, voltage increased",2,2,0);
	}
//o_10_s
else if(in==0 || in==4)	
	{
//	if( (in==0 && (net_av&2)==0) || (in==4 && (net_av&1)==0) )  {
		if(in==0) net_av&=~1;
		if(in==4) net_av&=~2;	 
		if(net_av==0) if(AV_OFF_AVT) SET_REG(avar_ind_stat,0,0,1);

	SET_REG(avar_stat,0,0,1);
	
	
//}
//o_10_e

     event_ptr=lc640_read_int(PTR_EVENT_LOG);
	event_ptr_find=event_ptr;
avar_unet_hndl_lbl1:
	lc640_adr=EVENT_LOG+(event_ptr_find*32);

     lc640_read_long_ptr(lc640_adr,data);
     
     if( !( (data[0]=='P')&&(data[1]==0)&&( (in==0 && data[2]=='A' )||(in==4 && data[2]=='B') ) ) )	//o_10
     	{        
     	if(event_ptr_find)event_ptr_find--;
     	else event_ptr_find=63;
     	if(event_ptr_find==event_ptr)goto avar_unet_hndl_end;
     	else goto avar_unet_hndl_lbl1;
     	}
     else 
     	{
     	lc640_read_long_ptr(lc640_adr+16,data);
     	if(!((data[0]=='A')&&(data[1]=='A')&&(data[2]=='A')&&(data[3]=='A')))
     		{        
     		if(event_ptr_find)event_ptr_find--;
         		else event_ptr_find=63;
         		if(event_ptr_find==event_ptr)goto avar_unet_hndl_end;
     		else goto avar_unet_hndl_lbl1;
     		}

     	}	
	
	data[0]=LPC_RTC->YEAR;
	data[1]=LPC_RTC->MONTH;
	data[2]=LPC_RTC->DOM;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+16,data);

	data[0]=LPC_RTC->HOUR;
	data[1]=LPC_RTC->MIN;
	data[2]=LPC_RTC->SEC;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+20,data); 
	
	//o_10_s
	if(in==4) {
	   	data[0]=*((char*)(&net_Ustore_max));
		data[1]=*(((char*)(&net_Ustore_max))+1);
	}else{
		data[0]=*((char*)(&net_Ustore));
		data[1]=*(((char*)(&net_Ustore))+1);
	}
	//o_10_e
	data[2]='B';
	data[3]='B';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='B';
	data[1]='B';
	data[2]='B';
	data[3]='B';
	lc640_write_long_ptr(lc640_adr+28,data);	
	
/*	memo_out0[0]=0x55;
     memo_out0[1]=0x00+2;
     memo_out0[2]=1;
     memo_out0[3]=0x0b;
     memo_out0[4]=0xaa;
     memo_out0[5]=0xaa; 
     	
     memo_out0[6]=crc_87(memo_out0,6);
	memo_out0[7]=crc_95(memo_out0,6);
     usart_out_adr0(memo_out0,8); */
     snmp_trap_send("Main power alarm  clear",2,1,1);	
	}
avar_unet_hndl_end:
	__nop();		
}


//-----------------------------------------------
void avar_uout_hndl(char in)
{

char data[4];
unsigned int event_ptr,lc640_adr,event_ptr_find,event_cnt;
char avar_data=0;

if(in==1)
	{
	//net_av=1;

	//beep_init(0x01L,'O');
	//a.av.bAN=1;


	//beep_init(0x33333333,'A');

	if(load_U>U_OUT_KONTR_MAX)uout_av=1;
	if(load_U<U_OUT_KONTR_MIN)uout_av=2;
	
	avar_data=uout_av;
		 
	event_ptr=lc640_read_int(PTR_EVENT_LOG);
	event_ptr++;	
	if(event_ptr>63)event_ptr=0;	
	lc640_write_int(PTR_EVENT_LOG,event_ptr);	
	
    event_cnt=lc640_read_int(CNT_EVENT_LOG);
	if(event_cnt!=63)event_cnt=event_ptr;
	lc640_write_int(CNT_EVENT_LOG,event_cnt); 
	
	lc640_adr=EVENT_LOG+(lc640_read_int(PTR_EVENT_LOG)*32);
	
	data[0]='Q';
	data[1]=0;
	data[2]='A';
	data[3]=avar_data;
	lc640_write_long_ptr(lc640_adr,data);

	 //load_U=4321;

	data[0]=*((char*)&load_U);
	data[1]=*(((char*)&load_U)+1);
	data[2]=0;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+4,data);

	data[0]=LPC_RTC->YEAR;
	data[1]=LPC_RTC->MONTH;
	data[2]=LPC_RTC->DOM;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+8,data);

	data[0]=LPC_RTC->HOUR;
	data[1]=LPC_RTC->MIN;
	data[2]=LPC_RTC->SEC;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+12,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+16,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+28,data);				
	
/*	memo_out0[0]=0x55;
     memo_out0[1]=0x00+2;
     memo_out0[2]=1;
     memo_out0[3]=0x0b;
     memo_out0[4]=0x55;
     memo_out0[5]=0x55; 
     	
     memo_out0[6]=crc_87(memo_out0,6);
	memo_out0[7]=crc_95(memo_out0,6);
     usart_out_adr0(memo_out0,8); 
	*/
	//snmp_trap_send("Main power is off",2,1,0);
	}

else if(in==0)
	{
	uout_av=0;

  /*
	SET_REG(C2GSR,3,24,8);
	C2MOD=0;
	bOUT_FREE2=1;*/


	//C2MOD=0;

	//can2_init(7,8,CANBitrate250k_60MHz);

     //beep_init(0x02L,'O');
	//a_.av.bAN=0;
//	SET_REG(avar_stat,0,0,1);

	//if(AV_OFF_AVT)a.av.bAN=0;
	
	//if(AV_OFF_AVT) SET_REG(avar_ind_stat,0,0,1);

    event_ptr=lc640_read_int(PTR_EVENT_LOG);
	event_ptr_find=event_ptr;
avar_uout_hndl_lbl1:
	lc640_adr=EVENT_LOG+(event_ptr_find*32);

     lc640_read_long_ptr(lc640_adr,data);
     
     if(!((data[0]=='Q')&&(data[1]==0)&&(data[2]=='A')))
     	{        
     	if(event_ptr_find)event_ptr_find--;
     	else event_ptr_find=63;
     	if(event_ptr_find==event_ptr)goto avar_unet_hndl_end;
     	else goto avar_uout_hndl_lbl1;
     	}
     else 
     	{
     	lc640_read_long_ptr(lc640_adr+16,data);
     	if(!((data[0]=='A')&&(data[1]=='A')&&(data[2]=='A')&&(data[3]=='A')))
     		{        
     		if(event_ptr_find)event_ptr_find--;
         		else event_ptr_find=63;
         		if(event_ptr_find==event_ptr)goto avar_unet_hndl_end;
     		else goto avar_uout_hndl_lbl1;
     		}

     	}	
	
	data[0]=LPC_RTC->YEAR;
	data[1]=LPC_RTC->MONTH;
	data[2]=LPC_RTC->DOM;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+16,data);

	data[0]=LPC_RTC->HOUR;
	data[1]=LPC_RTC->MIN;
	data[2]=LPC_RTC->SEC;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+20,data); 
	
	data[0]=*((char*)(&net_Ustore));
	data[1]=*(((char*)(&net_Ustore))+1);
	data[2]='B';
	data[3]='B';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='B';
	data[1]='B';
	data[2]='B';
	data[3]='B';
	lc640_write_long_ptr(lc640_adr+28,data);	
	
/*	memo_out0[0]=0x55;
     memo_out0[1]=0x00+2;
     memo_out0[2]=1;
     memo_out0[3]=0x0b;
     memo_out0[4]=0xaa;
     memo_out0[5]=0xaa; 
     	
     memo_out0[6]=crc_87(memo_out0,6);
	memo_out0[7]=crc_95(memo_out0,6);
     usart_out_adr0(memo_out0,8); */
     snmp_trap_send("Main power is on",2,1,1);	
	}
avar_unet_hndl_end:
	__nop();		
}

//-----------------------------------------------
void avar_bps_hndl(char dev, char v, char in)
{
char data[4];
unsigned short event_ptr,lc640_adr,event_ptr_find,event_cnt;
char avar_simbol;

avar_simbol='T';
if(v==0)avar_simbol='T';
else if(v==1)avar_simbol='U';
else if(v==2)avar_simbol='u';
else if(v==3)avar_simbol='L';

if(in==1)
	{
	//av_src[bps]=1;

//	SET_REG(avar_ind_stat,1,3+dev,1);
//	SET_REG(avar_stat,1,3+dev,1);
	
	if(v==0)bps[dev]._av|=(1<<0);
	else if(v==1) bps[dev]._av|=(1<<1);
	else if(v==2) bps[dev]._av|=(1<<2);
	else if(v==3) bps[dev]._av|=(1<<3);

	bps[dev]._last_avar=v;

	//beep_init(0x33333333,'A');

	event_ptr=lc640_read_int(PTR_EVENT_LOG);
	event_ptr++;	
	if(event_ptr>63)event_ptr=0;	
	lc640_write_int(PTR_EVENT_LOG,event_ptr);	
	
    event_cnt=lc640_read_int(CNT_EVENT_LOG);
	if(event_cnt!=63)event_cnt=event_ptr;
	lc640_write_int(CNT_EVENT_LOG,event_cnt); 
	
	lc640_adr=EVENT_LOG+(lc640_read_int(PTR_EVENT_LOG)*32);
	
	data[0]='S';
	data[1]=dev; 
	data[2]=avar_simbol;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr,data);

	data[0]=0;
	data[1]=0;
	data[2]=0;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+4,data);

	data[0]=LPC_RTC->YEAR;
	data[1]=LPC_RTC->MONTH;
	data[2]=LPC_RTC->DOM;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+8,data);

	data[0]=LPC_RTC->HOUR;
	data[1]=LPC_RTC->MIN;
	data[2]=LPC_RTC->SEC;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+12,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+16,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+28,data);		


	if(dev==0)
		{
		if(v==0)
		snmp_trap_send("BPS #1 Alarm, overheat",4,1,0);
		else if(v==1)
		snmp_trap_send("BPS #1 Alarm, voltage is up",4,1,1);
		else if(v==2)
		snmp_trap_send("BPS #1 Alarm, voltage is down",4,1,2);
		else if(v==3)
		snmp_trap_send("BPS #1 Alarm, connect is lost",4,1,3);
		}
	else if(dev==1)
		{
		if(v==0)
		snmp_trap_send("BPS #2 Alarm, overheat",4,2,0);
		else if(v==1)
		snmp_trap_send("BPS #2 Alarm, voltage is up",4,2,1);
		else if(v==2)
		snmp_trap_send("BPS #2 Alarm, voltage is down",4,2,2);
		else if(v==3)
		snmp_trap_send("BPS #2 Alarm, connect is lost",4,2,3);
		} 
	else if(dev==2)
		{
		if(v==0)
		snmp_trap_send("BPS #3 Alarm, overheat",4,3,0);
		else if(v==1)
		snmp_trap_send("BPS #3 Alarm, voltage is up",4,3,1);
		else if(v==2)
		snmp_trap_send("BPS #3 Alarm, voltage is down",4,3,2);
		else if(v==3)
		snmp_trap_send("BPS #3 Alarm, connect is lost",4,3,3);
		} 	
			
	}

else if(in==0)
	{      
//	av_bps[bps]=0;

	//SET_REG(avar_stat,0,3+bps,1);
	//if(AV_OFF_AVT) SET_REG(avar_ind_stat,0,3+bps,1);

     if(v==0) bps[dev]._av&=(~(1<<0));
     else if(v==1) bps[dev]._av&=(~(1<<1));
	else if(v==2) bps[dev]._av&=(~(1<<2));
	else if(v==3) bps[dev]._av&=(~(1<<3));
     
//	if((bps[dev]._av&0x0f==0))
//	     {
		//SET_REG(avar_stat,0,3+bps,1);
		//if(AV_OFF_AVT) SET_REG(avar_ind_stat,0,3+bps,1);
//	     }
 
	event_ptr=lc640_read_int(PTR_EVENT_LOG);
	event_ptr_find=event_ptr;
	
avar_src_hndl_lbl1: 

	lc640_adr=EVENT_LOG+(event_ptr_find*32);

     lc640_read_long_ptr(lc640_adr,data);
     
     if(!((data[0]=='S')&&(data[1]==dev)&&(data[2]==avar_simbol)))
     	{        
     	if(event_ptr_find)event_ptr_find--;
     	else event_ptr_find=63;
     	if(event_ptr_find==event_ptr)goto avar_src_hndl_end;
     	else goto avar_src_hndl_lbl1;
     	}
     else 
     	{
     	lc640_read_long_ptr(lc640_adr+16,data);
     	if(!((data[0]=='A')&&(data[1]=='A')&&(data[2]=='A')&&(data[3]=='A')))
     		{        
     		if(event_ptr_find)event_ptr_find--;
         		else event_ptr_find=63;
         		if(event_ptr_find==event_ptr)goto avar_src_hndl_end;
     		else goto avar_src_hndl_lbl1;
     		}

     	}	


	
	data[0]=LPC_RTC->YEAR;
	data[1]=LPC_RTC->MONTH;
	data[2]=LPC_RTC->DOM;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+16,data);

	data[0]=LPC_RTC->HOUR;
	data[1]=LPC_RTC->MIN;
	data[2]=LPC_RTC->SEC;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]='B';
	data[1]='B';
	data[2]='B';
	data[3]='B';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='B';
	data[1]='B';
	data[2]='B';
	data[3]='B';
	lc640_write_long_ptr(lc640_adr+28,data);
	
	}
	
avar_src_hndl_end:
__nop();		
}

//-----------------------------------------------
void wrk_mem_hndl(char b)
{
char data[4];
unsigned short event_ptr,lc640_adr/*,event_ptr_find*/,event_cnt;

signed short temp_temp;

if(bat[b]._Iintegr_<5) goto wrk_mem_hndl_end;

temp_temp=bat[b]._u_old[((bat_u_old_cnt+6)&0x07)]; 


event_ptr=lc640_read_int(PTR_EVENT_LOG);
event_ptr++;	
if(event_ptr>63)event_ptr=0;	
lc640_write_int(PTR_EVENT_LOG,event_ptr);	
	
event_cnt=lc640_read_int(CNT_EVENT_LOG);
if(event_cnt!=63)event_cnt=event_ptr;
lc640_write_int(CNT_EVENT_LOG,event_cnt); 
	
lc640_adr=EVENT_LOG+(lc640_read_int(PTR_EVENT_LOG)*32);
	
data[0]='B';
data[1]=b; 
data[2]='W';
data[3]=0;

lc640_write_long_ptr(lc640_adr,data);

data[0]=*((char*)&bat[b]._Iintegr_);
data[1]=*(((char*)(&bat[b]._Iintegr_))+1);
data[2]=*((char*)&temp_temp);
data[3]=*(((char*)(&temp_temp))+1);
lc640_write_long_ptr(lc640_adr+4,data);

lc640_write_long_ptr(lc640_adr+8,(char*)&bat[b]._wrk_date[0]);
	
lc640_write_long_ptr(lc640_adr+12,(char*)&bat[b]._wrk_date[1]);

data[0]=LPC_RTC->YEAR;
data[1]=LPC_RTC->MONTH;
data[2]=LPC_RTC->DOM;
data[3]=0;
lc640_write_long_ptr(lc640_adr+16,data);

data[0]=LPC_RTC->HOUR;
data[1]=LPC_RTC->MIN;
data[2]=LPC_RTC->SEC;
data[3]=0;
lc640_write_long_ptr(lc640_adr+20,data);
	


	
wrk_mem_hndl_end:	
__nop();
}  

#ifdef UKU_220_IPS_TERMOKOMPENSAT
//-----------------------------------------------
void avar_bat_ips_hndl(char in)
{
char data[4];
unsigned short event_ptr,lc640_adr,event_ptr_find,event_cnt;

if(in==1)
	{
	bat_ips[0]._av|=1;
    ips_bat_av_stat[0]=1;
	ips_bat_av_vzvod[0]=1;

	event_ptr=lc640_read_int(PTR_EVENT_LOG);
	event_ptr++;	
	if(event_ptr>63)event_ptr=0;	
	lc640_write_int(PTR_EVENT_LOG,event_ptr);	
	
	event_cnt=lc640_read_int(CNT_EVENT_LOG);
	if(event_cnt!=63)event_cnt=event_ptr;
	lc640_write_int(CNT_EVENT_LOG,event_cnt); 
	
	lc640_adr=EVENT_LOG+(lc640_read_int(PTR_EVENT_LOG)*32);
	
	data[0]='B';
	data[1]=0;
	data[2]='C';
	data[3]=0;
	lc640_write_long_ptr(lc640_adr,data);

	data[0]=0;
	data[1]=0;
	data[2]=0;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+4,data);

	data[0]=LPC_RTC->YEAR;
	data[1]=LPC_RTC->MONTH;
	data[2]=LPC_RTC->DOM;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+8,data);

	data[0]=LPC_RTC->HOUR;
	data[1]=LPC_RTC->MIN;
	data[2]=LPC_RTC->SEC;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+12,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+16,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+28,data);
					
	snmp_trap_send("BAT �1 Alarm, lost",5,1,1);
		
	}

else if(in==0)
	{
	bat_ips[0]._av&=~1;
	ips_bat_av_stat[0]=0;
	ips_bat_av_vzvod[0]=0;
     
	event_ptr=lc640_read_int(PTR_EVENT_LOG);
	event_ptr_find=event_ptr;
	
avar_bat_ips_hndl_lbl1: 

	lc640_adr=EVENT_LOG+(event_ptr_find*32);

     lc640_read_long_ptr(lc640_adr,data);
     
     if(!((data[0]=='B')&&(data[1]==0)&&(data[2]=='C')))
     	{        
     	if(event_ptr_find)event_ptr_find--;
     	else event_ptr_find=63;
     	if(event_ptr_find==event_ptr)goto avar_bat_ips_hndl_end;
     	else goto avar_bat_ips_hndl_lbl1;
     	}
     else 
     	{
     	lc640_read_long_ptr(lc640_adr+16,data);
     	if(!((data[0]=='A')&&(data[1]=='A')&&(data[2]=='A')&&(data[3]=='A')))
     		{        
     		if(event_ptr_find)event_ptr_find--;
         		else event_ptr_find=63;
         	    	if(event_ptr_find==event_ptr)goto avar_bat_ips_hndl_end;
     		else goto avar_bat_ips_hndl_lbl1;
     		}

     	}
     		
	data[0]=LPC_RTC->YEAR;
	data[1]=LPC_RTC->MONTH;
	data[2]=LPC_RTC->DOM;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+16,data);

	data[0]=LPC_RTC->HOUR;
	data[1]=LPC_RTC->MIN;
	data[2]=LPC_RTC->SEC;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]='B';
	data[1]='B';
	data[2]='B';
	data[3]='B';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='B';
	data[1]='B';
	data[2]='B';
	data[3]='B';
	lc640_write_long_ptr(lc640_adr+28,data);
	
	snmp_trap_send("BAT #1 detected",5,1,0);
	
	}
	
avar_bat_ips_hndl_end:
__nop();		
}

//-----------------------------------------------
void avar_bat_ips_hndl2(char in)
{
char data[4];
unsigned short event_ptr,lc640_adr,event_ptr_find,event_cnt;

if(in==1)
	{
	bat_ips[1]._av|=1;
    ips_bat_av_stat[1]=1;
	ips_bat_av_vzvod[1]=1;

	event_ptr=lc640_read_int(PTR_EVENT_LOG);
	event_ptr++;	
	if(event_ptr>63)event_ptr=0;	
	lc640_write_int(PTR_EVENT_LOG,event_ptr);	
	
	event_cnt=lc640_read_int(CNT_EVENT_LOG);
	if(event_cnt!=63)event_cnt=event_ptr;
	lc640_write_int(CNT_EVENT_LOG,event_cnt); 
	
	lc640_adr=EVENT_LOG+(lc640_read_int(PTR_EVENT_LOG)*32);
	
	data[0]='B';
	data[1]=1;
	data[2]='C';
	data[3]=0;
	lc640_write_long_ptr(lc640_adr,data);

	data[0]=0;
	data[1]=0;
	data[2]=0;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+4,data);

	data[0]=LPC_RTC->YEAR;
	data[1]=LPC_RTC->MONTH;
	data[2]=LPC_RTC->DOM;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+8,data);

	data[0]=LPC_RTC->HOUR;
	data[1]=LPC_RTC->MIN;
	data[2]=LPC_RTC->SEC;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+12,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+16,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+28,data);
					
	snmp_trap_send("BAT �2 Alarm, lost",5,2,1);
		
	}

else if(in==0)
	{
	bat_ips[1]._av&=~1;
	ips_bat_av_stat[1]=0;
	ips_bat_av_vzvod[1]=0;
     
	event_ptr=lc640_read_int(PTR_EVENT_LOG);
	event_ptr_find=event_ptr;
	
avar_bat2_ips_hndl_lbl1: 

	lc640_adr=EVENT_LOG+(event_ptr_find*32);

     lc640_read_long_ptr(lc640_adr,data);
     
     if(!((data[0]=='B')&&(data[1]==0)&&(data[2]=='C')))
     	{        
     	if(event_ptr_find)event_ptr_find--;
     	else event_ptr_find=63;
     	if(event_ptr_find==event_ptr)goto avar_bat2_ips_hndl_end;
     	else goto avar_bat2_ips_hndl_lbl1;
     	}
     else 
     	{
     	lc640_read_long_ptr(lc640_adr+16,data);
     	if(!((data[0]=='A')&&(data[1]=='A')&&(data[2]=='A')&&(data[3]=='A')))
     		{        
     		if(event_ptr_find)event_ptr_find--;
         		else event_ptr_find=63;
         	    	if(event_ptr_find==event_ptr)goto avar_bat2_ips_hndl_end;
     		else goto avar_bat2_ips_hndl_lbl1;
     		}

     	}
     		
	data[0]=LPC_RTC->YEAR;
	data[1]=LPC_RTC->MONTH;
	data[2]=LPC_RTC->DOM;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+16,data);

	data[0]=LPC_RTC->HOUR;
	data[1]=LPC_RTC->MIN;
	data[2]=LPC_RTC->SEC;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]='B';
	data[1]='B';
	data[2]='B';
	data[3]='B';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='B';
	data[1]='B';
	data[2]='B';
	data[3]='B';
	lc640_write_long_ptr(lc640_adr+28,data);
	
	snmp_trap_send("BAT #2 detected",5,2,0);
	
	}
	
avar_bat2_ips_hndl_end:
__nop();		
}
#endif

//-----------------------------------------------
void avar_bat_hndl(char b, char in)
{
char data[4];
unsigned short event_ptr,lc640_adr,event_ptr_find,event_cnt;
#ifndef UKU_220_IPS_TERMOKOMPENSAT	 //o_10
if(in==1)
	{
	bat[b]._av|=1;
    
	event_ptr=lc640_read_int(PTR_EVENT_LOG);
	event_ptr++;	
	if(event_ptr>63)event_ptr=0;	
	lc640_write_int(PTR_EVENT_LOG,event_ptr);	
	
     event_cnt=lc640_read_int(CNT_EVENT_LOG);
	if(event_cnt!=63)event_cnt=event_ptr;
	lc640_write_int(CNT_EVENT_LOG,event_cnt); 
	
	lc640_adr=EVENT_LOG+(lc640_read_int(PTR_EVENT_LOG)*32);
	
	data[0]='B';
	data[1]=b;
	data[2]='C';
	data[3]=0;
	lc640_write_long_ptr(lc640_adr,data);

	data[0]=0;
	data[1]=0;
	data[2]=0;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+4,data);

	data[0]=LPC_RTC->YEAR;
	data[1]=LPC_RTC->MONTH;
	data[2]=LPC_RTC->DOM;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+8,data);

	data[0]=LPC_RTC->HOUR;
	data[1]=LPC_RTC->MIN;
	data[2]=LPC_RTC->SEC;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+12,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+16,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+28,data);
					
	if(b==0)
		{
		snmp_trap_send("BAT #1 Alarm, lost",5,1,1);
		}
	else if(b==1)
		{
		snmp_trap_send("BAT #2 Alarm, lost",5,2,1);
		} 
			
	}

else if(in==0)
	{
	bat[b]._av&=~1;

     event_ptr=lc640_read_int(PTR_EVENT_LOG);
	event_ptr_find=event_ptr;
	
avar_bat_hndl_lbl1: 

	lc640_adr=EVENT_LOG+(event_ptr_find*32);

     lc640_read_long_ptr(lc640_adr,data);
     
     if(!((data[0]=='B')&&(data[1]==b)&&(data[2]=='C')))
     	{        
     	if(event_ptr_find)event_ptr_find--;
     	else event_ptr_find=63;
     	if(event_ptr_find==event_ptr)goto avar_bat_hndl_end;
     	else goto avar_bat_hndl_lbl1;
     	}
     else 
     	{
     	lc640_read_long_ptr(lc640_adr+16,data);
     	if(!((data[0]=='A')&&(data[1]=='A')&&(data[2]=='A')&&(data[3]=='A')))
     		{        
     		if(event_ptr_find)event_ptr_find--;
         		else event_ptr_find=63;
         	    	if(event_ptr_find==event_ptr)goto avar_bat_hndl_end;
     		else goto avar_bat_hndl_lbl1;
     		}

     	}
     		
	data[0]=LPC_RTC->YEAR;
	data[1]=LPC_RTC->MONTH;
	data[2]=LPC_RTC->DOM;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+16,data);

	data[0]=LPC_RTC->HOUR;
	data[1]=LPC_RTC->MIN;
	data[2]=LPC_RTC->SEC;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]='B';
	data[1]='B';
	data[2]='B';
	data[3]='B';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='B';
	data[1]='B';
	data[2]='B';
	data[3]='B';
	lc640_write_long_ptr(lc640_adr+28,data);
	
	if(b==0)
		{
		snmp_trap_send("BAT #1 detected",5,1,0);
		}
	else if(b==1)
		{
		snmp_trap_send("BAT #2 detected",5,2,0);
		}
	
	}
	
avar_bat_hndl_end:
__nop();
#endif    //o_10		
}

//-----------------------------------------------
void avar_bat_temper_hndl(char b, char in)
{
char data[4];
unsigned short event_ptr,lc640_adr,event_ptr_find,event_cnt;

if((in==1)||(in==3))
	{
	//bat[b]._av|=1;
    
	event_ptr=lc640_read_int(PTR_EVENT_LOG);
	event_ptr++;	
	if(event_ptr>63)event_ptr=0;	
	lc640_write_int(PTR_EVENT_LOG,event_ptr);	
	
     event_cnt=lc640_read_int(CNT_EVENT_LOG);
	if(event_cnt!=63)event_cnt=event_ptr;
	lc640_write_int(CNT_EVENT_LOG,event_cnt); 
	
	lc640_adr=EVENT_LOG+(lc640_read_int(PTR_EVENT_LOG)*32);
	
	data[0]='B';
	data[1]=b;
	data[2]='T';
	data[3]='1';
	if(in==3)data[3]='3';
	lc640_write_long_ptr(lc640_adr,data);

	data[0]=0;
	data[1]=0;
	data[2]=0;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+4,data);

	data[0]=LPC_RTC->YEAR;
	data[1]=LPC_RTC->MONTH;
	data[2]=LPC_RTC->DOM;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+8,data);

	data[0]=LPC_RTC->HOUR;
	data[1]=LPC_RTC->MIN;
	data[2]=LPC_RTC->SEC;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+12,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+16,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+28,data);
	if(in==1)
		{				
		if(b==0)
			{
			snmp_trap_send("BAT #1 Alarm, is heated",10,1,1);
			}
		else if(b==1)
			{
			snmp_trap_send("BAT #2 Alarm, is heated",10,2,1);
			} 
		}
	if(in==3)
		{				
		if(b==0)
			{
			snmp_trap_send("BAT #1 Alarm, is overheated",11,1,1);
			}
		else if(b==1)
			{
			snmp_trap_send("BAT #2 Alarm, is overheated",11,2,1);
			} 
		}			
	}

else if((in==0)||(in==2))
	{
	char i;

	event_ptr=lc640_read_int(PTR_EVENT_LOG);
	event_ptr_find=event_ptr;

/*	for (i=0;i<63;i++)
		{
		lc640_adr=EVENT_LOG+(event_ptr_find*32);

     	lc640_read_long_ptr(lc640_adr,data);
     
     	if(!((data[0]=='f')&&(data[2]=='Z')))
     		{        
     		if(event_ptr_find)event_ptr_find--;
     		else event_ptr_find=63;
     		if(event_ptr_find==event_ptr)
				{
				lc640_adr=0;
				break;
				}
     		else continue;
     		}
     	else 
     		{
     		lc640_read_long_ptr(lc640_adr+16,data);
     		if(!((data[0]=='A')&&(data[1]=='A')&&(data[2]=='A')&&(data[3]=='A')))
     			{        
     			if(event_ptr_find)event_ptr_find--;
         		else event_ptr_find=63;
         		if(event_ptr_find==event_ptr)
					{
					lc640_adr=0;
					break;
					}
     			else continue;
     			}
     		}	
		}*/
	
	for (i=0;i<63;i++)
		{
		char temp='1';
		if(in==2)temp='3';

		lc640_adr=EVENT_LOG+(event_ptr_find*32);

     	lc640_read_long_ptr(lc640_adr,data);
     
     	if(!((data[0]=='B')&&(data[1]==b)&&(data[2]=='T')&&(data[3]==temp)))
     		{        
     		if(event_ptr_find)event_ptr_find--;
     		else event_ptr_find=63;
     		if(event_ptr_find==event_ptr)				{
				lc640_adr=0;
				break;
				}
     		else continue;
     		}
	     else 
	     	{
	     	lc640_read_long_ptr(lc640_adr+16,data);
	     	if(!((data[0]=='A')&&(data[1]=='A')&&(data[2]=='A')&&(data[3]=='A')))
	     		{        
	     		if(event_ptr_find)event_ptr_find--;
	         	else event_ptr_find=63;
	         	if(event_ptr_find==event_ptr)				{
					lc640_adr=0;
					break;
					}
	     		else continue;
	     		}
	
	     	}
		}
     		
	data[0]=LPC_RTC->YEAR;
	data[1]=LPC_RTC->MONTH;
	data[2]=LPC_RTC->DOM;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+16,data);

	data[0]=LPC_RTC->HOUR;
	data[1]=LPC_RTC->MIN;
	data[2]=LPC_RTC->SEC;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]='B';
	data[1]='B';
	data[2]='B';
	data[3]='B';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='B';
	data[1]='B';
	data[2]='B';
	data[3]='B';
	lc640_write_long_ptr(lc640_adr+28,data);
	
	if(in==0)
		{				
		if(b==0)
			{
			snmp_trap_send("BAT #1 Alarm, heating eliminated",10,1,0);
			}
		else if(b==1)
			{
			snmp_trap_send("BAT #2 Alarm, heating eliminated",10,2,0);
			} 
		}
	if(in==2)
		{				
		if(b==0)
			{
			snmp_trap_send("BAT #1 Alarm, overheating eliminated",11,1,0);
			}
		else if(b==1)
			{
			snmp_trap_send("BAT #2 Alarm, overheating eliminated",11,2,0);
			} 
		}		
	
	}
	
}

//-----------------------------------------------
void avar_bat_as_hndl(char b, char in)
{
char data[4];
unsigned short event_ptr,lc640_adr,event_ptr_find,event_cnt;

if(in==1)
	{
	bat[b]._av|=2;
    
	event_ptr=lc640_read_int(PTR_EVENT_LOG);
	event_ptr++;	
	if(event_ptr>63)event_ptr=0;	
	lc640_write_int(PTR_EVENT_LOG,event_ptr);	
	
     event_cnt=lc640_read_int(CNT_EVENT_LOG);
	if(event_cnt!=63)event_cnt=event_ptr;
	lc640_write_int(CNT_EVENT_LOG,event_cnt); 
	
	lc640_adr=EVENT_LOG+(lc640_read_int(PTR_EVENT_LOG)*32);
	
	data[0]='B';
	data[1]=b;
	data[2]='S';
	data[3]=0;
	lc640_write_long_ptr(lc640_adr,data);

	data[0]=0;
	data[1]=0;
	data[2]=0;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+4,data);

	data[0]=LPC_RTC->YEAR;
	data[1]=LPC_RTC->MONTH;
	data[2]=LPC_RTC->DOM;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+8,data);

	data[0]=LPC_RTC->HOUR;
	data[1]=LPC_RTC->MIN;
	data[2]=LPC_RTC->SEC;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+12,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+16,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+28,data);				

	if(b==0)
		{
		snmp_trap_send("BAT #1 Alarm, asymmetry",5,1,2);
		}
	else if(b==1)
		{
		snmp_trap_send("BAT #2 Alarm, asymmetry",5,2,2);
		} 
	
	}

else if(in==0)
	{
	bat[b]._av&=~(1<<1);

     event_ptr=lc640_read_int(PTR_EVENT_LOG);
	event_ptr_find=event_ptr;
	
avar_bat_as_hndl_lbl1: 

	lc640_adr=EVENT_LOG+(event_ptr_find*32);

     lc640_read_long_ptr(lc640_adr,data);
     
     if(!((data[0]=='B')&&(data[1]==b)&&(data[2]=='C')))
     	{        
     	if(event_ptr_find)event_ptr_find--;
     	else event_ptr_find=63;
     	if(event_ptr_find==event_ptr)goto avar_bat_as_hndl_end;
     	else goto avar_bat_as_hndl_lbl1;
     	}
     else 
     	{
     	lc640_read_long_ptr(lc640_adr+16,data);
     	if(!((data[0]=='A')&&(data[1]=='A')&&(data[2]=='A')&&(data[3]=='A')))
     		{        
     		if(event_ptr_find)event_ptr_find--;
         		else event_ptr_find=63;
         	    	if(event_ptr_find==event_ptr)goto avar_bat_as_hndl_end;
     		else goto avar_bat_as_hndl_lbl1;
     		}

     	}
     		
	data[0]=LPC_RTC->YEAR;
	data[1]=LPC_RTC->MONTH;
	data[2]=LPC_RTC->DOM;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+16,data);

	data[0]=LPC_RTC->HOUR;
	data[1]=LPC_RTC->MIN;
	data[2]=LPC_RTC->SEC;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]='B';
	data[1]='B';
	data[2]='B';
	data[3]='B';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='B';
	data[1]='B';
	data[2]='B';
	data[3]='B';
	lc640_write_long_ptr(lc640_adr+28,data);
	
	if(b==0)
		{
		snmp_trap_send("BAT #1 Asymmetry alarm clear",5,1,0);
		}
	else if(b==1)
		{
		snmp_trap_send("BAT #2 Asymmetry alarm clear",5,2,0);
		}
	
	}
	
avar_bat_as_hndl_end:
__nop();		
}

//-----------------------------------------------
void ke_zvu_mem_hndl(char b,unsigned short in,unsigned short in1)
{
char data[4];
unsigned int event_ptr=0,lc640_adr/*,event_ptr_find*/,event_cnt;
//unsigned char temp,temp_;
//unsigned int tempUI;
//unsigned long tempUL; 
signed temp_temp;

temp_temp=bat[b]._u_old[bat_u_old_cnt+1]; 

event_ptr=lc640_read_int(PTR_EVENT_LOG);
event_ptr++;	
if(event_ptr>63)event_ptr=0;	
lc640_write_int(PTR_EVENT_LOG,event_ptr);	
	
event_cnt=lc640_read_int(CNT_EVENT_LOG);
if(event_cnt!=63)event_cnt=event_ptr;
lc640_write_int(CNT_EVENT_LOG,event_cnt); 
	
lc640_adr=EVENT_LOG+(lc640_read_int(PTR_EVENT_LOG)*32);
	
data[0]='B';
data[1]=b; 
data[2]='K';
data[3]='Z';

lc640_write_long_ptr(lc640_adr,data);

data[0]=*((char*)&in);
data[1]=*(((char*)(&in))+1);
data[2]=*((char*)&temp_temp);
data[3]=*(((char*)(&temp_temp))+1);
lc640_write_long_ptr(lc640_adr+4,data);

ke_date[0]=lc640_read_long(EE_SPC_KE_DATE0);
lc640_write_long_ptr(lc640_adr+8,(char*)&ke_date[0]);
ke_date[1]=lc640_read_long(EE_SPC_KE_DATE1);	
lc640_write_long_ptr(lc640_adr+12,(char*)&ke_date[1]);

data[0]=LPC_RTC->YEAR;
data[1]=LPC_RTC->MONTH;
data[2]=LPC_RTC->DOM;
data[3]=*((char*)&in1);
lc640_write_long_ptr(lc640_adr+16,data);

data[0]=LPC_RTC->HOUR;
data[1]=LPC_RTC->MIN;
data[2]=LPC_RTC->SEC;
data[3]=*(((char*)(&in1))+1);
lc640_write_long_ptr(lc640_adr+20,data);
 
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

temp_temp=bat[b]._u_old[bat_u_old_cnt+1]; 

event_ptr=lc640_read_int(PTR_EVENT_LOG);
event_ptr++;	
if(event_ptr>63)event_ptr=0;	
lc640_write_int(PTR_EVENT_LOG,event_ptr);	
	
event_cnt=lc640_read_int(CNT_EVENT_LOG);
if(event_cnt!=63)event_cnt=event_ptr;
lc640_write_int(CNT_EVENT_LOG,event_cnt); 
	
lc640_adr=EVENT_LOG+(lc640_read_int(PTR_EVENT_LOG)*32);
	
data[0]='B';
data[1]=b; 
data[2]='K';
data[3]=0;

lc640_write_long_ptr(lc640_adr,data);

data[0]=*((char*)&in);
data[1]=*(((char*)(&in))+1);
data[2]=*((char*)&temp_temp);
data[3]=*(((char*)(&temp_temp))+1);
lc640_write_long_ptr(lc640_adr+4,data);

ke_date[0]=lc640_read_long(EE_SPC_KE_DATE0);
lc640_write_long_ptr(lc640_adr+8,(char*)&ke_date[0]);
ke_date[1]=lc640_read_long(EE_SPC_KE_DATE1);	
lc640_write_long_ptr(lc640_adr+12,(char*)&ke_date[1]);

data[0]=LPC_RTC->YEAR;
data[1]=LPC_RTC->MONTH;
data[2]=LPC_RTC->DOM;
data[3]=0;
lc640_write_long_ptr(lc640_adr+16,data);

data[0]=LPC_RTC->HOUR;
data[1]=LPC_RTC->MIN;
data[2]=LPC_RTC->SEC;
data[3]=0;
lc640_write_long_ptr(lc640_adr+20,data);
 
}


//-----------------------------------------------
void vz_mem_hndl(unsigned short in)
{
char data[4];
unsigned int event_ptr=0,lc640_adr,event_ptr_find=0,event_cnt;
//char avar_simbol;

if(in==1)
	{
	event_ptr=lc640_read_int(PTR_EVENT_LOG);
	event_ptr++;	
	if(event_ptr>63)event_ptr=0;	
	lc640_write_int(PTR_EVENT_LOG,event_ptr);	
	
     event_cnt=lc640_read_int(CNT_EVENT_LOG);
	if(event_cnt!=63)event_cnt=event_ptr;
	lc640_write_int(CNT_EVENT_LOG,event_cnt); 
	
	lc640_adr=EVENT_LOG+(lc640_read_int(PTR_EVENT_LOG)*32);
	
	data[0]='B';
	data[1]=0; 
	data[2]='Z';
	data[3]=0;
	lc640_write_long_ptr(lc640_adr,data);

	data[0]=0;
	data[1]=0;
	data[2]=0;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+4,data);

	data[0]=LPC_RTC->YEAR;
	data[1]=LPC_RTC->MONTH;
	data[2]=LPC_RTC->DOM;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+8,data);

	data[0]=LPC_RTC->HOUR;
	data[1]=LPC_RTC->MIN;
	data[2]=LPC_RTC->SEC;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+12,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+16,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+28,data);		
 	
	}

else if(in==0)
	{      
     
vz_mem_hndl_lbl1: 

	lc640_adr=EVENT_LOG+(event_ptr_find*32);

     lc640_read_long_ptr(lc640_adr,data);
     
     if(!((data[0]=='B')&&(data[2]=='Z')))
     	{        
     	if(event_ptr_find)event_ptr_find--;
     	else event_ptr_find=63;
     	if(event_ptr_find==event_ptr)goto vz_mem_hndl_end;
     	else goto vz_mem_hndl_lbl1;
     	}
     else 
     	{
     	lc640_read_long_ptr(lc640_adr+16,data);
     	if(!((data[0]=='A')&&(data[1]=='A')&&(data[2]=='A')&&(data[3]=='A')))
     		{        
     		if(event_ptr_find)event_ptr_find--;
         		else event_ptr_find=63;
         		if(event_ptr_find==event_ptr)goto vz_mem_hndl_end;
     		else goto vz_mem_hndl_lbl1;
     		}

     	}	


	
	data[0]=LPC_RTC->YEAR;
	data[1]=LPC_RTC->MONTH;
	data[2]=LPC_RTC->DOM;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+16,data);

	data[0]=LPC_RTC->HOUR;
	data[1]=LPC_RTC->MIN;
	data[2]=LPC_RTC->SEC;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]='B';
	data[1]='B';
	data[2]='B';
	data[3]='B';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='B';
	data[1]='B';
	data[2]='B';
	data[3]='B';
	lc640_write_long_ptr(lc640_adr+28,data);
	
	}

vz_mem_hndl_end:
__nop(); 

}

//-----------------------------------------------
void speedz_mem_hndl(unsigned short in)	 //������� �� ���������� ������ (1-�����, 5-�������������� �����, 0-������� ����������, 10-������� �������)
{
char data[4];
unsigned int event_ptr=0,lc640_adr,event_ptr_find=0,event_cnt;
//char avar_simbol;

if((in==1)||(in==5)) //����� ����������� ������
	{
	event_ptr=lc640_read_int(PTR_EVENT_LOG);
	event_ptr++;	
	if(event_ptr>63)event_ptr=0;	
	lc640_write_int(PTR_EVENT_LOG,event_ptr);	
	
    event_cnt=lc640_read_int(CNT_EVENT_LOG);
	if(event_cnt!=63)event_cnt=event_ptr;
	lc640_write_int(CNT_EVENT_LOG,event_cnt); 
	
	lc640_adr=EVENT_LOG+(lc640_read_int(PTR_EVENT_LOG)*32);
	
	data[0]='s';
	data[1]=0;
	if(in==5)data[1]='a'; 
	data[2]='Z';
	data[3]=0;
	lc640_write_long_ptr(lc640_adr,data);

	data[0]=0;
	data[1]=0;
	data[2]=0;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+4,data);

	data[0]=LPC_RTC->YEAR;
	data[1]=LPC_RTC->MONTH;
	data[2]=LPC_RTC->DOM;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+8,data);

	data[0]=LPC_RTC->HOUR;
	data[1]=LPC_RTC->MIN;
	data[2]=LPC_RTC->SEC;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+12,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+16,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+28,data);		
 	
	}

else if(in==0)	//������� ����������
	{      
	char i;     
	event_ptr_find=0;
	for (i=0;i<63;i++)
		{
		lc640_adr=EVENT_LOG+(event_ptr_find*32);

     	lc640_read_long_ptr(lc640_adr,data);
     
     	if(!((data[0]=='s')&&(data[2]=='Z')))
     		{        
     		if(event_ptr_find)event_ptr_find--;
     		else event_ptr_find=63;
     		if(event_ptr_find==event_ptr)
				{
				lc640_adr=0;
				break;
				}
     		else continue;
     		}
     	else 
     		{
     		lc640_read_long_ptr(lc640_adr+16,data);
     		if(!((data[0]=='A')&&(data[1]=='A')&&(data[2]=='A')&&(data[3]=='A')))
     			{        
     			if(event_ptr_find)event_ptr_find--;
         		else event_ptr_find=63;
         		if(event_ptr_find==event_ptr)
					{
					lc640_adr=0;
					break;
					}
     			else continue;
     			}
     		}	
		}
	if(lc640_adr!=0)
		{
		data[0]=LPC_RTC->YEAR;
		data[1]=LPC_RTC->MONTH;
		data[2]=LPC_RTC->DOM;
		data[3]=0;
		lc640_write_long_ptr(lc640_adr+16,data);
	
		data[0]=LPC_RTC->HOUR;
		data[1]=LPC_RTC->MIN;
		data[2]=LPC_RTC->SEC;
		data[3]=0;
		lc640_write_long_ptr(lc640_adr+20,data);
		
		data[0]='C';
		data[1]='C';
		data[2]='C';
		data[3]='C';
		lc640_write_long_ptr(lc640_adr+24,data);
		
		data[0]='C';
		data[1]='C';
		data[2]='C';
		data[3]='C';
		lc640_write_long_ptr(lc640_adr+28,data);
		}
	}
else if(in==10) 	// �������� ����������
	{      
	char i;     

	event_ptr_find=0;
	for (i=0;i<63;i++)
		{
		lc640_adr=EVENT_LOG+(event_ptr_find*32);

     	lc640_read_long_ptr(lc640_adr,data);
     
     	if(!((data[0]=='s')&&(data[2]=='Z')))
     		{        
     		if(event_ptr_find)event_ptr_find--;
     		else event_ptr_find=63;
     		if(event_ptr_find==event_ptr)
				{
				lc640_adr=0;
				break;
				}
     		else continue;
     		}
     	else 
     		{
     		lc640_read_long_ptr(lc640_adr+16,data);
     		if(!((data[0]=='A')&&(data[1]=='A')&&(data[2]=='A')&&(data[3]=='A')))
     			{        
     			if(event_ptr_find)event_ptr_find--;
         		else event_ptr_find=63;
         		if(event_ptr_find==event_ptr)
					{
					lc640_adr=0;
					break;
					}
     			else continue;
     			}
     		}	
		}
	if(lc640_adr!=0)
		{
		data[0]=LPC_RTC->YEAR;
		data[1]=LPC_RTC->MONTH;
		data[2]=LPC_RTC->DOM;
		data[3]=0;
		lc640_write_long_ptr(lc640_adr+16,data);
	
		data[0]=LPC_RTC->HOUR;
		data[1]=LPC_RTC->MIN;
		data[2]=LPC_RTC->SEC;
		data[3]=0;
		lc640_write_long_ptr(lc640_adr+20,data);
		
		data[0]='B';
		data[1]='B';
		data[2]='B';
		data[3]='B';
		lc640_write_long_ptr(lc640_adr+24,data);
		
		data[0]='B';
		data[1]='B';
		data[2]='B';
		data[3]='B';
		lc640_write_long_ptr(lc640_adr+28,data);
		}
	}
}
  

//-----------------------------------------------
void uz_mem_hndl(unsigned short in)	 //������� �� ������������� ������
{
char data[4];
unsigned int event_ptr=0,lc640_adr,event_ptr_find=0,event_cnt;
//char avar_simbol;

if(in==1) //����� �������������� ������
	{
	event_ptr=lc640_read_int(PTR_EVENT_LOG);
	event_ptr++;	
	if(event_ptr>63)event_ptr=0;	
	lc640_write_int(PTR_EVENT_LOG,event_ptr);	
	
    event_cnt=lc640_read_int(CNT_EVENT_LOG);
	if(event_cnt!=63)event_cnt=event_ptr;
	lc640_write_int(CNT_EVENT_LOG,event_cnt); 
	
	lc640_adr=EVENT_LOG+(lc640_read_int(PTR_EVENT_LOG)*32);
	
	data[0]='u';
	data[1]=0; 
	data[2]='Z';
	data[3]=0;
	lc640_write_long_ptr(lc640_adr,data);

	data[0]=0;
	data[1]=0;
	data[2]=0;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+4,data);

	data[0]=LPC_RTC->YEAR;
	data[1]=LPC_RTC->MONTH;
	data[2]=LPC_RTC->DOM;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+8,data);

	data[0]=LPC_RTC->HOUR;
	data[1]=LPC_RTC->MIN;
	data[2]=LPC_RTC->SEC;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+12,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+16,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+28,data);		
 	
	}

else if(in==0)	//������� ����������
	{      
	char i;     
	event_ptr_find=0;
	for (i=0;i<63;i++)
		{
		lc640_adr=EVENT_LOG+(event_ptr_find*32);

     	lc640_read_long_ptr(lc640_adr,data);
     
     	if(!((data[0]=='u')&&(data[2]=='Z')))
     		{        
     		if(event_ptr_find)event_ptr_find--;
     		else event_ptr_find=63;
     		if(event_ptr_find==event_ptr)
				{
				lc640_adr=0;
				break;
				}
     		else continue;
     		}
     	else 
     		{
     		lc640_read_long_ptr(lc640_adr+16,data);
     		if(!((data[0]=='A')&&(data[1]=='A')&&(data[2]=='A')&&(data[3]=='A')))
     			{        
     			if(event_ptr_find)event_ptr_find--;
         		else event_ptr_find=63;
         		if(event_ptr_find==event_ptr)
					{
					lc640_adr=0;
					break;
					}
     			else continue;
     			}
     		}	
		}
	if(lc640_adr!=0)
		{
		data[0]=LPC_RTC->YEAR;
		data[1]=LPC_RTC->MONTH;
		data[2]=LPC_RTC->DOM;
		data[3]=0;
		lc640_write_long_ptr(lc640_adr+16,data);
	
		data[0]=LPC_RTC->HOUR;
		data[1]=LPC_RTC->MIN;
		data[2]=LPC_RTC->SEC;
		data[3]=0;
		lc640_write_long_ptr(lc640_adr+20,data);
		
		data[0]='C';
		data[1]='C';
		data[2]='C';
		data[3]='C';
		lc640_write_long_ptr(lc640_adr+24,data);
		
		data[0]='C';
		data[1]='C';
		data[2]='C';
		data[3]='C';
		lc640_write_long_ptr(lc640_adr+28,data);
		}
	}
else if(in==10) 	// �������� ����������
	{      
	char i;     

	event_ptr_find=0;
	for (i=0;i<63;i++)
		{
		lc640_adr=EVENT_LOG+(event_ptr_find*32);

     	lc640_read_long_ptr(lc640_adr,data);
     
     	if(!((data[0]=='u')&&(data[2]=='Z')))
     		{        
     		if(event_ptr_find)event_ptr_find--;
     		else event_ptr_find=63;
     		if(event_ptr_find==event_ptr)
				{
				lc640_adr=0;
				break;
				}
     		else continue;
     		}
     	else 
     		{
     		lc640_read_long_ptr(lc640_adr+16,data);
     		if(!((data[0]=='A')&&(data[1]=='A')&&(data[2]=='A')&&(data[3]=='A')))
     			{        
     			if(event_ptr_find)event_ptr_find--;
         		else event_ptr_find=63;
         		if(event_ptr_find==event_ptr)
					{
					lc640_adr=0;
					break;
					}
     			else continue;
     			}
     		}	
		}
	if(lc640_adr!=0)
		{
		data[0]=LPC_RTC->YEAR;
		data[1]=LPC_RTC->MONTH;
		data[2]=LPC_RTC->DOM;
		data[3]=0;
		lc640_write_long_ptr(lc640_adr+16,data);
	
		data[0]=LPC_RTC->HOUR;
		data[1]=LPC_RTC->MIN;
		data[2]=LPC_RTC->SEC;
		data[3]=0;
		lc640_write_long_ptr(lc640_adr+20,data);
		
		data[0]='B';
		data[1]='B';
		data[2]='B';
		data[3]='B';
		lc640_write_long_ptr(lc640_adr+24,data);
		
		data[0]='B';
		data[1]='B';
		data[2]='B';
		data[3]='B';
		lc640_write_long_ptr(lc640_adr+28,data);
		}
	}
}

//-----------------------------------------------
void fz_mem_hndl(unsigned short in)	 //������� �� ����������� ������
{
char data[4];
unsigned int event_ptr=0,lc640_adr,event_ptr_find=0,event_cnt;
//char avar_simbol;

if(in==1) //����� ������������ ������
	{
	event_ptr=lc640_read_int(PTR_EVENT_LOG);
	event_ptr++;	
	if(event_ptr>63)event_ptr=0;	
	lc640_write_int(PTR_EVENT_LOG,event_ptr);	
	
    event_cnt=lc640_read_int(CNT_EVENT_LOG);
	if(event_cnt!=63)event_cnt=event_ptr;
	lc640_write_int(CNT_EVENT_LOG,event_cnt); 
	
	lc640_adr=EVENT_LOG+(lc640_read_int(PTR_EVENT_LOG)*32);
	
	data[0]='f';
	data[1]=0; 
	data[2]='Z';
	data[3]=0;
	lc640_write_long_ptr(lc640_adr,data);

	data[0]=0;
	data[1]=0;
	data[2]=0;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+4,data);

	data[0]=LPC_RTC->YEAR;
	data[1]=LPC_RTC->MONTH;
	data[2]=LPC_RTC->DOM;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+8,data);

	data[0]=LPC_RTC->HOUR;
	data[1]=LPC_RTC->MIN;
	data[2]=LPC_RTC->SEC;
	data[3]=0;
	lc640_write_long_ptr(lc640_adr+12,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+16,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+20,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+24,data);
	
	data[0]='A';
	data[1]='A';
	data[2]='A';
	data[3]='A';
	lc640_write_long_ptr(lc640_adr+28,data);		
 	
	}

else if(in==0)	//������� ����������
	{      
	char i;     
	event_ptr_find=0;
	for (i=0;i<63;i++)
		{
		lc640_adr=EVENT_LOG+(event_ptr_find*32);

     	lc640_read_long_ptr(lc640_adr,data);
     
     	if(!((data[0]=='f')&&(data[2]=='Z')))
     		{        
     		if(event_ptr_find)event_ptr_find--;
     		else event_ptr_find=63;
     		if(event_ptr_find==event_ptr)
				{
				lc640_adr=0;
				break;
				}
     		else continue;
     		}
     	else 
     		{
     		lc640_read_long_ptr(lc640_adr+16,data);
     		if(!((data[0]=='A')&&(data[1]=='A')&&(data[2]=='A')&&(data[3]=='A')))
     			{        
     			if(event_ptr_find)event_ptr_find--;
         		else event_ptr_find=63;
         		if(event_ptr_find==event_ptr)
					{
					lc640_adr=0;
					break;
					}
     			else continue;
     			}
     		}	
		}
	if(lc640_adr!=0)
		{
		data[0]=LPC_RTC->YEAR;
		data[1]=LPC_RTC->MONTH;
		data[2]=LPC_RTC->DOM;
		data[3]=0;
		lc640_write_long_ptr(lc640_adr+16,data);
	
		data[0]=LPC_RTC->HOUR;
		data[1]=LPC_RTC->MIN;
		data[2]=LPC_RTC->SEC;
		data[3]=0;
		lc640_write_long_ptr(lc640_adr+20,data);
		
		data[0]='C';
		data[1]='C';
		data[2]='C';
		data[3]='C';
		lc640_write_long_ptr(lc640_adr+24,data);
		
		data[0]='C';
		data[1]='C';
		data[2]='C';
		data[3]='C';
		lc640_write_long_ptr(lc640_adr+28,data);
		}
	}
else if(in==10) 	// �������� ����������
	{      
	char i;     

	event_ptr_find=0;
	for (i=0;i<63;i++)
		{
		lc640_adr=EVENT_LOG+(event_ptr_find*32);

     	lc640_read_long_ptr(lc640_adr,data);
     
     	if(!((data[0]=='f')&&(data[2]=='Z')))
     		{        
     		if(event_ptr_find)event_ptr_find--;
     		else event_ptr_find=63;
     		if(event_ptr_find==event_ptr)
				{
				lc640_adr=0;
				break;
				}
     		else continue;
     		}
     	else 
     		{
     		lc640_read_long_ptr(lc640_adr+16,data);
     		if(!((data[0]=='A')&&(data[1]=='A')&&(data[2]=='A')&&(data[3]=='A')))
     			{        
     			if(event_ptr_find)event_ptr_find--;
         		else event_ptr_find=63;
         		if(event_ptr_find==event_ptr)
					{
					lc640_adr=0;
					break;
					}
     			else continue;
     			}
     		}	
		}
	if(lc640_adr!=0)
		{
		data[0]=LPC_RTC->YEAR;
		data[1]=LPC_RTC->MONTH;
		data[2]=LPC_RTC->DOM;
		data[3]=0;
		lc640_write_long_ptr(lc640_adr+16,data);
	
		data[0]=LPC_RTC->HOUR;
		data[1]=LPC_RTC->MIN;
		data[2]=LPC_RTC->SEC;
		data[3]=0;
		lc640_write_long_ptr(lc640_adr+20,data);
		
		data[0]='B';
		data[1]='B';
		data[2]='B';
		data[3]='B';
		lc640_write_long_ptr(lc640_adr+24,data);
		
		data[0]='B';
		data[1]='B';
		data[2]='B';
		data[3]='B';
		lc640_write_long_ptr(lc640_adr+28,data);
		}
	}
}
