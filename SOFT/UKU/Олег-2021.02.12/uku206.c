#include "lcd_AGM1232_uku206.h"
#include "uku206.h"
#include "simbol.h"
#include "25lc640.h"
#include "Timer.h"
#include "gran.h"
#include "uart0.h"
#include "uart1.h"
#include "cmd.h"
#include "ret.h"
#include "eeprom_map.h"
#include "common_func.h"
#include "pcf8563.h"
#include "control.h"
#include "mess.h"
#include "full_can.h"
#include "watchdog.h"
#include "ad7705.h"
#include "beep.h"
#include "avar_hndl.h"
#include "memo.h"

//***********************************************
//������
char b1000Hz,b100Hz,b50Hz,b10Hz,b5Hz,b2Hz,b1Hz;
short t0cnt,t0cnt0,t0cnt1,t0cnt2,t0cnt3,t0cnt4,t0cnt5,t0cnt6,t0_cnt7;
char bFL5,bFL2,bFL,bFL_;
signed short main_10Hz_cnt=0;
signed short main_1Hz_cnt=0;

 
//***********************************************
//��������� �����
char cnt_of_slave=3;
//char cnt_of_wrks;   //����������� ���������� ���������� , ��� ���������



//**********************************************
//������������, ������������ �� EEPROM
signed short Ktsrc[2];
signed short Kusrc[2];
signed short Kisrc[2];
signed short Ki0src[2];
signed short Kubat[2];
unsigned short Kibat0[2];
signed short Kibat1[2];
signed short Ktbat[2];
signed short Kunet;
signed short Ktout[3];
signed short Kuload;

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
signed short KVZ;
signed short IMAX;
signed short KIMAX;
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

signed short NUMBAT;
signed short NUMIST;
signed short NUMINV;
signed short NUMDT;
signed short NUMSK;
signed short NUMEXT;

enum_apv_on APV_ON1,APV_ON2;
signed short APV_ON2_TIME;

enum_bat_is_on BAT_IS_ON[2];
signed short BAT_DAY_OF_ON[2];
signed short BAT_MONTH_OF_ON[2];
signed short BAT_YEAR_OF_ON[2];
signed short BAT_C_NOM[2];
signed short BAT_RESURS[2];
signed short BAT_C_REAL[2];

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


//***********************************************
//��������� �������
BAT_STAT bat[2];
signed short		bat_u_old_cnt;

//***********************************************
//��������� ����������
BPS_STAT bps[12];

//***********************************************
//��������� ��������
signed short load_U;
signed short load_I;


//***********************************************
//���������

char lcd_buffer[LCD_SIZE+100]={"Hello World"};
signed char parol[3];
char phase;
char lcd_bitmap[1024];
char dig[5];
char zero_on;
char mnemo_cnt=50;
char simax;
short av_j_si_max;
const char ABCDEF[]={"0123456789ABCDEF"};
const char sm_mont[13][4]={"   ","���","���","���","���","���","���","���","���","���","���","���","���"}; //
signed short ptr_ind=0;
stuct_ind a_ind,b_ind[10];
signed short ind_pointer=0;

//***********************************************
//��������� ��������� ����
signed short net_U,net_Ustore;
char bFF,bFF_;
signed short net_F,hz_out,hz_out_cnt;
signed char unet_drv_cnt;
char net_av;

//***********************************************
//��������� ������� ��������
//signed short tout[4];
char tout_max_cnt[4],tout_min_cnt[4];
enum {tNORM,tMAX,tMIN}tout_stat[4];
signed short T_EXT[3];
BOOL ND_EXT[3];

//***********************************************
//�����
extern char beep_cnt;









signed short u_necc,u_necc_;
signed short main_cnt_5Hz;
signed short num_necc;
signed short num_necc_Imax;
signed short num_necc_Imin;
signed short cnt_num_necc;
//char bSAME_IST_ON;
signed mat_temper;

//***********************************************
//���
unsigned main_apv_cnt,hour_apv_cnt[2],reset_apv_cnt[2];
unsigned short apv_cnt_sec[2],apv_cnt[2];

//***********************************************
//��������� ���������
const char sm_[]	={"                    "};
const char sm_exit[]={" �����              "};
const char sm_time[]={" 0%:0^:0& 0</>  /0{ "};





//**********************************************
//������ � �������� 
char but;                            
unsigned long but_n,but_s;
char but0_cnt;
char but1_cnt;
char but_onL_temp;

//***********************************************
//����������� �����
char cnt_net_drv;

//***********************************************
//��� 
extern char ptr_can1_tx_wr,ptr_can1_tx_rd;
extern char ptr_can2_tx_wr,ptr_can2_tx_rd;
extern unsigned short rotor_can[6];
extern char RXBUFF[40],TXBUFF[40];





//***********************************************
//������ � ��������
char speed,l_but,n_but;

//***********************************************
//�������������
enum {wrkON=0x55,wrkOFF=0xAA}wrk;
char cnt_wrk;
signed short ibat_integr;
unsigned short av_beep,av_rele,av_stat;
char default_temp;
enum {tstOFF,tstON,tstU} tst_state[10];
char ND_out[3];

//***********************************************
//���
extern short adc_buff[16][16],adc_buff_[16];
extern char adc_cnt,adc_cnt1,adc_ch;

//***********************************************

char flag=0;


extern signed short bat_ver_cnt;
signed short Isumm;
signed short Isumm_;

#include <LPC21xx.H>                        /* LPC21xx definitions */



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
//���������������
extern signed short samokalibr_cnt;

//**********************************************
//���������
extern char mess[MESS_DEEP],mess_old[MESS_DEEP],mess_cnt[MESS_DEEP];
extern short mess_par0[MESS_DEEP],mess_par1[MESS_DEEP],mess_data[2];


//**********************************************
//�������� ������� �������
extern signed short 	main_kb_cnt;
extern signed short 	kb_cnt_1lev;
extern signed short 	kb_cnt_2lev;
extern char 			kb_full_ver;
extern char 			kb_start[2];

//***********************************************
//���������� �����
extern signed short cntrl_stat;
extern signed short cntrl_stat_old;
extern signed short cntrl_stat_new;
extern signed short Ibmax;





char plazma_plazma_plazma;







//-----------------------------------------------
void def_set(int umax__,int ub0__,int ub20__,int usign__,int imax__,int uob__,int numi)
{
;
lc640_write_int(EE_NUMIST,numi);
lc640_write_int(EE_MAIN_IST,0);
lc640_write_int(EE_UMAX,umax__);
lc640_write_int(EE_UB0,ub0__);
lc640_write_int(EE_UB20,ub20__);
lc640_write_int(EE_TSIGN,70);
lc640_write_int(EE_TMAX,80);
//lc640_write_int(EE_C_BAT,180);
lc640_write_int(EE_USIGN,usign__);
lc640_write_int(EE_UMN,187);
lc640_write_int(EE_ZV_ON,0xff);
lc640_write_int(EE_IKB,5);
lc640_write_int(EE_KVZ,1030);
lc640_write_int(EE_IMAX,imax__);
lc640_write_int(EE_KIMAX,8);
lc640_write_int(EE_APV_ON,0xff);
lc640_write_int(EE_IZMAX,20);
lc640_write_int(EE_U0B,uob__);
lc640_write_int(EE_TZAS,3);

lc640_write_int(EE_MNEMO_ON,mnON);
lc640_write_int(EE_MNEMO_TIME,30);	

lc640_write_int(EE_AV_OFF_AVT,1);
lc640_write_int(EE_APV_ON1,apvOFF);
}

//-----------------------------------------------
void net_drv(void)
{ 
//char temp_;    

if(++cnt_net_drv>=16) cnt_net_drv=0; //����� 16 �������

if(cnt_net_drv<=11) // � 1 �� 12 ������� ��������
	{    
	can2_out(cnt_net_drv,cnt_net_drv,GETTM,bps[cnt_net_drv]._flags_tu,*((char*)(&bps[cnt_net_drv]._vol_u)),*((char*)((&bps[cnt_net_drv]._vol_u))+1),*((char*)(&bps[cnt_net_drv]._vol_i)),*((char*)((&bps[cnt_net_drv]._vol_i))+1));
     
	if(cnt_net_drv<cnt_of_slave)
	     {
	     if(bps[cnt_net_drv]._cnt<CNT_SRC_MAX)
   	 		{    
   	 		bps[cnt_net_drv]._cnt++;
   	 		if( (bps[cnt_net_drv]._cnt>=CNT_SRC_MAX) && (!net_av) && (!(bps[cnt_net_drv]._av&0x08)) ) 
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
     can2_out(0xff,0xff,MEM_KF,*((char*)(&UMAX)),*((char*)((&UMAX))+1),*((char*)(&DU)),*((char*)((&DU))+1),0);
     } 
     
else if(cnt_net_drv==13)
	{
     can2_out(0xff,0xff,MEM_KF1,*((char*)(&TMAX)),*((char*)((&TMAX))+1),*((char*)(&TSIGN)),*((char*)((&TSIGN))+1),(char)TZAS);
     } 
else if(cnt_net_drv==14)
	{                 
//	can2_out(0xff,0xff,MEM_KF2,*((char*)(&U_INV_MAX)),*((char*)((&U_INV_MAX))+1),*((char*)(&U_INV_MIN)),*((char*)((&U_INV_MIN))+1),(char)T_INV_MAX);
     } 
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

for(ii=0;ii<488;ii++)
	{
	lcd_bitmap[ii]=0x00;
	}

if(!mnemo_cnt)
	{
		
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
const char* ptrs[40];
const char* sub_ptrs[40];
static char sub_cnt,sub_cnt1;
char i,sub_cnt_max;
char ii_;
static char ii_cnt,cnt_ind_bat;


	 
sub_cnt_max=5;
i=0;

if(spc_stat==spcVZ)
	{
	sub_ptrs[i++]=		" ������.�����  X:0x ";
	sub_cnt_max++;
	}
if(spc_stat==spcKE)
	{
	if(spc_bat==0)		sub_ptrs[i++]=		"�������� ���. ��� �1";
	else if(spc_bat==1)	sub_ptrs[i++]=		"�������� ���. ��� �2";
	sub_cnt_max++;
	}	
if(avar_stat&0x0001)
	{
	sub_ptrs[i++]=		"   ������ ����!!!   ";
	sub_cnt_max++;	
	}


if(avar_stat&0x0002)
	{
	sub_ptrs[i++]=	" ������ ������� �1  ";
	sub_cnt_max++;	
	}

if(avar_stat&0x0004)
	{
	sub_ptrs[i++]=	" ������ ������� �2  ";
	sub_cnt_max++;	
	}

if(avar_stat&(1<<(3+0)))
	{
	sub_ptrs[i++]=	"   ������ ��� �1    ";
	sub_cnt_max++;	
	}
if(avar_stat&(1<<(3+1)))
	{
	sub_ptrs[i++]=	"   ������ ��� �2    ";
	sub_cnt_max++;	
	}
if(avar_stat&(1<<(3+2)))
	{
	sub_ptrs[i++]=	"   ������ ��� �3    ";
	sub_cnt_max++;	
	}
if(avar_stat&(1<<(3+3)))
	{
	sub_ptrs[i++]=	"   ������ ��� �4    ";
	sub_cnt_max++;	
	}
if(avar_stat&(1<<(3+4)))
	{
	sub_ptrs[i++]=	"   ������ ��� �5    ";
	sub_cnt_max++;	
	}
if(avar_stat&(1<<(3+5)))
	{
	sub_ptrs[i++]=	"   ������ ��� �6    ";
	sub_cnt_max++;	
	}
if(avar_stat&(1<<(3+6)))
	{
	sub_ptrs[i++]=	"   ������ ��� �7    ";
	sub_cnt_max++;	
	}
if(avar_stat&(1<<(3+7)))
	{
	sub_ptrs[i++]=	"   ������ ��� �8    ";
	sub_cnt_max++;	
	}



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


if(ind==iMn)
	{

	if(num_of_wrks_bps)	
		{
		ptrs[0]	=	"  � ������    r���. ";
		}
	else if((NUMBAT>1)&&(!bat[0]._av)&&(!bat[1]._av))
		{
		ptrs[0]	=	" ������ �� �������  ";
		}
	else if((BAT_IS_ON[0]==bisON)&&(!bat[0]._av)/*&&(!(()&&()))*/)
		{
		ptrs[0]	=	"������ �� ������� �1";
		}
	else if((BAT_IS_ON[1]==bisON)&&(!bat[1]._av))
		{
		ptrs[0]	=	"������ �� ������� �2";
		}

	 
     i=0;
 /*	if(!num_of_wrks_bps)
 		{
 		ptrs[0]	=" ������ �� �������  "; 
 		ii_=139;
 		}
 	else ii_=0;*/	
 	
 	ptrs[1]="U�z=   ]� I�z=    @�";
     ptrs[2]="U�=    #� I�=     $�";
     ptrs[3]=" 0%:0^:0& 0</>  /0{ ";
     
 	ptrs[4]=										" ������� N1         ";
     ptrs[5]=										" ������� N2         ";
     if((NUMBAT==1)&&(BAT_IS_ON[0]!=bisON))ptrs[4]=		" ������� N2         ";
								
	ptrs[4+NUMBAT]=  								" ��� N1             ";
     ptrs[5+NUMBAT]=  								" ��� N2             ";
     ptrs[6+NUMBAT]=  								" ��� N3             ";
     ptrs[7+NUMBAT]=  								" ��� N4             ";
     ptrs[8+NUMBAT]= 								" ��� N5             ";
     ptrs[9+NUMBAT]= 								" ��� N6             ";
     ptrs[10+NUMBAT]= 								" ��� N7             ";
     ptrs[11+NUMBAT]= 								" ��� N8             ";
     ptrs[12+NUMBAT]= 								" ��� N9             ";
     ptrs[13+NUMBAT]= 								" ��� N10            ";
     ptrs[14+NUMBAT]= 								" ��� N11            ";
     ptrs[15+NUMBAT]= 								" ��� N12            ";

     
     ptrs[4+NUMIST+NUMBAT]=  							" �������� N1        ";
     ptrs[5+NUMIST+NUMBAT]=  							" �������� N2        ";
     ptrs[6+NUMIST+NUMBAT]=  							" �������� N3        ";
     ptrs[7+NUMIST+NUMBAT]=  							" �������� N4        ";

     ptrs[4+NUMIST+NUMBAT+NUMINV]= 					" ����               "; 
     ptrs[5+NUMIST+NUMBAT+NUMINV]= 					" ��������           "; 
     ptrs[6+NUMIST+NUMBAT+NUMINV]= 					" ������� �������    "; 
 	ptrs[6+NUMIST+NUMBAT+NUMINV+(NUMEXT!=0)]= 			" �����������    	 ";
     ptrs[7+NUMIST+NUMBAT+NUMINV+(NUMEXT!=0)]= 			" ���������          "; 
     ptrs[8+NUMIST+NUMBAT+NUMINV+(NUMEXT!=0)]= 			" ������ �������     "; 
     ptrs[9+NUMIST+NUMBAT+NUMINV+(NUMEXT!=0)]= 			" �����              "; 
     ptrs[10+NUMIST+NUMBAT+NUMINV+(NUMEXT!=0)]= 			" ������ ������� N1  "; 
     ptrs[11+NUMIST+NUMBAT+NUMINV+(NUMEXT!=0)]= 			" ������ ������� N2  "; 

     if(sub_ind==0)index_set=0;
	else if((index_set-sub_ind)>2)index_set=sub_ind+2;
	else if(sub_ind>index_set)index_set=sub_ind;
	
/*	if(ii_cnt>80)
		{
		if((spc_stat==spc_VZ)&&(!(av_.bAN))) ptrs[0]=" ���. �����  {�.";
		if(spc_stat==spc_KE1p1) ptrs[0]="�����.���.���.N1";
		if(spc_stat==spc_KE2p1) ptrs[0]="�����.���.���.N2";
		}*/
    



		
	if(sub_cnt<5)bgnd_par(ptrs[0],ptrs[index_set+1],ptrs[index_set+2],ptrs[index_set+3]);
	else bgnd_par(sub_ptrs[sub_cnt-5],ptrs[index_set+1],ptrs[index_set+2],ptrs[index_set+3]);
	
	if((ii_cnt<=80)||((spc_stat!=spcVZ)&&(spc_stat!=spcKE)/*&&(spc_stat!=spcKE2p1)*/)) 
		{
	if((ii_!=139)&&(/*(src_state[0]==ssWRK)||(src_state[1]==ssWRK)||(src_state[2]==ssWRK))*/num_of_wrks_bps!=0))
		{
		/*lcd_buffer[9]='N';
		lcd_buffer[10]=' ';
		lcd_buffer[11]=' ';
		lcd_buffer[12]=' ';
		lcd_buffer[13]=' ';
		lcd_buffer[14]=' ';
		lcd_buffer[15]=' ';*/
		
		if(sub_cnt<5)int2lcdyx(num_of_wrks_bps,0,14,0);
		/*lcd_buffer[11]='�';
		lcd_buffer[12]='�';
		lcd_buffer[13]='�';
		lcd_buffer[14]='.';*/

          }
     }
 
  	int2lcd(load_U,'#',1);
 	int2lcd(load_I,'$',1);
 	
	int2lcd(hour__,'%',0);
	int2lcd(min__,'^',0);
	int2lcd(sec__,'&',0);
	int2lcd(day__,'<',0);
	int2lcd(year__,'{',0); 
	if(!((month__>=1)&&(month__<=12)))month__=1;
	sub_bgnd(sm_mont[month__],'>',0);

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
	
	if(NUMBAT==0)sub_bgnd(" ������ ��� ������� ",'z',-2);
	else
		{
		int2lcd((cnt_ind_bat/20)+1+(!(BAT_IS_ON[0]==bisON)),'z',0);
		int2lcd((cnt_ind_bat/20)+1+(!(BAT_IS_ON[0]==bisON)),'z',0);
		int2lcd(bat[cnt_ind_bat/20]._Ub,']',1);
		if((mess_find_unvol(MESS2IND_HNDL))&&(mess_data[0]==PARAM_SAMOKALIBR)) sub_bgnd("����. ",'@',-4);
		else int2lcd_mmm(bat[cnt_ind_bat/20]._Ib,'@',2);
		}

	int2lcd(vz_cnt_s/60,'x',0);
	int2lcd(vz_cnt_h_,'X',0);
		
	int2lcdyx(plazma_suz[0],0,19,0);
/*	int2lcdyx(GET_REG(IO0PIN,7,1),0,8,0);*/

		 
	}

else if (ind==iBat)
	{
	if(bat[sub_ind1]._av/*&0x01*/)
		{
		if(bFL2)bgnd_par("       ������!        ",
		                 "     ������� �#       ",
		                 "    �� ����������     ",sm_);
		else bgnd_par(sm_,sm_,sm_,sm_);
		int2lcd(sub_ind1+1,'#',0);
		}               
	else
		{
		if(bat[sub_ind1]._Ib>0)
		     {
		     ptrs[1]="    ����������      ";
		     ptrs[3]="   I���=     #�     ";
		     }
		else
		     {
		     ptrs[1]="   �����������      ";
		     ptrs[3]="   I����=    #�     ";
		     }	
		ptrs[2]="   U���=     $�     ";
		
		if(bat[sub_ind1]._nd)ptrs[4]="    ��. ����������  ";
		else ptrs[4]="   t��� =   (�C     ";
		ptrs[5]="   �����=    w%     ";
		ptrs[6]="   C���=     Q�*�   ";
		ptrs[7]=sm_exit;
 
		bgnd_par("    ������� N@      ",
		          ptrs[sub_ind+1],ptrs[sub_ind+2],ptrs[sub_ind+3]);
	     
	     int2lcd(sub_ind1+1,'@',0);
	     int2lcd(bat[sub_ind1]._Ub,'$',1);
	     int2lcd_mmm(abs(bat[sub_ind1]._Ib),'#',2);
	     int2lcd(bat[sub_ind1]._Tb,'(',0);
	     int2lcd(bat[sub_ind1]._zar,'w',0);
	     if(BAT_C_REAL[sub_ind1]==0x5555)sub_bgnd("------",'Q',-1);
	     else int2lcd(BAT_C_REAL[sub_ind1],'Q',1);
	     if(sub_ind==4)lcd_buffer[60]=1;
		}

/*	int2lcdyx(lc640_read_int(ADR_EE_BAT_ZAR_CNT[sub_ind1]),0,5,0);
	long2lcdyx_mmm(bat[sub_ind1]._zar_cnt,1,8,0);
	int2lcdyx(bat[sub_ind1]._time_cnt,0,19,0);
	int2lcdyx(bat[sub_ind1]._Iintegr,3,5,0);
	int2lcdyx(bat[sub_ind1]._Iintegr_,3,10,0); */

	} 

 else if(ind==iBps)
	{
	const char* ptr[8];
 
	simax=5;

	ptr[1]=			" U��� =        (�   ";
	ptr[2]=			" I��� =        [A   ";
	ptr[3]=			" t��� =        ]��  ";
	ptr[4]=			" ����� ������       ";
	ptr[5]=			sm_exit;

	if(bps[sub_ind1]._state==bsWRK)
		{
		ptr[0]=		"      � ������      ";
		if((bps[sub_ind1]._flags_tm&0x08)&&(bFL2))
		ptr[0]=		"  ������� ������!!! ";	      
		}
 	 else if(bps[sub_ind1]._state==bsRDY)
	 	{
		ptr[0]=		"      � �������     ";	
		}

 	 else if(bps[sub_ind1]._state==bsBL)
	 	{
		ptr[0]=		" ������������ ����� ";	
		}

	 else if(bps[sub_ind1]._state==bsAPV)
	 	{
		ptr[0]=		"    �������� ���    ";
		}
	 
	 else if(bps[sub_ind1]._state==bsAV)
	 	{
		if(bps[sub_ind1]._av&(1<<0))
		ptr[0]=		" ������ - ��������! ";
		else if(bps[sub_ind1]._av&(1<<1))
		ptr[0]=		"������ - �����.U���!";
		else if(bps[sub_ind1]._av&(1<<2))	 
		ptr[0]=		"������ - �����.U���!";
		else if(bps[sub_ind1]._av&(1<<3))
			{
			ptr[0]=	"  ������ - �������� ";
			ptr[1]=	"      �����!!!      ";
			ptr[2]=	"                    ";
			simax=0;
			}
		}

	 else if(bps[sub_ind1]._state==bsOFF_AV_NET)
	 	{
		ptr[0]=		"      ��������      ";
		ptr[1]=		"     �����������    ";
		ptr[2]=		" ��������� �������! ";
		simax=0;
		}

	bgnd_par(			"       ��� N&       ",
					ptr[index_set],
					ptr[index_set+1],
					ptr[index_set+2]);

	if(sub_ind-index_set>2)index_set=sub_ind-2;
	else if (sub_ind<index_set)index_set=sub_ind;

	if(sub_ind>=4)	pointer_set(1);


//	if((/*(bps[sub_ind1]._av_net)||*/(bps[sub_ind1]._state==ssOFF)/*)&&(!net_av)*/)
//		{
//		bgnd_par( "     �������� N&    ",
//		          "   �� ���������!!!  ",
//		          sm_,sm_);
//		int2lcd(sub_ind1+1,'&',0);
//		}
//	else if(/*(net_av)&&(bps[sub_ind1]._cnt>10)*/(bps[sub_ind1]._state==ssOFF_AV_NET))
//		{
//		bgnd_par( "   ���. N&  ����.   ",
//		          "    ������  ����    ",
//		          sm_,sm_);
//		int2lcd(sub_ind1+1,'&',0);
//		}                          
//	else 
//		{
//		if((bps[sub_ind1]._state!=ssWRK)&&(bps[sub_ind1]._cnt>10))
//			{
//			ptr1="   ���. N&  ����.   ";
//			}	      
//		else ptr1="     �������� N&    ";
//		
//		if(bps[sub_ind1]._flags_bp=='A') ptr="   �������� ���     ";                                                              
//	     else if((bps[sub_ind1]._state==ssWRK)||((bps[sub_ind1]._Uii>200)&&(bps[sub_ind1]._cnt<10))) ptr="      � ������      ";
//		else if(bps[sub_ind1]._state==ssBL) ptr="      � �������     ";	
//		else
//			{
//			
//			if(bps[sub_ind1]._av_temper)ptr="     ��������!!!    ";
//			else if(bps[sub_ind1]._av_u_max)ptr="   �������� U���!   ";
//			else if(bps[sub_ind1]._av_u_min)ptr="   �������� U���!   ";
//			else ptr=sm_;
//			
//			}
//
//		if(!sub_ind)
//			{
//			bgnd_par(ptr1,ptr,"   U��� =     (�    ","   I��� =     [A    ");
//
//
//			}
//		else if(sub_ind)
//			{
//			bgnd_par(ptr1,"   t��� =     ] �   ","   ����� ������     ",sm_exit);
//			}
//			
//		//if(sub_ind!=0) lcd_buffer[29]=2;
//		if(sub_ind==1) 
//			{
//		     lcd_buffer[40]=1;
//			}
//		if(sub_ind==2) 
//			{
//		     lcd_buffer[60]=1;
//			}			

	int2lcd(sub_ind1+1,'&',0);
	int2lcd(bps[sub_ind1]._Uii,'(',1);
     int2lcd(bps[sub_ind1]._Ii,'[',1);  
   	int2lcd_mmm(bps[sub_ind1]._Ti,']',0); 
   			 
    // char2lcdhxy(bps[sub_ind1]._state,0x32);
    
    //	int2lcdyx(sub_ind,0,2,0);
//	int2lcdyx(index_set,0,4,0);	
     }  

else if(ind==iNet)
	{
	bgnd_par(		"        ����        ",
				" U   =     [�       ",
				" f   =     ]��      ",
				sm_exit);
	lcd_buffer[60]=1;
	     	
     int2lcd(net_U,'[',0);
     int2lcd(net_F,']',1);

     
                   	      	   	    		
     }

else if(ind==iLoad)
	{
	bgnd_par(		"      ��������      ",
				" U���� =     [�     ",
				" I���� =     ]�     ",
				sm_exit);
	lcd_buffer[60]=1;
	     	
     int2lcd(load_U,'[',1);
     int2lcd(load_I,']',1);

     
                   	      	   	    		
     }

else if(ind==iExtern)
	{

	ptrs[0]=  		" ��1        !��     ";
     ptrs[1]=  		" ��2        @��     ";
     ptrs[2]=  		" ��3        #��     ";
     ptrs[0+NUMDT]=  	" ��1        $       ";            
     ptrs[1+NUMDT]=  	" ��2        %       ";
	ptrs[2+NUMDT]=  	" ��3        ^       ";
	ptrs[3+NUMDT]=  	" ��4        &       ";
	ptrs[0+NUMEXT]=  	" �����              ";
	ptrs[1+NUMEXT]=  	"                    ";
	ptrs[2+NUMEXT]=  	"                    ";

	bgnd_par(		"  ������� �������   ",
				ptrs[index_set],
				ptrs[index_set+1],
				ptrs[index_set+2]);

	if(sub_ind-index_set>2)index_set=sub_ind-2;
	else if (sub_ind<index_set)index_set=sub_ind;
	pointer_set(1);
     }

else if(ind==iSpc)
	{

 	ptrs[0]=	" ���.�����          ";
 	ptrs[1]=	" ���.���.�����      ";
 	ptrs[2]=	" �.�. ������� N1    ";
 	ptrs[3]=	" �.�. ������� N2    ";
 //	ptrs[4]=	" �.�.�.  ���. N1    ";
 //	ptrs[5]=	" �.�.�.  ���. N2    ";
 	ptrs[4]=	" �����              ";
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
    	bgnd_par( "     �����������    ",
    	          ptrs[index_set],
    	          ptrs[index_set+1],
    	          ptrs[index_set+2]);
	pointer_set(1);
	}    		


 else if(ind==iVz)
	{          
	if(sub_ind==22) bgnd_par(	"������������� ����� ",
							"    ����������,     ",
							"  ������� ��������  ",
							"   ������� ���.N1   "); 
	else if(sub_ind==33) bgnd_par("������������� ����� ",
							"    ����������,     ",
							"  ������� ��������  ",
							"   ������� ���.N2   ");
	else if(spc_stat==spcVZ)
		{
		bgnd_par(				"������������� ����� ",
							" ����.-���     (�.  ",
							" �������            ",
							sm_exit);
		}
	else 
		{
		bgnd_par(				"������������� ����� ",
							" ����.-���     (�.  ",
							" ��������           ",
							sm_exit);
		}	

	pointer_set(1);	

	int2lcd(VZ_HR,'(',0);
	} 
	
	
else if(ind==iKe)
	{    
	if((spc_stat==spcKE)&&(spc_bat==sub_ind1))
		{
		bgnd_par(	"  �������� �������  ",
				"     ������� N{     ",
				" �������            ",
				sm_exit);
		}
	else
		{
		bgnd_par(	"  �������� �������  ",
				"     ������� N{     ",
				" ��������           ",
				sm_exit);
		}
	
	/*
	else if(sub_ind==11)
		{
		bgnd_par(	"  �������� �������  ",
				"     ������� N{     ",
				" ��������           ",
				sm_exit);
		}
	else if(sub_ind==12)
		{
		bgnd_par(	"  �������� �������  ",
				"     ������� N{     ",
				"  �������� �������  ",
				"    �������   N)    ");
		int2lcd(2-sub_ind1,')',0);
		}
	else if(sub_ind==13)
		{
		bgnd_par(	"  �������� �������  ",
				"     ������� N{     ",
				"      �������       ",
				"     �� �������     ");
		}
	else if(sub_ind==14)
		{
		bgnd_par(	"  �������� �������  ",
				"     ������� N{     ",
				"      ������        ",
				"      �������       ");
		}				
	else if(sub_ind==15)
		{
		bgnd_par(	"  �������� �������  ",
				"     ������� N{     ",
				"      �������       ",
				"     ����������     ");
		}
	else if(sub_ind==16)
		{
		bgnd_par(	"  �������� �������  ",
				"     ������� N{     ",
				"      �������       ",
				"    �����������     ");
		}				
	else if(sub_ind==17)
		{
		bgnd_par(	"  �������� �������  ",
				"     ������� N{     ",
				"  �������� ������   ",
				"    �������   N)    ");
		int2lcd(sub_ind1+1,')',0);
		}
	else if(sub_ind==18)
		{
		bgnd_par(	"  �������� �������  ",
				"     ������� N{     ",
				"      ������        ",
				"    �������   N)    ");
		int2lcd(2-sub_ind1,')',0);
		}				 
	else if(sub_ind==19)
		{
		bgnd_par(	"  �������� �������  ",
				"     ������� N{     ",
				"     ������� N)     ",
				"     ����������     ");
		int2lcd(2-sub_ind1,')',0);
		}
	else if(sub_ind==20)
		{
		bgnd_par(	"  �������� �������  ",
				"     ������� N{     ",
				"    �����������     ",
				"     ������� N)     ");
		int2lcd(2-sub_ind1,')',0);
		}				
	else if(sub_ind==21)
		{
		bgnd_par(	"  �������� �������  ",
				"     ������� N{     ",
				"  �������� ������   ",
				"    �������   N)    ");
		int2lcd(2-sub_ind1,')',0);
		}
	else if(sub_ind==22)
		{
		bgnd_par(	"  �������� �������  ",
				"     ������� N{     ",
				"      ������        ",
				"       ����          ");
		}	
	*/													
	//if(sub_ind==0) lcd_buffer[41]=1; 
	//else if(sub_ind==1) lcd_buffer[51]=1;
	pointer_set(2);
	int2lcd(sub_ind1+1,'{',0);
	}	  



else if(ind==iLog)
	{
	//char dt[4],dt_[4],dt__[4];
//	char iii;

	av_j_si_max=lc640_read_int(CNT_EVENT_LOG);
	if(av_j_si_max>64)av_j_si_max=0;

	if(av_j_si_max==0)
		{
		bgnd_par("   ������ �������   "," ������ ����        ",sm_exit,sm_);
		//lcd_buffer[33]=1;
		sub_ind=1;
		index_set=0;
		}       
		
	else if(av_j_si_max==1)
		{
		bgnd_par("   ������ �������   "," (                  ",sm_exit," �������� ������    ");
		//if(sub_ind==0)lcd_buffer[16]=1;
		//else if(sub_ind==1)lcd_buffer[33]=1;
		//else if(sub_ind==2)lcd_buffer[50]=1;		
		index_set=0;
		}

	else if(av_j_si_max==2)
		{
		if(sub_ind<index_set) index_set=sub_ind;
		else if((sub_ind-index_set)>2) index_set=sub_ind-2;		
		if(index_set==0) bgnd_par("   ������ �������   "," (                  "," [                  ",sm_exit);
		else if(index_set==1) bgnd_par("   ������ �������   "," (                  ",sm_exit," �������� ������    ");
		
		//if((sub_ind-index_set)==0) lcd_buffer[16]=1; 
		//else if((sub_ind-index_set)==1) lcd_buffer[33]=1;
		//else if((sub_ind-index_set)==2) lcd_buffer[50]=1;
		}
		
	else if(av_j_si_max>2)
		{
		if(sub_ind<index_set) index_set=sub_ind;
		else if((sub_ind-index_set)>2) index_set=sub_ind-2;  
		if(index_set==(av_j_si_max-1)) bgnd_par("   ������ �������   "," (                  ",sm_exit," �������� ������    ");
		else if(index_set==(av_j_si_max-2)) bgnd_par("   ������ �������   "," (                  "," [                  ",sm_exit);
		else bgnd_par("   ������ �������   "," (                  "," [                  "," {                  ");
		
		//if((sub_ind-index_set)==0) lcd_buffer[16]=1; 
		//else if((sub_ind-index_set)==1) lcd_buffer[33]=1;
		//else if((sub_ind-index_set)==2) lcd_buffer[50]=1;

		}
	pointer_set(1);
     event2ind(index_set,'(');
     event2ind(index_set+1,'[');	
     event2ind(index_set+2,'{');	  
     
	}



else if(ind==iLog_)
	{	
	unsigned short tempUI,tempUI_;
	unsigned long tempUL;
	char av_head[4],av_data_on[8],av_data_off[8],av_data[4];
	short av_head_int[2];
	
	bgnd_par(sm_,sm_,sm_,sm_);
	tempUI=lc640_read_int(PTR_EVENT_LOG);
	tempUI=ptr_carry(tempUI,64,-1*((signed)sub_ind1));
	tempUI*=32;
	tempUI+=EVENT_LOG;
     
     lc640_read_long_ptr(tempUI,av_head);
     lc640_read_long_ptr(tempUI+4,(char*)av_head_int);
     lc640_read_long_ptr(tempUI+8,av_data_on);
     lc640_read_long_ptr(tempUI+12,&(av_data_on[4])); 
     lc640_read_long_ptr(tempUI+16,av_data_off);
     lc640_read_long_ptr(tempUI+20,&(av_data_off[4]));      
	lc640_read_long_ptr(tempUI+24,av_data);
	
	//av_head_int[0]=123;  
//av_head_int[1]=456;	

	if((av_head[0]=='U')&&(av_head[2]=='R'))
		{
		bgnd_par(	"    ������������    ",
				"   ��� ���������    ",
				"        ����        ",
				"  0%(  0^ 0@:0#:0$  ");
				
				  	
		int2lcd(av_data_on[4],'@',0);
		int2lcd(av_data_on[5],'#',0);
		int2lcd(av_data_on[6],'$',0);
		int2lcd(av_data_on[2],'%',0);
		int2lcd(av_data_on[0],'^',0); 
		if(!((av_data_on[1]>=1)&&(av_data_on[1]<=12)))av_data_on[1]=1;
		sub_bgnd(sm_mont[av_data_on[1]],'(',0);
		//int2lcd(av_data_on[1],'(',0);
		//int2lcdyx(av_data_on[1],2,1,0);
		av_j_si_max=0;
		
		}

	else if((av_head[0]=='P')&&(av_head[2]=='A'))
		{  
		ptrs[0]="   ������ ����!!!   ";
		ptrs[1]="  0%(  0^ 0@:0#:0$  ";
		if((av_data_off[0]=='A')&&(av_data_off[1]=='A'))
			{
			ptrs[2]="    �� ���������    ";
			ptrs[3]="     U����=  +�     ";
			bgnd_par(ptrs[0],ptrs[1],ptrs[2],ptrs[3]);
			int2lcd(net_U,'+',0);
			}
		else 
			{
			gran_char(&index_set,0,1);
			ptrs[2]="      ���������     ";
			ptrs[3]="  0[]  0< 0>:0=:0)  ";
			ptrs[4]="     U���=  +�      ";
			bgnd_par(ptrs[index_set],ptrs[1+index_set],ptrs[2+index_set],ptrs[3+index_set]);
			int2lcd(av_data_off[4],'>',0);
			int2lcd(av_data_off[5],'=',0);
			int2lcd(av_data_off[6],')',0);
			int2lcd(av_data_off[2],'[',0);
			int2lcd(av_data_off[0],'<',0); 
			if(!((av_data_off[1]>=1)&&(av_data_off[1]<=12)))av_data_off[1]=1;
			sub_bgnd(sm_mont[av_data_off[1]],']',0);
			
			int2lcd(av_data[0]+(av_data[1]*256),'+',0);			
			}	
		
		int2lcd(av_data_on[4],'@',0);
		int2lcd(av_data_on[5],'#',0);
		int2lcd(av_data_on[6],'$',0);
		int2lcd(av_data_on[2],'%',0);
		int2lcd(av_data_on[0],'^',0); 
		if(!((av_data_on[1]>=1)&&(av_data_on[1]<=12)))av_data_on[1]=1;
		sub_bgnd(sm_mont[av_data_on[1]],'(',0);
		
		av_j_si_max=1;
		}

	else if((av_head[0]=='B')&&(av_head[2]=='C'))
		{  
		ptrs[0]="       ������       ";
		ptrs[1]="     ������� N+     ";
		ptrs[2]="  0%(  0^ 0@:0#:0$  ";
		if((av_data_off[0]=='A')&&(av_data_off[1]=='A'))
			{
			ptrs[3]="    �� ���������    ";
			bgnd_par(ptrs[0],ptrs[1],ptrs[2],ptrs[3]);
			}
		else 
			{
			gran_char(&index_set,0,1);
			ptrs[3]="      ���������     ";
			ptrs[4]="  0[]  0< 0>:0=:0)  ";
			bgnd_par(ptrs[index_set],ptrs[1+index_set],ptrs[2+index_set],ptrs[3+index_set]);
			int2lcd(av_data_off[4],'>',0);
			int2lcd(av_data_off[5],'=',0);
			int2lcd(av_data_off[6],')',0);
			int2lcd(av_data_off[2],'[',0);
			int2lcd(av_data_off[0],'<',0); 
			if(!((av_data_off[1]>=1)&&(av_data_off[1]<=12)))av_data_off[1]=1;
			sub_bgnd(sm_mont[av_data_off[1]],']',0);
			
						
			}	
		int2lcd(av_head[1]+1,'+',0);
		int2lcd(av_data_on[4],'@',0);
		int2lcd(av_data_on[5],'#',0);
		int2lcd(av_data_on[6],'$',0);
		int2lcd(av_data_on[2],'%',0);
		int2lcd(av_data_on[0],'^',0); 
		if(!((av_data_on[1]>=1)&&(av_data_on[1]<=12)))av_data_on[1]=1;
		sub_bgnd(sm_mont[av_data_on[1]],'(',0);
		
		av_j_si_max=1;
		}

	else if((av_head[0]=='B')&&(av_head[2]=='Z'))
		{  
		ptrs[0]="   �������������    ";
		ptrs[1]="       �����        ";
		ptrs[2]="  0%(  0^ 0@:0#:0$  ";
		if((av_data_off[0]=='A')&&(av_data_off[1]=='A'))
			{
			ptrs[3]="    �� ��������     ";
			bgnd_par(ptrs[0],ptrs[1],ptrs[2],ptrs[3]);
			}
		else 
			{
			gran_char(&index_set,0,1);
			ptrs[3]="      ��������      ";
			ptrs[4]="  0[]  0< 0>:0=:0)  ";
			bgnd_par(ptrs[index_set],ptrs[1+index_set],ptrs[2+index_set],ptrs[3+index_set]);
			int2lcd(av_data_off[4],'>',0);
			int2lcd(av_data_off[5],'=',0);
			int2lcd(av_data_off[6],')',0);
			int2lcd(av_data_off[2],'[',0);
			int2lcd(av_data_off[0],'<',0); 
			if(!((av_data_off[1]>=1)&&(av_data_off[1]<=12)))av_data_off[1]=1;
			sub_bgnd(sm_mont[av_data_off[1]],']',0);
			
						
			}	
		int2lcd(av_head[1]+1,'+',0);
		int2lcd(av_data_on[4],'@',0);
		int2lcd(av_data_on[5],'#',0);
		int2lcd(av_data_on[6],'$',0);
		int2lcd(av_data_on[2],'%',0);
		int2lcd(av_data_on[0],'^',0); 
		if(!((av_data_on[1]>=1)&&(av_data_on[1]<=12)))av_data_on[1]=1;
		sub_bgnd(sm_mont[av_data_on[1]],'(',0);
		
		av_j_si_max=1;
		}



	else if((av_head[0]=='B')&&(av_head[2]=='W'))
		{  
		ptrs[0]="       ������       ";
		ptrs[1]="     ������� N!     ";
		ptrs[2]="   ������           ";
		ptrs[3]="  0%(  0^ 0@:0#:0$  ";
		ptrs[4]="       U���=  <�";
		ptrs[5]="   �����            ";
		ptrs[6]="  0qw  0r 0i:0l:0s  ";
		ptrs[7]="         U���=  >�  ";
		ptrs[8]="   ������    /�*�.  ";
		
		bgnd_par(ptrs[index_set],ptrs[1+index_set],ptrs[2+index_set],ptrs[3+index_set]);
		
		int2lcd(av_head[1]+1,'!',0);

		int2lcd(av_data_on[4],'@',0);
		int2lcd(av_data_on[5],'#',0);
		int2lcd(av_data_on[6],'$',0);
		int2lcd(av_data_on[2],'%',0);
		int2lcd(av_data_on[0],'^',0); 
		if(!((av_data_on[1]>=1)&&(av_data_on[1]<=12)))av_data_on[1]=1;
		sub_bgnd(sm_mont[av_data_on[1]],'(',0);
				
		int2lcd(av_data_off[4],'i',0);
		int2lcd(av_data_off[5],'l',0);
		int2lcd(av_data_off[6],'s',0);
		int2lcd(av_data_off[2],'q',0);
		int2lcd(av_data_off[0],'r',0); 
		if(!((av_data_off[1]>=1)&&(av_data_off[1]<=12)))av_data_off[1]=1;
		sub_bgnd(sm_mont[av_data_off[1]],'w',0);
		
		
		int2lcd(av_head_int[0]/10,'/',1);
		int2lcd(av_data_on[3]+(av_data_on[7]*256),'<',1);
		int2lcd(av_head_int[1],'>',1);	
		av_j_si_max=5;				

		
		}

	else if((av_head[0]=='B')&&(av_head[2]=='K'))
		{  
		ptrs[0]="  �������� �������  ";
		ptrs[1]="       �������      ";
		ptrs[2]="   ������           ";
		ptrs[3]="  0%(  0^ 0@:0#:0$  ";
		ptrs[4]="         U���=  <�  ";
		ptrs[5]="   �����            ";
		ptrs[6]="  0qw  0r 0i:0l:0s  ";
		ptrs[7]="         U���=  >�  ";
		ptrs[8]="   �������   /�*�.  ";
		
		bgnd_par(ptrs[index_set],ptrs[1+index_set],ptrs[2+index_set],ptrs[3+index_set]);
		
		int2lcd(av_head[1]+1,'!',0);

		int2lcd(av_data_on[4],'@',0);
		int2lcd(av_data_on[5],'#',0);
		int2lcd(av_data_on[6],'$',0);
		int2lcd(av_data_on[2],'%',0);
		int2lcd(av_data_on[0],'^',0); 
		if(!((av_data_on[1]>=1)&&(av_data_on[1]<=12)))av_data_on[1]=1;
		sub_bgnd(sm_mont[av_data_on[1]],'(',0);
				
		int2lcd(av_data_off[4],'i',0);
		int2lcd(av_data_off[5],'l',0);
		int2lcd(av_data_off[6],'s',0);
		int2lcd(av_data_off[2],'q',0);
		int2lcd(av_data_off[0],'r',0); 
		if(!((av_data_off[1]>=1)&&(av_data_off[1]<=12)))av_data_off[1]=1;
		sub_bgnd(sm_mont[av_data_off[1]],'w',0);
		
		
		int2lcd(av_head_int[0],'/',1);
		int2lcd(av_data_on[3]+(av_data_on[7]*256),'<',1);
		int2lcd(av_head_int[1],'>',1);	
		av_j_si_max=5;				

		
		}



	else if(av_head[0]=='S')
		{  
		ptrs[0]="   ������ ��� N+    ";
		
		if(av_head[2]=='L')
			{
			ptrs[1]="     ����������     ";
			}
		else if(av_head[2]=='T')
			{
			ptrs[1]="      ��������      ";
			}		
		else if(av_head[2]=='U')
			{
			ptrs[1]="   �������� U���.   ";
			}		
		else if(av_head[2]=='u')
			{
			ptrs[1]="   �������� U���.   ";
			}								
		
		
		ptrs[2]="  0%(  0^ 0@:0#:0$  ";
		if((av_data_off[0]=='A')&&(av_data_off[1]=='A'))
			{
			ptrs[3]="    �� ���������    ";
			bgnd_par(ptrs[0],ptrs[1],ptrs[2],ptrs[3]);
			}
		else 
			{
			gran_char(&index_set,0,1);
			ptrs[3]="      ���������     ";
			ptrs[4]="  0[]  0< 0>:0=:0)  ";
			bgnd_par(ptrs[index_set],ptrs[1+index_set],ptrs[2+index_set],ptrs[3+index_set]);
			int2lcd(av_data_off[4],'>',0);
			int2lcd(av_data_off[5],'=',0);
			int2lcd(av_data_off[6],')',0);
			int2lcd(av_data_off[2],'[',0);
			int2lcd(av_data_off[0],'<',0); 
			if(!((av_data_off[1]>=1)&&(av_data_off[1]<=12)))av_data_off[1]=1;
			sub_bgnd(sm_mont[av_data_off[1]],']',0);
			
						
			}	
		int2lcd(av_head[1]+1,'+',0);
		int2lcd(av_data_on[4],'@',0);
		int2lcd(av_data_on[5],'#',0);
		int2lcd(av_data_on[6],'$',0);
		int2lcd(av_data_on[2],'%',0);
		int2lcd(av_data_on[0],'^',0); 
		if(!((av_data_on[1]>=1)&&(av_data_on[1]<=12)))av_data_on[1]=1;
		sub_bgnd(sm_mont[av_data_on[1]],'(',0);
		
		}

	
	}
		 
else if(ind==iBatLog)
	{
	if(BAT_IS_ON[sub_ind1]==bisON)ptrs[0]=" �������  0!/@  /0# ";
	else ptrs[0]=" �������� 0!/@  /0# ";
     ptrs[1]=" �����.���.     $A*�";
     ptrs[2]=" ���������      %�. ";
     ptrs[3]=" �������� �������   ";
     ptrs[4]=" ������������� �����";
     ptrs[5]=" �������            ";
     ptrs[6]=sm_exit;	
	if((sub_ind-index_set)>1)index_set=sub_ind-1;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par(	" ���������� ������  ",
			"     ������� N^     ",
			ptrs[index_set],
			ptrs[index_set+1]);
	pointer_set(2);	

	int2lcd(sub_ind1+1,'^',0); 
	int2lcd(BAT_DAY_OF_ON[sub_ind1],'!',0);
	sub_bgnd(sm_mont[BAT_MONTH_OF_ON[sub_ind1]],'@',0);
	int2lcd(BAT_YEAR_OF_ON[sub_ind1],'#',0); 
	int2lcd(BAT_C_NOM[sub_ind1],'$',0);
	int2lcd(BAT_RESURS[sub_ind1],'%',0);
	}

else if(ind==iBatLogKe)
	{             
	if(av_j_si_max==0)
		{
		bgnd_par(	"  �������� �������  ",
				"     ������� N!     ",
				" ������ ����        ",
				sm_exit);
		pointer_set(3);
		sub_ind=0;
		index_set=0;
		} 
	else if(av_j_si_max==1)
		{
		bgnd_par(	"  �������� �������  ",
				"     ������� N!     ",
				" (                  ",
				sm_exit);
		index_set=0;
		pointer_set(2);
		}	
	else
		{
		if(sub_ind<index_set) index_set=sub_ind;
		else if((sub_ind-index_set)>1) index_set=sub_ind-1;
		if(index_set==(av_j_si_max-1)) 
			{
			bgnd_par( "  �������� �������  ",
					"     ������� N!     ",
					" (                  ",
					sm_exit);
			}
		else
			{
			bgnd_par(	"  �������� �������  ",
					"     ������� N!     ",
					" (                  ",
					" [                  ");
			}
		pointer_set(2);			 
		}
		
   	int2lcd(sub_ind1+1,'!',0);
 	event_data2ind(content[index_set],'(');
 	event_data2ind(content[index_set+1],'[');
	}

else if(ind==iBatLogVz)
	{
	if(av_j_si_max==0)
		{
		bgnd_par(	"������������� ������",
				"     ������� N!     ",
				" ������ ����        ",
				sm_exit);
		sub_ind=0;
		index_set=0;
		pointer_set(3);
		} 
	else if(av_j_si_max==1)
		{
		bgnd_par(	"������������� ������",
				"     ������� N!     ",
				" (                  ",
				sm_exit);
		index_set=0;
		pointer_set(2);
		}	
	else
		{
		if(sub_ind<index_set) index_set=sub_ind;
		else if((sub_ind-index_set)>1) index_set=sub_ind-1;
		if(index_set==(av_j_si_max-1)) 
			{
			bgnd_par(	"������������� ������",
					"     ������� N!     ",
					" (                  ",
					sm_exit);
			}

		else bgnd_par(	"������������� ������",
					"     ������� N!     ",
					" (                  ",
					" [                  "); 
		pointer_set(2);			        
		}
   	int2lcd(sub_ind1+1,'!',0);
 	event_data2ind(content[index_set],'(');
 	event_data2ind(content[index_set+1],'[');
	
	}
   
else if(ind==iBatLogWrk)
	{
	if(av_j_si_max==0)
		{
		bgnd_par(	"      �������       ",
				"     ������� N!     ",
				" ������ ����        ",
				sm_exit);
		sub_ind=0;
		index_set=0;
		pointer_set(3);
		} 
	else if(av_j_si_max==1)
		{
		bgnd_par(	"      �������       ",
				"     ������� N!     ",
				" (                  ",
				sm_exit);
		index_set=0;
		pointer_set(2);
		}	

	else
		{
		if(sub_ind<index_set) index_set=sub_ind;
		else if((sub_ind-index_set)>1) index_set=sub_ind-1;
		if(index_set==(av_j_si_max-1))
			{
			bgnd_par(	"      �������       ",
					"     ������� N!     ",
					" (                  ",
					sm_exit);
			}
		else bgnd_par(	"      �������       ",
					"     ������� N!     ",
					" (                  ",
					" [                  ");

		pointer_set(2);
		}

   	int2lcd(sub_ind1+1,'!',0);
 	event_data2ind(content[index_set],'(');
 	event_data2ind(content[index_set+1],'[');

	

	} 
	
else if((ind==iSet_prl)||(ind==iK_prl)||(ind==iSpc_prl_vz)
	||(ind==iSpc_prl_ke)||(ind==iAusw_prl)||(ind==iPrltst))
	{
	bgnd_par("  �������  ������   ",sm_,sm_,sm_);
	int2lcdyx(parol[0],1,8,0);
     int2lcdyx(parol[1],1,9,0);
     int2lcdyx(parol[2],1,10,0);
     lcd_buffer[48+sub_ind]='�';
	}	
		
else if(ind==iPrl_bat_in_out)
	{
	if(BAT_IS_ON[sub_ind1]==bisON)ptrs[0]="��� ��������� ���.-�";
	else  ptrs[0]="��� �������� �������";
	bgnd_par(ptrs[0],"  �������� ������   ",sm_,sm_);
	
     int2lcdyx(parol[0],2,8,0);
     int2lcdyx(parol[1],2,9,0);
     int2lcdyx(parol[2],2,10,0);
     lcd_buffer[68+sub_ind]='�';	
	}
	
else if(ind==iSet)
	{
     ptrs[0]=		" �����������        ";
	ptrs[1]=		" ����� � ����       ";
     ptrs[2]=		" ���������          ";
     ptrs[3]=		" ���������         y";
	ptrs[4]=		" ��.����.   (       ";
	ptrs[5]=		" ���������� ������� ";
	ptrs[6]=		"  ������    )       ";
	ptrs[7]=		" ��� ����������     ";
	ptrs[8]=		" ������.������ z    ";
	ptrs[9]=		" T ��������   ����  ";
     ptrs[10]=		" �������     q���.  ";
     ptrs[11]=		" Umax=       !�     ";
     ptrs[12]=		" dU=         Z�     ";
     ptrs[13]=		" U�0�=       @�     ";
     ptrs[14]=		" U�20�=      #�     ";
     ptrs[15]=		" U����=      ^�     ";
     ptrs[16]=		" Umin.����=  &�     ";
	ptrs[17]=		" U0�=        >�     ";
	ptrs[18]=		" I��.=       j�     ";
     ptrs[19]=		" I�.���.=    J�     ";
     ptrs[20]=		" Imax=       ]A     ";
     ptrs[21]=		" Kimax=      {      ";
     ptrs[22]=		" K���.���.=    [    ";
     ptrs[23]=		" T�.���.�.�. !�     ";
	ptrs[24]=		" t�.max=     $�C    ";
	ptrs[25]=		" t�.����=    z�C    ";
	ptrs[26]=		" t���.max=   b�C    ";
	ptrs[27]=		" t���.����=  X�C    ";
     ptrs[28]=		" ������� �������    ";
     ptrs[29]=		" �����              ";
     ptrs[30]=		" ����������         "; 
     ptrs[31]=		" ����               ";        
	
	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("     ���������      ",
			ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);

	pointer_set(1);	
	
	if(index_set<18)
	     {
	     if(ZV_ON)sub_bgnd("���.",'(',0);
	     else sub_bgnd("���.",'(',0);
	     if(AV_OFF_AVT)sub_bgnd("�����.",')',0);
	     else sub_bgnd("����.",')',0);
		if(PAR)sub_bgnd("���.",'z',0);
	     else sub_bgnd("���.",'z',0);
	     if(MNEMO_ON==mnON)
	     	{
	     	sub_bgnd("����� y�.",'y',-8);
	     	int2lcd(MNEMO_TIME,'y',0);
	     	}
	     else sub_bgnd("����.",'y',-4);
	     int2lcd(UMAX,'!',1);
	     int2lcd(DU,'Z',1);
	     int2lcd(UB0,'@',1);
	     int2lcd(UB20,'#',1);
	     int2lcd(USIGN,'^',0);
	     int2lcd(UMN,'&',0);
	     int2lcd(U0B,'>',1);
	     } 
	int2lcd(TMAX,'$',0);
	int2lcd(IKB,'j',2);
	int2lcd(KVZ,'[',3);
	int2lcd(IMAX,']',1);
	int2lcd(KIMAX,'{',1);
	int2lcd(IZMAX,'J',1); 
	int2lcd(TZAS,'!',0);
	int2lcd(TBAT,'q',0);
	int2lcd(TSIGN,'z',0); 
	int2lcd(TBATMAX,'b',0); 
	int2lcd(TBATSIGN,'X',0); 
	}

else if (ind==iDef)
	{ 
	ptrs[0]=" ����220/48-80�     ";
	ptrs[1]=" ����220/60-80�     ";
	ptrs[2]=" ����220/48-100�    ";
	ptrs[3]=" ����220/60-100�    ";
	ptrs[4]=" ����220/48-120�    ";
	ptrs[5]=" ����220/60-120�    ";
	ptrs[6]=sm_exit;
	if(bFL5)ptrs[default_temp]=sm_;
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;

     bgnd_par("����������� ���.-�� ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);
	} 	
	        



else if(ind==iSet_T)
	{
	static char phase_cnt;
	if(++phase_cnt>=15)
	     {
	     phase_cnt=0;
	     if(++phase>=3)phase=0;
	     }
	ptrs[0]=sm_time;
	ptrs[1]=sm_;
	if(phase==0)ptrs[2]="     <> - �����     ";
     if(phase==1)ptrs[2]="   ^v - ���������   ";
     if(phase==2)ptrs[2]="     0  - �����     ";
	
	bgnd_par(" ���������  ������� ",ptrs[0],ptrs[1],ptrs[2]);
     if(sub_ind==0)lcd_buffer[42]='^';
     else if(sub_ind==1)lcd_buffer[45]='^';
     else if(sub_ind==2)lcd_buffer[48]='^';
     else if(sub_ind==3)lcd_buffer[51]='^';
     else if(sub_ind==4)lcd_buffer[54]='^';
     else if(sub_ind==5)lcd_buffer[58]='^';
  
 	int2lcd(sec__,'&',0);
 	int2lcd(min__,'^',0);
 	int2lcd(hour__,'%',0);
 	
 	int2lcd(day__,'<',0);
 	sub_bgnd(sm_mont[month__],'>',0);
 	int2lcd(year__,'{',0);
 	if(bFL2)
 	     {
 	     lcd_buffer[find(':')]=' ';
 	     lcd_buffer[find(':')]=' ';
 	     }  
	}  

else if(ind==iStr)
	{
	ptrs[0]=" �������           @";
	ptrs[1]=" ����������        !";
	ptrs[2]=" ����������        ^";	
	ptrs[3]=" �������� ������.  #";
	ptrs[4]=" ����� ���������   $";
	ptrs[5]=" �����                   ";
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;
	bgnd_par("      ���������     ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);

	int2lcd(NUMBAT,'@',0);		
	int2lcd(NUMIST,'!',0); 
	int2lcd(NUMINV,'^',0);
	int2lcd(NUMDT,'#',0);
	int2lcd(NUMSK,'$',0);
	}    

else if (ind==iApv)
	{ 
	ptrs[0]=			" ��� 1� ������� !   ";
	if(APV_ON1!=apvON)
	     {
	     ptrs[1]=		" �����              ";
	     ptrs[2]=sm_;
	     ptrs[3]=sm_;
	     ptrs[4]=sm_;
	     simax=1;
	     }
	else
	     {
	     if(APV_ON2!=apvON)
	          {
	          ptrs[1]=" ��� 2� ������� @   ";
	          ptrs[2]=" �����              ";
	          ptrs[3]=sm_;
	          ptrs[4]=sm_;
	          simax=2;
	          }
	     else 
	          {
               ptrs[1]=" ��� 2� ������� @   ";
	          ptrs[2]=" ������ ���2     #�.";
	          ptrs[3]=" �����              ";
	          ptrs[4]=sm_;
	          simax=3;	          
	          }     
	     }     
	
	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>2) index_set=sub_ind-2;	
     bgnd_par("   ��� ����������   ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	
	pointer_set(1);
	if(APV_ON1==apvON)sub_bgnd("���.",'!',0);
	else sub_bgnd("����.",'!',-1);
	
	if(APV_ON2==apvON)
	     {
	     sub_bgnd("���.",'@',0);
	     int2lcd(APV_ON2_TIME,'#',0);
	     }
	else sub_bgnd("����.",'@',-1);	
     
 	} 


else if(ind==iK)
	{
	char i;
	i=0;
	
	ptrs[i++]=" ����               ";
	if(NUMBAT)
     ptrs[i++]=" �������            ";
	if(NUMIST)
	ptrs[i++]=" ���                ";
	if(NUMINV)
     ptrs[i++]=" ���������          ";
	ptrs[i++]=" ��������           ";
     if(NUMDT)
     ptrs[i++]=" ������� �������    ";
     ptrs[i++]=" �����              ";
     ptrs[i++]="                    ";
     ptrs[i++]="                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("     ����������     ",
			ptrs[index_set],
			ptrs[index_set+1],
			ptrs[index_set+2]);

	pointer_set(1);	 
	}    	

else if(ind==iK_net)
	{
	ptrs[0]=" U =     @�         ";
     ptrs[1]=" �����              ";
	ptrs[2]="                    ";
	
	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("   ���������� ����  ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);

	pointer_set(1);	
	int2lcd(net_U,'@',0);
	//int2lcdyx(net_buff_,3,10,0);
	
	//int2lcdyx(Kunet,3,16,0);
     }


else if(ind==iK_load)
	{
	ptrs[0]=" U =     @�         ";
     ptrs[1]=" �����              ";
	ptrs[2]="                    ";
	
	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par(		" ���������� ��������",
				ptrs[index_set],
				ptrs[index_set+1],
				ptrs[index_set+2]);

	pointer_set(1);	
	int2lcd(load_U,'@',1);
     }

     
else if(ind==iK_bat_sel)
	{
	ptrs[0]=						" ������� N1         ";
     ptrs[1]=						" ������� N2         ";
     if(BAT_IS_ON[0]!=bisON)ptrs[0]=	" ������� N2         ";
	ptrs[0+NUMBAT]=				" �����              ";
	ptrs[1+NUMBAT]=				"                    ";
	ptrs[2+NUMBAT]=				"                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par(" ���������� ������� ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	pointer_set(1);
	
     }     

else if(ind==iK_bat)
	{
	ptrs[0]=		" U��� =     @�      ";
	ptrs[1]=		" ������������ U���  ";
	ptrs[2]=		"  �������� � ��� �  ";
     ptrs[3]=		" I��� =     #�      ";
     if(phase==0)
          {
          ptrs[4]=	"   ������� � ���    ";
          ptrs[5]=	"���������� ���� I���";
          }
     else          
          {
          ptrs[4]=	" ������������ I���  ";
          ptrs[5]=	"  �������� � ��� �  ";
          }
     if(bat[sub_ind1]._nd)
     	{
     	ptrs[6]=		" ������ ����������� ";
     	ptrs[7]=		"     ����������     ";
     	ptrs[8]=		"  ��� �����������.  ";
     	}
     else
     	{	     
     	ptrs[6]=		" t��� =    $�C      ";
     	ptrs[7]=		" ������������ t���  ";
     	ptrs[8]=		"  �������� � ��� �  ";
     	}
     ptrs[9]=		" �����              ";
     ptrs[10]=		"                    ";
     ptrs[11]=		"                    ";

	bgnd_par(		" ���������� ���. N! ",
				ptrs[index_set],
				ptrs[index_set+1],
				ptrs[index_set+2]);
     
     if(sub_ind==0)
     	{
     	mess_send(MESS2BPS_HNDL,PARAM_BPS_ALL_OFF_AFTER_2SEC,0xffff,10);
     	mess_send(MESS2BAT_HNDL,PARAM_BAT_MASK_OFF_AFTER_2SEC,(1<<(1-sub_ind1)),10);
     	//mess_send(MESS2BAT_HNDL1,PARAM_BAT_ON,(1<<sub_ind1),10);
     	}
     
     if(sub_ind==3)
     	{
     	if(phase==0)
     		{
     		mess_send(MESS2RELE_HNDL,PARAM_RELE_SAMOKALIBR_ENABLE,0,10);
			mess_send(MESS2BAT_HNDL,PARAM_BAT_MASK_OFF_AFTER_2SEC,(1<<sub_ind1),10);
     		//mess_send(MESS2BAT_HNDL1,PARAM_BAT_ON,(1<<(1-sub_ind1)),10);
     		}
     	else if(phase==1)
     		{
			mess_send(MESS2BPS_HNDL,PARAM_BPS_ALL_OFF_AFTER_2SEC,0xffff,10);
			mess_send(MESS2BAT_HNDL,PARAM_BAT_MASK_OFF_AFTER_2SEC,(1<<(1-sub_ind1)),10);
     		//mess_send(MESS2BAT_HNDL1,PARAM_BAT_ON,(1<<sub_ind1),10);
   			}
     		
     	}

     if(sub_ind==6)
     	{
   		//mess_send(_MESS_BAT_MASK_ON,_MESS_BAT_MASK_ON,(0xffff),10);
    		//mess_send(MESS_SRC_ON_OFF,_MESS_SRC_MASK_UNBLOK,0xffff,10);
     		
     	}
	
	if((sub_ind==0)||(sub_ind==1)||(sub_ind==2))index_set=0;
	else if((sub_ind==3)||(sub_ind==4)||(sub_ind==5))index_set=3;
	else if((sub_ind==6)||(sub_ind==7)||(sub_ind==8))index_set=6;
	else index_set=9;
	


	pointer_set(1);	
	int2lcd(sub_ind1+1,'!',0);
	int2lcd(bat[sub_ind1]._Ub,'@',1);
	int2lcd_mmm(bat[sub_ind1]._Ib,'#',2);
	int2lcd_mmm(bat[sub_ind1]._Tb,'$',0);
	//int2lcd(Ibat,'a',1);
	
	   /*  int2lcdyx(ad7705_res1,0,10,0);
         int2lcdyx(ad7705_res2,0,16,0); */
		    //int2lcdyx(ad7705_buff_[sub_ind1],0,10,0);
         //int2lcdyx(Kibat1[sub_ind1],0,16,0); 
	//int2lcdyx(Kubat[0],0,4,0);
	//int2lcdyx(adc_buff_[4],0,8,0);

	//int2lcdyx(rele_stat,2,15,0);
	
	
	/*	int2lcdyx(bat_cnt_to_block[0],3,3,0);
	int2lcdyx(bat_rel_stat[0],3,7,0); */

	//int2lcdyx(Ktbat[sub_ind1],0,16,0);
	//char2lcdhyx(rele_stat,0,19);
	}  	

else if(ind==iK_bps_sel)
	{
	ptrs[0]=						" ��� N1             ";
     ptrs[1]=						" ��� N2             ";
     ptrs[2]=						" ��� N3             ";
	ptrs[3]=						" ��� N4             ";
     ptrs[4]=						" ��� N5             ";
     ptrs[5]=						" ��� N6             ";
	ptrs[6]=						" ��� N7             ";
     ptrs[7]=						" ��� N8             ";
     ptrs[8]=						" ��� N9             ";
	ptrs[9]=						" ��� N10            ";
     ptrs[10]=						" ��� N11            ";
     ptrs[11]=						" ��� N12            ";               
	ptrs[NUMIST]=					" �����              ";
	ptrs[1+NUMIST]=				"                    ";
	ptrs[2+NUMIST]=				"                    ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("  ���������� �����  ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	pointer_set(1);
	
     }     

else if(ind==iK_bps)
	{
	
	ptrs[0]=" U��� =    @�       ";
     ptrs[1]=" ������������ U���  ";
     ptrs[2]="  �������� � ��� �  "; 
	ptrs[3]=" U���� =   #�       ";
     ptrs[4]=" ������������ U���� ";
     ptrs[5]="  �������� � ��� �  ";
	ptrs[6]=" U����� =   $�      ";
	if(bFL_)
		{
		ptrs[7]=" ���������� U�����  ";
     	ptrs[8]="  �������� � ��� �  ";
     	}
     else 
     	{
		ptrs[7]=" ����������� � ���  ";
     	ptrs[8]="    �����������     ";     	
     	}	
	ptrs[9]=" I��� =     %�      ";
	if(phase==0)
          {
          ptrs[10]=	"   ������� � ���    ";
          ptrs[11]=	"���������� ���� I���";
          }
     else
     	{
          ptrs[10]=" ������������ I���  ";
          ptrs[11]="  �������� � ��� �  ";     	
     	} 
     	
     ptrs[12]=" t��� =   ^�C       ";    
	ptrs[13]=" ������������ t���  ";
     ptrs[14]="  �������� � ��� �  ";
     ptrs[15]=sm_exit;
     ptrs[16]=sm_;
     ptrs[17]=sm_;     	     	    
	

     if((sub_ind==0)||(sub_ind==1)||(sub_ind==2))index_set=0;
	else if((sub_ind==3)||(sub_ind==4)||(sub_ind==5))index_set=3;
	else if((sub_ind==6)||(sub_ind==7)||(sub_ind==8))index_set=6;
	else if((sub_ind==9)||(sub_ind==10)||(sub_ind==11))index_set=9;
	else if((sub_ind==12)||(sub_ind==13)||(sub_ind==14))index_set=12;	
	else index_set=15;
	
	bgnd_par(" ���������� ��� N! ",ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);

	pointer_set(1);	
	int2lcd(sub_ind1+1,'!',0);
	int2lcd(bps[sub_ind1]._Uii,'@',1);
	int2lcd(bps[sub_ind1]._Uin,'#',1);
	int2lcd(U_AVT,'$',1);
	int2lcd(bps[sub_ind1]._Ii,'%',1);
	int2lcd(bps[sub_ind1]._Ti,'^',0); 
	 
	
     if((sub_ind==0)||(sub_ind==3))
		{
		mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,(1<<sub_ind1),10);
		mess_send(MESS2BAT_HNDL,PARAM_BAT_ALL_OFF_AFTER_2SEC,0,10);
	    	mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_SET,1000,10);
          }
     if(sub_ind==6)
		{
          mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,(1<<sub_ind1),10);
          mess_send(MESS2BAT_HNDL,PARAM_BAT_ALL_OFF_AFTER_2SEC,0,40);
          mess_send(MESS2UNECC_HNDL,PARAM_UNECC_SET,U_AVT,10);
	    	mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_FAST_REG,0,10);

          }

     if(sub_ind==9)
		{
		if(phase==0)
			{
          	mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,~(1<<sub_ind1),10);
          	}
      	else if(phase==1)
			{
          	mess_send(MESS2BPS_HNDL,PARAM_BPS_MASK_ON_OFF_AFTER_2SEC,(1<<sub_ind1),10);
			mess_send(MESS2BAT_HNDL,PARAM_BAT_ALL_OFF_AFTER_2SEC,0,10);
          	}
          mess_send(MESS2CNTRL_HNDL,PARAM_CNTRL_STAT_SET,1000,10);
          }
	
    	if(sub_ind==12)
		{
          }	
          
          
	if(mess_find( (MESS2IND_HNDL)) && (mess_data[0]==PARAM_U_AVT_GOOD) )
		{
		show_mess("     ���������      ",
	          	"    ����������      ",
	          	" ���������� ������  ",
	          	"    �����������     ",3000);
		}	     
	     
	//MSG_IND2PWM_SRC1=900;
	//MSG_IND2PWM_SRC2=900;         
/*int2lcdyx(sub_ind1,0,0,0);
int2lcdyx(sub_ind,0,1,0);
int2lcdyx(phase,0,2,0);
int2lcdyx(MSG_IND2OUT_DIS_SRC1,0,3,0);
int2lcdyx(MSG_IND2OUT_DIS_SRC2,0,4,0);  
int2lcdyx(MSG_IND2OUT_EN_SRC1,0,5,0);
int2lcdyx(MSG_IND2OUT_EN_SRC2,0,6,0); */

//int2lcdyx(cntrl_stat1,0,19,0); 
//int2lcdyx(load_U,0,5,0); 
//int2lcdyx(cntrl_stat,0,10,0); 
//int2lcdyx(bps[sub_ind1]._rotor,0,19,0); 
//int2lcdyx(u_necc,0,19,0);  
	 }			
if(ind==iDeb)
     {
     if(sub_ind==0)
     	{
     	bgnd_par("     ���������      ",
     	         "                    ",
     	         "                    ",
     	         "                    ");
		int2lcdyx(GET_REG(C2GSR,16,8),0,15,0);
		int2lcdyx(GET_REG(C2GSR,24,8),0,19,0);
		
		int2lcdyx(cntrl_stat,0,3,0);

		int2lcdyx(sub_ind1+1,1,0,0);
		int2lcdyx(sub_ind1+2,2,0,0);
		int2lcdyx(sub_ind1+3,3,0,0);
		
		
		int2lcdyx(bps[sub_ind1  ]._cnt,1,2,0);
		int2lcdyx(bps[sub_ind1+1]._cnt,2,2,0);
		int2lcdyx(bps[sub_ind1+2]._cnt,3,2,0);		
		
	/*	int2lcdyx(bps[sub_ind1  ]._ist_blok_cnt,1,5,0);
		int2lcdyx(bps[sub_ind1+1]._ist_blok_cnt,2,5,0);
		int2lcdyx(bps[sub_ind1+2]._ist_blok_cnt,3,5,0);*/			
		
		char2lcdhyx(bps[sub_ind1  ]._flags_tu,1,8);
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
		char2lcdhyx(bps[sub_ind1+2]._Ii,3,19);
	/*
		char2lcdhyx(bps[sub_ind1]._rotor>>8,1,15);
		char2lcdhyx(bps[sub_ind1+1]._rotor>>8,2,15);
		char2lcdhyx(bps[sub_ind1+2]._rotor>>8,3,15);		
		
		char2lcdhyx((char)bps[sub_ind1]._rotor,1,17);
		char2lcdhyx((char)bps[sub_ind1+1]._rotor,2,17);
		char2lcdhyx((char)bps[sub_ind1+2]._rotor,3,17);*/



     	}     

    	else if(sub_ind==1) 
     	{
     	bgnd_par(" ��                 ",
     	         "                    ",
     	         "                    ",
     	         "                    ");

			int2lcdyx(spc_stat,0,5,0);
			int2lcdyx(spc_bat,0,7,0);
			int2lcdyx(spc_phase,0,9,0);

			//long2lcdyx_mmm(bat[0]._zar_cnt_ke,2,6,0);
			//long2lcdyx_mmm(bat[1]._zar_cnt_ke,3,6,0);
			long2lcdyx_mmm(bat[0]._Ub,2,6,0);
			long2lcdyx_mmm(bat[1]._Ub,3,6,0);
			//int2lcdyx(abs(bat[0]._Ib),2,12,0);
			//int2lcdyx(abs(bat[1]._Ib),3,12,0);

			int2lcdyx(lc640_read_int(ADR_EE_BAT_C_REAL[0]),2,12,0);
			int2lcdyx(lc640_read_int(ADR_EE_BAT_C_REAL[1]),3,12,0);

			int2lcdyx(lc640_read_int(ADR_EE_BAT_ZAR_CNT_KE[0]),2,17,0);
			int2lcdyx(lc640_read_int(ADR_EE_BAT_ZAR_CNT_KE[1]),3,17,0);

		    ///lc640_write_int(ADR_EE_BAT_C_REAL[spc_bat],lc640_read_int(ADR_EE_BAT_ZAR_CNT_KE[spc_bat]));

//	bat[in]._zar_cnt_ke=0;
//	lc640_write_int(ADR_EE_BAT_ZAR_CNT_KE[in],0);

 
 		}

 

    else if(sub_ind==2)
     	{
     	bgnd_par("��                  ",
     		    "                    ",
     		    "                    ",
     		    "                    ");
     		    
     	int2lcdyx(TBAT*60,0,6,0);
    		int2lcdyx(main_kb_cnt,0,11,0);
     	
		
		int2lcdyx(cntrl_stat,0,19,0);
		int2lcdyx(cntrl_stat_old,0,15,0);

		int2lcdyx(kb_cnt_1lev,1,2,0);
     	int2lcdyx(kb_cnt_2lev,1,5,0);
		int2lcdyx(kb_full_ver,1,8,0);
		int2lcdyx(IKB,1,19,0);


		//int2lcdyx(num_of_wrks_bps,1,19,0);

		lcd_buffer[45]='a';
		int2lcd_mmm(bat[0]._Ib,'a',0);
		int2lcdyx(kb_start[0],2,9,0);
		int2lcdyx(bat[0]._av,2,15,0);
		lcd_buffer[65]='a';
		int2lcd_mmm(bat[1]._Ib,'a',0);
		int2lcdyx(kb_start[1],3,9,0);
		int2lcdyx(bat[1]._av,3,15,0);
    
    	
     
    	}  

	else if(sub_ind==3)
     	{
     	bgnd_par("*                   ",
     	         "                    ",
     	         "                    ",
     	         "                    ");
		int2lcdyx(load_U,0,4,0);
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
		/*int2lcd_mmm(Ibmax,'a',0);
		int2lcdyx(IZMAX,1,14,0);*/

	/*	lcd_buffer[65]='a';
		int2lcd_mmm(bat[0]._Ib,'a',0);

		lcd_buffer[70]='a';
		int2lcd_mmm(bat[1]._Ib,'a',0); */

	/*	lcd_buffer[75]='a';
		int2lcd_mmm(Ibmax,'a',0); */

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
 //    	char2lcdhyx(St,3,5);
		}

	else if(sub_ind==4)
     	{
     	bgnd_par(" ������             ",
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
     	bgnd_par("                    ",
     		    "                    ",
     		    "                    ",
     		    "                    ");
     		    
     	int2lcdyx(ad7705_buff_[0],1,5,0);
     	int2lcdyx(Kibat0[0],1,12,0);
     	int2lcdyx(Kibat1[0],1,17,0);
 
     	int2lcdyx(ad7705_buff_[1],2,5,0);
     	int2lcdyx(Kibat0[1],2,12,0);
     	int2lcdyx(Kibat1[1],2,17,0);


		lcd_buffer[70]='a';
		int2lcd_mmm(bat[0]._Ib,'a',0);

		
		lcd_buffer[78]='a';
		int2lcd_mmm(bat[1]._Ib,'a',0);

		int2lcdyx(samokalibr_cnt,3,4,0);

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

		int2lcdyx(adc_buff_[0],0,4,0);
    		int2lcdyx(adc_buff_[1],0,9,0);
     	int2lcdyx(adc_buff_[2],0,14,0);
     	int2lcdyx(adc_buff_[3],0,19,0); 
     	int2lcdyx(adc_buff_[4],1,4,0);	
     	int2lcdyx(adc_buff_[4],1,9,0);
     	int2lcdyx(adc_buff_[6],1,14,0);
     	int2lcdyx(adc_buff_[7],1,19,0); 
     	int2lcdyx(adc_buff_[8],2,4,0);
     	int2lcdyx(adc_buff_[9],2,9,0);
     	int2lcdyx(adc_buff_[10],2,14,0);
     	int2lcdyx(adc_buff_[11],2,19,0);
     	int2lcdyx(adc_buff_[12],3,4,0);
     	int2lcdyx(adc_buff_[13],3,9,0);
     	int2lcdyx(adc_buff_[14],3,14,0);
     	int2lcdyx(adc_buff_[15],3,19,0);
    	}  		  		
		     	
  /*   else if(sub_ind==3)
     	{
     	bgnd_par(sm_,sm_,sm_,sm_);
 		//long2lcdyx(0x1fffffffUL,0,19);	 ad7705_buff[2][16]
 		//long2lcdyx(cnt_vz_sec,1,19);
 		//long2lcdyx(cnt_vz_sec_,2,19);
 		int2lcdyx(adc_buff_[5],0,19,0);
 		
 		char2lcdhyx(wrk,0,3);
 		int2lcdyx(cnt_wrk,1,3,0);
 		int2lcdyx(ibat_integr,2,8,0);
 		//long2lcdyx(ibat_integr_,3,8);
 		
 		int2lcdyx(lc640_read_int(WRK_PTR),0,6,0);
		int2lcdyx(lc640_read_int(WRK_CNT),0,9,0);
		int2lcdyx(but,3,18,0);
 		} */


 
/*   else if(sub_ind==2)
     	{
     	bgnd_par("                    ",
     		    "                    ",
     		    "                    ",
     		    "                    ");
     		    
     	int2lcdyx(adc_buff_[0],0,4,0);
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
     	int2lcdyx(adc_buff_[15],3,19,0);
    	}  	   */



 /*    else if(sub_ind==2)
     	{
     	bgnd_par(	" ��� - �������      ",
     			"                    "
     			,sm_,sm_);


	int2lcdyx(cnt_net_drv,1,4,0);
	int2lcdyx(ptr_can1_tx_wr,1,3,0);
	int2lcdyx(ptr_can1_tx_rd,1,7,0);
	int2lcdyx(ptr_can2_tx_wr,1,11,0);
	int2lcdyx(ptr_can2_tx_rd,1,15,0);	
	
	int2lcdyx(rotor_can[0],2,3,0);
	int2lcdyx(rotor_can[1],2,7,0);
	int2lcdyx(rotor_can[2],2,11,0);
	int2lcdyx(rotor_can[3],3,3,0);
	int2lcdyx(rotor_can[4],3,7,0);
	int2lcdyx(rotor_can[5],3,11,0);  */

     	
     	/*int2lcd_mmm((signed)(zar_cnt/10),'<',0);
     	int2lcd_mmm((signed)(zar_cnt_ke/10),'>',0);
     	int2lcdyx(lc640_read_int(EE_ZAR_CNT),0,16,0); 
     	int2lcdyx(lc640_read_int(EE_ZAR_CNT_KE),1,16,0);*/
		   
     	/*
     	int2lcdyx(ND[0],0,7,0);
     	int2lcdyx(cnt_av_t[0],1,7,0);
     	int2lcdyx(adc_buff_[6],0,15,0);
     	
     	int2lcdyx(t_i[1],2,3,0);
     	int2lcdyx(ND[1],2,7,0);
     	int2lcdyx(cnt_av_t[1],3,7,0);
     	int2lcdyx(adc_buff_[4],1,15,0);*/
     	
     	
     	//int2lcdyx(plazma_uart1,0,2,0);
     	/*int2lcdyx(rx_wr_index1,1,2,0);
     	int2lcdyx(rx_rd_index1,2,2,0);
     	int2lcdyx(rx_counter1,3,2,0);
//     	char2lcdbyx(U1LCR,0,19);
 //    	char2lcdbyx(U1IER,1,19);
 //    	char2lcdbyx(U1IIR,2,19);
 //    	char2lcdbyx(U1LSR,3,19);
     	int2lcdyx(adc_buff_out_[0],0,19,0);
     	int2lcdyx(adc_buff_out_[1],1,19,0);
     	int2lcdyx(adc_buff_out_[2],2,19,0);
     	int2lcdyx(in_stat_out[0],0,14,0);
     	int2lcdyx(in_stat_out[1],1,14,0);*/
     	//int2lcdyx(can_cnt,1,15,0);
     	//int2lcdyx(can_tx_cnt,2,15,0);
     	//long2lcdyx(C1ICR,0,8);
     	//int2lcdyx(bOUT_FREE,1,2,0);
     	/*int2lcdyx(ptr_can1_tx_wr,2,2,0);
     	int2lcdyx(ptr_can1_tx_rd,3,2,0);
     	long2lcdyx(can1_data[0],0,19);
     	long2lcdyx(can1_datb[0],0,11);
     	long2lcdyx(can1_data[1],1,19);
     	long2lcdyx(can1_datb[1],1,11);
     	long2lcdyx(can1_data[2],2,19);
     	long2lcdyx(can1_datb[2],2,11);
     	long2lcdyx(can1_data[3],3,19);
     	long2lcdyx(can1_datb[3],3,11);
     	char2lcdhyx(bd[0],0,3);*/
     /*	} */


 
  
 
 
  /*   else if(sub_ind==1)
     	{
     	bgnd_par("Un    ( �1   !�2   @",
     	         "U�    < U1   #U2   $",
     	         "I�    > I1   %I2   ^",
     	         "St      St1   St2   ");
 //    	lcd_buffer[4]='a';            
 //    	int2lcd_mmm(Ibat,'a',1);   int2lcdyx(cntrl_stat,0,9,0);          int2lcdyx(hour_apv_cnt,0,13,0);                             char2lcdhyx(St_[0],0,19);  
 //    	int2lcdyx(Ubat,1,4,0);     int2lcdyx(main_apv_cnt,1,9,0);        int2lcdyx(lc640_read_int(SRC1_AVAR_PTR),1,13,0);            char2lcdhyx(St_[1],1,19);
 //    	int2lcdyx(Us[0],2,4,0);  int2lcdyx(apv_cnt_1,2,9,0);           int2lcdyx(lc640_read_int(SRC1_AVAR_CNT),2,13,0);                                     int2lcdhyx(av_stat,2,19);
 //    	int2lcdyx(Us[1],3,4,0);  int2lcdyx(reset_apv_cnt,3,9,0);                                            int2lcdyx(plazma,3,19,0);
     	int2lcd(u_necc,'(',1);
 */
     	/*int2lcd(Us[0],'#',1);
     	int2lcd(Us[1],'$',1);
     	int2lcd(Is[0],'%',1);
     	int2lcd(Is[1],'^',1);*/
  /*   	int2lcd(bat_Ub[0],'<',1);
     	int2lcd_mmm(bat_Ib[0],'>',2);*/
  //   	char2lcdhyx(St_[0],3,13);
  //   	char2lcdhyx(St_[1],3,19);
 //    	char2lcdhyx(St,3,5);
   /*  	} */
   	
     	
 /*   else if(sub_ind==5)
     	{
     	bgnd_par("                    ",
     		    "                    ",
     		    "                    ",
     		    "                    ");
     	int2lcdyx(tout_max_cnt[0],0,1,0);	
     	int2lcdyx(tout_min_cnt[0],1,1,0); 
     	int2lcdyx(tout_stat[0],2,1,0);   
     	
     	int2lcdyx(TMAX_EXT[0],0,5,0);	
     	int2lcdyx(tout[0],1,5,0); 
     	int2lcdyx(TMIN_EXT[0],2,5,0); */       	
     		
/*     		char tout_max_cnt[3],tout_min_cnt[3];
enum {tNORM,tMAX,tMIN}tout_stat[3];
char in_stat_out_old[4];
enum {skON,skOFF}sk_stat[3];*/
     		    
     /*	int2lcdyx(spc_stat,0,1,0);
     	int2lcdyx(lc640_read_int(EE_BAT_C_NOM),1,4,0);	
     	int2lcdyx(lc640_read_int(EE_BAT_C_REAL),2,4,0);
     	long2lcdyx(zar_cnt_ke,0,19); 
     	long2lcdyx(zar_cnt,1,19);
      	int2lcdyx(lc640_read_int(EE_ZAR_CNT_KE),0,10,0);	
     	int2lcdyx(lc640_read_int(EE_ZAR_CNT),1,10,0);
     	int2lcdyx(zar_percent,3,3,0);    
  		int2lcdyx(lc640_read_int(KE_PTR),3,6,0);
		int2lcdyx(lc640_read_int(KE_CNT),3,9,0);*/
   /*  	}  */
/*	else if(sub_ind==6)
     	{
     	ptrs[0]="    ?        $ # @ !";
     	ptrs[1]="               J   j";
     	ptrs[2]="               Y   y";
     	ptrs[3]="   h   m     Q q Z z";
     	ptrs[4]="   s   S       C   D";
     	ptrs[5]="   f   F       c   d";
     	ptrs[6]="             A a B b";
     	
     	
     	bgnd_par(ptrs[index_set],
     	         ptrs[index_set+1],
     	         ptrs[index_set+2],
     	         ptrs[index_set+3]);
     	
     	int2lcd_mmm(bat_Ib[0],'?',2); 
  //   	int2lcd(OFFBP1,'$',0);
 //    	char2lcdh(St_[0],'#');
 //    	int2lcd(OFFBP2,'@',0);
 //    	char2lcdh(St_[1],'!');
     	//int2lcd(Us[0],'J',0);
     	//int2lcd(Us[1],'j',0);
     	int2lcd(hour_apv_cnt[0],'h',0);
     	int2lcd(main_apv_cnt,'m',0); 
     	int2lcd(lc640_read_int(SRC1_AVAR_PTR),'s',0); 
     	int2lcd(lc640_read_int(SRC1_AVAR_CNT),'S',0);
     	int2lcd(lc640_read((lc640_read_int(SRC1_AVAR_PTR)*4)+SRC1_AVAR_DAT),'e',0); 	
     	int2lcd(lc640_read_int(SRC2_AVAR_PTR),'f',0); 
     	int2lcd(lc640_read_int(SRC2_AVAR_CNT),'F',0);     	
     	int2lcd(apv_cnt[0],'Q',0);
     	int2lcd(apv_cnt_sec[0],'q',0);
		int2lcd(apv_cnt[1],'Z',0);
     	int2lcd(apv_cnt_sec[1],'z',0);
 //    	int2lcd(cnt_av_umax[0],'A',0);
 //    	int2lcd(cnt_av_umin[0],'a',0);   
//     	int2lcd(cnt_av_umax[1],'B',0);
//     	int2lcd(cnt_av_umin[1],'b',0); 
     	int2lcd(hour_apv_cnt[0],'C',0); 
     	int2lcd(hour_apv_cnt[1],'D',0);
     	int2lcd(reset_apv_cnt[0],'c',0); 
     	int2lcd(reset_apv_cnt[1],'d',0);  */   	    	  	
 //    	lcd_buffer[4]='a';            
 //    	  int2lcdyx(cntrl_stat,0,9,0);          int2lcdyx(hour_apv_cnt,0,13,0);                             char2lcdhyx(St_[0],0,19);  
 //    	int2lcdyx(Ubat,1,4,0);     int2lcdyx(main_apv_cnt,1,9,0);                    char2lcdhyx(St_[1],1,19);
 //    	int2lcdyx(Us[0],2,4,0);  int2lcdyx(apv_cnt_1,2,9,0);                                                int2lcdhyx(av_stat,2,19);
 //    	int2lcdyx(Us[1],3,4,0);  int2lcdyx(reset_apv_cnt,3,9,0);                                            int2lcdyx(plazma,3,19,0);
 /*    	int2lcd(plazma,'(',0);
     	int2lcd(cntrl_stat1,'!',0);
     	int2lcd(cntrl_stat2,'@',0);
     	int2lcd(Us[0],'#',1);
     	int2lcd(Us[1],'$',1);
     	int2lcd(Is[0],'%',1);
     	int2lcd(Is[1],'^',1);
     	int2lcd(Ubat,'<',1);
     	int2lcd_mmm(Ibat,'>',2);
     	char2lcdhyx(St_[0],3,13);
     	char2lcdhyx(St_[1],3,19);
     	char2lcdhyx(St,3,5);*/
	/*	}  */   	       	
		
/*	else if(sub_ind==7)
     	{
     	bgnd_par(sm_,sm_,sm_,sm_);
     	//long2lcdyx(C1IER,0,10);
  		//long2lcdyx(C1GSR,1,10);
  		int2lcdyx(month__,0,6,0);
  		int2lcdyx(AVZ,1,6,0);
  		int2lcdyx(lc640_read_int(EE_MONTH_AVZ),2,6,0);
  		int2lcdyx(year__,0,16,0);
  		
  		int2lcdyx(lc640_read_int(EE_YEAR_AVZ),2,16,0);
  		//long2lcdyx(C1RFS,0,19);
  		//long2lcdyx(C1RID,1,19);
  		//long2lcdyx(C1RDA,2,19);
  		//long2lcdyx(C1RDB,3,19);
  		//long2lcdyx(plazma_can_long,3,10);
		}   */  	       			
     }

else if((ind==iAv_view)||(ind==iAv_view_avt))
	{
	unsigned short tempUI,tempUI_;
    	unsigned long tempUL;
	
	bgnd_par(sm_,sm_,sm_,sm_);
	if(sub_ind==0)
		{	
		if(avar_stat&0x00000001)
			{
			bgnd_par(	"    ������  ����    ",
				    	"    �� ���������    ",
				    	sm_,sm_); 
			int2lcd(net_U,']',0);
			}
    		else 
			{
	    		bgnd_par(	"    ������  ����    ",
	    				"     ���������      ",
					sm_,sm_); 
			}
		}
	else if((sub_ind==1)||(sub_ind==2))
		{
		if(avar_stat&(1<<sub_ind))
			{
			bgnd_par(	"   ������ ���. N!   ",
				    	"    �� ���������    ",
				    	sm_,sm_); 
			}
    		else 
			{
	    		bgnd_par(	"   ������ ���. N!   ",
	    				"     ���������      ",
					sm_,sm_); 
		
		     }
		int2lcd(sub_ind,'!',0);
		} 
     
	else if((sub_ind>=3)&&(sub_ind<=14))
		{
		if((sub_ind-2)<=9)					ptrs[0]=	"   ������ ��� N+    ";
		else 							ptrs[0]=	"   ������ ��� N +   ";
		if(bps[sub_ind-3]._last_avar==0)		ptrs[1]=	"     ��������!!!    ";
		else if(bps[sub_ind-3]._last_avar==1)	ptrs[1]=	"  �������� U���!!!  ";	
		else if(bps[sub_ind-3]._last_avar==2)	ptrs[1]=	"  �������� U���!!!  ";	
		else if(bps[sub_ind-3]._last_avar==3)	ptrs[1]=	"    ����������!!!   ";
		if(avar_stat&(1<<sub_ind)) 			ptrs[2]=	"    �� ���������    ";
		else								ptrs[2]=	"     ���������      ";	
										ptrs[3]=	"                    ";

		bgnd_par(ptrs[0],ptrs[1],ptrs[2],ptrs[3]);
		int2lcd((sub_ind-2),'+',0);
          
		//int2lcdxy(sub_ind,0x20,0);

		} 
		
	else if(sub_ind==4)
		{ 

		}

	else if(sub_ind==5)
		{

		}

	else if(sub_ind==6)
		{

		}

	else if(sub_ind==7)
		{

		} 
		
	else if(sub_ind==8)
		{

		}

	else if(sub_ind==9)
		{

		}

	else if(sub_ind==10)
		{

		}
	    		     
	else if(sub_ind==11)
		{

		} 
		
	else if(sub_ind==12)
		{

		}

	else if(sub_ind==13)
		{

		}

	else if(sub_ind==14)
		{

		}

	else if(sub_ind==15)
		{

		} 
					
	} 
	
else if(ind==iAvz)
	{
	
 	if(AVZ==AVZ_1) 		ptrs[0]=	" ��� � �����        ";
	else if(AVZ==AVZ_2) 	ptrs[0]=	" ��� � 2 ������     ";
	else if(AVZ==AVZ_3) 	ptrs[0]=	" ��� � 3 ������     "; 
	else if(AVZ==AVZ_6) 	ptrs[0]=	" ��� � �������      ";
	else if(AVZ==AVZ_12) 	ptrs[0]=	" ��� � ���          ";
	else 				ptrs[0]=	" ��������           "; 
	
	ptrs[1]=						" ������������    (�.";
	if(AVZ!=AVZ_OFF)
		{
		ptrs[2]=					" ��������� ���������";
		ptrs[3]=					"  0%  &0^  0@:0#:0$ ";
		ptrs[4]=					sm_exit;
		}
	else ptrs[2]=						sm_exit;

	if(sub_ind<index_set) index_set=sub_ind;
	else if((sub_ind-index_set)>1) index_set=sub_ind-1;
	if((sub_ind==2)&&(AVZ!=AVZ_OFF)) index_set=2;
	
	bgnd_par(	"   ��������������   ",
			"������������� ����� ",
			ptrs[index_set],
			ptrs[index_set+1]);

	pointer_set(2);
		
	int2lcd(HOUR_AVZ,'@',0);
	int2lcd(MIN_AVZ,'#',0);
	int2lcd(SEC_AVZ,'$',0);
	int2lcd(DATE_AVZ,'%',0);
	int2lcd(YEAR_AVZ,'^',0);

	sub_bgnd(sm_mont[MONTH_AVZ],'&',-2);

	int2lcd(AVZ_TIME,'(',0);
	
	}
	        	
	 

/*
const char sm7[]	={" �������� N2        "}; //
const char sm8[]	={" ��������           "}; //
const char sm9[]	={" ����               "}; //
const char sm10[]	={" �����������        "}; // 
const char sm11[]	={" ������ ������      "}; //
const char sm12[]	=" ���������� ������  "}; //
const cha		=" �������            "}; //
*/


//char2lcdhyx(bat_rel_stat[0],0,10);
//char2lcdhyx(bat_rel_stat[1],0,15);
//int2lcdyx(u_necc,0,19,0);
//int2lcdyx(cntrl_stat,0,5,0); 	   mess_cnt[i]

//char2lcdhyx(bat_rel_stat[0],0,5);
//char2lcdhyx(bat_rel_stat[1],0,10);
//int2lcdyx(mess_cnt[1],0,2,0);
//int2lcdyx(GET_REG(IOPIN1,21,1),0,5,0); 
//int2lcdyx(samokalibr_cnt,0,10,0);
//char2lcdhyx(rele_stat,0,19);
//char2lcdhyx(mess_cnt[1],0,16); 

//int2lcdyx(ad7705_res1,0,8,0);
//int2lcdyx(ad7705_res2,0,16,0); 
//	int2lcdyx(bat[0]._cnt_to_block,0,1,0);
//	int2lcdyx(bat[1]._cnt_to_block,0,3,0);
//	int2lcdyx(bat[0]._rel_stat,0,5,0);
/*	int2lcdyx(ind,0,3,0); 
	int2lcdyx(sub_ind,0,6,0);
	int2lcdyx(index_set,0,9,0);
	int2lcdyx(ptr_ind,0,14,0);
	int2lcdyx(ind_pointer,0,19,0);*/ 
}


#define BUT0	16
#define BUT1	17
#define BUT2	18
#define BUT3	19
#define BUT4	20   
#define BUT_MASK (1UL<<BUT0)|(1UL<<BUT1)|(1UL<<BUT2)|(1UL<<BUT3)|(1UL<<BUT4)

#define BUT_ON 4
#define BUT_ONL 20 

#define butU   254
#define butU_  126
#define butD   253
#define butD_  125
#define butL   251
#define butL_  123
#define butR   247
#define butR_  119
#define butE   239
#define butE_  111
#define butUD  252
#define butLR   243

//-----------------------------------------------
void but_drv(void)
{

but_n=IO1PIN|(0xFFFFFFFFUL&(~(1UL<<BUT0))&(~(1UL<<BUT1))&(~(1UL<<BUT2))&(~(1UL<<BUT3))&(~(1UL<<BUT4)));
if((but_n==0xffffffffUL)||(but_n!=but_s))
 	{
 	speed=0;
 
   	if (((but0_cnt>=BUT_ON)||(but1_cnt!=0))&&(!l_but))
  		{
   	     n_but=1;
          but=*(((char*)&but_s)+2);

          }
   	if (but1_cnt>=but_onL_temp)
  		{
   	     n_but=1;
 
          but=*(((char*)&but_s)+2)&0x7f;
          }
    	l_but=0;
   	but_onL_temp=BUT_ONL;
    	but0_cnt=0;
  	but1_cnt=0;          
     goto but_drv_out;
  	}
if(but_n==but_s)
 	{
  	but0_cnt++;
  	if(but0_cnt>=BUT_ON)
  		{
   		but0_cnt=0;
   		but1_cnt++;
   		if(but1_cnt>=but_onL_temp)
   			{              
    			but=*(((char*)&but_s)+2)&0x7f;;
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



PINSEL2&=~(1UL<<((BUT0-16)*2))&~(1UL<<(((BUT0-16)*2)+1))
	   &~(1UL<<((BUT1-16)*2))&~(1UL<<(((BUT1-16)*2)+1))
	   &~(1UL<<((BUT2-16)*2))&~(1UL<<(((BUT2-16)*2)+1))
	   &~(1UL<<((BUT3-16)*2))&~(1UL<<(((BUT3-16)*2)+1))
	   &~(1UL<<((BUT4-16)*2))&~(1UL<<(((BUT4-16)*2)+1));
IO1DIR&=~(1UL<<BUT0)&~(1UL<<BUT1)&~(1UL<<BUT2)&~(1UL<<BUT3)&~(1UL<<BUT4);
	   
}


//-----------------------------------------------
void but_an(void)
{
signed short temp_SS;
signed short deep,i,cap,ptr;
char av_head[4];
if(!n_but)goto but_an_end;

av_beep=0x0000;
av_rele=0x0000;


if(but==butUD)
     {
     if(ind!=iDeb)
          {
		tree_up(iDeb,0,0,0);
          }
     else 
          {
		tree_down(0,0);
          }     
     }

else if(ind==iDeb)
	{
	if(but==butR)
		{
		sub_ind++;
		index_set=0;
		gran_ring_char(&sub_ind,0,7);
		}
	else if(but==butL)
		{
		sub_ind--;
		index_set=0;
		gran_ring_char(&sub_ind,0,7);
		}
		
	else if(sub_ind==0)
		{
		if(but==butU)
	     	{
	     	sub_ind1--;
	     	gran_char(&sub_ind1,0,NUMIST);
	     	}
		if(but==butD)
	     	{
	     	sub_ind1++;
	     	gran_char(&sub_ind1,0,NUMIST);
	     	}
	     
		if(but==butE)
	     	{
	     	SET_REG(C2GSR,3,24,8);
			C2MOD=0;
			 bOUT_FREE2=1;
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
          can2_out(1,2,3,4,5,6,7,8);
          }   
          
     else if(but==butE_)
         	{
          //a=b[--ptr_ind];
          can1_out_adr(TXBUFF,3);
          }                      				
	}

else if(ind==iMn)
	{
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,8+NUMBAT+NUMIST+NUMINV+(NUMEXT!=0));
		}
		
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,8+NUMBAT+NUMIST+NUMINV+(NUMEXT!=0));
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
		    	if(BAT_IS_ON[0]!=bisON)tree_up(iBat,0,0,1);
		    	else tree_up(iBat,0,0,sub_ind-1);
		    	}
		else if((sub_ind>NUMBAT)&&(sub_ind<=(NUMBAT+NUMIST)))
		    	{
		    	tree_up(iBps,0,0,sub_ind-(1+NUMBAT));
		    	}
		else if((sub_ind>(NUMBAT+NUMIST))&&(sub_ind<=(NUMBAT+NUMIST+NUMINV)))
		    	{
		    	tree_up(iBps,0,0,sub_ind-(1+NUMBAT+NUMIST));
		    	}
		else if(sub_ind==(1+NUMBAT+NUMIST+NUMINV))
			{
			tree_up(iNet,0,0,0);
		     ret(1000);
			}
		else if(sub_ind==(2+NUMBAT+NUMIST+NUMINV))
			{
			tree_up(iLoad,0,0,0);
		     ret(1000);
			}
		else if((sub_ind==(3+NUMBAT+NUMIST+NUMINV))&&(NUMEXT))
			{
			tree_up(iExtern,0,0,0);
		     ret(1000);
			}

		else if(sub_ind==(3+NUMBAT+NUMIST+NUMINV+(NUMEXT!=0)))
			{
			tree_up(iSpc,0,0,0);
		     ret(1000);
			}

		else if(sub_ind==(4+NUMBAT+NUMIST+NUMINV+(NUMEXT!=0)))
			{
			tree_up(iSet_prl,0,0,0);
		     ret(50);
		     parol_init();
			}
		else if(sub_ind==(5+NUMBAT+NUMIST+NUMINV+(NUMEXT!=0)))
			{
			tree_up(iLog,0,0,0);
		     ret(1000);
			}
		else if(sub_ind==(6+NUMBAT+NUMIST+NUMINV+(NUMEXT!=0)))
			{
			sub_ind=0;
			}
		else if(sub_ind==(7+NUMBAT+NUMIST+NUMINV+(NUMEXT!=0)))
			{
			tree_up(iBatLog,0,0,0);
		     ret(1000);
			}
		else if(sub_ind==(8+NUMBAT+NUMIST+NUMINV+(NUMEXT!=0)))
			{
			tree_up(iBatLog,0,0,1);
		     ret(1000);
			}
		}
    	
	else if(sub_ind==100)
		{
		if(but==butE)
		     {
		     tree_up(iBat,0,0,0);
		     ret(1000);
		     }
		}
     else if(sub_ind==20)
		{
		if(but==butE)
		     {
		     tree_up(iBps,0,0,0);
		     ret(1000);
		     }
		}
     else if(sub_ind==30)
		{
		if(but==butE)
		     {
		     tree_up(iBps,0,0,1);
		     ret(1000);
		     }
		}		

     else if(sub_ind==40)
		{
		if(but==butE)
		     {
		     tree_up(iLoad,0,0,0);
		     ret(1000);
		     }
		}		

     else if(sub_ind==50)
		{
		if(but==butE)
		     {
		     tree_up(iNet,0,0,0);
		     ret(1000);
		     }
		}	

     else if(sub_ind==60)
		{
		if(but==butE)
		     {

		     tree_up(iSpc,0,0,0);
		     ret(1000);
		     }
		}	

     else if(sub_ind==70)
		{
		if(but==butE)
		     {
		     tree_up(iJAv_sel,0,0,0);
		     ret(1000);
		     }
		}	



    	else if(sub_ind==90)
		{
		if(but==butE)
		     {
			tree_up(iAusw,0,0,0);
		     ret(1000);
			}
		}	

 
     else if(sub_ind==110)
		{
		if(but==butE)
		     {
	//		St_[0]&=0xe3;
	//		St_[1]&=0xe3;
			}
		}		
     else if(sub_ind==7+NUMBAT+NUMIST+NUMINV)
		{
		if(but==butE)
		     {
			sub_ind=0;
			}
		}	
		
	else if(sub_ind==8+NUMBAT+NUMIST+NUMINV)
		{
		if(but==butE)
		     {
			tree_up(iJ_bat,0,0,0);
		     ret(1000);
			}
		}		
     else if(sub_ind==9+NUMBAT+NUMIST+NUMINV)
		{
		if(but==butE)
		     {
			tree_up(iJ_bat,0,0,1);
		     ret(1000);
			}
		}			
				
	}

else if(ind==iBat)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,4);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,4);
		}
	else if((but==butL)||((sub_ind==4)&&(but==butE)))
	     {
	     tree_down(0,0);
	     ret(0);
	     }
	else if(but==butD_)
		{
		sub_ind=4;
		}		     
     }

else if(ind==iBps)
	{
	ret_ind(0,0,0);
	if (but==butU)
		{      
		sub_ind--;
		if(sub_ind==3)sub_ind=1;
		else if(sub_ind==1)sub_ind=0;
		gran_char(&sub_ind,0,simax);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		if(sub_ind<3)sub_ind=3;
		gran_char(&sub_ind,0,simax);
		}
		
	else if((but==butE)&&(sub_ind==4))
		{
		can2_out(sub_ind1,sub_ind1,CMND,ALRM_RES,0,0,0,0);
		}
				
	else if(((but==butE)&&(sub_ind==5))||(but))
		{
	     tree_down(0,0);
	     ret(0);
		}	
		
	}		
else if(ind==iNet)
	{
	ret(1000);
	if((but==butL)||(but==butE))
	     {
	     tree_down(0,0);
	     ret(0);
	     }
	}
else if(ind==iLoad)
	{
	ret(1000);
	if((but==butL)||(but==butE))
	     {
	     tree_down(0,0);
	     ret(0);
	     }
	}	

else if(ind==iExtern)
	{
	ret(1000);
	if (but==butU)
		{      
		sub_ind--;
		gran_char(&sub_ind,0,NUMEXT);
		}
		
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,NUMEXT);		
		}
		
	else if((but==butE)&&(sub_ind==NUMEXT))
		{
	     tree_down(0,0);
	     ret(0);
		}	
	}	


else if((ind==iPrl_bat_in_out)||(ind==iSet_prl)||(ind==iK_prl)
	||(ind==iSpc_prl_vz)||(ind==iSpc_prl_ke)||(ind==iAusw_prl)
	||(ind==iPrltst))
	{
	ret(50);
	if(but==butR)
		{
		sub_ind++;
		gran_ring_char(&sub_ind,0,2);
		}
	else if(but==butL)
		{
		sub_ind--;
		gran_ring_char(&sub_ind,0,2);
		}	
	else if(but==butU)
		{
		parol[sub_ind]++;
		gran_ring_char(&parol[sub_ind],0,9);
		}	
	else if(but==butD)
		{
		parol[sub_ind]--;
		gran_ring_char(&parol[sub_ind],0,9);
		}	
	else if(but==butE)
		{
		unsigned short tempU;
		tempU=parol[2]+(parol[1]*10U)+(parol[0]*100U);
		
		if(ind==iPrl_bat_in_out)
		     {
		     if(BAT_IS_ON[sub_ind1]!=bisON)
		          {
		          if(tempU==PAROL_BAT_IN)
		               {
		               lc640_write_int(ADR_EE_BAT_IS_ON[sub_ind1],bisON);
		               lc640_write_int(ADR_EE_BAT_DAY_OF_ON[sub_ind1],day__);
		               lc640_write_int(ADR_EE_BAT_MONTH_OF_ON[sub_ind1],month__);
		               lc640_write_int(ADR_EE_BAT_YEAR_OF_ON[sub_ind1],year__);
		               lc640_write_int(ADR_EE_BAT_C_REAL[sub_ind1],0x5555);
		               lc640_write_int(ADR_EE_BAT_RESURS[sub_ind1],0);
					lc640_write_int(ADR_EE_BAT_ZAR_CNT[sub_ind1],0);
		               
		               lc640_write(KE_PTR,0);
					lc640_write(VZ_PTR,0);
					lc640_write(WRK_PTR,0);
					lc640_write(KE_CNT,0);
					lc640_write(VZ_CNT,0);
					lc640_write(WRK_CNT,0);
					lc640_write(BAT_AVAR_CNT,0);
					lc640_write(BAT_AVAR_PTR,0);					
		               
                         tree_down(0,0);
                         ret(0);
		               }
		          else
		               {
		               tree_down(0,0);
	    	               show_mess("                    ",
	          				"       ������       ",
	          				"     ��������!!!    ",
	          				"                    ",1000);
     	               }
		          }      
               else		          
		          {
		          if(tempU==PAROL_BAT_OUT)
		               {
		               lc640_write_int(ADR_EE_BAT_IS_ON[sub_ind1],bisOFF);
		               lc640_write_int(ADR_EE_BAT_DAY_OF_ON[sub_ind1],day__);
		               lc640_write_int(ADR_EE_BAT_MONTH_OF_ON[sub_ind1],month__);
		               lc640_write_int(ADR_EE_BAT_YEAR_OF_ON[sub_ind1],year__);

		               tree_down(0,0);
		               ret(0);
		               
		               }
	               else
		               {
		               tree_down(0,0);
	    	               show_mess("                    ",
	          				"       ������       ",
	          				"     ��������!!!    ",
	          				"                    ",1000);
		               }		               
		          }     
               }
		
		else if(ind==iSet_prl)
			{
	     	if(tempU==PAROL_SET) 
				{
				tree_down(0,0);
				tree_up(iSet,0,0,0);
				ret(1000);
				}
			else 
				{
		          tree_down(0,0);
	    	          show_mess("                    ",
	          			"       ������       ",
	          			"     ��������!!!    ",
	          			"                    ",1000);
				}
			}
		else	if(ind==iK_prl)
			{
	     	if(tempU==PAROL_KALIBR) 
				{
				tree_down(0,0);
				tree_up(iK,0,0,0);
				show_mess(	"�������� ���-�� ����",
 							"  �������,��������  ",
 							"   ���������� ���   ",
 							"   �������� 4-10�   ",3000);
				
				}
			else 
				{
		          tree_down(0,0);
	    	          show_mess("                    ",
	          			"       ������       ",
	          			"     ��������!!!    ",
	          			"                    ",1000);
				}
			} 
	
		else	if(ind==iAusw_prl)
			{
	     	if(tempU==PAROL_AUSW) 
				{
				tree_down(0,0);
				tree_up(iAusw_set,1,0,0);
				default_temp=10;
				ret(0);
				}
			else 
				{
		          tree_down(0,0);
	    	          show_mess("                    ",
	          			"       ������       ",
	          			"     ��������!!!    ",
	          			"                    ",1000);
				}
			} 	
			
		else	if(ind==iSet_st_prl)
			{
	     	if(tempU==PAROL_DEFAULT) 
				{
	//			ind=iDefault;
				sub_ind=1;
				index_set=0;
				default_temp=10;
				}
			else 
				{
		          tree_down(0,0);
	    	          show_mess("                    ",
	          			"       ������       ",
	          			"     ��������!!!    ",
	          			"                    ",1000);
				}
			} 
						
		else if(ind==iPrltst)
			{
			if(tempU==PAROL_TST) 
				{
				tree_down(0,0);
				tree_up(iTst,1,0,0);
				tst_state[0]=tstOFF;
				tst_state[1]=tstOFF;
				tst_state[2]=tstOFF;
				tst_state[3]=tstOFF;
				tst_state[4]=tstOFF;
				tst_state[5]=tstOFF;
				}
	  		else 
				{
		          tree_down(0,0);
	    	          /*show_mess("                    ",
	          			"       ������       ",
	          			"     ��������!!!    ",
	          			"                    ",1000);*/
				}
			}
		else if(ind==iSpc_prl_ke)
			{
			if(tempU==PAROL_KE) 
				{
				char temp;
				temp=sub_ind1;
				tree_down(0,0);
				tree_up(iKe,0,0,temp);
				ret(1000);
				}
	  		else 
				{	
		          tree_down(0,0);
	    	          show_mess("                    ",
	          			"       ������       ",
	          			"     ��������!!!    ",
	          			"                    ",1000);
				}
			}
		else if(ind==iSpc_prl_vz)
			{
			if(tempU==PAROL_VZ) 
				{
				tree_down(0,0);
				tree_up(iVz,0,0,0);
				ret(1000);
				}
	  		else 
				{
		          tree_down(0,0);
	    	          show_mess("                    ",
	          			"       ������       ",
	          			"     ��������!!!    ",
	          			"                    ",1000);
				}     	          
			}
		}
	}

else if(ind==iSpc)
	{
	ret_ind(0,0,0);
	ret_ind_sec(iMn,60);
	if (but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,4);
		}
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,4);
		}
	else if(but==butE)
		{
		if(sub_ind==0)
			{   
               tree_up(iSpc_prl_vz,0,0,0);
			}
		else if(sub_ind==1)
			{
               tree_up(iAvz,0,0,0);
               parol_init();
			}			
		else if((sub_ind==2)||(sub_ind==3))
			{
               tree_up(iSpc_prl_ke,0,0,sub_ind-2);
              	parol_init();
			} 
	/*	else if(sub_ind==4)
			{
			tree_up(iAKE,0,0,0);
			}	
		else if(sub_ind==5)
			{
			tree_up(iAKE,0,0,1);
			}*/						
		else if(sub_ind==4)
			{
			tree_down(0,0);
			}	
		}
	else if(but==butL)
		{
		tree_down(0,0);
		}			
	}

else if(ind==iVz)
	{
	ret_ind(0,0,0);
	ret_ind_sec(0,0);
	if (but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,2);
		}
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,2);
		}
	else if(sub_ind==0)
		{
		if(spc_stat!=spcVZ)
			{
			if(but==butR)
				{
				VZ_HR++;
				}
			else if(but==butL)
				{
				VZ_HR--;
				}
			gran(&VZ_HR,1,10);
			lc640_write_int(EE_VZ_HR,VZ_HR);
			}			
          }
	else if(sub_ind==1)
		{
          if(spc_stat!=spcVZ)
          	{
          	char temp;
          	temp=vz_start(VZ_HR);
          	if(temp==22) 
          		{
          		sub_ind=22;
          		ret_ind(iVz,1,5);
          		} 
			else if(temp==33) 
          		{
          		sub_ind=33;
          		ret_ind(iVz,1,5);
          		}          		
          	}    
         	else if(spc_stat==spcVZ)
          	{
          	vz_stop();
          	}             	 
		}			
	else if(sub_ind==2)
		{                 
		if(but==butE)
			{
			tree_down(0,0);
			}
          } 
	}

else if(ind==iKe)
	{
	ret_ind(0,0,0);
	ret_ind_sec(0,0);
	if (but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,1);
		}
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,1);
		}
	else if(sub_ind==0)
		{
		if(but==butE)
			{
			if((spc_stat==spcKE)&&(spc_bat==sub_ind1))
				{
				spc_stat=spcOFF;
				}
			else
				{

				ke_start(sub_ind1);
				if(ke_start_stat==kssNOT)
					{
					show_mess("��������� ��������  ",
	          				"  ������� �������   ",
	          				"     ����������.    ",
	          				" ������� ���������� ",
							3000);
					}
				else if(ke_start_stat==kssNOT_VZ)
					{
					show_mess("��������� ��������  ",
	          				"  ������� �������   ",
	          				" ����������. ����   ",
	          				"������������� ����� ",
							3000);
					}

				else if(ke_start_stat==kssYES)
					{
					if(sub_ind==0)
					show_mess("  �������� �������  ",
	          				"     ������� N1     ",
	          				"     �������!!!     ",
	          				"                    ",
							3000);
					else 
					show_mess("  �������� �������  ",
	          				"     ������� N2     ",
	          				"     �������!!!     ",
	          				"                    ",
							3000);

					}


				/*
				if(ke_start(sub_ind1)==1)
					{
					sub_ind=11;
					ret_ind(iKe,0,10);
					}
				else if(ke_start(sub_ind1)==2)
					{
					sub_ind=12;
					ret_ind(iKe,0,10);
					}			
				else if(ke_start(sub_ind1)==3)
					{
					sub_ind=13;
					ret_ind(iKe,0,10);
					}     
				else if(ke_start(sub_ind1)==4)
					{
					sub_ind=14;
					ret_ind(iKe,0,10);
					}     			
				else if(ke_start(sub_ind1)==5)
					{
					sub_ind=15;
					ret_ind(iKe,0,10);
					}			
				else if(ke_start(sub_ind1)==6)
					{
					sub_ind=16;
					ret_ind(iKe,0,10);
					}     
				else if(ke_start(sub_ind1)==7)
					{
					sub_ind=17;
					ret_ind(iKe,0,10);
					}  
				else if(ke_start(sub_ind1)==8)
					{
					sub_ind=18;
					ret_ind(iKe,0,10);
					}     			
				else if(ke_start(sub_ind1)==9)
					{
					sub_ind=19;
					ret_ind(iKe,0,10);
					}			
				else if(ke_start(sub_ind1)==10)
					{
					sub_ind=20;
					ret_ind(iKe,0,10);
					}     
				else if(ke_start(sub_ind1)==11)
					{
					sub_ind=21;
					ret_ind(iKe,0,10);
					}   
				else if(ke_start(sub_ind1)==12)
					{
					sub_ind=22;
					ret_ind(iKe,0,10);
					} */ 					  								   										
				}
			}						
          }
	
	else if(sub_ind==1)
		{                 
		if(but==butE)
			{
			tree_down(0,0);
			}
          } 
     else sub_ind=0;     
	}


else if(ind==iLog)
	{
	ret_ind_sec(0,0);
	ret_ind(0,0,0);
	if (but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,av_j_si_max+1);
		}
	else if (but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,av_j_si_max+1);
          
		}  

	else if (but==butD_)
		{
		sub_ind=av_j_si_max;
		} 
		 
	else if (but==butL)
		{
		tree_down(0,0);
		}  
		
	else if(but==butE)
		{  
		if(sub_ind==av_j_si_max+1)
			{
			lc640_write(CNT_EVENT_LOG,0);
			lc640_write(PTR_EVENT_LOG,0);
			tree_down(0,0);
			avar_ind_stat=0;
			avar_stat=0;
			avar_stat_old=0;				
			}
					
		else if(sub_ind==av_j_si_max)
			{
			tree_down(0,0);
			}
			
		else 
			{
			/*ind=iLog_;
			sub_ind1=sub_ind;
			index_set=0;
			sub_ind=0;*/
			tree_up(iLog_,0,0,sub_ind);
			}	
			
		} 

	else if(but==butR)
		{
	    //	avar_bat_hndl(0,1);	
		}
	else if(but==butR_)
		{
	    	//avar_bat_hndl(0,0);	
		}		
	else if(but==butL)
		{
	    	//avar_s_hndl(1,0,1);	
		}
				
	else if(but==butL_)
		{           
		/*lc640_write(CNT_EVENT_LOG,0);
		lc640_write(PTR_EVENT_LOG,0);
		ind=iMn;
		sub_ind=cnt_of_slave+10;
		index_set=0;*/				
	
		}	 		
	}

else if(ind==iLog_)
	{          
	if(but==butU)
		{
		index_set--;
		gran_char(&index_set,0,av_j_si_max);
		}
	else if(but==butD)
		{
		index_set++;
		gran_char(&index_set,0,av_j_si_max);
		}
	else 
		{
		/*ind=iLog;
		sub_ind=sub_ind1;*/
		tree_down(0,0/*sub_ind1-sub_ind*/);
		}		
	}	

else if(ind==iSet)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if(sub_ind==5)index_set=4;
		if(sub_ind==6)sub_ind=8;
		if(sub_ind==9)index_set=8;
		if(sub_ind==10)sub_ind=11;
		
		gran_char(&sub_ind,0,30);
		}
	else if(but==butU)
		{
		sub_ind--;
		if(sub_ind==6)sub_ind=5;
		if(sub_ind==10)sub_ind=9;
		
		gran_char(&sub_ind,0,30);
		}
	else if(but==butD_)
		{
		sub_ind=29;
		}
		
	else if(sub_ind==0)
	     {
	     if(but==butE)
	          {
	          tree_up(iDef,0,0,0);
	          ret(1000);
	          default_temp=10;
	          }
	     }	
	
     else if(sub_ind==1)
		{
		if(but==butE)
		     {
		     tree_up(iSet_T,0,0,0);
		     ret(1000);
		     phase=0;
		     }
		}	
					 
     else if(sub_ind==2)
		{
		if(but==butE)
		     {
		     tree_up(iStr,0,0,0);
		     ret(1000);
		     index_set=0;
		     }
		}	
	
	else if(sub_ind==3)
	     {
	     if(but==butR)MNEMO_TIME++;
	     else if(but==butR_)MNEMO_TIME+=10;
	     else if(but==butL)MNEMO_TIME--;
	     else if(but==butL_)MNEMO_TIME-=10;

	     if(((MNEMO_TIME<5)||(MNEMO_TIME>60))&&(MNEMO_ON!=mnOFF))lc640_write_int(EE_MNEMO_ON,mnOFF);	
	     if(((MNEMO_TIME>=5)&&(MNEMO_TIME<=60))&&(MNEMO_ON!=mnON))lc640_write_int(EE_MNEMO_ON,mnON);
	     gran((signed short*)&MNEMO_TIME,4,61);
	     lc640_write_int(EE_MNEMO_TIME,MNEMO_TIME);
	     speed=1;
	     }
				     		
	else if(sub_ind==4)
	     {
		if(ZV_ON)ZV_ON=0;
		else ZV_ON=1;
	     lc640_write_int(EE_ZV_ON,ZV_ON);
	     speed=1;
	     }	
	
	else if(sub_ind==5)
	     {
		if(AV_OFF_AVT)AV_OFF_AVT=0;
		else AV_OFF_AVT=1;
	     lc640_write_int(EE_AV_OFF_AVT,AV_OFF_AVT);
	     speed=1;
	     }	

	else if(sub_ind==7)
	     {
	     if(but==butE)
	          {
	          tree_up(iApv,0,0,0);
	          ret(1000);
	          }
	     }	

	else if(sub_ind==8)
	     {
		if(PAR)PAR=0;
		else PAR=1;
	     lc640_write_int(EE_PAR,PAR);
	     speed=1;
	     }

	else if(sub_ind==9)
	     {
	     if(but==butR)TBAT++;
	     else if(but==butR_)TBAT+=10;
	     else if(but==butL)TBAT--;
	     else if(but==butL_)TBAT-=10;
	     gran(&TBAT,5,60);
	     lc640_write_int(EE_TBAT,TBAT);
	     speed=1;
	     }	
	                    	     	
	else if(sub_ind==11)
	     {
	     if(but==butR)UMAX++;
	     else if(but==butR_)UMAX+=10;
	     else if(but==butL)UMAX--;
	     else if(but==butL_)UMAX-=10;
	     gran(&UMAX,10,1000);
	     lc640_write_int(EE_UMAX,UMAX);
	     speed=1;
	     }
	else if(sub_ind==12)
	     {
	     if(but==butR)DU++;
	     else if(but==butR_)DU+=10;
	     else if(but==butL)DU--;
	     else if(but==butL_)DU-=10;
	     gran(&DU,10,1000);
	     lc640_write_int(EE_DU,DU);
	     speed=1;
	     }	     
	else if(sub_ind==13)
	     {
	     if(but==butR)UB0++;
	     else if(but==butR_)UB0+=10;
	     else if(but==butL)UB0--;
	     else if(but==butL_)UB0-=10;
	     gran(&UB0,10,1000);
	     lc640_write_int(EE_UB0,UB0);
	     speed=1;
	     }
	     
	else if(sub_ind==14)
	     {
	     if(but==butR)UB20++;
	     else if(but==butR_)UB20+=10;
	     else if(but==butL)UB20--;
	     else if(but==butL_)UB20-=10;
	     gran(&UB20,10,1000);
	     lc640_write_int(EE_UB20,UB20);
	     speed=1;
	     }	

	else if(sub_ind==15)
	     {
	     if(but==butR)USIGN++;
	     else if(but==butR_)USIGN+=10;
	     else if(but==butL)USIGN--;
	     else if(but==butL_)USIGN-=10;
	     gran(&USIGN,1,500);
	     lc640_write_int(EE_USIGN,USIGN);
	     speed=1;
	     }	
	else if(sub_ind==16)
	     {
	     if(but==butR)UMN++;
	     else if(but==butR_)UMN+=10;
	     else if(but==butL)UMN--;
	     else if(but==butL_)UMN-=10;
	     gran(&UMN,1,220);
	     lc640_write_int(EE_UMN,UMN);
	     speed=1;
	     }	

	else if(sub_ind==17)
	     {
	     if(but==butR)U0B++;
	     else if(but==butR_)U0B+=10;
	     else if(but==butL)U0B--;
	     else if(but==butL_)U0B-=10;
	     gran(&U0B,10,1000);
	     lc640_write_int(EE_U0B,U0B);
	     speed=1;
	     }	
	     
	else if(sub_ind==18)
	     {
	     if(but==butR)IKB++;
	     else if(but==butR_)IKB+=10;
	     else if(but==butL)IKB--;
	     else if(but==butL_)IKB-=10;
	     gran(&IKB,1,500);
	     lc640_write_int(EE_IKB,IKB);
	     speed=1;
	     }		
            
	else if(sub_ind==19)
	     {
	     if(but==butR)IZMAX++;
	     else if(but==butR_)IZMAX+=10;
	     else if(but==butL)IZMAX--;
	     else if(but==butL_)IZMAX-=10;
	     gran(&IZMAX,10,1000);
	     lc640_write_int(EE_IZMAX,IZMAX);
	     speed=1;
	     }   

	else if(sub_ind==20)
	     {
	     if(but==butR)IMAX++;
	     else if(but==butR_)IMAX+=10;
	     else if(but==butL)IMAX--;
	     else if(but==butL_)IMAX-=10;
	     gran(&IMAX,1,120);
	     lc640_write_int(EE_IMAX,IMAX);
	     speed=1;
	     }		
	     
	else if(sub_ind==21)
	     {
	     if(but==butR)KIMAX++;
	     else if(but==butR_)KIMAX+=10;
	     else if(but==butL)KIMAX--;
	     else if(but==butL_)KIMAX-=10;
	     gran(&KIMAX,5,10);
	     lc640_write_int(EE_KIMAX,KIMAX);
	     speed=1;
	     }
	
	else if(sub_ind==22)
	     {
	     if ((but==butR)||(but==butR_))KVZ+=5;
		if ((but==butL)||(but==butL_))KVZ-=5;
		gran(&KVZ,1005,1030); 	          
		lc640_write_int(EE_KVZ,KVZ);
	     speed=1;
	     }
	     
	else if(sub_ind==23)
		{
		if ((but==butR)||(but==butR_))TZAS++;
		if ((but==butL)||(but==butL_))TZAS--;
		gran(&TZAS,0,3);
		lc640_write_int(EE_TZAS,TZAS);
		speed=1; 
		}	
			       	        
	else if(sub_ind==24)
	     {
	     if(but==butR)TMAX++;
	     else if(but==butR_)TMAX+=2;
	     else if(but==butL)TMAX--;
	     else if(but==butL_)TMAX-=2;
	     gran(&TMAX,10,100);
	     lc640_write_int(EE_TMAX,TMAX);
	     speed=1;
	     }	
	
	else if(sub_ind==25)
	     {
	     if(but==butR)TSIGN++;
	     else if(but==butR_)TSIGN+=2;
	     else if(but==butL)TSIGN--;
	     else if(but==butL_)TSIGN-=2;
	     gran(&TSIGN,10,100);
	     lc640_write_int(EE_TSIGN,TSIGN);
	     speed=1;
	     }	     
	else if(sub_ind==26)
	     {
	     if(but==butR)TBATMAX++;
	     else if(but==butR_)TBATMAX+=2;
	     else if(but==butL)TBATMAX--;
	     else if(but==butL_)TBATMAX-=2;
	     gran(&TBATMAX,10,100);
	     lc640_write_int(EE_TBATMAX,TBATMAX);
	     speed=1;
	     }	
	
	else if(sub_ind==27)
	     {
	     if(but==butR)TBATSIGN++;
	     else if(but==butR_)TBATSIGN+=2;
	     else if(but==butL)TBATSIGN--;
	     else if(but==butL_)TBATSIGN-=2;
	     gran(&TBATSIGN,10,100);
	     lc640_write_int(EE_TBATSIGN,TBATSIGN);
	     speed=1;
	     }	
     	     	     		     	     
    else if(sub_ind==28)
		{
		if(but==butE)
		     {
		     //tree_up(iExt_set,0,0,0);
		     //ret(1000);
		     }
		}		
     else if(sub_ind==29)
		{
		if(but==butE)
		     {
		     tree_down(0,0);
		     ret(0);
		     }
		}		
	else if(sub_ind==30)
		{
		if(but==butE)
		     {		
			tree_up(iK_prl,0,0,0);
			parol_init();
			ret(50);
			}						
		}			
	else if(sub_ind==31)
		{
		if(but==butE)
		     {
		     tree_up(iPrltst,0,0,0);
		     parol_init();
		     }
		}			
     }
#define SIMAXIDEF 6
else if(ind==iDef)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,SIMAXIDEF);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,SIMAXIDEF);
		}
	else if(but==butD_)
		{
		sub_ind=SIMAXIDEF;
		}
	
	else if(but==butE)
		{
		if(sub_ind==0)
			{
			def_set(600,564,545,44,100,480,4);
			//lc640_write_int(EE_AUSW_MAIN,4824);

			}
		else if(sub_ind==1)
			{
			def_set(750,705,681,55,100,600,4);
			//lc640_write_int(EE_AUSW_MAIN,6024);
			}	
		else if(sub_ind==2)
			{
			def_set(600,564,545,44,1600,480,5);
			//lc640_write_int(EE_AUSW_MAIN,4824);

			}
		else if(sub_ind==3)
			{
			def_set(750,705,681,55,100,600,5);
			//lc640_write_int(EE_AUSW_MAIN,6024);
			}
		else if(sub_ind==4)
			{
			def_set(600,564,545,44,100,480,6);
			//lc640_write_int(EE_AUSW_MAIN,4824);

			}
		else if(sub_ind==5)
			{
			def_set(750,705,681,55,100,600,6);
			//lc640_write_int(EE_AUSW_MAIN,6024);
			}

		else if(sub_ind==SIMAXIDEF)
			{
			tree_down(0,0);
			}
		default_temp=sub_ind;	
		}
     }
else if(ind==iSet_T)
	{
	signed char temp;
	if(but==butR)
		{
		sub_ind++;
		gran_char(&sub_ind,0,5);
		}
	else if(but==butL)
		{
		sub_ind--;
		gran_char(&sub_ind,0,5);
		}
	else if(but==butE)
		{
		tree_down(0,0);
		}	
	else if(sub_ind==0)
	     {
	     temp=hour__;
	     if((but==butU)||(but==butU_))
	          {
	          temp++;
	          gran_ring_char(&temp,0,23);
	          hour__=temp;
	          temp=((temp/10)<<4)+(temp%10);
	          pcf8563_write(0x04,temp);
	          }
          else if((but==butD)||(but==butD_))
	          {
	          temp--;
	          gran_ring_char(&temp,0,23);
	          hour__=temp;
	          temp=((temp/10)<<4)+(temp%10);
	          pcf8563_write(0x04,temp);
	          }	
	     speed=1;               
	     }
     else if(sub_ind==1)
	     {
	     temp=min__;
	     if((but==butU)||(but==butU_))
	          {
	          temp++;
	          gran_ring_char(&temp,0,59);
	          min__=temp;
	          temp=((temp/10)<<4)+(temp%10);
	          pcf8563_write(0x03,temp);
	          }
          else if((but==butD)||(but==butD_))
	          {
	          temp--;
	          gran_ring_char(&temp,0,59);
	          min__=temp;
	          temp=((temp/10)<<4)+(temp%10);
	          pcf8563_write(0x03,temp);
	          }	
	     speed=1;               
	     }
     else if(sub_ind==2)
	     {
	     temp=sec__;
	     if((but==butU)||(but==butU_))
	          {
	          temp=0;
	          gran_ring_char(&temp,0,59);
	          sec__=temp;
	          temp=((temp/10)<<4)+(temp%10);
	          pcf8563_write(0x02,temp);
	          }
          else if((but==butD)||(but==butD_))
	          {
	          temp=0;
	          gran_ring_char(&temp,0,59);
	          sec__=temp;
	          temp=((temp/10)<<4)+(temp%10);
	          pcf8563_write(0x02,temp);
	          }	
	     speed=1;               
	     }

     else if(sub_ind==3)
	     {
	     temp=day__;
	     if((but==butU)||(but==butU_))
	          {
	          temp++;
	          gran_ring_char(&temp,1,31);
	          day__=temp;
	          temp=((temp/10)<<4)+(temp%10);
	          pcf8563_write(0x05,temp);
	          }
          else if((but==butD)||(but==butD_))
	          {
	          temp--;
	          gran_ring_char(&temp,1,31);
	          day__=temp;
	          temp=((temp/10)<<4)+(temp%10);
	          pcf8563_write(0x05,temp);
	          }	
	     speed=1;               
	     }
     else if(sub_ind==4)
	     {
	     temp=month__;
	     if((but==butU)||(but==butU_))
	          {
	          temp++;
	          gran_ring_char(&temp,1,12);
	          month__=temp;
	          temp=((temp/10)<<4)+(temp%10);
	          pcf8563_write(0x07,temp);
	          }
          else if((but==butD)||(but==butD_))
	          {
	          temp--;
	          gran_ring_char(&temp,1,12);
	          month__=temp;
	          temp=((temp/10)<<4)+(temp%10);
	          pcf8563_write(0x07,temp);
	          }	
	     speed=1;               
	     }	  
     else if(sub_ind==5)
	     {
	     temp=year__;
	     if((but==butU)||(but==butU_))
	          {
	          temp++;
	          gran_ring_char(&temp,0,99);
	          year__=temp;
	          temp=((temp/10)<<4)+(temp%10);
	          pcf8563_write(0x08,temp);
	          }
          else if((but==butD)||(but==butD_))
	          {
	          temp--;
	          gran_ring_char(&temp,0,99);
	          year__=temp;
	          temp=((temp/10)<<4)+(temp%10);
	          pcf8563_write(0x08,temp);
	          }	
	     speed=1;               
	     }		        
	}  
	   	
else if(ind==iStr)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,1,5);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,1,5);
		}
	else if(but==butD_)
		{
		sub_ind=4;
		}				
     else if(sub_ind==1)
	     {
	     if((but==butR)||(but==butR_))
	     	{
	     	NUMIST++;
	     	gran(&NUMIST,0,12);
	     	lc640_write_int(EE_NUMIST,NUMIST);
	     	}
	     
	     else if((but==butL)||(but==butL_))
	     	{
	     	NUMIST--;
	     	gran(&NUMIST,0,12);
	     	lc640_write_int(EE_NUMIST,NUMIST);
	     	}
          }	
          
     else if(sub_ind==2)
	     {
	     if((but==butR)||(but==butR_))
	     	{
	     	NUMINV++;
	     	gran(&NUMINV,0,4);
	     	lc640_write_int(EE_NUMINV,NUMINV);
	     	}
	     
	     else if((but==butL)||(but==butL_))
	     	{
	     	NUMINV--;
	     	gran(&NUMINV,0,4);
	     	lc640_write_int(EE_NUMINV,NUMINV);
	     	}
          }	          
     else if(sub_ind==3)
	     {
	     if((but==butR)||(but==butR_))
	     	{
	     	NUMDT++;
	     	gran(&NUMDT,0,3);
	     	lc640_write_int(EE_NUMDT,NUMDT);
	     	}
	     
	     else if((but==butL)||(but==butL_))
	     	{
	     	NUMDT--;
	     	gran(&NUMDT,0,3);
	     	lc640_write_int(EE_NUMDT,NUMDT);
	     	}
          }	
     else if(sub_ind==4)
	     {
	     if((but==butR)||(but==butR_))
	     	{
	     	NUMSK++;
	     	gran(&NUMSK,0,4);
	     	lc640_write_int(EE_NUMSK,NUMSK);
	     	}
	     
	     else if((but==butL)||(but==butL_))
	     	{
	     	NUMSK--;
	     	gran(&NUMSK,0,4);
	     	lc640_write_int(EE_NUMSK,NUMSK);
	     	}
          }	                 
    else if(sub_ind==5)
	     {
	     if(but==butE)
	          {
			tree_down(0,0);
	          }
          }	          
	}     

else if (ind==iApv)
	{
     ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,simax);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,simax);
		}
	else if(but==butD_)
		{
		sub_ind=simax;
		}			
	else if(but==butE)
	     {
	     if(sub_ind==simax)
	          {
	          //a=b[--ptr_ind];
	          tree_down(0,0);
	          }
	     else if(sub_ind==0)   
	          {
	          if(APV_ON1==apvON)lc640_write_int(EE_APV_ON1,apvOFF);
	          else lc640_write_int(EE_APV_ON1,apvON);
	          }
          else if((sub_ind==1)&&(APV_ON1==apvON))   
	          {
	          if(APV_ON2==apvON)lc640_write_int(EE_APV_ON2,apvOFF);
	          else lc640_write_int(EE_APV_ON2,apvON);
	          }	 
          }
     
     else if((sub_ind==2)&&(APV_ON2==apvON))   
          {
	     if((but==butR)||(but==butR_))
	          {
	          signed short tempSS;
	          tempSS=APV_ON2_TIME;
	          tempSS++;
	          gran(&tempSS,1,24);
	          lc640_write_int(EE_APV_ON2_TIME,tempSS);
	          }
          else if((but==butL)||(but==butL_))
	          {
	          signed short tempSS;
	          tempSS=APV_ON2_TIME;
	          tempSS--;
	          gran(&tempSS,1,24);
	          lc640_write_int(EE_APV_ON2_TIME,tempSS);
	          }	          
	     speed=1;
	     }	 
  	} 
		     
else if(ind==iK)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+(NUMDT!=0));
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+(NUMDT!=0));
		}
	else if(but==butD_)
		{
		sub_ind=2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+(NUMDT!=0);
		}				
	else if(but==butE)
		{
		if(sub_ind==0)
			{
			tree_up(iK_net,0,0,0);	
			ret(1000);		
			}
		else if((NUMBAT)&&(sub_ind==1))
			{
			tree_up(iK_bat_sel,0,0,0);	
			ret(1000);
			}
		else if((NUMIST)&&(sub_ind==(1+(NUMBAT!=0))))
			{
			tree_up(iK_bps_sel,0,0,0);	
			ret(1000);
			}
		else if((NUMINV)&&(sub_ind==(1+(NUMBAT!=0)+(NUMIST!=0))))
			{
			tree_up(iK_inv_sel,0,0,1);	
			ret(1000);
			}		

		else if((sub_ind==(1+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0))))
			{
			tree_up(iK_load,0,0,0);	
			ret(1000);
			}
          
         	else if((NUMDT)&&(sub_ind==(2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0))))
			{
			tree_up(iK_t_out,0,0,0);	
			ret(1000);			
			}				
          else if(sub_ind==(2+(NUMBAT!=0)+(NUMIST!=0)+(NUMINV!=0)+(NUMDT!=0)))
			{
	          tree_down(0,0);
	          ret(0);
               }	               			
		}			
	}

else if(ind==iK_net)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,1);
		phase=0;
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,1);
		phase=0;
		}
	else if(but==butD_)
		{
		sub_ind=1;
		}				
	else if(sub_ind==0)
		{
		temp_SS=lc640_read_int(EE_KUNET);
		if(but==butR)
			{
			temp_SS++;
			}
		else if(but==butR_)
			{
			//temp_SS=lc640_read_int(EE_KUNET);
			temp_SS+=10;
			//lc640_write_int(EE_KUNET,temp_SS);
			}	
		else if(but==butL)
			{
			//temp_SS=lc640_read_int(EE_KUNET);
			temp_SS--;
			//lc640_write_int(EE_KUNET,temp_SS);
			}
		else if(but==butL_)
			{
			//temp_SS=lc640_read_int(EE_KUNET);
			temp_SS-=10;
			//lc640_write_int(EE_KUNET,temp_SS);
			}				
		speed=1;
		gran(&temp_SS,450,550);
		lc640_write_int(EE_KUNET,temp_SS);
					
		}
	else if(sub_ind==1)
		{
		if(but==butE)
			{
			tree_down(0,1);
			ret(0);
			}
		}			
	}

else if(ind==iK_bat_sel)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,NUMBAT);
		phase=0;
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,NUMBAT);
		phase=0;
		}
	else if(but==butD_)
		{
		sub_ind=1+NUMBAT;
		}	
	else if((but==butE)&&(NUMBAT)&&(BAT_IS_ON[0]==bisON)&&(sub_ind==0))
		{
		tree_up(iK_bat,0,0,0);	
		
		//mess_send(MESS_SRC_CONTROL,0xFFFF,0,10);
     	//mess_send(MESS_BAT_CONTROL,0xFFFF&(~(1<<sub_ind1)),1<<(sub_ind1),10);

		ret(1000);
		}	
	else if((but==butE)&&(NUMBAT)&&(BAT_IS_ON[1]==bisON)&&(sub_ind==((BAT_IS_ON[0]==bisON))))
		{
		tree_up(iK_bat,0,0,1);
		
		//mess_send(MESS_SRC_CONTROL,0xFFFF,0,10);
     	//mess_send(MESS_BAT_CONTROL,0xFFFF&(~(1<<sub_ind1)),1<<(sub_ind1),10);
     		
		ret(1000);
		}	
	else if(sub_ind==(NUMBAT))
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
		}				
	}

else if(ind==iK_bat)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if((sub_ind==1)||(sub_ind==2))
			{
			sub_ind=3;
			//mess_send(MESS_SRC_CONTROL,0,0xFFFF,10);
			}
     		
		else if((sub_ind==4)||(sub_ind==5))sub_ind=6;
		else if((sub_ind==7)||(sub_ind==8))sub_ind=9;
		gran_char(&sub_ind,0,9);
		phase=0;
		}
	else if(but==butU)
		{
		sub_ind--;
		if((sub_ind==1)||(sub_ind==2))
			{
			sub_ind=0;
			//mess_send(MESS_SRC_CONTROL,0xFFFF,0,10);
     		//mess_send(MESS_BAT_CONTROL,0xFFFF&(~(1<<sub_ind1)),1<<(sub_ind1),10);
     		}
		else if((sub_ind==4)||(sub_ind==5))sub_ind=3;
		else if((sub_ind==7)||(sub_ind==8))
			{
			sub_ind=6;
			//mess_send(MESS_SRC_CONTROL,0,0xFFFF,10);
		gran_char(&sub_ind,0,9);
		phase=0;
			}
		}
	else if(but==butD_)
		{
		sub_ind=9;
		}			
	else if(sub_ind==0)
		{
		temp_SS=lc640_read_int(ADR_KUBAT[sub_ind1]);
	     if(but==butR)
	     	{
	     	//temp_SS=lc640_read_int(ADR_KUBAT[sub_ind1]);
		     temp_SS++;
		    // gran(&temp_SS,500,600);
		    // lc640_write_int(ADR_KUBAT[sub_ind1],temp_SS);
	     	}
	     else if(but==butR_)
	     	{
	     	//temp_SS=lc640_read_int(ADR_KUBAT[sub_ind1]);
	     	temp_SS+=2;
	     	//gran(&temp_SS,500,600);
	     	//lc640_write_int(ADR_KUBAT[sub_ind1],temp_SS);
	     	}	
	     else if(but==butL)
	     	{
	     	//temp_SS=lc640_read_int(ADR_KUBAT[sub_ind1]);
	     	temp_SS--;
	     	//gran(&temp_SS,500,600);
	     	//lc640_write_int(ADR_KUBAT[sub_ind1],temp_SS);
	     	}
	     else if(but==butL_)
	     	{
	     	//temp_SS=lc640_read_int(ADR_KUBAT[sub_ind1]);
	     	temp_SS-=2;
	     	//gran(&temp_SS,500,600);
	     	//lc640_write_int(ADR_KUBAT[sub_ind1],temp_SS);
	     	}
	     gran(&temp_SS,500,650);
		lc640_write_int(ADR_KUBAT[sub_ind1],temp_SS);					
		speed=1;			
		}
					
	else if(sub_ind==3)
		{
		if(but==butE)
		     {
		     if(phase==0)
		          {
		          lc640_write_int(ADR_KI0BAT[sub_ind1],ad7705_buff_[sub_ind1]);
		          phase=1;
		          }
		     }	
		else
			{
			temp_SS=lc640_read_int(ADR_KI1BAT[sub_ind1]);
			if(but==butR)temp_SS++;
			else if(but==butR_)temp_SS+=2;
			else if(but==butL)temp_SS--;
			else if(but==butL_)temp_SS-=2;
						
			gran(&temp_SS,600,700);
			lc640_write_int(ADR_KI1BAT[sub_ind1],temp_SS);
			phase=1;
			}
				
		speed=1;			
		}					
	else if(sub_ind==6)
		{
		temp_SS=lc640_read_int(ADR_KTBAT[sub_ind1]);
		if(but==butR)temp_SS++;
		else if(but==butR_)temp_SS+=3;
		else if(but==butL)temp_SS--;
		else if(but==butL_)temp_SS-=3;
		gran(&temp_SS,1900,2100);
		lc640_write_int(ADR_KTBAT[sub_ind1],temp_SS);				
		speed=1;			
		}	
	else if(sub_ind==9)
		{
		if(but==butE)
			{
			//a=b[--ptr_ind];
			//sub_ind++;
			tree_down(0,1);
			ret(0);
			}
		}			
	}		

else if(ind==iK_bps_sel)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,NUMIST);
		phase=0;
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,NUMIST);
		phase=0;
		}
	else if(but==butD_)
		{
		sub_ind=1+NUMIST;
		}	
	else if((but==butE)&&(NUMIST)&&(sub_ind<NUMIST))
		{
		tree_up(iK_bps,0,0,sub_ind);	
		
		//mess_send(MESS_SRC_CONTROL,0xFFFF,0,10);
     	//mess_send(MESS_BAT_CONTROL,0xFFFF&(~(1<<sub_ind1)),1<<(sub_ind1),10);

		ret(1000);
		}	
	else if(sub_ind==(NUMIST))
		{
		if(but==butE)
			{
			tree_down(0,0);
			ret(0);
			}
		}				
	}

else if(ind==iK_bps)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind++;
		if((sub_ind==1)||(sub_ind==2))sub_ind=3;
		else if((sub_ind==4)||(sub_ind==5))sub_ind=6;
		else if((sub_ind==7)||(sub_ind==8))sub_ind=9;
		else if((sub_ind==10)||(sub_ind==11))sub_ind=12;
		else if((sub_ind==13)||(sub_ind==14))sub_ind=15;
		gran_char(&sub_ind,0,15);
		phase=0;
		}
	else if(but==butU)
		{
		sub_ind--;
		if((sub_ind==1)||(sub_ind==2))sub_ind=0;
		else if((sub_ind==4)||(sub_ind==5))sub_ind=3;
		else if((sub_ind==7)||(sub_ind==8))sub_ind=6;
		else if((sub_ind==10)||(sub_ind==11))sub_ind=9;
		else if((sub_ind==13)||(sub_ind==14))sub_ind=12;		
		gran_char(&sub_ind,0,15);
		phase=0;
		}
	else if(but==butD_)
		{
		sub_ind=15;
		}
	else if (sub_ind == 0)
		{
		if(but==butLR) can2_out(sub_ind1,sub_ind1,KLBR,(0*16)+1,(0*16)+1,0,0,0);
	     else if(but==butR) can2_out(sub_ind1,sub_ind1,KLBR,(0*16)+2,(0*16)+2,0,0,0);
		else if(but==butR_)	can2_out(sub_ind1,sub_ind1,KLBR,(0*16)+3,(0*16)+3,0,0,0);
    		else if(but==butL) can2_out(sub_ind1,sub_ind1,KLBR,(0*16)+4,(0*16)+4,0,0,0); 
		else if(but==butL_) can2_out(sub_ind1,sub_ind1,KLBR,(0*16)+5,(0*16)+5,0,0,0);
		speed=1;
		}	
		
	else if (sub_ind == 3)
		{
		if(but==butLR) can2_out(sub_ind1,sub_ind1,KLBR,(1*16)+1,(1*16)+1,0,0,0);
	     else if(but==butR) can2_out(sub_ind1,sub_ind1,KLBR,(1*16)+2,(1*16)+2,0,0,0);
		else if(but==butR_)	can2_out(sub_ind1,sub_ind1,KLBR,(1*16)+3,(1*16)+3,0,0,0);
    		else if(but==butL) can2_out(sub_ind1,sub_ind1,KLBR,(1*16)+4,(1*16)+4,0,0,0); 
		else if(but==butL_) can2_out(sub_ind1,sub_ind1,KLBR,(1*16)+5,(1*16)+5,0,0,0);
		speed=1;
		}		
		
	else if (sub_ind == 6)
		{
		temp_SS=lc640_read_int(EE_U_AVT);
		if(but==butR)temp_SS++;
		else if(but==butR_)temp_SS+=2;
		else if(but==butL)temp_SS--;
		else if(but==butL_)temp_SS-=2;
		else if(but==butE_)can2_out(sub_ind1,sub_ind1,CMND,0xee,0xee,0,0,0);   
						
		gran(&temp_SS,400,800);
		lc640_write_int(EE_U_AVT,temp_SS);
		
		speed=1;
		}	
		
	else if (sub_ind == 9)
		{
		if(but==butE)
			{
			can2_out(sub_ind1,sub_ind1,KLBR,(2*16)+1,(2*16)+1,0,0,0);
			phase=1;
			}
	     else if(but==butR) can2_out(sub_ind1,sub_ind1,KLBR,(2*16)+2,(2*16)+2,0,0,0);
		else if(but==butR_)	can2_out(sub_ind1,sub_ind1,KLBR,(2*16)+3,(2*16)+3,0,0,0);
    		else if(but==butL) can2_out(sub_ind1,sub_ind1,KLBR,(2*16)+4,(2*16)+4,0,0,0); 
		else if(but==butL_) can2_out(sub_ind1,sub_ind1,KLBR,(2*16)+5,(2*16)+5,0,0,0);
		speed=1;
		}		
		
	else if (sub_ind == 12)
		{
		if(but==butR) can2_out(sub_ind1,sub_ind1,KLBR,(3*16)+2,(3*16)+2,0,0,0);
		else if(but==butR_)	can2_out(sub_ind1,sub_ind1,KLBR,(3*16)+3,(3*16)+3,0,0,0);
    		else if(but==butL) can2_out(sub_ind1,sub_ind1,KLBR,(3*16)+4,(3*16)+4,0,0,0); 
		else if(but==butL_) can2_out(sub_ind1,sub_ind1,KLBR,(3*16)+5,(3*16)+5,0,0,0);
		speed=1;
		}								
			
	else if(sub_ind==0)
		{
		if(phase==0)
		     {
		     if(but==butE)
		          {
		          #if(UKU_VERSION==300)
		          if(sub_ind1==0)temp_SS=adc_buff_[3];
		          if(sub_ind1==1)temp_SS=adc_buff_[2];
                    #else
                    if(sub_ind1==0)temp_SS=adc_buff_[2];
		          if(sub_ind1==1)temp_SS=adc_buff_[3];
		          #endif
		          //lc640_write_int(ptr_ki0_src[sub_ind1],temp_SS);
		     	phase=1;
		          }
		     else phase=1;     
		     }
		else if(phase==2)
		     {
		     if(but==butR)
		     	{
		     	//temp_SS=lc640_read_int(ptr_ki_src[sub_ind1]);
		     	temp_SS++;
		     	//lc640_write_int(ptr_ki_src[sub_ind1],temp_SS);
	     		}
	     	else if(but==butR_)
	     		{
	     		//temp_SS=lc640_read_int(ptr_ki_src[sub_ind1]);
	     		temp_SS+=2;
	     		//lc640_write_int(ptr_ki_src[sub_ind1],temp_SS);
	     		}	
	     	else if(but==butL)
	     		{
	     		//temp_SS=lc640_read_int(ptr_ki_src[sub_ind1]);
	     		temp_SS--;
	     		//lc640_write_int(ptr_ki_src[sub_ind1],temp_SS);
	     		}
	     	else if(but==butL_)
	     		{
	     		//temp_SS=lc640_read_int(ptr_ki_src[sub_ind1]);
	     		temp_SS-=2;
	     		//lc640_write_int(ptr_ki_src[sub_ind1],temp_SS);
	     		}				
	     	speed=1;			
	     	}
	     }	
					
	else if(sub_ind==3)
		{
	     if(but==butR)
			{
			//temp_SS=lc640_read_int(ptr_ku_src[sub_ind1]);
			temp_SS++;
			//lc640_write_int(ptr_ku_src[sub_ind1],temp_SS);
			}
		else if(but==butR_)
			{
			//temp_SS=lc640_read_int(ptr_ku_src[sub_ind1]);
			temp_SS+=2;
			//lc640_write_int(ptr_ku_src[sub_ind1],temp_SS);
			}	
		else if(but==butL)
			{
			//temp_SS=lc640_read_int(ptr_ku_src[sub_ind1]);
			temp_SS--;
			//lc640_write_int(ptr_ku_src[sub_ind1],temp_SS);
			}
		else if(but==butL_)
			{
			//temp_SS=lc640_read_int(ptr_ku_src[sub_ind1]);
			temp_SS-=2;
			//lc640_write_int(ptr_ku_src[sub_ind1],temp_SS);
			}				
		speed=1;			
		}					
	else if(sub_ind==6)
		{
		if(but==butR)
			{
			//temp_SS=lc640_read_int(ptr_kt_src[sub_ind1]);
			temp_SS++;
			//lc640_write_int(ptr_kt_src[sub_ind1],temp_SS);
			}
		else if(but==butR_)
			{
			//temp_SS=lc640_read_int(ptr_kt_src[sub_ind1]);
			temp_SS+=3;
			//lc640_write_int(ptr_kt_src[sub_ind1],temp_SS);
			}	
		else if(but==butL)
			{
			//temp_SS=lc640_read_int(ptr_kt_src[sub_ind1]);
			temp_SS--;
			//lc640_write_int(ptr_kt_src[sub_ind1],temp_SS);
			}
		else if(but==butL_)
			{
			//temp_SS=lc640_read_int(ptr_kt_src[sub_ind1]);
			temp_SS-=3;
			//lc640_write_int(ptr_kt_src[sub_ind1],temp_SS);
			}				
		speed=1;			
		}	
	else if(sub_ind==15)
		{
		if(but==butE)
			{
			//a=b[--ptr_ind];
			//sub_ind++;
			tree_down(0,1);
			ret(0);
			}
		}			
	}		

else if(ind==iK_load)
	{
	ret(1000);
	if(but==butD)
		{
		sub_ind=1;
		}
	else if(but==butU)
		{
		sub_ind=0;
		}
	else if(sub_ind==0)
		{
		temp_SS=lc640_read_int(EE_KULOAD);
	     if(but==butR)
	     	{
		     temp_SS++;
	     	}
	     else if(but==butR_)
	     	{
	     	temp_SS+=2;
	     	}	
	     else if(but==butL)
	     	{
	     	temp_SS--;
	     	}
	     else if(but==butL_)
	     	{
	     	temp_SS-=2;
	     	}
	     gran(&temp_SS,500,600);
		lc640_write_int(EE_KULOAD,temp_SS);					
		speed=1;	
					
		}
	else if(sub_ind==1)
		{
		if(but==butE)
			{
			tree_down(0,1);
			ret(0);
			}
		}			
	}
			
else if(ind==iBatLog)
	{
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,6);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,6);
		}
	else if(but==butD_)
		{
		sub_ind=6;
		}				
	else if((but==butL)&&((sub_ind==0)||(sub_ind==3)||(sub_ind==4)))
		{
		tree_down(0,0);
		}		
	else if(sub_ind==0)
	     {
	     if(but==butE)
	          {
	          //b[ptr_ind++]=a;
	          //if(BAT_IS_ON[sub_ind1]==bisON) ind=iPrl_bat_in_out;
	          //else 
	               //{
	               //ind=iPdp1;
	               //ret_ind(iPrl_bat_in_out,0,10);
	               //}
	          tree_up(iPrl_bat_in_out,0,0,sub_ind1);
	          if(BAT_IS_ON[sub_ind1]!=bisON) show_mess("  �������� �������  ",
	          								 "    ��������� ���   ",
	          								 "   ��������������   ",
	          								 "      ������!!!     ",4000);     
	          parol_init();
	          }
	     }
	else if(sub_ind==1)
	     {
	     if(but==butR)BAT_C_NOM[sub_ind1]++;
	     else if(but==butR_)BAT_C_NOM[sub_ind1]+=10;
	     else if(but==butL)BAT_C_NOM[sub_ind1]--;
	     else if(but==butL_)BAT_C_NOM[sub_ind1]-=10;
	     gran(&BAT_C_NOM[sub_ind1],0,200);
	     lc640_write_int(ADR_EE_BAT_C_NOM[sub_ind1],BAT_C_NOM[sub_ind1]);
	     speed=1;
	     }		     
		
	else if(sub_ind==3)
		{
		if(but==butE)
			{ 
               cap=0;
			deep=lc640_read_int(CNT_EVENT_LOG);
			ptr=lc640_read_int(PTR_EVENT_LOG);
			
			if(deep>63)
				{
				deep=0;
			     ptr=0;
			     }
			     			
			for(i=0;i<deep;i++)
				{
				lc640_read_long_ptr(EVENT_LOG+(32*ptr),av_head);
				
				if((av_head[0]=='B')&&(av_head[1]==sub_ind1)&&(av_head[2]=='K')) 	//���� ������ ���������� ������� 'K'(�������� �������)
					{
					cap++;
					content[cap-1]=ptr;
					}
					
				ptr=ptr_carry(ptr,64,-1); 
	
				} 
				
			tree_up(iBatLogKe,0,0,sub_ind1);
			
			av_j_si_max=cap;
			if(av_j_si_max>63)av_j_si_max=0;
			} 
		}




	else if(sub_ind==4)
		{
		if(but==butE)
			{ 
               cap=0;
			deep=lc640_read_int(CNT_EVENT_LOG);
			ptr=lc640_read_int(PTR_EVENT_LOG);
			
			if(deep>63)
				{
				deep=0;
			     ptr=0;
			     }
			     
			for(i=0;i<deep;i++)
				{
				lc640_read_long_ptr(EVENT_LOG+(32*ptr),av_head);
				
				if((av_head[0]=='B')/*&&(av_head[1]==sub_ind1)*/&&(av_head[2]=='Z')) 	//���� ������ ���������� ������� 'z'(������������� �����)
					{
					cap++;
					content[cap-1]=ptr;
					}
					
				ptr=ptr_carry(ptr,64,-1); 
				} 
				
			tree_up(iBatLogVz,0,0,sub_ind1);   
			
			av_j_si_max=cap;
			if(av_j_si_max>63)av_j_si_max=0;
			} 
		else if(but==butR)
			{
			//vz_mem_hndl(0,5);
			//vz_mem_hndl(1,6);	       
			} 
		}

	else if(sub_ind==5)
		{
		if(but==butE)
			{ 
               cap=0;
			deep=lc640_read_int(CNT_EVENT_LOG);
			ptr=lc640_read_int(PTR_EVENT_LOG);

			if(deep>63)
				{
				deep=0;
			     ptr=0;
			     }
			
			//out_usart0 (8,0x11,*((char*)&deep),*(((char*)&deep)+1),*((char*)&ptr),*(((char*)&ptr)+1),cap,content[cap-1],i,0);
			
			for(i=0;i<deep;i++)
				{
				lc640_read_long_ptr(EVENT_LOG+(32*ptr),av_head);
				
				if((av_head[0]=='B')&&(av_head[1]==sub_ind1)&&(av_head[2]=='W')) 	//���� ������ ���������� ������� 'W'(�������)
					{
					cap++;
					content[cap-1]=ptr;
					}
					
		   	/*   	out_usart0 (8,0x22,*((char*)&deep),*(((char*)&deep)+1),*((char*)&ptr),*(((char*)&ptr)+1),cap,content[cap-1],i,0); 
				delay_ms(100);
				PORTC.7=!PORTC.7;
				#asm("wdr"); 	*/
				ptr=ptr_carry(ptr,64,-1); 
				} 
				
			/*ind=iJ_bat_wrk_sel;
			sub_ind=0;*/

			tree_up(iBatLogWrk,0,0,sub_ind1);
			
			av_j_si_max=cap;
			if(av_j_si_max>63)av_j_si_max=0;
			} 
		else if(but==butR)
			{
			//vz_mem_hndl(0,5);
			//vz_mem_hndl(1,6);	       
			} 
		}		
		 	         	
     else if(sub_ind==6)
	     {
	     if(but==butE)
	          {
	          tree_down(0,0);
	          }
	     }		     
		
	} 

else if(ind==iBatLogVz)
	{
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,av_j_si_max);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,av_j_si_max);
		}
	else if(but==butE)
		{
		if(sub_ind==av_j_si_max)
			{
			tree_down(0,0);
			}
		} 
	else if(but==butL)
		{
		tree_down(0,0);
		}		
    //	else if(but==butR) vz_mem_hndl(sub_ind1,_sec);
	
		
	}

else if(ind==iBatLogKe)
	{
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,av_j_si_max);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,av_j_si_max);
		}
	else if(but==butE)
		{
		if(sub_ind==av_j_si_max)
			{
			tree_down(0,0);
			}
		} 
	else if(but==butL)
		{
		tree_down(0,0);
		}		
    //	else if(but==butR) ke_mem_hndl(sub_ind1,_sec);		
	}

else if(ind==iBatLogWrk)
	{
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,av_j_si_max);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,av_j_si_max);
		}
	else if(but==butE)
		{
		if(sub_ind==av_j_si_max)
			{
			tree_down(0,0);
			}
		else if(sub_ind<=av_j_si_max)
			{
			//ind=iWrk;
			//sub_ind2=content[sub_ind];
			index_set=0;
			//sub_ind=0;
			}	
		} 
	else if(but==butL)
		{
		tree_down(0,0);
		}		
	else if(but==butR)
		{
	    //	wrk_mem_hndl(sub_ind1);

		} 
	}

else if(ind==iAv_view)
	{
	if(but==butE)
		{
		avar_ind_stat&=~(1L<<sub_ind);
		if(avar_ind_stat)
			{
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
	 	else 
			{
			tree_down(0,0);
			avar_ind_stat=0;
			}
		}
 	}

else if(ind==iAvz)
	{
	if(AVZ!=AVZ_OFF)
		{
		if(but==butU)
			{
			sub_ind--;
			if(sub_ind==3)sub_ind--;
			}
		else if(but==butD)
			{
			sub_ind++;
			if(sub_ind==3)sub_ind++;
			}
		else if(sub_ind==0)
			{
			if(but==butL)
				{
				if(AVZ==AVZ_1)AVZ=AVZ_OFF;
				else if(AVZ==AVZ_2)AVZ=AVZ_1;
				else if(AVZ==AVZ_3)AVZ=AVZ_2; 
				else if(AVZ==AVZ_6)AVZ=AVZ_3;
				else if(AVZ==AVZ_12)AVZ=AVZ_6;			
				else AVZ=AVZ_12;
				}
			else if((but==butR)||(but==butE))
				{
				if(AVZ==AVZ_1)AVZ=AVZ_2;
				else if(AVZ==AVZ_2)AVZ=AVZ_3;
				else if(AVZ==AVZ_3)AVZ=AVZ_6; 
				else if(AVZ==AVZ_6)AVZ=AVZ_12;
				else if(AVZ==AVZ_12)AVZ=AVZ_OFF;			
				else AVZ=AVZ_1;
				} 
			lc640_write_int(EE_AVZ,AVZ);
			if(AVZ!=AVZ_OFF)avz_next_date_hndl();		
			}      
		else if(sub_ind==1)
			{
			if((but==butR)||(but==butR_))
				{
				speed=1;
				AVZ_TIME++;
				} 
			else if((but==butL)||(but==butL_))
				{
				speed=1;
				AVZ_TIME--;
				}			
			gran(&AVZ_TIME,1,20);
			lc640_write_int(EE_AVZ_TIME,AVZ_TIME);
			}	
		else if(sub_ind==4)
			{
			if((but==butE))
				{
				ind=iSpc;
				sub_ind=1;
				}	
			}        
		gran_char(&sub_ind,0,4);						               
		} 
	else if(AVZ==AVZ_OFF)
		{
		if(but==butU)
			{
			sub_ind--;
			}
		else if(but==butD)
			{
			sub_ind++;
			}
		else if(sub_ind==0)
			{
			if(but==butL)
				{
				if(AVZ==AVZ_1)AVZ=AVZ_OFF;
				else if(AVZ==AVZ_2)AVZ=AVZ_1;
				else if(AVZ==AVZ_3)AVZ=AVZ_2; 
				else if(AVZ==AVZ_6)AVZ=AVZ_3;
				else if(AVZ==AVZ_12)AVZ=AVZ_6;			
				else AVZ=AVZ_12;
				}
			else if((but==butR)||(but==butE))
				{
				if(AVZ==AVZ_1)AVZ=AVZ_2;
				else if(AVZ==AVZ_2)AVZ=AVZ_3;
				else if(AVZ==AVZ_3)AVZ=AVZ_6; 
				else if(AVZ==AVZ_6)AVZ=AVZ_12;
				else if(AVZ==AVZ_12)AVZ=AVZ_OFF;			
				else AVZ=AVZ_1;
				}   
			lc640_write_int(EE_AVZ,AVZ);
			if(AVZ!=AVZ_OFF)avz_next_date_hndl();		
			}      
		else if(sub_ind==1)
			{
			if((but==butR)||(but==butR_))
				{
				speed=1;
				AVZ_TIME++;
				} 
			else if((but==butL)||(but==butL_))
				{
				speed=1;
				AVZ_TIME--;
				}			
			gran(&AVZ_TIME,1,20);
			lc640_write_int(EE_AVZ_TIME,AVZ_TIME);
			}	
		else if(sub_ind==2)
			{
			if((but==butE))
				{
				tree_down(0,0);
				}	
			}        
		gran_char(&sub_ind,0,2);						               
		} 
     }	
	
but_an_end:
n_but=0;
}



//***********************************************
//***********************************************
//***********************************************
//***********************************************
//***********************************************
__irq void timer1_interrupt(void) 
{
T1IR = 0xff; // Clear timer 0 interrupt line.

adc_drv();

if(++t0cnt4>=10)
     {
     t0cnt4=0;
     b1000Hz=1;

	bFF=(char)(GET_REG( IO0PIN, 22, 1));
	if(bFF!=bFF_) hz_out++;
	bFF_=bFF;
	

     }

if(++t0cnt5>=200)
     {
     t0cnt5=0;
     b50Hz=1;
     }
     
if(++t0cnt>=100)
     {
     t0cnt=0;
     b100Hz=1;

	if(beep_cnt)
		{
		beep_cnt--;
		if(beep_cnt==0)SET_REG(IO1CLR,1,27,1);
		} 

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

		flag=1;
	     }         

     if(++t0cnt3>=100)
	     {
	     t0cnt3=0;
	     b1Hz=1;
	    	if(bFL)bFL=0;
	     else 
	     	{
	     	bFL=1;
	     	if(bFL_)bFL_=0;
	     	else bFL_=1;
	     	}

		if(main_1Hz_cnt<10000) main_1Hz_cnt++;

	     }
     }

VICVectAddr = 0;

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


IODIR1 |= (1<<21);                        /* P1.16..23 defined as Outputs */

IODIR0 |= (1<<21);
IOSET0 |= (1<<21);

#ifndef SIMULATOR
lcd_init();
lcd_on();
lcd_clear(); 
#endif

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

ad7705_reset();
delay_ms(20);  

ad7705_write(0x20);
ad7705_write(BIN8(1101)); 
ad7705_write(0x10);
ad7705_write(0x44); 



// CAN interface 1, use IRQVec7, at 125kbit
//can1_init(7,8,CANBitrate125k_60MHz);

// Receive message with ID 102h on CAN 1
//FullCAN_SetFilter(1,0x0e9);

// CAN interface 1, use IRQVec7, at 125kbit
can2_init(7,8,CANBitrate250k_60MHz);

// Receive message with ID 102h on CAN 1
FullCAN_SetFilter(2,0x18e);

memo_read();

uart0_init();
uart1_init();
init_timer1();
adc_init();

pcf8563_read(1);
reload_hndl();
watchdog_init(60000000UL,1000UL);
samokalibr_init();
kb_init();

b1Hz=1;
while (1)  
	{ 
	watchdog_reset();
	if(bRXIN0) 
		{
		bRXIN0=0;
	
		uart_in0();
		} 

	if(bRXIN1) 
		{
		bRXIN1=0;
	
		uart_in1();
		} 

	if(b1000Hz)
		{
		b1000Hz=0;


		}
	
	if(b100Hz)
		{
		b100Hz=0;

		but_drv();
		but_an();
		}
		 
	if(b50Hz)
		{
		b50Hz=0;

		net_drv();
		}

	if(b10Hz)
		{
		char i;

		b10Hz=0;
				
		u_necc_hndl();
		
		for(i=0;i<NUMIST;i++)bps_drv(i);
		bps_hndl();
		
		if(BAT_IS_ON[0])bat_drv(0);
		if(BAT_IS_ON[1])bat_drv(1);
		bat_hndl();
				
		unet_drv();

		
		
		ind_hndl(); 
		#ifndef SIMULATOR
		bitmap_hndl();
		lcd_out(lcd_bitmap);
		#endif
		ad7705_drv();
		//ad7705_write(0x20);
		ret_hndl();
		mess_hndl();
		cntrl_hndl();
		ret_hndl();
		}

	if(b5Hz)
		{
		b5Hz=0;

		memo_read();
		matemat();
		rele_hndl();
		avar_hndl();
  		}

	if(b2Hz)
		{
		b2Hz=0;

		pcf8563_read(1);
  		}

	if(b1Hz)
		{
		b1Hz=0;
		
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

		plazma_plazma_plazma++;
		}
	}
}
