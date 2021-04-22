#include "main.h"
#include "sgep.h"
#include "eeprom_map.h"
#include "25lc640.h"

//-----------------------------------------------	 
void def_sgep_ips_set(short num_of_cell)
{
lc640_write_int(EE_SNTP_ENABLE,3);
lc640_write_int(EE_SNTP_GMT,9);

lc640_write_int(EE_NUMBAT,1);
lc640_write_int(EE_NUMIST,2);
lc640_write_int(EE_NUMDT,1);
lc640_write_int(EE_NUMMAKB,0);
lc640_write_int(EE_NUMSK,0);
lc640_write_int(EE_NUM_RKI,0);
lc640_write_int(EE_NUM_NET_IN,0);
lc640_write_int(EE_NUMBDR,0);

lc640_write_int(EE_BAT_C_POINT_NUM_ELEM,num_of_cell*6);
lc640_write_int(EE_BAT_C_POINT_20,1500);
lc640_write_int(EE_BAT_C_POINT_10,1500);
lc640_write_int(EE_BAT_C_POINT_5,1480);
lc640_write_int(EE_BAT_C_POINT_3,1410);
lc640_write_int(EE_BAT_C_POINT_1,1140);
lc640_write_int(EE_BAT_C_POINT_1_2,945);
lc640_write_int(EE_BAT_C_POINT_1_6,550);

lc640_write_int(EE_BAT_U_END_20,108*num_of_cell);
lc640_write_int(EE_BAT_U_END_10,108*num_of_cell);
lc640_write_int(EE_BAT_U_END_5,105*num_of_cell);
lc640_write_int(EE_BAT_U_END_3,102*num_of_cell);
lc640_write_int(EE_BAT_U_END_1,99*num_of_cell);
lc640_write_int(EE_BAT_U_END_1_2,96*num_of_cell);
lc640_write_int(EE_BAT_U_END_1_6,96*num_of_cell);

if(num_of_cell==17)
	{
	lc640_write_int(EE_UB0,2363);
	lc640_write_int(EE_UB20,2295);
	lc640_write_int(EE_USIGN,178);
	}
else if(num_of_cell==18)
	{
	lc640_write_int(EE_UB0,2502);
	lc640_write_int(EE_UB20,2430);
	lc640_write_int(EE_USIGN,189);
	}
else if(num_of_cell==20)
	{
	lc640_write_int(EE_UB0,2780);
	lc640_write_int(EE_UB20,2700);
	lc640_write_int(EE_USIGN,210);
	}
lc640_write_int(EE_IKB,20);
lc640_write_int(EE_IZMAX,150); 
lc640_write_int(EE_TBATMAX,50); 
lc640_write_int(EE_TBATSIGN,40);
lc640_write_int(EE_TBAT,60);
 
if(num_of_cell==17)
	{
	lc640_write_int(EE_UVENTOFF,250);
	lc640_write_int(EE_UVZ,2448);
	}
else if(num_of_cell==18)
	{
	lc640_write_int(EE_UVENTOFF,250);
	lc640_write_int(EE_UVZ,2560);
	}
else if(num_of_cell==20)
	{
	lc640_write_int(EE_UVENTOFF,278);
	lc640_write_int(EE_UVZ,2880);
	}
lc640_write_int(EE_IMAX_VZ,150);
lc640_write_int(EE_VZ_HR,48);

if(num_of_cell==17)
	{
	lc640_write_int(EE_SPEED_CHRG_CURR,250);
	lc640_write_int(EE_SPEED_CHRG_VOLT,2295);

	}
else if(num_of_cell==18)
	{
	lc640_write_int(EE_SPEED_CHRG_CURR,250);
	lc640_write_int(EE_SPEED_CHRG_VOLT,2430);

	}
else if(num_of_cell==20)
	{
	lc640_write_int(EE_SPEED_CHRG_CURR,210);
	lc640_write_int(EE_SPEED_CHRG_VOLT,2700);
	}
lc640_write_int(EE_SPEED_CHRG_TIME,5);
lc640_write_int(EE_SPEED_CHRG_D_U,10);	
lc640_write_int(EE_SPEED_CHRG_BLOCK_SRC,0);
lc640_write_int(EE_SPEED_CHRG_AVT_EN,1);
lc640_write_int(EE_BAT_K_OLD,100);
lc640_write_int(EE_ZV_ON,0);
lc640_write_int(EE_AV_OFF_AVT,1);
lc640_write_int(EE_APV_ON1,apvON);
lc640_write_int(EE_APV_ON2,apvON);
lc640_write_int(EE_APV_ON2_TIME,1);
lc640_write_int(EE_PAR,1);

if(num_of_cell==17)
	{
	lc640_write_int(EE_UMAX,2600);
	lc640_write_int(EE_DU,1200);

	}
else if(num_of_cell==18)
	{
	lc640_write_int(EE_UMAX,2700);
	lc640_write_int(EE_DU,1330);

	}
else if(num_of_cell==20)
	{
	lc640_write_int(EE_UMAX,3000);
	lc640_write_int(EE_DU,1580);
	}
lc640_write_int(EE_UMN,187);
lc640_write_int(EE_IMAX,80);
lc640_write_int(EE_IMIN,50);
lc640_write_int(EE_TZAS,3);
lc640_write_int(EE_TMAX,80);
lc640_write_int(EE_TSIGN,70);

if(num_of_cell==17)
	{
	lc640_write_int(EE_U_OUT_KONTR_MAX,2600);
	lc640_write_int(EE_U_OUT_KONTR_MIN,1750);
	lc640_write_int(EE_U_OUT_KONTR_DELAY,100);
	lc640_write_int(EE_AUSW_MAIN,22011);
	}
else if(num_of_cell==18)
	{
	lc640_write_int(EE_U_OUT_KONTR_MAX,2600);
	lc640_write_int(EE_U_OUT_KONTR_MIN,1870);
	lc640_write_int(EE_U_OUT_KONTR_DELAY,100);
	lc640_write_int(EE_AUSW_MAIN,22011);
	}
else if(num_of_cell==20)
	{
	lc640_write_int(EE_U_OUT_KONTR_MAX,2900);
	lc640_write_int(EE_U_OUT_KONTR_MIN,1870);
	lc640_write_int(EE_U_OUT_KONTR_DELAY,100);
	lc640_write_int(EE_AUSW_MAIN,22011);
	}
lc640_write_int(EE_TERMOKOMP,1);
lc640_write_int(EE_FORVARDBPSCHHOUR,24);
lc640_write_int(EE_DOP_RELE_FUNC,1);
lc640_write_int(EE_IPS_BLOCK_SRC,0);
lc640_write_int(EE_TVENTMAX,6000);
lc640_write_int(EE_ICA_EN,0);
lc640_write_int(EE_PWM_START,50);
lc640_write_int(EE_KB_ALGORITM,3);
lc640_write_int(EE_REG_SPEED,1);
}


