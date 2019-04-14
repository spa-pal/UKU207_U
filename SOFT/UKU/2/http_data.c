#include "http_data.h"
#include "control.h"
#include "eeprom_map.h"
#include "25lc640.h"
#include "common_func.h"
#include "main.h"

//Телеметрия сети
char http_power_num_of_phases;
short http_power_voltage_of_phase[3];
short http_power_frequncy;
char http_power_status;
char http_output_buff[70];
const char hex_alfa[16]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
char log_item_cnt=0;

//-----------------------------------------------
void http_data(void)
{
http_power_num_of_phases=1;
http_power_voltage_of_phase[0]=220+spirit_wrk_cnt;
http_power_voltage_of_phase[1]=225+spirit_wrk_cnt;
http_power_voltage_of_phase[2]=230+spirit_wrk_cnt;
http_power_frequncy = 500+spirit_wrk_cnt;
http_power_status=spirit_wrk_cnt/10;
};

//-----------------------------------------------
short http_get_log_deep(void)
{
return lc640_read_int(CNT_EVENT_LOG);
};

//-----------------------------------------------
char* http_get_log_rec(char num)
{
char i;
unsigned int tempii;
char buff[40];

for (i=0;i<40;i++) buff[i]=0;

tempii=lc640_read_int(PTR_EVENT_LOG);
tempii=ptr_carry(tempii,64,-1*((signed)num));
tempii*=32;
tempii+=EVENT_LOG;

lc640_read_long_ptr(tempii,buff);
lc640_read_long_ptr(tempii+4,buff+4);
lc640_read_long_ptr(tempii+8,buff+8);
lc640_read_long_ptr(tempii+12,buff+12);
lc640_read_long_ptr(tempii+16,buff+16);
lc640_read_long_ptr(tempii+20,buff+20);
lc640_read_long_ptr(tempii+24,buff+24);
lc640_read_long_ptr(tempii+28,buff+28);

for (i=0;i<32;i++)
	{
	http_output_buff[i*2]=hex_alfa[buff[i]/16];
	http_output_buff[(i*2)+1]=hex_alfa[buff[i]%16];
	}
http_output_buff[64]=0;

return http_output_buff;
}

//-----------------------------------------------
char* http_tm_dt_output(char numOfDt)
{
char buffer[100];

sprintf(buffer,"%d %d", t_ext[numOfDt], ND_EXT[numOfDt]);

return buffer;
}

//-----------------------------------------------
char* http_tm_sk_output(char numOfSk)
{
char buffer[100];
char temp1=0;
char temp2=0;

if(sk_stat[numOfSk]==ssON)temp1=1;
if(sk_av_stat[numOfSk]==sasON)temp2=1;

sprintf(buffer,"%d %d", temp1, temp2);

return buffer;
}




/*

 char iii;
char dt[4],dt_[4],dt__[4];
    
		


     
lc640_read_long_ptr(tempii,dt);
lc640_read_long_ptr(tempii+8,dt_);
lc640_read_long_ptr(tempii+12,dt__);
*/

void demo_avar_vrite(void)
{

}

