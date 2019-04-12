#include "http_data.h"
#include "control.h"

//Телеметрия сети
char http_power_num_of_phases;
short http_power_voltage_of_phase[3];
short http_power_frequncy;
char http_power_status;

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

