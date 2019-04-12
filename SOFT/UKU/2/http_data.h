//Телеметрия сети
extern char http_power_num_of_phases;
extern short http_power_voltage_of_phase[3];
extern short http_power_frequncy;
extern char http_power_status;

//-----------------------------------------------
void http_data(void);
//-----------------------------------------------
short http_get_log_deep(void);
//-----------------------------------------------
char* http_get_log_rec(char num);