//Телеметрия сети
extern char http_power_num_of_phases;
extern short http_power_voltage_of_phase[3];
extern short http_power_frequncy;
extern char http_power_status;
extern char http_output_buff[70];
extern char log_item_cnt;

//-----------------------------------------------
void http_data(void);
//-----------------------------------------------
short http_get_log_deep(void);
//-----------------------------------------------
char* http_get_log_rec(char num);
//-----------------------------------------------
char* http_tm_dt_output(char numOfDt);
//-----------------------------------------------
char* http_tm_sk_output(char numOfSk);

