#line 1 "modbus.c"

#line 1 "snmp_data_file.h"
extern char snmp_community[10];
extern char snmp_web_passw[4];


extern unsigned int snmp_device_code;
extern signed 	   snmp_sernum;
extern signed short snmp_sernum_lsb;
extern signed short snmp_sernum_msb;
extern char 	   snmp_location[100];
extern signed short snmp_numofbat;
extern signed short snmp_numofbps;
extern signed short snmp_numofinv;
extern signed short snmp_numofavt;
extern signed short snmp_numofdt;
extern signed short snmp_numofsk;
extern signed short snmp_numofevents;


extern signed short snmp_mains_power_voltage;
extern signed short snmp_mains_power_frequency;
extern signed short snmp_mains_power_status;
extern signed short snmp_mains_power_alarm;
extern signed short snmp_mains_power_voltage_phaseA;
extern signed short snmp_mains_power_voltage_phaseB;
extern signed short snmp_mains_power_voltage_phaseC;


extern signed short snmp_load_voltage;
extern signed short snmp_load_current;


extern signed short snmp_bps_number[8];
extern signed short snmp_bps_voltage[8];
extern signed short snmp_bps_current[8];
extern signed short snmp_bps_temperature[8];
extern signed short snmp_bps_stat[8];


extern signed short snmp_inv_number[3];
extern signed short snmp_inv_voltage[3];
extern signed short snmp_inv_current[3];
extern signed short snmp_inv_temperature[3];
extern signed short snmp_inv_stat[3];


extern signed short snmp_bat_number[2];
extern signed short snmp_bat_voltage[2];
extern signed short snmp_bat_part_voltage[2];
extern signed short snmp_bat_current[2];
extern signed short snmp_bat_temperature[2];
extern signed short snmp_bat_capacity[2];
extern signed short snmp_bat_charge[2];
extern signed short snmp_bat_status[2];
extern signed short snmp_bat_rem_time[2];
extern signed short snmp_bat_flag[2];
extern signed short snmp_bat_flag_puts[2];



extern signed short snmp_makb_number[4];
extern signed short snmp_makb_connect_status[4];
extern signed short snmp_makb_voltage0[4];
extern signed short snmp_makb_voltage1[4];
extern signed short snmp_makb_voltage2[4];
extern signed short snmp_makb_voltage3[4];
extern signed short snmp_makb_voltage4[4];
extern signed short snmp_makb_temper0[4];
extern signed short snmp_makb_temper1[4];
extern signed short snmp_makb_temper2[4];
extern signed short snmp_makb_temper3[4];
extern signed short snmp_makb_temper4[4];
extern signed short snmp_makb_temper0_stat[4];
extern signed short snmp_makb_temper1_stat[4];
extern signed short snmp_makb_temper2_stat[4];
extern signed short snmp_makb_temper3_stat[4];
extern signed short snmp_makb_temper4_stat[4];
extern signed short snmp_bat_voltage[2];
extern signed short snmp_bat_current[2];
extern signed short snmp_bat_temperature[2];
extern signed short snmp_bat_capacity[2];
extern signed short snmp_bat_charge[2];
extern signed short snmp_bat_status[2]; 



extern signed short snmp_spc_stat;
extern char snmp_spc_trap_message[100];
extern signed short snmp_spc_trap_value_0,snmp_spc_trap_value_1,snmp_spc_trap_value_2;


extern signed short snmp_energy_vvod_phase_a;
extern signed short snmp_energy_vvod_phase_b;
extern signed short snmp_energy_vvod_phase_c;
extern signed short snmp_energy_pes_phase_a;
extern signed short snmp_energy_pes_phase_b;
extern signed short snmp_energy_pes_phase_c;
extern signed short snmp_energy_input_voltage;


extern signed long snmp_energy_total_energy;
extern signed short snmp_energy_current_energy;


extern signed char snmp_sk_number[4];
extern signed char snmp_sk_aktiv[4];
extern signed char snmp_sk_alarm_aktiv[4];
extern signed char snmp_sk_alarm[4];
extern char snmp_sk_name[4][20];


extern signed char snmp_dt_number[3];
extern signed short snmp_dt_temper[3];
extern signed char snmp_dt_error[3];


extern signed char snmp_avt_number[12];
extern signed char snmp_avt_stat[12];


extern signed short snmp_command;
extern signed short snmp_command_parametr;


extern char snmp_log[64][128];


extern signed short snmp_main_bps;
extern signed short snmp_zv_en;
extern signed short snmp_alarm_auto_disable;
extern signed short snmp_bat_test_time;
extern signed short snmp_u_max;
extern signed short snmp_u_min;
extern signed short snmp_u_0_grad;
extern signed short snmp_u_20_grad;
extern signed short snmp_u_sign;
extern signed short snmp_u_min_power;
extern signed short snmp_u_max_power;	 
extern signed short snmp_u_withouth_bat;
extern signed short snmp_control_current;
extern signed short snmp_max_charge_current;
extern signed short snmp_max_current;
extern signed short snmp_min_current;
extern signed short snmp_uvz;
extern signed short snmp_max_current_koef;
extern signed short snmp_max_current_koef;
extern signed short snmp_up_charge_koef;
extern signed short snmp_powerup_psu_timeout;
extern signed short snmp_max_temperature;
extern signed short snmp_tsign_bat; 
extern signed short snmp_tmax_bat;
extern signed short snmp_tsign_bps;
extern signed short snmp_tmax_bps;
extern signed short snmp_bat_part_alarm;
extern signed short snmp_power_cnt_adress;


extern signed short snmp_klimat_box_temper;
extern signed short snmp_klimat_settings_box_alarm;
extern signed short snmp_klimat_settings_vent_on;
extern signed short snmp_klimat_settings_vent_off;
extern signed short snmp_klimat_settings_warm_on;
extern signed short snmp_klimat_settings_warm_off;
extern signed short snmp_klimat_settings_load_on;
extern signed short snmp_klimat_settings_load_off;
extern signed short snmp_klimat_settings_batt_on;
extern signed short snmp_klimat_settings_batt_off;


extern signed short snmp_dt_ext;
extern signed short snmp_dt_msan;
extern signed short snmp_dt_epu;


extern short snmp_lakb_number[7];				
extern short snmp_lakb_voltage[7];				
extern short snmp_lakb_max_cell_voltage[7];		
extern short snmp_lakb_min_cell_voltage[7];		
extern short snmp_lakb_max_cell_temperature[7];	
extern short snmp_lakb_min_cell_temperature[7];	
extern short snmp_lakb_ch_curr[7];				
extern short snmp_lakb_dsch_curr[7];			
extern short snmp_lakb_rat_cap[7];				
extern short snmp_lakb_soh[7];				
extern short snmp_lakb_soc[7];				
extern short snmp_lakb_cclv[7];  				
extern short snmp_lakb_rbt[7];				
extern short snmp_lakb_flags1[7];				
extern short snmp_lakb_flags2[7];				
extern char snmp_lakb_damp1[3][150];				
extern char snmp_lakb_damp2[100];				
extern signed char	snmp_lakb_cell_temperature_1[3];		
extern signed char	snmp_lakb_cell_temperature_2[3];		
extern signed char	snmp_lakb_cell_temperature_3[3];		
extern signed char	snmp_lakb_cell_temperature_4[3];		
extern signed char	snmp_lakb_cell_temperature_ambient[3];	
extern signed char	snmp_lakb_cell_temperature_power[3];	


extern signed char	snmp_warm_sign;				
extern signed char	snmp_cool_sign;				
extern signed char	snmp_warm_on_temper;		
extern signed char	snmp_warm_off_temper;		
extern signed char	snmp_warm_q;				
extern signed char	snmp_cool_100_temper;		
extern signed char	snmp_cool_80_temper;		
extern signed char	snmp_cool_60_temper;		
extern signed char	snmp_cool_40_temper;		
extern signed char	snmp_cool_20_temper;		
extern signed char	snmp_cool_100_dtemper;		
extern signed char	snmp_cool_80_dtemper;		
extern signed char	snmp_cool_60_dtemper;		
extern signed char	snmp_cool_40_dtemper;		
extern signed char	snmp_cool_20_dtemper;		
extern signed char 	snmp_warm_stat;				

extern unsigned char enmv_on[8]; 
extern unsigned char snmp_enmv_number[64];  
extern unsigned char snmp_enmv_data[64][8]; 
extern unsigned char enmv_data_pred[8][8], enmv_data[8][8]; 
 

void snmp_data (void);
void snmp_sernum_write (int mode); 
void snmp_location_write (int mode);
void snmp_command_execute (int mode);
void event2snmp(char num);
void snmp_main_bps_write (int mode);
void snmp_zv_on_write (int mode);
void snmp_alarm_auto_disable_write (int mode);
void snmp_bat_test_time_write (int mode);
void snmp_u_max_write (int mode);
void snmp_u_min_write (int mode);
void snmp_u_ips_set_write (int mode);
void snmp_u_0_grad_write (int mode);
void snmp_u_20_grad_write (int mode);
void snmp_u_sign_write (int mode);
void snmp_u_min_power_write (int mode);
void snmp_u_withouth_bat_write (int mode);
void snmp_u_max_power_write (int mode);	 
void snmp_control_current_write (int mode);
void snmp_max_charge_current_write (int mode);
void snmp_max_current_write (int mode);
void snmp_min_current_write (int mode);
void snmp_up_charge_koef_write (int mode);
void snmp_powerup_psu_timeout_write (int mode);
void snmp_max_temperature_write (int mode);
void event2snmp(char num);
void snmp_trap_send(char* str, signed short in0, signed short in1, signed short in2);
void snmp_alarm_aktiv_write1(int mode);
void snmp_alarm_aktiv_write2(int mode);
void snmp_alarm_aktiv_write3(int mode);
void snmp_alarm_aktiv_write4(int mode);
void snmp_klimat_settings_box_alarm_write(int mode);
void snmp_klimat_settings_vent_on_write(int mode);
void snmp_klimat_settings_vent_off_write(int mode);
void snmp_klimat_settings_warm_on_write(int mode);
void snmp_klimat_settings_warm_off_write(int mode);
void snmp_klimat_settings_load_on_write(int mode);
void snmp_klimat_settings_load_off_write(int mode);
void snmp_klimat_settings_batt_on_write(int mode);
void snmp_klimat_settings_batt_off_write(int mode);
void snmp_tsign_bat_write(int mode);
void snmp_tmax_bat_write(int mode);
void snmp_tsign_bps_write(int mode);
void snmp_tmax_bps_write(int mode);
void snmp_bat_part_alarm_write(int mode);
void snmp_power_cnt_adress_write(int mode);

void snmp_warm_sign_write(int mode);
void snmp_cool_sign_write(int mode);
void snmp_warm_on_temper_write(int mode);
void snmp_warm_off_temper_write(int mode);
void snmp_warm_q_write(int mode);
void snmp_cool_100_temper_write(int mode);
void snmp_cool_80_temper_write(int mode);
void snmp_cool_60_temper_write(int mode);
void snmp_cool_40_temper_write(int mode);
void snmp_cool_20_temper_write(int mode);
void snmp_cool_100_dtemper_write(int mode);
void snmp_cool_80_dtemper_write(int mode);
void snmp_cool_60_dtemper_write(int mode);
void snmp_cool_40_dtemper_write(int mode);
void snmp_cool_20_dtemper_write(int mode);
void snmp_u_out_kontr_max_write (int mode);
void snmp_u_out_kontr_min_write (int mode);
void snmp_u_out_kontr_delay_write (int mode);
void snmp_uvz_write (int mode);
void snmp_imax_vz_write (int mode);
void snmp_vz_hr_write (int mode);
void snmp_vz_ch_vent_block_write (int mode);
void snmp_spz_i_max_write (int mode);
void snmp_spz_u_write (int mode);
void snmp_spz_time_write (int mode);
void snmp_spz_avt_en_write (int mode);
void snmp_spz_delta_write (int mode);
void snmp_spz_block_en_src_write (int mode);
void snmp_spz_block_log_write (int mode);
void snmp_spz_vent_block_write (int mode);

void snmp_LVBD_Uload_rele_en (int mode);
void snmp_LVBD_Uakb_rele_en (int mode);
void snmp_LVBD_porog_U1 (int mode);
void snmp_LVBD_porog_U2 (int mode);
void snmp_LVBD_num_meas (int mode);





 
#line 3 "modbus.c"
#line 1 "C:\\Keil\\ARM\\INC\\NXP\\LPC17xx\\lpc17xx.h"




















 









 

typedef enum IRQn
{
 
  NonMaskableInt_IRQn           = -14,       
  MemoryManagement_IRQn         = -12,       
  BusFault_IRQn                 = -11,       
  UsageFault_IRQn               = -10,       
  SVCall_IRQn                   = -5,        
  DebugMonitor_IRQn             = -4,        
  PendSV_IRQn                   = -2,        
  SysTick_IRQn                  = -1,        

 
  WDT_IRQn                      = 0,         
  TIMER0_IRQn                   = 1,         
  TIMER1_IRQn                   = 2,         
  TIMER2_IRQn                   = 3,         
  TIMER3_IRQn                   = 4,         
  UART0_IRQn                    = 5,         
  UART1_IRQn                    = 6,         
  UART2_IRQn                    = 7,         
  UART3_IRQn                    = 8,         
  PWM1_IRQn                     = 9,         
  I2C0_IRQn                     = 10,        
  I2C1_IRQn                     = 11,        
  I2C2_IRQn                     = 12,        
  SPI_IRQn                      = 13,        
  SSP0_IRQn                     = 14,        
  SSP1_IRQn                     = 15,        
  PLL0_IRQn                     = 16,        
  RTC_IRQn                      = 17,        
  EINT0_IRQn                    = 18,        
  EINT1_IRQn                    = 19,        
  EINT2_IRQn                    = 20,        
  EINT3_IRQn                    = 21,        
  ADC_IRQn                      = 22,        
  BOD_IRQn                      = 23,        
  USB_IRQn                      = 24,        
  CAN_IRQn                      = 25,        
  DMA_IRQn                      = 26,        
  I2S_IRQn                      = 27,        
  ENET_IRQn                     = 28,        
  RIT_IRQn                      = 29,        
  MCPWM_IRQn                    = 30,        
  QEI_IRQn                      = 31,        
  PLL1_IRQn                     = 32,        
  USBActivity_IRQn              = 33,        
  CANActivity_IRQn              = 34,        
} IRQn_Type;






 

 





#line 1 "C:\\Keil\\ARM\\CMSIS\\Include\\core_cm3.h"
 




















 























  







 




 






 

 











#line 93 "C:\\Keil\\ARM\\CMSIS\\Include\\core_cm3.h"

#line 1 "C:\\Keil\\ARM\\RV31\\INC\\stdint.h"
 
 





 









#line 25 "C:\\Keil\\ARM\\RV31\\INC\\stdint.h"







 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     
typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;

     
typedef   signed       __int64 intmax_t;
typedef unsigned       __int64 uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     


     


     


     

     


     


     


     

     



     



     


     
    
 



#line 196 "C:\\Keil\\ARM\\RV31\\INC\\stdint.h"

     







     










     











#line 260 "C:\\Keil\\ARM\\RV31\\INC\\stdint.h"



 


#line 95 "C:\\Keil\\ARM\\CMSIS\\Include\\core_cm3.h"
#line 1 "C:\\Keil\\ARM\\CMSIS\\Include\\core_cmInstr.h"
 




















 





 



 


 




 







 







 






 








 







 







 









 









 



static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}









 



static __inline __asm int32_t __REVSH(int32_t value)
{
  revsh r0, r0
  bx lr
}











 









 









 









 











 











 











 







 














 










 









 






#line 772 "C:\\Keil\\ARM\\CMSIS\\Include\\core_cmInstr.h"

   

#line 96 "C:\\Keil\\ARM\\CMSIS\\Include\\core_cm3.h"
#line 1 "C:\\Keil\\ARM\\CMSIS\\Include\\core_cmFunc.h"
 




















 




 



 


 

 
 






 



static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}








 



static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}








 



static __inline uint32_t __get_IPSR(void)
{
  register uint32_t __regIPSR          __asm("ipsr");
  return(__regIPSR);
}








 



static __inline uint32_t __get_APSR(void)
{
  register uint32_t __regAPSR          __asm("apsr");
  return(__regAPSR);
}








 



static __inline uint32_t __get_xPSR(void)
{
  register uint32_t __regXPSR          __asm("xpsr");
  return(__regXPSR);
}








 



static __inline uint32_t __get_PSP(void)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  return(__regProcessStackPointer);
}








 



static __inline void __set_PSP(uint32_t topOfProcStack)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  __regProcessStackPointer = topOfProcStack;
}








 



static __inline uint32_t __get_MSP(void)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  return(__regMainStackPointer);
}








 



static __inline void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
}








 



static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}








 



static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}

 







 







 








 



static __inline uint32_t  __get_BASEPRI(void)
{
  register uint32_t __regBasePri         __asm("basepri");
  return(__regBasePri);
}








 



static __inline void __set_BASEPRI(uint32_t basePri)
{
  register uint32_t __regBasePri         __asm("basepri");
  __regBasePri = (basePri & 0xff);
}

 






 



static __inline uint32_t __get_FAULTMASK(void)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  return(__regFaultMask);
}








 



static __inline void __set_FAULTMASK(uint32_t faultMask)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  __regFaultMask = (faultMask & 1);
}





#line 348 "C:\\Keil\\ARM\\CMSIS\\Include\\core_cmFunc.h"


#line 840 "C:\\Keil\\ARM\\CMSIS\\Include\\core_cmFunc.h"

 


#line 97 "C:\\Keil\\ARM\\CMSIS\\Include\\core_cm3.h"









 
#line 114 "C:\\Keil\\ARM\\CMSIS\\Include\\core_cm3.h"

 





 








 





 


 
typedef union
{
  struct
  {

    uint32_t _reserved0:27;               





    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                              
} APSR_Type;



 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:23;               
  } b;                                    
  uint32_t w;                             
} IPSR_Type;



 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       

    uint32_t _reserved0:15;               





    uint32_t T:1;                         
    uint32_t IT:2;                        
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} xPSR_Type;



 
typedef union
{
  struct
  {
    uint32_t nPRIV:1;                     
    uint32_t SPSEL:1;                     
    uint32_t FPCA:1;                      
    uint32_t _reserved0:29;               
  } b;                                    
  uint32_t w;                             
} CONTROL_Type;

 






 


 
typedef struct
{
  volatile uint32_t ISER[8];                  
       uint32_t RESERVED0[24];                                   
  volatile uint32_t ICER[8];                  
       uint32_t RSERVED1[24];                                    
  volatile uint32_t ISPR[8];                  
       uint32_t RESERVED2[24];                                   
  volatile uint32_t ICPR[8];                  
       uint32_t RESERVED3[24];                                   
  volatile uint32_t IABR[8];                  
       uint32_t RESERVED4[56];                                   
  volatile uint8_t  IP[240];                  
       uint32_t RESERVED5[644];                                  
  volatile  uint32_t STIR;                     
}  NVIC_Type;                                               

 






 


 
typedef struct
{
  volatile const  uint32_t CPUID;                    
  volatile uint32_t ICSR;                     
  volatile uint32_t VTOR;                     
  volatile uint32_t AIRCR;                    
  volatile uint32_t SCR;                      
  volatile uint32_t CCR;                      
  volatile uint8_t  SHP[12];                  
  volatile uint32_t SHCSR;                    
  volatile uint32_t CFSR;                     
  volatile uint32_t HFSR;                     
  volatile uint32_t DFSR;                     
  volatile uint32_t MMFAR;                    
  volatile uint32_t BFAR;                     
  volatile uint32_t AFSR;                     
  volatile const  uint32_t PFR[2];                   
  volatile const  uint32_t DFR;                      
  volatile const  uint32_t ADR;                      
  volatile const  uint32_t MMFR[4];                  
  volatile const  uint32_t ISAR[5];                  
} SCB_Type;                                                

 












 






























 






 





















 









 


















 
































                                     









 









 









 















 






 


 
typedef struct
{
  volatile uint32_t CTRL;                     
  volatile uint32_t LOAD;                     
  volatile uint32_t VAL;                      
  volatile const  uint32_t CALIB;                    
} SysTick_Type;

 












 



 



 









 






 


 
typedef struct
{
  volatile  union  
  {
    volatile  uint8_t    u8;                   
    volatile  uint16_t   u16;                  
    volatile  uint32_t   u32;                  
  }  PORT [32];                           
       uint32_t RESERVED0[864];                                 
  volatile uint32_t TER;                      
       uint32_t RESERVED1[15];                                  
  volatile uint32_t TPR;                      
       uint32_t RESERVED2[15];                                  
  volatile uint32_t TCR;                      
       uint32_t RESERVED3[29];                                  
  volatile uint32_t IWR;                      
  volatile uint32_t IRR;                      
  volatile uint32_t IMCR;                     
       uint32_t RESERVED4[43];                                  
  volatile uint32_t LAR;                      
  volatile uint32_t LSR;                      
       uint32_t RESERVED5[6];                                   
  volatile const  uint32_t PID4;                     
  volatile const  uint32_t PID5;                     
  volatile const  uint32_t PID6;                     
  volatile const  uint32_t PID7;                     
  volatile const  uint32_t PID0;                     
  volatile const  uint32_t PID1;                     
  volatile const  uint32_t PID2;                     
  volatile const  uint32_t PID3;                     
  volatile const  uint32_t CID0;                     
  volatile const  uint32_t CID1;                     
  volatile const  uint32_t CID2;                     
  volatile const  uint32_t CID3;                     
} ITM_Type;                                                

 



 
























 



 



 



 









   






 


 
typedef struct
{
       uint32_t RESERVED0;
  volatile const  uint32_t ICTR;                     



       uint32_t RESERVED1;

} InterruptType_Type;

 



 









   







 


 
typedef struct
{
  volatile const  uint32_t TYPE;                     
  volatile uint32_t CTRL;                     
  volatile uint32_t RNR;                      
  volatile uint32_t RBAR;                     
  volatile uint32_t RASR;                     
  volatile uint32_t RBAR_A1;                  
  volatile uint32_t RASR_A1;                  
  volatile uint32_t RBAR_A2;                  
  volatile uint32_t RASR_A2;                  
  volatile uint32_t RBAR_A3;                  
  volatile uint32_t RASR_A3;                  
} MPU_Type;                                                

 









 









 



 









 



























 







 


 
typedef struct
{
  volatile uint32_t DHCSR;                    
  volatile  uint32_t DCRSR;                    
  volatile uint32_t DCRDR;                    
  volatile uint32_t DEMCR;                    
} CoreDebug_Type;

 




































 






 







































 




 
 
 
#line 848 "C:\\Keil\\ARM\\CMSIS\\Include\\core_cm3.h"

#line 855 "C:\\Keil\\ARM\\CMSIS\\Include\\core_cm3.h"






 





 






 



 



 










 
static __inline void NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);                          
  
  reg_value  =  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL))->AIRCR;                                                    
  reg_value &= ~((0xFFFFUL << 16) | (7UL << 8));              
  reg_value  =  (reg_value                       |
                (0x5FA << 16) | 
                (PriorityGroupTmp << 8));                                      
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL))->AIRCR =  reg_value;
}








 
static __inline uint32_t NVIC_GetPriorityGrouping(void)
{
  return ((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL))->AIRCR & (7UL << 8)) >> 8);    
}








 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL))->ISER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}








 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL))->ICER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}










 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t) ((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL))->ISPR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));  
}








 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL))->ISPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}








 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL))->ICPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}








 
static __inline uint32_t NVIC_GetActive(IRQn_Type IRQn)
{
  return((uint32_t)((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL))->IABR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));  
}












 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if(IRQn < 0) {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL))->SHP[((uint32_t)(IRQn) & 0xF)-4] = ((priority << (8 - 5)) & 0xff); }  
  else {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL))->IP[(uint32_t)(IRQn)] = ((priority << (8 - 5)) & 0xff);    }         
}













 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if(IRQn < 0) {
    return((uint32_t)(((SCB_Type *) ((0xE000E000UL) + 0x0D00UL))->SHP[((uint32_t)(IRQn) & 0xF)-4] >> (8 - 5)));  }  
  else {
    return((uint32_t)(((NVIC_Type *) ((0xE000E000UL) + 0x0100UL))->IP[(uint32_t)(IRQn)]           >> (8 - 5)));  }  
}















 
static __inline uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);           
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7 - PriorityGroupTmp) > 5) ? 5 : 7 - PriorityGroupTmp;
  SubPriorityBits     = ((PriorityGroupTmp + 5) < 7) ? 0 : PriorityGroupTmp - 7 + 5;
 
  return (
           ((PreemptPriority & ((1 << (PreemptPriorityBits)) - 1)) << SubPriorityBits) |
           ((SubPriority     & ((1 << (SubPriorityBits    )) - 1)))
         );
}















 
static __inline void NVIC_DecodePriority (uint32_t Priority, uint32_t PriorityGroup, uint32_t* pPreemptPriority, uint32_t* pSubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);           
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7 - PriorityGroupTmp) > 5) ? 5 : 7 - PriorityGroupTmp;
  SubPriorityBits     = ((PriorityGroupTmp + 5) < 7) ? 0 : PriorityGroupTmp - 7 + 5;
  
  *pPreemptPriority = (Priority >> SubPriorityBits) & ((1 << (PreemptPriorityBits)) - 1);
  *pSubPriority     = (Priority                   ) & ((1 << (SubPriorityBits    )) - 1);
}





 
static __inline void NVIC_SystemReset(void)
{
  __dsb(0xF);                                                     
               
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL))->AIRCR  = ((0x5FA << 16)      | 
                 (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL))->AIRCR & (7UL << 8)) | 
                 (1UL << 2));                    
  __dsb(0xF);                                                                    
  while(1);                                                     
}

 



 



 











 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{ 
  if (ticks > (0xFFFFFFUL << 0))  return (1);             
                                                               
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL))->LOAD  = (ticks & (0xFFFFFFUL << 0)) - 1;       
  NVIC_SetPriority (SysTick_IRQn, (1<<5) - 1);   
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL))->VAL   = 0;                                           
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL))->CTRL  = (1UL << 2) | 
                   (1UL << 1)   | 
                   (1UL << 0);                     
  return (0);                                                   
}



 



 



 

extern volatile int32_t ITM_RxBuffer;                     











 
static __inline uint32_t ITM_SendChar (uint32_t ch)
{
  if ((((CoreDebug_Type *) (0xE000EDF0UL))->DEMCR & (1UL << 24))  &&       
      (((ITM_Type *) (0xE0000000UL))->TCR & (1UL << 0))                  &&       
      (((ITM_Type *) (0xE0000000UL))->TER & (1UL << 0)        )                    )      
  {
    while (((ITM_Type *) (0xE0000000UL))->PORT[0].u32 == 0);
    ((ITM_Type *) (0xE0000000UL))->PORT[0].u8 = (uint8_t) ch;
  }  
  return (ch);
}










 
static __inline int32_t ITM_ReceiveChar (void) {
  int32_t ch = -1;                            

  if (ITM_RxBuffer != 0x5AA55AA5) {
    ch = ITM_RxBuffer;
    ITM_RxBuffer = 0x5AA55AA5;        
  }
  
  return (ch); 
}









 
static __inline int32_t ITM_CheckChar (void) {

  if (ITM_RxBuffer == 0x5AA55AA5) {
    return (0);                                  
  } else {
    return (1);                                  
  }
}

 









 
#line 97 "C:\\Keil\\ARM\\INC\\NXP\\LPC17xx\\lpc17xx.h"
#line 1 "C:\\Keil\\ARM\\INC\\NXP\\LPC17xx\\system_LPC17xx.h"




















 









extern uint32_t SystemFrequency;     










 
extern void SystemInit (void);





#line 98 "C:\\Keil\\ARM\\INC\\NXP\\LPC17xx\\lpc17xx.h"


 
 
 


#pragma anon_unions


 
typedef struct
{
  volatile uint32_t FLASHCFG;                
       uint32_t RESERVED0[31];
  volatile uint32_t PLL0CON;                 
  volatile uint32_t PLL0CFG;
  volatile const  uint32_t PLL0STAT;
  volatile  uint32_t PLL0FEED;
       uint32_t RESERVED1[4];
  volatile uint32_t PLL1CON;
  volatile uint32_t PLL1CFG;
  volatile const  uint32_t PLL1STAT;
  volatile  uint32_t PLL1FEED;
       uint32_t RESERVED2[4];
  volatile uint32_t PCON;
  volatile uint32_t PCONP;
       uint32_t RESERVED3[15];
  volatile uint32_t CCLKCFG;
  volatile uint32_t USBCLKCFG;
  volatile uint32_t CLKSRCSEL;
  volatile uint32_t	CANSLEEPCLR;
  volatile uint32_t	CANWAKEFLAGS;
       uint32_t RESERVED4[10];
  volatile uint32_t EXTINT;                  
       uint32_t RESERVED5;
  volatile uint32_t EXTMODE;
  volatile uint32_t EXTPOLAR;
       uint32_t RESERVED6[12];
  volatile uint32_t RSID;                    
       uint32_t RESERVED7[7];
  volatile uint32_t SCS;                     
  volatile uint32_t IRCTRIM;                 
  volatile uint32_t PCLKSEL0;
  volatile uint32_t PCLKSEL1;
       uint32_t RESERVED8[4];
  volatile uint32_t USBIntSt;                
  volatile uint32_t DMAREQSEL;
  volatile uint32_t CLKOUTCFG;               
 } LPC_SC_TypeDef;

 
typedef struct
{
  volatile uint32_t PINSEL0;
  volatile uint32_t PINSEL1;
  volatile uint32_t PINSEL2;
  volatile uint32_t PINSEL3;
  volatile uint32_t PINSEL4;
  volatile uint32_t PINSEL5;
  volatile uint32_t PINSEL6;
  volatile uint32_t PINSEL7;
  volatile uint32_t PINSEL8;
  volatile uint32_t PINSEL9;
  volatile uint32_t PINSEL10;
       uint32_t RESERVED0[5];
  volatile uint32_t PINMODE0;
  volatile uint32_t PINMODE1;
  volatile uint32_t PINMODE2;
  volatile uint32_t PINMODE3;
  volatile uint32_t PINMODE4;
  volatile uint32_t PINMODE5;
  volatile uint32_t PINMODE6;
  volatile uint32_t PINMODE7;
  volatile uint32_t PINMODE8;
  volatile uint32_t PINMODE9;
  volatile uint32_t PINMODE_OD0;
  volatile uint32_t PINMODE_OD1;
  volatile uint32_t PINMODE_OD2;
  volatile uint32_t PINMODE_OD3;
  volatile uint32_t PINMODE_OD4;
  volatile uint32_t I2CPADCFG;
} LPC_PINCON_TypeDef;

 
typedef struct
{
  union {
    volatile uint32_t FIODIR;
    struct {
      volatile uint16_t FIODIRL;
      volatile uint16_t FIODIRH;
    };
    struct {
      volatile uint8_t  FIODIR0;
      volatile uint8_t  FIODIR1;
      volatile uint8_t  FIODIR2;
      volatile uint8_t  FIODIR3;
    };
  };
  uint32_t RESERVED0[3];
  union {
    volatile uint32_t FIOMASK;
    struct {
      volatile uint16_t FIOMASKL;
      volatile uint16_t FIOMASKH;
    };
    struct {
      volatile uint8_t  FIOMASK0;
      volatile uint8_t  FIOMASK1;
      volatile uint8_t  FIOMASK2;
      volatile uint8_t  FIOMASK3;
    };
  };
  union {
    volatile uint32_t FIOPIN;
    struct {
      volatile uint16_t FIOPINL;
      volatile uint16_t FIOPINH;
    };
    struct {
      volatile uint8_t  FIOPIN0;
      volatile uint8_t  FIOPIN1;
      volatile uint8_t  FIOPIN2;
      volatile uint8_t  FIOPIN3;
    };
  };
  union {
    volatile uint32_t FIOSET;
    struct {
      volatile uint16_t FIOSETL;
      volatile uint16_t FIOSETH;
    };
    struct {
      volatile uint8_t  FIOSET0;
      volatile uint8_t  FIOSET1;
      volatile uint8_t  FIOSET2;
      volatile uint8_t  FIOSET3;
    };
  };
  union {
    volatile  uint32_t FIOCLR;
    struct {
      volatile  uint16_t FIOCLRL;
      volatile  uint16_t FIOCLRH;
    };
    struct {
      volatile  uint8_t  FIOCLR0;
      volatile  uint8_t  FIOCLR1;
      volatile  uint8_t  FIOCLR2;
      volatile  uint8_t  FIOCLR3;
    };
  };
} LPC_GPIO_TypeDef;

typedef struct
{
  volatile const  uint32_t IntStatus;
  volatile const  uint32_t IO0IntStatR;
  volatile const  uint32_t IO0IntStatF;
  volatile  uint32_t IO0IntClr;
  volatile uint32_t IO0IntEnR;
  volatile uint32_t IO0IntEnF;
       uint32_t RESERVED0[3];
  volatile const  uint32_t IO2IntStatR;
  volatile const  uint32_t IO2IntStatF;
  volatile  uint32_t IO2IntClr;
  volatile uint32_t IO2IntEnR;
  volatile uint32_t IO2IntEnF;
} LPC_GPIOINT_TypeDef;

 
typedef struct
{
  volatile uint32_t IR;
  volatile uint32_t TCR;
  volatile uint32_t TC;
  volatile uint32_t PR;
  volatile uint32_t PC;
  volatile uint32_t MCR;
  volatile uint32_t MR0;
  volatile uint32_t MR1;
  volatile uint32_t MR2;
  volatile uint32_t MR3;
  volatile uint32_t CCR;
  volatile const  uint32_t CR0;
  volatile const  uint32_t CR1;
       uint32_t RESERVED0[2];
  volatile uint32_t EMR;
       uint32_t RESERVED1[12];
  volatile uint32_t CTCR;
} LPC_TIM_TypeDef;

 
typedef struct
{
  volatile uint32_t IR;
  volatile uint32_t TCR;
  volatile uint32_t TC;
  volatile uint32_t PR;
  volatile uint32_t PC;
  volatile uint32_t MCR;
  volatile uint32_t MR0;
  volatile uint32_t MR1;
  volatile uint32_t MR2;
  volatile uint32_t MR3;
  volatile uint32_t CCR;
  volatile const  uint32_t CR0;
  volatile const  uint32_t CR1;
  volatile const  uint32_t CR2;
  volatile const  uint32_t CR3;
       uint32_t RESERVED0;
  volatile uint32_t MR4;
  volatile uint32_t MR5;
  volatile uint32_t MR6;
  volatile uint32_t PCR;
  volatile uint32_t LER;
       uint32_t RESERVED1[7];
  volatile uint32_t CTCR;
} LPC_PWM_TypeDef;

 
typedef struct
{
  union {
  volatile const  uint8_t  RBR;
  volatile  uint8_t  THR;
  volatile uint8_t  DLL;
       uint32_t RESERVED0;
  };
  union {
  volatile uint8_t  DLM;
  volatile uint32_t IER;
  };
  union {
  volatile const  uint32_t IIR;
  volatile  uint8_t  FCR;
  };
  volatile uint8_t  LCR;
       uint8_t  RESERVED1[7];
  volatile const  uint8_t  LSR;
       uint8_t  RESERVED2[7];
  volatile uint8_t  SCR;
       uint8_t  RESERVED3[3];
  volatile uint32_t ACR;
  volatile uint8_t  ICR;
       uint8_t  RESERVED4[3];
  volatile uint8_t  FDR;
       uint8_t  RESERVED5[7];
  volatile uint8_t  TER;
       uint8_t  RESERVED6[39];
  volatile uint32_t FIFOLVL;
} LPC_UART_TypeDef;

typedef struct
{
  union {
  volatile const  uint8_t  RBR;
  volatile  uint8_t  THR;
  volatile uint8_t  DLL;
       uint32_t RESERVED0;
  };
  union {
  volatile uint8_t  DLM;
  volatile uint32_t IER;
  };
  union {
  volatile const  uint32_t IIR;
  volatile  uint8_t  FCR;
  };
  volatile uint8_t  LCR;
       uint8_t  RESERVED1[7];
  volatile const  uint8_t  LSR;
       uint8_t  RESERVED2[7];
  volatile uint8_t  SCR;
       uint8_t  RESERVED3[3];
  volatile uint32_t ACR;
  volatile uint8_t  ICR;
       uint8_t  RESERVED4[3];
  volatile uint8_t  FDR;
       uint8_t  RESERVED5[7];
  volatile uint8_t  TER;
       uint8_t  RESERVED6[39];
  volatile uint32_t FIFOLVL;
} LPC_UART0_TypeDef;

typedef struct
{
  union {
  volatile const  uint8_t  RBR;
  volatile  uint8_t  THR;
  volatile uint8_t  DLL;
       uint32_t RESERVED0;
  };
  union {
  volatile uint8_t  DLM;
  volatile uint32_t IER;
  };
  union {
  volatile const  uint32_t IIR;
  volatile  uint8_t  FCR;
  };
  volatile uint8_t  LCR;
       uint8_t  RESERVED1[3];
  volatile uint8_t  MCR;
       uint8_t  RESERVED2[3];
  volatile const  uint8_t  LSR;
       uint8_t  RESERVED3[3];
  volatile const  uint8_t  MSR;
       uint8_t  RESERVED4[3];
  volatile uint8_t  SCR;
       uint8_t  RESERVED5[3];
  volatile uint32_t ACR;
       uint32_t RESERVED6;
  volatile uint32_t FDR;
       uint32_t RESERVED7;
  volatile uint8_t  TER;
       uint8_t  RESERVED8[27];
  volatile uint8_t  RS485CTRL;
       uint8_t  RESERVED9[3];
  volatile uint8_t  ADRMATCH;
       uint8_t  RESERVED10[3];
  volatile uint8_t  RS485DLY;
       uint8_t  RESERVED11[3];
  volatile uint32_t FIFOLVL;
} LPC_UART1_TypeDef;

 
typedef struct
{
  volatile uint32_t SPCR;
  volatile const  uint32_t SPSR;
  volatile uint32_t SPDR;
  volatile uint32_t SPCCR;
       uint32_t RESERVED0[3];
  volatile uint32_t SPINT;
} LPC_SPI_TypeDef;

 
typedef struct
{
  volatile uint32_t CR0;
  volatile uint32_t CR1;
  volatile uint32_t DR;
  volatile const  uint32_t SR;
  volatile uint32_t CPSR;
  volatile uint32_t IMSC;
  volatile uint32_t RIS;
  volatile uint32_t MIS;
  volatile uint32_t ICR;
  volatile uint32_t DMACR;
} LPC_SSP_TypeDef;

 
typedef struct
{
  volatile uint32_t I2CONSET;
  volatile const  uint32_t I2STAT;
  volatile uint32_t I2DAT;
  volatile uint32_t I2ADR0;
  volatile uint32_t I2SCLH;
  volatile uint32_t I2SCLL;
  volatile  uint32_t I2CONCLR;
  volatile uint32_t MMCTRL;
  volatile uint32_t I2ADR1;
  volatile uint32_t I2ADR2;
  volatile uint32_t I2ADR3;
  volatile const  uint32_t I2DATA_BUFFER;
  volatile uint32_t I2MASK0;
  volatile uint32_t I2MASK1;
  volatile uint32_t I2MASK2;
  volatile uint32_t I2MASK3;
} LPC_I2C_TypeDef;

 
typedef struct
{
  volatile uint32_t I2SDAO;
  volatile uint32_t I2SDAI;
  volatile  uint32_t I2STXFIFO;
  volatile const  uint32_t I2SRXFIFO;
  volatile const  uint32_t I2SSTATE;
  volatile uint32_t I2SDMA1;
  volatile uint32_t I2SDMA2;
  volatile uint32_t I2SIRQ;
  volatile uint32_t I2STXRATE;
  volatile uint32_t I2SRXRATE;
  volatile uint32_t I2STXBITRATE;
  volatile uint32_t I2SRXBITRATE;
  volatile uint32_t I2STXMODE;
  volatile uint32_t I2SRXMODE;
} LPC_I2S_TypeDef;

 
typedef struct
{
  volatile uint32_t RICOMPVAL;
  volatile uint32_t RIMASK;
  volatile uint8_t  RICTRL;
       uint8_t  RESERVED0[3];
  volatile uint32_t RICOUNTER;
} LPC_RIT_TypeDef;

 
typedef struct
{
  volatile uint8_t  ILR;
       uint8_t  RESERVED0[7];
  volatile uint8_t  CCR;
       uint8_t  RESERVED1[3];
  volatile uint8_t  CIIR;
       uint8_t  RESERVED2[3];
  volatile uint8_t  AMR;
       uint8_t  RESERVED3[3];
  volatile const  uint32_t CTIME0;
  volatile const  uint32_t CTIME1;
  volatile const  uint32_t CTIME2;
  volatile uint8_t  SEC;
       uint8_t  RESERVED4[3];
  volatile uint8_t  MIN;
       uint8_t  RESERVED5[3];
  volatile uint8_t  HOUR;
       uint8_t  RESERVED6[3];
  volatile uint8_t  DOM;
       uint8_t  RESERVED7[3];
  volatile uint8_t  DOW;
       uint8_t  RESERVED8[3];
  volatile uint16_t DOY;
       uint16_t RESERVED9;
  volatile uint8_t  MONTH;
       uint8_t  RESERVED10[3];
  volatile uint16_t YEAR;
       uint16_t RESERVED11;
  volatile uint32_t CALIBRATION;
  volatile uint32_t GPREG0;
  volatile uint32_t GPREG1;
  volatile uint32_t GPREG2;
  volatile uint32_t GPREG3;
  volatile uint32_t GPREG4;
  volatile uint8_t  RTC_AUXEN;
       uint8_t  RESERVED12[3];
  volatile uint8_t  RTC_AUX;
       uint8_t  RESERVED13[3];
  volatile uint8_t  ALSEC;
       uint8_t  RESERVED14[3];
  volatile uint8_t  ALMIN;
       uint8_t  RESERVED15[3];
  volatile uint8_t  ALHOUR;
       uint8_t  RESERVED16[3];
  volatile uint8_t  ALDOM;
       uint8_t  RESERVED17[3];
  volatile uint8_t  ALDOW;
       uint8_t  RESERVED18[3];
  volatile uint16_t ALDOY;
       uint16_t RESERVED19;
  volatile uint8_t  ALMON;
       uint8_t  RESERVED20[3];
  volatile uint16_t ALYEAR;
       uint16_t RESERVED21;
} LPC_RTC_TypeDef;

 
typedef struct
{
  volatile uint8_t  WDMOD;
       uint8_t  RESERVED0[3];
  volatile uint32_t WDTC;
  volatile  uint8_t  WDFEED;
       uint8_t  RESERVED1[3];
  volatile const  uint32_t WDTV;
  volatile uint32_t WDCLKSEL;
} LPC_WDT_TypeDef;

 
typedef struct
{
  volatile uint32_t ADCR;
  volatile uint32_t ADGDR;
       uint32_t RESERVED0;
  volatile uint32_t ADINTEN;
  volatile const  uint32_t ADDR0;
  volatile const  uint32_t ADDR1;
  volatile const  uint32_t ADDR2;
  volatile const  uint32_t ADDR3;
  volatile const  uint32_t ADDR4;
  volatile const  uint32_t ADDR5;
  volatile const  uint32_t ADDR6;
  volatile const  uint32_t ADDR7;
  volatile const  uint32_t ADSTAT;
  volatile uint32_t ADTRM;
} LPC_ADC_TypeDef;

 
typedef struct
{
  volatile uint32_t DACR;
  volatile uint32_t DACCTRL;
  volatile uint16_t DACCNTVAL;
} LPC_DAC_TypeDef;

 
typedef struct
{
  volatile const  uint32_t MCCON;
  volatile  uint32_t MCCON_SET;
  volatile  uint32_t MCCON_CLR;
  volatile const  uint32_t MCCAPCON;
  volatile  uint32_t MCCAPCON_SET;
  volatile  uint32_t MCCAPCON_CLR;
  volatile uint32_t MCTIM0;
  volatile uint32_t MCTIM1;
  volatile uint32_t MCTIM2;
  volatile uint32_t MCPER0;
  volatile uint32_t MCPER1;
  volatile uint32_t MCPER2;
  volatile uint32_t MCPW0;
  volatile uint32_t MCPW1;
  volatile uint32_t MCPW2;
  volatile uint32_t MCDEADTIME;
  volatile uint32_t MCCCP;
  volatile uint32_t MCCR0;
  volatile uint32_t MCCR1;
  volatile uint32_t MCCR2;
  volatile const  uint32_t MCINTEN;
  volatile  uint32_t MCINTEN_SET;
  volatile  uint32_t MCINTEN_CLR;
  volatile const  uint32_t MCCNTCON;
  volatile  uint32_t MCCNTCON_SET;
  volatile  uint32_t MCCNTCON_CLR;
  volatile const  uint32_t MCINTFLAG;
  volatile  uint32_t MCINTFLAG_SET;
  volatile  uint32_t MCINTFLAG_CLR;
  volatile  uint32_t MCCAP_CLR;
} LPC_MCPWM_TypeDef;

 
typedef struct
{
  volatile  uint32_t QEICON;
  volatile const  uint32_t QEISTAT;
  volatile uint32_t QEICONF;
  volatile const  uint32_t QEIPOS;
  volatile uint32_t QEIMAXPOS;
  volatile uint32_t CMPOS0;
  volatile uint32_t CMPOS1;
  volatile uint32_t CMPOS2;
  volatile const  uint32_t INXCNT;
  volatile uint32_t INXCMP;
  volatile uint32_t QEILOAD;
  volatile const  uint32_t QEITIME;
  volatile const  uint32_t QEIVEL;
  volatile const  uint32_t QEICAP;
  volatile uint32_t VELCOMP;
  volatile uint32_t FILTER;
       uint32_t RESERVED0[998];
  volatile  uint32_t QEIIEC;
  volatile  uint32_t QEIIES;
  volatile const  uint32_t QEIINTSTAT;
  volatile const  uint32_t QEIIE;
  volatile  uint32_t QEICLR;
  volatile  uint32_t QEISET;
} LPC_QEI_TypeDef;

 
typedef struct
{
  volatile uint32_t mask[512];               
} LPC_CANAF_RAM_TypeDef;

typedef struct                           
{
  volatile uint32_t AFMR;
  volatile uint32_t SFF_sa;
  volatile uint32_t SFF_GRP_sa;
  volatile uint32_t EFF_sa;
  volatile uint32_t EFF_GRP_sa;
  volatile uint32_t ENDofTable;
  volatile const  uint32_t LUTerrAd;
  volatile const  uint32_t LUTerr;
  volatile uint32_t FCANIE;
  volatile uint32_t FCANIC0;
  volatile uint32_t FCANIC1;
} LPC_CANAF_TypeDef;

typedef struct                           
{
  volatile const  uint32_t CANTxSR;
  volatile const  uint32_t CANRxSR;
  volatile const  uint32_t CANMSR;
} LPC_CANCR_TypeDef;

typedef struct                           
{
  volatile uint32_t MOD;
  volatile  uint32_t CMR;
  volatile uint32_t GSR;
  volatile const  uint32_t ICR;
  volatile uint32_t IER;
  volatile uint32_t BTR;
  volatile uint32_t EWL;
  volatile const  uint32_t SR;
  volatile uint32_t RFS;
  volatile uint32_t RID;
  volatile uint32_t RDA;
  volatile uint32_t RDB;
  volatile uint32_t TFI1;
  volatile uint32_t TID1;
  volatile uint32_t TDA1;
  volatile uint32_t TDB1;
  volatile uint32_t TFI2;
  volatile uint32_t TID2;
  volatile uint32_t TDA2;
  volatile uint32_t TDB2;
  volatile uint32_t TFI3;
  volatile uint32_t TID3;
  volatile uint32_t TDA3;
  volatile uint32_t TDB3;
} LPC_CAN_TypeDef;

 
typedef struct                           
{
  volatile const  uint32_t DMACIntStat;
  volatile const  uint32_t DMACIntTCStat;
  volatile  uint32_t DMACIntTCClear;
  volatile const  uint32_t DMACIntErrStat;
  volatile  uint32_t DMACIntErrClr;
  volatile const  uint32_t DMACRawIntTCStat;
  volatile const  uint32_t DMACRawIntErrStat;
  volatile const  uint32_t DMACEnbldChns;
  volatile uint32_t DMACSoftBReq;
  volatile uint32_t DMACSoftSReq;
  volatile uint32_t DMACSoftLBReq;
  volatile uint32_t DMACSoftLSReq;
  volatile uint32_t DMACConfig;
  volatile uint32_t DMACSync;
} LPC_GPDMA_TypeDef;

typedef struct                           
{
  volatile uint32_t DMACCSrcAddr;
  volatile uint32_t DMACCDestAddr;
  volatile uint32_t DMACCLLI;
  volatile uint32_t DMACCControl;
  volatile uint32_t DMACCConfig;
} LPC_GPDMACH_TypeDef;

 
typedef struct
{
  volatile const  uint32_t HcRevision;              
  volatile uint32_t HcControl;
  volatile uint32_t HcCommandStatus;
  volatile uint32_t HcInterruptStatus;
  volatile uint32_t HcInterruptEnable;
  volatile uint32_t HcInterruptDisable;
  volatile uint32_t HcHCCA;
  volatile const  uint32_t HcPeriodCurrentED;
  volatile uint32_t HcControlHeadED;
  volatile uint32_t HcControlCurrentED;
  volatile uint32_t HcBulkHeadED;
  volatile uint32_t HcBulkCurrentED;
  volatile const  uint32_t HcDoneHead;
  volatile uint32_t HcFmInterval;
  volatile const  uint32_t HcFmRemaining;
  volatile const  uint32_t HcFmNumber;
  volatile uint32_t HcPeriodicStart;
  volatile uint32_t HcLSTreshold;
  volatile uint32_t HcRhDescriptorA;
  volatile uint32_t HcRhDescriptorB;
  volatile uint32_t HcRhStatus;
  volatile uint32_t HcRhPortStatus1;
  volatile uint32_t HcRhPortStatus2;
       uint32_t RESERVED0[40];
  volatile const  uint32_t Module_ID;

  volatile const  uint32_t OTGIntSt;                
  volatile uint32_t OTGIntEn;
  volatile  uint32_t OTGIntSet;
  volatile  uint32_t OTGIntClr;
  volatile uint32_t OTGStCtrl;
  volatile uint32_t OTGTmr;
       uint32_t RESERVED1[58];

  volatile const  uint32_t USBDevIntSt;             
  volatile uint32_t USBDevIntEn;
  volatile  uint32_t USBDevIntClr;
  volatile  uint32_t USBDevIntSet;

  volatile  uint32_t USBCmdCode;              
  volatile const  uint32_t USBCmdData;

  volatile const  uint32_t USBRxData;               
  volatile  uint32_t USBTxData;
  volatile const  uint32_t USBRxPLen;
  volatile  uint32_t USBTxPLen;
  volatile uint32_t USBCtrl;
  volatile  uint32_t USBDevIntPri;

  volatile const  uint32_t USBEpIntSt;              
  volatile uint32_t USBEpIntEn;
  volatile  uint32_t USBEpIntClr;
  volatile  uint32_t USBEpIntSet;
  volatile  uint32_t USBEpIntPri;

  volatile uint32_t USBReEp;                 
  volatile  uint32_t USBEpInd;
  volatile uint32_t USBMaxPSize;

  volatile const  uint32_t USBDMARSt;               
  volatile  uint32_t USBDMARClr;
  volatile  uint32_t USBDMARSet;
       uint32_t RESERVED2[9];
  volatile uint32_t USBUDCAH;
  volatile const  uint32_t USBEpDMASt;
  volatile  uint32_t USBEpDMAEn;
  volatile  uint32_t USBEpDMADis;
  volatile const  uint32_t USBDMAIntSt;
  volatile uint32_t USBDMAIntEn;
       uint32_t RESERVED3[2];
  volatile const  uint32_t USBEoTIntSt;
  volatile  uint32_t USBEoTIntClr;
  volatile  uint32_t USBEoTIntSet;
  volatile const  uint32_t USBNDDRIntSt;
  volatile  uint32_t USBNDDRIntClr;
  volatile  uint32_t USBNDDRIntSet;
  volatile const  uint32_t USBSysErrIntSt;
  volatile  uint32_t USBSysErrIntClr;
  volatile  uint32_t USBSysErrIntSet;
       uint32_t RESERVED4[15];

  union {
  volatile const  uint32_t I2C_RX;                  
  volatile  uint32_t I2C_TX;
  };
  volatile const  uint32_t I2C_STS;
  volatile uint32_t I2C_CTL;
  volatile uint32_t I2C_CLKHI;
  volatile  uint32_t I2C_CLKLO;
       uint32_t RESERVED5[824];

  union {
  volatile uint32_t USBClkCtrl;              
  volatile uint32_t OTGClkCtrl;
  };
  union {
  volatile const  uint32_t USBClkSt;
  volatile const  uint32_t OTGClkSt;
  };
} LPC_USB_TypeDef;

 
typedef struct
{
  volatile uint32_t MAC1;                    
  volatile uint32_t MAC2;
  volatile uint32_t IPGT;
  volatile uint32_t IPGR;
  volatile uint32_t CLRT;
  volatile uint32_t MAXF;
  volatile uint32_t SUPP;
  volatile uint32_t TEST;
  volatile uint32_t MCFG;
  volatile uint32_t MCMD;
  volatile uint32_t MADR;
  volatile  uint32_t MWTD;
  volatile const  uint32_t MRDD;
  volatile const  uint32_t MIND;
       uint32_t RESERVED0[2];
  volatile uint32_t SA0;
  volatile uint32_t SA1;
  volatile uint32_t SA2;
       uint32_t RESERVED1[45];
  volatile uint32_t Command;                 
  volatile const  uint32_t Status;
  volatile uint32_t RxDescriptor;
  volatile uint32_t RxStatus;
  volatile uint32_t RxDescriptorNumber;
  volatile const  uint32_t RxProduceIndex;
  volatile uint32_t RxConsumeIndex;
  volatile uint32_t TxDescriptor;
  volatile uint32_t TxStatus;
  volatile uint32_t TxDescriptorNumber;
  volatile uint32_t TxProduceIndex;
  volatile const  uint32_t TxConsumeIndex;
       uint32_t RESERVED2[10];
  volatile const  uint32_t TSV0;
  volatile const  uint32_t TSV1;
  volatile const  uint32_t RSV;
       uint32_t RESERVED3[3];
  volatile uint32_t FlowControlCounter;
  volatile const  uint32_t FlowControlStatus;
       uint32_t RESERVED4[34];
  volatile uint32_t RxFilterCtrl;            
  volatile uint32_t RxFilterWoLStatus;
  volatile uint32_t RxFilterWoLClear;
       uint32_t RESERVED5;
  volatile uint32_t HashFilterL;
  volatile uint32_t HashFilterH;
       uint32_t RESERVED6[882];
  volatile const  uint32_t IntStatus;               
  volatile uint32_t IntEnable;
  volatile  uint32_t IntClear;
  volatile  uint32_t IntSet;
       uint32_t RESERVED7;
  volatile uint32_t PowerDown;
       uint32_t RESERVED8;
  volatile uint32_t Module_ID;
} LPC_EMAC_TypeDef;


#pragma no_anon_unions



 
 
 
 
#line 924 "C:\\Keil\\ARM\\INC\\NXP\\LPC17xx\\lpc17xx.h"

 
#line 945 "C:\\Keil\\ARM\\INC\\NXP\\LPC17xx\\lpc17xx.h"

 
#line 959 "C:\\Keil\\ARM\\INC\\NXP\\LPC17xx\\lpc17xx.h"

 
#line 972 "C:\\Keil\\ARM\\INC\\NXP\\LPC17xx\\lpc17xx.h"

 







 
 
 
#line 1031 "C:\\Keil\\ARM\\INC\\NXP\\LPC17xx\\lpc17xx.h"

#line 4 "modbus.c"
#line 1 "modbus.h"

extern unsigned char modbus_buf[20];
extern short modbus_crc16;
extern char modbus_timeout_cnt;
extern char bMODBUS_TIMEOUT;
extern unsigned char modbus_rx_buffer[30];	
extern unsigned char modbus_an_buffer[30];	
extern unsigned char modbus_rx_buffer_ptr;	
extern unsigned char modbus_rx_counter;		

extern short modbus_plazma;				
extern short modbus_plazma1;				
extern short modbus_plazma2;				
extern short modbus_plazma3;				

extern unsigned short modbus_rx_arg0;		
extern unsigned short modbus_rx_arg1;		
extern unsigned short modbus_rx_arg2;		
extern unsigned short modbus_rx_arg3;		

extern char modbus_tx_buff[100];



unsigned short CRC16_2(char* buf, short len);





void modbus_hold_registers_transmit(unsigned char adr,unsigned char func,unsigned short reg_adr,unsigned short reg_quantity, char prot);

void modbus_input_registers_transmit(unsigned char adr,unsigned char func,unsigned short reg_adr,unsigned short reg_quantity, char prot);


void modbus_zapros_ENMV (void);



#line 5 "modbus.c"

#line 1 "main.h"


#line 1 "C:\\Keil\\ARM\\RV31\\INC\\rtl.h"









 




 

 


#line 27 "C:\\Keil\\ARM\\RV31\\INC\\rtl.h"







 typedef unsigned int   size_t;


typedef signed char     S8;
typedef unsigned char   U8;
typedef short           S16;
typedef unsigned short  U16;
typedef int             S32;
typedef unsigned int    U32;
typedef long long       S64;
typedef unsigned long long U64;
typedef unsigned char   BIT;
typedef unsigned int    BOOL;

#line 54 "C:\\Keil\\ARM\\RV31\\INC\\rtl.h"

#line 66 "C:\\Keil\\ARM\\RV31\\INC\\rtl.h"



 





 
typedef U32 OS_SEM[2];

 

typedef U32 OS_MBX[];

 
typedef U32 OS_MUT[3];

 
typedef U32 OS_TID;

 
typedef void *OS_ID;

 
typedef U32 OS_RESULT;

 












 




#line 194 "C:\\Keil\\ARM\\RV31\\INC\\rtl.h"



 



 
extern void      os_set_env    (void);
extern void      rt_sys_init   (void (*task)(void), U8 priority, void *stk);
extern void      rt_tsk_pass   (void);
extern OS_TID    rt_tsk_self   (void);
extern OS_RESULT rt_tsk_prio   (OS_TID task_id, U8 new_prio);
extern OS_TID    rt_tsk_create (void (*task)(void), U8 priority, void *stk, void *argv);
extern OS_RESULT rt_tsk_delete (OS_TID task_id);

#line 230 "C:\\Keil\\ARM\\RV31\\INC\\rtl.h"

extern void      _os_sys_init(U32 p, void (*task)(void), U32 prio_stksz,
                                     void *stk)                        __svc_indirect(0);
extern OS_TID    _os_tsk_create (U32 p, void (*task)(void), U32 prio_stksz,
                                        void *stk, void *argv)         __svc_indirect(0);
extern OS_TID    _os_tsk_create_ex (U32 p, void (*task)(void *), U32 prio_stksz,
                                           void *stk, void *argv)      __svc_indirect(0);
extern OS_TID    _os_tsk_self (U32 p)                                  __svc_indirect(0);
extern void      _os_tsk_pass (U32 p)                                  __svc_indirect(0);
extern OS_RESULT _os_tsk_prio (U32 p, OS_TID task_id, U8 new_prio)     __svc_indirect(0);
extern OS_RESULT _os_tsk_delete (U32 p, OS_TID task_id)                __svc_indirect(0);

 
extern OS_RESULT rt_evt_wait (U16 wait_flags,  U16 timeout, BOOL and_wait);
extern void      rt_evt_set  (U16 event_flags, OS_TID task_id);
extern void      rt_evt_clr  (U16 clear_flags, OS_TID task_id);
extern U16       rt_evt_get  (void);







extern OS_RESULT _os_evt_wait(U32 p, U16 wait_flags, U16 timeout,
                                     BOOL and_wait)                    __svc_indirect(0);
extern void      _os_evt_set (U32 p, U16 event_flags, OS_TID task_id)  __svc_indirect(0);
extern void      _os_evt_clr (U32 p, U16 clear_flags, OS_TID task_id)  __svc_indirect(0);
extern U16       _os_evt_get (U32 p)                                   __svc_indirect(0);
extern void      isr_evt_set (U16 event_flags, OS_TID task_id);

 
extern void      rt_sem_init (OS_ID semaphore, U16 token_count);
extern OS_RESULT rt_sem_send (OS_ID semaphore);
extern OS_RESULT rt_sem_wait (OS_ID semaphore, U16 timeout);





extern void      _os_sem_init (U32 p, OS_ID semaphore, 
                                      U16 token_count)                 __svc_indirect(0);
extern OS_RESULT _os_sem_send (U32 p, OS_ID semaphore)                 __svc_indirect(0);
extern OS_RESULT _os_sem_wait (U32 p, OS_ID semaphore, U16 timeout)    __svc_indirect(0);
extern void      isr_sem_send (OS_ID semaphore);

 
extern void      rt_mbx_init  (OS_ID mailbox, U16 mbx_size);
extern OS_RESULT rt_mbx_send  (OS_ID mailbox, void *p_msg,    U16 timeout);
extern OS_RESULT rt_mbx_wait  (OS_ID mailbox, void **message, U16 timeout);
extern OS_RESULT rt_mbx_check (OS_ID mailbox);







extern void      _os_mbx_init (U32 p, OS_ID mailbox, U16 mbx_size)     __svc_indirect(0);
extern OS_RESULT _os_mbx_send (U32 p, OS_ID mailbox, void *message_ptr,
                                      U16 timeout)                     __svc_indirect(0);
extern OS_RESULT _os_mbx_wait (U32 p, OS_ID mailbox, void  **message,
                                      U16 timeout)                     __svc_indirect(0);
extern OS_RESULT _os_mbx_check (U32 p, OS_ID mailbox)                  __svc_indirect(0);
extern void      isr_mbx_send (OS_ID mailbox, void *message_ptr);
extern OS_RESULT isr_mbx_receive (OS_ID mailbox, void **message);

 
extern void      rt_mut_init    (OS_ID mutex);
extern OS_RESULT rt_mut_release (OS_ID mutex);
extern OS_RESULT rt_mut_wait    (OS_ID mutex, U16 timeout);





extern void      _os_mut_init (U32 p, OS_ID mutex)                     __svc_indirect(0);
extern OS_RESULT _os_mut_release (U32 p, OS_ID mutex)                  __svc_indirect(0);
extern OS_RESULT _os_mut_wait (U32 p, OS_ID mutex, U16 timeout)        __svc_indirect(0);

 
extern void      rt_dly_wait (U16 delay_time);
extern void      rt_itv_set  (U16 interval_time);
extern void      rt_itv_wait (void);





extern void      _os_dly_wait (U32 p, U16 delay_time)                  __svc_indirect(0);
extern void      _os_itv_set (U32 p, U16 interval_time)                __svc_indirect(0);
extern void      _os_itv_wait (U32 p)                                  __svc_indirect(0);

 
extern OS_ID     rt_tmr_create (U16 tcnt, U16 info);
extern OS_ID     rt_tmr_kill   (OS_ID timer);




extern OS_ID     _os_tmr_create (U32 p, U16 tcnt, U16 info)            __svc_indirect(0);
extern OS_ID     _os_tmr_kill (U32 p, OS_ID timer)                     __svc_indirect(0);

 
extern void      rt_tsk_lock   (void);
extern void      rt_tsk_unlock (void);




extern void      _os_tsk_lock (U32 p)                                  __svc_indirect(0);
extern void      _os_tsk_unlock (U32 p)                                __svc_indirect(0);

 
extern int       _init_box (void *box_mem, U32 box_size, U32 blk_size);
extern void     *_alloc_box (void *box_mem);
extern void     *_calloc_box (void *box_mem);
extern int       _free_box (void *box_mem, void *box);








 




 

typedef struct {                         
  U8  hr;                                
  U8  min;                               
  U8  sec;                               
  U8  day;                               
  U8  mon;                               
  U16 year;                              
} RL_TIME;

typedef struct {                         
  S8  name[256];                         
  U32 size;                              
  U16 fileID;                            
  U8  attrib;                            
  RL_TIME time;                          
} FINFO;

extern int finit (void);
extern int fdelete (const char *filename);
extern int frename (const char *oldname, const char *newname);
extern int ffind (const char *pattern, FINFO *info);
extern U64 ffree (const char *drive);
extern int fformat (const char *drive);
extern int fanalyse (const char *drive);
extern int fcheck (const char *drive);
extern int fdefrag (const char *drive);

 




 

 



 






 
#line 415 "C:\\Keil\\ARM\\RV31\\INC\\rtl.h"

 
#line 428 "C:\\Keil\\ARM\\RV31\\INC\\rtl.h"

 





 
#line 442 "C:\\Keil\\ARM\\RV31\\INC\\rtl.h"

 




 



extern void init_TcpNet (void);
extern void main_TcpNet (void);
extern void timer_tick (void);
extern U8   udp_get_socket (U8 tos, U8 opt, 
                            U16 (*listener)(U8 socket, U8 *remip, U16 port, U8 *buf, U16 len));
extern BOOL udp_release_socket (U8 socket);
extern BOOL udp_open (U8 socket, U16 locport);
extern BOOL udp_close (U8 socket);
extern BOOL udp_mcast_ttl (U8 socket, U8 ttl);
extern U8  *udp_get_buf (U16 size);
extern BOOL udp_send (U8 socket, U8 *remip, U16 remport, U8 *buf, U16 dlen);
extern U8   tcp_get_socket (U8 type, U8 tos, U16 tout,
                            U16 (*listener)(U8 socket, U8 event, U8 *buf, U16 len));
extern BOOL tcp_release_socket (U8 socket);
extern BOOL tcp_listen (U8 socket, U16 locport);
extern BOOL tcp_connect (U8 socket, U8 *remip, U16 remport, U16 locport);
extern U8  *tcp_get_buf (U16 size);
extern U16  tcp_max_dsize (U8 socket);
extern BOOL tcp_check_send (U8 socket);
extern U8   tcp_get_state (U8 socket);
extern BOOL tcp_send (U8 socket, U8 *buf, U16 dlen);
extern BOOL tcp_close (U8 socket);
extern BOOL tcp_abort (U8 socket);
extern void tcp_reset_window (U8 socket);
extern BOOL arp_cache_ip (U8 *ipadr, U8 type);
extern void ppp_listen (char const *user, char const *passw);
extern void ppp_connect (char const *dialnum, char const *user, char const *passw);
extern void ppp_close (void);
extern BOOL ppp_is_up (void);
extern void slip_listen (void);
extern void slip_connect (char const *dialnum);
extern void slip_close (void);
extern BOOL slip_is_up (void);
extern U8   get_host_by_name (U8 *hostn, void (*cbfunc)(U8 event, U8 *host_ip));
extern BOOL smtp_connect (U8 *ipadr, U16 port, void (*cbfunc)(U8 event));
extern void dhcp_disable (void);
extern BOOL igmp_join (U8 *group_ip);
extern BOOL igmp_leave (U8 *group_ip);
extern BOOL snmp_trap (U8 *manager_ip, U8 gen_trap, U8 spec_trap, U16 *obj_list);
extern BOOL snmp_set_community (const char *community);






 
  

 
#line 4 "main.h"

































#line 43 "main.h"





#line 60 "main.h"

#line 72 "main.h"






#line 85 "main.h"

#line 97 "main.h"











#line 114 "main.h"







#line 161 "main.h"





#line 176 "main.h"













#line 207 "main.h"

#line 245 "main.h"

#line 265 "main.h"

#line 276 "main.h"








#line 291 "main.h"






#line 303 "main.h"







#line 319 "main.h"







#line 473 "main.h"








































#line 541 "main.h"







		










#line 574 "main.h"

#line 596 "main.h"

#line 612 "main.h"





















#line 649 "main.h"




#line 668 "main.h"









 


#line 689 "main.h"

#line 699 "main.h"

#line 708 "main.h"

#line 717 "main.h"

#line 729 "main.h"

#line 739 "main.h"

#line 749 "main.h"

#line 758 "main.h"

#line 766 "main.h"

#line 775 "main.h"

#line 787 "main.h"

#line 799 "main.h"

#line 815 "main.h"


extern unsigned char ver_soft;
extern unsigned short r_iz_plus, r_iz_minus, r_iz_porog_pred, r_iz_porog_error;
extern unsigned char asymmetry;						
extern unsigned short v_plus, v_minus, u_asymmetry, Ubus;	

extern unsigned int sk1_24;
extern unsigned short Iddt_porog_pred, Iddt_porog_error; 
extern unsigned char n_error_ddt_uku, u_rki;  
extern unsigned short Rddt[8][4];  
extern unsigned char count_Iddt; 
extern unsigned char count_mess_rki;  
extern unsigned char no_rki;  
extern unsigned char num_rki; 
extern unsigned char command_rki; 

extern unsigned char ddt_error;
extern unsigned short status_izm_r;	
extern unsigned int sk_alarm ; 
extern unsigned char type_rki; 
extern unsigned char asymmetry_porog;
extern unsigned short porog_u_in;
extern unsigned char uku_or_rki; 
extern unsigned char u_asymmetry_porog_up, u_asymmetry_porog, u_asymmetry_porog_down;
extern unsigned char kalibr_r_most;
extern unsigned char sk1_24_table[24], sk_alarm_table[24], ddt_error_table[8]; 

						

extern unsigned short net_in_u1_a, net_in_u1_b, net_in_u1_c, net_in_i1_a, net_in_i1_b, net_in_i1_c;
extern unsigned short net_in_p1_a, net_in_p1_b, net_in_p1_c, net_in_s1_a, net_in_s1_b, net_in_s1_c; 
extern unsigned short net_in_f1; 
extern unsigned short net_in_u2_a, net_in_u2_b, net_in_u2_c, net_in_i2_a, net_in_i2_b, net_in_i2_c;
extern unsigned short net_in_p2_a, net_in_p2_b, net_in_p2_c, net_in_s2_a, net_in_s2_b, net_in_s2_c; 
extern unsigned short net_in_f2;
extern unsigned char count_mess_net_in;  
extern unsigned char num_net_in; 
extern unsigned char no_net_in; 
extern unsigned char command_net_in;
extern unsigned char priority_net_in;
extern unsigned short u_min_net_in, u_max_net_in, i_min_net_in;
extern unsigned char hysteresis_net_in;
extern unsigned short t_inclusion_net_in, t_shutdown_net_in;


extern unsigned char enmv_modbus_adress[8], cnt_enmv_modbus_adress;


extern unsigned char LVBD_status, lvbd_num_alarm_status;

extern unsigned char no_lvbd; 
extern unsigned short lvbd_Uload, lvbd_Uakb;
extern unsigned char  ver_soft_lvbd;
extern unsigned short LVBD_porog_U1, LVBD_porog_U2, LVBD_Uload_rele_en, LVBD_Uakb_rele_en;
extern unsigned short LVBD_speed_rs485;
extern unsigned char LVBD_adress_rs485, LVBD_mode_rele_enable, LVBD_num_meas;
extern unsigned char count_mess_lvbd, command_lvbd;
extern unsigned short data_lvbd;




extern char b1000Hz,b100Hz,b50Hz,b10Hz,b5Hz,b2Hz,b1Hz;
extern short t0cnt,t0cnt0,t0cnt1,t0cnt2,t0cnt3,t0cnt4,t0cnt5,t0cnt6,t0_cnt7;
extern char bFL5,bFL2,bFL,bFL_;
extern signed short main_10Hz_cnt;
extern signed short main_1Hz_cnt;



extern char cnt_of_slave;







typedef enum {

	iMn_220_IPS_TERMOKOMPENSAT,
#line 919 "main.h"





	iMn,iMn_3U,iMn_RSTKM,




	iMn_220,


	iMn_KONTUR,


	iMn_6U,


	iMn_GLONASS,


	iMn_220_V2,


	iMn_TELECORE2015,


	iMn_TELECORE2017,


	iMn_IPS_SGEP_GAZPROM,


	iMn_FSO,

	iRKI, iSetRKI, iK_RKI,iK_MOST,
	iNET_IN, iSetNetIn, iK_Net_In,
	iLVBD, iSetLVBD, iK_LVBD, iSetENMV,
	iSrv_sl,iNet,iNet3,iNetEM,iNet3LIN,iNet_IPS_SGEP_GAZPROM,
	iSet,iSet_3U,iSet_RSTKM,iSet_GLONASS,iSet_KONTUR,iSet_6U,iSet_220,iSet_220_IPS_TERMOKOMPENSAT,iSet_220_V2,iInv_set_sel, iSet_FSO,
	iBat, iBat_simple, iBat_li, iBat_SacredSun, iBat_universe, iBat_FSO, iInv_set, iSet_TELECORE2015, iSet_TELECORE2017, iSet_IPS_SGEP_GAZPROM, iBat_ZVU,
	iMakb,
	iSet_prl_FSO_inf, iSet_FSO_inf, iFSO_inf,
	iBps,iBps_elteh,iS2,iSet_prl,iK_prl,iDnd,iPrlVZ1,iPrlVZ2,
	iK,iK_3U,iK_RSTKM,iK_GLONASS,iK_KONTUR,iK_6U,iK_220,iK_220_380,iK_220_IPS_TERMOKOMPENSAT,iK_220_IPS_TERMOKOMPENSAT_IB,iK_TELECORE,iK_IPS_SGEP_GAZPROM,
	iSpcprl,iSpc,k,Crash_0,Crash_1,iKednd,iAv_view_avt,iAKE,iSpc_termocompensat,
	iLoad,iSpc_prl_vz,iSpc_prl_ke,iKe,iVz,iAvz,iAVAR,
	iStr,iStr_3U,iStr_RSTKM,iStr_GLONASS,iStr_KONTUR,iStr_6U,iStr_220_IPS_TERMOKOMPENSAT,iStr_TELECORE2015,iStr_IPS_SGEP_GAZPROM,iStr_FSO,
	iVrs,iPrltst,iApv,iVZ_set,iVZ1_set,iVZ2_set,
	iK_bps,iK_bps_sel,iK_bat,iK_bat_simple,iK_bat_ips_termokompensat_ib,iK_bat_TELECORE,iK_bat_FSO,iK_bat_sel,iK_bat_sel_TELECORE,iK_bat_sel_FSO,iK_load,iK_net, iK_net3, iK_FSO,
	iK_makb_sel,iK_makb,iK_out,
	iTst,iTst_3U,iTst_RSTKM,iTst_GLONASS,iTst_KONTUR,iTst_6U,iTst_220,iTst_220_380,iTst_220_IPS_TERMOKOMPENSAT,iTst_FSO,
	iTst_TELECORE, iTst_IPS_SGEP_GAZPROM,
	iTst_klbr,iTst_BPS1,iTst_BPS2,iTst_BPS12,iDebug,
	iDef,iDef_3U,iDef_RSTKM,iDef_GLONASS,iDef_KONTUR,iDef_6U,iDef_220,iDef_220_IPS_TERMOKOMPENSAT,iDef_220_V2,
	iSet_st_prl,iK_pdp,iSet_T,iSet_T_avt,
	iDeb,iBat_link_set,iK_inv,iK_inv_sel,iK_byps,
	iPrl_bat_in_out,iPrl_bat_in_sel,iPdp1,iJAv_sel,iJAv_net_sel,iJAv_net,iJAv_src1,
	iTst_bps, iAusw,iAusw_prl,iAusw_set,
	iK_t_ext,iK_t_3U,iK_t_ext_6U, iPrl_Def_220_IPS_TERMOKOMPENSAT,
	iAv_view,
	iBatLogKe,iJ_bat_ke,iBatLogVz,iJ_bat_vz,iBatLogWrk,
	iExtern,iExtern_3U,iExtern_GLONASS,iExtern_KONTUR,iExtern_6U,iExtern_220,iExtern_220_ZVU,
	iK_power_net,
	iExt_set,iExt_set_3U,iExt_set_GLONASS,iExt_set_TELECORE2015,
	iExt_dt,
	iExt_sk,iExt_sk_3U,iExt_sk_GLONASS,
	iExt_ddv,iExt_ddi,iExt_dud,iExt_dp,iSM,iLog,iLog_, iLog_reset_prl, iBatLog,iKlimat,iKlimat_kontur,iKlimat_TELECORE,
	iEnerg3,iEnerg,
	iExtern_TELECORE2015, iExtern_FSO, iAvt_FSO,
	iVent,
	iK_power_net3,
	iAvt,iLan_set,
	iInv,iInv_v2,
	iNpn_set,
	iByps,iInv_tabl,iSet_bat_sel,
	iBps_list,
	iSpch_set,
	iAvt_set_sel,iAvt_set,iAvt_comm_set,iSet_li_bat,
	iOut_volt_contr,iDop_rele_set,iBlok_ips_set,iIps_Curr_Avg_Set,
	iRele_set,iRele_set_,
	iRele_set_6U,
	iFWabout, iSpecInf,	iSpecInfFSO,
	iHV_STEP2_1,iHV_STEP2_2,iVZ1_STEP2_1,iVZ1_STEP2_2,iVZ2_STEP2_1,iVZ2_STEP2_2,
	iSet_load_off,
	iSet_bat_point,
	iSet_FSO_MINI_SIGN
	}i_enum;

typedef struct  
{

i_enum i;
signed char s_i;
signed char s_i1;
signed char s_i2;
signed char i_s;
} stuct_ind;







extern stuct_ind a_ind,b_ind[10],c_ind;
extern signed short ptr_ind;
extern char lcd_buffer[200+100];
extern signed char parol[3];
extern char phase;
extern char lcd_bitmap[1024];
extern char dig[5];
extern signed short ind_pointer;
extern char zero_on;
extern char mnemo_cnt;
extern char simax;
extern short av_j_si_max;
extern const char ABCDEF[];
extern const char sm_mont[13][4]; 
extern char *show_mess_p1,*show_mess_p2,*show_mess_p3,*show_mess_p4;
extern char show_mess_cnt;
extern short show_mess_number_;
extern char show_mess_komma;




extern signed short Ktsrc[2];
extern signed short Kusrc[2];
extern signed short Kisrc[2];
extern signed short Ki0src[2];
extern signed short Kubat[2];
extern signed short Kubatm[2];
extern unsigned short Kibat0[2];
extern signed short Kibat1[2];
extern signed short Ktbat[2];
extern signed short Kunet;
extern signed short Ktext[3];
extern signed short Kuload;
extern signed short Kunet_ext[3];
extern signed short KunetA;
extern signed short KunetB;
extern signed short KunetC;
extern signed short Kubps;
extern signed short Kuout;

extern signed short MAIN_IST;
extern signed short UMAX;
extern signed short UB0;
extern signed short UB20;
extern signed short TMAX;
extern signed short TSIGN;
extern signed short AV_OFF_AVT;
extern signed short USIGN;
extern signed short UMN;
extern signed short UMAXN;
extern signed short ZV_ON;
extern signed short IKB;
extern signed short UVZ;
extern signed short IMAX_VZ;
extern signed short IMAX;
extern signed short IMIN;
extern signed short APV_ON;
extern signed short IZMAX;
extern signed short U0B;
extern signed short TZAS;
extern signed short VZ_HR;
extern signed short TBAT;
extern signed short U_AVT;
extern signed short DU;
extern signed short PAR;
extern signed short TBATMAX;
extern signed short TBATSIGN;
extern signed short UBM_AV;
extern signed short RELE_LOG;
extern signed short TBOXMAX;
extern signed short TBOXREG;
extern signed short TBOXVENTMAX;
extern signed short TLOADDISABLE;
extern signed short TLOADENABLE;
extern signed short TBATDISABLE;
extern signed short TBATENABLE;
extern signed short TBOXMAX;
extern signed short TBOXREG;
extern signed short TBOXVENTMAX;
extern signed short TLOADDISABLE;
extern signed short TLOADENABLE;
extern signed short TBATDISABLE;
extern signed short TBATENABLE;
extern signed short TVENTON;
extern signed short TVENTOFF;
extern signed short TWARMON;
extern signed short TWARMOFF;
typedef enum {rvsAKB=0,rvsEXT=1,rvsBPS=2} enum_releventsign;	
extern enum_releventsign RELEVENTSIGN;


extern signed short TZNPN;
extern signed short UONPN;
extern signed short UVNPN;
extern signed short dUNPN;
typedef enum {npnoOFF=0,npnoRELEVENT=1,npnoRELEAVBAT2=2, npnoBDR=3} enum_npn_out;  
extern enum_npn_out NPN_OUT;
typedef enum {npnsULOAD=0,npnsAVNET} enum_npn_sign;
extern enum_npn_sign NPN_SIGN;
extern signed short TERMOKOMPENS;
extern signed short TBOXVENTON; 
extern signed short TBOXVENTOFF;
extern signed short TBOXWARMON; 
extern signed short TBOXWARMOFF;
extern signed short BAT_TYPE;	
extern signed short DU_LI_BAT;	
extern signed short FORVARDBPSCHHOUR;	
extern signed short NUMBAT;
extern signed short NUMBAT_TELECORE;
extern signed short NUMIST;
extern signed short NUMINV;
extern signed short NUMDT;
extern signed short NUMSK;
extern signed short NUMEXT;
extern signed short NUMAVT;
extern signed short NUMMAKB;
extern signed short NUMBYPASS;
extern signed short NUMBDR;
extern signed short NUMENMV;
extern signed short NUMLVBD; 
extern signed short NUMPHASE;  
extern signed short SMART_SPC;
extern signed short U_OUT_KONTR_MAX;
extern signed short U_OUT_KONTR_MIN;
extern signed short U_OUT_KONTR_DELAY;
extern signed short DOP_RELE_FUNC;
extern signed short CNTRL_HNDL_TIME;	
extern signed short USODERG_LI_BAT;		
extern signed short QSODERG_LI_BAT;		
extern signed short TVENTMAX;			
extern signed short ICA_EN;				
extern signed short ICA_CH;				
extern signed short ICA_MODBUS_ADDRESS;
extern signed short ICA_MODBUS_TCP_IP1,ICA_MODBUS_TCP_IP2,ICA_MODBUS_TCP_IP3,ICA_MODBUS_TCP_IP4;	
extern signed short ICA_MODBUS_TCP_UNIT_ID;	
extern signed short PWM_START;			
extern signed short KB_ALGORITM;		
extern signed short REG_SPEED;			

typedef enum {apvON=0x01,apvOFF=0x00}enum_apv_on;
extern enum_apv_on APV_ON1,APV_ON2;

extern signed short APV_ON2_TIME;
extern signed short RS485_QWARZ_DIGIT;
extern signed short UVENTOFF;			
extern signed short VZ_KIND;			
										
extern signed short SNTP_ENABLE;
extern signed short SNTP_GMT;

extern signed short NUMBAT_FSO;
#line 1190 "main.h"

extern signed short UZ_U;
extern signed short UZ_IMAX;
extern signed short UZ_T;

extern signed short FZ_U1;
extern signed short FZ_IMAX1;
extern signed short FZ_T1;
extern signed short FZ_ISW12;
extern signed short FZ_U2;
extern signed short FZ_IMAX2;
extern signed short FZ_T2;

extern signed short RELE_SET_MASK[4];

typedef enum {bisON=0x0055,bisOFF=0x00aa}enum_bat_is_on;
extern enum_bat_is_on BAT_IS_ON[2];

extern signed short BAT_DAY_OF_ON[2];
extern signed short BAT_MONTH_OF_ON[2];
extern signed short BAT_YEAR_OF_ON[2];
extern signed short BAT_C_NOM[2];
extern signed short BAT_RESURS[2];
extern signed short BAT_C_REAL[2];


extern unsigned short AUSW_MAIN;
extern unsigned long AUSW_MAIN_NUMBER;
extern unsigned short AUSW_DAY;
extern unsigned short AUSW_MONTH;
extern unsigned short AUSW_YEAR;
extern unsigned short AUSW_UKU;
extern unsigned short AUSW_UKU_SUB;
extern unsigned long AUSW_UKU_NUMBER;
extern unsigned long	AUSW_BPS1_NUMBER;
extern unsigned long  AUSW_BPS2_NUMBER;
extern unsigned short AUSW_RS232;
extern unsigned short AUSW_PDH;
extern unsigned short AUSW_SDH;
extern unsigned short AUSW_ETH;

extern signed short TMAX_EXT_EN[3];
extern signed short TMAX_EXT[3];
extern signed short TMIN_EXT_EN[3];
extern signed short TMIN_EXT[3];
extern signed short T_EXT_REL_EN[3];
extern signed short T_EXT_ZVUK_EN[3];
extern signed short T_EXT_LCD_EN[3];
extern signed short T_EXT_RS_EN[3];

extern signed short SK_SIGN[4];
extern signed short SK_REL_EN[4];
extern signed short SK_ZVUK_EN[4];
extern signed short SK_LCD_EN[4];
extern signed short SK_RS_EN[4];

typedef enum {AVZ_1=1,AVZ_2=2,AVZ_3=3,AVZ_6=6,AVZ_12=12,AVZ_OFF=0}enum_avz;
extern enum_avz AVZ;

extern unsigned short HOUR_AVZ;
extern unsigned short MIN_AVZ;
extern unsigned short SEC_AVZ;
extern unsigned short DATE_AVZ;
extern unsigned short MONTH_AVZ;
extern unsigned short YEAR_AVZ;
extern unsigned short AVZ_TIME;
typedef enum {mnON=0x55,mnOFF=0xAA}enum_mnemo_on;
extern enum_mnemo_on MNEMO_ON;
extern unsigned short MNEMO_TIME;
extern signed short POWER_CNT_ADRESS;

extern signed short ETH_IS_ON;
extern signed short ETH_DHCP_ON;
extern signed short ETH_IP_1;
extern signed short ETH_IP_2;
extern signed short ETH_IP_3;
extern signed short ETH_IP_4;
extern signed short ETH_MASK_1;
extern signed short ETH_MASK_2;
extern signed short ETH_MASK_3;
extern signed short ETH_MASK_4;
extern signed short ETH_TRAP1_IP_1;
extern signed short ETH_TRAP1_IP_2;
extern signed short ETH_TRAP1_IP_3;
extern signed short ETH_TRAP1_IP_4;
extern signed short ETH_TRAP2_IP_1;
extern signed short ETH_TRAP2_IP_2;
extern signed short ETH_TRAP2_IP_3;
extern signed short ETH_TRAP2_IP_4;
extern signed short ETH_TRAP3_IP_1;
extern signed short ETH_TRAP3_IP_2;
extern signed short ETH_TRAP3_IP_3;
extern signed short ETH_TRAP3_IP_4;
extern signed short ETH_TRAP4_IP_1;
extern signed short ETH_TRAP4_IP_2;
extern signed short ETH_TRAP4_IP_3;
extern signed short ETH_TRAP4_IP_4;
extern signed short ETH_TRAP5_IP_1;
extern signed short ETH_TRAP5_IP_2;
extern signed short ETH_TRAP5_IP_3;
extern signed short ETH_TRAP5_IP_4;
extern signed short ETH_SNMP_PORT_READ;
extern signed short ETH_SNMP_PORT_WRITE;
extern signed short ETH_GW_1;
extern signed short ETH_GW_2;
extern signed short ETH_GW_3;
extern signed short ETH_GW_4;

extern signed short RELE_VENT_LOGIC;

extern signed short MODBUS_ADRESS;
extern signed short MODBUS_BAUDRATE;
extern signed short BAT_LINK;


extern signed short BAT_C_POINT_1_6;  	
extern signed short BAT_C_POINT_1_2;  	
extern signed short BAT_C_POINT_1;		
extern signed short BAT_C_POINT_3;		
extern signed short BAT_C_POINT_5;		
extern signed short BAT_C_POINT_10;		
extern signed short BAT_C_POINT_20;		
extern signed short BAT_U_END_1_6;  	
extern signed short BAT_U_END_1_2;  	
extern signed short BAT_U_END_1;  		
extern signed short BAT_U_END_3;  		
extern signed short BAT_U_END_5;  		
extern signed short BAT_U_END_10;  		
extern signed short BAT_U_END_20;  		
extern signed short BAT_C_POINT_NUM_ELEM;	
extern signed short BAT_K_OLD;			


#line 1354 "main.h"

extern signed short SP_CH_VENT_BLOK;
extern signed short VZ_CH_VENT_BLOK;



typedef struct
     {
	char 		_cnt_to_block;
	signed short	_Ub;
     signed short	_Ubm;
     signed short	_dUbm;
	signed short	_Ib;
	signed short	_Ib_;
	signed short	_Tb;
	char 		_nd;
	char 		_cnt_wrk;
	char 		_wrk;
	unsigned short _zar;
	char 		_full_ver;
	signed long 	_zar_cnt;
	signed long 	_zar_cnt_ke;
	unsigned short _Iintegr,_Iintegr_; 
	signed short 	_u_old[8];
	signed short	_u_old_cnt;
	unsigned long 	_wrk_date[2];
	char 		_rel_stat;
	char			_av;
	char			_time_cnt;
	char 		_temper_stat;
	
	
	
	
	signed short 	_sign_temper_cnt;
	signed short 	_max_temper_cnt;
	signed long 	_resurs_cnt;
	signed short 	_cnt_as; 	
     
	
	unsigned short _time_min_cnt_ke;
	} BAT_STAT; 
extern BAT_STAT bat[2],bat_ips;
extern signed short		bat_u_old_cnt;
extern signed short 	Ib_ips_termokompensat;
extern signed short		Ib_ips_termokompensat_temp;


typedef enum {bsOFF=0,bsCOMM_ON,bsOK} enum_batStat;


typedef struct
     {
	
	signed short	_Ub;
     
     
	signed short	_Ib;
	signed short	_Tb;
	char 		_nd;
	char   		_soh;
	char 		_soc;
	signed short   _ratCap;
	char 		_comErrStat;	
	enum_batStat	_batStat;
	signed short 	_cclv;
	char 		_rbt;
	short 		_canErrorCnt;
	char			_canError;
	char 		_485Error;
	short 		_485ErrorCnt;
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
     
	
	} LI_BAT_STAT; 
extern LI_BAT_STAT li_bat;



typedef struct
     {
	signed short 	_Iout;
	signed short 	_Uout;
	signed short 	_Pout;
	signed short 	_Unet; 	
	signed short 	_Uin;
	char			_T;
	char 		_flags;
	char			_cnt;
	char 		_adress;
	} BYPS_STAT; 
extern BYPS_STAT byps;



typedef struct
     {
	signed short	_U[5];
	signed short	_Ub[5];
	signed short	_T[5];
	signed short	_T_nd[5];
	signed short 	_cnt; 	
	} MAKB_STAT; 
extern MAKB_STAT makb[4];



typedef struct
     {
	signed short	_max_cell_volt;
	signed short	_min_cell_volt;
	signed short	_max_cell_temp;
	signed short	_min_cell_temp;
	signed short	_tot_bat_volt;
	signed short	_ch_curr;
	signed short	_dsch_curr;
	signed short	_rat_cap;
	signed short	_s_o_h;
	signed short	_s_o_c;
	signed short	_c_c_l_v;
	signed short	_r_b_t;
	signed short	_b_p_ser_num;
	signed short   _flags1;
	signed short 	_flags2;
	signed short 	_communication2lvlErrorStat; 	
	signed short	_communication2lvlErrorCnt;  	
	signed short 	_cnt;
	signed short 	_communicationFullErrorStat;	
	signed short   _battIsOn;		
	char 		_plazma[8];		
	signed short 	_isOnCnt;
	signed short	_s_o_c_abs;		
	signed short 	_s_o_c_percent; 
	signed short	_plazma_ss;
	signed short	_zar_percent;	
	signed char		_cell_temp_1;	
	signed char		_cell_temp_2;	
	signed char		_cell_temp_3;	
	signed char		_cell_temp_4;	
	signed char		_cell_temp_ambient;	
	signed char		_cell_temp_power;	
	
	
	
	signed char		_charge_and_discharge_current_alarm_status;	 	
	signed char 	_battery_total_voltage_alarm_status;			
	signed char		_custom_alarm_quantity;							
	signed char		_balanced_event_code;							
	signed char 	_voltage_event_code;							
	signed char 	_temperature_event_code;						
	signed char		_current_event_code;							
	signed char		_fet_status_code;								
	signed short	_balanced_status_code;							
	signed char 	_system_status_code;							
	signed short 	_succes_transmission_cnt;
	signed short 	_no_transmission_second_cnt;
	} LAKB_STAT; 
extern LAKB_STAT lakb[3];
extern char lakb_damp[1][42];
extern char bLAKB_KONF_CH;
extern char bLAKB_KONF_CH_old;
extern char lakb_ison_mass[7];
extern short lakb_mn_ind_cnt;
extern char bLAKB_KONF_CH_EN;
extern char bRS485ERR;
extern short LBAT_STRUKT;
extern char lakb_error_cnt;	
extern short numOfPacks, numOfPacks_, post_length_;
extern short numOfCells, numOfTemperCells, baseOfData;
extern short lakb_stat_comm_error;	
extern short lakbNotErrorNum;		
extern short libat_comm_cnt;		






extern char can_slot[12][16];
extern char plazma_can_inv[3];





typedef struct
    {
    enum {dSRC=3,dINV=5,dNET_METR=7,dIBAT_METR=9,dMAKB=11}_device;
	char _av;
	
	
	
	
	
 	enum {bsAPV,bsWRK,bsRDY,bsBL,bsAV,bsOFF_AV_NET}_state;
    char _cnt;
     char _cnt_old;
     char _cnt_more2;
     char _buff[20]; 
     
     
     
     
     signed _Uii; 
     signed _Uin;
     signed _Ii;
     signed _Ti; 
     char _flags_tu;
     
     
     
     
     
     signed _vol_u;
     signed _vol_i;
     char _is_on_cnt;
     
     int _ist_blok_host_cnt;
     short _blok_cnt; 
     char _flags_tm;
	signed short _overload_av_cnt;     
     signed short _temp_av_cnt;
     signed short _umax_av_cnt;
     signed short _umin_av_cnt;		
	 signed short _umin_av_cnt_uku;	
     signed _rotor;
     signed  short _x_; 
     char _adr_ee;
	char _last_avar;
	char _vent_resurs_temp[4];
	unsigned short _vent_resurs;
	unsigned char _apv_timer_1_lev;		
	unsigned char _apv_cnt_1_lev;		
	unsigned short _apv_timer_2_lev;	
	unsigned char _apv_reset_av_timer;	
	unsigned char _apv_succes_timer;	
	} BPS_STAT; 
extern BPS_STAT bps[35];



typedef struct
     {
	char _av;
	
	
	
	
     enum {isAPV,isWRK,isRDY,isBL,isAV,isOFF_AV_NET}_state;
     char _cnt;
     char _cnt_old;
     char _cnt_more2;
     char _buff[16]; 
     signed _Uio; 
     signed _Ii;
     signed _Ti; 
	signed _Uil;
	signed _Uin;
	signed _Pio;
     char _flags_tu;
     signed _vol_u;
     signed _vol_i;
     char _is_on_cnt;
     
     int _ist_blok_host_cnt;
     short _blok_cnt; 
     char _flags_tm;
	char _flags_tm_old;
	signed short _overload_av_cnt;     
     signed short _temp_av_cnt;
     signed short _umax_av_cnt;
     signed short _umin_av_cnt;
     signed _rotor;
     signed  short _x_; 
     char _adr_ee;
	char _last_avar;
	char _Pnom;
	char _Uoutmin;
	char _Uoutmax;
	char _net_contr_en;
	char _pwm_en;
	char _phase_mode;

     } INV_STAT; 




extern INV_STAT inv[20];

extern char first_inv_slot;



extern signed short load_U;
extern signed short load_I;



extern signed short bps_U;
extern signed short out_U;
extern signed short bps_I;
extern signed short bps_I_phantom;



extern signed short net_U,net_Ustore,net_Ua,net_Ub,net_Uc, net_Umax, net_Ustore_max; 
extern char bFF,bFF_;
extern signed short net_F,hz_out,hz_out_cnt,net_F3;
extern signed char unet_drv_cnt;	 
extern signed char unet_max_drv_cnt; 
extern char net_av;
extern short net_av_2min_timer;

extern char plazma_plazma_plazma;

void bitmap_hndl(void);
void ind_hndl(void);
__irq void timer1_interrupt(void);
__irq void timer0_interrupt(void); 





extern char tout_max_cnt[4],tout_min_cnt[4];
typedef enum {tNORM,tMAX,tMIN}enum_tout_stat;
extern enum_tout_stat tout_stat[4];
extern signed short t_ext[3];
extern char ND_EXT[3];
extern signed char sk_cnt[4],sk_av_cnt[4];
typedef enum  {ssOFF,ssON} enum_sk_stat;
extern enum_sk_stat sk_stat[4],sk_stat_old[4];
typedef enum  {sasOFF,sasON} enum_sk_av_stat;
extern enum_sk_av_stat sk_av_stat[4],sk_av_stat_old[4];
extern signed short t_box,t_box_warm,t_box_vent;
extern char TELECORE2017_EXT_VENT_PWM,TELECORE2017_INT_VENT_PWM;



extern BOOL bSILENT;



typedef enum {tstOFF,tst1,tst2} enum_tst_state;
extern enum_tst_state tst_state[15];



extern char sign_U[2],sign_I[2];
extern char superviser_cnt;



extern unsigned short adc_buff_ext_[3];
extern unsigned short Uvv[3];
extern unsigned short Uvv0;
extern short pos_vent;
extern short t_ext_can;
extern char t_ext_can_nd;



extern char eb2_data[30];
extern short eb2_data_short[10];
extern short Uvv_eb2[3],Upes_eb2[3];
extern short Kvv_eb2[3],Kpes_eb2[3];



extern signed short vvod_pos;



extern signed long power_summary;
extern signed short power_current;
extern signed long power_summary_tempo,power_summary_tempo_old;
extern signed short power_current_tempo,power_current_tempo_old;
extern char powerSummaryCnt;
extern char powerCurrentCnt;


extern char bRESET;
extern char bRESET_EXT;
extern char bRESET_INT_WDT;
extern char bRESET_EXT_WDT;



extern signed short main_vent_pos;
extern signed char t_box_cnt;
typedef enum  {mvsOFF,mvsON} enum_mixer_vent_stat;
extern enum_mixer_vent_stat mixer_vent_stat;
typedef struct
     {
	signed short _T[4];
	char _nd[4];
	signed short _T_dispers[4];
	char _max_dispers_num;
	signed short _max_dispers;
    	signed short _avg1;
	signed short _avg2;
	char _avg_cnt;
     } INT_BOX_TEMPER;
extern INT_BOX_TEMPER ibt;
typedef enum {tbdsON,tbdsOFF,tbdsMNL} enum_tbatdisable_stat;
extern enum_tbatdisable_stat tbatdisable_stat;
typedef enum {tldsON,tldsOFF,tldsMNL} enum_tloaddisable_stat;
extern enum_tloaddisable_stat tloaddisable_stat;
typedef enum {atsOFF,atsON} enum_av_tbox_stat;
extern enum_av_tbox_stat av_tbox_stat;
extern signed short av_tbox_cnt;
extern char tbatdisable_cmnd,tloaddisable_cmnd;
extern short tbatdisable_cnt,tloaddisable_cnt;
#line 1792 "main.h"

#line 1803 "main.h"

#line 1819 "main.h"

extern char ext_can_cnt;


signed short abs_pal(signed short in);
void ADC_IRQHandler(void);




typedef enum  {avtOFF,avtON} enum_avt_stat;
extern enum_avt_stat avt_stat[12],avt_stat_old[12]; 



extern signed long ibat_metr_buff_[2];
extern short bIBAT_SMKLBR;
extern short bIBAT_SMKLBR_cnt;
extern short ibat_metr_cnt;



extern signed short npn_tz_cnt;
typedef enum {npnsOFF=0,npnsON} enum_npn_stat;
extern enum_npn_stat npn_stat,load_off_stat;
extern signed short load_off_cnt;

extern char snmp_plazma;


extern char ips_bat_av_vzvod;
extern char ips_bat_av_stat;

extern char rel_warm_plazma;
extern char can_byps_plazma0,can_byps_plazma1;

extern short plazma_bat_drv0,plazma_bat_drv1,bat_drv_cnt_cnt;
extern unsigned short bat_drv_rx_cnt;
extern char bat_drv_rx_buff[512];
extern char bat_drv_rx_in;

extern short can_plazma;
extern short modbus_modbus_adress_eq;
extern short modbus_modbus4f_cnt;



#line 1877 "main.h"



#line 1901 "main.h"




extern signed short TELECORE2017_USTART;		
extern signed short TELECORE2017_ULINECC;		
extern signed short TELECORE2017_ULINECC_;		
extern signed short TELECORE2017_AVAR_CNT;				
extern signed short TELECORE2017_Q;				
extern signed short TELECORE2017_IZMAX1;		
extern signed short TELECORE2017_IZMAX2;		
extern signed short TELECORE2017_K1;			
extern signed short TELECORE2017_K2;			
extern signed short TELECORE2017_K3;			
extern signed short TELECORE2017_T4;			














extern signed short speedChrgCurr;			
extern signed short speedChrgVolt;			
extern signed short speedChrgTimeInHour; 	
extern signed short speedChrgAvtEn;	 	
extern signed short speedChrgDU;	    		
extern signed short speedChIsOn;			
extern signed long  speedChTimeCnt;		
extern signed short speedChrgBlckSrc;		
extern signed short speedChrgBlckLog;		
extern signed short speedChrgBlckStat;		
extern char  		speedChrgShowCnt;		



typedef enum  {scsOFF,scsSTEP1,scsWRK,scsERR1,scsERR2} enum_sp_ch_stat;
extern enum_sp_ch_stat sp_ch_stat,sp_ch_stat_old;
extern short sp_ch_stat_cnt;
extern long sp_ch_wrk_cnt;
extern char speedChargeStartCnt;



extern signed short ipsBlckSrc;
extern signed short ipsBlckLog;
extern signed short ipsBlckStat;



extern signed short outVoltContrHndlCnt;		
extern signed short outVoltContrHndlCnt_;		
extern char uout_av;



extern short apsEnergiaCnt;
extern char apsEnergiaStat; 

extern short plazma_numOfCells;
extern short plazma_numOfTemperCells;
extern short plazma_numOfPacks;

extern char plazma_ztt[2];
extern char plazma_stark[32];
extern char spch_plazma[2];

extern U8 socket_tcp;



extern char ica_plazma[10];
extern char ica_timer_cnt;
extern signed short ica_my_current;
extern signed short ica_your_current;
extern signed short ica_u_necc;
extern signed short ica_cntrl_hndl;
extern signed short ica_cntrl_hndl_cnt;
extern U8 tcp_soc_avg;
extern U8 tcp_connect_stat;



typedef enum {hvsOFF,hvsSTEP1,hvsSTEP2,hvsSTEP3,hvsSTEP4,hvsWRK,hvsERR1,hvsERR2,hvsERR3,hvsERR4} enum_hv_vz_stat;
extern enum_hv_vz_stat hv_vz_stat,hv_vz_stat_old;
extern short hv_vz_stat_cnt;
extern long hv_vz_wrk_cnt;
extern long hv_vz_up_cnt;



extern char bdr_transmit_stat;
extern char bdr_avar_stat;



typedef enum {uassOFF,uassSTEP1,uassSTEP2,uassSTEP3,uassSTEP4} enum_uavt_set_stat;
extern enum_uavt_set_stat uavt_set_stat, uavt_set_stat_old;
typedef enum {uasrsGOOD,uasrsWRK,uasrsERR,uasrsSUCCESS} enum_uavt_set_result_stat;
extern enum_uavt_set_result_stat uavt_set_result_stat;
extern short u_max_temp,u_min_temp;
extern char  uavt_bps_pntr;
extern char  uavt_error_bps;
extern char avt_plazma;
extern char avt_error_bps;
extern char uavt_set_error_cnt;

extern short pvlk;
extern char web_plazma[5];
extern short web_cnt_main;
extern short web_cnt_2hz;
extern const char* web_str;
extern char uku_set_autorized;
extern long web_param_input;
extern short cntrl_stat_pwm;

extern char place_holder[70];

extern unsigned char count_reg_enmv, count_bit_enmv, enmv_puts_en, delay_enmv_puts; 














 
#line 7 "modbus.c"
#line 1 "control.h"







extern char num_of_wrks_bps;
extern char bps_all_off_cnt,bps_mask_off_cnt,bps_mask_on_off_cnt;
extern char bps_hndl_2sec_cnt;
extern unsigned bps_on_mask,bps_off_mask;
extern char num_necc_up,num_necc_down;
extern unsigned char sh_cnt0,b1Hz_sh;


extern short cntrl_stat_blok_cnt,cntrl_stat_blok_cnt_,cntrl_stat_blok_cnt_plus[2],cntrl_stat_blok_cnt_minus[2];



extern long adc_buff[16][16];
extern signed short adc_buff_max[12],adc_buff_min[12],unet_buff_max,unet_buff_min;
extern short adc_buff_[16];
extern char adc_self_ch_cnt,adc_ch_net;
extern char adc_cnt,adc_cnt1,adc_ch,adc_ch_cnt;
extern short zero_cnt;
typedef enum {asCH=1,asNET_WAIT=2,asNET_RDY=3,asNET=4} enum_adc_stat;
extern enum_adc_stat adc_stat;
extern unsigned short net_buff[32],net_buff_,net_metr_buff_[3];
extern char net_buff_cnt;
extern short ADWR,period_cnt,non_zero_cnt;
extern char rele_stat;
extern char bRELE_OUT;
extern short plazma_adc_cnt;
extern signed short adc_self_ch_buff[3],adc_self_ch_disp[3];
extern long main_power_buffer[8],main_power_buffer_;
extern short main_power_buffer_cnt;
extern short adc_gorb_cnt,adc_zero_cnt;
extern char adc_window_flag;
extern short adc_window_cnt;
extern short adc_net_buff_cnt;


char vz_start(char hour);
void vz_stop(void);
void vz_drv(void);
void samokalibr_init(void);
void samokalibr_hndl(void);
void kb_init(void);
void kb_hndl(void);
void ubat_old_drv(void);
void unet_drv(void);
void matemat(void);
void adc_init(void);
void adc_drv5(void);
void adc_drv_(void);
void avg_hndl(void);


void rele_hndl(void);
void bps_hndl(void);
void bps_drv(char in);
void bat_hndl(void);
void bat_drv(char in);
void u_necc_hndl(void);
void cntrl_hndl(void);
void zar_drv(void);
void num_necc_hndl(void);
void ke_start(char in);
void ke_drv(void);
void avz_drv(void);
void zar_drv(void);
void vent_hndl(void);
void avz_next_date_hndl(void);
void klimat_hndl(void);
void ext_drv(void);
void adc_drv7(void);
void avt_hndl(void);
void vent_resurs_hndl(void);
void ips_current_average_hndl(void);




typedef enum {spcOFF=0,spcKE, spcVZ}enum_spc_stat;
typedef enum {kssNOT=0,kssNOT_VZ,kssYES=100,kssNOT_BAT,kssNOT_BAT_AV,kssNOT_BAT_AV_T,kssNOT_BAT_AV_ASS,kssNOT_BAT_ZAR,kssNOT_BAT_RAZR,kssNOT_KE1,kssNOT_KE2}enum_ke_start_stat;
extern enum_spc_stat spc_stat;
extern enum_ke_start_stat ke_start_stat;
extern char spc_bat;
extern char spc_phase;
extern unsigned short vz_cnt_s,vz_cnt_s_,vz_cnt_h,vz_cnt_h_;
extern short cnt_end_ke;
extern unsigned long ke_date[2];
extern short __ee_vz_cnt;
extern short __ee_spc_stat;
extern short __ee_spc_bat;
extern short __ee_spc_phase;
extern char vz_error;   



extern signed char vent_stat;



extern signed short cntrl_stat;
extern signed short cntrl_stat_old;
extern signed short cntrl_stat_new;
extern signed short Ibmax, Ibmax_;
extern unsigned char unh_cnt0,unh_cnt1,b1Hz_unh;
extern unsigned char	ch_cnt0,b1Hz_ch,i,iiii;
extern unsigned char	ch_cnt1,b1_30Hz_ch;
extern unsigned char	ch_cnt2,b1_10Hz_ch;
extern unsigned short IZMAX_;
extern unsigned short IZMAX_70;
extern unsigned short IZMAX_130;
extern unsigned short Ubpsmax;
extern unsigned short cntrl_stat_blck_cnt;

extern short plazma_sk;
extern char	plazma_inv[4];
extern char plazma_bat;
extern char plazma_cntrl_stat;


extern signed int i_avg_max,i_avg_min,i_avg_summ,i_avg; 
extern signed int avg;
extern char bAVG;
extern const char sk_buff_TELECORE2015[4];



extern signed short 	main_kb_cnt;
extern signed short 	kb_cnt_1lev;
extern signed short 	kb_cnt_2lev;
extern char 		kb_full_ver;
extern char kb_start[2],kb_start_ips;
extern signed short ibat_ips,ibat_ips_;



extern char numOfForvardBps,numOfForvardBps_old;
extern char numOfForvardBps_minCnt;
extern short numOfForvardBps_hourCnt;



extern char bPARALLEL_NOT_ENOUG;
extern char bPARALLEL_ENOUG;
extern char bPARALLEL;



extern char rx_read_power_cnt_phase;
extern short read_power_cnt_main_cnt;
extern short ce102m_delayCnt;
extern char rx_read_power_cnt_plazma;
extern char rx_read_power_cnt_flag;
extern short volta_short;
extern short curr_short;
extern int power_int;

extern char bENERGOMETR_UIP;



typedef enum {vz1sOFF=0, vz1sSTEP1=1, vz1sSTEP2=2, vz1sSTEP3=3, vz1sWRK=10, vz1sERR1, vz1sERR2, vz1sERR3, vz1sERR4, vz1sFINE, vz1sSTOP}enum_vz1_stat;
extern enum_vz1_stat vz1_stat, vz1_stat_old;
extern short vz1_stat_cnt;
extern long vz1_wrk_cnt;
extern long vz1_up_cnt;
extern char volt_region;



typedef enum {vz2sOFF=0, vz2sSTEP1=1, vz2sSTEP2=2, vz2sSTEP3=3, vz2sWRK1=10, vz2sWRK2=11, vz2sERR1, vz2sERR2, vz2sERR3, vz2sERR4, vz2sERR5, vz2sERR6, vz2sFINE, vz2sSTOP}enum_vz2_stat;
extern enum_vz2_stat vz2_stat, vz2_stat_old;
extern short vz2_stat_cnt;
extern long vz2_wrk_cnt;
extern long vz2_up_cnt;
extern signed short vz2_stat_ph2_cnt;



extern short I_from_t_table[7];
extern char bat_hndl_zvu_init;
extern short bat_hndl_i;
extern long bat_hndl_t_razr;				
extern long bat_hndl_t_razr_ke;				
extern long bat_hndl_zvu_Q;
extern long bat_hndl_proc_razr;
extern long bat_hndl_remain_time;
extern short bat_hndl_t_razr_hour;
extern short bat_hndl_t_razr_min;
extern short bat_hndl_t_razr_mininhour;
extern char bat_hndl_zvu_ke_init;
extern short bat_hndl_i_temp;
extern short bat_hndl_u_end;
extern short U_end_from_i_table[7];
extern long bat_hndl_plazma[5];
extern char bat_hndl_zvu_Q_cnt;
extern char bat_hndl_i_vector,bat_hndl_i_vector_old;
extern long bat_hndl_i_zar_price;
extern long bat_hndl_i_summ;

extern char avar_bps_reset_cnt;

extern char cntrl_hndl_plazma;

extern short plazma_ica1,plazma_ica2;
extern char rele_hndl_plazma[3];

extern short spirit_wrk_cnt;

void zar_superviser_drv(void);
void zar_superviser_start(void);
void vent_hndl(void);
void speedChargeHndl(void);
void speedChargeStartStop(void);
void numOfForvardBps_init(void);
void outVoltContrHndl(void);


#line 8 "modbus.c"
#line 1 "C:\\Keil\\ARM\\RV31\\INC\\string.h"
 
 
 
 




 








 











#line 37 "C:\\Keil\\ARM\\RV31\\INC\\string.h"


  
  typedef unsigned int size_t;








extern __declspec(__nothrow) void *memcpy(void * __restrict  ,
                    const void * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) void *memmove(void *  ,
                    const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   







 
extern __declspec(__nothrow) char *strcpy(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncpy(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 

extern __declspec(__nothrow) char *strcat(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncat(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 






 

extern __declspec(__nothrow) int memcmp(const void *  , const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strcmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int strncmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcasecmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strncasecmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcoll(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   







 

extern __declspec(__nothrow) size_t strxfrm(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   













 


#line 184 "C:\\Keil\\ARM\\RV31\\INC\\string.h"
extern __declspec(__nothrow) void *memchr(const void *  , int  , size_t  ) __attribute__((__nonnull__(1)));

   





 

#line 200 "C:\\Keil\\ARM\\RV31\\INC\\string.h"
extern __declspec(__nothrow) char *strchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   




 

extern __declspec(__nothrow) size_t strcspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 

#line 223 "C:\\Keil\\ARM\\RV31\\INC\\string.h"
extern __declspec(__nothrow) char *strpbrk(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   




 

#line 238 "C:\\Keil\\ARM\\RV31\\INC\\string.h"
extern __declspec(__nothrow) char *strrchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   





 

extern __declspec(__nothrow) size_t strspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   



 

#line 261 "C:\\Keil\\ARM\\RV31\\INC\\string.h"
extern __declspec(__nothrow) char *strstr(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   





 

extern __declspec(__nothrow) char *strtok(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) char *_strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

extern __declspec(__nothrow) char *strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

   

































 

extern __declspec(__nothrow) void *memset(void *  , int  , size_t  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) char *strerror(int  );
   





 
extern __declspec(__nothrow) size_t strlen(const char *  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) size_t strlcpy(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   
















 

extern __declspec(__nothrow) size_t strlcat(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






















 

extern __declspec(__nothrow) void _membitcpybl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpybb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
    














































 







#line 493 "C:\\Keil\\ARM\\RV31\\INC\\string.h"



 
#line 9 "modbus.c"

#line 1 "eeprom_map.h"





						  
#line 36 "eeprom_map.h"



#line 168 "eeprom_map.h"



#line 179 "eeprom_map.h"








 









 

#line 250 "eeprom_map.h"




#line 263 "eeprom_map.h"


#line 274 "eeprom_map.h"







#line 335 "eeprom_map.h"

#line 344 "eeprom_map.h"

#line 385 "eeprom_map.h"








#line 407 "eeprom_map.h"


























































































#line 505 "eeprom_map.h"


#line 515 "eeprom_map.h"


























































#line 579 "eeprom_map.h"

extern const unsigned short ADR_EE_BAT_ZAR_CNT[2];
extern const unsigned short ADR_EE_BAT_ZAR_CNT_KE[2];
extern const unsigned short ADR_EE_BAT_C_NOM[2];
extern const unsigned short ADR_EE_BAT_YEAR_OF_ON[2];
extern const unsigned short ADR_EE_BAT_IS_ON[2];
extern const unsigned short ADR_EE_BAT_DAY_OF_ON[2];
extern const unsigned short ADR_EE_BAT_MONTH_OF_ON[2];
extern const unsigned short ADR_EE_BAT_RESURS[2];
extern const unsigned short ADR_EE_BAT_C_REAL[2];
extern const unsigned short ADR_EE_BAT_TYPE[2];
extern const unsigned short ADR_KUBAT[2];
extern const unsigned short ADR_KUBATM[2];
extern const unsigned short ADR_KI0BAT[2];
extern const unsigned short ADR_KI1BAT[2];
extern const unsigned short ADR_KTBAT[2];
extern const unsigned short ADR_EE_BAT_TYPE[2];


extern const unsigned short ADR_TMAX_EXT_EN[3];
extern const unsigned short ADR_TMAX_EXT[3];
extern const unsigned short ADR_TMIN_EXT_EN[3];
extern const unsigned short ADR_TMIN_EXT[3];
extern const unsigned short ADR_T_EXT_REL_EN[3];
extern const unsigned short ADR_T_EXT_ZVUK_EN[3];
extern const unsigned short ADR_T_EXT_LCD_EN[3];
extern const unsigned short ADR_T_EXT_RS_EN[3];

extern const unsigned short ADR_SK_SIGN[4];
extern const unsigned short ADR_SK_REL_EN[4];
extern const unsigned short ADR_SK_ZVUK_EN[4];
extern const unsigned short ADR_SK_LCD_EN[4];
extern const unsigned short ADR_SK_RS_EN[4];

extern const unsigned short ADR_EE_RELE_SET_MASK[4];

#line 11 "modbus.c"
#line 12 "modbus.c"
#line 1 "modbus_tcp.h"

extern char plazma_modbus_tcp[20];
extern char modbus_tcp_plazma[20];

U16 tcp_callback (U8 soc, U8 evt, U8 *ptr, U16 par);

extern char modbus_tcp_func;
extern char modbus_tcp_unit;
extern short modbus_tcp_rx_arg0;
extern short modbus_tcp_rx_arg1;



extern char* modbus_tcp_out_ptr;

U16 tcp_callback (U8 soc, U8 evt, U8 *ptr, U16 par);

#line 13 "modbus.c"
#line 1 "25lc640.h"












char spi1(char in);
void spi1_config(void);
void spi1_config_mcp2515(void);
void spi1_unconfig(void);
void lc640_wren(void);
char lc640_rdsr(void);
int lc640_read(int ADR);
int lc640_read_int(int ADR);
long lc640_read_long(int ADR);
void lc640_read_long_ptr(int ADR,char* out_ptr);
void lc640_read_str(int ADR, char* ram_ptr, char num);
char lc640_write(int ADR,char in);
char lc640_write_int(short ADR,short in);
char lc640_write_long(int ADR,long in);
char lc640_write_long_ptr(int ADR,char* in);
#line 14 "modbus.c"
#line 1 "sc16is7xx.h"
#line 2 "sc16is7xx.h"

#line 15 "sc16is7xx.h"















extern char sc16is700ByteAvailable;
extern char sc16is700TxFifoLevel;
extern char tx_buffer_sc16is700[32]; 
extern char tx_wr_index_sc16is700;
extern char tx_rd_index_sc16is700;
extern char sc16is700TxFifoEmptyCnt; 
extern char sc16is700TxPossibleFlag;

extern char sc16is700_spi_init_cnt;


void sc16is700_init(uint32_t baudrate);
void sc16is700_wr_byte(char reg_num,char data);
char sc16is700_rd_byte(char reg_num);


void sc16is700_wr_buff(char reg_num,char num);
void putchar_sc16is700(char out_byte);
void sc16is700_uart_hndl(void);

#line 15 "modbus.c"
#line 1 "uart0.h"




















#line 29 "uart0.h"



extern char bRXIN0;
extern char UIB0[100];
extern char flag0;
extern char rx_buffer0[1024];
extern unsigned char tx_buffer0[1024];
extern unsigned short rx_wr_index0,rx_rd_index0,rx_counter0;
extern unsigned short tx_wr_index0,tx_rd_index0,tx_counter0;
extern char rx_buffer_overflow0;
extern char plazma_uart0;
extern char memo_out[50];
extern char data_rs[50];
extern char data_rs0[50];
extern const char Table87[];
extern const char Table95[]; 

char crc_87(char* ptr,char num);
char crc_95(char* ptr,char num);
void putchar0(char c);
void uart_out0 (char num,char data0,char data1,char data2,char data3,char data4,char data5);
void uart_out_adr0 (char *ptr, char len);
void uart0_init(void);
char getchar0(void);
__irq void uart0_interrupt(void);
void uart_in_an0(void);
signed short index_offset0 (signed short index,signed short offset);
char control_check0(signed short index);
void uart_in0(void);
void uart_out_adr_block (unsigned long adress,char *ptr, char len);
void rs232_data_out(void);
void rs232_data_out_tki(void);
void uart_out_buff0 (char *ptr, char len);
void rs232_data_out_1(void);
uint32_t UARTInit( uint32_t PortNum, uint32_t baudrate );

#line 16 "modbus.c"
#line 1 "avar_hndl.h"




extern unsigned avar_stat;	 	
extern unsigned avar_ind_stat; 	
extern unsigned avar_stat_old;
extern unsigned avar_stat_new,avar_stat_offed;








extern char sk_avar_stat;	 	
extern char sk_avar_ind_stat; 	
extern char sk_avar_stat_old;
extern char sk_avar_stat_new,sk_avar_stat_offed;


extern unsigned rki_avar1_stat;	 	
extern unsigned rki_avar1_ind_stat; 	
extern unsigned rki_avar1_stat_old;
extern unsigned rki_avar1_stat_new, rki_avar1_stat_offed;










 
void avar_hndl(void);
void avar_unet_hndl(char in);
void avar_uout_hndl(char in);
void reload_hndl(void);
void avar_bps_hndl(char bps, char v, char in);
void avar_bat_hndl(char bat, char in);
void avar_bat_as_hndl(char b, char in);
void ke_mem_hndl(char b,unsigned short in);
void ke_zvu_mem_hndl(char b,unsigned short in,unsigned short in1);
void vz_mem_hndl(unsigned short in);
void wrk_mem_hndl(char b);
void avar_bat_ips_hndl(char in);



#line 17 "modbus.c"
#line 1 "curr_version.h"
extern const short HARDVARE_VERSION;
extern const short SOFT_VERSION;
extern const short BUILD;
extern const short BUILD_YEAR;
extern const short BUILD_MONTH;
extern const short BUILD_DAY;
#line 18 "modbus.c"



extern int  mem_copy (void *dp, void *sp, int len);

unsigned char modbus_buf[20];
short modbus_crc16;
char modbus_timeout_cnt;
char bMODBUS_TIMEOUT;
unsigned char modbus_rx_buffer[30];	
unsigned char modbus_an_buffer[30];    	
unsigned char modbus_rx_buffer_ptr;	
unsigned char modbus_rx_counter;		

short modbus_plazma;				
short modbus_plazma1;				
short modbus_plazma2;				
short modbus_plazma3;				
short modbus_plazma_p;				
short modbus_plazma_pp;				

unsigned short modbus_rx_arg0;		
unsigned short modbus_rx_arg1;		
unsigned short modbus_rx_arg2;		
unsigned short modbus_rx_arg3;		

char modbus_tx_buff[100];



























































































































































































 

	



unsigned short CRC16_2(char* buf, short len)
{
unsigned short crc = 0xFFFF;
short pos;
short i;

for (pos = 0; pos < len; pos++)
  	{
    	crc ^= (unsigned short)buf[pos];          

    	for ( i = 8; i != 0; i--) 
		{    
      	if ((crc & 0x0001) != 0) 
			{      
        		crc >>= 1;                    
        		crc ^= 0xA001;
      		}
      	else  crc >>= 1;                    
    		}
  	}
  
return crc;
}



void modbus_zapros_ENMV (void){	 
unsigned short crc_temp;
unsigned char i_cnt;
	if(cnt_enmv_modbus_adress<(NUMENMV-1)) ++cnt_enmv_modbus_adress;
	else cnt_enmv_modbus_adress=0;

	if(enmv_on[cnt_enmv_modbus_adress]<10) ++enmv_on[cnt_enmv_modbus_adress];
   	else {
		for (i_cnt=0;i_cnt<64;i_cnt++) snmp_enmv_data[i_cnt][cnt_enmv_modbus_adress]=0xFF;
		for (i_cnt=0;i_cnt<8;i_cnt++) {enmv_data[i_cnt][cnt_enmv_modbus_adress]=0; enmv_data_pred[i_cnt][cnt_enmv_modbus_adress]=0;}
	}

	modbus_tx_buff[0]=enmv_modbus_adress[cnt_enmv_modbus_adress];
	modbus_tx_buff[1]=1;
	modbus_tx_buff[2]=0;
	modbus_tx_buff[3]=0;
	modbus_tx_buff[4]=0;
	modbus_tx_buff[5]=64;
	crc_temp=CRC16_2(modbus_tx_buff,6);
	modbus_tx_buff[6]=(char)crc_temp;
	modbus_tx_buff[7]=crc_temp>>8;
	for (i_cnt=0;i_cnt<8;i_cnt++)	putchar_sc16is700(modbus_tx_buff[i_cnt]);
		

} 


void modbus_in(void)
{
short crc16_calculated;		
short crc16_incapsulated;	
unsigned short modbus_rx_arg0;		
unsigned short modbus_rx_arg1;		


unsigned char modbus_func;			
char i_cnt, j_cnt; 


mem_copy(modbus_an_buffer,modbus_rx_buffer,modbus_rx_buffer_ptr);
modbus_rx_counter=modbus_rx_buffer_ptr;
modbus_rx_buffer_ptr=0;
bMODBUS_TIMEOUT=0;
	
crc16_calculated  = CRC16_2((char*)modbus_an_buffer, modbus_rx_counter-2);
crc16_incapsulated = *((short*)&modbus_an_buffer[modbus_rx_counter-2]);

modbus_plazma1=modbus_rx_counter;
modbus_plazma2=crc16_calculated;
modbus_plazma3=crc16_incapsulated;

modbus_func=modbus_an_buffer[1];
modbus_rx_arg0=(((unsigned short)modbus_an_buffer[2])*((unsigned short)256))+((unsigned short)modbus_an_buffer[3]);
modbus_rx_arg1=(((unsigned short)modbus_an_buffer[4])*((unsigned short)256))+((unsigned short)modbus_an_buffer[5]);






if(modbus_an_buffer[0]=='r')
	{
	pvlk=1;
	if(modbus_an_buffer[1]=='e')
		{
		pvlk=2;
		if(modbus_an_buffer[2]=='a')
			{
			pvlk=3;
			if(modbus_an_buffer[3]=='d')
				{
				pvlk=4;
				if(modbus_an_buffer[6]==crc_87(modbus_an_buffer,6))
					{
					pvlk=5;
					if(modbus_an_buffer[7]==crc_95(modbus_an_buffer,6))
						{
						pvlk=6;	

							{
							unsigned short ptr;
							unsigned long data1,data2;
							char temp_out[20];
							pvlk++;
							ptr=modbus_an_buffer[4]+(modbus_an_buffer[5]*256U);
							data1=lc640_read_long(ptr);
							data2=lc640_read_long(ptr+4);
							temp_out[0]='r';
							temp_out[1]='e';
							temp_out[2]='a';
							temp_out[3]='d';
							temp_out[4]=*((char*)&ptr);
							temp_out[5]=*(((char*)&ptr)+1);	
							temp_out[6]=*((char*)&data1);
							temp_out[7]=*(((char*)&data1)+1);		
							temp_out[8]=*(((char*)&data1)+2);	
							temp_out[9]=*(((char*)&data1)+3);		
							temp_out[10]=*((char*)&data2);
							temp_out[11]=*(((char*)&data2)+1);		
							temp_out[12]=*(((char*)&data2)+2);	
							temp_out[13]=*(((char*)&data2)+3);	
							temp_out[14]=crc_87(temp_out,14);	
							temp_out[15]=crc_95(temp_out,14);			
							
							temp_out[17]=0;
							for (i=0;i<16;i++)
								{
								putchar_sc16is700(temp_out[i]);
								temp_out[17]^=temp_out[i];
								}
							putchar_sc16is700(16);
							putchar_sc16is700(temp_out[17]^16);
							putchar_sc16is700(0x0a);
							}
						}
					}
				}
			} 
		}	 
	} 

if(modbus_an_buffer[0]=='w')
	{

	if(modbus_an_buffer[1]=='r')
		{

		if(modbus_an_buffer[2]=='i')
			{

			if(modbus_an_buffer[3]=='t')
				{

				if(modbus_an_buffer[4]=='e')
					{

					if(modbus_an_buffer[15]==crc_87(modbus_an_buffer,15))
						{

						if(modbus_an_buffer[16]==crc_95(modbus_an_buffer,15))

							{
							unsigned short ptr;
							unsigned long data1,data2;
							char temp_out[20];

							ptr=modbus_an_buffer[5]+(modbus_an_buffer[6]*256U);
							*((char*)&data1)=modbus_an_buffer[7];
							*(((char*)&data1)+1)=modbus_an_buffer[8];
							*(((char*)&data1)+2)=modbus_an_buffer[9];
							*(((char*)&data1)+3)=modbus_an_buffer[10];
							*((char*)&data2)=modbus_an_buffer[11];
							*(((char*)&data2)+1)=modbus_an_buffer[12];
							*(((char*)&data2)+2)=modbus_an_buffer[13];
							*(((char*)&data2)+3)=modbus_an_buffer[14];	
							lc640_write_long(ptr,data1);
							lc640_write_long(ptr+4,data2);
							
							
							
							temp_out[0]='w';
							temp_out[1]='r';
							temp_out[2]='i';
							temp_out[3]='t';
							temp_out[4]='e';
							temp_out[5]=*((char*)&ptr);
							temp_out[6]=*(((char*)&ptr)+1);	
						
							temp_out[7]=crc_87(temp_out,7);	
							temp_out[8]=crc_95(temp_out,7);			
							
							temp_out[10]=0;
							for (i=0;i<9;i++)
								{
								putchar_sc16is700(temp_out[i]);
								temp_out[10]^=temp_out[i];
								}
							putchar_sc16is700(9);
							putchar_sc16is700(temp_out[10]^9);
							putchar_sc16is700(0x0a);
							}
						}
					}
				}
		   	}
		}
	}

if(crc16_calculated==crc16_incapsulated)
	{
	ica_plazma[4]++;
	
	if(NUMENMV!=0 && modbus_func==1 && modbus_an_buffer[0]==enmv_modbus_adress[cnt_enmv_modbus_adress] && modbus_an_buffer[2]==8) {
			
				  for(i_cnt=0;i_cnt<8;i_cnt++) {
				  	for(j_cnt=0;j_cnt<8;j_cnt++){
					   snmp_enmv_data[i_cnt*8+j_cnt][cnt_enmv_modbus_adress]=(modbus_an_buffer[3+i_cnt]>>j_cnt)&0x01;
					}
					enmv_data[i_cnt][cnt_enmv_modbus_adress]=modbus_an_buffer[3+i_cnt];				   
				  }
				  enmv_on[cnt_enmv_modbus_adress]=0;
				  enmv_puts_en=1;
	} 
			
 	else if(modbus_an_buffer[0]==MODBUS_ADRESS)	  
		{
		modbus_modbus_adress_eq++;
		if(modbus_func==3)		
			{
			modbus_plazma++;
			modbus_hold_registers_transmit(MODBUS_ADRESS,modbus_func,modbus_rx_arg0,modbus_rx_arg1,0);
			}

		if(modbus_func==4)		
			{
			modbus_input_registers_transmit(MODBUS_ADRESS,modbus_func,modbus_rx_arg0,modbus_rx_arg1,0);
			modbus_modbus4f_cnt++;
			}

		else if(modbus_func==6) 	
			{
			if(modbus_rx_arg0==11)		
				{
				((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->YEAR=(uint16_t)modbus_rx_arg1;
				}
			if(modbus_rx_arg0==12)		
				{
				((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MONTH=(uint16_t)modbus_rx_arg1;
				}
			if(modbus_rx_arg0==13)		
				{
				((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->DOM=(uint16_t)modbus_rx_arg1;
				}
			if(modbus_rx_arg0==14)		
				{
				((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->HOUR=(uint16_t)modbus_rx_arg1;
				}
			if(modbus_rx_arg0==15)		
				{
				((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MIN=(uint16_t)modbus_rx_arg1;
				}
			if(modbus_rx_arg0==16)		
				{
				((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->SEC=(uint16_t)modbus_rx_arg1;
				}
			if(modbus_rx_arg0==20)		
				{
				if((modbus_rx_arg1>=0)&&(modbus_rx_arg1<=18))
				lc640_write_int(0x10+100+36,modbus_rx_arg1);  
				}
			if(modbus_rx_arg0==21)		
				{
				if((modbus_rx_arg1==0)||(modbus_rx_arg1==1))
				lc640_write_int(0x10+100+86,modbus_rx_arg1);  
				}
			if(modbus_rx_arg0==22)		
				{
				if((modbus_rx_arg1==0)||(modbus_rx_arg1==1))
				lc640_write_int(0x10+100+18,modbus_rx_arg1);  
				}
			if(modbus_rx_arg0==23)		
				{
				if((modbus_rx_arg1==0)||(modbus_rx_arg1==1))
				lc640_write_int(0x10+100+126,modbus_rx_arg1);  
				}
			if(modbus_rx_arg0==24)		
				{
				if( (modbus_rx_arg1<=20))
				lc640_write_int(0x10+500+96,modbus_rx_arg1);  
				}


			if(modbus_rx_arg0==30)		
				{
				if((modbus_rx_arg1>0)&&(modbus_rx_arg1<5))modbus_rx_arg1=0;
				else if(modbus_rx_arg1>=60)TBAT=60;
				else TBAT=modbus_rx_arg1;
				lc640_write_int(0x10+100+78,TBAT);

				main_kb_cnt=(TBAT*60)-20;
	     		}
			if(modbus_rx_arg0==31)		
				{
				lc640_write_int(0x10+100+4,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==32)		
				{
				lc640_write_int(0x10+100+84,UB20-modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==33)		
				{
				lc640_write_int(0x10+100+6,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==34)		
				{
				lc640_write_int(0x10+100+8,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==35)		
				{
				lc640_write_int(0x10+100+14,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==36)		
				{
				lc640_write_int(0x10+100+16,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==37)		
				{
				lc640_write_int(0x10+100+32,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==38)		
				{
				lc640_write_int(0x10+100+20,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==39)		
				{
				lc640_write_int(0x10+100+30,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==40)		
				{
				lc640_write_int(0x10+100+24,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==41)		
				{
				lc640_write_int(0x10+100+26,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==42)		
				{
				lc640_write_int(0x10+100+106,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==43)		
				{
				if((modbus_rx_arg1>=0)&&(modbus_rx_arg1<=3))lc640_write_int(0x10+100+34,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==44)		
				{
				lc640_write_int(0x10+100+10,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==45)		
				{
				lc640_write_int(0x10+100+82,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==46)		
				{
				lc640_write_int(0x10+100+88,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==47)		
				{
				lc640_write_int(0x10+100+90,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==48)		
				{
				lc640_write_int(0x10+100+162,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==49)		
				{
				lc640_write_int(0x10+100+164,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==50)		
				{
				lc640_write_int(0x10+100+166,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==51)		
				{
				lc640_write_int(0x10+100+182,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==52)		
				{
				lc640_write_int(0x10+100+184,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==53)		
				{
				if((modbus_rx_arg1>=5)&&(modbus_rx_arg1<=100))lc640_write_int(0x10+100+186,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==54)		
				{
				lc640_write_int(0x10+100+6,modbus_rx_arg1);
				lc640_write_int(0x10+100+8,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==55)		
				{
				lc640_write_int(4470,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==56)		
				{
				if(modbus_rx_arg1<=3) lc640_write_int(0x10+350+26,modbus_rx_arg1);
	     		}						
			if(modbus_rx_arg0==57)		
				{
				signed short www=(signed short)modbus_rx_arg1;
				if(www>=-12 && www<=13) lc640_write_int(0x10+350+28,modbus_rx_arg1);
	     		}						
			if(modbus_rx_arg0==58)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(4474,modbus_rx_arg1);
	     		}						
			if(modbus_rx_arg0==59)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(4476,modbus_rx_arg1);
	     		}						
			if(modbus_rx_arg0==60)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(4478,modbus_rx_arg1);
	     		}						
			if(modbus_rx_arg0==61)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(4480,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==62)		
				{

				if(modbus_rx_arg1<=2) lc640_write_int(0x10+350+72,modbus_rx_arg1);



	     		}
			if(modbus_rx_arg0==63)		
				{
				if(modbus_rx_arg1<=3) lc640_write_int(0x10+500+90,modbus_rx_arg1);
	     		}	
			if(modbus_rx_arg0==64)		
				{
				if(modbus_rx_arg1==0 || modbus_rx_arg1==2 || modbus_rx_arg1==4) lc640_write_int(0x10+500+100,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==65)		
				{
				if(modbus_rx_arg1<=4) lc640_write_int(0x10+500+88,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==66)		
				{
				if(modbus_rx_arg1==0 || modbus_rx_arg1==1) lc640_write_int(0x10+54,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==67)		
				{
				if(modbus_rx_arg1==0 || modbus_rx_arg1==1) lc640_write_int(0x10+56,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==68)		
				{
				if(modbus_rx_arg1==0 || modbus_rx_arg1==1) lc640_write_int(0x10+500+104,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==69)		
				{
				if(modbus_rx_arg1<9) lc640_write_int(4352,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==70)		
				{
				if(modbus_rx_arg1>=1 && modbus_rx_arg1<=200) lc640_write_int(0x10+350+92,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==71)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=25000) lc640_write_int(0x10+350+70,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==72)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=21000) lc640_write_int(0x10+350+68,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==73)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=20000) lc640_write_int(0x10+350+66,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==74)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=18000) lc640_write_int(0x10+350+64,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==75)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=16000) lc640_write_int(0x10+350+62,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==76)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=13000) lc640_write_int(0x10+350+60,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==77)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=8000) lc640_write_int(0x10+350+58,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==78)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=10000) lc640_write_int(0x10+350+86,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==79)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=10000) lc640_write_int(0x10+350+84,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==80)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=10000) lc640_write_int(0x10+350+82,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==81)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=10000) lc640_write_int(0x10+350+80,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==82)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=10000) lc640_write_int(0x10+350+78,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==83)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=10000) lc640_write_int(0x10+350+76,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==84)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=10000) lc640_write_int(0x10+350+74,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==85)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=100) lc640_write_int(0x10+350+94,modbus_rx_arg1);
	     		}		
			if(modbus_rx_arg0==86)		
				{
				if(modbus_rx_arg1>=15 && modbus_rx_arg1<=250) lc640_write_int(0x10+100+240,modbus_rx_arg1);
	     		}			
			if(modbus_rx_arg0==87)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=2000) lc640_write_int(0x10+350+54,modbus_rx_arg1);
	     		}  
			if(modbus_rx_arg0==88)		
				{
				if(modbus_rx_arg1<=72) lc640_write_int(0x10+100+76,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==89)		
				{
				if(modbus_rx_arg1==0 || modbus_rx_arg1==1) lc640_write_int(0x10+350+98,modbus_rx_arg1);
	     		} 
			if(modbus_rx_arg0==90)		
				{
				if(modbus_rx_arg1==0 || modbus_rx_arg1==1) lc640_write_int(0x10+100+168,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==91)		
				{
				if(modbus_rx_arg1>0 && modbus_rx_arg1<=100) lc640_write_int(0x10+100+170,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==92)		
				{
				if(modbus_rx_arg1==0 || modbus_rx_arg1==2) lc640_write_int(0x10+100+172,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==93)		
				{
				if(modbus_rx_arg1==0 || modbus_rx_arg1==1) lc640_write_int(0x10+100+174,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==94)		
				{								
				if(modbus_rx_arg1==0 || modbus_rx_arg1==1) lc640_write_int(0x10+350+96,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==95)		
				{								
				if(modbus_rx_arg1>=UB20 && modbus_rx_arg1<=2600) lc640_write_int(0x10+350+30,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==96)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=10000) lc640_write_int(0x10+350+32,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==97)		
				{
				if(modbus_rx_arg1>=1 && modbus_rx_arg1<=72) lc640_write_int(0x10+350+34,modbus_rx_arg1);
	     		}					  
			if(modbus_rx_arg0==98)		
				{
				if(modbus_rx_arg1>=UB20 && modbus_rx_arg1<=3000) lc640_write_int(0x10+350+40,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==99)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=1000) lc640_write_int(0x10+350+42,modbus_rx_arg1);
	     		}
			
			if(modbus_rx_arg0==102)		
				{
				if(modbus_rx_arg1>=1 && modbus_rx_arg1<=10) lc640_write_int(0x10+350+44,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==103)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=1000) lc640_write_int(0x10+350+46,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==104)		
				{
				if(modbus_rx_arg1>=UB20 && modbus_rx_arg1<=3000) lc640_write_int(0x10+350+48,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==105)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=1000) lc640_write_int(0x10+350+50,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==106)		
				{
				if(modbus_rx_arg1>=1 && modbus_rx_arg1<=10) lc640_write_int(0x10+350+52,modbus_rx_arg1);
	     		}	 
			if(modbus_rx_arg0==107)		
				{
				if(modbus_rx_arg1==0 || modbus_rx_arg1==1) lc640_write_int(0x10+100+12,modbus_rx_arg1);
	     		}				
		   	if(modbus_rx_arg0==108)		
				{
				if(modbus_rx_arg1==0 || modbus_rx_arg1==1) lc640_write_int(0x10+100+44,modbus_rx_arg1);
	     		}
		  	if(modbus_rx_arg0==109)		
				{
				if(modbus_rx_arg1==0 || modbus_rx_arg1==1) lc640_write_int(0x10+100+46,modbus_rx_arg1);
	     		}		   
		 	if(modbus_rx_arg0==110)		
				{
				if(modbus_rx_arg1>=1 && modbus_rx_arg1<=24) lc640_write_int(0x10+100+48,modbus_rx_arg1);
	     		}  
			if(modbus_rx_arg0==111)		
				{
				if(modbus_rx_arg1==0)	   lc640_write_int(0x10+500+48,0xFFFF);
				else if(modbus_rx_arg1==1) lc640_write_int(0x10+500+48,0);
	     		} 	   
			if(modbus_rx_arg0==112)		
				{
				if(modbus_rx_arg1==0)	   lc640_write_int(0x10+500+52,0xFFFF);
				else if(modbus_rx_arg1==1) lc640_write_int(0x10+500+52,0);
	     		}
			if(modbus_rx_arg0==113)		
				{
				if(modbus_rx_arg1==0)	   lc640_write_int(0x10+500+54,0xFFFF);
				else if(modbus_rx_arg1==1) lc640_write_int(0x10+500+54,0);
	     		}
			if(modbus_rx_arg0==114)		
				{
				if(modbus_rx_arg1==0)	   lc640_write_int(0x10+500+58,0xFFFF);
				else if(modbus_rx_arg1==1) lc640_write_int(0x10+500+58,0);
	     		} 	   
			if(modbus_rx_arg0==115)		
				{
				if(modbus_rx_arg1==0)	   lc640_write_int(0x10+500+62,0xFFFF);
				else if(modbus_rx_arg1==1) lc640_write_int(0x10+500+62,0);
	     		}
			if(modbus_rx_arg0==116)		
				{
				if(modbus_rx_arg1==0)	   lc640_write_int(0x10+500+64,0xFFFF);
				else if(modbus_rx_arg1==1) lc640_write_int(0x10+500+64,0);
	     		}
			if(modbus_rx_arg0==117)		
				{
				if(modbus_rx_arg1==0)	   lc640_write_int(0x10+500+68,0xFFFF);
				else if(modbus_rx_arg1==1) lc640_write_int(0x10+500+68,0);
	     		} 	   
			if(modbus_rx_arg0==118)		
				{
				if(modbus_rx_arg1==0)	   lc640_write_int(0x10+500+72,0xFFFF);
				else if(modbus_rx_arg1==1) lc640_write_int(0x10+500+72,0);
	     		}
			if(modbus_rx_arg0==119)		
				{
				if(modbus_rx_arg1==0)	   lc640_write_int(0x10+500+74,0xFFFF);
				else if(modbus_rx_arg1==1) lc640_write_int(0x10+500+74,0);
	     		}
			if(modbus_rx_arg0==120)		
				{
				if(modbus_rx_arg1==0)	   lc640_write_int(0x10+500+78,0xFFFF);
				else if(modbus_rx_arg1==1) lc640_write_int(0x10+500+78,0);
	     		} 	   
			if(modbus_rx_arg0==121)		
				{
				if(modbus_rx_arg1==0)	   lc640_write_int(0x10+500+82,0xFFFF);
				else if(modbus_rx_arg1==1) lc640_write_int(0x10+500+82,0);
	     		}
			if(modbus_rx_arg0==122)		
				{
				if(modbus_rx_arg1==0)	   lc640_write_int(0x10+500+84,0xFFFF);
				else if(modbus_rx_arg1==1) lc640_write_int(0x10+500+84,0);
	     		}		  
		   	if(modbus_rx_arg0==123)		
				{
				if(modbus_rx_arg1==0 || modbus_rx_arg1==1) lc640_write_int(0x10+100+126,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==124)		
				{
				if(modbus_rx_arg1<=500) lc640_write_int(0x10+100+178,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==125)		
				{
				if(modbus_rx_arg1==0 || modbus_rx_arg1==1) lc640_write_int(0x10+100+188,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==126)		
				{
				if(modbus_rx_arg1<=2) lc640_write_int(0x10+100+190,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==127)		
				{
				if(modbus_rx_arg1==0 || modbus_rx_arg1==1) lc640_write_int(0x10+100+192,modbus_rx_arg1);
	     		} 
		   	if(modbus_rx_arg0==128)		
				{
				if(modbus_rx_arg1>0 && modbus_rx_arg1<=100) lc640_write_int(0x10+500+200+72,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==129)		
				{
				if(modbus_rx_arg1==120 || modbus_rx_arg1==240 || modbus_rx_arg1==480 || modbus_rx_arg1==960 || modbus_rx_arg1==1920 
				|| modbus_rx_arg1==3840 || modbus_rx_arg1==5760 || modbus_rx_arg1==11520){ 
					lc640_write_int(0x10+500+200+74,modbus_rx_arg1);
					MODBUS_BAUDRATE=modbus_rx_arg1;
					}
	     		} 	
			if(modbus_rx_arg0==130)		
				{
				if(modbus_rx_arg1==0 || modbus_rx_arg1==1) lc640_write_int(0x10+500+200,modbus_rx_arg1);
	     		}	 
			if(modbus_rx_arg0==131)		
				{
				if(modbus_rx_arg1==0 || modbus_rx_arg1==1) lc640_write_int(0x10+500+200+2,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==132)  
				{
				if(modbus_rx_arg1<=255) lc640_write_int(0x10+500+200+4,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==133)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(0x10+500+200+6,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==134)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(0x10+500+200+8,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==135)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(0x10+500+200+10,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==136)	
				{
				if(modbus_rx_arg1<=255) lc640_write_int(0x10+500+200+12,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==137)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(0x10+500+200+14,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==138)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(0x10+500+200+16,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==139)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(0x10+500+200+18,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==140)	
				{
				if(modbus_rx_arg1<=255) lc640_write_int(0x10+500+200+64,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==141)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(0x10+500+200+66,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==142)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(0x10+500+200+68,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==143)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(0x10+500+200+70,modbus_rx_arg1);
	     		}	 
			if(modbus_rx_arg0==144)		
				{
				lc640_write_int(0x10+500+200+60,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==145)		
				{
				lc640_write_int(0x10+500+200+62,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==146)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(0x10+500+200+270,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==147)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(0x10+500+200+270+2,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==148)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(0x10+500+200+270+4,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==149)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(0x10+500+200+270+6,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==150)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(0x10+500+200+270+8,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==151)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(0x10+500+200+270+10,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==152)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(0x10+500+200+270+12,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==153)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(0x10+500+200+270+14,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==154)  
				{
				if(modbus_rx_arg1<=255) lc640_write_int(0x10+500+200+20,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==155)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(0x10+500+200+22,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==156)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(0x10+500+200+24,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==157)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(0x10+500+200+26,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==158)  
				{
				if(modbus_rx_arg1<=255) lc640_write_int(0x10+500+200+28,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==159)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(0x10+500+200+30,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==160)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(0x10+500+200+32,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==161)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(0x10+500+200+34,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==162)  
				{
				if(modbus_rx_arg1<=255) lc640_write_int(0x10+500+200+36,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==163)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(0x10+500+200+38,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==164)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(0x10+500+200+40,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==165)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(0x10+500+200+42,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==166)  
				{
				if(modbus_rx_arg1<=255) lc640_write_int(0x10+500+200+44,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==167)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(0x10+500+200+46,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==168)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(0x10+500+200+48,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==169)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(0x10+500+200+50,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==170)  
				{
				if(modbus_rx_arg1<=255) lc640_write_int(0x10+500+200+52,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==171)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(0x10+500+200+54,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==172)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(0x10+500+200+56,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==173)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(0x10+500+200+58,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==174)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(4360,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==175)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(4360+2,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==176)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(4360+4,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==177 && modbus_rx_arg1>0) bRESET_INT_WDT=1;
			if(modbus_rx_arg0==178)		
				{
				modbus_rx_arg1/=10;
				if(modbus_rx_arg1<=6000) lc640_write_int(0x10+350+2,modbus_rx_arg1);
	     		} 
		   	if(modbus_rx_arg0==179)		
				{
				if(modbus_rx_arg1<=2) lc640_write_int(0x10+350+6,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==180)		
				{
				if(modbus_rx_arg1<=2) lc640_write_int(0x10+350+4,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==181)  
				{
				if(modbus_rx_arg1<=255) lc640_write_int(0x10+350+10,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==182)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(0x10+350+12,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==183)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(0x10+350+14,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==184)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(0x10+350+16,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==185)		
				{
				if(modbus_rx_arg1>0 && modbus_rx_arg1<=254) lc640_write_int(0x10+350+18,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==186)		
				{
				if(modbus_rx_arg1>0 && modbus_rx_arg1<=254) lc640_write_int(0x10+350+8,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==187)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=100) lc640_write_int(0x10+350+20,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==188)		
				{
				if(modbus_rx_arg1>=1 && modbus_rx_arg1<=3) lc640_write_int(0x10+350+22,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==189)		
				{
				if(modbus_rx_arg1>=1 && modbus_rx_arg1<=5) lc640_write_int(0x10+350+24,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==190)		
				{
				if(modbus_rx_arg1==0 || modbus_rx_arg1==1) lc640_write_int(0x10+350+56,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==191)		
				{
				if(modbus_rx_arg1==1 || modbus_rx_arg1==3) lc640_write_int(4354,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==192)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=100) lc640_write_int(0x10+100+110,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==193)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=100) lc640_write_int(0x10+100+112,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==194)		
				{
				if(modbus_rx_arg1<=2) lc640_write_int(0x10+100+114,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==195)		
				{
				if(modbus_rx_arg1<=2) lc640_write_int(0x10+100+116,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==196)		
				{
				if(modbus_rx_arg1>=100 && modbus_rx_arg1<=2500) lc640_write_int(0x10+100+120,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==197)		
				{
				if(modbus_rx_arg1>=100 && modbus_rx_arg1<=2500) lc640_write_int(0x10+100+122,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==198)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=60) lc640_write_int(0x10+100+124,modbus_rx_arg1);
	     		}		
	


 
				
			if(modbus_rx_arg0==200)		
				{
				if(modbus_rx_arg1==1 ) command_rki=36;
				else if(modbus_rx_arg1==10 ) command_rki=37;
				else if(modbus_rx_arg1==0xFFFF ) command_rki=38; 
				else if(modbus_rx_arg1==0xFFF6 ) command_rki=39;
	     		}
			if(modbus_rx_arg0==201)		
				{
				if(modbus_rx_arg1==1 ) command_rki=40;
				else if(modbus_rx_arg1==10 ) command_rki=41;
				else if(modbus_rx_arg1==0xFFFF ) command_rki=42;
				else if(modbus_rx_arg1==0xFFF6 ) command_rki=43;
	     		}
			if(modbus_rx_arg0==202)		
				{
				if(modbus_rx_arg1==1 ) command_rki=14;
				else if(modbus_rx_arg1==0xFFFF ) command_rki=13;
	     		}
			if(modbus_rx_arg0==203)		
				{
				if(modbus_rx_arg1==1 ) command_rki=20;
				else if(modbus_rx_arg1==5 ) command_rki=22;
				else if(modbus_rx_arg1==0xFFFF ) command_rki=19;
				else if(modbus_rx_arg1==0xFFFB ) command_rki=21;
	     		}
			if(modbus_rx_arg0==204)		
				{
				if(modbus_rx_arg1==1 ) command_rki=24;
				else if(modbus_rx_arg1==5 ) command_rki=26;
				else if(modbus_rx_arg1==0xFFFF ) command_rki=23;
				else if(modbus_rx_arg1==0xFFFB ) command_rki=25;
	     		}
			if(modbus_rx_arg0==205)		
				{
				if(modbus_rx_arg1==1 ) command_rki=28;
				else if(modbus_rx_arg1==5 ) command_rki=30;
				else if(modbus_rx_arg1==0xFFFF ) command_rki=27;
				else if(modbus_rx_arg1==0xFFFB ) command_rki=29;
	     		}
			if(modbus_rx_arg0==206)		
				{
				if(modbus_rx_arg1==1 ) command_rki=16;
				else if(modbus_rx_arg1==10 ) command_rki=18;
				else if(modbus_rx_arg1==0xFFFF ) command_rki=15;
				else if(modbus_rx_arg1==0xFFF6 ) command_rki=17;
	     		}

#line 1451 "modbus.c"

			if(modbus_rx_arg0==19)		
				{
	

















 
				}
			if(modbus_rx_arg0==20)		
				{

















 
				}
			

			if(modbus_rx_arg0==100)		
				{
				
				if(modbus_rx_arg1&0x4000)
					{
					short tempSSSS;
					
					tempSSSS=modbus_rx_arg1&0x3fff;
					
					if((tempSSSS>0)&&(tempSSSS<5))tempSSSS=0;
					else if(tempSSSS>=60)tempSSSS=60;
				
					if(TBAT!=tempSSSS)lc640_write_int(0x10+100+78,tempSSSS);

					main_kb_cnt=(tempSSSS*60)-20;
					}
				else ica_cntrl_hndl=modbus_rx_arg1;
				ica_cntrl_hndl_cnt=200;

				
				}
			if(modbus_rx_arg0==101)		
				{
				ica_your_current==modbus_rx_arg1;
				ica_cntrl_hndl_cnt=200;
				
				}

			
				{
			
 
				mem_copy(modbus_tx_buff,modbus_rx_buffer,8);
	
				for (i=0;i<(8);i++)
					{
					putchar0(modbus_tx_buff[i]);
					}

				for (i=0;i<(8);i++)
					{
					putchar_sc16is700(modbus_tx_buff[i]);
					}
			



 
				}
			}











  

		} 
	else if(modbus_an_buffer[0]==ICA_MODBUS_ADDRESS)
		{
		ica_plazma[3]++;
		if(modbus_func==4)		
			{
			ica_plazma[2]++;
			if(modbus_an_buffer[2]==2)
				{
				ica_your_current=(((unsigned short)modbus_an_buffer[3])*((unsigned short)256))+((unsigned short)modbus_an_buffer[4]);
				}
			}
		}
	
	}


}






































































































































 










































































































































 






























































































 


void modbus_hold_registers_transmit(unsigned char adr,unsigned char func,unsigned short reg_adr,unsigned short reg_quantity, char prot)
{
signed char modbus_registers[800];	

unsigned short crc_temp;
char i;

modbus_registers[20]=(char)((((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->YEAR)>>8);			
modbus_registers[21]=(char)((((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->YEAR));
modbus_registers[22]=(char)((((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MONTH)>>8);		
modbus_registers[23]=(char)((((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MONTH));
modbus_registers[24]=(char)((((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->DOM)>>8);			
modbus_registers[25]=(char)((((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->DOM));
modbus_registers[26]=(char)((((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->HOUR)>>8);			
modbus_registers[27]=(char)((((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->HOUR));
modbus_registers[28]=(char)((((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MIN)>>8);			
modbus_registers[29]=(char)((((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MIN));
modbus_registers[30]=(char)((((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->SEC)>>8);			
modbus_registers[31]=(char)((((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->SEC));
modbus_registers[38]=(char)(NUMIST>>8);				
modbus_registers[39]=(char)(NUMIST);
modbus_registers[40]=(char)(PAR>>8);					
modbus_registers[41]=(char)(PAR);
modbus_registers[42]=(char)(ZV_ON>>8);					
modbus_registers[43]=(char)(ZV_ON);
modbus_registers[46]=(char)(UBM_AV>>8);				
modbus_registers[47]=(char)(UBM_AV);
modbus_registers[58]=(char)(TBAT>>8);					
modbus_registers[59]=(char)(TBAT);
modbus_registers[60]=(char)(UMAX>>8);					
modbus_registers[61]=(char)(UMAX);
modbus_registers[62]=(char)((UB20-DU)>>8);				
modbus_registers[63]=(char)((UB20-DU));
modbus_registers[64]=(char)(UB0>>8);					
modbus_registers[65]=(char)(UB0);
modbus_registers[66]=(char)(UB20>>8);					
modbus_registers[67]=(char)(UB20);
modbus_registers[68]=(char)(USIGN>>8);					
modbus_registers[69]=(char)(USIGN);
modbus_registers[70]=(char)(UMN>>8);					
modbus_registers[71]=(char)(UMN);
modbus_registers[72]=(char)(U0B>>8);					
modbus_registers[73]=(char)(U0B);
modbus_registers[74]=(char)(IKB>>8);					
modbus_registers[75]=(char)(IKB);
modbus_registers[76]=(char)(IZMAX>>8);					
modbus_registers[77]=(char)(IZMAX);
modbus_registers[78]=(char)(IMAX>>8);					
modbus_registers[79]=(char)(IMAX);
modbus_registers[80]=(char)(IMIN>>8);					
modbus_registers[81]=(char)(IMIN);
modbus_registers[82]=(char)(UVZ>>8);					
modbus_registers[83]=(char)(UVZ);
modbus_registers[84]=(char)(TZAS>>8);					
modbus_registers[85]=(char)(TZAS);
modbus_registers[86]=(char)(TMAX>>8);					
modbus_registers[87]=(char)(TMAX);
modbus_registers[88]=(char)(TSIGN>>8);					
modbus_registers[89]=(char)(TSIGN);
modbus_registers[90]=(char)(TBATMAX>>8);				
modbus_registers[91]=(char)(TBATMAX);
modbus_registers[92]=(char)(TBATSIGN>>8);				
modbus_registers[93]=(char)(TBATSIGN);
modbus_registers[94]=(char)(speedChrgCurr>>8);			
modbus_registers[95]=(char)(speedChrgCurr);
modbus_registers[96]=(char)(speedChrgVolt>>8);			
modbus_registers[97]=(char)(speedChrgVolt);
modbus_registers[98]=(char)(speedChrgTimeInHour>>8);	
modbus_registers[99]=(char)(speedChrgTimeInHour);
modbus_registers[100]=(char)(U_OUT_KONTR_MAX>>8);		
modbus_registers[101]=(char)(U_OUT_KONTR_MAX);
modbus_registers[102]=(char)(U_OUT_KONTR_MIN>>8);		
modbus_registers[103]=(char)(U_OUT_KONTR_MIN);
modbus_registers[104]=(char)(U_OUT_KONTR_DELAY>>8);		
modbus_registers[105]=(char)(U_OUT_KONTR_DELAY);
modbus_registers[106]=(char)(UB0>>8);					
modbus_registers[107]=(char)(UB0);
modbus_registers[108]=(char)(UMAXN>>8);					
modbus_registers[109]=(char)(UMAXN);

modbus_registers[110]=0;								
modbus_registers[111]=(char)(SNTP_ENABLE);				
modbus_registers[112]=(char)(SNTP_GMT>>8);				
modbus_registers[113]=(char)(SNTP_GMT);					
modbus_registers[114]=0;								
modbus_registers[115]=(char)(lc640_read_int(4474));					
modbus_registers[116]=0;								
modbus_registers[117]=(char)(lc640_read_int(4476));					
modbus_registers[118]=0;								
modbus_registers[119]=(char)(lc640_read_int(4478));					
modbus_registers[120]=0;								
modbus_registers[121]=(char)(lc640_read_int(4480));
modbus_registers[122]=0;								
modbus_registers[123]=(char)(NUMBAT);
modbus_registers[124]=0;								
modbus_registers[125]=(char)(NUMDT);	
modbus_registers[126]=0;								
modbus_registers[127]=(char)(NUMMAKB);	
modbus_registers[128]=0;								
modbus_registers[129]=(char)(NUMSK);
modbus_registers[130]=0;								
modbus_registers[131]=(char)(num_rki);	
modbus_registers[132]=0;								
modbus_registers[133]=(char)(num_net_in);
modbus_registers[134]=0;								
modbus_registers[135]=(char)(NUMBDR);
modbus_registers[136]=0;								
modbus_registers[137]=(char)(NUMENMV);

modbus_registers[138]=(char)(BAT_C_POINT_NUM_ELEM>>8);	
modbus_registers[139]=(char)(BAT_C_POINT_NUM_ELEM);
modbus_registers[140]=(char)(BAT_C_POINT_20>>8);		
modbus_registers[141]=(char)(BAT_C_POINT_20);
modbus_registers[142]=(char)(BAT_C_POINT_10>>8);		
modbus_registers[143]=(char)(BAT_C_POINT_10);
modbus_registers[144]=(char)(BAT_C_POINT_5>>8);			
modbus_registers[145]=(char)(BAT_C_POINT_5);
modbus_registers[146]=(char)(BAT_C_POINT_3>>8);			
modbus_registers[147]=(char)(BAT_C_POINT_3);
modbus_registers[148]=(char)(BAT_C_POINT_1>>8);			
modbus_registers[149]=(char)(BAT_C_POINT_1);
modbus_registers[150]=(char)(BAT_C_POINT_1_2>>8);		
modbus_registers[151]=(char)(BAT_C_POINT_1_2);
modbus_registers[152]=(char)(BAT_C_POINT_1_6>>8);		
modbus_registers[153]=(char)(BAT_C_POINT_1_6);
modbus_registers[154]=(char)(BAT_U_END_20>>8);			
modbus_registers[155]=(char)(BAT_U_END_20);
modbus_registers[156]=(char)(BAT_U_END_10>>8);			
modbus_registers[157]=(char)(BAT_U_END_10);
modbus_registers[158]=(char)(BAT_U_END_5>>8);			
modbus_registers[159]=(char)(BAT_U_END_5);
modbus_registers[160]=(char)(BAT_U_END_3>>8);			
modbus_registers[161]=(char)(BAT_U_END_3);
modbus_registers[162]=(char)(BAT_U_END_1>>8);			
modbus_registers[163]=(char)(BAT_U_END_1);
modbus_registers[164]=(char)(BAT_U_END_1_2>>8);			
modbus_registers[165]=(char)(BAT_U_END_1_2);
modbus_registers[166]=(char)(BAT_U_END_1_6>>8);			
modbus_registers[167]=(char)(BAT_U_END_1_6);
modbus_registers[168]=(char)(BAT_K_OLD>>8);				
modbus_registers[169]=(char)(BAT_K_OLD);

modbus_registers[170]=(char)(UVENTOFF>>8);			   	
modbus_registers[171]=(char)(UVENTOFF);
modbus_registers[172]=(char)(IMAX_VZ>>8);				
modbus_registers[173]=(char)(IMAX_VZ);
modbus_registers[174]=(char)(VZ_HR>>8);					
modbus_registers[175]=(char)(VZ_HR);
modbus_registers[176]=0;								
modbus_registers[177]=(char)(VZ_CH_VENT_BLOK);
modbus_registers[178]=0;								
modbus_registers[179]=(char)(speedChrgAvtEn);
modbus_registers[180]=(char)(speedChrgDU>>8);			
modbus_registers[181]=(char)(speedChrgDU);
modbus_registers[182]=0;								
modbus_registers[183]=(char)(speedChrgBlckSrc);
modbus_registers[184]=0;								
modbus_registers[185]=(char)(speedChrgBlckLog);
modbus_registers[186]=0;								
modbus_registers[187]=(char)(SP_CH_VENT_BLOK);
modbus_registers[188]=(char)(UZ_U>>8);					
modbus_registers[189]=(char)(UZ_U);
modbus_registers[190]=(char)(UZ_IMAX>>8);				
modbus_registers[191]=(char)(UZ_IMAX);
modbus_registers[192]=(char)(UZ_T>>8);					
modbus_registers[193]=(char)(UZ_T);
modbus_registers[194]=(char)(FZ_U1>>8);					
modbus_registers[195]=(char)(FZ_U1);
modbus_registers[196]=(char)(FZ_IMAX1>>8);				
modbus_registers[197]=(char)(FZ_IMAX1);

modbus_registers[202]=(char)(FZ_T1>>8);					
modbus_registers[203]=(char)(FZ_T1);
modbus_registers[204]=(char)(FZ_ISW12>>8);				
modbus_registers[205]=(char)(FZ_ISW12);
modbus_registers[206]=(char)(FZ_U2>>8);					
modbus_registers[207]=(char)(FZ_U2);
modbus_registers[208]=(char)(FZ_IMAX2>>8);				
modbus_registers[209]=(char)(FZ_IMAX2);
modbus_registers[210]=(char)(FZ_T2>>8);					
modbus_registers[211]=(char)(FZ_T2);
modbus_registers[212]=0;								
modbus_registers[213]=(char)(AV_OFF_AVT);
modbus_registers[214]=0;								
modbus_registers[215]=(char)(APV_ON1);
modbus_registers[216]=0;								
modbus_registers[217]=(char)(APV_ON2);
modbus_registers[218]=0;								
modbus_registers[219]=(char)(APV_ON2_TIME);
if(SK_SIGN[0]==0){
	modbus_registers[220]=0;								
	modbus_registers[221]=1;
}else{
	modbus_registers[220]=0;								
	modbus_registers[221]=0;
}
if(SK_ZVUK_EN[0]==0){
	modbus_registers[222]=0;								
	modbus_registers[223]=1;
}else{
	modbus_registers[222]=0;								
	modbus_registers[223]=0;
}
if(SK_LCD_EN[0]==0){
	modbus_registers[224]=0;								
	modbus_registers[225]=1;
}else{
	modbus_registers[224]=0;								
	modbus_registers[225]=0;
}
if(SK_SIGN[1]==0){
	modbus_registers[226]=0;								
	modbus_registers[227]=1;
}else{
	modbus_registers[226]=0;								
	modbus_registers[227]=0;
}
if(SK_ZVUK_EN[1]==0){
	modbus_registers[228]=0;								
	modbus_registers[229]=1;
}else{
	modbus_registers[228]=0;								
	modbus_registers[229]=0;										   
}
if(SK_LCD_EN[1]==0){
	modbus_registers[230]=0;								
	modbus_registers[231]=1;
}else{
	modbus_registers[230]=0;								
	modbus_registers[231]=0;
}

if(SK_SIGN[2]==0){
	modbus_registers[232]=0;								
	modbus_registers[233]=1;
}else{
	modbus_registers[232]=0;								
	modbus_registers[233]=0;
}
if(SK_ZVUK_EN[2]==0){
	modbus_registers[234]=0;								
	modbus_registers[235]=1;
}else{
	modbus_registers[234]=0;								
	modbus_registers[235]=0;
}
if(SK_LCD_EN[2]==0){
	modbus_registers[236]=0;								
	modbus_registers[237]=1;
}else{
	modbus_registers[236]=0;								
	modbus_registers[237]=0;
}
if(SK_SIGN[3]==0){
	modbus_registers[238]=0;								
	modbus_registers[239]=1;
}else{
	modbus_registers[238]=0;								
	modbus_registers[239]=0;
}
if(SK_ZVUK_EN[3]==0){
	modbus_registers[240]=0;								
	modbus_registers[241]=1;
}else{
	modbus_registers[240]=0;								
	modbus_registers[241]=0;										   
}
if(SK_LCD_EN[3]==0){
	modbus_registers[242]=0;								
	modbus_registers[243]=1;
}else{
	modbus_registers[242]=0;								
	modbus_registers[243]=0;
}
modbus_registers[244]=0;									
modbus_registers[245]=(char)(TERMOKOMPENS);
modbus_registers[246]=(char)(FORVARDBPSCHHOUR>>8);			
modbus_registers[247]=(char)(FORVARDBPSCHHOUR);
modbus_registers[248]=0;									
modbus_registers[249]=(char)(DOP_RELE_FUNC);
modbus_registers[250]=0;									
modbus_registers[251]=(char)(ipsBlckSrc);
modbus_registers[252]=0;									
modbus_registers[253]=(char)(ipsBlckLog);
modbus_registers[254]=0;									
modbus_registers[255]=(char)(MODBUS_ADRESS);		
modbus_registers[256]=(char)(MODBUS_BAUDRATE>>8);			
modbus_registers[257]=(char)(MODBUS_BAUDRATE);
modbus_registers[258]=0;									
modbus_registers[259]=(char)(ETH_IS_ON);
modbus_registers[260]=0;									
modbus_registers[261]=(char)(ETH_DHCP_ON);
modbus_registers[262]=0;									
modbus_registers[263]=(char)(ETH_IP_1);
modbus_registers[264]=0;									
modbus_registers[265]=(char)(ETH_IP_2);
modbus_registers[266]=0;									
modbus_registers[267]=(char)(ETH_IP_3);
modbus_registers[268]=0;									
modbus_registers[269]=(char)(ETH_IP_4);
modbus_registers[270]=0;									
modbus_registers[271]=(char)(ETH_MASK_1);
modbus_registers[272]=0;									
modbus_registers[273]=(char)(ETH_MASK_2);
modbus_registers[274]=0;									
modbus_registers[275]=(char)(ETH_MASK_3);
modbus_registers[276]=0;									
modbus_registers[277]=(char)(ETH_MASK_4);
modbus_registers[278]=0;									
modbus_registers[279]=(char)(ETH_GW_1);
modbus_registers[280]=0;									
modbus_registers[281]=(char)(ETH_GW_2);
modbus_registers[282]=0;									
modbus_registers[283]=(char)(ETH_GW_3);
modbus_registers[284]=0;									
modbus_registers[285]=(char)(ETH_GW_4);
modbus_registers[286]=(char)(ETH_SNMP_PORT_READ>>8);		
modbus_registers[287]=(char)(ETH_SNMP_PORT_READ);
modbus_registers[288]=(char)(ETH_SNMP_PORT_WRITE>>8);		
modbus_registers[289]=(char)(ETH_SNMP_PORT_WRITE);
modbus_registers[290]=0;									
modbus_registers[291]=(char)(snmp_community[0]);
modbus_registers[292]=0;									
modbus_registers[293]=(char)(snmp_community[1]);
modbus_registers[294]=0;									
modbus_registers[295]=(char)(snmp_community[2]);
modbus_registers[296]=0;									
modbus_registers[297]=(char)(snmp_community[3]);
modbus_registers[298]=0;									
modbus_registers[299]=(char)(snmp_community[4]);
modbus_registers[300]=0;									
modbus_registers[301]=(char)(snmp_community[5]);
modbus_registers[302]=0;									
modbus_registers[303]=(char)(snmp_community[6]);
modbus_registers[304]=0;									
modbus_registers[305]=(char)(snmp_community[7]);
modbus_registers[306]=0;									
modbus_registers[307]=(char)(ETH_TRAP1_IP_1);
modbus_registers[308]=0;									
modbus_registers[309]=(char)(ETH_TRAP1_IP_2);
modbus_registers[310]=0;									
modbus_registers[311]=(char)(ETH_TRAP1_IP_3);
modbus_registers[312]=0;									
modbus_registers[313]=(char)(ETH_TRAP1_IP_4);
modbus_registers[314]=0;									
modbus_registers[315]=(char)(ETH_TRAP2_IP_1);
modbus_registers[316]=0;									
modbus_registers[317]=(char)(ETH_TRAP2_IP_2);
modbus_registers[318]=0;									
modbus_registers[319]=(char)(ETH_TRAP2_IP_3);
modbus_registers[320]=0;									
modbus_registers[321]=(char)(ETH_TRAP2_IP_4);
modbus_registers[322]=0;									
modbus_registers[323]=(char)(ETH_TRAP3_IP_1);
modbus_registers[324]=0;									
modbus_registers[325]=(char)(ETH_TRAP3_IP_2);
modbus_registers[326]=0;									
modbus_registers[327]=(char)(ETH_TRAP3_IP_3);
modbus_registers[328]=0;									
modbus_registers[329]=(char)(ETH_TRAP3_IP_4);
modbus_registers[330]=0;									
modbus_registers[331]=(char)(ETH_TRAP4_IP_1);
modbus_registers[332]=0;									
modbus_registers[333]=(char)(ETH_TRAP4_IP_2);
modbus_registers[334]=0;									
modbus_registers[335]=(char)(ETH_TRAP4_IP_3);
modbus_registers[336]=0;									
modbus_registers[337]=(char)(ETH_TRAP4_IP_4);
modbus_registers[338]=0;									
modbus_registers[339]=(char)(ETH_TRAP5_IP_1);
modbus_registers[340]=0;									
modbus_registers[341]=(char)(ETH_TRAP5_IP_2);
modbus_registers[342]=0;									
modbus_registers[343]=(char)(ETH_TRAP5_IP_3);
modbus_registers[344]=0;									
modbus_registers[345]=(char)(ETH_TRAP5_IP_4);
modbus_registers[346]=0;									
modbus_registers[347]=(char)(snmp_web_passw[0]);
modbus_registers[348]=0;									
modbus_registers[349]=(char)(snmp_web_passw[1]);
modbus_registers[350]=0;									
modbus_registers[351]=(char)(snmp_web_passw[2]);
modbus_registers[352]=0;									
modbus_registers[353]=0;
modbus_registers[354]=(char)((TVENTMAX*10)>>8);				
modbus_registers[355]=(char)(TVENTMAX*10);
modbus_registers[356]=0;							
modbus_registers[357]=(char)(ICA_EN);
modbus_registers[358]=0;							
modbus_registers[359]=(char)(ICA_CH);
modbus_registers[360]=0;									
modbus_registers[361]=(char)(ICA_MODBUS_TCP_IP1);
modbus_registers[362]=0;									
modbus_registers[363]=(char)(ICA_MODBUS_TCP_IP2);
modbus_registers[364]=0;									
modbus_registers[365]=(char)(ICA_MODBUS_TCP_IP3);
modbus_registers[366]=0;									
modbus_registers[367]=(char)(ICA_MODBUS_TCP_IP4);
modbus_registers[368]=0;									
modbus_registers[369]=(char)(ICA_MODBUS_TCP_UNIT_ID);
modbus_registers[370]=0;									
modbus_registers[371]=(char)(ICA_MODBUS_ADDRESS);
modbus_registers[372]=0;									
modbus_registers[373]=(char)(PWM_START);
modbus_registers[374]=0;									
modbus_registers[375]=(char)(KB_ALGORITM);
modbus_registers[376]=0;									
modbus_registers[377]=(char)(REG_SPEED);
modbus_registers[378]=0;									
modbus_registers[379]=(char)(SMART_SPC);

modbus_registers[380]=0;									
modbus_registers[381]=(char)(NUMPHASE);
modbus_registers[382]=0;									
modbus_registers[383]=(char)(TVENTON);
modbus_registers[384]=0;									
modbus_registers[385]=(char)(TVENTOFF);
modbus_registers[386]=0;									
modbus_registers[387]=(char)(RELEVENTSIGN);
modbus_registers[388]=0;									
modbus_registers[389]=(char)(NPN_OUT);
modbus_registers[390]=(char)(UONPN>>8);						
modbus_registers[391]=(char)(UONPN);
modbus_registers[392]=(char)(UVNPN>>8);						
modbus_registers[393]=(char)(UVNPN);
modbus_registers[394]=0;									
modbus_registers[395]=(char)(TZNPN);

 


 
modbus_registers[398]=(char)(r_iz_porog_pred>>8);			
modbus_registers[399]=(char)(r_iz_porog_pred); 
modbus_registers[400]=(char)(r_iz_porog_error>>8);			
modbus_registers[401]=(char)(r_iz_porog_error);
modbus_registers[402]=0;									
modbus_registers[403]=(char)(asymmetry_porog);   		          	   
modbus_registers[404]=0;									
modbus_registers[405]=(char)(u_asymmetry_porog_up);
modbus_registers[406]=0;									
modbus_registers[407]=(char)(u_asymmetry_porog);
modbus_registers[408]=0;									
modbus_registers[409]=(char)(u_asymmetry_porog_down);
modbus_registers[410]=(char)(porog_u_in>>8);				
modbus_registers[411]=(char)(porog_u_in); 


#line 2457 "modbus.c"

if(prot==0)
	{
	modbus_tx_buff[0]=adr;
	modbus_tx_buff[1]=func;
	modbus_tx_buff[2]=(char)(reg_quantity*2);
	mem_copy((char*)&modbus_tx_buff[3],(char*)&modbus_registers[(reg_adr-1)*2],reg_quantity*2);
	
	crc_temp=CRC16_2(modbus_tx_buff,(reg_quantity*2)+3);

	modbus_tx_buff[3+(reg_quantity*2)]=(char)crc_temp;
	modbus_tx_buff[4+(reg_quantity*2)]=crc_temp>>8;

	for (i=0;i<(5+(reg_quantity*2));i++)
		{
		putchar0(modbus_tx_buff[i]);
		}

	for (i=0;i<(5+(reg_quantity*2));i++)
		{
		putchar_sc16is700(modbus_tx_buff[i]);
		}
	}
else if(prot==1)
	{
	modbus_tcp_out_ptr=(char*)&modbus_registers[(reg_adr-1)*2];
	}
}


void modbus_input_registers_transmit(unsigned char adr,unsigned char func,unsigned short reg_adr,unsigned short reg_quantity, char prot)
{
signed char modbus_registers[700];	 

unsigned short crc_temp;
char i;
short tempS;




#line 2504 "modbus.c"
modbus_registers[0]=(signed char)(out_U>>8);					
modbus_registers[1]=(signed char)(out_U);
modbus_registers[2]=(signed char)(bps_I>>8);					
modbus_registers[3]=(signed char)(bps_I);






 
modbus_registers[4]=(signed char)(net_U>>8);					
modbus_registers[5]=(signed char)(net_U);
modbus_registers[6]=(signed char)(net_F>>8);					
modbus_registers[7]=(signed char)(net_F);
modbus_registers[8]=(signed char)(net_Ua>>8);					
modbus_registers[9]=(signed char)(net_Ua);		 	
modbus_registers[10]=(signed char)(net_Ub>>8);				
modbus_registers[11]=(signed char)(net_Ub);
modbus_registers[12]=(signed char)(net_Uc>>8);				
modbus_registers[13]=(signed char)(net_Uc);
modbus_registers[14]=(signed char)(bat[0]._Ub>>8);				
modbus_registers[15]=(signed char)(bat[0]._Ub);
modbus_registers[16]=(signed char)(bat[0]._Ib>>8);				
modbus_registers[17]=(signed char)(bat[0]._Ib);

modbus_registers[18]=(signed char)(t_ext[0]>>8);				
modbus_registers[19]=(signed char)(t_ext[0]);





modbus_registers[20]=(signed char)(((short)(bat_hndl_zvu_Q/10000L))>>8);			
modbus_registers[21]=(signed char)(((short)(bat_hndl_zvu_Q/10000L)));




modbus_registers[22]=(signed char)(bat[0]._Ubm>>8);			
modbus_registers[23]=(signed char)(bat[0]._Ubm);
modbus_registers[24]=(signed char)(bat[0]._dUbm>>8);			
modbus_registers[25]=(signed char)(bat[0]._dUbm);
modbus_registers[26]=(signed char)(BAT_C_REAL[0]>>8);			
modbus_registers[27]=(signed char)(BAT_C_REAL[0]);
modbus_registers[28]=(signed char)(bat[1]._Ub>>8);				
modbus_registers[29]=(signed char)(bat[1]._Ub);
modbus_registers[30]=(signed char)(bat[1]._Ib>>8);				
modbus_registers[31]=(signed char)(bat[1]._Ib);
modbus_registers[32]=(signed char)(bat[1]._Tb>>8);				
modbus_registers[33]=(signed char)(bat[1]._Tb);
modbus_registers[34]=(signed char)(bat[1]._zar>>8);			
modbus_registers[35]=(signed char)(bat[1]._zar);
modbus_registers[36]=(signed char)(bat[1]._Ubm>>8);			
modbus_registers[37]=(signed char)(bat[1]._Ubm);
modbus_registers[38]=(signed char)(bat[1]._dUbm>>8);			
modbus_registers[39]=(signed char)(bat[1]._dUbm);
modbus_registers[40]=(signed char)(BAT_C_REAL[1]>>8);			
modbus_registers[41]=(signed char)(BAT_C_REAL[1]);
modbus_registers[42]=(signed char)(bps[0]._Uii>>8);			
modbus_registers[43]=(signed char)(bps[0]._Uii);
modbus_registers[44]=(signed char)(bps[0]._Ii>>8);				
modbus_registers[45]=(signed char)(bps[0]._Ii);
modbus_registers[46]=(signed char)(bps[0]._Ti>>8);				
modbus_registers[47]=(signed char)(bps[0]._Ti);
modbus_registers[48]=(signed char)(bps[0]._av>>8);				
modbus_registers[49]=(signed char)(bps[0]._av);
modbus_registers[50]=(signed char)(bps[1]._Uii>>8);			
modbus_registers[51]=(signed char)(bps[1]._Uii);
modbus_registers[52]=(signed char)(bps[1]._Ii>>8);				
modbus_registers[53]=(signed char)(bps[1]._Ii);
modbus_registers[54]=(signed char)(bps[1]._Ti>>8);				
modbus_registers[55]=(signed char)(bps[1]._Ti);
modbus_registers[56]=(signed char)(bps[1]._av>>8);				
modbus_registers[57]=(signed char)(bps[1]._av);
modbus_registers[58]=(signed char)(bps[2]._Uii>>8);			
modbus_registers[59]=(signed char)(bps[2]._Uii);
modbus_registers[60]=(signed char)(bps[2]._Ii>>8);				
modbus_registers[61]=(signed char)(bps[2]._Ii);
modbus_registers[62]=(signed char)(bps[2]._Ti>>8);				
modbus_registers[63]=(signed char)(bps[2]._Ti);
modbus_registers[64]=(signed char)(bps[2]._av>>8);				
modbus_registers[65]=(signed char)(bps[2]._av);
modbus_registers[66]=(signed char)(bps[3]._Uii>>8);			
modbus_registers[67]=(signed char)(bps[3]._Uii);
modbus_registers[68]=(signed char)(bps[3]._Ii>>8);				
modbus_registers[69]=(signed char)(bps[3]._Ii);
modbus_registers[70]=(signed char)(bps[3]._Ti>>8);				
modbus_registers[71]=(signed char)(bps[3]._Ti);
modbus_registers[72]=(signed char)(bps[3]._av>>8);				
modbus_registers[73]=(signed char)(bps[3]._av);
modbus_registers[74]=(signed char)(bps[4]._Uii>>8);			
modbus_registers[75]=(signed char)(bps[4]._Uii);
modbus_registers[76]=(signed char)(bps[4]._Ii>>8);				
modbus_registers[77]=(signed char)(bps[4]._Ii);
modbus_registers[78]=(signed char)(bps[4]._Ti>>8);				
modbus_registers[79]=(signed char)(bps[4]._Ti);
modbus_registers[80]=(signed char)(bps[4]._av>>8);				
modbus_registers[81]=(signed char)(bps[4]._av);
modbus_registers[82]=(signed char)(bps[5]._Uii>>8);			
modbus_registers[83]=(signed char)(bps[5]._Uii);
modbus_registers[84]=(signed char)(bps[5]._Ii>>8);				
modbus_registers[85]=(signed char)(bps[5]._Ii);
modbus_registers[86]=(signed char)(bps[5]._Ti>>8);				
modbus_registers[87]=(signed char)(bps[5]._Ti);
modbus_registers[88]=(signed char)(bps[5]._av>>8);				
modbus_registers[89]=(signed char)(bps[5]._av);
modbus_registers[90]=(signed char)(bps[6]._Uii>>8);			
modbus_registers[91]=(signed char)(bps[6]._Uii);
modbus_registers[92]=(signed char)(bps[6]._Ii>>8);				
modbus_registers[93]=(signed char)(bps[6]._Ii);
modbus_registers[94]=(signed char)(bps[6]._Ti>>8);				
modbus_registers[95]=(signed char)(bps[6]._Ti);
modbus_registers[96]=(signed char)(bps[6]._av>>8);				
modbus_registers[97]=(signed char)(bps[6]._av);
modbus_registers[98]=(signed char)(bps[7]._Uii>>8);			
modbus_registers[99]=(signed char)(bps[7]._Uii);
modbus_registers[100]=(signed char)(bps[7]._Ii>>8);			
modbus_registers[101]=(signed char)(bps[7]._Ii);
modbus_registers[102]=(signed char)(bps[7]._Ti>>8);			
modbus_registers[103]=(signed char)(bps[7]._Ti);
modbus_registers[104]=(signed char)(bps[7]._av>>8);			
modbus_registers[105]=(signed char)(bps[7]._av);
modbus_registers[106]=(signed char)(bps_U>>8);					
modbus_registers[107]=(signed char)(bps_U);
tempS=0;
if((speedChIsOn)||(sp_ch_stat==scsWRK)) tempS=1;
modbus_registers[108]=(signed char)(tempS>>8);					
modbus_registers[109]=(signed char)(tempS);
tempS=0;
if(spc_stat==spcVZ) tempS=1;
modbus_registers[110]=(signed char)(tempS>>8);					
modbus_registers[111]=(signed char)(tempS);
modbus_registers[112]=(signed char)(uout_av>>8);					
modbus_registers[113]=(signed char)(uout_av);












 
tempS=0;
tempS=avar_stat;

if(bat_ips._av)			tempS|=(1<<1);
else 					tempS&=~(1<<1);














modbus_registers[118]=(signed char)(tempS>>8);
modbus_registers[119]=(signed char)(tempS);

modbus_registers[120]=(signed char)(volta_short>>8);		
modbus_registers[121]=(signed char)(volta_short);
modbus_registers[122]=(signed char)(curr_short>>8);			
modbus_registers[123]=(signed char)(curr_short);
modbus_registers[124]=(signed char)(power_int>>8);			
modbus_registers[125]=(signed char)(power_int);


modbus_registers[138]=(signed char)(HARDVARE_VERSION>>8);	
modbus_registers[139]=(signed char)(HARDVARE_VERSION);
modbus_registers[140]=(signed char)(SOFT_VERSION>>8);		
modbus_registers[141]=(signed char)(SOFT_VERSION);
modbus_registers[142]=(signed char)(BUILD>>8);				
modbus_registers[143]=(signed char)(BUILD);
modbus_registers[144]=(signed char)(BUILD_YEAR>>8);			
modbus_registers[145]=(signed char)(BUILD_YEAR);
modbus_registers[146]=(signed char)(BUILD_MONTH>>8);		
modbus_registers[147]=(signed char)(BUILD_MONTH);
modbus_registers[148]=(signed char)(BUILD_DAY>>8);			
modbus_registers[149]=(signed char)(BUILD_DAY);
modbus_registers[150]=(signed char)(AUSW_MAIN_NUMBER>>8); 	
modbus_registers[151]=(signed char)(AUSW_MAIN_NUMBER);
modbus_registers[152]=(signed char)(AUSW_MAIN_NUMBER>>24);			
modbus_registers[153]=(signed char)(AUSW_MAIN_NUMBER>>16);
tempS=cntrl_stat_old;
if(	(main_kb_cnt==(TBAT*60)-21) || (main_kb_cnt==(TBAT*60)-20) || (main_kb_cnt==(TBAT*60)-19)) tempS=((short)TBAT)|0x4000;

modbus_registers[198]=(signed char)(tempS>>8);				
modbus_registers[199]=(signed char)(tempS);

modbus_registers[200]=(signed char)(bps[8]._Uii>>8);		
modbus_registers[201]=(signed char)(bps[8]._Uii);
modbus_registers[202]=(signed char)(bps[8]._Ii>>8);			
modbus_registers[203]=(signed char)(bps[8]._Ii);
modbus_registers[204]=(signed char)(bps[8]._Ti>>8);			
modbus_registers[205]=(signed char)(bps[8]._Ti);
modbus_registers[206]=(signed char)(bps[8]._av>>8);			
modbus_registers[207]=(signed char)(bps[8]._av);
modbus_registers[208]=(signed char)(bps[9]._Uii>>8);		
modbus_registers[209]=(signed char)(bps[9]._Uii);
modbus_registers[210]=(signed char)(bps[9]._Ii>>8);			
modbus_registers[211]=(signed char)(bps[9]._Ii);
modbus_registers[212]=(signed char)(bps[9]._Ti>>8);			
modbus_registers[213]=(signed char)(bps[9]._Ti);
modbus_registers[214]=(signed char)(bps[9]._av>>8);			
modbus_registers[215]=(signed char)(bps[9]._av);
modbus_registers[216]=(signed char)(bps[10]._Uii>>8);		
modbus_registers[217]=(signed char)(bps[10]._Uii);
modbus_registers[218]=(signed char)(bps[10]._Ii>>8);			
modbus_registers[219]=(signed char)(bps[10]._Ii);
modbus_registers[220]=(signed char)(bps[10]._Ti>>8);			
modbus_registers[221]=(signed char)(bps[10]._Ti);
modbus_registers[222]=(signed char)(bps[10]._av>>8);			
modbus_registers[223]=(signed char)(bps[10]._av);
modbus_registers[224]=(signed char)(bps[11]._Uii>>8);		
modbus_registers[225]=(signed char)(bps[11]._Uii);
modbus_registers[226]=(signed char)(bps[11]._Ii>>8);			
modbus_registers[227]=(signed char)(bps[11]._Ii);
modbus_registers[228]=(signed char)(bps[11]._Ti>>8);			
modbus_registers[229]=(signed char)(bps[11]._Ti);
modbus_registers[230]=(signed char)(bps[11]._av>>8);			
modbus_registers[231]=(signed char)(bps[11]._av);
modbus_registers[232]=(signed char)(bps[12]._Uii>>8);		
modbus_registers[233]=(signed char)(bps[12]._Uii);
modbus_registers[234]=(signed char)(bps[12]._Ii>>8);			
modbus_registers[235]=(signed char)(bps[12]._Ii);
modbus_registers[236]=(signed char)(bps[12]._Ti>>8);			
modbus_registers[237]=(signed char)(bps[12]._Ti);
modbus_registers[238]=(signed char)(bps[12]._av>>8);			
modbus_registers[239]=(signed char)(bps[12]._av);
modbus_registers[240]=(signed char)(bps[13]._Uii>>8);		
modbus_registers[241]=(signed char)(bps[13]._Uii);
modbus_registers[242]=(signed char)(bps[13]._Ii>>8);			
modbus_registers[243]=(signed char)(bps[13]._Ii);
modbus_registers[244]=(signed char)(bps[13]._Ti>>8);			
modbus_registers[245]=(signed char)(bps[13]._Ti);
modbus_registers[246]=(signed char)(bps[13]._av>>8);			
modbus_registers[247]=(signed char)(bps[13]._av);
modbus_registers[248]=(signed char)(bps[14]._Uii>>8);		
modbus_registers[249]=(signed char)(bps[14]._Uii);
modbus_registers[250]=(signed char)(bps[14]._Ii>>8);			
modbus_registers[251]=(signed char)(bps[14]._Ii);
modbus_registers[252]=(signed char)(bps[14]._Ti>>8);			
modbus_registers[253]=(signed char)(bps[14]._Ti);
modbus_registers[254]=(signed char)(bps[14]._av>>8);			
modbus_registers[255]=(signed char)(bps[14]._av);
modbus_registers[256]=(signed char)(bps[15]._Uii>>8);		
modbus_registers[257]=(signed char)(bps[15]._Uii);
modbus_registers[258]=(signed char)(bps[15]._Ii>>8);			
modbus_registers[259]=(signed char)(bps[15]._Ii);
modbus_registers[260]=(signed char)(bps[15]._Ti>>8);			
modbus_registers[261]=(signed char)(bps[15]._Ti);
modbus_registers[262]=(signed char)(bps[15]._av>>8);			
modbus_registers[263]=(signed char)(bps[15]._av);
modbus_registers[264]=(signed char)(bps[16]._Uii>>8);		
modbus_registers[265]=(signed char)(bps[16]._Uii);
modbus_registers[266]=(signed char)(bps[16]._Ii>>8);			
modbus_registers[267]=(signed char)(bps[16]._Ii);
modbus_registers[268]=(signed char)(bps[16]._Ti>>8);			
modbus_registers[269]=(signed char)(bps[16]._Ti);
modbus_registers[270]=(signed char)(bps[16]._av>>8);			
modbus_registers[271]=(signed char)(bps[16]._av);
modbus_registers[272]=(signed char)(bps[17]._Uii>>8);		
modbus_registers[273]=(signed char)(bps[17]._Uii);
modbus_registers[274]=(signed char)(bps[17]._Ii>>8);			
modbus_registers[275]=(signed char)(bps[17]._Ii);
modbus_registers[276]=(signed char)(bps[17]._Ti>>8);			
modbus_registers[277]=(signed char)(bps[17]._Ti);
modbus_registers[278]=(signed char)(bps[17]._av>>8);			
modbus_registers[279]=(signed char)(bps[17]._av);
modbus_registers[280]=(signed char)(bps[18]._Uii>>8);		
modbus_registers[281]=(signed char)(bps[18]._Uii);
modbus_registers[282]=(signed char)(bps[18]._Ii>>8);			
modbus_registers[283]=(signed char)(bps[18]._Ii);
modbus_registers[284]=(signed char)(bps[18]._Ti>>8);			
modbus_registers[285]=(signed char)(bps[18]._Ti);
modbus_registers[286]=(signed char)(bps[18]._av>>8);			
modbus_registers[287]=(signed char)(bps[18]._av);
modbus_registers[288]=(signed char)(bps[19]._Uii>>8);		
modbus_registers[289]=(signed char)(bps[19]._Uii);
modbus_registers[290]=(signed char)(bps[19]._Ii>>8);			
modbus_registers[291]=(signed char)(bps[19]._Ii);
modbus_registers[292]=(signed char)(bps[19]._Ti>>8);			
modbus_registers[293]=(signed char)(bps[19]._Ti);
modbus_registers[294]=(signed char)(bps[19]._av>>8);			
modbus_registers[295]=(signed char)(bps[19]._av);
modbus_registers[296]=(signed char)(bps[20]._Uii>>8);		
modbus_registers[297]=(signed char)(bps[20]._Uii);
modbus_registers[298]=(signed char)(bps[20]._Ii>>8);			
modbus_registers[299]=(signed char)(bps[20]._Ii);
modbus_registers[300]=(signed char)(bps[20]._Ti>>8);			
modbus_registers[301]=(signed char)(bps[20]._Ti);
modbus_registers[302]=(signed char)(bps[20]._av>>8);			
modbus_registers[303]=(signed char)(bps[20]._av);
modbus_registers[304]=(signed char)(bps[21]._Uii>>8);		
modbus_registers[305]=(signed char)(bps[21]._Uii);
modbus_registers[306]=(signed char)(bps[21]._Ii>>8);			
modbus_registers[307]=(signed char)(bps[21]._Ii);
modbus_registers[308]=(signed char)(bps[21]._Ti>>8);			
modbus_registers[309]=(signed char)(bps[21]._Ti);
modbus_registers[310]=(signed char)(bps[21]._av>>8);			
modbus_registers[311]=(signed char)(bps[21]._av);
modbus_registers[312]=(signed char)(bps[22]._Uii>>8);		
modbus_registers[313]=(signed char)(bps[22]._Uii);
modbus_registers[314]=(signed char)(bps[22]._Ii>>8);			
modbus_registers[315]=(signed char)(bps[22]._Ii);
modbus_registers[316]=(signed char)(bps[22]._Ti>>8);			
modbus_registers[317]=(signed char)(bps[22]._Ti);
modbus_registers[318]=(signed char)(bps[22]._av>>8);			
modbus_registers[319]=(signed char)(bps[22]._av);
modbus_registers[320]=(signed char)(bps[23]._Uii>>8);		
modbus_registers[321]=(signed char)(bps[23]._Uii);
modbus_registers[322]=(signed char)(bps[23]._Ii>>8);			
modbus_registers[323]=(signed char)(bps[23]._Ii);
modbus_registers[324]=(signed char)(bps[23]._Ti>>8);			
modbus_registers[325]=(signed char)(bps[23]._Ti);
modbus_registers[326]=(signed char)(bps[23]._av>>8);			
modbus_registers[327]=(signed char)(bps[23]._av);
modbus_registers[328]=(signed char)(bps[24]._Uii>>8);		
modbus_registers[329]=(signed char)(bps[24]._Uii);
modbus_registers[330]=(signed char)(bps[24]._Ii>>8);			
modbus_registers[331]=(signed char)(bps[24]._Ii);
modbus_registers[332]=(signed char)(bps[24]._Ti>>8);			
modbus_registers[333]=(signed char)(bps[24]._Ti);
modbus_registers[334]=(signed char)(bps[24]._av>>8);			
modbus_registers[335]=(signed char)(bps[24]._av);
modbus_registers[336]=(signed char)(bps[25]._Uii>>8);		
modbus_registers[337]=(signed char)(bps[25]._Uii);
modbus_registers[338]=(signed char)(bps[25]._Ii>>8);			
modbus_registers[339]=(signed char)(bps[25]._Ii);
modbus_registers[340]=(signed char)(bps[25]._Ti>>8);			
modbus_registers[341]=(signed char)(bps[25]._Ti);
modbus_registers[342]=(signed char)(bps[25]._av>>8);			
modbus_registers[343]=(signed char)(bps[25]._av);
modbus_registers[344]=(signed char)(bps[26]._Uii>>8);		
modbus_registers[345]=(signed char)(bps[26]._Uii);
modbus_registers[346]=(signed char)(bps[26]._Ii>>8);			
modbus_registers[347]=(signed char)(bps[26]._Ii);
modbus_registers[348]=(signed char)(bps[26]._Ti>>8);			
modbus_registers[349]=(signed char)(bps[26]._Ti);
modbus_registers[350]=(signed char)(bps[26]._av>>8);			
modbus_registers[351]=(signed char)(bps[26]._av);
modbus_registers[352]=(signed char)(bps[27]._Uii>>8);		
modbus_registers[353]=(signed char)(bps[27]._Uii);
modbus_registers[354]=(signed char)(bps[27]._Ii>>8);			
modbus_registers[355]=(signed char)(bps[27]._Ii);
modbus_registers[356]=(signed char)(bps[27]._Ti>>8);			
modbus_registers[357]=(signed char)(bps[27]._Ti);
modbus_registers[358]=(signed char)(bps[27]._av>>8);			
modbus_registers[359]=(signed char)(bps[27]._av);
modbus_registers[360]=(signed char)(bps[28]._Uii>>8);		
modbus_registers[361]=(signed char)(bps[28]._Uii);
modbus_registers[362]=(signed char)(bps[28]._Ii>>8);			
modbus_registers[363]=(signed char)(bps[28]._Ii);
modbus_registers[364]=(signed char)(bps[28]._Ti>>8);			
modbus_registers[365]=(signed char)(bps[28]._Ti);
modbus_registers[366]=(signed char)(bps[28]._av>>8);			
modbus_registers[367]=(signed char)(bps[28]._av);
modbus_registers[368]=(signed char)(bps[29]._Uii>>8);		
modbus_registers[369]=(signed char)(bps[29]._Uii);
modbus_registers[370]=(signed char)(bps[29]._Ii>>8);			
modbus_registers[371]=(signed char)(bps[29]._Ii);
modbus_registers[372]=(signed char)(bps[29]._Ti>>8);			
modbus_registers[373]=(signed char)(bps[29]._Ti);
modbus_registers[374]=(signed char)(bps[29]._av>>8);			
modbus_registers[375]=(signed char)(bps[29]._av);
modbus_registers[376]=(signed char)(bps[30]._Uii>>8);		
modbus_registers[377]=(signed char)(bps[30]._Uii);
modbus_registers[378]=(signed char)(bps[30]._Ii>>8);			
modbus_registers[379]=(signed char)(bps[30]._Ii);
modbus_registers[380]=(signed char)(bps[30]._Ti>>8);			
modbus_registers[381]=(signed char)(bps[30]._Ti);
modbus_registers[382]=(signed char)(bps[30]._av>>8);			
modbus_registers[383]=(signed char)(bps[30]._av);
modbus_registers[384]=(signed char)(bps[31]._Uii>>8);		
modbus_registers[385]=(signed char)(bps[31]._Uii);
modbus_registers[386]=(signed char)(bps[31]._Ii>>8);			
modbus_registers[387]=(signed char)(bps[31]._Ii);
modbus_registers[388]=(signed char)(bps[31]._Ti>>8);			
modbus_registers[389]=(signed char)(bps[31]._Ti);
modbus_registers[390]=(signed char)(bps[31]._av>>8);			
modbus_registers[391]=(signed char)(bps[31]._av);
modbus_registers[392]=(signed char)(bps[32]._Uii>>8);		
modbus_registers[393]=(signed char)(bps[32]._Uii);
modbus_registers[394]=(signed char)(bps[32]._Ii>>8);			
modbus_registers[395]=(signed char)(bps[32]._Ii);
modbus_registers[396]=(signed char)(bps[32]._Ti>>8);			
modbus_registers[397]=(signed char)(bps[32]._Ti);
modbus_registers[398]=(signed char)(bps[32]._av>>8);			
modbus_registers[399]=(signed char)(bps[32]._av);






tempS=t_ext[0];
if(ND_EXT[0])tempS=-1000;
modbus_registers[400]=(signed char)(tempS>>8);				
modbus_registers[401]=(signed char)(tempS);
tempS=t_ext[1];
if(ND_EXT[1])tempS=-1000;
modbus_registers[402]=(signed char)(tempS>>8);				
modbus_registers[403]=(signed char)(tempS);
tempS=t_ext[2];
if(ND_EXT[2])tempS=-1000;
modbus_registers[404]=(signed char)(tempS>>8);				
modbus_registers[405]=(signed char)(tempS);



 

modbus_registers[406]=(signed char)(bat_hndl_t_razr_min>>8);
modbus_registers[407]=(signed char)(bat_hndl_t_razr_min);

tempS=0;
if(sk_stat[0]==ssON) tempS|=0x0001;
if(sk_av_stat[0]==sasON) tempS|=0x0002;
modbus_registers[420]=(signed char)(tempS>>8);				
modbus_registers[421]=(signed char)(tempS);
tempS=0;
if(sk_stat[1]==ssON) tempS|=0x0001;
if(sk_av_stat[1]==sasON) tempS|=0x0002;
modbus_registers[422]=(signed char)(tempS>>8);				
modbus_registers[423]=(signed char)(tempS);
tempS=0;
if(sk_stat[2]==ssON) tempS|=0x0001;
if(sk_av_stat[2]==sasON) tempS|=0x0002;
modbus_registers[424]=(signed char)(tempS>>8);				
modbus_registers[425]=(signed char)(tempS);
tempS=0;
if(sk_stat[3]==ssON) tempS|=0x0001;
if(sk_av_stat[3]==sasON) tempS|=0x0002;
modbus_registers[426]=(signed char)(tempS>>8);				
modbus_registers[427]=(signed char)(tempS);

tempS=bat[0]._av;

tempS=ips_bat_av_stat;
if(NUMBAT==0)tempS=0xff; 


modbus_registers[428]=(signed char)(tempS>>8);				
modbus_registers[429]=(signed char)(tempS);

tempS=bat[1]._av;


tempS=0xff;  


modbus_registers[430]=(signed char)(tempS>>8);				
modbus_registers[431]=(signed char)(tempS);

tempS=bat_hndl_t_razr_min;
modbus_registers[432]=(signed char)(tempS>>8);				
modbus_registers[433]=(signed char)(tempS);

modbus_registers[434]=(signed char)(snmp_bat_flag[0]>>8);	
modbus_registers[435]=(signed char)(snmp_bat_flag[0]);	
modbus_registers[436]=(signed char)(snmp_bat_flag[1]>>8);	
modbus_registers[437]=(signed char)(snmp_bat_flag[1]);














 

modbus_registers[438]=(signed char)(bps[0]._vent_resurs>>8);	
modbus_registers[439]=(signed char)(bps[0]._vent_resurs);
modbus_registers[440]=(signed char)(bps[1]._vent_resurs>>8);	
modbus_registers[441]=(signed char)(bps[1]._vent_resurs);
modbus_registers[442]=(signed char)(bps[2]._vent_resurs>>8);	
modbus_registers[443]=(signed char)(bps[2]._vent_resurs);
modbus_registers[444]=(signed char)(bps[3]._vent_resurs>>8);	
modbus_registers[445]=(signed char)(bps[3]._vent_resurs);
modbus_registers[446]=(signed char)(bps[4]._vent_resurs>>8);	
modbus_registers[447]=(signed char)(bps[4]._vent_resurs);
modbus_registers[448]=(signed char)(bps[5]._vent_resurs>>8);	
modbus_registers[449]=(signed char)(bps[5]._vent_resurs);
modbus_registers[450]=(signed char)(bps[6]._vent_resurs>>8);	
modbus_registers[451]=(signed char)(bps[6]._vent_resurs);
modbus_registers[452]=(signed char)(bps[7]._vent_resurs>>8);	
modbus_registers[453]=(signed char)(bps[7]._vent_resurs); 

modbus_registers[598]=0;  	
if(no_rki==15) modbus_registers[599]=0;				
else modbus_registers[599]=1;							

modbus_registers[600]=0;								
modbus_registers[601]=ver_soft;
modbus_registers[602]=0;								
modbus_registers[603]=type_rki;

modbus_registers[604]=0;								
if(u_rki==1) modbus_registers[605]=48;
else if(u_rki==2) modbus_registers[605]=110;
else modbus_registers[605]=220;

modbus_registers[606]=(signed char)(status_izm_r>>8);	
modbus_registers[607]=(signed char)(status_izm_r);
	
modbus_registers[608]=(signed char)(r_iz_plus>>8);		
modbus_registers[609]=(signed char)(r_iz_plus);		
modbus_registers[610]=(signed char)(r_iz_minus>>8);		
modbus_registers[611]=(signed char)(r_iz_minus);
modbus_registers[612]=0;								
modbus_registers[613]=(signed char)(asymmetry);
modbus_registers[614]=(signed char)(u_asymmetry>>8);	
modbus_registers[615]=(signed char)(u_asymmetry);
modbus_registers[616]=(signed char)(v_plus>>8);			
modbus_registers[617]=(signed char)(v_plus); 
modbus_registers[618]=(signed char)(v_minus>>8);		
modbus_registers[619]=(signed char)(v_minus);
modbus_registers[620]=(signed char)(Ubus>>8);			
modbus_registers[621]=(signed char)(Ubus);

#line 3086 "modbus.c"


if(prot==0)
	{
	modbus_tx_buff[0]=adr;
	modbus_tx_buff[1]=func;

	modbus_tx_buff[2]=(char)(reg_quantity*2);

	mem_copy((signed char*)&modbus_tx_buff[3],(signed char*)&modbus_registers[(reg_adr-1)*2],reg_quantity*2);

	crc_temp=CRC16_2(modbus_tx_buff,(reg_quantity*2)+3);

	modbus_tx_buff[3+(reg_quantity*2)]=(char)crc_temp;
	modbus_tx_buff[4+(reg_quantity*2)]=crc_temp>>8;

	for (i=0;i<(5+(reg_quantity*2));i++)
		{
		putchar0(modbus_tx_buff[i]);
		}
	for (i=0;i<(5+(reg_quantity*2));i++)
		{
		putchar_sc16is700(modbus_tx_buff[i]);
		}
	}
else if(prot==1)
	{
	mem_copy((signed char*)modbus_tx_buff,(signed char*)&modbus_registers[(reg_adr-1)*2],reg_quantity*2);
	modbus_tcp_out_ptr=(signed char*)modbus_tx_buff;
	}
}

