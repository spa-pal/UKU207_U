#line 1 "control.c"
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
#line 2 "control.c"
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


#line 3 "control.c"
#line 1 "mess.h"










		





void mess_hndl(void);
void mess_send(char _mess, short par0, short par1, char _time);
char mess_find(char _mess);
char mess_find_unvol(char _mess);

#line 4 "control.c"
#line 1 "gran.h"

void gran_ring_char(signed char *adr, signed char min, signed char max) ;
void gran_char(signed char *adr, signed char min, signed char max);
void gran(signed short *adr, signed short min, signed short max);
void gran_ring(signed short *adr, signed short min, signed short max);
void gran_long(signed long *adr, signed long min, signed long max); 
#line 5 "control.c"
#line 1 "common_func.h"




signed short abs(signed short in);
void clr_scrn(void);
char find(char xy);
void bin2bcd_int(unsigned int in);
void bcd2lcd_zero(char sig);
void int2lcd_m(signed short in,char xy,char des);
void int2lcd_mm(signed short in,char xy,char des);
void int2lcd_mmm(signed short in,char xy,char des);
void long2lcd_mmm(signed long in,char xy,char des);
void long2lcdyx_mmm(signed long in,char y,char x,char des);
void int2lcdyx(unsigned short in,char y,char x,char des);
void int2lcd(unsigned short in,char xy,char des);
void long2lcdhyx(unsigned long in,char y,char x);
void char2lcdh(char in,char yx);
void char2lcdhyx(char in,char y,char x);
void int2lcdhyx(unsigned short in,char y,char x);
void char2lcdbyx(char in,char y,char x);
void pointer_set(char num_of_first_row);
void tree_down(signed char offset_ind,signed char offset_sub_ind);
void tree_up(char tind, char tsub_ind, char tindex_set, char tsub_ind1);
void bgnd_par(char const *ptr0,char const *ptr1,char const *ptr2,char const *ptr3);
void sub_bgnd(char const *adr,char xy,signed char offset);
void show_mess(char* p1, char* p2, char* p3, char* p4,int m_sec);
void show_mess_number(char* p1, char* p2, char* p3, char* p4,int m_sec,short number,char komma);
void event2ind(char num, char simbol);
char ptr_carry(signed int in,unsigned char modul,signed int carry);
void event_data2ind(char num, char simbol);
void ip2lcd(	short in1,
			short in2,
			short in3,
			short in4,
			char xy,
			char flash_pos);

void data2lcd(	
			short day,
			short month,
			short year,
			char xy,
			char flash_pos);

void community2lcd(char* in,
			char xy,
			char flash_pos,
			char flash_on);

void place2lcd(char* in,
			char xy,
			char flash_pos,
			char flash_on);
unsigned int power_inc(unsigned int argument, char powerum);
unsigned int power_dec(unsigned int argument, char powerum);

#line 6 "control.c"
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

#line 7 "control.c"
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



#line 8 "control.c"
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














 
#line 9 "control.c"
#line 1 "beep.h"

extern unsigned long beep_stat_temp,beep_stat;
extern char beep_stat_cnt;
extern char beep_cnt;
extern char bU_BAT2REL_AV_BAT;

void beep_drv(void);
void beep_init(long zvuk,char fl);
void beep_hndl(void);
#line 10 "control.c"
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





 
#line 11 "control.c"
#line 1 "sacred_sun.h"

extern char portForSacredSunBatteryIsInitiated;
extern char sacredSunBatteryHndlPhase;
extern char liBatteryInBuff[300];
extern char sacredSunRequestPhase;
extern short sacredSunSilentCnt;

void sacred_san_bat_hndl(void);
short ascii2halFhex(char in);
#line 12 "control.c"
#line 1 "sc16is7xx.h"
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

#line 13 "control.c"
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



#line 14 "control.c"
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

#line 15 "control.c"
#line 1 "uart1.h"









extern char bRXIN1;
extern char UIB1[20];
extern char flag1;
extern char rx_buffer1[100];
extern char tx_buffer1[300];
extern unsigned short rx_wr_index1,rx_rd_index1,rx_counter1;
extern unsigned short tx_wr_index1,tx_rd_index1,tx_counter1;
extern char rx_buffer_overflow1;
extern char plazma_uart1;
extern char uart1_mess[10];
extern char data_rs1[40];
typedef enum {ursMEGA=0x55,ursXPORT=0xaa}enum_usart1_router_stat;
extern enum_usart1_router_stat usart1_router_stat;
extern char usart1_router_wrk;
extern char memo_out1[100];

extern char UIB10[30];
extern char usart1_router_cnt;
extern char plazma_suz[5];
extern volatile uint32_t UART1Status;
extern volatile uint8_t UART1TxEmpty;
extern char uart1_net_cnt;

void putchar1(char c);
void uart_out1 (char num,char data0,char data1,char data2,char data3,char data4,char data5);
void uart_out_adr1 (char *ptr, unsigned char len);
void uart_out__adr1 (char *ptr, unsigned char len);
void uart_out_buff1 (char *ptr, char len);
uint32_t uart1_init(uint32_t baudrate);
char getchar1(void);
void UART1_IRQHandler (void);
void uart_in_an1(void);
char index_offset1 (signed char index,signed char offset);
char control_check1(char index);
void uart_in1(void);

#line 16 "control.c"
#line 1 "cmd.h"


#line 78 "cmd.h"






#line 17 "control.c"
#line 1 "stark.h"
extern char portForSTARKBatteryIsInitiated;
extern char sTARKBatteryHndlPhase;
extern char sTARKRequestPhase;
extern short sTARKSilentCnt[3];
extern char sTARKButteryCnter;
extern char sTARKBatteryHndlCmnd;


		 
void stark_bat_hndl(void);

#line 18 "control.c"
#line 1 "C:\\Keil\\ARM\\INC\\NXP\\LPC17xx\\LPC17xx.h"




















 









 

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

 









 
#line 97 "C:\\Keil\\ARM\\INC\\NXP\\LPC17xx\\LPC17xx.h"
#line 1 "C:\\Keil\\ARM\\INC\\NXP\\LPC17xx\\system_LPC17xx.h"




















 









extern uint32_t SystemFrequency;     










 
extern void SystemInit (void);





#line 98 "C:\\Keil\\ARM\\INC\\NXP\\LPC17xx\\LPC17xx.h"


 
 
 


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



 
 
 
 
#line 924 "C:\\Keil\\ARM\\INC\\NXP\\LPC17xx\\LPC17xx.h"

 
#line 945 "C:\\Keil\\ARM\\INC\\NXP\\LPC17xx\\LPC17xx.h"

 
#line 959 "C:\\Keil\\ARM\\INC\\NXP\\LPC17xx\\LPC17xx.h"

 
#line 972 "C:\\Keil\\ARM\\INC\\NXP\\LPC17xx\\LPC17xx.h"

 







 
 
 
#line 1031 "C:\\Keil\\ARM\\INC\\NXP\\LPC17xx\\LPC17xx.h"

#line 19 "control.c"











extern signed short u_necc,u_necc_,u_necc_up,u_necc_dn;
extern signed short main_cnt_5Hz;
extern signed short num_necc;
extern signed short num_necc_Imax;
extern signed short num_necc_Imin;




extern signed mat_temper;







typedef struct  
	{
     unsigned long int bAN:1; 
     unsigned long int bAB1:1; 
     unsigned long int bAB2:1;
     unsigned long int bAS1:1;
     unsigned long int bAS2:1;
     unsigned long int bAS3:1;
     unsigned long int bAS4:1;
     unsigned long int bAS5:1;
     unsigned long int bAS6:1;
     unsigned long int bAS7:1;
     unsigned long int bAS8:1;
     unsigned long int bAS9:1;
     unsigned long int bAS10:1;
     unsigned long int bAS11:1;
     unsigned long int bAS12:1;
     unsigned long int bAS13:1;
     unsigned long int bAS14:1;
     unsigned long int bAS15:1;
     unsigned long int bAS16:1;
     unsigned long int bAS17:1;
     unsigned long int bAS18:1;
     unsigned long int bAS19:1;
     unsigned long int bAS20:1;
     unsigned long int bAS21:1;
     unsigned long int bAS22:1;
     unsigned long int bAS23:1;
     unsigned long int bAS24:1;
     unsigned long int bAS25:1;
     unsigned long int bAS26:1;
     unsigned long int bAS27:1;
     unsigned long int bAS28:1;
     unsigned long int bAS29:1;
     unsigned long int bAS30:1;
     unsigned long int bAS31:1;
     unsigned long int bAS32:1;
     }avar_struct;
     
extern union 
{
avar_struct av;
long int avar_stat;
}a__,a_;



long adc_buff[16][16];
signed short adc_buff_max[12],adc_buff_min[12]={5000,5000,5000,5000,5000,5000,5000,5000,5000,5000},unet_buff_max,unet_buff_min=5000;
char adc_self_ch_cnt,adc_ch_net;
short adc_buff_[16];
char adc_cnt,adc_cnt1,adc_ch,adc_ch_cnt;
short zero_cnt;
enum_adc_stat adc_stat=asCH;
unsigned short net_buff[32],net_buff_,net_metr_buff_[3];
char net_buff_cnt;
short ADWR,period_cnt,non_zero_cnt;
char rele_stat;
char bRELE_OUT;
signed short adc_self_ch_buff[3],adc_self_ch_disp[3];
long main_power_buffer[8],main_power_buffer_;
short adc_result;
short main_power_buffer_cnt;
short adc_gorb_cnt,adc_zero_cnt;
char adc_window_flag;
short adc_window_cnt;
short adc_net_buff_cnt;


extern int mess_par0[10],mess_par1[10],mess_data[2];

extern signed short TBAT;
extern signed short Kunet;
extern signed short Kubat[2];
extern unsigned short ad7705_buff[2][16],ad7705_buff_[2];
extern unsigned short Kibat0[2];
extern signed short Kibat1[2];
extern signed short Ktbat[2];

short adc_buff_out_[3];
extern char kb_full_ver;
extern signed short Kuload;

signed short bat_ver_cnt=150;
extern signed short Isumm;
extern signed short Isumm_;
extern char ND_out[3];



short plazma_adc_cnt;
short plazma_sk;
extern char cntrl_plazma;





signed char vent_stat=0;



signed short cntrl_stat=610;
signed short cntrl_stat_old=610;
signed short cntrl_stat_new;
signed short Ibmax, Ibmax_;
unsigned char unh_cnt0,unh_cnt1,b1Hz_unh;
unsigned char	ch_cnt0,b1Hz_ch,i,iiii;
unsigned char	ch_cnt1,b1_30Hz_ch;
unsigned char	ch_cnt2,b1_10Hz_ch;
unsigned short IZMAX_;
unsigned short IZMAX_70;
unsigned short IZMAX_130;
unsigned short Ubpsmax;
unsigned short cntrl_stat_blck_cnt;




signed short samokalibr_cnt;





short avg_main_cnt=20;
signed int i_avg_max,i_avg_min,i_avg_summ,i_avg; 
signed int avg;
char bAVG;
char avg_cnt;  
char avg_num; 
char avg_vektor;



signed short 	main_kb_cnt;
signed short 	kb_cnt_1lev;
signed short 	kb_cnt_2lev;
char 		kb_full_ver;
char kb_start[2],kb_start_ips;
signed short ibat_ips,ibat_ips_;



char num_of_wrks_bps;
char bps_all_off_cnt,bps_mask_off_cnt,bps_mask_on_off_cnt;
char bps_hndl_2sec_cnt;
unsigned bps_on_mask,bps_off_mask;
char num_necc_up,num_necc_down;
unsigned char sh_cnt0,b1Hz_sh;



enum_spc_stat spc_stat;
char spc_bat;
char spc_phase;
unsigned short vz_cnt_s,vz_cnt_s_,vz_cnt_h,vz_cnt_h_;
char bAVZ;
enum_ke_start_stat ke_start_stat;
short cnt_end_ke;
unsigned long ke_date[2];
short __ee_vz_cnt;
short __ee_spc_stat;
short __ee_spc_bat;
short __ee_spc_phase;
char vz_error=0;  		
char sp_ch_error=0;		
char vz1_error=0;		
char vz2_error=0;		



extern unsigned avar_stat;	 	
extern unsigned avar_ind_stat; 	
extern unsigned avar_stat_old;
extern unsigned avar_stat_new,avar_stat_offed;










short cntrl_stat_blok_cnt,cntrl_stat_blok_cnt_,cntrl_stat_blok_cnt_plus[2],cntrl_stat_blok_cnt_minus[2];



const char sk_buff_KONTUR[4]={13,11,15,14};
const char sk_buff_RSTKM[4]={13,11,15,14};
const char sk_buff_GLONASS[4]={11,13,15,14};
const char sk_buff_3U[4]={11,13,15,14};
const char sk_buff_6U[4]={11,13,15,14};
const char sk_buff_220[4]={11,13,15,14};
const char sk_buff_TELECORE2015[4]={11,13,15,14};

char	plazma_inv[4];
char plazma_bat;
char plazma_cntrl_stat;



char numOfForvardBps,numOfForvardBps_old;
char numOfForvardBps_minCnt;
short numOfForvardBps_hourCnt;



char bPARALLEL_NOT_ENOUG;
char bPARALLEL_ENOUG;
char bPARALLEL;



char rx_read_power_cnt_phase=15;
short read_power_cnt_main_cnt=100;
short ce102m_delayCnt;
char rx_read_power_cnt_plazma=0;
char rx_read_power_cnt_flag=0;
short volta_short;
short curr_short;
int power_int;

char bENERGOMETR_UIP=0;

char cntrl_hndl_plazma;




enum_vz1_stat vz1_stat=vz1sOFF, vz1_stat_old=vz1sOFF;
short vz1_stat_cnt;
long vz1_wrk_cnt;
long vz1_up_cnt;
char volt_region;
short volt_region_cnt;



enum_vz2_stat vz2_stat=vz2sOFF, vz2_stat_old=vz2sOFF;
short vz2_stat_cnt;
long vz2_wrk_cnt;
long vz2_up_cnt;
signed short vz2_stat_ph2_cnt;

short plazma_ica1,plazma_ica2;
char rele_hndl_plazma[3];



short I_from_t_table[7];
char bat_hndl_zvu_init=0;
short bat_hndl_i;
long bat_hndl_t_razr;		
long bat_hndl_t_razr_ke;	
long bat_hndl_zvu_Q;
long bat_hndl_proc_razr;
long bat_hndl_remain_time;
short bat_hndl_t_razr_hour;
short bat_hndl_t_razr_min;
short bat_hndl_t_razr_mininhour;
char bat_hndl_zvu_ke_init=0;
short bat_hndl_i_temp;
short bat_hndl_u_end;
short U_end_from_i_table[7];
long bat_hndl_plazma[5];
char bat_hndl_zvu_Q_cnt;
long amper_chas_cnt_drv_summ;
char bat_hndl_i_vector=0,bat_hndl_i_vector_old=0;
long bat_hndl_i_zar_price=0L;
long bat_hndl_i_summ;

char avar_bps_reset_cnt;

short spirit_wrk_cnt;




void bat_flag (void)
{  
if(spc_stat!=spcVZ) 
	{
	vz_error=0;
	sp_ch_error=0;
	vz1_error=0;
	vz2_error=0;
	}

if( (snmp_bat_status[0]==0 || snmp_bat_status[0]==2) && NUMBAT>0) 	
	{
	if((bat[0]._Ub<(USIGN*10)))		
		{
		snmp_bat_flag[0]|=0x0001; 
		if((snmp_bat_flag_puts[0]&0x0001)==0) 
			{
			snmp_trap_send("BAT #1 Alarm, battery is low",5,8,0); 
			snmp_bat_flag_puts[0]|=0x0001;
			}
		}
	else 
		{
		snmp_bat_flag[0]&=~0x0001; 
		if(snmp_bat_flag_puts[0]&0x0001) 
			{
			snmp_trap_send("BAT #1 Alarm clear, battery is not low",5,8,1);
			snmp_bat_flag_puts[0]&=~0x0001;
			}
		}

	if(bat[0]._temper_stat&0x01)	
		{
		snmp_bat_flag[0]|=0x0002;
		if((snmp_bat_flag_puts[0]&0x0002)==0) 
			{
			
			snmp_bat_flag_puts[0]|=0x0002;
			}
		}
	else 
		{
		snmp_bat_flag[0]&=~0x0002;
		if(snmp_bat_flag_puts[0]&0x0002) 
			{
			
			snmp_bat_flag_puts[0]&=~0x02;
			}
		}

	if(bat[0]._temper_stat&0x02)  	
		{
		snmp_bat_flag[0]|=0x0004;
		if((snmp_bat_flag_puts[0]&0x0004)==0) 
			{
			
			snmp_bat_flag_puts[0]|=0x0004;
			}
		}
	else 
		{
		snmp_bat_flag[0]&=~0x0004;
		if(snmp_bat_flag_puts[0]&0x0004) 
			{
			
			snmp_bat_flag_puts[0]&=~0x0004;
			}
		}

	if(bat[0]._Ib<(-IKB)) snmp_bat_flag[0]|=0x0008;		
	else if(bat[0]._Ib>IKB) snmp_bat_flag[0]&=~0x0008;	

	if((spc_stat==spcKE)&&(spc_bat==0))					
		{
		snmp_bat_flag[0]|=0x0010;
		if((snmp_bat_flag_puts[0]&0x0010)==0) 
			{
			snmp_trap_send("BAT #1, capacity test started",5,8,6); 
			snmp_bat_flag_puts[0]|=0x0010;
			}
		}
	else 
		{
		snmp_bat_flag[0]&=~0x0010;
		if(snmp_bat_flag_puts[0]&0x0010) 
			{
			snmp_trap_send("BAT #1, capacity test stopped",5,8,7);
			snmp_bat_flag_puts[0]&=~0x0010;
			}
		}
		
	if(spc_stat==spcVZ)									
		{
		snmp_bat_flag[0]|=0x0020;
		if((snmp_bat_flag_puts[0]&0x0020)==0) 
			{
			snmp_trap_send("BAT #1,leveling charge is started",5,8,8); 
			snmp_bat_flag_puts[0]|=0x0020;
			}
		}
	else 
		{
		snmp_bat_flag[0]&=~0x0020;
		if(snmp_bat_flag_puts[0]&0x0020) 
			{
			snmp_trap_send("BAT #1,leveling charge is stopped",5,8,9);
			snmp_bat_flag_puts[0]&=~0x0020;
			}
		}

	if(vz_error)										 
		{
		snmp_bat_flag[0]|=0x0040;
		if((snmp_bat_flag_puts[0]&0x0040)==0) 
			{
			snmp_trap_send("BAT #1,leveling charge is blocked",5,8,10); 
			snmp_bat_flag_puts[0]|=0x0040;
			}
		}
	else 
		{
		snmp_bat_flag[0]&=~0x0040;
		if(snmp_bat_flag_puts[0]&0x0040) 
			{
			snmp_trap_send("BAT #1,leveling charge is unblocked",5,8,11);
			snmp_bat_flag_puts[0]&=~0x0040;
			} 
		}

	if(sp_ch_stat==scsWRK) 								
		{
		snmp_bat_flag[0]|=0x0080;
		if((snmp_bat_flag_puts[0]&0x0080)==0) 
			{
			snmp_trap_send("BAT #1,speed charge is started",5,8,10); 
			snmp_bat_flag_puts[0]|=0x0080;
			}
		}
	else 
		{
		snmp_bat_flag[0]&=~0x0080;
		if(snmp_bat_flag_puts[0]&0x0080) 
			{
			snmp_trap_send("BAT #1,speed charge is stopped",5,8,11);
			snmp_bat_flag_puts[0]&=~0x0080;
			}
		}

	if(sp_ch_error) 									
		{
		snmp_bat_flag[0]|=0x0100;
		if((snmp_bat_flag_puts[0]&0x0100)==0) 
			{
			snmp_trap_send("BAT #1,speed charge is blocked",5,8,12); 
			snmp_bat_flag_puts[0]|=0x0100;
			}
		}
	else 
		{
		snmp_bat_flag[0]&=~0x0100;
		if(snmp_bat_flag_puts[0]&0x0100) 
			{
			snmp_trap_send("BAT #1,speed charge is unblocked",5,8,13);
			snmp_bat_flag_puts[0]&=~0x0100;
			} 
		}

	if(vz1_stat!=vz1sOFF)								
		{
		snmp_bat_flag[0]|=0x0200;
		if((snmp_bat_flag_puts[0]&0x0200)==0) 
			{
			snmp_trap_send("BAT #1,equalising charge is on",5,8,14); 
			snmp_bat_flag_puts[0]|=0x0200;
			}
		}
	else 
		{
		snmp_bat_flag[0]&=~0x0200;
		if(snmp_bat_flag_puts[0]&0x0200) 
			{
			snmp_trap_send("BAT #1,equalising charge is off",5,8,15);
			snmp_bat_flag_puts[0]&=~0x0200;
			}
		}

	if(vz1_error)										
		{
		snmp_bat_flag[0]|=0x0400;
		if((snmp_bat_flag_puts[0]&0x0400)==0) 
			{
			snmp_trap_send("BAT #1,equalising charge is blocked",5,8,16); 
			snmp_bat_flag_puts[0]|=0x0400;
			}
		}
	else 
		{
		snmp_bat_flag[0]&=~0x0400;
		if(snmp_bat_flag_puts[0]&0x0400) 
			{
			snmp_trap_send("BAT #1,equalising charge is unblocked",5,8,17);
			snmp_bat_flag_puts[0]&=~0x0400;
			} 
		}

	if(vz2_stat!=vz2sOFF)								
		{
		snmp_bat_flag[0]|=0x0800;
		if((snmp_bat_flag_puts[0]&0x0800)==0) 
			{
			snmp_trap_send("BAT #1,molding charge is on",5,8,18); 
			snmp_bat_flag_puts[0]|=0x0800;
			}
		}
	else 
		{
		snmp_bat_flag[0]&=~0x0800;
		if(snmp_bat_flag_puts[0]&0x0800) 
			{
			snmp_trap_send("BAT #1,molding charge is off",5,8,19);
			snmp_bat_flag_puts[0]&=~0x0800;
			}
		}

	if(vz2_error) 										
		{
		snmp_bat_flag[0]|=0x1000;
		if((snmp_bat_flag_puts[0]&0x1000)==0) 
			{
			snmp_trap_send("BAT #1,molding charge is blocked",5,8,20); 
			snmp_bat_flag_puts[0]|=0x1000;
			}
		}
	else 
		{
		snmp_bat_flag[0]&=~0x1000;
		if(snmp_bat_flag_puts[0]&0x1000) 
			{
			snmp_trap_send("BAT #1,molding charge is unblocked",5,8,21);
			snmp_bat_flag_puts[0]&=~0x1000;
			} 
		}
	}
else 
	{
	snmp_bat_flag[0]=0; 
	snmp_bat_flag_puts[0]=0;
	}

if( (snmp_bat_status[1]==0 || snmp_bat_status[1]==2) && NUMBAT==2 )	
	{
	if((bat[1]._Ub<(USIGN*10)))		
		{
		snmp_bat_flag[1]|=0x0001; 
		if((snmp_bat_flag_puts[1]&0x0001)==0) 
			{
			snmp_trap_send("BAT #2 Alarm, battery is low",5,8,22); 
			snmp_bat_flag_puts[1]|=0x0001;
			}
		}
	else 
		{
		snmp_bat_flag[1]&=~0x0001; 
		if(snmp_bat_flag_puts[1]&0x0001) 
			{
			snmp_trap_send("BAT #2 Alarm clear, battery is not low",5,8,23);
			snmp_bat_flag_puts[1]&=~0x0001;
			}
		}

	if(bat[1]._temper_stat&0x01)	
		{
		snmp_bat_flag[1]|=0x0002;
		if((snmp_bat_flag_puts[0]&0x0002)==0) 
			{
			
			snmp_bat_flag_puts[1]|=0x0002;
			}
		}
	else 
		{
		snmp_bat_flag[1]&=~0x0002;
		if(snmp_bat_flag_puts[1]&0x0002) 
			{
			
			snmp_bat_flag_puts[1]&=~0x02;
			}
		}

	if(bat[1]._temper_stat&0x02)  	
		{
		snmp_bat_flag[1]|=0x0004;
		if((snmp_bat_flag_puts[1]&0x0004)==0) 
			{
			
			snmp_bat_flag_puts[1]|=0x0004;
			}
		}
	else 
		{
		snmp_bat_flag[1]&=~0x0004;
		if(snmp_bat_flag_puts[1]&0x0004) 
			{
			
			snmp_bat_flag_puts[1]&=~0x0004;
			}
		}

	if(bat[1]._Ib<(-IKB)) snmp_bat_flag[1]|=0x0008;		
	else if(bat[1]._Ib>IKB) snmp_bat_flag[1]&=~0x0008;	

	if((spc_stat==spcKE)&&(spc_bat==0))					
		{
		snmp_bat_flag[1]|=0x0010;
		if((snmp_bat_flag_puts[1]&0x0010)==0) 
			{
			snmp_trap_send("BAT #2, capacity test started",5,8,28); 
			snmp_bat_flag_puts[1]|=0x0010;
			}
		}
	else 
		{
		snmp_bat_flag[1]&=~0x0010;
		if(snmp_bat_flag_puts[1]&0x0010) 
			{
			snmp_trap_send("BAT #2, capacity test stopped",5,8,29);
			snmp_bat_flag_puts[1]&=~0x0010;
			}
		}
		
	if(spc_stat==spcVZ)									
		{
		snmp_bat_flag[1]|=0x0020;
		if((snmp_bat_flag_puts[1]&0x0020)==0) 
			{
			snmp_trap_send("BAT #2,leveling charge is started",5,8,30); 
			snmp_bat_flag_puts[1]|=0x0020;
			}
		}
	else 
		{
		snmp_bat_flag[1]&=~0x0020;
		if(snmp_bat_flag_puts[1]&0x0020) 
			{
			snmp_trap_send("BAT #2,leveling charge is stopped",5,8,31);
			snmp_bat_flag_puts[1]&=~0x0020;
			}
		}

	if(vz_error)										 
		{
		snmp_bat_flag[1]|=0x0040;
		if((snmp_bat_flag_puts[1]&0x0040)==0) 
			{
			snmp_trap_send("BAT #2,leveling charge is blocked",5,8,32); 
			snmp_bat_flag_puts[1]|=0x0040;
			}
		}
	else 
		{
		snmp_bat_flag[1]&=~0x0040;
		if(snmp_bat_flag_puts[1]&0x0040) 
			{
			snmp_trap_send("BAT #2,leveling charge is unblocked",5,8,33);
			snmp_bat_flag_puts[1]&=~0x0040;
			} 
		}

	if(sp_ch_stat==scsWRK) 								
		{
		snmp_bat_flag[1]|=0x0080;
		if((snmp_bat_flag_puts[1]&0x0080)==0) 
			{
			snmp_trap_send("BAT #2,speed charge is started",5,8,34); 
			snmp_bat_flag_puts[1]|=0x0080;
			}
		}
	else 
		{
		snmp_bat_flag[1]&=~0x0080;
		if(snmp_bat_flag_puts[1]&0x0080) 
			{
			snmp_trap_send("BAT #2,speed charge is stopped",5,8,35);
			snmp_bat_flag_puts[1]&=~0x0080;
			}
		}

	if(sp_ch_error) 									
		{
		snmp_bat_flag[1]|=0x0100;
		if((snmp_bat_flag_puts[1]&0x0100)==0) 
			{
			snmp_trap_send("BAT #2,speed charge is blocked",5,8,36); 
			snmp_bat_flag_puts[1]|=0x0100;
			}
		}
	else 
		{
		snmp_bat_flag[1]&=~0x0100;
		if(snmp_bat_flag_puts[1]&0x0100) 
			{
			snmp_trap_send("BAT #2,speed charge is unblocked",5,8,37);
			snmp_bat_flag_puts[1]&=~0x0100;
			} 
		}

	if(vz1_stat!=vz1sOFF)								
		{
		snmp_bat_flag[1]|=0x0200;
		if((snmp_bat_flag_puts[1]&0x0200)==0) 
			{
			snmp_trap_send("BAT #2,equalising charge is on",5,8,38); 
			snmp_bat_flag_puts[1]|=0x0200;
			}
		}
	else 
		{
		snmp_bat_flag[1]&=~0x0200;
		if(snmp_bat_flag_puts[1]&0x0200) 
			{
			snmp_trap_send("BAT #2,equalising charge is off",5,8,39);
			snmp_bat_flag_puts[1]&=~0x0200;
			}
		}

	if(vz1_error)										
		{
		snmp_bat_flag[1]|=0x0400;
		if((snmp_bat_flag_puts[1]&0x0400)==0) 
			{
			snmp_trap_send("BAT #2,equalising charge is blocked",5,8,40); 
			snmp_bat_flag_puts[1]|=0x0400;
			}
		}
	else 
		{
		snmp_bat_flag[1]&=~0x0400;
		if(snmp_bat_flag_puts[1]&0x0400) 
			{
			snmp_trap_send("BAT #2,equalising charge is unblocked",5,8,41);
			snmp_bat_flag_puts[1]&=~0x0400;
			} 
		}

	if(vz2_stat!=vz2sOFF)								
		{
		snmp_bat_flag[1]|=0x0800;
		if((snmp_bat_flag_puts[1]&0x0800)==0) 
			{
			snmp_trap_send("BAT #2,molding charge is on",5,8,42); 
			snmp_bat_flag_puts[1]|=0x0800;
			}
		}
	else 
		{
		snmp_bat_flag[1]&=~0x0800;
		if(snmp_bat_flag_puts[1]&0x0800) 
			{
			snmp_trap_send("BAT #2,molding charge is off",5,8,43);
			snmp_bat_flag_puts[1]&=~0x0800;
			}
		}

	if(vz2_error) 										
		{
		snmp_bat_flag[1]|=0x1000;
		if((snmp_bat_flag_puts[1]&0x1000)==0) 
			{
			snmp_trap_send("BAT #2,molding charge is blocked",5,8,44); 
			snmp_bat_flag_puts[1]|=0x1000;
			}
		}
	else 
		{
		snmp_bat_flag[1]&=~0x1000;
		if(snmp_bat_flag_puts[1]&0x1000) 
			{
			snmp_trap_send("BAT #2,molding charge is unblocked",5,8,45);
			snmp_bat_flag_puts[1]&=~0x1000;
			} 
		}
	}
else 
	{
	snmp_bat_flag[1]=0; 
	snmp_bat_flag_puts[1]=0;
	}


























































































































 
}




void ke_start(char in)
{          
ke_start_stat=(enum_ke_start_stat)0;		 












 
	{

	ke_start_stat=kssYES;

	spc_stat=spcKE;
	__ee_spc_stat=spcKE;
	lc640_write_int(0x10+500+150,__ee_spc_stat);
	
	spc_bat=0;
	__ee_spc_bat=0;
	lc640_write_int(0x10+500+150+4,__ee_spc_bat);

	bat[0]._zar_cnt_ke=0;
	lc640_write_int(ADR_EE_BAT_ZAR_CNT_KE[0],0);

	bat[0]._time_min_cnt_ke=0;
	lc640_write_int(0x10+350+88,0);

	
	spc_phase=0;
	__ee_spc_phase=0;
	lc640_write_int(0x10+500+150+6,__ee_spc_phase);

	

		{					
		signed short temp_temp;
		signed char temp;
		temp_temp=bat[in]._u_old[((bat_u_old_cnt+1)&0x07)]; 
		    
		temp=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->YEAR;
		gran_char(&temp,1,99);
		*((char*)(&(ke_date[0])))=temp;
			
		temp=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MONTH;
		gran_char(&temp,1,12);
		*(((char*)(&(ke_date[0])))+1)=temp;
		
		temp=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->DOM;
		gran_char(&temp,1,31);
		*(((char*)(&(ke_date[0])))+2)=temp;			
				
		*(((char*)(&(ke_date[0])))+3)=*((char*)&temp_temp);
		lc640_write_long(0x10+500+150+8,ke_date[0]);

		temp=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->HOUR;
		gran_char(&temp,0,23);
		*((char*)(&(ke_date[1])))=temp;
               
		temp=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MIN;
		gran_char(&temp,0,59);
		*(((char*)(&(ke_date[1])))+1)=temp;
	          
		temp=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->SEC;
		gran_char(&temp,0,59);
		*(((char*)(&(ke_date[1])))+2)=temp;
			
		*(((char*)(&(ke_date[1])))+3)=*(((char*)&temp_temp)+1);
		lc640_write_long(0x10+500+150+12,ke_date[1]);
		}
	bat_hndl_zvu_ke_init=1;
	}
}


#line 1102 "control.c"



void ke_drv(void)
{
static char ke_drv_cnt_10s;
static short i_bat_buff[6];
static char i_bat_buff_cnt;
short ke_drv_i_temp;
short ke_drv_i_avg;
char i;
short ke_drv_i_temp_temp;
const long bat_hndl_t_razr_const[7]={600L,1800L,3600L,10800L,18000L,36000L,72000L};

if(bat_hndl_zvu_ke_init==1)	  
	{
	
	i_bat_buff[0]=0;
	i_bat_buff[0]=0;
	i_bat_buff[0]=0;
	i_bat_buff[0]=0;
	i_bat_buff[0]=0;
	i_bat_buff[0]=0;
	i_bat_buff_cnt=0;

	bat_hndl_zvu_ke_init=0;
	}




if(spc_stat==spcKE)
	{
	ke_drv_i_temp=-Ib_ips_termokompensat/10;
	if(ke_drv_i_temp<0)ke_drv_i_temp=0;

	bat_hndl_plazma[3]=ke_drv_cnt_10s;
	if(++ke_drv_cnt_10s>10)
		{
		ke_drv_cnt_10s=0;

		i_bat_buff_cnt++;
		if(i_bat_buff_cnt>=6)i_bat_buff_cnt=0;
		bat_hndl_plazma[2]=i_bat_buff_cnt;
		i_bat_buff[i_bat_buff_cnt]=ke_drv_i_temp;
		ke_drv_i_temp_temp=0;
		for(i=0;i<6;i++)
			{
			ke_drv_i_temp_temp+=i_bat_buff[i];
			}
		ke_drv_i_avg=ke_drv_i_temp_temp/6;

		I_from_t_table[0]=BAT_C_POINT_1_6*6; 	
		I_from_t_table[1]=BAT_C_POINT_1_2*2; 	
		I_from_t_table[2]=BAT_C_POINT_1; 		
		I_from_t_table[3]=BAT_C_POINT_3/3; 		
		I_from_t_table[4]=BAT_C_POINT_5/5; 		
		I_from_t_table[5]=BAT_C_POINT_10/10; 	
		I_from_t_table[6]=BAT_C_POINT_20/20; 	
		
		U_end_from_i_table[0]=BAT_U_END_1_6;	
		U_end_from_i_table[1]=BAT_U_END_1_2;	
		U_end_from_i_table[2]=BAT_U_END_1;		
		U_end_from_i_table[3]=BAT_U_END_3;		
		U_end_from_i_table[4]=BAT_U_END_5;		
		U_end_from_i_table[5]=BAT_U_END_10;		
		U_end_from_i_table[6]=BAT_U_END_20;		

		bat_hndl_plazma[1]=ke_drv_i_avg;
		bat_hndl_i_temp=ke_drv_i_avg;
		
		for(i=0;i<7;i++)
			{
			if(bat_hndl_i_temp>=I_from_t_table[i])
				{
				break;
				}
			}

		bat_hndl_plazma[0]=i;

		 if(i==0) bat_hndl_t_razr_ke=bat_hndl_t_razr_const[0];
		 else if((i>=1)&&(i<7))
		 	{
			short i1,i2;
			i1=I_from_t_table[i-1]-bat_hndl_i_temp;
			i2=I_from_t_table[i-1]-I_from_t_table[i];
			bat_hndl_t_razr_ke=bat_hndl_t_razr_const[i]-bat_hndl_t_razr_const[i-1];
			bat_hndl_t_razr_ke*=(long)i1;
			bat_hndl_t_razr_ke/=(long)i2;
			bat_hndl_t_razr_ke+=bat_hndl_t_razr_const[i-1];
			}
		else if(i>=7)
			{
			bat_hndl_t_razr_ke=bat_hndl_t_razr_const[6];
			}

		 if(i==0) bat_hndl_u_end=U_end_from_i_table[0];
		 else if((i>=1)&&(i<7))
		 	{
			long u1,tempL;

			tempL=(long)U_end_from_i_table[i]-(long)U_end_from_i_table[i-1];
			u1=bat_hndl_t_razr_ke-bat_hndl_t_razr_const[i-1];
			tempL*=u1;
			u1=bat_hndl_t_razr_const[i]-bat_hndl_t_razr_const[i-1];
			tempL/=u1;
			tempL+=(long)U_end_from_i_table[i-1];
			bat_hndl_u_end=(short)tempL;
			}
		else if(i>=7)
			{
			bat_hndl_u_end=U_end_from_i_table[6];
			}

		}



	if(spc_phase==0)
		{
		
		mess_send(205,206,0xffff,20);

		bat[0]._zar_cnt_ke+=abs(bat[spc_bat]._Ib);
	    	
		if(bat[0]._zar_cnt_ke>=36000L)
			{
			bat[0]._zar_cnt_ke-=36000L;
			lc640_write_int(ADR_EE_BAT_ZAR_CNT_KE[0],lc640_read_int(ADR_EE_BAT_ZAR_CNT_KE[0])+1);
			}

		bat[0]._time_min_cnt_ke++;
		if(bat[0]._time_min_cnt_ke>=60)
			{
			lc640_write_int(0x10+350+88,lc640_read_int(0x10+350+88)+1);
			bat[0]._time_min_cnt_ke=0;
			}
		}

	else if(spc_phase==1)
		{
		
		}

	if(out_U<bat_hndl_u_end)
		{
		cnt_end_ke++;
		if(cnt_end_ke>=30)
			{
			
			if((spc_stat==spcKE)&&(spc_phase==0))
				{
				lc640_write_int(ADR_EE_BAT_C_REAL[0],lc640_read_int(ADR_EE_BAT_ZAR_CNT_KE[0]));
				ke_zvu_mem_hndl(0,lc640_read_int(ADR_EE_BAT_ZAR_CNT_KE[0]),lc640_read_int(0x10+350+88));
				lc640_write_int(ADR_EE_BAT_ZAR_CNT[0],0);
				cntrl_stat=50;
				cntrl_stat_old=50;
				}

			spc_stat=spcOFF;
			__ee_spc_stat=spcOFF;
			lc640_write_int(0x10+500+150,spcOFF);

			}
		}
	else cnt_end_ke=0;

	}
			
}


#line 1342 "control.c"


char vz_start(char hour)
{          
char out;
out=0;
if((spc_stat==spcOFF)&&(speedChrgBlckStat!=1))
	{
	spc_stat=spcVZ;
	__ee_spc_stat=spcVZ; 
	lc640_write_int(0x10+500+150,__ee_spc_stat);   
	vz_cnt_h=hour;
	__ee_vz_cnt=hour*60;
	if(hour==0)__ee_vz_cnt=30;
	lc640_write_int(0x10+500+150+2,__ee_vz_cnt);
	lc640_write_int(0x10+500+150+14,__ee_vz_cnt);	
	vz_cnt_h_=0;
	vz_cnt_s=0;
	vz_cnt_s_=0;
	out=1;
	vz_mem_hndl(1);
	}



return out;
}


void vz_stop(void)
{
if(spc_stat==spcVZ)
     {
vz_mem_hndl(vz_cnt_h);          
vz_cnt_s=0;
vz_cnt_h=0;
vz_cnt_h_=0;
spc_stat=spcOFF;

		__ee_spc_stat=spcOFF;
		lc640_write_int(0x10+500+150,spcOFF);
     }

}



void amper_chas_cnt_drv(void)
{

amper_chas_cnt_drv_summ+=(long)Ib_ips_termokompensat;

if(amper_chas_cnt_drv_summ>=36000L)
	{
	amper_chas_cnt_drv_summ-=36000L;
	lc640_write_int(0x10+350+90,lc640_read_int(0x10+350+90)+1);
	}
else if(amper_chas_cnt_drv_summ<=-36000L)
	{
	amper_chas_cnt_drv_summ+=36000L;
	lc640_write_int(0x10+350+90,lc640_read_int(0x10+350+90)-1);
	}
}



void avz_next_date_hndl(void)
{
if((((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MONTH+AVZ)>12)lc640_write_int(0x10+100+68,((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->YEAR+1);
else lc640_write_int(0x10+100+68,((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->YEAR);



if((((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MONTH+AVZ)>12)lc640_write_int(0x10+100+66,(((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MONTH+AVZ)-12);
else lc640_write_int(0x10+100+66,((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MONTH+AVZ);                                                 



if(((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->DOM>28) lc640_write_int(0x10+100+64,28);
else lc640_write_int(0x10+100+64,((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->DOM);



lc640_write_int(0x10+100+58,((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->HOUR);
lc640_write_int(0x10+100+60,((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MIN);
lc640_write_int(0x10+100+62,((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->SEC);

}


void avz_drv(void)                               
{                
if(AVZ!=AVZ_OFF)
	{
	if((((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->YEAR==YEAR_AVZ)&&(((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MONTH==MONTH_AVZ)&&(((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->DOM==DATE_AVZ)&&(((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->HOUR==HOUR_AVZ)&&(((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MIN==MIN_AVZ)&&(((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->SEC==SEC_AVZ))
		{
		bAVZ=1;
		}
	}
if(bAVZ)
	{
	if(vz_start(AVZ_TIME))
		{
		bAVZ=0;
		avz_next_date_hndl();
		}
	}	

}


void vz_drv(void)
{ 
if(spc_stat==spcVZ)
	{

	if((sk_stat[0]==1)||(VZ_CH_VENT_BLOK==0))

		{

		if(vz_cnt_s_<3600)
			{
			vz_cnt_s_++;
			if(vz_cnt_s_>=3600)
				{
				vz_cnt_s_=0;
				if(vz_cnt_h)
					{
					vz_cnt_h--;
					vz_cnt_h_++;
					}
				}
			}
	
	
		if(vz_cnt_s<60)
			{
			vz_cnt_s++;
			if(vz_cnt_s>=60)
				{
				vz_cnt_s=0;
				
				__ee_vz_cnt--;
				lc640_write_int(0x10+500+150+2,__ee_vz_cnt);
				if((!__ee_vz_cnt)||(speedChrgBlckStat==1))
					{
					spc_stat=spcOFF;
							__ee_spc_stat=spcOFF;
			lc640_write_int(0x10+500+150,spcOFF);
	
	
	
					vz_mem_hndl(0);
					}
				}
			}
		vz_error=0; 
		}

	else 
		{
		vz_error=1; 
		if(((((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->SEC)%10)==0)
			{
			show_mess(	"������������� ����� ",
						"    ������������    ",
						"     ����������     ",
						"    ����������!!!   ",
						5000);			
			}
		}




 

	}


} 



void vz1_drv(void)
{
if(volt_region_cnt)volt_region_cnt--;
if(vz1_stat==vz1sOFF)
	{
	mess_send(210,100,0,20);
	}
if(vz1_stat==vz1sSTEP1)
	{
	if(vz1_stat_old!=vz1_stat)
		{
		vz1_stat_cnt=5;
		}
	if(vz1_stat_cnt)
		{
		vz1_stat_cnt--;
		if(vz1_stat_cnt==0)
			{
			vz1_stat=vz1sERR1; 	
			lc640_write(0x10+350+36,vz1sERR1);
			}
		}
	if(sk_stat[0]==1)
		{
		vz1_stat=vz1sSTEP2;
		lc640_write(0x10+350+36,vz1sSTEP2);
		tree_up(iVZ1_STEP2_2,1,0,0);
		tree_up(iVZ1_STEP2_1,0,0,0);
		ret(1200);
		}
	mess_send(210,100,0,20);
	}

if(vz1_stat==vz1sSTEP2)
	{
	if((vz1_stat_old!=vz1_stat)||(vz1_stat_cnt==0))
		{
		vz1_stat_cnt=15;
		}
	vz1_stat_cnt--;
	mess_send(210,100,0,20);
	}

if(vz1_stat==vz1sSTEP3)
	{
	if((vz1_stat_old!=vz1_stat)||(vz1_stat_cnt==0))
		{
		vz1_stat_cnt=10;

		}
	vz1_stat_cnt--;
	if((vz1_stat_cnt==10)||(vz1_stat_cnt==9))
		{
		show_mess(	"     ��������       ",
					"      �������       ",
					"     ��������       ",
					"    ����������      ",
					5000);
		}
	if(sk_stat[1]==1)
		{
		vz1_stat=vz1sWRK;
		lc640_write(0x10+350+36,vz1sWRK);
		volt_region=1;
		cntrl_stat=0;
		cntrl_stat_new=0;
		cntrl_stat_old=0;
		}
	mess_send(210,100,0,20);
	}

if(vz1_stat==vz1sWRK)
	{
	if(vz1_stat_old!=vz1_stat)
		{
		vz1_wrk_cnt=3600L *((long)UZ_T);
		
		vz1_up_cnt=0L;

		}
	vz1_wrk_cnt--;
	vz1_up_cnt++;

	if(vz1_wrk_cnt==0)
		{
		vz1_stat=vz1sFINE;
		lc640_write(0x10+350+36,vz1sFINE);
		uz_mem_hndl(0);
		}
	if(sk_stat[0]==0)
		{
		vz1_stat=vz1sERR2;
		lc640_write(0x10+350+36,vz1sERR2);
		}
	if(sk_stat[1]==0)
		{
		vz1_stat=vz1sERR3;
		lc640_write(0x10+350+36,vz1sERR3);
		}
	if(((Ibmax/10)>IZMAX_)&&(cntrl_stat<=20)&&(volt_region==1)&&(volt_region_cnt==0))
		{
		volt_region=0;
		cntrl_stat=1000;
		cntrl_stat_new=1000;
		cntrl_stat_old=1000;
		volt_region_cnt=110;
		}
	if(((Ibmax/10)<IZMAX_)&&(cntrl_stat>=1000)&&(volt_region==0)&&(volt_region_cnt==0))
		{
		volt_region=1;
		cntrl_stat=10;
		cntrl_stat_new=10;
		cntrl_stat_old=10;
		volt_region_cnt=10;
		}
	if(volt_region==0) 		mess_send(210,100,0,20);
	else if(volt_region==1) mess_send(210,100,1,20);
	}

if(vz1_stat==vz1sERR1)		
	{
	if((vz1_stat_old!=vz1_stat)||(vz1_stat_cnt==0))
		{
		vz1_stat_cnt=10;
		}
	vz1_stat_cnt--;
	if((vz1_stat_cnt==10)||(vz1_stat_cnt==9))
		{
		show_mess(	"������������� ����� ",
					"   �� ����� ����    ",
					"      �������       ",
					"  ��� ����������!!  ",
					5000);
		}
	mess_send(210,100,0,20);
	}
if(vz1_stat==vz1sERR2)		
	{
	if((vz1_stat_old!=vz1_stat)||(vz1_stat_cnt==0))
		{
		vz1_stat_cnt=10;
		}
	vz1_stat_cnt--;
	if((vz1_stat_cnt==10)||(vz1_stat_cnt==9))
		{
		show_mess(	"������������� ����� ",
					"    ������������    ",
					"     ����������     ",
					"    ����������!!!   ",
					5000);
		}
	if(sk_stat[0]==1)
		{
		vz1_stat=vz1sWRK;
		lc640_write(0x10+350+36,vz1sWRK);
		}
	mess_send(210,100,0,20);
	}

if(vz1_stat==vz1sERR3)		
	{
	if((vz1_stat_old!=vz1_stat)||(vz1_stat_cnt==0))
		{
		vz1_stat_cnt=10;
		}
	vz1_stat_cnt--;
	if((vz1_stat_cnt==10)||(vz1_stat_cnt==9))
		{
		show_mess(	"������������� ����� ",
					"  ����� ���������   ",
					"  ����� ���������   ",
					"*   ��������!!!     ",
					5000);
		}
	if(sk_stat[1]==1)
		{
		vz1_stat=vz1sWRK;
		lc640_write(0x10+350+36,vz1sWRK);
		}
	mess_send(210,100,0,20);
	}
if(vz1_stat==vz1sERR4)		
	{
	if((vz1_stat_old!=vz1_stat)||(vz1_stat_cnt==0))
		{
		vz1_stat_cnt=10;
		}
	vz1_stat_cnt--;
	if((vz1_stat_cnt==10)||(vz1_stat_cnt==9))
		{
		show_mess(	"*    ���������      ",
					"      �������       ",
					"     ��������       ",
					"    ����������      ",
					5000);
		}
	if(sk_stat[1]==0)
		{
		vz1_stat=vz1sOFF;
		lc640_write(0x10+350+36,vz1sOFF);
		vz_stop();

		}
	mess_send(210,100,1,20);
	}
if(vz1_stat==vz1sFINE)		
	{
	if((vz1_stat_old!=vz1_stat)||(vz1_stat_cnt==0))
		{
		vz1_stat_cnt=10;
		}
	vz1_stat_cnt--;
	if((vz1_stat_cnt==10)||(vz1_stat_cnt==9))
		{
		show_mess(	"   �������������    ",
					"       �����        ",
					"      �������       ",
					"     ��������       ",
					3000);
		}
	if((vz1_stat_cnt==6)||(vz1_stat_cnt==5))
		{
		show_mess(	"     ���������      ",
					"      �������       ",
					"     ��������       ",
					"    ����������      ",
					3000);
		}
	if(sk_stat[1]==0)
		{
		vz1_stat=vz1sOFF;
		lc640_write(0x10+350+36,vz1sOFF);
		}
	mess_send(210,100,0,20);
	}

if(vz1_stat==vz1sSTOP)		
	{
	if((vz1_stat_old!=vz1_stat)||(vz1_stat_cnt==0))
		{
		vz1_stat_cnt=10;
		}
	vz1_stat_cnt--;
	if((vz1_stat_cnt==10)||(vz1_stat_cnt==9))
		{
		show_mess(	"   �������������    ",
					"       �����        ",
					"     ���������      ",
					"                    ",
					3000);
		}
	if((vz1_stat_cnt==6)||(vz1_stat_cnt==5))
		{
		show_mess(	"     ���������      ",
					"      �������       ",
					"     ��������       ",
					"    ����������      ",
					3000);
		}
	if(sk_stat[1]==0)
		{
		vz1_stat=vz1sOFF;
		lc640_write(0x10+350+36,vz1sOFF);
		}
	mess_send(210,100,0,20);
	}

vz1_stat_old=vz1_stat;



}



char vz1_start(char hour)
{          
char out;
out=0;
if((spc_stat==spcOFF)&&(speedChrgBlckStat!=1)&&(vz1_stat==vz1sOFF))
	{
	if(vz1_stat==vz1sOFF)
		{
		vz1_stat=vz1sSTEP1;
		lc640_write(0x10+350+36,vz1sSTEP1);
		out=1;
		
		}












 
	}



return out;
}



void vz1_stop(void)
{
if(vz1_stat!=vz1sOFF)
	{
	vz1_stat=vz1sSTOP;
	lc640_write(0x10+350+36,vz1sSTOP);
	}
}



void vz2_drv(void)
{

if(vz2_stat==vz2sSTEP1)
	{
	if(vz2_stat_old!=vz2_stat)
		{
		vz2_stat_cnt=5;
		}
	if(vz2_stat_cnt)
		{
		vz2_stat_cnt--;
		if(vz2_stat_cnt==0)
			{
			vz2_stat=vz2sERR1; 	
			lc640_write(0x10+350+38,vz2sERR1);
			}
		}
	if(sk_stat[0]==1)
		{
		vz2_stat=vz2sSTEP2;
		lc640_write(0x10+350+38,vz2sSTEP2);
		tree_up(iVZ2_STEP2_2,1,0,0);
		tree_up(iVZ2_STEP2_1,0,0,0);
		ret(1200);
		}
	mess_send(210,100,0,20);
	}

if(vz2_stat==vz2sSTEP2)
	{
	if((vz2_stat_old!=vz2_stat)||(vz2_stat_cnt==0))
		{
		vz2_stat_cnt=15;
		}
	vz2_stat_cnt--;
	mess_send(210,100,0,20);
	}

if(vz2_stat==vz2sSTEP3)
	{
	if((vz2_stat_old!=vz2_stat)||(vz2_stat_cnt==0))
		{
		vz2_stat_cnt=10;
		cntrl_stat=0;
		cntrl_stat_new=0;
		cntrl_stat_old=0;
		}
	vz2_stat_cnt--;
	if((vz2_stat_cnt==10)||(vz2_stat_cnt==9))
		{
		show_mess(	"     ��������       ",
					"      �������       ",
					"     ��������       ",
					"    ����������      ",
					5000);
		}
	if(sk_stat[1]==1)
		{
		vz2_stat=vz2sWRK1;
		lc640_write(0x10+350+38,vz2sWRK1);
		volt_region=1;
		cntrl_stat=0;
		cntrl_stat_new=0;
		cntrl_stat_old=0;
		}
	mess_send(210,100,0,20);
	}

if(vz2_stat==vz2sWRK1)
	{

	if(vz2_stat_old!=vz2_stat)
		{
		vz2_wrk_cnt=3600L *((long)FZ_T1);
		
		vz2_up_cnt=0L;

		}
	vz2_wrk_cnt--;
	vz2_up_cnt++;

	if(vz2_wrk_cnt==0)
		{
		vz2_stat=vz2sWRK2;
		lc640_write(0x10+350+38,vz2sWRK2);
		}
	if(sk_stat[0]==0)
		{
		vz2_stat=vz2sERR2;
		lc640_write(0x10+350+38,vz2sERR2);
		}
	if(sk_stat[1]==0)
		{
		vz2_stat=vz2sERR3;
		lc640_write(0x10+350+38,vz2sERR3);
		}
	
	if((out_U<(FZ_U1+30))&&(out_U>(FZ_U1-10)))
		{
		if((Ib_ips_termokompensat/10)<FZ_ISW12)
			{
			if(vz2_stat_ph2_cnt)
				{
				vz2_stat_ph2_cnt--;
				if(vz2_stat_ph2_cnt==0)
					{
					vz2_stat=vz2sWRK2;
					lc640_write(0x10+350+38,vz2sWRK2);
					}
				}
			}
		else
			{
			vz2_stat_ph2_cnt=60;
			}
		}
	else
		{
		vz2_stat_ph2_cnt=60;
		}
	if(((Ibmax/10)>IZMAX_)&&(cntrl_stat<=20)&&(volt_region==1)&&(volt_region_cnt==0))
		{
		volt_region=0;
		cntrl_stat=1000;
		cntrl_stat_new=1000;
		cntrl_stat_old=1000;
		volt_region_cnt=110;
		}
	if(((Ibmax/10)<IZMAX_)&&(cntrl_stat>=1000)&&(volt_region==0)&&(volt_region_cnt==0))
		{
		volt_region=1;
		cntrl_stat=10;
		cntrl_stat_new=10;
		cntrl_stat_old=10;
		volt_region_cnt=10;
		}
	if(volt_region==0) 		mess_send(210,100,0,20);
	else if(volt_region==1) mess_send(210,100,1,20);
	}

if(vz2_stat==vz2sWRK2)
	{
	if(vz2_stat_old!=vz2_stat)
		{
		vz2_wrk_cnt=3600L *((long)FZ_T2);
		
		
		}
	vz2_wrk_cnt--;
	vz2_up_cnt++;

	if(vz2_wrk_cnt==0)
		{
		vz2_stat=vz2sFINE;
		lc640_write(0x10+350+38,vz2sFINE);
		fz_mem_hndl(0);
		}
	if(sk_stat[0]==0)
		{
		vz2_stat=vz2sERR5;
		lc640_write(0x10+350+38,vz2sERR5);
		}
	if(sk_stat[1]==0)
		{
		vz2_stat=vz2sERR6;
		lc640_write(0x10+350+38,vz2sERR6);
		}
	if(((Ibmax/10)>IZMAX_)&&(cntrl_stat<=20)&&(volt_region==1)&&(volt_region_cnt==0))
		{
		volt_region=0;
		cntrl_stat=1000;
		cntrl_stat_new=1000;
		cntrl_stat_old=1000;
		volt_region_cnt=110;
		}
	if(((Ibmax/10)<IZMAX_)&&(cntrl_stat>=1000)&&(volt_region==0)&&(volt_region_cnt==0))
		{
		volt_region=1;
		cntrl_stat=10;
		cntrl_stat_new=10;
		cntrl_stat_old=10;
		volt_region_cnt=10;
		}
	if(volt_region==0) 		mess_send(210,100,0,20);
	else if(volt_region==1) mess_send(210,100,1,20);
	}

if(vz2_stat==vz2sERR1)		
	{
	if((vz2_stat_old!=vz2_stat)||(vz2_stat_cnt==0))
		{
		vz2_stat_cnt=10;
		}
	vz2_stat_cnt--;
	if((vz2_stat_cnt==10)||(vz2_stat_cnt==9))
		{
		show_mess(	" ����������� �����  ",
					"   �� ����� ����    ",
					"      �������       ",
					"  ��� ����������!!  ",
					5000);
		}
	mess_send(210,100,0,20);
	}
if(vz2_stat==vz2sERR2)		
	{
	if((vz2_stat_old!=vz2_stat)||(vz2_stat_cnt==0))
		{
		vz2_stat_cnt=10;
		}
	vz2_stat_cnt--;
	if((vz2_stat_cnt==10)||(vz2_stat_cnt==9))
		{
		show_mess(	" ����������� �����  ",
					"    ������������    ",
					"     ����������     ",
					"    ����������!!!   ",
					5000);
		}
	if(sk_stat[0]==1)
		{
		vz2_stat=vz2sWRK1;
		lc640_write(0x10+350+38,vz2sWRK1);
		}
	mess_send(210,100,0,20);
	}

if(vz2_stat==vz2sERR3)		
	{
	if((vz2_stat_old!=vz2_stat)||(vz2_stat_cnt==0))
		{
		vz2_stat_cnt=10;
		}
	vz2_stat_cnt--;
	if((vz2_stat_cnt==10)||(vz2_stat_cnt==9))
		{
		show_mess(	" ����������� �����  ",
					"  ����� ���������   ",
					"  ����� ���������   ",
					"    ��������!!!     ",
					5000);
		}
	if(sk_stat[1]==1)
		{
		vz2_stat=vz2sWRK1;
		lc640_write(0x10+350+38,vz2sWRK1);
		}
	mess_send(210,100,0,20);
	}
if(vz2_stat==vz2sERR5)		
	{
	if((vz2_stat_old!=vz2_stat)||(vz2_stat_cnt==0))
		{
		vz2_stat_cnt=10;
		}
	vz2_stat_cnt--;
	if((vz2_stat_cnt==10)||(vz2_stat_cnt==9))
		{
		show_mess(	" ����������� �����  ",
					"    ������������    ",
					"     ����������     ",
					"    ����������!!!   ",
					5000);
		}
	if(sk_stat[0]==1)
		{
		vz2_stat=vz2sWRK2;
		lc640_write(0x10+350+38,vz2sWRK2);
		}
	mess_send(210,100,0,20);
	}

if(vz2_stat==vz2sERR6)		
	{
	if((vz2_stat_old!=vz2_stat)||(vz2_stat_cnt==0))
		{
		vz2_stat_cnt=10;
		}
	vz2_stat_cnt--;
	if((vz2_stat_cnt==10)||(vz2_stat_cnt==9))
		{
		show_mess(	" ����������� �����  ",
					"  ����� ���������   ",
					"  ����� ���������   ",
					"    ��������!!!     ",
					5000);
		}
	if(sk_stat[1]==1)
		{
		vz2_stat=vz2sWRK2;
		lc640_write(0x10+350+38,vz2sWRK2);
		}
	mess_send(210,100,0,20);
	}
if(vz2_stat==vz2sERR4)		
	{
	if((vz2_stat_old!=vz2_stat)||(vz2_stat_cnt==0))
		{
		vz2_stat_cnt=10;
		}
	vz2_stat_cnt--;
	if((vz2_stat_cnt==10)||(vz2_stat_cnt==9))
		{
		show_mess(	"     ���������      ",
					"      �������       ",
					"     ��������       ",
					"    ����������      ",
					5000);
		}
	if(sk_stat[1]==0)
		{
		vz2_stat=vz2sOFF;
		lc640_write(0x10+350+38,vz2sOFF);
		vz_stop();

		}
	mess_send(210,100,0,20);
	}
if(vz2_stat==vz2sFINE)		
	{
	if((vz2_stat_old!=vz2_stat)||(vz2_stat_cnt==0))
		{
		vz2_stat_cnt=10;
		}
	vz2_stat_cnt--;
	if((vz2_stat_cnt==10)||(vz2_stat_cnt==9))
		{
		show_mess(	"    �����������     ",
					"       �����        ",
					"      �������       ",
					"     ��������       ",
					3000);
		}
	if((vz2_stat_cnt==6)||(vz2_stat_cnt==5))
		{
		show_mess(	"     ���������      ",
					"      �������       ",
					"     ��������       ",
					"    ����������      ",
					3000);
		}
	if(sk_stat[1]==0)
		{
		vz2_stat=vz2sOFF;
		lc640_write(0x10+350+38,vz2sOFF);
		}
	mess_send(210,100,0,20);
	}
if(vz2_stat==vz2sSTOP)		
	{
	if((vz2_stat_old!=vz2_stat)||(vz2_stat_cnt==0))
		{
		vz2_stat_cnt=10;
		}
	vz2_stat_cnt--;
	if((vz2_stat_cnt==10)||(vz2_stat_cnt==9))
		{
		show_mess(	"    �����������     ",
					"       �����        ",
					"     ���������      ",
					"                    ",
					3000);
		}
	if((vz2_stat_cnt==6)||(vz2_stat_cnt==5))
		{
		show_mess(	"     ���������      ",
					"      �������       ",
					"     ��������       ",
					"    ����������      ",
					3000);
		}
	if(sk_stat[1]==0)
		{
		vz2_stat=vz2sOFF;
		lc640_write(0x10+350+38,vz2sOFF);
		}
	mess_send(210,100,0,20);
	}
vz2_stat_old=vz2_stat;

}



char vz2_start(char hour)
{          
char out;
out=0;
if((spc_stat==spcOFF)&&(speedChrgBlckStat!=1)&&(vz1_stat==vz1sOFF))
	{
	if(vz1_stat==vz1sOFF)
		{
		vz2_stat=vz2sSTEP1;
		lc640_write(0x10+350+38,vz2sSTEP1);
		out=1;
		
		}












 
	}



return out;
}


void vz2_stop(void)
{
if(vz2_stat!=vz2sOFF)
	{
	vz2_stat=vz2sSTOP;
	lc640_write(0x10+350+38,vz2sSTOP);
	}
}


void kb_init(void)
{
main_kb_cnt=(TBAT*60)-60 ;
}


void kb_hndl(void)
{

static signed short ibat[2],ibat_[2];



if(((++main_kb_cnt>=TBAT*60)&&(TBAT)))

	{
	main_kb_cnt=0;
	
	kb_start[0]=0;
	kb_start[1]=0;
	kb_start_ips=0;

	if( (BAT_IS_ON[0]==bisON) && (bat[0]._Ub>80) && ( (abs(bat[0]._Ib)<IKB) || (bat[0]._av&1) ) ) kb_start[0]=1;
	if( (BAT_IS_ON[1]==bisON) && (bat[1]._Ub>80) && ( (abs(bat[1]._Ib)<IKB) || (bat[1]._av&1) ) ) kb_start[1]=1;

	if( (!ips_bat_av_vzvod)                      && ((abs(Ib_ips_termokompensat)<IKB) || (bat_ips._av&1) ) ) kb_start_ips=1;

	if( (net_av) || (num_of_wrks_bps==0) || ( (spc_stat!=spcOFF) && (spc_stat!=spcVZ) ) 

	  ||(((vz1_stat!=vz1sOFF)||(vz2_stat!=vz2sOFF))&&SMART_SPC)

	  ||(sp_ch_stat!=scsOFF) 	)
 
		{
		kb_start[0]=0;
		kb_start[1]=0;
		kb_start_ips=0;
		}

	if((kb_start[0]==1)||(kb_start[1]==1)||(kb_start_ips==1))
		{
		kb_cnt_1lev=10;
		}
	else kb_cnt_1lev=0;
	}

if(kb_cnt_1lev)
	{
	kb_cnt_1lev--;

	if(kb_cnt_1lev>5)mess_send(225,100,30,15);
	else if(kb_cnt_1lev>0) mess_send(225,105,30,15);


	if(kb_cnt_1lev==5)
		{
		ibat[0]=abs(bat[0]._Ib);
		ibat[1]=abs(bat[1]._Ib);
		ibat_ips=abs(Ib_ips_termokompensat);
		}
	
	if(kb_cnt_1lev==0)
		{
		ibat_[0]=abs(bat[0]._Ib);
		ibat_[1]=abs(bat[1]._Ib);
		ibat_ips_=abs(Ib_ips_termokompensat);

		kb_cnt_2lev=0;


		if(( (ibat[0]+ibat_[0]) < IKB )&& (kb_start[0]==1))
			{
			kb_cnt_2lev=10;  
			}
		else if(bat[0]._Ub>200)
			{
			kb_start[0]=0;
			avar_bat_hndl(0,0);
			}
		
		if(( (ibat[1]+ibat_[1]) < IKB ) && (kb_start[1]==1))
			{
			kb_cnt_2lev=10;     
			}
		else  if(bat[1]._Ub>200)
			{
			kb_start[1]=0;
			avar_bat_hndl(1,0);
			}

		if(( (ibat_ips+ibat_ips_) < IKB ) && (kb_start_ips==1))
			{
			if(KB_ALGORITM==1)
				{
				avar_bat_ips_hndl(1);
				kb_start_ips=0;
				}
			else
				{
				kb_cnt_2lev=10;     
				}
			}

		}	


	}
else if(kb_cnt_2lev)
	{
	kb_cnt_2lev--;

	if(kb_cnt_2lev>5)mess_send(225,100,200,15);
	else if(kb_cnt_2lev>0) mess_send(225,105,200,15);


	if(kb_cnt_2lev==5)
		{
		ibat[0]=abs(bat[0]._Ib);
		ibat[1]=abs(bat[1]._Ib);
		ibat_ips=abs(Ib_ips_termokompensat);
		}
	
	if(kb_cnt_2lev==0)
		{
		ibat_[0]=abs(bat[0]._Ib);
		ibat_[1]=abs(bat[1]._Ib);
		ibat_ips_=abs(Ib_ips_termokompensat);

		kb_full_ver=0;

		if(( (ibat[0]+ibat_[0]) < IKB ) && (kb_start[0]==1))
			{
			kb_full_ver=1;  
			}
		else if(bat[0]._Ub>200)			
			{
			kb_start[0]=0;
			avar_bat_hndl(0,0);
			}

		if(( (ibat[1]+ibat_[1]) < IKB ) && (kb_start[1]==1))
			{
			kb_full_ver=1;     
			}
		else	if(bat[1]._Ub>200)		
			{
			kb_start[1]=0;
			avar_bat_hndl(1,0);
			}

		if(( (ibat_ips+ibat_ips_) < IKB )  && (kb_start_ips==1))
			{
			if(KB_ALGORITM==2)
				{
				avar_bat_ips_hndl(1);
				kb_start_ips=0;
				}
			else
				{
				kb_full_ver=1;     
				}
			}

		}	
	}

else if(kb_full_ver)
	{
	
	mess_send(225,110,0,15);

	if( abs(bat[0]._Ib) > IKB ) 
		{
		if(kb_start[0]==1)
			{
			kb_start[0]=0;
			avar_bat_hndl(0,0);
			}
		}

	if( abs(bat[1]._Ib) > IKB ) 
		{
		if(kb_start[1]==1)
			{
			kb_start[1]=0;
			avar_bat_hndl(1,0);
			}
		}

	if( abs(Ib_ips_termokompensat) > IKB ) 
		{
		if(kb_start_ips==1)
			{
			kb_start_ips=0;
			avar_bat_ips_hndl(0);
			}
		}


	if ((kb_start[0]==0) && (kb_start[1]==0) && (kb_start_ips==0)) 
		{
		kb_full_ver=0;
		}

	if(( (mess_find(230))	&& (mess_data[0]==231) ) || (load_U<(USIGN*10)) )
		{
		kb_full_ver=0;
		if((kb_start[0]==1)&&((load_I>(2*IKB)/10))&&(!(bat[0]._av&0x01))) avar_bat_hndl(0,1);
		if((kb_start[1]==1)&&((load_I>(2*IKB)/10))&&(!(bat[1]._av&0x01))) avar_bat_hndl(1,1);

		if((kb_start_ips==1)&&((load_I>(2*IKB)/10))&&(!(bat_ips._av&0x01))) avar_bat_ips_hndl(1);

		}
	}

}





void samokalibr_init(void)
{
samokalibr_cnt=1785;
}

void samokalibr_hndl(void)
{
#line 2521 "control.c"
}




void ubat_old_drv(void)
{        
bat_u_old_cnt++;
gran_ring(&bat_u_old_cnt,0,8);

bat[0]._u_old[bat_u_old_cnt]=bat[0]._Ub;
bat[1]._u_old[bat_u_old_cnt]=bat[1]._Ub;
}


void unet_drv(void)
{
if(net_av_2min_timer)net_av_2min_timer--;

if(net_U<UMN)
	{
	if((unet_drv_cnt<10)&&(main_1Hz_cnt>15))
		{
		unet_drv_cnt++;
		if(unet_drv_cnt>=10)
			{
			net_Ustore=net_U;
		 	avar_unet_hndl(1);
			
			}
		}
	else if(unet_drv_cnt>=10)unet_drv_cnt=10;

	if(net_U<net_Ustore) net_Ustore=net_U;	
	}

else if(net_U>UMN)
	{                 
	if(unet_drv_cnt)
		{
		unet_drv_cnt--;
		if(unet_drv_cnt<=0)
			{
			avar_unet_hndl(0);
			avar_bps_reset_cnt=10;
			}
		}
	else if(unet_drv_cnt<0)unet_drv_cnt=0;
	
	}


if(net_Umax>UMAXN) 
	{
	if((unet_max_drv_cnt<10)&&(main_1Hz_cnt>15))
		{
		unet_max_drv_cnt++;
		if(unet_max_drv_cnt>=10)
			{
			net_Ustore_max=net_Umax; 
		 	avar_unet_hndl(2);
			
			}
		}
	else if(unet_max_drv_cnt>=10)unet_max_drv_cnt=10;

	if(net_Umax>net_Ustore_max) net_Ustore_max=net_Umax; 
	}

else if(net_Umax<UMAXN) 
	{                 
	if(unet_max_drv_cnt)
		{
		unet_max_drv_cnt--;
		if(unet_max_drv_cnt<=0)
			{
			avar_unet_hndl(4); 
			avar_bps_reset_cnt=10;
			}
		}
	else if(unet_max_drv_cnt<0)unet_max_drv_cnt=0;
	
	}

if(avar_bps_reset_cnt)	avar_bps_reset_cnt--;
}



void matemat(void)
{

signed long temp_SL ;
char  i;


#line 2624 "control.c"

#line 2632 "control.c"

#line 2640 "control.c"

#line 2708 "control.c"

#line 2716 "control.c"

#line 2724 "control.c"

#line 2732 "control.c"

#line 2765 "control.c"


#line 2804 "control.c"





	if(bps[11]._device==dNET_METR)
		{
		net_metr_buff_[0]=((signed short)bps[11]._buff[0])+(((signed short)bps[11]._buff[1])<<8);
		net_metr_buff_[1]=((signed short)bps[11]._buff[2])+(((signed short)bps[11]._buff[3])<<8);
		net_metr_buff_[2]=((signed short)bps[11]._buff[4])+(((signed short)bps[11]._buff[5])<<8);

		temp_SL=(signed long)net_metr_buff_[2];
		temp_SL*=KunetA;
		temp_SL/=6000L;
		net_Ua=(signed short)temp_SL;
	
		temp_SL=(signed long)net_metr_buff_[1];
		temp_SL*=KunetB;
		temp_SL/=6000L;
		net_Ub=(signed short)temp_SL;
	
		temp_SL=(signed long)net_metr_buff_[0];
		temp_SL*=KunetC;
		temp_SL/=6000L;
		net_Uc=(signed short)temp_SL;

		net_F3=((signed short)bps[11]._buff[6])+(((signed short)bps[11]._buff[7])<<8);

		net_U=net_Ua;
		if(net_Ub<net_U)net_U=net_Ub;
		if(net_Uc<net_U)net_U=net_Uc;
		
		net_Umax=net_Ua;
		if(net_Ub>net_Umax)net_Umax=net_Ub;
		if(net_Uc>net_Umax)net_Umax=net_Uc;
		
		}
	  else if(AUSW_MAIN==22033)
	{
	temp_SL=(signed long)net_buff_;
	temp_SL*=KunetA;
	temp_SL/=4000L;
	net_Ua=(signed short)temp_SL;

	temp_SL=(signed long)adc_buff_[3];
	temp_SL*=KunetB;
	temp_SL/=6000L;
	net_Ub=(signed short)temp_SL;

	temp_SL=(signed long)adc_buff_[10];
	temp_SL*=KunetC;
	temp_SL/=6000L;
	net_Uc=(signed short)temp_SL;

	net_U=net_Ua;
	if(net_Ub<net_U)net_U=net_Ub;
	if(net_Uc<net_U)net_U=net_Uc;
	
	net_Umax=net_Ua;
	if(net_Ub>net_Umax)net_Umax=net_Ub;
	if(net_Uc>net_Umax)net_Umax=net_Uc;
	
	}
else if((AUSW_MAIN==22063)||(AUSW_MAIN==22023)||(AUSW_MAIN==22043)||(AUSW_MAIN==22044)||(AUSW_MAIN==22018))
	{
	temp_SL=(signed long)net_buff_;
	temp_SL*=KunetA;
	temp_SL/=40000L;
	net_Ua=(signed short)temp_SL;

	temp_SL=(signed long)adc_buff_[3];
	temp_SL*=KunetB;
	temp_SL/=6000L;
	net_Ub=(signed short)temp_SL;

	temp_SL=(signed long)adc_buff_[10];
	temp_SL*=KunetC;
	temp_SL/=6000L;
	net_Uc=(signed short)temp_SL;

	net_U=net_Ua;
	if(net_Ub<net_U)net_U=net_Ub;
	if(net_Uc<net_U)net_U=net_Uc;
	
	net_Umax=net_Ua;
	if(net_Ub>net_Umax)net_Umax=net_Ub;
	if(net_Uc>net_Umax)net_Umax=net_Uc;
	
	}
else	if((AUSW_MAIN==22010)||(AUSW_MAIN==22011) )
	{
	temp_SL=(signed long)net_buff_;
	temp_SL*=Kunet;
	temp_SL/=35000L;
	net_U=(signed short)temp_SL;
	net_Umax=net_U; 
	}
else
	{
	temp_SL=(signed long)net_buff_;
	temp_SL*=Kunet;

	temp_SL/=500L;



	net_U=(signed short)temp_SL;
	net_Umax=net_U; 
	
	}
if(bps[11]._device!=dNET_METR) net_F3=net_F;


#line 2924 "control.c"

#line 2932 "control.c"


#line 2970 "control.c"


temp_SL=(signed long)adc_buff_[0];
temp_SL*=Kubat[0];
temp_SL/=2000L;
bat[0]._Ub=(signed short)temp_SL;

#line 2984 "control.c"

#line 2992 "control.c"



temp_SL=(signed long)adc_buff_[4];
temp_SL*=Kubatm[0];
temp_SL/=700L;
bat[0]._Ubm=(signed short)temp_SL;

#line 3006 "control.c"

#line 3013 "control.c"

temp_SL=(signed long)adc_buff_[12];
temp_SL*=Kubat[1];
temp_SL/=2000L;
bat[1]._Ub=(signed short)temp_SL;

#line 3025 "control.c"

#line 3032 "control.c"



temp_SL=(signed long)adc_buff_[1];
temp_SL*=Kubatm[1];
temp_SL/=700L;
bat[1]._Ubm=(signed short)temp_SL;
#line 3051 "control.c"

#line 3058 "control.c"



















 



if(!mess_find_unvol(220))
	{
	temp_SL=(signed long)ad7705_buff_[0];
	temp_SL-=(signed long)Kibat0[0];
	temp_SL*=(signed long)Kibat1[0];
	if((AUSW_MAIN==24120)||(AUSW_MAIN==24210))temp_SL/=300L;
	else if((AUSW_MAIN==22010)||(AUSW_MAIN==22035)||(AUSW_MAIN==22033))temp_SL/=2000L;
	else temp_SL/=1000L;




	
	
	
	
	bat[0]._Ib=(signed short)temp_SL;

	temp_SL=(signed long)ad7705_buff[0];
	temp_SL-=(signed long)Kibat0[0];
	temp_SL*=(signed long)Kibat1[0];
	if((AUSW_MAIN==24120)||(AUSW_MAIN==24210))temp_SL/=300L;
	else if((AUSW_MAIN==22010)||(AUSW_MAIN==22035)||(AUSW_MAIN==22033))temp_SL/=2000L;
	else temp_SL/=1000L;



	bat[0]._Ib_=(signed short)temp_SL;


	temp_SL=(signed long)ad7705_buff_[1];
	temp_SL-=(signed long)Kibat0[1];
	temp_SL*=(signed long)Kibat1[1];
	if((AUSW_MAIN==24120)||(AUSW_MAIN==24210))temp_SL/=300L;
	else if((AUSW_MAIN==22010)||(AUSW_MAIN==22035)||(AUSW_MAIN==22033))temp_SL/=2000L;
	else temp_SL/=1000L;
	bat[1]._Ib=(signed short)temp_SL;

	temp_SL=(signed long)ad7705_buff[1];
	temp_SL-=(signed long)Kibat0[1];
	temp_SL*=(signed long)Kibat1[1];
	if((AUSW_MAIN==24120)||(AUSW_MAIN==24210))temp_SL/=300L;
	else if((AUSW_MAIN==22010)||(AUSW_MAIN==22035)||(AUSW_MAIN==22033))temp_SL/=2000L;
	else temp_SL/=1000L;
	bat[1]._Ib_=(signed short)temp_SL;
	}







#line 3143 "control.c"
if((adc_buff_[6]>800)&&(adc_buff_[6]<3800))bat[0]._nd=0;
else bat[0]._nd=1;
temp_SL=(signed long)adc_buff_[6];
temp_SL*=Ktbat[0];
temp_SL/=20000L;
temp_SL-=273L;
bat[0]._Tb=(signed short)temp_SL;


#line 3161 "control.c"
if((adc_buff_[7]>800)&&(adc_buff_[7]<3800))bat[1]._nd=0;
else bat[1]._nd=1;
temp_SL=(signed long)adc_buff_[7];
temp_SL*=Ktbat[1];
temp_SL/=20000L;
temp_SL-=273L;
bat[1]._Tb=(signed short)temp_SL;


#line 3242 "control.c"



temp_SL=(signed long)adc_buff_[2];
temp_SL*=Kuload;
temp_SL/=2000L;
load_U=(signed short)temp_SL;

#line 3257 "control.c"

#line 3265 "control.c"



temp_SL=(signed long)adc_buff_[1];
temp_SL*=Kuout;
if(AUSW_MAIN==22010)temp_SL/=400L;
else temp_SL/=500L;
out_U=(signed short)temp_SL;
load_U=out_U;


temp_SL=(signed long)adc_buff_[2];
temp_SL*=Kubps;
if(AUSW_MAIN==22010)temp_SL/=400L;
else temp_SL/=500L;
bps_U=(signed short)temp_SL;

if(bps_U<100)
	{
	char i;
	for(i=0;i<NUMIST;i++)
		{
		if(bps[i]._Uin>bps_U)bps_U=bps[i]._Uin;
		}
	}


temp_SL=0;
for (i=0;i<NUMIST;i++)
	{
	temp_SL+=((signed long)bps[i]._Ii);
	}
bps_I=(signed short)temp_SL;





#line 3313 "control.c"

if((adc_buff_[5]>800)&&(adc_buff_[5]<3800))ND_EXT[0]=0;
else ND_EXT[0]=1;
temp_SL=(signed long)adc_buff_[5];
temp_SL*=Ktext[0];
temp_SL/=20000L;
temp_SL-=273L;
t_ext[0]=(signed short)temp_SL;







 


#line 3352 "control.c"





if((adc_buff_[6]>800)&&(adc_buff_[6]<3800))ND_EXT[0]=0;
else ND_EXT[0]=1;
temp_SL=(signed long)adc_buff_[6];
temp_SL*=Ktext[0];
temp_SL/=20000L;
temp_SL-=273L;
t_ext[0]=(signed short)temp_SL;


if((adc_buff_[5]>800)&&(adc_buff_[5]<3800))ND_EXT[1]=0;
else ND_EXT[1]=1;
temp_SL=(signed long)adc_buff_[5];
temp_SL*=Ktext[1];
temp_SL/=20000L;
temp_SL-=273L;
t_ext[1]=(signed short)temp_SL;
#line 3397 "control.c"

#line 3419 "control.c"



if(bps[8]._device==dIBAT_METR)
	{
	ibat_metr_buff_[0]=((signed long)bps[8]._buff[0])+(((signed long)bps[8]._buff[1])<<8);
	ibat_metr_buff_[1]=((signed long)bps[8]._buff[2])+(((signed long)bps[8]._buff[3])<<8);
	bIBAT_SMKLBR=((signed short)bps[8]._buff[4])+(((signed short)bps[8]._buff[5])<<8);
	if(bIBAT_SMKLBR) bIBAT_SMKLBR_cnt=50;
	if(!bIBAT_SMKLBR)
		{
		signed long temp_SL;
		temp_SL=(signed long)ibat_metr_buff_[0];
		temp_SL-=(signed long)ibat_metr_buff_[1];
		temp_SL*=(signed long)Kibat1[0];
		if((AUSW_MAIN==22010)||(AUSW_MAIN==22011)||(AUSW_MAIN==22035)||(AUSW_MAIN==22033)||(AUSW_MAIN==22063)||(AUSW_MAIN==22023)||(AUSW_MAIN==22043)||(AUSW_MAIN==22044))temp_SL/=2000L;
	
		Ib_ips_termokompensat =(signed short)temp_SL;
		if(bIBAT_SMKLBR_cnt)
			{
			bIBAT_SMKLBR_cnt--;
			Ib_ips_termokompensat=Ib_ips_termokompensat_temp;
			}
		else 
			{
			Ib_ips_termokompensat_temp=Ib_ips_termokompensat;
			}
		}
	}

bat[0]._Ub=load_U;
if(AUSW_MAIN==22018) Ib_ips_termokompensat=bat[0]._Ib;
else bat[0]._Ib=Ib_ips_termokompensat;





#line 3479 "control.c"

#line 3502 "control.c"


temp_SL=(signed long)adc_buff_ext_[0];
temp_SL*=Kunet_ext[0];
temp_SL/=4000L;
Uvv[0]=(signed short)temp_SL;
if(Uvv[0]<100) Uvv0=Uvv[0];
else Uvv0=net_U;


temp_SL=(signed long)adc_buff_ext_[1];
temp_SL*=Kunet_ext[1];
temp_SL/=4000L;
Uvv[1]=(signed short)temp_SL;



temp_SL=(signed long)eb2_data_short[0];
temp_SL*=Kvv_eb2[0];
temp_SL/=6000L;
Uvv_eb2[0]=(signed short)temp_SL;

temp_SL=(signed long)eb2_data_short[1];
temp_SL*=Kvv_eb2[1];
temp_SL/=6000L;
Uvv_eb2[1]=(signed short)temp_SL;

temp_SL=(signed long)eb2_data_short[2];
temp_SL*=Kvv_eb2[2];
temp_SL/=6000L;
Uvv_eb2[2]=(signed short)temp_SL;


temp_SL=(signed long)eb2_data_short[3];
temp_SL*=Kpes_eb2[0];
temp_SL/=6000L;
Upes_eb2[0]=(signed short)temp_SL;

temp_SL=(signed long)eb2_data_short[4];
temp_SL*=Kpes_eb2[1];
temp_SL/=6000L;
Upes_eb2[1]=(signed short)temp_SL;

temp_SL=(signed long)eb2_data_short[5];
temp_SL*=Kpes_eb2[2];
temp_SL/=6000L;
Upes_eb2[2]=(signed short)temp_SL;



ibt._T[0]=t_ext[1]+273;
ibt._T[1]=t_ext[2]+273;

ibt._nd[0]=ND_EXT[1];
ibt._nd[1]=ND_EXT[2];


if((ibt._nd[0]==0) &&  (ibt._nd[1]==0))
	{
	t_box=((ibt._T[0]+ibt._T[1])/2)-273;
	}
else if((ibt._nd[0]==1) &&  (ibt._nd[1]==0))
	{
	t_box=ibt._T[1]-273;
	}
else if((ibt._nd[0]==0) &&  (ibt._nd[1]==1))
	{
	t_box=ibt._T[0]-273;
	}
else if((ibt._nd[0]==1) &&  (ibt._nd[1]==1))
	{
	if(t_ext_can_nd<5)t_box= t_ext_can;
	else t_box=20;
	}





























































































































































 





if((BAT_IS_ON[0]==bisON)&&(bat[0]._Ub>200)) Ibmax=bat[0]._Ib;
if((BAT_IS_ON[1]==bisON)&&(bat[1]._Ub>200)&&(bat[1]._Ib>bat[0]._Ib)) Ibmax=bat[1]._Ib;


#line 3755 "control.c"

#line 3764 "control.c"








Ibmax=Ib_ips_termokompensat;

for(i=0;i<NUMIST;i++)
	{
	if(bps[i]._cnt<25)
     	{
     	bps[i]._Ii=bps[i]._buff[0]+(bps[i]._buff[1]*256);
     	bps[i]._Uin=bps[i]._buff[2]+(bps[i]._buff[3]*256);
     	bps[i]._Uii=bps[i]._buff[4]+(bps[i]._buff[5]*256);
     	bps[i]._Ti=(signed)(bps[i]._buff[6]);
     	bps[i]._adr_ee=bps[i]._buff[7];
     	bps[i]._flags_tm=bps[i]._buff[8];
	    bps[i]._rotor=bps[i]._buff[10]+(bps[i]._buff[11]*256);    
     	} 
	else 
     	{
     	bps[i]._Uii=0; 
     	bps[i]._Ii=0;
     	bps[i]._Uin=0;
     	bps[i]._Ti=0;
     	bps[i]._flags_tm=0; 
	     bps[i]._rotor=0;    
     	}
     
     }

load_I=0;
#line 3809 "control.c"
load_I=-(bat[0]._Ib/10)-(bat[1]._Ib/10);

Isumm=0;

for(i=0;i<NUMIST;i++)
     {
     if(bps[i]._cnt<5)Isumm+=bps[i]._Ii;
     }  
     
load_I=load_I+Isumm;
if(load_I<0)load_I=0;


load_I=0;

Isumm=0;

for(i=0;i<NUMIST;i++)
     {
     if(bps[i]._cnt<5)Isumm+=bps[i]._Ii;
     }  
     
load_I=load_I+Isumm;
if(load_I<0)load_I=0;







#line 3869 "control.c"


if (NUMINV)
	{
	for(i=0;i<NUMINV;i++)
		{
		if(bps[i+20]._cnt<25)
     		{
     		inv[i]._Ii=bps[i+20]._buff[0]+(bps[i+20]._buff[1]*256);
     		inv[i]._Pio=bps[i+20]._buff[2]+(bps[i+20]._buff[3]*256);
     		inv[i]._Uio=bps[i+20]._buff[4]+(bps[i+20]._buff[5]*256);
     		inv[i]._Ti=(signed)(bps[i+20]._buff[6]);
     		inv[i]._flags_tm=bps[i+20]._buff[7];
     		inv[i]._Uin=bps[i+20]._buff[8]+(bps[i+20]._buff[9]*256);
     		inv[i]._Uil=bps[i+20]._buff[10]+(bps[i+20]._buff[11]*256);
			inv[i]._cnt=0;
			inv[i]._Uoutmin=bps[i+20]._buff[12]; 
			inv[i]._Uoutmax=bps[i+20]._buff[13]; 
			inv[i]._Pnom=bps[i+20]._buff[14]; 
			inv[i]._net_contr_en=bps[i+20]._buff[15];
			inv[i]._pwm_en=bps[i+20]._buff[16];  
			inv[i]._phase_mode=bps[i+20]._buff[17];  
     		} 
		else 
     		{
      		inv[i]._Ii=0;
			inv[i]._Pio=0;
			inv[i]._Uio=0;
     		inv[i]._Ti=0;
     		inv[i]._flags_tm=0; 
     		inv[i]._Uil=0;
     		inv[i]._Uin=0;
			inv[i]._cnt=25; 
			inv[i]._Uoutmin=0; 
			inv[i]._Uoutmax=0; 
			inv[i]._Pnom=0; 
			inv[i]._net_contr_en=0;
			inv[i]._pwm_en=0;   
			   
     		}
     	}
   	}


#line 3932 "control.c"
















 

#line 4054 "control.c"



#line 4272 "control.c"

#line 4327 "control.c"




































 
}


#line 4424 "control.c"


void mnemo_hndl(void)
{
if(((a_ind . i==iMn_220)||(a_ind . i==iMn))&&(a_ind . s_i==0)&&(MNEMO_ON==mnON))
	{
	if(mnemo_cnt)mnemo_cnt--;
	}
else mnemo_cnt=MNEMO_TIME;
}


void apv_start(char in)
{
if(	(bps[in]._apv_timer_1_lev==0)&&
	(bps[in]._apv_cnt_1_lev==0)&&
	(bps[in]._apv_timer_2_lev==0) )
		{
 		bps[in]._apv_timer_1_lev=60;
		bps[in]._apv_cnt_1_lev=3;
		bps[in]._apv_timer_2_lev=(short)(APV_ON2_TIME*3600);
		}
}


void apv_stop(char in)
{
bps[in]._apv_timer_1_lev=0;
bps[in]._apv_cnt_1_lev=0;
bps[in]._apv_timer_2_lev=0;
}


void apv_drv(void)		
{
for(i=0;i<NUMIST;i++)
	{
	if(APV_ON1==apvOFF)		
		{
		bps[i]._apv_timer_1_lev=0;
		bps[i]._apv_cnt_1_lev=0;
		bps[i]._apv_timer_2_lev=0;
		}
	if(APV_ON2==apvOFF)	   
		{
		bps[i]._apv_timer_2_lev=0;
		}

	if(	(bps[i]._apv_timer_1_lev!=0)||	
		(bps[i]._apv_cnt_1_lev!=0)||	
		(bps[i]._apv_timer_2_lev!=0) )		 
			{
			if(bps[i]._state==bsWRK)
				{
				if(bps[i]._apv_succes_timer<60)
					{
					bps[i]._apv_succes_timer++;
					if(bps[i]._apv_succes_timer>=60)
						{
						apv_stop(i);
						}
					}
				}
			else bps[i]._apv_succes_timer=0;
			}

	if(bps[i]._apv_timer_1_lev)
		{
		bps[i]._apv_timer_2_lev=0;
		bps[i]._apv_timer_1_lev--;
		if(bps[i]._apv_timer_1_lev==0)
			{
			if(bps[i]._apv_cnt_1_lev)
				{
				bps[i]._apv_cnt_1_lev--;
				bps[i]._apv_timer_1_lev=60;
				bps[i]._apv_reset_av_timer=2;
				}
			else
				{
				if(APV_ON2==apvON)
					{
					bps[i]._apv_timer_1_lev=0;
					bps[i]._apv_cnt_1_lev=0;
					bps[i]._apv_timer_2_lev=(short)(APV_ON2_TIME*3600);
					}
				}
			
			}
		}
	if(bps[i]._apv_timer_2_lev)
		{
		bps[i]._apv_timer_2_lev--;
		if(bps[i]._apv_timer_2_lev==0)
			{
			bps[i]._apv_cnt_1_lev=2;
			bps[i]._apv_timer_1_lev=60;
			}
		}

	if(bps[i]._apv_reset_av_timer)bps[i]._apv_reset_av_timer--;
	} 














































 		
}


void adc_init(void)
{

((LPC_PINCON_TypeDef *) ((0x40000000UL) + 0x2C000) )->PINSEL1 = ( (((LPC_PINCON_TypeDef *) ((0x40000000UL) + 0x2C000) )->PINSEL1 & ~((0xffffffff>>(32-2))<<(25-16)*2)) | (1 << (25-16)*2) );
((LPC_PINCON_TypeDef *) ((0x40000000UL) + 0x2C000) )->PINSEL1 = ( (((LPC_PINCON_TypeDef *) ((0x40000000UL) + 0x2C000) )->PINSEL1 & ~((0xffffffff>>(32-2))<<(24-16)*2)) | (1 << (24-16)*2) );
((LPC_PINCON_TypeDef *) ((0x40000000UL) + 0x2C000) )->PINSEL1 = ( (((LPC_PINCON_TypeDef *) ((0x40000000UL) + 0x2C000) )->PINSEL1 & ~((0xffffffff>>(32-2))<<(23-16)*2)) | (1 << (23-16)*2) );


((LPC_PINCON_TypeDef *) ((0x40000000UL) + 0x2C000) )->PINMODE1 = ( (((LPC_PINCON_TypeDef *) ((0x40000000UL) + 0x2C000) )->PINMODE1 & ~((0xffffffff>>(32-2))<<(25-16)*2)) | (2 << (25-16)*2) );
((LPC_PINCON_TypeDef *) ((0x40000000UL) + 0x2C000) )->PINMODE1 = ( (((LPC_PINCON_TypeDef *) ((0x40000000UL) + 0x2C000) )->PINMODE1 & ~((0xffffffff>>(32-2))<<(24-16)*2)) | (2 << (24-16)*2) );
((LPC_PINCON_TypeDef *) ((0x40000000UL) + 0x2C000) )->PINMODE1 = ( (((LPC_PINCON_TypeDef *) ((0x40000000UL) + 0x2C000) )->PINMODE1 & ~((0xffffffff>>(32-2))<<(23-16)*2)) | (2 << (23-16)*2) );

((LPC_ADC_TypeDef *) ((0x40000000UL) + 0x34000) )->ADCR = ( (((LPC_ADC_TypeDef *) ((0x40000000UL) + 0x34000) )->ADCR & ~((0xffffffff>>(32-3))<<24)) | (0 << 24) );

((LPC_ADC_TypeDef *) ((0x40000000UL) + 0x34000) )->ADCR = ( (((LPC_ADC_TypeDef *) ((0x40000000UL) + 0x34000) )->ADCR & ~((0xffffffff>>(32-1))<<21)) | (1 << 21) );
((LPC_ADC_TypeDef *) ((0x40000000UL) + 0x34000) )->ADCR = ( (((LPC_ADC_TypeDef *) ((0x40000000UL) + 0x34000) )->ADCR & ~((0xffffffff>>(32-1))<<16)) | (0 << 16) );
((LPC_ADC_TypeDef *) ((0x40000000UL) + 0x34000) )->ADCR = ( (((LPC_ADC_TypeDef *) ((0x40000000UL) + 0x34000) )->ADCR & ~((0xffffffff>>(32-8))<<8)) | (1 << 8) );



	
	 
     
 

((LPC_ADC_TypeDef *) ((0x40000000UL) + 0x34000) )->ADINTEN     =  (1<< 8);       

NVIC_EnableIRQ(ADC_IRQn);              


}


void adc_drv7(void) 
{




adc_self_ch_disp[0]=abs_pal(adc_self_ch_buff[1]-adc_self_ch_buff[0]);
adc_self_ch_disp[1]=abs_pal(adc_self_ch_buff[2]-adc_self_ch_buff[1]);
adc_self_ch_disp[2]=abs_pal(adc_self_ch_buff[2]-adc_self_ch_buff[0]);






if(adc_self_ch_disp[2]<300)
	{
	adc_result=adc_self_ch_buff[2];
	} 
else if(adc_self_ch_disp[1]<300)
	{
	adc_result=adc_self_ch_buff[1];
	}
else if(adc_self_ch_disp[0]<300)
	{
	adc_result=adc_self_ch_buff[0];
	}
    

if(adc_ch_net)
	{

	main_power_buffer[0]+=(long)(adc_result);
	main_power_buffer[1]+=(long)(adc_result);
	main_power_buffer[2]+=(long)(adc_result);
	main_power_buffer[3]+=(long)(adc_result);

	adc_net_buff_cnt++;
	if(adc_net_buff_cnt>=0x1000)
		{
		adc_net_buff_cnt=0;
		}
	if((adc_net_buff_cnt&0x03ff)==0)
		{
#line 4659 "control.c"
		net_buff_=(short)((main_power_buffer[adc_net_buff_cnt>>10])>>8);


		main_power_buffer[adc_net_buff_cnt>>10]=0;
		}


	} 
else if(!adc_ch_net)
	{
	adc_buff[adc_ch][adc_ch_cnt]=(long)adc_result;
	
	if((adc_ch_cnt&0x03)==0)
		{
		long temp_L;
		char i;
		temp_L=0;
		for(i=0;i<16;i++)
			{
			temp_L+=adc_buff[adc_ch][i];
			}
		adc_buff_[adc_ch]= (short)(temp_L>>4);

		
		}
	if(++adc_ch>=16) 
		{
		adc_ch=0;
		adc_ch_cnt++;
		if(adc_ch_cnt>=16)adc_ch_cnt=0;
		}
	}
















 


		  

adc_self_ch_cnt=0;

adc_ch_net++;
adc_ch_net&=1;




if(adc_ch_net)
	{
	
	
	((LPC_ADC_TypeDef *) ((0x40000000UL) + 0x34000) )->ADCR = ( (((LPC_ADC_TypeDef *) ((0x40000000UL) + 0x34000) )->ADCR & ~((0xffffffff>>(32-8))<<0)) | (1<<2 << 0) );
	}
else
	{
	
	
	if(!(adc_ch&(1<<3)))((LPC_ADC_TypeDef *) ((0x40000000UL) + 0x34000) )->ADCR = ( (((LPC_ADC_TypeDef *) ((0x40000000UL) + 0x34000) )->ADCR & ~((0xffffffff>>(32-8))<<0)) | (1<<0 << 0) );
	else 			((LPC_ADC_TypeDef *) ((0x40000000UL) + 0x34000) )->ADCR = ( (((LPC_ADC_TypeDef *) ((0x40000000UL) + 0x34000) )->ADCR & ~((0xffffffff>>(32-8))<<0)) | (1<<1 << 0) );


	((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIODIR = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIODIR & ~((0xffffffff>>(32-1))<<28)) | (1 << 28) );
	((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00020) )->FIODIR = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00020) )->FIODIR & ~((0xffffffff>>(32-1))<<30)) | (1 << 30) );
	((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIODIR = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIODIR & ~((0xffffffff>>(32-1))<<26)) | (1 << 26) );

	if(!(adc_ch&(1<<0)))((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOPIN = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOPIN & ~((0xffffffff>>(32-1))<<28)) | (0 << 28) );
	else 			((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOPIN = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOPIN & ~((0xffffffff>>(32-1))<<28)) | (1 << 28) );

	if(!(adc_ch&(1<<1)))((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00020) )->FIOPIN = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00020) )->FIOPIN & ~((0xffffffff>>(32-1))<<30)) | (0 << 30) );
	else 			((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00020) )->FIOPIN = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00020) )->FIOPIN & ~((0xffffffff>>(32-1))<<30)) | (1 << 30) );

	if(!(adc_ch&(1<<2)))((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIOPIN = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIOPIN & ~((0xffffffff>>(32-1))<<26)) | (0 << 26) );
	else 			((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIOPIN = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIOPIN & ~((0xffffffff>>(32-1))<<26)) | (1 << 26) );
	}
	



((LPC_ADC_TypeDef *) ((0x40000000UL) + 0x34000) )->ADCR |=  (1<<24);

}


void adc_drv6(void) 
{




adc_self_ch_disp[0]=abs_pal(adc_self_ch_buff[1]-adc_self_ch_buff[0]);
adc_self_ch_disp[1]=abs_pal(adc_self_ch_buff[2]-adc_self_ch_buff[1]);
adc_self_ch_disp[2]=abs_pal(adc_self_ch_buff[2]-adc_self_ch_buff[0]);






if(adc_self_ch_disp[2]<300)
	{
	adc_result=adc_self_ch_buff[2];
	} 
else if(adc_self_ch_disp[1]<300)
	{
	adc_result=adc_self_ch_buff[1];
	}
else if(adc_self_ch_disp[0]<300)
	{
	adc_result=adc_self_ch_buff[0];
	}
    

if(adc_ch_net)
	{

	if(adc_window_flag)
		{
		main_power_buffer[0]+=(long)(adc_result>>2);
		main_power_buffer[1]+=(long)(adc_result>>2);
		main_power_buffer[2]+=(long)(adc_result>>2);
		main_power_buffer[3]+=(long)(adc_result>>2);
		}







	if(adc_result<100)
		{
		adc_zero_cnt++;
		}
	else adc_zero_cnt=0;

	if(adc_zero_cnt>=2000)
		{
		adc_zero_cnt=2000;
		main_power_buffer[0]=0;
		main_power_buffer[1]=0;
		main_power_buffer[2]=0;
		main_power_buffer[3]=0;
		net_buff_=0;
		}

	if(adc_zero_cnt==5)
		{
		
		if(adc_window_flag)
			{
			adc_gorb_cnt++;
			if(adc_gorb_cnt>=512)
				{
				adc_gorb_cnt=0;
				
				
			   	}

			if((adc_gorb_cnt&0x007f)==0)
				{
				net_buff_=main_power_buffer[adc_gorb_cnt>>7]>>8;
				main_power_buffer[adc_gorb_cnt>>7]=0;
				}
			}

		
		

		if((adc_window_cnt>150)&&(adc_window_flag))
			{
			adc_window_flag=0;

			
			}
		if((adc_window_cnt>30)&&(adc_window_cnt<70)&&(!adc_window_flag))
			{
			adc_window_flag=1;

			
			
			}
		}
	} 
else if(!adc_ch_net)
	{
	adc_buff[adc_ch][adc_ch_cnt]=(long)adc_result;
	
	if((adc_ch_cnt&0x03)==0)
		{
		long temp_L;
		char i;
		temp_L=0;
		for(i=0;i<16;i++)
			{
			temp_L+=adc_buff[adc_ch][i];
			}
		adc_buff_[adc_ch]= (short)(temp_L>>4);

		
		}
	if(++adc_ch>=16) 
		{
		adc_ch=0;
		adc_ch_cnt++;
		if(adc_ch_cnt>=16)adc_ch_cnt=0;
		}
	}
















 


		  

adc_self_ch_cnt=0;

adc_ch_net++;
adc_ch_net&=1;




if(adc_ch_net)
	{
	
	
	((LPC_ADC_TypeDef *) ((0x40000000UL) + 0x34000) )->ADCR = ( (((LPC_ADC_TypeDef *) ((0x40000000UL) + 0x34000) )->ADCR & ~((0xffffffff>>(32-8))<<0)) | (1<<2 << 0) );
	}
else
	{
	
	
	if(!(adc_ch&(1<<3)))((LPC_ADC_TypeDef *) ((0x40000000UL) + 0x34000) )->ADCR = ( (((LPC_ADC_TypeDef *) ((0x40000000UL) + 0x34000) )->ADCR & ~((0xffffffff>>(32-8))<<0)) | (1<<0 << 0) );
	else 			((LPC_ADC_TypeDef *) ((0x40000000UL) + 0x34000) )->ADCR = ( (((LPC_ADC_TypeDef *) ((0x40000000UL) + 0x34000) )->ADCR & ~((0xffffffff>>(32-8))<<0)) | (1<<1 << 0) );


	((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIODIR = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIODIR & ~((0xffffffff>>(32-1))<<28)) | (1 << 28) );
	((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00020) )->FIODIR = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00020) )->FIODIR & ~((0xffffffff>>(32-1))<<30)) | (1 << 30) );
	((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIODIR = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIODIR & ~((0xffffffff>>(32-1))<<26)) | (1 << 26) );

	if(!(adc_ch&(1<<0)))((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOPIN = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOPIN & ~((0xffffffff>>(32-1))<<28)) | (0 << 28) );
	else 			((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOPIN = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOPIN & ~((0xffffffff>>(32-1))<<28)) | (1 << 28) );

	if(!(adc_ch&(1<<1)))((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00020) )->FIOPIN = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00020) )->FIOPIN & ~((0xffffffff>>(32-1))<<30)) | (0 << 30) );
	else 			((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00020) )->FIOPIN = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00020) )->FIOPIN & ~((0xffffffff>>(32-1))<<30)) | (1 << 30) );

	if(!(adc_ch&(1<<2)))((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIOPIN = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIOPIN & ~((0xffffffff>>(32-1))<<26)) | (0 << 26) );
	else 			((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIOPIN = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIOPIN & ~((0xffffffff>>(32-1))<<26)) | (1 << 26) );
	}
	



((LPC_ADC_TypeDef *) ((0x40000000UL) + 0x34000) )->ADCR |=  (1<<24);

}
 































































 




void avg_hndl(void)
{ 
char i;


if(avg_main_cnt)
	{
	avg_main_cnt--;
	
	return;
	}                 

avg_main_cnt=5;
avg_num=0;

for(i=0;i<NUMIST;i++)
	{
	if((bps[i]._state==bsWRK)&&(bps[i]._cnt<20))avg_num++;
	}



 

if(avg_vektor) avg_vektor=0;
else avg_vektor=1;
	
if(avg_num<2)
	{
	
	return;
	}
	
else
	{
	i_avg_min=5000;
	i_avg_max=0;
	i_avg_summ=0;
	for(i=0;i<NUMIST;i++)
		{
		if(bps[i]._state==bsWRK)
			{
			if(bps[i]._Ii>i_avg_max)i_avg_max=bps[i]._Ii;
			if(bps[i]._Ii<i_avg_min)i_avg_min=bps[i]._Ii;
			
			i_avg_summ+=bps[i]._Ii;
			}
		}
	i_avg=i_avg_summ/avg_num;	
	
	if(i_avg_min==0)i_avg_min=1;

	avg=i_avg_max;
	avg*=100;
	avg/=i_avg_min;

	if(avg>130) bAVG=1;
	if(avg<110) bAVG=0;

	if(bAVG==1)
		{
		for(i=0;i<NUMIST;i++)
			{
			if(bps[i]._state==bsWRK)
				{
				if((bps[i]._Ii>i_avg)&&(!avg_vektor))bps[i]._x_--;
				if((bps[i]._Ii<i_avg)&&(avg_vektor))bps[i]._x_++;
			
				if(bps[i]._x_<-50)bps[i]._x_=-50;
				if(bps[i]._x_>50)bps[i]._x_=50;	
				}
			}		
		}			
	}   	 


avg_hndl_end:
__nop();  
}












 

void rele_hndl(void)
{






((LPC_PINCON_TypeDef *) ((0x40000000UL) + 0x2C000) )->PINSEL0 = ( (((LPC_PINCON_TypeDef *) ((0x40000000UL) + 0x2C000) )->PINSEL0 & ~((0xffffffff>>(32-6*2))<<4*2)) | (0 << 4*2) );
((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIODIR = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIODIR & ~((0xffffffff>>(32-6))<<4)) | (63 << 4) );
((LPC_PINCON_TypeDef *) ((0x40000000UL) + 0x2C000) )->PINSEL7 = ( (((LPC_PINCON_TypeDef *) ((0x40000000UL) + 0x2C000) )->PINSEL7 & ~((0xffffffff>>(32-2))<<(25-16)*2)) | (0 << (25-16)*2) );
((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIODIR = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIODIR & ~((0xffffffff>>(32-1))<<25)) | (1 << 25) );
((LPC_PINCON_TypeDef *) ((0x40000000UL) + 0x2C000) )->PINSEL1 = ( (((LPC_PINCON_TypeDef *) ((0x40000000UL) + 0x2C000) )->PINSEL1 & ~((0xffffffff>>(32-2))<<(29-16)*2)) | (0 << (29-16)*2) );
((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIODIR = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIODIR & ~((0xffffffff>>(32-1))<<29)) | (1 << 29) );






if((((bat[0]._rel_stat)  || (tbatdisable_stat!=tbdsON))&&(tbatdisable_cmnd))	&& (main_1Hz_cnt>5))
	{
	((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET & ~((0xffffffff>>(32-1))<<8)) | (1 << 8) );
	}
else ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR & ~((0xffffffff>>(32-1))<<8)) | (1 << 8) );	  	

if((((bat[1]._rel_stat) || (tbatdisable_stat!=tbdsON))&&(tbatdisable_cmnd))	&& (main_1Hz_cnt>5))
	{
	((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET & ~((0xffffffff>>(32-1))<<6)) | (1 << 6) );
	}
else ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR & ~((0xffffffff>>(32-1))<<6)) | (1 << 6) );


if(mess_find_unvol((210))&&(mess_data[0]==100)) 
	{
	if(mess_data[1]==1)((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET & ~((0xffffffff>>(32-1))<<29)) | (1 << 29) );
	else if(mess_data[1]==0) ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR & ~((0xffffffff>>(32-1))<<29)) | (1 << 29) );
	}
else ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR & ~((0xffffffff>>(32-1))<<29)) | (1 << 29) );


#line 5157 "control.c"

if((mess_find_unvol(210))&&	(mess_data[0]==102))
	{
	if(mess_data[1]==0)				((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIOCLR = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIOCLR & ~((0xffffffff>>(32-1))<<25)) | (1 << 25) ); 
	else 						((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIOSET = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIOSET & ~((0xffffffff>>(32-1))<<25)) | (1 << 25) );
	}
else	if(!(avar_ind_stat&0x00000001))	((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIOCLR = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIOCLR & ~((0xffffffff>>(32-1))<<25)) | (1 << 25) );
else 					  		((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIOSET = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIOSET & ~((0xffffffff>>(32-1))<<25)) | (1 << 25) );


#line 5203 "control.c"


#line 5241 "control.c"





#line 5312 "control.c"

#line 5426 "control.c"

#line 5492 "control.c"

#line 5537 "control.c"

#line 5582 "control.c"



if((AUSW_MAIN==22010)||(AUSW_MAIN==22011))
	{
#line 5596 "control.c"



		if((mess_find_unvol(210))&&	(mess_data[0]==102))
			{
			if(mess_data[1]==0) ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIOCLR = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIOCLR & ~((0xffffffff>>(32-1))<<25)) | (1 << 25) );
			else ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIOSET = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIOSET & ~((0xffffffff>>(32-1))<<25)) | (1 << 25) );
			}
		else	if(!(avar_ind_stat&0x00000001)) ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIOCLR = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIOCLR & ~((0xffffffff>>(32-1))<<25)) | (1 << 25) );
		else ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIOSET = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIOSET & ~((0xffffffff>>(32-1))<<25)) | (1 << 25) );


#line 5617 "control.c"


	if((mess_find_unvol(210))&&	(mess_data[0]==106))
		{
		if(mess_data[1]==0) ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR & ~((0xffffffff>>(32-1))<<7)) | (1 << 7) );
		else if(mess_data[1]==1) ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET & ~((0xffffffff>>(32-1))<<7)) | (1 << 7) );
	     }
	else 
		{
		if(!(avar_ind_stat&0x100007f8)) ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR & ~((0xffffffff>>(32-1))<<7)) | (1 << 7) );
	     else ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET & ~((0xffffffff>>(32-1))<<7)) | (1 << 7) );
		} 

	

	if((mess_find_unvol(210))&&	(mess_data[0]==103))
		{
		if(mess_data[1]==0) ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR & ~((0xffffffff>>(32-1))<<4)) | (1 << 4) );
		else if(mess_data[1]==1) ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET & ~((0xffffffff>>(32-1))<<4)) | (1 << 4) );
     	}
	else 
		{
		if(!(ips_bat_av_stat)) ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR & ~((0xffffffff>>(32-1))<<4)) | (1 << 4) );
     	else ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET & ~((0xffffffff>>(32-1))<<4)) | (1 << 4) );
		}
#line 5655 "control.c"
	}

else	if(AUSW_MAIN==22023)
	{

	
	if((mess_find_unvol(210))&&	(mess_data[0]==103))
		{
		if(mess_data[1]==0) ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR & ~((0xffffffff>>(32-1))<<4)) | (1 << 4) );
		else if(mess_data[1]==1) ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET & ~((0xffffffff>>(32-1))<<4)) | (1 << 4) );
     	}
	else 
		{
		if(!(ips_bat_av_stat)) ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR & ~((0xffffffff>>(32-1))<<4)) | (1 << 4) );
     	else ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET & ~((0xffffffff>>(32-1))<<4)) | (1 << 4) );
		} 


	
#line 5684 "control.c"
	if((mess_find_unvol(210))&&	(mess_data[0]==102))
		{
		if(mess_data[1]==0) ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET & ~((0xffffffff>>(32-1))<<25)) | (1 << 25) );
		else ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR & ~((0xffffffff>>(32-1))<<25)) | (1 << 25) );
		}
	else	if(!(avar_ind_stat&0x00000001)) ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00020) )->FIOSET = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00020) )->FIOSET & ~((0xffffffff>>(32-1))<<25)) | (1 << 25) );
	else ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIOSET = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIOSET & ~((0xffffffff>>(32-1))<<25)) | (1 << 25) );


	if((mess_find_unvol(210))&&	(mess_data[0]==106))
		{
		if(mess_data[1]==0) ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR & ~((0xffffffff>>(32-1))<<7)) | (1 << 7) );
		else if(mess_data[1]==1) ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET & ~((0xffffffff>>(32-1))<<7)) | (1 << 7) );
	     }
	else 
		{
		if(!(avar_ind_stat&0x100007f8)) ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR & ~((0xffffffff>>(32-1))<<7)) | (1 << 7) );
	     else ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET & ~((0xffffffff>>(32-1))<<7)) | (1 << 7) );
		} 
	}
else	if((AUSW_MAIN==22043)||(AUSW_MAIN==22044))
	{
	
	if((mess_find_unvol(210))&&	(mess_data[0]==103))
		{
		if(mess_data[1]==0) ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR & ~((0xffffffff>>(32-1))<<4)) | (1 << 4) );
		else if(mess_data[1]==1) ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET & ~((0xffffffff>>(32-1))<<4)) | (1 << 4) );
     	}
	else 
		{
		if(!(ips_bat_av_stat)) ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR & ~((0xffffffff>>(32-1))<<4)) | (1 << 4) );
     	else ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET & ~((0xffffffff>>(32-1))<<4)) | (1 << 4) );
		} 
	
#line 5728 "control.c"
	if((mess_find_unvol(210))&&	(mess_data[0]==102))
		{
		if(mess_data[1]==0) ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET & ~((0xffffffff>>(32-1))<<25)) | (1 << 25) );
		else ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR & ~((0xffffffff>>(32-1))<<25)) | (1 << 25) );
		}
	else	if(!(avar_ind_stat&0x00000001)) ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00020) )->FIOSET = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00020) )->FIOSET & ~((0xffffffff>>(32-1))<<25)) | (1 << 25) );
	else ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIOSET = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIOSET & ~((0xffffffff>>(32-1))<<25)) | (1 << 25) );


	if((mess_find_unvol(210))&&	(mess_data[0]==106))
		{
		if(mess_data[1]==0) ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR & ~((0xffffffff>>(32-1))<<7)) | (1 << 7) );
		else if(mess_data[1]==1) ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET & ~((0xffffffff>>(32-1))<<7)) | (1 << 7) );
	     }
	else 
		{
		if(!(avar_ind_stat&0x100007f8)) ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR & ~((0xffffffff>>(32-1))<<7)) | (1 << 7) );
	     else ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET & ~((0xffffffff>>(32-1))<<7)) | (1 << 7) );
		}
	
	if((mess_find_unvol(210))&&	(mess_data[0]==106))
		{
		if(mess_data[1]==0) ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR & ~((0xffffffff>>(32-1))<<5)) | (1 << 5) );
		else if(mess_data[1]==1) ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET & ~((0xffffffff>>(32-1))<<5)) | (1 << 5) );
     	}
	else 
		{
		if(!(avar_ind_stat&0x100007f8)) ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR & ~((0xffffffff>>(32-1))<<5)) | (1 << 5) );
     	else ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET & ~((0xffffffff>>(32-1))<<5)) | (1 << 5) );
		} 
	}
else	if((AUSW_MAIN==22033)||(AUSW_MAIN==22018))
	{
#line 5771 "control.c"
	if((mess_find_unvol(210))&&	(mess_data[0]==102))
		{
		if(mess_data[1]==0) ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIOCLR = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIOCLR & ~((0xffffffff>>(32-1))<<25)) | (1 << 25) );
		else ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIOSET = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIOSET & ~((0xffffffff>>(32-1))<<25)) | (1 << 25) );
		}
	else	if(!(avar_ind_stat&0x00000001)) ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIOCLR = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIOCLR & ~((0xffffffff>>(32-1))<<25)) | (1 << 25) );
	else ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIOSET = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIOSET & ~((0xffffffff>>(32-1))<<25)) | (1 << 25) );

	
	
	if((mess_find_unvol(210))&&	(mess_data[0]==106))
		{
		if(mess_data[1]==0) ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR & ~((0xffffffff>>(32-1))<<7)) | (1 << 7) );
		else if(mess_data[1]==1) ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET & ~((0xffffffff>>(32-1))<<7)) | (1 << 7) );
	     }
	else 
		{
		if(!(avar_ind_stat&0x100007f8)) ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR & ~((0xffffffff>>(32-1))<<7)) | (1 << 7) );
	     else ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET & ~((0xffffffff>>(32-1))<<7)) | (1 << 7) );
		} 

	
	if((mess_find_unvol(210))&&	(mess_data[0]==103))
		{
		if(mess_data[1]==0) ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR & ~((0xffffffff>>(32-1))<<4)) | (1 << 4) );
		else if(mess_data[1]==1) ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET & ~((0xffffffff>>(32-1))<<4)) | (1 << 4) );
     	}
	else 
		{
		if(!(ips_bat_av_stat)) ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR & ~((0xffffffff>>(32-1))<<4)) | (1 << 4) );
     	else ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET & ~((0xffffffff>>(32-1))<<4)) | (1 << 4) );
		} 
	} 	 
else	
	{
	
#line 5817 "control.c"
	if((mess_find_unvol(210))&&	(mess_data[0]==102))
		{
		if(mess_data[1]==0) ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET & ~((0xffffffff>>(32-1))<<25)) | (1 << 25) );
		else ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR & ~((0xffffffff>>(32-1))<<25)) | (1 << 25) );
		}
	else	if(!(avar_ind_stat&0x00000001)) ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00020) )->FIOSET = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00020) )->FIOSET & ~((0xffffffff>>(32-1))<<25)) | (1 << 25) );
	else ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIOSET = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00060) )->FIOSET & ~((0xffffffff>>(32-1))<<25)) | (1 << 25) );


	if((mess_find_unvol(210))&&	(mess_data[0]==106))
		{
		if(mess_data[1]==0) ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR & ~((0xffffffff>>(32-1))<<5)) | (1 << 5) );
		else if(mess_data[1]==1) ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET & ~((0xffffffff>>(32-1))<<5)) | (1 << 5) );
     	}
	else 
		{
		if(!(avar_ind_stat&0x100007f8)) ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR & ~((0xffffffff>>(32-1))<<5)) | (1 << 5) );
     	else ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET & ~((0xffffffff>>(32-1))<<5)) | (1 << 5) );
		}
	}

if((mess_find_unvol(210))&&	(mess_data[0]==113))	 
	{
	if(mess_data[1]==0) ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR & ~((0xffffffff>>(32-1))<<9)) | (1 << 9) );
	else if(mess_data[1]==1) ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET & ~((0xffffffff>>(32-1))<<9)) | (1 << 9) );
	}
else if(DOP_RELE_FUNC==0)	
	{
	if((!speedChIsOn)&&(spc_stat!=spcVZ)&&(hv_vz_stat==hvsOFF)&&(sp_ch_stat==scsOFF)&&(vz1_stat==vz1sOFF)&&(vz2_stat==vz2sOFF)&&(load_U/10<UVENTOFF))   ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR & ~((0xffffffff>>(32-1))<<9)) | (1 << 9) );
	else if((speedChIsOn)||(spc_stat==spcVZ)||(hv_vz_stat!=hvsOFF)||(sp_ch_stat!=scsOFF)||(vz1_stat!=vz1sOFF)||(vz2_stat!=vz2sOFF)) ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET & ~((0xffffffff>>(32-1))<<9)) | (1 << 9) );
	}
else if(DOP_RELE_FUNC==1)  
	{
	if((mess_find_unvol(210))&& (mess_data[0]==114)) ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOCLR & ~((0xffffffff>>(32-1))<<9)) | (1 << 9) );
	else ((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET = ( (((LPC_GPIO_TypeDef *) ((0x2009C000UL) + 0x00000) )->FIOSET & ~((0xffffffff>>(32-1))<<9)) | (1 << 9) );
	}



#line 5921 "control.c"



if((mess_find_unvol(210))&&	(mess_data[0]==117))
	{
	if(mess_data[1]==0) 		bdr_transmit_stat&=0xfe;
	else if(mess_data[1]==1) 	bdr_transmit_stat|=0x01;
	}
else 
	{
	if(bdr_avar_stat&0x01)  	bdr_transmit_stat|=0x01;
	else 						bdr_transmit_stat&=0xfe;
	}	

if((mess_find_unvol(210))&&	(mess_data[0]==118))
	{
	if(mess_data[1]==0) 		bdr_transmit_stat&=0xfd;
	else if(mess_data[1]==1) 	bdr_transmit_stat|=0x02;
	}
else 
	{
	if(bdr_avar_stat&0x02)  	bdr_transmit_stat|=0x02;	 
	else 						bdr_transmit_stat&=0xfd;
	}	
	
if((mess_find_unvol(210))&&	(mess_data[0]==119))
	{
	if(mess_data[1]==0) 		bdr_transmit_stat&=0xfb;
	else if(mess_data[1]==1) 	bdr_transmit_stat|=0x04;
	}
else 
	{
	if(bdr_avar_stat&0x04)  	bdr_transmit_stat|=0x04;	
	else 						bdr_transmit_stat&=0xfb;
	}	
	
if((mess_find_unvol(210))&&	(mess_data[0]==120))
	{
	if(mess_data[1]==0) 		bdr_transmit_stat&=0xf7;
	else if(mess_data[1]==1) 	bdr_transmit_stat|=0x08;
	}
else 
	{
	if(bdr_avar_stat&0x08)  	bdr_transmit_stat|=0x08;	 
	else 						bdr_transmit_stat&=0xf7;
	}	
					 	


#line 6083 "control.c"


#line 6144 "control.c"

#line 6226 "control.c"

#line 6250 "control.c"

if(NUMBDR==1)
	{
	char ii_;
	char bdr_avar_stat_temp=0;
	for	(ii_=0;ii_<4;ii_++)
		{
	
#line 6282 "control.c"
	
		if((RELE_SET_MASK[ii_]&0x01)&&
			(load_U<(USIGN*10)))			bdr_avar_stat_temp|=(1<<ii_);
		
		if((RELE_SET_MASK[ii_]&0x02)&&
			(sp_ch_stat==scsWRK))			bdr_avar_stat_temp|=(1<<ii_);
		
		if((RELE_SET_MASK[ii_]&0x04)&&
			(spc_stat==spcVZ))			bdr_avar_stat_temp|=(1<<ii_);
		
		if((RELE_SET_MASK[ii_]&0x08)&&
			(avar_stat))					bdr_avar_stat_temp|=(1<<ii_);
		
		if((RELE_SET_MASK[ii_]&0x10)&&
			(uout_av==1))					bdr_avar_stat_temp|=(1<<ii_);
		
		if((RELE_SET_MASK[ii_]&0x20)&&
			(uout_av==2))					bdr_avar_stat_temp|=(1<<ii_);
		if((RELE_SET_MASK[ii_]&0x40)&&
			(
			((bps[0]._av&(1<<4))&&(NUMIST>=1))||
			((bps[1]._av&(1<<4))&&(NUMIST>=2))||
			((bps[2]._av&(1<<4))&&(NUMIST>=3))
			))bdr_avar_stat_temp|=(1<<ii_);
		if((RELE_SET_MASK[ii_]&0x80)&&
			(
			((bps[0]._av&(0x0f))&&(NUMIST>=1))||
			((bps[1]._av&(0x0f))&&(NUMIST>=2))||
			((bps[2]._av&(0x0f))&&(NUMIST>=3))
			))bdr_avar_stat_temp|=(1<<ii_);

		if(!(RELE_SET_MASK[ii_]&(1<<15))) bdr_avar_stat_temp^=(1<<ii_); 
		}
	bdr_avar_stat=bdr_avar_stat_temp;
	}
}



void bps_hndl(void)
{
char ptr__,i;
unsigned short tempUS;

if(sh_cnt0<10)
	{
	sh_cnt0++;
	if(sh_cnt0>=10)
		{
		sh_cnt0=0;
		b1Hz_sh=1;
		}
	}









 








































 



  
bps_on_mask=0;
bps_off_mask=0;

if(mess_find_unvol(205))
	{
	if(mess_data[0]==206)
		{
		bps_off_mask=0xffff;
		}

	if(mess_data[0]==207)
		{
		bps_off_mask=mess_data[1];
		}

	if(mess_data[0]==209)
		{
		bps_on_mask=mess_data[1];
		}

	if(mess_data[0]==210)
		{
		bps_on_mask=0xffff;
		}

	if(mess_data[0]==208)
		{
		bps_on_mask=(unsigned)mess_data[1];
		bps_off_mask=~((unsigned)mess_data[1]);
		}

 	if(mess_data[0]==308)
		{
		bps_on_mask=(unsigned)(1<<mess_data[1]);
		bps_off_mask=~((unsigned)(1<<mess_data[1]));
		}

	for(i=0;i<=NUMIST;i++)
		{
		if(bps_off_mask&(1<<i)) bps[i]._blok_cnt++;
		else bps[i]._blok_cnt=0;
		gran(&bps[i]._blok_cnt,0,50);
		if(bps[i]._blok_cnt>20) bps[i]._flags_tu=1;
		if(bps_on_mask&(1<<i)) bps[i]._flags_tu=0;
	     }

	

































 										
	}


else if(b1Hz_sh)
	{
	ptr__=0;
     for(i=0;i<=NUMIST;i++)
		{
	     bps[i]._flags_tu=1;
	     }	
  	     
  	for(i=0;(i<NUMIST)&&(ptr__<num_necc);i++)
  		{
		char ii,iii;

		ii=(char)NUMIST;
		
		if(ii>32)ii=32;
		iii=numOfForvardBps;
		
		if(iii>=NUMIST)iii=0;
		iii+=i;
		iii=iii%ii;
		
  	     if((bps[iii]._state==bsRDY)||(bps[iii]._state==bsWRK))
  	         	{
  	         	bps[iii]._flags_tu=0;
  	         	ptr__++;
  	         	}
			
  	     }
	bps[numOfForvardBps_old]._flags_tu=0;

	if(main_1Hz_cnt<60)
		{
     	for(i=0;i<=NUMIST;i++)
			{
	     	bps[i]._flags_tu=0;
	     	}	
		}
	if(ipsBlckStat)
		{
     	for(i=0;i<=NUMIST;i++)
			{
	     	bps[i]._flags_tu=1;
	     	}
		}

     for(i=0;i<=NUMIST;i++)
		{
	    if(bps[i]._flags_tu==1) 	bps[i]._x_=-50;
	   	}	
		 
  	}


for(i=0;i<NUMIST;i++)
     {
     if(bps[i]._ist_blok_host_cnt!=0)
          {
          bps[i]._flags_tu=99;
	     bps[i]._ist_blok_host_cnt--;
          }
     }




b1Hz_sh=0;


num_of_wrks_bps=0;
tempUS=0;
for(i=0;i<NUMIST;i++)
	{
	if(bps[i]._state==bsWRK)
		{
		num_of_wrks_bps++;
		if(bps[i]._Uii>tempUS)tempUS=bps[i]._Uii;
		}
	}
Ubpsmax=tempUS;

bPARALLEL_ENOUG=0;
bPARALLEL_NOT_ENOUG=1;

for(i=0;i<NUMIST;i++)
	{
	if(bps[i]._Ti>=TSIGN)
		{
		bPARALLEL_ENOUG=1;
		}
	if(bps[i]._Ti>=(TSIGN-5))
		{
		bPARALLEL_NOT_ENOUG=0;
		}
	}

if(bPARALLEL_ENOUG==1)
	{
	bPARALLEL=1;
	}
else if(bPARALLEL && bPARALLEL_NOT_ENOUG)
	{
	bPARALLEL=0;
	}
}








void powerAntiAliasingHndl(void)
{
if((power_summary_tempo/10UL)==(power_summary_tempo_old/10UL))
	{
	if(powerSummaryCnt<15)powerSummaryCnt++;
	if(powerSummaryCnt>=10)
		{
		power_summary=power_summary_tempo;
		}
	}
else powerSummaryCnt=0;
power_summary_tempo_old=power_summary_tempo;

if((power_current_tempo/10UL)==(power_current_tempo_old/10UL))
	{
	if(powerCurrentCnt<15)powerCurrentCnt++;
	if(powerCurrentCnt>=10)
		{
		power_current=power_current_tempo;
		}
	}
else powerCurrentCnt=0;
power_current_tempo_old=power_current_tempo;
}


void ips_current_average_hndl_(void)
{
if(++ica_timer_cnt>=10)
	{
	ica_timer_cnt=0;
	ica_plazma[0]++;

	ica_my_current=bps_I;

	if((ica_my_current>ica_your_current)&&((ica_my_current-ica_your_current)>=10)&&(ICA_EN==1))
		{
		ica_plazma[1]++;
		ica_u_necc--;
		}
	else if((ica_my_current<ica_your_current)&&((ica_your_current-ica_my_current)>=10)&&(ICA_EN==1))
		{
		ica_plazma[1]--;
		ica_u_necc++;
		}
	gran(&ica_u_necc,-20,20);
	}

if((ica_timer_cnt==8)&&(ICA_EN==1))
	{
	char modbus_buff[20],i;
	short crc_temp;

	modbus_buff[0] = ICA_MODBUS_ADDRESS;
	modbus_buff[1] = 4;
	modbus_buff[2] = 0;
	modbus_buff[3] = 2;
	modbus_buff[4] = 0;	
	modbus_buff[5] = 1;

	crc_temp= CRC16_2(modbus_buff,6);

	modbus_buff[6]= (char)crc_temp;
	modbus_buff[7]= (char)(crc_temp>>8);



	if(ICA_CH==0)
		{
		for (i=0;i<8;i++)
			{
			putchar_sc16is700(modbus_buff[i]);
			}
		}
	else if(ICA_CH==1)
		{
	



 
  		
  		if (tcp_soc_avg != 0) 
			{
    		
			
    		
			

 
			
			

			}
		}
	}

if((ica_timer_cnt==3)&&(ICA_EN==1))
	{
	
		{
		
		
		}
	}

if((main_kb_cnt==(TBAT*60)-21)&&(ICA_EN==1))
	{
	char modbus_buff[20],i;
	short crc_temp;

	modbus_buff[0] = ICA_MODBUS_ADDRESS;
	modbus_buff[1] = 6;
	modbus_buff[2] = 0;
	modbus_buff[3] = 30;
	modbus_buff[4] = (char)(TBAT/256);	
	modbus_buff[5] = (char)(TBAT%256);

	crc_temp= CRC16_2(modbus_buff,6);

	modbus_buff[6]= (char)crc_temp;
	modbus_buff[7]= (char)(crc_temp>>8);

	if(ICA_CH==0)
		{
		for (i=0;i<8;i++)
			{
			putchar_sc16is700(modbus_buff[i]);
			}
		}
	}

}


void energometr_hndl(void)
{







#line 6863 "control.c"
}


void ips_current_average_hndl(void)		
{

if(++ica_timer_cnt>=10) 
	{
	ica_timer_cnt=0;
	}

if((ica_timer_cnt==0) && (num_of_wrks_bps)&&((spc_stat==spcOFF)&&(vz1_stat==vz1sOFF)&&(vz2_stat==vz2sOFF)&&(sp_ch_stat==scsOFF)))
	{
	
	ica_plazma[0]++;

	ica_my_current=bps_I;

	if((ica_my_current>ica_your_current)&&((ica_my_current-ica_your_current)>=5)&&((ICA_EN==1)||((ICA_EN==2)&&(ica_cntrl_hndl_cnt>5))))
		{
		ica_plazma[1]++;
		ica_u_necc--;
		}
	else if((ica_my_current<ica_your_current)&&((ica_your_current-ica_my_current)>=5)&&((ICA_EN==1)||((ICA_EN==2)&&(ica_cntrl_hndl_cnt>5))))
		{
		ica_plazma[1]--;
		ica_u_necc++;
		}
	gran(&ica_u_necc,-100,100);
	}


if((ICA_EN==1)&&((spc_stat==spcOFF)&&(vz1_stat==vz1sOFF)&&(vz2_stat==vz2sOFF)&&(sp_ch_stat==scsOFF)) && (num_of_wrks_bps))
	{
	


	if(ica_timer_cnt==8)
		{
		char modbus_buff[20],i;
		short crc_temp;
	
		modbus_buff[0] = ICA_MODBUS_ADDRESS;
		modbus_buff[1] = 4;
		modbus_buff[2] = 0;
		modbus_buff[3] = 2;
		modbus_buff[4] = 0;	
		modbus_buff[5] = 1;
	
		crc_temp= CRC16_2(modbus_buff,6);
	
		modbus_buff[6]= (char)crc_temp;
		modbus_buff[7]= (char)(crc_temp>>8);
	
		if(ICA_CH==0)
			{
			for (i=0;i<8;i++)
				{
				putchar_sc16is700(modbus_buff[i]);
				}
			}
		else if(ICA_CH==2)
			{
			uart_out1 (5,4,0,2,0,1,0);
			}
		}
	else
		{
		char modbus_buff[20],i;
		short crc_temp, tempSSSS;

		tempSSSS=cntrl_stat_old;
		if(	(main_kb_cnt==(TBAT*60)-21) || (main_kb_cnt==(TBAT*60)-20) || (main_kb_cnt==(TBAT*60)-19)) tempSSSS=((short)TBAT)|0x4000;


		modbus_buff[0] = ICA_MODBUS_ADDRESS;
		modbus_buff[1] = 6;
		modbus_buff[2] = 0;
		modbus_buff[3] = 100;
		modbus_buff[4] = (char)(tempSSSS/256);	
		modbus_buff[5] = (char)(tempSSSS%256);
	
		crc_temp= CRC16_2(modbus_buff,6);
	
		modbus_buff[6]= (char)crc_temp;
		modbus_buff[7]= (char)(crc_temp>>8);
	
		crc_temp= CRC16_2(modbus_buff,6);
	
		plazma_ica1=tempSSSS;
		if(ICA_CH==0)
			{
			for (i=0;i<8;i++)
				{
				putchar_sc16is700(modbus_buff[i]);
				}
			}
		else if(ICA_CH==2)
			{
			uart_out1 (5,6,0,100,modbus_buff[4],modbus_buff[5],0);
			}
		}
	}
}


void inv_drv(char in)
{
char temp,temp_;



gran_char((signed char*)&first_inv_slot,1,7);


temp=inv[in]._flags_tm_old^inv[in]._flags_tm;
if(temp)plazma_inv[1] =temp;

temp_=inv[in]._flags_tm&temp;
if(temp_)plazma_inv[2] =temp_;

if( (temp&(1<<0)) && (temp_&(1<<0)) ) 
	{
	plazma_inv[3]++;
	if(in==0)snmp_trap_send("Invertor #1 Alarm, overload",14,1,1);
	else if(in==1)snmp_trap_send("Invertor #2 Alarm, overload",14,2,1);
	else if(in==2)snmp_trap_send("Invertor #3 Alarm, overload",14,3,1);
	else if(in==3)snmp_trap_send("Invertor #4 Alarm, overload",14,4,1);
	}
else if( (temp&(1<<1)) && (temp_&(1<<1)) )
	{
		plazma_inv[3]++;
	if(in==0)snmp_trap_send("Invertor #1 Alarm, overheat",14,1,2);
	else if(in==1)snmp_trap_send("Invertor #2 Alarm, overheat",14,2,2);
	else if(in==2)snmp_trap_send("Invertor #3 Alarm, overheat",14,3,2);
	else if(in==3)snmp_trap_send("Invertor #4 Alarm, overheat",14,4,2);
	}

else if( (temp&(1<<2)) && (temp_&(1<<2)) )
	{
		plazma_inv[3]++;
	if(in==0)snmp_trap_send("Invertor #1 Alarm, is warm",14,1,3);
	else if(in==1)snmp_trap_send("Invertor #2 Alarm, is warm",14,2,3);
	else if(in==2)snmp_trap_send("Invertor #3 Alarm, is warm",14,3,3);
	else if(in==3)snmp_trap_send("Invertor #4 Alarm, is warm",14,4,3);
	}

else if( (temp&(1<<3)) && (temp_&(1<<3)) ) 
	{
		plazma_inv[3]++;
	if(in==0)snmp_trap_send("Invertor #1 Alarm, voltage is up",14,1,4);
	else if(in==1)snmp_trap_send("Invertor #2 Alarm, voltage is up",14,2,4);
	else if(in==2)snmp_trap_send("Invertor #3 Alarm, voltage is up",14,3,4);
	else if(in==3)snmp_trap_send("Invertor #4 Alarm, voltage is up",14,4,4);
	}

else if( (temp&(1<<4)) && (temp_&(1<<4)) ) 
	{
		plazma_inv[3]++;
	if(in==0)snmp_trap_send("Invertor #1 Alarm, voltage is down",14,1,5);
	else if(in==1)snmp_trap_send("Invertor #2 Alarm, voltage is down",14,2,5);
	else if(in==2)snmp_trap_send("Invertor #3 Alarm, voltage is down",14,3,5);
	else if(in==3)snmp_trap_send("Invertor #4 Alarm, voltage is down",14,4,5);
	}

else if( (temp&(1<<5)) && (temp_&(1<<5)) )
	{
		plazma_inv[3]++;
	if(in==0)snmp_trap_send("Invertor #1 Alarm, output is offed",14,1,6);
	else if(in==1)snmp_trap_send("Invertor #2 Alarm, output is offed",14,2,6);
	else if(in==2)snmp_trap_send("Invertor #3 Alarm, output is offed",14,3,6);
	else if(in==3)snmp_trap_send("Invertor #4 Alarm, output is offed",14,4,6);
	}

else if((temp)&&(!temp_)) 
	{
		plazma_inv[3]++;
	if(in==0)snmp_trap_send("Invertor #1 is norm",14,1,0);
	else if(in==1)snmp_trap_send("Invertor #2 is norm",14,2,0);
	else if(in==2)snmp_trap_send("Invertor #3 is norm",14,3,0);
	else if(in==3)snmp_trap_send("Invertor #4 is norm",14,4,0);
	}

inv[in]._flags_tm_old=inv[in]._flags_tm;

}	


void ipsBlckHndl(char in)
{

ipsBlckStat=0;
if(ipsBlckSrc==1)
	{
	if(((ipsBlckLog==0)&&(adc_buff_[11]>2000)) || ((ipsBlckLog==1)&&(adc_buff_[11]<2000))) ipsBlckStat=1;
	}
else if(ipsBlckSrc==2)
	{
	if(((ipsBlckLog==0)&&(adc_buff_[13]>2000)) || ((ipsBlckLog==1)&&(adc_buff_[13]<2000))) ipsBlckStat=1;
	}
}


void bps_drv(char in)
{
char temp;

if (bps[in]._device!=dSRC) return;
temp=bps[in]._flags_tm;
if(temp&(1<<1))
	{
	if(bps[in]._temp_av_cnt<1200) 
		{
		bps[in]._temp_av_cnt++;
		if(bps[in]._temp_av_cnt>=1200)
			{
			bps[in]._temp_av_cnt=1200;
		   	if(!(bps[in]._av&(1<<0)))avar_bps_hndl(in,0,1);
			}
		}
	}

else if(!(temp&(1<<1)))
	{
	if(bps[in]._temp_av_cnt) 
		{
		bps[in]._temp_av_cnt--;
		if(!bps[in]._temp_av_cnt)
			{
			if(bps[in]._av&(1<<0))avar_bps_hndl(in,0,0);
			}
		} 	

	}

if((temp&(1<<3)))
	{
	if(bps[in]._umax_av_cnt<10) 
		{
		bps[in]._umax_av_cnt++;
		if(bps[in]._umax_av_cnt>=10)
			{ 
			bps[in]._umax_av_cnt=10;
			if(!(bps[in]._av&(1<<1)))avar_bps_hndl(in,1,1);
			apv_start(in);
		  	






 
						
			}
		} 
	}		
else if(!(temp&(1<<3)))
	{
	if(bps[in]._umax_av_cnt>0) 
		{
		bps[in]._umax_av_cnt--;
		if(bps[in]._umax_av_cnt==0)
			{
			bps[in]._umax_av_cnt=0;
			avar_bps_hndl(in,1,0);
			
	 
	
	 
			}
		}
	else if(bps[in]._umax_av_cnt<0) bps[in]._umax_av_cnt=0;		 
	}

if(temp&(1<<4))
	{
	if(bps[in]._umin_av_cnt<10) 
		{
		bps[in]._umin_av_cnt++;
		if(bps[in]._umin_av_cnt>=10)
			{ 
			bps[in]._umin_av_cnt=10;
			if(!(bps[in]._av&(1<<2)))avar_bps_hndl(in,2,1);
			apv_start(in);
		  	






 				
			}
		} 
	}	
	
else if(!(temp&(1<<4)))
	{
	if(bps[in]._umin_av_cnt) 
		{
		bps[in]._umin_av_cnt--;
		if(bps[in]._umin_av_cnt==0)
			{
			bps[in]._umin_av_cnt=0;
			avar_bps_hndl(in,2,0);
			
		
		
		
			}
		}
	else if(bps[in]._umin_av_cnt>10)bps[in]._umin_av_cnt--;	 
	}

if((bps[in]._Uii<(UB20-DU)))
	{
	if(bps[in]._state==bsWRK)
		{
		if(bps[in]._umin_av_cnt_uku<300) 
			{
			bps[in]._umin_av_cnt_uku++;
			if(bps[in]._umin_av_cnt_uku>=300)
				{ 
				bps[in]._umin_av_cnt_uku=300;
				if(!(bps[in]._av&(1<<2)))avar_bps_hndl(in,2,1);
			  	






 				
				}
			}
		else
			{
			bps[in]._umin_av_cnt_uku=0;
			} 
		}
	}	
	
else if(bps[in]._Uii>=(UB20-DU))
	{
	if(bps[in]._umin_av_cnt_uku) 
		{
		bps[in]._umin_av_cnt_uku--;
		if(bps[in]._umin_av_cnt_uku==0)
			{
			bps[in]._umin_av_cnt_uku=0;
			avar_bps_hndl(in,2,0);
		
		
		
			}
		}
	else if(bps[in]._umin_av_cnt_uku>300)bps[in]._umin_av_cnt_uku=300;	 
	}



if (bps[in]._av&0x0f)					bps[in]._state=bsAV;
else if ( (net_av) && (bps[in]._cnt>20)
 )				bps[in]._state=bsOFF_AV_NET;
else if (bps[in]._flags_tm&(((0x100000) | 0x100000>>3 | 0x100000>>6 | 0x100000>>9) & 0xf | ((0x100000) | 0x100000>>3 | 0x100000>>6 | 0x100000>>9)>>12 & 0xf0))	bps[in]._state=bsRDY;
else if (bps[in]._cnt<20)				bps[in]._state=bsWRK;























     















  


if(bps[in]._cnt>=10) bps[in]._flags_tu|=(((0x10000000) | 0x10000000>>3 | 0x10000000>>6 | 0x10000000>>9) & 0xf | ((0x10000000) | 0x10000000>>3 | 0x10000000>>6 | 0x10000000>>9)>>12 & 0xf0);
else bps[in]._flags_tu&=(((0x1111111) | 0x1111111>>3 | 0x1111111>>6 | 0x1111111>>9) & 0xf | ((0x1111111) | 0x1111111>>3 | 0x1111111>>6 | 0x1111111>>9)>>12 & 0xf0);

if(avar_bps_reset_cnt) 
	{
	bps[in]._flags_tu|=(((0x10) | 0x10>>3 | 0x10>>6 | 0x10>>9) & 0xf | ((0x10) | 0x10>>3 | 0x10>>6 | 0x10>>9)>>12 & 0xf0);
	bps[in]._av=0;
	}
else if(bps[in]._apv_reset_av_timer) bps[in]._flags_tu|=(((0x10) | 0x10>>3 | 0x10>>6 | 0x10>>9) & 0xf | ((0x10) | 0x10>>3 | 0x10>>6 | 0x10>>9)>>12 & 0xf0);
else bps[in]._flags_tu&=(((0x11111101) | 0x11111101>>3 | 0x11111101>>6 | 0x11111101>>9) & 0xf | ((0x11111101) | 0x11111101>>3 | 0x11111101>>6 | 0x11111101>>9)>>12 & 0xf0);
	
bps[in]._vol_u=cntrl_stat+bps[in]._x_;	
bps[in]._vol_i=1000;


}


void avt_hndl(void)
{
char i;
for(i=0;i<12;i++)
	{
	if(eb2_data_short[6]&(1<<i))
		{
		avt_stat[i]=avtON;
		}
	else avt_stat[i]=avtOFF;
	}

if((avt_stat_old[0]!=avt_stat[0])&&(NUMAVT>=1))
	{
	if(avt_stat[0]==avtON) 	snmp_trap_send("Avtomat #1 is ON ",11,1,1);
	else 				snmp_trap_send("Avtomat #1 is OFF",11,1,0);
	}
if((avt_stat_old[1]!=avt_stat[1])&&(NUMAVT>=2))
	{
	if(avt_stat[1]==avtON) 	snmp_trap_send("Avtomat #2 is ON ",11,2,1);
	else 				snmp_trap_send("Avtomat #2 is OFF",11,2,0);
	}
if((avt_stat_old[2]!=avt_stat[2])&&(NUMAVT>=3))
	{
	if(avt_stat[2]==avtON) 	snmp_trap_send("Avtomat #3 is ON ",11,3,1);
	else 				snmp_trap_send("Avtomat #3 is OFF",11,3,0);
	}
if((avt_stat_old[3]!=avt_stat[3])&&(NUMAVT>=4))
	{
	if(avt_stat[3]==avtON) 	snmp_trap_send("Avtomat #4 is ON ",11,4,1);
	else 				snmp_trap_send("Avtomat #4 is OFF",11,4,0);
	}
if((avt_stat_old[4]!=avt_stat[4])&&(NUMAVT>=5))
	{
	if(avt_stat[4]==avtON) 	snmp_trap_send("Avtomat #5 is ON ",11,5,1);
	else 				snmp_trap_send("Avtomat #5 is OFF",11,5,0);
	}
if((avt_stat_old[5]!=avt_stat[5])&&(NUMAVT>=6))
	{
	if(avt_stat[5]==avtON) 	snmp_trap_send("Avtomat #6 is ON ",11,6,1);
	else 				snmp_trap_send("Avtomat #6 is OFF",11,6,0);
	}
if((avt_stat_old[6]!=avt_stat[6])&&(NUMAVT>=7))
	{
	if(avt_stat[6]==avtON) 	snmp_trap_send("Avtomat #7 is ON ",11,7,1);
	else 				snmp_trap_send("Avtomat #7 is OFF",11,7,0);
	}
if((avt_stat_old[7]!=avt_stat[7])&&(NUMAVT>=8))
	{
	if(avt_stat[7]==avtON) 	snmp_trap_send("Avtomat #8 is ON ",11,8,1);
	else 				snmp_trap_send("Avtomat #8 is OFF",11,8,0);
	}
if((avt_stat_old[8]!=avt_stat[8])&&(NUMAVT>=9))
	{
	if(avt_stat[8]==avtON) 	snmp_trap_send("Avtomat #9 is ON ",11,9,1);
	else 				snmp_trap_send("Avtomat #9 is OFF",11,9,0);
	}

for(i=0;i<12;i++)
	{
	avt_stat_old[i]=avt_stat[i];
	}
}


void bat_hndl(void)
{






























 

if(mess_find_unvol(200))
	{ 
	char i;
	
	if(mess_data[0]==201)
		{
		for(i=0;i<2;i++)
			{
			if(bat[i]._cnt_to_block<50)bat[i]._cnt_to_block++;
			}
		}

	else if(mess_data[0]==202)
		{
		for(i=0;i<2;i++)
			{
			if(mess_data[1]&(1<<i))
				{
				if(bat[i]._cnt_to_block<50) bat[i]._cnt_to_block++;
				}
			else bat[i]._cnt_to_block=0;
			}
		}
	else 
	 	{
		for(i=0;i<2;i++)
			{
			bat[i]._cnt_to_block=0;
			}

		}
	for(i=0;i<2;i++)
		{
		if(bat[i]._cnt_to_block>20)bat[i]._rel_stat=1;
		else bat[i]._rel_stat=0;
		}

	}

else 
	{
	char i;
	for(i=0;i<2;i++)
		{
		bat[i]._cnt_to_block=0;
		bat[i]._rel_stat=0;
		}

	}















 
}

#line 7644 "control.c"

#line 7817 "control.c"



void klimat_hndl(void)
{


if(t_box>TBOXMAX)
	{
	av_tbox_cnt++;
	} 
else if(t_box<TBOXMAX)
	{
	av_tbox_cnt--;
	}
gran(&av_tbox_cnt,0,6);

if(av_tbox_cnt>5)
	{
	av_tbox_stat=atsON;
	}
if(av_tbox_cnt<1)
	{
	av_tbox_stat=atsOFF;
	}

if(t_box<(TBOXREG-2))
	{
	if(t_box_cnt<30)
		{
		t_box_cnt++;
		if(t_box_cnt>=30)
			{
			main_vent_pos--;
			t_box_cnt=0;
			}
		}
	}
else if(t_box>(TBOXREG))
	{
	if(t_box_cnt<30)
		{
		t_box_cnt++;
		if(t_box_cnt>=30)
			{
			main_vent_pos++;
			t_box_cnt=0;
			}
		}
	}
else
	{
	t_box_cnt=0;
	}


if(t_box>TBOXVENTMAX)gran(&main_vent_pos,0,20); 
else gran(&main_vent_pos,0,pos_vent+9);

if((mess_find_unvol(240))&&(mess_data[0]==241))
	{
	main_vent_pos=mess_data[1];
	}


if(main_vent_pos<=1)mixer_vent_stat=mvsON;
else mixer_vent_stat=mvsOFF;


#line 7903 "control.c"

if((TBATDISABLE>=50) && (TBATDISABLE<=90))
	{
	if(t_box>TBATDISABLE)
		{
		tbatdisable_cnt++;
		}
	if(t_box<TBATENABLE)
		{
		tbatdisable_cnt--;
		}
	gran(&tbatdisable_cnt,0,6);

	if(tbatdisable_cnt>5)
		{
		tbatdisable_stat=tbdsOFF;
		}
	if(tbatdisable_cnt<1)
		{
		tbatdisable_stat=tbdsON;
		}
	}
else 
	{
	tbatdisable_stat=tbdsON;
	}

if((TLOADDISABLE>=50) && (TLOADDISABLE<=80))
	{
	if(t_box>TLOADDISABLE)
		{
		tloaddisable_cnt++;
		}
	if(t_box<TLOADENABLE)
		{
		tloaddisable_cnt--;
		}
	gran(&tloaddisable_cnt,0,6);

	if(tloaddisable_cnt>5)
		{
		tloaddisable_stat=tldsOFF;
		}
	if(tloaddisable_cnt<1)
		{
		tloaddisable_stat=tldsON;
		}
	}
else 
	{
	tloaddisable_stat=tldsON;
	}

}


#line 8092 "control.c"


void bat_drv(char in)
{
unsigned short  tempUS_;
unsigned long tempUL,tempUL_;
unsigned short b_zar;




if(cntrl_stat_blok_cnt_plus[in])cntrl_stat_blok_cnt_plus[in]--;
if(cntrl_stat_blok_cnt_minus[in])cntrl_stat_blok_cnt_minus[in]--;


if(bat[in]._Ib>IZMAX) cntrl_stat_blok_cnt_plus[in]=100;
if(bat[in]._Ib<0)     cntrl_stat_blok_cnt_minus[in]=100;

if(cntrl_stat_blok_cnt_plus[in] && cntrl_stat_blok_cnt_minus[in])
     {
     if(!cntrl_stat_blok_cnt_)
          {
          cntrl_stat_blok_cnt_=600; 
          cntrl_stat_blok_cnt_plus[in]=0;
          cntrl_stat_blok_cnt_minus[in]=0;
          }
     else cntrl_stat_blok_cnt=3000;
     }
cntrl_stat_blok_cnt=0;

if(++(bat[in]._time_cnt)>=10)
	{
	bat[in]._time_cnt=0;
	
	}

if(main_10Hz_cnt==50)
	{
	if(!bat[in]._rel_stat)
		{
		
		if(bat[in]._Ub<80) 
			{
			
			if(!(bat[in]._av&1))
				{
				avar_bat_hndl(in,1);
				
				}
			}				

		}
	}

if(main_10Hz_cnt>200)
	{
	if(abs(bat[in]._Ib)>IKB) 
		{
		if((bat[in]._av&1))avar_bat_hndl(in,0);
		}
	}







if(bat[in]._Ib>(-IKB))
	{
	if(bat[in]._cnt_wrk<10)
		{
		bat[in]._cnt_wrk++;
		if((bat[in]._cnt_wrk>=10)&&(bat[in]._wrk)) 
			{
			bat[in]._wrk=0;
			
			
			wrk_mem_hndl(in);
			
			}
		}
	else bat[in]._cnt_wrk=10;	
	}	

else if(bat[in]._Ib<(-IKB))
	{
	if(bat[in]._cnt_wrk)
		{
		bat[in]._cnt_wrk--;
		if((bat[in]._cnt_wrk==0)&&(bat[in]._wrk==0)) 
			{
			bat[in]._wrk=1;

				{
				char temp;
				signed short temp_temp;
				temp_temp=bat[in]._u_old[((bat_u_old_cnt+1)&0x07)];
			 
				temp=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->YEAR;
				gran_char((signed char*)&temp,1,99);
				*((char*)(&(bat[in]._wrk_date[0])))=temp;
			
				temp=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MONTH;
				gran_char((signed char*)&temp,1,12);
				*(((char*)(&(bat[in]._wrk_date[0])))+1)=temp;
			
				temp=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->DOM;
				gran_char((signed char*)&temp,1,31);
				*(((char*)(&(bat[in]._wrk_date[0])))+2)=temp;			
				
				*(((char*)(&(bat[in]._wrk_date[0])))+3)=*((char*)&temp_temp);

				temp=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->HOUR;
				gran_char((signed char*)&temp,0,23);
				*((char*)(&(bat[in]._wrk_date[1])))=temp;
               	
				temp=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MIN;
				gran_char((signed char*)&temp,0,59);
				*(((char*)(&(bat[in]._wrk_date[1])))+1)=temp;
	          
				temp=((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->SEC;
				gran_char((signed char*)&temp,0,59);
				*(((char*)(&(bat[in]._wrk_date[1])))+2)=temp;
			
				*(((char*)(&(bat[in]._wrk_date[1])))+3)=*(((char*)&temp_temp)+1);
				bat[in]._Iintegr=0;		
				bat[in]._Iintegr_=0;	
				}
	
			}

		}
	else bat[in]._cnt_wrk=0;	 
	
	}					













































































 










 










                            
if(bat[in]._time_cnt==0)
	{
	bat[in]._zar_cnt+=(signed long)bat[in]._Ib;
	
	if(bat[in]._zar_cnt>=36000L)
		{
		if(BAT_C_REAL[in]==0x5555) tempUS_=BAT_C_NOM[in];
		else tempUS_=BAT_C_REAL[in];
		
		b_zar=lc640_read_int(ADR_EE_BAT_ZAR_CNT[in]);

		if(b_zar<(tempUS_ ))
			{
			bat[in]._zar_cnt-=36000L;

			lc640_write_int(ADR_EE_BAT_ZAR_CNT[in],b_zar+1);
			}
		else if(b_zar>(tempUS_ ))  
			{
			lc640_write_int(ADR_EE_BAT_ZAR_CNT[in],tempUS_);
			bat[in]._zar_cnt=36000L;

			}

		}

	else if(bat[in]._zar_cnt<=-36000L)
		{
		if(BAT_C_REAL[in]==0x5555) tempUS_=BAT_C_NOM[in];
		else tempUS_=BAT_C_REAL[in];
		
		b_zar=lc640_read_int(ADR_EE_BAT_ZAR_CNT[in]);

		if(b_zar>tempUS_)
			{
			lc640_write_int(ADR_EE_BAT_ZAR_CNT[in],tempUS_);
			b_zar=tempUS_;
			}
		if(b_zar)
			{
			bat[in]._zar_cnt+=36000L;

			lc640_write_int(ADR_EE_BAT_ZAR_CNT[in],b_zar-1);
			}
		else 
			{
			bat[in]._zar_cnt=-36000L;
			}

		}

			
	tempUL=(unsigned long)lc640_read_int(ADR_EE_BAT_ZAR_CNT[in]);
	
	if(BAT_C_REAL[in]==0x5555) tempUL_=(unsigned long)BAT_C_NOM[in];
	else tempUL_=(unsigned long)BAT_C_REAL[in];
		           	
	tempUL*=1000L;


	if(tempUL_==0) tempUL=0;
	else tempUL/=tempUL_;

	tempUL/=10L;

	bat[in]._zar=(unsigned short)tempUL;

	if(BAT_TYPE==1)
		{
		bat[in]._zar=lakb[in]._s_o_c;
		}
	
	gran((signed short*)&bat[in]._zar,0,100);
     }



if(bat[in]._wrk==1)
	{
	if(bat[in]._Iintegr<36000)
		{          
		bat[in]._Iintegr+=abs(bat[in]._Ib);
		if(bat[in]._Iintegr>=36000)
			{
			bat[in]._Iintegr=0;		
			bat[in]._Iintegr_++;	
			}
		}
	else 
		{
		bat[in]._Iintegr=0;
		}	
	} 
	    

if((t_ext[0]>TBATSIGN)&&(!ND_EXT[0]))	
	{
	bat[in]._sign_temper_cnt++;
	}
else 
	{
	bat[in]._sign_temper_cnt--;
	}
#line 8441 "control.c"
gran(&bat[in]._sign_temper_cnt,0,600);
if(bat[in]._sign_temper_cnt>=590)	bat[in]._temper_stat|=(1<<0);
if(bat[in]._sign_temper_cnt<=10)	bat[in]._temper_stat&=~(1<<0);

if((bat[in]._temper_stat&(1<<0))&&(!(bat[in]._temper_stat&(1<<4))))	avar_bat_temper_hndl(in,1);
if((!(bat[in]._temper_stat&(1<<0)))&&(bat[in]._temper_stat&(1<<4)))	avar_bat_temper_hndl(in,0);

if(bat[in]._temper_stat&(1<<0))		bat[in]._temper_stat|=(1<<4);
else 								bat[in]._temper_stat&=~(1<<4);


if((t_ext[0]>TBATMAX)&&(!ND_EXT[0]))	
	{
	bat[in]._max_temper_cnt++;
	}
else 
	{
	bat[in]._max_temper_cnt--;
	}
#line 8470 "control.c"

gran(&bat[in]._max_temper_cnt,0,600);
if(bat[in]._max_temper_cnt>=590)	bat[in]._temper_stat|=(1<<1);
if(bat[in]._max_temper_cnt<=10)	bat[in]._temper_stat&=~(1<<1);

if((bat[in]._temper_stat&(1<<1))&&(!(bat[in]._temper_stat&(1<<5))))	avar_bat_temper_hndl(in,3);
if((!(bat[in]._temper_stat&(1<<1)))&&(bat[in]._temper_stat&(1<<5)))	avar_bat_temper_hndl(in,2);

if(bat[in]._temper_stat&(1<<1))		bat[in]._temper_stat|=(1<<5);
else 								bat[in]._temper_stat&=~(1<<5);


if(bat[in]._resurs_cnt<36000)
	{               
	bat[in]._resurs_cnt++;
	if(bat[in]._resurs_cnt>=36000)
		{
		bat[in]._resurs_cnt=0;
		lc640_write_int(ADR_EE_BAT_RESURS[in],lc640_read_int(ADR_EE_BAT_RESURS[in])+1);
		}
	}
else bat[in]._resurs_cnt=0;


#line 8551 "control.c"

}



void bat_hndl_zvu(void)
{
char i;
short bat_hndl_i_temp;
const long bat_hndl_t_razr_const[7]={600L,1800L,3600L,10800L,18000L,36000L,72000L};


 
if(bat_hndl_zvu_init==0)
	{
	
	bat_hndl_zvu_Q=(long)lc640_read_int(0x10+450+12);
	if((bat_hndl_zvu_Q>100L)||(bat_hndl_zvu_Q<0L)) bat_hndl_zvu_Q=100L;
	bat_hndl_zvu_Q*=10000L;

	bat_hndl_zvu_init=1;
	}
else 
	{
	if(Ib_ips_termokompensat<-IKB)
		{
		bat_hndl_i_vector=0;
		bat_hndl_i_zar_price=0L;
			
		bat_hndl_i=-Ib_ips_termokompensat;
		I_from_t_table[0]=BAT_C_POINT_1_6*6; 
		I_from_t_table[1]=BAT_C_POINT_1_2*2; 
		I_from_t_table[2]=BAT_C_POINT_1; 
		I_from_t_table[3]=BAT_C_POINT_3/3; 
		I_from_t_table[4]=BAT_C_POINT_5/5; 
		I_from_t_table[5]=BAT_C_POINT_10/10; 
		I_from_t_table[6]=BAT_C_POINT_20/20; 
		
		I_from_t_table[0]=(short)((((long)I_from_t_table[0])*((long)BAT_K_OLD))/100L);
		I_from_t_table[1]=(short)((((long)I_from_t_table[1])*((long)BAT_K_OLD))/100L);
		I_from_t_table[2]=(short)((((long)I_from_t_table[2])*((long)BAT_K_OLD))/100L);
		I_from_t_table[3]=(short)((((long)I_from_t_table[3])*((long)BAT_K_OLD))/100L);
		I_from_t_table[4]=(short)((((long)I_from_t_table[4])*((long)BAT_K_OLD))/100L);
		I_from_t_table[5]=(short)((((long)I_from_t_table[5])*((long)BAT_K_OLD))/100L);
		I_from_t_table[6]=(short)((((long)I_from_t_table[6])*((long)BAT_K_OLD))/100L);

		bat_hndl_i_temp=bat_hndl_i/10;
		for(i=0;i<7;i++)
			{
			if(bat_hndl_i_temp>=I_from_t_table[i])
				{
				break;
				}
			}
		 if(i==0) bat_hndl_t_razr=bat_hndl_t_razr_const[0];
		 else if((i>=1)&&(i<7))
		 	{
			short i1,i2;
			i1=I_from_t_table[i-1]-bat_hndl_i_temp;
			i2=I_from_t_table[i-1]-I_from_t_table[i];
			bat_hndl_t_razr=bat_hndl_t_razr_const[i]-bat_hndl_t_razr_const[i-1];
			bat_hndl_t_razr*=(long)i1;
			bat_hndl_t_razr/=(long)i2;
			bat_hndl_t_razr+=bat_hndl_t_razr_const[i-1];
			}
		else if(i>=7)
			{
			bat_hndl_t_razr=bat_hndl_t_razr_const[6];
			}
		bat_hndl_proc_razr=1000000L/bat_hndl_t_razr;

		if(bat_hndl_zvu_Q>bat_hndl_proc_razr)bat_hndl_zvu_Q-=bat_hndl_proc_razr;
		else bat_hndl_zvu_Q=0L;

		bat_hndl_t_razr_hour=(short)(bat_hndl_remain_time/3600L);
		bat_hndl_t_razr_min=(short)(bat_hndl_remain_time/60L);
		bat_hndl_t_razr_mininhour=bat_hndl_t_razr_min%60L;

		}
	else if(Ib_ips_termokompensat>IKB)
		{
		bat_hndl_i_vector=1;

		bat_hndl_i=Ib_ips_termokompensat;
		bat_hndl_i_summ+=(long)bat_hndl_i;
		if(bat_hndl_i_summ>=36000L)

		
		
			{
			bat_hndl_i_summ-=36000L;
			if(bat_hndl_zvu_Q<1000000L)bat_hndl_zvu_Q+=bat_hndl_i_zar_price;
			else bat_hndl_zvu_Q=1000000L; 
			}
		}


	if(bat_hndl_i_vector!=bat_hndl_i_vector_old)
		{
		if(bat_hndl_i_vector==1)
			{
			signed short tempSS;
			tempSS=lc640_read_int(0x10+350+90);
			bat_hndl_i_zar_price=(bat_hndl_zvu_Q-1000000L)/((long)tempSS);
			bat_hndl_i_summ=0;
			}
		}
	bat_hndl_i_vector_old=bat_hndl_i_vector;

	if((bat_hndl_zvu_Q/10000L)!=lc640_read_int(0x10+450+12)) lc640_write_int(0x10+450+12,bat_hndl_zvu_Q/10000L);
	bat_hndl_remain_time=bat_hndl_zvu_Q/bat_hndl_proc_razr;
	}

if(bat_hndl_zvu_Q>1000000L)	bat_hndl_zvu_Q=1000000L;
else if(bat_hndl_zvu_Q<0L)	bat_hndl_zvu_Q=0L;

if((Ib_ips_termokompensat/10<(2*IKB))&&(Ib_ips_termokompensat/10>(-2*IKB))&&(!(bat[0]._av&0x01))&& (out_U<=u_necc_up) && (out_U>u_necc_dn) && (main_kb_cnt>=10) && (main_kb_cnt<=200)    )
	{
	if(bat_hndl_zvu_Q_cnt<60)
		{
		bat_hndl_zvu_Q_cnt++;
		if(bat_hndl_zvu_Q_cnt>=60)
			{
			lc640_write_int(0x10+450+12,100);
			bat_hndl_zvu_Q=1000000L;


			}
		}
	}
else 
	{
	bat_hndl_zvu_Q_cnt=0;
	}



};





void u_avt_set_hndl(void)
{
if(uavt_set_stat==uassSTEP1)
	{
	char i,find_succes;

	
	
	if(uavt_set_stat_old!=uavt_set_stat)
		{
		uavt_set_error_cnt=60;
		}
	
	mess_send(225,230,0,10);
	find_succes=0;








 
	if((bps_U>=U_AVT-1)&&(bps_U<=U_AVT+1))find_succes=1;

	if(find_succes==1)
		{
		uavt_set_stat=uassSTEP2;
		}
	if(uavt_set_error_cnt)
		{
		uavt_set_error_cnt--;
		if(!uavt_set_error_cnt)
			{
			uavt_set_stat=uassOFF;
			uavt_set_result_stat=uasrsERR;
			avt_error_bps=100;
			}
		}
	}
if(uavt_set_stat==uassSTEP2)
	{
	char i,find_succes;

	u_max_temp=(short)((((long)U_AVT)*101U)/100U);
	u_min_temp=(short)((((long)U_AVT)*99U)/100U);

	if(uavt_set_stat_old!=uavt_set_stat)
		{
		uavt_set_error_cnt=60;
		}

	mess_send(190,191,U_AVT,10);
	mess_send(225,230,0,10);
	find_succes=1;

	for(i=0;i<NUMIST;i++)
		{
		if((bps[i]._Uii<u_max_temp)&&(bps[i]._Uii>u_min_temp))continue;
		else
			{
			find_succes=0;
			avt_error_bps=i+1;
			break;
			}
		}

	if(find_succes==1)
		{
		uavt_set_stat=uassSTEP3;
		uavt_bps_pntr=0;
		avt_plazma=0;
		}

	if(uavt_set_error_cnt)
		{
		uavt_set_error_cnt--;
		if(!uavt_set_error_cnt)
			{
			uavt_set_stat=uassOFF;
			uavt_set_result_stat=uasrsERR;
			}
		}
	}
else if(uavt_set_stat==uassSTEP3)
	{
	char i,find_succes;

	u_max_temp=(short)((((long)U_AVT)*101U)/100U);
	u_min_temp=(short)((((long)U_AVT)*99U)/100U);
	
	if(uavt_set_stat_old!=uavt_set_stat)
		{
		uavt_bps_pntr=0;
		}

	mess_send(190,191,U_AVT,10);
	mess_send(225,230,0,10);

	find_succes=1;
	for(i=0;i<NUMIST;i++)
		{
		if((bps[i]._Uii<u_max_temp)&&(bps[i]._Uii>u_min_temp))continue;
		else
			{
			find_succes=0;
			break;
			}
		}

	if(mess_find( (215)) && (mess_data[0]==217) )
		{
		if(++uavt_bps_pntr>=NUMIST)
			{
			uavt_set_stat=uassOFF;
			uavt_set_result_stat=uasrsSUCCESS;
			}
		
		
		}
	if(find_succes==1)
		{
		mcp2515_transmit(uavt_bps_pntr,uavt_bps_pntr,0x16,0xee,0xee,0,0,0);
		avt_plazma++;
		}
	}



uavt_set_stat_old=uavt_set_stat;

}




void u_avt_set_hndl1(void)
{
if(uavt_set_stat==uassSTEP1)
	{
	char i,find_succes;

	u_max_temp=(short)((((long)U_AVT)*101U)/100U);
	u_min_temp=(short)((((long)U_AVT)*99U)/100U);
	if(uavt_set_stat_old!=uavt_set_stat)
		{
		uavt_bps_pntr=0;

		}
	
	mess_send(225,230,0,10);
	find_succes=1;
	for(i=0;i<NUMIST;i++)
		{
		if((bps[i]._Uii<u_max_temp)&&(bps[i]._Uii>u_min_temp))continue;
		else
			{
			find_succes=0;
			break;
			}
		}

	if(find_succes==1)
		{
		uavt_set_stat=uassSTEP2;
		}
	}
else if(uavt_set_stat==uassSTEP2)
	{
	char i,find_succes;

	u_max_temp=(short)((((long)U_AVT)*101U)/100U);
	u_min_temp=(short)((((long)U_AVT)*99U)/100U);
	
	if(uavt_set_stat_old!=uavt_set_stat)
		{
		uavt_bps_pntr=0;
		}

	
	mess_send(225,230,0,10);

	find_succes=1;
	for(i=0;i<NUMIST;i++)
		{
		if((bps[i]._Uii<u_max_temp)&&(bps[i]._Uii>u_min_temp))continue;
		else
			{
			find_succes=0;
			break;
			}
		}

	if(find_succes==1)
		{
		mcp2515_transmit(uavt_bps_pntr,uavt_bps_pntr,0x16,0xee,0xee,0,0,0);
		if(++uavt_bps_pntr>=NUMIST)uavt_set_stat=uassOFF;
		}
	}



uavt_set_stat_old=uavt_set_stat;

}



void u_necc_hndl(void)
{
signed long temp_L;
signed long temp_SL;





signed short t[2];






if(!TERMOKOMPENS)
	{
	
	u_necc=UB20;
	}
else
	{
	if(ND_EXT[0])t[0]=20;
	else t[0]=t_ext[0];

	mat_temper=t[0];
			
	if(mat_temper<0)temp_SL=UB0; 
	else 
		{
		if(mat_temper>40)mat_temper=40; 
		temp_SL=(UB20-UB0)*10;
		temp_SL*=mat_temper;
		temp_SL/=200;
		temp_SL+=UB0;
		}
	if((spc_stat==spcVZ)&&((sk_stat[0]==1)||(VZ_CH_VENT_BLOK==0)))
		{
		temp_SL=UVZ;
		}
	u_necc=(unsigned int)temp_SL;
	
	}  



if((speedChIsOn)||(sp_ch_stat==scsWRK))
	{
	u_necc=speedChrgVolt;
	}
if(hv_vz_stat==hvsWRK)
	{
	u_necc=UVZ;
	}
if(vz1_stat==vz1sWRK)
	{
	u_necc=UZ_U;
	}
if(vz2_stat==vz2sWRK1)
	{
	u_necc=FZ_U1;
	}
if(vz2_stat==vz2sWRK2)
	{
	u_necc=FZ_U2;
	}

if((uavt_set_stat==uassSTEP1)||(uavt_set_stat==uassSTEP2))
	{
	u_necc=U_AVT;
	}

if(mess_find_unvol(190))
	{		
	if(mess_data[0]==191)
		{
		u_necc=mess_data[1];
		}		
	} 





#line 9452 "control.c"



temp_L=(signed long) u_necc;
temp_L*=98L;
temp_L/=100L;
u_necc_dn=(signed short)temp_L;

temp_L=(signed long) u_necc;
temp_L*=102L;
temp_L/=100L;
u_necc_up=(signed short)temp_L;



 
}



void num_necc_hndl(void)
{

static short num_necc_block_cnt;
if(num_necc_block_cnt) num_necc_block_cnt--;

Isumm_=Isumm;

if(bat[0]._Ib<0) Isumm_+=(abs(bat[0]._Ib))/10;
if(bat[1]._Ib<0) Isumm_+=(abs(bat[1]._Ib))/10;

num_necc_up=(Isumm_/((signed short)IMAX))+1;



num_necc_down=(Isumm_/((signed short)IMIN))+1;

if(num_necc_up>num_necc)
	{
	num_necc=num_necc_up;
	num_necc_block_cnt=60;
	}
else if(num_necc_down<num_necc)
	{
	if(!num_necc_block_cnt)
		{
		num_necc=num_necc_down;
		num_necc_block_cnt=60;
		}
	}

if(PAR) num_necc=NUMIST;

if(bPARALLEL) num_necc=NUMIST;
if(vz1_stat==vz1sWRK)num_necc=NUMIST; 
if((vz2_stat==vz2sWRK1)||(vz2_stat==vz2sWRK2))num_necc=NUMIST; 


gran(&num_necc,1,NUMIST);

}




void cntrl_hndl(void)
{



IZMAX_=IZMAX;



if((speedChIsOn)||(sp_ch_stat==scsWRK))IZMAX_=speedChrgCurr;
if(vz1_stat==vz1sWRK) IZMAX_=UZ_IMAX;
if(vz2_stat==vz2sWRK1) IZMAX_=FZ_IMAX1;
if(vz2_stat==vz2sWRK2) IZMAX_=FZ_IMAX2;


if(cntrl_stat_blok_cnt)cntrl_stat_blok_cnt--;
if(cntrl_stat_blok_cnt_)cntrl_stat_blok_cnt_--;

if((bat[0]._temper_stat&0x03)||(bat[1]._temper_stat&0x03))IZMAX_=IZMAX_/10;



if((REG_SPEED<1)||(REG_SPEED>5)) REG_SPEED=1;
if(ch_cnt0<(10*REG_SPEED))
	{
	ch_cnt0++;
	if(ch_cnt0>=10*REG_SPEED)
		{
		ch_cnt0=0;
		b1Hz_ch=1;
		}
	}
#line 9561 "control.c"


if(mess_find_unvol(225))
	{
	if(mess_data[0]==100)
		{
		cntrl_stat =cntrl_stat_old+mess_data[1];
		}
	else if(mess_data[0]==105)
		{
		cntrl_stat =cntrl_stat_old-mess_data[1];
		}
	else if(mess_data[0]==110)
		{
		static char cntrlStatIsDownCnt;
		cntrl_stat--;

		if((cntrl_stat<=30)||(load_U<USIGN))
			{
			if(++cntrlStatIsDownCnt==250)mess_send(230,231,0,10);
			}
		else 
			{
			cntrlStatIsDownCnt=0;
			}

		}
	else if(mess_data[0]==229)
		{
		cntrl_stat =mess_data[1];
		}

	else if(mess_data[0]==230)
		{

		if(bps_U>u_necc)
			{
			cntrl_hndl_plazma=11;
			if(((bps_U-u_necc)>40)&&(cntrl_stat >0))cntrl_stat -=5;
			else if((cntrl_stat )&&b1Hz_ch)cntrl_stat --;
			}
		else if(bps_U<u_necc)
			{
			cntrl_hndl_plazma=12;	
			if(((u_necc-bps_U)>40)&&(cntrl_stat <1015))cntrl_stat +=5;
			else	if((cntrl_stat <1020)&&b1Hz_ch)cntrl_stat ++;
			}
#line 9649 "control.c"
	 	}

	

 
	}


else if((b1Hz_ch)&&((!bIBAT_SMKLBR)||(bps[8]._cnt>40)))
	{
	cntrl_stat_new=cntrl_stat_old;
	cntrl_hndl_plazma=19;
	if((Ibmax/10)>(2*IZMAX_))
		{
		cntrl_hndl_plazma=20;
          if(cntrl_stat_blok_cnt)cntrl_stat_new--;
		else	cntrl_stat_new-=10;
		}		
	else if(((Ibmax/10)<(IZMAX_*2))&&(Ibmax>(IZMAX_*15)))
		{
		cntrl_hndl_plazma=21;
          if(cntrl_stat_blok_cnt)cntrl_stat_new--;
          else	cntrl_stat_new-=3;
		}   
	else if((Ibmax<(IZMAX_*15))&&((Ibmax/10)>IZMAX_))
		{
		cntrl_hndl_plazma=22;
		cntrl_stat_new--;
		}
		
	else if(bps_U<u_necc)
		{
		cntrl_hndl_plazma=23;
		if(bps_U<(u_necc-(UB0-UB20)))
			{
			cntrl_hndl_plazma=24;
			if(Ibmax<0)
				{
				cntrl_hndl_plazma=25;
                    if(cntrl_stat_blok_cnt)cntrl_stat_new++;
				else cntrl_stat_new+=10;
				}
			else if(Ibmax<(IZMAX_*5))
				{
				cntrl_hndl_plazma=26;
                    if(cntrl_stat_blok_cnt)cntrl_stat_new++;
				else	cntrl_stat_new+=2;
				}
			else if(Ibmax<((IZMAX_*10) ))
				{
				cntrl_hndl_plazma=27;
				cntrl_stat_new++;
				}					
			}
		else if(bps_U<(u_necc-((UB0-UB20)/4)))
			{
			cntrl_hndl_plazma=28;
			if(Ibmax<(IZMAX_*5))
				{
				cntrl_hndl_plazma=29;
                    if(cntrl_stat_blok_cnt)cntrl_stat_new++;
				else	cntrl_stat_new+=2;
				}
			else if(Ibmax<((IZMAX_*10) ))
				{
				cntrl_hndl_plazma=30;
				cntrl_stat_new++;
				}					
			}	
		else if(bps_U<(u_necc-1))
			{
			cntrl_hndl_plazma=31;
			if(Ibmax<((IZMAX_*10) ))
				{
				cntrl_hndl_plazma=32;
				cntrl_stat_new++;
				}					
			}					
		}	
	else if((bps_U>u_necc) )
		{ 	
		cntrl_hndl_plazma=33;
		if(bps_U>(u_necc+(UB0-UB20)))
			{
			cntrl_hndl_plazma=34;
               if((cntrl_stat_blok_cnt)||(!TERMOKOMPENS))cntrl_stat_new--;
			else	cntrl_stat_new-=10;
			}
		else if(bps_U>(u_necc+((UB0-UB20)/4)))
			{
			cntrl_hndl_plazma=35;
               if(cntrl_stat_blok_cnt)cntrl_stat_new--;
			else cntrl_stat_new-=2;
			}	
		else if(bps_U>(u_necc+1))
			{
			cntrl_hndl_plazma=36;
			cntrl_stat_new--;
			}					
		}

	if((hv_vz_stat==hvsOFF)&&(vz1_stat==vz1sOFF)&&(vz2_stat==vz2sOFF))
		{
		if(((sk_stat[1]==1)&&(sk_stat_old[1]==0))&&(VZ_KIND==1))cntrl_stat_new=50;
		}

	gran(&cntrl_stat_new,10,1010);
	if(net_av_2min_timer)cntrl_stat_new=cntrl_stat_old;			
	cntrl_stat_old=cntrl_stat_new;
	cntrl_stat=cntrl_stat_new;
	
	if(ICA_EN==0)
		{
		if(ica_cntrl_hndl_cnt)
			{
			cntrl_stat = ica_cntrl_hndl;
			cntrl_stat_new = ica_cntrl_hndl;
			cntrl_stat_old = ica_cntrl_hndl;
			}
		}
	
	if((ICA_EN==1)||(ICA_EN==2))
		{
		cntrl_stat=cntrl_stat_new+ica_u_necc;
		}			
	}
#line 9876 "control.c"

iiii=0;
for(i=0;i<NUMIST;i++)
     {
     if(bps[i]._cnt<30)iiii=1;
     }

if(iiii==0)
	{
	cntrl_stat=620;	
	cntrl_stat_old=620;
	cntrl_stat_new=620;
	cntrl_stat=10*PWM_START;
	cntrl_stat_old=10*PWM_START;
	cntrl_stat_new=10*PWM_START;
	}


if(ica_cntrl_hndl_cnt)	ica_cntrl_hndl_cnt--;






gran(&cntrl_stat,10,1022); 
b1Hz_ch=0;
}


#line 10148 "control.c"

#line 10392 "control.c"

#line 10646 "control.c"


void ext_drv(void)
{
char i;





for(i=0;i<NUMSK;i++)
	{
#line 10683 "control.c"
	if(adc_buff_[sk_buff_220[i]]<2000)
#line 10697 "control.c"
		{
		if(sk_cnt[i]<10)
			{
			sk_cnt[i]++;
			if(sk_cnt[i]>=10)
				{
				sk_stat[i]=ssON;
				}
			}
		else 
			{
			sk_cnt[i]=10;
			}
               
		}
	else
		{
		if(sk_cnt[i]>0)
			{
			sk_cnt[i]--;
			if(sk_cnt[i]<=0)
				{
				sk_stat[i]=ssOFF;
				}
			}
		else 
			{
			sk_cnt[i]=0;
			}
		}
	}

for(i=0;i<NUMSK;i++)
	{
	if(((SK_SIGN[i]==0)&&(sk_stat[i]==ssON))||((SK_SIGN[i])&&(sk_stat[i]==ssOFF)) )
		{
		if(sk_av_cnt[i]<10)
			{
			sk_av_cnt[i]++;
			if(sk_av_cnt[i]>=10)
				{
				sk_av_stat[i]=sasON;
				}
			}
		else 
			{
			sk_av_cnt[i]=10;
			}
		}
	else
		{
		if(sk_av_cnt[i]>=0)
			{
			sk_av_cnt[i]--;
			if(sk_av_cnt[i]<=0)
				{
				sk_av_stat[i]=sasOFF;
				}
			}
		else 
			{
			sk_av_cnt[i]=0;
			}
		}


	if(sk_av_stat_old[i]!=sk_av_stat[i])
		{
		plazma_sk++;
		if(sk_av_stat[i]==sasON)
			{
			if(i==0)snmp_trap_send("SK #1 Alarm",15,1,1);
			else if(i==1)
				{

				snmp_trap_send("SK #2 Alarm",15,2,1);




				}
			else if(i==2)snmp_trap_send("SK #3 Alarm",15,3,1);
			else if(i==3)snmp_trap_send("SK #4 Alarm",15,4,1);
			}
		else 
			{
			if(i==0)snmp_trap_send("SK #1 Alarm is off",15,1,0);
			else if(i==1)
				{

				snmp_trap_send("SK #2 Alarm is off",15,2,0);




				}
			else if(i==2)snmp_trap_send("SK #3 Alarm is off",15,3,0);
			else if(i==3)snmp_trap_send("SK #4 Alarm is off",15,4,0);
			}
	 	}


#line 10819 "control.c"
	sk_av_stat_old[i]=sk_av_stat[i];
	}
}



void zar_superviser_drv(void)
{

if(((bat[0]._Ub>u_necc_up) || (bat[0]._Ub<u_necc_dn))&&(main_kb_cnt<((TBAT*60)-30))&&(main_kb_cnt>((TBAT*60)-150))) sign_U[0]=0;

if(((bat[0]._Ib>(2*IKB)) || (bat[0]._Ib<(-IKB*2))) &&(main_kb_cnt<((TBAT*60)-30))&&(main_kb_cnt>((TBAT*60)-150))) sign_I[0]=0;
																 
if((main_kb_cnt==((TBAT*60)-10)) &&(spc_stat==spcOFF))
	{
	if((sign_U[0]==1) && (sign_I[0]==1) && (lc640_read_int(0x10+450+12)!=BAT_C_REAL[0]) && (NUMBAT) && (!(bat[0]._av&1)))
		{
		lc640_write_int(0x10+450+12,BAT_C_REAL[0]);
		superviser_cnt++;
		}
	
	}

if(((bat[0]._Ub>u_necc_up) || (bat[1]._Ub<u_necc_dn))&&(main_kb_cnt<((TBAT*60)-30))&&(main_kb_cnt>((TBAT*60)-150))) sign_U[1]=0;

if(((bat[1]._Ib>(2*IKB)) || (bat[1]._Ib<(-IKB*2))) &&(main_kb_cnt<((TBAT*60)-30))&&(main_kb_cnt>((TBAT*60)-150))) sign_I[1]=0;
																 
if((main_kb_cnt==((TBAT*60)-10)) &&(spc_stat==spcOFF))
	{
	if((sign_U[1]==1) && (sign_I[1]==1) && (lc640_read_int(0x10+450+42)!=BAT_C_REAL[1]) && (NUMBAT==2) && (!(bat[1]._av&1)))
		{
		lc640_write_int(0x10+450+42,BAT_C_REAL[1]);
		superviser_cnt++;
		}
	
	}

if(main_kb_cnt==((TBAT*60)-2)) zar_superviser_start();
}


void zar_superviser_start(void)
{
sign_U[0]=1;
sign_I[0]=1;
sign_U[1]=1;
sign_I[1]=1;

}


void npn_hndl(void)
{
if(NPN_OUT!=npnoOFF)
	{
























 
























 

	if((load_U<UONPN)&&((net_Ua<UMN)||(net_Ub<UMN)||(net_Uc<UMN)))
		{
		if(npn_tz_cnt<TZNPN)
			{
			npn_tz_cnt++;
			if(npn_tz_cnt==TZNPN)
				{
				npn_stat=npnsOFF;
				}
			}
		}
	else if((load_U>UVNPN)&&(net_Ua>UMN)&&(net_Ub>UMN)&&(net_Uc>UMN))
		{
		if(npn_tz_cnt)
			{
			npn_tz_cnt--;
			if(npn_tz_cnt==0)
				{
				npn_stat=npnsON;
				}
			}
		}
	}
else
	{
	npn_tz_cnt=0;
	npn_stat=npnsON;
	}

if(npn_stat==npnsOFF) mess_send(210,110,1,15);


}



void loadoff_hndl(void)
{
if((load_U>UONPN)||(load_U<UVNPN))
	{
	if(load_off_cnt<TZNPN)
		{
		load_off_cnt++;
		if(load_off_cnt>=TZNPN)
			{
			load_off_stat=npnsOFF;
			load_off_cnt=TZNPN;
			}
		}
	}
else if((load_U>(UVNPN+dUNPN))&&(load_U<(UONPN-dUNPN)))
	{
	if(load_off_cnt)
		{
		load_off_cnt--;
		if(load_off_cnt<=0)
			{
			load_off_stat=npnsON;
			load_off_cnt=0;
			}
		}
	}


if(load_off_stat==npnsOFF) tloaddisable_cmnd=10;


}


void speedChargeHndl(void)
{




 
	 
if(sp_ch_stat==scsSTEP1)
	{
	if(sp_ch_stat_old!=sp_ch_stat)
		{
		sp_ch_stat_cnt=5;
		if(SP_CH_VENT_BLOK==0)
			{
			sp_ch_stat_cnt=0;
			sp_ch_stat=scsWRK;
			}
		}
	if(sp_ch_stat_cnt)
		{
		sp_ch_stat_cnt--;
		if(sp_ch_stat_cnt==0)
			{
			sp_ch_stat=scsERR1; 	

			}
		}
	if(sk_stat[0]==1)sp_ch_stat=scsWRK;
	}

if(sp_ch_stat==scsWRK)
	{
	if(sp_ch_stat_old!=sp_ch_stat)
		{
		sp_ch_wrk_cnt=(signed long)speedChrgTimeInHour*3600L;
		hv_vz_up_cnt=0;
		}
	sp_ch_wrk_cnt--;
	hv_vz_up_cnt++;
	if(sp_ch_wrk_cnt==0)
		{
		sp_ch_stat=scsOFF;
		speedz_mem_hndl(0);
		}

	if((sk_stat[0]==0)&&(SP_CH_VENT_BLOK==1))sp_ch_stat=scsERR2;

	}

if(sp_ch_stat==scsERR1)		
	{
	if((sp_ch_stat_old!=sp_ch_stat)||(sp_ch_stat_cnt==0))
		{
		sp_ch_stat_cnt=10;
		}
	sp_ch_stat_cnt--;
	if((sp_ch_stat_cnt==10)||(sp_ch_stat_cnt==9))
		{
		show_mess(	"  ���������� �����  ",
					"   �� ����� ����    ",
					"      �������       ",
					"  ��� ����������!!  ",
					5000);
		}
	}
if(sp_ch_stat==scsERR2)		
	{
	if((sp_ch_stat_old!=sp_ch_stat)||(sp_ch_stat_cnt==0))
		{
		sp_ch_stat_cnt=10;
		}
	sp_ch_stat_cnt--;
	if((sp_ch_stat_cnt==10)||(sp_ch_stat_cnt==9))
		{
		show_mess(	"  ���������� �����  ",
					"    ������������    ",
					"     ����������     ",
					"    ����������!!!   ",
					5000);
		}
	if(sk_stat[0]==1)sp_ch_stat=scsWRK;
	}


sp_ch_stat_old=sp_ch_stat;



if(speedChrgAvtEn==1)
	{
	if((sp_ch_stat==scsOFF)&&(spc_stat==spcOFF)

		&&(vz1_stat==vz1sOFF)&&(vz2_stat==vz2sOFF)

		)
		{
		if((load_U<u_necc)&&((u_necc-load_U)>speedChrgDU)

		&&(abs(Ib_ips_termokompensat/10-IZMAX)<5)




		&&(!speedChrgBlckStat))
			{
			speedChargeStartCnt++;
			if(speedChargeStartCnt>=60)
				{
				speedChargeStartStop();
				speedz_mem_hndl(5);
				}
			}
		else speedChargeStartCnt=0;
		}
	else speedChargeStartCnt=0;
	}
































 
if( (speedChrgBlckSrc!=2)) speedChrgBlckStat=0;
else
	{
	speedChrgBlckStat=0;
	if(speedChrgBlckSrc==1)
		{
		if(((speedChrgBlckLog==0)&&(adc_buff_[11]>2000)) || ((speedChrgBlckLog==1)&&(adc_buff_[11]<2000))) speedChrgBlckStat=1;
		}
	else if(speedChrgBlckSrc==2)
		{
		if(((speedChrgBlckLog==0)&&(adc_buff_[13]>2000)) || ((speedChrgBlckLog==1)&&(adc_buff_[13]<2000))) speedChrgBlckStat=1;
		}
	}


















 


}


void speedChargeStartStop(void)
{
spch_plazma[1]++;



















 

if(sp_ch_stat!=scsOFF)
	{
	sp_ch_stat=scsOFF;
	speedz_mem_hndl(10);
	spch_plazma[1]=10;
	}

else
	{
	spch_plazma[1]=20;
	if((speedChrgBlckStat==0)&&(spc_stat==spcOFF)

		&&(vz1_stat==vz1sOFF)&&(vz2_stat==vz2sOFF)

		)
		{

		sp_ch_stat=scsSTEP1;



		speedz_mem_hndl(1);
		}
	else 
		{
		show_mess(	"     ����������     ",
	          		"       �����        ",
	          		"    ������������!   ",
	          		"                    ",2000);
		}
	}
}


void averageChargeHndl(void)
{




 
if(hv_vz_stat!=hvsOFF)
	{
	hv_vz_stat=hvsOFF; 	
	lc640_write(0x10+100+244,hvsOFF);
	}

if(hv_vz_stat==hvsSTEP1)
	{
	if(hv_vz_stat_old!=hv_vz_stat)
		{
		hv_vz_stat_cnt=5;
		}
	if(hv_vz_stat_cnt)
		{
		hv_vz_stat_cnt--;
		if(hv_vz_stat_cnt==0)
			{
			hv_vz_stat=hvsERR1; 	
			lc640_write(0x10+100+244,hvsERR1);
			}
		}
	if(sk_stat[0]==1)
		{
		hv_vz_stat=hvsSTEP2;
		lc640_write(0x10+100+244,hvsSTEP2);
		tree_up(iHV_STEP2_2,1,0,0);
		tree_up(iHV_STEP2_1,0,0,0);
		ret(1200);
		}
	}

if(hv_vz_stat==hvsSTEP2)
	{
	if((hv_vz_stat_old!=hv_vz_stat)||(hv_vz_stat_cnt==0))
		{
		hv_vz_stat_cnt=15;
		}
	hv_vz_stat_cnt--;
	
	

	
	
	
	
	
	
	
	
	
	
	
	
	




 
	}

if(hv_vz_stat==hvsSTEP3)
	{
	if((hv_vz_stat_old!=hv_vz_stat)||(hv_vz_stat_cnt==0))
		{
		hv_vz_stat_cnt=10;
		}
	hv_vz_stat_cnt--;
	if((hv_vz_stat_cnt==10)||(hv_vz_stat_cnt==9))
		{
		show_mess(	"     ��������       ",
					"      �������       ",
					"   �������������    ",
					"       �����        ",
					5000);
		}
	if(sk_stat[1]==1)
		{
		hv_vz_stat=hvsWRK;
		lc640_write(0x10+100+244,hvsWRK);
		}
	}

if(hv_vz_stat==hvsWRK)
	{
	if(hv_vz_stat_old!=hv_vz_stat)
		{
		hv_vz_wrk_cnt=3600L *((long)VZ_HR);
		if(VZ_HR==0)  	hv_vz_wrk_cnt=1800L;
		hv_vz_up_cnt=0L;
		}
	hv_vz_wrk_cnt--;
	hv_vz_up_cnt++;

	if(hv_vz_wrk_cnt==0)
		{
		hv_vz_stat=hvsERR4;
		lc640_write(0x10+100+244,hvsERR4);
		}
	if(sk_stat[0]==0)
		{
		hv_vz_stat=hvsERR2;
		lc640_write(0x10+100+244,hvsERR2);
		}
	if(sk_stat[1]==0)
		{
		hv_vz_stat=hvsERR3;
		lc640_write(0x10+100+244,hvsERR3);
		}
	}

if(hv_vz_stat==hvsERR1)		
	{
	if((hv_vz_stat_old!=hv_vz_stat)||(hv_vz_stat_cnt==0))
		{
		hv_vz_stat_cnt=10;
		}
	hv_vz_stat_cnt--;
	if((hv_vz_stat_cnt==10)||(hv_vz_stat_cnt==9))
		{
		show_mess(	"������������� ����� ",
					"   �� ����� ����    ",
					"      �������       ",
					"  ��� ����������!!  ",
					5000);
		}
	}
if(hv_vz_stat==hvsERR2)		
	{
	if((hv_vz_stat_old!=hv_vz_stat)||(hv_vz_stat_cnt==0))
		{
		hv_vz_stat_cnt=10;
		}
	hv_vz_stat_cnt--;
	if((hv_vz_stat_cnt==10)||(hv_vz_stat_cnt==9))
		{
		show_mess(	"������������� ����� ",
					"    ������������    ",
					"     ����������     ",
					"    ����������!!!   ",
					5000);
		}
	if(sk_stat[0]==1)
		{
		hv_vz_stat=hvsWRK;
		lc640_write(0x10+100+244,hvsWRK);
		}
	}

if(hv_vz_stat==hvsERR3)		
	{
	if((hv_vz_stat_old!=hv_vz_stat)||(hv_vz_stat_cnt==0))
		{
		hv_vz_stat_cnt=10;
		}
	hv_vz_stat_cnt--;
	if((hv_vz_stat_cnt==10)||(hv_vz_stat_cnt==9))
		{
		show_mess(	"������������� ����� ",
					"  ����� ���������   ",
					"  ����� ���������   ",
					"    ��������!!!     ",
					5000);
		}
	if(sk_stat[1]==1)
		{
		hv_vz_stat=hvsWRK;
		lc640_write(0x10+100+244,hvsWRK);
		}
	}
if(hv_vz_stat==hvsERR4)		
	{
	if((hv_vz_stat_old!=hv_vz_stat)||(hv_vz_stat_cnt==0))
		{
		hv_vz_stat_cnt=10;
		}
	hv_vz_stat_cnt--;
	if((hv_vz_stat_cnt==10)||(hv_vz_stat_cnt==9))
		{
		show_mess(	"     ���������      ",
					"      �������       ",
					"   �������������    ",
					"       �����        ",
					5000);
		}
	if(sk_stat[1]==0)
		{
		hv_vz_stat=hvsOFF;
		lc640_write(0x10+100+244,hvsOFF);
		vz_stop();

		}
	}
hv_vz_stat_old=hv_vz_stat;































































 
}


void averageChargeStartStop(void)
{
if(hv_vz_stat!=hvsOFF)
	{
	hv_vz_stat=hvsOFF;
	lc640_write(0x10+100+244,hvsOFF);
	}

else
	{
	hv_vz_stat=hvsSTEP1;
	lc640_write(0x10+100+244,hvsSTEP1);
	}
}


void	numOfForvardBps_hndl(void)			
{

numOfForvardBps_old=numOfForvardBps;

numOfForvardBps=0;



if((FORVARDBPSCHHOUR<=0)||(FORVARDBPSCHHOUR>500))
	{
	FORVARDBPSCHHOUR=0;
	return;
	}

numOfForvardBps_minCnt++;


if(numOfForvardBps_minCnt>=60)
	{
	numOfForvardBps_minCnt=0;
	numOfForvardBps_hourCnt=lc640_read_int(0x10+100+180);
	numOfForvardBps_hourCnt++;
	if(numOfForvardBps_hourCnt>=(FORVARDBPSCHHOUR*NUMIST))
		{
		numOfForvardBps_hourCnt=0;
		}
	lc640_write_int(0x10+100+180,numOfForvardBps_hourCnt);
	}

numOfForvardBps=numOfForvardBps_hourCnt/FORVARDBPSCHHOUR;



}


void	numOfForvardBps_init(void)			
{									
lc640_write_int(0x10+100+180,0);
numOfForvardBps_minCnt=58;
}


void outVoltContrHndl(void)
{ 
if((load_U>U_OUT_KONTR_MAX)||(load_U<U_OUT_KONTR_MIN))
	{
	outVoltContrHndlCnt_=0;
	if(outVoltContrHndlCnt<U_OUT_KONTR_DELAY)
		{
		outVoltContrHndlCnt++;
		if(outVoltContrHndlCnt==U_OUT_KONTR_DELAY)
			{
			avar_uout_hndl(1);
			}
		}
	}
else
	{
	if(outVoltContrHndlCnt)
		{
		if(outVoltContrHndlCnt_<5)
			{
			outVoltContrHndlCnt_++;
			if(outVoltContrHndlCnt_>=5)
				{
				outVoltContrHndlCnt=0;
				if(uout_av)avar_uout_hndl(0);
				}
			}
		}
	}

if (load_U<(USIGN*10)) 
	{
	if(!bSILENT)
		{
		mess_send(210,114,1,20);
		}

	
	}


}


void vent_resurs_hndl(void)
{
char i;
char crc_in,crc_eval;

for(i=0;i<NUMIST;i++)
	{
	if((bps[i]._buff[7]&0xc0)==0x00)
		{
		bps[i]._vent_resurs_temp[0]=bps[i]._buff[7];
		}
	else if((bps[i]._buff[7]&0xc0)==0x40)
		{
		bps[i]._vent_resurs_temp[1]=bps[i]._buff[7];
		}
	else if((bps[i]._buff[7]&0xc0)==0x80)
		{
		bps[i]._vent_resurs_temp[2]=bps[i]._buff[7];
		}
	else if((bps[i]._buff[7]&0xc0)==0xc0)
		{
		bps[i]._vent_resurs_temp[3]=bps[i]._buff[7];
		}
	crc_in=0;
	crc_in|=(bps[i]._vent_resurs_temp[0]&0x30)>>4;
	crc_in|=(bps[i]._vent_resurs_temp[1]&0x30)>>2;
	crc_in|=(bps[i]._vent_resurs_temp[2]&0x30);
	crc_in|=(bps[i]._vent_resurs_temp[3]&0x30)<<2;

	crc_eval =bps[i]._vent_resurs_temp[0]&0x0f;
	crc_eval^=bps[i]._vent_resurs_temp[1]&0x0f;
	crc_eval^=bps[i]._vent_resurs_temp[2]&0x0f;
	crc_eval^=bps[i]._vent_resurs_temp[3]&0x0f;

	if(crc_eval==crc_in)
		{
		unsigned short temp_US;
		temp_US=0;

		temp_US|=(bps[i]._vent_resurs_temp[3]&0x0f);
		temp_US<<=4;
		temp_US|=(bps[i]._vent_resurs_temp[2]&0x0f);
		temp_US<<=4;
		temp_US|=(bps[i]._vent_resurs_temp[1]&0x0f);
		temp_US<<=4;
		temp_US|=(bps[i]._vent_resurs_temp[0]&0x0f);

		if(bps[i]._vent_resurs!=temp_US)bps[i]._vent_resurs=temp_US;
		}

	if((bps[i]._vent_resurs>TVENTMAX*10)&&(TVENTMAX>0))
		{
		bps[i]._av|=(1<<4);
		}
	else bps[i]._av&=~(1<<4);
	}
}



void vent_hndl(void)
{
if(RELEVENTSIGN==rvsAKB)
	{
	if(vent_stat==0)
		{
		if	(
			(BAT_IS_ON[0]==bisON)&&((bat[0]._Tb>TVENTON)||(bat[0]._nd))
			||
			(BAT_IS_ON[1]==bisON)&&((bat[1]._Tb>TVENTON)||(bat[1]._nd))
			)
			{
			vent_stat=1;
			}
		}
	else if(vent_stat==1)
		{
		if	(
			((BAT_IS_ON[0]!=bisON)||((BAT_IS_ON[0]==bisON)&&(bat[0]._Tb<TVENTOFF)&&(!bat[0]._nd)))
			&&
			((BAT_IS_ON[1]!=bisON)||((BAT_IS_ON[1]==bisON)&&(bat[1]._Tb<TVENTOFF)&&(!bat[1]._nd)))
			)
			{
			vent_stat=0;
			}
		}
	}
else if(RELEVENTSIGN==rvsBPS)
	{













 

	if	(
		((NUMIST)&&((bps[0]._Ti>TVENTON)||(bps[0]._cnt>=30)))
		||
		((NUMIST>1)&&((bps[1]._Ti>TVENTON)||(bps[1]._cnt>=30)))
		||
		((NUMIST>2)&&((bps[2]._Ti>TVENTON)||(bps[2]._cnt>=30)))
		||
		((NUMIST>3)&&((bps[3]._Ti>TVENTON)||(bps[3]._cnt>=30)))
		||
		((NUMIST>4)&&((bps[4]._Ti>TVENTON)||(bps[4]._cnt>=30)))
		||
		((NUMIST>5)&&((bps[5]._Ti>TVENTON)||(bps[5]._cnt>=30)))
		||
		((NUMIST>6)&&((bps[6]._Ti>TVENTON)||(bps[6]._cnt>=30)))
		||
		((NUMIST>7)&&((bps[7]._Ti>TVENTON)||(bps[7]._cnt>=30)))
		)
		{
		vent_stat=1;
		}
	else if(vent_stat==1)
		{
		if	(
			((!NUMIST)||((NUMIST)&&(bps[0]._Ti<TVENTOFF)&&(bps[0]._cnt<10)))
			&&
			((NUMIST<2)||((NUMIST>=2)&&(bps[1]._Ti<TVENTOFF)&&(bps[1]._cnt<10)))
			&&
			((NUMIST<3)||((NUMIST>=3)&&(bps[2]._Ti<TVENTOFF)&&(bps[2]._cnt<10)))
			&&
			((NUMIST<4)||((NUMIST>=4)&&(bps[3]._Ti<TVENTOFF)&&(bps[3]._cnt<10)))
			&&
			((NUMIST<5)||((NUMIST>=5)&&(bps[4]._Ti<TVENTOFF)&&(bps[4]._cnt<10)))
			&&
			((NUMIST<6)||((NUMIST>=6)&&(bps[5]._Ti<TVENTOFF)&&(bps[5]._cnt<10)))
			&&
			((NUMIST<7)||((NUMIST>=7)&&(bps[6]._Ti<TVENTOFF)&&(bps[6]._cnt<10)))
			&&
			((NUMIST<8)||((NUMIST>=8)&&(bps[7]._Ti<TVENTOFF)&&(bps[7]._cnt<10)))
			)
			{
			vent_stat=0;
			}
		}
	}
else if(RELEVENTSIGN==rvsEXT)
	{
	if	(
		((NUMDT)&&((t_ext[0]>TVENTON)||(ND_EXT[0])))
		||
		((NUMDT>1)&&((t_ext[1]>TVENTON)||(ND_EXT[1])))
		||
		((NUMDT>2)&&((t_ext[2]>TVENTON)||(ND_EXT[2])))
		)
		{
		vent_stat=1;
		}
	else if(vent_stat==1)
		{
		if	(
			((!NUMDT)||((NUMDT)&&(t_ext[0]<TVENTOFF)&&(!ND_EXT[0])))
			&&
			((NUMDT<2)||((NUMDT>=2)&&(t_ext[1]<TVENTOFF)&&(!ND_EXT[1])))
			&&
			((NUMDT<3)||((NUMDT>=3)&&(t_ext[2]<TVENTOFF)&&(!ND_EXT[2])))
			)
			{
			vent_stat=0;
			}
		}
	}
else vent_stat=1;

}


#line 11841 "control.c"

