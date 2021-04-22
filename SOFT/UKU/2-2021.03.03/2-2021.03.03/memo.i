#line 1 "memo.c"
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
#line 2 "memo.c"
#line 1 "memo.h"

void memo_read (void);






#line 3 "memo.c"
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

#line 4 "memo.c"
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














 
#line 5 "memo.c"
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





 
#line 6 "memo.c"
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


#line 7 "memo.c"
#line 1 "gran.h"

void gran_ring_char(signed char *adr, signed char min, signed char max) ;
void gran_char(signed char *adr, signed char min, signed char max);
void gran(signed short *adr, signed short min, signed short max);
void gran_ring(signed short *adr, signed short min, signed short max);
void gran_long(signed long *adr, signed long min, signed long max); 
#line 8 "memo.c"


void memo_read (void)
{
char i;










Kubat[0]=lc640_read_int(0x10);
Kubat[1]=lc640_read_int(0x10+2);
Kubatm[0]=lc640_read_int(0x10+28);
Kubatm[1]=lc640_read_int(0x10+30);
Kibat0[0]=lc640_read_int(0x10+4);
Kibat0[1]=lc640_read_int(0x10+6);
Kibat1[0]=lc640_read_int(0x10+8);
Kibat1[1]=lc640_read_int(0x10+10);
Ktbat[0]=lc640_read_int(0x10+12);
Ktbat[1]=lc640_read_int(0x10+14);
Kunet=lc640_read_int(0x10+16);
Kuload=lc640_read_int(0x10+20);
Kunet_ext[0]=lc640_read_int(0x10+22);
Kunet_ext[1]=lc640_read_int(0x10+24);
Kunet_ext[2]=lc640_read_int(0x10+26);
Kvv_eb2[0]=lc640_read_int(0x10+32);
Kvv_eb2[1]=lc640_read_int(0x10+34);
Kvv_eb2[2]=lc640_read_int(0x10+36);
Kpes_eb2[0]=lc640_read_int(0x10+38);
Kpes_eb2[1]=lc640_read_int(0x10+40);
Kpes_eb2[2]=lc640_read_int(0x10+42);
KunetA=lc640_read_int(0x10+44);
KunetB=lc640_read_int(0x10+46);
KunetC=lc640_read_int(0x10+48);
Kubps=lc640_read_int(0x10+50);
Kuout=lc640_read_int(0x10+52);

Ktext[0]=lc640_read_int(0x10+100+50);
Ktext[1]=lc640_read_int(0x10+100+52);
Ktext[2]=lc640_read_int(0x10+100+54);
	
MAIN_IST=lc640_read_int(0x10+100+2);
UMAX=lc640_read_int(0x10+100+4);
UB0=lc640_read_int(0x10+100+6);
UB20=lc640_read_int(0x10+100+8);
TMAX=lc640_read_int(0x10+100+10);
TSIGN=lc640_read_int(0x10+100+82);
DU=lc640_read_int(0x10+100+84);
USIGN=lc640_read_int(0x10+100+14);
UMN=lc640_read_int(0x10+100+16);
UMAXN=lc640_read_int(4470);
ZV_ON=lc640_read_int(0x10+100+18);
IKB=lc640_read_int(0x10+100+20);
UVZ=lc640_read_int(0x10+100+106);
IMAX_VZ=lc640_read_int(0x10+350+54);
SMART_SPC=lc640_read_int(0x10+350+56);
IMAX=lc640_read_int(0x10+100+24);
IMIN=lc640_read_int(0x10+100+26);
APV_ON=lc640_read_int(0x10+100+28);
RS485_QWARZ_DIGIT=lc640_read_int(0x10+100+230);
SNTP_ENABLE=lc640_read_int(0x10+350+26);
SNTP_GMT=lc640_read_int(0x10+350+28);
UZ_U=lc640_read_int(0x10+350+30);
UZ_IMAX=lc640_read_int(0x10+350+32);
UZ_T=lc640_read_int(0x10+350+34);
FZ_U1=lc640_read_int(0x10+350+40);	
FZ_IMAX1=lc640_read_int(0x10+350+42);
FZ_T1=lc640_read_int(0x10+350+44);					
FZ_ISW12=lc640_read_int(0x10+350+46);				
FZ_U2=lc640_read_int(0x10+350+48);					
FZ_IMAX2=lc640_read_int(0x10+350+50);		
FZ_T2=lc640_read_int(0x10+350+52);				
UVENTOFF=lc640_read_int(0x10+100+240);
IZMAX=lc640_read_int(0x10+100+30);
U0B=lc640_read_int(0x10+100+32);
TZAS=lc640_read_int(0x10+100+34);
NUMIST=lc640_read_int(0x10+100+36);

NUMBAT=lc640_read_int(0x10+350+72);

NUMBAT_TELECORE=lc640_read_int(0x10+100+194);

NUMINV=lc640_read_int(0x10+100+38);
gran(&NUMINV,0,15);
NUMSK=lc640_read_int(0x10+500+88);
NUMDT=lc640_read_int(0x10+500+90);
NUMAVT=lc640_read_int(0x10+500+98);
NUMMAKB=lc640_read_int(0x10+500+100);
NUMEXT=NUMSK+NUMDT;
NUMBDR=lc640_read_int(0x10+500+104);
NUMENMV=lc640_read_int(4352);

if(NUMENMV>8)NUMENMV=0;
NUMLVBD=lc640_read_int(4356); 
if(NUMLVBD>1) NUMLVBD=0;
for(i=0;i<8;i++) enmv_modbus_adress[i]=lc640_read_int(4500+(2*i) );

NUMPHASE=lc640_read_int(4354);
AV_OFF_AVT=lc640_read_int(0x10+100+12);
MNEMO_ON=(enum_mnemo_on)lc640_read_int(0x10+100+72);
MNEMO_TIME=lc640_read_int(0x10+100+74);
U_AVT=lc640_read_int(0x10+100+80);
PAR=lc640_read_int(0x10+100+86);
TBATMAX=lc640_read_int(0x10+100+88);
TBATSIGN=lc640_read_int(0x10+100+90);
UBM_AV=lc640_read_int(0x10+500+96);
RELE_LOG=lc640_read_int(0x10+500+102);
TBOXMAX=lc640_read_int(0x10+100+92);
TBOXREG=lc640_read_int(0x10+100+94);
TBOXVENTMAX=lc640_read_int(0x10+100+96);
TLOADDISABLE=lc640_read_int(0x10+100+98);
TLOADENABLE=lc640_read_int(0x10+100+100);
TBATDISABLE=lc640_read_int(0x10+100+102);
TBATENABLE=lc640_read_int(0x10+100+104);
TVENTON=lc640_read_int(0x10+100+110);
TVENTOFF=lc640_read_int(0x10+100+112);
TWARMON=lc640_read_int(0x10+100+138);
TWARMOFF=lc640_read_int(0x10+100+140);
RELEVENTSIGN=(enum_releventsign)lc640_read_int(0x10+100+114);
NPN_OUT=(enum_npn_out)lc640_read_int(0x10+100+116);
NPN_SIGN=(enum_npn_sign)lc640_read_int(0x10+100+118);
UONPN=lc640_read_int(0x10+100+120);
UVNPN=lc640_read_int(0x10+100+122);
TZNPN=lc640_read_int(0x10+100+124);
dUNPN=lc640_read_int(0x10+100+248);
TERMOKOMPENS=lc640_read_int(0x10+100+126);
TBOXVENTON=lc640_read_int(0x10+100+128);
TBOXVENTOFF=lc640_read_int(0x10+100+130);
TBOXWARMON=lc640_read_int(0x10+100+132);
TBOXWARMOFF=lc640_read_int(0x10+100+134);
NUMBYPASS=lc640_read_int(0x10+100+136);
BAT_TYPE=lc640_read_int(0x10+100+142); 
#line 175 "memo.c"

speedChrgCurr =lc640_read_int(0x10+100+162); 
speedChrgVolt =lc640_read_int(0x10+100+164);
speedChrgTimeInHour =lc640_read_int(0x10+100+166);
speedChrgAvtEn =lc640_read_int(0x10+100+168);
speedChrgBlckSrc=lc640_read_int(0x10+100+172);
speedChrgBlckLog=lc640_read_int(0x10+100+174);
speedChrgDU=lc640_read_int(0x10+100+170);
DU_LI_BAT=lc640_read_int(0x10+100+176);
FORVARDBPSCHHOUR=lc640_read_int(0x10+100+178);
U_OUT_KONTR_MAX=lc640_read_int(0x10+100+182);
U_OUT_KONTR_MIN=lc640_read_int(0x10+100+184);
U_OUT_KONTR_DELAY=lc640_read_int(0x10+100+186);
DOP_RELE_FUNC=lc640_read_int(0x10+100+188);
ipsBlckSrc=lc640_read_int(0x10+100+190);
ipsBlckLog=lc640_read_int(0x10+100+192);
CNTRL_HNDL_TIME=lc640_read_int(0x10+100+196);
USODERG_LI_BAT=lc640_read_int(0x10+100+198);
QSODERG_LI_BAT=lc640_read_int(0x10+350);
TVENTMAX=lc640_read_int(0x10+350+2);
ICA_EN=lc640_read_int(0x10+350+6);
ICA_CH=lc640_read_int(0x10+350+4);
ICA_MODBUS_ADDRESS=lc640_read_int(0x10+350+8);
ICA_MODBUS_TCP_IP1=lc640_read_int(0x10+350+10);
ICA_MODBUS_TCP_IP2=lc640_read_int(0x10+350+12);
ICA_MODBUS_TCP_IP3=lc640_read_int(0x10+350+14);
ICA_MODBUS_TCP_IP4=lc640_read_int(0x10+350+16);
ICA_MODBUS_TCP_UNIT_ID=lc640_read_int(0x10+350+18);
PWM_START=lc640_read_int(0x10+350+20);
KB_ALGORITM=lc640_read_int(0x10+350+22);
REG_SPEED=lc640_read_int(0x10+350+24);
#line 235 "memo.c"
BAT_IS_ON[0]=(enum_bat_is_on)lc640_read_int(0x10+450);
BAT_IS_ON[1]=(enum_bat_is_on)lc640_read_int(0x10+450+30);	






BAT_DAY_OF_ON[0]=lc640_read_int(0x10+450+2);
BAT_MONTH_OF_ON[0]=lc640_read_int(0x10+450+4);
BAT_YEAR_OF_ON[0]=lc640_read_int(0x10+450+6);
BAT_C_REAL[0]=lc640_read_int(0x10+450+8);
BAT_C_NOM[0]=lc640_read_int(0x10+450+16);
BAT_RESURS[0]=lc640_read_int(0x10+450+10);


BAT_DAY_OF_ON[1]=lc640_read_int(0x10+450+32);
BAT_MONTH_OF_ON[1]=lc640_read_int(0x10+450+34);
BAT_YEAR_OF_ON[1]=lc640_read_int(0x10+450+36);
BAT_C_REAL[1]=lc640_read_int(0x10+450+38);
BAT_C_NOM[1]=lc640_read_int(0x10+450+48);
BAT_RESURS[1]=lc640_read_int(0x10+450+40);


BAT_LINK=lc640_read_int(0x10+500+200+76);

APV_ON1=(enum_apv_on)lc640_read_int(0x10+100+44);
APV_ON2=(enum_apv_on)lc640_read_int(0x10+100+46);
APV_ON2_TIME=lc640_read_int(0x10+100+48);
VZ_HR=lc640_read_int(0x10+100+76);
TBAT=lc640_read_int(0x10+100+78);

AUSW_MAIN=lc640_read_int(0x10+100+224);
AUSW_MAIN_NUMBER=lc640_read_long(0x10+100+226);











AUSW_UKU_NUMBER=lc640_read_long(0x10+100+228);			


TMAX_EXT_EN[0]=lc640_read_int(0x10+500);
TMAX_EXT[0]=lc640_read_int(0x10+500+2);
TMIN_EXT_EN[0]=lc640_read_int(0x10+500+4);
TMIN_EXT[0]=lc640_read_int(0x10+500+6);	
T_EXT_REL_EN[0]=lc640_read_int(0x10+500+8);
T_EXT_ZVUK_EN[0]=lc640_read_int(0x10+500+10);
T_EXT_LCD_EN[0]=lc640_read_int(0x10+500+12);
T_EXT_RS_EN[0]=lc640_read_int(0x10+500+14);
SK_SIGN[0]=lc640_read_int(0x10+500+48);
SK_REL_EN[0]=lc640_read_int(0x10+500+50);
SK_ZVUK_EN[0]=lc640_read_int(0x10+500+52);
SK_LCD_EN[0]=lc640_read_int(0x10+500+54);	
SK_RS_EN[0]=lc640_read_int(0x10+500+56);
SK_SIGN[1]=lc640_read_int(0x10+500+58);
SK_REL_EN[1]=lc640_read_int(0x10+500+60);
SK_ZVUK_EN[1]=lc640_read_int(0x10+500+62);
SK_LCD_EN[1]=lc640_read_int(0x10+500+64);	
SK_RS_EN[1]=lc640_read_int(0x10+500+66);
SK_SIGN[2]=lc640_read_int(0x10+500+68);
SK_REL_EN[2]=lc640_read_int(0x10+500+70);
SK_ZVUK_EN[2]=lc640_read_int(0x10+500+72);
SK_LCD_EN[2]=lc640_read_int(0x10+500+74);	
SK_RS_EN[2]=lc640_read_int(0x10+500+76);
SK_SIGN[3]=lc640_read_int(0x10+500+78);
SK_REL_EN[3]=lc640_read_int(0x10+500+80);
SK_ZVUK_EN[3]=lc640_read_int(0x10+500+82);
SK_LCD_EN[3]=lc640_read_int(0x10+500+84);	
SK_RS_EN[3]=lc640_read_int(0x10+500+86);
RELE_VENT_LOGIC=lc640_read_int(0x10+100+108);


AVZ=(enum_avz)(lc640_read_int(0x10+100+70));
AVZ_TIME=lc640_read_int(0x10+100+56);
HOUR_AVZ=lc640_read_int(0x10+100+58);
MIN_AVZ=lc640_read_int(0x10+100+60);
SEC_AVZ=lc640_read_int(0x10+100+62);
DATE_AVZ=lc640_read_int(0x10+100+64);
MONTH_AVZ=lc640_read_int(0x10+100+66);
if(!((MONTH_AVZ>0)&&(MONTH_AVZ<13)))MONTH_AVZ=0;
YEAR_AVZ=lc640_read_int(0x10+100+68);

pos_vent=lc640_read_int(0x10+500+92);
POWER_CNT_ADRESS=lc640_read_int(0x10+500+94);

ETH_IS_ON=lc640_read_int(0x10+500+200);
ETH_DHCP_ON=lc640_read_int(0x10+500+200+2);
ETH_IP_1=lc640_read_int(0x10+500+200+4);
ETH_IP_2=lc640_read_int(0x10+500+200+6);
ETH_IP_3=lc640_read_int(0x10+500+200+8);
ETH_IP_4=lc640_read_int(0x10+500+200+10);
ETH_MASK_1=lc640_read_int(0x10+500+200+12);
ETH_MASK_2=lc640_read_int(0x10+500+200+14);
ETH_MASK_3=lc640_read_int(0x10+500+200+16);
ETH_MASK_4=lc640_read_int(0x10+500+200+18);
ETH_GW_1=lc640_read_int(0x10+500+200+64);
ETH_GW_2=lc640_read_int(0x10+500+200+66);
ETH_GW_3=lc640_read_int(0x10+500+200+68);
ETH_GW_4=lc640_read_int(0x10+500+200+70);
ETH_TRAP1_IP_1=lc640_read_int(0x10+500+200+20);
ETH_TRAP1_IP_2=lc640_read_int(0x10+500+200+22);
ETH_TRAP1_IP_3=lc640_read_int(0x10+500+200+24);
ETH_TRAP1_IP_4=lc640_read_int(0x10+500+200+26);
ETH_TRAP2_IP_1=lc640_read_int(0x10+500+200+28);
ETH_TRAP2_IP_2=lc640_read_int(0x10+500+200+30);
ETH_TRAP2_IP_3=lc640_read_int(0x10+500+200+32);
ETH_TRAP2_IP_4=lc640_read_int(0x10+500+200+34);
ETH_TRAP3_IP_1=lc640_read_int(0x10+500+200+36);
ETH_TRAP3_IP_2=lc640_read_int(0x10+500+200+38);
ETH_TRAP3_IP_3=lc640_read_int(0x10+500+200+40);
ETH_TRAP3_IP_4=lc640_read_int(0x10+500+200+42);
ETH_TRAP4_IP_1=lc640_read_int(0x10+500+200+44);
ETH_TRAP4_IP_2=lc640_read_int(0x10+500+200+46);
ETH_TRAP4_IP_3=lc640_read_int(0x10+500+200+48);
ETH_TRAP4_IP_4=lc640_read_int(0x10+500+200+50);
ETH_TRAP5_IP_1=lc640_read_int(0x10+500+200+52);
ETH_TRAP5_IP_2=lc640_read_int(0x10+500+200+54);
ETH_TRAP5_IP_3=lc640_read_int(0x10+500+200+56);
ETH_TRAP5_IP_4=lc640_read_int(0x10+500+200+58);
ETH_SNMP_PORT_READ=lc640_read_int(0x10+500+200+60);
ETH_SNMP_PORT_WRITE=lc640_read_int(0x10+500+200+62);
MODBUS_ADRESS=lc640_read_int(0x10+500+200+72);
MODBUS_BAUDRATE=lc640_read_int(0x10+500+200+74);
RELE_SET_MASK[0]=lc640_read_int(0x10+100+232);
RELE_SET_MASK[1]=lc640_read_int(0x10+100+234);
RELE_SET_MASK[2]=lc640_read_int(0x10+100+236);
RELE_SET_MASK[3]=lc640_read_int(0x10+100+238);
num_rki=(char)lc640_read_int(0x10+54);

num_net_in=(char)lc640_read_int(0x10+56);
hv_vz_stat=(enum_hv_vz_stat)lc640_read_int(0x10+100+244);
vz1_stat=(enum_vz1_stat)lc640_read_int(0x10+350+36);
if(vz1_stat==255)vz1_stat=vz1sOFF;
vz2_stat=(enum_vz2_stat)lc640_read_int(0x10+350+38);
if(vz2_stat==255)vz2_stat=vz2sOFF;
VZ_KIND=lc640_read_int(0x10+100+242);

BAT_C_POINT_1_6=lc640_read_int(0x10+350+58);
BAT_C_POINT_1_2=lc640_read_int(0x10+350+60);
BAT_C_POINT_1=lc640_read_int(0x10+350+62);				
BAT_C_POINT_3=lc640_read_int(0x10+350+64);
BAT_C_POINT_5=lc640_read_int(0x10+350+66);
BAT_C_POINT_10=lc640_read_int(0x10+350+68);
BAT_C_POINT_20=lc640_read_int(0x10+350+70);
BAT_U_END_1_6=lc640_read_int(0x10+350+74);
BAT_U_END_1_2=lc640_read_int(0x10+350+76);
BAT_U_END_1=lc640_read_int(0x10+350+78);				
BAT_U_END_3=lc640_read_int(0x10+350+80);
BAT_U_END_5=lc640_read_int(0x10+350+82);
BAT_U_END_10=lc640_read_int(0x10+350+84);
BAT_U_END_20=lc640_read_int(0x10+350+86);
BAT_C_POINT_NUM_ELEM=lc640_read_int(0x10+350+92);
BAT_K_OLD=lc640_read_int(0x10+350+94);


#line 435 "memo.c"

SP_CH_VENT_BLOK=lc640_read_int(0x10+350+96);
VZ_CH_VENT_BLOK=lc640_read_int(0x10+350+98);

for(i=0;i<64;i++)
	{
	snmp_location[i]=lc640_read(0x10+500+200+200+i);
	}
for(i=0;i<10;i++)
	{
	snmp_community[i]=lc640_read(0x10+500+200+270+(i*2));
	}
for(i=0;i<64;i++)
	{
	place_holder[i]=lc640_read(4370+i);
	}
for(i=0;i<3;i++)
	{
	snmp_web_passw[i]=lc640_read(4360+(i*2));
	snmp_web_passw[3]=0;
	}

__ee_vz_cnt=lc640_read_int(0x10+500+150+2);
__ee_spc_stat=lc640_read_int(0x10+500+150);
__ee_spc_bat=lc640_read_int(0x10+500+150+4);
__ee_spc_phase=lc640_read_int(0x10+500+150+6);
}


