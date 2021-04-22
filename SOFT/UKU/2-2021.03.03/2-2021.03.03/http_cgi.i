#line 1 "HTTP_CGI.c"









 

#line 1 "C:\\Keil\\ARM\\RV31\\INC\\Net_Config.h"









 




#line 1 "C:\\Keil\\ARM\\RV31\\INC\\RTL.h"









 




 

 


#line 27 "C:\\Keil\\ARM\\RV31\\INC\\RTL.h"







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

#line 54 "C:\\Keil\\ARM\\RV31\\INC\\RTL.h"

#line 66 "C:\\Keil\\ARM\\RV31\\INC\\RTL.h"



 





 
typedef U32 OS_SEM[2];

 

typedef U32 OS_MBX[];

 
typedef U32 OS_MUT[3];

 
typedef U32 OS_TID;

 
typedef void *OS_ID;

 
typedef U32 OS_RESULT;

 












 




#line 194 "C:\\Keil\\ARM\\RV31\\INC\\RTL.h"



 



 
extern void      os_set_env    (void);
extern void      rt_sys_init   (void (*task)(void), U8 priority, void *stk);
extern void      rt_tsk_pass   (void);
extern OS_TID    rt_tsk_self   (void);
extern OS_RESULT rt_tsk_prio   (OS_TID task_id, U8 new_prio);
extern OS_TID    rt_tsk_create (void (*task)(void), U8 priority, void *stk, void *argv);
extern OS_RESULT rt_tsk_delete (OS_TID task_id);

#line 230 "C:\\Keil\\ARM\\RV31\\INC\\RTL.h"

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

 




 

 



 






 
#line 415 "C:\\Keil\\ARM\\RV31\\INC\\RTL.h"

 
#line 428 "C:\\Keil\\ARM\\RV31\\INC\\RTL.h"

 





 
#line 442 "C:\\Keil\\ARM\\RV31\\INC\\RTL.h"

 




 



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






 
  

 
#line 16 "C:\\Keil\\ARM\\RV31\\INC\\Net_Config.h"

 



                                   





 




 




 
#line 50 "C:\\Keil\\ARM\\RV31\\INC\\Net_Config.h"

 





 
#line 73 "C:\\Keil\\ARM\\RV31\\INC\\Net_Config.h"

 





typedef struct os_frame {          
  U16 length;                      
  U16 index;                       
  U8  data[1];                     
} OS_FRAME;


typedef struct arp_info {          
  U8  State;                       
  U8  Type;                        
  U8  Retries;                     
  U8  Tout;                        
  U8  HwAdr[6];           
  U8  IpAdr[4];            
} ARP_INFO;


typedef struct igmp_info {         
  U8  State;                       
  U8  Tout;                        
  U8  GrpIpAdr[4];         
} IGMP_INFO;


typedef struct udp_info {          
  U8  State;                       
  U8  McastTtl;                    
  U16 LocPort;                     
  U8  Tos;                         
  U8  Opt;                         
                                   
  U16 (*cb_func)(U8 socket, U8 *rem_ip, U16 port, U8 *buf, U16 len);
} UDP_INFO;


typedef struct tcp_info {          
  U8  State;                       
  U8  Type;                        
  U8  Flags;                       
  U8  Tos;                         
  U8  RemIpAdr[4];         
  U16 RemPort;                     
  U16 LocPort;                     
  U16 MaxSegSize;                  
  U16 WinSize;                     
  U32 SendSeq;                     
  U32 SendSeqNext;                 
  U32 RecSeqNext;                  
  U16 Tout;                        
  U16 AliveTimer;                  
  U16 RetryTimer;                  
  U8  TxFlags;                     
  U8  Retries;                     
  OS_FRAME *ReTransFrm;            
                                   
  U16 (*cb_func)(U8 socket, U8 event, U8 *p1, U16 p2);
} TCP_INFO;


typedef struct http_info {         
  U8  State;                       
  U8  Socket;                      
  U16 Flags;                       
  U8  FType;                       
  U8  PostSt;                      
  U16 DelimSz;                     
  U32 CGIvar;                      
  U32 DLen;                        
  U32 Count;                       
  U16 BCnt;                        
  U8  Lang[6];                     
  U32 LMDate;                      
  U8 *Script;                      
  U8 *pDelim;                      
  void *sFile;                     
  void *dFile;                     
} HTTP_INFO;


typedef struct http_file {         
  const U32 Id;                    
  const U8 *Start;                 
} HTTP_FILE;


typedef struct tnet_info {         
  U8  State;                       
  U8  Socket;                      
  U8  Flags;                       
  U8  BCnt;                        
  U16 Tout;                        
  U8  Widx;                        
  U8  Ridx;                        
  U32 SVar;                        
  U8  LBuf[96];           
  U8  Fifo[128];           
  U8  hNext;                       
  U8  hCurr;                       
  U8  Hist[128];           
} TNET_INFO;


typedef struct tftp_info {         
  U8  State;                       
  U8  Retries;                     
  U8  Flags;                       
  U16 Timer;                       
  U8  RemIpAdr[4];         
  U16 RemPort;                     
  U16 BlockNr;                     
  void *File;                      
  U32 FPos;                        
} TFTP_INFO;


typedef struct ftp_info {          
  U8  State;                       
  U8  Socket;                      
  U8  Flags;                       
  U8  Resp;                        
  U8  RemIpAdr[4];         
  U16 DPort;                       
  U8  DSocket;                     
  U8  PathLen;                     
  U8 *Path;                        
  U8 *Name;                        
  void *File;                      
} FTP_INFO;


typedef struct dns_cache {         
  U32 HostId;                      
  U32 Ttl;                         
  U8  IpAdr[4];            
} DNS_CACHE;


typedef struct localm {            
  U8 IpAdr[4];             
  U8 DefGW[4];             
  U8 NetMask[4];           
  U8 PriDNS[4];            
  U8 SecDNS[4];            
} LOCALM;


typedef struct remotem {           
  U8 IpAdr[4];             
  U8 HwAdr[6];            
} REMOTEM;


typedef struct mib_entry {         
  U8   Type;                       
  U8   OidLen;                     
  U8   Oid[13];             
  U8   ValSz;                      
  void *Val;                       
  void (*cb_func)(int mode);       
} MIB_ENTRY;


typedef enum {                     
  ERR_MEM_ALLOC,
  ERR_MEM_FREE,
  ERR_MEM_CORRUPT,
  ERR_UDP_ALLOC,
  ERR_TCP_ALLOC,
  ERR_TCP_STATE
} ERROR_CODE;




 

 
extern void init_system (void);
extern void run_system (void);
extern void process_hl_igmp (OS_FRAME *frame);
extern void process_hl_udp (OS_FRAME *frame);
extern void process_hl_tcp (OS_FRAME *frame);
extern BOOL dispatch_frame (OS_FRAME *frame, U8 netif);
extern BOOL eth_chk_adr (OS_FRAME *frame);
extern U8  *eth_get_adr (U8 *ipadr);
__weak void arp_notify (void);
extern void sys_error (ERROR_CODE code);

 
extern OS_FRAME *alloc_mem (U32 byte_size);
extern void free_mem (OS_FRAME *mem_ptr);

 
extern void init_eth_link (void);
extern void run_eth_link (void);
extern void put_in_queue (OS_FRAME *frame);
extern BOOL eth_send_frame (OS_FRAME *frame);

 
extern void init_ppp_link (void);
extern void run_ppp_link (void);
extern BOOL ppp_send_frame (OS_FRAME *frame, U16 prot);

 
extern void init_slip_link (void);
extern void run_slip_link (void);
extern BOOL slip_send_frame (OS_FRAME *frame);

 
extern int  mem_copy (void *dp, void *sp, int len);
extern void mem_rcopy (void *dp, void *sp, int len);
extern BOOL mem_comp (void *sp1, void *sp2, int len);
extern void mem_set (void *dp, U8 val, int len);
extern BOOL mem_test (void *sp, U8 val, int len);
extern BOOL str_scomp (U8 *sp, U8 const *cp);
extern int  str_copy (U8 *dp, U8 *sp);
extern void str_up_case (U8 *dp, U8 *sp);
extern U16  SwapB (U16 w16);
extern U16  get_u16 (U8 *p16);
extern U32  get_u32 (U8 *p32);
extern void set_u32 (U8 *p32, U32 val);

 
extern void arp_send_req (U32 entry);

 
extern void init_igmp (void);
extern void run_igmp_host (void);
extern void process_igmp (OS_FRAME *frame);

 
extern void init_udp (void);
extern void process_udp (OS_FRAME *frame);

 
extern void init_tcp (void);
extern void tcp_poll_sockets (void);
extern void process_tcp (OS_FRAME *frame_r);

 
extern void init_http (void);
extern void run_http_server (void);
extern void *http_fopen (U8 *name);
extern void http_fclose (void *file);
extern U16  http_fread (void *file, U8 *buf, U16 len);
extern BOOL http_fgets (void *file, U8 *buf, U16 size);
extern U32  http_finfo (U8 *name);
extern void cgi_process_var (U8 *qstr);
extern void cgi_process_data (U8 code, U8 *dat, U16 len);
extern U16  cgi_func (U8 *env, U8 *buf, U16 buflen, U32 *pcgi);
extern U8  *cgx_content_type (void);
extern BOOL http_accept_host (U8 *rem_ip, U16 rem_port);
extern U8  *http_get_var (U8 *env, void *ansi, U16 maxlen);
extern U8  *http_get_lang (void);
extern void http_get_info (REMOTEM *info);
extern U8   http_get_session (void);
extern U8  *http_get_content_type (void);
extern U32  http_date (RL_TIME *time);

 
extern void init_tnet (void);
extern void run_tnet_server (void);
extern U16  tnet_cbfunc (U8 code, U8 *buf, U16 buflen);
extern U16  tnet_process_cmd (U8 *cmd, U8 *buf, U16 buflen, U32 *pvar);
extern BOOL tnet_ccmp (U8 *buf, U8 *cmd);
extern void tnet_set_delay (U16 cnt);
extern void tnet_get_info (REMOTEM *info);
extern U8   tnet_get_session (void);
extern BOOL tnet_msg_poll (U8 session);

 
extern void init_tftp (void);
extern void run_tftp_server (void);
extern void *tftp_fopen (U8 *fname, U8 *mode);
extern void tftp_fclose (void *file);
extern U16  tftp_fread (void *file, U32 fpos, U8 *buf);
extern U16  tftp_fwrite (void *file, U8 *buf, U16 len);

 
extern void init_ftp (void);
extern void run_ftp_server (void);
extern void *ftp_fopen (U8 *fname, U8 *mode);
extern void ftp_fclose (void *file);
extern U16  ftp_fread (void *file, U8 *buf, U16 len);
extern U16  ftp_fwrite (void *file, U8 *buf, U16 len);
extern BOOL ftp_fdelete (U8 *fname);
extern BOOL ftp_frename (U8 *fname, U8 *newn);
extern U16  ftp_ffind (U8 code, U8 *buf, U8 *mask, U16 len);

 
extern void init_dhcp (void);
extern void run_dhcp_client (void);

 
extern void init_nbns (void);

 
extern void init_dns (void);
extern void run_dns_client (void);
extern U8   get_host_by_name (U8 *hostn, void (*cbfunc)(U8, U8 *));

 
extern void init_smtp (void);
extern void run_smtp_client (void);
extern U16  smtp_cbfunc (U8 code, U8 *buf, U16 buflen, U32 *pvar);
extern BOOL smtp_accept_auth (U8 *srv_ip);

 
extern void init_ethernet (void);
extern void send_frame (OS_FRAME *frame);
extern void poll_ethernet (void);
extern void int_enable_eth (void);
extern void int_disable_eth (void);

 
extern void init_serial (void);
extern int  com_getchar (void);
extern BOOL com_putchar (U8 c);
extern BOOL com_tx_active (void);

 
extern void init_modem (void);
extern void modem_dial (U8 *dialnum);
extern void modem_hangup (void);
extern void modem_listen (void);
extern BOOL modem_online (void);
extern BOOL modem_process (U8 ch);
extern void modem_run (void);







 



#line 13 "HTTP_CGI.c"
#line 1 "C:\\Keil\\ARM\\RV31\\INC\\stdio.h"
 
 
 





 






 









#line 34 "C:\\Keil\\ARM\\RV31\\INC\\stdio.h"


  
  typedef unsigned int size_t;    








 
 

 
  typedef struct __va_list __va_list;





   




 




typedef struct __fpos_t_struct {
    unsigned __int64 __pos;
    



 
    struct {
        unsigned int __state1, __state2;
    } __mbstate;
} fpos_t;
   


 


   

 

typedef struct __FILE FILE;
   






 

extern FILE __stdin, __stdout, __stderr;
extern FILE *__aeabi_stdin, *__aeabi_stdout, *__aeabi_stderr;

#line 125 "C:\\Keil\\ARM\\RV31\\INC\\stdio.h"
    

    

    





     



   


 


   


 

   



 

   


 




   


 





    


 






extern __declspec(__nothrow) int remove(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int rename(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) FILE *tmpfile(void);
   




 
extern __declspec(__nothrow) char *tmpnam(char *  );
   











 

extern __declspec(__nothrow) int fclose(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) int fflush(FILE *  );
   







 
extern __declspec(__nothrow) FILE *fopen(const char * __restrict  ,
                           const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   








































 
extern __declspec(__nothrow) FILE *freopen(const char * __restrict  ,
                    const char * __restrict  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(2,3)));
   








 
extern __declspec(__nothrow) void setbuf(FILE * __restrict  ,
                    char * __restrict  ) __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) int setvbuf(FILE * __restrict  ,
                   char * __restrict  ,
                   int  , size_t  ) __attribute__((__nonnull__(1)));
   















 
#pragma __printf_args
extern __declspec(__nothrow) int fprintf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   


















 
#pragma __printf_args
extern __declspec(__nothrow) int _fprintf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   




 
#pragma __printf_args
extern __declspec(__nothrow) int _printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






 
#pragma __printf_args
extern __declspec(__nothrow) int _sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   















 

#pragma __printf_args
extern __declspec(__nothrow) int _snprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int fscanf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






























 
#pragma __scanf_args
extern __declspec(__nothrow) int _fscanf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   






 
#pragma __scanf_args
extern __declspec(__nothrow) int _scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int sscanf(const char * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   








 
#pragma __scanf_args
extern __declspec(__nothrow) int _sscanf(const char * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

 
extern __declspec(__nothrow) int vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int _vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int _vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int _vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int _vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int vfprintf(FILE * __restrict  ,
                    const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int vsprintf(char * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 

extern __declspec(__nothrow) int vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   







 

extern __declspec(__nothrow) int _vsprintf(char * __restrict  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vfprintf(FILE * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vsnprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   



 
extern __declspec(__nothrow) int fgetc(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) char *fgets(char * __restrict  , int  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   










 
extern __declspec(__nothrow) int fputc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   







 
extern __declspec(__nothrow) int fputs(const char * __restrict  , FILE * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int getc(FILE *  ) __attribute__((__nonnull__(1)));
   







 




    extern __declspec(__nothrow) int (getchar)(void);

   





 
extern __declspec(__nothrow) char *gets(char *  ) __attribute__((__nonnull__(1)));
   









 
extern __declspec(__nothrow) int putc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   





 




    extern __declspec(__nothrow) int (putchar)(int  );

   



 
extern __declspec(__nothrow) int puts(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int ungetc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   






















 

extern __declspec(__nothrow) size_t fread(void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   











 

extern __declspec(__nothrow) size_t __fread_bytes_avail(void * __restrict  ,
                    size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   











 

extern __declspec(__nothrow) size_t fwrite(const void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   







 

extern __declspec(__nothrow) int fgetpos(FILE * __restrict  , fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) int fseek(FILE *  , long int  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) int fsetpos(FILE * __restrict  , const fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   










 
extern __declspec(__nothrow) long int ftell(FILE *  ) __attribute__((__nonnull__(1)));
   











 
extern __declspec(__nothrow) void rewind(FILE *  ) __attribute__((__nonnull__(1)));
   





 

extern __declspec(__nothrow) void clearerr(FILE *  ) __attribute__((__nonnull__(1)));
   




 

extern __declspec(__nothrow) int feof(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) int ferror(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) void perror(const char *  );
   









 

extern __declspec(__nothrow) int _fisatty(FILE *   ) __attribute__((__nonnull__(1)));
    
 

extern __declspec(__nothrow) void __use_no_semihosting_swi(void);
extern __declspec(__nothrow) void __use_no_semihosting(void);
    





 











#line 944 "C:\\Keil\\ARM\\RV31\\INC\\stdio.h"



 
#line 14 "HTTP_CGI.c"
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



 
#line 15 "HTTP_CGI.c"
#line 1 "main.h"


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














 
#line 16 "HTTP_CGI.c"
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


#line 17 "HTTP_CGI.c"
#line 1 "sntp.h"










extern U8 socket_udp;
extern U16 udp_callback_cnt,udp_callback_cnt1;
extern U8 Rem_IP[4];
extern const int NTP_PACKET_SIZE;
extern U16 udp_callback_plazma[12];
extern U16 day_of_year;
extern U32 full_days_since_2000_01_01;
extern U32 curr_days_since_2000_01_01;
extern U32 sec_after_2000_01_01;
extern U16 day_after_this_month;
extern const U16 days_of_month[];
extern U16	this_year;
extern long tempL;
extern U32 day_after_this_year;
extern U16 this_month;
extern U16 day_of_month;
extern U32 sec_in_this_day;
extern U16 hour_in_this_day;
extern U16 min_in_this_hour;
extern U16 sec_in_this_min;
extern U16 time_sinc_hndl_req_cnt;
extern U32 time_sinc_hndl_main_cnt;

extern char sntp_plazma;

U16 udp_callback (U8 soc, U8 *rip, U16 rport, U8 *buf, U16 len);
void sntp_requ (void);
void time_sinc_hndl(void);


#line 18 "HTTP_CGI.c"
#line 1 "http_data.h"

extern char http_power_num_of_phases;
extern short http_power_voltage_of_phase[3];
extern short http_power_frequncy;
extern char http_power_status;
extern char http_output_buff[70];
extern char log_item_cnt;
extern char http_string_of_model[50];
extern char* http_model;


void http_data(void);

short http_get_log_deep(void);

char* http_get_log_rec(char num);

char* http_tm_dt_output(char numOfDt);

char* http_tm_sk_output(char numOfSk);

char* http_tm_src_output(char numOfSrc);

char* http_ip_output(char ip1, char ip2, char ip3, char ip4);

char* http_tm_bat_output(char numOfSrc);

char* pal_cyr_decoder(char* input);

#line 19 "HTTP_CGI.c"
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

#line 20 "HTTP_CGI.c"
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





 
#line 21 "HTTP_CGI.c"
#line 1 "C:\\Keil\\ARM\\INC\\NXP\\LPC17xx\\LPC17xx.H"




















 









 

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

 









 
#line 97 "C:\\Keil\\ARM\\INC\\NXP\\LPC17xx\\LPC17xx.H"
#line 1 "C:\\Keil\\ARM\\INC\\NXP\\LPC17xx\\system_LPC17xx.h"




















 









extern uint32_t SystemFrequency;     










 
extern void SystemInit (void);





#line 98 "C:\\Keil\\ARM\\INC\\NXP\\LPC17xx\\LPC17xx.H"


 
 
 


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



 
 
 
 
#line 924 "C:\\Keil\\ARM\\INC\\NXP\\LPC17xx\\LPC17xx.H"

 
#line 945 "C:\\Keil\\ARM\\INC\\NXP\\LPC17xx\\LPC17xx.H"

 
#line 959 "C:\\Keil\\ARM\\INC\\NXP\\LPC17xx\\LPC17xx.H"

 
#line 972 "C:\\Keil\\ARM\\INC\\NXP\\LPC17xx\\LPC17xx.H"

 







 
 
 
#line 1031 "C:\\Keil\\ARM\\INC\\NXP\\LPC17xx\\LPC17xx.H"

#line 22 "HTTP_CGI.c"
















 

 
extern U16 AD_in (U32 ch);
extern U8  get_button (void);
char psw_err;

 
extern  LOCALM localm[];


 
extern struct tcp_info tcp_socket[];
extern U8 const tcp_NumSocks;
extern U8 const http_EnAuth;
extern U8       http_auth_passw[20];

extern BOOL LEDrun;
extern void LED_out (U32 val);
extern BOOL LCDupdate;
extern U8   lcd_text[2][16+1];

 
static U8 P2;
static char const state[][11] = {
  "FREE",
  "CLOSED",
  "LISTEN",
  "SYN_REC",
  "SYN_SENT",
  "FINW1",
  "FINW2",
  "CLOSING",
  "LAST_ACK",
  "TWAIT",
  "CONNECT"};

 
 
 
typedef struct {
  U16 xcnt;
  U16 unused;
} MY_BUF;














 






 

 

void cgi_process_var (U8 *qs) {
   
   
   
  U8 *var;
  int s[4];

	web_plazma[1]++;

  var = (U8 *)alloc_mem (40);
  do {
     
    qs = http_get_var (qs, var, 40);
     
    if (var[0] != 0) {
       
      if (str_scomp (var, "ip=") == 1) {
         
        sscanf ((const char *)&var[3], "%d.%d.%d.%d",&s[0],&s[1],&s[2],&s[3]);
        localm[0].IpAdr[0]   = s[0];
        localm[0].IpAdr[1]   = s[1];
        localm[0].IpAdr[2]   = s[2];
        localm[0].IpAdr[3]   = s[3];
      }
      else if (str_scomp (var, "msk=") == 1) {
         
        sscanf ((const char *)&var[4], "%d.%d.%d.%d",&s[0],&s[1],&s[2],&s[3]);
        localm[0].NetMask[0] = s[0];
        localm[0].NetMask[1] = s[1];
        localm[0].NetMask[2] = s[2];
        localm[0].NetMask[3] = s[3];
      }
      else if (str_scomp (var, "gw=") == 1) {
         
        sscanf ((const char *)&var[3], "%d.%d.%d.%d",&s[0],&s[1],&s[2],&s[3]);
        localm[0].DefGW[0]   = s[0];
        localm[0].DefGW[1]   = s[1];
        localm[0].DefGW[2]   = s[2];
        localm[0].DefGW[3]   = s[3];
      }
      else if (str_scomp (var, "pdns=") == 1) {
         
        sscanf ((const char *)&var[5], "%d.%d.%d.%d",&s[0],&s[1],&s[2],&s[3]);
        localm[0].PriDNS[0]  = s[0];
        localm[0].PriDNS[1]  = s[1];
        localm[0].PriDNS[2]  = s[2];
        localm[0].PriDNS[3]  = s[3];
      }
      else if (str_scomp (var, "sdns=") == 1) {
         
        sscanf ((const char *)&var[5], "%d.%d.%d.%d",&s[0],&s[1],&s[2],&s[3]);
        localm[0].SecDNS[0]  = s[0];
        localm[0].SecDNS[1]  = s[1];
        localm[0].SecDNS[2]  = s[2];
        localm[0].SecDNS[3]  = s[3];
      }
    }
  }while (qs);
  free_mem ((OS_FRAME *)var);
}




 

void cgi_process_data (U8 code, U8 *dat, U16 len) {
   
   
   
   
   
   
   
   
   
   
     
   
   
  U8 passw[12],retyped[12];
  U8 *var,stpassw; 
U8 *varr[3];
U8 i;

web_plazma[2]++;
web_plazma[3]+=len;



  switch (code) {
    case 0:
       
      break;

    default:
       
      return;
  }

  P2 = 0;
  
  if (len == 0) {
     
    
    return;
  }
  
  
stpassw = 0;
var = (U8 *)alloc_mem (40);
varr[0] = (U8 *)alloc_mem (40);
varr[1] = (U8 *)alloc_mem (150);
varr[2] = (U8 *)alloc_mem (40);

i=0;

do 
	{
     
	if(i==1)dat = http_get_var (dat, varr[i++], 150);
    else dat = http_get_var (dat, varr[i++], 40);
	web_plazma[0]++;
	}
while (dat);


    if (varr[0,0] != 0) 
		{
		 
		if (str_scomp (varr[0], "parol") == 1)
			{
			char str_buff[4];
        	web_plazma[1]++;
			
			if ((str_scomp (varr[0]+6, snmp_web_passw ) == 1)&&(len == 9)) 
				{
				uku_set_autorized=1;
				psw_err=0;
				}
			else
				{
				psw_err=1;
				uku_set_autorized=0;
				}
			}
		else if (str_scomp (varr[0], "param=") == 1) 
			{
			if(strstr (varr[1], "value="))
				{
				
				web_plazma[4]=22;
				if(strstr (varr[0], "serno"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					lc640_write_int(0x10+100+226,(short)(web_param_input&0x0000ffffUL));
					lc640_write_int(0x10+100+226+2,(short)((web_param_input&0xffff0000UL)>>16UL));
					}
				else if(strstr (varr[0], "place"))
					{
					char i = 0;
					str_copy(place_holder,"                                                                      ");
					str_copy(place_holder,pal_cyr_decoder(varr[1]+6));
					
					



 
					for (i=0;i<70;i++)lc640_write(4370+i,place_holder[i]);
					}

				else if(strstr (varr[0], "par_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					lc640_write_int(0x10+100+86,(short)(web_param_input&0x00000001UL));

					}
				else if(strstr (varr[0], "zv_on_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					lc640_write_int(0x10+100+18,(short)(web_param_input&0x00000001UL));

					}
				else if(strstr (varr[0], "av_avt_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					lc640_write_int(0x10+100+12,(short)(web_param_input&0x00000001UL));

					}				

				else if(strstr (varr[0], "un_max_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					lc640_write_int(4470,(short)(web_param_input&0x0000ffffUL));

					}		
					
				else if(strstr (varr[0], "un_min_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					lc640_write_int(0x10+100+16,(short)(web_param_input&0x0000ffffUL));

					}	
				else if(strstr (varr[0], "rot_t_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					lc640_write_int(0x10+100+178,(short)(web_param_input&0x0000ffffUL));

					}	
				else if(strstr (varr[0], "i_max_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					lc640_write_int(0x10+100+24,(short)(web_param_input&0x0000ffffUL));

					}
				else if(strstr (varr[0], "i_min_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					lc640_write_int(0x10+100+26,(short)(web_param_input&0x0000ffffUL));

					}
				else if(strstr (varr[0], "u_max_i_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					lc640_write_int(0x10+100+4,(short)(web_param_input&0x0000ffffUL));

					}
				else if(strstr (varr[0], "u_min_i_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					DU=UB20-((short)(web_param_input&0x0000ffffUL));
					gran(&DU,50,UB20-100);
					lc640_write_int(0x10+100+84,DU);
					
					}
				else if(strstr (varr[0], "ub20_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					lc640_write_int(0x10+100+8,(short)(web_param_input&0x0000ffffUL));
 					}
						
				else if(strstr (varr[0], "ub0_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					lc640_write_int(0x10+100+6,(short)(web_param_input&0x0000ffffUL));
					}

				else if(strstr (varr[0], "usign_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					lc640_write_int(0x10+100+14,(short)(web_param_input&0x0000ffffUL));
					}
																																																
				else if(strstr (varr[0], "izmax_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					lc640_write_int(0x10+100+30,(short)(web_param_input&0x0000ffffUL));
					}

				else if(strstr (varr[0], "tbatsign_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					lc640_write_int(0x10+100+90,(short)(web_param_input&0x0000ffffUL));
					}
					
				else if(strstr (varr[0], "tbatmax_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					lc640_write_int(0x10+100+88,(short)(web_param_input&0x0000ffffUL));
					}
					
				else if(strstr (varr[0], "uvz_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					lc640_write_int(0x10+100+106,(short)(web_param_input&0x0000ffffUL));
					}
					
				else if(strstr (varr[0], "tvz_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					lc640_write_int(0x10+100+76,(short)(web_param_input&0x0000ffffUL));
					}	
				else if(strstr (varr[0], "tsign_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					lc640_write_int(0x10+100+82,(short)(web_param_input&0x0000ffffUL));
					}
					
				else if(strstr (varr[0], "tmax_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					lc640_write_int(0x10+100+10,(short)(web_param_input&0x0000ffffUL));
					}

				else if(strstr (varr[0], "tzas_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					lc640_write_int(0x10+100+34,(short)(web_param_input&0x0000ffffUL));
					}

				else if(strstr (varr[0], "apv1_on_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					if((web_param_input&0x0000ffffUL)==1)lc640_write_int(0x10+100+44,apvON);
					else lc640_write_int(0x10+100+44,apvOFF);
					}

				else if(strstr (varr[0], "apv2_on_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					if((web_param_input&0x0000ffffUL)==1)lc640_write_int(0x10+100+46,apvON);
					else lc640_write_int(0x10+100+46,apvOFF);
					}

				else if(strstr (varr[0], "apv2_time_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					lc640_write_int(0x10+100+48,(short)(web_param_input&0x0000ffffUL));
					}

				else if(strstr (varr[0], "u0b_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					lc640_write_int(0x10+100+32,(short)(web_param_input&0x0000ffffUL));
					}
					
				else if(strstr (varr[0], "tbat_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					lc640_write_int(0x10+100+78,(short)(web_param_input&0x0000ffffUL));
					}
					
				else if(strstr (varr[0], "ikb_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					lc640_write_int(0x10+100+20,(short)(web_param_input&0x0000ffffUL));
					}

				else if(strstr (varr[0], "year_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->YEAR=(short)(web_param_input&0x0000ffffUL);
					}

				else if(strstr (varr[0], "month_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MONTH=(short)(web_param_input&0x0000ffffUL);
					}

				else if(strstr (varr[0], "day_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->DOM=(short)(web_param_input&0x0000ffffUL);
					}

				else if(strstr (varr[0], "hour_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->HOUR=(short)(web_param_input&0x0000ffffUL);
					}
					
				else if(strstr (varr[0], "minut_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MIN=(short)(web_param_input&0x0000ffffUL);
					}
					
				else if(strstr (varr[0], "secund_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->SEC=(short)(web_param_input&0x0000ffffUL);
					}

				else if(strstr (varr[0], "sk1avsign_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					if((short)(web_param_input&0x000000ffUL))lc640_write_int(0x10+500+48,0);
					else lc640_write_int(0x10+500+48,0);
					}
				else if(strstr (varr[0], "sk2avsign_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					if((short)(web_param_input&0x000000ffUL))lc640_write_int(0x10+500+58,0);
					else lc640_write_int(0x10+500+58,0);
					}
				else if(strstr (varr[0], "sk3avsign_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					if((short)(web_param_input&0x000000ffUL))lc640_write_int(0x10+500+68,0);
					else lc640_write_int(0x10+500+68,0);
					}
				else if(strstr (varr[0], "sk4avsign_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					if((short)(web_param_input&0x000000ffUL))lc640_write_int(0x10+500+78,0);
					else lc640_write_int(0x10+500+78,0);
					}
				else if(strstr (varr[0], "sk1lcden_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					if((short)(web_param_input&0x000000ffUL))lc640_write_int(0x10+500+54,0);
					else lc640_write_int(0x10+500+54,1);
					}
				else if(strstr (varr[0], "sk2lcden_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					if((short)(web_param_input&0x000000ffUL))lc640_write_int(0x10+500+64,0);
					else lc640_write_int(0x10+500+64,1);
					}
				else if(strstr (varr[0], "sk3lcden_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					if((short)(web_param_input&0x000000ffUL))lc640_write_int(0x10+500+74,0);
					else lc640_write_int(0x10+500+74,1);

					}
				else if(strstr (varr[0], "sk4lcden_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					if((short)(web_param_input&0x000000ffUL))lc640_write_int(0x10+500+84,0);
					else lc640_write_int(0x10+500+84,1);

					}
				else if(strstr (varr[0], "sk1zven_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					if((short)(web_param_input&0x000000ffUL))lc640_write_int(0x10+500+52,0);
					else lc640_write_int(0x10+500+52,1);
					}
				else if(strstr (varr[0], "sk2zven_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					if((short)(web_param_input&0x000000ffUL))lc640_write_int(0x10+500+62,0);
					else lc640_write_int(0x10+500+62,1);
					}
				else if(strstr (varr[0], "sk3zven_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					if((short)(web_param_input&0x000000ffUL))lc640_write_int(0x10+500+72,0);
					else lc640_write_int(0x10+500+72,1);
					}
				else if(strstr (varr[0], "sk4zven_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					if((short)(web_param_input&0x000000ffUL))lc640_write_int(0x10+500+82,0);
					else lc640_write_int(0x10+500+82,1);
					}	
				else if(strstr (varr[0], "sntp_gmt_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					lc640_write_int(0x10+350+28,(short)web_param_input);
					}
				else if(strstr (varr[0], "sntp_exe_"))
					{
					time_sinc_hndl_req_cnt=5;
					Rem_IP[0]=88;
					Rem_IP[1]=147;
					Rem_IP[2]=254;
					Rem_IP[3]=230;
					sntp_requ();
					
					}
				else if(strstr (varr[0], "sntp_src_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					if((short)(web_param_input&0x000000ffUL))lc640_write_int(4472,0);
					else lc640_write_int(4472,1);
					}
				else if(strstr (varr[0], "sntp_ip_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					lc640_write_int(4480,(char)(web_param_input&0x000000ffUL));
					lc640_write_int(4478,(char)((web_param_input&0x0000ff00UL)>>8));
					lc640_write_int(4476,(char)((web_param_input&0x00ff0000UL)>>16));
					lc640_write_int(4474,(char)((web_param_input&0xff000000UL)>>24));
					}	
				else if(strstr (varr[0], "numist_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					lc640_write_int(0x10+100+36,(short)(web_param_input&0x000000ffUL));
					}
				else if(strstr (varr[0], "numphase_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					if((short)(web_param_input&0x000000ffUL))lc640_write_int(4354,3);
					else lc640_write_int(4354,1);
					}
				else if(strstr (varr[0], "numsk_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					lc640_write_int(0x10+500+88,(short)web_param_input);
					}
				else if(strstr (varr[0], "numdt_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					lc640_write_int(0x10+500+90,(short)web_param_input);
					}
				else if(strstr (varr[0], "nummakb_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					lc640_write_int(0x10+500+100,(short)web_param_input);
					}
				else if(strstr (varr[0], "numenmv_"))
					{
					sscanf ((const char *)varr[1]+6, "%d",&web_param_input);
					lc640_write_int(4352,(short)web_param_input);
					}																																																																																																																																																																																																	
				}

			
			
			}
      else if (str_scomp (var, "but=but2") == 1) {
        
      }
      else if (str_scomp (var, "but=but3") == 1) {
        
      }
      else if (str_scomp (var, "pw=") == 1) {
         
        str_copy (passw, var+3);
        stpassw |= 1;
      }
      else if (str_scomp (var, "pw2=") == 1) {
         
        str_copy (retyped, var+4);
        stpassw |= 2;
      }
      else if (str_scomp (var, "lcd1=") == 1) {
         
        
        
      }
      else if (str_scomp (var, "lcd2=") == 1) {
         
        
        
			}
		}

free_mem ((OS_FRAME *)var);
free_mem ((OS_FRAME *)varr[0]);
free_mem ((OS_FRAME *)varr[1]);
free_mem ((OS_FRAME *)varr[2]);
  

  if (stpassw == 0x03) {
    len = strlen ((const char *)passw);
    if (mem_comp (passw, retyped, len) == 1) {
       
      str_copy (http_auth_passw, passw);
    }
  }
}



 

U16 cgi_func (U8 *env, U8 *buf, U16 buflen, U32 *pcgi) {
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
  TCP_INFO *tsoc;
  U32 len = 0;
  U8 id, *lang;
  static U32 adv;
  char temp;
  switch (env[0]) {
     
     

	case 'a':
		
		switch (env[1]) {
			case '0':
				switch (env[2]) {
					case '1':
						len = sprintf((char *)buf,(const char *)&env[4],"normal");
						break;
			        case '2':
			          	len = sprintf((char *)buf,(const char *)&env[4],pal_cyr_coder("������ ��� �1"));
						break;
				}
		  		break;
		}
		break;

    case 'b':
		
		switch (env[1]) {
			case '1':
				len = sprintf((char *)buf,(const char *)&env[3],http_tm_bat_output(0));
		  		break;
			case '2':
				len = sprintf((char *)buf,(const char *)&env[3],http_tm_bat_output(1));
		  		break;
		}
		break;

    case 'c':
       
      while ((len + 150) < buflen) {
        tsoc = &tcp_socket[((MY_BUF *)pcgi)->xcnt];
        ((MY_BUF *)pcgi)->xcnt++;
         
        len += sprintf((char *)(buf+len),"<tr align=\"center\">");
        if (tsoc->State <= 1) {
          len += sprintf ((char *)(buf+len),
                          "<td>%d</td><td>%s</td><td>-</td><td>-</td>"
                          "<td>-</td><td>-</td></tr>\r\n",
                          ((MY_BUF *)pcgi)->xcnt,state[tsoc->State]);
        }
        else if (tsoc->State == 2) {
          len += sprintf ((char *)(buf+len),
                          "<td>%d</td><td>%s</td><td>-</td><td>-</td>"
                          "<td>%d</td><td>-</td></tr>\r\n",
                          ((MY_BUF *)pcgi)->xcnt,state[tsoc->State],tsoc->LocPort);
        }
        else {
          len += sprintf ((char *)(buf+len),
                          "<td>%d</td><td>%s</td><td>%d.%d.%d.%d</td>"
                          "<td>%d</td><td>%d</td><td>%d</td></tr>\r\n",
                          ((MY_BUF *)pcgi)->xcnt,state[tsoc->State],
                          tsoc->RemIpAdr[0],tsoc->RemIpAdr[1],
                          tsoc->RemIpAdr[2],tsoc->RemIpAdr[3],
                          tsoc->RemPort,tsoc->LocPort,tsoc->AliveTimer);
        }
         
        if (((MY_BUF *)pcgi)->xcnt == tcp_NumSocks) {
          break;
        }
      }
      if (((MY_BUF *)pcgi)->xcnt < tcp_NumSocks) {
         
        len |= 0x8000;
      }
      break;

	case 'd':
		
		
		switch (env[1]) {
			case '0':
				
				switch (env[2]) {
					case '1':
						
						if(AUSW_MAIN==24120)
							{
							if(NUMIST==2)			len = sprintf((char *)buf,(const char *)&env[4],pal_cyr_coder("����220/24-120�-2/4"));
							else if(NUMIST==3)		len = sprintf((char *)buf,(const char *)&env[4],pal_cyr_coder("����220/24-120�-3/4"));
							else					len = sprintf((char *)buf,(const char *)&env[4],pal_cyr_coder("����220/24-120�-4/4"));
							}
						else if(AUSW_MAIN==24210)
							{
							if(NUMIST==3)			len = sprintf((char *)buf,(const char *)&env[4],pal_cyr_coder("����220/24-210�-3/7"));
							else if(NUMIST==4)		len = sprintf((char *)buf,(const char *)&env[4],pal_cyr_coder("����220/24-210�-4/7"));
							else if(NUMIST==5)		len = sprintf((char *)buf,(const char *)&env[4],pal_cyr_coder("����220/24-210�-5/7"));
							else if(NUMIST==6)		len = sprintf((char *)buf,(const char *)&env[4],pal_cyr_coder("����220/24-210�-6/7"));
							else					len = sprintf((char *)buf,(const char *)&env[4],pal_cyr_coder("����220/24-210�-7/7"));
							}
						else if(AUSW_MAIN==4880)
							{
							if(NUMIST==3)			len = sprintf((char *)buf,(const char *)&env[4],pal_cyr_coder("����220/48-80�-3/4 "));
							else					len = sprintf((char *)buf,(const char *)&env[4],pal_cyr_coder("����220/48-80�-4/4 "));
							}

						else if(AUSW_MAIN==4883)	len = sprintf((char *)buf,(const char *)&env[4],pal_cyr_coder("����380/48-80�-4/4 "));
						else if(AUSW_MAIN==48123)	len = sprintf((char *)buf,(const char *)&env[4],pal_cyr_coder("����380/48-120�-4/4"));
						else if(AUSW_MAIN==48123)	len = sprintf((char *)buf,(const char *)&env[4],pal_cyr_coder("����380/48-120�-4/4"));
						else 						len = sprintf((char *)buf,(const char *)&env[4],pal_cyr_coder("����220/48-120�-4/4"));
					break;
			        case '2':
			          	len = sprintf((char *)buf,(const char *)&env[4],AUSW_MAIN_NUMBER);
					break;
			        case '3':
			          	len = sprintf((char *)buf,(const char *)&env[4],pal_cyr_coder(place_holder));
					break;
			        case '4':  	
			          	len = sprintf((char *)buf,(const char *)&env[4],NUMBAT);
					break;
			        case '5':	
						len = sprintf((char *)buf,(const char *)&env[4],NUMIST);
					break;
			        case '6':	
			          	len = sprintf((char *)buf,(const char *)&env[4],NUMINV);
					break;
			        case '7': 	
			          	len = sprintf((char *)buf,(const char *)&env[4],NUMBYPASS);
					break;
			        case '8': 	
			          	len = sprintf((char *)buf,(const char *)&env[4],NUMSK);
					break;
			        case '9': 	
			          	len = sprintf((char *)buf,(const char *)&env[4],NUMDT);
					break;
				}
		  	break;
		}
	break;

    case 'e':
       
      lang = http_get_lang();
      if (strcmp ((const char *)lang, "en") == 0) {
        lang = "English";
      }
      else if (strcmp ((const char *)lang, "en-us") == 0) {
        lang = "English USA";
      }
      else if (strcmp ((const char *)lang, "en-gb") == 0) {
        lang = "English GB";
      }
      else if (strcmp ((const char *)lang, "de") == 0) {
        lang = "German";
      }
      else if (strcmp ((const char *)lang, "de-ch") == 0) {
        lang = "German CH";
      }
      else if (strcmp ((const char *)lang, "de-at") == 0) {
        lang = "German AT";
      }
      else if (strcmp ((const char *)lang, "fr") == 0) {
        lang = "French";
      }
      else if (strcmp ((const char *)lang, "sl") == 0) {
        lang = "Slovene";
      }
      else {
        lang = "Unknown";
      }
      len = sprintf((char *)buf,(const char *)&env[2],lang,http_get_lang());
      break;

    case 'f':
       
      switch (env[2]) {
        case '1':
          
          break;
        case '2':
          
          break;
      }
      break;

	case 'g':
       
		switch (env[2]) {
        case '1':
          
          len = sprintf((char *)buf,(const char *)&env[4],adv);
          break;
        case '2':
          len = sprintf((char *)buf,(const char *)&env[4],(float)adv*3.3/1024);
          break;
        case '3':
          adv = (adv * 100) / 1024;
          len = sprintf((char *)buf,(const char *)&env[4],adv);
          break;
      }
      break;

    case 'k':
		
      	switch (env[1]) {
        	case '1':
          		len = sprintf((char *)buf,(const char *)&env[3],http_tm_sk_output(0));
          		break;
     		case '2':
          		len = sprintf((char *)buf,(const char *)&env[3],http_tm_sk_output(1));
          		break;
         	case '3':
          		len = sprintf((char *)buf,(const char *)&env[3],http_tm_sk_output(2));
          		break;
     		case '4':
          		len = sprintf((char *)buf,(const char *)&env[3],http_tm_sk_output(3));
          		break;
		}
		break;

    case 'm':
      	switch (env[1]) {
        	case '1':
          		len = sprintf((char *)buf,(const char *)&env[3],web_cnt_2hz);
          		break;
     		case '2':
          		len = sprintf((char *)buf,(const char *)&env[3],web_cnt_main);
          		break;
     		case '3':
				if(psw_err)	len = sprintf((char *)buf,(const char *)&env[3],"error");
				else 	   	len = sprintf((char *)buf,(const char *)&env[3],"good");
          		break;
		}
		break;

    case 'p':
		
      	switch (env[1]) {
        	case 'n':
          		len = sprintf((char *)buf,(const char *)&env[3],http_power_num_of_phases);
          		break;
     		case 'A':
          		len = sprintf((char *)buf,(const char *)&env[3],http_power_voltage_of_phase[0]);
          		break;
     		case 'B':
          		len = sprintf((char *)buf,(const char *)&env[3],http_power_voltage_of_phase[1]);
          		break;       		
     		case 'C':
          		len = sprintf((char *)buf,(const char *)&env[3],http_power_voltage_of_phase[2]);
          		break;
			case 'F':
          		len = sprintf((char *)buf,(const char *)&env[3],http_power_frequncy);
          		break;
     		case 'S':
				len = sprintf((char *)buf,(const char *)&env[3],http_power_status);
          		break;		}
		break;

    case 't':
		
		switch (env[1]) {
        	case '1':
          		len = sprintf((char *)buf,(const char *)&env[3],http_tm_dt_output(0));
          		break;
     		case '2':
          		len = sprintf((char *)buf,(const char *)&env[3],http_tm_dt_output(1));
          		break;
         	case '3':
          		len = sprintf((char *)buf,(const char *)&env[3],http_tm_dt_output(2));
          		break;
 		}
		break;

	case 'l':
		
		switch (env[1]) {
			case 'd':
          		len = sprintf((char *)buf,(const char *)&env[3],http_get_log_deep());
          		break;
			case 'n':
          		len = sprintf((char *)buf,(const char *)&env[3],log_item_cnt);
				uku_set_autorized=0;
				break;
			case '0':
				if(NUMMAKB==0)	len = sprintf((char *)buf,(const char *)&env[3],http_get_log_rec(log_item_cnt));
				else
					{
					len = sprintf((char *)buf,(const char *)&env[3],http_get_log_rec(NUMINV));
					}
				
				break;
			case 'e':
				len = sprintf((char *)buf,(const char *)&env[3],pal_cyr_coder("end"));
				if(++log_item_cnt>=http_get_log_deep())log_item_cnt=0;
				break;
		}
		
		break;

    case 'o':
		
		switch (env[1]) {
			case 'u':
				len = sprintf((char *)buf,(const char *)&env[3],load_U);
		  		break;
			case 'i':
				len = sprintf((char *)buf,(const char *)&env[3],load_I);
		  		break;
		}
		break;

    case 's':
		 
      	switch (env[1]) {
        	case '0':
          		switch (env[2]) {
		        	case '0':
		          		len = sprintf((char *)buf,(const char *)&env[4],http_tm_src_output(0));
		          		break;
		     		case '1':
		          		len = sprintf((char *)buf,(const char *)&env[4],http_tm_src_output(1));
		          		break;
		     		case '2':
		          		len = sprintf((char *)buf,(const char *)&env[4],http_tm_src_output(2));
		          		break;
		     		case '3':
		          		len = sprintf((char *)buf,(const char *)&env[4],http_tm_src_output(3));
		          		break;
		     		case '4':
		          		len = sprintf((char *)buf,(const char *)&env[4],http_tm_src_output(4));
		          		break;
		        	case '5':
		          		len = sprintf((char *)buf,(const char *)&env[4],http_tm_src_output(5));
		          		break;
		     		case '6':
		          		len = sprintf((char *)buf,(const char *)&env[4],http_tm_src_output(6));
		          		break;
		     		case '7':
		          		len = sprintf((char *)buf,(const char *)&env[4],http_tm_src_output(7));
		          		break;
		     		case '8':
		          		len = sprintf((char *)buf,(const char *)&env[4],http_tm_src_output(8));
		          		break;
		     		case '9':
		          		len = sprintf((char *)buf,(const char *)&env[4],http_tm_src_output(9));
		          		break;
				}
				break;
        	case '1':
          		switch (env[2]) {
		        	case '0':
		          		len = sprintf((char *)buf,(const char *)&env[4],http_tm_src_output(10));
		          		break;
		     		case '1':
		          		len = sprintf((char *)buf,(const char *)&env[4],http_tm_src_output(11));
		          		break;
		     		case '2':
		          		len = sprintf((char *)buf,(const char *)&env[4],http_tm_src_output(12));
		          		break;
		     		case '3':
		          		len = sprintf((char *)buf,(const char *)&env[4],http_tm_src_output(13));
		          		break;
		     		case '4':
		          		len = sprintf((char *)buf,(const char *)&env[4],http_tm_src_output(14));
		          		break;
		        	case '5':
		          		len = sprintf((char *)buf,(const char *)&env[4],http_tm_src_output(15));
		          		break;
		     		case '6':
		          		len = sprintf((char *)buf,(const char *)&env[4],http_tm_src_output(16));
		          		break;
		     		case '7':
		          		len = sprintf((char *)buf,(const char *)&env[4],http_tm_src_output(17));
		          		break;
		     		case '8':
		          		len = sprintf((char *)buf,(const char *)&env[4],http_tm_src_output(18));
		          		break;
		     		case '9':
		          		len = sprintf((char *)buf,(const char *)&env[4],http_tm_src_output(19));
		          		break;
				}
				break;
		}
		break;

    case 'w':
		 


      	switch (env[1]) {
		    case 'n':
          		len = sprintf((char *)buf,(const char *)&env[3],64);
          		break;
        	case '0':
          		switch (env[2]) {
		        	case '0':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[0]);
		          		break;
		     		case '1':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[1]);
		          		break;
		     		case '2':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[2]);
		          		break;
		     		case '3':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[3]);
		          		break;
		     		case '4':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[4]);
		          		break;
		        	case '5':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[5]);
		          		break;
		     		case '6':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[6]);
		          		break;
		     		case '7':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[7]);
		          		break;
		     		case '8':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[8]);
		          		break;
		     		case '9':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[9]);
		          		break;
				}
				break;
        	case '1':
          		switch (env[2]) {
		        	case '0':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[10]);
		          		break;
		     		case '1':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[11]);
		          		break;
		     		case '2':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[12]);
		          		break;
		     		case '3':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[13]);
		          		break;
		     		case '4':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[14]);
		          		break;
		        	case '5':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[15]);
		          		break;
		     		case '6':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[16]);
		          		break;
		     		case '7':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[17]);
		          		break;
		     		case '8':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[18]);
		          		break;
		     		case '9':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[19]);
		          		break;
				}
				break;
        	case '2':
          		switch (env[2]) {
		        	case '0':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[20]);
		          		break;
		     		case '1':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[21]);
		          		break;
		     		case '2':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[22]);
		          		break;
		     		case '3':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[23]);
		          		break;
		     		case '4':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[24]);
		          		break;
		        	case '5':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[25]);
		          		break;
		     		case '6':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[26]);
		          		break;
		     		case '7':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[27]);
		          		break;
		     		case '8':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[28]);
		          		break;
		     		case '9':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[29]);
		          		break;
				}
				break;
        	case '3':
          		switch (env[2]) {
		        	case '0':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[30]);
		          		break;
		     		case '1':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[31]);
		          		break;
		     		case '2':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[32]);
		          		break;
		     		case '3':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[33]);
		          		break;
		     		case '4':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[34]);
		          		break;
		        	case '5':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[35]);
		          		break;
		     		case '6':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[36]);
		          		break;
		     		case '7':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[37]);
		          		break;
		     		case '8':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[38]);
		          		break;
		     		case '9':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[39]);
		          		break;
				}
				break;
        	case '4':
          		switch (env[2]) {
		        	case '0':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[40]);
		          		break;
		     		case '1':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[41]);
		          		break;
		     		case '2':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[42]);
		          		break;
		     		case '3':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[43]);
		          		break;
		     		case '4':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[44]);
		          		break;
		        	case '5':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[45]);
		          		break;
		     		case '6':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[46]);
		          		break;
		     		case '7':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[47]);
		          		break;
		     		case '8':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[48]);
		          		break;
		     		case '9':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[49]);
		          		break;
				}
				break;
        	case '5':
          		switch (env[2]) {
		        	case '0':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[50]);
		          		break;
		     		case '1':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[51]);
		          		break;
		     		case '2':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[52]);
		          		break;
		     		case '3':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[53]);
		          		break;
		     		case '4':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[54]);
		          		break;
		        	case '5':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[55]);
		          		break;
		     		case '6':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[56]);
		          		break;
		     		case '7':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[57]);
		          		break;
		     		case '8':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[58]);
		          		break;
		     		case '9':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[59]);
		          		break;
				}
				break;
        	case '6':
          		switch (env[2]) {
		        	case '0':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[60]);
		          		break;
		     		case '1':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[61]);
		          		break;
		     		case '2':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[62]);
		          		break;
		     		case '3':
		          		len = sprintf((char *)buf,(const char *)&env[4],snmp_enmv_data[63]);
		          		break;
				}
				break;

		}
		break;

    case 'y':
		 
      	switch (env[1]) {
        	case 'n':
          		len = sprintf((char *)buf,(const char *)&env[3],66);
          		break;
        	case '0':
          		switch (env[2]) {
		        	case '1':
		          		len = sprintf((char *)buf,(const char *)&env[4],AUSW_MAIN_NUMBER," ");
		          		break;
		     		case '2':
		          		len = sprintf((char *)buf,(const char *)&env[4],0,pal_cyr_coder(place_holder));
		          		break;
		     		case '3':
		          		len = sprintf((char *)buf,(const char *)&env[4],(ZV_ON==1)?1:0," ");
		          		break;
		     		case '4':
		          		len = sprintf((char *)buf,(const char *)&env[4],(PAR==1)?1:0," ");
		          		break;
		     		case '5':
		          		len = sprintf((char *)buf,(const char *)&env[4],(AV_OFF_AVT==1)?1:0," ");
		          		break;
		     		case '6':
		         		len = sprintf((char *)buf,(const char *)&env[4],UMN," ");
		          		break;
		     		case '7':
		         		len = sprintf((char *)buf,(const char *)&env[4],UMAXN," ");
		          		break;
		     		case '8':
		         		len = sprintf((char *)buf,(const char *)&env[4],FORVARDBPSCHHOUR," ");
		          		break;		     		
					case '9':
		         		len = sprintf((char *)buf,(const char *)&env[4],0,pal_cyr_coder("����������� ����������� 123456789012345678901234"));
		          		break;																				   
				}
				break;
        	case '1':
          		switch (env[2]) {
					case '0':
		         		len = sprintf((char *)buf,(const char *)&env[4],IMAX," ");
		          		break;																				   
					case '1':
		          		len = sprintf((char *)buf,(const char *)&env[4],IMIN," ");
		          		break;
		     		case '2':
		          		len = sprintf((char *)buf,(const char *)&env[4],UMAX," ");
		          		break;
		     		case '3':
		          		len = sprintf((char *)buf,(const char *)&env[4],UB20-DU," ");
		          		break;
		     		case '4':
		          		len = sprintf((char *)buf,(const char *)&env[4],UB20," ");
		          		break;
		     		case '5':
		          		len = sprintf((char *)buf,(const char *)&env[4],UB0," ");
		          		break;
		     		case '6':
		         		len = sprintf((char *)buf,(const char *)&env[4],USIGN," ");
		          		break;
		     		case '7':
		         		len = sprintf((char *)buf,(const char *)&env[4],IZMAX," ");
		          		break;
		     		case '8':
		         		len = sprintf((char *)buf,(const char *)&env[4],TBATSIGN," ");
		          		break;		     		
					case '9':
		         		len = sprintf((char *)buf,(const char *)&env[4],TBATMAX," ");
		          		break;																				   
				}
				break;
        	case '2':
          		switch (env[2]) {
					case '0':
		         		len = sprintf((char *)buf,(const char *)&env[4],UVZ," ");
		          		break;																				   
					case '1':
		          		len = sprintf((char *)buf,(const char *)&env[4],VZ_HR," ");
		          		break;
		     		case '2':
		          		len = sprintf((char *)buf,(const char *)&env[4],TSIGN," ");
		          		break;
		     		case '3':
		          		len = sprintf((char *)buf,(const char *)&env[4],TMAX," ");
		          		break;
		     		case '4':
		          		len = sprintf((char *)buf,(const char *)&env[4],TZAS," ");
		          		break;
		     		case '5':
		          		len = sprintf((char *)buf,(const char *)&env[4],(APV_ON1==apvON)?1:0," ");
		          		break;
		     		case '6':
		         		len = sprintf((char *)buf,(const char *)&env[4],(APV_ON2==apvON)?1:0," ");
		          		break;
		     		case '7':
		         		len = sprintf((char *)buf,(const char *)&env[4],APV_ON2_TIME," ");
		          		break;
		     		case '8':
		         		len = sprintf((char *)buf,(const char *)&env[4],U0B," ");
		          		break;		     		
					case '9':
		         		len = sprintf((char *)buf,(const char *)&env[4],TBAT," ");
		          		break;																				   
				}
				break;
       		case '3':
          		switch (env[2]) {
					case '0':
		         		len = sprintf((char *)buf,(const char *)&env[4],IKB," ");
		          		break;																				   
					case '1':
		          		len = sprintf((char *)buf,(const char *)&env[4],((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->YEAR," ");
		          		break;
		     		case '2':
		          		len = sprintf((char *)buf,(const char *)&env[4],((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MONTH," ");
		          		break;
		     		case '3':
		          		len = sprintf((char *)buf,(const char *)&env[4],((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->DOM," ");
		          		break;
		     		case '4':
		          		len = sprintf((char *)buf,(const char *)&env[4],((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->HOUR," ");
		          		break;
		     		case '5':
		          		len = sprintf((char *)buf,(const char *)&env[4],((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->MIN," ");
		          		break;
		     		case '6':
		         		len = sprintf((char *)buf,(const char *)&env[4],((LPC_RTC_TypeDef *) ((0x40000000UL) + 0x24000) )->SEC," ");
		          		break;
		     		case '7':
						if(AVZ==AVZ_1)temp=1;
						else if(AVZ==AVZ_2)temp=2;
						else if(AVZ==AVZ_3)temp=3; 
						else if(AVZ==AVZ_6)temp=4;
						else if(AVZ==AVZ_12)temp=5;			
						else temp=0;
						len = sprintf((char *)buf,(const char *)&env[4],temp," ");
		          		break;
		     		case '8':
		         		if(sk_stat[0]==ssON)temp=1;
						else temp=0;
						len = sprintf((char *)buf,(const char *)&env[4],temp," ");
		          		break;		     		
					case '9':
		         		if(SK_SIGN[0]==1)temp=0;
						else temp=1;
						len = sprintf((char *)buf,(const char *)&env[4],temp," ");
		          		break;																				   
				}
				break;
       		case '4':
          		switch (env[2]) {
					case '0':
		         		if(sk_av_stat[0]==sasON)temp=1;
						else temp=0;
						len = sprintf((char *)buf,(const char *)&env[4],temp," ");
		          		break;																				   
		     		case '1':
		         		if(sk_stat[1]==ssON)temp=1;
						else temp=0;
						len = sprintf((char *)buf,(const char *)&env[4],temp," ");
		          		break;		     		
					case '2':
		         		if(SK_SIGN[1]==1)temp=0;
						else temp=1;
						len = sprintf((char *)buf,(const char *)&env[4],temp," ");
		          		break;																				   
					case '3':
		         		if(sk_av_stat[1]==sasON)temp=1;
						else temp=0;
						len = sprintf((char *)buf,(const char *)&env[4],temp," ");
		          		break;																				   
		     		case '4':
		         		if(sk_stat[2]==ssON)temp=1;
						else temp=0;
						len = sprintf((char *)buf,(const char *)&env[4],temp," ");
		          		break;		     		
					case '5':
		         		if(SK_SIGN[2]==1)temp=0;
						else temp=1;
						len = sprintf((char *)buf,(const char *)&env[4],temp," ");
		          		break;																				   
					case '6':
		         		if(sk_av_stat[2]==sasON)temp=1;
						else temp=0;
						len = sprintf((char *)buf,(const char *)&env[4],temp," ");
		          		break;																				   
		     		case '7':
		         		if(sk_stat[3]==ssON)temp=1;
						else temp=0;
						len = sprintf((char *)buf,(const char *)&env[4],temp," ");
		          		break;		     		
					case '8':
		         		if(SK_SIGN[3]==1)temp=0;
						else temp=1;
						len = sprintf((char *)buf,(const char *)&env[4],temp," ");
		          		break;																				   
					case '9':
		         		if(sk_av_stat[3]==sasON)temp=1;
						else temp=0;
						len = sprintf((char *)buf,(const char *)&env[4],temp," ");
		          		break;																				   
				}
				break;
       		case '5':
          		switch (env[2]) {
					case '0':
		         		if(!SK_LCD_EN[0])temp=1;
						else temp=0;
						len = sprintf((char *)buf,(const char *)&env[4],temp," ");
						break;																				   
					case '1':
		         		if(!SK_LCD_EN[1])temp=1;
						else temp=0;
						len = sprintf((char *)buf,(const char *)&env[4],temp," ");
		          		break;
		     		case '2':
		         		if(!SK_LCD_EN[2])temp=1;
						else temp=0;
						len = sprintf((char *)buf,(const char *)&env[4],temp," ");
		          		break;
		     		case '3':
		         		if(!SK_LCD_EN[3])temp=1;
						else temp=0;
						len = sprintf((char *)buf,(const char *)&env[4],temp," ");
		          		break;
					case '4':
		         		if(!SK_ZVUK_EN[0])temp=1;
						else temp=0;
						len = sprintf((char *)buf,(const char *)&env[4],temp," ");
						break;																				   
					case '5':
		         		if(!SK_ZVUK_EN[1])temp=1;
						else temp=0;
						len = sprintf((char *)buf,(const char *)&env[4],temp," ");
		          		break;
		     		case '6':
		         		if(!SK_ZVUK_EN[2])temp=1;
						else temp=0;
						len = sprintf((char *)buf,(const char *)&env[4],temp," ");
		          		break;
		     		case '7':
		         		if(!SK_ZVUK_EN[3])temp=1;
						else temp=0;
						len = sprintf((char *)buf,(const char *)&env[4],temp," ");
		          		break;
		     		case '8':
						len = sprintf((char *)buf,(const char *)&env[4],(signed)SNTP_GMT," ");
		          		break;		     		
					case '9':
		         		if(lc640_read_int(4472)==1)temp=0;
						else temp=1;
						len = sprintf((char *)buf,(const char *)&env[4],temp," ");
		          		break;																				   
				}
				break;

       		case '6':
          		switch (env[2]) {
					case '0':
		         		len = sprintf((char *)buf,(const char *)&env[4],http_ip_output(	(char)lc640_read_int(4474),
																						(char)lc640_read_int(4476),
																						(char)lc640_read_int(4478),
																						(char)lc640_read_int(4480)));
						break;																				   
					case '1':
						len = sprintf((char *)buf,(const char *)&env[4],NUMIST," ");
		          		break;
		     		case '2':
						if(NUMPHASE==1)temp=0;
						else temp=1;
						len = sprintf((char *)buf,(const char *)&env[4],temp," ");
		          		break;
		     		case '3':
						len = sprintf((char *)buf,(const char *)&env[4],NUMSK," ");
		          		break;
					case '4':
						len = sprintf((char *)buf,(const char *)&env[4],NUMDT," ");
						break;																				   
					case '5':
						len = sprintf((char *)buf,(const char *)&env[4],NUMMAKB," ");
		          		break;
		     		case '6':
						len = sprintf((char *)buf,(const char *)&env[4],NUMENMV," ");
		          		break;
		     		case '7':
		         		if(!SK_ZVUK_EN[3])temp=1;
						else temp=0;
						len = sprintf((char *)buf,(const char *)&env[4],temp," ");
		          		break;
		     		case '8':
						len = sprintf((char *)buf,(const char *)&env[4],(signed)SNTP_GMT," ");
		          		break;		     		
					case '9':
		         		if(lc640_read_int(4472)==1)temp=0;
						else temp=1;
						len = sprintf((char *)buf,(const char *)&env[4],temp," ");
		          		break;																				   
				}
				break;
		}
		break;

    case 'x':
      if(uku_set_autorized)	len = sprintf((char *)buf,(const char *)&env[1],"ON");
	  else if(psw_err)		len = sprintf((char *)buf,(const char *)&env[1],"DENIED");
	  else 					len = sprintf((char *)buf,(const char *)&env[1],"OFF");
      break;

    case '2':
       
	  web_plazma[0]++;
      len = sprintf((char *)buf,"<checkbox><id>button%c</id><on>%s</on></checkbox>",
                    env[1],( 3 & (1 << (env[1]-'0'))) ? "true" : "false");
      break;
    case '3':
       
	  web_plazma[0]++;
      len = sprintf((char *)buf,"<checkbox><id>button%c</id><on>%s</on></checkbox>",
                    env[1],(  6 & (1 << (env[1]-'0'))) ? "true" : "false");
      break;
    case 'z':
		 
      	len = sprintf((char *)buf,(const char *)&env[2],"end");
		uku_set_autorized=0;
   		break;
  }
  return ((U16)len);
}




 
