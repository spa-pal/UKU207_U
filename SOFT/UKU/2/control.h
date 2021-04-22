#ifndef _CONTROL_H_

#define _CONTROL_H_


//**********************************************
//������ � ������
extern char num_of_wrks_bps;
extern char bps_all_off_cnt,bps_mask_off_cnt,bps_mask_on_off_cnt;
extern char bps_hndl_2sec_cnt;
extern unsigned bps_on_mask,bps_off_mask;
extern char num_necc_up,num_necc_down;
extern unsigned char sh_cnt0,b1Hz_sh;


extern short cntrl_stat_blok_cnt,cntrl_stat_blok_cnt_,cntrl_stat_blok_cnt_plus[2],cntrl_stat_blok_cnt_minus[2];

//***********************************************
//��� 
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
//void bp_on(char in);
//void bp_off(char in);
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
#ifdef UKU_ZVU
void amper_chas_cnt_drv(char in);
void bat_hndl_zvu(char in);
#endif

//***********************************************
//�����������

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
extern char vz_error;   // ���������������, ���� ���. ����� ������������

//***********************************************
//���������� ������������
extern signed char vent_stat;

//***********************************************
//���������� �����
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
#endif

extern signed int i_avg_max,i_avg_min,i_avg_summ,i_avg; 
extern signed int avg;
extern char bAVG;
extern const char sk_buff_TELECORE2015[4];

//**********************************************
//�������� ������� �������
extern signed short 	main_kb_cnt;
extern signed short 	kb_cnt_1lev;
extern signed short 	kb_cnt_2lev;
extern char 		kb_full_ver;
extern char kb_start[2],kb_start_ips;
extern signed short ibat_ips,ibat_ips_;

//***********************************************
//������� �������� ���������
extern char numOfForvardBps,numOfForvardBps_old;
extern char numOfForvardBps_minCnt;
extern short numOfForvardBps_hourCnt;

//***********************************************
// ������������ ������ � ������ ��������� ���������
extern char bPARALLEL_NOT_ENOUG;
extern char bPARALLEL_ENOUG;
extern char bPARALLEL;

//***********************************************
//������� ��������������
extern char rx_read_power_cnt_phase;
extern short read_power_cnt_main_cnt;
extern short ce102m_delayCnt;
extern char rx_read_power_cnt_plazma;
extern char rx_read_power_cnt_flag;
extern short volta_short;
extern short curr_short;
extern int power_int;

extern char bENERGOMETR_UIP;

//***********************************************
//������������� �����
typedef enum {vz1sOFF=0, vz1sSTEP1=1, vz1sSTEP2=2, vz1sSTEP3=3, vz1sWRK=10, vz1sERR1, vz1sERR2, vz1sERR3, vz1sERR4, vz1sFINE, vz1sSTOP}enum_vz1_stat;
extern enum_vz1_stat vz1_stat, vz1_stat_old;
extern short vz1_stat_cnt;
extern long vz1_wrk_cnt;
extern long vz1_up_cnt;
extern char volt_region;

//***********************************************
//����������� �����
typedef enum {vz2sOFF=0, vz2sSTEP1=1, vz2sSTEP2=2, vz2sSTEP3=3, vz2sWRK1=10, vz2sWRK2=11, vz2sERR1, vz2sERR2, vz2sERR3, vz2sERR4, vz2sERR5, vz2sERR6, vz2sFINE, vz2sSTOP}enum_vz2_stat;
extern enum_vz2_stat vz2_stat, vz2_stat_old;
extern short vz2_stat_cnt;
extern long vz2_wrk_cnt;
extern long vz2_up_cnt;
extern signed short vz2_stat_ph2_cnt;

//***********************************************
//������������ ������� ���
extern short I_from_t_table[2][7];
extern char bat_hndl_zvu_init[2];
extern short bat_hndl_i[2];
extern long bat_hndl_t_razr[2];				//�������������� ����� ������� ������� ������� (�� 100% �� �������)
extern long bat_hndl_t_razr_ke[2];				//�������������� ����� ������� ������� ������� (�� 100% �� �������) ��� �������� �������
extern long bat_hndl_zvu_Q[2];
extern long bat_hndl_proc_razr[2];
extern long bat_hndl_remain_time[2];
extern short bat_hndl_t_razr_hour[2];
extern short bat_hndl_t_razr_min[2];
extern short bat_hndl_t_razr_mininhour[2];
extern char bat_hndl_zvu_ke_init;
extern short bat_hndl_i_temp;
extern short bat_hndl_u_end[2];
extern short U_end_from_i_table[2][7];
extern long bat_hndl_plazma[5];
extern char bat_hndl_zvu_Q_cnt[2];
extern long amper_chas_cnt_drv_summ[2];
extern long amper_chas_cnt_drv_summ_[2];
extern char bat_hndl_i_vector[2],bat_hndl_i_vector_old[2];
extern long bat_hndl_i_zar_price[2];
extern long bat_hndl_i_summ[2];
extern char amper_chas_cnt_drv_cnter[2];
extern char bat_hndl_zvu_cnter[2];

extern char avar_bps_reset_cnt;

extern char cntrl_hndl_plazma,cntrl_hndl_plazma_;

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


