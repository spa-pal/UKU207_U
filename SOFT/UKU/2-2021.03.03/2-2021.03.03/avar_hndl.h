

//***********************************************
//������
extern unsigned avar_stat;	 	//"�����������" ���� ��������� � ������ ������ ��������� � ����� �����
extern unsigned avar_ind_stat; 	//"�����������" ���� �� ������������� ��������� ��������� � ����� �����
extern unsigned avar_stat_old;
extern unsigned avar_stat_new,avar_stat_offed;
//��������� ����������
//1���  - �������� ����
//2���� - �������
//12��� - ����
//5���  - ���������
//4���� - ������� ������� �����������
//4���� - ������� ����� ��������

extern char sk_avar_stat;	 	//"�����������" ���� ��������� � ������ ������ �� � ����� �����
extern char sk_avar_ind_stat; 	//"�����������" ���� �� ������������� ��������� �� � ����� �����
extern char sk_avar_stat_old;
extern char sk_avar_stat_new,sk_avar_stat_offed;

//������
extern unsigned rki_avar1_stat;	 	//"�����������" ���� ��������� � ������ ������ ��������� � ����� �����
extern unsigned rki_avar1_ind_stat; 	//"�����������" ���� �� ������������� ��������� ��������� � ����� �����
extern unsigned rki_avar1_stat_old;
extern unsigned rki_avar1_stat_new, rki_avar1_stat_offed;
/*
extern unsigned rki_avarI1_stat;	 	//"�����������" ���� ��������� � ������ ������ ��������� � ����� �����
extern unsigned rki_avarI1_ind_stat; 	//"�����������" ���� �� ������������� ��������� ��������� � ����� �����
extern unsigned rki_avarI1_stat_old;
extern unsigned rki_avarI1_stat_new, rki_avarI1_stat_offed;

extern unsigned rki_avarI2_stat;	 	//"�����������" ���� ��������� � ������ ������ ��������� � ����� �����
extern unsigned rki_avarI2_ind_stat; 	//"�����������" ���� �� ������������� ��������� ��������� � ����� �����
extern unsigned rki_avarI2_stat_old;
extern unsigned rki_avarI2_stat_new, rki_avarI2_stat_offed;
*/
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



