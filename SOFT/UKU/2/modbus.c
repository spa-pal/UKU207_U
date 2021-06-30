
#include "snmp_data_file.h" 
#include <lpc17xx.h>
#include "modbus.h"
//#include "LPC17xx.H"
#include "main.h"
#include "control.h"
#include <string.h>

#include "eeprom_map.h"
#include "rtl.h"
#include "modbus_tcp.h"
#include "25lc640.h"
#include "sc16is7xx.h"
#include "uart0.h"
#include "avar_hndl.h"
#include "curr_version.h"

#define MODBUS_RTU_PROT	0

extern int  mem_copy (void *dp, void *sp, int len);

unsigned char modbus_buf[20];
short modbus_crc16;
char modbus_timeout_cnt;
char bMODBUS_TIMEOUT;
unsigned char modbus_rx_buffer[30];	//�����, ���� ���������� ����������� ������� ���������� ���������� �� ������ ����� 
unsigned char modbus_an_buffer[30];    	//�����, ���� ��� ����� ���������� ��� �������
unsigned char modbus_rx_buffer_ptr;	//��������� �� ������� ������� ������������ ������
unsigned char modbus_rx_counter;		//���������� �������� ����, ������������ ��� ������� ����������� ������� � ��� �����������

short modbus_plazma;				//�������
short modbus_plazma1;				//�������
short modbus_plazma2;				//�������
short modbus_plazma3;				//�������
short modbus_plazma_p;				//�������
short modbus_plazma_pp;				//�������

unsigned short modbus_rx_arg0;		//���������� � ������� ������ ��������
unsigned short modbus_rx_arg1;		//���������� � ������� ������ ��������
unsigned short modbus_rx_arg2;		//���������� � ������� ������ ��������
unsigned short modbus_rx_arg3;		//���������� � ������� ��������� ��������

char modbus_tx_buff[100];

//char modbus_registers[200];

//static const char foo[] = "I wish I'd read K&R, and other tomes more diligently";



/*modbus_registers[3]=(char)(bps_I%256);
modbus_registers[4]=(char)(net_U/256);					//���3   	���������� ���� �������, 1�
modbus_registers[5]=(char)(net_U%256);
modbus_registers[6]=(char)(net_F/256);					//���4   	������� ���� �������, 0.1��
modbus_registers[7]=(char)(net_F%256);
modbus_registers[8]=(char)(net_Ua/256);					//���5	���������� ���� ������� ���� A, 1�	
modbus_registers[9]=(char)(net_Ua%256);		 	
modbus_registers[10]=(char)(net_Ub/256);				//���6	���������� ���� ������� ���� B, 1�
modbus_registers[11]=(char)(net_Ub%256);
modbus_registers[12]=(char)(net_Uc/256);				//���7	���������� ���� ������� ���� C, 1�
modbus_registers[13]=(char)(net_Uc%256);
modbus_registers[14]=(char)(bat[0]._Ub/256);				//���8	���������� ������� �1, 0.1�
modbus_registers[15]=(char)(bat[0]._Ub%256);
modbus_registers[16]=(char)(bat[0]._Ib/256);				//���9   	��� ������� �1, 0.01�
modbus_registers[17]=(char)(bat[0]._Ib%256);
modbus_registers[18]=(char)(bat[0]._Tb/256);				//���10	����������� ������� �1, 1��
modbus_registers[19]=(char)(bat[0]._Tb%256);
modbus_registers[20]=(char)(bat[0]._zar/256);			//���11	����� ������� �1, %
modbus_registers[21]=(char)(bat[0]._zar%256);
modbus_registers[22]=(char)(bat[0]._Ubm/256);			//���12	���������� ������� ����� ������� �1, 0.1�
modbus_registers[23]=(char)(bat[0]._Ubm%256);
modbus_registers[24]=(char)(bat[0]._dUbm/256);			//���13	������ ������� ����� ������� �1, %
modbus_registers[25]=(char)(bat[0]._dUbm%256);
modbus_registers[26]=(char)(BAT_C_REAL[0]/256);			//���14	�������� ������� ������� �1, 0.1�*�, ���� 0x5555 �� �� ����������
modbus_registers[27]=(char)(BAT_C_REAL[0]%256);
modbus_registers[28]=(char)(bat[1]._Ub/256);				//���15	���������� ������� �1, 0.1�
modbus_registers[29]=(char)(bat[1]._Ub%256);
modbus_registers[30]=(char)(bat[1]._Ib/256);				//���16   	��� ������� �1, 0.01�
modbus_registers[31]=(char)(bat[1]._Ib%256);
modbus_registers[32]=(char)(bat[1]._Tb/256);				//���17	����������� ������� �1, 1��
modbus_registers[33]=(char)(bat[1]._Tb%256);
modbus_registers[34]=(char)(bat[1]._zar/256);			//���18	����� ������� �1, %
modbus_registers[35]=(char)(bat[1]._zar%256);
modbus_registers[36]=(char)(bat[1]._Ubm/256);			//���19	���������� ������� ����� ������� �1, 0.1�
modbus_registers[37]=(char)(bat[1]._Ubm%256);
modbus_registers[38]=(char)(bat[1]._dUbm/256);			//���20	������ ������� ����� ������� �1, %
modbus_registers[39]=(char)(bat[1]._dUbm%256);
modbus_registers[40]=(char)(BAT_C_REAL[1]/256);			//���21	�������� ������� ������� �1, 0.1�*�, ���� 0x5555 �� �� ����������
modbus_registers[41]=(char)(BAT_C_REAL[1]%256);
modbus_registers[42]=(char)(bps[0]._Uii/256);			//���22	�������� ���������� ����������� �1, 0.1�
modbus_registers[43]=(char)(bps[0]._Uii%256);
modbus_registers[44]=(char)(bps[0]._Ii/256);				//���23	�������� ��� ����������� �1, 0.1�
modbus_registers[45]=(char)(bps[0]._Ii%256);
modbus_registers[46]=(char)(bps[0]._Ti/256);				//���24	����������� ��������� ����������� �1, 1��
modbus_registers[47]=(char)(bps[0]._Ti%256);
modbus_registers[48]=(char)(bps[0]._av/256);				//���25	���� ������ ����������� �1, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[49]=(char)(bps[0]._av%256);
modbus_registers[50]=(char)(bps[1]._Uii/256);			//���26	�������� ���������� ����������� �2, 0.1�
modbus_registers[51]=(char)(bps[1]._Uii%256);
modbus_registers[52]=(char)(bps[1]._Ii/256);				//���27	�������� ��� ����������� �2, 0.1�
modbus_registers[53]=(char)(bps[1]._Ii%256);
modbus_registers[54]=(char)(bps[1]._Ti/256);				//���28	����������� ��������� ����������� �2, 1��
modbus_registers[55]=(char)(bps[1]._Ti%256);
modbus_registers[56]=(char)(bps[1]._av/256);				//���29	���� ������ ����������� �2, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[57]=(char)(bps[1]._av%256);
modbus_registers[58]=(char)(bps[2]._Uii/256);			//���30	�������� ���������� ����������� �3, 0.1�
modbus_registers[59]=(char)(bps[2]._Uii%256);
modbus_registers[60]=(char)(bps[2]._Ii/256);				//���31	�������� ��� ����������� �3, 0.1�
modbus_registers[61]=(char)(bps[2]._Ii%256);
modbus_registers[62]=(char)(bps[2]._Ti/256);				//���32	����������� ��������� ����������� �3, 1��
modbus_registers[63]=(char)(bps[2]._Ti%256);
modbus_registers[64]=(char)(bps[2]._av/256);				//���33	���� ������ ����������� �3, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[65]=(char)(bps[2]._av%256);
modbus_registers[66]=(char)(bps[3]._Uii/256);			//���34	�������� ���������� ����������� �4, 0.1�
modbus_registers[67]=(char)(bps[3]._Uii%256);
modbus_registers[68]=(char)(bps[3]._Ii/256);				//���35	�������� ��� ����������� �4, 0.1�
modbus_registers[69]=(char)(bps[3]._Ii%256);
modbus_registers[70]=(char)(bps[3]._Ti/256);				//���36	����������� ��������� ����������� �4, 1��
modbus_registers[71]=(char)(bps[3]._Ti%256);
modbus_registers[72]=(char)(bps[3]._av/256);				//���37	���� ������ ����������� �4, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[73]=(char)(bps[3]._av%256);
modbus_registers[74]=(char)(bps[4]._Uii/256);			//���38	�������� ���������� ����������� �5, 0.1�
modbus_registers[75]=(char)(bps[4]._Uii%256);
modbus_registers[76]=(char)(bps[4]._Ii/256);				//���39	�������� ��� ����������� �5, 0.1�
modbus_registers[77]=(char)(bps[4]._Ii%256);
modbus_registers[78]=(char)(bps[4]._Ti/256);				//���40	����������� ��������� ����������� �5, 1��
modbus_registers[79]=(char)(bps[4]._Ti%256);
modbus_registers[80]=(char)(bps[4]._av/256);				//���41	���� ������ ����������� �5, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[81]=(char)(bps[4]._av%256);
modbus_registers[82]=(char)(bps[5]._Uii/256);			//���42	�������� ���������� ����������� �6, 0.1�
modbus_registers[83]=(char)(bps[5]._Uii%256);
modbus_registers[84]=(char)(bps[5]._Ii/256);				//���43	�������� ��� ����������� �6, 0.1�
modbus_registers[85]=(char)(bps[5]._Ii%256);
modbus_registers[86]=(char)(bps[5]._Ti/256);				//���44	����������� ��������� ����������� �6, 1��
modbus_registers[87]=(char)(bps[5]._Ti%256);
modbus_registers[88]=(char)(bps[5]._av/256);				//���45	���� ������ ����������� �6, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[89]=(char)(bps[5]._av%256);
modbus_registers[90]=(char)(bps[6]._Uii/256);			//���46	�������� ���������� ����������� �7, 0.1�
modbus_registers[91]=(char)(bps[6]._Uii%256);
modbus_registers[92]=(char)(bps[6]._Ii/256);				//���47	�������� ��� ����������� �7, 0.1�
modbus_registers[93]=(char)(bps[6]._Ii%256);
modbus_registers[94]=(char)(bps[6]._Ti/256);				//���48	����������� ��������� ����������� �7, 1��
modbus_registers[95]=(char)(bps[6]._Ti%256);
modbus_registers[96]=(char)(bps[6]._av/256);				//���49	���� ������ ����������� �7, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[97]=(char)(bps[6]._av%256);
modbus_registers[98]=(char)(bps[7]._Uii/256);			//���50	�������� ���������� ����������� �8, 0.1�
modbus_registers[99]=(char)(bps[7]._Uii%256);
modbus_registers[100]=(char)(bps[7]._Ii/256);			//���51	�������� ��� ����������� �8, 0.1�
modbus_registers[101]=(char)(bps[7]._Ii%256);
modbus_registers[102]=(char)(bps[7]._Ti/256);			//���52	����������� ��������� ����������� �8, 1��
modbus_registers[103]=(char)(bps[7]._Ti%256);
modbus_registers[104]=(char)(bps[7]._av/256);			//���53	���� ������ ����������� �8, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[105]=(char)(bps[7]._av%256);
modbus_registers[106]=(char)(bps_U/256);					//���54   	���������� ������������, 0.1�
modbus_registers[107]=(char)(bps_U%256);
tempS=0;
if(speedChIsOn) tempS=1;
modbus_registers[108]=(char)(tempS/256);					//���55   	���������� ����� ������������, (1 - ���, 0 - ����)
modbus_registers[109]=(char)(tempS%256);
tempS=0;
if(spc_stat==spcVZ) tempS=1;
modbus_registers[110]=(char)(tempS/256);					//���56   	������������� ����� ������������, (1 - ���, 0 - ����)
modbus_registers[111]=(char)(tempS%256);
modbus_registers[112]=(char)(uout_av/256);					//���57   �������� ��������� ����������, (0 - �����, 1 - ��������, 2 - ��������)
modbus_registers[113]=(char)(uout_av%256);

tempS=t_ext[0];
if(ND_EXT[0])tempS=-1000;
modbus_registers[400]=(char)(tempS/256);				//���201	������� ������ ����������� �1
modbus_registers[401]=(char)(tempS%256);
tempS=t_ext[1];
if(ND_EXT[1])tempS=-1000;
modbus_registers[402]=(char)(tempS/256);				//���202	������� ������ ����������� �2
modbus_registers[403]=(char)(tempS%256);
tempS=t_ext[2];
if(ND_EXT[2])tempS=-1000;
modbus_registers[404]=(char)(tempS/256);				//���203	������� ������ ����������� �3
modbus_registers[405]=(char)(tempS%256);
tempS=t_ext[3];
if(ND_EXT[3])tempS=-1000;
modbus_registers[406]=(char)(tempS/256);				//���204	������� ������ ����������� �4
modbus_registers[407]=(char)(tempS%256);

tempS=0;
if(sk_stat[0]==ssON) tempS|=0x0001;
if(sk_av_stat[0]==sasON) tempS|=0x0002;
modbus_registers[420]=(char)(tempS/256);				//���211	���������  ������ �������� �1, (������� ��� - ���������� ���������, 1 - �������, 0 - ���������, ������ ��� - �����������, 1 - ������, 0 - �����)
modbus_registers[421]=(char)(tempS%256);
tempS=0;
if(sk_stat[1]==ssON) tempS|=0x0001;
if(sk_av_stat[1]==sasON) tempS|=0x0002;
modbus_registers[422]=(char)(tempS/256);				//���212	���������  ������ �������� �1, (������� ��� - ���������� ���������, 1 - �������, 0 - ���������, ������ ��� - �����������, 1 - ������, 0 - �����)
modbus_registers[423]=(char)(tempS%256);
tempS=0;
if(sk_stat[2]==ssON) tempS|=0x0001;
if(sk_av_stat[2]==sasON) tempS|=0x0002;
modbus_registers[424]=(char)(tempS/256);				//���213	���������  ������ �������� �1, (������� ��� - ���������� ���������, 1 - �������, 0 - ���������, ������ ��� - �����������, 1 - ������, 0 - �����)
modbus_registers[425]=(char)(tempS%256);
tempS=0;
if(sk_stat[3]==ssON) tempS|=0x0001;
if(sk_av_stat[3]==sasON) tempS|=0x0002;
modbus_registers[426]=(char)(tempS/256);				//���214	���������  ������ �������� �1, (������� ��� - ���������� ���������, 1 - �������, 0 - ���������, ������ ��� - �����������, 1 - ������, 0 - �����)
modbus_registers[427]=(char)(tempS%256);

if(prot==MODBUS_RTU_PROT)
	{
	modbus_tx_buff[0]=adr;
	modbus_tx_buff[1]=func;

	modbus_tx_buff[2]=(char)(reg_quantity*2);

	mem_copy((char*)&modbus_tx_buff[3],(char*)&modbus_registers[(reg_adr-1)*2],reg_quantity*2);

	crc_temp=CRC16_2(modbus_tx_buff,(reg_quantity*2)+3);

	modbus_tx_buff[3+(reg_quantity*2)]=crc_temp%256;
	modbus_tx_buff[4+(reg_quantity*2)]=crc_temp/256;

	for (i=0;i<(5+(reg_quantity*2));i++)
		{
		putchar0(modbus_tx_buff[i]);
		}
	for (i=0;i<(5+(reg_quantity*2));i++)
		{
		putchar_sc16is700(modbus_tx_buff[i]);
		}
	}
else if(prot==MODBUS_TCP_PROT)
	{
	modbus_tcp_out_ptr=(char*)&modbus_registers[(reg_adr-1)*2];
	} */

	//   	};


//-----------------------------------------------
unsigned short CRC16_2(char* buf, short len)
{
unsigned short crc = 0xFFFF;
short pos;
short i;

for (pos = 0; pos < len; pos++)
  	{
    	crc ^= (unsigned short)buf[pos];          // XOR byte into least sig. byte of crc

    	for ( i = 8; i != 0; i--) 
		{    // Loop over each bit
      	if ((crc & 0x0001) != 0) 
			{      // If the LSB is set
        		crc >>= 1;                    // Shift right and XOR 0xA001
        		crc ^= 0xA001;
      		}
      	else  crc >>= 1;                    // Just shift right
    		}
  	}
  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
return crc;
}

//-----------------------------------------------
//o_12_s
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

//-----------------------------------------------
void modbus_in(void)
{
short crc16_calculated;		//����������� �� �������� ������ CRC
short crc16_incapsulated;	//����������� � ������� CRC
unsigned short modbus_rx_arg0;		//���������� � ������� ������ ��������
unsigned short modbus_rx_arg1;		//���������� � ������� ������ ��������
//unsigned short modbus_rx_arg2;		//���������� � ������� ������ ��������
//unsigned short modbus_rx_arg3;		//���������� � ������� ��������� ��������
unsigned char modbus_func;			//���������� � ������� ��� �������
char i_cnt, j_cnt; //o_2


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
//modbus_rx_arg2=(((unsigned short)modbus_an_buffer[6])*((unsigned short)256))+((unsigned short)modbus_an_buffer[7]);
//modbus_rx_arg3=(((unsigned short)modbus_an_buffer[8])*((unsigned short)256))+((unsigned short)modbus_an_buffer[9]);

//#define IPS_CURR_AVG_MODBUS_ADRESS	1


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
//	pvlk=1;
	if(modbus_an_buffer[1]=='r')
		{
//		pvlk=2;
		if(modbus_an_buffer[2]=='i')
			{
//			pvlk=3;
			if(modbus_an_buffer[3]=='t')
				{
//				pvlk=4;
				if(modbus_an_buffer[4]=='e')
					{
//					pvlk=5;
					if(modbus_an_buffer[15]==crc_87(modbus_an_buffer,15))
						{
//						pvlk=6;
						if(modbus_an_buffer[16]==crc_95(modbus_an_buffer,15))

							{
							unsigned short ptr;
							unsigned long data1,data2;
							char temp_out[20];
//							pvlk=7;
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
							
							//data1=lc640_read_long(ptr);
							//data2=lc640_read_long(ptr+4);
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
	//o_12_s
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
			
 	else if(modbus_an_buffer[0]==MODBUS_ADRESS)	  //o_12_e
		{
		modbus_modbus_adress_eq++;
		if(modbus_func==3)		//������ ������������� ���-�� ��������� ��������
			{
			modbus_plazma++;
			modbus_hold_registers_transmit(MODBUS_ADRESS,modbus_func,modbus_rx_arg0,modbus_rx_arg1,MODBUS_RTU_PROT);
			}

		if(modbus_func==4)		//������ ������������� ���-�� ���������	������
			{
			modbus_input_registers_transmit(MODBUS_ADRESS,modbus_func,modbus_rx_arg0,modbus_rx_arg1,MODBUS_RTU_PROT);
			modbus_modbus4f_cnt++;
			}

		else if(modbus_func==6) 	//������ ��������� ��������
			{
			if(modbus_rx_arg0==11)		//��������� ������� 
				{
				LPC_RTC->YEAR=(uint16_t)modbus_rx_arg1;
				}
			if(modbus_rx_arg0==12)		//��������� ������� 
				{
				LPC_RTC->MONTH=(uint16_t)modbus_rx_arg1;
				}
			if(modbus_rx_arg0==13)		//��������� ������� 
				{
				LPC_RTC->DOM=(uint16_t)modbus_rx_arg1;
				}
			if(modbus_rx_arg0==14)		//��������� ������� 
				{
				LPC_RTC->HOUR=(uint16_t)modbus_rx_arg1;
				}
			if(modbus_rx_arg0==15)		//��������� ������� 
				{
				LPC_RTC->MIN=(uint16_t)modbus_rx_arg1;
				}
			if(modbus_rx_arg0==16)		//��������� ������� 
				{
				LPC_RTC->SEC=(uint16_t)modbus_rx_arg1;
				}
			if(modbus_rx_arg0==20)		//��� ������������ ��� ������ ������������ ����
				{
				if((modbus_rx_arg1>=0)&&(modbus_rx_arg1<=18))
				lc640_write_int(EE_NUMIST,modbus_rx_arg1);  
				}
			if(modbus_rx_arg0==21)		//��� ������������ ��� ������ ������������ ����
				{
				if((modbus_rx_arg1==0)||(modbus_rx_arg1==1))
				lc640_write_int(EE_PAR,modbus_rx_arg1);  
				}
			if(modbus_rx_arg0==22)		//��� ������������ ��� ������ ������������ ����
				{
				if((modbus_rx_arg1==0)||(modbus_rx_arg1==1))
				lc640_write_int(EE_ZV_ON,modbus_rx_arg1);  
				}
			if(modbus_rx_arg0==23)		//��� ������������ ��� ������ ������������ ����
				{
				if((modbus_rx_arg1==0)||(modbus_rx_arg1==1))
				lc640_write_int(EE_TERMOKOMP,modbus_rx_arg1);  
				}
			if(modbus_rx_arg0==24)		//��� ������������ ��� ������ ������������ ����
				{
				if(/*(modbus_rx_arg1>=0)||*/(modbus_rx_arg1<=20))
				lc640_write_int(EE_UBM_AV,modbus_rx_arg1);  
				}


			if(modbus_rx_arg0==30)		//���������� ������������ ��� ������ ������������ ����������
				{
				if((modbus_rx_arg1>0)&&(modbus_rx_arg1<5))modbus_rx_arg1=0;
				else if(modbus_rx_arg1>=60)TBAT=60;
				else TBAT=modbus_rx_arg1;
				lc640_write_int(EE_TBAT,TBAT);

				main_kb_cnt=(TBAT*60)-20;
	     		}
			if(modbus_rx_arg0==31)		//
				{
				lc640_write_int(EE_UMAX,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==32)		//
				{
				lc640_write_int(EE_DU,UB20-modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==33)		//
				{
				lc640_write_int(EE_UB0,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==34)		//
				{
				lc640_write_int(EE_UB20,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==35)		//
				{
				lc640_write_int(EE_USIGN,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==36)		//
				{
				lc640_write_int(EE_UMN,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==37)		//
				{
				lc640_write_int(EE_U0B,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==38)		//
				{
				lc640_write_int(EE_IKB,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==39)		//
				{
				lc640_write_int(EE_IZMAX,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==40)		//
				{
				lc640_write_int(EE_IMAX,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==41)		//
				{
				lc640_write_int(EE_IMIN,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==42)		//
				{
				lc640_write_int(EE_UVZ,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==43)		//
				{
				if((modbus_rx_arg1>=0)&&(modbus_rx_arg1<=3))lc640_write_int(EE_TZAS,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==44)		//
				{
				lc640_write_int(EE_TMAX,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==45)		//
				{
				lc640_write_int(EE_TSIGN,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==46)		//
				{
				lc640_write_int(EE_TBATMAX,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==47)		//
				{
				lc640_write_int(EE_TBATSIGN,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==48)		//
				{
				lc640_write_int(EE_SPEED_CHRG_CURR,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==49)		//
				{
				lc640_write_int(EE_SPEED_CHRG_VOLT,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==50)		//
				{
				lc640_write_int(EE_SPEED_CHRG_TIME,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==51)		//
				{
				lc640_write_int(EE_U_OUT_KONTR_MAX,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==52)		//
				{
				lc640_write_int(EE_U_OUT_KONTR_MIN,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==53)		//
				{
				if((modbus_rx_arg1>=5)&&(modbus_rx_arg1<=100))lc640_write_int(EE_U_OUT_KONTR_DELAY,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==54)		//
				{
				lc640_write_int(EE_UB0,modbus_rx_arg1);
				lc640_write_int(EE_UB20,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==55)		//
				{
				lc640_write_int(EE_UMAXN,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==56)		
				{
				if(modbus_rx_arg1<=3) lc640_write_int(EE_SNTP_ENABLE,modbus_rx_arg1);
	     		}						
			if(modbus_rx_arg0==57)		
				{
				signed short www=(signed short)modbus_rx_arg1;
				if(www>=-12 && www<=13) lc640_write_int(EE_SNTP_GMT,modbus_rx_arg1);
	     		}						
			if(modbus_rx_arg0==58)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_SNTP_IP1,modbus_rx_arg1);
	     		}						
			if(modbus_rx_arg0==59)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_SNTP_IP2,modbus_rx_arg1);
	     		}						
			if(modbus_rx_arg0==60)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_SNTP_IP3,modbus_rx_arg1);
	     		}						
			if(modbus_rx_arg0==61)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_SNTP_IP4,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==62)		
				{
			/*	#ifdef UKU_220_IPS_TERMOKOMPENSAT
				
				#else
				if(modbus_rx_arg1<=1) lc640_write_int(EE_NUMBAT,modbus_rx_arg1);
				#endif */
				if(modbus_rx_arg1<=2) lc640_write_int(EE_NUMBAT,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==63)		
				{
				if(modbus_rx_arg1<=3) lc640_write_int(EE_NUMDT,modbus_rx_arg1);
	     		}	
			if(modbus_rx_arg0==64)		
				{
				if(modbus_rx_arg1==0 || modbus_rx_arg1==2 || modbus_rx_arg1==4) lc640_write_int(EE_NUMMAKB,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==65)		
				{
				if(modbus_rx_arg1<=4) lc640_write_int(EE_NUMSK,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==66)		
				{
				if(modbus_rx_arg1==0 || modbus_rx_arg1==1) lc640_write_int(EE_NUM_RKI,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==67)		
				{
				if(modbus_rx_arg1==0 || modbus_rx_arg1==1) lc640_write_int(EE_NUM_NET_IN,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==68)		
				{
				if(modbus_rx_arg1==0 || modbus_rx_arg1==1) lc640_write_int(EE_NUMBDR,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==69)		
				{
				if(modbus_rx_arg1<9) lc640_write_int(EE_NUMENMV,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==70)		
				{
				if(modbus_rx_arg1>=1 && modbus_rx_arg1<=200) lc640_write_int(EE_BAT_C_POINT_NUM_ELEM,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==71)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=25000) lc640_write_int(EE_BAT_C_POINT_20,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==72)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=21000) lc640_write_int(EE_BAT_C_POINT_10,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==73)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=20000) lc640_write_int(EE_BAT_C_POINT_5,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==74)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=18000) lc640_write_int(EE_BAT_C_POINT_3,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==75)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=16000) lc640_write_int(EE_BAT_C_POINT_1,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==76)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=13000) lc640_write_int(EE_BAT_C_POINT_1_2,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==77)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=8000) lc640_write_int(EE_BAT_C_POINT_1_6,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==78)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=10000) lc640_write_int(EE_BAT_U_END_20,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==79)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=10000) lc640_write_int(EE_BAT_U_END_10,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==80)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=10000) lc640_write_int(EE_BAT_U_END_5,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==81)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=10000) lc640_write_int(EE_BAT_U_END_3,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==82)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=10000) lc640_write_int(EE_BAT_U_END_1,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==83)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=10000) lc640_write_int(EE_BAT_U_END_1_2,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==84)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=10000) lc640_write_int(EE_BAT_U_END_1_6,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==85)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=100) lc640_write_int(EE_BAT_K_OLD,modbus_rx_arg1);
	     		}		
			if(modbus_rx_arg0==86)		
				{
				if(modbus_rx_arg1>=15 && modbus_rx_arg1<=250) lc640_write_int(EE_UVENTOFF,modbus_rx_arg1);
	     		}			
			if(modbus_rx_arg0==87)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=2000) lc640_write_int(EE_IMAX_VZ,modbus_rx_arg1);
	     		}  
			if(modbus_rx_arg0==88)		
				{
				if(modbus_rx_arg1<=72) lc640_write_int(EE_VZ_HR,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==89)		
				{
				if(modbus_rx_arg1==0 || modbus_rx_arg1==1) lc640_write_int(EE_VZ_CH_VENT_BLOK,modbus_rx_arg1);
	     		} 
			if(modbus_rx_arg0==90)		
				{
				if(modbus_rx_arg1==0 || modbus_rx_arg1==1) lc640_write_int(EE_SPEED_CHRG_AVT_EN,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==91)		
				{
				if(modbus_rx_arg1>0 && modbus_rx_arg1<=100) lc640_write_int(EE_SPEED_CHRG_D_U,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==92)		
				{
				if(modbus_rx_arg1==0 || modbus_rx_arg1==2) lc640_write_int(EE_SPEED_CHRG_BLOCK_SRC,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==93)		
				{
				if(modbus_rx_arg1==0 || modbus_rx_arg1==1) lc640_write_int(EE_SPEED_CHRG_BLOCK_LOG,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==94)		
				{								
				if(modbus_rx_arg1==0 || modbus_rx_arg1==1) lc640_write_int(EE_SP_CH_VENT_BLOK,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==95)		
				{								
				if(modbus_rx_arg1>=UB20 && modbus_rx_arg1<=2600) lc640_write_int(EE_UZ_U,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==96)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=10000) lc640_write_int(EE_UZ_IMAX,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==97)		
				{
				if(modbus_rx_arg1>=1 && modbus_rx_arg1<=72) lc640_write_int(EE_UZ_T,modbus_rx_arg1);
	     		}					  
			if(modbus_rx_arg0==98)		
				{
				if(modbus_rx_arg1>=UB20 && modbus_rx_arg1<=3000) lc640_write_int(EE_FZ_U1,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==99)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=1000) lc640_write_int(EE_FZ_IMAX1,modbus_rx_arg1);
	     		}
			// 2 �������� �����
			if(modbus_rx_arg0==102)		
				{
				if(modbus_rx_arg1>=1 && modbus_rx_arg1<=10) lc640_write_int(EE_FZ_T1,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==103)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=1000) lc640_write_int(EE_FZ_ISW12,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==104)		
				{
				if(modbus_rx_arg1>=UB20 && modbus_rx_arg1<=3000) lc640_write_int(EE_FZ_U2,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==105)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=1000) lc640_write_int(EE_FZ_IMAX2,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==106)		
				{
				if(modbus_rx_arg1>=1 && modbus_rx_arg1<=10) lc640_write_int(EE_FZ_T2,modbus_rx_arg1);
	     		}	 
			if(modbus_rx_arg0==107)		
				{
				if(modbus_rx_arg1==0 || modbus_rx_arg1==1) lc640_write_int(EE_AV_OFF_AVT,modbus_rx_arg1);
	     		}				
		   	if(modbus_rx_arg0==108)		
				{
				if(modbus_rx_arg1==0 || modbus_rx_arg1==1) lc640_write_int(EE_APV_ON1,modbus_rx_arg1);
	     		}
		  	if(modbus_rx_arg0==109)		
				{
				if(modbus_rx_arg1==0 || modbus_rx_arg1==1) lc640_write_int(EE_APV_ON2,modbus_rx_arg1);
	     		}		   
		 	if(modbus_rx_arg0==110)		
				{
				if(modbus_rx_arg1>=1 && modbus_rx_arg1<=24) lc640_write_int(EE_APV_ON2_TIME,modbus_rx_arg1);
	     		}  
			if(modbus_rx_arg0==111)		
				{
				if(modbus_rx_arg1==0)	   lc640_write_int(EE_SK_SIGN0,0xFFFF);
				else if(modbus_rx_arg1==1) lc640_write_int(EE_SK_SIGN0,0);
	     		} 	   
			if(modbus_rx_arg0==112)		
				{
				if(modbus_rx_arg1==0)	   lc640_write_int(EE_SK_ZVUK_EN0,0xFFFF);
				else if(modbus_rx_arg1==1) lc640_write_int(EE_SK_ZVUK_EN0,0);
	     		}
			if(modbus_rx_arg0==113)		
				{
				if(modbus_rx_arg1==0)	   lc640_write_int(EE_SK_LCD_EN0,0xFFFF);
				else if(modbus_rx_arg1==1) lc640_write_int(EE_SK_LCD_EN0,0);
	     		}
			if(modbus_rx_arg0==114)		
				{
				if(modbus_rx_arg1==0)	   lc640_write_int(EE_SK_SIGN1,0xFFFF);
				else if(modbus_rx_arg1==1) lc640_write_int(EE_SK_SIGN1,0);
	     		} 	   
			if(modbus_rx_arg0==115)		
				{
				if(modbus_rx_arg1==0)	   lc640_write_int(EE_SK_ZVUK_EN1,0xFFFF);
				else if(modbus_rx_arg1==1) lc640_write_int(EE_SK_ZVUK_EN1,0);
	     		}
			if(modbus_rx_arg0==116)		
				{
				if(modbus_rx_arg1==0)	   lc640_write_int(EE_SK_LCD_EN1,0xFFFF);
				else if(modbus_rx_arg1==1) lc640_write_int(EE_SK_LCD_EN1,0);
	     		}
			if(modbus_rx_arg0==117)		
				{
				if(modbus_rx_arg1==0)	   lc640_write_int(EE_SK_SIGN2,0xFFFF);
				else if(modbus_rx_arg1==1) lc640_write_int(EE_SK_SIGN2,0);
	     		} 	   
			if(modbus_rx_arg0==118)		
				{
				if(modbus_rx_arg1==0)	   lc640_write_int(EE_SK_ZVUK_EN2,0xFFFF);
				else if(modbus_rx_arg1==1) lc640_write_int(EE_SK_ZVUK_EN2,0);
	     		}
			if(modbus_rx_arg0==119)		
				{
				if(modbus_rx_arg1==0)	   lc640_write_int(EE_SK_LCD_EN2,0xFFFF);
				else if(modbus_rx_arg1==1) lc640_write_int(EE_SK_LCD_EN2,0);
	     		}
			if(modbus_rx_arg0==120)		
				{
				if(modbus_rx_arg1==0)	   lc640_write_int(EE_SK_SIGN3,0xFFFF);
				else if(modbus_rx_arg1==1) lc640_write_int(EE_SK_SIGN3,0);
	     		} 	   
			if(modbus_rx_arg0==121)		
				{
				if(modbus_rx_arg1==0)	   lc640_write_int(EE_SK_ZVUK_EN3,0xFFFF);
				else if(modbus_rx_arg1==1) lc640_write_int(EE_SK_ZVUK_EN3,0);
	     		}
			if(modbus_rx_arg0==122)		
				{
				if(modbus_rx_arg1==0)	   lc640_write_int(EE_SK_LCD_EN3,0xFFFF);
				else if(modbus_rx_arg1==1) lc640_write_int(EE_SK_LCD_EN3,0);
	     		}		  
		   	if(modbus_rx_arg0==123)		
				{
				if(modbus_rx_arg1==0 || modbus_rx_arg1==1) lc640_write_int(EE_TERMOKOMP,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==124)		
				{
				if(modbus_rx_arg1<=500) lc640_write_int(EE_FORVARDBPSCHHOUR,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==125)		
				{
				if(modbus_rx_arg1==0 || modbus_rx_arg1==1) lc640_write_int(EE_DOP_RELE_FUNC,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==126)		
				{
				if(modbus_rx_arg1<=2) lc640_write_int(EE_IPS_BLOCK_SRC,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==127)		
				{
				if(modbus_rx_arg1==0 || modbus_rx_arg1==1) lc640_write_int(EE_IPS_BLOCK_LOG,modbus_rx_arg1);
	     		} 
		   	if(modbus_rx_arg0==128)		
				{
				if(modbus_rx_arg1>0 && modbus_rx_arg1<=100) lc640_write_int(EE_MODBUS_ADRESS,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==129)		
				{
				if(modbus_rx_arg1==120 || modbus_rx_arg1==240 || modbus_rx_arg1==480 || modbus_rx_arg1==960 || modbus_rx_arg1==1920 
				|| modbus_rx_arg1==3840 || modbus_rx_arg1==5760 || modbus_rx_arg1==11520){ 
					lc640_write_int(EE_MODBUS_BAUDRATE,modbus_rx_arg1);
					MODBUS_BAUDRATE=modbus_rx_arg1;
					}
	     		} 	
			if(modbus_rx_arg0==130)		
				{
				if(modbus_rx_arg1==0 || modbus_rx_arg1==1) lc640_write_int(EE_ETH_IS_ON,modbus_rx_arg1);
	     		}	 
			if(modbus_rx_arg0==131)		
				{
				if(modbus_rx_arg1==0 || modbus_rx_arg1==1) lc640_write_int(EE_ETH_DHCP_ON,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==132)  //IP �����
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_ETH_IP_1,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==133)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_ETH_IP_2,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==134)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_ETH_IP_3,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==135)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_ETH_IP_4,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==136)	//����� �������	
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_ETH_MASK_1,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==137)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_ETH_MASK_2,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==138)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_ETH_MASK_3,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==139)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_ETH_MASK_4,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==140)	//����
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_ETH_GW_1,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==141)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_ETH_GW_2,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==142)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_ETH_GW_3,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==143)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_ETH_GW_4,modbus_rx_arg1);
	     		}	 
			if(modbus_rx_arg0==144)		
				{
				lc640_write_int(EE_ETH_SNMP_PORT_READ,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==145)		
				{
				lc640_write_int(EE_ETH_SNMP_PORT_WRITE,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==146)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_COMMUNITY,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==147)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_COMMUNITY+2,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==148)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_COMMUNITY+4,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==149)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_COMMUNITY+6,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==150)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_COMMUNITY+8,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==151)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_COMMUNITY+10,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==152)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_COMMUNITY+12,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==153)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_COMMUNITY+14,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==154)  //TRAP1 IP �����
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_ETH_TRAP1_IP_1,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==155)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_ETH_TRAP1_IP_2,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==156)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_ETH_TRAP1_IP_3,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==157)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_ETH_TRAP1_IP_4,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==158)  //TRAP2 IP �����
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_ETH_TRAP2_IP_1,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==159)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_ETH_TRAP2_IP_2,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==160)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_ETH_TRAP2_IP_3,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==161)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_ETH_TRAP2_IP_4,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==162)  //TRAP3 IP �����
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_ETH_TRAP3_IP_1,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==163)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_ETH_TRAP3_IP_2,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==164)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_ETH_TRAP3_IP_3,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==165)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_ETH_TRAP3_IP_4,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==166)  //TRAP4 IP �����
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_ETH_TRAP4_IP_1,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==167)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_ETH_TRAP4_IP_2,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==168)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_ETH_TRAP4_IP_3,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==169)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_ETH_TRAP4_IP_4,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==170)  //TRAP5 IP �����
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_ETH_TRAP5_IP_1,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==171)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_ETH_TRAP5_IP_2,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==172)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_ETH_TRAP5_IP_3,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==173)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_ETH_TRAP5_IP_4,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==174)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_WEB_PASSWORD,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==175)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_WEB_PASSWORD+2,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==176)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_WEB_PASSWORD+4,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==177 && modbus_rx_arg1>0) bRESET_INT_WDT=1;// ���������� ���, ���������������� ��������    
			if(modbus_rx_arg0==178)		
				{
				modbus_rx_arg1/=10;
				if(modbus_rx_arg1<=6000) lc640_write_int(EE_TVENTMAX,modbus_rx_arg1);
	     		} 
		   	if(modbus_rx_arg0==179)		
				{
				if(modbus_rx_arg1<=2) lc640_write_int(EE_ICA_EN,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==180)		
				{
				if(modbus_rx_arg1<=2) lc640_write_int(EE_ICA_CH,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==181)  //IP ����� ������� ���
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_ICA_MODBUS_TCP_IP1,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==182)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_ICA_MODBUS_TCP_IP2,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==183)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_ICA_MODBUS_TCP_IP3,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==184)		
				{
				if(modbus_rx_arg1<=255) lc640_write_int(EE_ICA_MODBUS_TCP_IP4,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==185)		
				{
				if(modbus_rx_arg1>0 && modbus_rx_arg1<=254) lc640_write_int(EE_ICA_MODBUS_TCP_UNIT_ID,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==186)		
				{
				if(modbus_rx_arg1>0 && modbus_rx_arg1<=254) lc640_write_int(EE_ICA_MODBUS_ADDRESS,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==187)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=100) lc640_write_int(EE_PWM_START,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==188)		
				{
				if(modbus_rx_arg1>=1 && modbus_rx_arg1<=3) lc640_write_int(EE_KB_ALGORITM,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==189)		
				{
				if(modbus_rx_arg1>=1 && modbus_rx_arg1<=5) lc640_write_int(EE_REG_SPEED,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==190)		
				{
				if(modbus_rx_arg1==0 || modbus_rx_arg1==1) lc640_write_int(EE_SMART_SPC,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==191)		
				{
				if(modbus_rx_arg1==1 || modbus_rx_arg1==3) lc640_write_int(EE_NUMPHASE,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==192)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=100) lc640_write_int(EE_TVENTON,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==193)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=100) lc640_write_int(EE_TVENTOFF,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==194)		
				{
				if(modbus_rx_arg1<=2) lc640_write_int(EE_RELEVENTSIGN,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==195)		
				{
				if(modbus_rx_arg1<=2) lc640_write_int(EE_NPN_OUT,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==196)		
				{
				if(modbus_rx_arg1>=100 && modbus_rx_arg1<=2500) lc640_write_int(EE_UONPN,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==197)		
				{
				if(modbus_rx_arg1>=100 && modbus_rx_arg1<=2500) lc640_write_int(EE_UVNPN,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==198)		
				{
				if(modbus_rx_arg1>=10 && modbus_rx_arg1<=60) lc640_write_int(EE_TZNPN,modbus_rx_arg1);
	     		}		
	/*		if(modbus_rx_arg0==199)		
				{
				if(modbus_rx_arg1<=50) lc640_write_int(EE_UBM_AV,modbus_rx_arg1);
	     		} */
				//o_10_s
			if(modbus_rx_arg0==200)		
				{
				if(modbus_rx_arg1==1 ) command_rki=36;
				else if(modbus_rx_arg1==10 ) command_rki=37;
				else if(modbus_rx_arg1==0xFFFF ) command_rki=38; //-1
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

			if(modbus_rx_arg0==210)		
				{
				lc640_write_int(ADR_EE_BAT_C_NOM[0],modbus_rx_arg1);
				}

			if(modbus_rx_arg0==211)		
				{
				lc640_write_int(ADR_EE_BAT_C_NOM[1],modbus_rx_arg1);
				}

#ifdef UKU_FSO
			if(modbus_rx_arg0==251)			//���251  �������� ����� ����(������� �����)		  	
				{
				lc640_write_int(EE_UKUFSO_IBEP_SN,modbus_rx_arg1);			
	     		}
			if(modbus_rx_arg0==252)			//���252  �������� ����� ����(������� �����)	
				{
				lc640_write_int(EE_UKUFSO_IBEP_SN+2,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==253)			//���253  ��� �������� � ������������ ����	
				{
				lc640_write_int(EE_UKUFSO_IBEP_START_DATE_YEAR,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==254)			//���254  ���� � ����� �������� � ������������ ����
				{
				lc640_write_int(EE_UKUFSO_IBEP_START_DATE_MONTH,(modbus_rx_arg1>>8)&0x00ff);
				lc640_write_int(EE_UKUFSO_IBEP_START_DATE_DAY,modbus_rx_arg1&0x00ff);
	     		}
			if(modbus_rx_arg0==281)			//���281  �������� ����� ��ѹ1(������� �����)	  	
				{
				lc640_write_int(EE_UKUFSO_BPS1_SN,modbus_rx_arg1);			
	     		}
			if(modbus_rx_arg0==282)			//���282  �������� ����� ��ѹ1(������� �����)	
				{
				lc640_write_int(EE_UKUFSO_BPS1_SN+2,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==283)			//���283  ��� �������� � ������������ ��ѹ1
				{
				lc640_write_int(EE_UKUFSO_BPS1_START_DATE_YEAR,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==284)			//���284  ���� � ����� �������� � ������������ ��ѹ1
				{
				lc640_write_int(EE_UKUFSO_BPS1_START_DATE_MONTH,(modbus_rx_arg1>>8)&0x00ff);
				lc640_write_int(EE_UKUFSO_BPS1_START_DATE_DAY,modbus_rx_arg1&0x00ff);
	     		}
			if(modbus_rx_arg0==285)			//���285  �������� ����� ��ѹ2(������� �����) 	
				{
				lc640_write_int(EE_UKUFSO_BPS2_SN,modbus_rx_arg1);			
	     		}
			if(modbus_rx_arg0==286)			//���286  �������� ����� ��ѹ2(������� �����)	
				{
				lc640_write_int(EE_UKUFSO_BPS2_SN+2,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==287)			//���287  ��� �������� � ������������ ��ѹ2	
				{
				lc640_write_int(EE_UKUFSO_BPS2_START_DATE_YEAR,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==288)			//���288  ���� � ����� �������� � ������������ ��ѹ2		
				{
				lc640_write_int(EE_UKUFSO_BPS2_START_DATE_MONTH,(modbus_rx_arg1>>8)&0x00ff);
				lc640_write_int(EE_UKUFSO_BPS2_START_DATE_DAY,modbus_rx_arg1&0x00ff);
	     		}
			if(modbus_rx_arg0==289)			//���289  �������� ����� ��ѹ3(������� �����)			  	
				{
				lc640_write_int(EE_UKUFSO_BPS3_SN,modbus_rx_arg1);			
	     		}
			if(modbus_rx_arg0==290)			//���290  �������� ����� ��ѹ3(������� �����)				
				{
				lc640_write_int(EE_UKUFSO_BPS3_SN+2,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==291)			//���291  ��� �������� � ������������ ��ѹ3	
				{
				lc640_write_int(EE_UKUFSO_BPS3_START_DATE_YEAR,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==292)			//���292  ���� � ����� �������� � ������������ ��ѹ3		
				{
				lc640_write_int(EE_UKUFSO_BPS3_START_DATE_MONTH,(modbus_rx_arg1>>8)&0x00ff);
				lc640_write_int(EE_UKUFSO_BPS3_START_DATE_DAY,modbus_rx_arg1&0x00ff);
	     		}
			if(modbus_rx_arg0==293)			//���293  �������� ����� ��ѹ4(������� �����)  	
				{
				lc640_write_int(EE_UKUFSO_BPS4_SN,modbus_rx_arg1);			
	     		}
			if(modbus_rx_arg0==294)			//���294  �������� ����� ��ѹ4(������� �����)		
				{
				lc640_write_int(EE_UKUFSO_BPS4_SN+2,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==295)			//���295  ��� �������� � ������������ ��ѹ4
				{
				lc640_write_int(EE_UKUFSO_BPS4_START_DATE_YEAR,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==296)			//���296  ���� � ����� �������� � ������������ ��ѹ4	
				{
				lc640_write_int(EE_UKUFSO_BPS4_START_DATE_MONTH,(modbus_rx_arg1>>8)&0x00ff);
				lc640_write_int(EE_UKUFSO_BPS4_START_DATE_DAY,modbus_rx_arg1&0x00ff);
	     		}
			if(modbus_rx_arg0==297)			//���297  �������� ����� ��ҹ1(������� �����)	
				{
				lc640_write_int(EE_UKUFSO_BAT1_SN,modbus_rx_arg1);			
	     		}
			if(modbus_rx_arg0==298)			//���298  �������� ����� ��ҹ1(������� �����)				
				{
				lc640_write_int(EE_UKUFSO_BAT1_SN+2,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==299)			//���299  ��� �������� � ������������ ��ҹ1		
				{
				lc640_write_int(EE_UKUFSO_BAT1_START_DATE_YEAR,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==300)			//���300  ���� � ����� �������� � ������������ ��ҹ1	
				{
				lc640_write_int(EE_UKUFSO_BAT1_START_DATE_MONTH,(modbus_rx_arg1>>8)&0x00ff);
				lc640_write_int(EE_UKUFSO_BAT1_START_DATE_DAY,modbus_rx_arg1&0x00ff);
	     		}
			if(modbus_rx_arg0==301)			//���301  �������� ����� ��ҹ2(������� �����)			  	
				{
				lc640_write_int(EE_UKUFSO_BAT2_SN,modbus_rx_arg1);			
	     		}
			if(modbus_rx_arg0==302)			//���302  �������� ����� ��ҹ2(������� �����)			
				{
				lc640_write_int(EE_UKUFSO_BAT2_SN+2,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==303)			//���303  ��� �������� � ������������ ��ҹ2	
				{
				lc640_write_int(EE_UKUFSO_BAT2_START_DATE_YEAR,modbus_rx_arg1);
	     		}
			if(modbus_rx_arg0==304)			//���304  ���� � ����� �������� � ������������ ��ҹ2	
				{
				lc640_write_int(EE_UKUFSO_BAT2_START_DATE_MONTH,(modbus_rx_arg1>>8)&0x00ff);
				lc640_write_int(EE_UKUFSO_BAT2_START_DATE_DAY,modbus_rx_arg1&0x00ff);
	     		}

 /*
modbus_registers[500]=(char)(UKUFSO_IBEP_SN>>8);				
modbus_registers[501]=(char)(UKUFSO_IBEP_SN); 
modbus_registers[502]=(char)(UKUFSO_IBEP_SN>>24);				
modbus_registers[503]=(char)(UKUFSO_IBEP_SN>>16); 
modbus_registers[504]=(char)(UKUFSO_IBEP_START_DATE_YEAR>>8);	
modbus_registers[505]=(char)(UKUFSO_IBEP_START_DATE_YEAR); 
modbus_registers[506]=(char)(UKUFSO_IBEP_START_DATE_MONTH);		
modbus_registers[507]=(char)(UKUFSO_IBEP_START_DATE_DAY); 

modbus_registers[560]=(char)(UKUFSO_BPS1_SN>>8);				
modbus_registers[561]=(char)(UKUFSO_BPS1_SN); 
modbus_registers[562]=(char)(UKUFSO_BPS1_SN>>24);				
modbus_registers[563]=(char)(UKUFSO_BPS1_SN>>16); 
modbus_registers[564]=(char)(UKUFSO_BPS1_START_DATE_YEAR>>8);	
modbus_registers[565]=(char)(UKUFSO_BPS1_START_DATE_YEAR); 
modbus_registers[566]=(char)(UKUFSO_BPS1_START_DATE_MONTH);		
modbus_registers[567]=(char)(UKUFSO_BPS1_START_DATE_DAY);
modbus_registers[568]=(char)(UKUFSO_BPS2_SN>>8);				
modbus_registers[569]=(char)(UKUFSO_BPS2_SN); 
modbus_registers[570]=(char)(UKUFSO_BPS2_SN>>24);				
modbus_registers[571]=(char)(UKUFSO_BPS2_SN>>16); 
modbus_registers[572]=(char)(UKUFSO_BPS2_START_DATE_YEAR>>8);	
modbus_registers[573]=(char)(UKUFSO_BPS2_START_DATE_YEAR); 
modbus_registers[574]=(char)(UKUFSO_BPS2_START_DATE_MONTH);		
modbus_registers[575]=(char)(UKUFSO_BPS2_START_DATE_DAY);
modbus_registers[576]=(char)(UKUFSO_BPS3_SN>>8);				
modbus_registers[577]=(char)(UKUFSO_BPS3_SN); 
modbus_registers[578]=(char)(UKUFSO_BPS3_SN>>24);				
modbus_registers[579]=(char)(UKUFSO_BPS3_SN>>16); 
modbus_registers[580]=(char)(UKUFSO_BPS3_START_DATE_YEAR>>8);	
modbus_registers[581]=(char)(UKUFSO_BPS3_START_DATE_YEAR); 
modbus_registers[582]=(char)(UKUFSO_BPS3_START_DATE_MONTH);		
modbus_registers[583]=(char)(UKUFSO_BPS3_START_DATE_DAY);
modbus_registers[584]=(char)(UKUFSO_BPS4_SN>>8);				
modbus_registers[585]=(char)(UKUFSO_BPS4_SN); 
modbus_registers[586]=(char)(UKUFSO_BPS4_SN>>24);				
modbus_registers[587]=(char)(UKUFSO_BPS4_SN>>16); 
modbus_registers[588]=(char)(UKUFSO_BPS4_START_DATE_YEAR>>8);	
modbus_registers[589]=(char)(UKUFSO_BPS4_START_DATE_YEAR); 
modbus_registers[590]=(char)(UKUFSO_BPS4_START_DATE_MONTH);		
modbus_registers[591]=(char)(UKUFSO_BPS4_START_DATE_DAY);
 


	*/
#endif

			if(modbus_rx_arg0==19)		//���/���� ��������� ����.
				{
	/*			if(modbus_rx_arg1==1)
					{
					if(work_stat!=wsPS)
						{
						work_stat=wsPS;
						time_proc=0;
						time_proc_remain=T_PROC_PS;
						restart_on_PS();
						lc640_write_int(EE_MAIN_MENU_MODE,mmmIN);
						}
					}
				if(modbus_rx_arg1==0)
					{
					if(work_stat==wsPS)
						{
						work_stat=wsOFF;
						restart_off();
						}
					} */
				}
			if(modbus_rx_arg0==20)		//���/���� ��������� ����
				{
/*				if(modbus_rx_arg1==1)
					{
					if(work_stat!=wsGS)
						{
						work_stat=wsGS;
						time_proc=0;
						time_proc_remain=T_PROC_GS;
						lc640_write_int(EE_MAIN_MENU_MODE,mmmIT);
						}
					}
				if(modbus_rx_arg1==0)
					{
					if(work_stat==wsGS)
						{
						work_stat=wsOFF;
						restart_off();
						}
					}*/
				}
			//modbus_hold_register_transmit(MODBUS_ADRESS,modbus_func,modbus_rx_arg0);

			if(modbus_rx_arg0==100)		//�������� ���� ��� ���������� �� �������� ���
				{
				//plazma1000[2]=modbus_rx_arg1;
				if(modbus_rx_arg1&0x4000)
					{
					short tempSSSS;
					
					tempSSSS=modbus_rx_arg1&0x3fff;
					//plazma1000[3]=tempSSSS;
					if((tempSSSS>0)&&(tempSSSS<5))tempSSSS=0;
					else if(tempSSSS>=60)tempSSSS=60;
				//	else tempSSSS=modbus_rx_arg1;
					if(TBAT!=tempSSSS)lc640_write_int(EE_TBAT,tempSSSS);

					main_kb_cnt=(tempSSSS*60)-20;
					}
				else ica_cntrl_hndl=modbus_rx_arg1;
				ica_cntrl_hndl_cnt=200;

				//plazma1000[1]++;
				}
			if(modbus_rx_arg0==101)		//�������� ���� � ������� ���� (����������� �������� � ������� � ���������� ��������)
				{
				ica_your_current==modbus_rx_arg1;
				ica_cntrl_hndl_cnt=200;
				//plazma1000[2]++;
				}

			//modbus_hold_registers_transmit(MODBUS_ADRESS,modbus_func,modbus_rx_arg0,1,MODBUS_RTU_PROT);
				{
			/*if(prot==MODBUS_RTU_PROT)
				{*/
				mem_copy(modbus_tx_buff,modbus_rx_buffer,8);
	
				for (i=0;i<(8);i++)
					{
					putchar0(modbus_tx_buff[i]);
					}

				for (i=0;i<(8);i++)
					{
					putchar_sc16is700(modbus_tx_buff[i]);
					}
			/*	}
			else if(prot==MODBUS_TCP_PROT)
				{
				modbus_tcp_out_ptr=(char*)&modbus_registers[(reg_adr-1)*2];
				}*/
				}
			}
/*			else if(modbus_func==1) {	
				if(modbus_an_buffer[2]==8){
				  for(i_cnt=0;i_cnt<8;i_cnt++) {
				  	for(j_cnt=0;j_cnt<8;j_cnt++){
					   snmp_enmv_data[i_cnt*8+j_cnt]=(modbus_an_buffer[3+i_cnt]>>j_cnt)&0x01;
					}
					enmv_data[i_cnt]=modbus_an_buffer[3+i_cnt];				   
				  }
				  enmv_on=0;
				  enmv_puts_en=1;
				}
			}*/ 

		} 
	else if(modbus_an_buffer[0]==ICA_MODBUS_ADDRESS)
		{
		ica_plazma[3]++;
		if(modbus_func==4)		//������ ������������� ���-�� ���������	������
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
/*
//-----------------------------------------------
void modbus_register_transmit(unsigned char adr,unsigned char func,unsigned short reg_adr)
{
char modbus_registers[150];
//char modbus_tx_buff[50];
unsigned short crc_temp;
char i;


modbus_registers[0]=(char)(load_U/256);					//���1
modbus_registers[1]=(char)(load_U%256);
modbus_registers[2]=(char)(load_I/256);					//���2
modbus_registers[3]=(char)(load_I%256);
modbus_registers[4]=(char)(Ib_ips_termokompensat/256);		//���3
modbus_registers[5]=(char)(Ib_ips_termokompensat%256);
modbus_registers[6]=(char)(t_ext[0]/256);				//���4
modbus_registers[7]=(char)(t_ext[0]%256);
modbus_registers[8]=(char)(net_Ua/256);					//���5
modbus_registers[9]=(char)(net_Ua%256);		 	
modbus_registers[10]=(char)(net_Ub/256);				//���6
modbus_registers[11]=(char)(net_Ub%256);
modbus_registers[12]=(char)(net_Uc/256);				//���7
modbus_registers[13]=(char)(net_Uc%256);
modbus_registers[14]=(char)(net_F3/256);				//���8
modbus_registers[15]=(char)(net_F3%256);
modbus_registers[16]=(char)(load_I/256);				//���9
modbus_registers[17]=(char)(load_I%256);
modbus_registers[18]=(char)(load_I/256);				//���10
modbus_registers[19]=(char)(load_I%256);
modbus_registers[20]=(char)((LPC_RTC->YEAR)/256);			//���11
modbus_registers[21]=(char)((LPC_RTC->YEAR)%256);
modbus_registers[22]=(char)((LPC_RTC->MONTH)/256);		//���12
modbus_registers[23]=(char)((LPC_RTC->MONTH)%256);
modbus_registers[24]=(char)((LPC_RTC->DOM)/256);			//���13
modbus_registers[25]=(char)((LPC_RTC->DOM)%256);
modbus_registers[26]=(char)((LPC_RTC->HOUR)/256);			//���14
modbus_registers[27]=(char)((LPC_RTC->HOUR)%256);
modbus_registers[28]=(char)((LPC_RTC->MIN)/256);			//���15
modbus_registers[29]=(char)((LPC_RTC->MIN)%256);
modbus_registers[30]=(char)((LPC_RTC->SEC)/256);			//���16
modbus_registers[31]=(char)((LPC_RTC->SEC)%256);
modbus_registers[32]=(char)(load_I/256);				//���17
modbus_registers[33]=(char)(load_I%256);
modbus_registers[34]=(char)(load_I/256);				//���18
modbus_registers[35]=(char)(load_I%256);
modbus_registers[36]=(char)(load_I/256);		//���19
modbus_registers[37]=(char)(load_I%256);
modbus_registers[38]=(char)(NUMIST/256);		//���20
modbus_registers[39]=(char)(NUMIST%256);
modbus_registers[40]=(char)(PAR/256);		//���21
modbus_registers[41]=(char)(PAR%256);
modbus_registers[42]=(char)(ZV_ON/256);		//���22
modbus_registers[43]=(char)(ZV_ON%256);
modbus_registers[44]=(char)(TERMOKOMPENS/256);		//���23
modbus_registers[45]=(char)(TERMOKOMPENS%256);
modbus_registers[46]=(char)(UBM_AV/256);		//���24
modbus_registers[47]=(char)(UBM_AV%256);
modbus_registers[48]=(char)(load_I/256);		//���25
modbus_registers[49]=(char)(load_I%256);
modbus_registers[50]=(char)(load_I/256);		//���26
modbus_registers[51]=(char)(load_I%256);
modbus_registers[52]=(char)(load_I/256);		//���27
modbus_registers[53]=(char)(load_I%256);
modbus_registers[54]=(char)(load_I/256);		//���28
modbus_registers[55]=(char)(load_I%256);
modbus_registers[56]=(char)(load_I/256);		//���29
modbus_registers[57]=(char)(load_I%256);
modbus_registers[58]=(char)(TBAT/256);			//���30
modbus_registers[59]=(char)(TBAT%256);
modbus_registers[60]=(char)(UMAX/256);			//���31
modbus_registers[61]=(char)(UMAX%256);
modbus_registers[62]=(char)((UB20-DU)/256);		//���32
modbus_registers[63]=(char)((UB20-DU)%256);
modbus_registers[64]=(char)(UB0/256);			//���33
modbus_registers[65]=(char)(UB0%256);
modbus_registers[66]=(char)(UB20/256);			//���34
modbus_registers[67]=(char)(UB20%256);
modbus_registers[68]=(char)(USIGN/256);		//���35
modbus_registers[69]=(char)(USIGN%256);
modbus_registers[70]=(char)(UMN/256);		//���36
modbus_registers[71]=(char)(UMN%256);
modbus_registers[72]=(char)(U0B/256);		//���37
modbus_registers[73]=(char)(U0B%256);
modbus_registers[74]=(char)(IKB/256);		//���38
modbus_registers[75]=(char)(IKB%256);
modbus_registers[76]=(char)(IZMAX/256);		//���39
modbus_registers[77]=(char)(IZMAX%256);
modbus_registers[78]=(char)(IMAX/256);		//���40
modbus_registers[79]=(char)(IMAX%256);
modbus_registers[80]=(char)(IMIN/256);		//���41
modbus_registers[81]=(char)(IMIN%256);
modbus_registers[82]=(char)(UVZ/256);		//���42
modbus_registers[83]=(char)(UVZ%256);
modbus_registers[84]=(char)(TZAS/256);		//���43
modbus_registers[85]=(char)(TZAS%256);
modbus_registers[86]=(char)(TMAX/256);		//���44
modbus_registers[87]=(char)(TMAX%256);
modbus_registers[88]=(char)(TSIGN/256);		//���45
modbus_registers[89]=(char)(TSIGN%256);
modbus_registers[90]=(char)(TBATMAX/256);		//���46
modbus_registers[91]=(char)(TBATMAX%256);
modbus_registers[92]=(char)(TBATSIGN/256);		//���47
modbus_registers[93]=(char)(TBATSIGN%256);
modbus_registers[94]=(char)(load_I/256);		//���48
modbus_registers[95]=(char)(load_I%256);
modbus_registers[96]=(char)(load_I/256);		//���49
modbus_registers[97]=(char)(load_I%256);
modbus_registers[98]=(char)(load_I/256);		//���50
modbus_registers[99]=(char)(load_I%256);

modbus_tx_buff[0]=adr;
modbus_tx_buff[1]=func;
modbus_tx_buff[2]=(char)(reg_adr/256);
modbus_tx_buff[3]=(char)(reg_adr%256);
//modbus_tx_buff[4]=(char)(reg_quantity/256);
//modbus_tx_buff[5]=(char)(reg_quantity%256);


mem_copy((char*)&modbus_tx_buff[4],(char*)&modbus_registers[(reg_adr-1)*2],2);

crc_temp=CRC16_2(modbus_tx_buff,6);

modbus_tx_buff[6]=crc_temp%256;
modbus_tx_buff[7]=crc_temp/256;

for (i=0;i<8;i++)
	{
	putchar0(modbus_tx_buff[i]);
	}
for (i=0;i<8;i++)
	{
	putchar_sc16is700(modbus_tx_buff[i]);
	}
}*/
/*//-----------------------------------------------
void modbus_registers_transmit(unsigned char adr,unsigned char func,unsigned short reg_adr,unsigned short reg_quantity)
{
char modbus_registers[100];
//char modbus_tx_buff[100];
unsigned short crc_temp;
char i;


modbus_registers[0]=(char)(load_U/256);					//���1
modbus_registers[1]=(char)(load_U%256);
modbus_registers[2]=(char)(load_I/256);					//���2
modbus_registers[3]=(char)(load_I%256);
modbus_registers[4]=(char)(Ib_ips_termokompensat/256);		//���3
modbus_registers[5]=(char)(Ib_ips_termokompensat%256);
modbus_registers[6]=(char)(t_ext[0]/256);				//���4
modbus_registers[7]=(char)(t_ext[0]%256);
modbus_registers[8]=(char)(net_Ua/256);					//���5
modbus_registers[9]=(char)(net_Ua%256);		 	
modbus_registers[10]=(char)(net_Ub/256);				//���6
modbus_registers[11]=(char)(net_Ub%256);
modbus_registers[12]=(char)(net_Uc/256);				//���7
modbus_registers[13]=(char)(net_Uc%256);
modbus_registers[14]=(char)(net_F3/256);				//���8
modbus_registers[15]=(char)(net_F3%256);
modbus_registers[16]=(char)(load_I/256);				//���9
modbus_registers[17]=(char)(load_I%256);
modbus_registers[18]=(char)(load_I/256);				//���10
modbus_registers[19]=(char)(load_I%256);
modbus_registers[20]=(char)((LPC_RTC->YEAR)/256);			//���11
modbus_registers[21]=(char)((LPC_RTC->YEAR)%256);
modbus_registers[22]=(char)((LPC_RTC->MONTH)/256);		//���12
modbus_registers[23]=(char)((LPC_RTC->MONTH)%256);
modbus_registers[24]=(char)((LPC_RTC->DOM)/256);			//���13
modbus_registers[25]=(char)((LPC_RTC->DOM)%256);
modbus_registers[26]=(char)((LPC_RTC->HOUR)/256);			//���14
modbus_registers[27]=(char)((LPC_RTC->HOUR)%256);
modbus_registers[28]=(char)((LPC_RTC->MIN)/256);			//���15
modbus_registers[29]=(char)((LPC_RTC->MIN)%256);
modbus_registers[30]=(char)((LPC_RTC->SEC)/256);			//���16
modbus_registers[31]=(char)((LPC_RTC->SEC)%256);
modbus_registers[32]=(char)(load_I/256);				//���17
modbus_registers[33]=(char)(load_I%256);
modbus_registers[34]=(char)(load_I/256);				//���18
modbus_registers[35]=(char)(load_I%256);
modbus_registers[36]=(char)(load_I/256);		//���19
modbus_registers[37]=(char)(load_I%256);
modbus_registers[38]=(char)(NUMIST/256);		//���20
modbus_registers[39]=(char)(NUMIST%256);
modbus_registers[40]=(char)(PAR/256);		//���21
modbus_registers[41]=(char)(PAR%256);
modbus_registers[42]=(char)(ZV_ON/256);		//���22
modbus_registers[43]=(char)(ZV_ON%256);
modbus_registers[44]=(char)(TERMOKOMPENS/256);		//���23
modbus_registers[45]=(char)(TERMOKOMPENS%256);
modbus_registers[46]=(char)(UBM_AV/256);		//���24
modbus_registers[47]=(char)(UBM_AV%256);
modbus_registers[48]=(char)(load_I/256);		//���25
modbus_registers[49]=(char)(load_I%256);
modbus_registers[50]=(char)(load_I/256);		//���26
modbus_registers[51]=(char)(load_I%256);
modbus_registers[52]=(char)(load_I/256);		//���27
modbus_registers[53]=(char)(load_I%256);
modbus_registers[54]=(char)(load_I/256);		//���28
modbus_registers[55]=(char)(load_I%256);
modbus_registers[56]=(char)(load_I/256);		//���29
modbus_registers[57]=(char)(load_I%256);
modbus_registers[58]=(char)(TBAT/256);			//���30
modbus_registers[59]=(char)(TBAT%256);
modbus_registers[60]=(char)(UMAX/256);			//���31
modbus_registers[61]=(char)(UMAX%256);
modbus_registers[62]=(char)((UB20-DU)/256);		//���32
modbus_registers[63]=(char)((UB20-DU)%256);
modbus_registers[64]=(char)(UB0/256);			//���33
modbus_registers[65]=(char)(UB0%256);
modbus_registers[66]=(char)(UB20/256);			//���34
modbus_registers[67]=(char)(UB20%256);
modbus_registers[68]=(char)(USIGN/256);		//���35
modbus_registers[69]=(char)(USIGN%256);
modbus_registers[70]=(char)(UMN/256);		//���36
modbus_registers[71]=(char)(UMN%256);
modbus_registers[72]=(char)(U0B/256);		//���37
modbus_registers[73]=(char)(U0B%256);
modbus_registers[74]=(char)(IKB/256);		//���38
modbus_registers[75]=(char)(IKB%256);
modbus_registers[76]=(char)(IZMAX/256);		//���39
modbus_registers[77]=(char)(IZMAX%256);
modbus_registers[78]=(char)(IMAX/256);		//���40
modbus_registers[79]=(char)(IMAX%256);
modbus_registers[80]=(char)(IMIN/256);		//���41
modbus_registers[81]=(char)(IMIN%256);
modbus_registers[82]=(char)(UVZ/256);		//���42
modbus_registers[83]=(char)(UVZ%256);
modbus_registers[84]=(char)(TZAS/256);		//���43
modbus_registers[85]=(char)(TZAS%256);
modbus_registers[86]=(char)(TMAX/256);		//���44
modbus_registers[87]=(char)(TMAX%256);
modbus_registers[88]=(char)(TSIGN/256);		//���45
modbus_registers[89]=(char)(TSIGN%256);
modbus_registers[90]=(char)(TBATMAX/256);		//���46
modbus_registers[91]=(char)(TBATMAX%256);
modbus_registers[92]=(char)(TBATSIGN/256);		//���47
modbus_registers[93]=(char)(TBATSIGN%256);
modbus_registers[94]=(char)(load_I/256);		//���48
modbus_registers[95]=(char)(load_I%256);
modbus_registers[96]=(char)(load_I/256);		//���49
modbus_registers[97]=(char)(load_I%256);
modbus_registers[98]=(char)(load_I/256);		//���50
modbus_registers[99]=(char)(load_I%256);




modbus_tx_buff[0]=adr;
modbus_tx_buff[1]=func;
//modbus_tx_buff[2]=(char)(reg_adr/256);
//modbus_tx_buff[3]=(char)(reg_adr%256);
modbus_tx_buff[2]=(char)(reg_quantity*2);
//modbus_tx_buff[5]=(char)(reg_quantity%256);


mem_copy((char*)&modbus_tx_buff[3],(char*)&modbus_registers[(reg_adr-1)*2],reg_quantity*2);

crc_temp=CRC16_2(modbus_tx_buff,(reg_quantity*2)+3);

modbus_tx_buff[3+(reg_quantity*2)]=crc_temp%256;
modbus_tx_buff[4+(reg_quantity*2)]=crc_temp/256;

//int2lcdyx(reg_quantity,0,10,0);

for (i=0;i<15;i++)
	{
	putchar0(modbus_tx_buff[i]);
	} 
for (i=0;i<15;i++)
	{
	putchar_sc16is700(modbus_tx_buff[i]);
	}
}*/
/*
//-----------------------------------------------
void modbus_hold_register_transmit(unsigned char adr,unsigned char func,unsigned short reg_adr)
{
char modbus_registers[150];
//char modbus_tx_buff[150];
unsigned short crc_temp;
char i;

modbus_registers[20]=(char)((LPC_RTC->YEAR)/256);			//���11  �����, ���
modbus_registers[21]=(char)((LPC_RTC->YEAR)%256);
modbus_registers[22]=(char)((LPC_RTC->MONTH)/256);		//���12  �����, �����
modbus_registers[23]=(char)((LPC_RTC->MONTH)%256);
modbus_registers[24]=(char)((LPC_RTC->DOM)/256);			//���13  �����, ���� ������
modbus_registers[25]=(char)((LPC_RTC->DOM)%256);
modbus_registers[26]=(char)((LPC_RTC->HOUR)/256);			//���14  �����, ���
modbus_registers[27]=(char)((LPC_RTC->HOUR)%256);
modbus_registers[28]=(char)((LPC_RTC->MIN)/256);			//���15  �����, ������
modbus_registers[29]=(char)((LPC_RTC->MIN)%256);
modbus_registers[30]=(char)((LPC_RTC->SEC)/256);			//���16  �����, �������
modbus_registers[31]=(char)((LPC_RTC->SEC)%256);
modbus_registers[38]=(char)(NUMIST/256);				//���20  ���������� ������������ � ���������
modbus_registers[39]=(char)(NUMIST%256);
modbus_registers[40]=(char)(PAR/256);					//���21  ������������ ������ ������������ ���./����.
modbus_registers[41]=(char)(PAR%256);
modbus_registers[42]=(char)(ZV_ON/256);					//���22  �������� ��������� ������������ ���./����.
modbus_registers[43]=(char)(ZV_ON%256);
modbus_registers[46]=(char)(UBM_AV/256);				//���24  ��������� ������� ���������� ���������� ������� ����� �������, %
modbus_registers[47]=(char)(UBM_AV%256);
modbus_registers[58]=(char)(TBAT/256);					//���30  ������ �������� ���� �������, �����.
modbus_registers[59]=(char)(TBAT%256);
modbus_registers[60]=(char)(UMAX/256);					//���31  ������������ (���������) ���������� ������������, 0.1�
modbus_registers[61]=(char)(UMAX%256);
modbus_registers[62]=(char)((UB20-DU)/256);				//���32  ����������� (���������) ���������� ������������, 0.1�
modbus_registers[63]=(char)((UB20-DU)%256);
modbus_registers[64]=(char)(UB0/256);					//���33  ���������� ���������� ������� ��� 0��, 0.1�
modbus_registers[65]=(char)(UB0%256);
modbus_registers[66]=(char)(UB20/256);					//���34  ���������� ���������� ������� ��� 20��, 0.1�
modbus_registers[67]=(char)(UB20%256);
modbus_registers[68]=(char)(USIGN/256);					//���35  ����������� (����������) ���������� �������, 1�
modbus_registers[69]=(char)(USIGN%256);
modbus_registers[70]=(char)(UMN/256);					//���36  ����������� (���������) ���������� �������� ����, 1�
modbus_registers[71]=(char)(UMN%256);
modbus_registers[72]=(char)(U0B/256);					//���37  ������� ���������� ��� ����������� ��������, 0.1�
modbus_registers[73]=(char)(U0B%256);
modbus_registers[74]=(char)(IKB/256);					//���38  ��� �������� ������� �������, 0.1�
modbus_registers[75]=(char)(IKB%256);
modbus_registers[76]=(char)(IZMAX/256);					//���39  ��� ������ ������� ������������, 0.1�
modbus_registers[77]=(char)(IZMAX%256);
modbus_registers[78]=(char)(IMAX/256);					//���40  ��� ������������ �� ������� ���-�� ������������, 0.1�
modbus_registers[79]=(char)(IMAX%256);
modbus_registers[80]=(char)(IMIN/256);					//���41  ��� ������������ �� ������� ���-�� ������������, 0.1�
modbus_registers[81]=(char)(IMIN%256);
modbus_registers[82]=(char)(UVZ/256);					//���42  ���������� �������������� ������, 0.1�
modbus_registers[83]=(char)(UVZ%256);
modbus_registers[84]=(char)(TZAS/256);					//���43  ����� �������� ��������� ������������, ���
modbus_registers[85]=(char)(TZAS%256);
modbus_registers[86]=(char)(TMAX/256);					//���44  ����������� ������������ ���������, 1��
modbus_registers[87]=(char)(TMAX%256);
modbus_registers[88]=(char)(TSIGN/256);					//���45  ����������� ������������ ����������, 1��
modbus_registers[89]=(char)(TSIGN%256);
modbus_registers[90]=(char)(TBATMAX/256);				//���46  ����������� ������� ���������, 1��
modbus_registers[91]=(char)(TBATMAX%256);
modbus_registers[92]=(char)(TBATSIGN/256);				//���47  ����������� ������� ����������, 1��
modbus_registers[93]=(char)(TBATSIGN%256);
modbus_registers[94]=(char)(speedChrgCurr/256);					//���48  ��� ����������� ������, 0.1�
modbus_registers[95]=(char)(speedChrgCurr%256);
modbus_registers[96]=(char)(speedChrgVolt/256);				//���49	 ���������� ����������� ������, 0.1� 
modbus_registers[97]=(char)(speedChrgVolt%256);
modbus_registers[98]=(char)(speedChrgTimeInHour/256);				//���50	 ����� ����������� ������, 1�
modbus_registers[99]=(char)(speedChrgTimeInHour%256);


modbus_tx_buff[0]=adr;
modbus_tx_buff[1]=func;
modbus_tx_buff[2]=(char)(reg_adr/256);
modbus_tx_buff[3]=(char)(reg_adr%256);

mem_copy((char*)&modbus_tx_buff[4],(char*)&modbus_registers[(reg_adr-1)*2],2);

crc_temp=CRC16_2(modbus_tx_buff,6);

modbus_tx_buff[6]=crc_temp%256;
modbus_tx_buff[7]=crc_temp/256;

for (i=0;i<8;i++)
	{
	putchar0(modbus_tx_buff[i]);
	}
for (i=0;i<8;i++)
	{
	putchar_sc16is700(modbus_tx_buff[i]);
	}
}
*/

//-----------------------------------------------
void modbus_hold_registers_transmit(unsigned char adr,unsigned char func,unsigned short reg_adr,unsigned short reg_quantity, char prot)
{
signed char modbus_registers[800];	//o_10
//char modbus_tx_buff[150];
unsigned short crc_temp;
char i;

modbus_registers[20]=(char)((LPC_RTC->YEAR)>>8);			//���11  �����, ���
modbus_registers[21]=(char)((LPC_RTC->YEAR));
modbus_registers[22]=(char)((LPC_RTC->MONTH)>>8);		//���12  �����, �����
modbus_registers[23]=(char)((LPC_RTC->MONTH));
modbus_registers[24]=(char)((LPC_RTC->DOM)>>8);			//���13  �����, ���� ������
modbus_registers[25]=(char)((LPC_RTC->DOM));
modbus_registers[26]=(char)((LPC_RTC->HOUR)>>8);			//���14  �����, ���
modbus_registers[27]=(char)((LPC_RTC->HOUR));
modbus_registers[28]=(char)((LPC_RTC->MIN)>>8);			//���15  �����, ������
modbus_registers[29]=(char)((LPC_RTC->MIN));
modbus_registers[30]=(char)((LPC_RTC->SEC)>>8);			//���16  �����, �������
modbus_registers[31]=(char)((LPC_RTC->SEC));
modbus_registers[38]=(char)(NUMIST>>8);				//���20  ���������� ������������ � ���������
modbus_registers[39]=(char)(NUMIST);
modbus_registers[40]=(char)(PAR>>8);					//���21  ������������ ������ ������������ ���./����.
modbus_registers[41]=(char)(PAR);
modbus_registers[42]=(char)(ZV_ON>>8);					//���22  �������� ��������� ������������ ���./����.
modbus_registers[43]=(char)(ZV_ON);
modbus_registers[46]=(char)(UBM_AV>>8);				//���24  ��������� ������� ���������� ���������� ������� ����� �������, %
modbus_registers[47]=(char)(UBM_AV);
modbus_registers[58]=(char)(TBAT>>8);					//���30  ������ �������� ���� �������, �����.
modbus_registers[59]=(char)(TBAT);
modbus_registers[60]=(char)(UMAX>>8);					//���31  ������������ (���������) ���������� ������������, 0.1�
modbus_registers[61]=(char)(UMAX);
modbus_registers[62]=(char)((UB20-DU)>>8);				//���32  ����������� (���������) ���������� ������������, 0.1�
modbus_registers[63]=(char)((UB20-DU));
modbus_registers[64]=(char)(UB0>>8);					//���33  ���������� ���������� ������� ��� 0��, 0.1�
modbus_registers[65]=(char)(UB0);
modbus_registers[66]=(char)(UB20>>8);					//���34  ���������� ���������� ������� ��� 20��, 0.1�
modbus_registers[67]=(char)(UB20);
modbus_registers[68]=(char)(USIGN>>8);					//���35  ����������� (����������) ���������� �������, 1�
modbus_registers[69]=(char)(USIGN);
modbus_registers[70]=(char)(UMN>>8);					//���36  ����������� (���������) ���������� �������� ����, 1�
modbus_registers[71]=(char)(UMN);
modbus_registers[72]=(char)(U0B>>8);					//���37  ������� ���������� ��� ����������� ��������, 0.1�
modbus_registers[73]=(char)(U0B);
modbus_registers[74]=(char)(IKB>>8);					//���38  ��� �������� ������� �������, 0.1�
modbus_registers[75]=(char)(IKB);
modbus_registers[76]=(char)(IZMAX>>8);					//���39  ��� ������ ������� ������������, 0.1�
modbus_registers[77]=(char)(IZMAX);
modbus_registers[78]=(char)(IMAX>>8);					//���40  ��� ������������ �� ������� ���-�� ������������, 0.1�
modbus_registers[79]=(char)(IMAX);
modbus_registers[80]=(char)(IMIN>>8);					//���41  ��� ������������ �� ������� ���-�� ������������, 0.1�
modbus_registers[81]=(char)(IMIN);
modbus_registers[82]=(char)(UVZ>>8);					//���42  ���������� �������������� ������, 0.1�
modbus_registers[83]=(char)(UVZ);
modbus_registers[84]=(char)(TZAS>>8);					//���43  ����� �������� ��������� ������������, ���
modbus_registers[85]=(char)(TZAS);
modbus_registers[86]=(char)(TMAX>>8);					//���44  ����������� ������������ ���������, 1��
modbus_registers[87]=(char)(TMAX);
modbus_registers[88]=(char)(TSIGN>>8);					//���45  ����������� ������������ ����������, 1��
modbus_registers[89]=(char)(TSIGN);
modbus_registers[90]=(char)(TBATMAX>>8);				//���46  ����������� ������� ���������, 1��
modbus_registers[91]=(char)(TBATMAX);
modbus_registers[92]=(char)(TBATSIGN>>8);				//���47  ����������� ������� ����������, 1��
modbus_registers[93]=(char)(TBATSIGN);
modbus_registers[94]=(char)(speedChrgCurr>>8);			//���48  ��� ����������� ������, 0.1�
modbus_registers[95]=(char)(speedChrgCurr);
modbus_registers[96]=(char)(speedChrgVolt>>8);			//���49	 ���������� ����������� ������, 0.1� 
modbus_registers[97]=(char)(speedChrgVolt);
modbus_registers[98]=(char)(speedChrgTimeInHour>>8);	//���50	 ����� ����������� ������, 1�
modbus_registers[99]=(char)(speedChrgTimeInHour);
modbus_registers[100]=(char)(U_OUT_KONTR_MAX>>8);		//���51	 �������� ��������� ����������, Umax, 0.1�
modbus_registers[101]=(char)(U_OUT_KONTR_MAX);
modbus_registers[102]=(char)(U_OUT_KONTR_MIN>>8);		//���52	 �������� ��������� ����������, Umin, 0.1�
modbus_registers[103]=(char)(U_OUT_KONTR_MIN);
modbus_registers[104]=(char)(U_OUT_KONTR_DELAY>>8);		//���53	 �������� ��������� ����������, T��������, 1���.
modbus_registers[105]=(char)(U_OUT_KONTR_DELAY);
modbus_registers[106]=(char)(UB0>>8);					//���54	 ��������� ��������� ���������� ��� ��� ��� �������(����-�������)
modbus_registers[107]=(char)(UB0);
modbus_registers[108]=(char)(UMAXN>>8);					//���55  ������������ (���������) ���������� �������� ����, 1�
modbus_registers[109]=(char)(UMAXN);

modbus_registers[110]=0;								//���56  ������ ������������� �������, 0-����, 1-1�, 2-1���., 3-1-���.
modbus_registers[111]=(char)(SNTP_ENABLE);				
modbus_registers[112]=(char)(SNTP_GMT>>8);				//���57  ������� ����, �� -12 �� +13
modbus_registers[113]=(char)(SNTP_GMT);					
modbus_registers[114]=0;								//���58  1 ����� IP
modbus_registers[115]=(char)(lc640_read_int(EE_SNTP_IP1));					
modbus_registers[116]=0;								//���59  2 ����� IP
modbus_registers[117]=(char)(lc640_read_int(EE_SNTP_IP2));					
modbus_registers[118]=0;								//���60  3 ����� IP
modbus_registers[119]=(char)(lc640_read_int(EE_SNTP_IP3));					
modbus_registers[120]=0;								//���61  4 ����� IP
modbus_registers[121]=(char)(lc640_read_int(EE_SNTP_IP4));
modbus_registers[122]=0;								//���62  ���������� ���
modbus_registers[123]=(char)(NUMBAT);
modbus_registers[124]=0;								//���63  ���������� �������� �����������
modbus_registers[125]=(char)(NUMDT);	
modbus_registers[126]=0;								//���64  ���������� ��������� ��� 0,2,4
modbus_registers[127]=(char)(NUMMAKB);	
modbus_registers[128]=0;								//���65  ���������� ��
modbus_registers[129]=(char)(NUMSK);
modbus_registers[130]=0;								//���66  ���������� ���
modbus_registers[131]=(char)(num_rki);	
modbus_registers[132]=0;								//���67  ���������� ������� ������
modbus_registers[133]=(char)(num_net_in);
modbus_registers[134]=0;								//���68  ���������� ���. ����
modbus_registers[135]=(char)(NUMBDR);
modbus_registers[136]=0;								//���69  ���������� ����
modbus_registers[137]=(char)(NUMENMV);
#ifdef UKU_220_IPS_TERMOKOMPENSAT
modbus_registers[138]=(char)(BAT_C_POINT_NUM_ELEM>>8);	//���70  ���������� 2� ��������� ���  0-200
modbus_registers[139]=(char)(BAT_C_POINT_NUM_ELEM);
modbus_registers[140]=(char)(BAT_C_POINT_20>>8);		//���71  ������� �20   0,1�*�	10-25000
modbus_registers[141]=(char)(BAT_C_POINT_20);
modbus_registers[142]=(char)(BAT_C_POINT_10>>8);		//���72  ������� �10   0,1�*�	10-21000
modbus_registers[143]=(char)(BAT_C_POINT_10);
modbus_registers[144]=(char)(BAT_C_POINT_5>>8);			//���73  ������� �5	   0,1�*�	10-20000
modbus_registers[145]=(char)(BAT_C_POINT_5);
modbus_registers[146]=(char)(BAT_C_POINT_3>>8);			//���74  ������� �3	   0,1�*�	10-18000
modbus_registers[147]=(char)(BAT_C_POINT_3);
modbus_registers[148]=(char)(BAT_C_POINT_1>>8);			//���75  ������� �1	   0,1�*�	10-16000
modbus_registers[149]=(char)(BAT_C_POINT_1);
modbus_registers[150]=(char)(BAT_C_POINT_1_2>>8);		//���76  ������� �1/2    0,1�*�	   10-13000
modbus_registers[151]=(char)(BAT_C_POINT_1_2);
modbus_registers[152]=(char)(BAT_C_POINT_1_6>>8);		//���77  ������� �1/6    0,1�*�	   10-8000
modbus_registers[153]=(char)(BAT_C_POINT_1_6);
modbus_registers[154]=(char)(BAT_U_END_20>>8);			//���78  ������� U20   0,1�	 1.0-1000.0
modbus_registers[155]=(char)(BAT_U_END_20);
modbus_registers[156]=(char)(BAT_U_END_10>>8);			//���79  ������� U10   0,1�	  1.0-1000.0
modbus_registers[157]=(char)(BAT_U_END_10);
modbus_registers[158]=(char)(BAT_U_END_5>>8);			//���80  ������� U5	  0,1�	  1.0-1000.0
modbus_registers[159]=(char)(BAT_U_END_5);
modbus_registers[160]=(char)(BAT_U_END_3>>8);			//���81  ������� U3	  0,1�	  1.0-1000.0
modbus_registers[161]=(char)(BAT_U_END_3);
modbus_registers[162]=(char)(BAT_U_END_1>>8);			//���82  ������� U1	  0,1�	  1.0-1000.0
modbus_registers[163]=(char)(BAT_U_END_1);
modbus_registers[164]=(char)(BAT_U_END_1_2>>8);			//���83  ������� U1/2   0,1�  1.0-1000.0
modbus_registers[165]=(char)(BAT_U_END_1_2);
modbus_registers[166]=(char)(BAT_U_END_1_6>>8);			//���84  ������� U1/6  0,1�	  1.0-1000.0
modbus_registers[167]=(char)(BAT_U_END_1_6);
modbus_registers[168]=(char)(BAT_K_OLD>>8);				//���85  ����������� �������� ��� , 0,01, ����=1,00 min=0.10
modbus_registers[169]=(char)(BAT_K_OLD);
#endif
modbus_registers[170]=(char)(UVENTOFF>>8);			   	//���86  U����. ����������� ���, 1�	 15-250
modbus_registers[171]=(char)(UVENTOFF);
modbus_registers[172]=(char)(IMAX_VZ>>8);				//���87  ����. ��� ������. ������ , 0,1�   10-2000
modbus_registers[173]=(char)(IMAX_VZ);
modbus_registers[174]=(char)(VZ_HR>>8);					//���88  ����� ������. ������, ���� =0,  �� 0,5 ����. max=72�, 1�
modbus_registers[175]=(char)(VZ_HR);
modbus_registers[176]=0;								//���89  ������������ �� ����������� ��1, 1-���
modbus_registers[177]=(char)(VZ_CH_VENT_BLOK);
modbus_registers[178]=0;								//���90  �����. ��, 1-���
modbus_registers[179]=(char)(speedChrgAvtEn);
modbus_registers[180]=(char)(speedChrgDU>>8);			//���91  dU ��, 1�	  1-100
modbus_registers[181]=(char)(speedChrgDU);
modbus_registers[182]=0;								//���92  ������������ ��  0-����, 2-CK2
modbus_registers[183]=(char)(speedChrgBlckSrc);
modbus_registers[184]=0;								//���93  ������ ������������ ��  1-�����, 0-�������.
modbus_registers[185]=(char)(speedChrgBlckLog);
modbus_registers[186]=0;								//���94  ������������ �� ����������� ��1 1-���, 0-����.
modbus_registers[187]=(char)(SP_CH_VENT_BLOK);
modbus_registers[188]=(char)(UZ_U>>8);					//���95  ���������� �������������� ������, 0,1�	  U�20*10-2600
modbus_registers[189]=(char)(UZ_U);
modbus_registers[190]=(char)(UZ_IMAX>>8);				//���96  ��� �������������� ������, 0,1�  10-10000
modbus_registers[191]=(char)(UZ_IMAX);
modbus_registers[192]=(char)(UZ_T>>8);					//���97  ����� ������ �������������� ������, 1�	 1-72
modbus_registers[193]=(char)(UZ_T);
modbus_registers[194]=(char)(FZ_U1>>8);					//���98  ���������� ������������ ������ 1, 0,1�	  U�20*10-3000
modbus_registers[195]=(char)(FZ_U1);
modbus_registers[196]=(char)(FZ_IMAX1>>8);				//���99  ��� ������������ ������ 1, 0,1�   10-1000
modbus_registers[197]=(char)(FZ_IMAX1);
// ����� 2 �������� ��� ������������ �����
modbus_registers[202]=(char)(FZ_T1>>8);					//���102  ����� ������ ������������ ������ 1, 1� 1-10
modbus_registers[203]=(char)(FZ_T1);
modbus_registers[204]=(char)(FZ_ISW12>>8);				//���103  ��� ������������ ��, 0,1�	  10-1000
modbus_registers[205]=(char)(FZ_ISW12);
modbus_registers[206]=(char)(FZ_U2>>8);					//���104  ���������� ������������ ������ 2, 0,1�   U�20*10-3000
modbus_registers[207]=(char)(FZ_U2);
modbus_registers[208]=(char)(FZ_IMAX2>>8);				//���105  ��� ������������ ������ 2, 0,1�	10-1000
modbus_registers[209]=(char)(FZ_IMAX2);
modbus_registers[210]=(char)(FZ_T2>>8);					//���106  ����� ������ ������������ ������ 2, 1�  1-10
modbus_registers[211]=(char)(FZ_T2);
modbus_registers[212]=0;								//���107  ���������� ���������� ������� 0-����., 1-�����.
modbus_registers[213]=(char)(AV_OFF_AVT);
modbus_registers[214]=0;								//���108  ��� 1� ������� 0-����., 1-���.
modbus_registers[215]=(char)(APV_ON1);
modbus_registers[216]=0;								//���109  ��� 2� ������� 0-����., 1-���.
modbus_registers[217]=(char)(APV_ON2);
modbus_registers[218]=0;								//���110  ��� 2� ������� ������, 1� , 1-24�
modbus_registers[219]=(char)(APV_ON2_TIME);
if(SK_SIGN[0]==0){
	modbus_registers[220]=0;								//���111  ��1 ������: 1-�������, 0-���������
	modbus_registers[221]=1;
}else{
	modbus_registers[220]=0;								//���111  ��1 ������: 1-�������, 0-���������
	modbus_registers[221]=0;
}
if(SK_ZVUK_EN[0]==0){
	modbus_registers[222]=0;								//���112  ��1 ������ ����: 0-����, 1-���
	modbus_registers[223]=1;
}else{
	modbus_registers[222]=0;								//���112  ��1 ������ ����: 0-����, 1-���
	modbus_registers[223]=0;
}
if(SK_LCD_EN[0]==0){
	modbus_registers[224]=0;								//���113  ��1 ������ ���: 0-����, 1-���
	modbus_registers[225]=1;
}else{
	modbus_registers[224]=0;								//���113  ��1 ������ ���: 0-����, 1-���
	modbus_registers[225]=0;
}
if(SK_SIGN[1]==0){
	modbus_registers[226]=0;								//���114  ��2 ������: 1-�������, 0-���������
	modbus_registers[227]=1;
}else{
	modbus_registers[226]=0;								//���114  ��2 ������: 1-�������, 0-���������
	modbus_registers[227]=0;
}
if(SK_ZVUK_EN[1]==0){
	modbus_registers[228]=0;								//���115  ��2 ������ ����: 0-����, 1-���
	modbus_registers[229]=1;
}else{
	modbus_registers[228]=0;								//���115  ��2 ������ ����: 0-����, 1-���
	modbus_registers[229]=0;										   
}
if(SK_LCD_EN[1]==0){
	modbus_registers[230]=0;								//���116  ��2 ������ ���: 0-����, 1-���
	modbus_registers[231]=1;
}else{
	modbus_registers[230]=0;								//���116  ��2 ������ ���: 0-����, 1-���
	modbus_registers[231]=0;
}

if(SK_SIGN[2]==0){
	modbus_registers[232]=0;								//���117  ��3 ������: 1-�������, 0-���������
	modbus_registers[233]=1;
}else{
	modbus_registers[232]=0;								//���117  ��3 ������: 1-�������, 0-���������
	modbus_registers[233]=0;
}
if(SK_ZVUK_EN[2]==0){
	modbus_registers[234]=0;								//���118  ��3 ������ ����: 0-����, 1-���
	modbus_registers[235]=1;
}else{
	modbus_registers[234]=0;								//���118  ��3 ������ ����: 0-����, 1-���
	modbus_registers[235]=0;
}
if(SK_LCD_EN[2]==0){
	modbus_registers[236]=0;								//���119  ��3 ������ ���: 0-����, 1-���
	modbus_registers[237]=1;
}else{
	modbus_registers[236]=0;								//���119  ��3 ������ ���: 0-����, 1-���
	modbus_registers[237]=0;
}
if(SK_SIGN[3]==0){
	modbus_registers[238]=0;								//���120  ��4 ������: 1-�������, 0-���������
	modbus_registers[239]=1;
}else{
	modbus_registers[238]=0;								//���120  ��4 ������: 1-�������, 0-���������
	modbus_registers[239]=0;
}
if(SK_ZVUK_EN[3]==0){
	modbus_registers[240]=0;								//���121  ��4 ������ ����: 0-����, 1-���
	modbus_registers[241]=1;
}else{
	modbus_registers[240]=0;								//���121  ��4 ������ ����: 0-����, 1-���
	modbus_registers[241]=0;										   
}
if(SK_LCD_EN[3]==0){
	modbus_registers[242]=0;								//���122  ��4 ������ ���: 0-����, 1-���
	modbus_registers[243]=1;
}else{
	modbus_registers[242]=0;								//���122  ��4 ������ ���: 0-����, 1-���
	modbus_registers[243]=0;
}
modbus_registers[244]=0;									//���123  ���������������� 0-����, 1-���
modbus_registers[245]=(char)(TERMOKOMPENS);
modbus_registers[246]=(char)(FORVARDBPSCHHOUR>>8);			//���124  ����� ������� ���, 1�, 0-500�, 0-����
modbus_registers[247]=(char)(FORVARDBPSCHHOUR);
modbus_registers[248]=0;									//���125  ���������� ��� ���� 0-�� ��� ��, 1-������.���
modbus_registers[249]=(char)(DOP_RELE_FUNC);
modbus_registers[250]=0;									//���126  ���������� ���, 0-����, 1-��1, 2-��2
modbus_registers[251]=(char)(ipsBlckSrc);
modbus_registers[252]=0;									//���127  ������ ������������, 0-�������. 1-�����.
modbus_registers[253]=(char)(ipsBlckLog);
modbus_registers[254]=0;									//���128  ����� ������	1-100
modbus_registers[255]=(char)(MODBUS_ADRESS);		
modbus_registers[256]=(char)(MODBUS_BAUDRATE>>8);			//���129  ��������/10 
modbus_registers[257]=(char)(MODBUS_BAUDRATE);
modbus_registers[258]=0;									//���130  ��������, 0-���. 1-����
modbus_registers[259]=(char)(ETH_IS_ON);
modbus_registers[260]=0;									//���131  �������� DHCP, 0-���. 1-����.
modbus_registers[261]=(char)(ETH_DHCP_ON);
modbus_registers[262]=0;									//���132  �������� IP ����� 1
modbus_registers[263]=(char)(ETH_IP_1);
modbus_registers[264]=0;									//���133  �������� IP ����� 2
modbus_registers[265]=(char)(ETH_IP_2);
modbus_registers[266]=0;									//���134  �������� IP ����� 3
modbus_registers[267]=(char)(ETH_IP_3);
modbus_registers[268]=0;									//���135  �������� IP ����� 4
modbus_registers[269]=(char)(ETH_IP_4);
modbus_registers[270]=0;									//���136  ����� ���� ����� 1
modbus_registers[271]=(char)(ETH_MASK_1);
modbus_registers[272]=0;									//���137  ����� ���� ����� 2
modbus_registers[273]=(char)(ETH_MASK_2);
modbus_registers[274]=0;									//���138  ����� ���� ����� 3
modbus_registers[275]=(char)(ETH_MASK_3);
modbus_registers[276]=0;									//���139  ����� ���� ����� 4
modbus_registers[277]=(char)(ETH_MASK_4);
modbus_registers[278]=0;									//���140  ���� ����� 1
modbus_registers[279]=(char)(ETH_GW_1);
modbus_registers[280]=0;									//���141  ���� ����� 2
modbus_registers[281]=(char)(ETH_GW_2);
modbus_registers[282]=0;									//���142  ���� ����� 3
modbus_registers[283]=(char)(ETH_GW_3);
modbus_registers[284]=0;									//���143  ���� ����� 4
modbus_registers[285]=(char)(ETH_GW_4);
modbus_registers[286]=(char)(ETH_SNMP_PORT_READ>>8);		//���144  ���� ������ 
modbus_registers[287]=(char)(ETH_SNMP_PORT_READ);
modbus_registers[288]=(char)(ETH_SNMP_PORT_WRITE>>8);		//���145  ���� ������ 
modbus_registers[289]=(char)(ETH_SNMP_PORT_WRITE);
modbus_registers[290]=0;									//���146  ������ ���� 1
modbus_registers[291]=(char)(snmp_community[0]);
modbus_registers[292]=0;									//���147  ������ ���� 2
modbus_registers[293]=(char)(snmp_community[1]);
modbus_registers[294]=0;									//���148  ������ ���� 3
modbus_registers[295]=(char)(snmp_community[2]);
modbus_registers[296]=0;									//���149  ������ ���� 4
modbus_registers[297]=(char)(snmp_community[3]);
modbus_registers[298]=0;									//���150  ������ ���� 5
modbus_registers[299]=(char)(snmp_community[4]);
modbus_registers[300]=0;									//���151  ������ ���� 6
modbus_registers[301]=(char)(snmp_community[5]);
modbus_registers[302]=0;									//���152  ������ ���� 7
modbus_registers[303]=(char)(snmp_community[6]);
modbus_registers[304]=0;									//���153  ������ ���� 7
modbus_registers[305]=(char)(snmp_community[7]);
modbus_registers[306]=0;									//���154  TRAP1 IP ����� 1
modbus_registers[307]=(char)(ETH_TRAP1_IP_1);
modbus_registers[308]=0;									//���155  TRAP1 IP ����� 2
modbus_registers[309]=(char)(ETH_TRAP1_IP_2);
modbus_registers[310]=0;									//���156  TRAP1 IP ����� 3
modbus_registers[311]=(char)(ETH_TRAP1_IP_3);
modbus_registers[312]=0;									//���157  TRAP1 IP ����� 4
modbus_registers[313]=(char)(ETH_TRAP1_IP_4);
modbus_registers[314]=0;									//���158  TRAP2 IP ����� 1
modbus_registers[315]=(char)(ETH_TRAP2_IP_1);
modbus_registers[316]=0;									//���159  TRAP2 IP ����� 2
modbus_registers[317]=(char)(ETH_TRAP2_IP_2);
modbus_registers[318]=0;									//���160  TRAP2 IP ����� 3
modbus_registers[319]=(char)(ETH_TRAP2_IP_3);
modbus_registers[320]=0;									//���161  TRAP2 IP ����� 4
modbus_registers[321]=(char)(ETH_TRAP2_IP_4);
modbus_registers[322]=0;									//���162  TRAP3 IP ����� 1
modbus_registers[323]=(char)(ETH_TRAP3_IP_1);
modbus_registers[324]=0;									//���163  TRAP3 IP ����� 2
modbus_registers[325]=(char)(ETH_TRAP3_IP_2);
modbus_registers[326]=0;									//���164  TRAP3 IP ����� 3
modbus_registers[327]=(char)(ETH_TRAP3_IP_3);
modbus_registers[328]=0;									//���165  TRAP3 IP ����� 4
modbus_registers[329]=(char)(ETH_TRAP3_IP_4);
modbus_registers[330]=0;									//���166  TRAP4 IP ����� 1
modbus_registers[331]=(char)(ETH_TRAP4_IP_1);
modbus_registers[332]=0;									//���167  TRAP4 IP ����� 2
modbus_registers[333]=(char)(ETH_TRAP4_IP_2);
modbus_registers[334]=0;									//���168  TRAP4 IP ����� 3
modbus_registers[335]=(char)(ETH_TRAP4_IP_3);
modbus_registers[336]=0;									//���169  TRAP4 IP ����� 4
modbus_registers[337]=(char)(ETH_TRAP4_IP_4);
modbus_registers[338]=0;									//���170  TRAP5 IP ����� 1
modbus_registers[339]=(char)(ETH_TRAP5_IP_1);
modbus_registers[340]=0;									//���171  TRAP5 IP ����� 2
modbus_registers[341]=(char)(ETH_TRAP5_IP_2);
modbus_registers[342]=0;									//���172  TRAP5 IP ����� 3
modbus_registers[343]=(char)(ETH_TRAP5_IP_3);
modbus_registers[344]=0;									//���173  TRAP5 IP ����� 4
modbus_registers[345]=(char)(ETH_TRAP5_IP_4);
modbus_registers[346]=0;									//���174  ������ ���� 1
modbus_registers[347]=(char)(snmp_web_passw[0]);
modbus_registers[348]=0;									//���175  ������ ���� 2
modbus_registers[349]=(char)(snmp_web_passw[1]);
modbus_registers[350]=0;									//���176  ������ ���� 3
modbus_registers[351]=(char)(snmp_web_passw[2]);
modbus_registers[352]=0;									//���177  ������������ ���
modbus_registers[353]=0;
modbus_registers[354]=(char)((TVENTMAX*10)>>8);				//���178  ����� ������� ����������� 
modbus_registers[355]=(char)(TVENTMAX*10);
modbus_registers[356]=0;							//���179  ������.�����, 0-�������, 1-��������, 2-�����.���.
modbus_registers[357]=(char)(ICA_EN);
modbus_registers[358]=0;							//���180  ������.�����, 0-MODBUS-RTU, 1-MODBUS-TCP, 2-RS485-2
modbus_registers[359]=(char)(ICA_CH);
modbus_registers[360]=0;									//���181  IP �������� ����� 1
modbus_registers[361]=(char)(ICA_MODBUS_TCP_IP1);
modbus_registers[362]=0;									//���182  IP �������� ����� 2
modbus_registers[363]=(char)(ICA_MODBUS_TCP_IP2);
modbus_registers[364]=0;									//���183  IP �������� ����� 3
modbus_registers[365]=(char)(ICA_MODBUS_TCP_IP3);
modbus_registers[366]=0;									//���184  IP �������� ����� 4
modbus_registers[367]=(char)(ICA_MODBUS_TCP_IP4);
modbus_registers[368]=0;									//���185  ����� �������� MODBUS-TCP	 1-254
modbus_registers[369]=(char)(ICA_MODBUS_TCP_UNIT_ID);
modbus_registers[370]=0;									//���186  ����� ��������  MODBUS-RTU  1-254
modbus_registers[371]=(char)(ICA_MODBUS_ADDRESS);
modbus_registers[372]=0;									//���187  ��������� ���, 1%, 10-100%
modbus_registers[373]=(char)(PWM_START);
modbus_registers[374]=0;									//���188  �������� ���� ���, 1-1�������, 2-2, 3-3
modbus_registers[375]=(char)(KB_ALGORITM);
modbus_registers[376]=0;									//���189  �������� �������������, 1-��������, 2-�/2, 3-�/3, 4-c/4, 5-c/5
modbus_registers[377]=(char)(REG_SPEED);
modbus_registers[378]=0;									//���190  ����������, 0-���������, 1-���������
modbus_registers[379]=(char)(SMART_SPC);
//��� ����
modbus_registers[380]=0;									//���191  �������� ����, 1-1�, 3-3�
modbus_registers[381]=(char)(NUMPHASE);
modbus_registers[382]=0;									//���192  �����.���������
modbus_registers[383]=(char)(TVENTON);
modbus_registers[384]=0;									//���193  �����.����������
modbus_registers[385]=(char)(TVENTOFF);
modbus_registers[386]=0;									//���194  ������ ����. 0-T���, 1-������., 2-����
modbus_registers[387]=(char)(RELEVENTSIGN);
modbus_registers[388]=0;									//���195  ����. ��� 0-����, 1-���� ����., 2-���� ����.���2
modbus_registers[389]=(char)(NPN_OUT);
modbus_registers[390]=(char)(UONPN>>8);						//���196  U���� ��� 0,1�, 10,0-250,0�
modbus_registers[391]=(char)(UONPN);
modbus_registers[392]=(char)(UVNPN>>8);						//���197  U��� ��� 0,1�, 10,0-250,0�
modbus_registers[393]=(char)(UVNPN);
modbus_registers[394]=0;									//���198  �������� ����. ���, 1�, 10-60
modbus_registers[395]=(char)(TZNPN);
/*modbus_registers[396]=0;									//���199  �������� �����. �����, 1%, 0-50
modbus_registers[397]=(char)(UBM_AV);  */
//o_8_e	   		   	   

 //o_10_s	          
modbus_registers[398]=(char)(r_iz_porog_pred>>8);			//���200  ����� ��������������
modbus_registers[399]=(char)(r_iz_porog_pred); 
modbus_registers[400]=(char)(r_iz_porog_error>>8);			//���201  ����� ������
modbus_registers[401]=(char)(r_iz_porog_error);
modbus_registers[402]=0;									//���202  ����� ���������� � %
modbus_registers[403]=(char)(asymmetry_porog);   		          	   
modbus_registers[404]=0;									//���203  ����� ���������� � ������� >1MOm
modbus_registers[405]=(char)(u_asymmetry_porog_up);
modbus_registers[406]=0;									//���204  ����� ���������� � �������
modbus_registers[407]=(char)(u_asymmetry_porog);
modbus_registers[408]=0;									//���205  ����� ���������� � ������� <20kOm
modbus_registers[409]=(char)(u_asymmetry_porog_down);
modbus_registers[410]=(char)(porog_u_in>>8);				//���206  ����� U����
modbus_registers[411]=(char)(porog_u_in); 
//o_10_e

modbus_registers[418]=(char)(BAT_C_NOM[0]>>8);				//���210  ������� ������� �1 (1�*�)
modbus_registers[419]=(char)(BAT_C_NOM[0]); 
modbus_registers[420]=(char)(BAT_C_NOM[1]>>8);				//���211  ������� ������� �2 (1�*�)
modbus_registers[421]=(char)(BAT_C_NOM[1]); 

#ifdef UKU_FSO
//UKUFSO_IBEP_SN=100000;
modbus_registers[500]=(char)(UKUFSO_IBEP_SN>>8);				//���251  �������� ����� ����(������� �����)
modbus_registers[501]=(char)(UKUFSO_IBEP_SN); 
modbus_registers[502]=(char)(UKUFSO_IBEP_SN>>24);				//���252  �������� ����� ����(������� �����)
modbus_registers[503]=(char)(UKUFSO_IBEP_SN>>16); 
modbus_registers[504]=(char)(UKUFSO_IBEP_START_DATE_YEAR>>8);	//���253  ��� �������� � ������������ ����
modbus_registers[505]=(char)(UKUFSO_IBEP_START_DATE_YEAR); 
modbus_registers[506]=(char)(UKUFSO_IBEP_START_DATE_MONTH);		//���254  ���� � ����� �������� � ������������ ����
modbus_registers[507]=(char)(UKUFSO_IBEP_START_DATE_DAY);

modbus_registers[560]=(char)(UKUFSO_BPS1_SN>>8);				//���281  �������� ����� ��ѹ1(������� �����)
modbus_registers[561]=(char)(UKUFSO_BPS1_SN); 
modbus_registers[562]=(char)(UKUFSO_BPS1_SN>>24);				//���282  �������� ����� ��ѹ1(������� �����)
modbus_registers[563]=(char)(UKUFSO_BPS1_SN>>16); 
modbus_registers[564]=(char)(UKUFSO_BPS1_START_DATE_YEAR>>8);	//���283  ��� �������� � ������������ ��ѹ1
modbus_registers[565]=(char)(UKUFSO_BPS1_START_DATE_YEAR); 
modbus_registers[566]=(char)(UKUFSO_BPS1_START_DATE_MONTH);		//���284  ���� � ����� �������� � ������������ ��ѹ1
modbus_registers[567]=(char)(UKUFSO_BPS1_START_DATE_DAY);
modbus_registers[568]=(char)(UKUFSO_BPS2_SN>>8);				//���285  �������� ����� ��ѹ2(������� �����)
modbus_registers[569]=(char)(UKUFSO_BPS2_SN); 
modbus_registers[570]=(char)(UKUFSO_BPS2_SN>>24);				//���286  �������� ����� ��ѹ2(������� �����)
modbus_registers[571]=(char)(UKUFSO_BPS2_SN>>16); 
modbus_registers[572]=(char)(UKUFSO_BPS2_START_DATE_YEAR>>8);	//���287  ��� �������� � ������������ ��ѹ2
modbus_registers[573]=(char)(UKUFSO_BPS2_START_DATE_YEAR); 
modbus_registers[574]=(char)(UKUFSO_BPS2_START_DATE_MONTH);		//���288  ���� � ����� �������� � ������������ ��ѹ2
modbus_registers[575]=(char)(UKUFSO_BPS2_START_DATE_DAY);
modbus_registers[576]=(char)(UKUFSO_BPS3_SN>>8);				//���289  �������� ����� ��ѹ3(������� �����)
modbus_registers[577]=(char)(UKUFSO_BPS3_SN); 
modbus_registers[578]=(char)(UKUFSO_BPS3_SN>>24);				//���290  �������� ����� ��ѹ3(������� �����)
modbus_registers[579]=(char)(UKUFSO_BPS3_SN>>16); 
modbus_registers[580]=(char)(UKUFSO_BPS3_START_DATE_YEAR>>8);	//���291  ��� �������� � ������������ ��ѹ3
modbus_registers[581]=(char)(UKUFSO_BPS3_START_DATE_YEAR); 
modbus_registers[582]=(char)(UKUFSO_BPS3_START_DATE_MONTH);		//���292  ���� � ����� �������� � ������������ ��ѹ3
modbus_registers[583]=(char)(UKUFSO_BPS3_START_DATE_DAY);
modbus_registers[584]=(char)(UKUFSO_BPS4_SN>>8);				//���293  �������� ����� ��ѹ4(������� �����)
modbus_registers[585]=(char)(UKUFSO_BPS4_SN); 
modbus_registers[586]=(char)(UKUFSO_BPS4_SN>>24);				//���294  �������� ����� ��ѹ4(������� �����)
modbus_registers[587]=(char)(UKUFSO_BPS4_SN>>16); 
modbus_registers[588]=(char)(UKUFSO_BPS4_START_DATE_YEAR>>8);	//���295  ��� �������� � ������������ ��ѹ4
modbus_registers[589]=(char)(UKUFSO_BPS4_START_DATE_YEAR); 
modbus_registers[590]=(char)(UKUFSO_BPS4_START_DATE_MONTH);		//���296  ���� � ����� �������� � ������������ ��ѹ4
modbus_registers[591]=(char)(UKUFSO_BPS4_START_DATE_DAY);
modbus_registers[592]=(char)(UKUFSO_BAT1_SN>>8);				//���297  �������� ����� ����1(������� �����)
modbus_registers[593]=(char)(UKUFSO_BAT1_SN); 
modbus_registers[594]=(char)(UKUFSO_BAT1_SN>>24);				//���298  �������� ����� ����1(������� �����)
modbus_registers[595]=(char)(UKUFSO_BAT1_SN>>16); 
modbus_registers[596]=(char)(UKUFSO_BAT1_START_DATE_YEAR>>8);	//���299  ��� �������� � ������������ ����1
modbus_registers[597]=(char)(UKUFSO_BAT1_START_DATE_YEAR); 
modbus_registers[598]=(char)(UKUFSO_BAT1_START_DATE_MONTH);		//���300  ���� � ����� �������� � ������������ ����1
modbus_registers[599]=(char)(UKUFSO_BAT1_START_DATE_DAY);
modbus_registers[600]=(char)(UKUFSO_BAT2_SN>>8);				//���301  �������� ����� ����2(������� �����)
modbus_registers[601]=(char)(UKUFSO_BAT2_SN); 
modbus_registers[602]=(char)(UKUFSO_BAT2_SN>>24);				//���302  �������� ����� ����2(������� �����)
modbus_registers[603]=(char)(UKUFSO_BAT2_SN>>16); 
modbus_registers[604]=(char)(UKUFSO_BAT2_START_DATE_YEAR>>8);	//���303  ��� �������� � ������������ ����2
modbus_registers[605]=(char)(UKUFSO_BAT2_START_DATE_YEAR); 
modbus_registers[606]=(char)(UKUFSO_BAT2_START_DATE_MONTH);		//���304  ���� � ����� �������� � ������������ ����2
modbus_registers[607]=(char)(UKUFSO_BAT2_START_DATE_DAY); 
#endif //UKU_FSO

if(prot==MODBUS_RTU_PROT)
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
else if(prot==MODBUS_TCP_PROT)
	{
	modbus_tcp_out_ptr=(char*)&modbus_registers[(reg_adr-1)*2];
	}
}

//-----------------------------------------------
void modbus_input_registers_transmit(unsigned char adr,unsigned char func,unsigned short reg_adr,unsigned short reg_quantity, char prot)
{
signed char modbus_registers[800];	 //o_10
//char modbus_tx_buff[200];
unsigned short crc_temp;
char i;
short tempS;

//tempS=(MODBUS_INPUT_REGS[0]);
//bps_I=bps_I_phantom;

#if defined UKU_6U || defined UKU_220_V2 //o_10  � UKU_220_V2 �� ������������ U � I � UKU_6U-������������ 
modbus_registers[0]=(signed char)(load_U>>8);					//���1   	���������� �������� ����, 0.1�
modbus_registers[1]=(signed char)(load_U);
modbus_registers[2]=(signed char)(load_I>>8);					//���2   	��� ������������, 0.1�
modbus_registers[3]=(signed char)(load_I);
#else
modbus_registers[0]=(signed char)(out_U>>8);					//���1   	���������� �������� ����, 0.1�
modbus_registers[1]=(signed char)(out_U);
modbus_registers[2]=(signed char)(bps_I>>8);					//���2   	��� ������������, 0.1�
modbus_registers[3]=(signed char)(bps_I);
#endif
/*
modbus_registers[0]=(signed char)(out_U>>8);					//���1   	���������� �������� ����, 0.1�
modbus_registers[1]=(signed char)(out_U);
modbus_registers[2]=(signed char)(bps_I>>8);					//���2   	��� ������������, 0.1�
modbus_registers[3]=(signed char)(bps_I);
*/
modbus_registers[4]=(signed char)(net_U>>8);					//���3   	���������� ���� �������, 1�
modbus_registers[5]=(signed char)(net_U);
modbus_registers[6]=(signed char)(net_F>>8);					//���4   	������� ���� �������, 0.1��
modbus_registers[7]=(signed char)(net_F);
modbus_registers[8]=(signed char)(net_Ua>>8);					//���5	���������� ���� ������� ���� A, 1�	
modbus_registers[9]=(signed char)(net_Ua);		 	
modbus_registers[10]=(signed char)(net_Ub>>8);				//���6	���������� ���� ������� ���� B, 1�
modbus_registers[11]=(signed char)(net_Ub);
modbus_registers[12]=(signed char)(net_Uc>>8);				//���7	���������� ���� ������� ���� C, 1�
modbus_registers[13]=(signed char)(net_Uc);
modbus_registers[14]=(signed char)(bat[0]._Ub>>8);				//���8	���������� ������� �1, 0.1�
modbus_registers[15]=(signed char)(bat[0]._Ub);
modbus_registers[16]=(signed char)(bat[0]._Ib>>8);				//���9   	��� ������� �1, 0.01�
modbus_registers[17]=(signed char)(bat[0]._Ib);
#ifdef UKU_ZVU
modbus_registers[18]=(signed char)(t_ext[0]>>8);				//���10	����������� ������� �1, 1��
modbus_registers[19]=(signed char)(t_ext[0]);
#else
modbus_registers[18]=(signed char)(bat[0]._Tb>>8);				//���10	����������� ������� �1, 1��
modbus_registers[19]=(signed char)(bat[0]._Tb);
#endif
#ifdef UKU_ZVU
modbus_registers[20]=(signed char)(((short)(bat_hndl_zvu_Q[0]/10000L))>>8);			//���11	����� ������� �1, %
modbus_registers[21]=(signed char)(((short)(bat_hndl_zvu_Q[0]/10000L)));
#else
modbus_registers[20]=(signed char)(bat[0]._zar>>8);			//���11	����� ������� �1, %
modbus_registers[21]=(signed char)(bat[0]._zar);
#endif
modbus_registers[22]=(signed char)(bat[0]._Ubm>>8);			//���12	���������� ������� ����� ������� �1, 0.1�
modbus_registers[23]=(signed char)(bat[0]._Ubm);
modbus_registers[24]=(signed char)(bat[0]._dUbm>>8);			//���13	������ ������� ����� ������� �1, %
modbus_registers[25]=(signed char)(bat[0]._dUbm);
modbus_registers[26]=(signed char)(BAT_C_REAL[0]>>8);			//���14	�������� ������� ������� �1, 0.1�*�, ���� 0x5555 �� �� ����������
modbus_registers[27]=(signed char)(BAT_C_REAL[0]);
modbus_registers[28]=(signed char)(bat[1]._Ub>>8);				//���15	���������� ������� �1, 0.1�
modbus_registers[29]=(signed char)(bat[1]._Ub);
modbus_registers[30]=(signed char)(bat[1]._Ib>>8);				//���16   	��� ������� �1, 0.01�
modbus_registers[31]=(signed char)(bat[1]._Ib);
modbus_registers[32]=(signed char)(bat[1]._Tb>>8);				//���17	����������� ������� �1, 1��
modbus_registers[33]=(signed char)(bat[1]._Tb);
#ifdef UKU_ZVU
modbus_registers[34]=(signed char)(((short)(bat_hndl_zvu_Q[1]/10000L))>>8);			//���18	����� ������� �1, %
modbus_registers[35]=(signed char)(((short)(bat_hndl_zvu_Q[1]/10000L)));
#else
modbus_registers[34]=(signed char)(bat[1]._zar>>8);			//���18	����� ������� �1, %
modbus_registers[35]=(signed char)(bat[1]._zar);
#endif
modbus_registers[36]=(signed char)(bat[1]._Ubm>>8);			//���19	���������� ������� ����� ������� �1, 0.1�
modbus_registers[37]=(signed char)(bat[1]._Ubm);
modbus_registers[38]=(signed char)(bat[1]._dUbm>>8);			//���20	������ ������� ����� ������� �1, %
modbus_registers[39]=(signed char)(bat[1]._dUbm);
modbus_registers[40]=(signed char)(BAT_C_REAL[1]>>8);			//���21	�������� ������� ������� �1, 0.1�*�, ���� 0x5555 �� �� ����������
modbus_registers[41]=(signed char)(BAT_C_REAL[1]);
modbus_registers[42]=(signed char)(bps[0]._Uii>>8);			//���22	�������� ���������� ����������� �1, 0.1�
modbus_registers[43]=(signed char)(bps[0]._Uii);
modbus_registers[44]=(signed char)(bps[0]._Ii>>8);				//���23	�������� ��� ����������� �1, 0.1�
modbus_registers[45]=(signed char)(bps[0]._Ii);
modbus_registers[46]=(signed char)(bps[0]._Ti>>8);				//���24	����������� ��������� ����������� �1, 1��
modbus_registers[47]=(signed char)(bps[0]._Ti);
modbus_registers[48]=(signed char)(bps[0]._av>>8);				//���25	���� ������ ����������� �1, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[49]=(signed char)(bps[0]._av);
modbus_registers[50]=(signed char)(bps[1]._Uii>>8);			//���26	�������� ���������� ����������� �2, 0.1�
modbus_registers[51]=(signed char)(bps[1]._Uii);
modbus_registers[52]=(signed char)(bps[1]._Ii>>8);				//���27	�������� ��� ����������� �2, 0.1�
modbus_registers[53]=(signed char)(bps[1]._Ii);
modbus_registers[54]=(signed char)(bps[1]._Ti>>8);				//���28	����������� ��������� ����������� �2, 1��
modbus_registers[55]=(signed char)(bps[1]._Ti);
modbus_registers[56]=(signed char)(bps[1]._av>>8);				//���29	���� ������ ����������� �2, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[57]=(signed char)(bps[1]._av);
modbus_registers[58]=(signed char)(bps[2]._Uii>>8);			//���30	�������� ���������� ����������� �3, 0.1�
modbus_registers[59]=(signed char)(bps[2]._Uii);
modbus_registers[60]=(signed char)(bps[2]._Ii>>8);				//���31	�������� ��� ����������� �3, 0.1�
modbus_registers[61]=(signed char)(bps[2]._Ii);
modbus_registers[62]=(signed char)(bps[2]._Ti>>8);				//���32	����������� ��������� ����������� �3, 1��
modbus_registers[63]=(signed char)(bps[2]._Ti);
modbus_registers[64]=(signed char)(bps[2]._av>>8);				//���33	���� ������ ����������� �3, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[65]=(signed char)(bps[2]._av);
modbus_registers[66]=(signed char)(bps[3]._Uii>>8);			//���34	�������� ���������� ����������� �4, 0.1�
modbus_registers[67]=(signed char)(bps[3]._Uii);
modbus_registers[68]=(signed char)(bps[3]._Ii>>8);				//���35	�������� ��� ����������� �4, 0.1�
modbus_registers[69]=(signed char)(bps[3]._Ii);
modbus_registers[70]=(signed char)(bps[3]._Ti>>8);				//���36	����������� ��������� ����������� �4, 1��
modbus_registers[71]=(signed char)(bps[3]._Ti);
modbus_registers[72]=(signed char)(bps[3]._av>>8);				//���37	���� ������ ����������� �4, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[73]=(signed char)(bps[3]._av);
modbus_registers[74]=(signed char)(bps[4]._Uii>>8);			//���38	�������� ���������� ����������� �5, 0.1�
modbus_registers[75]=(signed char)(bps[4]._Uii);
modbus_registers[76]=(signed char)(bps[4]._Ii>>8);				//���39	�������� ��� ����������� �5, 0.1�
modbus_registers[77]=(signed char)(bps[4]._Ii);
modbus_registers[78]=(signed char)(bps[4]._Ti>>8);				//���40	����������� ��������� ����������� �5, 1��
modbus_registers[79]=(signed char)(bps[4]._Ti);
modbus_registers[80]=(signed char)(bps[4]._av>>8);				//���41	���� ������ ����������� �5, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[81]=(signed char)(bps[4]._av);
modbus_registers[82]=(signed char)(bps[5]._Uii>>8);			//���42	�������� ���������� ����������� �6, 0.1�
modbus_registers[83]=(signed char)(bps[5]._Uii);
modbus_registers[84]=(signed char)(bps[5]._Ii>>8);				//���43	�������� ��� ����������� �6, 0.1�
modbus_registers[85]=(signed char)(bps[5]._Ii);
modbus_registers[86]=(signed char)(bps[5]._Ti>>8);				//���44	����������� ��������� ����������� �6, 1��
modbus_registers[87]=(signed char)(bps[5]._Ti);
modbus_registers[88]=(signed char)(bps[5]._av>>8);				//���45	���� ������ ����������� �6, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[89]=(signed char)(bps[5]._av);
modbus_registers[90]=(signed char)(bps[6]._Uii>>8);			//���46	�������� ���������� ����������� �7, 0.1�
modbus_registers[91]=(signed char)(bps[6]._Uii);
modbus_registers[92]=(signed char)(bps[6]._Ii>>8);				//���47	�������� ��� ����������� �7, 0.1�
modbus_registers[93]=(signed char)(bps[6]._Ii);
modbus_registers[94]=(signed char)(bps[6]._Ti>>8);				//���48	����������� ��������� ����������� �7, 1��
modbus_registers[95]=(signed char)(bps[6]._Ti);
modbus_registers[96]=(signed char)(bps[6]._av>>8);				//���49	���� ������ ����������� �7, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[97]=(signed char)(bps[6]._av);
modbus_registers[98]=(signed char)(bps[7]._Uii>>8);			//���50	�������� ���������� ����������� �8, 0.1�
modbus_registers[99]=(signed char)(bps[7]._Uii);
modbus_registers[100]=(signed char)(bps[7]._Ii>>8);			//���51	�������� ��� ����������� �8, 0.1�
modbus_registers[101]=(signed char)(bps[7]._Ii);
modbus_registers[102]=(signed char)(bps[7]._Ti>>8);			//���52	����������� ��������� ����������� �8, 1��
modbus_registers[103]=(signed char)(bps[7]._Ti);
modbus_registers[104]=(signed char)(bps[7]._av>>8);			//���53	���� ������ ����������� �8, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[105]=(signed char)(bps[7]._av);
modbus_registers[106]=(signed char)(bps_U>>8);					//���54   	���������� ������������, 0.1�
modbus_registers[107]=(signed char)(bps_U);
tempS=0;
if((speedChIsOn)||(sp_ch_stat==scsWRK)) tempS=1;
modbus_registers[108]=(signed char)(tempS>>8);					//���55   	���������� ����� ������������, (1 - ���, 0 - ����)
modbus_registers[109]=(signed char)(tempS);
tempS=0;
if(spc_stat==spcVZ) tempS=1;
modbus_registers[110]=(signed char)(tempS>>8);					//���56   	������������� ����� ������������, (1 - ���, 0 - ����)
modbus_registers[111]=(signed char)(tempS);
modbus_registers[112]=(signed char)(uout_av>>8);					//���57   �������� ��������� ����������, (0 - �����, 1 - ��������, 2 - ��������)
modbus_registers[113]=(signed char)(uout_av);
/*
tempS=0;													 //���60	������� ������ ��������� �������
if(bat_ips._av)			tempS|=(1<<0);						 // ��� 0	������ �������
if(avar_stat&0x0001)   	tempS|=(1<<1);						 //	��� 1	������ �������� ���� 
if(avar_stat&(1<<(3+0)))tempS|=(1<<2);						 //	��� 2	������ ����������� �1
if(avar_stat&(1<<(3+1)))tempS|=(1<<3);						 //	��� 3	������ ����������� �2
if(avar_stat&(1<<(3+2)))tempS|=(1<<4);						 //	��� 4	������ ����������� �3
if(avar_stat&(1<<(3+3)))tempS|=(1<<5);						 	//	��� 5	������ ����������� �4
if(avar_stat&(1<<(3+4)))tempS|=(1<<6);						 	//	��� 6	������ ����������� �5
if(avar_stat&(1<<(3+5)))tempS|=(1<<7);						 	//	��� 7	������ ����������� �6
if(avar_stat&(1<<(3+6)))tempS|=(1<<8);						 	//	��� 8	������ ����������� �7
if(avar_stat&(1<<(3+6)))tempS|=(1<<9);						 	//	��� 8	������ ����������� �8
*/
tempS=0;
tempS=avar_stat;
#ifdef UKU_ZVU
if(bat_ips[0]._av)			tempS|=(1<<1);
else 						tempS&=~(1<<1);
if(bat_ips[1]._av)			tempS|=(1<<2);
else 						tempS&=~(1<<2);
#endif
//���60	������� ������ ��������� �������
// 	��� 0	������ �������� ���� 
//	��� 1	������ ������� �1(������ ������� ��� ���)
//	��� 2	������ ������� �2
//	��� 3	������ ����������� �1
//	��� 4	������ ����������� �2
//	��� 5	������ ����������� �3
//	��� 6	������ ����������� �4
//	��� 7	������ ����������� �5
//	��� 8	������ ����������� �6
//	��� 9	������ ����������� �7
//	��� 10	������ ����������� �8

modbus_registers[118]=(signed char)(tempS>>8);
modbus_registers[119]=(signed char)(tempS);

modbus_registers[120]=(signed char)(volta_short>>8);		//���61   	���������� ��������, 0.1�
modbus_registers[121]=(signed char)(volta_short);
modbus_registers[122]=(signed char)(curr_short>>8);			//���62  	��� ��������, 0.01�
modbus_registers[123]=(signed char)(curr_short);
modbus_registers[124]=(signed char)(power_int>>8);			//���63   	�������� ��������, 1��
modbus_registers[125]=(signed char)(power_int);


modbus_registers[138]=(signed char)(HARDVARE_VERSION>>8);	//��� 70  	���������� ������
modbus_registers[139]=(signed char)(HARDVARE_VERSION);
modbus_registers[140]=(signed char)(SOFT_VERSION>>8);		//��� 71  	������ ��
modbus_registers[141]=(signed char)(SOFT_VERSION);
modbus_registers[142]=(signed char)(BUILD>>8);				//��� 72  	����� ���������� ��
modbus_registers[143]=(signed char)(BUILD);
modbus_registers[144]=(signed char)(BUILD_YEAR>>8);			//��� 73  	���	���������� ��
modbus_registers[145]=(signed char)(BUILD_YEAR);
modbus_registers[146]=(signed char)(BUILD_MONTH>>8);		//��� 74  	����� ���������� ��
modbus_registers[147]=(signed char)(BUILD_MONTH);
modbus_registers[148]=(signed char)(BUILD_DAY>>8);			//��� 75  	���� ���������� ��
modbus_registers[149]=(signed char)(BUILD_DAY);
modbus_registers[150]=(signed char)(AUSW_MAIN_NUMBER>>8); 	//?aa 76 caaianeie iiia?
modbus_registers[151]=(signed char)(AUSW_MAIN_NUMBER);
modbus_registers[152]=(signed char)(AUSW_MAIN_NUMBER>>24);			//��� 77  	caaianeie iiia?
modbus_registers[153]=(signed char)(AUSW_MAIN_NUMBER>>16);
tempS=cntrl_stat_old;
if(	(main_kb_cnt==(TBAT*60)-21) || (main_kb_cnt==(TBAT*60)-20) || (main_kb_cnt==(TBAT*60)-19)) tempS=((short)TBAT)|0x4000;
//tempS=0x800f;
modbus_registers[198]=(signed char)(tempS>>8);				//???100	????????? ???
modbus_registers[199]=(signed char)(tempS);

modbus_registers[200]=(signed char)(bps[8]._Uii>>8);		//���101	�������� ���������� ����������� �9, 0.1�
modbus_registers[201]=(signed char)(bps[8]._Uii);
modbus_registers[202]=(signed char)(bps[8]._Ii>>8);			//���102	�������� ��� ����������� �9, 0.1�
modbus_registers[203]=(signed char)(bps[8]._Ii);
modbus_registers[204]=(signed char)(bps[8]._Ti>>8);			//���103	����������� ��������� ����������� �9, 1��
modbus_registers[205]=(signed char)(bps[8]._Ti);
modbus_registers[206]=(signed char)(bps[8]._av>>8);			//���104	���� ������ ����������� �9, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[207]=(signed char)(bps[8]._av);
modbus_registers[208]=(signed char)(bps[9]._Uii>>8);		//���101	�������� ���������� ����������� �9, 0.1�
modbus_registers[209]=(signed char)(bps[9]._Uii);
modbus_registers[210]=(signed char)(bps[9]._Ii>>8);			//���102	�������� ��� ����������� �9, 0.1�
modbus_registers[211]=(signed char)(bps[9]._Ii);
modbus_registers[212]=(signed char)(bps[9]._Ti>>8);			//���103	����������� ��������� ����������� �9, 1��
modbus_registers[213]=(signed char)(bps[9]._Ti);
modbus_registers[214]=(signed char)(bps[9]._av>>8);			//���104	���� ������ ����������� �9, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[215]=(signed char)(bps[9]._av);
modbus_registers[216]=(signed char)(bps[10]._Uii>>8);		//���101	�������� ���������� ����������� �9, 0.1�
modbus_registers[217]=(signed char)(bps[10]._Uii);
modbus_registers[218]=(signed char)(bps[10]._Ii>>8);			//���102	�������� ��� ����������� �9, 0.1�
modbus_registers[219]=(signed char)(bps[10]._Ii);
modbus_registers[220]=(signed char)(bps[10]._Ti>>8);			//���103	����������� ��������� ����������� �9, 1��
modbus_registers[221]=(signed char)(bps[10]._Ti);
modbus_registers[222]=(signed char)(bps[10]._av>>8);			//���104	���� ������ ����������� �9, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[223]=(signed char)(bps[10]._av);
modbus_registers[224]=(signed char)(bps[11]._Uii>>8);		//���101	�������� ���������� ����������� �9, 0.1�
modbus_registers[225]=(signed char)(bps[11]._Uii);
modbus_registers[226]=(signed char)(bps[11]._Ii>>8);			//���102	�������� ��� ����������� �9, 0.1�
modbus_registers[227]=(signed char)(bps[11]._Ii);
modbus_registers[228]=(signed char)(bps[11]._Ti>>8);			//���103	����������� ��������� ����������� �9, 1��
modbus_registers[229]=(signed char)(bps[11]._Ti);
modbus_registers[230]=(signed char)(bps[11]._av>>8);			//���104	���� ������ ����������� �9, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[231]=(signed char)(bps[11]._av);
modbus_registers[232]=(signed char)(bps[12]._Uii>>8);		//���101	�������� ���������� ����������� �9, 0.1�
modbus_registers[233]=(signed char)(bps[12]._Uii);
modbus_registers[234]=(signed char)(bps[12]._Ii>>8);			//���102	�������� ��� ����������� �9, 0.1�
modbus_registers[235]=(signed char)(bps[12]._Ii);
modbus_registers[236]=(signed char)(bps[12]._Ti>>8);			//���103	����������� ��������� ����������� �9, 1��
modbus_registers[237]=(signed char)(bps[12]._Ti);
modbus_registers[238]=(signed char)(bps[12]._av>>8);			//���104	���� ������ ����������� �9, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[239]=(signed char)(bps[12]._av);
modbus_registers[240]=(signed char)(bps[13]._Uii>>8);		//���101	�������� ���������� ����������� �9, 0.1�
modbus_registers[241]=(signed char)(bps[13]._Uii);
modbus_registers[242]=(signed char)(bps[13]._Ii>>8);			//���102	�������� ��� ����������� �9, 0.1�
modbus_registers[243]=(signed char)(bps[13]._Ii);
modbus_registers[244]=(signed char)(bps[13]._Ti>>8);			//���103	����������� ��������� ����������� �9, 1��
modbus_registers[245]=(signed char)(bps[13]._Ti);
modbus_registers[246]=(signed char)(bps[13]._av>>8);			//���104	���� ������ ����������� �9, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[247]=(signed char)(bps[13]._av);
modbus_registers[248]=(signed char)(bps[14]._Uii>>8);		//���101	�������� ���������� ����������� �9, 0.1�
modbus_registers[249]=(signed char)(bps[14]._Uii);
modbus_registers[250]=(signed char)(bps[14]._Ii>>8);			//���102	�������� ��� ����������� �9, 0.1�
modbus_registers[251]=(signed char)(bps[14]._Ii);
modbus_registers[252]=(signed char)(bps[14]._Ti>>8);			//���103	����������� ��������� ����������� �9, 1��
modbus_registers[253]=(signed char)(bps[14]._Ti);
modbus_registers[254]=(signed char)(bps[14]._av>>8);			//���104	���� ������ ����������� �9, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[255]=(signed char)(bps[14]._av);
modbus_registers[256]=(signed char)(bps[15]._Uii>>8);		//���101	�������� ���������� ����������� �9, 0.1�
modbus_registers[257]=(signed char)(bps[15]._Uii);
modbus_registers[258]=(signed char)(bps[15]._Ii>>8);			//���102	�������� ��� ����������� �9, 0.1�
modbus_registers[259]=(signed char)(bps[15]._Ii);
modbus_registers[260]=(signed char)(bps[15]._Ti>>8);			//���103	����������� ��������� ����������� �9, 1��
modbus_registers[261]=(signed char)(bps[15]._Ti);
modbus_registers[262]=(signed char)(bps[15]._av>>8);			//���104	���� ������ ����������� �9, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[263]=(signed char)(bps[15]._av);
modbus_registers[264]=(signed char)(bps[16]._Uii>>8);		//���101	�������� ���������� ����������� �9, 0.1�
modbus_registers[265]=(signed char)(bps[16]._Uii);
modbus_registers[266]=(signed char)(bps[16]._Ii>>8);			//���102	�������� ��� ����������� �9, 0.1�
modbus_registers[267]=(signed char)(bps[16]._Ii);
modbus_registers[268]=(signed char)(bps[16]._Ti>>8);			//���103	����������� ��������� ����������� �9, 1��
modbus_registers[269]=(signed char)(bps[16]._Ti);
modbus_registers[270]=(signed char)(bps[16]._av>>8);			//���104	���� ������ ����������� �9, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[271]=(signed char)(bps[16]._av);
modbus_registers[272]=(signed char)(bps[17]._Uii>>8);		//���101	�������� ���������� ����������� �9, 0.1�
modbus_registers[273]=(signed char)(bps[17]._Uii);
modbus_registers[274]=(signed char)(bps[17]._Ii>>8);			//���102	�������� ��� ����������� �9, 0.1�
modbus_registers[275]=(signed char)(bps[17]._Ii);
modbus_registers[276]=(signed char)(bps[17]._Ti>>8);			//���103	����������� ��������� ����������� �9, 1��
modbus_registers[277]=(signed char)(bps[17]._Ti);
modbus_registers[278]=(signed char)(bps[17]._av>>8);			//���104	���� ������ ����������� �9, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[279]=(signed char)(bps[17]._av);
modbus_registers[280]=(signed char)(bps[18]._Uii>>8);		//���101	�������� ���������� ����������� �9, 0.1�
modbus_registers[281]=(signed char)(bps[18]._Uii);
modbus_registers[282]=(signed char)(bps[18]._Ii>>8);			//���102	�������� ��� ����������� �9, 0.1�
modbus_registers[283]=(signed char)(bps[18]._Ii);
modbus_registers[284]=(signed char)(bps[18]._Ti>>8);			//���103	����������� ��������� ����������� �9, 1��
modbus_registers[285]=(signed char)(bps[18]._Ti);
modbus_registers[286]=(signed char)(bps[18]._av>>8);			//���104	���� ������ ����������� �9, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[287]=(signed char)(bps[18]._av);
modbus_registers[288]=(signed char)(bps[19]._Uii>>8);		//���101	�������� ���������� ����������� �9, 0.1�
modbus_registers[289]=(signed char)(bps[19]._Uii);
modbus_registers[290]=(signed char)(bps[19]._Ii>>8);			//���102	�������� ��� ����������� �9, 0.1�
modbus_registers[291]=(signed char)(bps[19]._Ii);
modbus_registers[292]=(signed char)(bps[19]._Ti>>8);			//���103	����������� ��������� ����������� �9, 1��
modbus_registers[293]=(signed char)(bps[19]._Ti);
modbus_registers[294]=(signed char)(bps[19]._av>>8);			//���104	���� ������ ����������� �9, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[295]=(signed char)(bps[19]._av);
modbus_registers[296]=(signed char)(bps[20]._Uii>>8);		//���101	�������� ���������� ����������� �9, 0.1�
modbus_registers[297]=(signed char)(bps[20]._Uii);
modbus_registers[298]=(signed char)(bps[20]._Ii>>8);			//���102	�������� ��� ����������� �9, 0.1�
modbus_registers[299]=(signed char)(bps[20]._Ii);
modbus_registers[300]=(signed char)(bps[20]._Ti>>8);			//���103	����������� ��������� ����������� �9, 1��
modbus_registers[301]=(signed char)(bps[20]._Ti);
modbus_registers[302]=(signed char)(bps[20]._av>>8);			//���104	���� ������ ����������� �9, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[303]=(signed char)(bps[20]._av);
modbus_registers[304]=(signed char)(bps[21]._Uii>>8);		//���101	�������� ���������� ����������� �9, 0.1�
modbus_registers[305]=(signed char)(bps[21]._Uii);
modbus_registers[306]=(signed char)(bps[21]._Ii>>8);			//���102	�������� ��� ����������� �9, 0.1�
modbus_registers[307]=(signed char)(bps[21]._Ii);
modbus_registers[308]=(signed char)(bps[21]._Ti>>8);			//���103	����������� ��������� ����������� �9, 1��
modbus_registers[309]=(signed char)(bps[21]._Ti);
modbus_registers[310]=(signed char)(bps[21]._av>>8);			//���104	���� ������ ����������� �9, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[311]=(signed char)(bps[21]._av);
modbus_registers[312]=(signed char)(bps[22]._Uii>>8);		//���101	�������� ���������� ����������� �9, 0.1�
modbus_registers[313]=(signed char)(bps[22]._Uii);
modbus_registers[314]=(signed char)(bps[22]._Ii>>8);			//���102	�������� ��� ����������� �9, 0.1�
modbus_registers[315]=(signed char)(bps[22]._Ii);
modbus_registers[316]=(signed char)(bps[22]._Ti>>8);			//���103	����������� ��������� ����������� �9, 1��
modbus_registers[317]=(signed char)(bps[22]._Ti);
modbus_registers[318]=(signed char)(bps[22]._av>>8);			//���104	���� ������ ����������� �9, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[319]=(signed char)(bps[22]._av);
modbus_registers[320]=(signed char)(bps[23]._Uii>>8);		//���101	�������� ���������� ����������� �9, 0.1�
modbus_registers[321]=(signed char)(bps[23]._Uii);
modbus_registers[322]=(signed char)(bps[23]._Ii>>8);			//���102	�������� ��� ����������� �9, 0.1�
modbus_registers[323]=(signed char)(bps[23]._Ii);
modbus_registers[324]=(signed char)(bps[23]._Ti>>8);			//���103	����������� ��������� ����������� �9, 1��
modbus_registers[325]=(signed char)(bps[23]._Ti);
modbus_registers[326]=(signed char)(bps[23]._av>>8);			//���104	���� ������ ����������� �9, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[327]=(signed char)(bps[23]._av);
modbus_registers[328]=(signed char)(bps[24]._Uii>>8);		//���101	�������� ���������� ����������� �9, 0.1�
modbus_registers[329]=(signed char)(bps[24]._Uii);
modbus_registers[330]=(signed char)(bps[24]._Ii>>8);			//���102	�������� ��� ����������� �9, 0.1�
modbus_registers[331]=(signed char)(bps[24]._Ii);
modbus_registers[332]=(signed char)(bps[24]._Ti>>8);			//���103	����������� ��������� ����������� �9, 1��
modbus_registers[333]=(signed char)(bps[24]._Ti);
modbus_registers[334]=(signed char)(bps[24]._av>>8);			//���104	���� ������ ����������� �9, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[335]=(signed char)(bps[24]._av);
modbus_registers[336]=(signed char)(bps[25]._Uii>>8);		//���101	�������� ���������� ����������� �9, 0.1�
modbus_registers[337]=(signed char)(bps[25]._Uii);
modbus_registers[338]=(signed char)(bps[25]._Ii>>8);			//���102	�������� ��� ����������� �9, 0.1�
modbus_registers[339]=(signed char)(bps[25]._Ii);
modbus_registers[340]=(signed char)(bps[25]._Ti>>8);			//���103	����������� ��������� ����������� �9, 1��
modbus_registers[341]=(signed char)(bps[25]._Ti);
modbus_registers[342]=(signed char)(bps[25]._av>>8);			//���104	���� ������ ����������� �9, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[343]=(signed char)(bps[25]._av);
modbus_registers[344]=(signed char)(bps[26]._Uii>>8);		//���101	�������� ���������� ����������� �9, 0.1�
modbus_registers[345]=(signed char)(bps[26]._Uii);
modbus_registers[346]=(signed char)(bps[26]._Ii>>8);			//���102	�������� ��� ����������� �9, 0.1�
modbus_registers[347]=(signed char)(bps[26]._Ii);
modbus_registers[348]=(signed char)(bps[26]._Ti>>8);			//���103	����������� ��������� ����������� �9, 1��
modbus_registers[349]=(signed char)(bps[26]._Ti);
modbus_registers[350]=(signed char)(bps[26]._av>>8);			//���104	���� ������ ����������� �9, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[351]=(signed char)(bps[26]._av);
modbus_registers[352]=(signed char)(bps[27]._Uii>>8);		//���101	�������� ���������� ����������� �9, 0.1�
modbus_registers[353]=(signed char)(bps[27]._Uii);
modbus_registers[354]=(signed char)(bps[27]._Ii>>8);			//���102	�������� ��� ����������� �9, 0.1�
modbus_registers[355]=(signed char)(bps[27]._Ii);
modbus_registers[356]=(signed char)(bps[27]._Ti>>8);			//���103	����������� ��������� ����������� �9, 1��
modbus_registers[357]=(signed char)(bps[27]._Ti);
modbus_registers[358]=(signed char)(bps[27]._av>>8);			//���104	���� ������ ����������� �9, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[359]=(signed char)(bps[27]._av);
modbus_registers[360]=(signed char)(bps[28]._Uii>>8);		//���101	�������� ���������� ����������� �9, 0.1�
modbus_registers[361]=(signed char)(bps[28]._Uii);
modbus_registers[362]=(signed char)(bps[28]._Ii>>8);			//���102	�������� ��� ����������� �9, 0.1�
modbus_registers[363]=(signed char)(bps[28]._Ii);
modbus_registers[364]=(signed char)(bps[28]._Ti>>8);			//���103	����������� ��������� ����������� �9, 1��
modbus_registers[365]=(signed char)(bps[28]._Ti);
modbus_registers[366]=(signed char)(bps[28]._av>>8);			//���104	���� ������ ����������� �9, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[367]=(signed char)(bps[28]._av);
modbus_registers[368]=(signed char)(bps[29]._Uii>>8);		//���101	�������� ���������� ����������� �9, 0.1�
modbus_registers[369]=(signed char)(bps[29]._Uii);
modbus_registers[370]=(signed char)(bps[29]._Ii>>8);			//���102	�������� ��� ����������� �9, 0.1�
modbus_registers[371]=(signed char)(bps[29]._Ii);
modbus_registers[372]=(signed char)(bps[29]._Ti>>8);			//���103	����������� ��������� ����������� �9, 1��
modbus_registers[373]=(signed char)(bps[29]._Ti);
modbus_registers[374]=(signed char)(bps[29]._av>>8);			//���104	���� ������ ����������� �9, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[375]=(signed char)(bps[29]._av);
modbus_registers[376]=(signed char)(bps[30]._Uii>>8);		//���101	�������� ���������� ����������� �9, 0.1�
modbus_registers[377]=(signed char)(bps[30]._Uii);
modbus_registers[378]=(signed char)(bps[30]._Ii>>8);			//���102	�������� ��� ����������� �9, 0.1�
modbus_registers[379]=(signed char)(bps[30]._Ii);
modbus_registers[380]=(signed char)(bps[30]._Ti>>8);			//���103	����������� ��������� ����������� �9, 1��
modbus_registers[381]=(signed char)(bps[30]._Ti);
modbus_registers[382]=(signed char)(bps[30]._av>>8);			//���104	���� ������ ����������� �9, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[383]=(signed char)(bps[30]._av);
modbus_registers[384]=(signed char)(bps[31]._Uii>>8);		//���101	�������� ���������� ����������� �9, 0.1�
modbus_registers[385]=(signed char)(bps[31]._Uii);
modbus_registers[386]=(signed char)(bps[31]._Ii>>8);			//���102	�������� ��� ����������� �9, 0.1�
modbus_registers[387]=(signed char)(bps[31]._Ii);
modbus_registers[388]=(signed char)(bps[31]._Ti>>8);			//���103	����������� ��������� ����������� �9, 1��
modbus_registers[389]=(signed char)(bps[31]._Ti);
modbus_registers[390]=(signed char)(bps[31]._av>>8);			//���104	���� ������ ����������� �9, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[391]=(signed char)(bps[31]._av);
modbus_registers[392]=(signed char)(bps[32]._Uii>>8);		//���101	�������� ���������� ����������� �9, 0.1�
modbus_registers[393]=(signed char)(bps[32]._Uii);
modbus_registers[394]=(signed char)(bps[32]._Ii>>8);			//���102	�������� ��� ����������� �9, 0.1�
modbus_registers[395]=(signed char)(bps[32]._Ii);
modbus_registers[396]=(signed char)(bps[32]._Ti>>8);			//���103	����������� ��������� ����������� �9, 1��
modbus_registers[397]=(signed char)(bps[32]._Ti);
modbus_registers[398]=(signed char)(bps[32]._av>>8);			//���104	���� ������ ����������� �9, 0x01 - ��������, 0x02 �������� U���, 0x04 �������� U���, 0x08 - ����������� ����� � ������������
modbus_registers[399]=(signed char)(bps[32]._av);






tempS=t_ext[0];
if(ND_EXT[0])tempS=-1000;
modbus_registers[400]=(signed char)(tempS>>8);				//���201	������� ������ ����������� �1
modbus_registers[401]=(signed char)(tempS);
tempS=t_ext[1];
if(ND_EXT[1])tempS=-1000;
modbus_registers[402]=(signed char)(tempS>>8);				//���202	������� ������ ����������� �2
modbus_registers[403]=(signed char)(tempS);
tempS=t_ext[2];
if(ND_EXT[2])tempS=-1000;
modbus_registers[404]=(signed char)(tempS>>8);				//���203	������� ������ ����������� �3
modbus_registers[405]=(signed char)(tempS);
/*tempS=t_ext[3];
if(ND_EXT[3])tempS=-1000;
modbus_registers[406]=(signed char)(tempS>>8);				//���204	������� ������ ����������� �4
modbus_registers[407]=(signed char)(tempS);   */

modbus_registers[406]=(signed char)(bat_hndl_t_razr_min[0]>>8);
modbus_registers[407]=(signed char)(bat_hndl_t_razr_min[0]);
modbus_registers[408]=(signed char)(bat_hndl_t_razr_min[1]>>8);
modbus_registers[409]=(signed char)(bat_hndl_t_razr_min[1]);

tempS=0;
if(sk_stat[0]==ssON) tempS|=0x0001;
if(sk_av_stat[0]==sasON) tempS|=0x0002;
modbus_registers[420]=(signed char)(tempS>>8);				//���211	���������  ������ �������� �1, (������� ��� - ���������� ���������, 1 - �������, 0 - ���������, ������ ��� - �����������, 1 - ������, 0 - �����)
modbus_registers[421]=(signed char)(tempS);
tempS=0;
if(sk_stat[1]==ssON) tempS|=0x0001;
if(sk_av_stat[1]==sasON) tempS|=0x0002;
modbus_registers[422]=(signed char)(tempS>>8);				//���212	���������  ������ �������� �1, (������� ��� - ���������� ���������, 1 - �������, 0 - ���������, ������ ��� - �����������, 1 - ������, 0 - �����)
modbus_registers[423]=(signed char)(tempS);
tempS=0;
if(sk_stat[2]==ssON) tempS|=0x0001;
if(sk_av_stat[2]==sasON) tempS|=0x0002;
modbus_registers[424]=(signed char)(tempS>>8);				//���213	���������  ������ �������� �1, (������� ��� - ���������� ���������, 1 - �������, 0 - ���������, ������ ��� - �����������, 1 - ������, 0 - �����)
modbus_registers[425]=(signed char)(tempS);
tempS=0;
if(sk_stat[3]==ssON) tempS|=0x0001;
if(sk_av_stat[3]==sasON) tempS|=0x0002;
modbus_registers[426]=(signed char)(tempS>>8);				//���214	���������  ������ �������� �1, (������� ��� - ���������� ���������, 1 - �������, 0 - ���������, ������ ��� - �����������, 1 - ������, 0 - �����)
modbus_registers[427]=(signed char)(tempS);

tempS=bat[0]._av;
#ifdef UKU_220_IPS_TERMOKOMPENSAT 
tempS=ips_bat_av_stat[0];
if(NUMBAT==0)tempS=0xff; //o_10   ������� ��� � SNMP  255-�� �������
#endif

modbus_registers[428]=(signed char)(tempS>>8);				//���215	���� ������� ������� �1(0x01 - ������ ���� �������, 0x02 - ������ ������� ����� �������)
modbus_registers[429]=(signed char)(tempS);

tempS=bat[1]._av;
//o_10_s
#ifdef UKU_220_IPS_TERMOKOMPENSAT
tempS=ips_bat_av_stat[1];
if(NUMBAT<2)tempS=0xff;  //������� ��� � SNMP   255-�� �������
#endif
//o_10_e
modbus_registers[430]=(signed char)(tempS>>8);				//���216	���� ������� ������� �2(0x01 - ������ ���� �������, 0x02 - ������ ������� ����� �������)
modbus_registers[431]=(signed char)(tempS);

tempS=bat_hndl_t_razr_min[0];
modbus_registers[432]=(signed char)(tempS>>8);				//���217	���������� ����� ������ ������� 1 � �������
modbus_registers[433]=(signed char)(tempS);

modbus_registers[434]=(signed char)(snmp_bat_flag[0]>>8);	//���218 //  //����� ��� �1
modbus_registers[435]=(signed char)(snmp_bat_flag[0]);	
modbus_registers[436]=(signed char)(snmp_bat_flag[1]>>8);	//���219 //  //����� ��� �2
modbus_registers[437]=(signed char)(snmp_bat_flag[1]);
/*
��� 0- ����� 1, ���� ���������� �� ��� ���� ������� U����., ����� ����� ����.
��� 1- ����� 1, ���� ��������� ������� ����������� ��� ���� ������� t ���.����., ����� ����� ����. 
��� 2- ����� 1, ���� ��������� ������� ����������� ��� ���� ������� t ���.���., ����� ����� ����.
��� 3- ����� 1, ���� ��� ��� ������ ���� (��� �����������), ����� ����� ����.
��� 4- ����� 1, ���� �������� ������� �������� ������� ���, ����� ����� ����.
��� 5- ����� 1, ���� ������� ������������� ����� ���, ����� ����� ����.
��� 6- ����� 1, ���� ����� �������������� ������ ������������.
��� 7- ����� 1, ���� ������� ���������� ����� ���, ����� ����� ����.
��� 8- ����� 1, ���� ����� ����������� ������ ������������.
��� 9- ����� 1, ���� ������� ������������� ����� ���, ����� ����� ����.
��� 10- ����� 1, ���� ����� �������������� ������ ������������.
��� 11- ����� 1, ���� ������� ����������� ����� ���, ����� ����� ����.
��� 12- ����� 1, ���� ����� ������������ ������ ������������.
*/

modbus_registers[438]=(signed char)(bps[0]._vent_resurs>>8);	//���220 ������ ����������� ���1
modbus_registers[439]=(signed char)(bps[0]._vent_resurs);
modbus_registers[440]=(signed char)(bps[1]._vent_resurs>>8);	//���221 ������ ����������� ���2
modbus_registers[441]=(signed char)(bps[1]._vent_resurs);
modbus_registers[442]=(signed char)(bps[2]._vent_resurs>>8);	//���222 ������ ����������� ���3
modbus_registers[443]=(signed char)(bps[2]._vent_resurs);
modbus_registers[444]=(signed char)(bps[3]._vent_resurs>>8);	//���223 ������ ����������� ���4
modbus_registers[445]=(signed char)(bps[3]._vent_resurs);
modbus_registers[446]=(signed char)(bps[4]._vent_resurs>>8);	//���224 ������ ����������� ���5
modbus_registers[447]=(signed char)(bps[4]._vent_resurs);
modbus_registers[448]=(signed char)(bps[5]._vent_resurs>>8);	//���225 ������ ����������� ���6
modbus_registers[449]=(signed char)(bps[5]._vent_resurs);
modbus_registers[450]=(signed char)(bps[6]._vent_resurs>>8);	//���226 ������ ����������� ���7
modbus_registers[451]=(signed char)(bps[6]._vent_resurs);
modbus_registers[452]=(signed char)(bps[7]._vent_resurs>>8);	//���227 ������ ����������� ���8
modbus_registers[453]=(signed char)(bps[7]._vent_resurs); 
//o_10_s

tempS=bat_hndl_t_razr_min[1];
modbus_registers[454]=(signed char)(tempS>>8);				//���228	���������� ����� ������ ������� 2 � �������
modbus_registers[455]=(signed char)(tempS);

modbus_registers[598]=0;  	
if(no_rki==NO_RKI) modbus_registers[599]=0;				//���300 ���� 0, �� ��� ����� � ���
else modbus_registers[599]=1;							//���300 ���� 1, �� ���� ����� � ���			

modbus_registers[600]=0;								//���301 ������ ����� ���		
modbus_registers[601]=ver_soft;
modbus_registers[602]=0;								//���302 ��� ���		
modbus_registers[603]=type_rki;

modbus_registers[604]=0;								//���303 ������� ���������� ���		
if(u_rki==1) modbus_registers[605]=48;
else if(u_rki==2) modbus_registers[605]=110;
else modbus_registers[605]=220;

modbus_registers[606]=(signed char)(status_izm_r>>8);	//���304 ������ ��������� 
modbus_registers[607]=(signed char)(status_izm_r);
	
modbus_registers[608]=(signed char)(r_iz_plus>>8);		//���305 ������������� �������� �������������� ������
modbus_registers[609]=(signed char)(r_iz_plus);		
modbus_registers[610]=(signed char)(r_iz_minus>>8);		//���306 ������������� �������� �������������� ������
modbus_registers[611]=(signed char)(r_iz_minus);
modbus_registers[612]=0;								//���307 ���������� � %
modbus_registers[613]=(signed char)(asymmetry);
modbus_registers[614]=(signed char)(u_asymmetry>>8);	//���308 ���������� � �������
modbus_registers[615]=(signed char)(u_asymmetry);
modbus_registers[616]=(signed char)(v_plus>>8);			//���309 U+
modbus_registers[617]=(signed char)(v_plus); 
modbus_registers[618]=(signed char)(v_minus>>8);		//���310 U-
modbus_registers[619]=(signed char)(v_minus);
modbus_registers[620]=(signed char)(Ubus>>8);			//���311 U����
modbus_registers[621]=(signed char)(Ubus);

#ifdef UKU_FSO
modbus_registers[600]=(signed char)(lakb[0]._tot_bat_volt>>8);			//���301	���������� ������� �1, �
modbus_registers[601]=(signed char)(lakb[0]._tot_bat_volt);
modbus_registers[602]=(signed char)(lakb[0]._ch_curr>>8);				//���302	��� ������� �1, �
modbus_registers[603]=(signed char)(lakb[0]._ch_curr);
modbus_registers[604]=(signed char)(lakb[0]._cell_temp_1>>8);			//���303	������������ ������� ������� ������� �1, �
modbus_registers[605]=(signed char)(lakb[0]._cell_temp_1);
modbus_registers[606]=(signed char)(lakb[0]._cell_temp_2>>8);			//���304	������������ ������� ������� ������� �1, �
modbus_registers[607]=(signed char)(lakb[0]._cell_temp_2);
modbus_registers[608]=(signed char)(lakb[0]._cell_temp_3>>8);			//���305	������������ �������� ������� ������� �1, �
modbus_registers[609]=(signed char)(lakb[0]._cell_temp_3);
modbus_registers[610]=(signed char)(lakb[0]._cell_temp_4>>8);			//���306	������������ ���������� ������� ������� �1, �
modbus_registers[611]=(signed char)(lakb[0]._cell_temp_4);
modbus_registers[612]=(signed char)(lakb[0]._s_o_h>>8);					//���307	������� ������� �1, 0.01�*�
modbus_registers[613]=(signed char)(lakb[0]._s_o_h);
modbus_registers[614]=(signed char)(lakb[0]._s_o_c>>8);					//���308	����� ������� �1, 0.01�*�
modbus_registers[615]=(signed char)(lakb[0]._s_o_c);
modbus_registers[616]=(signed char)(lakb[0]._s_o_c_percent>>8);			//���309	����� ������� �1, %
modbus_registers[617]=(signed char)(lakb[0]._s_o_c_percent);
modbus_registers[618]=(signed char)(lakb[0]._tot_bat_volt>>8);			//���310	�������������� ����� ������� ������� �1, ���
modbus_registers[619]=(signed char)(lakb[0]._tot_bat_volt);

modbus_registers[620]=(signed char)(lakb[1]._tot_bat_volt>>8);			//���311	���������� ������� �1, �
modbus_registers[621]=(signed char)(lakb[1]._tot_bat_volt);
modbus_registers[622]=(signed char)(lakb[1]._ch_curr>>8);				//���312	��� ������� �1, �
modbus_registers[623]=(signed char)(lakb[1]._ch_curr);
modbus_registers[624]=(signed char)(lakb[1]._cell_temp_1>>8);			//���313	������������ ������� ������� ������� �1, �
modbus_registers[625]=(signed char)(lakb[1]._cell_temp_1);
modbus_registers[626]=(signed char)(lakb[1]._cell_temp_2>>8);			//���314	������������ ������� ������� ������� �1, �
modbus_registers[627]=(signed char)(lakb[1]._cell_temp_2);
modbus_registers[628]=(signed char)(lakb[1]._cell_temp_3>>8);			//���315	������������ �������� ������� ������� �1, �
modbus_registers[629]=(signed char)(lakb[1]._cell_temp_3);
modbus_registers[630]=(signed char)(lakb[1]._cell_temp_4>>8);			//���316	������������ ���������� ������� ������� �1, �
modbus_registers[631]=(signed char)(lakb[1]._cell_temp_4);
modbus_registers[632]=(signed char)(lakb[1]._s_o_h>>8);					//���317	������� ������� �1, 0.01�*�
modbus_registers[633]=(signed char)(lakb[1]._s_o_h);
modbus_registers[634]=(signed char)(lakb[1]._s_o_c>>8);					//���318	����� ������� �1, 0.01�*�
modbus_registers[635]=(signed char)(lakb[1]._s_o_c);
modbus_registers[636]=(signed char)(lakb[1]._s_o_c_percent>>8);			//���319	����� ������� �1, %
modbus_registers[637]=(signed char)(lakb[1]._s_o_c_percent);
modbus_registers[638]=(signed char)(lakb[1]._tot_bat_volt>>8);			//���320	�������������� ����� ������� ������� �1, ���
modbus_registers[639]=(signed char)(lakb[1]._tot_bat_volt);

modbus_registers[660]=(signed char)(fso_vent_valid_cntrl_stat[0]>>8);	//���331	����������� ������� �����������( 1 - ������)
modbus_registers[661]=(signed char)(fso_vent_valid_cntrl_stat[0]);
modbus_registers[662]=(signed char)(fso_vent_valid_cntrl_stat[1]>>8);	//���332	����������� ������� �����������( 1 - ������)
modbus_registers[663]=(signed char)(fso_vent_valid_cntrl_stat[1]);


#endif //UKU_FSO

//o_10_e
if(prot==MODBUS_RTU_PROT)
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
else if(prot==MODBUS_TCP_PROT)
	{
	mem_copy((signed char*)modbus_tx_buff,(signed char*)&modbus_registers[(reg_adr-1)*2],reg_quantity*2);
	modbus_tcp_out_ptr=(signed char*)modbus_tx_buff;
	}
}

