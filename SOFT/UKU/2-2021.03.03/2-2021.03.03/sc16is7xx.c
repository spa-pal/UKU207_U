
#include "sc16is7xx.h"
#include <LPC17xx.H>
#include "main.h"
#include "modbus.h"

char sc16is700ByteAvailable;
char sc16is700TxFifoLevel;
char tx_buffer_sc16is700[TX_BUFFER_SIZE_SC16IS700];//����������� ����� ��������
char tx_wr_index_sc16is700;//��������� ������ � ����������� ����� ��������
char tx_rd_index_sc16is700;//��������� ������ �� ������������ ������ ��������
char sc16is700TxFifoEmptyCnt; //��������� ������� ����������� ���� ��������
char sc16is700TxPossibleFlag;//���� ����������� ��������
char sc16is700RecieveDisableFlag;


char sc16is700_spi_init_cnt;

//----------------------------------------------- 
//��������� SPI1
void sc16is700_spi_init(void)
{
SET_REG( LPC_PINCON->PINSEL0, 0, 0*2, 2);
SET_REG( LPC_PINCON->PINSEL0, 3, 15*2, 2);
SET_REG( LPC_PINCON->PINSEL1, 0, (16-16)*2, 2);
SET_REG( LPC_PINCON->PINSEL1, 3, (17-16)*2, 2);
SET_REG( LPC_PINCON->PINSEL1, 3, (18-16)*2, 2); 
/*
S1SPCCR=100;
S1SPCR=0x3f; */

LPC_SPI->SPCCR=40;
LPC_SPI->SPCR=0x20;
}

//----------------------------------------------- 
//�������� num ���� �� ������������ ������ �������� � sc16is700
void sc16is700_wr_buff(char reg_num,char num)
{
short i;
sc16is700_spi_init();
delay_us(2);
sc16is700_CS_ON 
spi1((reg_num&0x0f)<<3);
for (i=0;i<num;i++)spi1(tx_buffer_sc16is700[i]);
sc16is700_CS_OFF
}

void sc16is700_wr_byte(char reg_num,char data)
{
sc16is700_spi_init();
delay_us(2);
sc16is700_CS_ON
spi1((reg_num&0x0f)<<3);
spi1(data);
sc16is700_CS_OFF
}


char sc16is700_rd_byte(char reg_num)
{
char out;
sc16is700_spi_init();
delay_us(2);
sc16is700_CS_ON
spi1(((reg_num&0x0f)<<3)|0x80);
out = spi1(0xff);
sc16is700_CS_OFF
return out;
}

void sc16is700_init_(void)
{
sc16is700_wr_byte(CS16IS7xx_LCR, 0x80);
sc16is700_wr_byte(CS16IS7xx_DLL, 0x41);
sc16is700_wr_byte(CS16IS7xx_DLH, 0x00);
sc16is700_wr_byte(CS16IS7xx_LCR, 0xBF);
sc16is700_wr_byte(CS16IS7xx_EFR, 0X10);
sc16is700_wr_byte(CS16IS7xx_LCR, 0x03);
sc16is700_wr_byte(CS16IS7xx_FCR, 0x06);
sc16is700_wr_byte(CS16IS7xx_FCR, 0x01);
sc16is700_wr_byte(CS16IS7xx_EFCR, 0X30);
//sc16is700_wr_byte(CS16IS7xx_DLH, 0x04);
//sc16is700_wr_byte(CS16IS7xx_DLH, 0x04);
//sc16is700_wr_byte(CS16IS7xx_DLH, 0x04);
//sc16is700_wr_byte(CS16IS7xx_DLH, 0x04);
//sc16is700_wr_byte(CS16IS7xx_DLH, 0x04);
}



void sc16is700_init(uint32_t baudrate)
{

unsigned char baud_h,baud_l;

baud_h = (char)((10000000U/16U/baudrate)>>8);
baud_l = (char)((10000000U/16U/baudrate));

if(RS485_QWARZ_DIGIT==40)
	{
	baud_h = (char)((40000000U/16U/baudrate)>>8);
	baud_l = (char)((40000000U/16U/baudrate));
	} 
if(RS485_QWARZ_DIGIT==30)
	{
	baud_h = (char)((30000000U/16U/baudrate)>>8);
	baud_l = (char)((30000000U/16U/baudrate));
	}
 

sc16is700_wr_byte(CS16IS7xx_LCR, 0x80);
sc16is700_wr_byte(CS16IS7xx_DLL, baud_l);
sc16is700_wr_byte(CS16IS7xx_DLH, baud_h);
sc16is700_wr_byte(CS16IS7xx_LCR, 0xBF);
sc16is700_wr_byte(CS16IS7xx_EFR, 0X10);
sc16is700_wr_byte(CS16IS7xx_LCR, 0x03);
sc16is700_wr_byte(CS16IS7xx_FCR, 0x06);
sc16is700_wr_byte(CS16IS7xx_FCR, 0x01);
sc16is700_wr_byte(CS16IS7xx_EFCR, 0X30);
//sc16is700_wr_byte(CS16IS7xx_DLH, 0x04);
//sc16is700_wr_byte(CS16IS7xx_DLH, 0x04);
//sc16is700_wr_byte(CS16IS7xx_DLH, 0x04);
//sc16is700_wr_byte(CS16IS7xx_DLH, 0x04);
//sc16is700_wr_byte(CS16IS7xx_DLH, 0x04);
}

/*
Void SC16IS752_Init_ChA (void) // program channel A for SPI-UART
{ // set 115200 baud, 8N1
SPI_wr_752 (LCR, 0x80, 0); // 0x80 to program baud rate
SPI_wr_752 (DLL, 0x08, 0); // 0x08 = 115.2K with X1 = 14.7456 MHz
SPI_wr_752 (DLM, 0x00, 0); // divisor = 0x0008 for 115200 bps
SPI_wr_752 (LCR, 0xBF, 0); // access EFR register
SPI_wr_752 (EFR, 0X10, 0); // enable enhanced registers
SPI_wr_752 (LCR, 0x03, 0); // 8 data bit, 1 stop bit, no parity
SPI_wr_752 (FCR, 0x01, 0); // enable FIFO mode
SPI_wr_752 (SPR, 'A', 0); // scratch pad = character A (0x41)
SPI_wr_752 (IODIR, 0xFF, 0); // set GPIO [7:0] to output
// (default: 0x00=input)
SPI_wr_752 (IOSTATE, 0x00, 0); // set GPIO [7:0] to 0x00 (LEDs on)
SPI_wr_752 (IER, 0x01, 0); // enable Rx data ready interrupt
}*/

//----------------------------------------------- 
//������� ����� ����� sc16is700
void putchar_sc16is700(char out_byte)
{
tx_buffer_sc16is700[tx_wr_index_sc16is700]=out_byte;
if (++tx_wr_index_sc16is700 == TX_BUFFER_SIZE_SC16IS700) tx_wr_index_sc16is700=0;
}


//----------------------------------------------- 
//���������� sc16is700
void sc16is700_uart_hndl(void)
{

sc16is700ByteAvailable=sc16is700_rd_byte(CS16IS7xx_RXLVL); //������ ��������� ���� ������ ����������

if(sc16is700ByteAvailable) //���� � �������� ����	���������� ���� ������
	{
	char i;
	for(i=0;(i<sc16is700ByteAvailable)&&(i<5);i++) //������ �� ������� �� ������ 5 � ����������� ����� ������
		{
		if(!sc16is700RecieveDisableFlag)
			{
			modbus_rx_buffer[modbus_rx_buffer_ptr]=sc16is700_rd_byte(CS16IS7xx_RHR);
			modbus_rx_buffer_ptr++;
			modbus_timeout_cnt=0;   //��������� ������ ����������� ����� ������� 
			//pvlk++;
			}
		else sc16is700_rd_byte(CS16IS7xx_RHR);
		}
	}



sc16is700TxFifoLevel=sc16is700_rd_byte(CS16IS7xx_TXLVL);//������ ��������� ���� ��������

if(sc16is700TxFifoLevel!=64) sc16is700TxFifoEmptyCnt=0;//���� ���� �� ������ �������� ������� ����������� ���� ��������
if(sc16is700TxFifoLevel==64) //���� ���� ������ �� ������� ������� ���� �� ������ ���������
	{
	if(sc16is700TxFifoEmptyCnt<SC16IS700TXFIFOEMPTYCNTMAX)sc16is700TxFifoEmptyCnt++;
	}
if(sc16is700TxFifoEmptyCnt==SC16IS700TXFIFOEMPTYCNTMAX) sc16is700TxPossibleFlag=1;//���� ������� ��������� � ���������� ��������� ���� ����������� ��������
else sc16is700TxPossibleFlag=0;//���� �� ��������� - ���� ����������.


if((tx_wr_index_sc16is700)&&(tx_wr_index_sc16is700!=tx_rd_index_sc16is700)) //���� ����������� ����� �������� �� ����
	{
	if(sc16is700TxPossibleFlag)//��������� ����������� ��������
		{
		//char i;
		//for(;tx_rd_index_sc16is700++;tx_rd_index_sc16is700<=tx_wr_index_sc16is700)
			//{
		sc16is700RecieveDisableFlag=1;
		sc16is700_wr_buff(CS16IS7xx_THR, tx_wr_index_sc16is700);
			//}
		tx_wr_index_sc16is700=0;
		}
	}

if((sc16is700_rd_byte(CS16IS7xx_LSR))&0x40)	sc16is700RecieveDisableFlag=0;


}
