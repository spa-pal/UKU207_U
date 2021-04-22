#include "stdint.h"

#define CS16IS7xx_RHR	0x00
#define CS16IS7xx_THR	0x00
#define CS16IS7xx_IER	0x01
#define CS16IS7xx_IIR	0x02
#define CS16IS7xx_FCR	0x02
#define CS16IS7xx_LCR	0x03
#define CS16IS7xx_MCR	0x04
#define CS16IS7xx_LSR	0x05
#define CS16IS7xx_SPR	0x07
#define CS16IS7xx_TXLVL	0x08
#define CS16IS7xx_RXLVL	0x09
#define CS16IS7xx_EFCR	0x0f


#define CS16IS7xx_DLL	0x00
#define CS16IS7xx_DLH	0x01
#define CS16IS7xx_EFR	0x02

#define sc16is700_CS_ON  LPC_GPIO0->FIODIR|=(1<<0);LPC_GPIO0->FIOCLR|=(1<<0);
#define sc16is700_CS_OFF LPC_GPIO0->FIODIR|=(1<<0);LPC_GPIO0->FIOSET|=(1<<0);

//������ ������������ ������ ��������
#define TX_BUFFER_SIZE_SC16IS700	32

//������������ � ������������� ����������� ���� �������� ��� ����������� ������ ����� ��������
#define SC16IS700TXFIFOEMPTYCNTMAX		5

extern char sc16is700ByteAvailable;
extern char sc16is700TxFifoLevel;
extern char tx_buffer_sc16is700[TX_BUFFER_SIZE_SC16IS700]; //����������� ����� ��������
extern char tx_wr_index_sc16is700;//��������� ������ � ����������� ����� ��������
extern char tx_rd_index_sc16is700;//��������� ������ �� ������������ ������ ��������
extern char sc16is700TxFifoEmptyCnt; //��������� ������� ����������� ���� ��������
extern char sc16is700TxPossibleFlag;//���� ����������� ��������

extern char sc16is700_spi_init_cnt;


void sc16is700_init(uint32_t baudrate);
void sc16is700_wr_byte(char reg_num,char data);
char sc16is700_rd_byte(char reg_num);
//----------------------------------------------- 
//�������� num ���� �� ������������ ������ �������� � sc16is700
void sc16is700_wr_buff(char reg_num,char num);
void putchar_sc16is700(char out_byte);
void sc16is700_uart_hndl(void);

