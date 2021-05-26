
#include "stdint.h"
#include "uart0.h"
#include "LPC17XX.H" 
#include "uart2.h"
#include "main.h"
#include "control.h"
#include "cmd.c"
#include "eeprom_map.h"
#include "gran.h"
#include "25lc640.h"
#include "common_func.h"
#include "modbus.h"

char bRXIN2;
char UIB2[100]={0,0,0,0,0,0,0,0,0,0};
char UIB20[30]={0,0,0,0,0,0,0,0,0,0};
char flag2;
char rx_buffer2[RX_BUFFER_SIZE2];
char tx_buffer2[TX_BUFFER_SIZE2];
unsigned short rx_wr_index2,rx_rd_index2,rx_counter2;
unsigned short tx_wr_index2,tx_rd_index2,tx_counter2;
char rx_buffer_overflow2;
char plazma_uart2;
char memo_out2[50];
char data_rs2[50];
char data_rs02[50];

volatile uint32_t UART2Status;
volatile uint8_t UART2TxEmpty = 1;
//volatile uint8_t UART0Buffer[BUFSIZE];
volatile uint32_t UART2Count = 0;



//-----------------------------------------------
void putchar2(char c)
{
while (tx_counter2 == TX_BUFFER_SIZE2);
if (tx_counter2 || ((LPC_UART2->LSR & 0x60)==0))
   {
   tx_buffer2[tx_wr_index2]=c;
   if (++tx_wr_index2 == TX_BUFFER_SIZE2) tx_wr_index2=0;
   ++tx_counter2;
   }
else 
	{
	LPC_PINCON->PINSEL4 &= ~0x0000c000;//!!!!!!!!!!!
	LPC_PINCON->PINSEL4 |= 0x00000000; //!!!!!!!!!!! 
	LPC_GPIO2->FIODIR|=(1UL<<7);
	LPC_GPIO2->FIOPIN|=(1UL<<7);
	LPC_UART2->THR=c;
	}
}

//-----------------------------------------------
void uart_out2 (char num,char data0,char data1,char data2,char data3,char data4,char data5)
{
char i,t=0;
//char *ptr=&data1;
char UOB2[16]; 
UOB2[0]=data0;
UOB2[1]=data1;
UOB2[2]=data2;
UOB2[3]=data3;
UOB2[4]=data4;
UOB2[5]=data5;

for (i=0;i<num;i++)
	{
	t^=UOB2[i];
	}    
UOB2[num]=num;
t^=UOB2[num];
UOB2[num+1]=t;
UOB2[num+2]=END;

for (i=0;i<num+3;i++)
	{
	putchar2(UOB2[i]);
	}   	
}

//-----------------------------------------------
void uart_out_adr2 (char *ptr, char len)
{
char UOB2[100];
char i,t=0;

for(i=0;i<len;i++)
	{
	UOB2[i]=ptr[i];
	t^=UOB2[i];
	}
//if(!t)t=0xff;
UOB2[len]=len;
t^=len;	
UOB2[len+1]=t;	
UOB2[len+2]=END;
	
for (i=0;i<len+3;i++)
	{
	putchar2(UOB2[i]);
	}   
}


//-----------------------------------------------
void uart_out_adr2_block (unsigned long adress,char *ptr, char len)
{
//char UOB2[100]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
char /*i,*/temp11,t=0;
unsigned i11;

t=0;
temp11=CMND;
t^=temp11;
putchar2(temp11);

temp11=10;
t^=temp11;
putchar2(temp11);

temp11=(*((char*)&adress));
t^=temp11;
putchar2(temp11);

temp11=(*(((char*)&adress)+1));
t^=temp11;
putchar2(temp11);

temp11=(*(((char*)&adress)+2));
t^=temp11;
putchar2(temp11);

temp11=(*(((char*)&adress)+3));
t^=temp11;
putchar2(temp11);


for(i11=0;i11<len;i11++)
	{
	temp11=ptr[i11];
	t^=temp11;
	putchar2(temp11);
	}
	
temp11=(len+6);
t^=temp11;
putchar2(temp11);

putchar2(t);

putchar2(0x0a);
	
}


//-----------------------------------------------
uint32_t UART_2_Init(uint32_t baudrate )
{
uint32_t Fdiv;
uint32_t pclkdiv, pclk;

LPC_PINCON->PINSEL4 &= ~0x000f0000;//!!!!!!!!!!!
LPC_PINCON->PINSEL4 |= 0x000a0000; //!!!!!!!!!!! 

	LPC_PINCON->PINSEL4 &= ~0x0000c000;//!!!!!!!!!!!
	LPC_PINCON->PINSEL4 |= 0x00000000; //!!!!!!!!!!! 
	LPC_GPIO2->FIODIR|=(1UL<<7);
	LPC_GPIO2->FIOPIN|=(1UL<<7);

/* RxD0 is P0.3 and TxD0 is P0.2 */
	/* By default, the PCLKSELx value is zero, thus, the PCLK for
	all the peripherals is 1/4 of the SystemFrequency. */
	/* Bit 6~7 is for UART0 */
pclkdiv = (LPC_SC->PCLKSEL0 >> 6) & 0x03;
switch ( pclkdiv )
	{
	case 0x00:
	default:
     pclk = SystemFrequency/4;
	break;
	case 0x01:
	pclk = SystemFrequency;
	break; 
	case 0x02:
	pclk = SystemFrequency/2;
	break; 
	case 0x03:
	pclk = SystemFrequency/8;
	break;
	}

LPC_UART2->LCR = 0x83;		/* 8 bits, no Parity, 1 Stop bit */
/*
#ifdef UKU_ZVU
		if(MODBUS_PARITY==0) 		
			{
			char temp;
			temp=LPC_UART2->LCR;
			temp&=0xc7;
			LPC_UART2->LCR=temp;
			}
		else if(MODBUS_PARITY==1) 	
			{
			char temp;
			temp=LPC_UART2->LCR;
			temp&=0xc7;
			temp|=0x08;
			LPC_UART2->LCR=temp;
			}
		else
			{ 
			char temp;
			temp=LPC_UART2->LCR;
			temp&=0xc7;
			temp|=0x18;
			LPC_UART2->LCR=temp;
			}  

#endif
*/
Fdiv = ( pclk / 16 ) / baudrate ;	/*baud rate */
LPC_UART2->DLM = Fdiv / 256;							
LPC_UART2->DLL = Fdiv % 256;
LPC_UART2->LCR = 0x03;		/* DLAB = 0 */
LPC_UART2->FCR = 0x07;		/* Enable and reset TX and RX FIFO. */

NVIC_EnableIRQ(UART2_IRQn);

LPC_UART2->IER = IER_RBR | IER_THRE | IER_RLS;	/* Enable UART2 interrupt */
return (TRUE);
}

//-----------------------------------------------
char getchar2(void)
{
char data;
while (rx_counter2==0);
data=rx_buffer2[rx_rd_index2];
if (++rx_rd_index2 == RX_BUFFER_SIZE2) rx_rd_index2=0;
--rx_counter2;
return data;
}

//***********************************************
void UART2_IRQHandler (void) 
{
uint8_t IIRValue, LSRValue;
uint8_t Dummy = Dummy;
char /*status,u2iir,*/data;

			
IIRValue = LPC_UART2->IIR;
    
IIRValue >>= 1;			/* skip pending bit in IIR */
IIRValue &= 0x07;			/* check bit 1~3, interrupt identification */
if ( IIRValue == IIR_RLS )		/* Receive Line Status */
  	{
	LSRValue = LPC_UART2->LSR;
	/* Receive Line Status */
	
	
	
	if ( LSRValue & (LSR_OE|LSR_PE|LSR_FE|LSR_RXFE|LSR_BI) )
		{
	  	/* There are errors or break interrupt */
	  	/* Read LSR will clear the interrupt */
	  	UART2Status = LSRValue;
	  	Dummy = LPC_UART2->RBR;		/* Dummy read on RX to clear 
							interrupt, then bail out */
	  	return;
		}
	if ( LSRValue & LSR_RDR )	/* Receive Data Ready */			
		{
		
		data=LPC_UART2->RBR;
		rx_buffer2[rx_wr_index2]=data;
   		bRXIN2=1;
   		if (++rx_wr_index2 == RX_BUFFER_SIZE2) rx_wr_index2=0;
   		if (++rx_counter2 == RX_BUFFER_SIZE2)
      		{
      		rx_counter2=0;
      		rx_buffer_overflow2=1;
      		}


		}

  	}
else if ( IIRValue == IIR_RDA )	/* Receive Data Available */
  	{
	plazma_uart2++;
	data=LPC_UART2->RBR;
	rx_buffer2[rx_wr_index2]=data;
   	bRXIN2=1;
   	if (++rx_wr_index2 == RX_BUFFER_SIZE2) rx_wr_index2=0;
   	if (++rx_counter2 == RX_BUFFER_SIZE2)
      	{
      	rx_counter2=0;
      	rx_buffer_overflow2=1;
      	}
	modbus_rx_buffer[modbus_rx_buffer_ptr]=data;
	modbus_rx_buffer_ptr++;
	modbus_timeout_cnt=0;

  	}
else if ( IIRValue == IIR_CTI )	/* Character timeout indicator */
  	{
	/* Character Time-out indicator */
	UART2Status |= 0x100;		/* Bit 9 as the CTI error */
  	}
else if ( IIRValue == IIR_THRE )	/* THRE, transmit holding register empty */
  	{
	/* THRE interrupt */
	
	LSRValue = LPC_UART2->LSR;		/* Check status in the LSR to see if
									valid data in U0THR or not */
	if ( LSRValue & LSR_THRE )
		{
	  	UART2TxEmpty = 1;
		if (tx_counter2)
   			{
   			--tx_counter2;
   			LPC_UART2->THR=tx_buffer2[tx_rd_index2];
   			if (++tx_rd_index2 == TX_BUFFER_SIZE2) tx_rd_index2=0;
   			}
		//else LPC_GPIO2->FIOPIN&=~(1UL<<7);
		}
	else
		{
	  	UART2TxEmpty = 0;
		}
  	}
}



//-----------------------------------------------
void uart_in_an2(void)
{



}



//-----------------------------------------------
signed short index_offset2 (signed short index,signed short offset)
{
index=index+offset;
if(index>=RX_BUFFER_SIZE2) index-=RX_BUFFER_SIZE2; 
if(index<0) index+=RX_BUFFER_SIZE2;
return index;
}

//-----------------------------------------------
char control_check2(signed short index)
{
char i=0,ii=0,iii;

if(rx_buffer2[index]!=END) goto error_cc;

ii=rx_buffer2[index_offset2(index,-2)];
iii=0;
for(i=0;i<=ii;i++)
	{
	iii^=rx_buffer2[index_offset2(index,-2-ii+i)];
	}
if (iii!=rx_buffer2[index_offset2(index,-1)]) goto error_cc;	


//success_cc:
return 1;
error_cc:
return 0;

}

//-----------------------------------------------
void uart_in2(void)
{
char temp,i;



if(rx_buffer_overflow2)
	{
	rx_wr_index2=0;
	rx_rd_index2=0;
	rx_counter2=0;
	rx_buffer_overflow2=0;
	}    
	
if(rx_counter2&&(rx_buffer2[index_offset2(rx_wr_index2,-1)])==END)
	{
	
     //plazma_uart2++;

     temp=rx_buffer2[index_offset2(rx_wr_index2,-3)];
    	if(temp<100) 
    		{
    		if(control_check2(index_offset2(rx_wr_index2,-1)))
    			{
    		
    			rx_rd_index2=index_offset2(rx_wr_index2,-3-temp);
    			for(i=0;i<temp;i++)
				{
				UIB2[i]=rx_buffer2[index_offset2(rx_rd_index2,i)];
				} 
			rx_rd_index2=rx_wr_index2;
			rx_counter2=0;
			
			uart_in_an2();
    		}
 	
    	} 
    }	
}