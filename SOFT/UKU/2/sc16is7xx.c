
#include "sc16is7xx.h"
#include <LPC17xx.H>
#include "main.h"


//----------------------------------------------- 
//настройка SPI1
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

LPC_SPI->SPCCR=20;
LPC_SPI->SPCR=0x38;
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


void sc16is700_init(void)
{
sc16is700_wr_byte(CS16IS7xx_LCR, 0x80);
sc16is700_wr_byte(CS16IS7xx_DLL, 0x12);
sc16is700_wr_byte(CS16IS7xx_DLH, 0x04);
sc16is700_wr_byte(CS16IS7xx_LCR, 0xBF);
sc16is700_wr_byte(CS16IS7xx_EFR, 0X00);
sc16is700_wr_byte(CS16IS7xx_LCR, 0x03);
sc16is700_wr_byte(CS16IS7xx_FCR, 0x01);
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

