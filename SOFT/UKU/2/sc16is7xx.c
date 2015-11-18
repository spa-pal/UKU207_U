
#include "sc16is7xx.h"
#include <LPC17xx.H>
#include "main.h"


//----------------------------------------------- 
//настройка SPI1
void sc16is700_spi_init(void)
{
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
delay_us(10);
sc16is700_CS_ON
spi1((reg_num&0x0f)<<3);
spi1(data);
}


void sc16is700_init(void)
{
sc16is700_wr_byte(0x01,0x02);
}