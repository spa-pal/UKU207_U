#define CS16IS7xx_RHR	0x00
#define CS16IS7xx_THR	0x00
#define CS16IS7xx_IER	0x01
#define CS16IS7xx_IIR	0x02
#define CS16IS7xx_FCR	0x02
#define CS16IS7xx_LCR	0x03
#define CS16IS7xx_MCR	0x04
#define CS16IS7xx_LSR	0x05
#define CS16IS7xx_SPR	0x07
#define CS16IS7xx_EFCR	0x0f


#define CS16IS7xx_DLL	0x00
#define CS16IS7xx_DLH	0x01
#define CS16IS7xx_EFR	0x02

#define sc16is700_CS_ON  LPC_GPIO0->FIODIR|=(1<<0);LPC_GPIO0->FIOCLR|=(1<<0);
#define sc16is700_CS_OFF LPC_GPIO0->FIODIR|=(1<<0);LPC_GPIO0->FIOSET|=(1<<0);




void sc16is700_init(void);
void sc16is700_wr_byte(char reg_num,char data);
char sc16is700_rd_byte(char reg_num);
void sc16is700_wr_buff(char reg_num,char num);