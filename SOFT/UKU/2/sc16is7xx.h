#define sc16is700_CS_ON  LPC_GPIO2->FIODIR|=(1<<7);LPC_GPIO2->FIOCLR|=(1<<7);
#define sc16is700_CS_OFF LPC_GPIO2->FIODIR|=(1<<7);LPC_GPIO2->FIOSET|=(1<<7);




void sc16is700_init(void);
void sc16is700_wr_byte(char reg_num,char data);