
#include <rtl.h>
#include "main.h"
#include "modbus_tcp.h"

char plazma_modbus_tcp[10];
/*--------------------------- TCP socket ------------------------------------*/

U16 tcp_callback (U8 soc, U8 evt, U8 *ptr, U16 par) {
  /* This function is called by the TCP module on TCP event */
  /* Check the 'Net_Config.h' for possible events.          */
  par = par;

  

  if (soc != socket_tcp) {
    return (0);
  }

  switch (evt) {
    case TCP_EVT_DATA:
      /* TCP data frame has arrived, data is located at *par1, */
      /* data length is par2. Allocate buffer to send reply.   */
      //procrec(ptr);
	  plazma_modbus_tcp[0]++;
	  //plazma_modbus_tcp[1]=ptr[0];
	  //plazma_modbus_tcp[2]=ptr[1];
	  //plazma_modbus_tcp[3]=ptr[2];
	  //plazma_modbus_tcp[4]=ptr[3];
	  plazma_modbus_tcp[5]=par;
	  //plazma_modbus_tcp[6]=ptr[5];
      break;

    case TCP_EVT_CONREQ:
      /* Remote peer requested connect, accept it */
      return (1);

    case TCP_EVT_CONNECT:
      /* The TCP socket is connected */
      return (1);
  }
  return (0);
}