
#ifndef UART_H_
#define UART_H_

#include "xparameters.h"
#include "xuartps.h"
#include "xil_printf.h"
#include "sleep.h"
#include "xscugic.h"
#include "xscutimer.h"


#ifndef TESTAPP_GEN
#define TIMER_DEVICE_ID		XPAR_XSCUTIMER_0_DEVICE_ID
#define INTC_DEVICE_ID		XPAR_SCUGIC_SINGLE_DEVICE_ID
#define TIMER_IRPT_INTR		XPAR_SCUTIMER_INTR
#endif




#define UART_DEVICE_ID     XPAR_PS7_UART_1_DEVICE_ID
#define UART_INT_IRQ_ID    XPAR_XUARTPS_1_INTR
#define INTC_DEVICE_ID	   XPAR_SCUGIC_SINGLE_DEVICE_ID

extern unsigned int sync_flg;
extern unsigned int recv_finish_flg;
//////////////
//static void T_intr_Handler(void *param);
//static void R_intr_Handler(void *param);
void IntcTypeSetup(XScuGic *InstancePtr, int intId, int intType);
int IntcInitFunction(XScuGic *INTCInstPtr);
////////////





#endif /* SRC_USER_UART_H_ */

