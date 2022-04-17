#include "swat.h"






#define PSS_RST_CTRL_REG 0xF8000200 //PSS_RST_CTRL寄存器，绝对地址，
#define SLCR_UNLOCK_ADDR 0xF8000008 //SLCR_UNLOCK寄存器，绝对地址，
#define UNLOCK_KEY 0xDF0D //使能码
#define PSS_RST_MASK 0x01 //复位码

void PsSoftwareReset(void)
{
	Xil_Out32(SLCR_UNLOCK_ADDR, UNLOCK_KEY); //写使能
	Xil_Out32(PSS_RST_CTRL_REG, PSS_RST_MASK); //复位
}
/*
int main()
{
	init_platform();
	print("Hello World\r\n");
	sleep(5);
	PsSoftwareReset();
	cleanup_platform();
	return 0;
}
*/





/*
int main(void)
{
	int Status;
	int Count = 0;
	Status = watchdog(&Watchdog, WDT_DEVICE_ID,10);
	if (Status != XST_SUCCESS) {
		xil_printf("start the watchdog timer fail!\r\n");
		return XST_FAILURE;
	}
	while (Count < 5) {
		sleep(1);
		Count++;
		printf("the second is %d \n",Count);
		//XScuWdt_RestartWdt(WdtInstancePtr);
	}
	xil_printf("the watchdog timer will restart the system \r\n");
	return XST_SUCCESS;
}

int watchdog(XScuWdt * WdtInstancePtr, u16 DeviceId,float number)
{
	int Status;
	XScuWdt_Config *ConfigPtr;
	u32 result;

	 xil_printf("start the watchdog time``r successful! \r\n");
	ConfigPtr = XScuWdt_LookupConfig(DeviceId);


	Status = XScuWdt_CfgInitialize(WdtInstancePtr, ConfigPtr,
					ConfigPtr->BaseAddr);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}


	XScuWdt_SetWdMode(WdtInstancePtr);


	result = (unsigned long)(333333333*number);
	XScuWdt_LoadWdt(WdtInstancePtr,result);


	XScuWdt_Start(WdtInstancePtr);

	return XST_SUCCESS;
}
*/
