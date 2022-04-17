
#include "uart.h"
#include "adc_core.h"
#include "dac_core.h"
#include "command.h"
#include "sleep.h"
///////////////////////
#define INT_CFG0_OFFSET 0x00000C00

// Parameter definitions
#define SW1_INT_ID              88  //dac
#define SW2_INT_ID              89  //adc
#define IPPS_INT_ID             61
#define CT_INT_ID               62
#define INT_TYPE_RISING_EDGE    0x03
#define INT_TYPE_HIGHLEVEL      0x01
#define INT_TYPE_MASK           0x03

#define  MARK_LENGTH      0x488
#define  PPS_CNT          0x48C
#define  CT_CNT_START     0x490
#define  CT_CNT_STOP      0x494
static XScuGic INTCInst;
unsigned int recv_finish_flg=0;
unsigned int sync_flg = 0;
static void T_intr_Handler(void *param)
{
	//printf("T_intr_Handler\n");
	uint32_t reg_val;
	dac_dma_read(AXI_DMAC_REG_IRQ_PENDING, &reg_val);
	dac_dma_write(AXI_DMAC_REG_IRQ_PENDING, reg_val);
	if(reg_val & IRQ_TRANSFER_QUEUED)
	{
	//dac_dma_write(AXI_DMAC_REG_START_TRANSFER, 0x1);
		//printf("T_TRANSFER_QUEUED\n");
	}
	if(reg_val & IRQ_TRANSFER_COMPLETED)
	{
		dac_dma_read(AXI_DMAC_REG_TRANSFER_ID, &reg_val);
		//printf("NEST_TRANSFER_ID = %x   \n",reg_val);

		//printf("T_TRANSFER_COMPLETED\n");
	}
}

static void R_intr_Handler(void *param)
{
	//printf("R_intr_Handler\n");
	uint32_t reg_val;
	static uint8_t index=0;
	adc_dma_read(AXI_DMAC_REG_IRQ_PENDING, &reg_val);
	adc_dma_write(AXI_DMAC_REG_IRQ_PENDING, reg_val);
	if(reg_val & IRQ_TRANSFER_QUEUED)
	{
	//dac_dma_write(AXI_DMAC_REG_START_TRANSFER, 0x1);
		//printf("R_TRANSFER_QUEUED\n");
	}
	if(reg_val & IRQ_TRANSFER_COMPLETED)
	{
		adc_dma_read(AXI_DMAC_REG_TRANSFER_ID, &reg_val);
		//printf("NEST_TRANSFER_ID = %x   \n",reg_val);
	   // printf("R_TRANSFER_COMPLETED\n");
		recv_finish_flg=1;

	}
}
static void IPPS_intr_Handler(void *param)
{
	printf("1ppS\n");
	unsigned int data=0;
	adc_dma_read(PPS_CNT, &data);
}

static void CT_intr_Handler(void *param)
{
	unsigned int data=0;
	printf("CT\n");
	sync_flg++;
	adc_dma_read(PPS_CNT, &data);
	adc_dma_read(CT_CNT_START, &data);
	//printf("ct_cnt_start = %d \n",data);
	adc_dma_read(CT_CNT_STOP, &data);
	//printf("ct_cnt_stop = %d \n",data);
	adc_dma_read(MARK_LENGTH, &data);
	//printf("mark_length = %d \n",data);
}





void IntcTypeSetup(XScuGic *InstancePtr, int intId, int intType)
{
    int mask;

    intType &= INT_TYPE_MASK;
    mask = XScuGic_DistReadReg(InstancePtr, INT_CFG0_OFFSET + (intId/16)*4);
    mask &= ~(INT_TYPE_MASK << (intId%16)*2);
    mask |= intType << ((intId%16)*2);
    XScuGic_DistWriteReg(InstancePtr, INT_CFG0_OFFSET + (intId/16)*4, mask);
}

int IntcInitFunction(XScuGic *INTCInstPtr)
{
    XScuGic_Config *IntcConfig;
    int status;

    // Interrupt controller initialisation
    IntcConfig = XScuGic_LookupConfig(XPAR_PS7_SCUGIC_0_DEVICE_ID);
    status = XScuGic_CfgInitialize(INTCInstPtr, IntcConfig, IntcConfig->CpuBaseAddress);
    if(status != XST_SUCCESS) return XST_FAILURE;

    // Call to interrupt setup
    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
                                 (Xil_ExceptionHandler)XScuGic_InterruptHandler,
                                 INTCInstPtr);
    Xil_ExceptionEnable();

    // Connect SW1~SW3 interrupt to handler
    status = XScuGic_Connect(INTCInstPtr,
                             SW1_INT_ID,
                             (Xil_ExceptionHandler)T_intr_Handler,
                             (void *)1);
    if(status != XST_SUCCESS) return XST_FAILURE;

    status = XScuGic_Connect(INTCInstPtr,
                             SW2_INT_ID,
                             (Xil_ExceptionHandler)R_intr_Handler,
                             (void *)2);
    if(status != XST_SUCCESS) return XST_FAILURE;

    status = XScuGic_Connect(INTCInstPtr,
                             IPPS_INT_ID,
                             (Xil_ExceptionHandler)IPPS_intr_Handler,
                             (void *)3);
    if(status != XST_SUCCESS) return XST_FAILURE;

    status = XScuGic_Connect(INTCInstPtr,
                             CT_INT_ID,
                             (Xil_ExceptionHandler)CT_intr_Handler,
                             (void *)4);
    if(status != XST_SUCCESS) return XST_FAILURE;

    // Set interrupt type of SW1~SW3 to rising edge
    IntcTypeSetup(INTCInstPtr, SW1_INT_ID, INT_TYPE_RISING_EDGE);
    IntcTypeSetup(INTCInstPtr, SW2_INT_ID, INT_TYPE_RISING_EDGE);
    IntcTypeSetup(INTCInstPtr, IPPS_INT_ID, INT_TYPE_RISING_EDGE);
    IntcTypeSetup(INTCInstPtr, CT_INT_ID, INT_TYPE_RISING_EDGE);
    // Enable SW1~SW3 interrupts in the controller
    XScuGic_Enable(INTCInstPtr, SW1_INT_ID);
    XScuGic_Enable(INTCInstPtr, SW2_INT_ID);//接收中断使能
    XScuGic_Enable(INTCInstPtr, IPPS_INT_ID);
    XScuGic_Enable(INTCInstPtr, CT_INT_ID);

    return XST_SUCCESS;
}
