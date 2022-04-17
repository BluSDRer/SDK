/**************************************************************************//**
 *   @file   command.c
 *   @brief  Implementation of AD9361 Command Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
 *******************************************************************************
 * Copyright 2013(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "command.h"
#include "xil_io.h"
#include "console.h"

#include "ad9361_api.h"
#include "dac_core.h"
#include "math.h"
#include "uart.h"
#include "fft.h"
#include "parameters.h"
#define PI 3.141592653589793
#define  TRRIGER_CNTRL 0x43C00038
#define  MODE_CNTROL   0x43C0003C
/******************************************************************************/
/************************ Constants Definitions *******************************/
/******************************************************************************/
command cmd_list[] = {
	{"help?", "Displays all available commands.", "", get_help},
	{"send_trigger=", "send trigger signal.", "", send_trigger},
	{"board_middle_cali=", "board_middle_cali", "", board_middle_cali},
	{"board_iner_cali=", "board_iner_cali", "", board_iner_cali},
	{"phase_compensate=", "phase_compensate", "", phase_compensate},
	{"self_check=", "self_check", "", self_check},
	{"set_mode=", "set_mode", "", set_mode},
	{"set_reset=", "set_reset", "", set_reset},
	{"set_rf_port=", "set_rf_port", "", set_rf_port},
	{"set__attenuation=", "set__attenuation", "", set__attenuation},
	{"set_lo_freq=", "set_lo_freq", "", set_lo_freq},
	{"write_reg=", "write_reg", "", write_reg},
	{"read_reg=", "read_reg", "", read_reg},
	{"set_signal_fre=", "set_signal_fre", "", set_signal_fre},
	{"set_tx_port=", "set_tx_port", "", set_tx_port},
	{"set_rx_port=", "set_rx_port", "", set_rx_port},
	{"get_phase=", "get_phase", "", get_phase},
	{"get_data=", "get_data", "", get_data},
	{"set_step=", "set_step", "", set_step},
	{"set_mgc=", "set_mgc", "", set_mgc},
	{"set_rx_lo_freq=", "set_rx_lo_freq", "", set_rx_lo_freq},
};
const char cmd_no = (sizeof(cmd_list) / sizeof(command));


unsigned int step_set = 0;
unsigned int state_ready=0;
extern struct ad9361_rf_phy *ad9361_phy;
extern struct ad9361_rf_phy *ad9361_phy_b;
/******************************************************************************/
/************************ Variables Definitions *******************************/
/******************************************************************************/

void get_help(double* param, char param_no) // "help?" command
{
      printf("param_no = %d \n",param_no);
}


void send_trigger(double* param, char param_no) //set_trigger
{
	unsigned int trigger_num = param[0];
	Xil_Out32(MODE_CNTROL, 1);//0£ºclose 1:cw 2:adsb 3:dmac
	if(param_no >= 1)
	{
		printf("set_trigger \n");
		mdelay(100);
		Xil_Out32(TRRIGER_CNTRL, 0x3ff);
		mdelay(100);
		Xil_Out32(TRRIGER_CNTRL, 0x000);
		console_print("set_trigger \n");
	}
	else
		printf("set_trigger--invalid_param_message \n");
}


void board_iner_cali(double* param, char param_no) //board_iner_cali
{
	fft_result ad9361_1_2;
	fft_result ad9361_1_3;
	fft_result ad9361_1_4;
	ad9361_set_tx_rf_port_output(ad9361_phy,1);   //TXB
	ad9361_set_tx_rf_port_output(ad9361_phy_b,1); //TXB
	ad9361_set_rx_rf_port_input(ad9361_phy,2);    //RXC
	ad9361_set_rx_rf_port_input(ad9361_phy_b,2);   //RXC
	printf(" set_tx--TXB set_rx--RXC \n");
	Xil_Out32(MODE_CNTROL, 1);//0£ºclose 1:cw 2:adsb 3:dmac
	mdelay(1000);
	ad9361_1_2 = phase_calculation(1,0);
	ad9361_1_3 = phase_calculation(0,1);
	ad9361_1_4 = phase_calculation(1,1);
	console_print("board_iner_cali: %f %f %f \n",ad9361_1_2.phase,ad9361_1_3.phase,ad9361_1_4.phase);
	ad9361_set_tx_rf_port_output(ad9361_phy,0);   //TXA
	ad9361_set_tx_rf_port_output(ad9361_phy_b,0); //TXA
	ad9361_set_rx_rf_port_input(ad9361_phy,0);    //RXA
	ad9361_set_rx_rf_port_input(ad9361_phy_b,0);  //RXA
}
void board_middle_cali(double* param, char param_no) //board_middle_cali
{
	fft_result board_1_2;
	ad9361_set_tx_rf_port_output(ad9361_phy,0);   //TXA
	ad9361_set_tx_rf_port_output(ad9361_phy_b,0); //TXA
	ad9361_set_rx_rf_port_input(ad9361_phy,0);    //RXA
	ad9361_set_rx_rf_port_input(ad9361_phy_b,0);   //RXA
	printf(" set_tx--TXA set_rx--RXA \n");
	Xil_Out32(MODE_CNTROL, 1);//0£ºclose 1:cw 2:adsb 3:dmac
	mdelay(1000);
	board_1_2 = phase_calculation(0,0);
	printf("board_middle_cali\n");
	console_print("board_middle_cali: %f \n",board_1_2.phase);

}


void phase_compensate(double* param, char param_no) //phase_compensate
{
	if(param_no>=1)
	{
		unsigned int channel1 = 28;
		unsigned int channel2 = 20;
		unsigned int channel3 = 12;
		unsigned int channel4 = 4;
		double phase_value_i;
		double phase_value_q;
		uint32_t addr_i;
		uint32_t addr_q;
		int32_t value_i;
		int32_t value_q;
		//channnel1 config
		if(param_no >=1)
		{
			addr_i=0x43C00000+channel1;
			addr_q=0x43C00000+channel1+4;
			phase_value_i=32765*sin(2*PI*param[0]/360);
			phase_value_q=32765*cos(2*PI*param[0]/360);
			value_i = round(phase_value_i);
			value_q = round(phase_value_q);
			Xil_Out32(addr_i,value_i);
			Xil_Out32(addr_q,value_q);
			printf("phase_compensate = %f \n",param[0]);
		}
		//channnel2 config
		if(param_no >= 2)
		{
			addr_i=0x43C00000+channel2;
			addr_q=0x43C00000+channel2+4;
			phase_value_i=32765*sin(2*PI*param[1]/360);
			phase_value_q=32765*cos(2*PI*param[1]/360);
			value_i = round(phase_value_i);
			value_q = round(phase_value_q);
			Xil_Out32(addr_i,value_i);
			Xil_Out32(addr_q,value_q);
			printf("phase_compensate = %f, %f \n",param[0],param[1]);
		}
		//channnel3 config
		if(param_no >= 3)
		{
			addr_i=0x43C00000+channel3;
			addr_q=0x43C00000+channel3+4;
			phase_value_i=32765*sin(2*PI*param[2]/360);
			phase_value_q=32765*cos(2*PI*param[2]/360);
			value_i = round(phase_value_i);
			value_q = round(phase_value_q);
			Xil_Out32(addr_i,value_i);
			Xil_Out32(addr_q,value_q);
			printf("phase_compensate = %f, %f , %f \n",param[0],param[1],param[2]);
		}
		//channnel4 config
		if(param_no >= 4)
		{
			addr_i=0x43C00000+channel4;
			addr_q=0x43C00000+channel4+4;
			phase_value_i=32765*sin(2*PI*param[3]/360);
			phase_value_q=32765*cos(2*PI*param[3]/360);
			value_i = round(phase_value_i);
			value_q = round(phase_value_q);
			Xil_Out32(addr_i,value_i);
			Xil_Out32(addr_q,value_q);
			printf("phase_compensate = %f, %f , %f, %f \n",param[0],param[1],param[2],param[3]);
		}
		console_print("phase_compensate \n");

	}

}



fft_result phase_calculation(unsigned int A,unsigned int B)//00:1-1 10:1-2 01:1-3 11:1-4 switch_rf
{
	unsigned int k=0,j=0;
	unsigned int buf_i[163840];
	unsigned int buf_q[163840];
	unsigned int recv_data=0;
	float phase=0;
	fft_result amp_phi,amp_phi_i,amp_phi_q;
	gpio_set_value(54, A);
	gpio_set_value(56, B);
	usleep(300000);//delay 200ms
	for(int i=0;i<1;i++)
	{
		adc_capture(16384, ADC_DDR_BASEADDR);
		while(!recv_finish_flg);
		recv_finish_flg = 0;
		Xil_DCacheInvalidateRange(ADC_DDR_BASEADDR, 16384 * 16);
		for(k=0,j=0;k<16384*16;k=k+16,j++)
		{
			recv_data = Xil_In16(ADC_DDR_BASEADDR + k);
			buf_i[j] = 0xFFF & recv_data;
			if(step_set == 0)
			{
				buf_q[j] = 0;
			}
			else if(step_set == 1)
			{
				recv_data = Xil_In16(ADC_DDR_BASEADDR + k +2);
				buf_q[j] = 0xFFF & recv_data;
			}

			//if(print_out)
			//{
			//	printf("%d ",buf_i[j]);
			//}
		}
		amp_phi_i = fft_calculate(16384,buf_i,buf_q);
		for(k=4,j=0;k<16384*16;k=k+16,j++)
		{
			recv_data = Xil_In16(ADC_DDR_BASEADDR + k);
			buf_i[j] = 0xFFF & recv_data;
			if(step_set == 0)
			{
				buf_q[j] = 0;
			}
			else if(step_set == 1)
			{
				recv_data = Xil_In16(ADC_DDR_BASEADDR + k +2);
				buf_q[j] = 0xFFF & recv_data;
			}
			//if(print_out)
			//{
			//	printf("%d ",buf_q[j]);
			//}
		}
		//printf("\n");

		amp_phi_q = fft_calculate(16384,buf_i,buf_q);
		//amp_phi.amplitude = amp_phi_i.amplitude-amp_phi_q.amplitude;
		if(step_set == 0)
		{
			amp_phi.phase =  amp_phi_q.phase - amp_phi_i.phase;
		}
		else if(step_set == 1)
		{
			amp_phi.phase = amp_phi_i.phase + amp_phi_q.phase;
		}

		if(amp_phi.phase>180)
		{
			amp_phi.phase = amp_phi.phase - 360;
		}
		if(amp_phi.phase<-180)
		{
			amp_phi.phase = amp_phi.phase + 360;
		}
		phase = phase + amp_phi.phase;
		printf("amp_phi = %f\n",amp_phi.phase);
	}
	printf("phase = %f\n",phase);
	amp_phi.phase = phase;
	return amp_phi;
}
void self_check(double* param, char param_no) //self_check
{
	unsigned int check = param[0];

	if(param_no >= 1)
	{
		if(check == 1 )
		{
			if(state_ready==1)
				console_print("self_check: state_ready\n");
			else
				console_print("self_check: state_no_ready\n");
		}
	}
}
void set_mode(double* param, char param_no) //set_mode**************************************************************************
{
	unsigned int mode = param[0];
	if(param_no >= 1)
	{
		Xil_Out32(MODE_CNTROL, mode); //0£ºclose 1:cw 2:adsb 3:dmac
		console_print("mode  = %d \n",Xil_In32(MODE_CNTROL));


	}
}
void set_reset(double* param, char param_no) //
{
	unsigned int reset = param[0];

	if(param_no >= 1)
	{
		if(reset == 1)
		{
			printf("reset ***********\n");
			PsSoftwareReset();
		}
	}

}

void set_rf_port(double* param, char param_no) //set_rf_port_switch
{
	unsigned int port = param[0];

	if(param_no >= 1)
	{
		if(port == 0)
		{
			ad9361_set_tx_rf_port_output(ad9361_phy,0);   //TXA
			ad9361_set_tx_rf_port_output(ad9361_phy_b,0); //TXA
			ad9361_set_rx_rf_port_input(ad9361_phy,0);    //RXA
			ad9361_set_rx_rf_port_input(ad9361_phy_b,0);   //RXA
			printf(" set_tx--TXA set_rx--RXA \n");
		}
		else if(port == 1)
		{
			ad9361_set_tx_rf_port_output(ad9361_phy,1);   //TXB
			ad9361_set_tx_rf_port_output(ad9361_phy_b,1); //TXB
			ad9361_set_rx_rf_port_input(ad9361_phy,2);    //RXC
			ad9361_set_rx_rf_port_input(ad9361_phy_b,2);   //RXC
			printf(" set_tx--TXB set_rx--RXC \n");
		}

		console_print("set_rf_port_switch\n");

	}
}

void set_tx_port(double* param, char param_no) //set_tx_port
{


	uint32_t phy_a = param[0];
	uint32_t phy_b = param[1];
	ad9361_set_tx_rf_port_output(ad9361_phy,phy_a);   // 0:TXA 1:TXB
	ad9361_set_tx_rf_port_output(ad9361_phy_b,phy_b); // 0:TXA 1:TXB
}
void set_rx_port(double* param, char param_no) //set_rx_port
{


	uint32_t phy_a = param[0];
	uint32_t phy_b = param[1];
	ad9361_set_rx_rf_port_input(ad9361_phy,phy_a);   //0:RX1A 1:RX1B 2:RX1C
	ad9361_set_rx_rf_port_input(ad9361_phy_b,phy_b); //0:RX1A 1:RX1B 2:RX1C
}
void get_phase(double* param, char param_no) //get_phase
{
	fft_result ad9361_1_2;
    unsigned int A = param[0];
    unsigned int B = param[1];
	ad9361_1_2 = phase_calculation(A,B);
	console_print("get_phase: %f \n",ad9361_1_2.phase);

}

void get_data(double* param, char param_no) //get_data
{
	int recv_data=0;
	uint32_t k,j;
	adc_capture(16384, ADC_DDR_BASEADDR);
	while(!recv_finish_flg);
	recv_finish_flg = 0;
	Xil_DCacheInvalidateRange(ADC_DDR_BASEADDR, 16384 * 16);
	for(k=0,j=0;k<16384*16;k=k+2,j++)
	{
		recv_data = Xil_In16(ADC_DDR_BASEADDR + k);
		printf("%d ",recv_data);
	}

}


void set__attenuation(double* param, char param_no) // "tx1_attenuation=" command
{
	uint32_t attenuation_mdb1;
	uint32_t attenuation_mdb2;
	uint32_t attenuation_mdb3;
	uint32_t attenuation_mdb4;
	if(param_no >= 1)
	{
		attenuation_mdb1 = param[0];
		attenuation_mdb2 = param[1];
		attenuation_mdb3 = param[2];
		attenuation_mdb4 = param[3];
		ad9361_set_tx_attenuation(ad9361_phy, 0, attenuation_mdb1);
		ad9361_set_tx_attenuation(ad9361_phy, 1, attenuation_mdb2);
		ad9361_set_tx_attenuation(ad9361_phy_b, 0, attenuation_mdb3);
		ad9361_set_tx_attenuation(ad9361_phy_b, 1, attenuation_mdb4);
		console_print("tx_attenuation=%d %d %d %d \n", attenuation_mdb1,attenuation_mdb2,attenuation_mdb3,attenuation_mdb4);
		console_print("tx1_attenuation\n");
	}
}


void set_lo_freq(double* param, char param_no) //set_lo_freq
{

	double par1 = param[0];
	uint64_t lo_freq=par1*1000000;
	uint64_t lo;
	if(param_no >= 1)
	{
		ad9361_set_tx_lo_freq(ad9361_phy,lo_freq);
		ad9361_set_tx_lo_freq(ad9361_phy_b,lo_freq);
		ad9361_get_tx_lo_freq (ad9361_phy,&lo);
		console_print("set_lo_freq =%d Hz  %d Hz\n",lo,lo_freq);
	}
}
void set_rx_lo_freq(double* param, char param_no) //set_rx_lo_freq
{

	double par1 = param[0];
	double par2 = param[1];
	uint64_t lo_freq=par1*1000000;
	uint64_t lo_freq_b=par2*1000000;
	console_print("ad9361_set_rx_lo_freq =%f  %f Hz\n",lo_freq,lo_freq_b);
	if(param_no >= 1)
	{
		ad9361_set_rx_lo_freq(ad9361_phy,lo_freq);
		ad9361_set_rx_lo_freq(ad9361_phy_b,lo_freq_b);

	   //ad9361_get_rx_lo_freq (ad9361_phy,&lo);

	}
}
void write_reg(double* param, char param_no) //write_reg
{
	unsigned int device = param[0];
	unsigned int reg    = param[1];
	unsigned int data   = param[2];
	if(param_no >= 1)
	{
		if(device == 0)
		{
			ad9361_spi_write(ad9361_phy->spi,reg,data);
		}
		else
		{
			ad9361_spi_write(ad9361_phy_b->spi,reg,data);
		}
		printf("device = %x reg = %x data = %x\n",device,reg,data);
		usleep(100000);//delay 100ms
		console_print("device_%d_reg_%x_write_%x",device,reg,data);
	}
}
void read_reg(double* param, char param_no) //read_reg
{
	unsigned int device = param[0];
	unsigned int reg    = param[1];
	unsigned int data   = 0;
	if(param_no >= 1)
	{
		//ad9361_do_mcs(ad9361_phy, ad9361_phy_b);
		if(device == 0)
		{
			data = ad9361_spi_read(ad9361_phy->spi,reg);
		}
		else
		{
			data = ad9361_spi_read(ad9361_phy_b->spi,reg);
		}
		printf("device = %x reg = %x data = %x\n",device,reg,data);
		usleep(100000);//delay 100ms
		console_print("device_%d_reg_%x_read_%x",device,reg,data);
	}
}

void set_signal_fre(double* param, char param_no) //set_signal_fre
{
	uint32_t fre = param[0];
	uint32_t signal_fre = 0;
	printf("fre =  %d\n",fre);
	if(param_no >= 1)
	{
		signal_fre = (uint32_t) 20*1000000.0/fre;
		Xil_Out32(0x43C00000, signal_fre);
	}
	console_print("fre = %d",signal_fre);
}

void set_step(double* param, char param_no) //set_step
{
	unsigned int step = param[0];
	printf("step =  %d\n",step);
	if(param_no >= 1)
	{
		step_set = step;
	}
	console_print("step = %d",step);
}

void set_mgc(double* param, char param_no) //set_mgc
{
	uint32_t mgc_r0 = param[0];
	uint32_t mgc_r1 = param[1];
	if(param_no >= 1)
	{
		ad9361_spi_write(ad9361_phy_b->spi, 0x109,mgc_r1);
		ad9361_spi_write(ad9361_phy_b->spi, 0x10C,mgc_r1);
		ad9361_spi_write(ad9361_phy->spi, 0x109,mgc_r0);
		ad9361_spi_write(ad9361_phy->spi, 0x10C,mgc_r0);
	}
	console_print("mgc_r0 = %d mgc_r1 = %d ",mgc_r0,mgc_r1);
}


void do_command(char *recvbuf)
{
	char	cmd	 =  0;
	int		cmd_type		 = -1;
	double	param[128] = {0, 0, 0, 0, 0};
	char    param_no = 0;
	char	invalid_cmd =  0;
	for(cmd = 0; cmd < cmd_no; cmd++)
	{
		param_no = 0;
		cmd_type = console_check_commands(recvbuf, cmd_list[cmd].name,
										  param, &param_no);
		if(cmd_type == UNKNOWN_CMD)
		{
			invalid_cmd++;
		}
		else
		{
			cmd_list[cmd].function(param, param_no);
		}
	}
	if(invalid_cmd == cmd_no)
	{
		print("Invalid command!\n");
	}
}
