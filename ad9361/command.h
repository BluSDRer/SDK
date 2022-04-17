/**************************************************************************//**
 *   @file   command.h
 *   @brief  Header file of AD9361 Command Driver.
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
#ifndef __COMMAND_H__
#define __COMMAND_H__
#include "fft.h"
/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
#define NULL		((void *)0)
#define SUCCESS		0
#define ERROR		-1

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/
typedef void (*cmd_function)(double* param, char param_no);
typedef struct
{
	const char* name;
	const char* description;
	const char* example;
	cmd_function function;
}command;
void do_command(char *recvbuf);
fft_result phase_calculation(unsigned int A,unsigned int B);
/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/* Displays all available commands. */
void get_help(double* param, char param_no);
void send_trigger(double* param, char param_no) ;//set_trigger
void board_iner_cali(double* param, char param_no); //board_iner_cali
void board_middle_cali(double* param, char param_no); //board_middle_cali
void phase_compensate(double* param, char param_no); //phase_compensate
void self_check(double* param, char param_no); //self_check
void set_mode(double* param, char param_no);
void set_reset(double* param, char param_no);
void set_rf_port(double* param, char param_no); //set_rf_port_switch
void set__attenuation(double* param, char param_no); // "tx1_attenuation=" command
void set_lo_freq(double* param, char param_no); //set_lo_freq
void write_reg(double* param, char param_no); //write_reg
void read_reg(double* param, char param_no); //read_reg
void set_signal_fre(double* param, char param_no); //set_signal_fre

void set_tx_port(double* param, char param_no); //set_tx_port
void set_rx_port(double* param, char param_no); //set_rx_port

void get_phase(double* param, char param_no); //get_phase
void get_data(double* param, char param_no); //get_data
void set_step(double* param, char param_no); //set_step
void set_mgc(double* param, char param_no); //set_mgc
void set_rx_lo_freq(double* param, char param_no); //set_rx_lo_freq
#endif  // __COMMAND_H__
