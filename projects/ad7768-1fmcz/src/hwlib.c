/******************************************************************************
*
* Copyright 2014 Altera Corporation. All Rights Reserved.
* 
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* 
* 1. Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* 
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
* 
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software without
* specific prior written permission. spi_engine_read Wait for the end sync signal
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
* 
******************************************************************************/

/*
 * $Id: //acds/rel/18.1std/embedded/examples/software/Altera-SoCFPGA-HardwareLib-SPI-CV-GNU/hwlib.c#1 $
 */

/* Test run for SPI EEPROM M95256 */
#include "stdio.h"
#include "stdlib.h"
#include "stdint.h"
#include "stdbool.h"
#include "string.h"
#include <inttypes.h>
#include "includes/socal.h"
#include "includes/hps.h"
#include "includes/hwlib.h"
#include "includes/parameters.h"
#include "includes/spi_engine.h"
#include "includes/ad77681.h"
#include "includes/error.h"

#include "includes/alt_acpidmap.h"
#include "includes/alt_address_space.h"
#include "includes/arm_cache_modified.h"
#include "includes/fpga_dmac_api.h"
#include "includes/alt_cache.h"


#if defined(__ARMCC_VERSION) && defined(PRINTF_UART)
#pragma import(__use_no_semihosting)
void _sys_exit(int return_code)
{
    while(1);
}
#endif



void cache_fun();


uint32_t spi_msg_cmds[6] = {CS_LOW, CS_HIGH, CS_LOW, WRITE_READ(1), CS_HIGH};

#define SUCCESS		0
#define FAILURE		-1


#define SPI_ENGINE_OFFLOAD_EXAMPLE	1
#define DMA_TRANSFER_SIZE 1024    //256



struct spi_engine_init_param spi_eng_init_param  = {
	.type = SPI_ENGINE,
	.spi_engine_baseaddr = AD77681_SPI1_ENGINE_BASEADDR,
	.cs_delay = 0,
	.data_width = 32,
	.ref_clk_hz = AD77681_SPI_ENG_REF_CLK_FREQ_HZ,
};

struct ad77681_init_param ADC_default_init_param = {
	/* SPI */
	{
		.chip_select = AD77681_SPI_CS,
		.max_speed_hz = 1000000,
		.mode = SPI_MODE_3,
		.platform_ops = &spi_eng_platform_ops,
		.extra = (void*)&spi_eng_init_param,
	},
	/* Configuration */
	AD77681_FAST,				// power_mode
	AD77681_MCLK_DIV_8,			// mclk_div
	AD77681_CONV_CONTINUOUS,	// conv_mode
	AD77681_POSITIVE_FS,		// diag_mux_sel
	false,						// conv_diag_sel
	AD77681_CONV_24BIT,			// conv_len
	AD77681_NO_CRC, 				// crc_sel
	0, 							// status_bit
	AD77681_VCM_HALF_VCC,		/* VCM setup*/
	AD77681_AINn_ENABLED,		/* AIN- precharge buffer*/
	AD77681_AINp_ENABLED,		/* AIN+ precharge buffer*/
	AD77681_BUFn_ENABLED,		/* REF- buffer*/
	AD77681_BUFp_ENABLED,		/* REF+ buffer*/
	AD77681_FIR,			/* FIR Filter*/
	AD77681_SINC5_FIR_DECx32,	/* Decimate by 32*/
	0,				/* OS ratio of SINC3*/
	4096,				/* Reference voltage*/
	16384,				/* MCLK in kHz*/
};




int __auto_semihosting;
int main(int argc, char** argv)
{

    printf("AD7768-1 main function starts Here   !\r\n");
		struct ad77681_dev	*adc_dev;
		struct ad77681_status_registers *adc_status;

		uint8_t			adc_data[5];
		uint32_t 		i;
		uint32_t *offload_data=NULL;
		int32_t ret;
		uint8_t commands_data[4] = {0, 0, 0, AD77681_REG_READ(AD77681_REG_ADC_DATA)};
		struct spi_engine_offload_init_param spi_engine_offload_init_param = {
			.offload_config = (OFFLOAD_RX_EN), // | OFFLOAD_TX_EN),
			.rx_dma_baseaddr = AD77681_DMA_1_BASEADDR,
			.tx_dma_baseaddr = AD77681_DMA_1_BASEADDR
		};
		struct spi_engine_offload_message spi_engine_offload_message;

//commented by shivashankar
   //ALT_STATUS_CODE ret_val = ALT_E_SUCCESS;
#if defined(__ARMCC_VERSION) && defined(PRINTF_UART)
/* Without Semihosting we don't have an argc/argv. Attempting to use them causes linker errors */
    int argc_=0; char **argv_=NULL;
  #define argc argc_
  #define argv argv_
#endif

    cache_fun();
    ad77681_setup(&adc_dev, ADC_default_init_param, &adc_status);
    if (SPI_ENGINE_OFFLOAD_EXAMPLE == 0) {
    		while(1) {
    			ad77681_spi_read_adc_data(adc_dev, adc_data);
    			printf("[ADC DATA]: 0x");
    			for(i = 1; i < sizeof(adc_data)-1 / sizeof(uint8_t); i++) {
    				printf("%x", adc_data[i]);
    			}
    			printf("\r\n");

    		}
    	} else {
    		ad77681_setup(&adc_dev, ADC_default_init_param, &adc_status);
    		ad77681_set_continuos_read(adc_dev ,AD77681_CONTINUOUS_READ_ENABLE);
    		ret = spi_engine_offload_init(adc_dev->spi_desc,
    					      &spi_engine_offload_init_param);
    		if (ret != SUCCESS)
    			return FAILURE;
    		spi_engine_offload_message.commands = spi_msg_cmds;
    		spi_engine_offload_message.no_commands = ARRAY_SIZE(spi_msg_cmds);
    		memcpy(spi_engine_offload_message.commands_data,commands_data,4);

    		void* unalligned_Buffer;
    		uint8_t* Buffer = (uint8_t*) align_malloc(DMA_TRANSFER_SIZE, &unalligned_Buffer);
    		memset(Buffer,0x00,DMA_TRANSFER_SIZE);
    		spi_engine_offload_message.rx_addr = ((unsigned int ) &Buffer[0]);
    		printf("memory allocation for DMA : spi_engine_offload_message.rx_addr-0x%x !\r\n",(unsigned int)spi_engine_offload_message.rx_addr);
    		ret = spi_engine_offload_transfer(adc_dev->spi_desc, spi_engine_offload_message,
    						  AD77681_EVB_SAMPLE_NO);
    		if (ret != SUCCESS)
    			return ret;
    		offload_data  =(uint32_t *)Buffer;

    		alt_cache_system_invalidate((void *)spi_engine_offload_message.rx_addr, AD77681_EVB_SAMPLE_NO  *32 );

    		for (i = 0; i < AD77681_EVB_SAMPLE_NO ; i++) {
    			printf("offload_address:0x%x  offload_data:0x%x\r\n",(unsigned int)offload_data,(unsigned int)(*offload_data));
    			offload_data +=1;
    		}
    	}

    	printf("Bye\n");




}

