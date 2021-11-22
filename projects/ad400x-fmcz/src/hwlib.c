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

#include "includes/ad4020.h"
#include "includes/parameters.h"
#include "includes/spi_engine.h"
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


uint32_t spi_msg_cmds[3] = { CS_LOW, READ(2), CS_HIGH };

#define SUCCESS		0
#define FAILURE		-1

#define SPI_ENGINE_OFFLOAD_EXAMPLE		1
#define DMA_TRANSFER_SIZE 			1024

int __auto_semihosting;
int main(int argc, char** argv)
{

		printf("AD4020 main function starts Here   !\r\n");
		struct ad400x_dev *adc_dev;

		uint32_t adc_data;
		uint32_t i;
		uint32_t *offload_data = NULL;
		int32_t ret;

		struct spi_engine_offload_init_param spi_engine_offload_init_param = {
			.offload_config = OFFLOAD_RX_EN,
			.rx_dma_baseaddr = AD4020_DMA_BASEADDR,
		};
		struct spi_engine_offload_message spi_engine_offload_message;

		uint8_t commands_data[2] = {0xFF, 0xFF};

		enum ad400x_supported_dev_ids dev_id = ID_AD4020;

		struct spi_engine_init_param spi_eng_init_param  = {
				.ref_clk_hz = AD4020_SPI_ENG_REF_CLK_FREQ_HZ,
				.type = SPI_ENGINE,
				.spi_engine_baseaddr = AD4020_SPI_ENGINE_BASEADDR,
				.cs_delay = 2,
				.data_width = ad400x_device_resol[dev_id],
		};

		struct ad400x_init_param ADC_default_init_param = {
			 /* SPI */
			{
					.chip_select = AD4020_SPI_CS,
					.max_speed_hz = 83333333,
					.mode = SPI_MODE_0,
					.platform_ops = &spi_eng_platform_ops,
					.extra = (void*)&spi_eng_init_param,
			},
			/* Configuration */
			.reg_access_speed = 1000000,
			dev_id, /* dev_id */
			1,0,0,0,
		};

//commented by shivashankar
   //ALT_STATUS_CODE ret_val = ALT_E_SUCCESS;
#if defined(__ARMCC_VERSION) && defined(PRINTF_UART)
/* Without Semihosting we don't have an argc/argv. Attempting to use them causes linker errors */
    int argc_=0; char **argv_=NULL;
  #define argc argc_
  #define argv argv_
#endif

    cache_fun();
	ret = ad400x_init(&adc_dev, &ADC_default_init_param);
	if (ret < 0)
		return ret;

    if (SPI_ENGINE_OFFLOAD_EXAMPLE == 0) {
    		while(1) {
    			ad400x_spi_single_conversion(adc_dev, &adc_data);
    			printf("[ADC DATA]: 0x%x\n\r", (unsigned int)adc_data);
    			printf("\r\n");
    		}
    	} else {
    		ret = spi_engine_offload_init(adc_dev->spi_desc,
    					      &spi_engine_offload_init_param);
    		if (ret != SUCCESS)
    			return FAILURE;

    		spi_engine_offload_message.commands = spi_msg_cmds;
    		spi_engine_offload_message.no_commands = ARRAY_SIZE(spi_msg_cmds);
    		memcpy(spi_engine_offload_message.commands_data,commands_data,2);

    		void* unaligned_Buffer;
    		uint8_t* Buffer = (uint8_t*) align_malloc(DMA_TRANSFER_SIZE, &unaligned_Buffer);
    		memset(Buffer,0x00,DMA_TRANSFER_SIZE);
    		spi_engine_offload_message.rx_addr = ((unsigned int ) &Buffer[0]);
    		printf("memory allocation for DMA : spi_engine_offload_message.rx_addr-0x%x !\r\n",(unsigned int)spi_engine_offload_message.rx_addr);
    		ret = spi_engine_offload_transfer(adc_dev->spi_desc, spi_engine_offload_message,
    					(AD4020_EVB_SAMPLE_NO * 2));
    		if (ret != SUCCESS)
    			return ret;

    		offload_data = (uint32_t *)spi_engine_offload_message.rx_addr;

    		alt_cache_system_invalidate((void *)spi_engine_offload_message.rx_addr, AD4020_EVB_SAMPLE_NO  * 32 );

    		for (i = 0; i < AD4020_EVB_SAMPLE_NO; i++) {
    			printf("offload_address:0x%lx  offload_data:0x%lx\r\n",(uint32_t)offload_data,(uint32_t)(*offload_data));
    			offload_data +=1;
    		}
    	}

    	printf("Bye\n");




}

