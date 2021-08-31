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
#include "includes/ad7606.h"
#include "includes/error.h"
#include "includes/axi_dmac.h"

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

uint32_t spi_msg_cmds[3] = {CS_LOW, WRITE_READ(2), CS_HIGH};

#define SUCCESS		0
#define FAILURE		-1

#define PARALLEL_SERIAL				1		// 0 = PAR, 1 = SER
#define DMA_TRANSFER_SIZE			1024

struct spi_engine_init_param spi_eng_init_param  = {
	.type = SPI_ENGINE,
	.spi_engine_baseaddr = AD7606B_SPI_ENGINE_BASEADDR,
	.cs_delay = 0,
	.data_width = 32,
	.ref_clk_hz = AD7606B_SPI_ENG_REF_CLK_FREQ_HZ,
};

struct ad7606_init_param ADC_default_init_param = {
	/* SPI initialization parameters */
	.spi_init = {
		.chip_select = AD7606B_SPI_CS,
		.max_speed_hz = 500000000,
		.mode = SPI_MODE_2,
		.platform_ops = &spi_eng_platform_ops,
		.extra = (void*)&spi_eng_init_param,
	},

	.sw_mode = true,
	.gpio_reset = NULL,
	.gpio_stby_n = NULL,
	.gpio_range = NULL,
	.gpio_os0 = NULL,
	.gpio_os1 = NULL,
	.gpio_os2 = NULL,
	.gpio_convst = NULL,
	.device_id = ID_AD7606B,
	.oversampling = {0},
	.config = {
		.op_mode = AD7606_NORMAL,
		.dout_format = AD7606_4_DOUT,
		.ext_os_clock = false,
		.status_header = false,
	},

	.core_baseaddr = AD7606B_SPI_ENGINE_BASEADDR,
	.digital_diag_enable = {0},
	.range_ch = {
			{-10000, 10000, false},	/* 000 */
			{-10000, 10000, false},	/* 001 */
			{-10000, 10000, false},	/* 010 */
			{-10000, 10000, false},	/* 011 */
			{-10000, 10000, false},	/* 100 */
			{-10000, 10000, false},	/* 101 */
			{-10000, 10000, false},	/* 110 */
			{-10000, 10000, false},	/* 111 */
	},
};

int __auto_semihosting;
int main(int argc, char** argv)
{
	printf("AD7606b main function starts Here   !\r\n");
	struct ad7606_dev *adc_dev;
	uint16_t i;

#if defined(__ARMCC_VERSION) && defined(PRINTF_UART)
/* Without Semihosting we don't have an argc/argv. Attempting to use them causes linker errors */
    int argc_=0; char **argv_=NULL;
  #define argc argc_
  #define argv argv_
#endif

	cache_fun();

#if PARALLEL_SERIAL
	uint16_t chan_break = 4;
	uint32_t *offload_data = NULL;
	uint8_t commands_data[1] = {0x00};

	struct spi_engine_offload_init_param spi_engine_offload_init_param = {
		.offload_config = OFFLOAD_RX_EN,
		.rx_dma_baseaddr = AD7606B_DMA_BASEADDR,
	};

	struct spi_engine_offload_message spi_engine_offload_message;

	ad7606_serial_init(&adc_dev, &ADC_default_init_param);

	spi_engine_offload_init(adc_dev->spi_desc,
    			      &spi_engine_offload_init_param);

   	spi_engine_offload_message.commands = spi_msg_cmds;
   	spi_engine_offload_message.no_commands = ARRAY_SIZE(spi_msg_cmds);
   	memcpy(spi_engine_offload_message.commands_data, commands_data, 1);

   	void* unalligned_Buffer;
   	uint8_t* Buffer = (uint8_t*) align_malloc(DMA_TRANSFER_SIZE, &unalligned_Buffer);
	memset(Buffer,0x00,DMA_TRANSFER_SIZE);
	spi_engine_offload_message.rx_addr = ((unsigned int) &Buffer[0]);
	printf("memory allocation for DMA : spi_engine_offload_message.rx_addr-0x%x !\r\n",(unsigned int)spi_engine_offload_message.rx_addr);

	axi_io_write(AD7606B_SPI_ENGINE_BASEADDR, AD7606B_REG_UP_CNV_RATE, 0x82); //to configure up_cnv rate BE
	axi_io_write(AD7606B_SPI_ENGINE_BASEADDR, AD7606B_REG_CNVST_EN, 0x03); //to enable cnvst and reset high

	spi_engine_offload_transfer(adc_dev->spi_desc, spi_engine_offload_message,
    				  AD7606B_EVB_SAMPLE_NO);

   	alt_cache_system_invalidate((void *) spi_engine_offload_message.rx_addr, AD7606B_EVB_SAMPLE_NO  * 32);
   	offload_data = (uint32_t *) Buffer;

	printf("CH1\tCH2\tCH3\tCH4\tCH5\tCH6\tCH7\tCH8");

   	for (i = 0; i < AD7606B_EVB_SAMPLE_NO; i++) {
		if ((i % chan_break) == 0)
			printf("\n\r");

   		printf("0x%x\t0x%x\t", (uint16_t)(*offload_data), (uint16_t)(*offload_data >> 16));
   		offload_data += 1;
   	}

#else
   	uint16_t chan_no = 8;
	uint16_t *par_data = NULL;

	ad7606_parallel_init(&adc_dev, &ADC_default_init_param);

	par_data = ad7606_par_data_read(adc_dev, AD7606B_DMA_BASEADDR,
    		AD7606B_EVB_SAMPLE_NO, DMA_TRANSFER_SIZE);

	printf("CH1\tCH2\tCH3\tCH4\tCH5\tCH6\tCH7\tCH8");
	for (i = 0; i < AD7606B_EVB_SAMPLE_NO; i++) {
		if ((i % chan_no) == 0)
			printf("\n\r");

		printf("0x%x\t", *par_data);
		par_data += 1;
	}
#endif

	printf("\n\r");
	axi_io_write(AD7606B_SPI_ENGINE_BASEADDR, AD7606B_REG_CNVST_EN, 0x00); //to disable cnvst and reset low
   	printf("End of ADC Data Readings\n");
}

