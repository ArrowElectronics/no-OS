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
#include "includes/ad738x.h"
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

#define SUCCESS		0
#define FAILURE		-1

#define SPI_ENGINE_OFFLOAD_EXAMPLE				1
#define DMA_TRANSFER_SIZE						1024

int __auto_semihosting;
int main(int argc, char** argv)
{
	printf("AD7381 main function starts here   !\r\n");
	struct ad738x_dev *dev;

#if defined(__ARMCC_VERSION) && defined(PRINTF_UART)
/* Without Semihosting we don't have an argc/argv. Attempting to use them causes linker errors */
    int argc_=0; char **argv_=NULL;
  #define argc argc_
  #define argv argv_
#endif

	struct spi_engine_offload_init_param spi_engine_offload_init_param = {
		.offload_config = OFFLOAD_RX_EN,
		.rx_dma_baseaddr = AD7381_DMA_BASEADDR,
	};

	struct spi_engine_init_param spi_eng_init_param  = {
		.ref_clk_hz = 100000000,
		.type = SPI_ENGINE,
		.spi_engine_baseaddr = AD7381_SPI_ENGINE_BASEADDR,
		.cs_delay = 0,
	};

	struct spi_init_param ad738x_spi_init = {
		.chip_select = AD7381_SPI_CS,
		.max_speed_hz = 50000000,
		.mode = SPI_MODE_2,
		.platform_ops = &spi_eng_platform_ops,
		.extra = (void*)&spi_eng_init_param,
	};

	struct ad738x_init_param ad738x_default_init_param = {
		/* SPI engine*/
		.spi_param = &ad738x_spi_init,
		.offload_init_param = &spi_engine_offload_init_param,
		/* Configuration */
		.conv_mode = TWO_WIRE_MODE,		// Two/One-Wire Mode
		.ref_sel = INT_REF,
		.resolution = DEFAULT_RES_BIT, 	// Default/High Resolution
		.dcache_invalidate_range =
		(void (*)(void *, size_t))alt_cache_system_invalidate,
	};

#if SPI_ENGINE_OFFLOAD_EXAMPLE
	/* Set data width according to resolution bit */
	if (ad738x_default_init_param.resolution == DEFAULT_RES_BIT)
		spi_eng_init_param.data_width = 14;
	else if (ad738x_default_init_param.resolution == HIGH_RES_BIT)
		spi_eng_init_param.data_width = 16;

	uint32_t *data = NULL;
	uint16_t i;
#else
	ad738x_default_init_param.conv_mode = ONE_WIRE_MODE;
	ad738x_default_init_param.resolution = HIGH_RES_BIT;
	spi_eng_init_param.data_width = 16;
#endif

	cache_fun();

	ad738x_init(&dev, &ad738x_default_init_param);

#if SPI_ENGINE_OFFLOAD_EXAMPLE
	/* Read data */
	while (true) {
		data = ad738x_read_data(dev, AD7381_EVB_SAMPLE_NO, DMA_TRANSFER_SIZE);

	if (dev->conv_mode == TWO_WIRE_MODE) {
		for (i = 0; i < AD7381_EVB_SAMPLE_NO; i++)
			printf("ADC0 sample: 0x%x\tADC1 sample: 0x%x\r\n", (uint16_t)(data[i] & 0x3FFF),
														(uint16_t)((data[i] >> 16) & 0x3FFF));
	}
	else {
		for (i = 0; i < AD7381_EVB_SAMPLE_NO; i += 2)
			printf("ADC0 sample: 0x%x\tADC1 sample: 0x%x\r\n", (uint16_t)(data[i] & 0x3FFF),
															(uint16_t)(data[i+1] & 0x3FFF));
	}

	/* To use the High Resolution mode comment out the above block
	 * and uncomment the following block */

/*		if (dev->conv_mode == TWO_WIRE_MODE) {
			for (i = 0; i < AD7381_EVB_SAMPLE_NO; i++)
				printf("ADC0 sample: 0x%x\tADC1 sample: 0x%x\r\n", (uint16_t)(data[i]), (uint16_t)(data[i] >> 16));
		}
		else {
			for (i = 0; i < AD7381_EVB_SAMPLE_NO; i += 2) {
				printf("ADC0 sample: 0x%x\tADC1 sample: 0x%x\r\n", (uint16_t)data[i], (uint16_t)data[i+1]);
				continue;
			}
		}
*/


	}
	printf("\r\n");
#else
	/* SPI single conversion mode supports only the following
	 * supported bit resolutions
	 * 		- 8
	 * 		- 16
	 * 		- 24
	 * 		- 32
	 *
	 * 	AD7380, AD7383, AD7386
	 * 		- Supports 16-bit output in default resolution mode only.
	 * 	AD7381, AD7384, AD7387
	 * 		- Supports 16-bit output in enhanced resolution mode only.
	 */
	uint16_t adc_data[2];

	while(true) {
		ad738x_spi_single_conversion(dev, adc_data);
		printf("ADC0 sample: 0x%x\tADC1 sample: 0x%x\r\n", adc_data[0], adc_data[1]);
	}

	printf("\r\n");
#endif

	ad738x_remove(dev);

	printf("End of ADC Data Readings\r\n");

	return SUCCESS;
}

