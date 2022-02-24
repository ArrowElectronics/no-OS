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
#include "includes/ad463x.h"
#include "includes/error.h"
#include "includes/axi_adc_core.h"
#include "includes/axi_dmac.h"
#include "includes/alt_acpidmap.h"
#include "includes/alt_address_space.h"
#include "includes/alt_cache.h"
#include "includes/arm_cache_modified.h"
#include "includes/fpga_dmac_api.h"

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

#define DMA_TRANSFER_SIZE				1024

int __auto_semihosting;
int main(int argc, char** argv)
{
	printf("AD463x main function starts here   !\r\n");

#if defined(__ARMCC_VERSION) && defined(PRINTF_UART)
/* Without Semihosting we don't have an argc/argv. Attempting to use them causes linker errors */
    int argc_=0; char **argv_=NULL;
  #define argc argc_
  #define argv argv_
#endif

	uint64_t buf[AD463x_EVB_SAMPLE_NO] __attribute__ ((aligned));
	struct ad463x_dev *dev;
	int32_t ret;
	uint32_t i, samples = 0;

	struct spi_engine_offload_init_param spi_engine_offload_init_param = {
		.offload_config = OFFLOAD_RX_EN,
		.rx_dma_baseaddr = AD463x_DMA_BASEADDR,
	};

	struct spi_engine_init_param spi_eng_init_param  = {
		.ref_clk_hz = 160000000,
		.type = SPI_ENGINE,
		.spi_engine_baseaddr = AD463x_SPI_ENGINE_BASEADDR,
		.cs_delay = 1,
		.data_width = 32,
	};

	struct spi_init_param spi_init = {
		.chip_select = AD463x_SPI_CS,
		.max_speed_hz = 80000000,
		.mode = SPI_MODE_0,
		.platform_ops = &spi_eng_platform_ops,
		.extra = (void*)&spi_eng_init_param,
	};

	struct ad463x_init_param ad463x_init_param = {
		.spi_init = &spi_init,
		.offload_init_param = &spi_engine_offload_init_param,
		.reg_data_width = 8,
		.capture_data_width = 8,
		.output_mode = AD463X_24_DIFF_8_COM,
		.lane_mode = AD463X_FOUR_LANES_PER_CH,
		.clock_mode = AD463X_SPI_CLOCK_MODE,
		.data_rate = AD463X_SDR_MODE,
		.device_id = ID_AD4630_24, /* dev_id */
		.spi_engine_baseaddr = AD463x_SPI_ENGINE_BASEADDR,
		.dcache_invalidate_range =
		(void (*)(void *, size_t))alt_cache_system_invalidate,
	};

	ret = ad463x_init(&dev, &ad463x_init_param);
	if (ret != SUCCESS)
		return ret;

	/* Exit register configuration mode */
	ret = ad463x_exit_reg_cfg_mode(dev);
	if (ret != SUCCESS)
		return ret;

	/* Number of samples to read */
	if (ad463x_init_param.lane_mode == AD463X_FOUR_LANES_PER_CH)
		samples = AD463x_EVB_SAMPLE_NO * 8;
	else if (ad463x_init_param.lane_mode == AD463X_TWO_LANES_PER_CH)
		samples = AD463x_EVB_SAMPLE_NO * 4;
	else
		samples = AD463x_EVB_SAMPLE_NO * 2;

	/* Read data */
	while (true) {
		ret = ad463x_read_data(dev, buf, samples);

        if (ret != SUCCESS)
			return ret;

        /* Format the data according to the mode selected */
		if (ad463x_init_param.output_mode == AD463X_24_DIFF) {
			for (i = 0; i < AD463x_EVB_SAMPLE_NO; i++) {
				printf("ADC CH0: 0x%lx    \t\tADC CH1: 0x%lx\r\n", ((uint32_t) buf[i]) >> 8,
																((uint32_t) (buf[i] >> 32)) >> 8);
			}
		}
		else if (ad463x_init_param.output_mode == AD463X_16_DIFF_8_COM) {
			if (ad463x_init_param.clock_mode == AD463X_MASTER_CLOCK_MODE) {
				for (i = 0; i < AD463x_EVB_SAMPLE_NO; i++) {
					/* Data for 4-Lane Master Clock is packed in exact 24-bits */
					if (ad463x_init_param.lane_mode == AD463X_FOUR_LANES_PER_CH) {
						printf("ADC CH0: 0x%lx   \t\tVCOM0: 0x%lx  \t\tADC CH1: 0x%lx   \t\tVCOM1: 0x%lx\r\n",
	        													((uint32_t) buf[i]) >> 8, ((uint32_t) buf[i]) & 0xFF,
																((uint32_t) (buf[i] >> 32)) >> 8, ((uint32_t) (buf[i] >> 32)) & 0xFF);
					}
					else {
						printf("ADC CH0: 0x%lx   \t\tVCOM0: 0x%lx  \t\tADC CH1: 0x%lx   \t\tVCOM1: 0x%lx\r\n",
	        													((uint32_t) buf[i]) >> 16, ((uint32_t) buf[i]) & 0xFF,
																((uint32_t) (buf[i] >> 32)) >> 16, ((uint32_t) (buf[i] >> 32)) & 0xFF);

					}
				}
			}
			else {
				for (i = 0; i < AD463x_EVB_SAMPLE_NO; i++) {
					printf("ADC CH0: 0x%lx   \t\tVCOM0: 0x%lx  \t\tADC CH1: 0x%lx   \t\tVCOM1: 0x%lx\r\n",
	        										((uint32_t) buf[i]) >> 16, (((uint32_t) buf[i]) >> 8) & 0xFF,
													((uint32_t) (buf[i] >> 32)) >> 16, (((uint32_t) (buf[i] >> 32)) >> 8) & 0xFF);
				}
			}

		}
		else if (ad463x_init_param.output_mode == AD463X_24_DIFF_8_COM) {
			for (i = 0; i < AD463x_EVB_SAMPLE_NO; i++) {
				printf("ADC CH0: 0x%lx    \t\tVCOM0: 0x%lx  \t\tADC CH1: 0x%lx    \t\tVCOM1: 0x%lx\r\n",
	        										((uint32_t) buf[i]) >> 8, ((uint32_t) buf[i]) & 0xFF,
													((uint32_t) (buf[i] >> 32)) >> 8, ((uint32_t) (buf[i] >> 32)) & 0xFF);
			}
		}

        else if (ad463x_init_param.output_mode == AD463X_30_AVERAGED_DIFF) {
    		for (i = 0; i < AD463x_EVB_SAMPLE_NO; i++) {
    			printf("ADC CH0: 0x%lx    \t\tOR_SYNC0: 0x%lx\t\tADC CH1: 0x%lx    \t\tOR_SYNC1: 0x%lx\r\n",
	        										((uint32_t) buf[i]) >> 2, ((uint32_t) buf[i]) & 0x3,
													(uint32_t) (buf[i] >> 32) >> 2, ((uint32_t) (buf[i] >> 32)) & 0x3);
    		}
        }
        else {
    		for (i = 0; i < AD463x_EVB_SAMPLE_NO; i++) {
				printf("ADC CH0: 0x%lx    \t\tADC CH1: 0x%lx\r\n", (uint32_t) buf[i], (uint32_t) (buf[i] >> 32));
    		}
        }
	}

	ad463x_remove(dev);

	printf("Done.\n");

	return SUCCESS;
}

