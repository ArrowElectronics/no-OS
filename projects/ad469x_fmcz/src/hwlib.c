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
#include "includes/ad469x.h"
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

// Channel Numbers To Enable
#define CHANNEL_NUMS					AD469x_CHANNEL(0) | AD469x_CHANNEL(1) | \
							AD469x_CHANNEL(2) | AD469x_CHANNEL(3) | \
							AD469x_CHANNEL(4) | AD469x_CHANNEL(5) | \
							AD469x_CHANNEL(6) | AD469x_CHANNEL(7) | \
							AD469x_CHANNEL(8) | AD469x_CHANNEL(9) | \
							AD469x_CHANNEL(10) | AD469x_CHANNEL(11) | \
							AD469x_CHANNEL(12) | AD469x_CHANNEL(13) | \
							AD469x_CHANNEL(14) | AD469x_CHANNEL(15)
#define STANDARD_SEQ

#define DMA_TRANSFER_SIZE				1024

int __auto_semihosting;
int main(int argc, char** argv)
{
	printf("AD469x main function starts here   !\r\n");

#if defined(__ARMCC_VERSION) && defined(PRINTF_UART)
/* Without Semihosting we don't have an argc/argv. Attempting to use them causes linker errors */
    int argc_=0; char **argv_=NULL;
  #define argc argc_
  #define argv argv_
#endif

	uint32_t buf[AD469x_EVB_SAMPLE_NO * TOTAL_CH] __attribute__ ((aligned));
	struct ad469x_dev *dev;
	uint32_t i = 0;
	int32_t ret;

	struct spi_engine_offload_init_param spi_engine_offload_init_param = {
		.offload_config = OFFLOAD_RX_EN,
		.rx_dma_baseaddr = AD469x_DMA_BASEADDR,
	};

	struct spi_engine_init_param spi_eng_init_param  = {
		.ref_clk_hz = AD469x_SPI_ENG_REF_CLK_FREQ_HZ,
		.type = SPI_ENGINE,
		.spi_engine_baseaddr = AD469x_SPI_ENGINE_BASEADDR,
		.cs_delay = 0,
	};

	struct spi_init_param spi_init = {
		.chip_select = AD469x_SPI_CS,
		.max_speed_hz = 40000000,
		.mode = SPI_MODE_3,
		.platform_ops = &spi_eng_platform_ops,
		.extra = (void*)&spi_eng_init_param,
	};

	struct ad469x_init_param ad469x_init_param = {
		.spi_init = &spi_init,
		.offload_init_param = &spi_engine_offload_init_param,
		.reg_data_width = 32,
		.capture_data_width = 24,
		.spi_engine_baseaddr = AD469x_SPI_ENGINE_BASEADDR,
		.dev_id = ID_AD4696, /* dev_id */
		.data_mode = 0, /* 0 = Staggered Mode; 1 = Continuous Mode */
		.dcache_invalidate_range =
		(void (*)(void *, size_t))alt_cache_system_invalidate,
	};

	struct axi_adc *axi_adc_core_desc;

	struct axi_adc_init axi_adc_initial = {
		.base = AD469x_AXI_ADC_BASEADDR,
		.name = "axi_ad469x_adc",
		.num_channels = TOTAL_CH,
	};

	cache_fun();

	ret = ad469x_init(&dev, &ad469x_init_param);
	if (ret < 0)
		return ret;

	ret = axi_adc_init(&axi_adc_core_desc, &axi_adc_initial);
	if (ret != SUCCESS)
		return FAILURE;

#ifdef STANDARD_SEQ
	printf("\r\nIn Standard Sequencer Mode\r\n");

	if (!ad469x_init_param.data_mode)
		printf("Staggered Mode Enabled\r\n");
	else
		printf("Continuous Mode Enabled\r\n");

	ret = ad469x_std_sequence_ch(dev, CHANNEL_NUMS);
	if (ret != SUCCESS)
		return ret;

	ret = ad469x_sequence_disable_temp(dev);
	if (ret != SUCCESS)
		return ret;

	ret = ad469x_set_channel_sequence(dev, AD469x_standard_seq);
	if (ret != SUCCESS)
		return ret;

	ret = ad469x_enter_conversion_mode(dev);
	if (ret != SUCCESS)
		return ret;

	printf("CH0\tCH1\tCH2\tCH3\tCH4\tCH5\tCH6\tCH7\tCH8\tCH9\tCH10\tCH11\tCH12\tCH13\tCH14\tCH15\r\n\r\n");
	while (1) {
		ret = ad469x_seq_read_data(dev, &buf[0], AD469x_EVB_SAMPLE_NO);
		if (ret != SUCCESS)
			return ret;

		for (i = 0; i < (AD469x_EVB_SAMPLE_NO / 2); i++) {
			if (i % 8 == 0)
				printf("\r\n");

			printf("0x%x\t0x%x\t", (uint16_t) (buf[i]),
										(uint16_t) (buf[i] >> 16));
		}
	}
#elif defined(ADVANCED_SEQ)
	if (ad469x_init_param.data_mode == 1) {
		printf("\r\nContinuous Mode only applicable in Standard Sequencer Mode. Exiting.\r\n");
		return FAILURE;
	}

	printf("\r\nIn Advanced Sequencer Mode\r\n");
	ret = ad469x_adv_sequence_set_num_slots(dev, no_of_ones(CHANNEL_NUMS));
	if (ret != SUCCESS)
		return ret;

	for (i = 0; i < no_of_ones(CHANNEL_NUMS); i++) {
		ret = ad469x_adv_sequence_set_slot(dev, i, i);
		if (ret != SUCCESS)
			return ret;
	}

	ret = ad469x_sequence_disable_temp(dev);
	if (ret != SUCCESS)
		return ret;

	ret = ad469x_set_channel_sequence(dev, AD469x_advanced_seq);
	if (ret != SUCCESS)
		return ret;

	ret = ad469x_enter_conversion_mode(dev);
	if (ret != SUCCESS)
		return ret;

	printf("CH0\tCH1\tCH2\tCH3\tCH4\tCH5\tCH6\tCH7\tCH8\tCH9\tCH10\tCH11\tCH12\tCH13\tCH14\tCH15\r\n\r\n");
	while (1) {
		ret = ad469x_seq_read_data(dev, buf, AD469x_EVB_SAMPLE_NO);
		if (ret != SUCCESS)
			return ret;

		/* Discard initial 256 values to skip the initial sampling duration */
		for (i = 256; i < AD469x_EVB_SAMPLE_NO; i++) {
			if (i % 8 == 0)
				printf("\r\n");

			printf("0x%x\t0x%x\t", (uint16_t) buf[i], (uint16_t) (buf[i] >> 16));
		}
	}
#else
	if (ad469x_init_param.data_mode == 1) {
		printf("\r\nContinuous Mode only applicable in Standard Sequencer Mode. Exiting.\r\n");
		return FAILURE;
	}

	printf("\r\nIn Single Cycle Mode\r\n");
	uint32_t channel = 0;
	ret = ad469x_set_channel_sequence(dev, AD469x_single_cycle);
	if (ret != SUCCESS)
		return ret;

	ret = ad469x_enter_conversion_mode(dev);
	if (ret != SUCCESS)
		return ret;

	ret = ad469x_read_data(dev, channel, buf, AD469x_EVB_SAMPLE_NO);
	if (ret != SUCCESS)
		return ret;

	for (i = 0; i < (AD469x_EVB_SAMPLE_NO / 2); i++)
		printf("CH%lu: 0x%x\r\n", channel, (uint16_t) buf[i]);
#endif

	printf("Success\n\r");

	return SUCCESS;
}

