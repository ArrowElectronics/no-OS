/***************************************************************************//**
 *   @file   ad7768_evb.c
 *   @brief  Implementation of Main Function.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2016(c) Analog Devices, Inc.
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

#include <stdio.h>
#include <stdlib.h>
#include "stdint.h"
#include "stdbool.h"
#include "string.h"
#include <inttypes.h>
#include "includes/socal.h"
#include "includes/hps.h"
#include "includes/hwlib.h"
#include "includes/parameters.h"
#include "includes/gpio.h"
#include "includes/spi.h"
#include "includes/spi_extra.h"
#include "includes/gpio_extra.h"
#include "includes/ad7768.h"
#include "includes/error.h"
#include "includes/alt_acpidmap.h"
#include "includes/alt_address_space.h"
#include "includes/arm_cache_modified.h"
#include "includes/fpga_dmac_api.h"
#include "includes/alt_cache.h"
#include "includes/axi_dmac.h"
#include "includes/axi_adc_core.h"
#include "includes/axi_io.h"


#ifdef IIO_SUPPORT
#include "app_iio.h"
#endif

/* Select the channels here which you want */
#define SELECT_CHANNEL (1<<CH7 | 1<<CH6 | 1<<CH5 | 1<<CH4 | 1<<CH3 | 1<<CH2 | 1<<CH1 | 1<<CH0);

/***************************************************************************//**
* @brief ad7768evb_clear_status
*******************************************************************************/
void ad7768_evb_clear_status(struct altera_gpio_init_param *brd_gpio_init)
{
	uint8_t i;
	struct gpio_init_param temp_init = {
		.number = 0,
		.platform_ops = &altera_gpio_platform_ops,
		.extra = brd_gpio_init
	};
	struct gpio_desc *temp_desc;
	int32_t ret;

	for (i = 0; i < 32; i ++) {
		temp_init.number = UP_STATUS_OFFSET + i;
		ret = gpio_get(&temp_desc, &temp_init);
		if (ret != SUCCESS)
			return;
		gpio_direction_output(temp_desc, GPIO_HIGH);
		ret = gpio_remove(temp_desc);
		if (ret != SUCCESS)
			return;
	}
}

/***************************************************************************//**
* @brief ad7768evb_verify_status
*******************************************************************************/
uint8_t ad7768_evb_verify_status(struct altera_gpio_init_param *brd_gpio_init)
{
	uint8_t i;
	uint8_t value = 0;
	uint8_t status = 0;
	struct gpio_init_param temp_init = {
		.number = 0,
		.platform_ops = &altera_gpio_platform_ops,
		.extra = brd_gpio_init
	};
	struct gpio_desc *temp_desc;
	int32_t ret;

	for (i = 0; i < 32; i ++) {
		temp_init.number = UP_STATUS_OFFSET + i;
		ret = gpio_get(&temp_desc, &temp_init);
		if (ret != SUCCESS)
			return FAILURE;
		gpio_direction_input(temp_desc);
		gpio_get_value(temp_desc, &value);
		ret = gpio_remove(temp_desc);
		if (ret != SUCCESS)
			return FAILURE;
		status += value;
	}

	return status;
}

static int32_t ad7768_if_gpio_setup(uint32_t gpio_no, uint8_t gpio_val)
{
	struct altera_gpio_init_param ps_gpio_init = {
		.device_id = GPIO_DEVICE_ID,
		.type = HWLIB_GPIO,
		.base_address = 0xff220020
	};
	struct gpio_init_param temp_init = {
		.number = gpio_no,
		.platform_ops = &altera_gpio_platform_ops,
		.extra = &ps_gpio_init
	};
	struct gpio_desc *temp_desc;
	int32_t ret;

	ret = gpio_get(&temp_desc, &temp_init);
	if (ret != SUCCESS)
		return FAILURE;
	ret = gpio_direction_output(temp_desc, gpio_val);
	if (ret != SUCCESS)
		return FAILURE;
	return gpio_remove(temp_desc);
}

/***************************************************************************//**
* @brief main
*******************************************************************************/
int main(int argc, char** argv)
{
	printf("\nAD7768_NO_OS: main() start here...\r\n");
	ad7768_dev	*dev;
	struct altera_gpio_init_param axi_gpio_init;
	
	struct axi_dmac_init dma_initial = {
		.base = AD7768_DMA_BASEADDR,
		.direction = DMA_DEV_TO_MEM,
		.flags = 0,
		.name = "ad7768_dma"
	};
	struct axi_dmac *dma_desc;
	int32_t ret;
	uint32_t data_size;
	/*const uint32_t chan_no = AD7768_CH_NO, resolution = AD7768_RESOLUTION,
		       sample_no = 1024;*/
	const uint32_t resolution = AD7768_RESOLUTION, sample_no = 1024;
	uint8_t chan_no = SELECT_CHANNEL;
	struct altera_spi_init_param altera_spi_initial = {
		.base_address = 0xff220060,
		.type = HWLIB_SPI
	};

	struct altera_gpio_init_param altera_gpio_initial = {
		.device_id = GPIO_DEVICE_ID,
		.type = HWLIB_GPIO,
	    .base_address = 0xff220050
	};

	ad7768_init_param default_init_param = {
		/* SPI */
		.spi_init = {
			.device_id = SPI_DEVICE_ID,
			.max_speed_hz = 1000000,
			.chip_select = SPI_AD7768_CS,
			.mode = SPI_MODE_0,
			.platform_ops = &altera_platform_ops,
			.extra = &altera_spi_initial
		},
		/* GPIO */
		.gpio_reset = {
			.number = GPIO_RESET_N,
			.platform_ops = &altera_gpio_platform_ops,
			.extra = &altera_gpio_initial
		},
		.gpio_mode0 = {
			.number = GPIO_MODE_0_GPIO_0,
			.platform_ops = &altera_gpio_platform_ops,
			.extra = &altera_gpio_initial
		},
		.gpio_mode1 = {
			.number = GPIO_MODE_1_GPIO_1,
			.platform_ops = &altera_gpio_platform_ops,
			.extra = &altera_gpio_initial
		},
		.gpio_mode2 = {
			.number = GPIO_MODE_2_GPIO_2,
			.platform_ops = &altera_gpio_platform_ops,
			.extra = &altera_gpio_initial
		},
		.gpio_mode3 = {
			.number = GPIO_MODE_3_GPIO_3,
			.platform_ops = &altera_gpio_platform_ops,
			.extra = &altera_gpio_initial
		},
		.gpio_reset_value = GPIO_HIGH,
		/* Configuration */
		.pin_spi_input_value = AD7768_SPI_CTRL,
		.sleep_mode = AD7768_ACTIVE,
		.power_mode = AD7768_FAST,
		.mclk_div = AD7768_MCLK_DIV_4,
		.dclk_div = AD7768_DCLK_DIV_1,
		.conv_op = AD7768_STANDARD_CONV,
		.crc_sel = AD7768_NO_CRC
	};
	struct axi_adc *axi_adc_core_desc;
	struct axi_adc_init axi_adc_initial = {
		.base = AD7768_ADC_BASEADDR,
		.name = "ad7768_axi_adc",
		.num_channels = chan_no
	};
    int32_t *data_ptr = NULL, i;
    int32_t ch_data = 0;
    int32_t ch_no = 0, j = 0, k = 0;

#if defined(__ARMCC_VERSION) && defined(PRINTF_UART)
/* Without Semihosting we don't have an argc/argv. Attempting to use them causes linker errors */
    int argc_=0; char **argv_=NULL;
  #define argc argc_
  #define argv argv_
#endif

	ret = ad7768_if_gpio_setup(GPIO_UP_SSHOT, GPIO_LOW);
	if (ret != 0)
		return ret;

	/* Configure HDL */
	ret = ad7768_if_gpio_setup(GPIO_UP_FORMAT_1, GPIO_LOW);
	if (ret != 0)
		return ret;

	ret = ad7768_if_gpio_setup(GPIO_UP_FORMAT_0, GPIO_LOW);
	if (ret != 0)
		return ret;

	ret = ad7768_if_gpio_setup(GPIO_UP_CRC_ENABLE,
				   default_init_param.crc_sel ? GPIO_LOW : GPIO_LOW);
	if (ret != 0)
		return ret;

	ret = ad7768_if_gpio_setup(GPIO_UP_CRC_4_OR_16_N,
				   (default_init_param.crc_sel == AD7768_CRC_4) ? GPIO_LOW : GPIO_LOW);
	if (ret != 0)
		return ret;

	ad7768_setup(&dev, default_init_param);

	ret = axi_adc_init(&axi_adc_core_desc, &axi_adc_initial);
	if (ret != SUCCESS)
		return FAILURE;

	axi_gpio_init.type = HWLIB_GPIO;
	axi_gpio_init.device_id = GPIO_STATUS_DEVICE_ID;

	ad7768_evb_clear_status(&axi_gpio_init);
	if (ad7768_evb_verify_status(&axi_gpio_init))
		printf("Interface errors\r\n");
	else
		printf("Interface OK\r\n");

	ret = axi_dmac_init(&dma_desc, &dma_initial);
	if (ret != SUCCESS)
		return FAILURE;

	for(k = 0; k < 8; k++) {
		if(chan_no & (1 << k))
			ch_no += 1;
	}
	data_size = (sample_no * ch_no *
			     ((resolution + AD7768_HEADER_SIZE) / BITS_IN_BYTE));

	printf("Capture samples...\r\n");
	ret = axi_dmac_transfer(dma_desc, ADC_DDR_BASEADDR, data_size);
	if (ret != SUCCESS)
		return FAILURE;
	printf("Capture done\r\n");

	if (ad7768_evb_verify_status(&axi_gpio_init))
		printf("Interface errors\r\n");
	else
		printf("Interface OK\r\n");
		
	alt_cache_system_invalidate((void *)ADC_DDR_BASEADDR, sample_no  *32 );	

	printf("    CH0     CH1     CH2     CH3     CH4     CH5     CH6     CH7   ");
	printf("\n\r");
	for (i = 0; i < (sample_no * ch_no); i++) {
		for( ; j < 8; ) {
				if(chan_no & (1 << j++)) {
					data_ptr = (int32_t *)(ADC_DDR_BASEADDR + (i * sizeof(uint32_t)));
					ch_data = ((int)((*data_ptr) & 0x007FFFFF));
					printf("%8.5f ", ((float)ch_data * 0.000000488));
					break;
				}
				else {
					printf("        ");
				}
		}
		if(j == 8) {
			j = 0;
			if(!(chan_no & (1 << 7)))
				i--;
			printf("\n\r");
		}
	}

#ifdef IIO_SUPPORT
	printf("The board accepts libiio clients connections through the serial backend.\n");

	struct iio_axi_adc_init_param iio_axi_adc_init_par;
	iio_axi_adc_init_par = (struct iio_axi_adc_init_param) {
		.rx_adc = axi_adc_core_desc,
		.rx_dmac = dma_desc,
		.dcache_invalidate_range = (void (*)(uint32_t,
						     uint32_t))Xil_DCacheInvalidateRange
	};

	return iio_app_start(&iio_axi_adc_init_par);
#endif
	printf("\r\nBye...\r\n");
	return 0;
}
