/***************************************************************************//**
 *   @file   ad463x.c
 *   @brief  Implementation of AD463x Driver.
 *   @author Antoniu Miclaus (antoniu.miclaus@analog.com)
********************************************************************************
 * Copyright 2021(c) Analog Devices, Inc.
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
#include <stdlib.h>
#include <stdio.h>
#include "includes/error.h"
#include "includes/delay.h"
#include "includes/ad463x.h"
#include "includes/axi_io.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
#define AD463x_TEST_DATA 0xAA

static struct ad463x_chip_info ad463x_chip_info_tbl[] = {
	[ID_AD4630_24] = {
		.num_channels = 2,
		.freq_samp = 2000000,
		.cnv_width = 0x25,
		.cnv_rate_4_lane =  0x50,
		.cnv_rate_2_lane =  0x50,
		.cnv_rate_1_lane_sdr =  0x60,
		.cnv_rate_1_lane_ddr =  0x50,
	}
};

/******************************************************************************/
/************************* Functions Definitions ******************************/
/******************************************************************************/

/**
 * @brief Read device register.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param reg_data - The data read from the register.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t ad463x_spi_reg_read(struct ad463x_dev *dev,
			    uint16_t reg_addr,
			    uint8_t *reg_data)
{
	int32_t ret;
	uint8_t buf[3];

	buf[0] = AD463X_REG_READ| ((reg_addr >> 8) & 0x7F);
	buf[1] = (uint8_t)reg_addr;
	buf[2] = AD463X_REG_READ_DUMMY;

	ret = spi_write_and_read(dev->spi_desc, buf, 3);
	*reg_data = buf[2];

	return ret;
}

/**
 * @brief Write device register.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param reg_data - The register data.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t ad463x_spi_reg_write(struct ad463x_dev *dev,
			     uint16_t reg_addr,
			     uint8_t reg_data)
{
	uint8_t buf[3];

	buf[0] = ((reg_addr >> 8) & 0x7F);
	buf[1] = (uint8_t)reg_addr;
	buf[2] = reg_data;

	return spi_write_and_read(dev->spi_desc, buf, 3);
}

/**
 * @brief SPI write device register using a mask.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param mask - The mask.
 * @param data - The register data.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t ad463x_spi_reg_write_masked(struct ad463x_dev *dev,
				    uint16_t reg_addr,
				    uint8_t mask,
				    uint8_t data)
{
	uint8_t reg_data;
	int32_t ret;

	ret = ad463x_spi_reg_read(dev, reg_addr, &reg_data);
	if (ret != SUCCESS)
		return ret;

	reg_data &= ~mask;
	reg_data |= data;

	return ad463x_spi_reg_write(dev, reg_addr, reg_data);
}

/**
 * @brief Set power mode.
 * @param dev - The device structure.
 * @param mode - The power mode.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t ad463x_set_pwr_mode(struct ad463x_dev *dev, uint8_t mode)
{
	if ((mode != AD463X_NORMAL_MODE) && (mode != AD463X_LOW_POWER_MODE))
		return FAILURE;

	return ad463x_spi_reg_write(dev, AD463X_REG_DEVICE_CONFIG, mode);
}

/**
 * @brief Set average frame length.
 * @param dev - The device structure.
 * @param mode - Average filter frame length mode.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t ad463x_set_avg_frame_len(struct ad463x_dev *dev, uint8_t mode)
{
	int32_t ret;

	if (mode == AD463X_NORMAL_MODE) {
		ret = ad463x_spi_reg_write_masked(dev,
						  AD463X_REG_MODES,
						  AD463X_OUT_DATA_MODE_MSK,
						  AD463X_24_DIFF_8_COM);
		if (ret != SUCCESS)
			return ret;
		ret = ad463x_spi_reg_write(dev,
					   AD463X_REG_AVG,
					   AD463X_AVG_FILTER_RESET);
	} else {
		if(mode < 1 || mode > 16)
			return -EINVAL;
		ret = ad463x_spi_reg_write_masked(dev,
						  AD463X_REG_MODES,
						  AD463X_OUT_DATA_MODE_MSK,
						  AD463X_30_AVERAGED_DIFF);
		if (ret != SUCCESS)
			return ret;
		ret = ad463x_spi_reg_write(dev,
					   AD463X_REG_AVG,
					   mode);
	}

	return ret;
}

/**
 * @brief Set drive strength.
 * @param dev - The device structure.
 * @param mode - The register data.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t ad463x_set_drive_strength(struct ad463x_dev *dev, uint8_t mode)
{
	if((mode != AD463X_NORMAL_OUTPUT_STRENGTH)
	    && (mode != AD463X_DOUBLE_OUTPUT_STRENGTH))
		return -EINVAL;

	return ad463x_spi_reg_write_masked(dev, AD463X_REG_IO,
					   AD463X_DRIVER_STRENGTH_MASK, mode);
}

/**
 * @brief Exit register configuration mode.
 * @param dev - The device structure.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t ad463x_exit_reg_cfg_mode(struct ad463x_dev *dev)
{
	int32_t ret;

	ret = ad463x_spi_reg_write(dev, AD463X_REG_EXIT_CFG_MODE,
				   AD463X_EXIT_CFG_MODE);
	if (ret != SUCCESS)
		return ret;

	ret = spi_engine_set_transfer_width(dev->spi_desc,
					    dev->capture_data_width);
	if (ret != SUCCESS)
		return ret;

	return ret;
}

/**
 * @brief Set channel gain
 * @param dev - The device structure.
 * @param ch_idx - The channel index.
 * @param gain - The gain value scaled by 10000.
 * 			Example: to set gain 1.5, use 150000
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t ad463x_set_ch_gain(struct ad463x_dev *dev, uint8_t ch_idx,
			   uint64_t gain)
{
	int32_t ret;
	uint32_t g;

	if (gain < 0 || gain > AD463X_GAIN_MAX_VAL_SCALED)
		return -EINVAL;

	g = ((gain * 0xFFFF) / AD463X_GAIN_MAX_VAL_SCALED);

	ret = ad463x_spi_reg_write(dev,
				   AD463X_REG_CHAN_GAIN(ch_idx, AD463X_GAIN_LSB),
				   g);
	if (ret != SUCCESS)
		return ret;

	return ad463x_spi_reg_write(dev,
				    AD463X_REG_CHAN_GAIN(ch_idx, AD463X_GAIN_MSB),
				    g >> 8);
}

/**
 * @brief Set channel offset
 * @param dev - The device structure.
 * @param ch_idx - The channel index.
 * @param offset - The channel offset.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t ad463x_set_ch_offset(struct ad463x_dev *dev, uint8_t ch_idx,
			     uint32_t offset)
{
	int32_t ret;

	ret = ad463x_spi_reg_write(dev,
				   AD463X_REG_CHAN_OFFSET(ch_idx, AD463X_OFFSET_0),
				   offset);
	if (ret < 0)
		return ret;

	ret = ad463x_spi_reg_write(dev,
				   AD463X_REG_CHAN_OFFSET(ch_idx, AD463X_OFFSET_1),
				   offset >> 8);
	if (ret < 0)
		return ret;

	return ad463x_spi_reg_write(dev,
				    AD463X_REG_CHAN_OFFSET(ch_idx, AD463X_OFFSET_2),
				    offset >> 16);
}

/**
 * @brief Perform device hard reset using axi_io APIs
 * @param void
 * @return \ref SUCCESS in case of success, \ref FAILURE otherwise.
 */
static int32_t ad469x_reset(void)
{
	uint32_t gpiofpga_addr = 0xff220060;
	uint32_t value = 0;
	uint32_t dir_reg = 0x04;
	uint32_t data_reg = 0x00;

	axi_io_read(gpiofpga_addr, dir_reg, &value);
	value |= 0x01;	// adding value of bit no 8 to initial value of dir register
	axi_io_write(gpiofpga_addr, dir_reg, value);	//setting the gpio pin to output .

	value = 0;
	axi_io_read(gpiofpga_addr, data_reg, &value);
	value &= ~(0x01);	// RST pin is active low
	axi_io_write(gpiofpga_addr, data_reg, value);

	udelay(10);

	value = 0;
	axi_io_read(gpiofpga_addr, data_reg, &value);
	value |= 0x01;
	axi_io_write(gpiofpga_addr, data_reg, value);

	return SUCCESS;
}


/**
 * @brief Get data from modes register and store it in a variable
 * @param [in] m_info - Mode information struct.
 * @param [out] dev - ad469x_dev device handler.
 * @return \ref SUCCESS in case of success, \ref FAILURE otherwise.
 */
static int32_t get_mode_info(struct ad463x_dev *dev, struct mode_info *m_info)
{
	int32_t ret;
	uint8_t modes;

	ret = ad463x_spi_reg_read(dev, AD463X_REG_MODES, &modes);
	if (ret != SUCCESS)
		return ret;

	/* Lane Mode */
	m_info->lane_mode = (modes & 0xC0);
	/* Clock Mode */
	m_info->clock_mode = (modes & 0x30);
	/* DDR Mode */
	m_info->ddr_mode = (modes & 0x08);
	/* Out Data Mode */
	m_info->out_data_mode = (modes & 0x07);
	return 0;
}

/**
 * @brief Perform initial boot-up configuration of the device
 * @param [out] dev - ad469x_dev device handler.
 * @return \ref SUCCESS in case of success, \ref FAILURE otherwise.
 */
static int32_t ad463x_device_config(struct ad463x_dev *dev)
{
	int32_t ret;

	switch (dev->device_id) {
	case ID_AD4630_24:
		ret = ad463x_spi_reg_write(dev, AD463X_REG_INTERFACE_CONFIG_A, 0x10);
		if (ret != SUCCESS)
			return ret;

		ret = ad463x_spi_reg_write(dev, AD463X_REG_INTERFACE_CONFIG_B, 0x80);
		if (ret != SUCCESS)
			return ret;

		ret = ad463x_spi_reg_write(dev, AD463X_REG_DEVICE_CONFIG, 0x00);
			if (ret != SUCCESS)
				return ret;

		ret = ad463x_spi_reg_write(dev, AD463X_REG_AVG, 0x00);
		if (ret != SUCCESS)
			return ret;

		ret = ad463x_spi_reg_write(dev, AD463X_REG_OSCILATOR, 0x00);
		if (ret != SUCCESS)
			return ret;

		ret = ad463x_spi_reg_write(dev, AD463X_REG_PAT0, 0x00);
		if (ret != SUCCESS)
			return ret;

		ret = ad463x_spi_reg_write(dev, AD463X_REG_PAT1, 0x30);
		if (ret != SUCCESS)
			return ret;

		ret = ad463x_spi_reg_write(dev, AD463X_REG_PAT2, 0x46);
		if (ret != SUCCESS)
			return ret;

		ret = ad463x_spi_reg_write(dev, AD463X_REG_PAT3, 0xAD);
		if (ret != SUCCESS)
			return ret;

		ret = axi_io_write(dev->spi_engine_baseaddr, UP_CONFIG_REG, 0x42); // set data format register to default
		if (ret != SUCCESS)
			return ret;

		break;
	default:
		printf("NOTE: Device may not fully supported yet.\r\n \
				The program will not exit, but output may not be as intended.\r\n");
	}

	return SUCCESS;
}

/**
 * @brief Read from device.
 *        Enter register mode to read/write registers
 * @param [in] dev - ad469x_dev device handler.
 * @param [out] buf - data buffer.
 * @param [in] samples - sample number.
 * @return \ref SUCCESS in case of success, \ref FAILURE otherwise.
 */
int32_t ad463x_read_data(struct ad463x_dev *dev,
			 uint64_t *buf,
			 uint16_t samples)
{
	int32_t ret;
	uint32_t commands_data[1] = {0};
	struct spi_engine_offload_message msg;

	if (dev->lane_mode == AD463X_FOUR_LANES_PER_CH) {
		uint32_t spi_eng_msg_cmds[3] = {CS_LOW, READ(1), CS_HIGH};
		msg.commands = spi_eng_msg_cmds;
		msg.no_commands = ARRAY_SIZE(spi_eng_msg_cmds);
	} else if (dev->lane_mode == AD463X_TWO_LANES_PER_CH) {
		uint32_t spi_eng_msg_cmds[4] = {CS_LOW, READ(1), READ(1), CS_HIGH};
		msg.commands = spi_eng_msg_cmds;
		msg.no_commands = ARRAY_SIZE(spi_eng_msg_cmds);

	} else {
		uint32_t spi_eng_msg_cmds[6] = {CS_LOW, READ(1), READ(1), READ(1), READ(1), CS_HIGH};
		msg.commands = spi_eng_msg_cmds;
		msg.no_commands = ARRAY_SIZE(spi_eng_msg_cmds);
	}

	axi_io_write(dev->spi_engine_baseaddr, CNV_CONFIG_REG, CNV_ENABLE(1) | UP_RESETN(1));

	ret = spi_engine_offload_init(dev->spi_desc, dev->offload_init_param);
	if (ret != SUCCESS)
		return ret;

	msg.rx_addr = (uint32_t)buf;
	msg.commands_data = commands_data;

	ret = spi_engine_offload_transfer(dev->spi_desc, msg, samples);
	if (ret != SUCCESS)
		return ret;

	if (dev->dcache_invalidate_range)
		dev->dcache_invalidate_range((void *) msg.rx_addr, samples * 64);

	return ret;
}

/**
 * @brief Initialize the device.
 * @param [out] device - The device structure.
 * @param [in] init_param - The structure that contains the device initial
 * 		parameters.
 * @return \ref SUCCESS in case of success, \ref FAILURE otherwise.
 */
int32_t ad463x_init(struct ad463x_dev **device,
		    struct ad463x_init_param *init_param)
{
	printf("In ad463x_init \r\n");
	struct ad463x_dev *dev;
	int32_t ret;
	uint8_t data = 0;
	uint8_t hdl_reg = 0;
	struct mode_info m_info;

	if (!init_param || !device)
		return FAILURE;

	if (init_param->clock_mode == AD463X_SPI_CLOCK_MODE &&
	    init_param->data_rate == AD463X_DDR_MODE) {
		printf("DDR_MODE not available when clock mode is SPI_COMPATIBLE\r\n");
		return FAILURE;
	}

	dev = (struct ad463x_dev *)malloc(sizeof(*dev));
	if (!dev)
		return FAILURE;

	ret = ad469x_reset();
	if (ret != SUCCESS)
		goto error_dev;

	udelay(750);

	ret = spi_init(&dev->spi_desc, init_param->spi_init);
	if (ret != SUCCESS)
		goto error_spi;

	dev->offload_init_param = init_param->offload_init_param;
	dev->reg_data_width = init_param->reg_data_width;
	dev->capture_data_width = init_param->capture_data_width;
	dev->output_mode = init_param->output_mode;
	dev->lane_mode = init_param->lane_mode;
	dev->clock_mode = init_param->clock_mode;
	dev->data_rate = init_param->data_rate;
	dev->device_id = init_param->device_id;
	dev->spi_engine_baseaddr = init_param->spi_engine_baseaddr;
	dev->dcache_invalidate_range = init_param->dcache_invalidate_range;

	if (dev->data_rate == AD463X_DDR_MODE)
		dev->capture_data_width /= 2;

	ret = spi_engine_set_transfer_width(dev->spi_desc, dev->reg_data_width);
	if (ret != SUCCESS)
		goto error_spi;

	spi_engine_set_speed(dev->spi_desc, dev->spi_desc->max_speed_hz);

	ret = ad463x_spi_reg_read(dev, AD463X_CONFIG_TIMING, &data);
	if (ret != SUCCESS)
		goto error_spi;

	ret = ad463x_device_config(dev);
	if (ret != SUCCESS)
		goto error_spi;

	do {
		ret = ad463x_spi_reg_read(dev, AD463X_REG_CHIP_TYPE, &data);
		if (ret != SUCCESS)
			goto error_spi;

		printf("AD463X_REG_CHIP_TYPE = 0x%x\r\n", data);

		ret = ad463x_spi_reg_read(dev, AD463X_REG_SPI_REVISION, &data);
		if (ret != SUCCESS)
			goto error_spi;

		printf("AD463X_REG_SPI_REVISION = 0x%x\r\n", data);

		ret = ad463x_spi_reg_read(dev, AD463X_REG_VENDOR_L, &data);
		if (ret != SUCCESS)
			goto error_spi;

		printf("AD463X_REG_VENDOR_L = 0x%x\r\n", data);

		ret = ad463x_spi_reg_write(dev, AD463X_REG_SCRATCH_PAD, AD463x_TEST_DATA);
		if (ret != SUCCESS)
			goto error_spi;

		ret = ad463x_spi_reg_read(dev, AD463X_REG_SCRATCH_PAD, &data);
		if (ret != SUCCESS)
			goto error_spi;

		printf("AD463X_REG_SCRATCH_PAD = 0x%x\r\n", data);

		if (data != AD463x_TEST_DATA) {
			printf("Test Data read failed. Check device configurations again.\r\n");
		}
	} while (data != AD463x_TEST_DATA);

	/* Set data mode register to required values */
	ret = ad463x_spi_reg_write_masked(dev, AD463X_REG_MODES, AD463X_LANE_MODE_MSK,
					  dev->lane_mode);
	if (ret != SUCCESS)
		return ret;

	ret = ad463x_spi_reg_write_masked(dev, AD463X_REG_MODES, AD463X_CLK_MODE_MSK,
					  dev->clock_mode);
	if (ret != SUCCESS)
		return ret;

	ret = ad463x_spi_reg_write_masked(dev, AD463X_REG_MODES, AD463X_DDR_MODE_MSK,
					  dev->data_rate);
	if (ret != SUCCESS)
		return ret;

	ret = ad463x_spi_reg_write_masked(dev, AD463X_REG_MODES,
					  AD463X_OUT_DATA_MODE_MSK, dev->output_mode);
	if (ret != SUCCESS)
		return ret;

	ret = get_mode_info(dev, &m_info);
	if (ret != SUCCESS)
		return ret;

    hdl_reg = 1 << (4 + (m_info.lane_mode >> 6));
    hdl_reg |= (m_info.clock_mode >> 2);
    hdl_reg |= (m_info.ddr_mode >> 2);
    hdl_reg |= (m_info.out_data_mode >> 2);

    axi_io_write(dev->spi_engine_baseaddr, UP_CONFIG_REG, hdl_reg); // update mode configuration in HDL
	axi_io_write(dev->spi_engine_baseaddr, CNV_CONFIG_REG, 0x00); // to disable up_resetn and up_cnv pulse
	if (dev->lane_mode == AD463X_FOUR_LANES_PER_CH) {
		axi_io_write(dev->spi_engine_baseaddr, CNV_RATE_CONFIG, ad463x_chip_info_tbl[dev->device_id].cnv_rate_4_lane); // value to calculate as 1000ns*spi_clk_frequency **
	}
	else if(dev->lane_mode == AD463X_TWO_LANES_PER_CH) {
		axi_io_write(dev->spi_engine_baseaddr, CNV_RATE_CONFIG, ad463x_chip_info_tbl[dev->device_id].cnv_rate_2_lane); // value to calculate as 1000ns*spi_clk_frequency **
	}
	else if(dev->lane_mode == AD463X_ONE_LANE_PER_CH) {
		if (dev->data_rate == AD463X_DDR_MODE)
			axi_io_write(dev->spi_engine_baseaddr, CNV_RATE_CONFIG, ad463x_chip_info_tbl[dev->device_id].cnv_rate_1_lane_ddr); // value to calculate as 1000ns*spi_clk_frequency **
		else
			axi_io_write(dev->spi_engine_baseaddr, CNV_RATE_CONFIG, ad463x_chip_info_tbl[dev->device_id].cnv_rate_1_lane_sdr); // value to calculate as 1000ns*spi_clk_frequency **
	}
	axi_io_write(dev->spi_engine_baseaddr, CNV_PULSE_WIDTH, ad463x_chip_info_tbl[dev->device_id].cnv_width); // value to calculate as 250ns*spi_clk_frequency  **

	uint32_t hdl_val = 0;
	uint8_t reg_val = 0;
	ad463x_spi_reg_read(dev, AD463X_REG_MODES, &reg_val);
	axi_io_read(dev->spi_engine_baseaddr, UP_CONFIG_REG, &hdl_val);
	printf("Mode Register = 0x%x\tHDL Config Register = 0x%lx\r\n", reg_val, hdl_val);

	*device = dev;

	return ret;

error_spi:
	spi_remove(dev->spi_desc);
error_dev:
	free(dev);

	return FAILURE;
}

/**
 * @brief Free the memory allocated by ad463x_init().
 * @param [in] dev - Pointer to the device handler.
 * @return \ref SUCCESS in case of success, \ref FAILURE otherwise
 */
int32_t ad463x_remove(struct ad463x_dev *dev)
{
	int32_t ret;

	if (!dev)
		return FAILURE;

	ret = spi_remove(dev->spi_desc);
	if (ret != SUCCESS)
		return ret;

	free(dev);

	return ret;
}
