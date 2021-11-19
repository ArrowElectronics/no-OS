/***************************************************************************//**
 *   @file   ad469x.c
 *   @brief  Implementation of ad69x Driver.
 *   @author Cristian Pop (cristian.pop@analog.com)
********************************************************************************
 * Copyright 2020(c) Analog Devices, Inc.
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
#include <string.h>
#include "stdio.h"
#include "stdlib.h"
#include "includes/ad469x.h"
#include "includes/spi_engine.h"
#include "includes/error.h"
#include "includes/util.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
#define AD469x_TEST_DATA 0xEA

/**
 * @brief Device resolution
 */
const uint8_t ad469x_device_resol[] = {
	[AD469x_OSR_1] = 16,
	[AD469x_OSR_4] = 17,
	[AD469x_OSR_16] = 18,
	[AD469x_OSR_64] = 19
};

static struct ad469x_chip_info ad469x_chip_info_tbl[] = {
	[ID_AD4695] = {
		.num_channels = 16,
		.freq_samp = 500000,
		.cnv_width = 0x30,
		.cnv_rate =  0x140,
	},
	[ID_AD4696] = {
		.num_channels = 16,
		.freq_samp = 1000000,
		.cnv_width = 0x30,
		.cnv_rate =  0xA0,
	},
};

/******************************************************************************/
/************************** Functions Implementation **************************/
/******************************************************************************/

/**
 * Read from device.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param reg_data - The register data.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t ad469x_spi_reg_read(struct ad469x_dev *dev,
			    uint16_t reg_addr,
			    uint16_t *reg_data)
{
	int32_t ret;
	uint8_t buf[4];

	ret = spi_engine_set_transfer_width(dev->spi_desc, dev->reg_data_width);
	if (ret != SUCCESS)
		return ret;

	buf[0] = (1 << 7) | ((reg_addr >> 8) & 0x7F);
	buf[1] = 0xFF & reg_addr;
	buf[2] = 0xFF;
	buf[3] = 0xFF;

	ret = spi_write_and_read(dev->spi_desc, buf, 4);

	if ((reg_addr == 0x25) | ((reg_addr >= 0x40) & (reg_addr < 0x100))) {
		reg_data[0] = buf[3];
		reg_data[1] = buf[2];
	}
	else {
		reg_data[0] = buf[2];
		reg_data[1] = buf[3];
	}

	ret = spi_engine_set_transfer_width(dev->spi_desc,
					    dev->capture_data_width);
	if (ret != SUCCESS)
		return ret;

	return ret;
}

/**
 * Write to device.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param reg_data - The register data.
 * @@eturn SUCCESS in case of success, negative error code otherwise.
 */
int32_t ad469x_spi_reg_write(struct ad469x_dev *dev,
			     uint16_t reg_addr,
			     uint16_t reg_data)
{
	int32_t ret;
	uint8_t buf[4];

	ret = spi_engine_set_transfer_width(dev->spi_desc, dev->reg_data_width);
	if (ret != SUCCESS)
		return ret;

	buf[0] = ((reg_addr >> 8) & 0x7F);
	buf[1] = 0xFF & reg_addr;

	if ((reg_addr == 0x25) | ((reg_addr >= 0x40) & (reg_addr < 0x100))) {
		buf[3] = reg_data;
		buf[2] = reg_data >> 8;
	}
	else {
		buf[2] = reg_data ;
		buf[3] = reg_data >> 8;
	}

	ret = spi_write_and_read(dev->spi_desc, buf, 4);

	ret = spi_engine_set_transfer_width(dev->spi_desc,
					    dev->capture_data_width);
	if (ret != SUCCESS)
		return ret;

	return ret;
}

/**
 * SPI read from device using a mask.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param mask - The mask.
 * @param data - The register data.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t ad469x_spi_read_mask(struct ad469x_dev *dev,
			     uint16_t reg_addr,
			     uint8_t mask,
			     uint8_t *data)
{
	uint16_t reg_data[2];
	int32_t ret;

	ret = ad469x_spi_reg_read(dev, reg_addr, reg_data);
	if (ret != SUCCESS)
		return ret;

	*data = (reg_data[0] & mask);

	return ret;
}

/**
 * SPI write to device using a mask.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param mask - The mask.
 * @param data - The register data.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t ad469x_spi_write_mask(struct ad469x_dev *dev,
			      uint16_t reg_addr,
			      uint8_t mask,
			      uint8_t data)
{
	uint16_t reg_data;
	int32_t ret;

	ret = ad469x_spi_reg_read(dev, reg_addr, &reg_data);
	if (ret != SUCCESS)
		return ret;

	reg_data &= ~mask;
	reg_data |= data;

	return ad469x_spi_reg_write(dev, reg_addr, reg_data);
}

/**
 * @brief Configure register access mode
 * @param [in] dev - ad469x_dev device handler.
 * @param [in] access - Access mode
 * @return \ref SUCCESS in case of success, \ref FAILURE otherwise.
 */
int32_t ad469x_set_reg_access_mode(struct ad469x_dev *dev,
				   enum ad469x_reg_access access)
{
	return ad469x_spi_write_mask(dev,
				     AD469x_REG_IF_CONFIG_C,
				     AD469x_REG_IF_CONFIG_C_MB_STRICT_MASK,
				     AD469x_REG_IF_CONFIG_C_MB_STRICT(access));
}

/**
 * @brief Configure over sampling ratio in advanced sequencer mode
 * @param [in] dev - ad469x_dev device handler.
 * @param [in] ch - Channel to configure.
 * @param [in] ratio - OSR ratio.
 * @return \ref SUCCESS in case of success, \ref FAILURE otherwise.
 */
int32_t ad469x_adv_seq_osr(struct ad469x_dev *dev, uint16_t ch,
			   enum ad469x_osr_ratios ratio)
{
	int32_t ret;

	if (dev->ch_sequence == AD469x_single_cycle ||
	    dev->ch_sequence == AD469x_two_cycle)
		return FAILURE;

	if (ch >= AD469x_CHANNEL_NO)
		return FAILURE;

	ret = ad469x_spi_write_mask(dev,
				    AD469x_REG_CONFIG_IN(ch),
				    AD469x_REG_CONFIG_IN_OSR_MASK,
				    AD469x_REG_CONFIG_IN_OSR(AD469x_OSR_1));
	if (ret != SUCCESS)
		return ret;

	dev->adv_seq_osr_resol[ch] = ad469x_device_resol[AD469x_OSR_1];

	return SUCCESS;
}

/**
 * @brief Configure over sampling ratio to 1 in single and two cycle modes.
 * @param [in] dev - ad469x_dev device handler.
 * @return \ref SUCCESS in case of success, \ref FAILURE otherwise.
 */
static int32_t ad469x_seq_osr_clear(struct ad469x_dev *dev)
{
	int32_t ret;
	uint8_t i = 0;

	for (i = 0; i < AD469x_CHANNEL_NO; i++) {
		ret = ad469x_spi_write_mask(dev,
					    AD469x_REG_CONFIG_IN(i),
					    AD469x_REG_CONFIG_IN_OSR_MASK,
					    AD469x_REG_CONFIG_IN_OSR(AD469x_OSR_1));
		if (ret != SUCCESS)
			return ret;
		dev->adv_seq_osr_resol[i] = ad469x_device_resol[AD469x_OSR_1];
	}

	return SUCCESS;
}

/**
 * @brief Configure over sampling ratio in standard sequencer mode
 * @param [in] dev - ad469x_dev device handler.
 * @param [in] ratio - OSR ratio.
 * @return \ref SUCCESS in case of success, \ref FAILURE otherwise.
 */
int32_t ad469x_std_seq_osr(struct ad469x_dev *dev, enum ad469x_osr_ratios ratio)
{
	int ret;

	if (dev->ch_sequence == AD469x_single_cycle ||
	    dev->ch_sequence == AD469x_two_cycle)
		return FAILURE;

	ret = ad469x_spi_write_mask(dev,
				    AD469x_REG_CONFIG_IN(0),
				    AD469x_REG_CONFIG_IN_OSR_MASK,
				    AD469x_REG_CONFIG_IN_OSR(ratio));
	if (ret != SUCCESS)
		return ret;

	return ret;
}

/**
 * @brief Set channel sequence.
 * @param [in] dev - ad469x_dev device handler.
 * @param [in] seq - Channel sequence.
 * @return \ref SUCCESS in case of success, \ref FAILURE otherwise.
 */
int32_t ad469x_set_channel_sequence(struct ad469x_dev *dev,
				    enum ad469x_channel_sequencing seq)
{
	int32_t ret;

	switch (seq) {
	case AD469x_single_cycle:
		ret = ad469x_spi_write_mask(dev,
					    AD469x_REG_SEQ_CTRL,
					    AD469x_SEQ_CTRL_STD_SEQ_EN_MASK,
					    AD469x_SEQ_CTRL_STD_SEQ_EN(0));
		if (ret != SUCCESS)
			return ret;

		ret = ad469x_spi_write_mask(dev,
					    AD469x_REG_SEQ_CTRL,
					    AD469x_SEQ_CTRL_NUM_SLOTS_AS_MASK,
					    AD469x_SEQ_CTRL_NUM_SLOTS_AS(0));
		if (ret != SUCCESS)
			return ret;

		ret = ad469x_spi_write_mask(dev,
					    AD469x_REG_SETUP,
					    AD469x_SETUP_CYC_CTRL_MASK,
					    AD469x_SETUP_CYC_CTRL_SINGLE(0));
		if (ret != SUCCESS)
			return ret;

		ret = ad469x_seq_osr_clear(dev);
		if (ret != SUCCESS)
			return ret;

		break;

	case AD469x_two_cycle:
		ret = ad469x_spi_write_mask(dev,
					    AD469x_REG_SEQ_CTRL,
					    AD469x_SEQ_CTRL_STD_SEQ_EN_MASK,
					    AD469x_SEQ_CTRL_STD_SEQ_EN(0));
		if (ret != SUCCESS)
			return ret;

		ret = ad469x_spi_write_mask(dev,
					    AD469x_REG_SEQ_CTRL,
					    AD469x_SEQ_CTRL_NUM_SLOTS_AS_MASK,
					    AD469x_SEQ_CTRL_NUM_SLOTS_AS(0));
		if (ret != SUCCESS)
			return ret;

		ret = ad469x_spi_write_mask(dev,
					    AD469x_REG_SETUP,
					    AD469x_SETUP_CYC_CTRL_MASK,
					    AD469x_SETUP_CYC_CTRL_SINGLE(1));
		if (ret != SUCCESS)
			return ret;

		ret = ad469x_seq_osr_clear(dev);
		if (ret != SUCCESS)
			return ret;

		break;

	case AD469x_standard_seq:
		ret = ad469x_spi_write_mask(dev,
					    AD469x_REG_SEQ_CTRL,
					    AD469x_SEQ_CTRL_STD_SEQ_EN_MASK,
					    AD469x_SEQ_CTRL_STD_SEQ_EN(1));
		if (ret != SUCCESS)
			return ret;

		break;

	case AD469x_advanced_seq:
		ret = ad469x_spi_write_mask(dev,
					    AD469x_REG_SEQ_CTRL,
					    AD469x_SEQ_CTRL_STD_SEQ_EN_MASK,
					    AD469x_SEQ_CTRL_STD_SEQ_EN(0));
		if (ret != SUCCESS)
			return ret;

		break;

	default:
		return FAILURE;
		break;
	}

	dev->ch_sequence = seq;

	return ret;
}

/**
 * @brief Configure advanced sequencer number of slots, temp channel not
 * included
 * @param [in] dev - ad469x_dev device handler.
 * @param [in] num_slots - Number of slots, max value = 0x7f
 * @return \ref SUCCESS in case of success, \ref FAILURE otherwise.
 */
int32_t ad469x_adv_sequence_set_num_slots(struct ad469x_dev *dev,
		uint8_t num_slots)
{
	int32_t ret;
	uint8_t write_num_slots = 0;

	if (num_slots)
		write_num_slots = num_slots - 1;

	ret = ad469x_spi_write_mask(dev,
				    AD469x_REG_SEQ_CTRL,
				    AD469x_SEQ_CTRL_NUM_SLOTS_AS_MASK,
				    AD469x_SEQ_CTRL_NUM_SLOTS_AS(write_num_slots));
	if (ret != SUCCESS)
		return ret;

	dev->num_slots = num_slots;

	return SUCCESS;
}

/**
 * @brief Advanced sequencer, assign channel to a slot
 * @param [in] dev - ad469x_dev device handler.
 * @param [in] slot - Slot number [0x00, 0x7f]
 * @param [in] channel - Assigned channel [0x00, 0x0f].
 * @return \ref SUCCESS in case of success, \ref FAILURE otherwise.
 */
int32_t ad469x_adv_sequence_set_slot(struct ad469x_dev *dev,
				     uint8_t slot,
				     uint8_t channel)
{
	int32_t ret;

	ret = ad469x_spi_reg_write(dev,
				   AD469x_REG_AS_SLOT(slot),
				   AD469x_REG_AS_SLOT_INX(channel));
	if (ret != SUCCESS)
		return ret;

	dev->ch_slots[slot] = channel;

	return SUCCESS;
}

/**
 * @brief Configure standard sequencer channels
 * @param [in] dev - ad469x_dev device handler.
 * @param [in] ch_mask - Extra channels to activate.
 * @return \ref SUCCESS in case of success, \ref FAILURE otherwise.
 */
int32_t ad469x_std_sequence_ch(struct ad469x_dev *dev, uint16_t ch_mask)
{
	int32_t ret;

	ret = ad469x_spi_reg_write(dev,
			   	   AD469x_REG_STD_SEQ_CONFIG + 1,
				   ch_mask);
	if (ret != SUCCESS)
		return ret;

	dev->num_slots = hweight8(ch_mask);

	return ret;
}

/**
 * @brief Enable temperature read at the end of the sequence, for standard and
 * advanced sequencer
 * @param [in] dev - ad469x_dev device handler.
 * @return \ref SUCCESS in case of success, \ref FAILURE otherwise.
 */
int32_t ad469x_sequence_enable_temp(struct ad469x_dev *dev)
{
	int32_t ret;

	ret = ad469x_spi_write_mask(dev,
				    AD469x_REG_TEMP_CTRL,
				    AD469x_REG_TEMP_CTRL_TEMP_EN_MASK,
				    AD469x_REG_TEMP_CTRL_TEMP_EN(1));
	if (ret != SUCCESS)
		return ret;

	dev->temp_enabled = true;

	return ret;
}

/**
 * @brief Disable temperature read at the end of the sequence, for standard and
 * advanced sequencer
 * @param [in] dev - ad469x_dev device handler.
 * @return \ref SUCCESS in case of success, \ref FAILURE otherwise.
 */
int32_t ad469x_sequence_disable_temp(struct ad469x_dev *dev)
{
	int32_t ret;

	ret = ad469x_spi_write_mask(dev,
				    AD469x_REG_TEMP_CTRL,
				    AD469x_REG_TEMP_CTRL_TEMP_EN_MASK,
				    AD469x_REG_TEMP_CTRL_TEMP_EN(0));
	if (ret != SUCCESS)
		return ret;

	dev->temp_enabled = false;

	return ret;
}

/**
 * @brief Enter conversion mode.
 *        To exit conversion mode send a 5 bit conversion mode command
 *        AD469x_CMD_REG_CONFIG_MODE
 * @param [in] dev - ad469x_dev device handler.
 * @return \ref SUCCESS in case of success, \ref FAILURE otherwise.
 */
int32_t ad469x_enter_conversion_mode(struct ad469x_dev *dev)
{
	return ad469x_spi_write_mask(dev,
				     AD469x_REG_SETUP,
				     AD469x_SETUP_IF_MODE_MASK,
				     AD469x_SETUP_IF_MODE_CONV);
}

/**
 * @brief Exit conversion mode.
 *        Enter register mode to read/write registers
 * @param [in] dev - ad469x_dev device handler.
 * @return \ref SUCCESS in case of success, \ref FAILURE otherwise.
 */
int32_t ad469x_exit_conversion_mode(struct ad469x_dev *dev)
{
	int32_t ret;
	uint8_t i;

	for(i = 0; i <= 4; i++)
		ret = ad469x_spi_reg_write(dev, (AD469x_CMD_REG_CONFIG_MODE << 8), 0x00);

	return ret;
}

/**
 * @brief Advanced sequencer, get util data bits in a sample
 * @param [in] dev - ad469x_dev device handler.
 * @param [in] cur_sample - Current sample number
 * @param [in] sample - Sample data
 * @return \ref SUCCESS in case of success, \ref FAILURE otherwise.
 */
static int32_t ad469x_adv_seq_osr_get_util_data(struct ad469x_dev *dev,
		uint16_t cur_sample, uint32_t *sample)
{
	uint8_t cur_slot;

	cur_slot = cur_sample % (dev->num_slots + dev->temp_enabled);

	/* Temperature channel sample */
	if (dev->temp_enabled && cur_slot == dev->num_slots)
		return SUCCESS;

	return SUCCESS;
}

/**
 * @brief Read from device when converter has the channel sequencer activated.
 *        Enter register mode to read/write registers
 * @param [in] dev - ad469x_dev device handler.
 * @param [out] buf - data buffer.
 * @param [in] samples - Number of samples per channel. For example, if  with
 * ad469x_std_sequence_ch 2 channel where activated, buf will be filled with
 * 10 samples for each of them. If temp is enable, the there will be an other 10
 * samples for temperature
 * @return \ref SUCCESS in case of success, \ref FAILURE otherwise.
 */
int32_t ad469x_seq_read_data(struct ad469x_dev *dev,
			     uint32_t *buf,
			     uint16_t samples)
{
	int32_t ret;
	uint16_t i;
	uint32_t total_samples;

	total_samples = samples * (dev->num_slots + dev->temp_enabled);
	ret = ad469x_read_data(dev, 0, buf, total_samples);
	if (ret != SUCCESS)
		return ret;

	if (dev->ch_sequence != AD469x_advanced_seq)
		return SUCCESS;

	for (i = 0; i < total_samples; i++) {
		ret = ad469x_adv_seq_osr_get_util_data(dev, i, &buf[i]);
		if (ret != SUCCESS)
			return ret;
	}

	return SUCCESS;
}

/**
 * @brief Helper function to count number of ones.
 * @param [in] data - Any 16-bit unsigned integer data.
 * @return count - Count of the number of ones in the data.
 */
uint8_t no_of_ones(uint16_t data)
{
	uint16_t i = 0;
	uint8_t count, temp;

	for (i = 0; i < 16 ;i++) {
		temp = (data >> i) & 0x01;
		if (temp == 0x01)
			count++;
	}
	return count;
}

/**
 * @brief Read from device.
 *        Enter register mode to read/write registers
 * @param [in] dev - ad469x_dev device handler.
 * @param [in] channel - ad469x selected channel.
 * @param [out] buf - data buffer.
 * @param [in] samples - sample number.
 * @return \ref SUCCESS in case of success, \ref FAILURE otherwise.
 */
int32_t ad469x_read_data(struct ad469x_dev *dev,
			 uint8_t channel,
			 uint32_t *buf,
			 uint16_t samples)
{
	int32_t ret;
	uint32_t commands_data[1];
	uint16_t val1, val2, data;
	struct spi_engine_offload_message msg;
	uint32_t spi_eng_msg_cmds[3] = {
		CS_LOW,
		WRITE_READ(1),
		CS_HIGH
	};
	if (channel < AD469x_CHANNEL_NO)
		commands_data[0] = AD469x_CMD_CONFIG_CH_SEL(channel) << 8;
	else if (channel == AD469x_CHANNEL_TEMP)
		commands_data[0] = AD469x_CMD_SEL_TEMP_SNSOR_CH << 8;
	else
		return FAILURE;

	if (dev->data_mode)
	{
		ad469x_exit_conversion_mode(dev);
		ad469x_spi_write_mask(dev, AD469x_REG_SEQ_CTRL,
				AD469x_SEQ_CTRL_STD_SEQ_EN_MASK, AD469x_SEQ_CTRL_STD_SEQ_EN(1));
		ad469x_spi_reg_read(dev, AD469x_REG_STD_SEQ_CONFIG, &val1);
		ad469x_spi_reg_read(dev, (AD469x_REG_STD_SEQ_CONFIG + 1), &val2);
		data = (val1 << 8) | val2;
		axi_io_write(dev->spi_engine_baseaddr, UP_DATA_SEQ, data);
		axi_io_write(dev->spi_engine_baseaddr, UP_DATA_COUNT, no_of_ones(data));
		ad469x_enter_conversion_mode(dev);
	}

	axi_io_write(dev->spi_engine_baseaddr, CNV_CONFIG_REG, CNV_ENABLE(1) | UP_RESETN(1) | (dev->data_mode << 2));

	ret = spi_engine_offload_init(dev->spi_desc, dev->offload_init_param);
	if (ret != SUCCESS)
		return ret;

	msg.commands = spi_eng_msg_cmds;
	msg.no_commands = ARRAY_SIZE(spi_eng_msg_cmds);
	msg.rx_addr = (uint32_t)buf;
	msg.commands_data = commands_data;

	ret = spi_engine_offload_transfer(dev->spi_desc, msg, samples);
	if (ret != SUCCESS)
		return ret;

	if (dev->dcache_invalidate_range)
		dev->dcache_invalidate_range((void *) msg.rx_addr, samples * 32);

	return ret;
}

/**
 * Initialize the device.
 * @param [out] device - The device structure.
 * @param [in] init_param - The structure that contains the device initial
 * 		parameters.
 * @return \ref SUCCESS in case of success, \ref FAILURE otherwise.
 */
int32_t ad469x_init(struct ad469x_dev **device,
		    struct ad469x_init_param *init_param)
{
	struct ad469x_dev *dev;
	int32_t ret;
	uint16_t data = 0;

	dev = (struct ad469x_dev *)malloc(sizeof(*dev));
	if (!dev)
		return FAILURE;

	ret = spi_init(&dev->spi_desc, init_param->spi_init);
	if (ret != SUCCESS)
		goto error_dev;

	dev->offload_init_param = init_param->offload_init_param;

	dev->reg_data_width = init_param->reg_data_width;
	dev->capture_data_width = init_param->capture_data_width;
	dev->data_mode = init_param->data_mode;
	dev->dev_id = init_param->dev_id;
	dev->dcache_invalidate_range = init_param->dcache_invalidate_range;
	dev->ch_sequence = AD469x_standard_seq;
	dev->num_slots = 0;
	dev->temp_enabled = false;
	dev->spi_engine_baseaddr = init_param->spi_engine_baseaddr;
	memset(dev->ch_slots, 0, sizeof(dev->ch_slots));

	/* Exit conversion mode */
	ret = ad469x_exit_conversion_mode(dev);
	if (ret != SUCCESS)
		goto error_dev;

	/* Use single instruction mode by default */
	ret = ad469x_spi_reg_write(dev, AD469x_REG_IF_CONFIG_B, 0x80);
	if (ret != SUCCESS)
		return ret;

	ret = ad469x_spi_reg_write(dev, AD469x_REG_SCRATCH_PAD, AD469x_TEST_DATA);
	if (ret != SUCCESS)
		goto error_spi;

	ret = ad469x_spi_reg_read(dev, AD469x_REG_SCRATCH_PAD, &data);
	if (ret != SUCCESS)
		goto error_spi;

	if (data != AD469x_TEST_DATA)
		goto error_spi;

	ret = ad469x_set_reg_access_mode(dev, AD469x_WORD_ACCESS);
	if (ret != SUCCESS)
		goto error_spi;

	ret = ad469x_seq_osr_clear(dev);
	if (ret != SUCCESS)
		goto error_spi;

	ret = ad469x_spi_reg_write(dev, AD469x_REG_GP_MODE, 0x8E);
	if (ret != SUCCESS)
		goto error_spi;

	axi_io_write(dev->spi_engine_baseaddr, CNV_CONFIG_REG, 0x00); // to disable up_resetn and up_cnv pulse
	axi_io_write(dev->spi_engine_baseaddr, CNV_RATE_CONFIG, ad469x_chip_info_tbl[dev->dev_id].cnv_rate); // value to calculate as 1000ns*spi_clk_frequency **
	axi_io_write(dev->spi_engine_baseaddr, CNV_PULSE_WIDTH, ad469x_chip_info_tbl[dev->dev_id].cnv_width); // value to calculate as 250ns*spi_clk_frequency  **

	ret = ad469x_spi_reg_write(dev, AD469x_REG_SETUP, 0x30);
	if (ret != SUCCESS)
		goto error_spi;

	*device = dev;

	return ret;

error_spi:
	spi_remove(dev->spi_desc);
error_dev:
	free(dev);

	return FAILURE;
}

/**
 * @brief Free the memory allocated by ad469x_init().
 * @param [in] dev - Pointer to the device handler.
 * @return \ref SUCCESS in case of success, \ref FAILURE otherwise
 */
int32_t ad469x_remove(struct ad469x_dev *dev)
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
