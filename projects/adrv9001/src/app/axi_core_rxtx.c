/***************************************************************************//**
 *   @file   axi_core_rxtx.c
 *   @brief  adrv9002 axi core defaults project file.
 *   @author Darius Berghe (darius.berghe@analog.com)
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

#include "axi_core_rxtx.h"

struct axi_adc_init rx1_adc_init = {
    "axi-adrv9002-rx-lpc",
    RX1_ADC_BASEADDR,
    ADRV9001_NUM_CHANNELS,
};

struct axi_dac_channel tx1_dac_channels[]={
   { 3000000, //freq is in Hz (i.e. set to 1*1000*1000 for 1 MHz)
     90000,
     50000,
     3000000, //freq is in Hz (i.e. set to 1*1000*1000 for 1 MHz)
     90000,
     50000,
     0,
     0,
     AXI_DAC_DATA_SEL_DMA,},
    { 3000000, //freq is in Hz (i.e. set to 1*1000*1000 for 1 MHz)
      90000,
      50000,
      3000000, //freq is in Hz (i.e. set to 1*1000*1000 for 1 MHz)
      90000,
      50000,
      0,
      0,
      AXI_DAC_DATA_SEL_DMA,},
};


struct axi_dac_init tx1_dac_init = {
    "axi-adrv9002-tx-lpc",
    TX1_DAC_BASEADDR,
    ADRV9001_NUM_CHANNELS,
    tx1_dac_channels,
};


struct axi_adc_init rx2_adc_init = {
    "axi-adrv9002-rx2-lpc",
    RX2_ADC_BASEADDR,
    ADRV9001_NUM_CHANNELS,
};

struct axi_dac_channel tx2_dac_channels[]={
   { 3000000, //freq is in Hz (i.e. set to 1*1000*1000 for 1 MHz)
     90000,
     50000,
     3000000, //freq is in Hz (i.e. set to 1*1000*1000 for 1 MHz)
     90000,
     50000,
     0,
     0,
     AXI_DAC_DATA_SEL_DMA,},
    { 3000000, //freq is in Hz (i.e. set to 1*1000*1000 for 1 MHz)
      90000,
      50000,
      3000000, //freq is in Hz (i.e. set to 1*1000*1000 for 1 MHz)
      90000,
      50000,
      0,
      0,
      AXI_DAC_DATA_SEL_DMA,},
};

struct axi_dac_init tx2_dac_init = {
    "axi-adrv9002-tx2-lpc",
    TX2_DAC_BASEADDR,
    ADRV9001_NUM_CHANNELS,
    tx2_dac_channels,
};

struct axi_dmac_init rx1_dmac_init = {
    "rx_dmac",
    RX1_DMA_BASEADDR,
    DMA_DEV_TO_MEM,
    0
};

struct axi_dmac_init tx1_dmac_init = {
    "tx_dmac",
    TX1_DMA_BASEADDR,
    DMA_MEM_TO_DEV,
    DMA_CYCLIC,
};

struct axi_dmac_init rx2_dmac_init = {
    "rx_dmac",
    RX2_DMA_BASEADDR,
    DMA_DEV_TO_MEM,
    0
};

struct axi_dmac_init tx2_dmac_init = {
    "tx_dmac",
    TX2_DMA_BASEADDR,
    DMA_MEM_TO_DEV,
    DMA_CYCLIC,
};

struct axi_dmac_init *tx_dmac_init_defaults[] = {
    &tx1_dmac_init,
    &tx2_dmac_init,

};

struct axi_dmac_init *rx_dmac_init_defaults[] = {
    &rx1_dmac_init,
    &rx2_dmac_init,

};

struct axi_adc_init *rx_adc_init_defaults[] = {
    &rx1_adc_init,
    &rx2_adc_init,

};

struct axi_dac_init *tx_dac_init_defaults[] = {
    &tx1_dac_init,
    &tx2_dac_init,

};









