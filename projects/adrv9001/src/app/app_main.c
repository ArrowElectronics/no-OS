/***************************************************************************//**
 *   @file   app_main.c
 *   @brief  adrv9002 main project file.
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


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "error.h"
#include "util.h"
#include "spi.h"

//#include "alt_cache.h"

#ifdef IIO_SUPPORT
#include "app_iio.h"
#include "xil_cache.h"
#endif

#include "adrv9002.h"
#include "adi_adrv9001.h"
#include "adi_adrv9001_arm.h"
#include "adi_adrv9001_radio.h"
#include "axi_core_rxtx.h"
#include "alt_cache.h"
#include "axi_io.h"

#include "app_main.h"

#define ADI_TX_REG_CHAN_CTRL_7(c)	(0x0418 + (c) * 0x40)
#define ADI_TX_REG_CTRL_1		0x44
#ifdef IIO_SUPPORT
int get_sampling_frequency(struct axi_adc *dev, uint32_t chan,
               uint64_t *sampling_freq_hz)
{
    if (!dev || !sampling_freq_hz)
        return -EINVAL;

    *sampling_freq_hz =
        adrv9002_init_get()->rx.rxChannelCfg[chan].profile.rxOutputRate_Hz;
    return SUCCESS;
}
#endif
/* Initialize the ADRV9001 Daughter Board */
static int adrv9001_init(struct adrv9002_rf_phy *phy, Cmd_Args * args) {
    int ret;
    struct adi_common_ApiVersion api_version;
    struct adi_adrv9001_ArmVersion arm_version;
    struct adi_adrv9001_SiliconVersion silicon_version;

    ret = adrv9002_setup(phy, adrv9002_init_get(args->args.CMOS_lane_sel));
    if (ret)
        goto err;

    adi_adrv9001_ApiVersion_Get(phy->adrv9001, &api_version);
    adi_adrv9001_arm_Version(phy->adrv9001, &arm_version);
    adi_adrv9001_SiliconVersion_Get(phy->adrv9001, &silicon_version);

    printf("%s Rev %d.%d, Firmware %u.%u.%u.%u API version: %u.%u.%u successfully initialized\n",
           "ADRV9002", silicon_version.major, silicon_version.minor,
           arm_version.majorVer, arm_version.minorVer,
           arm_version.maintVer, arm_version.rcVer, api_version.major,
           api_version.minor, api_version.patch);

err:
    return ret;
}

/* Initialize the ADC/DAC & AXI DMA Controller cores */
static int axi_cores_init(struct adrv9002_rf_phy *phy, Cmd_Args * args){

    int i,ret;
    struct axi_adc	*rx_adc[MODE_MAX];
    struct axi_dac	*tx_dac[MODE_MAX];
    struct axi_dmac	*rx_dmac[MODE_MAX];
    struct axi_dmac	*tx_dmac[MODE_MAX];

    if(args->args.device_mode==MIMO)
        tx_dac_init_defaults[0]->num_channels=ADRV9001_NUM_CHANNELS;
    else{
        for(i=0;i<=args->args.device_mode;i++){
            tx_dac_init_defaults[i]->num_channels=ADRV9001_NUM_SUBCHANNELS;

        }
        tx_dac_init_defaults[1]->channels->dds_frequency_1=args->dds.dds_frequency_1;
        tx_dac_init_defaults[1]->channels->dds_phase_1=args->dds.dds_phase_1;
        tx_dac_init_defaults[1]->channels->dds_scale_1=args->dds.dds_scale_1;
        tx_dac_init_defaults[1]->channels->sel=args->args.src_select;
    }

    tx_dac_init_defaults[0]->channels->dds_frequency_0=args->dds.dds_frequency_0;
    tx_dac_init_defaults[0]->channels->dds_phase_0=args->dds.dds_phase_0;
    tx_dac_init_defaults[0]->channels->dds_scale_0=args->dds.dds_scale_0;
    tx_dac_init_defaults[0]->channels->sel=args->args.src_select;

    tx_dac_init_defaults[0]->channels=NULL;

    for(i=0;i<=args->args.device_mode;i++){

        /* Initialize the ADC/DAC cores */
        ret = axi_adc_init(&rx_adc[i], rx_adc_init_defaults[i]);
        if (ret) {
            printf("axi_adc_init() failed with status %d\n", ret);
            goto error;
        }

        ret = axi_dac_init(&tx_dac[i], tx_dac_init_defaults[i]);
        if (ret) {
            printf("axi_dac_init() failed with status %d\n", ret);
            goto error;
        }

        tx_dac[i]->clock_hz = adrv9002_init_get(args->args.CMOS_lane_sel)->tx.txProfile[i].txInputRate_Hz;

        /* Initialize the AXI DMA Controller cores */
        ret = axi_dmac_init(&tx_dmac[i], tx_dmac_init_defaults[i]);
        if (ret) {
            printf("axi_dmac_init() failed with status %d\n", ret);
            goto error;
        }

        ret = axi_dmac_init(&rx_dmac[i], rx_dmac_init_defaults[i]);
        if (ret) {
            printf("axi_dmac_init() failed with status %d\n", ret);
            goto error;
        }
    }

    if(args->args.device_mode == MIMO)
         phy->tx1_dac->clock_hz=tx_dac[0]->clock_hz;
    else{
        phy->tx2_dac->clock_hz=tx_dac[0]->clock_hz;
        phy->tx2_dac->clock_hz=tx_dac[1]->clock_hz;
    }

    phy->rx1_adc=rx_adc[0];
    phy->rx2_adc=rx_adc[1];
    phy->tx1_dac=tx_dac[0];
    phy->tx2_dac=tx_dac[1];
    phy->rx1_dmac=rx_dmac[0];
    phy->rx2_dmac=rx_dmac[1];
    phy->tx1_dmac=tx_dmac[0];
    phy->tx2_dmac=tx_dmac[1];

error:
    return ret;

}

/* clean up the resources */
static void clean_up(struct adrv9002_rf_phy *phy, mode_sel device_mode){

    adi_adrv9001_HwClose(phy->adrv9001);
    axi_adc_remove(phy->rx1_adc);
    axi_dac_remove(phy->tx1_dac);
    axi_adc_remove(phy->rx2_adc);
    axi_dac_remove(phy->tx2_dac);
    axi_dmac_remove(phy->rx1_dmac);
    axi_dmac_remove(phy->tx1_dmac);
    axi_dmac_remove(phy->rx2_dmac);
    axi_dmac_remove(phy->tx2_dmac);

    if(phy != NULL){
        free(phy);
        phy=NULL;
    }

}

int adrv9002_axi_tx_data_sel(struct adrv9002_rf_phy *phy, const data_sel data)
{
    if(data != ADC_DATA_SEL_TP_RAMP){
        int c;
        /* I and Q channels */
        for (c = 0; c < ADRV9001_NUM_CHANNELS; c++)
            axi_io_write(phy->tx1_dac->base, ADI_TX_REG_CHAN_CTRL_7(c), data);

        axi_io_write(phy->tx1_dac->base, ADI_TX_REG_CTRL_1, 1);

    }

    return 0;
}


static void load_dac_buff(struct adrv9002_rf_phy *phy, Cmd_Args *args){

    struct axi_dac	*tx_dac[MODE_MAX];
    struct axi_dmac	*tx_dmac[MODE_MAX];
    uint32_t dmac_addr[MODE_MAX],i;
    tx_dac[0]=phy->tx1_dac;
    tx_dac[1]=phy->tx2_dac;
    tx_dmac[0]=phy->tx1_dmac;
    tx_dmac[1]=phy->tx2_dmac;
    dmac_addr[0]=DAC1_DDR_BASEADDR;
    dmac_addr[1]=DAC2_DDR_BASEADDR;

    for(i=0;i<=args->args.device_mode;i++){

        axi_dac_load_custom_data(tx_dac[i], sine_lut_iq, ARRAY_SIZE(sine_lut_iq), dmac_addr[i]);
        axi_dmac_transfer(tx_dmac[i], dmac_addr[i], sizeof(sine_lut_iq));

    }

}

static void load_adc_buff(struct adrv9002_rf_phy *phy, Cmd_Args *args){

    struct axi_dmac	*rx_dmac[MODE_MAX];
    uint32_t dmac_addr[MODE_MAX],size[MODE_MAX],i;
    rx_dmac[0]=phy->rx1_dmac;
    rx_dmac[1]=phy->rx2_dmac;
    dmac_addr[0]=ADC1_DDR_BASEADDR;
    dmac_addr[1]=ADC2_DDR_BASEADDR;

    size[0]=ADRV9001_NUM_CHANNELS; /* rx1 i/q, rx2 i/q*/
    size[1]=ADRV9001_NUM_SUBCHANNELS; /* rx1 i/q */

    for(i=0;i<=args->args.device_mode;i++){

        /* Transfer 16384 samples from ADC to MEM */
        axi_dmac_transfer(rx_dmac[i],
                          dmac_addr[i],
                          16384 *  /* nr of samples */
                          size[i] *
                          2 /* bytes per sample */);
        alt_cache_system_invalidate((void*)(dmac_addr[i]),
                      16384 * /* nr of samples */
                      size[i] *
                      2 /* bytes per sample */);

    }
}


static void read_adc_I_Q_samples(void){

    uint32_t i;
    int32_t *data_ptr=NULL;
    printf(" CH0 CH1 \r\n");

    for (i = 0; i < 16384; i++){
        if(i%2 == 0){
            printf("\r\n");
        }

        data_ptr = (int32_t *)(ADC1_DDR_BASEADDR + (i * sizeof(uint32_t)));
        printf("0x%x ", ((*data_ptr) >> 16 & 0x0000ffff));
        printf("0x%x ", ((*data_ptr) & 0x0000ffff));
    }


}



/******************************************************************************
 * appMain
 ******************************************************************************/
int appMain(Cmd_Args * args)
{
    int ret;

    struct adrv9002_rf_phy *phy = (struct adrv9002_rf_phy *)malloc(sizeof(struct adrv9002_rf_phy));
    if(phy == NULL){
        free(phy);
        phy=NULL;
        printf("unable to allocate memory for adrv9002 PHY\r\n");
        return -1;
    }

    memset(phy, 0, sizeof(struct adrv9002_rf_phy));

    if(args->args.device_mode == MIMO) {
        phy->rx2tx2 = true;
    }

    ret=adrv9001_init(phy,args);
    if (ret)
        goto error;

    ret=axi_cores_init(phy,args);
    if (ret)
        goto error;

    /* Post AXI DAC/ADC setup, digital interface tuning */
    ret = adrv9002_post_setup(phy);
    if (ret) {
        printf("adrv9002_post_setup() failed with status %d\n", ret);
        goto error;
    }

    /* TODO: Remove this when it gets fixed in the API. */
    adi_adrv9001_Radio_Channel_ToState(phy->adrv9001, ADI_RX,
                       ADI_CHANNEL_1, ADI_ADRV9001_CHANNEL_PRIMED);
    adi_adrv9001_Radio_Channel_ToState(phy->adrv9001, ADI_RX,
                       ADI_CHANNEL_1, ADI_ADRV9001_CHANNEL_RF_ENABLED);
    adi_adrv9001_Radio_Channel_ToState(phy->adrv9001, ADI_RX,
                       ADI_CHANNEL_2, ADI_ADRV9001_CHANNEL_PRIMED);
    adi_adrv9001_Radio_Channel_ToState(phy->adrv9001, ADI_RX,
                       ADI_CHANNEL_2, ADI_ADRV9001_CHANNEL_RF_ENABLED);

    if(args->args.intl_loopback) {
        printf("TX-RX loopback is enabled\r\n");
        ret=adrv9002_Ssi_Loopback_Set(phy,args->args.intl_loopback);
        if (ret)
            goto error;
    }

    switch (args->args.src_select) {
    case DAC_DATA_SEL_DMA:
        printf("Running in Input DMA mode\r\n");
        adrv9002_axi_tx_data_sel(phy, args->args.src_select);
        load_dac_buff(phy,args);
        break;
    case DAC_DATA_SEL_TP_RAMP:
        printf("TX TestPattern Enabled\r\n");
        adrv9002_axi_tx_data_sel(phy, args->args.src_select);
        break;
    case ADC_DATA_SEL_TP_RAMP:
        printf("RX TestPattern Enabled \n");
        for (int c = 0; c < ADRV9002_CHANN_MAX; c++) {
            /* start test */
            ret = adrv9002_intf_test_cfg(phy, phy->rx_channels[c].channel.idx,
                                         phy->rx_channels[c].channel.port == ADI_TX, false);
            if (ret)
                return ret;
        }

        break;
    default:
        printf("you are in default\n");
        break;
    }

    load_adc_buff(phy, args);
    read_adc_I_Q_samples();

    printf("!!!Bye!!!\r\n");

#ifdef IIO_SUPPORT
    printf("The board accepts libiio clients connections through the serial backend.\n");

    struct iio_axi_adc_init_param iio_axi_adc1_init_par = {
        .rx_adc = phy.rx1_adc,
        .rx_dmac = phy.rx1_dmac,
        .dcache_invalidate_range = (void (*)(uint32_t, uint32_t))Xil_DCacheInvalidateRange,
        .get_sampling_frequency = get_sampling_frequency,
    };

    struct iio_axi_dac_init_param iio_axi_dac1_init_par = {
        .tx_dac = phy.tx1_dac,
        .tx_dmac = phy.tx1_dmac,
        .dcache_flush_range = (void (*)(uint32_t, uint32_t))Xil_DCacheFlushRange,
    };
#ifndef ADRV9002_RX2TX2
    struct iio_axi_adc_init_param iio_axi_adc2_init_par = {
        .rx_adc = phy.rx2_adc,
        .rx_dmac = phy.rx2_dmac,
        .dcache_invalidate_range = (void (*)(uint32_t, uint32_t))Xil_DCacheInvalidateRange,
        .get_sampling_frequency = get_sampling_frequency,
    };

    struct iio_axi_dac_init_param iio_axi_dac2_init_par = {
        .tx_dac = phy.tx2_dac,
        .tx_dmac = phy.tx2_dmac,
        .dcache_flush_range = (void (*)(uint32_t, uint32_t))Xil_DCacheFlushRange,
    };
    ret = iio_server_init(&iio_axi_adc1_init_par,
                  &iio_axi_adc2_init_par,
                  &iio_axi_dac1_init_par,
                  &iio_axi_dac2_init_par);
#else
    ret = iio_server_init(&iio_axi_adc1_init_par,
                  NULL,
                  &iio_axi_dac1_init_par,
                  NULL);
#endif
#endif

error:
    clean_up(phy,args->args.device_mode);
    return ret;
}
