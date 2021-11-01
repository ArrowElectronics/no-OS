/***************************************************************************//**
 *   @file   app_main.h
 *   @brief  adrv9002 main project header file.
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

#ifndef APP_MAIN_H
#define APP_MAIN_H
#include <stdbool.h>
#include <stdint.h>
#include "jsmn.h"

#define ARGS_ADDR 0x10000

#define ADI_ARRAY_LEN(arr) (sizeof(arr) / sizeof(arr[0]))

#define COPY_TOKEN_TO_BUFFER(tokenArray, tokenIndex, jsonBuffer, copyBuffer) \
snprintf(copyBuffer, \
    ADI_ARRAY_LEN(copyBuffer), \
    "%.*s", tokenArray[tokenIndex + 1].end - tokenArray[tokenIndex + 1].start, \
    jsonBuffer + tokenArray[tokenIndex + 1].start);

#define ADI_STORE_INT(tokenArray, tokenIndex, jsonBuffer, intParsingBuffer, intDestination) \
COPY_TOKEN_TO_BUFFER(tokenArray, tokenIndex, jsonBuffer, intParsingBuffer) \
intDestination = atoi(intParsingBuffer);

#define ADI_STORE_BOOL(tokenArray, tokenIndex, jsonBuffer, boolParsingBuffer, boolDestination) \
COPY_TOKEN_TO_BUFFER(tokenArray, tokenIndex, jsonBuffer, boolParsingBuffer) \
boolDestination = ('0' != boolParsingBuffer[0]) && ('f' != boolParsingBuffer[0]) && ('F' != boolParsingBuffer[0]);

#define ADI_IF_JSON_EQ(jsonBuffer, jsmnTok, token) \
if (jsmnTok.type == JSMN_STRING && (int)strlen(token) == jsmnTok.end - jsmnTok.start && \
    strncmp(jsonBuffer + jsmnTok.start, token, jsmnTok.end - jsmnTok.start) == 0)

#define ADI_PROCESS_X(X, tokenArray, tokenIndex, jsonBuffer, parsingBuffer, varDestination, varName) \
ADI_IF_JSON_EQ(jsonBuffer, tokenArray[tokenIndex], varName) { \
    X(tokenArray, tokenIndex, jsonBuffer, parsingBuffer, varDestination); \
    tokenIndex++; \
    continue; \
}

#define ADI_PROCESS_INT(tokenArray, tokenIndex, jsonBuffer, intParsingBuffer, intDestination, intName) \
ADI_PROCESS_X(ADI_STORE_INT, tokenArray, tokenIndex, jsonBuffer, intParsingBuffer, intDestination, intName)

#define ADI_PROCESS_BOOL(tokenArray, tokenIndex, jsonBuffer, boolParsingBuffer, boolDestination, boolName) \
ADI_PROCESS_X(ADI_STORE_BOOL, tokenArray, tokenIndex, jsonBuffer, boolParsingBuffer, boolDestination, boolName)

#define ADI_PROCESS_STRUCT_X(X, tokenArray, tokenIndex, jsonBuffer, parsingBuffer, structName) \
ADI_IF_JSON_EQ(jsonBuffer, tokenArray[tokenIndex], structName) { \
    int16_t size = tokenArray[++tokenIndex].size; \
    int16_t end = tokenArray[tokenIndex].end; \
    { \
        int16_t j = 0; \
        for (j = 0; j < size; j++) { \
            tokenIndex++; \
            \
            X; \
        } \
    } \
    while (tokenArray[tokenIndex + 1].start <= end) { \
        tokenIndex++; \
    } \
    continue; \
}

/* ---- ADRV9001_ARGS ---- */
#define ADRV9001_ARGS(tokenArray, tokenIndex, jsonBuffer, parsingBuffer, cmd_line_args_tInstance) \
ADI_PROCESS_INT      (tokenArray, tokenIndex, jsonBuffer, parsingBuffer, cmd_line_args_tInstance.device_mode, "device_mode"); \
ADI_PROCESS_INT      (tokenArray, tokenIndex, jsonBuffer, parsingBuffer, cmd_line_args_tInstance.src_select, "src_select"); \
ADI_PROCESS_BOOL     (tokenArray, tokenIndex, jsonBuffer, parsingBuffer, cmd_line_args_tInstance.intl_loopback, "intl_loopback"); \
ADI_PROCESS_INT      (tokenArray, tokenIndex, jsonBuffer, parsingBuffer, cmd_line_args_tInstance.CMOS_lane_sel, "CMOS_lane_sel"); \


/* ---- DDS_CONFIF ---- */
#define DDS_CONFIG(tokenArray, tokenIndex, jsonBuffer, parsingBuffer, cmd_line_args_tInstance) \
ADI_PROCESS_INT      (tokenArray, tokenIndex, jsonBuffer, parsingBuffer, cmd_line_args_tInstance.dds_frequency_0, "dds_frequency_0"); \
ADI_PROCESS_INT      (tokenArray, tokenIndex, jsonBuffer, parsingBuffer, cmd_line_args_tInstance.dds_phase_0, "dds_phase_0"); \
ADI_PROCESS_INT      (tokenArray, tokenIndex, jsonBuffer, parsingBuffer, cmd_line_args_tInstance.dds_scale_0, "dds_scale_0"); \
ADI_PROCESS_INT      (tokenArray, tokenIndex, jsonBuffer, parsingBuffer, cmd_line_args_tInstance.dds_frequency_1, "dds_frequency_1"); \
ADI_PROCESS_INT      (tokenArray, tokenIndex, jsonBuffer, parsingBuffer, cmd_line_args_tInstance.dds_phase_1, "dds_phase_1"); \
ADI_PROCESS_INT      (tokenArray, tokenIndex, jsonBuffer, parsingBuffer, cmd_line_args_tInstance.dds_scale_1, "dds_scale_1"); \
ADI_PROCESS_INT      (tokenArray, tokenIndex, jsonBuffer, parsingBuffer, cmd_line_args_tInstance.dds_dual_tone, "dds_dual_tone"); \


#define ADI_PROCESS_STRUCT_ADRV9001_ARGS(tokenArray, tokenIndex, jsonBuffer, parsingBuffer, cmd_line_args_tInstance, arg_struct_tName) \
ADI_PROCESS_STRUCT_X(ADRV9001_ARGS(tokenArray, tokenIndex, jsonBuffer, parsingBuffer, cmd_line_args_tInstance), tokenArray, tokenIndex, jsonBuffer, parsingBuffer, arg_struct_tName);

#define ADI_PROCESS_STRUCT_DDS_CONFIG(tokenArray, tokenIndex, jsonBuffer, parsingBuffer, cmd_line_args_tInstance, arg_struct_tName) \
ADI_PROCESS_STRUCT_X(DDS_CONFIG(tokenArray, tokenIndex, jsonBuffer, parsingBuffer, cmd_line_args_tInstance), tokenArray, tokenIndex, jsonBuffer, parsingBuffer, arg_struct_tName);


#define ADRV9001_CMDLINE_ARGS(tokenArray, tokenIndex, jsonBuffer, parsingBuffer, struct_args_tInstance) \
ADI_PROCESS_STRUCT_ADRV9001_ARGS(tokenArray, tokenIndex, jsonBuffer, parsingBuffer, struct_args_tInstance.args, "Args"); \
ADI_PROCESS_STRUCT_DDS_CONFIG(tokenArray, tokenIndex, jsonBuffer, parsingBuffer, struct_args_tInstance.dds, "dds_config"); \


typedef enum bool_flag {
    TRUE=1,
    FALSE=0

}bool_flag;

typedef enum mode_sel {
    MIMO=0,
    INDEPENDENT,
    MODE_MAX
}mode_sel;

typedef enum lane {
	SSI_1_LANE=1,
	SSI_4_LANE=4
}lane;

typedef enum data_sel {
    DAC_DATA_SEL_DDS=0,
    DAC_DATA_SEL_DMA=2,
    DAC_DATA_SEL_ZERO=3,
    DAC_DATA_SEL_PN7=5,
    DAC_DATA_SEL_PN15=6,
    DAC_DATA_SEL_TP_RAMP=10,
    ADC_DATA_SEL_TP_RAMP=20,
}data_sel;

enum test_pattern {
    TX_RAMP_NIBBLE,
    RX_RAMP_NIBBLE
};


/**
 * \brief Data structure to hold ADRV9001 device config arguments initialization settings
 *
 */

/* Arguments for app_main */
typedef struct Args {
    mode_sel device_mode;
    data_sel src_select;
    bool intl_loopback;
    uint8_t CMOS_lane_sel;
} Args;


typedef struct dds_config {
    uint32_t dds_frequency_0;       // in hz (1000*1000 for MHz)
    uint32_t dds_phase_0;           // in milli(?) angles (90*1000 for 90 degrees = pi/2)
    int32_t dds_scale_0;            // in micro units (1.0*1000*1000 is 1.0)
    uint32_t dds_frequency_1;       // in hz (1000*1000 for MHz)
    uint32_t dds_phase_1;           // in milli(?) angles (90*1000 for 90 degrees = pi/2)
    int32_t dds_scale_1;            // in micro units (1.0*1000*1000 is 1.0)
    uint32_t dds_dual_tone;         // if using single tone for this channel, set to 0x0
}dds_config;

typedef struct Cmd_Args {
    Args args;
    dds_config dds;
}Cmd_Args;

#if defined (__cplusplus)
extern "C" {
#endif

extern int appMain(Cmd_Args * args);

#if defined (__cplusplus)
}
#endif

#endif // APP_MAIN_H
