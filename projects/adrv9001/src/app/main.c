#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <unistd.h>
#include <string.h>


#include "app_main.h"

/* Default arguments for app */
#define DEFAULT_ARGS { {MIMO, DAC_DATA_SEL_DMA, TRUE, SSI_1_LANE},{0} }


int argConfig_Parser(Cmd_Args *argsp, char * jsonBuffer, unsigned int length)
{ 
#define ADI_ADRV9001_TOKEN_MAX_LENGTH	 32
 
    unsigned int ii = 0;
    int numTokens = 0;
    jsmn_parser parser = { 0 }; 
    jsmntok_t * tokens = NULL; 
    char parsingBuffer[ADI_ADRV9001_TOKEN_MAX_LENGTH]; /* This buffer only needs to hold a stringified number like '123.4567'. */ 
 
 
    /* initialize the JSMN parser and determine the number of JSON tokens */ 
    jsmn_init(&parser); 
    numTokens = jsmn_parse(&parser, jsonBuffer, length, NULL, 0); 
 
    /* The JSON file must be tokenized successfully. */ 
    if (numTokens < 1) 
    { 
        printf("Fatal error while parsing profile file. The JSON may be invalid, or the token buffer may be too small.");
        goto err;
       
    } 
 
    /* allocate space for tokens */ 
    tokens = (jsmntok_t*)calloc(numTokens, sizeof(jsmntok_t)); 
 
    if (NULL == tokens) 
    { 
        printf("Fatal error while reading profile file. Possible memory shortage."); 
        goto err;
       
    } 
 
    /* initialize the JSMN parser and parse the profile file into the tokens array */ 
    jsmn_init(&parser); 
    numTokens = jsmn_parse(&parser, jsonBuffer, length, tokens, numTokens); 
 
    /* The top-level element must be an object. */ 
    if (numTokens < 1 || tokens[0].type != JSMN_OBJECT) 
    { 
        free(tokens); 
        printf("Fatal error while parsing profile file. The JSON may be invalid, or the token buffer may be too small."); 
        goto err;
    } 
 
    /* Loop over all keys of the root object, searching for matching fields. */ 
    for (ii = 1; ii < numTokens; ii++) 
    { 
        ADRV9001_CMDLINE_ARGS(tokens, ii, jsonBuffer, parsingBuffer, (*argsp));
    } 
 
    free(tokens); 
    tokens = NULL; 
 
    return 0; 

err:
    return -1;
} 

/******************************************************************************
 * parseArgs
 ******************************************************************************/
static void parseArgs(Cmd_Args *argsp, char * jsonBuffer, unsigned int length)
{
    
    int ret=0;
    if (jsonBuffer != NULL || length != 0) {
        ret=argConfig_Parser(argsp, jsonBuffer, length);
        if(ret != 0)
            printf("Error while parsing profile file. Taken the Default Arguments\r\n");
    }

}

/******************************************************************************
 * main
 ******************************************************************************/
int main(void)
{
    Cmd_Args args = DEFAULT_ARGS;
    int ret;
    uint32_t size = 0;

    char *buf=(char*)ARGS_ADDR;

    /* Parse the arguments given to the app */
    while(1){
    	if(buf[0] != '{')
    		break;
    	if(buf[size] == '}' && buf[size+1] != ',' ){
    		size++;
    		break;
    	}
    	size++;
    }
//    printf("size = %d\r\n",size);
    if(size == 0)
    	printf("Jason profile for arguments is missing. Taken the Default Arguments\r\n");
    else
    	parseArgs(&args,buf,size);
/***
//    printf("%d , %d, %d, %d, %d, %d, %d , %d, %d, %d\r\n",args.args.src_select,args.args.intl_loopback,args.args.device_mode,args.args.CMOS_lane_sel,
//    args.dds.dds_frequency_0,\
//    args.dds.dds_phase_0, \
//    args.dds.dds_scale_0, \
//    args.dds.dds_frequency_1, \
//    args.dds.dds_phase_1, \
//    args.dds.dds_scale_1 );
***/
    ret = appMain(&args);

    return ret;
}
