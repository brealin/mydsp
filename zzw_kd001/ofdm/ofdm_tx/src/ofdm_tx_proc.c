/***********************************************************************************************************************
* Copyright (C) 2020 Shanghai Kindroid Network Tech Ltd. All rights reserved
*
* License:
*
* Description:
*
* Author:
*
* Revision History:
* 1.0 2020-05-14 created by changyi.zhang
***********************************************************************************************************************/
#undef  THIS_FILE_NAME_ID
#define THIS_FILE_NAME_ID OAL_SCFDE_TX_FILE_ID

/***********************************************************************************************************************
* INCLUDE FILES
***********************************************************************************************************************/
#include "ks_oal.h"
#include "ks_turbo_encoder.h"
#include "ofdm_tx_proc.h"
#include "l1cc.h"
#include "ks_fft.h"
#include "zzw_common.h"
#include "ks_kd001_main.h"

/***********************************************************************************************************************
* MACRO DEFINITION
***********************************************************************************************************************/

/***********************************************************************************************************************
* GLOBAL VARIABLES DEFINITION
***********************************************************************************************************************/
OAL_DTCM_DATA_SECTION UINT32 u32_scram_table[192] = 
{
	#include "./scrambler0.dat"	
};

#ifndef VECTOR_TX_TEST

OAL_SRAM_DATA_SECTION OAL_ALIGNED(64) complex_s16_t agc_cca[2][AGC_CCA_LEN]=
{	
	{
		#include "./gap_seq.dat"
	},
	{
		#include "./gap_seq.dat" 
	}    
};
#ifdef MIMO2 
OAL_SRAM_DATA_SECTION OAL_ALIGNED(64) complex_s16_t gap_data[2][GAP0_LEN]={0};

OAL_SRAM_DATA_SECTION OAL_ALIGNED(64) complex_s16_t pilot_data[2][SLOT_LEN] = 
{
	{
		#include "./pilot_tx_antenna1_new_cp0.dat"
	},		
	{
		#include "./pilot_tx_antenna2_new_cp0.dat"
	}
};
#else
OAL_SRAM_DATA_SECTION OAL_ALIGNED(64) complex_s16_t gap_data[2][GAP1_LEN]={0};

OAL_SRAM_DATA_SECTION OAL_ALIGNED(64) complex_s16_t pilot_data[2][SLOT_LEN] = 
{
	{
		#include "./pilot_tx_antenna1_new_cp0.dat"
	},		
	{
		#include "./pilot_tx_antenna2_new_cp0.dat"
	}
};
#endif
OAL_SRAM_DATA_SECTION OAL_ALIGNED(64) complex_s16_t ifft_output_data_ant1[TOTAL_MOD_SYM_NUM * ALGO_DATA_MOD_CNT_MAX][2048]={0};
OAL_SRAM_DATA_EXT_SECTION OAL_ALIGNED(64) complex_s16_t ifft_output_data_ant2[TOTAL_MOD_SYM_NUM * ALGO_DATA_MOD_CNT_MAX][2048]={0};

#else

OAL_SRAM_DATA_SECTION OAL_ALIGNED(64) complex_s16_t framing_data[2][30720]=
{
	{
		#include "./triangle_sin_square_wav4m_30720_with_frame_struct.dat"
	},		
	{
		#include "./triangle_sin_square_wav4m_30720_with_frame_struct.dat"
	}	
};
OAL_SRAM_DATA_EXT_SECTION OAL_ALIGNED(64) UINT32 framing_data_test;

#endif

OAL_DTCM_DATA_SECTION OAL_ALIGNED(64) UINT8 ctrl_encode_output[SC_NUM >> 3] = {0};//1200
OAL_DTCM_DATA_SECTION OAL_ALIGNED(64) UINT8 data_encode_output[DATA_SYM_NUM][SC_NUM] = {0};	// data enc & mod should not exceed 1ms frame structure, no need pingpang protection.

OAL_DTCM_DATA_SECTION UINT32 frame_dmac_info[2][TOTAL_FRAME_SEG_NUM * 3]={0};
OAL_DTCM_DATA_SECTION UINT32 encode_cnt = 0;
OAL_DTCM_DATA_SECTION UINT32 dmac_cnt = 0; 
OAL_DTCM_DATA_SECTION UINT32 fft_cnt = 0; 
OAL_DTCM_DATA_SECTION UINT32 dma_send_cnt = 0;
OAL_DTCM_DATA_SECTION UINT32 turbo_input_addr[2] = {TURBO_ENCODER_INPUT_PING_ADDR, TURBO_ENCODER_INPUT_PONG_ADDR};

OAL_DTCM_DATA_SECTION OAL_ALIGNED(64) complex_s16_t mod_output_data[TOTAL_MOD_SYM_NUM * 2][SYMBOL_LEN]={0};// plcp + 9*data. 

OAL_DTCM_DATA_SECTION OAL_ALIGNED(16)  UINT8 g_u8_tx_buffer[3][TX_BUFFER_SIZE + TX_BUFF_HEAD_SIZE]={0};

OAL_DRAM_DATA_SECTION FFT_CFG  algo_ul_dft_fft0_cfg={0};
OAL_DRAM_DATA_SECTION OAL_ALIGNED(64) DMAC_LLI_ITEM dft0_link_in_item[TOTAL_MOD_SYM_NUM]={0};
OAL_DRAM_DATA_SECTION OAL_ALIGNED(64) DMAC_LLI_ITEM dft0_link_out_item[TOTAL_MOD_SYM_NUM]={0};

OAL_DRAM_DATA_SECTION FFT_CFG  algo_ul_ifft0_data_cfg={0};
OAL_DRAM_DATA_SECTION OAL_ALIGNED(64) DMAC_LLI_ITEM ifft0_link_in_item[TOTAL_MOD_SYM_NUM]={0};
OAL_DRAM_DATA_SECTION OAL_ALIGNED(64) DMAC_LLI_ITEM ifft0_link_out_item[TOTAL_MOD_SYM_NUM]={0};

OAL_DRAM_DATA_SECTION FFT_CFG  algo_ul_ifft1_data_cfg={0};
OAL_DRAM_DATA_SECTION OAL_ALIGNED(64) DMAC_LLI_ITEM ifft1_link_in_item[TOTAL_MOD_SYM_NUM]={0};
OAL_DRAM_DATA_SECTION OAL_ALIGNED(64) DMAC_LLI_ITEM ifft1_link_out_item[TOTAL_MOD_SYM_NUM]={0};

OAL_DTCM_DATA_SECTION algo_tx_brp_para ctrl_info_cfg = {0};
OAL_DTCM_DATA_SECTION algo_ul_encoder_info_t algo_ul_encoder_info = {0}; 
OAL_DTCM_DATA_SECTION OAL_ALIGNED(4) l1cc2algo_ctrl_info_t g_st_ctrl_info[TX_CTRL_INFO_CNT_MAX] = {0};

OAL_DTCM_DATA_SECTION UINT32 g_u32_turbo_status = 0;  //0: plcp; 1: data
OAL_DTCM_DATA_SECTION VUINT32 g_u32_turbo_left = 0;  //0: plcp; 1: data
OAL_DTCM_DATA_SECTION TURBO_ENCODER_PARA g_st_algo_turbo_encoder_para[ZZW_MAX_MCS_NUM + 1] = {0};	// 0 for plcp , 1 ~~ ZZW_MAX_MCS_NUM for mcs0 ~~ mcs13
OAL_DTCM_DATA_SECTION UINT8 g_u8_turbo_encoder_para_ready[ZZW_MAX_MCS_NUM + 1] = {0};  // 0 for plcp , 1 ~~ ZZW_MAX_MCS_NUM for mcs0 ~~ mcs13

OAL_DTCM_DATA_SECTION BOOLEAN g_bool_is_ant1_data_proc_done = OAL_TRUE;	// used as protection

OAL_DTCM_DATA_SECTION UINT8 g_u8_data_ifft_cnt = 0;

OAL_DTCM_DATA_SECTION UINT8 g_u8_tx_ctrl_info_cnt_in = 0;
OAL_DTCM_DATA_SECTION UINT8 g_u8_tx_ctrl_info_cnt_out = 0;

OAL_DTCM_DATA_SECTION UINT8 g_u8_algo_ul_ctrl_mod_cnt_in = 0;
OAL_DTCM_DATA_SECTION UINT8 g_u8_algo_ul_ctrl_mod_cnt_out = 0;

OAL_DTCM_DATA_SECTION UINT8 g_u8_algo_ul_data_mod_cnt_in = 0;
OAL_DTCM_DATA_SECTION UINT8 g_u8_algo_ul_data_mod_cnt_out = 0;

OAL_DTCM_DATA_SECTION UINT8 g_u8_algo_ul_data_mod_type = 0;

extern SINT32 g_s32_frame_no;
extern UINT32 g_u32_tx_encode_cancel;
extern VOID _scramble_asm(VOID_PTR src_addr,VOID_PTR dst_addr,VOID_PTR scram_table, UINT32 len);
extern VOID _modulation_asm(VOID_PTR src_addr,VOID_PTR dst_addr,VOID_PTR mcs_table, UINT32 len);

OAL_DTCM_DATA_SECTION
UINT32 algo_k_table[AGLO_K_TABLE_LEN] = {
      40,   48,   56,   64,   72,   80,   88,   96,  104,  112,  120,  128,  136,  144,  152,
     160,  168,  176,  184,  192,  200,  208,  216,  224,  232,  240,  248,  256,  264,  272,
     280,  288,  296,  304,  312,  320,  328,  336,  344,  352,  360,  368,  376,  384,  392,
     400,  408,  416,  424,  432,  440,  448,  456,  464,  472,  480,  488,  496,  504,  512,
     528,  544,  560,  576,  592,  608,  624,  640,  656,  672,  688,  704,  720,  736,  752,
     768,  784,  800,  816,  832,  848,  864,  880,  896,  912,  928,  944,  960,  976,  992, 
    1008, 1024, 1056, 1088, 1120, 1152, 1184, 1216, 1248, 1280, 1312, 1344, 1376, 1408, 1440,
    1472, 1504, 1536, 1568, 1600, 1632, 1664, 1696, 1728, 1760, 1792, 1824, 1856, 1888, 1920,
    1952, 1984, 2016, 2048, 2112, 2176, 2240, 2304, 2368, 2432, 2496, 2560, 2624, 2688, 2752,
    2816, 2880, 2944, 3008, 3072, 3136, 3200, 3264, 3328, 3392, 3456, 3520, 3584, 3648, 3712,
    3776, 3840, 3904, 3968, 4032, 4096, 4160, 4224, 4288, 4352, 4416, 4480, 4544, 4608, 4672,
    4736, 4800, 4864, 4928, 4992, 5056, 5120, 5184, 5248, 5312, 5376, 5440, 5504, 5568, 5632,
    5696, 5760, 5824, 5888, 5952, 6016, 6080, 6144
};

OAL_DTCM_DATA_SECTION
UINT32 algo_f1_table[AGLO_K_TABLE_LEN] = {
      3,   7,  19,   7,   7,  11,   5,  11,   7,  41, 103,  15,   9,  17,   9,  21,
    101,  21,  57,  23,  13,  27,  11,  27,  85,  29,  33,  15,  17,  33, 103,  19,
     19,  37,  19,  21,  21, 115, 193,  21, 133,  81,  45,  23, 243, 151, 155,  25,
     51,  47,  91,  29,  29, 247,  29,  89,  91, 157,  55,  31,  17,  35, 227,  65,
     19,  37,  41,  39, 185,  43,  21, 155,  79, 139,  23, 217,  25,  17, 127,  25,
    239,  17, 137, 215,  29,  15, 147,  29,  59,  65,  55,  31,  17, 171,  67,  35,
     19,  39,  19, 199,  21, 211,  21,  43, 149,  45,  49,  71,  13,  17,  25, 183,
     55, 127,  27,  29,  29,  57,  45,  31,  59, 185, 113,  31,  17, 171, 209, 253,
    367, 265, 181,  39,  27, 127, 143,  43,  29,  45, 157,  47,  13, 111, 443,  51,
     51, 451, 257,  57, 313, 271, 179, 331, 363, 375, 127,  31,  33,  43,  33, 477,
     35, 233, 357, 337,  37,  71,  71,  37,  39, 127,  39,  39,  31, 113,  41, 251,
     43, 21,   43,  45,  45, 161,  89, 323,  47,  23,  47, 263
};

OAL_DTCM_DATA_SECTION
UINT32 algo_f2_table[AGLO_K_TABLE_LEN] = {
     10,  12,  42,  16,  18,  20,  22,  24,  26,  84,  90,  32,  34, 108,  38, 120,
     84,  44,  46,  48,  50,  52,  36,  56,  58,  60,  62,  32, 198,  68, 210,  36,
     74,  76,  78, 120,  82,  84,  86,  44,  90,  46,  94,  48,  98,  40, 102,  52,
    106,  72, 110, 168, 114,  58, 118, 180, 122,  62,  84,  64,  66,  68, 420,
     96,  74,  76, 234,  80,  82, 252,  86,  44, 120,  92,  94,  48,  98,  80, 102,  52,
    106,  48, 110, 112, 114,  58, 118,  60, 122, 124,  84,  64,  66, 204, 140,  72,  74,
     76,  78, 240,  82, 252,  86,  88,  60,  92, 846,  48,  28,  80, 102, 104, 954,  96,
    110, 112, 114, 116, 354, 120, 610, 124, 420,  64,  66, 136, 420, 216, 444, 456, 468,
     80, 164, 504, 172,  88, 300,  92, 188,  96,  28, 240, 204, 104, 212, 192, 220, 336, 
    228, 232, 236, 120, 244, 248, 168,  64, 130, 264, 134, 408, 138, 280, 142, 480, 146,
    444, 120, 152, 462, 234, 158,  80,  96, 902, 166, 336, 170,  86, 174, 176, 178, 120,
    182, 184, 186,  94, 190, 480
};
/****************lte~{5wVF~}0~{1d3I~}1~{#,~}1~{1d3I~}0~{#,~}llr~{2;PhR*H!8:2YWw~},~{GR~}Q14~{6(1j~}*************************/
#if 0
OAL_DTCM_DATA_SECTION complex_s16_t bpsk_mapping_table[2] ={{0xD2BF,0xD2BF},{0x2D41,0x2D41}};/*Q=D[31:16],I=D[15:0],Q14*/
OAL_DTCM_DATA_SECTION complex_s16_t bpsk_mapping_conj_table[2] ={{0xD2BF,0x2D41},{0x2D41,0xD2BF}};/*Q=D[31:16],I=D[15:0],Q14*/
OAL_DTCM_DATA_SECTION complex_s16_t bpsk_mapping_minusconj_table[2] ={{0x2D41,0xD2BF},{0xD2BF,0x2D41}};/*Q=D[31:16],I=D[15:0],Q14*/

OAL_DTCM_DATA_SECTION complex_s16_t qbsk_mapping_table[4] = {{0xD2BF,0xD2BF},{0x2D41,0xD2BF},{0xD2BF,0x2D41},{0x2D41,0x2D41}}; /*Q=D[31:16],I=D[15:0],Q14*/
OAL_DTCM_DATA_SECTION complex_s16_t qbsk_mapping_conj_table[4] = {{0xD2BF,0x2D41},{0x2D41,0x2D41},{0xD2BF,0xD2BF},{0x2D41,0xD2BF}}; /*Q=D[31:16],I=D[15:0],Q14*/
OAL_DTCM_DATA_SECTION complex_s16_t qbsk_mapping_minusconj_table[4] = {{0x2D41,0xD2BF},{0xD2BF,0xD2BF},{0x2D41,0x2D41},{0xD2BF,0x2D41}}; /*Q=D[31:16],I=D[15:0],Q14*/

OAL_DTCM_DATA_SECTION complex_s16_t qam16_mapping_table[16] =
{
    {0xC349,0xC349}, //15
    {0x3CB7,0xC349},// 14
    {0xC349,0x3CB7},//13
    {0x3CB7,0x3CB7},//12
    {0xEBC3,0xC349},//11
    {0x143D,0xC349}, // 10
    {0xEBC3,0x3CB7}, //9
    {0x143D,0x3CB7},//8
    {0xC349,0xEBC3},//7
    {0x3CB7,0xEBC3}, //6
    {0xC349,0x143D}, // 5
    {0x3CB7,0x143D},//4//4
    {0xEBC3,0xEBC3}, //3//
    {0x143D,0xEBC3},// 2
    {0xEBC3,0x143D},// 1
    {0x143D,0x143D},//0
}; /*Q=D[31:16],I=D[15:0],Q14 SCALING*/

OAL_DTCM_DATA_SECTION complex_s16_t qam16_mapping_conj_table[16] ={
    {0xC349,0x3CB7}, //15
    {0x3CB7,0x3CB7},// 14
    {0xC349,0xC349},//13
    {0x3CB7,0xC349},//12
    {0xEBC3,0x3CB7},//11
    {0x143D,0x3CB7}, // 10
    {0xEBC3,0xC349}, //9
    {0x143D,0xC349},//8
    {0xC349,0x143D},//7
    {0x3CB7,0x143D}, //6
    {0xC349,0xEBC3}, // 5
    {0x3CB7,0xEBC3},//4//4
    {0xEBC3,0x143D}, //3//
    {0x143D,0x143D},// 2
    {0xEBC3,0xEBC3},// 1
    {0x143D,0xEBC3},//0
}; /*Q=D[31:16],I=D[15:0],Q14 SCALING*/

OAL_DTCM_DATA_SECTION complex_s16_t qam16_mapping_minusconj_table[16] =
{
    {0x3CB7,0xC349}, //15
    {0xC349,0xC349},// 14
    {0x3CB7,0x3CB7},//13
    {0xC349,0x3CB7},//12
    {0x143D,0xC349},//11
    {0xEBC3,0xC349}, // 10
    {0x143D,0x3CB7}, //9
    {0xEBC3,0x3CB7},//8
    {0x3CB7,0xEBC3},//7
    {0xC349,0xEBC3}, //6
    {0x3CB7,0x143D}, // 5
    {0xC349,0x143D},//4//4
    {0x143D,0xEBC3}, //3//
    {0xEBC3,0xEBC3},// 2
    {0x143D,0x143D},// 1
    {0xEBC3,0x143D},//0
}; /*Q=D[31:16],I=D[15:0],Q14 SCALING*/
#else
OAL_DTCM_DATA_SECTION complex_s16_t bpsk_mapping_table[2] ={{0xE960,0xE960},{0x16A0,0x16A0}};/*Q=D[31:16],I=D[15:0],Q13*/
OAL_DTCM_DATA_SECTION complex_s16_t bpsk_mapping_conj_table[2] ={{0xE960,0x16A0},{0x16A0,0xE960}};/*Q=D[31:16],I=D[15:0],Q13*/
OAL_DTCM_DATA_SECTION complex_s16_t bpsk_mapping_minusconj_table[2] ={{0x16A0,0xE960},{0xE960,0x16A0}};/*Q=D[31:16],I=D[15:0],Q13*/

OAL_DTCM_DATA_SECTION complex_s16_t qbsk_mapping_table[4] = {{0xE960,0xE960},{0x16A0,0xE960},{0xE960,0x16A0},{0x16A0,0x16A0}}; /*Q=D[31:16],I=D[15:0],Q13*/
OAL_DTCM_DATA_SECTION complex_s16_t qbsk_mapping_conj_table[4] = {{0xE960,0x16A0},{0x16A0,0x16A0},{0xE960,0xE960},{0x16A0,0xE960}}; /*Q=D[31:16],I=D[15:0],Q13*/
OAL_DTCM_DATA_SECTION complex_s16_t qbsk_mapping_minusconj_table[4] = {{0x16A0,0xE960},{0xE960,0xE960},{0x16A0,0x16A0},{0xE960,0x16A0}}; /*Q=D[31:16],I=D[15:0],Q13*/

OAL_DTCM_DATA_SECTION complex_s16_t qam16_mapping_table[16] =
{
    {0xE1A5,0xE1A5}, //15
    {0x1E5B,0xE1A5},// 14
    {0xE1A5,0x1E5B},//13
    {0x1E5B,0x1E5B},//12
    {0xF5E2,0xE1A5},//11
    {0x0A1E,0xE1A5}, // 10
    {0xF5E2,0x1E5B}, //9
    {0x0A1E,0x1E5B},//8
    {0xE1A5,0xF5E2},//7
    {0x1E5B,0xF5E2}, //6
    {0xE1A5,0x0A1E}, // 5
    {0x1E5B,0x0A1E},//4//4
    {0xF5E2,0xF5E2}, //3//
    {0x0A1E,0xF5E2},// 2
    {0xF5E2,0x0A1E},// 1
    {0x0A1E,0x0A1E},//0
}; /*Q=D[31:16],I=D[15:0],Q14 SCALING*/

OAL_DTCM_DATA_SECTION complex_s16_t qam16_mapping_conj_table[16] =
{
    {0xE1A5,0x1E5B}, //15
    {0x1E5B,0x1E5B},// 14
    {0xE1A5,0xE1A5},//13
    {0x1E5B,0xE1A5},//12
    {0xF5E2,0x1E5B},//11
    {0x0A1E,0x1E5B}, // 10
    {0xF5E2,0xE1A5}, //9
    {0x0A1E,0xE1A5},//8
    {0xE1A5,0x0A1E},//7
    {0x1E5B,0x0A1E}, //6
    {0xE1A5,0xF5E2}, // 5
    {0x1E5B,0xF5E2},//4//4
    {0xF5E2,0x0A1E}, //3//
    {0x0A1E,0x0A1E},// 2
    {0xF5E2,0xF5E2},// 1
    {0x0A1E,0xF5E2},//0
}; /*Q=D[31:16],I=D[15:0],Q14 SCALING*/

OAL_DTCM_DATA_SECTION complex_s16_t qam16_mapping_minusconj_table[16] =
{
    {0x1E5B,0xE1A5}, //15
    {0xE1A5,0xE1A5},// 14
    {0x1E5B,0x1E5B},//13
    {0xE1A5,0x1E5B},//12
    {0x0A1E,0xE1A5},//11
    {0xF5E2,0xE1A5}, // 10
    {0x0A1E,0x1E5B}, //9
    {0xF5E2,0x1E5B},//8
    {0x1E5B,0xF5E2},//7
    {0xE1A5,0xF5E2}, //6
    {0x1E5B,0x0A1E}, // 5
    {0xE1A5,0x0A1E},//4//4
    {0x0A1E,0xF5E2}, //3//
    {0xF5E2,0xF5E2},// 2
    {0x0A1E,0x0A1E},// 1
    {0xF5E2,0x0A1E},//0
}; /*Q=D[31:16],I=D[15:0],Q14 SCALING*/
#endif
OAL_DTCM_DATA_SECTION complex_s16_t qam64_mapping_table[64] =
{
    {0xDD70,0xDD70},// 0
    {0x2290,0xDD70},// 1
    {0xDD70,0x2290},// 2
    {0x2290,0x2290},// 3
    {0xFB10,0xDD70},// 4
    {0x4F0,0xDD70},// 5
	{0xFB10,0x2290},// 6
    {0x4F0,0x2290},// 7
    {0xDD70,0xFB10},// 8
    {0x2290,0xFB10},// 9
    {0xDD70,0x4F0},// 10
    {0x2290,0x4F0},// 11
    {0xFB10,0xFB10},// 12
    {0x4F0,0xFB10},// 13
	{0xFB10,0x4F0},// 14
    {0x4F0,0x4F0},// 15
    {0xE750,0xDD70},// 16
    {0x18B0,0xDD70},// 17
    {0xE750,0x2290},// 18
    {0x18B0,0x2290},// 19
    {0xF130,0xDD70},// 20
    {0xED0,0xDD70},// 21
	{0xF130,0x2290},// 22
    {0xED0,0x2290},// 23
    {0xE750,0xFB10},// 24
    {0x18B0,0xFB10},// 25
	{0xE750,0x4F0},// 26
    {0x18B0,0x4F0},// 27
    {0xF130,0xFB10},// 28
    {0xED0,0xFB10},// 29
	{0xF130,0x4F0},// 30
    {0xED0,0x4F0},// 31
    {0xDD70,0xE750},// 32
    {0x2290,0xE750},// 33
    {0xDD70,0x18B0},// 34
    {0x2290,0x18B0},// 33
    {0xFB10,0xE750},// 36
    {0x4F0,0xE750},// 37
    {0xFB10,0x18B0},// 38
    {0x4F0,0x18B0},// 39
    {0xDD70,0xF130},// 40
    {0x2290,0xF130},// 41
    {0xDD70,0xED0},// 42
    {0x2290,0xED0},// 43
    {0xFB10,0xF130},// 44
    {0x4F0,0xF130},// 45
	{0xFB10,0xED0},// 46
    {0x4F0,0xED0},// 47
    {0xE750,0xE750},// 48
    {0x18B0,0xE750},// 49
    {0xE750,0x18B0},// 50
    {0x18B0,0x18B0},// 51
    {0xF130,0xE750},// 52
    {0xED0,0xE750},// 53
	{0xF130,0x18B0},// 54
    {0xED0,0x18B0},// 55
    {0xE750,0xF130},// 56
    {0x18B0,0xF130},// 57
	{0xE750,0xED0},// 58
    {0x18B0,0xED0},// 59
    {0xF130,0xF130},// 60
    {0xED0,0xF130},// 61
    {0xF130,0xED0},// 62
    {0xED0,0xED0},// 63
}; /*Q=D[31:16],I=D[15:0],Q13 SCALING*/
OAL_DTCM_DATA_SECTION complex_s16_t qam64_mapping_conj_table[64] =
{
    {0xDD70,0x2290},// 0
    {0x2290,0x2290},// 1
    {0xDD70,0xDD70},// 2
    {0x2290,0xDD70},// 3
    {0xFB10,0x2290},// 4
    {0x4F0,0x2290},// 5
	{0xFB10,0xDD70},// 6
    {0x4F0,0xDD70},// 7
    {0xDD70,0x4F0},// 8
    {0x2290,0x4F0},// 9
    {0xDD70,0xFB10},// 10
    {0x2290,0xFB10},// 11
    {0xFB10,0x4F0},// 12
    {0x4F0,0x4F0},// 13
	{0xFB10,0xFB10},// 14
    {0x4F0,0xFB10},// 15
    {0xE750,0x2290},// 16
    {0x18B0,0x2290},// 17
    {0xE750,0xDD70},// 18
    {0x18B0,0xDD70},// 19
    {0xF130,0x2290},// 20
    {0xED0,0x2290},// 21
	{0xF130,0xDD70},// 22
    {0xED0,0xDD70},// 23
    {0xE750,0x4F0},// 24
    {0x18B0,0x4F0},// 25
	{0xE750,0xFB10},// 26
    {0x18B0,0xFB10},// 27
    {0xF130,0x4F0},// 28
    {0xED0,0x4F0},// 29
	{0xF130,0xFB10},// 30
    {0xED0,0xFB10},// 31
    {0xDD70,0x18B0},// 32
    {0x2290,0x18B0},// 33
    {0xDD70,0xE750},// 34
    {0x2290,0xE750},// 33
    {0xFB10,0x18B0},// 36
    {0x4F0,0x18B0},// 37
    {0xFB10,0xE750},// 38
    {0x4F0,0xE750},// 39
    {0xDD70,0xED0},// 40
    {0x2290,0xED0},// 41
    {0xDD70,0xF130},// 42
    {0x2290,0xF130},// 43
    {0xFB10,0xED0},// 44
    {0x4F0,0xED0},// 45
	{0xFB10,0xF130},// 46
    {0x4F0,0xF130},// 47
    {0xE750,0x18B0},// 48
    {0x18B0,0x18B0},// 49
    {0xE750,0xE750},// 50
    {0x18B0,0xE750},// 51
    {0xF130,0x18B0},// 52
    {0xED0,0x18B0},// 53
	{0xF130,0xE750},// 54
    {0xED0,0xE750},// 55
    {0xE750,0xED0},// 56
    {0x18B0,0xED0},// 57
	{0xE750,0xF130},// 58
    {0x18B0,0xF130},// 59
    {0xF130,0xED0},// 60
    {0xED0,0xED0},// 61
    {0xF130,0xF130},// 62
    {0xED0,0xF130},// 63
}; /*Q=D[31:16],I=D[15:0],Q13 SCALING*/

OAL_DTCM_DATA_SECTION complex_s16_t qam64_mapping_minusconj_table[64] =
{
    {0x2290,0xDD70},// 0
    {0xDD70,0xDD70},// 1
    {0x2290,0x2290},// 2
    {0xDD70,0x2290},// 3
    {0x4F0,0xDD70},// 4
    {0xFB10,0xDD70},// 5
	{0x4F0,0x2290},// 6
    {0xFB10,0x2290},// 7
    {0x2290,0xFB10},// 8
    {0xDD70,0xFB10},// 9
    {0x2290,0x4F0},// 10
    {0xDD70,0x4F0},// 11
    {0x4F0,0xFB10},// 12
    {0xFB10,0xFB10},// 13
	{0x4F0,0x4F0},// 14
    {0xFB10,0x4F0},// 15
    {0x18B0,0xDD70},// 16
    {0xE750,0xDD70},// 17
    {0x18B0,0x2290},// 18
    {0xE750,0x2290},// 19
    {0xED0,0xDD70},// 20
    {0xF130,0xDD70},// 21
	{0xED0,0x2290},// 22
    {0xF130,0x2290},// 23
    {0x18B0,0xFB10},// 24
    {0xE750,0xFB10},// 25
	{0x18B0,0x4F0},// 26
    {0xE750,0x4F0},// 27
    {0xED0,0xFB10},// 28
    {0xF130,0xFB10},// 29
	{0xED0,0x4F0},// 30
    {0xF130,0x4F0},// 31
    {0x2290,0xE750},// 32
    {0xDD70,0xE750},// 33
    {0x2290,0x18B0},// 34
    {0xDD70,0x18B0},// 33
    {0x4F0,0xE750},// 36
    {0xFB10,0xE750},// 37
    {0x4F0,0x18B0},// 38
    {0xFB10,0x18B0},// 39
    {0x2290,0xF130},// 40
    {0xDD70,0xF130},// 41
    {0x2290,0xED0},// 42
    {0xDD70,0xED0},// 43
    {0x4F0,0xF130},// 44
    {0xFB10,0xF130},// 45
	{0x4F0,0xED0},// 46
    {0xFB10,0xED0},// 47
    {0x18B0,0xE750},// 48
    {0xE750,0xE750},// 49
    {0x18B0,0x18B0},// 50
    {0xE750,0x18B0},// 51
    {0xED0,0xE750},// 52
    {0xF130,0xE750},// 53
	{0xED0,0x18B0},// 54
    {0xF130,0x18B0},// 55
    {0x18B0,0xF130},// 56
    {0xE750,0xF130},// 57
	{0x18B0,0xED0},// 58
    {0xE750,0xED0},// 59
    {0xED0,0xF130},// 60
    {0xF130,0xF130},// 61
    {0xED0,0xED0},// 62
    {0xF130,0xED0},// 63
}; /*Q=D[31:16],I=D[15:0],Q13 SCALING*/
OAL_DTCM_DATA_SECTION complex_s16_t qam256_mapping_table[256] =
{
    #include "./qam256_table.dat"
};
/*----------------------------DEBUG-----------------------------------*/
//OAL_DTCM_DATA_SECTION UINT32 algo_timer_tcv[100] = {0,0,0,0,0,0,0,0,0,0};
//OAL_DTCM_DATA_SECTION UINT32 algo_cnt   = 0;
/***********************************************************************************************************************
* EXTERN VARIABLES DECLARATION
***********************************************************************************************************************/
/***********************************************************************************************************************
* FUNCTION
*
* VOID algo_ul_ctrl_info_encode_cfg(VOID)
*
* DESCRIPTION
*
*
*
* NOTE
*
*   <the limitations to use this function or other comments>
*
* PARAMETERS
*
*   NULL
*
* RETURNS
*
*   NULL
*
* VERSION
*
*   <DATE>           <AUTHOR>           <CR_ID>              <DESCRIPTION>
*
***********************************************************************************************************************/
OAL_ITCM_CODE_SECTION
VOID algo_ul_ctrl_info_encode_cfg(VOID)
{
    ctrl_info_cfg.u32_in_bit_length           = ZZW_PLCP_BIT_LEN - 24;
    ctrl_info_cfg.u32_NL                      = 1;
    ctrl_info_cfg.u32_out_bit_length          = PLCP_ENCODE_LEN;
    ctrl_info_cfg.u32_Qm                      = QPSK;//rd_mode~{N*~}3~{J1#,KYBJF%Ed1XPk04UU~}64QAM~{=xPP~}
    ctrl_info_cfg.u32_rv                      = 0;
    ctrl_info_cfg.u32_cp_stytle               = NORMAL_CP;
    ctrl_info_cfg.u32_sch_symb_num            = 12;
    ctrl_info_cfg.u32_endian_mode             = 0;
    ctrl_info_cfg.u32_rd_mode                 = 3;
    ctrl_info_cfg.u32_ul_sch_ram_sel   = 0;
    ctrl_info_cfg.u32_cfg_ubp_frm_flag = 0;
}

/***********************************************************************************************************************
* FUNCTION
*
* VOID VOID algo_tx_frame_dmac_addr_updata(UINT32 data_num)
*
* DESCRIPTION
*
*
*
* NOTE
*
*   <the limitations to use this function or other comments>
*
* PARAMETERS
*
*   NULL
*
* RETURNS
*
*   NULL
*
* VERSION
*
*   <DATE>           <AUTHOR>           <CR_ID>              <DESCRIPTION>
*
***********************************************************************************************************************/
#if 0
OAL_ITCM_CODE_SECTION
VOID algo_tx_frame_dmac_addr_init(UINT8 u8_frm_ch_id, UINT32_PTR data_addr)
{
#ifndef VECTOR_TX_TEST
   frame_dmac_info[u8_frm_ch_id][0] = (UINT32)&gap_data[u8_frm_ch_id][0];
   frame_dmac_info[u8_frm_ch_id][1] = (UINT32)&agc_cca[u8_frm_ch_id][0];
   frame_dmac_info[u8_frm_ch_id][2] = (UINT32)&pilot_data[u8_frm_ch_id][0];
   frame_dmac_info[u8_frm_ch_id][3] = (UINT32)data_addr;
   frame_dmac_info[u8_frm_ch_id][4] = (UINT32)(data_addr + SYMBOL_LEN);
   frame_dmac_info[u8_frm_ch_id][5] = (UINT32)(data_addr + SYMBOL_LEN * 2);
   frame_dmac_info[u8_frm_ch_id][6] = (UINT32)(data_addr + SYMBOL_LEN * 3);
   frame_dmac_info[u8_frm_ch_id][7] = (UINT32)(data_addr + SYMBOL_LEN * 4);
   frame_dmac_info[u8_frm_ch_id][8] = (UINT32)&pilot_data[u8_frm_ch_id][0];
   frame_dmac_info[u8_frm_ch_id][9] = (UINT32)(data_addr + SYMBOL_LEN * 5);
   frame_dmac_info[u8_frm_ch_id][10] = (UINT32)(data_addr + SYMBOL_LEN * 6);
   frame_dmac_info[u8_frm_ch_id][11] = (UINT32)(data_addr + SYMBOL_LEN * 7);
   frame_dmac_info[u8_frm_ch_id][12] = (UINT32)(data_addr + SYMBOL_LEN * 8);
   frame_dmac_info[u8_frm_ch_id][13] = (UINT32)(data_addr + SYMBOL_LEN * 9);
   frame_dmac_info[u8_frm_ch_id][14] = (UINT32)&pilot_data[u8_frm_ch_id][0];
   frame_dmac_info[u8_frm_ch_id][15] = (UINT32)&gap_data[u8_frm_ch_id][0];

   frame_dmac_info[u8_frm_ch_id][16] = (UINT32)&gap_data[u8_frm_ch_id][0];
   frame_dmac_info[u8_frm_ch_id][17] = (UINT32)&agc_cca[u8_frm_ch_id][0];
   frame_dmac_info[u8_frm_ch_id][18] = (UINT32)&pilot_data[u8_frm_ch_id][0];
   frame_dmac_info[u8_frm_ch_id][19] = (UINT32)(data_addr + SYMBOL_LEN * 10);
   frame_dmac_info[u8_frm_ch_id][20] = (UINT32)(data_addr + SYMBOL_LEN * 11);
   frame_dmac_info[u8_frm_ch_id][21] = (UINT32)(data_addr + SYMBOL_LEN * 12);
   frame_dmac_info[u8_frm_ch_id][22] = (UINT32)(data_addr + SYMBOL_LEN * 13);
   frame_dmac_info[u8_frm_ch_id][23] = (UINT32)(data_addr + SYMBOL_LEN * 14);
   frame_dmac_info[u8_frm_ch_id][24] = (UINT32)&pilot_data[u8_frm_ch_id][0];
   frame_dmac_info[u8_frm_ch_id][25] = (UINT32)(data_addr + SYMBOL_LEN * 15);
   frame_dmac_info[u8_frm_ch_id][26] = (UINT32)(data_addr + SYMBOL_LEN * 16);
   frame_dmac_info[u8_frm_ch_id][27] = (UINT32)(data_addr + SYMBOL_LEN * 17);
   frame_dmac_info[u8_frm_ch_id][28] = (UINT32)(data_addr + SYMBOL_LEN * 18);
   frame_dmac_info[u8_frm_ch_id][29] = (UINT32)(data_addr + SYMBOL_LEN * 19);
   frame_dmac_info[u8_frm_ch_id][30] = (UINT32)&pilot_data[u8_frm_ch_id][0];
   frame_dmac_info[u8_frm_ch_id][31] = (UINT32)&gap_data[u8_frm_ch_id][0];

   frame_dmac_info[u8_frm_ch_id][32] = (UINT32)&gap_data[u8_frm_ch_id][0];
   frame_dmac_info[u8_frm_ch_id][33] = (UINT32)&agc_cca[u8_frm_ch_id][0];
   frame_dmac_info[u8_frm_ch_id][34] = (UINT32)&pilot_data[u8_frm_ch_id][0];
   frame_dmac_info[u8_frm_ch_id][35] = (UINT32)(data_addr + SYMBOL_LEN * 20);
   frame_dmac_info[u8_frm_ch_id][36] = (UINT32)(data_addr + SYMBOL_LEN * 21);
   frame_dmac_info[u8_frm_ch_id][37] = (UINT32)(data_addr + SYMBOL_LEN * 22);
   frame_dmac_info[u8_frm_ch_id][38] = (UINT32)(data_addr + SYMBOL_LEN * 23);
   frame_dmac_info[u8_frm_ch_id][39] = (UINT32)(data_addr + SYMBOL_LEN * 24);
   frame_dmac_info[u8_frm_ch_id][40] = (UINT32)&pilot_data[u8_frm_ch_id][0];
   frame_dmac_info[u8_frm_ch_id][41] = (UINT32)(data_addr + SYMBOL_LEN * 25);
   frame_dmac_info[u8_frm_ch_id][42] = (UINT32)(data_addr + SYMBOL_LEN * 26);
   frame_dmac_info[u8_frm_ch_id][43] = (UINT32)(data_addr + SYMBOL_LEN * 27);
   frame_dmac_info[u8_frm_ch_id][44] = (UINT32)(data_addr + SYMBOL_LEN * 28);
   frame_dmac_info[u8_frm_ch_id][45] = (UINT32)(data_addr + SYMBOL_LEN * 29);
   frame_dmac_info[u8_frm_ch_id][46] = (UINT32)&pilot_data[u8_frm_ch_id][0];
   frame_dmac_info[u8_frm_ch_id][47] = (UINT32)&gap_data[u8_frm_ch_id][0];    
#else
   frame_dmac_info[u8_frm_ch_id][0] = (UINT32)&framing_data[u8_frm_ch_id][0];	// gap
   frame_dmac_info[u8_frm_ch_id][1] = frame_dmac_info[u8_frm_ch_id][0] + GP_LENGTH;	// agc-cca
   frame_dmac_info[u8_frm_ch_id][2] = frame_dmac_info[u8_frm_ch_id][1] + AGC_LENGTH + CCA_LENGTH;	// pilot
   frame_dmac_info[u8_frm_ch_id][3] = frame_dmac_info[u8_frm_ch_id][2] + SLOT_LENGTH;	// plcp
   frame_dmac_info[u8_frm_ch_id][4] = frame_dmac_info[u8_frm_ch_id][3] + SLOT_LENGTH;		// data 1
   frame_dmac_info[u8_frm_ch_id][5] = frame_dmac_info[u8_frm_ch_id][4] + SLOT_LENGTH;		// data 2
   frame_dmac_info[u8_frm_ch_id][6] = frame_dmac_info[u8_frm_ch_id][5] + SLOT_LENGTH;		// data 3
   frame_dmac_info[u8_frm_ch_id][7] = frame_dmac_info[u8_frm_ch_id][6] + SLOT_LENGTH;		// data 4
   frame_dmac_info[u8_frm_ch_id][8] = frame_dmac_info[u8_frm_ch_id][7] + SLOT_LENGTH;		// pilot
   frame_dmac_info[u8_frm_ch_id][9] = frame_dmac_info[u8_frm_ch_id][8] + SLOT_LENGTH;		// data 5
   frame_dmac_info[u8_frm_ch_id][10] = frame_dmac_info[u8_frm_ch_id][9] + SLOT_LENGTH;		// data 6
   frame_dmac_info[u8_frm_ch_id][11] = frame_dmac_info[u8_frm_ch_id][10] + SLOT_LENGTH;		// data 7
   frame_dmac_info[u8_frm_ch_id][12] = frame_dmac_info[u8_frm_ch_id][11] + SLOT_LENGTH;		// data 8
   frame_dmac_info[u8_frm_ch_id][13] = frame_dmac_info[u8_frm_ch_id][12] + SLOT_LENGTH;		// data 9
   frame_dmac_info[u8_frm_ch_id][14] = frame_dmac_info[u8_frm_ch_id][13] + SLOT_LENGTH;		// pilot
   frame_dmac_info[u8_frm_ch_id][15] = frame_dmac_info[u8_frm_ch_id][14] + SLOT_LENGTH;		// gap

   frame_dmac_info[u8_frm_ch_id][16] = (UINT32)&framing_data[u8_frm_ch_id][0];	// gap
   frame_dmac_info[u8_frm_ch_id][17] = frame_dmac_info[u8_frm_ch_id][0] + GP_LENGTH;	// agc-cca
   frame_dmac_info[u8_frm_ch_id][18] = frame_dmac_info[u8_frm_ch_id][1] + AGC_LENGTH + CCA_LENGTH;	// pilot
   frame_dmac_info[u8_frm_ch_id][19] = frame_dmac_info[u8_frm_ch_id][2] + SLOT_LENGTH;	// plcp
   frame_dmac_info[u8_frm_ch_id][20] = frame_dmac_info[u8_frm_ch_id][3] + SLOT_LENGTH;		// data 1
   frame_dmac_info[u8_frm_ch_id][21] = frame_dmac_info[u8_frm_ch_id][4] + SLOT_LENGTH;		// data 2
   frame_dmac_info[u8_frm_ch_id][22] = frame_dmac_info[u8_frm_ch_id][5] + SLOT_LENGTH;		// data 3
   frame_dmac_info[u8_frm_ch_id][23] = frame_dmac_info[u8_frm_ch_id][6] + SLOT_LENGTH;		// data 4
   frame_dmac_info[u8_frm_ch_id][24] = frame_dmac_info[u8_frm_ch_id][7] + SLOT_LENGTH;		// pilot
   frame_dmac_info[u8_frm_ch_id][25] = frame_dmac_info[u8_frm_ch_id][8] + SLOT_LENGTH;		// data 5
   frame_dmac_info[u8_frm_ch_id][26] = frame_dmac_info[u8_frm_ch_id][9] + SLOT_LENGTH;		// data 6
   frame_dmac_info[u8_frm_ch_id][27] = frame_dmac_info[u8_frm_ch_id][10] + SLOT_LENGTH;		// data 7
   frame_dmac_info[u8_frm_ch_id][28] = frame_dmac_info[u8_frm_ch_id][11] + SLOT_LENGTH;		// data 8
   frame_dmac_info[u8_frm_ch_id][29] = frame_dmac_info[u8_frm_ch_id][12] + SLOT_LENGTH;		// data 9
   frame_dmac_info[u8_frm_ch_id][30] = frame_dmac_info[u8_frm_ch_id][13] + SLOT_LENGTH;		// pilot
   frame_dmac_info[u8_frm_ch_id][31] = frame_dmac_info[u8_frm_ch_id][14] + SLOT_LENGTH;		// gap

   frame_dmac_info[u8_frm_ch_id][32] = (UINT32)&framing_data[u8_frm_ch_id][0];	// gap
   frame_dmac_info[u8_frm_ch_id][33] = frame_dmac_info[u8_frm_ch_id][0] + GP_LENGTH;	// agc-cca
   frame_dmac_info[u8_frm_ch_id][34] = frame_dmac_info[u8_frm_ch_id][1] + AGC_LENGTH + CCA_LENGTH;	// pilot
   frame_dmac_info[u8_frm_ch_id][35] = frame_dmac_info[u8_frm_ch_id][2] + SLOT_LENGTH;	// plcp
   frame_dmac_info[u8_frm_ch_id][36] = frame_dmac_info[u8_frm_ch_id][3] + SLOT_LENGTH;		// data 1
   frame_dmac_info[u8_frm_ch_id][37] = frame_dmac_info[u8_frm_ch_id][4] + SLOT_LENGTH;		// data 2
   frame_dmac_info[u8_frm_ch_id][38] = frame_dmac_info[u8_frm_ch_id][5] + SLOT_LENGTH;		// data 3
   frame_dmac_info[u8_frm_ch_id][39] = frame_dmac_info[u8_frm_ch_id][6] + SLOT_LENGTH;		// data 4
   frame_dmac_info[u8_frm_ch_id][40] = frame_dmac_info[u8_frm_ch_id][7] + SLOT_LENGTH;		// pilot
   frame_dmac_info[u8_frm_ch_id][41] = frame_dmac_info[u8_frm_ch_id][8] + SLOT_LENGTH;		// data 5
   frame_dmac_info[u8_frm_ch_id][42] = frame_dmac_info[u8_frm_ch_id][9] + SLOT_LENGTH;		// data 6
   frame_dmac_info[u8_frm_ch_id][43] = frame_dmac_info[u8_frm_ch_id][10] + SLOT_LENGTH;		// data 7
   frame_dmac_info[u8_frm_ch_id][44] = frame_dmac_info[u8_frm_ch_id][11] + SLOT_LENGTH;		// data 8
   frame_dmac_info[u8_frm_ch_id][45] = frame_dmac_info[u8_frm_ch_id][12] + SLOT_LENGTH;		// data 9
   frame_dmac_info[u8_frm_ch_id][46] = frame_dmac_info[u8_frm_ch_id][13] + SLOT_LENGTH;		// pilot
   frame_dmac_info[u8_frm_ch_id][47] = frame_dmac_info[u8_frm_ch_id][14] + SLOT_LENGTH;		// gap
#endif
}
#else
OAL_ITCM_CODE_SECTION
VOID algo_tx_frame_dmac_addr_init(UINT8 u8_frm_ch_id, UINT32_PTR u32p_data_addr)
{
	UINT8 u8_rpt_times = 3;
	UINT32 u32_i = 0;
	UINT32 u32_rpt_idx = 0;
	UINT32 u32_len_idx, u32_map_idx, u32_seg_num = 0, u32_data_sym_cnt = 0;

	UINT32 u32_seg_addr[MAX_SEG_TYPE] = {(UINT32)&gap_data[u8_frm_ch_id][0], (UINT32)&agc_cca[u8_frm_ch_id][0], \
				(UINT32)&pilot_data[u8_frm_ch_id][0], (UINT32)u32p_data_addr, (UINT32)&gap_data[u8_frm_ch_id][0]}; 
	WAVEFORM_OFDM_CFG *stp_waveform_cfg = &g_st_waveform_cfg;
				
	for(u32_rpt_idx = 0; u32_rpt_idx < u8_rpt_times; u32_rpt_idx++)
	{
		u32_seg_num = 0;
		//u32_data_sym_cnt = 0;
		//for(u32_i = 0; u32_i < TOTAL_FRAME_SEG_NUM; u32_i++)
		for(u32_map_idx = 0; u32_map_idx < ((stp_waveform_cfg->u8_total_frame_seg + 0x7) >> 3); u32_map_idx++)
		{
			for(u32_i = 0; u32_i < 8; u32_i++)
			{
				u32_len_idx = (stp_waveform_cfg->u32_frame_seg_map[u32_map_idx] >> (u32_i * 4)) & 0xF;
				if(u32_len_idx EQ DATA_SYM_TYPE)
				{
					frame_dmac_info[u8_frm_ch_id][u32_rpt_idx * TOTAL_FRAME_SEG_NUM + u32_i + (u32_map_idx << 3)] = \
							u32_seg_addr[u32_len_idx] + ((u32_data_sym_cnt * SYMBOL_LEN) << 2);
					u32_data_sym_cnt += 1;
				}
				else
				{
					frame_dmac_info[u8_frm_ch_id][u32_rpt_idx * TOTAL_FRAME_SEG_NUM + u32_i + (u32_map_idx << 3)] = u32_seg_addr[u32_len_idx];
				}
				if(u32_seg_num++ >= TOTAL_FRAME_SEG_NUM)
				{
					break;
				}
			}
		}		
	}
	
	
}
#endif
/***********************************************************************************************************************
* FUNCTION
*
*   algo_ul_turbo_encoder_para_config
*
* DESCRIPTION
*
*   turbo encoder para calc
*
* NOTE
*
*   <the limitations to use this function or other comments>
*
* PARAMETERS
*
*   NULL
*
* RETURNS
*
*   NULL
*
* VERSION
*
*   <DATE>           <AUTHOR>           <CR_ID>              <DESCRIPTION>
*
***********************************************************************************************************************/
OAL_ITCM_CODE_SECTION
VOID algo_ul_turbo_encoder_para_config(algo_tx_brp_para* tx_brp_para, TURBO_ENCODER_PARA* turbo_encoder_para)
{
    UINT32 B,L,C,B1,sch_symb_num,G,endian_mode;
    UINT32 ii, r;
    UINT32 k_index;
    UINT32 K, K_plus, K_minus, C_plus, C_minus, delta_K, F;
    UINT32 NL, Qm, G1, gamma, E_minus, E_plus, rv;
    UINT32 D, Kr[20], E[2], R_TC_subblock[20],Nd[20],K_pi,Kw,Ncb,k0[20];
    UINT32 ubp_cb1_len, ubp_cb2_len, sch_row_num, ubp_cb_fill_num, ubp_cb1_num, ubp_cb_num, ubp_rm_cb1,ubp_cb_num_h,ubp_cb1_num_h;
    UINT32 ubp_cb1_f1, ubp_cb1_f2, ubp_cb2_f1, ubp_cb2_f2, ubp_int_fill_num1, ubp_int_fill_num2;
    UINT32 ubp_int_row_num1, ubp_int_row_num2, ubp_rm_k1, ubp_rm_rv1, ubp_rm_k2, ubp_rm_rv2;
    UINT32 ubp_rmout_num1, ubp_rmout_num2;
    UINT32 H1,H2;
    B  = tx_brp_para->u32_in_bit_length + AGLO_CRC24_LEN;
    NL = tx_brp_para->u32_NL;
    Qm = tx_brp_para->u32_Qm;
    G  = tx_brp_para->u32_out_bit_length;
    rv = tx_brp_para->u32_rv;
    sch_symb_num = tx_brp_para->u32_sch_symb_num;
    endian_mode  = tx_brp_para->u32_endian_mode;
    if(tx_brp_para->u32_rd_mode == 3)
    {
        Qm = QAM64;
    }
    if (B <= AGLO_CB_MAX_LEN)
    {
        L  = 0;
        C  = 1;
        B1 = B;
    }
    else
    {
        L  = AGLO_CRC24_LEN;
        C  = (B + AGLO_CB_MAX_LEN - L - 1)/(AGLO_CB_MAX_LEN - L);     //ceil()
        B1 = B + C*L;
    }

    for(ii=0; ii < AGLO_K_TABLE_LEN; ii++)
    {
        if(C*algo_k_table[ii] >= B1)
        {
            k_index = ii;
            break;
        }
    }

    K = algo_k_table[k_index];
    
    if(C == 1)
    {
        K_plus  = K;
        K_minus = 0; 
        C_minus = 0;
        C_plus  = 1;
    }
    else
    {
        K_plus  = K;
        K_minus = algo_k_table[k_index - 1];
        delta_K = K_plus - K_minus;
        C_minus = (C*K_plus - B1)/delta_K;
        C_plus  = C - C_minus;
    }
    F = C_plus*K_plus + C_minus*K_minus - B1;

    for(r = 0; r < C; r++)
    {
        if(r < C_minus)
        {
            Kr[r] = K_minus;
        }
        else
        {
            Kr[r] = K_plus;
        }
    }

    G1      = G/(NL*Qm);
    gamma   = G1%C;
    E_minus = C-gamma;
    E_plus  = E_minus;
    for(r = 0; r < C; r++)
    {
        D = Kr[r] + 4;
        R_TC_subblock[r] = (D + AGLO_C_TC_SUBBLOCK - 1)/AGLO_C_TC_SUBBLOCK;
        Nd[r]   = R_TC_subblock[r]*AGLO_C_TC_SUBBLOCK - D;
        K_pi = R_TC_subblock[r]*AGLO_C_TC_SUBBLOCK;
        Kw   = 3*K_pi;
        Ncb  = Kw;

        if(r <= (E_minus-1))
        {
            E[0] = NL*Qm*(G1/C);
        }
        else
        {
            E[1] = NL*Qm*((G1 + C - 1)/C);
        }

        k0[r] = R_TC_subblock[r]*(2*((Ncb + 8*R_TC_subblock[r] - 1)/(8*R_TC_subblock[r]))*rv + 2);
    }
    ubp_cb1_len = K_minus;
    ubp_cb2_len = K_plus;
    //sch_row_num = (G+sch_symb_num*Qm - 1)/(sch_symb_num*Qm);
    H1 = (G+Qm-1)/Qm;
    H2 = ((H1*Qm)+12-1)/12;
    sch_row_num = (H2+Qm-1)/Qm;
    ubp_cb_fill_num = F;
    ubp_cb1_num = C_minus;
    ubp_cb_num  = C;
    ubp_rm_cb1  = E_minus;

    if(C > 15)
    {
        ubp_cb_num_h = 1;
        ubp_cb_num    = (C%16);
    }
    else
    {
        ubp_cb_num_h = 0;
    }

    if(C_minus > 15)
    {
        ubp_cb1_num_h = 1;
        ubp_cb1_num   = (C_minus%16) ;
    }
    else
    {
        ubp_cb1_num_h = 0;
    }

    if(C_minus == 0)
    {
        ubp_cb1_f2        = 0;
        ubp_cb1_f1        = 0;
        ubp_cb2_f2        = algo_f2_table[k_index];
        ubp_cb2_f1        = algo_f1_table[k_index];
        ubp_int_fill_num2 = Nd[0];
        ubp_int_fill_num1 = 0;
        ubp_int_row_num2  = R_TC_subblock[0];
        ubp_int_row_num1  = 0;
        ubp_rm_rv2        = rv;
        ubp_rm_k2         = k0[0];
        ubp_rm_rv1        = 0;
        ubp_rm_k1         = 0;
    }
    else
    {
        ubp_cb1_f2        = algo_f2_table[k_index - 1];
        ubp_cb1_f1        = algo_f1_table[k_index - 1];
        ubp_cb2_f2        = algo_f2_table[k_index];
        ubp_cb2_f1        = algo_f1_table[k_index];
        ubp_int_fill_num2 = Nd[C - 1];
        ubp_int_fill_num1 = Nd[C_minus - 1];;
        ubp_int_row_num2  = R_TC_subblock[C - 1];
        ubp_int_row_num1  = R_TC_subblock[C_minus - 1];
        ubp_rm_rv2        = rv;
        ubp_rm_k2         = k0[C - 1];
        ubp_rm_rv1        = rv;
        ubp_rm_k1         = k0[C_minus - 1];
    }

    ubp_rmout_num1 = E[0];
    if(C==1)
    {
        ubp_rmout_num2 = 0;
    }
    else
    {
        ubp_rmout_num2 = E[1];
    }

    turbo_encoder_para->turbo_encoder_lteu_ubp_para0.u32_sch_trch_act      = 1;
    turbo_encoder_para->turbo_encoder_lteu_ubp_para0.u32_reserved0         = 0;
    turbo_encoder_para->turbo_encoder_lteu_ubp_para0.u32_ubp_mode          = 0;
    turbo_encoder_para->turbo_encoder_lteu_ubp_para0.u32_cp_stytle         = tx_brp_para->u32_cp_stytle;
    turbo_encoder_para->turbo_encoder_lteu_ubp_para0.u32_pucch_format      = 0;
    turbo_encoder_para->turbo_encoder_lteu_ubp_para0.u32_ul_sch_ram_sel    = tx_brp_para->u32_ul_sch_ram_sel;
    turbo_encoder_para->turbo_encoder_lteu_ubp_para0.u32_reserved1         = 0;
    turbo_encoder_para->turbo_encoder_lteu_ubp_para0.u32_endian_mode       = endian_mode;

    if(Qm == QAM64)
    {
        turbo_encoder_para->turbo_encoder_lteu_ubp_para0.u32_qam64         = 1;
        turbo_encoder_para->turbo_encoder_lteu_ubp_para0.u32_modu_mode     = 0;
    }
    else
    {
        turbo_encoder_para->turbo_encoder_lteu_ubp_para0.u32_qam64         = 0;
        turbo_encoder_para->turbo_encoder_lteu_ubp_para0.u32_modu_mode     = Qm/2 - 1;
    }

    turbo_encoder_para->turbo_encoder_lteu_ubp_para1.u32_ubp_cb1_len       = ubp_cb1_len;
    turbo_encoder_para->turbo_encoder_lteu_ubp_para1.u32_reserved0         = 0;
    turbo_encoder_para->turbo_encoder_lteu_ubp_para1.u32_ubp_cb2_len       = ubp_cb2_len;
    turbo_encoder_para->turbo_encoder_lteu_ubp_para1.u32_reserved1         = 0;

    turbo_encoder_para->turbo_encoder_lteu_ubp_para2.u32_ubp_cb1_f1        = ubp_cb1_f1;
    turbo_encoder_para->turbo_encoder_lteu_ubp_para2.u32_reserved0         = 0;
    turbo_encoder_para->turbo_encoder_lteu_ubp_para2.u32_ubp_cb1_f2        = ubp_cb1_f2;
    turbo_encoder_para->turbo_encoder_lteu_ubp_para2.u32_reserved1         = 0;

    turbo_encoder_para->turbo_encoder_lteu_ubp_para3.u32_ubp_cb2_f1        = ubp_cb2_f1;
    turbo_encoder_para->turbo_encoder_lteu_ubp_para3.u32_ubp_cb_num_h      = ubp_cb_num_h;
    turbo_encoder_para->turbo_encoder_lteu_ubp_para3.u32_ubp_cb1_num_h     = ubp_cb1_num_h;
    turbo_encoder_para->turbo_encoder_lteu_ubp_para3.u32_reserved0         = 0;
    turbo_encoder_para->turbo_encoder_lteu_ubp_para3.u32_ubp_cb2_f2        = ubp_cb2_f2;
    turbo_encoder_para->turbo_encoder_lteu_ubp_para3.u32_reserved1         = 0;

    turbo_encoder_para->turbo_encoder_lteu_ubp_para4.u32_ubp_cb_num        = ubp_cb_num;
    turbo_encoder_para->turbo_encoder_lteu_ubp_para4.u32_ubp_cb1_num       = ubp_cb1_num;
    turbo_encoder_para->turbo_encoder_lteu_ubp_para4.u32_ubp_cb_fill_num   = ubp_cb_fill_num;
    turbo_encoder_para->turbo_encoder_lteu_ubp_para4.u32_sch_symb_num      = 12;
    turbo_encoder_para->turbo_encoder_lteu_ubp_para4.u32_sch_row_num       = sch_row_num;

    turbo_encoder_para->turbo_encoder_lteu_ubp_para5.u32_ubp_int_row_num1  = ubp_int_row_num1;
    turbo_encoder_para->turbo_encoder_lteu_ubp_para5.u32_ubp_int_row_num2  = ubp_int_row_num2;
    turbo_encoder_para->turbo_encoder_lteu_ubp_para5.u32_ubp_int_fill_num1 = ubp_int_fill_num1;
    turbo_encoder_para->turbo_encoder_lteu_ubp_para5.u32_ubp_int_fill_num2 = ubp_int_fill_num2;
    turbo_encoder_para->turbo_encoder_lteu_ubp_para5.u32_ubp_rm_cb1        = ubp_rm_cb1;
    turbo_encoder_para->turbo_encoder_lteu_ubp_para5.u32_reserved          = 0;

    turbo_encoder_para->turbo_encoder_lteu_ubp_para6.u32_ubp_rmout_num1    = ubp_rmout_num1;
    turbo_encoder_para->turbo_encoder_lteu_ubp_para6.u32_ubp_rmout_num2    = ubp_rmout_num2;

    turbo_encoder_para->turbo_encoder_lteu_ubp_para7.u32_ubp_rm_k1         = ubp_rm_k1;
    turbo_encoder_para->turbo_encoder_lteu_ubp_para7.u32_ubp_rm_rv1        = ubp_rm_rv1;
    turbo_encoder_para->turbo_encoder_lteu_ubp_para7.u32_ubp_rm_k2         = ubp_rm_k2;
    turbo_encoder_para->turbo_encoder_lteu_ubp_para7.u32_ubp_rm_rv2        = ubp_rm_rv2;
    turbo_encoder_para->turbo_encoder_cfg_rd_mode.u32_rd_mode              = tx_brp_para->u32_rd_mode;
    turbo_encoder_para->turbo_encoder_cfg_ubp_frm_flag.u32_cfg_ubp_frm_flag = tx_brp_para->u32_cfg_ubp_frm_flag;
}

/***********************************************************************************************************************
* FUNCTION
*   
*   algo_ul_dmac_callback
*
* DESCRIPTION
*
*   dmac callback func
*
* NOTE
*
*   <the limitations to use this function or other comments>
*
* PARAMETERS
*
*   NULL
*
* RETURNS
*
*   NULL
*
* VERSION
*
*   <DATE>           <AUTHOR>           <CR_ID>              <DESCRIPTION>
*
***********************************************************************************************************************/
OAL_ITCM_CODE_SECTION
VOID algo_ul_dmac_callback()
{
    oal_msg_t *stp_msg = NULL_PTR;

    KS_LOG_TMRL(0xA009);   

    if(dmac_cnt == 0)
    {
       *(volatile UINT32*)TURBO_ENCODER_CFG_USP_ENABLE_ADDR &= 0xfffffffe;
       dmac_cnt = 1;
    }
    else
    {
       *(volatile UINT32*)TURBO_ENCODER_CFG_USP_ENABLE_ADDR &= 0xfffffffd;
       dmac_cnt = 0;
    }
    
    dma_send_cnt++;	

	//if(u32_data_type EQ (UINT32)DATA_TYPE_PLCP)
	if(dmac_cnt EQ (UINT32)1)
	{	
		//plcp mod direct
		#ifdef PLCP_SOFT_COMBINE
		ks_oal_mem_copy((VOID *)&ctrl_encode_output[PLCP_ENCODE_LEN >> 3], (VOID *)ctrl_encode_output, PLCP_ENCODE_LEN >> 3);
		#endif
		algo_ul_ctrl_data_modulation();
	}
	else
	{	
		g_u32_turbo_left--;
		
		if(g_u32_turbo_left)
	    {
			algo_data_tx_process_plcp();
	    }
	    stp_msg = ks_oal_msg_create(0);
	    OAL_ASSERT(NULL_PTR != stp_msg, "");
	    stp_msg->msg_primitive_id = MSG_UL_MODULATION_PRIMITIVE_ID;
	    ks_oal_msg_send(stp_msg, L1CC_UL_MOD_MAIN_TASK_ID);
    } 
} 

OAL_ITCM_CODE_SECTION
VOID algo_ul_data_ifft_pre()
{
	SINT32 i;
	complex_s16_t *stp_ifft_out_ptr;
	FFT_STATUS	e_fft_sta;

	OAL_IRQ_SAVE_AREA;

	KS_LOG_TMRL(0xA203);  

#ifdef SCFDMA_SWITCH
	// dft shift
	for(i = 0; i < TOTAL_MOD_SYM_NUM; i++)
	{
		ks_oal_mem_copy((UINT32_PTR)&mod_output_data[i][1448],(UINT32_PTR)&mod_output_data[i][600],600<<2);
		ks_oal_mem_set((UINT32_PTR)&mod_output_data[i][600], 0, 600<<2);
		ks_oal_mem_copy_backward((UINT32_PTR)&mod_output_data[i][1], (UINT32_PTR)&mod_output_data[i][0], 600<<2);
		ks_oal_mem_set((UINT32_PTR)&mod_output_data[i][0],0,4);
	}
#endif

#ifndef VECTOR_TX_TEST
	stp_ifft_out_ptr = (complex_s16_t *)ifft_output_data_ant1 + (g_u8_algo_ul_data_mod_cnt_out * 2048 * TOTAL_MOD_SYM_NUM);
#endif	

	// antenna 1 ifft use FFT0
	OAL_IRQ_DISABLE;
	e_fft_sta = fft_get_status(KS_FFT0);	
	OAL_IRQ_RESTORE;

	OAL_ASSERT(FFT_IDLE EQ e_fft_sta, "");	
	for(i = 0; i < TOTAL_MOD_SYM_NUM; i++)
	{	
		algo_ul_ifft0_data_cfg.fft_rd_lli_item[i].src		  = (0x11000000+(UINT32)(&mod_output_data[i][0]));
#ifndef VECTOR_TX_TEST			
		algo_ul_ifft0_data_cfg.fft_wr_lli_item[i].dst		  = (UINT32)((stp_ifft_out_ptr + i * 2048));			
#else
		algo_ul_ifft0_data_cfg.fft_wr_lli_item[i].dst		  = algo_ul_ifft0_data_cfg.fft_rd_lli_item[i].src;
#endif			
	}		
	
	fft_dma_start(&algo_ul_ifft0_data_cfg); 
	fft_ingress_interface(KS_FFT0,&algo_ul_ifft0_data_cfg);

	#if MIMO_2ANTS
	// antenna 2 SFBC	
	for(i=0; i<TOTAL_MOD_SYM_NUM; i++)
	{
		_mimo_sfbc_asm((UINT32_PTR)&mod_output_data[i][0], (UINT32_PTR)&mod_output_data[TOTAL_MOD_SYM_NUM + i][0]);
	}


#ifndef VECTOR_TX_TEST
	stp_ifft_out_ptr = (complex_s16_t *)ifft_output_data_ant2 + (g_u8_algo_ul_data_mod_cnt_out * 2048 * TOTAL_MOD_SYM_NUM);
#endif	

	OAL_IRQ_DISABLE;
	e_fft_sta = fft_get_status(KS_FFT1);	
	OAL_IRQ_RESTORE;

	OAL_ASSERT(FFT_IDLE EQ e_fft_sta, "");	


	// antenna 2 ifft use FFT1
	for(i=0; i<TOTAL_MOD_SYM_NUM; i++)
	{
		algo_ul_ifft1_data_cfg.fft_rd_lli_item[i].src		  = (0x11000000+(UINT32)&mod_output_data[TOTAL_MOD_SYM_NUM + i][0]);
#ifndef VECTOR_TX_TEST				
		algo_ul_ifft1_data_cfg.fft_wr_lli_item[i].dst		  = (UINT32)((stp_ifft_out_ptr + i * 2048));			 
#else
		algo_ul_ifft1_data_cfg.fft_wr_lli_item[i].dst		  = algo_ul_ifft1_data_cfg.fft_rd_lli_item[i].src;
#endif
	}
	
	fft_dma_start(&algo_ul_ifft1_data_cfg); 
	fft_ingress_interface(KS_FFT1,&algo_ul_ifft1_data_cfg); 
		
#endif

	g_u8_algo_ul_data_mod_cnt_out = (g_u8_algo_ul_data_mod_cnt_out + 1)%ALGO_DATA_MOD_CNT_MAX;	

	KS_LOG(OAL_TRACE_LEVEL_0 , 0xA808 , (g_u8_algo_ul_data_mod_cnt_in<<4)|g_u8_algo_ul_data_mod_cnt_out);
}

/***********************************************************************************************************************
* FUNCTION
*
* algo_ul_scram
*
* DESCRIPTION
*
* scram data
*
* NOTE
*
*   <the limitations to use this function or other comments>
*
* PARAMETERS
*
*   NULL
*
* RETURNS
*
*   NULL
*
* VERSION
*
*   <DATE>           <AUTHOR>           <CR_ID>              <DESCRIPTION>
*
***********************************************************************************************************************/
OAL_ITCM_CODE_SECTION
VOID algo_ul_scram(IN UINT16 tb_size, IN UINT32* input_buf, OUT UINT32* output_buf)
{
	_scramble_asm(input_buf, output_buf, u32_scram_table, ((tb_size + 3) >> 2));
}

/***********************************************************************************************************************
* FUNCTION
*
* algo_ul_scram
*
* DESCRIPTION
*
* scram data
*
* NOTE
*
*   <the limitations to use this function or other comments>
*
* PARAMETERS
*
*   NULL
*
* RETURNS
*
*   NULL
*
* VERSION
*
*   <DATE>           <AUTHOR>           <CR_ID>              <DESCRIPTION>
*
***********************************************************************************************************************/
OAL_ITCM_CODE_SECTION
VOID algo_ul_scram_data(IN UINT16 tb_size,IN UINT32 mcs, IN UINT8* input_buf, OUT UINT8* output_buf)
{
    UINT32 u32_cb_index;
    UINT8 *u8p_in = input_buf;
    UINT8 *u8p_out = output_buf;    
    UINT32 u32_len;
    UINT32 u32_cb_num = 0;
	
	u32_len = (g_st_mcs_tbl[mcs].u32_raw_len_cb - 24) >> 3; //unit : byte
	u32_cb_num = (UINT32)g_st_mcs_tbl[mcs].u16_cb_num;
	for(u32_cb_index = 0; u32_cb_index < u32_cb_num; u32_cb_index++)
	{
		u8p_in = input_buf + u32_len * u32_cb_index;
		u8p_out = output_buf + u32_len * u32_cb_index;

		_scramble_asm(u8p_in, u8p_out, u32_scram_table, ((u32_len + 15) >> 4));
	}  
}

/***********************************************************************************************************************
* FUNCTION
*   
*   algo_ul_turbo_enc_callback
*
* DESCRIPTION
*
*   turbo encode finished callback func
*
* NOTE
*
*   <the limitations to use this function or other comments>
*
* PARAMETERS
*
*   NULL
*
* RETURNS
*
*   NULL
*
* VERSION
*
*   <DATE>           <AUTHOR>           <CR_ID>              <DESCRIPTION>
*
***********************************************************************************************************************/
OAL_ITCM_CODE_SECTION
VOID algo_ul_turbo_enc_callback()
{
    UINT32 block_ts;
    UINT32 u32_in_bit_length = 0;
    UINT32 u32_mcs_idx = 0;
	
	KS_LOG_TMRL(0xA000);		
	KS_LOG(OAL_TRACE_LEVEL_0 , 0xA802 , g_u32_turbo_status);	

    turbo_encoder_set_usp();

    if(g_u32_turbo_status EQ 0)
    {
		u32_in_bit_length = algo_ul_encoder_info.tx_brp_cfg[0].u32_in_bit_length;
		KS_LOG(OAL_TRACE_LEVEL_0 , 0xAF02 , (UINT16)u32_in_bit_length);	

    	//PLCP 
        turbo_encoder_dmad_single_copy(KS_DMAC2_CP, (UINT32)TURBO_ENCODER_INPUT_PING_ADDR, (UINT32)ctrl_encode_output + 0x11000000, 160, algo_ul_dmac_callback);

		g_u32_turbo_status = 1;
		
    	/*~{EdVCOB4N5D~}turbo~{1`Bk2NJ}~}*/
        turbo_encoder_register_reset();

        u32_mcs_idx = algo_ul_encoder_info.mcs_idx;
        algo_ul_scram_data(u32_in_bit_length, u32_mcs_idx, ((UINT8 *)algo_ul_encoder_info.data_addr[0]),\
            ((UINT8 *)algo_ul_encoder_info.ping_pong_addr[0]));
		
		if(g_u8_turbo_encoder_para_ready[u32_mcs_idx + 1] EQ OAL_FALSE)
		{
        	algo_ul_turbo_encoder_para_config(&algo_ul_encoder_info.tx_brp_cfg[0], &g_st_algo_turbo_encoder_para[u32_mcs_idx + 1]);
        	g_u8_turbo_encoder_para_ready[u32_mcs_idx + 1] = OAL_TRUE;
        }
		
        turbo_encoder_run(&g_st_algo_turbo_encoder_para[u32_mcs_idx + 1]);
    }
    else
    {   
		block_ts = ((algo_ul_encoder_info.tx_brp_cfg[0].u32_out_bit_length\
			/algo_ul_encoder_info.modulation)+15)/16;

		if(algo_ul_encoder_info.modulation == BPSK)
		{
		   block_ts = (algo_ul_encoder_info.tx_brp_cfg[0].u32_out_bit_length+127)/128;
		}

		KS_LOG(OAL_TRACE_LEVEL_0 , 0xAF01 , (UINT16)block_ts);	  

		turbo_encoder_dmad_single_copy(KS_DMAC1_CP, (UINT32)TURBO_ENCODER_INPUT_PING_ADDR,\
			(UINT32)&data_encode_output[0][0] + 0x11000000, block_ts*16, algo_ul_dmac_callback); 

	 	g_u32_turbo_status = 0;		

    }

	encode_cnt++;

}
/***********************************************************************************************************************
* FUNCTION
*   
*   algo_ul_modulation
*
* DESCRIPTION
*
*   tx data modulation
*
* NOTE
*
*   <the limitations to use this function or other comments>
*
* PARAMETERS
*
*   NULL
*
* RETURNS
*
*   NULL
*
* VERSION
*
*   <DATE>           <AUTHOR>           <CR_ID>              <DESCRIPTION>
*
***********************************************************************************************************************/
OAL_ITCM_CODE_SECTION
VOID algo_ul_modulation(UINT8 mod_type,UINT32 offset)
{
    UINT32 u32_mod_index, i, j;
	complex_s16_t * mod_table[5] = {bpsk_mapping_table, qbsk_mapping_table, qam16_mapping_table, qam64_mapping_table, qam256_mapping_table};

#ifdef SCFDMA_SWITCH
	if(BPSK == mod_type)
	{
		for(j = 0 ; j < 75; j++)
		{
			for(i = 0 ; i < 8 ; i++)
			{
	            //~{5wVF~}+~{WSTX2(S3Id~}
	    		u32_mod_index = (*((UINT8_PTR)&data_encode_output[0][0] + offset * 150 + j ) >> i)&0x1;
	            mod_output_data[offset + 1][j * 8 + i] = bpsk_mapping_table[u32_mod_index];
	    		u32_mod_index = (*((UINT8_PTR)&data_encode_output[0][0] + offset * 150 + j + 75) >> i)&0x1;
	            mod_output_data[offset + 1][j * 8 + i + 600] = bpsk_mapping_table[u32_mod_index];				
			}
		}		
	}
	else
    {    
		_modulation_asm(&data_encode_output[offset][0], &mod_output_data[offset + 1][0], mod_table[mod_type >> 1], 50);
	}
#else
	if(BPSK == mod_type)
	{
		for(j = 0 ; j < 75; j++)
		{
			for(i = 0 ; i < 8 ; i++)
			{
	            //~{5wVF~}+~{WSTX2(S3Id~}
	    		u32_mod_index = (*((UINT8_PTR)&data_encode_output[0][0] + offset * 150 + j ) >> i)&0x1;
	            mod_output_data[offset + 1][(j << 3) + i + 1448] = bpsk_mapping_table[u32_mod_index];			
				u32_mod_index = (*((UINT8_PTR)&data_encode_output[0][0] + offset * 150 + j + 75) >> i)&0x1;
				mod_output_data[offset + 1][(j << 3) + i + 1] = bpsk_mapping_table[u32_mod_index];		
			}
		}	
	}
	else
    {
	    _modulation_asm(&data_encode_output[offset][600], &mod_output_data[offset + 1][1], mod_table[mod_type >> 1], 25);
	    _modulation_asm(&data_encode_output[offset][0], &mod_output_data[offset + 1][1448], mod_table[mod_type >> 1], 25);
	}
#endif	
}

/***********************************************************************************************************************
* FUNCTION
*   
*   algo_ul_ctrl_data_modulation
*
* DESCRIPTION
*
*   ctrl info modulation
*
* NOTE
*
*   <the limitations to use this function or other comments>
*
* PARAMETERS
*
*   NULL
*
* RETURNS
*
*   NULL
*
* VERSION
*
*   <DATE>           <AUTHOR>           <CR_ID>              <DESCRIPTION>
*
***********************************************************************************************************************/
VOID algo_ul_ctrl_data_modulation()
{
    UINT32 u32_mod_index, j, i;
	UINT16 *ctrl_encode_out_ptr = (UINT16 *)&ctrl_encode_output[0];

#ifdef SCFDMA_SWITCH
	for(j = 0 ; j < 75; j++)
	{
		for(i = 0 ; i < 8 ; i++)
		{
            //~{5wVF~}+~{WSTX2(S3Id~}
    		u32_mod_index = (ctrl_encode_output[j] >> i)&0x1;
            mod_output_data[0][j * 8 + i] = bpsk_mapping_table[u32_mod_index];
    		u32_mod_index = (ctrl_encode_output[75 + j] >> i)&0x1;
            mod_output_data[0][600 + j * 8 + i] = bpsk_mapping_table[u32_mod_index];			
		}
	}	
#else
	for(j = 0 ; j < 75; j++)
	{
		for(i = 0 ; i < 8 ; i++)
		{
            //~{5wVF~}+~{WSTX2(S3Id~}
    		u32_mod_index = (ctrl_encode_output[j] >> i)&0x1;
            mod_output_data[0][(j << 3) + i + 1448] = bpsk_mapping_table[u32_mod_index];			
			u32_mod_index = (ctrl_encode_output[j+75] >> i)&0x1;
			mod_output_data[0][(j << 3) + i + 1] = bpsk_mapping_table[u32_mod_index];		
		}
	}
#endif
}

/***********************************************************************************************************************
* FUNCTION
*   
*   algo_ul_encode_info_calc
*
* DESCRIPTION
*
*   according to l1cc info ,calc encoding information
*
* NOTE
*
*   <the limitations to use this function or other comments>
*
* PARAMETERS
*
*   NULL
*
* RETURNS
*
*   NULL
*
* VERSION
*
*   <DATE>           <AUTHOR>           <CR_ID>              <DESCRIPTION>
*
***********************************************************************************************************************/
OAL_ITCM_CODE_SECTION
VOID algo_ul_encode_info_calc(l1cc2algo_ctrl_info_t *l1cc2algo_ctrl_info, UINT32 u32_buf_id)
{
	UINT16 idx = 0;
	UINT16 indata_size,outdata_size;
	//UINT32 data_offset;

	algo_ul_encoder_info.tx_mod = 0;
	algo_ul_encoder_info.block_num = l1cc2algo_ctrl_info->u8_data_num;
	algo_ul_encoder_info.mcs_idx   = l1cc2algo_ctrl_info->u8_mcs;
	
	algo_ul_encoder_info.modulation = (UINT32)(g_st_mcs_tbl[l1cc2algo_ctrl_info->u8_mcs].u16_mod_type);
	g_u8_algo_ul_data_mod_type = algo_ul_encoder_info.modulation;
	
	algo_ul_encoder_info.rd_mode = (UINT32)(g_st_mcs_tbl[l1cc2algo_ctrl_info->u8_mcs].u16_rd_mode);
	algo_ul_encoder_info.turbo_encode_cnt = 1;
	
	indata_size = (g_st_mcs_tbl[l1cc2algo_ctrl_info->u8_mcs].u32_raw_len);
	//data_offset = ((indata_size + 7) >> 3);
	outdata_size  = (g_st_mcs_tbl[l1cc2algo_ctrl_info->u8_mcs].u32_len);	

	algo_ul_encoder_info.ping_pong_addr[idx]= turbo_input_addr[(idx+1)&1];

	if(l1cc2algo_ctrl_info->u8_data_type == 0)
	{
	   algo_ul_encoder_info.data_addr[0] = (UINT32)&g_zzw_sf_b[8];
	}
	else
	{
	   //algo_ul_encoder_info[g_u8_algo_ul_encoder_info_cnt_in].data_addr[idx] = (UINT32)&g_u8_tx_buffer[u32_buf_id][idx*data_offset + 16];
	   algo_ul_encoder_info.data_addr[idx] = (UINT32)&g_u8_tx_buffer[u32_buf_id][TX_BUFF_HEAD_SIZE];
	}
	algo_ul_encoder_info.tx_brp_cfg[idx].u32_in_bit_length           = indata_size;
	algo_ul_encoder_info.tx_brp_cfg[idx].u32_NL                      = 1;
	algo_ul_encoder_info.tx_brp_cfg[idx].u32_out_bit_length          = outdata_size;
	if(algo_ul_encoder_info.modulation EQ BPSK)
	{
		algo_ul_encoder_info.tx_brp_cfg[idx].u32_Qm                      = QPSK;
	}
	else
	{
		algo_ul_encoder_info.tx_brp_cfg[idx].u32_Qm                      = algo_ul_encoder_info.modulation;
	}
	algo_ul_encoder_info.tx_brp_cfg[idx].u32_rv                      = 0;
	algo_ul_encoder_info.tx_brp_cfg[idx].u32_cp_stytle               = NORMAL_CP;
	algo_ul_encoder_info.tx_brp_cfg[idx].u32_sch_symb_num            = 12;
	algo_ul_encoder_info.tx_brp_cfg[idx].u32_endian_mode             = 0;
	algo_ul_encoder_info.tx_brp_cfg[idx].u32_rd_mode                 = algo_ul_encoder_info.rd_mode;
	algo_ul_encoder_info.tx_brp_cfg[idx].u32_ul_sch_ram_sel          = (idx+1)&1;
	algo_ul_encoder_info.tx_brp_cfg[idx].u32_cfg_ubp_frm_flag        = (idx+1)&1;          
	//algo_ul_encoder_info[g_u8_algo_ul_encoder_info_cnt_in].tx_brp_cfg[idx].u32_ul_sch_ram_sel          = 0;
	//algo_ul_encoder_info[g_u8_algo_ul_encoder_info_cnt_in].tx_brp_cfg[idx].u32_cfg_ubp_frm_flag        = 0;  

}
/***********************************************************************************************************************
* FUNCTION
*
* VOID algo_tx_proc_init(VOID)
*
* DESCRIPTION
*
* tx proc init
*
* NOTE
*
*   <the limitations to use this function or other comments>
*
* PARAMETERS
*
*   NULL
*
* RETURNS
*
*   NULL
*
* VERSION
*
*   <DATE>           <AUTHOR>           <CR_ID>              <DESCRIPTION>
*
***********************************************************************************************************************/
OAL_ITCM_CODE_SECTION
VOID algo_tx_proc_init(VOID)
{
    turbo_encoder_init();
    turbo_encoder_set_callback_func(algo_ul_turbo_enc_callback);
    algo_ul_ctrl_info_encode_cfg();
	
#ifndef VECTOR_TX_TEST		
    algo_tx_frame_dmac_addr_init(0, (UINT32_PTR)ifft_output_data_ant1);
#else
	algo_tx_frame_dmac_addr_init(0, 0);	
#endif  

    fft_init(KS_FFT0);
    fft_init(KS_FFT1);
	algo_ul_ifft_config_init();
	
#ifdef SCFDMA_SWITCH
	algo_ul_dft1200_config_init();
#endif

#if MIMO_2ANTS
#ifndef VECTOR_TX_TEST	
	algo_tx_frame_dmac_addr_init(1, (UINT32_PTR)ifft_output_data_ant2);
#else
	algo_tx_frame_dmac_addr_init(1, 0);
#endif
#endif
	encode_cnt  = 0; 	

	{
		UINT32 u32_i = 0;

		#if 0
		for(u32_i = 0; u32_i < 512 * 2; u32_i++)
		{
			//*((UINT16_PTR)agc_cca + 816 + u32_i) <<= 1;
			*((UINT16_PTR)agc_cca + 816 + u32_i) += (*((UINT16_PTR)agc_cca + 816 + u32_i) >> 1);
		}

		for(u32_i = 0; u32_i < 512 * 2; u32_i++)
		{
			//*((UINT16_PTR)agc_cca + 816 + u32_i) <<= 1;
			*((UINT16_PTR)agc_cca + 816 + 1316 + u32_i) += (*((UINT16_PTR)agc_cca + 816 + 1316 + u32_i) >> 1);
		}
		#else
		#ifndef VECTOR_TX_TEST
		for(u32_i = 0; u32_i < (AGC_LEN) * 2; u32_i++)
		{
			//*((UINT16_PTR)agc_cca + 816 + u32_i) <<= 1;
			*((SINT16_PTR)agc_cca[0] + u32_i) = (*((SINT16_PTR)agc_cca[0] + u32_i) >> 1);
			*((SINT16_PTR)agc_cca[1] + u32_i) = (*((SINT16_PTR)agc_cca[1] + u32_i) >> 1);

		}
		
		for(u32_i = 0; u32_i < (CCA_LEN * 4); u32_i++)
		{
			//*((UINT16_PTR)agc_cca + 816 + u32_i) <<= 1;
			//*((SINT16_PTR)pilot_data + u32_i) += (*((SINT16_PTR)pilot_data + u32_i) << 1);
			*((SINT16_PTR)(agc_cca[0]) + AGC_LEN * 2 + u32_i) += (*((SINT16_PTR)(agc_cca[0]) + AGC_LEN * 2 + u32_i) >> 1);
			*((SINT16_PTR)(agc_cca[1]) + AGC_LEN * 2 + u32_i) += (*((SINT16_PTR)(agc_cca[1]) + AGC_LEN * 2 + u32_i) >> 1);
		}
		#endif
		#endif
	}
}

OAL_ITCM_CODE_SECTION
VOID algo_data_tx_process_plcp()
{
	algo_ul_scram((ctrl_info_cfg.u32_in_bit_length+31) >>5, (UINT32 *)&g_st_ctrl_info[g_u8_tx_ctrl_info_cnt_out], (UINT32 *)TURBO_ENCODER_INPUT_PING_ADDR);

	turbo_encoder_register_reset();
	
	g_u8_tx_ctrl_info_cnt_out = (g_u8_tx_ctrl_info_cnt_out + 1)%TX_CTRL_INFO_CNT_MAX;

	if(g_u8_turbo_encoder_para_ready[0] EQ OAL_FALSE)
	{
    	algo_ul_turbo_encoder_para_config(&ctrl_info_cfg, &g_st_algo_turbo_encoder_para[0]);
    	g_u8_turbo_encoder_para_ready[0] = OAL_TRUE;
    }	
	
	turbo_encoder_run(&g_st_algo_turbo_encoder_para[0]);
}

/***********************************************************************************************************************
* FUNCTION
*   
*   algo_data_tx_process
*
* DESCRIPTION
*
*   tx data proccess
*
* NOTE
*
*   <the limitations to use this function or other comments>
*
* PARAMETERS
*
*   NULL
*
* RETURNS
*
*   NULL
*
* VERSION
*
*   <DATE>           <AUTHOR>           <CR_ID>              <DESCRIPTION>
*
***********************************************************************************************************************/
OAL_ITCM_CODE_SECTION
VOID algo_data_tx_process(UINT32 u32_buf_id)
{
	VUINT32 u32_turbo_left = 0;
	TURBO_ENCODER_STATUS e_turbo_sta;	
	
	OAL_IRQ_SAVE_AREA;
	
    //algo_cnt                        = 0;
    dma_send_cnt                    = 0;
	encode_cnt                      = 0; 
    //dmac_cnt                        = 0; 
	fft_cnt                         = 0;

	KS_LOG(0, 0xa801, (g_u8_tx_ctrl_info_cnt_in<<8) | g_u8_tx_ctrl_info_cnt_out);
	
	//u32_ctrl_buf_id = algo_ul_queue_get_buf_id_in();
	//u32_ctrl_buf_id = algo_ul_queue_get_queue_out();	
	 
	OAL_ASSERT(g_u8_tx_ctrl_info_cnt_out NEQ g_u8_tx_ctrl_info_cnt_in, "");	
	algo_ul_encode_info_calc(&g_st_ctrl_info[g_u8_tx_ctrl_info_cnt_out], u32_buf_id);		

	OAL_IRQ_DISABLE;
	u32_turbo_left = g_u32_turbo_left;
	e_turbo_sta = turbo_encoder_get_status();
	g_u32_turbo_left++;
	OAL_IRQ_RESTORE;

	OAL_ASSERT((u32_turbo_left <= 2), "");
    #if 1
    if((e_turbo_sta EQ TURBO_ENCODER_IDLE) && (u32_turbo_left EQ 0))
    //algo_ul_scram((ctrl_info_cfg.u32_in_bit_length+31) >>5, (UINT32 *)&test_ctrl_data, (UINT32 *)TURBO_ENCODER_INPUT_PING_ADDR);
    {
		WRITE_REG32(0x1034000c, READ_REG32(0x1034000c) | ((0x1 << SRST_TURBO_ENCODER)));
		WRITE_REG32(0x1034000c, READ_REG32(0x1034000c) & (~(0x1 << SRST_TURBO_ENCODER)));
		algo_data_tx_process_plcp();
    }
    #endif
}

OAL_ITCM_CODE_SECTION
VOID ifft1_data_callback()
{
	KS_LOG_TMRL(0xA007);
	fft_set_status(KS_FFT1,FFT_IDLE);		
}
OAL_ITCM_CODE_SECTION
VOID ifft0_data_callback()
{
	KS_LOG_TMRL(0xA008);
	fft_set_status(KS_FFT0,FFT_IDLE);	
	g_bool_is_ant1_data_proc_done = OAL_TRUE;
}

/* data info dft0 callback */
OAL_ITCM_CODE_SECTION
VOID dft0_data_callback()
{
	oal_msg_t *stp_msg = NULL_PTR;

	KS_LOG_TMRL(0xA003);

	fft_set_status(KS_FFT0,FFT_IDLE);		
	
	stp_msg = ks_oal_msg_create(0);
	OAL_ASSERT(NULL_PTR != stp_msg, "");
	stp_msg->msg_primitive_id = MSG_UL_DFT_DATA_PRIMITIVE_ID;
	ks_oal_msg_send(stp_msg, L1CC_UL_MOD_MAIN_TASK_ID);

}

/***********************************************************************************************************************
* FUNCTION
*
* VOID algo_ul_ifft2048_data_cfg(VOID)
*
* DESCRIPTION
*
* ifft 2048 point cfg
*
* NOTE
*
*   <the limitations to use this function or other comments>
*
* PARAMETERS
*
*   NULL
*
* RETURNS
*
*   NULL
*
* VERSION
*
*   <DATE>           <AUTHOR>           <CR_ID>              <DESCRIPTION>
*
***********************************************************************************************************************/
OAL_ITCM_CODE_SECTION
VOID algo_ul_ifft2048_data_cfg(UINT32 symb_num,FFT_CFG *stp_fft_ip_cfg,eKS_FFT fft_sel)
{
	UINT32 i = 0;
	stp_fft_ip_cfg->eFft = fft_sel;
	stp_fft_ip_cfg->cp_idx = 0;
	stp_fft_ip_cfg->fft_ifft_sel = 1;
	stp_fft_ip_cfg->iq_data_width = 1;
	stp_fft_ip_cfg->cfg_sym_len_num = 1;
	stp_fft_ip_cfg->cfg_sym_len[0] = 2048;
	stp_fft_ip_cfg->cfg_sym_num = symb_num; 
	stp_fft_ip_cfg->cfg_sym_len_type[0] = 0;

	if(stp_fft_ip_cfg->cfg_sym_num <= 32)
	{
		stp_fft_ip_cfg->cfg_int_mask0 = 0xFFFFFFFF^(1<<(stp_fft_ip_cfg->cfg_sym_num-1));
		stp_fft_ip_cfg->cfg_int_mask1 = 0xFFFFFFFF;
	}
	else
	{
		stp_fft_ip_cfg->cfg_int_mask0 = 0xFFFFFFFF;
		stp_fft_ip_cfg->cfg_int_mask1 = 0xFFFFFFFF^(1<<(stp_fft_ip_cfg->cfg_sym_num-1));
	}
	
	for(i=0;i<stp_fft_ip_cfg->cfg_sym_num;i++)
	{
		 stp_fft_ip_cfg->fft_block_size[i] = 2048;
	}
	
	stp_fft_ip_cfg->ifft_ctrl_work_mode = 1;
	stp_fft_ip_cfg->cfg_wreq_cnt = 0x00000020;
	stp_fft_ip_cfg->cfg_hac_mode = 1;
	stp_fft_ip_cfg->cfg_output_mode = 1;
	stp_fft_ip_cfg->cfg_int_mask = 0;
	stp_fft_ip_cfg->cfg_shedu_en = 0;
	stp_fft_ip_cfg->cfg_ifft_start = 1;	
	stp_fft_ip_cfg->lli_mode = DMAC_LLI_LOOP;//DMAC_LLI_NORMAL;//DMAC_LLI_LOOP;
	stp_fft_ip_cfg->intr = DMAC_CH_BLOCK_TRF_DONE;//DMAC_CH_DMA_TRF_DONE;DMAC_CH_BLOCK_TRF_DONE;
	if(fft_sel EQ KS_FFT0)
	{
		stp_fft_ip_cfg->fft_dmac_fb_func = ifft0_data_callback;
		stp_fft_ip_cfg->fft_rd_lli_item = ifft0_link_in_item;
		stp_fft_ip_cfg->fft_wr_lli_item = ifft0_link_out_item;
	}
	else
	{
		stp_fft_ip_cfg->fft_dmac_fb_func = ifft1_data_callback;
		stp_fft_ip_cfg->fft_rd_lli_item = ifft1_link_in_item;
		stp_fft_ip_cfg->fft_wr_lli_item = ifft1_link_out_item;	
	}

}

/***********************************************************************************************************************
* FUNCTION
*
* VOID algo_ul_fft_dma_cfg(VOID)
*
* DESCRIPTION
*
* fft dma link info cfg
*
* NOTE
*
*   <the limitations to use this function or other comments>
*
* PARAMETERS
*
*   NULL
*
* RETURNS
*
*   NULL
*
* VERSION
*
*   <DATE>           <AUTHOR>           <CR_ID>              <DESCRIPTION>
*
***********************************************************************************************************************/
OAL_ITCM_CODE_SECTION
VOID algo_ul_fft_dma_cfg(FFT_CFG *stp_fft_cfg, UINT32 *fft_input,UINT32 *fft_output)
{
	fft_link_item_dsp_to_tce(stp_fft_cfg, fft_input);
    fft_link_item_tce_to_dsp(stp_fft_cfg, fft_output);
}

/***********************************************************************************************************************
* FUNCTION
*
* VOID algo_ul_ifft_config_init(VOID)
*
* DESCRIPTION
*
* ifft int
*
* NOTE
*
*   <the limitations to use this function or other comments>
*
* PARAMETERS
*
*   NULL
*
* RETURNS
*
*   NULL
*
* VERSION
*
*   <DATE>           <AUTHOR>           <CR_ID>              <DESCRIPTION>
*
***********************************************************************************************************************/
OAL_ITCM_CODE_SECTION
VOID algo_ul_ifft_config_init(VOID)
{
	algo_ul_ifft2048_data_cfg(TOTAL_MOD_SYM_NUM, &algo_ul_ifft0_data_cfg,KS_FFT0);
	algo_ul_fft_dma_cfg(&algo_ul_ifft0_data_cfg,(UINT32*)(0x11000000+(UINT32)&mod_output_data[0][0]),(UINT32*)(0x11000000+(UINT32)&mod_output_data[0][0]));
	algo_ul_ifft2048_data_cfg(TOTAL_MOD_SYM_NUM,&algo_ul_ifft1_data_cfg,KS_FFT1);
	algo_ul_fft_dma_cfg(&algo_ul_ifft1_data_cfg,(UINT32*)(0x11000000+(UINT32)&mod_output_data[0][0]),(UINT32*)(0x11000000+(UINT32)&mod_output_data[0][0]));
}

OAL_ITCM_CODE_SECTION
VOID algo_ul_data_symbproc(VOID)
{
	UINT32 i;
	UINT8 mod_type, tmp_index;
	FFT_STATUS e_fft_sta;

	OAL_IRQ_SAVE_AREA;

	KS_LOG_TMRL(0xA202);	

	mod_type = g_u8_algo_ul_data_mod_type;

	tmp_index = (g_u8_algo_ul_data_mod_cnt_in + 1)%ALGO_DATA_MOD_CNT_MAX;
	OAL_ASSERT(tmp_index != g_u8_algo_ul_data_mod_cnt_out, ""); 	
	g_u8_algo_ul_data_mod_cnt_in = tmp_index;	

	// Assert protection : modulation will modify mod_output_data[0][2048].
	OAL_ASSERT(OAL_TRUE EQ g_bool_is_ant1_data_proc_done, "");

	g_bool_is_ant1_data_proc_done = OAL_FALSE;	
	
	for(i = 0; i < (DATA_SYM_NUM); i++)
	{
		algo_ul_modulation(mod_type, i);
	}	

#ifdef SCFDMA_SWITCH
	OAL_IRQ_DISABLE;
	e_fft_sta = fft_get_status(KS_FFT0);	
	OAL_IRQ_RESTORE;

	OAL_ASSERT(FFT_IDLE EQ e_fft_sta, "");

	// dft for plcp + data
	for(i = 0; i < TOTAL_MOD_SYM_NUM; i++)
	{
		algo_ul_dft_fft0_cfg.fft_rd_lli_item[i].src		  = 0x11000000 + (UINT32)&mod_output_data[i][0];
		algo_ul_dft_fft0_cfg.fft_wr_lli_item[i].dst		  = algo_ul_dft_fft0_cfg.fft_rd_lli_item[i].src;			
	}	

	fft_dma_start(&algo_ul_dft_fft0_cfg); 
	fft_ingress_interface(KS_FFT0,&algo_ul_dft_fft0_cfg);		
#else
    algo_ul_data_ifft_pre();
#endif
	
	KS_LOG(OAL_TRACE_LEVEL_0 , 0xA807 , (mod_type<<8)|(g_u8_algo_ul_data_mod_cnt_in<<4)|g_u8_algo_ul_data_mod_cnt_out);

}

/***********************************************************************************************************************
* FUNCTION
*
* VOID algo_ul_dft1200_cfg(VOID)
*
* DESCRIPTION
*
* 1200 poinit DFT for data
*
* NOTE
*
*   <the limitations to use this function or other comments>
*
* PARAMETERS
*
*   NULL
*
* RETURNS
*
*   NULL
*
* VERSION
*
*   <DATE>           <AUTHOR>           <CR_ID>              <DESCRIPTION>
*
***********************************************************************************************************************/
OAL_ITCM_CODE_SECTION
VOID algo_ul_dft1200_cfg(UINT32 symb_num,FFT_CFG *stp_fft_ip_cfg)
{
	UINT32 i = 0;
	stp_fft_ip_cfg->eFft = KS_FFT0;
	stp_fft_ip_cfg->cp_idx = 0;
	stp_fft_ip_cfg->fft_ifft_sel = 0;	// fft

	stp_fft_ip_cfg->iq_data_width = 1;

	stp_fft_ip_cfg->cfg_sym_len_num = 1;

	stp_fft_ip_cfg->cfg_sym_len[0] = 1200;
	stp_fft_ip_cfg->cfg_sym_num = symb_num;
 
	stp_fft_ip_cfg->cfg_sym_len_type[0] = 0;

	if(stp_fft_ip_cfg->cfg_sym_num <= 32)
	{
		stp_fft_ip_cfg->cfg_int_mask0 = 0xFFFFFFFF^(1<<(stp_fft_ip_cfg->cfg_sym_num-1));
		stp_fft_ip_cfg->cfg_int_mask1 = 0xFFFFFFFF;
	}
	else
	{
		stp_fft_ip_cfg->cfg_int_mask0 = 0xFFFFFFFF;
		stp_fft_ip_cfg->cfg_int_mask1 = 0xFFFFFFFF^(1<<(stp_fft_ip_cfg->cfg_sym_num-1));
	}
	for(i=0;i<stp_fft_ip_cfg->cfg_sym_num;i++)
	{
		 stp_fft_ip_cfg->fft_block_size[i] = 1200;
	}
	stp_fft_ip_cfg->ifft_ctrl_work_mode = 1;
	stp_fft_ip_cfg->cfg_wreq_cnt = 0x00000020;
	stp_fft_ip_cfg->cfg_hac_mode = 1;
	stp_fft_ip_cfg->cfg_output_mode = 1;
	stp_fft_ip_cfg->cfg_int_mask = 0;
	stp_fft_ip_cfg->cfg_shedu_en = 0;
	stp_fft_ip_cfg->cfg_ifft_start = 1;
	stp_fft_ip_cfg->fft_dmac_fb_func = dft0_data_callback;
	stp_fft_ip_cfg->lli_mode = DMAC_LLI_LOOP;//DMAC_LLI_NORMAL;//DMAC_LLI_LOOP;
	stp_fft_ip_cfg->intr = DMAC_CH_BLOCK_TRF_DONE;//DMAC_CH_DMA_TRF_DONE;DMAC_CH_BLOCK_TRF_DONE;
	stp_fft_ip_cfg->fft_rd_lli_item = dft0_link_in_item;
	stp_fft_ip_cfg->fft_wr_lli_item = dft0_link_out_item;
}

/***********************************************************************************************************************
* FUNCTION
*
* VOID algo_ul_dft1200_config_init(VOID)
*
* DESCRIPTION
*
* 1200 poinit DFT after modulation
*
* NOTE
*
*   <the limitations to use this function or other comments>
*
* PARAMETERS
*
*   NULL
*
* RETURNS
*
*   NULL
*
* VERSION
*
*   <DATE>           <AUTHOR>           <CR_ID>              <DESCRIPTION>
*
***********************************************************************************************************************/
OAL_ITCM_CODE_SECTION
VOID algo_ul_dft1200_config_init(void)
{
	algo_ul_dft1200_cfg(TOTAL_MOD_SYM_NUM, &algo_ul_dft_fft0_cfg);
	algo_ul_fft_dma_cfg(&algo_ul_dft_fft0_cfg, (UINT32*)(0x11000000+(UINT32)&mod_output_data[0][0]),(UINT32*)(0x11000000+(UINT32)&mod_output_data[0][0]));
}

/***********************************************************************************************************************
* FUNCTION
*
* VOID algo_ul_tx_cancel_proc(VOID)
*
* DESCRIPTION
*
* tx event cancel handler
*
* NOTE
*
*   <the limitations to use this function or other comments>
*
* PARAMETERS
*
*   NULL
*
* RETURNS
*
*   NULL
*
* VERSION
*
*   <DATE>           <AUTHOR>           <CR_ID>              <DESCRIPTION>
*
***********************************************************************************************************************/
OAL_ITCM_CODE_SECTION
VOID algo_ul_tx_cancel_proc(VOID)
{	
	OAL_ASSERT(g_u8_tx_ctrl_info_cnt_in EQ g_u8_tx_ctrl_info_cnt_out, "");
	g_u8_tx_ctrl_info_cnt_in = (g_u8_tx_ctrl_info_cnt_in + TX_CTRL_INFO_CNT_MAX - 1)%TX_CTRL_INFO_CNT_MAX;
	g_u8_tx_ctrl_info_cnt_out = g_u8_tx_ctrl_info_cnt_in;
	
	OAL_ASSERT(g_u8_algo_ul_data_mod_cnt_in EQ g_u8_algo_ul_data_mod_cnt_out, "");
	g_u8_algo_ul_data_mod_cnt_in = (g_u8_algo_ul_data_mod_cnt_in + ALGO_DATA_MOD_CNT_MAX - 1)%ALGO_DATA_MOD_CNT_MAX;;
	g_u8_algo_ul_data_mod_cnt_out = g_u8_algo_ul_data_mod_cnt_in;	
}

/**************************************************** END OF FILE *****************************************************/
