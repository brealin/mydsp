/***********************************************************************************************************************
* Copyright (C) 2019 Shanghai Kindroid Network Tech Ltd. All rights reserved
*
* License:
*
* Description:
*
* Author:
*
* Revision History:
* 1.0 2019-06-17 created by can.yin
***********************************************************************************************************************/
#undef THIS_FILE_NAME_ID
#define THIS_FILE_NAME_ID 0//DL_PROC_FILE_ID

/***********************************************************************************************************************
* INCLUDE FILES
***********************************************************************************************************************/
#include "ks_typedef.h"
#include "ks_macro.h"
#include "ks_section.h"
#include "ks_oal.h"
#include "ks_turbo_decoder.h"
#include "alg_func.h"
#include "ofdm_rx_proc.h"
#include "l1cc.h"
#include "ks_phych_func.h"
#include "ks_kd001_main.h"

/***********************************************************************************************************************
* MACRO DEFINITION
***********************************************************************************************************************/
/*function switch start*/
#define RX_STUB                                     (0)  /*1: RX DMA NOT TRANSFER RX DATA TO RX BUF*/
#define RTC_ADJ                                     (0)
#define RTC_ADJ_FAST                                (0)
#define FREQ_OFFSET_COARSE                          (1)  
#define ICS_OPTIMIZE                                (0)  /*1: ONLY PROC ONE FRAME EVERY 2 FRAMES WHEN DO ICS*/
#define DEMOD_WITH_SNR                              (0)  /*1: DEMOD WITH SNR*/
#define H_INTERP_REDUCE                             (0)  
#define VECTOR_CMP									(0)
#define H_CALC_DFT   								(0)
#define CCA_FIRST_PATH 								(1)
#define RSSI_BIT_SHIFT_SAME							(1)
#define H_CALC_OPTIMIZE                             (1)
#define FUN_DC_REMOVE                               (0)
/*function switch end*/

#define TX_ANT_SNR_THRES    						(1)
#define RX_SHIFT_VAL                                (2)  
#define MIMO_SNR_THRES								(10)
#define MULTI_PATH_WINDOW_LENGTH                    (16)
#define TA_LEN					                    (4)
#define DEMOD_THRES_QAM16                           (0x01E001E0)//(0x00D800D8)
#define DEMOD_THRES_QAM64                           (0x00F000F0)//(0x00D800D8)
#define DEMOD_THRES_QAM256                          (0x01E001E0)//(0x00D800D8)
#define AFC_ALPHA                                   (40000)//(0x00D800D8)
#define AGC_ALPHA                                   (50000)//(0x00D800D8)
#define ASSERT_4500_ADDR                            (0x1724FFE0)
#define H_CALC_EDGE 								(16)
#define CFO_ANGLE_THRES								(315)
#define CP_DELTA_LEN								(16)
#define CCA_DELTA_LEN								(32)
#define RSSI_TARGET  								(-30)
#define RSSI_THRES_LOW								(-110)
#define AFC_DELTA_THRES								(200)
#define FFT_SHIFT_VAL								(-4) 
#define RX_DUMP_DATA_ERR							(0x1234)
#define RX_DUMP_PLCP_ERR							(0x1235)
#define RX_DUMP_CCA_MIS 							(0x1236)
#define RX_DUMP_RX_ANT_LESS 						(0x1237)
#define RX_DUMP_RX_ANT_PATHLOSS 					(0x1238)


#if RX_STUB
#define RX_STUB_MCS 								(11)
#endif
/***********************************************************************************************************************
* GLOBAL VARIABLES DEFINITION
***********************************************************************************************************************/
#if FREQ_OFFSET_COARSE
typedef struct cfo_seq_tag
{
    complex_s16_t st_cfo_seq[(SLOT_LEN >> 4)];
    complex_s16_t st_slot_phase[TOTAL_SYM_NUM];
    complex_s16_t st_symbol_phase;
    complex_s16_t st_plcp_phase;
    
}cfo_seq_t;
#else
typedef struct cfo_seq_tag
{
    complex_s16_t st_cfo_seq[(SLOT_LEN)];
    complex_s16_t st_slot_phase[TOTAL_SYM_NUM];
    complex_s16_t st_symbol_phase;
    complex_s16_t st_plcp_phase;

}cfo_seq_t;
#endif

OAL_DTCM_DATA_SECTION STATIC OAL_ALIGNED(64) dl_rx_buf_t s_st_rx_buffer = 
{
	#if RX_STUB
	//#include "./rx_buf_mcs12_kd001_st_2t2r_3.dat"
	//#include "./rx_buf_mcs2_kd001_st.dat"
	//#include "./rx_buf_mcs11_kd001_st_2t1r_3.dat"
	//#include "./mcs13_crc_err2.dat"
	//#include "./rx_buf_mcs11_kd001_st_2t2r_10db_3.dat"
	//#include "./rx_buf_mcs11_kd001_st_2t1r.dat"
	//#include "./rx_buf_mcs11_kd001_st_2t1r.dat"
	//#include "./rx_buf_mcs11_kd001_st_1t1r.dat"
	//#include "./rx_buf_mcs11_kd001_st_1t1r.dat"
	//#include "./rx_buf_mcs11_kd001_st_2t2r.dat"
	//#include "./rx_buf_mcs0_kd001_st_1t1r_test.dat"
	//#include "./rx_buf_mcs11_kd001_st_mcs11_1t1r_test.dat"
	//#include "./rx_buf_mcs13_kd001_st_2t2r_awgn_True(12.7db)_06.dat" 
	//#include "./rx_buf_mcs13_kd001_st_2t2r_18.5db_f_full_5.dat" 
	//#include "./rx_buf_mcs11_kd001_st_2t2r_9.8db_3.dat" 
	#include "./rx_buf_mcs0_mimo_st_2t2r_5.dat"  
	#else
	0
	#endif
}; 
#if RX_STUB
OAL_DRAM_DATA_SECTION STATIC OAL_ALIGNED(16) complex_s16_t s_st_rx_buffer_ddr[2][RX_BUF_LEN] = 
{    
	//#include "./rx_buf2_nofoff.dat"
	//#include "./rx_buf.dat"
	0
}; 
#endif

OAL_DTCM_DATA_SECTION STATIC complex_s16_t s_st_pilot_iq[SYMBOL_LEN];

OAL_DTCM_DATA_BLOCK1_SECTION STATIC complex_s16_t s_st_eq_out[(DATA_SYM_NUM + 1) * SC_NUM];

OAL_DTCM_DATA_SECTION STATIC SINT32 s_s32_shift_amt_fft[5]  = {(UINT32)MODV_SV2, (UINT32)MODV_SV1, (UINT32)MODV_SV1, (UINT32)MODV_SV1, (UINT32)MODV_SV0};
OAL_DTCM_DATA_SECTION STATIC SINT32 s_s32_shift_amt_ifft[5] = {(UINT32)MODV_SV2, (UINT32)MODV_SV1, (UINT32)MODV_SV1, (UINT32)MODV_SV1, (UINT32)MODV_SV1};

OAL_DTCM_DATA_SECTION STATIC complex_s16_t s_st_fft_in[SYMBOL_LEN];
OAL_DTCM_DATA_SECTION STATIC OAL_ALIGNED(64) complex_s16_t s_st_fft_out[SYMBOL_LEN + 8];
OAL_DTCM_DATA_BLOCK1_SECTION STATIC complex_s16_t s_st_temp_buffer[2][SC_NUM << 2];

/*
  s_st_H_vec:
  1st symbol: h11 h12 h21 h22
  2nd symbol: h11 h12 h21 h22
  3rd symbol: h11 h12 h21 h22
  4th symbol: h11 h12 h21 h22
*/
OAL_DTCM_DATA_SECTION OAL_ALIGNED(16) STATIC complex_s16_t s_st_H_vec[4][MAX_H_NUM][SC_NUM];
OAL_DTCM_DATA_SECTION OAL_ALIGNED(16) STATIC complex_s16_t s_st_H_sub_tbl[MAX_H_NUM][SC_NUM];
OAL_DTCM_DATA_SECTION STATIC cfo_seq_t s_st_cfo_seq = {0};
OAL_DTCM_DATA_SECTION STATIC complex_s16_t s_st_fd_cca_seq[SYMBOL_LEN] = 
{
    #include "./fd_cca_seq2.dat"
};

#if 0
OAL_DTCM_DATA_SECTION STATIC complex_s16_t s_st_fd_pilot_seq[HALF_SC_NUM] = 
{
    #include "./fd_pilot_seq.dat"
};
OAL_DTCM_DATA_SECTION STATIC OAL_ALIGNED(4) complex_s16_t s_st_exp_seq[1024] = 
{
    #include "./exp_seq.dat"
};

#endif

OAL_DTCM_DATA_SECTION STATIC complex_s16_t s_st_fd_pilot_seq[HALF_SC_NUM] = 
{
	#include "./ofdm_pilot_fre1.txt"
};

OAL_DTCM_DATA_SECTION STATIC OAL_ALIGNED(4) complex_s16_t s_st_exp_seq[1024] = 
{
    #include "./exp_seq.dat"
};

#if 0
OAL_DTCM_DATA_SECTION STATIC complex_s16_t s_st_fd_pilot_seq2[HALF_SC_NUM] = 
{
	#include "./ofdm_pilot_fre2.txt"
};

OAL_DTCM_DATA_SECTION STATIC OAL_ALIGNED(4) complex_s16_t s_st_window_seq[1024] = 
{
    #include "./ofdm_td_win.txt"
};
#endif

OAL_DTCM_DATA_SECTION STATIC UINT32 s_u32_scramble_table[] =
{
    #include "./scrambler.dat"
};

OAL_DTCM_DATA_SECTION STATIC complex_s16_t s_st_data_iq[MAX_RX_ANT_NUM][SC_NUM * (PILOT_DIS_SYM)] = {0};


OAL_DTCM_DATA_SECTION STATIC plcp_info_t s_st_plcp_info;
OAL_DEBUG_DATA_SECTION STATIC dl_debug_info_t s_st_debug_info = {0};
OAL_DTCM_DATA_SECTION STATIC SINT32 s_s32_comb_squa[SYMBOL_LEN] = {0};
//OAL_DTCM_DATA_SECTION OAL_ALIGNED(16) STATIC SINT8 s_s8_llr[MAX_DATA_SYM_PER_SLOT][SC_NUM * 6 + 384];
OAL_DTCM_DATA_BLOCK2_SECTION OAL_ALIGNED(64) STATIC SINT8 s_s8_llr[TURBO_DEC_LLR_BUFFER_SIZE + 64];
OAL_DTCM_DATA_SECTION OAL_ALIGNED(16) STATIC turbo_decode_info_link_t s_st_turbo_dec_info_link;
TURBO_DECODER_DATA_SECTION OAL_ALIGNED(16) STATIC TURBO_DECODER_OUTPUT s_st_turbo_dec_output;
OAL_DTCM_DATA_SECTION OAL_ALIGNED(64) STATIC TURBO_DECODER_PARA s_st_turbo_dec_para;
OAL_DTCM_DATA_SECTION STATIC complex_s16_t s_st_H_mod[SC_NUM * MAX_H_NUM];

OAL_DTCM_DATA_SECTION STATIC SINT16 s_s16_cordic_table[15] = {8192, 4836, 2555, 1297, 651, 326, 163, 81, 41, 20, 10, 5, 3, 1, 1};//sum =18182
DTCM_TCE_DMAD_DATA_SECTION STATIC DMAC_LLI_ITEM s_st_dmac_lli_item[2][RXDFE_PACK_AMT];
#ifdef MIMO2
OAL_DTCM_DATA_SECTION STATIC SINT16 s_dl_h_intp_scale[5] = {8192, 16384, 24576, 21845, 27306};
#else
OAL_DTCM_DATA_SECTION STATIC SINT16 s_dl_h_intp_scale[5] = {5461, 10922, 16384, 21845, 27306};
#endif
OAL_DTCM_DATA_SECTION STATIC SINT32 s_s32_noise_pwr[3][MAX_H_NUM] = {0};
OAL_DTCM_DATA_SECTION STATIC SINT32 s_s32_sig_pwr[3][MAX_H_NUM] = {0};
OAL_DTCM_DATA_SECTION STATIC SINT32 s_s32_snr[3][MAX_H_NUM] = {0};
OAL_DTCM_DATA_SECTION STATIC SINT32 s_s32_snr_rpt = 0;
OAL_DTCM_DATA_SECTION STATIC algo2l1cc_ctrl_info_ind_t s_st_algo2l1cc_ctrl_info = {0};
//OAL_DTCM_DATA_BLOCK2_SECTION OAL_ALIGNED(16) STATIC SINT8 s_s8_decode_out[0x2800] = {0};



OAL_DTCM_DATA_SECTION STATIC UINT32 s_u32_data_buffer_wr_addr = L1CC_BB_SCFDE_DEC_DATA_BUF_START_ADDR;

OAL_DTCM_DATA_SECTION UINT32 g_u32_cca_detected = 0;
OAL_DTCM_DATA_SECTION UINT32 g_u32_rx_ant_num = 0;
OAL_DTCM_DATA_SECTION UINT32 g_u32_rx_ant_1st_id = 0;
OAL_DTCM_DATA_SECTION UINT32 g_u32_tx_ant_num = 0;
OAL_DTCM_DATA_SECTION UINT32 g_u32_tx_ant_1st_id = 0;
OAL_DTCM_DATA_SECTION UINT32 g_u32_dl_status = 0;
OAL_DTCM_DATA_SECTION UINT32 g_u32_ics_flag = 0;
OAL_DTCM_DATA_SECTION UINT32 g_u32_ics_frm_cnt = 0;
OAL_DTCM_DATA_SECTION UINT32 g_u32_ics_sfrm_cnt = 0;
OAL_DTCM_DATA_SECTION UINT32 g_u32_ics_frm_cnt_cca = 0;
OAL_DTCM_DATA_SECTION UINT32 g_u32_ics_frm_cnt_rpt = 0;
OAL_DTCM_DATA_SECTION UINT32 g_u32_ics_fft_cnt = 0;
OAL_DTCM_DATA_SECTION UINT32 g_u32_ics_cca_cnt = 0;
OAL_DTCM_DATA_SECTION UINT32 g_u32_ics_left_cnt = 0;
OAL_DTCM_DATA_SECTION UINT32 g_u32_ics_left_sym = 0;
OAL_DTCM_DATA_SECTION VUINT32 g_u32_ics_head_crc = 0;
OAL_DTCM_DATA_SECTION UINT32 g_u32_ics_data_crc = 0;
OAL_DTCM_DATA_SECTION UINT32 g_u32_ics_sym_bmp = 0; //symbol processed bitmap(eq or channel est finish); bit 0\6\12: pilot symbols bitmap; bit 1 ~ 5, 7 ~ 11: plcp\data symbols bitmap
OAL_DTCM_DATA_SECTION UINT32 g_u32_ics_crc_bmp = 0; //cb crc bitmap: bit0->cb0 to max cb bit
OAL_DTCM_DATA_SECTION UINT32 g_u32_ics_cb_cnt = 0; //cb crc bitmap: bit0->cb0 to max cb bit
OAL_DTCM_DATA_SECTION UINT32 g_u8_node_id = 0; //cb crc bitmap: bit0->cb0 to max cb bit

OAL_DTCM_DATA_SECTION UINT32 s_u32_data_sym_pre_cnt = 0; //symbols pre eq done cnt. unit : symbol
OAL_DTCM_DATA_SECTION UINT32 s_u32_data_sym_eq_cnt = 0; //symbols eq done cnt. unit : symbol
OAL_DTCM_DATA_SECTION UINT32 s_u32_data_left_cnt = 0;   //unit : llr sample 

OAL_DTCM_DATA_SECTION UINT32 s_u32_data_cb_start_cnt = 0; 
OAL_DTCM_DATA_SECTION UINT32 s_u32_data_cb_end_cnt = 0;


OAL_DTCM_DATA_SECTION STATIC SINT16 s_s16_shift_value[MAX_RX_ANT_NUM] = {0};
OAL_DTCM_DATA_SECTION STATIC SINT16 s_s16_shift_value_before[MAX_RX_ANT_NUM] = {0};
OAL_DTCM_DATA_SECTION STATIC complex_s16_t s_st_dc[MAX_RX_ANT_NUM] = {0};

OAL_DTCM_DATA_SECTION STATIC VOLATILE_SINT32 s_s32_sync_position[MAX_RX_ANT_NUM] = {OAL_FAILURE};
OAL_DTCM_DATA_SECTION STATIC VOLATILE_SINT32 s_s32_sync_position_prev = OAL_FAILURE;
OAL_DTCM_DATA_SECTION STATIC VOLATILE_SINT32 s_s32_sync_pos_min = OAL_FAILURE;
OAL_DTCM_DATA_SECTION STATIC VOLATILE_SINT32 s_s32_sync_pos_off[MAX_RX_ANT_NUM] = {0};


OAL_DTCM_DATA_SECTION STATIC UINT32 s_u32_pingpong_idx = 0;
OAL_DTCM_DATA_SECTION volatile STATIC UINT32 s_u32_dma_cnt_in = 0;
OAL_DTCM_DATA_SECTION volatile STATIC UINT32 s_u32_dma_cnt_out = 0;
OAL_DTCM_DATA_SECTION STATIC UINT32 s_u32_rtc_adj_flg = OAL_TRUE;
OAL_DTCM_DATA_SECTION STATIC UINT32 s_u32_rtc_adj_flg_fast = OAL_FALSE;
OAL_DTCM_DATA_SECTION STATIC UINT32 s_u32_rtc_adj_wait_cnt = 0;

OAL_DTCM_DATA_SECTION STATIC VOLATILE_UINT32 s_u32_rxdfe_pre_status = 0;
OAL_DTCM_DATA_SECTION STATIC VOLATILE_UINT32 s_u32_rxdfe_cur_status = 0;
OAL_DTCM_DATA_SECTION STATIC VOLATILE_UINT32 s_u32_rxdfe_restart = OAL_FALSE;

OAL_DTCM_DATA_SECTION STATIC UINT8 s_u8_zzw_state = 0;
OAL_DTCM_DATA_SECTION STATIC UINT8 s_u8_zzw_state_pre = 0;
OAL_DTCM_DATA_SECTION STATIC UINT8 s_u8_rx_gain = 0;
OAL_DTCM_DATA_SECTION STATIC SINT8 s_u8_node_id_local = 0;

OAL_DTCM_DATA_SECTION STATIC SINT32 s_s32_frame_num = 0;
OAL_DTCM_DATA_SECTION STATIC SINT32 s_s32_frame_num_cca = 0;
OAL_DTCM_DATA_SECTION STATIC SINT32 s_s32_frame_num_rpt = 0;



OAL_DTCM_DATA_SECTION STATIC VOLATILE_SINT32 g_s32_rf_delay_compensate = 12;
OAL_DTCM_DATA_SECTION VOLATILE_UINT32 s_s32_compete_timing_compensate = 0;
OAL_DTCM_DATA_SECTION ZZW_CHANNEL_STATUS_INDICATION_T g_zzw_ch_status = {0}; 

OAL_DTCM_DATA_SECTION STATIC UINT32 s_u32_dec_cnt = 0;
OAL_DTCM_DATA_SECTION UINT8 g_u8_rpt_type = 0;
OAL_DTCM_DATA_SECTION UINT8 g_u8_rx_gain_expect = 0;
OAL_DTCM_DATA_SECTION SINT8 g_s8_tx_pwr_expect = 0;
OAL_DTCM_DATA_SECTION UINT8 g_u8_mimo_type = RTX_2T2R;

OAL_DTCM_DATA_SECTION STATIC SINT32 s_s32_angle_pilot[MAX_RX_ANT_NUM][3] = {0};
OAL_DTCM_DATA_SECTION STATIC SINT32 s_s32_angle_cp[MAX_RX_ANT_NUM] = {0};
OAL_DTCM_DATA_SECTION STATIC SINT32 s_s32_angle_cp_avg = 0;

OAL_DTCM_DATA_SECTION STATIC SINT32 s_s32_angle_cca[MAX_RX_ANT_NUM] = {0};
OAL_DTCM_DATA_SECTION STATIC SINT32 s_s32_angle_cca_avg = 0;
OAL_DTCM_DATA_SECTION STATIC SINT32 s_s32_angle_avg = 0;

OAL_DTCM_DATA_SECTION STATIC SINT32 s_s32_snr_mean = 0;
OAL_DTCM_DATA_SECTION STATIC UINT32 s_u32_freq_ind = 0;

OAL_DTCM_DATA_SECTION STATIC UINT32 s_u32_turbo_in_len[ZZW_MAX_MCS_NUM] = {0};
OAL_DTCM_DATA_SECTION STATIC UINT32 s_u32_mod_len[ZZW_MAX_MCS_NUM] = {0};
OAL_DTCM_DATA_SECTION STATIC UINT32 s_u32_llr_shift[ZZW_MAX_MCS_NUM] = {0};
OAL_DTCM_DATA_SECTION STATIC UINT32 s_u32_llr_shift_plcp = 0;
OAL_DTCM_DATA_SECTION STATIC UINT32 s_u32_turbo_in_len_plcp = 0;
OAL_DTCM_DATA_SECTION STATIC complex_s16_t *s_stp_iq_ptr[MAX_RX_ANT_NUM];

#if DEMOD_WITH_SNR
OAL_DTCM_DATA_BLOCK1_SECTION UINT32 g_u32_snr[SC_NUM] = {0};
OAL_DTCM_DATA_BLOCK2_SECTION UINT16 g_u16_snr[10 * SC_NUM] = {0};
#endif
OAL_DTCM_DATA_SECTION STATIC SINT32 s_s32_cca_power_thres[16] = {3000, 3000, 3000, 3000, 3000, 3000, 2800, 2600, 2500, 2300, 1900, 1800, 1700, 1500, 1500, 1500};
//OAL_DTCM_DATA_SECTION STATIC SINT32 s_s32_cca_power_thres[16] = {750, 750, 750, 700, 600, 500, 500, 450, 400, 400, 400, 400, 400, 1500, 1500, 1500};
OAL_DTCM_DATA_SECTION STATIC SINT16 s_s16_shift_value_min = 0;
OAL_DTCM_DATA_SECTION STATIC SINT16 s_s16_shift_value_max = 0;

OAL_DTCM_DATA_SECTION STATIC SINT32 s_u32_tmr_start = 0;
OAL_DTCM_DATA_SECTION STATIC SINT32 s_u32_tmr_end = 0;
OAL_DTCM_DATA_SECTION STATIC SINT32 s_u32_dl_scn_cnt = 0;
OAL_DTCM_DATA_SECTION STATIC SINT32 s_u32_dl_sfb_cnt = 0;
OAL_DTCM_DATA_SECTION STATIC SINT32 s_u32_ics_optimize = OAL_TRUE;
OAL_DTCM_DATA_SECTION STATIC SINT32 s_s32_peak_min[MAX_RX_ANT_NUM] = {0};


OAL_DTCM_DATA_SECTION UINT32 g_u32_qam16_boundary = DEMOD_THRES_QAM16;
OAL_DTCM_DATA_SECTION UINT32 g_u32_qam64_boundary = DEMOD_THRES_QAM64;
OAL_DTCM_DATA_SECTION UINT32 g_u32_qam256_boundary = DEMOD_THRES_QAM256;

//OAL_DTCM_DATA_SECTION STATIC node_info_tbl_t s_st_node_info_tbl = {0};

extern node_info_tbl_t s_st_node_info_tbl;
#if VECTOR_CMP
OAL_DTCM_DATA_SECTION UINT32 g_test_flg = 1;
#endif


extern UINT32 g_u32_build_time;
extern UINT32 g_u32_build_date;


#define DL_GET_TMR2_START  						do{s_u32_tmr_start = ks_timer_get_counter(KS_CP0_TIMER2);}while(0)
#define DL_GET_TMR2_END    						do{s_u32_tmr_end = ks_timer_get_counter(KS_CP0_TIMER2);}while(0)
#define DL_PRINT_TMR2_DURATION(LOG_ID)			do{KS_LOG(0, LOG_ID, (UINT16)(s_u32_tmr_start - s_u32_tmr_end));}while(0)

void ks_dft1200(complex_s16_t * in_buffer, complex_s16_t * out_buffer, complex_s16_t * tmp_buffer, SINT32* modv0);
void ks_idft1200(complex_s16_t * in_buffer, complex_s16_t * out_buffer, complex_s16_t * tmp_buffer, SINT32* modv0);

/***********************************************************************************************************************
* FUNCTION
*
* dl_scramble
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


/***********************************************************************************************************************
* FUNCTION
*
* dl_scramble
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
VOID dl_scramble(IN UINT16 tb_size, IN VOLATILE_UINT32_PTR input_buf, OUT VOLATILE_UINT32_PTR output_buf)
{
    UINT32 u32_index;
#if 1
    for(u32_index = 0; u32_index < tb_size; u32_index++)
    #pragma dsp_ceva_trip_count_min = 1
    {
        *(output_buf + u32_index) = *(input_buf + u32_index)^s_u32_scramble_table[u32_index];
    }
#else

    vec_t v32_1;
    vec_t v32_2;
    vec_t v32_3;

    for(u32_index = 0; u32_index < tb_size; )
    #pragma dsp_ceva_trip_count_min = 1
    #pragma dsp_ceva_unroll = 2
    {
        v32_1 = vlddw_v32(8, (SINT32_PTR)&input_buf[u32_index], 0);
        v32_2 = vlddw_v32(8, (SINT32_PTR)&s_u32_scramble_table[u32_index], 0);
        v32_3 = vxor_v32_v32_v32 (8, v32_1, v32_2);

        vstdw_v32 (8, v32_3, 0, (SINT32_PTR)&output_buf[u32_index]);
 
        u32_index += 16;
    }

#endif
}

OAL_ITCM_CODE_SECTION
VOID dl_scramble_vc(IN UINT16 tb_size, IN VOLATILE_UINT32_PTR input_buf, OUT VOLATILE_UINT32_PTR output_buf)
{
    UINT32 u32_index;
#if 0
    for(u32_index = 0; u32_index < tb_size; u32_index++)
    #pragma dsp_ceva_trip_count_min = 1
    {
        *(output_buf + u32_index) = *(input_buf + u32_index)^s_u32_scramble_table[u32_index];
    }
#else

    vec_t v32_1;
    vec_t v32_2;
    vec_t v32_3;

    for(u32_index = 0; u32_index < tb_size; )
    #pragma dsp_ceva_trip_count_min = 1
    #pragma dsp_ceva_unroll = 2
    {
        v32_1 = vlddw_v32(8, (SINT32_PTR)&input_buf[u32_index], 0);
        v32_2 = vlddw_v32(8, (SINT32_PTR)&s_u32_scramble_table[u32_index], 0);
        v32_3 = vxor_v32_v32_v32 (8, v32_1, v32_2);

        vstdw_v32 (8, v32_3, 0, (SINT32_PTR)&output_buf[u32_index]);
 
        u32_index += 16;
    }
 
#endif
}

/***********************************************************************************************************************
* FUNCTION
*
*
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
VOID_PTR dl_turbo_input_addr_get(UINT32 u32_input_length)
{
    UINT32 u32_rd_index, u32_rd_offset;
    UINT32 u32_wr_index, u32_wr_offset;
    UINT32 u32_free_size;
    UINT32 u32_input_len;
    UINT32 u32_i;

    OAL_IRQ_SAVE_AREA;

    u32_input_len = (u32_input_length + 0xF) & 0xFFFFFFF0; /*aligned 128 bit*/

    OAL_IRQ_DISABLE;
    u32_rd_index = s_st_turbo_dec_info_link.u32_read_index;
    u32_wr_index = s_st_turbo_dec_info_link.u32_write_index;
    OAL_IRQ_RESTORE;

    u32_free_size = (u32_rd_index + TURBO_DEC_INFO_AMT - 1 - u32_wr_index) % TURBO_DEC_INFO_AMT;

	OAL_ASSERT(u32_free_size >= 1, "turbo decode start exception");

	//KS_LOG(OAL_TRACE_LEVEL_0 , 0xA500 , (UINT16)(((u32_free_size&0xF)<<8) | ((u32_wr_index&0xF)<<4) | (u32_rd_index&0xF)));

    u32_rd_offset = s_st_turbo_dec_info_link.st_turbo_dec_info[u32_rd_index].u32_write_offset;
    u32_wr_offset = s_st_turbo_dec_info_link.st_turbo_dec_info[u32_wr_index].u32_write_offset;

    u32_free_size = (u32_rd_offset + TURBO_DEC_LLR_BUFFER_SIZE - 1 - u32_wr_offset) % TURBO_DEC_LLR_BUFFER_SIZE;
    OAL_ASSERT(u32_free_size >= u32_input_len, "turbo decode start exception");
	//KS_LOG(OAL_TRACE_LEVEL_0 , 0xA501 , (UINT16)u32_free_size);
	//KS_LOG(OAL_TRACE_LEVEL_0 , 0xA511 , (UINT16)u32_wr_offset);
	//KS_LOG(OAL_TRACE_LEVEL_0 , 0xA512 , (UINT16)u32_input_len);

    if((u32_wr_offset + u32_input_len) > (UINT32)TURBO_DEC_LLR_BUFFER_SIZE)
    {
        u32_wr_offset = 0;

        if((u32_wr_offset + u32_input_len) >= u32_rd_offset) 
        {
            OAL_ASSERT(0, "turbo decode start exception");
        }
    }

    s_st_turbo_dec_info_link.st_turbo_dec_info[u32_wr_index].u32_input_lenght  = u32_input_length;
    s_st_turbo_dec_info_link.st_turbo_dec_info[u32_wr_index].u32_write_offset  = u32_wr_offset; /*save the start address*/
	KS_LOG(OAL_TRACE_LEVEL_0 , 0xA502 , (UINT16)u32_wr_offset);

	return (VOID_PTR)&s_s8_llr[u32_wr_offset];
}


OAL_ITCM_CODE_SECTION
VOID dl_rxdfe_dmac_callback()
{
    oal_msg_t *stp_msg;
	sram_1643_4500_t *stp_sarm_1643_4500 = (sram_1643_4500_t*)(SRAM_1643_4500_ADDR);	//g_st_iq_dump

	KS_LOG_TMR(0xA000);
    s_u32_rxdfe_cur_status = stp_sarm_1643_4500->u32_rx_restart;

    if((s_u32_rxdfe_pre_status + 1) == s_u32_rxdfe_cur_status)
    {
        s_u32_rxdfe_restart = OAL_TRUE;
        s_u32_rxdfe_pre_status = s_u32_rxdfe_cur_status;
        
		s_s32_frame_num = stp_sarm_1643_4500->s16_rx_fn;
		s_u8_node_id_local = stp_sarm_1643_4500->u8_node_id_local;
		KS_LOG(0, 0xA111, (UINT16)s_s32_frame_num);
    }
    else if(s_u32_rxdfe_pre_status != s_u32_rxdfe_cur_status)
    {
        OAL_ASSERT(0, "rx ctrl exception");
    }

	s_u8_rx_gain = stp_sarm_1643_4500->u8_rx_gain;
	s_u8_zzw_state_pre = s_u8_zzw_state;
	s_u8_zzw_state = stp_sarm_1643_4500->u8_rx_state;
	#if 1
	stp_msg = ks_oal_msg_create(0>>1);
	OAL_ASSERT(NULL_PTR != stp_msg, "ks_oal_msg_create failure\n");
	stp_msg->msg_primitive_id = MSG_DL_RX_PRIMITIVE_ID;
	stp_msg->u16_mode_type	  = 0; //FLIGHT_MODE;
	stp_msg->u16_standby_id   = 0; //STANDBY_0;
	ks_oal_msg_send(stp_msg, OAL_SYS_TASK_ID + 5);
	#endif
	//KS_LOG(0, 0xAF00, s_u32_dma_cnt_in);
	s_u32_dma_cnt_in++;
	
	//OAL_ASSERT(s_u32_dma_cnt_in - s_u32_dma_cnt_out < 2, "");
	
	if(s_u32_rtc_adj_wait_cnt)
	{
		s_u32_rtc_adj_wait_cnt--;
	}
}


OAL_ITCM_CODE_SECTION
VOID dl_rxdfe_dmac_callback2()
{
	KS_LOG_TMRL(0xA002);
}


/***********************************************************************************************************************
* FUNCTION
*
*
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
VOID dl_rxdfe_dmac_config(UINT16 block_len, UINT16 iter_num, UINT32 *s16_iq_rxdfe_out_add, UINT8 u8_ch_idx)
{
    DMAC_LINK_INFO link_item;
    UINT32 u32_lli_idx;
    UINT32 u32_idx;
    UINT32 u32_src[2] = {0x13000000, 0x13002000};

    for(u32_lli_idx = 0; u32_lli_idx < iter_num; u32_lli_idx++)
    {
        u32_idx = u32_lli_idx & (0x1);
        s_st_dmac_lli_item[u8_ch_idx][u32_lli_idx].src         = u32_src[u32_idx] + 0x200000 * u8_ch_idx;
        #if RX_STUB
        s_st_dmac_lli_item[u8_ch_idx][u32_lli_idx].dst         = (UINT32)(s16_iq_rxdfe_out_add + block_len*u32_lli_idx);//dst[LLI_idx];
        #else
        s_st_dmac_lli_item[u8_ch_idx][u32_lli_idx].dst         = (UINT32)(s16_iq_rxdfe_out_add + block_len*u32_lli_idx) + 0x11200000;//dst[LLI_idx];
		#endif
        s_st_dmac_lli_item[u8_ch_idx][u32_lli_idx].block_ts    = BLOCK_TS(block_len*4, DMAC_TR_WIDTH_128);// block_size = block_len*4;
        s_st_dmac_lli_item[u8_ch_idx][u32_lli_idx].src_width   = DMAC_TR_WIDTH_128;
        s_st_dmac_lli_item[u8_ch_idx][u32_lli_idx].dst_width   = DMAC_TR_WIDTH_128;
        s_st_dmac_lli_item[u8_ch_idx][u32_lli_idx].llp         = (UINT32)&s_st_dmac_lli_item[u8_ch_idx][u32_lli_idx + 1];
        s_st_dmac_lli_item[u8_ch_idx][u32_lli_idx].src_sts     = DMAC_ADDR_INCREMENT;
        s_st_dmac_lli_item[u8_ch_idx][u32_lli_idx].dst_sts     = DMAC_ADDR_INCREMENT;
        s_st_dmac_lli_item[u8_ch_idx][u32_lli_idx].lli_last    = DMAC_LLI_NOT_LAST;
        s_st_dmac_lli_item[u8_ch_idx][u32_lli_idx].lli_valid   = DMAC_LLI_VALID;
		s_st_dmac_lli_item[u8_ch_idx][u32_lli_idx].ioc_blk	   = DMAC_IOC_BLKTRF_DISABLE;
        s_st_dmac_lli_item[u8_ch_idx][u32_lli_idx].RESEVED_src = 0x0;
        s_st_dmac_lli_item[u8_ch_idx][u32_lli_idx].RESEVED_dst = 0x0;
        s_st_dmac_lli_item[u8_ch_idx][u32_lli_idx].RESEVED     = 0x0;
        s_st_dmac_lli_item[u8_ch_idx][u32_lli_idx].RESEVED_llp = 0x0;
        s_st_dmac_lli_item[u8_ch_idx][u32_lli_idx].reseved1    = 0x0;
        s_st_dmac_lli_item[u8_ch_idx][u32_lli_idx].reseved2    = 0x0;
        s_st_dmac_lli_item[u8_ch_idx][u32_lli_idx].reseved3    = 0x0;
        s_st_dmac_lli_item[u8_ch_idx][u32_lli_idx].src_msize   = DMAC_MSIZE(s_st_dmac_lli_item[u8_ch_idx][u32_lli_idx].block_ts); // DMAC_MSIZE_1024;
        s_st_dmac_lli_item[u8_ch_idx][u32_lli_idx].dst_msize   = DMAC_MSIZE(s_st_dmac_lli_item[u8_ch_idx][u32_lli_idx].block_ts); // DMAC_MSIZE_1024;
    }

	s_st_dmac_lli_item[u8_ch_idx][(iter_num - 1)].ioc_blk	   = DMAC_IOC_BLKTRF_ENABLE;
	s_st_dmac_lli_item[u8_ch_idx][((iter_num >> 1) - 1)].ioc_blk	   = DMAC_IOC_BLKTRF_ENABLE;

    s_st_dmac_lli_item[u8_ch_idx][u32_lli_idx - 1].llp = (UINT32)&s_st_dmac_lli_item[u8_ch_idx][0];

    link_item.ch_id        = dmac_ch_request(KS_DMAC0_CP);
    link_item.prior        = DMAC_CH_PRIO7;
    link_item.lli_mode     = DMAC_LLI_LOOP;
    link_item.lli          = &s_st_dmac_lli_item[u8_ch_idx][0];
    link_item.lli_count    = iter_num;
    link_item.trf_type     = DMAC_TRF_PERI2MEM;
    link_item.src_trf_mode = DMAC_TRF_LINK;
    link_item.dst_trf_mode = DMAC_TRF_LINK;
    link_item.hs_type      = DMAC_HS_HARDWARE;
    link_item.dst_hs_ch    = DMAC_CP0_CP1_DEFAULT_VAL;
#if !defined(CP1_XC4500)
    link_item.src_hs_ch    = DMAC_CP0_RXDFE0_PATH0_RAM_REQ + 2*u8_ch_idx;
#else
    link_item.src_hs_ch    = DMAC_CP1_RXDFE0_PATH0_RAM_REQ + 2*u8_ch_idx;
#endif
    link_item.intr         = DMAC_CH_BLOCK_TRF_DONE;

    OAL_ASSERT(link_item.ch_id EQ (DMA_CHAN_FOR_RX + u8_ch_idx), "dma channel req failure");

    ks_dmac_link_config(KS_DMAC0_CP, &link_item);
    #ifdef DVB
    if(u8_ch_idx EQ 1)
    #else
    if(u8_ch_idx EQ 0)
    #endif
    {
    	ks_dmac_register_handle(KS_DMAC0_CP, link_item.ch_id, dl_rxdfe_dmac_callback, NULL);
    }
    else
    {
     	//ks_dmac_register_handle(KS_DMAC0_CP, link_item.ch_id, NULL_PTR, NULL);   
		ks_dmac_register_handle(KS_DMAC0_CP, link_item.ch_id, dl_rxdfe_dmac_callback2, NULL);
    }
    ks_dmac_ch_start(KS_DMAC0_CP, link_item.ch_id);
}

/***********************************************************************************************************************
* FUNCTION
*
*
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
VOID dl_turbo_dec_callback()
{
    UINT32 u32_write_index;
    UINT32 u32_read_index;
    UINT32 u32_write_offset;
	static UINT16 data_send_cnt_bak = 0;
	uint16 cnt_delta = 0;
	node_info_t st_node_info_tmp = {0};
	
	node_info_t *stp_node_info = NULL_PTR;
	

    u32_write_index = s_st_turbo_dec_info_link.u32_write_index;
    u32_read_index  = s_st_turbo_dec_info_link.u32_read_index;

	KS_LOG(0, 0xAF03, ((u32_write_index << 8) | u32_read_index));

    if(u32_write_index EQ u32_read_index)
    {	
    	//temp here, abnormal turbo int 
    	//OAL_ASSERT(0, "");
    	return;
   	}

	KS_LOG_TMRL(0xA001);
	//KS_LOG(0, 0xAF01, (UINT16)(s_st_turbo_dec_para.lte_param.turbo_decoder_lte_k.u32_lte_k));

    ks_oal_mem_copy_ext((VOID_PTR)(&s_st_turbo_dec_output.st_monitor_info), (VOID_PTR)TURBO_DECODER_QBLKID_ADDR, (CONST_UINT16)LTE_MONITOR_INFO_LEN, BITWIDTH_128, OAL_TRUE);
    ks_oal_mem_copy_ext((VOID_PTR)(s_st_turbo_dec_output.u32_hd_results), (VOID_PTR)TURBO_DECODER_HD_OUTPUT_ADDR, (CONST_UINT16)((((s_st_turbo_dec_para.lte_param.turbo_decoder_lte_k.u32_lte_k + 128) >> 7) << 4)), BITWIDTH_128, OAL_TRUE);
    //ks_oal_mem_copy_ext((VOID_PTR)(&s_st_turbo_dec_output.st_monitor_info), (VOID_PTR)TURBO_DECODER_QBLKID_ADDR, (CONST_UINT16)LTE_MONITOR_INFO_LEN, BITWIDTH_128, OAL_TRUE);
    //ks_oal_mem_copy_ext((VOID_PTR)(s_st_turbo_dec_output.u32_hd_results), (VOID_PTR)TURBO_DECODER_HD_OUTPUT_ADDR, (CONST_UINT16)((((s_st_turbo_dec_para.lte_param.turbo_decoder_lte_k.u32_lte_k + 128) >> 7) << 4)), BITWIDTH_128, OAL_TRUE);

	KS_LOG(0, 0xAF01, (UINT16)((s_st_turbo_dec_output.st_monitor_info.crc_error << 8) | (s_st_turbo_dec_info_link.st_turbo_dec_info[u32_read_index].u32_data_type)));

    /*plcp decode*/
    if(PLCP_TYPE == s_st_turbo_dec_info_link.st_turbo_dec_info[u32_read_index].u32_data_type)
    {
	    s_u32_data_buffer_wr_addr = L1CC_BB_SCFDE_DEC_DATA_BUF_START_ADDR;
        OAL_ASSERT(OAL_FALSE == s_st_plcp_info.u16_is_valid, "turbo dec callback exception");
        #if 0//RX_STUB
		s_st_turbo_dec_output.st_monitor_info.crc_error = OAL_SUCCESS;
        #endif
		g_u32_ics_head_crc = s_st_turbo_dec_output.st_monitor_info.crc_error;
        if(OAL_SUCCESS == s_st_turbo_dec_output.st_monitor_info.crc_error)
        {
			//g_u32_ics_flag = ICS_DATA_COLLECT;
			dl_scramble((((ZZW_PLCP_BIT_LEN - 24 + 31) >> 5)), &s_st_turbo_dec_output.u32_hd_results[0], (UINT32_PTR)(&s_st_algo2l1cc_ctrl_info.st_ctrl_info));	
			#if RX_STUB
			s_st_algo2l1cc_ctrl_info.st_ctrl_info.u8_magic_word = ZZW_PLCP_MAGIC_WORD_NORMAL;
			#endif

			if(s_st_algo2l1cc_ctrl_info.st_ctrl_info.u8_magic_word NEQ ZZW_PLCP_MAGIC_WORD_NORMAL) 
			{
				if(s_st_algo2l1cc_ctrl_info.st_ctrl_info.u8_magic_word EQ ZZW_PLCP_MAGIC_WORD_ASSERT)
				{
					//ASSERT BY TOWARD NODE
					OAL_ASSERT(0, "");
				}
				else
				{
					//PLCP OVER DETECT
					g_u32_ics_head_crc = OAL_CRC_FAILURE;
					s_st_plcp_info.u16_is_valid    = OAL_FALSE;
					KS_LOG(0, 0xAF08, s_st_algo2l1cc_ctrl_info.st_ctrl_info.u8_magic_word);
				}
			}
			else
			{
				s_st_plcp_info.u16_is_valid    = OAL_TRUE;
				s_st_plcp_info.u16_need_report = OAL_TRUE;
				s_st_debug_info.u32_plcp_dec_success_cnt++;
				KS_LOG(0, 0xAF06, s_st_algo2l1cc_ctrl_info.st_ctrl_info.u32_frame_num);
				KS_LOG(0, 0xAF07, s_st_algo2l1cc_ctrl_info.st_ctrl_info.u16_send_cnt);
				#if 0//RX_STUB
				KS_LOG(0, 0xAF02, (s_st_algo2l1cc_ctrl_info.st_ctrl_info.u8_mcs));
				s_st_algo2l1cc_ctrl_info.st_ctrl_info.u8_mcs = RX_STUB_MCS;
				s_st_algo2l1cc_ctrl_info.st_ctrl_info.u8_data_type = 1;
				#else
				g_zzw_ch_status.node_id = (s_st_algo2l1cc_ctrl_info.st_ctrl_info.u8_node_id);
				//OAL_ASSERT(((g_zzw_ch_status.node_id > 0) && (g_zzw_ch_status.node_id <= 32)), "");
				g_zzw_ch_status.txpwr	= s_st_algo2l1cc_ctrl_info.st_ctrl_info.u8_tx_pwr;
				g_zzw_ch_status.mcs 	= s_st_algo2l1cc_ctrl_info.st_ctrl_info.u8_mcs; 	
				g_zzw_ch_status.snr = (s_st_algo2l1cc_ctrl_info.st_ctrl_info.s8_snr); 
				KS_LOG(0, 0xAF0A, g_zzw_ch_status.snr);//SNR OF toward node 
				#endif
				s_st_plcp_info.u16_data_type = (UINT16)s_st_algo2l1cc_ctrl_info.st_ctrl_info.u8_data_type;
				s_st_plcp_info.u16_mcs = g_st_mcs_tbl[s_st_algo2l1cc_ctrl_info.st_ctrl_info.u8_mcs].u16_mod_type;
				#if 1
				s_st_plcp_info.u32_output_lenght[0] = g_st_mcs_tbl[s_st_algo2l1cc_ctrl_info.st_ctrl_info.u8_mcs].u32_raw_len_cb;
				s_st_plcp_info.u32_input_lenght[0] = g_st_mcs_tbl[s_st_algo2l1cc_ctrl_info.st_ctrl_info.u8_mcs].u32_len_cb;
				s_st_plcp_info.u16_subframe_num = g_st_mcs_tbl[s_st_algo2l1cc_ctrl_info.st_ctrl_info.u8_mcs].u16_cb_num;
				#else
				s_st_plcp_info.u32_output_lenght[0] = g_st_mcs_tbl[s_st_algo2l1cc_ctrl_info.st_ctrl_info.u8_mcs].u32_turbo_len_in;
				s_st_plcp_info.u32_input_lenght[0] = g_st_mcs_tbl[s_st_algo2l1cc_ctrl_info.st_ctrl_info.u8_mcs].u32_turbo_len_out;
				s_st_plcp_info.u16_subframe_num = g_st_mcs_tbl[s_st_algo2l1cc_ctrl_info.st_ctrl_info.u8_mcs].u16_cb_num;
				#endif
				KS_LOG(0, 0xAF02, ((s_st_plcp_info.u16_data_type << 12) | (g_zzw_ch_status.node_id << 4) \
						|g_zzw_ch_status.mcs));
				KS_LOG(0, 0xAF04, (UINT16)s_st_algo2l1cc_ctrl_info.st_ctrl_info.u16_mac1_len);
				//OAL_ASSERT((s_st_plcp_info.u16_data_type NEQ 0) , "");
				#if 0
				if(s_st_plcp_info.u16_data_type)
				{
					#if 0
					cnt_delta = (s_st_algo2l1cc_ctrl_info.st_ctrl_info.u16_send_cnt - data_send_cnt_bak);
				
					if(cnt_delta > 1)
					{
						KS_LOG(0, 0xAF04, 0xDEAD);
						KS_LOG(0, 0xAF05, data_send_cnt_bak);
					}
				
					data_send_cnt_bak = s_st_algo2l1cc_ctrl_info.st_ctrl_info.u16_send_cnt; 		
					#endif
				}
				else
				#endif
				if(!s_st_plcp_info.u16_data_type)
				{
					//sfb
					g_u32_ics_frm_cnt_rpt = g_u32_ics_frm_cnt_cca;
					s_s32_frame_num_rpt = s_s32_frame_num_cca;
					//OAL_ASSERT(s_st_algo2l1cc_ctrl_info.st_ctrl_info.u8_mcs EQ 0, "");				
					KS_LOG(0, 0xAF06, s_st_algo2l1cc_ctrl_info.st_ctrl_info.u32_frame_num >> 16);
					if(s_u8_zzw_state EQ ZZW_STATE_NET)
					{
						s_u32_dl_sfb_cnt += 1;
						if(s_u32_dl_sfb_cnt >= 100)
						{
							s_u32_dl_scn_cnt = 0;
						}
					}
				}
				#if 0
				u32_fill_bits = (32 - ((s_st_plcp_info.u32_output_lenght[0] + 4) & 0x1F)) & 0x1F;
				
				if(u32_fill_bits > 16)
				{
					u32_llr_shift = (((s_st_plcp_info.u32_output_lenght[0] + 4 + 31)>>5)<<1) - 2;
				}
				else
				{
					u32_llr_shift = (((s_st_plcp_info.u32_output_lenght[0] + 4 + 31)>>5)<<1) - 1;
				}
				s_u32_llr_shift = u32_llr_shift;
				s_u32_turbo_in_len = MIN((s_st_plcp_info.u32_input_lenght[0] + u32_llr_shift), (3*s_st_plcp_info.u32_output_lenght[0] + 12));
				s_u32_mod_len = s_st_plcp_info.u32_input_lenght[0]/s_st_plcp_info.u16_mcs;
				#endif
				#if RTC_ADJ_FAST
				#if 0
				if((((SINT32)s_s32_sync_position <= (SINT32)(CCA_POS_IDEAL - 50)) && (s_s32_sync_position EQ (s_s32_sync_position_prev - 1))) || \
				(((SINT32)s_s32_sync_position >= (SINT32)(CCA_POS_IDEAL + 50)) && (s_s32_sync_position EQ (s_s32_sync_position_prev + 1))))
				#else
				if((((SINT32)s_s32_sync_pos_min <= (SINT32)(CCA_POS_IDEAL - 50)) || ((SINT32)s_s32_sync_pos_min >= (SINT32)(CCA_POS_IDEAL + 50))) && \
				((s_s32_sync_pos_min EQ (s_s32_sync_position_prev - 1)) || (s_s32_sync_pos_min EQ (s_s32_sync_position_prev + 1)))\
				&& (s_u32_rtc_adj_wait_cnt EQ 0))
				
				#endif
				{
					s_u32_rtc_adj_flg_fast = OAL_TRUE;
				}
				#endif
				
				s_st_debug_info.s32_time_adj_value = CCA_POS_IDEAL - s_s32_sync_position[0];

				#if 0
				if(g_u8_node_id)
				{
					OAL_ASSERT(g_u8_node_id EQ s_st_algo2l1cc_ctrl_info.st_ctrl_info.u8_node_id, "");
				}
				#endif
				#if !RX_STUB
				g_zzw_ch_status.pathloss_sub[0] += (g_zzw_ch_status.txpwr>>1);
				g_zzw_ch_status.pathloss_sub[1] += (g_zzw_ch_status.txpwr>>1);
				g_zzw_ch_status.pathloss += (g_zzw_ch_status.txpwr>>1);

				g_u8_rx_gain_expect = (UINT8)(RSSI_TARGET - g_zzw_ch_status.rssi);
				g_s8_tx_pwr_expect = (UINT8)(g_zzw_ch_status.pathloss - 100);
				KS_LOG(0, 0xAF07, g_zzw_ch_status.txpwr);
				KS_LOG(0, 0xAF0B, (UINT16)((g_s8_tx_pwr_expect << 8) | (g_u8_rx_gain_expect )));
				if((SINT8)g_u8_rx_gain_expect < 0)
				{
					g_u8_rx_gain_expect = 0;
				}
				#ifdef FUN_OTD
				KS_LOG(0, 0xAF0C, (UINT16)s_st_algo2l1cc_ctrl_info.st_ctrl_info.u8_node_id_target);
				if(s_st_algo2l1cc_ctrl_info.st_ctrl_info.u8_node_id_target EQ s_u8_node_id_local)
				{
					KS_LOG(0, 0xAF0D, (UINT16)s_st_algo2l1cc_ctrl_info.st_ctrl_info.s16_otd);
					if(s_st_algo2l1cc_ctrl_info.st_ctrl_info.s16_otd NEQ 0x7FFF)
					{	
						if((s_st_node_info_tbl.u8_otd_adj_flg EQ OAL_FALSE) && \
						(s_st_node_info_tbl.s16_otd_bak NEQ s_st_algo2l1cc_ctrl_info.st_ctrl_info.s16_otd) \
						&& ((s_st_node_info_tbl.s16_otd - s_st_algo2l1cc_ctrl_info.st_ctrl_info.s16_otd > 100)\
						|| (s_st_node_info_tbl.s16_otd - s_st_algo2l1cc_ctrl_info.st_ctrl_info.s16_otd < -100)))
						{
							s_st_node_info_tbl.u8_otd_adj_flg = OAL_TRUE;
							s_st_node_info_tbl.u8_ref_nodeid = s_st_algo2l1cc_ctrl_info.st_ctrl_info.u8_node_id;
							if(cp1_xc4500_def_flag == 1)
							{     
								ks_oal_mailbox_send_direct((UINT32)KS_CPU_CP1_X1643, (UINT32)KS_CPU_CP1_XC4500, (UINT32)SCFDE_4500_SEND_RTC_1643, (UINT32)-s_st_algo2l1cc_ctrl_info.st_ctrl_info.s16_otd);
							}
							else
							{
								ks_oal_mailbox_send_direct((UINT32)KS_CPU_CP0_X1643, (UINT32)KS_CPU_CP0_XC4500, (UINT32)SCFDE_4500_SEND_RTC_1643, (UINT32)-s_st_algo2l1cc_ctrl_info.st_ctrl_info.s16_otd);
							}
							s_st_node_info_tbl.s16_otd += s_st_algo2l1cc_ctrl_info.st_ctrl_info.s16_otd;
							s_st_node_info_tbl.s16_otd_bak = s_st_algo2l1cc_ctrl_info.st_ctrl_info.s16_otd;

						}

					}
				}
				#endif
				#ifdef TX_TEST
				s_st_algo2l1cc_ctrl_info.st_ctrl_info.u8_node_id = 1;
				#endif
				stp_node_info = (node_info_t*)node_tbl_find(s_st_algo2l1cc_ctrl_info.st_ctrl_info.u8_node_id);
				if(stp_node_info)//stp_node_info NEQ NULL_PTR, which means node exists already
				{
					//stp_node_info->s32_afc_angle = ((s_s32_angle_avg * (65536 - AFC_ALPHA) + stp_node_info->s32_afc_angle * AFC_ALPHA) >> 16);
					stp_node_info->s32_afc_angle = s_s32_angle_avg;
					stp_node_info->s8_snr = (SINT8)(((((sram_1643_4500_t*)(SRAM_1643_4500_ADDR))->s8_snr) * (65536 - AFC_ALPHA) + stp_node_info->s8_snr * AFC_ALPHA) >> 16);
					stp_node_info->u8_gain = (UINT8)((g_u8_rx_gain_expect * (65536 - AGC_ALPHA) + stp_node_info->u8_gain * AGC_ALPHA) >> 16);
					stp_node_info->s8_tx_pwr_expect = (SINT8)((g_s8_tx_pwr_expect * (65536 - AFC_ALPHA) + stp_node_info->s8_tx_pwr_expect * AFC_ALPHA) >> 16);
					stp_node_info->s32_cca_pos = s_s32_sync_pos_min;

					#if APC_TEST
					stp_node_info->s8_tx_pwr = s_st_algo2l1cc_ctrl_info.st_ctrl_info.s8_tx_pwr_expect;
					#else
					if(s_st_algo2l1cc_ctrl_info.st_ctrl_info.u8_node_id_target EQ s_u8_node_id_local)
					{
						//stp_node_info->s8_tx_pwr = s_st_algo2l1cc_ctrl_info.st_ctrl_info.s8_tx_pwr_expect;
						#ifdef FUN_OTD
						if(s_st_algo2l1cc_ctrl_info.st_ctrl_info.s16_otd EQ 0x7FFF)
						#else
						stp_node_info->s8_tx_pwr = s_st_algo2l1cc_ctrl_info.st_ctrl_info.s8_tx_pwr_expect;
						#endif
						{
							//otd invalid
							if((s_st_plcp_info.u16_data_type))
							{
								stp_node_info->u16_dec_cnt += 1;
								if(s_st_algo2l1cc_ctrl_info.st_ctrl_info.u8_mcs > stp_node_info->u8_mcs)
								{
									stp_node_info->u8_mcs = s_st_algo2l1cc_ctrl_info.st_ctrl_info.u8_mcs;
								}
							}
						}
					}
					else
					{
						//stp_node_info->s8_tx_pwr = stp_node_info->s8_tx_pwr_expect;
					}
					#endif
					//KS_LOG(0, 0xA417, (UINT16)((-stp_node_info->s32_afc_angle * 937) >> 8));
					//KS_LOG(0, 0xA41B, (UINT16)(stp_node_info->s8_snr));

					if(s_st_plcp_info.u16_data_type)
					{
						if(s_st_algo2l1cc_ctrl_info.st_ctrl_info.u16_send_cnt - stp_node_info->u16_send_cnt > 1)
						{
							KS_LOG(0, 0xAF04, 0xDEAD);						
							KS_LOG(0, 0xAF05, stp_node_info->u16_send_cnt);
						}
						stp_node_info->u16_send_cnt = s_st_algo2l1cc_ctrl_info.st_ctrl_info.u16_send_cnt; 		
					}
					stp_node_info->u32_rx_fn = s_st_algo2l1cc_ctrl_info.st_ctrl_info.u32_frame_num;
				}
				else
				{
					st_node_info_tmp.u8_node_id = s_st_algo2l1cc_ctrl_info.st_ctrl_info.u8_node_id;
					st_node_info_tmp.s8_snr = (((sram_1643_4500_t*)(SRAM_1643_4500_ADDR))->s8_snr);
					st_node_info_tmp.s32_afc_angle = s_s32_angle_avg;
					st_node_info_tmp.s32_cca_pos = s_s32_sync_pos_min;
					st_node_info_tmp.u32_rx_fn = s_st_algo2l1cc_ctrl_info.st_ctrl_info.u32_frame_num;
					st_node_info_tmp.u8_gain = g_u8_rx_gain_expect;
					st_node_info_tmp.s8_tx_pwr_expect = (g_s8_tx_pwr_expect);

					st_node_info_tmp.u16_dec_cnt = 1;
					st_node_info_tmp.u16_suc_cnt = 0;
					st_node_info_tmp.s8_mcs_offset = 0;
					st_node_info_tmp.s8_mcs_direction = 0;
					st_node_info_tmp.u8_mcs_punish = 0;

					st_node_info_tmp.u8_mcs = s_st_algo2l1cc_ctrl_info.st_ctrl_info.u8_mcs;
					#ifdef FUN_OTD
					st_node_info_tmp.s16_otd = 0;
					st_node_info_tmp.u16_otd_rpt_fn = 0;
					#endif
					#if APC_TEST
					st_node_info_tmp.s8_tx_pwr = s_st_algo2l1cc_ctrl_info.st_ctrl_info.s8_tx_pwr_expect;
					#else
					#ifndef FUN_OTD
					if(s_st_algo2l1cc_ctrl_info.st_ctrl_info.u8_node_id_target EQ s_u8_node_id_local)
					{
						st_node_info_tmp.s8_tx_pwr = s_st_algo2l1cc_ctrl_info.st_ctrl_info.s8_tx_pwr_expect;
					}
					else
					{	
						//open loop apc, use local tx pwr expect instead
						st_node_info_tmp.s8_tx_pwr = g_s8_tx_pwr_expect;
					}
					#endif
					#endif
					node_tbl_add(&st_node_info_tmp);
				}
				#endif
				((sram_1643_4500_t*)(SRAM_1643_4500_ADDR))->s8_snr = (SINT8)s_s32_snr_rpt;
			}
        }
        else
        {
			if(!s_u32_rtc_adj_wait_cnt)
			{
        		//OAL_ASSERT(0, "");
        	}

            s_st_debug_info.u32_plcp_dec_failure_cnt++;

            if(1 == s_st_debug_info.u32_enable_iq_dump)
            {
                ks_dmac_reset(KS_DMAC0_CP);

                __asm__("dint");

                OAL_ASSERT(0, "PLCP dec error");

                __asm__("eint");
            }
			//g_u32_ics_flag = ICS_CCA_DETECT;

			if(READ_REG32(ASSERT_4500_ADDR) EQ RX_DUMP_PLCP_ERR)
			{
				OAL_ASSERT(0, "");
			}
			KS_LOG(0, 0xAF08, s_s32_peak_min[0]);//debug for cca power thres 
			KS_LOG(0, 0xAF09, s_s32_peak_min[1]);//debug for cca power thres 
			
			g_zzw_ch_status.snr = -3; 
        }
    }
    else
    {
    	#if 0
        OAL_ASSERT((s_u32_data_buffer_wr_addr < L1CC_BB_SCFDE_DEC_DATA_BUF_END_ADDR) && (s_u32_data_buffer_wr_addr >= L1CC_BB_SCFDE_DEC_DATA_BUF_START_ADDR), "dl_subframe_dec error");

        if(0 == s_st_turbo_dec_output.st_monitor_info.crc_error)
        {
            s_u64_crc_bitmap |= (0x1<<s_u32_data_dec_cnt);
        }

        s_u32_data_dec_cnt++;

        dl_scramble(((s_st_turbo_dec_para.lte_param.turbo_decoder_lte_k.u32_lte_k - 24 + 31)>>5), &s_st_turbo_dec_output.u32_hd_results[0], (UINT32_PTR)s_u32_data_buffer_wr_addr);
        s_u32_data_buffer_wr_addr += ((s_st_turbo_dec_para.lte_param.turbo_decoder_lte_k.u32_lte_k - 24)>>3);

        OAL_ASSERT((s_u32_data_buffer_wr_addr < L1CC_BB_SCFDE_DEC_DATA_BUF_END_ADDR) && (s_u32_data_buffer_wr_addr >= L1CC_BB_SCFDE_DEC_DATA_BUF_START_ADDR), "dl_subframe_dec error");
		#endif

		//g_u32_ics_data_crc = s_st_turbo_dec_output.st_monitor_info.crc_error;
        if(OAL_SUCCESS == s_st_turbo_dec_output.st_monitor_info.crc_error)
		{
			g_u32_ics_crc_bmp |= (1 << g_u32_ics_cb_cnt);	
		}
		g_u32_ics_cb_cnt += 1;
		s_u32_data_cb_end_cnt++; 
		
		if(s_u32_data_cb_end_cnt EQ s_st_plcp_info.u16_subframe_num)
		{
			#if RTC_ADJ
			if(((s_u32_rtc_adj_flg || s_u32_rtc_adj_flg_fast)) && (((SINT32)s_s32_sync_pos_min <= (SINT32)(CCA_POS_IDEAL - 10)) || ((SINT32)s_s32_sync_pos_min >= (SINT32)(CCA_POS_IDEAL + 10))) \
					&& (g_u32_ics_crc_bmp EQ ((1 << s_u32_data_cb_end_cnt) - 1)))
			{
			    if(cp1_xc4500_def_flag == 1)
			    {     
					ks_oal_mailbox_send_direct((UINT32)KS_CPU_CP1_X1643, (UINT32)KS_CPU_CP1_XC4500, (UINT32)SCFDE_4500_SEND_RTC_1643, (UINT32)(CCA_POS_IDEAL- s_s32_sync_pos_min));
				}
				else
				{
					ks_oal_mailbox_send_direct((UINT32)KS_CPU_CP0_X1643, (UINT32)KS_CPU_CP0_XC4500, (UINT32)SCFDE_4500_SEND_RTC_1643, (UINT32)(CCA_POS_IDEAL- s_s32_sync_pos_min));
				}
				s_u32_rtc_adj_flg = OAL_FALSE;
				s_u32_rtc_adj_flg_fast = OAL_FALSE;
				s_u32_rtc_adj_wait_cnt = 10;
			}
			#endif
			
			if((0 == s_st_algo2l1cc_ctrl_info.st_ctrl_info.u8_data_type) && (g_u32_ics_crc_bmp EQ ((1 << s_u32_data_cb_end_cnt) - 1)))
			{
				g_u8_rpt_type = (1);
				//ks_oal_sema_put(OAL_L1CC_IPC_SEMA);
			}
			else if((g_u32_ics_crc_bmp EQ ((1 << s_u32_data_cb_end_cnt) - 1)))
			{
				g_u8_rpt_type = (2);
				//ks_oal_sema_put(OAL_L1CC_IPC_SEMA);
				stp_node_info = (node_info_t*)node_tbl_find(s_st_algo2l1cc_ctrl_info.st_ctrl_info.u8_node_id);
				if(stp_node_info)//stp_node_info NEQ NULL_PTR, which means node exists already
				{
					if(s_st_algo2l1cc_ctrl_info.st_ctrl_info.u8_node_id_target EQ s_u8_node_id_local)
					{
						#ifdef FUN_OTD
						if(s_st_algo2l1cc_ctrl_info.st_ctrl_info.s16_otd EQ 0x7FFF)
						#endif
						{
							stp_node_info->u16_suc_cnt += 1;
						}
					}
				}

			} 
			else
			{
				if(g_u32_ics_crc_bmp & 0x1)
				{
					g_u8_rpt_type = (0x10003);//1st cb is crc good
				}
				else
				{
					g_u8_rpt_type = (0x3);//1st cb is crc bad
				}
				#if RX_STUB
				OAL_ASSERT(0, "");
				#endif
				#if 1
	        	if(READ_REG32(ASSERT_4500_ADDR) EQ RX_DUMP_DATA_ERR)
	        	{
					OAL_ASSERT(0, "");
	        	}
	        	#endif
			}
			#ifndef TX_TEST
			#if !RX_STUB
			ks_oal_sema_put(OAL_L1CC_IPC_SEMA);
			#endif
			#endif
			s_st_plcp_info.u16_is_valid = OAL_FALSE;
			s_u32_data_cb_end_cnt = 0;
			g_u32_ics_sym_bmp = 0;
			g_u32_ics_head_crc = OAL_CRC_FAILURE;	
			g_u32_ics_cb_cnt = 0;
			g_u32_ics_crc_bmp = 0;
						
		}


        if(OAL_SUCCESS == s_st_turbo_dec_output.st_monitor_info.crc_error)
        {
            s_st_debug_info.u32_data_dec_success_cnt++;
        	dl_scramble(((s_st_turbo_dec_para.lte_param.turbo_decoder_lte_k.u32_lte_k - 24 + 31)>>5), &s_st_turbo_dec_output.u32_hd_results[0], (UINT32_PTR)s_u32_data_buffer_wr_addr);
			//ks_oal_mem_copy((VOID_PTR)(s_u32_data_buffer_wr_addr), (VOID_PTR)(s_st_turbo_dec_output.u32_hd_results[0]), ((s_st_turbo_dec_para.lte_param.turbo_decoder_lte_k.u32_lte_k-24)>>3));
			s_u32_data_buffer_wr_addr += ((s_st_turbo_dec_para.lte_param.turbo_decoder_lte_k.u32_lte_k-24)>>3);
        	OAL_ASSERT((s_u32_data_buffer_wr_addr < L1CC_BB_SCFDE_DEC_DATA_BUF_END_ADDR) && (s_u32_data_buffer_wr_addr >= L1CC_BB_SCFDE_DEC_DATA_BUF_START_ADDR), "dl_subframe_dec error");

        }
        else
        {
	
			if(READ_REG32(ASSERT_4500_ADDR) EQ 0x1234)
			{
				//OAL_ASSERT(0, "");
			}

            s_st_debug_info.u32_data_dec_failure_cnt++;
            

            if(2 == s_st_debug_info.u32_enable_iq_dump)
            {
                ks_dmac_reset(KS_DMAC0_CP);

                __asm__("dint");

                 OAL_ASSERT(0, "Data dec error");

                __asm__("eint");
            }
        }

    }

    u32_read_index = (u32_read_index + 1) % TURBO_DEC_INFO_AMT;
    s_st_turbo_dec_info_link.u32_read_index = u32_read_index;

    if(u32_write_index != u32_read_index)
    {
        /*start turbo decode*/
        s_st_turbo_dec_para.lte_param.turbo_decoder_lte_length.u32_lte_length = s_st_turbo_dec_info_link.st_turbo_dec_info[u32_read_index].u32_input_lenght;
        s_st_turbo_dec_para.lte_param.turbo_decoder_lte_k.u32_lte_k           = s_st_turbo_dec_info_link.st_turbo_dec_info[u32_read_index].u32_output_lenght;
        s_st_turbo_dec_para.lte_param.turbo_decoder_lte_n.u32_lte_n           = s_st_turbo_dec_info_link.st_turbo_dec_info[u32_read_index].u32_input_lenght;
        s_st_turbo_dec_para.lte_param.turbo_decoder_lte_crc_select.u32_lte_crc_select = s_st_turbo_dec_info_link.st_turbo_dec_info[u32_read_index].u32_crc_type;

        turbo_decoder_config_register(&s_st_turbo_dec_para);

        u32_write_offset = s_st_turbo_dec_info_link.st_turbo_dec_info[u32_read_index].u32_write_offset;
		if(s_st_turbo_dec_info_link.st_turbo_dec_info[u32_read_index].u32_data_type EQ DATA_TYPE)
		{
			turbo_decoder_run_dma(s_st_turbo_dec_para.lte_param.turbo_decoder_lte_length.u32_lte_length, (SINT8_PTR)&s_s8_llr[u32_write_offset]);
		}
		else
		{
			turbo_decoder_run(s_st_turbo_dec_para.lte_param.turbo_decoder_lte_length.u32_lte_length, (SINT8_PTR)&s_s8_llr[u32_write_offset]);
		}
        //turbo_decoder_run(s_st_turbo_dec_para.lte_param.turbo_decoder_lte_length.u32_lte_length, s_s8_llr[u32_read_index]);
    }
	//KS_LOG_TMRL(0xA001);
}
    
OAL_ITCM_CODE_SECTION
VOID dl_turbo_dec_start(UINT32 u32_data_type, UINT32 u32_input_data_addr, UINT32 u32_input_length, UINT32 u32_output_length,UINT32 u32_crc_type)
{
	TURBO_DECODER_STATUS e_turbo_sta;
    UINT32 u32_input_len;
    UINT32 u32_rd_index, u32_rd_offset;
    UINT32 u32_wr_index, u32_wr_offset;

    OAL_IRQ_SAVE_AREA;
    OAL_IRQ_DISABLE;
    u32_input_len = (u32_input_length + 0xF) & 0xFFFFFFF0; /*aligned 128 bit*/
    //u32_rd_index = s_st_turbo_dec_info_link.u32_read_index;
    u32_wr_index = s_st_turbo_dec_info_link.u32_write_index;
    u32_wr_offset = s_st_turbo_dec_info_link.st_turbo_dec_info[u32_wr_index].u32_write_offset;

    s_st_turbo_dec_info_link.st_turbo_dec_info[u32_wr_index].u32_input_lenght  =  u32_input_length;
    s_st_turbo_dec_info_link.st_turbo_dec_info[u32_wr_index].u32_output_lenght =  u32_output_length;
    s_st_turbo_dec_info_link.st_turbo_dec_info[u32_wr_index].u32_data_type     =  u32_data_type;
    s_st_turbo_dec_info_link.st_turbo_dec_info[u32_wr_index].u32_crc_type      =  u32_crc_type;

	e_turbo_sta = turbo_decoder_get_status();
	//KS_LOG(0, 0xAE02, (UINT16)s_st_turbo_dec_info_link.u32_write_index);

    if(TURBO_DECODER_IDLE == e_turbo_sta)
    {
        /*start turbo decode*/
        s_st_turbo_dec_para.lte_param.turbo_decoder_lte_length.u32_lte_length         = u32_input_length;
        s_st_turbo_dec_para.lte_param.turbo_decoder_lte_k.u32_lte_k                   = u32_output_length;
        s_st_turbo_dec_para.lte_param.turbo_decoder_lte_n.u32_lte_n                   = u32_input_length;
        s_st_turbo_dec_para.lte_param.turbo_decoder_lte_crc_select.u32_lte_crc_select = u32_crc_type;

        turbo_decoder_config_register(&s_st_turbo_dec_para);
		if(u32_data_type EQ DATA_TYPE)
		{
	        turbo_decoder_run_dma(u32_input_length, (SINT8_PTR)u32_input_data_addr);		
		}
		else
		{
	        turbo_decoder_run(u32_input_length, (SINT8_PTR)u32_input_data_addr);		
		}
    }
    else
    {
        
    }
    s_st_turbo_dec_info_link.u32_write_index = (u32_wr_index + 1) % TURBO_DEC_INFO_AMT;
    s_st_turbo_dec_info_link.st_turbo_dec_info[s_st_turbo_dec_info_link.u32_write_index].u32_write_offset = (u32_wr_offset + u32_input_len);
    OAL_ASSERT(s_st_turbo_dec_info_link.u32_write_index != s_st_turbo_dec_info_link.u32_read_index, "turbo buffer full");

    OAL_IRQ_RESTORE;
}

OAL_ITCM_CODE_SECTION
VOID dl_turbo_dec_init()
{
    turbo_decoder_init();

    /*start turbo decode*/
    s_st_turbo_dec_para.lte_param.turbo_decoder_lte_blkid.u32_lte_blkid             = 1;
    //s_st_turbo_dec_para.lte_param.turbo_decoder_lte_length.u32_lte_length         = u32_input_length;
    //s_st_turbo_dec_para.lte_param.turbo_decoder_lte_k.u32_lte_k                   = u32_output_length;
    s_st_turbo_dec_para.lte_param.turbo_decoder_lte_crc_select.u32_lte_crc_select   = CRC_24A;
    //s_st_turbo_dec_para.lte_param.turbo_decoder_lte_n.u32_lte_n                   = u32_input_length;
    s_st_turbo_dec_para.lte_param.turbo_decoder_lte_nfiller.u32_lte_nfiller         = 0;
    s_st_turbo_dec_para.lte_param.turbo_decoder_lte_bypass_nbi.u32_lte_bypass_nbi   = 0;
    s_st_turbo_dec_para.lte_param.turbo_decoder_lte_sw_speed.u32_lte_sw_speed       = SW_SPEED_8;
    s_st_turbo_dec_para.lte_param.turbo_decoder_lte_algo_select.u32_lte_algo_select = LTE_ALGO_SELECT;
    s_st_turbo_dec_para.lte_param.turbo_decoder_lte_halfit.u32_lte_halfit           = LTE_HALFIT;
    s_st_turbo_dec_para.lte_param.turbo_decoder_lte_dyn_stop.u32_lte_dyn_stop       = 0;
    s_st_turbo_dec_para.lte_param.turbo_decoder_lte_threshold.u32_lte_threshold     = 0;
    s_st_turbo_dec_para.lte_param.turbo_decoder_lte_table_index.u32_lte_table_index = LTE_TABLE_INDEX;
    s_st_turbo_dec_para.lte_hspa_ind                                                = 0;

    turbo_decoder_set_callback_func(dl_turbo_dec_callback);
}

OAL_ITCM_CODE_SECTION
SINT16 dl_scale_calc(SINT32_PTR s32p_rx_buf, UINT32 u32_len)
{
    SINT16 s16_shift = 0;
    SINT32 s32_rssi, s32_sqrt_rssi;
    SINT32_PTR s32p_temp = NULL_PTR;

	/*calc mean pwr*/
    //s32p_temp = (SINT32_PTR)&s_st_rx_buffer[s_u32_pingpong_idx][GP_LENGTH + AGC_LENGTH];
    s32p_temp = (SINT32_PTR)s32p_rx_buf;

    s32_rssi = ks_cxv_rssi_calc(s32p_temp, u32_len);
    s32_sqrt_rssi = ks_sqrt_s32(s32_rssi);

    s16_shift = (14 - (30 - exp_acW_acZ(s32_sqrt_rssi) + 1));
    s16_shift = s16_shift > 0 ? s16_shift : 0;

    return s16_shift;
}
OAL_ITCM_CODE_SECTION
VOID dl_h_mod_calc(IN complex_s16_t* stp_H_in, OUT complex_s16_t* stp_H_mod, IN UINT32 u32_h_type , IN SINT32 s32_noise) 
{
	/*u32_h_type : 0(1t1r) 1(1t2r) 2(2t1r) 3(2t2r)*/
    SINT32 s32_iloop;
    SINT32 s32_noise_pwr = (s32_noise >> 15); //为了定标一致
    UINT32 u32_permute[2] = {0x66442200, 0xEECCAA88};
    UINT32 u32_h_num[4] = {1, 2, 2, 4};
    UINT32 u32_h_delta[4] = {0, SC_NUM * 2, SC_NUM, SC_NUM};    
	complex_s16_t* stp_h[4]; //h11,h12,h21,h22

    vec_t  v32_1;
    vec_t  v32_2;
    vec_t  v32_3;
    vec_t  v32_4;
    vec_t  v32_tmp; 
	vec_t  v32_permute;	
    vec_t  v32_h;
    vec_t  v32_h2;
    vec_t  v32_h3;
    vec_t  v32_h4;    

    coef_t vcoef;

    /*calculate H*conj(H)*/
    /*calc H*conj(H) + noise*/
    v32_permute = vlddw_v32_clone (2, u32_permute, 0);
    vcoef = vlddw_c32_clone_1dw (&s32_noise_pwr);
    //vshift_val = vlddw_c32_clone_1dw (&s32_shift_val);

	__asm__("vpush {dw} modv0");
	__asm__("vpush {dw} modv2");
	__asm__("nop");
	__asm__("nop");
	__asm__("push {dw} a8");
	__asm__("mov #0x00000C20, a8");/* 功能寄存器压栈 */
	__asm__("nop");
	__asm__("nop");
	__asm__("vmova a8, modv0");/* 功能寄存器设置 */
	
	__asm__("mov #0x00800C20, a8");/* 功能寄存器压栈 */
	__asm__("nop");
	__asm__("nop");
	__asm__("vmova a8, modv2");/* 功能寄存器设置 */
	
	__asm__("pop {dw} a8");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
    
	if(u32_h_type EQ RTX_2T2R) //2t2r
	{
		#if 0
	    for(s32_iloop = 0; s32_iloop < (SC_NUM>>4); s32_iloop++)
	    #pragma dsp_ceva_unroll = 2
	    #pragma dsp_ceva_trip_count_min = 2
	    {

			v32_h = vlddw_v32 (8, stp_H_mod + 16*s32_iloop, 0);
			v32_1 = vadd16_v32_c32_v32 (8, v32_h, vcoef, LOW);
			
			v32_h = vlddw_v32 (8, stp_H_mod + 16*s32_iloop + SC_NUM * 1, 0);
			v32_2 = vadd16_v32_v32_v32 (8, v32_h, v32_1);
			
			v32_h = vlddw_v32 (8, stp_H_mod + 16*s32_iloop + SC_NUM * 2, 0);
			v32_3 = vadd16_v32_v32_v32 (8, v32_h, v32_2);
			
			v32_h = vlddw_v32 (8, stp_H_mod + 16*s32_iloop + SC_NUM * 3, 0);
			v32_4 = vadd16_v32_v32_v32 (8, v32_h, v32_3);

			v32_tmp = vpermutew_v32_v32 (8, v32_permute, v32_4);
			
			vstw_v32(8, v32_tmp, 0, stp_H_mod + 16*s32_iloop);
			vstw_v32(8, v32_tmp, 4, stp_H_mod + 16*s32_iloop + 4);
	    }
	    #else
		for(s32_iloop = 0; s32_iloop < (SC_NUM>>4); s32_iloop++)
		#pragma dsp_ceva_unroll = 2
		#pragma dsp_ceva_trip_count_min = 1
		{
		
			v32_1 = vlddw_v32(8, stp_H_in + 16*s32_iloop,0); //Vy1[0..7]<-complex_data
			v32_h = vmpyxp_v32_v32_v32_conj(8, v32_1, 0, v32_1, 0);
			v32_h = vadd16_v32_c32_v32 (8, v32_h, vcoef, LOW);
			
			v32_2 = vlddw_v32(8, stp_H_in + 16*s32_iloop + SC_NUM * 1,0); //Vy1[0..7]<-complex_data
			v32_h2 = vmpyxp_v32_v32_v32_conj(8, v32_2, 0, v32_2, 0);
 			v32_h2 = vadd16_v32_v32_v32 (8, v32_h, v32_h2);
 			
			v32_3 = vlddw_v32(8, stp_H_in + 16*s32_iloop + SC_NUM * 2,0); //Vy1[0..7]<-complex_data
			v32_h3 = vmpyxp_v32_v32_v32_conj(8, v32_3, 0, v32_3, 0);
			v32_h3 = vadd16_v32_v32_v32 (8, v32_h2, v32_h3);			
			
			v32_4 = vlddw_v32(8, stp_H_in + 16*s32_iloop + SC_NUM * 3,0); //Vy1[0..7]<-complex_data
			v32_h4 = vmpyxp_v32_v32_v32_conj(8, v32_4, 0, v32_4, 0);
			v32_h4 = vadd16_v32_v32_v32 (8, v32_h3, v32_h4);

			v32_tmp = vpermutew_v32_v32 (8, v32_permute, v32_h4);
			
			vstw_v32(8, v32_tmp, 0, stp_H_mod + 16*s32_iloop);
			vstw_v32(8, v32_tmp, 4, stp_H_mod + 16*s32_iloop + 4);
		}

	    #endif
    }
    else
	{
	    for(s32_iloop = 0; s32_iloop < u32_h_num[u32_h_type]; s32_iloop++)
	    {
	    	stp_h[s32_iloop] = stp_H_in + u32_h_delta[u32_h_type] * s32_iloop;
		    ks_cxv_mul_cxv(stp_h[s32_iloop], stp_h[s32_iloop], SC_NUM, 2, stp_H_mod + SC_NUM * s32_iloop);
	    }

	    if(u32_h_type EQ RTX_1T1R)
	    {
		    for(s32_iloop = 0; s32_iloop < (SC_NUM>>4); s32_iloop++)
		    #pragma dsp_ceva_unroll = 2
		    #pragma dsp_ceva_trip_count_min = 1
		    {
		        v32_2 = vlddw_v32 (8, stp_H_mod + 16*s32_iloop, 0);
		        v32_3 = vpermutew_v32_v32 (8, v32_permute, v32_2);
		        v32_4 = vadd16_v32_c32_v32 (8, v32_3, vcoef, LOW);

		        vstw_v32 (8, v32_4, 0, stp_H_mod + 16*s32_iloop);
		        vstw_v32 (8, v32_4, 4, stp_H_mod + 16*s32_iloop + 4);
		    }    	
	    }
	    else if(u32_h_type EQ RTX_1T2R)
	    {
		    for(s32_iloop = 0; s32_iloop < (SC_NUM>>4); s32_iloop++)
		    #pragma dsp_ceva_unroll = 2
		    #pragma dsp_ceva_trip_count_min = 1
		    {
		        v32_2 = vlddw_v32 (8, stp_H_mod + 16*s32_iloop, 0);
		        v32_3 = vadd16_v32_c32_v32 (8, v32_2, vcoef, LOW);

		        v32_4 = vlddw_v32 (8, stp_H_mod + 16*s32_iloop + SC_NUM, 0);
		        v32_tmp = vadd16_v32_v32_v32 (8, v32_3, v32_4);

		        v32_h = vpermutew_v32_v32 (8, v32_permute, v32_tmp);

		        vstw_v32 (8, v32_h, 0, stp_H_mod + 16*s32_iloop);
		        vstw_v32 (8, v32_h, 4, stp_H_mod + 16*s32_iloop + 4);
		    }    	
	    }
	    else if(u32_h_type EQ RTX_2T1R)
	    {
		    for(s32_iloop = 0; s32_iloop < (SC_NUM>>4); s32_iloop++)
		    #pragma dsp_ceva_unroll = 2
		    #pragma dsp_ceva_trip_count_min = 1
		    {
		        v32_2 = vlddw_v32 (8, stp_H_mod + 16*s32_iloop, 0);
		        v32_3 = vadd16_v32_c32_v32 (8, v32_2, vcoef, LOW);

		        v32_4 = vlddw_v32 (8, stp_H_mod + 16*s32_iloop + SC_NUM, 0);
		        v32_tmp = vadd16_v32_v32_v32 (8, v32_3, v32_4);

		        v32_h = vpermutew_v32_v32 (8, v32_permute, v32_tmp);

		        vstw_v32(8, v32_h, 0, stp_H_mod + 16*s32_iloop);
		        vstw_v32(8, v32_h, 4, stp_H_mod + 16*s32_iloop + 4);
		    }    
	    }
	}
    __asm__("nop");
    __asm__("nop");
    __asm__("nop");
    __asm__("nop");
    __asm__("vpop {dw} modv2");
    __asm__("vpop {dw} modv0");
    __asm__("nop");
    __asm__("nop");
    __asm__("nop");
    __asm__("nop");    

}
OAL_ITCM_CODE_SECTION
VOID dl_data_sfbc_reorder(INOUT complex_s16_t* stp_data_in, IN UINT32 u32_length) //length must be aligned with 16
{
    SINT32 s32_iloop;
    UINT32 u32_permute[2] = {0x54761032, 0xDCFE98BA};
	vec_t  v32_tmp; 
	vec_t  v32_permute;	
	vec_t  v32_data;
	vprex_t vprex_1;

	//KS_LOG(0, 0xA223, (UINT16)s32_noise_pwr);
	//DL_GET_TMR2_START;

	#if 0
	if(g_test_flg)
	{
		OAL_IRQ_DISABLE_ALL;
	}
	#endif

	v32_permute = vlddw_v32_clone (2, u32_permute, 0);
	vprex_1 = vcopy_imm16_8_vpr((SINT16)0x6666);

 
	for(s32_iloop = 0; s32_iloop < (u32_length>>4); s32_iloop++)
    #pragma dsp_ceva_unroll = 2
    #pragma dsp_ceva_trip_count_min = 1	
	{
		v32_data = vlddw_v32 (8, stp_data_in + 16*s32_iloop, 0);
		v32_tmp = vpermutew_v32_v32 (8, v32_permute, v32_data);
        v32_tmp = vnegw_v32_v32_p (8, v32_tmp, vprex_1);
		vstdw_v32(8, v32_tmp, 0, stp_data_in + 16*s32_iloop);
	}
}

UINT32 g_test_flg = 0;
OAL_ITCM_CODE_SECTION
VOID dl_equalize_1t1r(complex_s16_t* stp_H_in, complex_s16_t* stp_data_in, complex_s16_t *stp_equ_out, UINT32 u32_length) //length must be aligned with 16
{
    SINT32 s32_iloop;
    UINT32 u32_permute[2] = {0x66442200, 0xEECCAA88};
    SINT32 s32_shift_amt_1[6] = {(UINT32)MODV_SV1, (UINT32)MODV_SV1, (UINT32)MODV_SV1, (UINT32)MODV_SV1,(UINT32)MODV_SV1};
    SINT32 s32_shift_amt_2[6] = {(UINT32)MODV_SV0, (UINT32)MODV_SV0, (UINT32)MODV_SV0, (UINT32)MODV_SV0,(UINT32)MODV_SV0};
    //complex_s16_t st_dft_600_factor = {26913, 0}; // factor=512/600
    complex_s16_t st_dft_600_factor = {28835, 0}; // factor=512/600
    OAL_IRQ_SAVE_AREA;

    vec_t  v32_1;
    vec_t  v32_2;
    vec_t  v32_3;
    vec_t  v32_4;
    coef_t vcoef;

	//KS_LOG(0, 0xA220, (UINT16)0);

	#if VECTOR_CMP
	if(g_test_flg)
	{
		OAL_IRQ_DISABLE_ALL;
	}
	#endif
    /*calculate Y*conj(H)/(H*conj(H) + noise_pwr)*/
    ks_cxv_mul_cxv(stp_data_in, stp_H_in, u32_length, 2, s_st_temp_buffer[1]);
	#if VECTOR_CMP
    if(g_test_flg)
	{
		while(READ_REG32(ASSERT_4500_ADDR) != 2);
		WRITE_REG32(ASSERT_4500_ADDR, 2);	
	}
	#endif
	#ifdef SCFDMA_SWITCH
	ks_s16v_div_s16v((SINT16_PTR)s_st_temp_buffer[1], (SINT16_PTR)s_st_H_mod, u32_length, (SINT16_PTR)s_st_temp_buffer[0]);
	#if VECTOR_CMP
	if(g_test_flg)
	{
		while(READ_REG32(ASSERT_4500_ADDR) != 3);
		WRITE_REG32(ASSERT_4500_ADDR, 3); 
	}
	#endif

	if(g_u32_tx_ant_1st_id)
	{
		//1t1r, tx ant is ant 2 , need do data reorder
		dl_data_sfbc_reorder(s_st_temp_buffer, SC_NUM);
	}
	#if 1
	//fftshift
	for(s32_iloop = 0; s32_iloop < ((SINT32)HALF_SC_NUM>>4); s32_iloop++)
	{
		v32_1 = vlddw_v32 (8, ((SINT32_PTR)s_st_temp_buffer) + 16*s32_iloop, 0);
		vstdw_v32 (8, v32_1, 0, ((SINT32_PTR)s_st_temp_buffer) + SC_NUM + 16*s32_iloop);
	}
	v32_1 = vlddw_v32 (8, ((SINT32_PTR)s_st_temp_buffer) + 16*s32_iloop, 0);
	vstdw_v32 (8, v32_1, 0, ((SINT32_PTR)s_st_temp_buffer) + SC_NUM + 16*s32_iloop);

    /*idft(1200) transform into time domain*/
	ks_idft1200(((SINT32_PTR)s_st_temp_buffer + HALF_SC_NUM), stp_equ_out, s_st_temp_buffer[1], s32_shift_amt_1);
	#else
    /*dft(1200) transform into freq domain*/
    dft_ceva_algo_1200(SC_NUM, s32_shift_amt_1, 1, ((SINT32_PTR)s_st_temp_buffer), stp_equ_out, s_st_temp_buffer[1]);
	#endif
	//_ks_shift_s16v((SINT16_PTR)stp_equ_out, (u32_length << 1), 2, (SINT16_PTR)s_st_temp_buffer[1]);
    //_ks_cxv_mul_cx(s_st_temp_buffer[1], SC_NUM, &st_dft_600_factor, 0, stp_equ_out); 
    _ks_cxv_mul_cx(stp_equ_out, SC_NUM, &st_dft_600_factor, 0, stp_equ_out); 
	#if VECTOR_CMP
	if(g_test_flg)
	{
		while(READ_REG32(ASSERT_4500_ADDR) != 4);
		WRITE_REG32(ASSERT_4500_ADDR, 4); 
		OAL_IRQ_RESTORE;
	}
	g_test_flg++;
	#endif
	#else
	ks_s16v_div_s16v((SINT16_PTR)s_st_temp_buffer[1], (SINT16_PTR)s_st_H_mod, u32_length, (SINT16_PTR)stp_equ_out);
	if(g_u32_tx_ant_1st_id)
	{
		//1t1r, tx ant is ant 2 , need do data reorder
		dl_data_sfbc_reorder(stp_equ_out, SC_NUM);
	}
	#endif

    
}

OAL_ITCM_CODE_SECTION
VOID dl_equalize_1t2r(complex_s16_t* stp_H_in, complex_s16_t* stp_H_in2, complex_s16_t* stp_data_in, complex_s16_t* stp_data_in2, complex_s16_t *stp_equ_out, UINT32 u32_length) //length must be aligned with 16
{
    SINT32 s32_iloop;
    UINT32 u32_permute[2] = {0x66442200, 0xEECCAA88};
    SINT32 s32_shift_amt_1[6] = {(UINT32)MODV_SV1, (UINT32)MODV_SV1, (UINT32)MODV_SV1, (UINT32)MODV_SV1,(UINT32)MODV_SV1};
    SINT32 s32_shift_amt_2[6] = {(UINT32)MODV_SV0, (UINT32)MODV_SV0, (UINT32)MODV_SV0, (UINT32)MODV_SV0,(UINT32)MODV_SV0};
    complex_s16_t st_dft_600_factor = {28835, 0}; // factor=512/600
    //complex_s16_t st_dft_600_factor = {26350, 0}; // factor=512/600
    OAL_IRQ_SAVE_AREA;

    vec_t  v32_1;
    vec_t  v32_2;
    vec_t  v32_3;
    vec_t  v32_4;
    vec_t  v32_5;
    coef_t vcoef;

	//KS_LOG(0, 0xA220, (UINT16)1);

    /*calculate Y*conj(H)/(H*conj(H) + noise_pwr)*/
    ks_cxv_mul_cxv(stp_data_in, stp_H_in, u32_length, 2, s_st_temp_buffer[0]);
	//_ks_shift_s16v((SINT16_PTR)s_st_temp_buffer[0], (u32_length<<1), -1, (SINT16_PTR)s_st_temp_buffer[0]);
    ks_cxv_mul_cxv(stp_data_in2, stp_H_in2, u32_length, 2, s_st_temp_buffer[1]);
	//_ks_shift_s16v((SINT16_PTR)s_st_temp_buffer[1], (u32_length<<1), -1, (SINT16_PTR)s_st_temp_buffer[1]);    
	_ks_cxv_add_cxv(s_st_temp_buffer[0], s_st_temp_buffer[1], SC_NUM, &s_st_temp_buffer[1][SC_NUM]);
    
#ifdef SCFDMA_SWITCH
	ks_s16v_div_s16v((SINT16_PTR)&s_st_temp_buffer[1][SC_NUM], (SINT16_PTR)s_st_H_mod, u32_length, (SINT16_PTR)s_st_temp_buffer[0]);
	if(g_u32_tx_ant_1st_id)
	{
		//1t2r, tx ant is ant 2 , need do data reorder
		dl_data_sfbc_reorder(s_st_temp_buffer, SC_NUM);
	}

#if 1
	//fftshift
	for(s32_iloop = 0; s32_iloop < ((SINT32)HALF_SC_NUM>>4); s32_iloop++)
	{
		v32_1 = vlddw_v32 (8, ((SINT32_PTR)s_st_temp_buffer) + 16*s32_iloop, 0);
		vstdw_v32 (8, v32_1, 0, ((SINT32_PTR)s_st_temp_buffer) + SC_NUM + 16*s32_iloop);
	}
	v32_1 = vlddw_v32 (8, ((SINT32_PTR)s_st_temp_buffer) + 16*s32_iloop, 0);
	vstdw_v32 (8, v32_1, 0, ((SINT32_PTR)s_st_temp_buffer) + SC_NUM + 16*s32_iloop);

    /*idft(1200) transform into time domain*/
	ks_idft1200(((SINT32_PTR)s_st_temp_buffer + HALF_SC_NUM), stp_equ_out, s_st_temp_buffer[1], s32_shift_amt_1);
#else
    /*dft(1200) transform into freq domain*/
    dft_ceva_algo_1200(SC_NUM, s32_shift_amt_1, 1, ((SINT32_PTR)s_st_temp_buffer), stp_equ_out, s_st_temp_buffer[1]);
#endif
	//_ks_shift_s16v((SINT16_PTR)stp_equ_out, (u32_length << 1), 2, (SINT16_PTR)s_st_temp_buffer[1]);
    //_ks_cxv_mul_cx(s_st_temp_buffer[1], SC_NUM, &st_dft_600_factor, 0, stp_equ_out); 
    _ks_cxv_mul_cx(stp_equ_out, SC_NUM, &st_dft_600_factor, 0, stp_equ_out); 
	//OAL_IRQ_RESTORE;
#else
	ks_s16v_div_s16v((SINT16_PTR)&s_st_temp_buffer[1][SC_NUM], (SINT16_PTR)s_st_H_mod, u32_length, (SINT16_PTR)stp_equ_out);
	//OAL_ASSERT(0, "");
	if(g_u32_tx_ant_1st_id)
	{
		//1t1r, tx ant is ant 2 , need do data reorder
		dl_data_sfbc_reorder(stp_equ_out, SC_NUM);
	}
#endif
	
    
}

OAL_ITCM_CODE_SECTION
VOID dl_equalize_2t1r(complex_s16_t* stp_H_in, complex_s16_t* stp_H_in2, complex_s16_t* stp_data_in, complex_s16_t *stp_equ_out, UINT32 u32_length) //length must be aligned with 16
{
    SINT32 s32_iloop;
    UINT32 u32_permute[2] = {0x66442200, 0xEECCAA88};
    UINT32 u32_permute_even[2] = {0xDC985410, 0x0};
    UINT32 u32_permute_odd[2] = {0xFEBA7632, 0x0};
    UINT32 u32_permute_eo[2] = {0x54761032, 0xDCFE98BA};

    SINT32 s32_shift_amt_1[6] = {(UINT32)MODV_SV1, (UINT32)MODV_SV1, (UINT32)MODV_SV1, (UINT32)MODV_SV1,(UINT32)MODV_SV0};
    SINT32 s32_shift_amt_2[6] = {(UINT32)MODV_SV0, (UINT32)MODV_SV0, (UINT32)MODV_SV0, (UINT32)MODV_SV0,(UINT32)MODV_SV0};
    //complex_s16_t st_dft_600_factor = {28111, 0}; // factor=512/600
    complex_s16_t st_dft_600_factor = {30119, 0}; // factor=512/600
    OAL_IRQ_SAVE_AREA;

    vec_t  v32_1;
    vec_t  v32_2;
    vec_t  v32_3;
    vec_t  v32_4;
    vec_t  v32_5;
    vec_t  v32_6;
    vec_t  v32_7;
    vec_t  v32_8;
	vec_t  v32_permute_even;
	vec_t  v32_permute_odd;
	vec_t  v32_permute_eo;	
	vec_t  v32_data;
    vec_t  v32_h;
    vec_t  v32_h2;	
	vprex_t vprex_even;
	vprex_t vprex_odd;

    coef_t vcoef;

	//KS_LOG(0, 0xA222, (UINT16)s32_noise_pwr);
	//OAL_IRQ_DISABLE_ALL;
	//while(READ_REG32(0x1724FFE0) != 1);
	//WRITE_REG32(0x1724FFE4, 1);

	v32_permute_even = vlddw_v32_clone (2, u32_permute_even, 0);
	v32_permute_odd = vlddw_v32_clone (2, u32_permute_odd, 0);
	v32_permute_eo = vlddw_v32_clone (2, u32_permute_eo, 0);
	vprex_even = vcopy_imm16_8_vpr((SINT16)0x3333);
	vprex_odd = vcopy_imm16_8_vpr((SINT16)0xCCCC);

    v32_1 = vclr_v32(8);
    v32_2 = vclr_v32(8);
    //v32_3 = vclr_v32(8);
    v32_4 = vclr_v32(8); 
    v32_5 = vclr_v32(8);
    v32_6 = vclr_v32(8);
    //v32_7 = vclr_v32(8);
    v32_8 = vclr_v32(8);     
	for(s32_iloop = 0; s32_iloop < (u32_length>>4); s32_iloop++)
#pragma dsp_ceva_unroll = 2
#pragma dsp_ceva_trip_count_min = 1
	{
		v32_data = vlddw_v32 (8, stp_data_in + 16*s32_iloop, 0);
		v32_h = vlddw_v32 (8, stp_H_in + 16*s32_iloop, 0);
		v32_h2 = vlddw_v32 (8, stp_H_in2 + 16*s32_iloop, 0);

		//out even
		v32_1 = vmpyxp_v32_v32_v32_conj_p(8, v32_data, 0, v32_h, 0, vprex_even);
		v32_2 = vmpyxp_v32_v32_v32_conj_p(8, v32_h2, 0, v32_data, 0, vprex_odd);
		v32_3 = vpermutew_v32_v32 (8, v32_permute_eo, v32_2);
		v32_4 = vadd16_v32_v32_v32_p(8, v32_1, v32_3, vprex_even);

		//out odd
		v32_5 = vmpyxp_v32_v32_v32_conj_p(8, v32_data, 0, v32_h, 0, vprex_odd);
		v32_6 = vmpyxp_v32_v32_v32_conj_pneg_p(8, v32_h2, 0, v32_data, 0, vprex_even);
		v32_7 = vpermutew_v32_v32 (8, v32_permute_eo, v32_6);
		v32_8 = vadd16_v32_v32_v32_p(8, v32_5, v32_7, vprex_odd);

		v32_data = vadd16_v32_v32_v32(8, v32_4, v32_8);

		vstdw_v32(8, v32_data, 0, &s_st_temp_buffer[1][0] + 16*s32_iloop);
	}

	//while(READ_REG32(0x1724FFE0) != 2);
	//WRITE_REG32(0x1724FFE4, 2);	

#ifdef SCFDMA_SWITCH
	/*calculate Y*conj(H)/(H*conj(H) + noise_pwr)*/
	ks_s16v_div_s16v((SINT16_PTR)s_st_temp_buffer[1], (SINT16_PTR)s_st_H_mod, u32_length, (SINT16_PTR)s_st_temp_buffer[0]);
	#if 0
	if(g_u32_tx_ant_1st_id)
	{
		//1t1r, tx ant is ant 2 , need do data reorder
		dl_data_sfbc_reorder((SINT32_PTR)s_st_temp_buffer, SC_NUM);
	}
	#endif
	#if 1
	//fftshift
	for(s32_iloop = 0; s32_iloop < ((SINT32)HALF_SC_NUM>>4); s32_iloop++)
	{
		v32_1 = vlddw_v32 (8, ((SINT32_PTR)s_st_temp_buffer) + 16*s32_iloop, 0);
		vstdw_v32 (8, v32_1, 0, ((SINT32_PTR)s_st_temp_buffer) + SC_NUM + 16*s32_iloop);
	}
	v32_1 = vlddw_v32 (8, ((SINT32_PTR)s_st_temp_buffer) + 16*s32_iloop, 0);
	vstdw_v32 (8, v32_1, 0, ((SINT32_PTR)s_st_temp_buffer) + SC_NUM + 16*s32_iloop);

    /*idft(1200) transform into time domain*/
	ks_idft1200(((SINT32_PTR)s_st_temp_buffer + HALF_SC_NUM), stp_equ_out, s_st_temp_buffer[1], s32_shift_amt_1);

	#else
    /*dft(1200) transform into freq domain*/
    dft_ceva_algo_1200(SC_NUM, s32_shift_amt_1, 1, ((SINT32_PTR)s_st_temp_buffer), stp_equ_out, s_st_temp_buffer[1]);
	#endif
	//_ks_shift_s16v((SINT16_PTR)stp_equ_out, (u32_length << 1), 1, (SINT16_PTR)s_st_temp_buffer[1]);
    //_ks_cxv_mul_cx(s_st_temp_buffer[1], SC_NUM, &st_dft_600_factor, 0, stp_equ_out); 
    _ks_cxv_mul_cx(stp_equ_out, SC_NUM, &st_dft_600_factor, 0, stp_equ_out); 
	//OAL_IRQ_RESTORE;
#else
	/*calculate Y*conj(H)/(H*conj(H) + noise_pwr)*/
	_ks_shift_s16v((SINT16_PTR)s_st_temp_buffer[1], (u32_length << 1), 1, (SINT16_PTR)s_st_temp_buffer[0]);
	ks_s16v_div_s16v((SINT16_PTR)s_st_temp_buffer[0], (SINT16_PTR)s_st_H_mod, u32_length, (SINT16_PTR)stp_equ_out);	
	//ks_s16v_div_s16v((SINT16_PTR)s_st_temp_buffer[1], (SINT16_PTR)s_st_H_mod, u32_length, (SINT16_PTR)stp_equ_out);	
#endif

	//while(READ_REG32(0x1724FFE0) != 4);
	//WRITE_REG32(0x1724FFE4, 4);    

    
}

OAL_ITCM_CODE_SECTION
VOID dl_equalize_2t2r(complex_s16_t* stp_H_in, complex_s16_t* stp_data_in, complex_s16_t* stp_data_in2, complex_s16_t *stp_equ_out, UINT32 u32_length) //length must be aligned with 16
{
    SINT32 s32_iloop;
    UINT32 u32_permute_even[2] = {0xDC985410, 0x0};
    UINT32 u32_permute_odd[2] = {0xFEBA7632, 0x0};
    UINT32 u32_permute_eo[2] = {0x54761032, 0xDCFE98BA};

    SINT32 s32_shift_amt_1[6] = {(UINT32)MODV_SV1, (UINT32)MODV_SV1, (UINT32)MODV_SV1, (UINT32)MODV_SV1,(UINT32)MODV_SV0};
    SINT32 s32_shift_amt_2[6] = {(UINT32)MODV_SV0, (UINT32)MODV_SV0, (UINT32)MODV_SV0, (UINT32)MODV_SV0,(UINT32)MODV_SV0};
    //complex_s16_t st_dft_600_factor = {28050, 0};// {28111, 0}; // factor=512/600
    complex_s16_t st_dft_600_factor = {30119, 0}; // factor=512/600
    OAL_IRQ_SAVE_AREA;

    vec_t  v32_1;
    vec_t  v32_2;
    vec_t  v32_3;
    vec_t  v32_4;
    vec_t  v32_data_even;
    vec_t  v32_data_odd;
    vec_t  v32_data_even2;
    vec_t  v32_data_odd2;
    vec_t  v32_tmp; 
    vec_t  v32_tmp2;     
	vec_t  v32_permute_even;
	vec_t  v32_permute_odd;
	vec_t  v32_permute_eo;	
	vec40_t  v32_data;
    vec40_t  v32_h;
    vec40_t  v32_h2;	
	vprex_t vprex_even;
	vprex_t vprex_odd;

	//KS_LOG(0, 0xA223, (UINT16)s32_noise_pwr);
	//DL_GET_TMR2_START;

	#if 0
	if(g_test_flg)
	{
		OAL_IRQ_DISABLE_ALL;
	}
	#endif

	v32_permute_even = vlddw_v32_clone (2, u32_permute_even, 0);
	v32_permute_odd = vlddw_v32_clone (2, u32_permute_odd, 0);
	v32_permute_eo = vlddw_v32_clone (2, u32_permute_eo, 0);
	vprex_even = vcopy_imm16_8_vpr((SINT16)0x3333);
	vprex_odd = vcopy_imm16_8_vpr((SINT16)0xCCCC);
    v32_data_even = vclr_v32(8);
    v32_data_odd = vclr_v32(8);
    v32_data_even2 = vclr_v32(8);
    v32_data_odd2 = vclr_v32(8);
    v32_1 = vclr_v32(8); 
    v32_2 = vclr_v32(8);
 
	for(s32_iloop = 0; s32_iloop < (u32_length>>4); s32_iloop++)
    #pragma dsp_ceva_unroll = 2
    #pragma dsp_ceva_trip_count_min = 1	
	{
		v32_data = vlddw_v32 (8, stp_data_in + 16*s32_iloop, 0);
		v32_h = vlddw_v32 (8, stp_H_in + 16*s32_iloop, 0);
		v32_h2 = vlddw_v32 (8, stp_H_in + SC_NUM + 16*s32_iloop, 0);
		//out even
		v32_data_even = vmpyxp_v32_v32_v32_conj_p(8, v32_data, 0, v32_h, 0, vprex_even);
		v32_data_odd = vmpyxp_v32_v32_v32_conj_p(8, v32_h2, 0, v32_data, 0, vprex_odd);
		v32_tmp = vpermutew_v32_v32 (8, v32_permute_eo, v32_data_odd);
		v32_1 = vadd16_v32_v32_v32_p(8, v32_data_even, v32_tmp, vprex_even);
		//out odd
		v32_data_odd2 = vmpyxp_v32_v32_v32_conj_p(8, v32_data, 0, v32_h, 0, vprex_odd);
		v32_data_even2 = vmpyxp_v32_v32_v32_conj_pneg_p(8, v32_h2, 0, v32_data, 0, vprex_even);
		v32_tmp2 = vpermutew_v32_v32 (8, v32_permute_eo, v32_data_even2);
		v32_2 = vadd16_v32_v32_v32_p(8, v32_data_odd2, v32_tmp2, vprex_odd);
		v32_3 = vadd16_v32_v32_v32(8, v32_1, v32_2);

		v32_data = vlddw_v32 (8, stp_data_in2 + 16*s32_iloop, 0);
		v32_h = vlddw_v32 (8, stp_H_in + SC_NUM * 2 + 16*s32_iloop, 0);
		v32_h2 = vlddw_v32 (8, stp_H_in + SC_NUM * 3 + 16*s32_iloop, 0);
		//out even
		v32_data_even = vmpyxp_v32_v32_v32_conj_p(8, v32_data, 0, v32_h, 0, vprex_even);
		v32_data_odd = vmpyxp_v32_v32_v32_conj_p(8, v32_h2, 0, v32_data, 0, vprex_odd);
		v32_tmp = vpermutew_v32_v32 (8, v32_permute_eo, v32_data_odd);
		v32_1 = vadd16_v32_v32_v32_p(8, v32_data_even, v32_tmp, vprex_even);
		//out odd
		v32_data_odd2 = vmpyxp_v32_v32_v32_conj_p(8, v32_data, 0, v32_h, 0, vprex_odd);
		v32_data_even2 = vmpyxp_v32_v32_v32_conj_pneg_p(8, v32_h2, 0, v32_data, 0, vprex_even);
		v32_tmp2 = vpermutew_v32_v32 (8, v32_permute_eo, v32_data_even2);
		v32_2 = vadd16_v32_v32_v32_p(8, v32_data_odd2, v32_tmp2, vprex_odd);
		v32_4 = vadd16_v32_v32_v32(8, v32_1, v32_2);
		
		v32_data = vadd16_v32_v32_v32(8, v32_3, v32_4);
		vstdw_v32(8, v32_data, 0, &s_st_temp_buffer[1][0] + 16*s32_iloop);
	}


	#if 0
	if(g_test_flg)
	{
		while(READ_REG32(0x1724FFE0) != 2);
		WRITE_REG32(0x1724FFE4, 2);	
	}
	#endif
	
#ifdef SCFDMA_SWITCH
    /*calculate Y*conj(H)/(H*conj(H) + noise_pwr)*/
	ks_s16v_div_s16v((SINT16_PTR)s_st_temp_buffer[1], (SINT16_PTR)s_st_H_mod, u32_length, (SINT16_PTR)s_st_temp_buffer[0]);
	#if 0
	if(g_test_flg)
	{
		while(READ_REG32(0x1724FFE0) != 3);
		WRITE_REG32(0x1724FFE4, 3);	
	}
	#endif
#if 1
	//fftshift
	for(s32_iloop = 0; s32_iloop < ((SINT32)HALF_SC_NUM>>4); s32_iloop++)
	{
		v32_1 = vlddw_v32 (8, ((SINT32_PTR)s_st_temp_buffer) + 16*s32_iloop, 0);
		vstdw_v32 (8, v32_1, 0, ((SINT32_PTR)s_st_temp_buffer) + SC_NUM + 16*s32_iloop);
	}
	v32_1 = vlddw_v32 (8, ((SINT32_PTR)s_st_temp_buffer) + 16*s32_iloop, 0);
	vstdw_v32 (8, v32_1, 0, ((SINT32_PTR)s_st_temp_buffer) + SC_NUM + 16*s32_iloop);

    /*idft(1200) transform into time domain*/
	ks_idft1200(((SINT32_PTR)s_st_temp_buffer[0] + HALF_SC_NUM), stp_equ_out, s_st_temp_buffer[1], s32_shift_amt_1);
#else
    /*dft(1200) transform into freq domain*/
    dft_ceva_algo_1200(SC_NUM, s32_shift_amt_1, 1, ((SINT32_PTR)s_st_temp_buffer), stp_equ_out, s_st_temp_buffer[1]);
#endif
	//_ks_shift_s16v((SINT16_PTR)stp_equ_out, (u32_length << 1), 0, (SINT16_PTR)s_st_temp_buffer[1]);
    //_ks_cxv_mul_cx(s_st_temp_buffer[1], SC_NUM, &st_dft_600_factor, 0, stp_equ_out); 
    _ks_cxv_mul_cx(stp_equ_out, SC_NUM, &st_dft_600_factor, 0, stp_equ_out); 
#else
    /*calculate Y*conj(H)/(H*conj(H) + noise_pwr)*/
	_ks_shift_s16v((SINT16_PTR)s_st_temp_buffer[1], (u32_length << 1), 1, (SINT16_PTR)s_st_temp_buffer[0]);
	ks_s16v_div_s16v((SINT16_PTR)s_st_temp_buffer[0], (SINT16_PTR)s_st_H_mod, u32_length, (SINT16_PTR)stp_equ_out);
#endif

	#if 0
	if(g_test_flg)
	{
		while(READ_REG32(0x1724FFE0) != 4);
		WRITE_REG32(0x1724FFE4, 4);	
	}
	#endif

	//DL_GET_TMR2_END;
	//DL_PRINT_TMR2_DURATION(0xA41B);

	//g_test_flg += 1;
    
}


#define MCS_OFFSET_INVALID (100)
#define MCS_UP_THRES       (32113)  //Q15
#define MCS_DOWN_THRES     (31130)  //Q15

OAL_ITCM_CODE_SECTION
VOID dl_data_dec_rpt()
{
	UINT16 downlink_pkg_size = 0;
	DOWNLINK_PHY_HEADER_TYPE	*downlink_phy_ptr = NULL_PTR;
	oal_msg_t *stp_msg;
	oal_hls_msg_t* stp_hls_msg;
	THls_Phy_buf_info mbuf_info;
	UINT32 *pkt_addr_ptr;
	UINT32_PTR u32p_temp = NULL_PTR;
	UINT8_PTR u8p_temp = NULL_PTR;
	UINT16 msg_len = sizeof(oal_hls_msg_t) - sizeof(VOID_PTR) + 8;
	SINT32 s32_frame_no, s32_rtc_value = 0;
	UINT32 u32_rpt_type;
	node_info_t *stp_node_info = NULL_PTR;
	node_info_tbl_t *stp_node_info_tbl = &s_st_node_info_tbl;


    OAL_IRQ_SAVE_AREA;

	OAL_IRQ_DISABLE;
    u32_rpt_type = g_u8_rpt_type;
    g_u8_rpt_type = OAL_FALSE;
	OAL_IRQ_RESTORE;

	if(!u32_rpt_type)
	{
		//return;
		OAL_ASSERT(0, "");
	}
	KS_LOG(OAL_TRACE_LEVEL_0 , 0xAF20 , (UINT16)u32_rpt_type);
	g_zzw_ch_status.mcs_offset = MCS_OFFSET_INVALID;
	if((u32_rpt_type&0xF) EQ 1)
	{
		stp_node_info = (node_info_t*)node_tbl_find(g_zzw_ch_status.node_id);

		OAL_ASSERT((SINT32)s_s32_sync_pos_min < (SINT32)RX_BUF_LEN, "");
        
		//KS_LOG(OAL_TRACE_LEVEL_0 , 0xAF21 , (UINT16)s_st_algo2l1cc_ctrl_info.st_ctrl_info.u16_mac4_len);
		KS_LOG(OAL_TRACE_LEVEL_0 , 0xAF23 , (UINT16)g_u32_ics_frm_cnt_rpt);
		KS_LOG(OAL_TRACE_LEVEL_0 , 0xAF24 , (UINT16)s_st_algo2l1cc_ctrl_info.st_ctrl_info.u32_frame_num);

		//s_st_algo2l1cc_ctrl_info.st_ctrl_info.u16_frame_num = 1;

        s32_frame_no  = (s_s32_frame_num_rpt + g_u32_ics_frm_cnt_rpt);// & 0x7FF;
        s32_frame_no  = (s32_frame_no - (s_st_algo2l1cc_ctrl_info.st_ctrl_info.u32_frame_num));// & 0x7FF;
        //s32_rtc_value = 568 + 576 - ((SINT32)s_s32_sync_position % 30720);
        #ifdef FUN_OTD
        #if 0
        if(stp_node_info_tbl->u8_otd_adj_flg)
        {
        	//otd调整期间不上报rtc
			return;
        }
        else
        #endif
        {
        	if(stp_node_info_tbl->u32_ref_bmp & (1 << (g_zzw_ch_status.node_id - 1)))
        	{
				KS_LOG(OAL_TRACE_LEVEL_0 , 0xAF2B , (UINT16)stp_node_info_tbl->u32_ref_bmp);
				
        		if(stp_node_info EQ NULL_PTR)
        		{
        			//节点表清空状态
        			return;
        		}
        		else
        		{
					KS_LOG(OAL_TRACE_LEVEL_0 , 0xAF2B , (UINT16)stp_node_info->u16_otd_rpt_fn);
        			if((UINT16)((UINT16)(s_st_algo2l1cc_ctrl_info.st_ctrl_info.u32_frame_num & 0xFFFF) - stp_node_info->u16_otd_rpt_fn) >= (UINT16)0x7FF)
        			{
        				//otd rpt
		        		s32_rtc_value = CCA_POS_IDEAL - stp_node_info->s16_otd - ((SINT32)s_s32_sync_pos_min); 
						KS_LOG(OAL_TRACE_LEVEL_0 , 0xAF28 , (UINT16)stp_node_info->s16_otd);
						stp_node_info->u16_otd_rpt_fn = (SINT16)(s_st_algo2l1cc_ctrl_info.st_ctrl_info.u32_frame_num & 0xFFFF);
					}
					else
					{
		        		s32_rtc_value = 0; 
					}
				}
        	}
        	else
        	{
        		s32_rtc_value = CCA_POS_IDEAL - stp_node_info_tbl->s16_otd - ((SINT32)s_s32_sync_pos_min);        
        	}
        }
        #else
        s32_rtc_value = CCA_POS_IDEAL  - ((SINT32)s_s32_sync_pos_min);
        #endif

        if(s32_rtc_value > 15360)
        {
            s32_frame_no  = (s32_frame_no - 1);// & 0x7FF;
            s32_rtc_value -= 30720;
        }
        else if(s32_rtc_value < -15360)
        {
            s32_frame_no  = (s32_frame_no + 1);// & 0x7FF;
            s32_rtc_value += 30720;
        }
		KS_LOG(OAL_TRACE_LEVEL_0 , 0xAF25 , (UINT16)s32_frame_no);
		KS_LOG(OAL_TRACE_LEVEL_0 , 0xAF26 , (UINT16)s32_rtc_value);

		#if RTC_ADJ
		if(s_u32_rtc_adj_wait_cnt)
		{
			s32_rtc_value = 0;
		}
		#endif

		#if 0
		if(cp1_xc4500_def_flag == 1)
		{	  
			ks_oal_mailbox_send_direct((UINT32)KS_CPU_CP1_X1643, (UINT32)KS_CPU_CP1_XC4500, (UINT32)SCFDE_4500_SEND_RTC2_1643, (UINT32)((UINT16)s32_frame_no << 16) | (UINT16)s32_rtc_value);
		}
		else
		{
			ks_oal_mailbox_send_direct((UINT32)KS_CPU_CP0_X1643, (UINT32)KS_CPU_CP0_XC4500, (UINT32)SCFDE_4500_SEND_RTC2_1643, (UINT32)((UINT16)s32_frame_no << 16) | (UINT16)s32_rtc_value);
		}

		#else
        pkt_addr_ptr = (UINT8 *)to_cp_hs5.get_mbuf_ptr();
        downlink_pkg_size = s_st_algo2l1cc_ctrl_info.st_ctrl_info.u16_mac1_len + sizeof(DOWNLINK_PHY_HEADER_TYPE);

        ks_oal_mem_copy((UINT8 *)pkt_addr_ptr + sizeof(ZZW_CHANNEL_STATUS_INDICATION_T) + sizeof(DOWNLINK_PHY_HEADER_TYPE), (VOID_PTR)L1CC_BB_SCFDE_DEC_DATA_BUF_START_ADDR, downlink_pkg_size);

        stp_msg = OAL_MSG_CREAT((msg_len + 1) >> 1);
        OAL_ASSERT(NULL_PTR != stp_msg, "ks_oal_msg_create failure\n");

        stp_msg->msg_primitive_id = MSG_L1_HLS_PRIMITIVE_ID;
        stp_msg->u16_mode_type    = 3;//FLIGHT_MODE;
        stp_msg->u16_standby_id   = 0;//STANDBY_0;

        stp_hls_msg = (oal_hls_msg_t*)&(stp_msg->vp_msg_body);
        stp_hls_msg->u16_msg_size   = msg_len;
        stp_hls_msg->b8_src_id      = 10;
        stp_hls_msg->u16_pdu_size   = 0;
        stp_hls_msg->u8_phy_task_id = 2;
        stp_hls_msg->u32_msg_id     = ZZW_MACREQ_L2H_PACKAGE;
        stp_hls_msg->b8_standby_id  = 0;//STANDBY_0
        stp_hls_msg->b8_mode_type   = 3;//FLIGHT_MODE;

        u8p_temp = (UINT8_PTR)pkt_addr_ptr;

		KS_LOG(OAL_TRACE_LEVEL_0 , 0xAF27 , (UINT16)g_zzw_ch_status.node_id);
		KS_LOG(OAL_TRACE_LEVEL_0 , 0xAF29 , (UINT16)s_s32_frame_num);
		KS_LOG(OAL_TRACE_LEVEL_0 , 0xAF2A , (UINT16)s_s32_frame_num_rpt);

		#ifdef MCS_AUTO_ADJ
		if(stp_node_info)//stp_node_info NEQ NULL_PTR, which means node exists already
		{
			if(stp_node_info->u16_dec_cnt > 200)
			{
				KS_LOG(OAL_TRACE_LEVEL_0 , 0xAF2F , (UINT16)stp_node_info->s8_mcs_direction);

				if(((UINT32)(stp_node_info->u16_suc_cnt) << 15) < stp_node_info->u16_dec_cnt * MCS_DOWN_THRES)
				{
					//对包率小于0.9，下调mcs
					stp_node_info->s8_mcs_offset -= 1;
					if(stp_node_info->s8_mcs_direction EQ 0)
					{
						stp_node_info->u8_mcs_punish = 0;
					}
					stp_node_info->s8_mcs_direction = 0;
				}
				else if(((UINT32)(stp_node_info->u16_suc_cnt) << 15) > stp_node_info->u16_dec_cnt * MCS_UP_THRES)
				{
					//对包率大于0.95，上调mcs
					if((stp_node_info->u8_mcs < (ZZW_MAX_MCS_NUM - 1)) && \
							(stp_node_info->u8_mcs + stp_node_info->s8_mcs_offset < (ZZW_MAX_MCS_NUM - 1)))
					{
						if((stp_node_info->s8_mcs_direction EQ 0) )
						{	
							//刚刚下调mcs，为了防止mcs乒乓切换，增加惩罚保护
							//if(stp_node_info->u8_mcs_punish < 5))
							{
								stp_node_info->u8_mcs_punish += 1;
							}
							if(stp_node_info->u8_mcs_punish > 30) //temp here
							{
								stp_node_info->u8_mcs_punish = 0;
							}
						}
						else
						{
							stp_node_info->u8_mcs_punish = 0;
						}
						if(stp_node_info->u8_mcs_punish < 3) //temp here
						{
							stp_node_info->s8_mcs_offset += 1;
							stp_node_info->s8_mcs_direction = 1;
						}
					}
				}
				KS_LOG(OAL_TRACE_LEVEL_0 , 0xAF2C , (UINT16)stp_node_info->u16_dec_cnt);
				KS_LOG(OAL_TRACE_LEVEL_0 , 0xAF2D , (UINT16)stp_node_info->u16_suc_cnt);
				KS_LOG(OAL_TRACE_LEVEL_0 , 0xAF2E , (UINT16)stp_node_info->u8_mcs);
				KS_LOG(OAL_TRACE_LEVEL_0 , 0xAF2F , (UINT16)stp_node_info->u8_mcs_punish);

				stp_node_info->u16_dec_cnt = 0;
				stp_node_info->u16_suc_cnt = 0;
				stp_node_info->u8_mcs = 0;

				g_zzw_ch_status.mcs_offset = stp_node_info->s8_mcs_offset;
				KS_LOG(OAL_TRACE_LEVEL_0 , 0xAF2B , (UINT16)g_zzw_ch_status.mcs_offset);
			}
		}
		#endif
		#if 0
        u8p_temp[0] = g_zzw_ch_status.node_id;
        u8p_temp[1] = g_zzw_ch_status.txpwr;    //txpwr
        u8p_temp[2] = g_zzw_ch_status.rssi;     //rssi
        u8p_temp[3] = g_zzw_ch_status.gain;     //gain
        u8p_temp[4] = g_zzw_ch_status.pathloss; //pathloss
        u8p_temp[5] = g_zzw_ch_status.snr;      //snr
        u8p_temp[6] = g_zzw_ch_status.mcs;      //mcs
        u8p_temp[7] = 0x1;
		#else
		g_zzw_ch_status.data_valid = 1;
		ks_oal_mem_copy((VOID_PTR)u8p_temp, (VOID_PTR)&g_zzw_ch_status, sizeof(ZZW_CHANNEL_STATUS_INDICATION_T));
		#endif
        downlink_phy_ptr = (DOWNLINK_PHY_HEADER_TYPE *)(((UINT8_PTR)pkt_addr_ptr) + sizeof(ZZW_CHANNEL_STATUS_INDICATION_T));
        downlink_phy_ptr->mcs = g_zzw_ch_status.mcs;
        downlink_phy_ptr->type = s_st_algo2l1cc_ctrl_info.st_ctrl_info.u16_mac0_len;
        downlink_phy_ptr->frame_len = s_st_algo2l1cc_ctrl_info.st_ctrl_info.u16_mac1_len;

        u32p_temp = (UINT32_PTR)(((UINT8_PTR)pkt_addr_ptr) + sizeof(ZZW_CHANNEL_STATUS_INDICATION_T) + sizeof(DOWNLINK_PHY_HEADER_TYPE));
      
        u32p_temp[1] = s32_frame_no;
        u32p_temp[2] = s32_rtc_value;

		KS_LOG(OAL_TRACE_LEVEL_0 , 0xAF30 , (UINT16)g_zzw_ch_status.rssi_sub[0]);
		KS_LOG(OAL_TRACE_LEVEL_0 , 0xAF31 , (UINT16)g_zzw_ch_status.pathloss_sub[0]);

		KS_LOG(OAL_TRACE_LEVEL_0 , 0xAF32 , (UINT16)g_zzw_ch_status.rssi_sub[1]);
		KS_LOG(OAL_TRACE_LEVEL_0 , 0xAF33 , (UINT16)g_zzw_ch_status.pathloss_sub[1]);

        to_cp_hs5.send_mbuf(stp_msg, ARM2DSP_DL_DATA_BUFSIZE);

        s_st_debug_info.s32_frame_no = s32_frame_no;
        s_st_debug_info.s32_rtc_time = s32_rtc_value;	
        //OAL_ASSERT(0, "");
        #endif
	}
	else if((u32_rpt_type&0xF) EQ 2)
	{
		//OAL_ASSERT(((*(VUINT32_PTR)L1CC_BB_SCFDE_DEC_DATA_BUF_START_ADDR EQ 0xA5A5A5A5) || (*(VUINT32_PTR)L1CC_BB_SCFDE_DEC_DATA_BUF_START_ADDR EQ 0x0)), "");
		xc4500_phych_proc_data((void *)L1CC_BB_SCFDE_DEC_DATA_BUF_START_ADDR, 1, 0, 0, \
			s_st_algo2l1cc_ctrl_info.st_ctrl_info.u16_mac0_len, s_st_algo2l1cc_ctrl_info.st_ctrl_info.u16_mac1_len);
	}
	else if((u32_rpt_type&0xF) EQ 3)
	{
		//KS_LOG(OAL_TRACE_LEVEL_0 , 0xAF22 , (UINT16)s_st_algo2l1cc_ctrl_info.st_ctrl_info.u16_mac5_len);
		pkt_addr_ptr = (UINT8 *)to_cp_hs5.get_mbuf_ptr();

		stp_msg = OAL_MSG_CREAT((msg_len + 1) >> 1);
		OAL_ASSERT(NULL_PTR != stp_msg, "ks_oal_msg_create failure\n");

		stp_msg->msg_primitive_id = MSG_L1_HLS_PRIMITIVE_ID;
		stp_msg->u16_mode_type    = 3;//FLIGHT_MODE;
		stp_msg->u16_standby_id   = 0;//STANDBY_0;

		stp_hls_msg = (oal_hls_msg_t*)&(stp_msg->vp_msg_body);
		stp_hls_msg->u16_msg_size   = msg_len;
		stp_hls_msg->b8_src_id      = 10;
		stp_hls_msg->u16_pdu_size   = 0;
		stp_hls_msg->u8_phy_task_id = 2;
		stp_hls_msg->u32_msg_id     = ZZW_MACREQ_L2H_PACKAGE;
		stp_hls_msg->b8_standby_id  = 0;//STANDBY_0
		stp_hls_msg->b8_mode_type   = 3;//FLIGHT_MODE;

		u8p_temp = (UINT8_PTR)pkt_addr_ptr;     
		#if 0
		u8p_temp[0] = g_zzw_ch_status.node_id;
		u8p_temp[1] = g_zzw_ch_status.txpwr;    //txpwr
		u8p_temp[2] = g_zzw_ch_status.rssi;     //rssi
		u8p_temp[3] = g_zzw_ch_status.gain;     //gain
		u8p_temp[4] = g_zzw_ch_status.pathloss; //pathloss
		u8p_temp[5] = g_zzw_ch_status.snr;      //snr
		u8p_temp[6] = g_zzw_ch_status.mcs;      //mcs
		if(u32_rpt_type & (1 << 16))
		{
			//1st cb is crc good
			u8p_temp[7] = 2;
		}
		else
		{
			//1st cb is crc bad
			u8p_temp[7] = 0;
		}
		#else
		if(u32_rpt_type & (1 << 16))
		{
			//1st cb is crc good
			g_zzw_ch_status.data_valid = 2;
		}
		else
		{
			//1st cb is crc bad
			g_zzw_ch_status.data_valid = 0;
		}
		ks_oal_mem_copy((VOID_PTR)u8p_temp, (VOID_PTR)&g_zzw_ch_status, sizeof(ZZW_CHANNEL_STATUS_INDICATION_T));
		#endif
		downlink_phy_ptr = (DOWNLINK_PHY_HEADER_TYPE *)(((UINT8_PTR)pkt_addr_ptr)+sizeof(ZZW_CHANNEL_STATUS_INDICATION_T));						
		downlink_phy_ptr->mcs = g_zzw_ch_status.mcs;
		downlink_phy_ptr->type = s_st_algo2l1cc_ctrl_info.st_ctrl_info.u16_mac0_len;
		downlink_phy_ptr->frame_len = s_st_algo2l1cc_ctrl_info.st_ctrl_info.u16_mac1_len;

		to_cp_hs5.send_mbuf(stp_msg, ARM2DSP_DL_DATA_BUFSIZE);
	}
	else
	{	
		OAL_ASSERT(0, "");
	}
}

#if 0
OAL_ITCM_CODE_SECTION
VOID dl_H_intp(IN complex_s16_t* stp_H_in_0, IN complex_s16_t* stp_H_in_1,  IN UINT32 u32_sym_idx_start, UINT32 u32_sym_num, OUT complex_s16_t *stp_H_intp_out)
{
    SINT32 s32_iloop;
    SINT16 s16_scale;

    _ks_cxv_sub_cxv(stp_H_in_1, stp_H_in_0, SC_NUM, s_st_temp_buffer[0]);

    for(s32_iloop = u32_sym_idx_start; s32_iloop < (u32_sym_idx_start + u32_sym_num); s32_iloop++)
    {
        s16_scale = (s32_iloop + 1)*32767/(PILOT_INSERT_GAP + 1);

        _ks_s16v_mul_s16((SINT16_PTR)s_st_temp_buffer[0], (SINT16)SC_NUM*2, s16_scale, (SINT16_PTR)s_st_temp_buffer[1]);
        _ks_cxv_add_cxv(stp_H_in_0, s_st_temp_buffer[1], SC_NUM, &stp_H_intp_out[(s32_iloop - u32_sym_idx_start) * SC_NUM]);
    }
}
#else
OAL_ITCM_CODE_SECTION
VOID dl_H_intp(IN complex_s16_t* stp_H_in_0, IN complex_s16_t* stp_H_in_1,  IN UINT32 u32_sym_idx, OUT complex_s16_t *stp_H_intp_out)
{

    SINT16 s16_scale;

    _ks_cxv_sub_cxv(stp_H_in_1, stp_H_in_0, SC_NUM, s_st_temp_buffer[0]);

    //s16_scale = (u32_sym_idx + 1)*32767/(PILOT_INSERT_GAP + 1);
    s16_scale = s_dl_h_intp_scale[u32_sym_idx]; 

    _ks_s16v_mul_s16((SINT16_PTR)s_st_temp_buffer[0], (SINT16)SC_NUM*2, s16_scale, (SINT16_PTR)s_st_temp_buffer[1]);
    _ks_cxv_add_cxv(stp_H_in_0, s_st_temp_buffer[1], SC_NUM, &stp_H_intp_out[0]);

}

OAL_ITCM_CODE_SECTION
VOID dl_H_intp2x2(IN complex_s16_t* stp_H_in_0, IN complex_s16_t* stp_H_in_1,  IN UINT32 u32_sym_idx,  IN UINT32 u32_gen_sub_tbl, OUT complex_s16_t *stp_H_intp_out)
{

    SINT16 s16_scale;
    SINT32 s32_iloop;    
	vec_t  v32_1;
	vec_t  v32_2;
	vec_t  v32_3;
	vec_t  v32_4;
	coef_t vcoef;

	if(u32_gen_sub_tbl)
	{
    	_ks_cxv_sub_cxv(stp_H_in_1, stp_H_in_0, SC_NUM << 2, s_st_H_sub_tbl[0]);
	}
    //s16_scale = (u32_sym_idx + 1)*32767/(PILOT_INSERT_GAP + 1);
    s16_scale = s_dl_h_intp_scale[u32_sym_idx]; 
	#if 1
    _ks_s16v_mul_s16((SINT16_PTR)s_st_H_sub_tbl[0], (SINT16)(SC_NUM << 3), s16_scale, (SINT16_PTR)s_st_temp_buffer[0]);
    _ks_cxv_add_cxv(stp_H_in_0, s_st_temp_buffer[0], SC_NUM << 2, &stp_H_intp_out[0]);
	#else
    vcoef = vlddw_c32_clone_1dw (&s16_scale);

	__asm__("vpush {dw} modv0");
	__asm__("vpush {dw} modv2");
	__asm__("nop");
	__asm__("nop");
	__asm__("push {dw} a8");
	__asm__("mov #0x00000C00, a8");/* 功能寄存器压栈 */
	__asm__("nop");
	__asm__("nop");
	__asm__("vmova a8, modv0");/* 功能寄存器设置 */
	
	__asm__("mov #0x00800C20, a8");/* 功能寄存器压栈 */
	__asm__("nop");
	__asm__("nop");
	__asm__("vmova a8, modv2");/* 功能寄存器设置 */
	
	__asm__("pop {dw} a8");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");

	
	for(s32_iloop = 0; s32_iloop < (SC_NUM >> 2); s32_iloop++)
	#pragma dsp_ceva_unroll = 4
	#pragma dsp_ceva_trip_count_min = 2
	{
	
		v32_1 = vlddw_v32(8, s_st_H_sub_tbl[0] + 16*s32_iloop,0); //Vy1[0..7]<-complex_data
		v32_2 = vlddw_v32(8, stp_H_in_0 + 16*s32_iloop,0); //Vy1[0..7]<-complex_data

		v32_3 = vmpyp_v32_c32_v32 (8, v32_1, 0, LOW, vcoef, LOW);
		v32_4 = vadd16_v32_v32_v32 (8, v32_2, v32_3);
	
		vstdw_v32(8, v32_4, 0, stp_H_intp_out + 16*s32_iloop);
	}
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("vpop {dw} modv2");
	__asm__("vpop {dw} modv0");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");   
	#endif
}


#endif
#if H_CALC_DFT
OAL_ITCM_CODE_SECTION
SINT32 dl_H_calc(complex_s16_t* stp_pilot_in, complex_s16_t* stp_H_vec_out, SINT32_PTR s32p_sig_pwr, SINT32_PTR s32p_noise_pwr, UINT32 u32_ant_id)
{
    ks_fft_para_t st_fft_para;
    SINT32 s32_shift_amt_600[5] = {(UINT32)MODV_SV2, (UINT32)MODV_SV2, (UINT32)MODV_SV2, (UINT32)MODV_SV1, (UINT32)MODV_SV0};
    SINT32 s32_shift_amt_1200[6] = {(UINT32)MODV_SV0, (UINT32)MODV_SV0, (UINT32)MODV_SV0, (UINT32)MODV_SV0, (UINT32)MODV_SV0, (UINT32)MODV_SV0};
    
    UINT32 u32_permute[2] = {0xDC985410, 0xFEBA7632};
    SINT32 s32_signal_pwr, s32_iloop;
    SINT32 s32_snr = 0;
    SINT32 s32_edge_len = 50;
    //complex_s16_t st_dft_600_factor = {27961, 0}; // factor=512/600
    #if H_CALC_DFT
    complex_s16_t s_dft_600_factor = {27961, 0}; // factor=512/600    
    #else
    complex_s16_t st_dft_600_factor = {23170, 0}; // factor=512/600
    #endif
	//OAL_IRQ_SAVE_AREA;
    SINT32 s32_peak, s32_noise, s32_threshold;

    vec_t v32_1;
    vec_t v32_2;
    vec_t v32_3;
    vec_t v32_4;
    vec_t v32_5;
    coef_t vcoef;
    vprex_t v_prex;

	//DL_GET_TMR2_START;

    /*transform into freq domain*/
    st_fft_para.s16cp_indata_addr   = stp_pilot_in;
    st_fft_para.s16cp_outdata_addr  = s_st_fft_out;
    st_fft_para.s16cp_twiddle_addr  = (complex_s16_t*)ceva_fft_lib_fft_2048_twi_table;
    st_fft_para.s32p_shift_amt_addr = s_s32_shift_amt_fft;
    st_fft_para.s16cp_temp_addr     = (complex_s16_t*)(s_st_temp_buffer[1]);

	#if 0
	//if(u32_ant_id)
	{
		OAL_IRQ_DISABLE_ALL;
	}
	#endif
	//#if SCFDMA_SWITCH
	#if 0
    v32_1 = vlddw_v32_clone(2, u32_permute, 0);
    for(s32_iloop = 0; s32_iloop < ((SINT32)HALF_SC_NUM>>4); s32_iloop++)
    {
        v32_2 = vlddw_v32 (8, stp_pilot_in + SYMBOL_LEN - HALF_SC_NUM + 16*s32_iloop + 1 - u32_ant_id, 0); //正频率
        v32_3 = vpermutew_v32_v32 (8, v32_1, v32_2);

        v32_4 = vlddw_v32 (8, stp_pilot_in + 1 + 16*s32_iloop + u32_ant_id, 0); //负频率
        v32_5 = vpermutew_v32_v32 (8, v32_1, v32_4);

        vstdw_v32_concat (4, v32_5, 0, &s_st_temp_buffer[1][s32_iloop*8]);
        vstdw_v32_concat (4, v32_3, 0, &s_st_temp_buffer[1][s32_iloop*8 + QUARTER_SC_NUM]);
    }
    v32_2 = vlddw_v32 (4, stp_pilot_in + SYMBOL_LEN - HALF_SC_NUM + 16*s32_iloop + 1 - u32_ant_id, 0); //正频率
    v32_3 = vpermutew_v32_v32 (8, v32_1, v32_2);

    v32_4 = vlddw_v32 (4, stp_pilot_in + 1 + 16*s32_iloop + u32_ant_id, 0); //负频率
    v32_5 = vpermutew_v32_v32 (8, v32_1, v32_4);

    vstdw_v32_concat (2, v32_5, 0, &s_st_temp_buffer[1][s32_iloop*8]);
    vstdw_v32_concat (2, v32_3, 0, &s_st_temp_buffer[1][s32_iloop*8 + QUARTER_SC_NUM]);    
	#else
    v32_1 = vlddw_v32_clone(2, u32_permute, 0);
    for(s32_iloop = 0; s32_iloop < ((SINT32)HALF_SC_NUM>>4); s32_iloop++)
    {
        v32_2 = vlddw_v32 (8, stp_pilot_in + SYMBOL_LEN - HALF_SC_NUM + 16*s32_iloop + 1 - u32_ant_id, 0); //正频率
        v32_3 = vpermutew_v32_v32 (8, v32_1, v32_2);

        v32_4 = vlddw_v32 (8, stp_pilot_in + 1 + 16*s32_iloop + u32_ant_id, 0); //负频率
        v32_5 = vpermutew_v32_v32 (8, v32_1, v32_4);

        vstdw_v32_concat (4, v32_3, 0, &s_st_temp_buffer[1][s32_iloop*8]);
        vstdw_v32_concat (4, v32_5, 0, &s_st_temp_buffer[1][s32_iloop*8 + QUARTER_SC_NUM]);
    }
    v32_2 = vlddw_v32 (4, stp_pilot_in + SYMBOL_LEN - HALF_SC_NUM + 16*s32_iloop + 1 - u32_ant_id, 0); //正频率
    v32_3 = vpermutew_v32_v32 (8, v32_1, v32_2);

    v32_4 = vlddw_v32 (4, stp_pilot_in + 1 + 16*s32_iloop + u32_ant_id, 0); //负频率
    v32_5 = vpermutew_v32_v32 (8, v32_1, v32_4);

    vstdw_v32_concat (2, v32_3, 0, &s_st_temp_buffer[1][s32_iloop*8]);
    vstdw_v32_concat (2, v32_5, 0, &s_st_temp_buffer[1][s32_iloop*8 + QUARTER_SC_NUM]);

	#endif

    ks_cxv_mul_cxv(s_st_temp_buffer[1], s_st_fd_pilot_seq, HALF_SC_NUM, 2, s_st_temp_buffer[0]);

	#if 0
	//if(u32_ant_id)
	{
		while(READ_REG32(0x1724FFE0) NEQ 0);
		WRITE_REG32(0x1724FFE4, 0);
	}
	#endif
	#if 1
    _ks_shift_s16v((SINT16_PTR)s_st_temp_buffer[0], (HALF_SC_NUM<<1), -2, (SINT16_PTR)s_st_fft_in);

    //dft_ceva_algo(HALF_SC_NUM, s32_shift_amt_600, 1, s_st_fft_in, s_st_fft_out, s_st_temp_buffer[1]); //idft 600 td ls
    ks_idft600(s_st_fft_in, s_st_fft_out, s_st_temp_buffer[1], s32_shift_amt_600);
	#if 0
	//if(u32_ant_id)
	{
		while(READ_REG32(0x1724FFE0) NEQ 1);
		WRITE_REG32(0x1724FFE4, 1);
	}
	#endif
    //dft_ceva_algo_600(HALF_SC_NUM, s32_shift_amt_600, 1, ((SINT16_PTR)s_st_fft_in), s_st_fft_out, s_st_temp_buffer[1]);
	//while(READ_REG32(0x1724FFE0) NEQ 0);
	#if 0
    ks_oal_mem_copy(&s_st_fft_in[0], &s_st_fft_out[0], (s32_edge_len << 2));
    ks_oal_mem_copy(&s_st_fft_in[1200 - s32_edge_len], &s_st_fft_out[600 - s32_edge_len], (s32_edge_len << 2));
    ks_oal_mem_set(&s_st_fft_in[s32_edge_len], 0, ((1200 - s32_edge_len * 2) << 2));
    #else
    ks_cxv_squa_s16(s_st_fft_out, HALF_SC_NUM, s_st_temp_buffer[1]);
    s32_peak = _ks_max_s32v_idx(s_st_temp_buffer[1], &s32_iloop, HALF_SC_NUM);
    s32_noise = ks_cxv_rssi_calc(&s_st_fft_out[267], 64);

    if((s32_peak/100) > s32_noise)
    {
        s32_threshold = (s32_noise<<2) + (s32_noise<<1) + (s32_noise>>2);
    }
    else if((s32_peak/36) > s32_noise)
    {
        s32_threshold = (s32_noise<<1) + (s32_noise>>2);
    }
    else
    {
        s32_threshold = s32_noise;
    }

    vcoef = vlddw_c32_clone_1dw (&s32_threshold);

    for(s32_iloop = 0; s32_iloop < 264; )
    {
        v32_2 = vlddw_v32 (8, &s_st_temp_buffer[1][s32_iloop], 0);
        v_prex = vcmp_v32_c32_vpr_lt (8, v32_2, vcoef);

        v32_3 = vlddw_v32 (8, &s_st_fft_out[s32_iloop], 0);
        v32_3 = vclr_v32_p (8, v_prex);
        vstdw_v32 (8, v32_3, 0, &s_st_fft_in[s32_iloop]);

        s32_iloop += 16;
    }

    for(s32_iloop = 424; s32_iloop < HALF_SC_NUM; )
    {
        v32_2 = vlddw_v32 (8, &s_st_temp_buffer[1][s32_iloop], 0);
        v_prex = vcmp_v32_c32_vpr_lt (8, v32_2, vcoef);

        v32_3 = vlddw_v32 (8, &s_st_fft_out[s32_iloop], 0);
        v32_3 = vclr_v32_p (8, v_prex);
        vstdw_v32 (8, v32_3, 0, &s_st_fft_in[s32_iloop]);

        s32_iloop += 16;
    }
    ks_oal_mem_set(&s_st_fft_in[264], 0, ((424 - 264) << 2));
	#endif
    //ks_cxv_mul_cxv(s_st_fft_out, s_st_window_seq, HALF_SC_NUM, 0, s_st_fft_in); //add td win size=600

    ks_oal_mem_copy(&s_st_fft_in[HALF_SC_NUM/2 + HALF_SC_NUM], &s_st_fft_in[HALF_SC_NUM/2], HALF_SC_NUM*2); //可做优化
    ks_oal_mem_set(&s_st_fft_in[HALF_SC_NUM/2], 0, HALF_SC_NUM*4);

	/*dft(1200) transform into freq domain*/
    ks_dft1200(s_st_fft_in, s_st_fft_out, s_st_temp_buffer[1], s32_shift_amt_1200);
    //_ks_shift_s16v((SINT16_PTR)s_st_fft_out, SC_NUM * 2, 2, (SINT16_PTR)s_st_temp_buffer[1]);
    //_ks_cxv_mul_cx((SINT16_PTR)s_st_temp_buffer[1], SC_NUM, &s_dft_600_factor, 0, stp_H_vec_out); 
	_ks_cxv_mul_cx((SINT16_PTR)s_st_fft_out, SC_NUM, &s_dft_600_factor, 0, stp_H_vec_out);     

	#if 0
	//if(u32_ant_id)
	{
		while(READ_REG32(0x1724FFE0) NEQ 2);
		WRITE_REG32(0x1724FFE4, 2);
	}
	#endif
	#if 0
    for(s32_iloop = 0; s32_iloop < EDGE_SUBCARRIER_NUM; )
    {
        stp_H_vec_out[s32_iloop].s16_re = (s_st_temp_buffer[0][s32_iloop>>1].s16_re >> 0);
        stp_H_vec_out[s32_iloop].s16_im = (s_st_temp_buffer[0][s32_iloop>>1].s16_im >> 0);

        s32_iloop += 2;
    }

    for(s32_iloop = (SC_NUM - EDGE_SUBCARRIER_NUM); s32_iloop < SC_NUM; )
    {
        stp_H_vec_out[s32_iloop].s16_re = (s_st_temp_buffer[0][s32_iloop>>1].s16_re >> 0);
        stp_H_vec_out[s32_iloop].s16_im = (s_st_temp_buffer[0][s32_iloop>>1].s16_im >> 0);

        s32_iloop += 2;
    }


    for(s32_iloop = 1; s32_iloop < EDGE_SUBCARRIER_NUM; ) 
    {
        stp_H_vec_out[s32_iloop].s16_re = ((stp_H_vec_out[s32_iloop - 1].s16_re + stp_H_vec_out[s32_iloop + 1].s16_re)>>1);
        stp_H_vec_out[s32_iloop].s16_im = ((stp_H_vec_out[s32_iloop - 1].s16_im + stp_H_vec_out[s32_iloop + 1].s16_im)>>1);

        s32_iloop += 2;
    }

    for(s32_iloop = 1 + (SC_NUM - EDGE_SUBCARRIER_NUM); s32_iloop < SC_NUM; ) 
    {
        stp_H_vec_out[s32_iloop].s16_re = ((stp_H_vec_out[s32_iloop - 1].s16_re + stp_H_vec_out[s32_iloop + 1].s16_re)>>1);
        stp_H_vec_out[s32_iloop].s16_im = ((stp_H_vec_out[s32_iloop - 1].s16_im + stp_H_vec_out[s32_iloop + 1].s16_im)>>1);

        s32_iloop += 2;
    }
	#else
	if(u32_ant_id EQ 0)
	{
	
		for(s32_iloop = HALF_SC_NUM - 1; s32_iloop > 0; s32_iloop--) //边带优化  偶数子载波 =ls
		{
			stp_H_vec_out[s32_iloop].s16_re = stp_H_vec_out[s32_iloop - 1].s16_re;
			stp_H_vec_out[s32_iloop].s16_im = stp_H_vec_out[s32_iloop - 1].s16_im;
		}
		#if 1
		for(s32_iloop = 0; s32_iloop < EDGE_SUBCARRIER_OPT_NUM; s32_iloop+=2) //边带优化  偶数子载波 =ls
		{
			stp_H_vec_out[s32_iloop + 1].s16_re=s_st_temp_buffer[0][s32_iloop>>1].s16_re;
			stp_H_vec_out[s32_iloop + 1].s16_im=s_st_temp_buffer[0][s32_iloop>>1].s16_im;
			stp_H_vec_out[SC_NUM-EDGE_SUBCARRIER_OPT_NUM+s32_iloop].s16_re=s_st_temp_buffer[0][HALF_SC_NUM-EDGE_SUBCARRIER_OPT_NUM/2+(s32_iloop>>1)].s16_re;
			stp_H_vec_out[SC_NUM-EDGE_SUBCARRIER_OPT_NUM+s32_iloop].s16_im=s_st_temp_buffer[0][HALF_SC_NUM-EDGE_SUBCARRIER_OPT_NUM/2+(s32_iloop>>1)].s16_im;
		}

		for(s32_iloop = 1; s32_iloop < EDGE_SUBCARRIER_OPT_NUM; s32_iloop+=2) //边带优化 奇数子载波=1/2(两边和)
		{
			stp_H_vec_out[s32_iloop + 1].s16_re=((stp_H_vec_out[s32_iloop].s16_re+stp_H_vec_out[s32_iloop+2].s16_re)>>1);
			stp_H_vec_out[s32_iloop + 1].s16_im=((stp_H_vec_out[s32_iloop].s16_im+stp_H_vec_out[s32_iloop+2].s16_im)>>1);
			stp_H_vec_out[SC_NUM-EDGE_SUBCARRIER_OPT_NUM+s32_iloop].s16_re=((stp_H_vec_out[SC_NUM-EDGE_SUBCARRIER_OPT_NUM+s32_iloop-1].s16_re+stp_H_vec_out[SC_NUM-EDGE_SUBCARRIER_OPT_NUM+s32_iloop+1].s16_re)>>1);//last is over
			stp_H_vec_out[SC_NUM-EDGE_SUBCARRIER_OPT_NUM+s32_iloop].s16_im=((stp_H_vec_out[SC_NUM-EDGE_SUBCARRIER_OPT_NUM+s32_iloop-1].s16_im+stp_H_vec_out[SC_NUM-EDGE_SUBCARRIER_OPT_NUM+s32_iloop+1].s16_im)>>1);//last is over

		}
		#endif
		stp_H_vec_out[0].s16_re = (stp_H_vec_out[1].s16_re<< 1) - stp_H_vec_out[2].s16_re;
		stp_H_vec_out[0].s16_im = (stp_H_vec_out[1].s16_im << 1) - stp_H_vec_out[2].s16_im;

		stp_H_vec_out[1199].s16_re = (stp_H_vec_out[1198].s16_re << 1) - stp_H_vec_out[1197].s16_re;
		stp_H_vec_out[1199].s16_im = (stp_H_vec_out[1198].s16_im << 1) - stp_H_vec_out[1197].s16_im;
	}
	else
	{
		#if 0
        h_fd12(useful_subcarrier_num/2+1:end) = [h_fd12(useful_subcarrier_num/2+2:end) h_fd_ls12(end)];
        h_fd12(1:2:N_left) = h_fd_ls12(1:N_left/2);
        h_fd12(end-N_right+2:2:end) = h_fd_ls12(end-N_right/2+1:end);
        for jj=2:2:N_left
            h_fd12(jj) = h_fd12(jj-1)+1/2*(h_fd12(jj+1)-h_fd12(jj-1));
        end
        for jj=1:2:N_right-1
            h_fd12(end-N_right+jj) = h_fd12(end-N_right+jj-1)+1/2*(h_fd12(end-N_right+jj+1)-h_fd12(end-N_right+jj-1));
        end

        h_fd12(1:2:N_left) = h_fd_ls12(1:N_left/2);
        h_fd12(end-N_right+2:2:end) = h_fd_ls12(end-N_right/2+1:end);
		#endif
		for(s32_iloop = HALF_SC_NUM - 1; s32_iloop > 0; s32_iloop--) //边带优化  偶数子载波 =ls
		{
			stp_H_vec_out[s32_iloop + HALF_SC_NUM].s16_re = stp_H_vec_out[s32_iloop + HALF_SC_NUM - 1].s16_re;
			stp_H_vec_out[s32_iloop + HALF_SC_NUM].s16_im = stp_H_vec_out[s32_iloop + HALF_SC_NUM - 1].s16_im;
		}
		#if 1

		for(s32_iloop = 0; s32_iloop < EDGE_SUBCARRIER_OPT_NUM; s32_iloop+=2) //边带优化  偶数子载波 =ls
		{
			stp_H_vec_out[s32_iloop].s16_re=s_st_temp_buffer[0][s32_iloop>>1].s16_re;
			stp_H_vec_out[s32_iloop].s16_im=s_st_temp_buffer[0][s32_iloop>>1].s16_im;
			stp_H_vec_out[SC_NUM-EDGE_SUBCARRIER_OPT_NUM+s32_iloop+1].s16_re=s_st_temp_buffer[0][HALF_SC_NUM-EDGE_SUBCARRIER_OPT_NUM/2+(s32_iloop>>1)].s16_re;
			stp_H_vec_out[SC_NUM-EDGE_SUBCARRIER_OPT_NUM+s32_iloop+1].s16_im=s_st_temp_buffer[0][HALF_SC_NUM-EDGE_SUBCARRIER_OPT_NUM/2+(s32_iloop>>1)].s16_im;
		}
		#endif
		#if 0
		for(s32_iloop = 1; s32_iloop < EDGE_SUBCARRIER_OPT_NUM; s32_iloop+=2) //边带优化 奇数子载波=1/2(两边和)
		{
			stp_H_vec_out[s32_iloop].s16_re=((stp_H_vec_out[s32_iloop - 1].s16_re+stp_H_vec_out[s32_iloop + 1].s16_re)>>1);
			stp_H_vec_out[s32_iloop].s16_im=((stp_H_vec_out[s32_iloop - 1].s16_im+stp_H_vec_out[s32_iloop + 1].s16_im)>>1);
			stp_H_vec_out[SC_NUM-EDGE_SUBCARRIER_OPT_NUM+s32_iloop - 1].s16_re=((stp_H_vec_out[SC_NUM-EDGE_SUBCARRIER_OPT_NUM+s32_iloop-2].s16_re+stp_H_vec_out[SC_NUM-EDGE_SUBCARRIER_OPT_NUM+s32_iloop + 0].s16_re)>>1);//last is over
			stp_H_vec_out[SC_NUM-EDGE_SUBCARRIER_OPT_NUM+s32_iloop - 1].s16_im=((stp_H_vec_out[SC_NUM-EDGE_SUBCARRIER_OPT_NUM+s32_iloop-2].s16_im+stp_H_vec_out[SC_NUM-EDGE_SUBCARRIER_OPT_NUM+s32_iloop + 0].s16_im)>>1);//last is over
		} 
		#endif
	}
	#endif
    //stp_H_vec_out[SC_NUM - 1].s16_re = (stp_H_vec_out[SC_NUM - 2].s16_re<<1) - stp_H_vec_out[SC_NUM - 3].s16_re;
    //stp_H_vec_out[SC_NUM - 1].s16_im = (stp_H_vec_out[SC_NUM - 2].s16_im<<1) - stp_H_vec_out[SC_NUM - 3].s16_im;
    //stp_H_vec_out[SC_NUM - 1].s16_re = (stp_H_vec_out[SC_NUM - 2].s16_re);
    //stp_H_vec_out[SC_NUM - 1].s16_im = (stp_H_vec_out[SC_NUM - 2].s16_im);

    /*pick up even subcarrier*/
    for(s32_iloop = 0; s32_iloop < ((SINT32)SC_NUM>>4); s32_iloop++)
    {
        v32_2 = vlddw_v32 (8, stp_H_vec_out + 16*s32_iloop, 0); 
        v32_3 = vpermutew_v32_v32 (8, v32_1, v32_2);

        vstdw_v32_concat (4, v32_3, 0, s_st_fft_in + s32_iloop*8);
    }
	//while(1);


    /*est noise pwr, excluding edge subcarrier*/
    _ks_cxv_sub_cxv(s_st_temp_buffer[0], s_st_fft_in, HALF_SC_NUM, s_st_temp_buffer[1]);

    *s32p_noise_pwr = ks_cxv_rssi_calc(&s_st_temp_buffer[1][6], HALF_SC_NUM - EDGE_SUBCARRIER_NUM);
    *s32p_sig_pwr  = ks_cxv_rssi_calc(&s_st_fft_in[6], HALF_SC_NUM - EDGE_SUBCARRIER_NUM);

    //s32_snr = (ks_logx(10, s32_signal_pwr) - ks_logx(10, *s32p_noise_pwr)); /*Q26*/
    s32_snr = ks_logx(10, *s32p_sig_pwr / (*s32p_noise_pwr + 1)); /*Q26*/

    if(s32_snr >= 0)
    {
        s32_snr = (s32_snr + (0x1<<25))>>20;
    }
    else
    {
        s32_snr = (s32_snr - (0x1<<25))>>20; 
    }

    s32_snr = ((10*s32_snr)>>6) - 7;
    if(s32_snr < 0)
    {
    	s32_snr = 0;
    }
	KS_LOG(0, 0xA11B, (UINT16)(s32_snr));

	#if 0
	if(READ_REG32(0x1724FFE0) EQ 1)
	{
		OAL_ASSERT((g_zzw_ch_status.snr > 20), "");
	}
	#endif

	//DL_GET_TMR2_END;
	//DL_PRINT_TMR2_DURATION(0xA21B);

    return s32_snr;

	#else
	
    s32_signal_pwr  = ks_cxv_rssi_calc(&s_st_fft_in[6], HALF_SC_NUM - EDGE_SUBCARRIER_NUM);

	for(s32_iloop = 0; s32_iloop < ((SINT32)HALF_SC_NUM); s32_iloop++)
    {
		((UINT32_PTR)stp_H_vec_out)[s32_iloop << 1] = ((UINT32_PTR)s_st_temp_buffer)[s32_iloop];
		((UINT32_PTR)stp_H_vec_out)[(s32_iloop << 1) + 1] = ((UINT32_PTR)s_st_temp_buffer)[s32_iloop];
    }
	*s32p_noise_pwr = 1;
    return s32_signal_pwr;

	#endif

}

#else
OAL_ITCM_CODE_SECTION
SINT32 dl_H_calc(complex_s16_t* stp_pilot_in, complex_s16_t* stp_H_vec_out, SINT32_PTR s32p_sig_pwr, SINT32_PTR s32p_noise_pwr, UINT32 u32_ant_id)
{
    ks_fft_para_t st_fft_para;
    SINT32 s32_shift_amt_600[5] = {(UINT32)MODV_SV2, (UINT32)MODV_SV2, (UINT32)MODV_SV2, (UINT32)MODV_SV1, (UINT32)MODV_SV0};
    SINT32 s32_shift_amt_1200[6] = {(UINT32)MODV_SV0, (UINT32)MODV_SV0, (UINT32)MODV_SV0, (UINT32)MODV_SV0, (UINT32)MODV_SV0, (UINT32)MODV_SV0};
    
    UINT32 u32_permute[2] = {0xDC985410, 0xFEBA7632};
    SINT32 s32_signal_pwr, s32_iloop;
    SINT32 s32_snr = 0;
    SINT32 s32_edge_len = 50;
    //complex_s16_t st_dft_600_factor = {27961, 0}; // factor=512/600
    complex_s16_t st_dft_600_factor = {23170, 0}; // factor=512/600
	//OAL_IRQ_SAVE_AREA;
    SINT32 s32_peak, s32_noise, s32_threshold;

    vec_t v32_1;
    vec_t v32_2;
    vec_t v32_3;
    vec_t v32_4;
    vec_t v32_5;
    coef_t vcoef;
    vprex_t v_prex;

	//DL_GET_TMR2_START;

    /*transform into freq domain*/
    st_fft_para.s16cp_indata_addr   = stp_pilot_in;
    st_fft_para.s16cp_outdata_addr  = s_st_fft_out;
    st_fft_para.s16cp_twiddle_addr  = (complex_s16_t*)ceva_fft_lib_fft_2048_twi_table;
    st_fft_para.s32p_shift_amt_addr = s_s32_shift_amt_fft;
    st_fft_para.s16cp_temp_addr     = (complex_s16_t*)(s_st_temp_buffer[1]);

	#if 0
	//if(u32_ant_id)
	{
		OAL_IRQ_DISABLE_ALL;
	}
	#endif
	//#if SCFDMA_SWITCH
	#if 0
    v32_1 = vlddw_v32_clone(2, u32_permute, 0);
    for(s32_iloop = 0; s32_iloop < ((SINT32)HALF_SC_NUM>>4); s32_iloop++)
    {
        v32_2 = vlddw_v32 (8, stp_pilot_in + SYMBOL_LEN - HALF_SC_NUM + 16*s32_iloop + 1 - u32_ant_id, 0); //正频率
        v32_3 = vpermutew_v32_v32 (8, v32_1, v32_2);

        v32_4 = vlddw_v32 (8, stp_pilot_in + 1 + 16*s32_iloop + u32_ant_id, 0); //负频率
        v32_5 = vpermutew_v32_v32 (8, v32_1, v32_4);

        vstdw_v32_concat (4, v32_5, 0, &s_st_temp_buffer[1][s32_iloop*8]);
        vstdw_v32_concat (4, v32_3, 0, &s_st_temp_buffer[1][s32_iloop*8 + QUARTER_SC_NUM]);
    }
    v32_2 = vlddw_v32 (4, stp_pilot_in + SYMBOL_LEN - HALF_SC_NUM + 16*s32_iloop + 1 - u32_ant_id, 0); //正频率
    v32_3 = vpermutew_v32_v32 (8, v32_1, v32_2);

    v32_4 = vlddw_v32 (4, stp_pilot_in + 1 + 16*s32_iloop + u32_ant_id, 0); //负频率
    v32_5 = vpermutew_v32_v32 (8, v32_1, v32_4);

    vstdw_v32_concat (2, v32_5, 0, &s_st_temp_buffer[1][s32_iloop*8]);
    vstdw_v32_concat (2, v32_3, 0, &s_st_temp_buffer[1][s32_iloop*8 + QUARTER_SC_NUM]);    
	#else
    v32_1 = vlddw_v32_clone(2, u32_permute, 0);
    for(s32_iloop = 0; s32_iloop < ((SINT32)HALF_SC_NUM>>4); s32_iloop++)
    {
        v32_2 = vlddw_v32 (8, stp_pilot_in + SYMBOL_LEN - HALF_SC_NUM + 16*s32_iloop + 1 - u32_ant_id, 0); //正频率
        v32_3 = vpermutew_v32_v32 (8, v32_1, v32_2);

        v32_4 = vlddw_v32 (8, stp_pilot_in + 1 + 16*s32_iloop + u32_ant_id, 0); //负频率
        v32_5 = vpermutew_v32_v32 (8, v32_1, v32_4);

        vstdw_v32_concat (4, v32_3, 0, &s_st_temp_buffer[1][s32_iloop*8]);
        vstdw_v32_concat (4, v32_5, 0, &s_st_temp_buffer[1][s32_iloop*8 + QUARTER_SC_NUM]);
    }
    v32_2 = vlddw_v32 (4, stp_pilot_in + SYMBOL_LEN - HALF_SC_NUM + 16*s32_iloop + 1 - u32_ant_id, 0); //正频率
    v32_3 = vpermutew_v32_v32 (8, v32_1, v32_2);

    v32_4 = vlddw_v32 (4, stp_pilot_in + 1 + 16*s32_iloop + u32_ant_id, 0); //负频率
    v32_5 = vpermutew_v32_v32 (8, v32_1, v32_4);

    vstdw_v32_concat (2, v32_3, 0, &s_st_temp_buffer[1][s32_iloop*8]);
    vstdw_v32_concat (2, v32_5, 0, &s_st_temp_buffer[1][s32_iloop*8 + QUARTER_SC_NUM]);

	#endif

    ks_cxv_mul_cxv(s_st_temp_buffer[1], s_st_fd_pilot_seq, HALF_SC_NUM, 2, s_st_temp_buffer[0]);

	#if 0
	//if(u32_ant_id)
	{
		while(READ_REG32(0x1724FFE0) NEQ 0);
		WRITE_REG32(0x1724FFE4, 0);
	}
	#endif
	#if 1
	#if H_CALC_OPTIMIZE
    _ks_shift_s16v((SINT16_PTR)s_st_temp_buffer[0], SC_NUM, FFT_SHIFT_VAL, (SINT16_PTR)&s_st_fft_in[H_CALC_EDGE]);
    for(s32_iloop = 0; s32_iloop < ((SINT32)H_CALC_EDGE); s32_iloop++)
    {
    	*(((UINT32_PTR)s_st_fft_in) + s32_iloop) = *(((UINT32_PTR)s_st_fft_in) + H_CALC_EDGE * 2 - s32_iloop - 1);
    	*(((UINT32_PTR)s_st_fft_in) + s32_iloop + HALF_SC_NUM + H_CALC_EDGE) = *(((UINT32_PTR)s_st_fft_in) + H_CALC_EDGE + HALF_SC_NUM - 1 - s32_iloop);  	
    }
    #else
    _ks_shift_s16v((SINT16_PTR)s_st_temp_buffer[0], SC_NUM, FFT_SHIFT_VAL, (SINT16_PTR)s_st_fft_in);
	#endif
    /*idft(600) transform into time domain*/
    //dft_ceva_algo(HALF_SC_NUM, s32_shift_amt_1, 1, s_st_fft_in, s_st_fft_out, s_st_temp_buffer[1]);
    /*idft(600) transform into time domain*/
    st_fft_para.s16cp_indata_addr   = s_st_fft_in;
    //st_fft_para.s16cp_outdata_addr  = s_st_fft_out;
    st_fft_para.s16cp_twiddle_addr  = (complex_s16_t*)ceva_fft_lib_fft_1024_twi_table;
    
	#if H_CALC_OPTIMIZE
    ks_oal_mem_set(&s_st_fft_in[(H_CALC_EDGE << 1) + HALF_SC_NUM], 0, ((1024 - (H_CALC_EDGE << 1) - HALF_SC_NUM) << 2));
	#else
    ks_oal_mem_set(&s_st_fft_in[HALF_SC_NUM], 0, (424 << 2));
	#endif
    ks_ifft1024(&st_fft_para);
	#if 0
	//if(u32_ant_id)
	{
		while(READ_REG32(0x1724FFE0) NEQ 1);
		WRITE_REG32(0x1724FFE4, 1);
	}
	#endif
	#if 0
    ks_oal_mem_copy(&s_st_fft_in[0], &s_st_fft_out[0], (s32_edge_len << 2));
    ks_oal_mem_copy(&s_st_fft_in[2048 - s32_edge_len], &s_st_fft_out[1024 - s32_edge_len], (s32_edge_len << 2));
    ks_oal_mem_set(&s_st_fft_in[s32_edge_len], 0, ((2048 - s32_edge_len * 2) << 2));
    #else
    ks_cxv_squa_s16(s_st_fft_out, 1024, s_st_temp_buffer[1]);
    s32_peak = _ks_max_s32v_idx(s_st_temp_buffer[1], &s32_iloop, 1024);
    s32_noise = ks_cxv_rssi_calc(&s_st_fft_out[512 - 32 - 1], 64);

    if((s32_peak) >= s32_noise * 100)
    {
        s32_threshold = (s32_noise<<2) + (s32_noise<<1) + (s32_noise>>2);
    }
    else if((s32_peak) >= s32_noise * 36)
    {
        s32_threshold = (s32_noise<<1) + (s32_noise>>2);
    }
    else
    {
        s32_threshold = s32_noise;
    }

    vcoef = vlddw_c32_clone_1dw (&s32_threshold);

    for(s32_iloop = 0; s32_iloop < 512; )
    {
        v32_2 = vlddw_v32 (8, &s_st_temp_buffer[1][s32_iloop], 0);
        v_prex = vcmp_v32_c32_vpr_lt (8, v32_2, vcoef);

        v32_3 = vlddw_v32 (8, &s_st_fft_out[s32_iloop], 0);
        v32_3 = vclr_v32_p (8, v_prex);
        vstdw_v32 (8, v32_3, 0, &s_st_fft_in[s32_iloop]);

        s32_iloop += 16;
    }

    for(s32_iloop = 512; s32_iloop < 1024; )
    {
        v32_2 = vlddw_v32 (8, &s_st_temp_buffer[1][s32_iloop], 0);
        v_prex = vcmp_v32_c32_vpr_lt (8, v32_2, vcoef);

        v32_3 = vlddw_v32 (8, &s_st_fft_out[s32_iloop], 0);
        v32_3 = vclr_v32_p (8, v_prex);
        vstdw_v32 (8, v32_3, 0, &s_st_fft_in[s32_iloop + 1024]);

        s32_iloop += 16;
    }
    //ks_oal_mem_set(&s_st_fft_in[512], 0, (1024 << 2));
	ks_oal_mem_set(&s_st_fft_in[256], 0, (1536 << 2));    
	#endif
    /*dft(1200) transform into freq domain*/
    //dft_ceva_algo(SC_NUM, s32_shift_amt_2, 0, s_st_fft_in, s_st_fft_out, s_st_temp_buffer[1]);
    st_fft_para.s16cp_twiddle_addr  = (complex_s16_t*)ceva_fft_lib_fft_2048_twi_table;
    ks_fft2048(&st_fft_para);

    //_ks_cxv_mul_cx(s_st_fft_out, SC_NUM, &st_dft_600_factor, 0, stp_H_vec_out); 
    //_ks_shift_s16v((SINT16_PTR)s_st_fft_out, SC_NUM * 2, 4, (SINT16_PTR)stp_H_vec_out);
	#if 0
	//if(u32_ant_id)
	{
		while(READ_REG32(0x1724FFE0) NEQ 2);
		WRITE_REG32(0x1724FFE4, 2);
	}
	#endif

	_ks_shift_s16v((SINT16_PTR)(s_st_fft_out + (H_CALC_EDGE << 1)), SC_NUM << 1, -FFT_SHIFT_VAL, (SINT16_PTR)(stp_H_vec_out));
	//_ks_shift_s16v((SINT16_PTR)(s_st_fft_out + HALF_SC_NUM), SC_NUM, 4, (SINT16_PTR)(stp_H_vec_out + HALF_SC_NUM));

    /*pick up even subcarrier*/
    for(s32_iloop = 0; s32_iloop < ((SINT32)SC_NUM>>4); s32_iloop++)
    {
        v32_2 = vlddw_v32 (8, stp_H_vec_out + 16*s32_iloop, 0); 
        v32_3 = vpermutew_v32_v32 (8, v32_1, v32_2);

        vstdw_v32_concat (4, v32_3, 0, s_st_fft_in + s32_iloop*8);
    }
	//while(1);


    /*est noise pwr, excluding edge subcarrier*/
    _ks_cxv_sub_cxv(s_st_temp_buffer[0], s_st_fft_in, HALF_SC_NUM, s_st_temp_buffer[1]);

    *s32p_noise_pwr = ks_cxv_rssi_calc(&s_st_temp_buffer[1][6], HALF_SC_NUM - EDGE_SUBCARRIER_NUM);
    *s32p_sig_pwr  = ks_cxv_rssi_calc(&s_st_fft_in[6], HALF_SC_NUM - EDGE_SUBCARRIER_NUM);

    //s32_snr = (ks_logx(10, s32_signal_pwr) - ks_logx(10, *s32p_noise_pwr)); /*Q26*/
    if(*s32p_sig_pwr >= *s32p_noise_pwr )
    {
    	s32_snr = ks_logx(10, *s32p_sig_pwr / (*s32p_noise_pwr)); /*Q26*/
        s32_snr = (s32_snr + (0x1<<25))>>20;
	}
	else
	{
    	s32_snr = ks_logx(10, (*s32p_noise_pwr) / *s32p_sig_pwr); /*Q26*/
        s32_snr = (-s32_snr + (0x1<<25))>>20; 
	}

    s32_snr = ((10*s32_snr)>>6) - 4;
    #if 0
    if(s32_snr < 0)
    {
    	s32_snr = 0;
    }
    #endif
	KS_LOG(0, 0xA11B, (UINT16)(s32_snr));

	#if 0
	if(READ_REG32(0x1724FFE0) EQ 1)
	{
		OAL_ASSERT((g_zzw_ch_status.snr > 20), "");
	}
	#endif

	//DL_GET_TMR2_END;
	//DL_PRINT_TMR2_DURATION(0xA21B);

    return s32_snr;

	#else
	
    s32_signal_pwr  = ks_cxv_rssi_calc(&s_st_fft_in[6], HALF_SC_NUM - EDGE_SUBCARRIER_NUM);

	for(s32_iloop = 0; s32_iloop < ((SINT32)HALF_SC_NUM); s32_iloop++)
    {
		((UINT32_PTR)stp_H_vec_out)[s32_iloop << 1] = ((UINT32_PTR)s_st_temp_buffer)[s32_iloop];
		((UINT32_PTR)stp_H_vec_out)[(s32_iloop << 1) + 1] = ((UINT32_PTR)s_st_temp_buffer)[s32_iloop];
    }
	*s32p_noise_pwr = 1;
    return s32_signal_pwr;

	#endif

}
#endif
OAL_ITCM_CODE_SECTION 
SINT32 ks_angle(SINT32 s32_cfo_re, SINT32 s32_cfo_im)
{
	SINT32 s32_angle = 0, s32_temp;
	UINT32 u32_iloop;
    UINT32 u32_comp_flag = OAL_FALSE;
    SINT32 s32_sign = 0;
	

	if(s32_cfo_re < 0)
	{
		u32_comp_flag = OAL_TRUE;
		s32_cfo_re = -s32_cfo_re;
	}
	
	//KS_LOG_TMRL(0xA116);

	#if 1
	/*caculate the angle*/
	for(u32_iloop = 0; u32_iloop < 15; u32_iloop++)
	{
		if(s32_cfo_im >= 0)
		{
		   s32_temp   = s32_cfo_re + (s32_cfo_im>>u32_iloop);
		   s32_cfo_im = s32_cfo_im - (s32_cfo_re>>u32_iloop);

		   s32_cfo_re = s32_temp;

		   s32_angle += s_s16_cordic_table[u32_iloop];
		}
		else
		{
			s32_temp   = s32_cfo_re - (s32_cfo_im>>u32_iloop);
			s32_cfo_im = s32_cfo_im + (s32_cfo_re>>u32_iloop);

			s32_cfo_re = s32_temp;

			s32_angle -= s_s16_cordic_table[u32_iloop];
		}
	}
	#else
	for(u32_iloop = 0; u32_iloop < 15; u32_iloop++)
	{
		s32_sign = 1 - (SINT32)((((UINT32)s32_cfo_im) >> 31) << 1);

		s32_temp   = s32_cfo_re + (s32_cfo_im>>u32_iloop) * s32_sign;
		s32_cfo_im = s32_cfo_im - (s32_cfo_re>>u32_iloop) * s32_sign;

		s32_cfo_re = s32_temp;

		s32_angle += s_s16_cordic_table[u32_iloop] * s32_sign;

	}

	#endif
	//KS_LOG_TMRL(0xA116);

    if(OAL_TRUE == u32_comp_flag)
    {
        if(s32_angle > 0)
        {
            s32_angle = 8192*4 - s32_angle; /*PI - s32_angle*/
        }
        else
        {
            s32_angle = -8192*4 - s32_angle;
        }
    }
    return s32_angle;
}
VOID dl_snr_shift(IN UINT32_PTR u32p_snr, OUT UINT16_PTR u16p_snr, IN UINT32 u32_len, IN SINT32 s32_snr_mean)
{
	SINT32 s32_iloop = 0;
	SINT32 s32_shift = 0;

	coef_t vcoef_shift;
	vec_t v32_1;
	vec_t v32_2;


	s32_shift = exp_acW_acZ (s32_snr_mean);
	__asm__("push {dw} a8");
	__asm__("vpush {dw} modv0");
	__asm__("mov #0x00000C20, a8");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("vmova a8, modv0");

	vcoef_shift = vlddw_c32_clone_1dw (&s32_shift);

	for(s32_iloop = 0; s32_iloop < u32_len; )
	{
		v32_1 = vlddw_v32 (8, &u32p_snr[s32_iloop], 0);
		v32_2 = vshift_v32_c32_v32_ar (8, v32_1, vcoef_shift);

		vstw_v32_vuX_hp (8, 0, v32_2, &u16p_snr[s32_iloop]);
		vstw_v32_vuX_hp (8, 1, v32_2, &u16p_snr[s32_iloop + 8]);

		s32_iloop += 16;
	}

	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("vpop {dw} modv0");
	__asm__("pop {dw} a8");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");

}
OAL_ITCM_CODE_SECTION
VOID dl_demod_bpsk(complex_s16_t* stp_equout_in, SINT8* s8p_llr_out,UINT32 u32_fill_len,UINT32 u32_equ_len)
{
	UINT32 u32_permute[2];
	SINT32 s32_iloop;
    vec_t   v32_1;
    vec_t   v32_2;
    vec_t   v32_3;
    vec_t   v32_4;
	vec_t	v32_5;

	
	coef_t 	vcoef_shift;
	coef_t	vcoef_bound;

	SINT32 demod_shift=4;

	__asm__("push {dw} a8");
	__asm__("vpush {dw} modv0");
	__asm__("mov #0x00000C20, a8");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("vmova a8, modv0");
	
	vcoef_shift = vlddw_c32_clone_1dw (&demod_shift);
		
    u32_permute[0] = 0x67452301;
    u32_permute[1] = 0xEFCDAB89;

    v32_1 = vlddw_v32_clone(2, u32_permute, 0);
    u32_permute[0] = 0xeca86420;
    u32_permute[1] = 0xfdb97531;
    
	v32_5=vlddw_v32_clone(2, u32_permute, 0);

    for(s32_iloop = 0; s32_iloop < ((u32_equ_len)>>4); s32_iloop++)
	#pragma dsp_ceva_trip_count_min = 1
	#pragma dsp_ceva_unroll = 2
    {
    	#if 0
        v32_2 = vlddw_v32 (8, stp_equout_in + 16*s32_iloop, 0);
        //v32_3 = vshiftw_v32_imm5_v32_ar (8, v32_2, -9);
		v32_3 = vshiftw_v32_c32_v32(8, v32_2, vcoef_shift);

		
        v32_2 = vpermutew_v32_v32 (8, v32_1, v32_3);
        v32_4 = vadd16_v32_v32_v32 (8, v32_2, v32_3);
        
        v32_2 = vpermutew_v32_v32 (8, v32_5, v32_4);
		#else
        v32_2 = vlddw_v32 (8, stp_equout_in + 16*s32_iloop, 0);
        v32_3 = vpermutew_v32_v32 (8, v32_1, v32_2);
        v32_4 = vadd16_v32_v32_v32 (8, v32_2, v32_3);
		v32_3 = vshiftw_v32_c32_v32(8, v32_4, vcoef_shift);
        v32_2 = vpermutew_v32_v32 (8, v32_5, v32_3);
		#endif
        vstb_v32_mw_h_concat (4, v32_2, s8p_llr_out+s32_iloop*16 +u32_fill_len);
    }
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("vpop {dw} modv0");
	__asm__("pop {dw} a8");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
}
OAL_ITCM_CODE_SECTION
VOID dl_demod_qpsk(complex_s16_t* stp_equout_in, SINT8* s8p_llr_out,UINT32 u32_fill_len,UINT32 u32_equ_len)
{
	UINT32 u32_permute[2];
	SINT32 s32_iloop;
    vec_t   v32_1;
    vec_t   v32_2;
    vec_t   v32_3;
  
	coef_t 	vcoef_shift;
	SINT32 demod_shift=5;

	__asm__("push {dw} a8");
	__asm__("vpush {dw} modv0");
	__asm__("mov #0x00000C20, a8");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("vmova a8, modv0");
	
	vcoef_shift = vlddw_c32_clone_1dw (&demod_shift);


    for(s32_iloop = 0; s32_iloop < ((u32_equ_len)>>4); s32_iloop++)
	#pragma dsp_ceva_trip_count_min = 1
	#pragma dsp_ceva_unroll = 2
    {
        v32_2 = vlddw_v32 (8, stp_equout_in + 16*s32_iloop, 0);
		v32_3 = vshiftw_v32_c32_v32(8, v32_2, vcoef_shift);
			
        //vstb_v32_mw_concat (8, v32_3, s8p_llr_out+s32_iloop*32 +u32_fill_len);
        vstb_v32_mw_h_concat (8, v32_3, s8p_llr_out+s32_iloop*32 +u32_fill_len);

    }
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("vpop {dw} modv0");
	__asm__("pop {dw} a8");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
}

OAL_ITCM_CODE_SECTION
VOID dl_demod_qam16(complex_s16_t* stp_equout_in, SINT8* s8p_llr_out,UINT32 u32_fill_len,UINT32 u32_equ_len)
{
	UINT32 u32_permute[2];
	SINT32 s32_iloop;
    vec_t   v32_1;
    vec_t   v32_2;
    vec_t   v32_3;
    vec_t   v32_4;
	vec_t	v32_5;
	vec_t	v32_6;
	vec_t	v32_7;
	
	coef_t 	vcoef_shift;
	//coef_t 	vcoef_shift2;
	coef_t	vcoef_bound;


	SINT32 demod_shift = 5;
	//SINT32 demod_shift2 = -2;

    #if 0
    for(u32_iloop = 0; u32_iloop < u32_kloop*SYMBOL_LEN; u32_iloop++)
    {
        s_s8_llr[4*u32_iloop + u32_llr_shift[0]]     = (s_st_eq_out[u32_iloop].s16_re>>7);
        s_s8_llr[4*u32_iloop + u32_llr_shift[0] + 1] = (s_st_eq_out[u32_iloop].s16_im>>7);
        s_s8_llr[4*u32_iloop + u32_llr_shift[0] + 2] = ((SINT16)0x1cc - (SINT16)abs_acW_acZ((SINT32)s_st_eq_out[u32_iloop].s16_re))>>7;
        s_s8_llr[4*u32_iloop + u32_llr_shift[0] + 3] = ((SINT16)0x1cc - (SINT16)abs_acW_acZ((SINT32)s_st_eq_out[u32_iloop].s16_im))>>7;
    }
	#else

	//UINT32 u32_qam16_boundary = DEMOD_THRES_QAM16;
	//UINT32 u32_qam16_boundary = 0x2000200;

	__asm__("push {dw} a8");
	__asm__("vpush {dw} modv0");
	__asm__("mov #0x00000C20, a8");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("vmova a8, modv0");
	
	vcoef_shift = vlddw_c32_clone_1dw (&demod_shift);
	//vcoef_shift = vlddw_c32_clone_1dw (&demod_shift2);

	u32_permute[0] = 0xb3a29180;
	u32_permute[1] = 0xf7e6d5c4;

	vcoef_bound = vlddw_c32_clone_1dw (&g_u32_qam16_boundary);	
	v32_6 = vfillw_c32_v40 (8, vcoef_bound);
	v32_7 = vlddw_v32_clone (2, u32_permute, 0);

	
	for(s32_iloop = 0; s32_iloop < ((u32_equ_len)>>4); s32_iloop++)
	#pragma dsp_ceva_trip_count_min = 1
	#pragma dsp_ceva_unroll = 2
	{
		//v32_2 = vlddw_v32 (8, stp_equout_in + 16*s32_iloop, 0);  //16个dw

        v32_2 = vlddw_v32 (4, stp_equout_in + 16*s32_iloop, 0);
		v32_2 = vlddw_v32 (4, stp_equout_in + 16*s32_iloop + 8, 4);		
		
		v32_3 = vshiftw_v32_c32_v32(8, v32_2, vcoef_shift);  //0/1，实部虚部都已经移位

		v32_4 = vabs16_v32_v32 (8, v32_2);				
		v32_5 = vsub16_v32_v32_v32 (8, v32_6, v32_4);
		v32_1 = vshiftw_v32_c32_v32(8, v32_5, vcoef_shift);  //2/3，实部虚部都已经移位
	
		v32_4 = vpack_v40_v40_v32_bh_Ohop(8, v32_3, v32_1);//32   10
	
		v32_3 = vpermutew_v32_v32 (8, v32_7, v32_4);

		vstb_v32_pb_concat (16, v32_3, 0, s8p_llr_out+s32_iloop*64 +u32_fill_len);
		vstb_v32_pb_concat (16, v32_3, 4, s8p_llr_out+s32_iloop*64 +u32_fill_len + 32);

	}		
	#endif
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("vpop {dw} modv0");
	__asm__("pop {dw} a8");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
}
OAL_ITCM_CODE_SECTION
VOID dl_demod_qam64(complex_s16_t* stp_equout_in, SINT8* s8p_llr_out,UINT32 u32_fill_len,UINT32 u32_equ_len)
{
	UINT32 u32_permute[2];
	SINT32 s32_iloop;
    vec_t   v32_1;
    vec_t   v32_2;
    vec_t   v32_3;
    vec_t   v32_4;
	vec_t	v32_5;
	vec_t	v32_6;
	vec_t	v32_7;

	vec_t	v32_8;
	vec_t	v32_9;

	vec_t	v32_10;
	vec_t	v32_11;
	vec_t	v32_12;	
	
	coef_t 	vcoef_shift;
	//coef_t 	vcoef_shift2;
	coef_t	vcoef_bound;

	vprex_t vecPredRes1;
	vprex_t vecPredRes2;
	vprex_t vecPredRes3;
	SINT32 demod_shift = 5;
	//SINT32 demod_shift2 = -2;


	//UINT32 u32_qam16_boundary = DEMOD_THRES_QAM16;
	//UINT32 u32_qam16_boundary = 0x2000200;

	__asm__("push {dw} a8");
	__asm__("vpush {dw} modv0");
	__asm__("mov #0x00000C20, a8");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("vmova a8, modv0");
	
	#if 0
    for(s32_iloop = 0; s32_iloop < u32_equ_len; s32_iloop++)
    {
        *(s8p_llr_out+6*s32_iloop + u32_fill_len)   = (s_st_eq_out[s32_iloop].s16_re>>5);
        *(s8p_llr_out+6*s32_iloop + u32_fill_len+1)	= (s_st_eq_out[s32_iloop].s16_im>>5);
        *(s8p_llr_out+6*s32_iloop + u32_fill_len+2)	= ((SINT16)0x1c0 - (SINT16)abs_acW_acZ((SINT32)s_st_eq_out[s32_iloop].s16_re))>>5;
        *(s8p_llr_out+6*s32_iloop + u32_fill_len+3) = ((SINT16)0x1c0 - (SINT16)abs_acW_acZ((SINT32)s_st_eq_out[s32_iloop].s16_im))>>5;

		if(abs_acW_acZ((SINT16)0x1c0-(SINT16)abs_acW_acZ((SINT32)s_st_eq_out[s32_iloop].s16_re))<=0xe0)
		{
			*(s8p_llr_out+6*s32_iloop + u32_fill_len+4)=((SINT16)0xe0 - (SINT16)abs_acW_acZ((SINT16)0x1c0-(SINT16)abs_acW_acZ((SINT32)s_st_eq_out[s32_iloop].s16_re)))>>5;
		}
		else if(abs_acW_acZ((SINT32)s_st_eq_out[s32_iloop].s16_re)<=0xe0)
		{
			*(s8p_llr_out+6*s32_iloop + u32_fill_len+4)=((SINT16)abs_acW_acZ((SINT32)s_st_eq_out[s32_iloop].s16_re)-(SINT16)0xe0)>>5;
		}
		else
		{
			*(s8p_llr_out+6*s32_iloop + u32_fill_len+4)=((SINT16)0x29e - (SINT16)abs_acW_acZ((SINT32)s_st_eq_out[s32_iloop].s16_re))>>5;
		}
		
		if(abs_acW_acZ((SINT16)0x1c0-(SINT16)abs_acW_acZ((SINT32)s_st_eq_out[s32_iloop].s16_im))<=0xe0)
		{
			*(s8p_llr_out+6*s32_iloop + u32_fill_len+5)=((SINT16)0xe0 - (SINT16)abs_acW_acZ((SINT32)((SINT16)0x1c0-(SINT16)abs_acW_acZ((SINT32)s_st_eq_out[s32_iloop].s16_im))))>>5;
		}
		else if(abs_acW_acZ((SINT32)s_st_eq_out[s32_iloop].s16_im)<=0xe0)
		{
			*(s8p_llr_out+6*s32_iloop + u32_fill_len+5)=((SINT16)abs_acW_acZ((SINT32)s_st_eq_out[s32_iloop].s16_im)-(SINT16)0xe0)>>5;
		}
		else
		{
			*(s8p_llr_out+6*s32_iloop + u32_fill_len+5)=((SINT16)0x29e - (SINT16)abs_acW_acZ((SINT32)s_st_eq_out[s32_iloop].s16_im))>>5;
		}

		
    }
	#else
	#if 0
	UINT32 u32_qam64_boundary1 = 0xe000e0;
	UINT32 u32_qam64_boundary2 = 0x1c001c0;
	UINT32 u32_qam64_boundary3 = 0x29e029e;
	#else
	UINT32 u32_qam64_boundary1 = g_u32_qam64_boundary;   //242
	UINT32 u32_qam64_boundary2 = (g_u32_qam64_boundary << 1);  //484
	UINT32 u32_qam64_boundary3 = (g_u32_qam64_boundary * 3);  //726
	#endif
	demod_shift = 5;
	vcoef_shift = vlddw_c32_clone_1dw (&demod_shift);

	vcoef_bound  = vlddw_c32_clone_1dw (&u32_qam64_boundary1);
	v32_1= vfillw_c32_v40 (8, vcoef_bound);

	vcoef_bound  = vlddw_c32_clone_1dw (&u32_qam64_boundary2);
	v32_2= vfillw_c32_v40 (8, vcoef_bound);

	vcoef_bound  = vlddw_c32_clone_1dw (&u32_qam64_boundary3);
	v32_3= vfillw_c32_v40 (8, vcoef_bound);
		
	
	for(s32_iloop = 0; s32_iloop < ((u32_equ_len)>>4); s32_iloop++)
	#pragma dsp_ceva_trip_count_min = 1
	#pragma dsp_ceva_unroll = 2
	{
		v32_4 = vlddw_v32 (8, stp_equout_in + 16*s32_iloop, 0);  //16个dw

        //v32_2 = vlddw_v32 (4, stp_equout_in + 16*s32_iloop, 0);
		//v32_2 = vlddw_v32 (4, stp_equout_in + 16*s32_iloop + 8, 4);		
		
		v32_5 = vshiftw_v32_c32_v32(8, v32_4, vcoef_shift);  //0/1，实部虚部都已经移位

		v32_6 = vabs16_v32_v32 (8, v32_4);				
		v32_7 = vsub16_v32_v32_v32 (8, v32_2, v32_6);
		v32_8 = vshiftw_v32_c32_v32(8, v32_7, vcoef_shift);  //2/3，实部虚部都已经移位

		vecPredRes1 = vcmpw_v32_v32_vpr_le(8, v32_6, v32_1);  // abs()<=0xe0
		vecPredRes2 = vcmpw_v32_v32_vpr_gt(8, v32_6, v32_3);  // abs()>0x29e
		v32_9 = vabs16_v32_v32 (8, v32_7);	//abs(0x1c0-abs())			
		vecPredRes3 = vcmpw_v32_v32_vpr_le(8, v32_9, v32_1);  // abs(0x1c0-abs())<=0xe0
		

		v32_10= vsub16_v32_v32_v32_p (8, v32_1, v32_9, vecPredRes3);
		v32_10= vsub16_v32_v32_v32_p (8, v32_6, v32_1, vecPredRes1);
		v32_10= vsub16_v32_v32_v32_p (8, v32_3, v32_6, vecPredRes2);

		v32_7= vshiftw_v32_c32_v32(8, v32_10, vcoef_shift);  //4/5，实部虚部都已经移位


		v32_11 = vtransdw_v32_v32_v32_uimm3_v32_3dw(2, v32_5, v32_8, v32_7, 0);
		vstb_v32_mw_vuX_h (6, 0, v32_11, s8p_llr_out+s32_iloop*96 +u32_fill_len);
		vstb_v32_mw_vuX_h (6, 1, v32_11, s8p_llr_out+s32_iloop*96 +48+u32_fill_len);


		v32_11 = vtransdw_v32_v32_v32_uimm3_v32_3dw(2, v32_5, v32_8, v32_7, 2);	
		vstb_v32_mw_vuX_h (6, 0, v32_11, s8p_llr_out+s32_iloop*96 +12+u32_fill_len);
		vstb_v32_mw_vuX_h (6, 1, v32_11, s8p_llr_out+s32_iloop*96 +48+12+u32_fill_len);
		

		v32_11 = vtransdw_v32_v32_v32_uimm3_v32_3dw(2, v32_5, v32_8, v32_7, 4);			
		vstb_v32_mw_vuX_h (6, 0, v32_11, s8p_llr_out+s32_iloop*96 +24+u32_fill_len);
		vstb_v32_mw_vuX_h (6, 1, v32_11, s8p_llr_out+s32_iloop*96 +48+24+u32_fill_len);
		

		v32_11 = vtransdw_v32_v32_v32_uimm3_v32_3dw(2, v32_5, v32_8, v32_7, 6);
		vstb_v32_mw_vuX_h (6, 0, v32_11, s8p_llr_out+s32_iloop*96 +36+u32_fill_len);
		vstb_v32_mw_vuX_h (6, 1, v32_11, s8p_llr_out+s32_iloop*96 +48+36+u32_fill_len);		

	}
	#endif

	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("vpop {dw} modv0");
	__asm__("pop {dw} a8");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
}

OAL_ITCM_CODE_SECTION
VOID dl_demod_qam256(complex_s16_t* stp_equout_in, SINT8* s8p_llr_out,UINT32 u32_fill_len,UINT32 u32_equ_len)
{
	UINT32 u32_permute[2];
	SINT32 s32_iloop;
	SINT32 demod_shift = 4;
	//SINT32 demod_shift2 = -2;

    UINT32 u32_qam256_boundary1 = g_u32_qam256_boundary; //1A8
    UINT32 u32_qam256_boundary2 = (g_u32_qam256_boundary >> 1);
    UINT32 u32_qam256_boundary3 = (g_u32_qam256_boundary >> 2);
	
    vec_t   v32_1;
    vec_t   v32_2;
    vec_t   v32_3;
    vec_t   v32_4;
	vec_t	v32_5;
	vec_t	v32_6;
	vec_t	v32_7;

	vec_t	v32_8;
	vec_t	v32_9;

	vec_t	v32_10;
	vec_t	v32_11;
	vec_t	v32_12;	
	
	coef_t 	vcoef_shift;
	coef_t 	vcoef_shift2;
	coef_t	vcoef_bound;

	vprex_t vecPredRes1;
	vprex_t vecPredRes2;
	vprex_t vecPredRes3;

    __asm__("vpush {dw} modv0");
    __asm__("nop");
    __asm__("nop");
    __asm__("push {dw} a8");
    __asm__("mov #0x00000E07, a8");
    __asm__("nop");
    __asm__("nop");
    __asm__("vmova a8, modv0");
    __asm__("pop {dw} a8");
    __asm__("nop");
    __asm__("nop");


    vcoef_shift = vlddw_c32_clone_1dw (&demod_shift);

    vcoef_bound = vlddw_c32_clone_1dw (&u32_qam256_boundary1);
    v32_1 = vfillw_c32_v40 (8, vcoef_bound);

    vcoef_bound = vlddw_c32_clone_1dw (&u32_qam256_boundary2);
    v32_2 = vfillw_c32_v40 (8, vcoef_bound);

    vcoef_bound = vlddw_c32_clone_1dw (&u32_qam256_boundary3);
    v32_3 = vfillw_c32_v40 (8, vcoef_bound);

    for(s32_iloop = 0; s32_iloop <= (u32_equ_len>>4); s32_iloop++)
    #pragma dsp_ceva_trip_count_min = 1
    #pragma dsp_ceva_unroll = 2
    {
        v32_4 = vlddw_v32 (8, stp_equout_in + 16*s32_iloop, 0);       // 16个dw
        v32_5 = vshiftw_v32_c32_v32_ar (8, v32_4, vcoef_shift);      // 0/1，实部虚部都已经移位

        v32_6 = vabs16_v32_v32 (8, v32_4);
        v32_4 = vsub16_v32_v32_v32 (8, v32_1, v32_6);
        v32_7 = vshiftw_v32_c32_v32_ar (8, v32_4, vcoef_shift);      // 2/3，实部虚部都已经移位

        v32_6 = vabs16_v32_v32 (8, v32_4);
        v32_4 = vsub16_v32_v32_v32 (8, v32_2, v32_6);
        v32_8 = vshiftw_v32_c32_v32_ar (8, v32_4, vcoef_shift);      // 4/5，实部虚部都已经移位

        v32_6 = vabs16_v32_v32 (8, v32_4);
        v32_4 = vsub16_v32_v32_v32 (8, v32_3, v32_6);
        v32_9 = vshiftw_v32_c32_v32_ar (8, v32_4, vcoef_shift);      // 6/7，实部虚部都已经移位

        v32_10 = vtransdw_v32_v32_v32_v32_uimm3_v32_4dw (2, v32_5, v32_7, v32_8, v32_9, 0);
        vstb_v32_mw_vuX_h (8, 0, v32_10, s8p_llr_out + u32_fill_len + s32_iloop*128);
        vstb_v32_mw_vuX_h (8, 1, v32_10, s8p_llr_out + u32_fill_len + s32_iloop*128 + 64);

        v32_10 = vtransdw_v32_v32_v32_v32_uimm3_v32_4dw (2, v32_5, v32_7, v32_8, v32_9, 2);
        vstb_v32_mw_vuX_h (8, 0, v32_10, s8p_llr_out + u32_fill_len + s32_iloop*128 + 16);
        vstb_v32_mw_vuX_h (8, 1, v32_10, s8p_llr_out + u32_fill_len + s32_iloop*128 + 16 + 64);

        v32_10 = vtransdw_v32_v32_v32_v32_uimm3_v32_4dw (2, v32_5, v32_7, v32_8, v32_9, 4);
        vstb_v32_mw_vuX_h (8, 0, v32_10, s8p_llr_out + u32_fill_len + s32_iloop*128 + 32);
        vstb_v32_mw_vuX_h (8, 1, v32_10, s8p_llr_out + u32_fill_len + s32_iloop*128 + 32 + 64);

        v32_10 = vtransdw_v32_v32_v32_v32_uimm3_v32_4dw (2, v32_5, v32_7, v32_8, v32_9, 6);
        vstb_v32_mw_vuX_h (8, 0, v32_10, s8p_llr_out + u32_fill_len + s32_iloop*128 + 48);
        vstb_v32_mw_vuX_h (8, 1, v32_10, s8p_llr_out + u32_fill_len + s32_iloop*128 + 48 + 64);
    }

    __asm__("nop");
    __asm__("nop");
    __asm__("vpop {dw} modv0");
    __asm__("nop");
    __asm__("nop");

}

#if 1
OAL_ITCM_CODE_SECTION
VOID dl_demod(complex_s16_t* stp_equout_in, SINT8* s8p_llr_out,UINT32 u32_fill_len,UINT32 u32_equ_len,MOD_TYPE_E mod_type)
{
	UINT32 u32_permute[2];
	SINT32 s32_iloop;
    vec_t   v32_1;
    vec_t   v32_2;
    vec_t   v32_3;
    vec_t   v32_4;
	vec_t	v32_5;
	vec_t	v32_6;
	vec_t	v32_7;

	vec_t	v32_8;
	vec_t	v32_9;

	vec_t	v32_10;
	vec_t	v32_11;
	vec_t	v32_12;	
	
	coef_t 	vcoef_shift;
	coef_t 	vcoef_shift2;

	coef_t	vcoef_bound;

	vprex_t vecPredRes1;
	vprex_t vecPredRes2;
	vprex_t vecPredRes3;

	SINT32 demod_shift=0;
	SINT32 demod_shift2=-2;
	
	UINT32 u32_equ_len_fix=u32_equ_len;
	
	//DL_GET_TMR2_START;
	if(0!=(u32_equ_len&0xF))
	{
		u32_equ_len_fix=u32_equ_len+15;   //多运行一轮 llr out，所以llr_out需要预留一点空间
	}

	switch(mod_type)
	{
		case BPSK:
			dl_demod_bpsk(stp_equout_in, s8p_llr_out, u32_fill_len, u32_equ_len_fix);
			break;
		case QPSK:
		{
			dl_demod_qpsk(stp_equout_in, s8p_llr_out, u32_fill_len, u32_equ_len_fix);
		    break;
		}		
		case QAM16:
			dl_demod_qam16(stp_equout_in, s8p_llr_out, u32_fill_len, u32_equ_len_fix);
			break;
		case QAM64:
		{
			dl_demod_qam64(stp_equout_in, s8p_llr_out, u32_fill_len, u32_equ_len_fix);
			break;
		}

		case QAM256:
		{
			dl_demod_qam256(stp_equout_in, s8p_llr_out, u32_fill_len, u32_equ_len_fix);
			break;
		}
				
	}
	//DL_GET_TMR2_END;
	//DL_PRINT_TMR2_DURATION(0xA31B);

	return;
	
}

OAL_ITCM_CODE_SECTION
VOID dl_demod_with_snr(complex_s16_t* stp_equout_in, SINT8* s8p_llr_out, UINT16_PTR u16p_snr, UINT32 u32_fill_len,UINT32 u32_equ_len,MOD_TYPE_E mod_type)
{
	UINT32 u32_permute[0];
	complex_s16_t st_scale_factor;
	SINT32 s32_iloop = 0, s32_snr_max, s32_snr_mean;
	SINT32 s32_shift = 0;
	SINT32 s32_bean = 0x10000;
	STATIC SINT32 s32_mcs4_demod_cnt = 0;
	SINT16 s16_snr_mean = 0;

	coef_t vcoef_bean, vcoef_shift, vcoef_bound;
	vec_t v32_1;
	vec_t v32_2;
	vec_t v32_3;
	vec_t v32_4;
	vec_t v32_5;

	if(QAM16 == mod_type)
	{
		st_scale_factor.s16_re = 18236; //18236 = (1024/460)*2^13
	}
	else if(QAM64 == mod_type)
	{
		st_scale_factor.s16_re = 18724; //18724 = (512/224)*2^13
	}
	else
	{
		st_scale_factor.s16_re = 8192;
	}

	st_scale_factor.s16_im = 0;

	#if 0
	_ks_cxv_mul_cx(stp_equout_in, u32_equ_len, &st_scale_factor, 0, s_st_temp_buffer[0]);
	_ks_shift_s16v((CONST_SINT16_PTR)s_st_temp_buffer[0], u32_equ_len*2, (2 + 4), (CONST_SINT16_PTR)stp_equout_in);
	_ks_shift_s16v((CONST_SINT16_PTR)stp_equout_in, u32_equ_len*2, -4, s_st_temp_buffer[0]);
	#else
	_ks_cxv_mul_cx(stp_equout_in, u32_equ_len, &st_scale_factor, 0, stp_equout_in);
	_ks_shift_s16v((CONST_SINT16_PTR)stp_equout_in, u32_equ_len*2, (2 + 4), (CONST_SINT16_PTR)stp_equout_in);
	_ks_shift_s16v((CONST_SINT16_PTR)stp_equout_in, u32_equ_len*2, -4, (CONST_SINT16_PTR)stp_equout_in);
	#endif
	//s32_snr_mean = ks_cxv_rssi_calc(s_st_temp_buffer[0], HALF_SC_NUM);
	//KS_LOG(0, 0xA21A, (UINT16)(exp_acW_acZ ((SINT32)s32_snr_mean)));


	if(BPSK == mod_type)
	{
		CEVA_LTE_COMM_DEMAP_BPSK_SNR(stp_equout_in,		 // [i], detector estimations for demodulation
									 12,						 // symbols precision, 16..12 - Q16..Q12
									 u16p_snr, 				 // [i], SNR per tone
									 stp_equout_in,		 // [o], output soft-bits
									 u32_equ_len,				 // number of symbols to demodulate
									 0);

		u32_permute[0] = 0x67452301;
		u32_permute[1] = 0xEFCDAB89;
		v32_1 = vlddw_v32_clone(2, u32_permute, 0);

		u32_permute[0] = 0xeca86420;
		u32_permute[1] = 0xfdb97531;
		v32_2 = vlddw_v32_clone(2, u32_permute, 0);

		for(s32_iloop = 0; s32_iloop < u32_equ_len; )
    #pragma dsp_ceva_trip_count_min = 1
    #pragma dsp_ceva_unroll = 2
		{
			v32_3 = vlddw_v32 (8, stp_equout_in + s32_iloop, 0);

			v32_4 = vshiftw_v32_imm5_v32_ar(8, v32_3, -9);

			v32_5 = vpermutew_v32_v32 (8, v32_1, v32_4);
			v32_3 = vadd16_v32_v32_v32 (8, v32_4, v32_5);

			v32_4 = vpermutew_v32_v32 (8, v32_2, v32_3);

			vstb_v32_mw_concat (4, v32_4, s8p_llr_out + s32_iloop + u32_fill_len);

			s32_iloop += 16;
		}
	}
	else if(QPSK == mod_type)
	{
		CEVA_LTE_COMM_DEMAP_QPSK_SNR(stp_equout_in,		 // [i], detector estimations for demodulation
									 12,						 // symbols precision, 16..12 - Q16..Q12
									 u16p_snr, 				 // [i], SNR per tone
									 s8p_llr_out + u32_fill_len, // [o], output soft-bits
									 u32_equ_len,				 // number of symbols to demodulate
									 0);
	}
	else if(QAM16 == mod_type)
	{
		CEVA_LTE_COMM_DEMAP_QAM16_SNR(stp_equout_in,		  // [i], detector estimations for demodulation
									  12,						  // symbols precision, 16..12 - Q16..Q12
									  u16p_snr,				  // [i], SNR per tone
									  s8p_llr_out + u32_fill_len, // [o], output soft-bits
									  u32_equ_len,				  // number of symbols to demodulate
									  0);
	}
	else if(QAM64 == mod_type)
	{
		CEVA_LTE_COMM_DEMAP_QAM64_SNR(stp_equout_in,		  // [i], detector estimations for demodulation
									  12,						  // symbols precision, 16..12 - Q16..Q12
									  u16p_snr,				  // [i], SNR per tone
									  s8p_llr_out + u32_fill_len, // [o], output soft-bits
									  u32_equ_len,				  // number of symbols to demodulate
									  0);
	}
	else if(QAM256 == mod_type)
	{
		dl_demod_qam256(stp_equout_in, s8p_llr_out, u32_fill_len, u32_equ_len);
	}
	else
	{
		OAL_ASSERT(0, "");
	}
}


#endif
OAL_ITCM_CODE_SECTION
VOID dl_freq_comp_seq_gen(OUT cfo_seq_t* stp_freq_seq, IN SINT32 s32_angle, IN BOOLEAN b_is_coarse, IN BOOLEAN b_gen_phase, IN BOOLEAN b_gen_seq)
{
    UINT32 u32_iloop, u32_comp_flag = OAL_FALSE , u32_cfo_seq_len = SLOT_LEN;
    SINT32 s32_round_fix, s32_acc_value = 0, s32_idx = 0;

	if(b_is_coarse)
	{
		s32_angle <<= 4;
		u32_cfo_seq_len >>= 4;
		s32_acc_value = 8;
	}

	#if 1
	s32_round_fix = 0x1000;
	#else
  	if(s32_angle >= 0)
    {
        s32_round_fix = 0x1000;
    }
    else
    {
        s32_round_fix = -1*0x1000;
    }
    #endif

	if(b_gen_seq)
	{
	    for(u32_iloop = 0; u32_iloop < u32_cfo_seq_len; u32_iloop++)
	    {
	        s32_idx = ((s32_acc_value + s32_round_fix)>>13) & 0x3FF;
	        stp_freq_seq->st_cfo_seq[u32_iloop] = s_st_exp_seq[s32_idx];
	        s32_acc_value += s32_angle;
	    }
    }

	if(b_gen_phase)
	{
		s32_angle >>= (4 * (UINT32)b_is_coarse);

		for(u32_iloop = 0; u32_iloop < TOTAL_SYM_NUM; u32_iloop++)
		{
			#if 1//def MIMO2
			if(b_is_coarse)
			{
				//pilot1 phase is zero, pilot0 & pilot2 is phase max
				//stp_freq_seq->st_slot_phase[u32_iloop] = s_st_exp_seq[((s32_angle * SLOT_LEN * (u32_iloop + 1) + s32_round_fix)>>13) & 0x3FF];
				stp_freq_seq->st_slot_phase[u32_iloop] = s_st_exp_seq[((s32_angle * SLOT_LEN * (u32_iloop - PILOT_DIS_SYM) + s32_round_fix)>>13) & 0x3FF];
			}
			else
			{
				stp_freq_seq->st_slot_phase[u32_iloop] = s_st_exp_seq[((s32_angle * SLOT_LEN * (u32_iloop - PILOT_DIS_SYM) + s32_round_fix)>>13) & 0x3FF];
			}
			#ifdef MIMO2
			stp_freq_seq->st_plcp_phase = s_st_exp_seq[((s32_angle * SLOT_LEN * (-(PILOT_DIS_SYM + 1)) + s32_round_fix)>>13) & 0x3FF];
			#else
			stp_freq_seq->st_plcp_phase = s_st_exp_seq[((s32_angle * SLOT_LEN * (-(PILOT_DIS_SYM - 1)) + s32_round_fix)>>13) & 0x3FF];
			#endif
			#else
			if(b_is_coarse)
			{
				//stp_freq_seq->st_slot_phase[u32_iloop] = s_st_exp_seq[((s32_angle * SLOT_LEN * (u32_iloop + 1) + s32_round_fix)>>13) & 0x3FF];
				stp_freq_seq->st_slot_phase[u32_iloop] = s_st_exp_seq[((s32_angle * SLOT_LEN * (u32_iloop) + s32_round_fix)>>13) & 0x3FF];
			}
			else
			{
				stp_freq_seq->st_slot_phase[u32_iloop] = s_st_exp_seq[((s32_angle * SLOT_LEN * (u32_iloop) + s32_round_fix)>>13) & 0x3FF];
			}
			stp_freq_seq->st_plcp_phase = s_st_exp_seq[((s32_angle * SYMBOL_LEN * (-1) + s32_round_fix)>>13) & 0x3FF];
			#endif
		}
		stp_freq_seq->st_symbol_phase = s_st_exp_seq[((s32_angle * SYMBOL_LEN * (1) + s32_round_fix)>>13) & 0x3FF];

	}
	//ks_log(0, 0xa117, (UINT16)(s32_angle >> 16));
	//ks_log(0, 0xa117, (UINT16)(s32_angle));	
}

OAL_ITCM_CODE_SECTION
VOID dl_freq_comp_seq_gen_symbol(OUT cfo_seq_t* stp_freq_seq, IN SINT32 s32_angle, IN BOOLEAN b_is_coarse, IN BOOLEAN b_gen_phase)
{
    UINT32 u32_iloop, u32_comp_flag = OAL_FALSE , u32_cfo_seq_len = SLOT_LEN;
    SINT32 s32_round_fix, s32_acc_value = 0, s32_idx = 0;

	if(b_is_coarse)
	{
		s32_angle <<= 4;
		u32_cfo_seq_len >>= 4;
		s32_acc_value = 8;
	}

	#if 1
	s32_round_fix = 0x1000;
	#else
  	if(s32_angle >= 0)
    {
        s32_round_fix = 0x1000;
    }
    else
    {
        s32_round_fix = -1*0x1000;
    }
    #endif

	if(b_gen_phase)
	{
		s32_angle >>= (4 * (UINT32)b_is_coarse);
		for(u32_iloop = 0; u32_iloop < TOTAL_SYM_NUM; u32_iloop++)
		{
			#if 1//def MIMO_NEW
			if(b_is_coarse)
			{
				//pilot1 phase is zero, pilot0 & pilot2 is phase max
				//stp_freq_seq->st_slot_phase[u32_iloop] = s_st_exp_seq[((s32_angle * SLOT_LEN * (u32_iloop + 1) + s32_round_fix)>>13) & 0x3FF];
				s32_idx = ((s32_angle * SLOT_LEN * (u32_iloop - PILOT_DIS_SYM) + s32_round_fix)>>13);
				//KS_LOG(0, 0XDDDD, s32_idx);
				stp_freq_seq->st_slot_phase[u32_iloop] = s_st_exp_seq[s32_idx & 0x3FF];
				//KS_LOG(0, 0XDDDD, stp_freq_seq->st_slot_phase[u32_iloop].s16_im);
			}
			else
			{
				stp_freq_seq->st_slot_phase[u32_iloop] = s_st_exp_seq[((s32_angle * SLOT_LEN * (u32_iloop - PILOT_DIS_SYM) + s32_round_fix)>>13) & 0x3FF];
			}
			#ifdef MIMO2
			stp_freq_seq->st_plcp_phase = s_st_exp_seq[((s32_angle * SLOT_LEN * (-(PILOT_DIS_SYM + 1)) + s32_round_fix)>>13) & 0x3FF];
			#else
			stp_freq_seq->st_plcp_phase = s_st_exp_seq[((s32_angle * SLOT_LEN * (-(PILOT_DIS_SYM - 1)) + s32_round_fix)>>13) & 0x3FF];
			#endif
			#else
			if(b_is_coarse)
			{
				//stp_freq_seq->st_slot_phase[u32_iloop] = s_st_exp_seq[((s32_angle * SLOT_LEN * (u32_iloop + 1) + s32_round_fix)>>13) & 0x3FF];
				stp_freq_seq->st_slot_phase[u32_iloop] = s_st_exp_seq[((s32_angle * SLOT_LEN * (u32_iloop) + s32_round_fix)>>13) & 0x3FF];
			}
			else
			{
				stp_freq_seq->st_slot_phase[u32_iloop] = s_st_exp_seq[((s32_angle * SLOT_LEN * (u32_iloop) + s32_round_fix)>>13) & 0x3FF];
			}
			stp_freq_seq->st_plcp_phase = s_st_exp_seq[((s32_angle * SYMBOL_LEN * (-1) + s32_round_fix)>>13) & 0x3FF];
			#endif
		}
		stp_freq_seq->st_symbol_phase = s_st_exp_seq[((s32_angle * SYMBOL_LEN * (1) + s32_round_fix)>>13) & 0x3FF];


	}
	//ks_log(0, 0xa117, (UINT16)(s32_angle >> 16));
	//ks_log(0, 0xa117, (UINT16)(s32_angle));	
}


OAL_ITCM_CODE_SECTION
SINT32 dl_freq_offset_est_cca(IN complex_s16_t* st_input_seq_0, IN complex_s16_t* st_input_seq_1, IN SINT32 s32_gap_len, IN BOOLEAN b_is_coarse)
{
    SINT32 s32_idx, s32_angle = 0, s32_temp;
    UINT32 u32_iloop, u32_comp_flag = OAL_FALSE , u32_cfo_seq_len = SLOT_LEN;
    SINT32 s32_round_fix, s32_acc_value = 0;
    SINT32 s32_cfo[2];
    SINT32 s32_shift, s32_shift_remain;
    SINT32 s32_foff = 0;

	s32_cfo[0] = ((SINT32)st_input_seq_0->s16_re * st_input_seq_1->s16_re) + ((SINT32)st_input_seq_0->s16_im * st_input_seq_1->s16_im);
	s32_cfo[1] = -((SINT32)st_input_seq_0->s16_re * st_input_seq_1->s16_im) + ((SINT32)st_input_seq_0->s16_im * st_input_seq_1->s16_re);

    s32_angle = ks_angle(s32_cfo[0], s32_cfo[1]);
    s32_angle = ((s32_angle * 32768) / s32_gap_len) >> 8;

	//freq offset, UNIT: Hz, only use to debug
    s32_foff = ((- s32_angle * 937) >> 8) ;
	KS_LOG(0, 0xA117, (UINT16)s32_foff);

	dl_freq_comp_seq_gen(&s_st_cfo_seq, s32_angle, b_is_coarse, OAL_TRUE, OAL_TRUE);

    return s32_angle;
}

OAL_ITCM_CODE_SECTION
SINT32 dl_freq_offset_est(IN complex_s16_t* st_input_seq_0, IN complex_s16_t* st_input_seq_1, IN UINT32 u32_seq_len, IN SINT32 s32_gap_len, SINT32 s32_comp_angle , UINT32 u32_ant_id, BOOLEAN b_gen_seq)
{
    SINT32 s32_idx, s32_angle = 0, s32_temp;
    UINT32 u32_iloop, u32_comp_flag = OAL_FALSE , u32_cfo_seq_len = SLOT_LEN;
    SINT32 s32_round_fix, s32_acc_value = 0;
    SINT32 s32_cfo[2];
    SINT32 s32_shift, s32_shift_remain;
    SINT32 s32_foff = 0;

    vec_t v32_0;
    vec_t v32_1;
    vec_t v32_2;

    vec40_t v40_0;
    vec40_t v40_1;
    vec40_t v40_shift;

    coef_t  vcoef_shift;
    coef_t  vcoef_shift_remain;

	if(u32_seq_len > 1000)
	{
    	s32_shift = (2*s_s16_shift_value[u32_ant_id] - 10);
    }
    else
    {
	    s32_shift = (2*s_s16_shift_value[u32_ant_id] - 10);
    }
    s32_shift_remain = 0;
    
    if(s32_shift > 7)
    {
        s32_shift_remain = s32_shift - 7;
        s32_shift = 7;
    }
    else if(s32_shift < -8)
    {
        s32_shift_remain = s32_shift + 8;
        s32_shift = -8;
    }

    vcoef_shift = vlddw_c32_clone_1dw (&s32_shift);
    v40_shift = vfillw_c32_v40 (8, vcoef_shift);

    vcoef_shift_remain = vlddw_c32_clone_1dw (&s32_shift_remain);

    v40_0 = vclr_v32 (8);
    v40_1 = vclr_v32 (8);

    /*est freq offset*/
    for(u32_iloop = 0; u32_iloop < u32_seq_len; )
    {
        v32_0 = vlddw_v32 (8, st_input_seq_0 + u32_iloop, 0);
        v32_1 = vlddw_v32 (8, st_input_seq_1 + u32_iloop, 0);

        v40_1 = vmacx_v32_v32_v40_v40_conj (8, v32_0, 0, v32_1, 0, v40_1, v40_0);

        u32_iloop += 16;
    }

    v40_0 = vadd40_v40_v40_v40_out (8, v40_0, v40_1);

    v32_0 = vshiftx40_v40_v40_v32_w_s (4, v40_0, v40_shift, 0);

    v32_1 = vshift_v32_c32_v32_ar (8, v32_0, vcoef_shift_remain);

    v32_0 = vaddintx_v32_v32 (4, v32_1);

    v32_1 = vvmov_v40_v32_vuX (1, v32_0);
    v32_2 = vadd32_v32_v32_v32 (2, v32_1, v32_0);

    vstdw_v32_vuX (2, 0, v32_2, 0, s32_cfo);

    #if 0
	ks_log(0, 0xa218, (UINT16)(s32_cfo[0] >> 16));
	ks_log(0, 0xa218, (UINT16)(s32_cfo[0]));
	ks_log(0, 0xa218, (UINT16)(s32_cfo[1] >> 16));
	ks_log(0, 0xa218, (UINT16)(s32_cfo[1]));
	#endif
	
    s32_angle = ks_angle(s32_cfo[0], s32_cfo[1]);
    s32_angle = ((s32_angle*32768) / s32_gap_len)>>8;
    s32_temp  = s32_angle + s32_comp_angle;
	//freq offset, UNIT: Hz, only use to debug
	s32_foff = ((- s32_temp * 937) >> 8) ;  //1875*32768 = 61440000

    if(s32_comp_angle)
    {
		//freq offset, UNIT: Hz, only use to debug
		//s32_temp = 204;//(1500 << 8)/1875;
		KS_LOG(0, 0xA217, (UINT16)s32_foff);

	}
	else
	{
		//freq offset, UNIT: Hz, only use to debug
		KS_LOG(0, 0xA117, (UINT16)s32_foff);
		
	}

	if(b_gen_seq)
	{
		#if FREQ_OFFSET_COARSE
		dl_freq_comp_seq_gen(&s_st_cfo_seq, s32_temp, OAL_TRUE, OAL_TRUE, OAL_TRUE);
		#else
		dl_freq_comp_seq_gen(&s_st_cfo_seq, s32_temp, OAL_FALSE, OAL_TRUE, OAL_TRUE);
		#endif
	}
	return s32_temp;
}

OAL_ITCM_CODE_SECTION
VOID dl_freq_offset_comp(complex_s16_t* stp_input, complex_s16_t* stp_output, complex_s16_t* stp_cfo_seq, UINT32 u32_len, BOOLEAN b_is_coarse)
{
	SINT32 s32_iloop = 0;
	UINT32 u32_permute[2] = {0x10101010, 0x10101010};

	vec_t	v32_1;
	vec_t	v32_2;
	vec_t	v32_3;
	vec_t	v32_4;

	vec40_t v40_1;
	vec40_t v40_2;

	if(b_is_coarse)
	{
        __asm__("nop");
        __asm__("nop");
        __asm__("nop");
        __asm__("nop");
        __asm__("push {dw} a8");
        __asm__("vpush {dw} modv0");
        __asm__("mov #0x00000C20,a8");/* 功能寄存器压栈 */
        __asm__("nop");
        __asm__("nop");
        __asm__("nop");
        __asm__("nop");

        __asm__("nop");
        __asm__("nop");
        __asm__("nop");
        __asm__("nop");
        __asm__("vmova a8, modv0");/* 功能寄存器设置 */
        __asm__("nop");
        __asm__("nop");
        __asm__("nop");
        __asm__("nop");
        
		v32_1 = vlddw_v32_clone (2, u32_permute, 0);
		for(s32_iloop = 0; s32_iloop < (u32_len >> 4); s32_iloop++)
		#pragma dsp_ceva_unroll = 2
		#pragma dsp_ceva_trip_count_min = 1
		{
			v32_2 = vlddw_v32_clone(1, stp_cfo_seq + s32_iloop, 0);
			v32_3 = vpermutew_v32_v32 (8, v32_1, v32_2);
			v32_4 = vlddw_v32 (8, stp_input + 16 * s32_iloop, 0);
			v40_1 = vmpyxp_v32_v32_v32(8,v32_3,0,v32_4,0);
			vstdw_v32_concat(8, v40_1, 0, &stp_output[16 * s32_iloop]);
		}

        __asm__("nop");
        __asm__("nop");
        __asm__("nop");
        __asm__("nop");
        __asm__("vpop {dw} modv0");
        __asm__("pop {dw} a8");
        __asm__("nop");
        __asm__("nop");
        __asm__("nop");
        __asm__("nop");
	}
	else
	{
	    ks_cxv_mul_cxv(stp_input, &stp_cfo_seq[0], u32_len, 0, stp_output);
	}

}


OAL_ITCM_CODE_SECTION
VOID dl_ics_init()
{
	//g_u32_ics_frm_cnt = 0;
	#if 0
	UINT32 u32_reg_read;
	__asm__("nop");
	__asm__("nop");
	__asm__("push {dw} a0");	
	__asm__("push {dw} a1");	
	__asm__("nop");
	__asm__("nop");
	
	__asm__("mov sp, a0");
	__asm__("nop");
	
	__asm__("mov #0x1724e020, a1");/* 功能寄存器压栈 */
	__asm__("nop");
	__asm__("nop");	

	__asm__("mov a1, sp");
	__asm__("nop");	
	__asm__("nop");
	__asm__("vpush {dw} modv0");
	__asm__("vpush {dw} modv1");
	__asm__("vpush {dw} modv2");
	
	__asm__("mov a0, sp");
	__asm__("nop");
	__asm__("nop");

	__asm__("pop {dw} a1");
	__asm__("pop {dw} a0");
	
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");

	u32_reg_read = READ_REG32(0x1724e01C);
	KS_LOG(0, 0xA151, (UINT16)(u32_reg_read >> 16)); 
	KS_LOG(0, 0xA151, (UINT16)(u32_reg_read)); 

	u32_reg_read = READ_REG32(0x1724e014);
	KS_LOG(0, 0xA152, (UINT16)(u32_reg_read >> 16)); 
	KS_LOG(0, 0xA152, (UINT16)(u32_reg_read)); 
	#endif
	g_u32_ics_fft_cnt = 0;
	g_u32_ics_left_cnt = 0;
	g_u32_ics_left_sym = 0;
	g_u32_ics_head_crc = 0;
	g_u32_ics_frm_cnt = 0;
	g_u32_ics_flag = ICS_CCA_DETECT;
	g_u32_ics_head_crc = OAL_CRC_FAILURE;
	g_u32_ics_sym_bmp = 0; //symbol processed bitmap(eq or channel est finish); bit 0\6\12: pilot symbols bitmap; bit 1 ~ 5, 7 ~ 11: plcp\data symbols bitmap
	g_u32_ics_crc_bmp = 0; //cb crc bitmap: bit0->cb0 to max cb bit
	g_u32_ics_cb_cnt = 0; //cb crc bitmap: bit0->cb0 to max cb bit
	
	s_u32_pingpong_idx = 0;
	s_u32_rxdfe_restart = OAL_FALSE;
	s_u32_data_left_cnt = 0;
	s_u32_data_cb_start_cnt = 0;
	s_u32_data_sym_pre_cnt = 0;
	s_u32_data_sym_eq_cnt = 0;
	s_st_plcp_info.u16_is_valid = OAL_FALSE;
	s_u32_data_cb_end_cnt = 0;
	s_u32_dec_cnt = 0;

	ks_oal_mem_set((VOID_PTR)s_st_rx_buffer.st_ics_left, 0x0, (ICS_RSV_LEN << 2));	
	ks_oal_mem_set((VOID_PTR)s_st_rx_buffer.st_ics_left2, 0x0, (ICS_RSV_LEN << 2));	
}
OAL_ITCM_CODE_SECTION
SINT32 dl_cfo_judge(IN SINT32 s32_cfo, IN SINT32 s32_cfo_base)
{
	if((s32_cfo - s32_cfo_base > CFO_ANGLE_THRES) || (s32_cfo - s32_cfo_base < -CFO_ANGLE_THRES))
	{
		//out of range
		return s32_cfo_base;
	}
	else
	{
		return s32_cfo;	
	}
}
OAL_ITCM_CODE_SECTION
VOID dl_head_pilot_proc(complex_s16_t* stp_input)
{
    SINT32 s32_rssi = 0;
    UINT32 u32_i = 0;
    UINT32 u32_ant_id;
	complex_s16_t* stp_temp = (complex_s16_t*)stp_input;
	UINT32 u32_delta_of_ants = DELTA_OF_ANT_BUF;    
	UINT32 u32_pilot_num[RX_ANT_NUM] = {0};
	UINT32 u32_sym_num[RX_ANT_NUM] = {0};
	UINT32 u32_pilot_cnt = 0;
    SINT32 s32_cca_angle_avg_bak = 0;
    SINT32 s32_angle_tmp = 0;
    SINT32 s32_angle_node_tbl = 0;
	node_info_t* stp_node_info;
	OAL_IRQ_SAVE_AREA;
	
    //s32_rssi = ks_cxv_rssi_calc(stp_temp, SYMBOL_LEN);
    g_zzw_ch_status.gain = s_u8_rx_gain;
 
	/*gen cca cfo seq for 2nd cfo est start*/
	s_s32_angle_cca_avg = 0;
	s_s32_angle_cp_avg = 0;

	for(u32_ant_id = 0; u32_ant_id < RX_ANT_NUM; u32_ant_id++)
	{
		s_s32_angle_cp[u32_ant_id] = 0;
		if(g_u32_cca_detected & (1 << u32_ant_id))
		{
			s_s32_angle_cca_avg += s_s32_angle_cca[u32_ant_id];
		}
	}
	
	if(g_u32_rx_ant_num > 1)
	{	
		//temp here , because g_u32_rx_ant_num less than 2 
		s_s32_angle_cca_avg >>= (g_u32_rx_ant_num - 1);
	}

	#if FREQ_OFFSET_COARSE
	dl_freq_comp_seq_gen(&s_st_cfo_seq, s_s32_angle_cca_avg, OAL_TRUE, OAL_TRUE, OAL_TRUE);
	#else
	dl_freq_comp_seq_gen(&s_st_cfo_seq, s_s32_angle_cca_avg, OAL_FALSE, OAL_TRUE, OAL_TRUE);
	#endif
	/*gen cca cfo seq for 2nd cfo est end*/

	if(1)
	{
		s_s16_shift_value_min = 20;
		s_s16_shift_value_max = 0;

		for(u32_ant_id = 0; u32_ant_id < RX_ANT_NUM; u32_ant_id++)
		{
			stp_temp = stp_input + (u32_delta_of_ants * u32_ant_id + s_s32_sync_pos_off[u32_ant_id]);
			if(g_u32_cca_detected & (1 << u32_ant_id))
			{
				//if(s_u32_freq_ind)
				#if FUN_DC_REMOVE
				//dc remove
		        _ks_cxv_add_cx(stp_temp, SLOT_LEN, &s_st_dc[u32_ant_id], &s_st_temp_buffer[0][0]);
				s_s16_shift_value[u32_ant_id] = dl_scale_calc(&s_st_temp_buffer[0][0], SLOT_LEN);
				#else
				s_s16_shift_value[u32_ant_id] = dl_scale_calc(stp_temp, SLOT_LEN);
				#endif
				KS_LOG(0, 0xA11C, (UINT16)s_s16_shift_value[u32_ant_id]);
				if(s_s16_shift_value[u32_ant_id] < s_s16_shift_value_min)
				{
					s_s16_shift_value_min = s_s16_shift_value[u32_ant_id];
				}
				if(s_s16_shift_value[u32_ant_id] > s_s16_shift_value_max)
				{
					s_s16_shift_value_max = s_s16_shift_value[u32_ant_id];
				}
			}
		}
			
	}

	stp_node_info = (node_info_t*)node_tbl_find(g_u8_node_id);
	if(stp_node_info)
	{
		s32_angle_node_tbl = stp_node_info->s32_afc_angle;
	}
	/*pilot cp cfo est*/
	for(u32_ant_id = 0; u32_ant_id < RX_ANT_NUM; u32_ant_id++)
	{
		s_s32_angle_cp[u32_ant_id] = 0;
		if(g_u32_cca_detected & (1 << u32_ant_id))
		{
			//pilot num for cfo est
			if(s_s32_sync_position[u32_ant_id] < CCA_POS_FULL_FRAME)
			{
				u32_pilot_num[u32_ant_id] = 3;
				u32_sym_num[u32_ant_id] = 4;
			}
			else if((s_s32_sync_position[u32_ant_id] + (PILOT_DIS_SYM + 2) * SLOT_LEN + (CCA_LEN << 1)) < FRAME_LEN)
			{
				u32_pilot_num[u32_ant_id] = 2;
				u32_sym_num[u32_ant_id] = 4;

			}
			else
			{
				u32_pilot_num[u32_ant_id] = 1;
				u32_sym_num[u32_ant_id] = (FRAME_LEN - (s_s32_sync_position[u32_ant_id] + (CCA_LEN << 1))) / SLOT_LEN - 1;

			}

			//if(u32_pilot_num[u32_ant_id] > 1)
			{
				stp_temp = stp_input + (u32_delta_of_ants * u32_ant_id + s_s32_sync_pos_off[u32_ant_id]);
				#if 0
				for(u32_i = 0; u32_i < u32_pilot_num[u32_ant_id]; u32_i++)
				{
					//TEMP HERE, ONLY USE FIRST 2 PILOTS
					//dc remove & collect all cp for cfo calc
					#if FUN_DC_REMOVE
			        _ks_cxv_add_cx(stp_temp + PILOT_DIS_SYM * SLOT_LEN * (u32_i + 0), CP_LEN, &s_st_dc[u32_ant_id],\
			        		&s_st_temp_buffer[1][(u32_i + u32_pilot_cnt) * 2 * CP_LEN]);
					//dl_freq_offset_comp(&s_st_temp_buffer[1][(u32_i + u32_pilot_cnt) * 2 * CP_LEN], \
					//		&s_st_temp_buffer[1][(u32_i + u32_pilot_cnt) * 2 * CP_LEN], &(s_st_cfo_seq.st_cfo_seq), CP_LEN, OAL_TRUE);
		        
			        _ks_cxv_add_cx(stp_temp + PILOT_DIS_SYM * SLOT_LEN * (u32_i + 0) + SYMBOL_LEN, CP_LEN, &s_st_dc[u32_ant_id],\
			        		&s_st_temp_buffer[1][((u32_i + u32_pilot_cnt) * 2 + 1) * CP_LEN]);			        		
					//dl_freq_offset_comp(&s_st_temp_buffer[1][((u32_i + u32_pilot_cnt) * 2 + 1) * CP_LEN],\
					//		&s_st_temp_buffer[1][((u32_i + u32_pilot_cnt) * 2 + 1) * CP_LEN], (complex_s16_t* )(&(s_st_cfo_seq.st_cfo_seq)) + (SYMBOL_LEN >> 4), CP_LEN, OAL_TRUE);        									
					_ks_cxv_mul_cx(&s_st_temp_buffer[1][((u32_i + u32_pilot_cnt) * 2 + 1) * CP_LEN], CP_LEN, &(s_st_cfo_seq.st_symbol_phase), 0, \
							&s_st_temp_buffer[1][((u32_i + u32_pilot_cnt) * 2 + 1) * CP_LEN]);

	        		#endif
					s_s32_angle_cp[u32_ant_id] += dl_freq_offset_est(&s_st_temp_buffer[1][(u32_i + u32_pilot_cnt) * 2 * CP_LEN + CP_DELTA_LEN], \
							&s_st_temp_buffer[1][((u32_i + u32_pilot_cnt) * 2 + 1) * CP_LEN + CP_DELTA_LEN], \
		    				CP_LEN - CP_DELTA_LEN, SYMBOL_LEN, s_s32_angle_cca_avg, u32_ant_id, OAL_FALSE);	        		
        		}
				u32_pilot_cnt += u32_pilot_num[u32_ant_id];
				s_s32_angle_cp[u32_ant_id] /= (SINT32)u32_pilot_num[u32_ant_id];


        		#else
				for(u32_i = 0; u32_i < u32_sym_num[u32_ant_id]; u32_i++)
				{
					//TEMP HERE, ONLY USE FIRST 2 PILOTS
					//dc remove & collect all cp for cfo calc
					#if 1//FUN_DC_REMOVE
			        _ks_cxv_add_cx(stp_temp + SLOT_LEN * (u32_i + 0), CP_LEN, &s_st_dc[u32_ant_id],\
			        		&s_st_temp_buffer[1][(u32_i + u32_pilot_cnt) * 2 * CP_LEN]);
					//dl_freq_offset_comp(&s_st_temp_buffer[1][(u32_i + u32_pilot_cnt) * 2 * CP_LEN], \
					//		&s_st_temp_buffer[1][(u32_i + u32_pilot_cnt) * 2 * CP_LEN], &(s_st_cfo_seq.st_cfo_seq), CP_LEN, OAL_TRUE);
		        
			        _ks_cxv_add_cx(stp_temp + SLOT_LEN * (u32_i + 0) + SYMBOL_LEN, CP_LEN, &s_st_dc[u32_ant_id],\
			        		&s_st_temp_buffer[1][((u32_i + u32_pilot_cnt) * 2 + 1) * CP_LEN]);			        		
					//dl_freq_offset_comp(&s_st_temp_buffer[1][((u32_i + u32_pilot_cnt) * 2 + 1) * CP_LEN],\
					//		&s_st_temp_buffer[1][((u32_i + u32_pilot_cnt) * 2 + 1) * CP_LEN], (complex_s16_t* )(&(s_st_cfo_seq.st_cfo_seq)) + (SYMBOL_LEN >> 4), CP_LEN, OAL_TRUE);        									
					_ks_cxv_mul_cx(&s_st_temp_buffer[1][((u32_i + u32_pilot_cnt) * 2 + 1) * CP_LEN], CP_LEN, &(s_st_cfo_seq.st_symbol_phase), 0, \
							&s_st_temp_buffer[1][((u32_i + u32_pilot_cnt) * 2 + 1) * CP_LEN]);
					s32_angle_tmp = dl_freq_offset_est(&s_st_temp_buffer[1][(u32_i + u32_pilot_cnt) * 2 * CP_LEN + CP_DELTA_LEN], \
							&s_st_temp_buffer[1][((u32_i + u32_pilot_cnt) * 2 + 1) * CP_LEN + CP_DELTA_LEN], \
		    				CP_LEN - CP_DELTA_LEN, SYMBOL_LEN, s_s32_angle_cca_avg, u32_ant_id, OAL_FALSE);	 
							
					#else
					_ks_cxv_mul_cx(&s_st_temp_buffer[1][((u32_i + u32_pilot_cnt) * 2 + 1) * CP_LEN], CP_LEN, &(s_st_cfo_seq.st_symbol_phase), 0, \
							&s_st_temp_buffer[1][((u32_i + u32_pilot_cnt) * 2 + 1) * CP_LEN]);
					s32_angle_tmp = dl_freq_offset_est(&s_st_temp_buffer[1][(u32_i + u32_pilot_cnt) * 2 * CP_LEN + CP_DELTA_LEN], \
							&s_st_temp_buffer[1][((u32_i + u32_pilot_cnt) * 2 + 1) * CP_LEN + CP_DELTA_LEN], \
		    				CP_LEN - CP_DELTA_LEN, SYMBOL_LEN, s_s32_angle_cca_avg, u32_ant_id, OAL_FALSE);	 							
	        		#endif
					if(stp_node_info)
					{
						if((s32_angle_tmp - s32_angle_node_tbl > AFC_DELTA_THRES))
						{
							s32_angle_tmp = s32_angle_node_tbl + (AFC_DELTA_THRES >> 1 );
						}
						else if((s32_angle_tmp - s32_angle_node_tbl < - AFC_DELTA_THRES ))
						{
							s32_angle_tmp = s32_angle_node_tbl - (AFC_DELTA_THRES >> 1 );
						}
					}
    				s_s32_angle_cp[u32_ant_id] += s32_angle_tmp;
        		}
				u32_pilot_cnt += u32_sym_num[u32_ant_id];
				s_s32_angle_cp[u32_ant_id] /= (SINT32)u32_sym_num[u32_ant_id];
        		#endif
			}
			s_s32_angle_cp_avg += s_s32_angle_cp[u32_ant_id];

		}
	}

	if(g_u32_rx_ant_num > 1)
	{	
		//temp here , because g_u32_rx_ant_num less than 2 
		s_s32_angle_cp_avg >>= (g_u32_rx_ant_num - 1);
	}
	if(u32_pilot_cnt EQ 0)
	{
		s_s32_angle_cp_avg = s_s32_angle_cca_avg;
	}
	/*gen cca cfo seq for 2nd cfo est end*/
	#if FREQ_OFFSET_COARSE
	dl_freq_comp_seq_gen(&s_st_cfo_seq, s_s32_angle_cp_avg, OAL_TRUE, OAL_TRUE, OAL_FALSE);
	#else
	dl_freq_comp_seq_gen(&s_st_cfo_seq, s_s32_angle_cp_avg, OAL_FALSE, OAL_TRUE, OAL_FALSE);
	#endif

	//s32_cca_angle_avg_bak = s_s32_angle_cca_avg;
	s_s32_angle_avg = 0;
	//s_s32_angle_cp_avg = 0;
	
	for(u32_ant_id = 0; u32_ant_id < RX_ANT_NUM; u32_ant_id++)
	{
		#ifdef KD001_RF8242
		g_zzw_ch_status.rssi_sub[u32_ant_id] = 0;
		g_zzw_ch_status.pathloss_sub[u32_ant_id] = 0;
		g_zzw_ch_status.snr_sub[u32_ant_id * 2] = -20;
		g_zzw_ch_status.snr_sub[u32_ant_id * 2 + 1] = -20;
		#else
		g_zzw_ch_status.rssi_sub[u32_ant_id] = RSSI_THRES_LOW;
		g_zzw_ch_status.pathloss_sub[u32_ant_id] =  -RSSI_THRES_LOW;
		g_zzw_ch_status.snr_sub[u32_ant_id * 2] = -20;
		g_zzw_ch_status.snr_sub[u32_ant_id * 2 + 1] = -20;
		#endif
		stp_temp = stp_input + (u32_delta_of_ants * u32_ant_id + s_s32_sync_pos_off[u32_ant_id]);
		if(g_u32_cca_detected & (1 << u32_ant_id))
		{
			s32_rssi = ks_cxv_rssi_calc(stp_temp, 256);
			#ifdef KD001_RF8242
			//temp here, do nothing
			#else
			g_zzw_ch_status.rssi_sub[u32_ant_id] = ((10*(ks_logx(10, s32_rssi)>>8))>>18) - 105 - (SINT8)(g_zzw_ch_status.gain);
			g_zzw_ch_status.pathloss_sub[u32_ant_id] = 20 - g_zzw_ch_status.rssi_sub[u32_ant_id];
			#endif
			if(READ_REG32(ASSERT_4500_ADDR) EQ RX_DUMP_RX_ANT_PATHLOSS)
			{
				if(g_zzw_ch_status.pathloss_sub[u32_ant_id] < READ_REG32(ASSERT_4500_ADDR + 4))
				{
					OAL_ASSERT(0, "");
				}
			}
			
			if(u32_pilot_num[u32_ant_id] > 1)
			{
				
				for(u32_i = 0; u32_i < u32_pilot_num[u32_ant_id]; u32_i++)
				{
					#if 1//FUN_DC_REMOVE
			        _ks_cxv_add_cx(stp_temp + PILOT_DIS_SYM * SLOT_LEN * (u32_i + 0) + CP_LEN, SYMBOL_LEN, &s_st_dc[u32_ant_id],\
			        		&s_st_temp_buffer[0][u32_i * SYMBOL_LEN]);
					_ks_cxv_mul_cx(&s_st_temp_buffer[0][u32_i * SYMBOL_LEN], SYMBOL_LEN, &(s_st_cfo_seq.st_slot_phase[u32_i * PILOT_DIS_SYM]), 0, \
							&s_st_temp_buffer[0][u32_i * SYMBOL_LEN]);			        		
					#else
					//if(u32_i NEQ 1)
					{
						_ks_cxv_mul_cx(stp_temp + PILOT_DIS_SYM * SLOT_LEN * (u32_i + 0) + CP_LEN, SYMBOL_LEN, &(s_st_cfo_seq.st_slot_phase[u32_i * PILOT_DIS_SYM]), 0, \
								&s_st_temp_buffer[0][u32_i * SYMBOL_LEN]);
					}
					#endif
				}
				

				//calc cfo by pilot 1 & 2
				s_s32_angle_pilot[u32_ant_id][0] = dl_freq_offset_est(&s_st_temp_buffer[0][0], &s_st_temp_buffer[0][SYMBOL_LEN], SYMBOL_LEN, \
						PILOT_DIS_SYM * SLOT_LEN, s_s32_angle_cp_avg, u32_ant_id, OAL_FALSE);
						//6 * SLOT_LEN, s32_cca_angle_avg_bak, u32_ant_id, OAL_FALSE);
						
				//s_s32_angle_pilot[u32_ant_id][0] = dl_cfo_judge(s_s32_angle_pilot[u32_ant_id][0], s_s32_angle_cp[u32_ant_id]);
				if(u32_pilot_num[u32_ant_id] > 2)
				{
					//calc cfo by pilot 2 & 3
					s_s32_angle_pilot[u32_ant_id][1] = dl_freq_offset_est(&s_st_temp_buffer[0][SYMBOL_LEN], &s_st_temp_buffer[0][SYMBOL_LEN * 2], SYMBOL_LEN, \
							PILOT_DIS_SYM * SLOT_LEN, s_s32_angle_cp_avg, u32_ant_id, OAL_FALSE);
					//s_s32_angle_pilot[u32_ant_id][1] = dl_cfo_judge(s_s32_angle_pilot[u32_ant_id][1], s_s32_angle_cp[u32_ant_id]);
					s_s32_angle_cca[u32_ant_id] = ((s_s32_angle_pilot[u32_ant_id][0] + s_s32_angle_pilot[u32_ant_id][1]) >> 1);

				}
				else
				{
					s_s32_angle_cca[u32_ant_id] = s_s32_angle_pilot[u32_ant_id][0];
				}

				s_u32_freq_ind = 1;
				s_s32_angle_avg += s_s32_angle_cca[u32_ant_id];

			}
			else
			{
				#if 0
				#if FREQ_OFFSET_COARSE
				dl_freq_offset_comp(stp_temp, &s_st_temp_buffer[0][0], s_st_cfo_seq, CP_LEN, OAL_TRUE);
				dl_freq_offset_comp(stp_temp + SYMBOL_LEN, &s_st_temp_buffer[0][SYMBOL_LEN], s_st_cfo_seq + (SYMBOL_LEN >> 4),\
							CP_LEN, OAL_TRUE);
				#else
				dl_freq_offset_comp(stp_temp, &s_st_temp_buffer[0][0], s_st_cfo_seq, CP_LEN, OAL_FALSE);
				dl_freq_offset_comp(stp_temp + SYMBOL_LEN, &s_st_temp_buffer[0][SYMBOL_LEN], s_st_cfo_seq + (SYMBOL_LEN), \
							CP_LEN, OAL_FALSE);
				#endif
				#if 0//VECTOR_CMP
				//if(g_test_flg)
				{
					while(READ_REG32(0x1724FFE0) != 2);
					WRITE_REG32(0x1724FFE4, 2); 
				}
				#endif

			    /*the second freq offset est*/
			    
			    s_s32_angle_cca[u32_ant_id] = dl_freq_offset_est(&s_st_temp_buffer[0][0], &s_st_temp_buffer[0][SYMBOL_LEN], \
			    			CP_LEN, SYMBOL_LEN, s32_cca_angle_avg_bak, u32_ant_id, OAL_FALSE);
				#endif
				s_u32_freq_ind = 0;
				s_s32_angle_avg += s_s32_angle_cp_avg;				
			}
		}
	}

	if(g_u32_rx_ant_num > 1)
	{	
		//temp here , because g_u32_rx_ant_num less than 2 
		s_s32_angle_avg >>= (g_u32_rx_ant_num - 1);
	}	
	KS_LOG(0, 0xA317, (UINT16)((-s_s32_angle_cca_avg * 937) >> 8));
	KS_LOG(0, 0xA317, (UINT16)((-s_s32_angle_cp_avg * 937) >> 8));
	KS_LOG(0, 0xA317, (UINT16)((-s_s32_angle_avg * 937) >> 8));

	if(stp_node_info)//stp_node_info NEQ NULL_PTR, which means node exists already
	{
		if((s_s32_angle_avg - s32_angle_node_tbl > AFC_DELTA_THRES))
		{
			s_s32_angle_avg = s32_angle_node_tbl + (AFC_DELTA_THRES >> 1 );
		}
		else if((s_s32_angle_avg - s32_angle_node_tbl < - AFC_DELTA_THRES ))
		{
			s_s32_angle_avg = s32_angle_node_tbl - (AFC_DELTA_THRES >> 1 );
		}
		else
		{
			s_s32_angle_avg = ((s_s32_angle_avg * (65536 - AFC_ALPHA) + s32_angle_node_tbl * AFC_ALPHA) >> 16);
		}
		KS_LOG(0, 0xA317, (UINT16)((-s_s32_angle_avg * 937) >> 8));
	}
	#if FREQ_OFFSET_COARSE
	dl_freq_comp_seq_gen(&s_st_cfo_seq, s_s32_angle_avg, OAL_TRUE, OAL_TRUE, OAL_TRUE);
	#else
	dl_freq_comp_seq_gen(&s_st_cfo_seq, s_s32_angle_avg, OAL_FALSE, OAL_TRUE, OAL_TRUE);
	#endif

	dl_data_dec_pilot(stp_input, 0);


}

UINT16 g_u16_eq_abs_mean[2] = {0};
OAL_ITCM_CODE_SECTION
VOID dl_head_decode(complex_s16_t* stp_input , UINT8 u8_pilot_need_proc)
{
	SINT32 s32_snr_mean;
    ks_fft_para_t st_fft_para;
	complex_s16_t* stp_temp = (complex_s16_t*)stp_input;
	SINT8 *s8p_llr_ptr;
	UINT32 u32_ant_id = 0;
	SINT32 s32_angle = 0;
	UINT32 u32_delta_of_ants = DELTA_OF_ANT_BUF;    


	KS_LOG_TMRL(0xB116);

    /*the first freq offset correction of pilot*/
    if(u8_pilot_need_proc EQ 1)
    {
    	#ifdef MIMO2
		dl_head_pilot_proc(stp_temp + SLOT_LEN);
		//stp_temp += SLOT_LEN;
    	#else
		dl_head_pilot_proc(stp_temp);
		stp_temp += SLOT_LEN;
		#endif
	}
	else if(u8_pilot_need_proc EQ 2)
	{
    	#ifdef MIMO2
		dl_head_pilot_proc(stp_temp);
		stp_temp = &s_st_data_iq[0][0]; // plcp iq store here
		u32_delta_of_ants = SC_NUM * 5; //temp here, zz
		#endif		
	}

	for(u32_ant_id = 0; u32_ant_id < RX_ANT_NUM; u32_ant_id++)
	{
		if(g_u32_cca_detected & (1 << u32_ant_id))
		{
			/*correct the freq offset of head*/
			#if FUN_DC_REMOVE
			//dc remove
	        _ks_cxv_add_cx(stp_temp + u32_delta_of_ants * u32_ant_id + CP_LEN + s_s32_sync_pos_off[u32_ant_id], SYMBOL_LEN, &s_st_dc[u32_ant_id],	&s_st_temp_buffer[0][0]);
			#if FREQ_OFFSET_COARSE
			dl_freq_offset_comp(&s_st_temp_buffer[0][0], &s_st_temp_buffer[0][0], &(s_st_cfo_seq.st_cfo_seq[CP_LEN >> 4]), SYMBOL_LEN, OAL_TRUE);
			#else
		    /*the second freq offset correction of pilot*/
			dl_freq_offset_comp(&s_st_temp_buffer[0][0], &s_st_temp_buffer[0][0], &(s_st_cfo_seq.st_cfo_seq[CP_LEN]), SYMBOL_LEN, OAL_FALSE);
			#endif
			#else
			#if FREQ_OFFSET_COARSE
			dl_freq_offset_comp(stp_temp + u32_delta_of_ants * u32_ant_id + CP_LEN + s_s32_sync_pos_off[u32_ant_id], &s_st_temp_buffer[0][0], \
						&(s_st_cfo_seq.st_cfo_seq[CP_LEN >> 4]), SYMBOL_LEN, OAL_TRUE);
			#else
			dl_freq_offset_comp(stp_temp + u32_delta_of_ants * u32_ant_id + CP_LEN + s_s32_sync_pos_off[u32_ant_id], &s_st_temp_buffer[0][0], \
						&(s_st_cfo_seq.st_cfo_seq[CP_LEN]), SYMBOL_LEN, OAL_FALSE);
			#endif
			#endif
		    #if 1//def MIMO2
			_ks_cxv_mul_cx(s_st_temp_buffer[0], SYMBOL_LEN, &s_st_cfo_seq.st_plcp_phase, 0, s_st_temp_buffer[1]);
			#else
			_ks_cxv_mul_cx(s_st_temp_buffer[0], SYMBOL_LEN, &s_st_cfo_seq.st_slot_phase[0], 0, s_st_temp_buffer[1]);
			#endif
			#if RSSI_BIT_SHIFT_SAME
			_ks_shift_s16v((SINT16_PTR)&s_st_temp_buffer[1][0], (SYMBOL_LEN<<1), s_s16_shift_value_min - RX_SHIFT_VAL, (SINT16_PTR)s_st_fft_in);//OFDM PAPR 较高，shift_value进行减1操作，即再/2的操作，后面的1bit对应fft
			#else
			if(s_s16_shift_value_max - s_s16_shift_value_min < 2)
			{
				_ks_shift_s16v((SINT16_PTR)&s_st_temp_buffer[1][0], (SYMBOL_LEN<<1), s_s16_shift_value[u32_ant_id] - RX_SHIFT_VAL, (SINT16_PTR)s_st_fft_in);//OFDM PAPR 较高，shift_value进行减1操作，即再/2的操作，后面的1bit对应fft
			}
			else
			{
				_ks_shift_s16v((SINT16_PTR)&s_st_temp_buffer[1][0], (SYMBOL_LEN<<1), s_s16_shift_value_min - RX_SHIFT_VAL, (SINT16_PTR)s_st_fft_in);//OFDM PAPR 较高，shift_value进行减1操作，即再/2的操作，后面的1bit对应fft
			}
			#endif
			
			/*transform into freq domain*/
			st_fft_para.s16cp_indata_addr	= s_st_fft_in;
			st_fft_para.s16cp_outdata_addr	= s_st_fft_out;
			st_fft_para.s16cp_twiddle_addr	= (complex_s16_t*)ceva_fft_lib_fft_2048_twi_table;
			st_fft_para.s32p_shift_amt_addr = s_s32_shift_amt_fft;
			st_fft_para.s16cp_temp_addr 	= (complex_s16_t*)(s_st_temp_buffer[0]);
			
			ks_fft2048(&st_fft_para);
			
			/*collect the 1200 subcarrier*/
			#ifdef SCFDMA_SWITCH
			#if 0
			ks_oal_mem_copy(&s_st_data_iq[u32_ant_id][HALF_SC_NUM], &s_st_fft_out[SYMBOL_LEN - HALF_SC_NUM], (HALF_SC_NUM<<2));
			ks_oal_mem_copy(&s_st_data_iq[u32_ant_id][0], &s_st_fft_out[1], (HALF_SC_NUM<<2));
			#else
			ks_oal_mem_copy(&s_st_data_iq[u32_ant_id][0], &s_st_fft_out[SYMBOL_LEN - HALF_SC_NUM], (HALF_SC_NUM<<2));
			ks_oal_mem_copy(&s_st_data_iq[u32_ant_id][HALF_SC_NUM], &s_st_fft_out[1], (HALF_SC_NUM<<2));
			#endif
			#else
			ks_oal_mem_copy(&s_st_data_iq[u32_ant_id][0], &s_st_fft_out[SYMBOL_LEN - HALF_SC_NUM], (HALF_SC_NUM<<2));
			ks_oal_mem_copy(&s_st_data_iq[u32_ant_id][HALF_SC_NUM], &s_st_fft_out[1], (HALF_SC_NUM<<2));
			#endif
		}
	}



    /*equalize*/
    if(g_u32_rx_ant_num EQ 1)
    {
    	if(TX_ANT_NUM EQ 1)
    	{
			dl_h_mod_calc((complex_s16_t*)s_st_H_vec[0][g_u32_rx_ant_1st_id * MAX_RX_ANT_NUM], s_st_H_mod, RTX_1T1R, (s_s32_noise_pwr[0][g_u32_rx_ant_1st_id * MAX_RX_ANT_NUM] >> 0));
    		dl_equalize_1t1r(s_st_H_vec[0][g_u32_rx_ant_1st_id * MAX_RX_ANT_NUM], &s_st_data_iq[g_u32_rx_ant_1st_id][0], \
    					&s_st_eq_out[0], SC_NUM); //length 为16的倍数
    	}
    	else
    	{
    		if(g_u32_tx_ant_num EQ 2)
    		{
				dl_h_mod_calc((complex_s16_t*)s_st_H_vec[0][g_u32_rx_ant_1st_id * MAX_RX_ANT_NUM], s_st_H_mod, RTX_2T1R, \
							(((s_s32_noise_pwr[0][g_u32_rx_ant_1st_id * MAX_RX_ANT_NUM]) + (s_s32_noise_pwr[0][g_u32_rx_ant_1st_id * MAX_RX_ANT_NUM + 1])) >> 1));
	    		dl_equalize_2t1r(s_st_H_vec[0][g_u32_rx_ant_1st_id * MAX_RX_ANT_NUM], s_st_H_vec[0][g_u32_rx_ant_1st_id * MAX_RX_ANT_NUM + 1], \
	    					&s_st_data_iq[g_u32_rx_ant_1st_id][0], &s_st_eq_out[0], SC_NUM); //length 为16的倍数    		
    		}
    		else
    		{
    			//one tx ant with no signal
				dl_h_mod_calc((complex_s16_t*)s_st_H_vec[0][g_u32_rx_ant_1st_id * MAX_RX_ANT_NUM + g_u32_tx_ant_1st_id], s_st_H_mod, RTX_1T1R, \
							s_s32_noise_pwr[0][g_u32_rx_ant_1st_id * MAX_RX_ANT_NUM + g_u32_tx_ant_1st_id]);
	    		dl_equalize_1t1r(s_st_H_vec[0][g_u32_rx_ant_1st_id * MAX_RX_ANT_NUM + g_u32_tx_ant_1st_id], &s_st_data_iq[g_u32_rx_ant_1st_id][0], \
	    					&s_st_eq_out[0], SC_NUM); //length 为16的倍数    		    			
    		}
    	}
	}
	else
	{
    	if(TX_ANT_NUM EQ 1)
    	{
			dl_h_mod_calc((complex_s16_t*)s_st_H_vec[0][0], s_st_H_mod, RTX_1T2R, ((s_s32_noise_pwr[0][0] + s_s32_noise_pwr[0][2]) >> 1));
     		dl_equalize_1t2r(s_st_H_vec[0][0], s_st_H_vec[0][2], &s_st_data_iq[0][0], &s_st_data_iq[1][0], &s_st_eq_out[0], SC_NUM); //length 为16的倍数   		
    	}
    	else
    	{
    		if(g_u32_tx_ant_num EQ 2)
    		{
				dl_h_mod_calc((complex_s16_t*)s_st_H_vec[0][0], s_st_H_mod, RTX_2T2R, \
							((s_s32_noise_pwr[0][0] + s_s32_noise_pwr[0][1] + s_s32_noise_pwr[0][2] + s_s32_noise_pwr[0][3]) >> 2));
	    		dl_equalize_2t2r(s_st_H_vec[0][0], &s_st_data_iq[0][0], &s_st_data_iq[1][0], &s_st_eq_out[0], SC_NUM); //length 为16的倍数
    		}
    		else
			{	
				dl_h_mod_calc((complex_s16_t*)s_st_H_vec[0][g_u32_tx_ant_1st_id], s_st_H_mod, RTX_1T2R, \
							((s_s32_noise_pwr[0][g_u32_tx_ant_1st_id] + s_s32_noise_pwr[0][2 + g_u32_tx_ant_1st_id]) >> 1));
	     		dl_equalize_1t2r(s_st_H_vec[0][g_u32_tx_ant_1st_id], s_st_H_vec[0][g_u32_tx_ant_1st_id + 2], &s_st_data_iq[0][0], &s_st_data_iq[1][0], &s_st_eq_out[0], SC_NUM); //length 为16的倍数   		
    		}    		
    	}
	}
	//s8p_llr_ptr = dl_turbo_input_addr_get(ZZW_PLCP_BIT_LEN*3 + s_u32_llr_shift_plcp);
	s8p_llr_ptr = dl_turbo_input_addr_get(SC_NUM);
    ks_oal_mem_set(s8p_llr_ptr, 0x0, s_u32_llr_shift_plcp);
	#if DEMOD_WITH_SNR
	s32_snr_mean = ks_cxv_rssi_calc(s_st_H_vec[0], SC_NUM);
	ks_cxv_squa_s16(s_st_H_vec[0], SC_NUM, &g_u32_snr[0]);
	dl_snr_shift(&g_u32_snr[0], &g_u16_snr[0], SC_NUM, s32_snr_mean);
    dl_demod_with_snr(&s_st_eq_out[0], s8p_llr_ptr, &g_u16_snr[0], s_u32_llr_shift_plcp, SC_NUM, BPSK);
	#else
	#ifdef PLCP_SOFT_COMBINE
	_ks_cxv_add_cxv(s_st_eq_out, &s_st_eq_out[PLCP_ENCODE_LEN], PLCP_ENCODE_LEN, s_st_eq_out);
	_ks_shift_s16v(s_st_eq_out, PLCP_ENCODE_LEN << 1, -1, s_st_eq_out);
	#endif
    dl_demod(&s_st_eq_out[0], s8p_llr_ptr, s_u32_llr_shift_plcp, s_u32_turbo_in_len_plcp, BPSK);
    //dl_demod(&s_st_eq_out[0], s8p_llr_ptr, s_u32_llr_shift_plcp, ZZW_PLCP_BIT_LEN * 3, BPSK);
	#endif
	#ifdef MIMO2
	g_u32_ics_sym_bmp |= (1 << 0);
	#else
	g_u32_ics_sym_bmp |= (1 << 1);
	#endif

	#if 0
	ks_abs_mean_calc(&s_st_eq_out[0], SC_NUM, g_u16_eq_abs_mean);
	KS_LOG(0, 0xA31A, g_u16_eq_abs_mean[0]);
	KS_LOG(0, 0xA31A, g_u16_eq_abs_mean[1]);			
	#endif
    /*start turbo decode*/
    g_u32_ics_head_crc = 2;
    g_u32_ics_cca_cnt++;
    dl_turbo_dec_start(PLCP_TYPE, (UINT32)s8p_llr_ptr, s_u32_turbo_in_len_plcp, ZZW_PLCP_BIT_LEN, CRC_24A);
}
OAL_ITCM_CODE_SECTION
VOID dl_head_dec_post(SINT32_PTR s32p_in , UINT32 b_head_case_type, UINT32 u32_pre_ret)
{
	/*b_plcp_case_type  0 : ICS_HEAD_DECODE; 1 : ICS_HEAD_COLLECT; 2 : ICS_HEAD_COLLECT2; 3 : ICS_HEAD_COLLECT3*/
	UINT32 u32_cca_detected = OAL_FALSE;
	UINT32 u32_left_sym = 0;
	UINT32 u32_left_sym_previous = 0;
	UINT32 u32_need_drop_frame = 0;
	UINT32 u32_i = 0;

	//while(2 NEQ g_u32_ics_head_crc);

	//if(OAL_SUCCESS EQ g_u32_ics_head_crc)
	if(1)
	{

		u32_left_sym_previous = g_u32_ics_left_sym; //left symbol(s) in last frame
		g_u32_ics_left_sym = g_u32_ics_left_cnt / SLOT_LEN;  //left symbol(s) in curr frame

		if(((u32_left_sym_previous EQ 4) && (g_u32_ics_left_sym EQ 5)) || ((u32_left_sym_previous EQ 10) && (g_u32_ics_left_sym EQ 11)))
		{
			u32_need_drop_frame = OAL_TRUE;
		}

		if(g_u32_ics_left_sym)
		{
			u32_left_sym = (g_u32_ics_left_sym >= (LAST_SYM_ID + 1)) ? (LAST_SYM_ID + 1) : g_u32_ics_left_sym;
			dl_data_dec_pre(&s32p_in[RX_BUF_LEN - g_u32_ics_left_cnt], 0, u32_left_sym - 1);
			g_u32_ics_left_cnt -= u32_left_sym * SLOT_LEN;
		}

		//rx buf not finish , length left is g_u32_ics_left_cnt, not swith pingpang ind here
		//wait plcp decode done
		if(g_u32_ics_left_sym < (PILOT1_SYM_ID + 1))
		{
			//not rx 2nd pilot, will not start data turbo decoder, wait plcp decode finish
			while(TURBO_DECODER_IDLE NEQ turbo_decoder_get_status());
		}


		if(OAL_SUCCESS EQ g_u32_ics_head_crc)
		{
			if(g_u32_ics_left_sym < (LAST_SYM_ID + 1))
			{
				//not rx full data frame, which contains 11 symbols(9 data symbols and 2 pilots), need collect symbols left
				g_u32_ics_flag = ICS_DATA_COLLECT;
				if((g_u32_ics_left_cnt > 0))
				{
					for(u32_i = 0; u32_i < RX_ANT_NUM; u32_i++)
					{
						//ks_oal_mem_copy(((UINT32_PTR)(s_st_ics_left + DELTA_OF_ANT_BUF * u32_i)), &(s32p_in[RX_BUF_LEN - g_u32_ics_left_cnt]), (g_u32_ics_left_cnt << 2));
						ks_oal_mem_copy(((UINT32_PTR)(s_st_rx_buffer.st_ics_left + DELTA_OF_ANT_BUF * u32_i)), \
									(s32p_in + RX_BUF_LEN - g_u32_ics_left_cnt  + DELTA_OF_ANT_BUF * u32_i), (g_u32_ics_left_cnt << 2));
					}				
				}
			}
			else
			{
				g_u32_ics_flag = ICS_CCA_DETECT;
				//ks_oal_mem_copy(((UINT32_PTR)s_st_ics_left), &(s32p_in[RX_BUF_LEN - (CCA_LEN << 1)]), ((CCA_LEN << 1) << 2));

				#if 0
				//resync
				if((!u32_need_drop_frame) && (g_u32_ics_left_cnt))
				{
					//only continue ics when plcp is valid
					//if(g_u32_ics_left_cnt > (CCA_LEN << 1)
					//g_u32_ics_fft_cnt = (RX_BUF_LEN - g_u32_ics_left_cnt - (CCA_LEN << 1)) / ICS_CORR_LEN;
					if(g_u32_ics_left_cnt < (948 + 572 + 192))
					{
						//ks_oal_mem_copy(((UINT32_PTR)s_st_ics_left + ((CCA_LEN << 1) - g_u32_ics_left_cnt)), &(s32p_in[RX_BUF_LEN - g_u32_ics_left_cnt]), (g_u32_ics_left_cnt << 2));
					}
					else
					{
						if(b_plcp_case_type)
						{
							g_u32_ics_fft_cnt -= 1;
						}
						u32_cca_detected = dl_cca_proc(s32p_in, 0);
						if(u32_cca_detected)
						{
							dl_head_dec_pre(s32p_in, 0);
						}
					}
				}
				#endif
							
				#if 1 //ICS_OPTIMIZE
				if(1)//(s_u32_ics_optimize)
				{
			    	if((s_u8_zzw_state EQ ZZW_STATE_SCN) && (u32_pre_ret) && (b_head_case_type))
					{

						g_u32_ics_fft_cnt -= 1;
						g_u32_cca_detected = dl_cca_proc(s32p_in, 0, (ICS_SEG_NUM + 1), &g_u32_ics_fft_cnt);
						
						if(g_u32_cca_detected)
						{
							dl_head_dec_pre(s32p_in);
						}
					}
				}
				#endif
			}
		}
		else
		{
			//may need continue do cca detect in curr frame
			g_u32_ics_flag = ICS_CCA_DETECT;
			#if 1//ICS_OPTIMIZE
			if(1)//(s_u32_ics_optimize)
			{
		    	if((s_u8_zzw_state EQ ZZW_STATE_SCN) && ((b_head_case_type EQ 0) || ((b_head_case_type) && (u32_pre_ret))) \
		    		&& (g_u32_ics_cca_cnt < 3))
				{
					if(b_head_case_type)
					{
						g_u32_ics_fft_cnt = 1;
					}
					g_u32_cca_detected = dl_cca_proc(s32p_in, 0, (ICS_SEG_NUM + 1), &g_u32_ics_fft_cnt);
					
					if(g_u32_cca_detected)
					{
						dl_head_dec_pre(s32p_in);
					}
				}
			}
			#endif
		}
	}
	else
	{
		g_u32_ics_flag = ICS_CCA_DETECT;
	}


}

OAL_ITCM_CODE_SECTION
VOID  dl_head_dec_pre(SINT32_PTR s32p_rx_buf)
{
	SINT32 s32_sync_pos = 30720;
	UINT32 u32_i = 0;
	UINT32 u32_delta_of_ants = DELTA_OF_ANT_BUF; 
	SINT16 s16_shift_value_min = 20;
	SINT16 s16_shift_value_max = 0;	
	
	KS_LOG_TMRL(0xB116);

	for(u32_i = 0; u32_i < RX_ANT_NUM; u32_i++)
	{
		if(g_u32_cca_detected & (1 << u32_i))
		{
			//temp using first ant(cca detected)'s sync pos
			if(s32_sync_pos > s_s32_sync_position[u32_i])
			{
				s32_sync_pos = s_s32_sync_position[u32_i];
				s_s32_sync_pos_min = s32_sync_pos;
				g_u32_rx_ant_1st_id = u32_i;
			}
			if(s_s16_shift_value[u32_i] < s16_shift_value_min)
			{
				s16_shift_value_min = s_s16_shift_value[u32_i];
			}
			if(s_s16_shift_value[u32_i] > s16_shift_value_max)
			{
				s16_shift_value_max = s_s16_shift_value[u32_i];
			}
			//s_stp_iq_ptr[u32_i] = &(s32p_rx_buf[s_s32_sync_position[u32_i] + DIS_FROM_CCA2PILOT + u32_delta_of_ants * u32_i]);
		}
	}
	s_s16_shift_value_min = s16_shift_value_min;
	s_s16_shift_value_max = s16_shift_value_max;


	for(u32_i = 0; u32_i < RX_ANT_NUM; u32_i++)
	{
		s_s32_sync_pos_off[u32_i] = 0;
		if(g_u32_cca_detected & (1 << u32_i))
		{
			s_s32_sync_pos_off[u32_i] = s_s32_sync_position[u32_i] - s_s32_sync_pos_min;
			OAL_ASSERT(s_s32_sync_pos_off[u32_i] >= 0, "");
			#if 1
			if(s_s16_shift_value[u32_i] > (s_s16_shift_value_min + 5))
			{
				//SIG IN THIS ANT TOO LOW, DISCARD IT, temp here
				g_u32_cca_detected &=  ~(1 << u32_i);
				g_u32_rx_ant_num -= 1;
				if(g_u32_rx_ant_1st_id EQ u32_i)
				{
					g_u32_rx_ant_1st_id = ((g_u32_rx_ant_1st_id + 1) & 0x1);//TEMP HERE, because RX_ANT_NUM = 2
					s_s32_sync_pos_min = s_s32_sync_position[g_u32_rx_ant_1st_id];
					s_s32_sync_pos_off[g_u32_rx_ant_1st_id] = 0; 
				}
				OAL_ASSERT(g_u32_rx_ant_num >= 1, "");
			}
			#endif
		}
	}
	
	KS_LOG(0, 0xA118, (UINT16)s32_sync_pos);
	
	if((RX_BUF_LEN - s32_sync_pos) >= (DIS_FROM_CCA2PILOT + (SLOT_LEN << 1) - 0))
	{
		//1st pilot & plcp in curr frame fully
		g_u32_ics_flag = ICS_HEAD_DECODE;
		dl_head_decode(&(s32p_rx_buf[s32_sync_pos + DIS_FROM_CCA2PILOT]), 1);
		g_u32_ics_left_cnt = RX_BUF_LEN - (s32_sync_pos + (SLOT_LEN << 1) + DIS_FROM_CCA2PILOT); 
		KS_LOG(0, 0xA119, (UINT16)g_u32_ics_left_cnt);
		dl_head_dec_post(s32p_rx_buf, (UINT32)0, 1);
	}
	else 
	{
		#ifdef MIMO2
		if((RX_BUF_LEN - s32_sync_pos) >= (DIS_FROM_CCA2PILOT + (SLOT_LEN) - 0))
		{
			//plcp in curr frame fully, part of 1st pilot in curr frame
			g_u32_ics_flag = ICS_HEAD_COLLECT;
			g_u32_ics_left_cnt = RX_BUF_LEN - (s32_sync_pos + (SLOT_LEN) + DIS_FROM_CCA2PILOT);
			if(g_u32_ics_left_cnt)
			{
				for(u32_i = 0; u32_i < RX_ANT_NUM; u32_i++)
				{
					//store plcp
					ks_oal_mem_copy(((UINT32_PTR)&s_st_data_iq[u32_i][0]), \
								(s32p_rx_buf + (s32_sync_pos) + DIS_FROM_CCA2PILOT + DELTA_OF_ANT_BUF * u32_i),\
								(SLOT_LEN << 2));
					//store part of 1st pilot
					ks_oal_mem_copy(((UINT32_PTR)(s_st_rx_buffer.st_ics_left + DELTA_OF_ANT_BUF * u32_i)), \
								(s32p_rx_buf + (s32_sync_pos) + (SLOT_LEN) + DIS_FROM_CCA2PILOT + DELTA_OF_ANT_BUF * u32_i),\
								(g_u32_ics_left_cnt << 2));
				}				
			}
		}
		else if((RX_BUF_LEN - s32_sync_pos) >= (DIS_FROM_CCA2PILOT - 0))
		{
			//part of plcp in curr frame
			g_u32_ics_flag = ICS_HEAD_COLLECT2;
			g_u32_ics_left_cnt = RX_BUF_LEN - (s32_sync_pos + DIS_FROM_CCA2PILOT);

			if(g_u32_ics_left_cnt)
			{
				for(u32_i = 0; u32_i < RX_ANT_NUM; u32_i++)
				{			
					ks_oal_mem_copy(((UINT32_PTR)(s_st_rx_buffer.st_ics_left + DELTA_OF_ANT_BUF * u32_i)), \
								(s32p_rx_buf + (s32_sync_pos) + DIS_FROM_CCA2PILOT + DELTA_OF_ANT_BUF * u32_i),\
								(g_u32_ics_left_cnt << 2));
				}
			}
		}
		else
		{
			//1st pilot not in curr frame
			//OAL_ASSERT(0, "");
			g_u32_ics_flag = ICS_HEAD_COLLECT3;
			g_u32_ics_left_cnt = (s32_sync_pos + DIS_FROM_CCA2PILOT) - RX_BUF_LEN;
		}
		#else
		if((RX_BUF_LEN - s32_sync_pos) >= (DIS_FROM_CCA2PILOT + (SLOT_LEN) - 0))
		{
			//1st pilot in curr frame fully
			g_u32_ics_flag = ICS_HEAD_COLLECT;
			dl_head_pilot_proc(&(s32p_rx_buf[s32_sync_pos + DIS_FROM_CCA2PILOT]));
			g_u32_ics_left_cnt = RX_BUF_LEN - (s32_sync_pos + (SLOT_LEN) + DIS_FROM_CCA2PILOT);
			if(g_u32_ics_left_cnt)
			{
				for(u32_i = 0; u32_i < RX_ANT_NUM; u32_i++)
				{
					ks_oal_mem_copy(((UINT32_PTR)(s_st_rx_buffer.st_ics_left + DELTA_OF_ANT_BUF * u32_i)), \
								(s32p_rx_buf + (s32_sync_pos) + (SLOT_LEN) + DIS_FROM_CCA2PILOT + DELTA_OF_ANT_BUF * u32_i),\
								(g_u32_ics_left_cnt << 2));
				}				
			}
		}
		else if((RX_BUF_LEN - s32_sync_pos) >= (DIS_FROM_CCA2PILOT - 0))
		{
			//part of 1st pilot in curr frame
			g_u32_ics_flag = ICS_HEAD_COLLECT2;
			g_u32_ics_left_cnt = RX_BUF_LEN - (s32_sync_pos + DIS_FROM_CCA2PILOT);

			if(g_u32_ics_left_cnt)
			{
				for(u32_i = 0; u32_i < RX_ANT_NUM; u32_i++)
				{			
					ks_oal_mem_copy(((UINT32_PTR)(s_st_rx_buffer.st_ics_left + DELTA_OF_ANT_BUF * u32_i)), \
								(s32p_rx_buf + (s32_sync_pos) + DIS_FROM_CCA2PILOT + DELTA_OF_ANT_BUF * u32_i),\
								(g_u32_ics_left_cnt << 2));
				}
			}
		}
		else
		{
			//1st pilot not in curr frame
			//OAL_ASSERT(0, "");
			g_u32_ics_flag = ICS_HEAD_COLLECT3;
			g_u32_ics_left_cnt = (s32_sync_pos + DIS_FROM_CCA2PILOT) - RX_BUF_LEN;
		}
		#endif
		//s_s32_sync_position += g_u32_ics_frm_cnt * RX_BUF_LEN;
		KS_LOG(0, 0xA119, (UINT16)g_u32_ics_left_cnt);
	}
}

OAL_ITCM_CODE_SECTION
VOID dl_data_dec_sf(UINT32 u32_sym_start, UINT32 u32_sym_num)
{
    UINT32 u32_iloop;
    UINT32 u32_mcs = (UINT32)s_st_algo2l1cc_ctrl_info.st_ctrl_info.u8_mcs;    
    UINT32 u32_sym_offset = 0; 
    UINT32 u32_H_intp_offset = 0;
    SINT32 s32_noise_pwr_mean;

	SINT8 *s8p_llr_ptr;

    UINT32 u32_1st_sf = 0;
    UINT32 u32_2nd_sf = 0;

    UINT32 u32_crc_type = CRC_24B;
    SINT32 s32_snr_mean;
    UINT32 u32_mod_len = s_u32_mod_len[u32_mcs];
    UINT32 u32_llr_shift = s_u32_llr_shift[u32_mcs];
    UINT32 u32_turbo_in_len = s_u32_turbo_in_len[u32_mcs];
	UINT16 u16_eq_abs_mean[2];
    /*
    s_st_H_vec[0] : pilot 0
	s_st_H_vec[1] : pilot 1
	s_st_H_vec[2] : pilot 2
	//s_st_H_vec[3] ~ s_st_H_vec[6] : data0 ~ data3
	//s_st_H_vec[3] ~ s_st_H_vec[7] : data4 ~ data8
    */
    OAL_ASSERT(g_u32_rx_ant_1st_id < 2, "");
	//KS_LOG(0, 0xA11F, (u32_sym_start << 8) + u32_sym_num);
	if(s_st_plcp_info.u16_subframe_num EQ 1)
	{
		u32_crc_type = CRC_24A;
	}
	
	if(OAL_CRC_FAILURE EQ g_u32_ics_head_crc)
	{
		KS_LOG(0, 0xA11F, 0xDEAD);
		return;
	}

    
	if(u32_sym_start < PILOT1_SYM_ID)
	{
		u32_1st_sf = 1;
		u32_sym_start += 1;	
		#ifdef MIMO2
		u32_H_intp_offset = -1;
		#endif
	}
	else
	{
		u32_2nd_sf = 1;	
	    u32_sym_offset = -(PILOT1_SYM_ID + 1);		
	}

	for(u32_iloop = u32_sym_start; u32_iloop < u32_sym_start + u32_sym_num; u32_iloop++)
	{
		//KS_LOG(0, 0xA11F, (u32_iloop + u32_sym_start + u32_sym_offset));
		if(g_u32_rx_ant_num EQ 1)
		{
			if(TX_ANT_NUM EQ 1)
			{	
				//1T1R
				s32_noise_pwr_mean = ((s_s32_noise_pwr[0 + u32_2nd_sf][g_u32_rx_ant_1st_id * MAX_RX_ANT_NUM + g_u32_tx_ant_1st_id] +\
							s_s32_noise_pwr[1 + u32_2nd_sf][g_u32_rx_ant_1st_id * MAX_RX_ANT_NUM + g_u32_tx_ant_1st_id])>>1);			
		
				#if H_INTERP_REDUCE
				if(u32_iloop & 0x1)
				#endif
				{
					dl_H_intp(s_st_H_vec[0 + u32_2nd_sf][g_u32_rx_ant_1st_id * MAX_RX_ANT_NUM + g_u32_tx_ant_1st_id],\
								s_st_H_vec[1 + u32_2nd_sf][g_u32_rx_ant_1st_id * MAX_RX_ANT_NUM + g_u32_tx_ant_1st_id], \
								(u32_iloop + u32_sym_offset + u32_H_intp_offset), s_st_H_vec[3][0]);
					dl_h_mod_calc((complex_s16_t*)s_st_H_vec[3][0], s_st_H_mod, RTX_1T1R, s32_noise_pwr_mean);
				}
				dl_equalize_1t1r(s_st_H_vec[3][0], &s_st_data_iq[g_u32_rx_ant_1st_id][(u32_iloop + u32_sym_offset) * SC_NUM], \
							&s_st_eq_out[(u32_iloop) * SC_NUM], SC_NUM); //length 为16的倍数
			}
			else
			{
	    		if(g_u32_tx_ant_num EQ 2)
    			{
					//2T1R
					s32_noise_pwr_mean = ((s_s32_noise_pwr[0 + u32_2nd_sf][g_u32_rx_ant_1st_id * MAX_RX_ANT_NUM] +\
								s_s32_noise_pwr[0 + u32_2nd_sf][g_u32_rx_ant_1st_id * MAX_RX_ANT_NUM + 1] + \
								s_s32_noise_pwr[1 + u32_2nd_sf][g_u32_rx_ant_1st_id * MAX_RX_ANT_NUM] + \
								s_s32_noise_pwr[1 + u32_2nd_sf][g_u32_rx_ant_1st_id * MAX_RX_ANT_NUM + 1]) >> 2);

					#if H_INTERP_REDUCE
					if((u32_iloop) & 0x1)
					#endif
					{
						dl_H_intp(s_st_H_vec[0 + u32_2nd_sf][g_u32_rx_ant_1st_id * MAX_RX_ANT_NUM],\
									s_st_H_vec[1 + u32_2nd_sf][g_u32_rx_ant_1st_id * MAX_RX_ANT_NUM], \
									(u32_iloop + u32_sym_offset + u32_H_intp_offset), s_st_H_vec[3][0]);
						dl_H_intp(s_st_H_vec[0 + u32_2nd_sf][g_u32_rx_ant_1st_id * MAX_RX_ANT_NUM + 1], \
									s_st_H_vec[1 + u32_2nd_sf][g_u32_rx_ant_1st_id * MAX_RX_ANT_NUM + 1], \
									(u32_iloop + u32_sym_offset + u32_H_intp_offset), s_st_H_vec[3][1]);
						dl_h_mod_calc((complex_s16_t*)s_st_H_vec[3][0], s_st_H_mod, RTX_2T1R, s32_noise_pwr_mean);
					}
					dl_equalize_2t1r(s_st_H_vec[3][0], \
								s_st_H_vec[3][1], \
								&s_st_data_iq[g_u32_rx_ant_1st_id][(u32_iloop + u32_sym_offset) * SC_NUM],
								&s_st_eq_out[(u32_iloop) * SC_NUM], SC_NUM); //length 为16的倍数		
				}
				else
				{
				
					//1T1R
					s32_noise_pwr_mean = ((s_s32_noise_pwr[0 + u32_2nd_sf][g_u32_rx_ant_1st_id * MAX_RX_ANT_NUM + g_u32_tx_ant_1st_id] +\
								s_s32_noise_pwr[1 + u32_2nd_sf][g_u32_rx_ant_1st_id * MAX_RX_ANT_NUM + g_u32_tx_ant_1st_id])>>1);			
			
					#if H_INTERP_REDUCE
					if(u32_iloop & 0x1)
					#endif
					{
						dl_H_intp(s_st_H_vec[0 + u32_2nd_sf][g_u32_rx_ant_1st_id * MAX_RX_ANT_NUM + g_u32_tx_ant_1st_id], \
									s_st_H_vec[1 + u32_2nd_sf][g_u32_rx_ant_1st_id * MAX_RX_ANT_NUM + g_u32_tx_ant_1st_id], \
									(u32_iloop + u32_sym_offset + u32_H_intp_offset), s_st_H_vec[3][g_u32_rx_ant_1st_id * MAX_RX_ANT_NUM/* + g_u32_tx_ant_1st_id*/]);
						dl_h_mod_calc((complex_s16_t*)s_st_H_vec[3][g_u32_rx_ant_1st_id * MAX_RX_ANT_NUM/*  + g_u32_tx_ant_1st_id*/], s_st_H_mod, RTX_1T1R, s32_noise_pwr_mean);
					}
					dl_equalize_1t1r(s_st_H_vec[3][g_u32_rx_ant_1st_id * MAX_RX_ANT_NUM/* + g_u32_tx_ant_1st_id*/], \
								&s_st_data_iq[g_u32_rx_ant_1st_id][(u32_iloop + u32_sym_offset) * SC_NUM], \
								&s_st_eq_out[(u32_iloop) * SC_NUM], SC_NUM); //length 为16的倍数
				}
			}
		}
		else
		{
			//OAL_ASSERT(g_u32_rx_ant_1st_id EQ 0, "");
			
			if(TX_ANT_NUM EQ 1)
			{
				//1T2R
				s32_noise_pwr_mean = ((s_s32_noise_pwr[0 + u32_2nd_sf][0 + g_u32_tx_ant_1st_id] +\
							s_s32_noise_pwr[0 + u32_2nd_sf][MAX_RX_ANT_NUM + g_u32_tx_ant_1st_id] + \
							s_s32_noise_pwr[1 + u32_2nd_sf][0 + g_u32_tx_ant_1st_id] + \
							s_s32_noise_pwr[1 + u32_2nd_sf][MAX_RX_ANT_NUM + g_u32_tx_ant_1st_id]) >> 2);

				#if H_INTERP_REDUCE
				if((u32_iloop) & 0x1)
				#endif
				{
					//while(READ_REG32(0x1724ffe0) NEQ 0);
					dl_H_intp(s_st_H_vec[0 + u32_2nd_sf][g_u32_tx_ant_1st_id], \
								s_st_H_vec[1 + u32_2nd_sf][g_u32_tx_ant_1st_id], \
								(u32_iloop + u32_sym_offset + u32_H_intp_offset), s_st_H_vec[3][0]);
					//while(READ_REG32(0x1724ffe0) NEQ 1);

					dl_H_intp(s_st_H_vec[0 + u32_2nd_sf][MAX_RX_ANT_NUM + g_u32_tx_ant_1st_id], \
								s_st_H_vec[1 + u32_2nd_sf][MAX_RX_ANT_NUM + g_u32_tx_ant_1st_id], \
								(u32_iloop + u32_sym_offset + u32_H_intp_offset), s_st_H_vec[3][MAX_RX_ANT_NUM]);
					//while(READ_REG32(0x1724ffe0) NEQ 2);
					dl_h_mod_calc((complex_s16_t*)s_st_H_vec[3][0], s_st_H_mod, RTX_1T2R, s32_noise_pwr_mean);
				}
				dl_equalize_1t2r(s_st_H_vec[3][0], s_st_H_vec[3][MAX_RX_ANT_NUM], &s_st_data_iq[0][(u32_iloop + u32_sym_offset) * SC_NUM],\
							&s_st_data_iq[1][(u32_iloop + u32_sym_offset) * SC_NUM],&s_st_eq_out[(u32_iloop) * SC_NUM], SC_NUM); //length 为16的倍数			
				//while(READ_REG32(0x1724ffe0) NEQ 3);
		
			}
			else
			{
	    		if(g_u32_tx_ant_num EQ 2)
    			{
    				//2T2R
					//KS_LOG(0, 0xA21F, (u32_iloop + u32_sym_start));
					s32_noise_pwr_mean = ((s_s32_noise_pwr[0 + u32_2nd_sf][0] +\
								s_s32_noise_pwr[0 + u32_2nd_sf][1] + \
								s_s32_noise_pwr[0 + u32_2nd_sf][2] + \
								s_s32_noise_pwr[0 + u32_2nd_sf][3] + \
								s_s32_noise_pwr[1 + u32_2nd_sf][0] + \
								s_s32_noise_pwr[1 + u32_2nd_sf][1] + \
								s_s32_noise_pwr[1 + u32_2nd_sf][2] + \								
								s_s32_noise_pwr[1 + u32_2nd_sf][3]) >> 3);
					
					//DL_GET_TMR2_START;
					#if H_INTERP_REDUCE
					if((u32_iloop) & 0x1)
					#endif
					{
						//KS_LOG(0, 0xA31F, (u32_iloop + u32_sym_start));
						//while(READ_REG32(0x1724ffe0) NEQ 0);
						#if 0
						dl_H_intp(s_st_H_vec[0 + u32_2nd_sf][0], \
									s_st_H_vec[1 + u32_2nd_sf][0], \
									(u32_iloop + u32_sym_offset), s_st_H_vec[3][0]);
						//while(READ_REG32(0x1724ffe0) NEQ 1);
						dl_H_intp(s_st_H_vec[0 + u32_2nd_sf][1], \
									s_st_H_vec[1 + u32_2nd_sf][1], \
									(u32_iloop + u32_sym_offset), s_st_H_vec[3][1]);
						dl_H_intp(s_st_H_vec[0 + u32_2nd_sf][2], \
									s_st_H_vec[1 + u32_2nd_sf][2], \
									(u32_iloop + u32_sym_offset), s_st_H_vec[3][2]);
						dl_H_intp(s_st_H_vec[0 + u32_2nd_sf][3], \
									s_st_H_vec[1 + u32_2nd_sf][3], \
									(u32_iloop + u32_sym_offset), s_st_H_vec[3][3]);
						#else
						dl_H_intp2x2(s_st_H_vec[0 + u32_2nd_sf][0], s_st_H_vec[1 + u32_2nd_sf][0], \
									(u32_iloop + u32_sym_offset + u32_H_intp_offset), ((u32_iloop EQ 1) || (u32_iloop EQ (PILOT1_SYM_ID + 1))), s_st_H_vec[3][0]);
						#endif
						//while(READ_REG32(0x1724ffe0) NEQ 2);
						dl_h_mod_calc((complex_s16_t*)s_st_H_vec[3][0], s_st_H_mod, RTX_2T2R, s32_noise_pwr_mean);
					}
					
					//DL_GET_TMR2_END;
					//DL_PRINT_TMR2_DURATION(0xA51B);
		    		dl_equalize_2t2r(s_st_H_vec[3][0], &s_st_data_iq[0][(u32_iloop + u32_sym_offset) * SC_NUM],\
		    					&s_st_data_iq[1][(u32_iloop + u32_sym_offset) * SC_NUM], \
		    					&s_st_eq_out[(u32_iloop) * SC_NUM], SC_NUM); //length 为16的倍数
				}
				else
				{
					//1T2R
					s32_noise_pwr_mean = ((s_s32_noise_pwr[0 + u32_2nd_sf][0 + g_u32_tx_ant_1st_id] +\
								s_s32_noise_pwr[0 + u32_2nd_sf][MAX_RX_ANT_NUM + g_u32_tx_ant_1st_id] + \
								s_s32_noise_pwr[1 + u32_2nd_sf][0 + g_u32_tx_ant_1st_id] + \
								s_s32_noise_pwr[1 + u32_2nd_sf][MAX_RX_ANT_NUM + g_u32_tx_ant_1st_id]) >> 2);

					#if H_INTERP_REDUCE
					if((u32_iloop) & 0x1)
					#endif
					{
						//while(READ_REG32(0x1724ffe0) NEQ 0);
						dl_H_intp(s_st_H_vec[0 + u32_2nd_sf][g_u32_tx_ant_1st_id], \
									s_st_H_vec[1 + u32_2nd_sf][g_u32_tx_ant_1st_id], \
									(u32_iloop + u32_sym_offset + u32_H_intp_offset), s_st_H_vec[3][0]);
						//while(READ_REG32(0x1724ffe0) NEQ 1);

						dl_H_intp(s_st_H_vec[0 + u32_2nd_sf][MAX_RX_ANT_NUM + g_u32_tx_ant_1st_id], \
									s_st_H_vec[1 + u32_2nd_sf][MAX_RX_ANT_NUM + g_u32_tx_ant_1st_id], \
									(u32_iloop + u32_sym_offset + u32_H_intp_offset), s_st_H_vec[3][MAX_RX_ANT_NUM]);
						//while(READ_REG32(0x1724ffe0) NEQ 2);
						dl_h_mod_calc((complex_s16_t*)s_st_H_vec[3][0], s_st_H_mod, RTX_1T2R, s32_noise_pwr_mean);
					}
					dl_equalize_1t2r(s_st_H_vec[3][0], s_st_H_vec[3][MAX_RX_ANT_NUM], &s_st_data_iq[0][(u32_iloop + u32_sym_offset) * SC_NUM],\
								&s_st_data_iq[1][(u32_iloop + u32_sym_offset) * SC_NUM],&s_st_eq_out[(u32_iloop) * SC_NUM], SC_NUM); //length 为16的倍数			
					//while(READ_REG32(0x1724ffe0) NEQ 3);
				}
			}
		}


		s_u32_data_sym_eq_cnt += 1;
		KS_LOG(0, 0xA11A, (UINT16)((s_u32_data_cb_start_cnt <<8) | (u32_iloop)));

		if( (s_u32_data_sym_eq_cnt * SC_NUM)  >= ((s_u32_data_cb_start_cnt + 1) * u32_mod_len))
		{
			//eq data enough to do demod
			s8p_llr_ptr = dl_turbo_input_addr_get(u32_turbo_in_len);
			//eq data enough to do turbo decode
			ks_oal_mem_set(s8p_llr_ptr, 0x0, u32_llr_shift);
			#if DEMOD_WITH_SNR
			dl_demod_with_snr(&s_st_eq_out[u32_mod_len * s_u32_data_cb_start_cnt + SC_NUM], s8p_llr_ptr, \
						&g_u16_snr[u32_mod_len * s_u32_data_cb_start_cnt + SC_NUM], u32_llr_shift, u32_mod_len, s_st_plcp_info.u16_mcs); 
			#else
			dl_demod(&s_st_eq_out[u32_mod_len * s_u32_data_cb_start_cnt + SC_NUM], s8p_llr_ptr, \
						u32_llr_shift, u32_mod_len, s_st_plcp_info.u16_mcs); 
			#endif
			dl_turbo_dec_start(DATA_TYPE, s8p_llr_ptr, u32_turbo_in_len, s_st_plcp_info.u32_output_lenght[0], u32_crc_type);
			s_u32_data_cb_start_cnt++;
		}			
	}

	while( (s_u32_data_sym_eq_cnt * SC_NUM)  >= ((s_u32_data_cb_start_cnt + 1) * u32_mod_len))
	{
		//eq data enough to do demod
		s8p_llr_ptr = dl_turbo_input_addr_get(u32_turbo_in_len);
		//eq data enough to do turbo decode
		ks_oal_mem_set(s8p_llr_ptr, 0x0, u32_llr_shift);
		#if DEMOD_WITH_SNR
		dl_demod_with_snr(&s_st_eq_out[u32_mod_len * s_u32_data_cb_start_cnt + SC_NUM], s8p_llr_ptr, \
					&g_u16_snr[u32_mod_len * s_u32_data_cb_start_cnt + SC_NUM], u32_llr_shift, u32_mod_len, s_st_plcp_info.u16_mcs); 
		#else
		dl_demod(&s_st_eq_out[u32_mod_len * s_u32_data_cb_start_cnt + SC_NUM], s8p_llr_ptr, \
					u32_llr_shift, u32_mod_len, s_st_plcp_info.u16_mcs); 
		#endif
		dl_turbo_dec_start(DATA_TYPE, s8p_llr_ptr, u32_turbo_in_len, s_st_plcp_info.u32_output_lenght[0], u32_crc_type);
		s_u32_data_cb_start_cnt++;
	}		

    if(u32_2nd_sf)
	{
		//last subframe
		OAL_ASSERT(s_u32_data_cb_start_cnt <= s_st_plcp_info.u16_subframe_num, "");
		OAL_ASSERT(s_u32_data_sym_eq_cnt <= DATA_SYM_NUM, ""); 
		
		if(s_u32_data_cb_start_cnt EQ s_st_plcp_info.u16_subframe_num)
		{
			s_u32_data_left_cnt = 0;
			s_u32_data_cb_start_cnt = 0;
			s_u32_data_sym_pre_cnt = 0;
			s_u32_data_sym_eq_cnt = 0;
			g_u32_ics_flag = ICS_CCA_DETECT;
			s_u32_dec_cnt = 0;
			#if 1
			ks_abs_mean_calc(&s_st_eq_out[SC_NUM], SC_NUM, u16_eq_abs_mean);
			KS_LOG(0, 0xA21A, u16_eq_abs_mean[0]);
			KS_LOG(0, 0xA21A, u16_eq_abs_mean[1]);			
			#endif
		}
	}

}

OAL_ITCM_CODE_SECTION
VOID dl_mimo_proc(UINT32 u32_pilot_num)
{
	UINT32 u32_i = 0;
	UINT32 u32_max_id = 0;
	UINT32 u32_rx_ant_id = 0;	
	UINT32 u32_tx_ant_id = 0;	
	SINT32 s32_snr_max = -200;
	SINT32 s32_snr_ant0 = 0;	
	SINT32 s32_snr_ant1 = 0;
	SINT32 s32_sig_pwr = 0;
	SINT32 s32_noise_pwr = 0;
	SINT32 s32_snr = 0;
	SINT8  s8_rssi = 0;
	SINT8  s8_pathloss = 0;
	SINT32 s32_snr_tmp[4] = {0};
	sram_1643_4500_t *stp_sarm_1643_4500 = (sram_1643_4500_t*)(SRAM_1643_4500_ADDR);	//g_st_iq_dump


	for(u32_i = 0; u32_i < 4; u32_i++ )
	{
		if(s_s32_snr[u32_pilot_num][u32_i] > 0)
		{
			s32_snr_tmp[u32_i] = s_s32_snr[u32_pilot_num][u32_i];
		}
		else
		{
			s32_snr_tmp[u32_i] = 0;
		}
	}
	s32_snr_ant0 = (s32_snr_tmp[0] + s32_snr_tmp[1]);// * (s_s16_shift_value_max - s_s16_shift_value[0] + 1);
	s32_snr_ant1 = (s32_snr_tmp[2] + s32_snr_tmp[3]);// * (s_s16_shift_value_max - s_s16_shift_value[1] + 1);

	if(s32_snr_ant0 > s32_snr_ant1 + (SINT32)(MIMO_SNR_THRES << 1))
	{
		//temp here, one snr lower 10db, 
		g_u32_rx_ant_num = 1;
		g_u32_rx_ant_1st_id = 0;
	}
	else if(s32_snr_ant1 > s32_snr_ant0 + (SINT32)(MIMO_SNR_THRES << 1))
	{
		g_u32_rx_ant_num = 1;
		g_u32_rx_ant_1st_id = 1;
	}
	
	if(READ_REG32(ASSERT_4500_ADDR) EQ 0x1236)
	{
		OAL_ASSERT(g_u32_rx_ant_num EQ 2, "");
	}
	if(g_u32_rx_ant_num EQ 1)
	{
		if(READ_REG32(ASSERT_4500_ADDR) EQ RX_DUMP_RX_ANT_LESS)
		{
			OAL_ASSERT(0, "");
		}
		u32_max_id = (g_u32_rx_ant_1st_id << 1);
		for(u32_i = 0; u32_i < 2; u32_i++)
		{
			if(s32_snr_max <= s_s32_snr[u32_pilot_num][(g_u32_rx_ant_1st_id << 1) + u32_i])
			{
				//find snr max
				s32_snr_max = s_s32_snr[u32_pilot_num][(g_u32_rx_ant_1st_id << 1) + u32_i];
				u32_max_id = (g_u32_rx_ant_1st_id << 1) + u32_i;
			}
		}

		u32_rx_ant_id = (u32_max_id >> 1);
		u32_tx_ant_id = (u32_max_id & 1);	

		OAL_ASSERT(u32_rx_ant_id EQ g_u32_rx_ant_1st_id, "" );
		g_u32_tx_ant_num = 2;
		#if 1
		if((s_s32_snr[u32_pilot_num][(u32_rx_ant_id << 1) + (u32_tx_ant_id ^ 1)] + MIMO_SNR_THRES) < s32_snr_max)
		{
			//1t1r
			g_u32_tx_ant_num = 1;
			g_u32_tx_ant_1st_id = u32_tx_ant_id;
			s32_sig_pwr = s_s32_sig_pwr[u32_pilot_num][u32_rx_ant_id * MAX_RX_ANT_NUM + u32_tx_ant_id];
			s32_noise_pwr = s_s32_noise_pwr[u32_pilot_num][u32_rx_ant_id * MAX_RX_ANT_NUM + u32_tx_ant_id];
			//g_zzw_ch_status.snr_sub[u32_rx_ant_id * 2 + u32_tx_ant_id] = s_s32_snr[u32_pilot_num][u32_rx_ant_id * MAX_RX_ANT_NUM + u32_tx_ant_id];
		}
		else
		#endif
		{
			//2t1r		
			s32_sig_pwr = s_s32_sig_pwr[u32_pilot_num][u32_rx_ant_id * MAX_RX_ANT_NUM] + \
						s_s32_sig_pwr[u32_pilot_num][u32_rx_ant_id * MAX_RX_ANT_NUM + 1];
			s32_noise_pwr = s_s32_noise_pwr[u32_pilot_num][u32_rx_ant_id * MAX_RX_ANT_NUM] + \
						s_s32_noise_pwr[u32_pilot_num][u32_rx_ant_id * MAX_RX_ANT_NUM + 1];
			
			//g_zzw_ch_status.snr_sub[u32_rx_ant_id * 2] = s_s32_snr[u32_pilot_num][u32_rx_ant_id * MAX_RX_ANT_NUM];
			//g_zzw_ch_status.snr_sub[u32_rx_ant_id * 2 + 1] = s_s32_snr[u32_pilot_num][u32_rx_ant_id * MAX_RX_ANT_NUM + 1];
		}
		s8_rssi = g_zzw_ch_status.rssi_sub[g_u32_rx_ant_1st_id];
		s8_pathloss = g_zzw_ch_status.pathloss_sub[g_u32_rx_ant_1st_id];
		g_zzw_ch_status.snr_sub[(u32_rx_ant_id << 1)] = s_s32_snr[u32_pilot_num][(u32_rx_ant_id << 1)];
		g_zzw_ch_status.snr_sub[(u32_rx_ant_id << 1) + 1] = s_s32_snr[u32_pilot_num][(u32_rx_ant_id << 1) + 1];
	}
	else
	{
		s32_snr_ant0 = s32_snr_tmp[0] + s32_snr_tmp[2];
		s32_snr_ant1 = s32_snr_tmp[1] + s32_snr_tmp[3];
		
		g_u32_tx_ant_num = 2;
		#if 1
		if(s32_snr_ant0 > s32_snr_ant1 + (MIMO_SNR_THRES << 2))
		{
			//1t2r		
			g_u32_tx_ant_num = 1;
			g_u32_tx_ant_1st_id = 0;
			
			s32_sig_pwr = s_s32_sig_pwr[u32_pilot_num][0] + s_s32_sig_pwr[u32_pilot_num][2];
			s32_noise_pwr = s_s32_noise_pwr[u32_pilot_num][0] + s_s32_noise_pwr[u32_pilot_num][2];
			//g_zzw_ch_status.snr_sub[0] = s_s32_snr[u32_pilot_num][0];
			//g_zzw_ch_status.snr_sub[2] = s_s32_snr[u32_pilot_num][2];

			
		}
		else if(s32_snr_ant1 > s32_snr_ant0 + (MIMO_SNR_THRES << 2))
		{
			//1t2r	
			g_u32_tx_ant_num = 1;
			g_u32_tx_ant_1st_id = 1;
			
			s32_sig_pwr = s_s32_sig_pwr[u32_pilot_num][1] + s_s32_sig_pwr[u32_pilot_num][3];
			s32_noise_pwr = s_s32_noise_pwr[u32_pilot_num][1] + s_s32_noise_pwr[u32_pilot_num][3];

			//g_zzw_ch_status.snr_sub[1] = s_s32_snr[u32_pilot_num][1];
			//g_zzw_ch_status.snr_sub[3] = s_s32_snr[u32_pilot_num][3];
			
		}
		else
		#endif
		{
			//2t2r
			s32_sig_pwr = s_s32_sig_pwr[u32_pilot_num][0] + s_s32_sig_pwr[u32_pilot_num][1] + \
						s_s32_sig_pwr[u32_pilot_num][2] + s_s32_sig_pwr[u32_pilot_num][3];
			s32_noise_pwr = s_s32_noise_pwr[u32_pilot_num][0] + s_s32_noise_pwr[u32_pilot_num][1] + \
						s_s32_noise_pwr[u32_pilot_num][2] + s_s32_noise_pwr[u32_pilot_num][3];
		}

		s8_rssi = (g_zzw_ch_status.rssi_sub[0]);
		if(s8_rssi < (SINT8)g_zzw_ch_status.rssi_sub[1])
		{
			s8_rssi = g_zzw_ch_status.rssi_sub[1];
		}

		s8_pathloss = (g_zzw_ch_status.pathloss_sub[0]);
		if(s8_pathloss > g_zzw_ch_status.pathloss_sub[1])
		{
			s8_pathloss = g_zzw_ch_status.pathloss_sub[1];
		}
		g_zzw_ch_status.snr_sub[0] = s_s32_snr[u32_pilot_num][0];
		g_zzw_ch_status.snr_sub[1] = s_s32_snr[u32_pilot_num][1];
		g_zzw_ch_status.snr_sub[2] = s_s32_snr[u32_pilot_num][2];
		g_zzw_ch_status.snr_sub[3] = s_s32_snr[u32_pilot_num][3];	

	}

	g_zzw_ch_status.pathloss = s8_pathloss;
	g_zzw_ch_status.rssi = s8_rssi;
	//g_u32_rx_ant_num = 1;
	KS_LOG(0, 0xA11E, (UINT16)((g_u32_tx_ant_num << 12) | (g_u32_rx_ant_num << 8) | (g_u32_tx_ant_1st_id << 4) | (g_u32_rx_ant_1st_id)));
	KS_LOG(0, 0xA11E, (UINT16)s8_rssi);

	if(s32_sig_pwr >= s32_noise_pwr )
    {
    	s32_snr = ks_logx(10, s32_sig_pwr / (s32_noise_pwr)); /*Q26*/
        s32_snr = (s32_snr + (0x1<<25))>>20;
	}
	else
	{
    	s32_snr = ks_logx(10, (s32_noise_pwr) / s32_sig_pwr); /*Q26*/
        s32_snr = (-s32_snr + (0x1<<25))>>20; 
	}
    s32_snr = ((10*s32_snr)>>6) - 4;
	if(g_u32_rx_ant_num EQ 1)
	{
		s32_snr -= 3;
	}
	s_s32_snr_rpt = s32_snr;

	KS_LOG(0, 0xA21B, (UINT16)s32_snr);

}

OAL_ITCM_CODE_SECTION
VOID dl_data_dec_pilot(complex_s16_t* stp_input, UINT32 u32_pilot_num)
{
	UINT32 u32_ant_id;
	UINT32 u32_delta_of_ants = DELTA_OF_ANT_BUF;    
	UINT32 u32_tx_ant_bmp = 0;

	node_info_t *stp_node_info;
    ks_fft_para_t st_fft_para;
	complex_s16_t* stp_temp = (complex_s16_t*)stp_input;

	/*transform into freq domain*/
	//st_fft_para.s16cp_outdata_addr	= s_st_fft_out;
	st_fft_para.s16cp_twiddle_addr	= (complex_s16_t*)ceva_fft_lib_fft_2048_twi_table;
	st_fft_para.s32p_shift_amt_addr = s_s32_shift_amt_fft;
	st_fft_para.s16cp_temp_addr 	= (complex_s16_t*)(s_st_temp_buffer[1]);
		

	//KS_LOG(0, 0xA11E, (UINT16)u32_pilot_num);
	//KS_LOG_TMRL(0xA11E);
	//KS_LOG(0, 0xa11e, (UINT16)((UINT32)stp_input));
	//KS_LOG(0, 0xa11e, (UINT16)(((UINT32)stp_input) >>16 ));
	if(u32_pilot_num EQ 0)
	{
		g_u32_tx_ant_num = 0;
		g_u32_tx_ant_1st_id = 0;
	}
	#if 0
	else if(u32_pilot_num EQ 1)
	{
		if(stp_node_info = (node_info_t*)node_tbl_find(s_st_algo2l1cc_ctrl_info.st_ctrl_info.u8_node_id))
		{
			//KS_LOG(0, 0xA417, (UINT16)((-stp_node_info->s32_afc_angle * 937) >> 8));

			#if FREQ_OFFSET_COARSE
			dl_freq_comp_seq_gen(s_st_cfo_seq, stp_node_info->s32_afc_angle, OAL_TRUE, OAL_TRUE);
			#else
			dl_freq_comp_seq_gen(s_st_cfo_seq, stp_node_info->s32_afc_angle, OAL_FALSE, OAL_FALSE);
			#endif
		}
	}
	#endif
 
	for(u32_ant_id = 0; u32_ant_id < RX_ANT_NUM; u32_ant_id++)
	{
		stp_temp = stp_input + (u32_delta_of_ants * u32_ant_id + s_s32_sync_pos_off[u32_ant_id]);
		if(g_u32_cca_detected & (1 << u32_ant_id))
		{
			//if(s_u32_freq_ind)
			#if FUN_DC_REMOVE
			//dc remove
	        _ks_cxv_add_cx(stp_temp + CP_LEN, SYMBOL_LEN, &s_st_dc[u32_ant_id],	&s_st_temp_buffer[0][0]);
			#if FREQ_OFFSET_COARSE
			dl_freq_offset_comp(&s_st_temp_buffer[0][0], &s_st_temp_buffer[0][0], &(s_st_cfo_seq.st_cfo_seq[CP_LEN >> 4]), SYMBOL_LEN, OAL_TRUE);
			#else
		    /*the second freq offset correction of pilot*/
			dl_freq_offset_comp(&s_st_temp_buffer[0][0], &s_st_temp_buffer[0][0], &(s_st_cfo_seq.st_cfo_seq[CP_LEN]), SYMBOL_LEN, OAL_FALSE);
			#endif
			#else
			#if FREQ_OFFSET_COARSE
			dl_freq_offset_comp(stp_temp + CP_LEN, &s_st_temp_buffer[0][0], &(s_st_cfo_seq.st_cfo_seq[CP_LEN >> 4]), SYMBOL_LEN, OAL_TRUE);
			#else
		    /*the second freq offset correction of pilot*/
			dl_freq_offset_comp(stp_temp + CP_LEN, &s_st_temp_buffer[0][0], &(s_st_cfo_seq.st_cfo_seq[CP_LEN]), SYMBOL_LEN, OAL_FALSE);
			#endif
			#endif

		    //ks_cxv_mul_cxv(stp_temp + CP_LEN, &s_st_cfo_seq[0], SYMBOL_LEN, 0, s_st_temp_buffer[0]);
			//dl_freq_offset_est(&s_st_temp_buffer[0][0], &s_st_temp_buffer[0][SYMBOL_LEN], CP_LEN, SYMBOL_LEN, 0, OAL_FALSE);
			#if 1//def MIMO2
		    //if(u32_pilot_num NEQ 1)
		    {
		    	_ks_cxv_mul_cx(&s_st_temp_buffer[0][0], SYMBOL_LEN, &s_st_cfo_seq.st_slot_phase[PILOT_DIS_SYM * u32_pilot_num], 0, &s_st_temp_buffer[0][0]);
			}
			#else
		    if(u32_pilot_num > 0)
		    {
		    	_ks_cxv_mul_cx(&s_st_temp_buffer[0][0], SYMBOL_LEN, &s_st_cfo_seq.st_slot_phase[PILOT_DIS_SYM * u32_pilot_num - 1], 0, &s_st_temp_buffer[0][0]);
			}
			#endif
			#if RSSI_BIT_SHIFT_SAME
		    _ks_shift_s16v((SINT16_PTR)(&s_st_temp_buffer[0][0]), SYMBOL_LEN*2, s_s16_shift_value_min - RX_SHIFT_VAL, (SINT16_PTR)(&s_st_temp_buffer[0][0]));
			#else
			if(s_s16_shift_value_max - s_s16_shift_value_min < 2)
			{
		    	_ks_shift_s16v((SINT16_PTR)(&s_st_temp_buffer[0][0]), SYMBOL_LEN*2, s_s16_shift_value[u32_ant_id] - RX_SHIFT_VAL, (SINT16_PTR)(&s_st_temp_buffer[0][0]));
			}
			else
			{
		    	_ks_shift_s16v((SINT16_PTR)(&s_st_temp_buffer[0][0]), SYMBOL_LEN*2, s_s16_shift_value_min - RX_SHIFT_VAL, (SINT16_PTR)(&s_st_temp_buffer[0][0]));
			}
			#endif
			st_fft_para.s16cp_indata_addr	= &s_st_temp_buffer[0][0];
			st_fft_para.s16cp_outdata_addr	= s_st_pilot_iq;
			ks_fft2048(&st_fft_para);

		    /*caculate H*/
			//Hx1
		    s_s32_snr[u32_pilot_num][u32_ant_id * MAX_RX_ANT_NUM] = dl_H_calc(s_st_pilot_iq, \
		    			s_st_H_vec[u32_pilot_num][u32_ant_id * MAX_RX_ANT_NUM], \
		    			&s_s32_sig_pwr[u32_pilot_num][u32_ant_id * MAX_RX_ANT_NUM],\
		    			&s_s32_noise_pwr[u32_pilot_num][u32_ant_id * MAX_RX_ANT_NUM], 0);

		    #if 0
			if((u32_pilot_num EQ 0) && (s_s32_snr[u32_pilot_num][u32_ant_id * MAX_RX_ANT_NUM] >= TX_ANT_SNR_THRES))
			{	
				//tmp here, judge whether tx sig exists in ant0
				//g_u32_tx_ant_num += 1;
				u32_tx_ant_bmp |= 0x1;
			}
			#endif


			//Hx2
			if(TX_ANT_NUM EQ 2)
			{
			    s_s32_snr[u32_pilot_num][u32_ant_id * MAX_RX_ANT_NUM + 1] = dl_H_calc(s_st_pilot_iq,\
			    			s_st_H_vec[u32_pilot_num][u32_ant_id * MAX_RX_ANT_NUM + 1], \
			    			&s_s32_sig_pwr[u32_pilot_num][u32_ant_id * MAX_RX_ANT_NUM + 1],\
			    			&s_s32_noise_pwr[u32_pilot_num][u32_ant_id * MAX_RX_ANT_NUM + 1], 1);

			    #if 0
			   	if((u32_pilot_num EQ 0) && (s_s32_snr[u32_pilot_num][u32_ant_id * MAX_RX_ANT_NUM + 1] >= TX_ANT_SNR_THRES))
			   	{	
					//tmp here, judge whether tx sig exists in ant0
			   		//g_u32_tx_ant_num += 1;
					u32_tx_ant_bmp |= (0x1 << 1);
			   	}
			   	#endif
			}
		}
		else
		{
			s_s32_snr[u32_pilot_num][u32_ant_id * MAX_RX_ANT_NUM] = -200;
			s_s32_snr[u32_pilot_num][u32_ant_id * MAX_RX_ANT_NUM + 1] = -200;			
		}
	}
	
	if(u32_pilot_num EQ 0)
	{
		dl_mimo_proc(0);
	}
	#if 0
	//#if DEMOD_WITH_SNR
	if(u32_pilot_num EQ 1)
	{
		s_s32_snr_mean = ks_cxv_rssi_calc(s_st_H_vec[1], SC_NUM);
		ks_cxv_squa_s16(s_st_H_vec[1], SC_NUM, &g_u32_snr[SC_NUM]);
		dl_snr_shift(&g_u32_snr[SC_NUM], &g_u16_snr[SC_NUM], \
					SC_NUM, s_s32_snr_mean);
	}
	#endif

	#ifdef MIMO2
    g_u32_ics_sym_bmp |= (1 << (u32_pilot_num * PILOT_DIS_SYM + 1));
	#else
    g_u32_ics_sym_bmp |= (1 << (u32_pilot_num * PILOT_DIS_SYM));
	#endif

	if((u32_pilot_num EQ 1) && (g_u32_ics_sym_bmp & ICS_1ST_SF_BMP) && (s_u32_data_sym_pre_cnt > 0))
	{
		//2nd pilot in curr frame, 0 ~ (s_u32_data_sym_pre_cnt - 1) data symbols in last frame , need do decode here 
		//OAL_ASSERT((s_u32_data_sym_pre_cnt <= 4), "");
		dl_data_dec_sf(0, s_u32_data_sym_pre_cnt);
	}

	if((u32_pilot_num EQ 2) && (g_u32_ics_sym_bmp & ICS_2ND_SF_BMP) && (s_u32_data_sym_pre_cnt >= 4))
	{
		//3rd pilot in curr frame, 4 ~ (s_u32_data_sym_pre_cnt - 1) data symbols in last frame , need do decode here 
		//OAL_ASSERT((s_u32_data_sym_pre_cnt <= 9), "");
		#ifdef MIMO2
		dl_data_dec_sf(PILOT1_SYM_ID + 1, s_u32_data_sym_pre_cnt - PILOT1_SYM_ID);		
		#else
		dl_data_dec_sf(PILOT1_SYM_ID + 1, s_u32_data_sym_pre_cnt - PILOT1_SYM_ID);		
		#endif
	}

	#if 0
	if(u32_pilot_num > 0)
	{
    	dl_freq_offset_est(&s_st_H_vec[u32_pilot_num - 1][0], &s_st_H_vec[u32_pilot_num][0], SC_NUM, SLOT_LEN * 6, 0, OAL_FALSE);
	}
	if(u32_pilot_num >= 1)
	{
		if((g_u32_ics_sym_bmp&ICS_1ST_SF_BMP) EQ ICS_1ST_SF_BMP)
		{
			//1st subframe processed, need decode here
			dl_data_dec_sf(OAL_TRUE);
			g_u32_ics_sym_bmp &= (~ICS_1ST_SF_BMP);
		}
		else if((g_u32_ics_sym_bmp&ICS_2ND_SF_BMP) EQ ICS_2ND_SF_BMP)
		{
			//2nd subframe processed, need decode here
			dl_data_dec_sf(OAL_FALSE);
			g_u32_ics_sym_bmp &= (~ICS_2ND_SF_BMP);
		}
	}
    #endif
}

OAL_ITCM_CODE_SECTION
VOID dl_data_dec_data(complex_s16_t* stp_input, UINT32 u32_sym_idx)
{
	/*u32_sym_idx = 0, means first data symbol in one frame*/
    ks_fft_para_t st_fft_para;
    SINT32 s32_noise_pwr = 0;
    #ifdef MIMO2
    UINT32 u32_cfo_idx = u32_sym_idx + 1;
    #else
    UINT32 u32_cfo_idx = u32_sym_idx + 2;
    #endif
    UINT32 u32_data_idx = u32_sym_idx;
	complex_s16_t* stp_temp = (complex_s16_t*)stp_input;
	UINT32 u32_ant_id;
	UINT32 u32_delta_of_ants = DELTA_OF_ANT_BUF;    

	//KS_LOG(0, 0xA11D, u32_sym_idx);
		
	#if 1
	if(u32_data_idx >= DATA_SYM_ID_2SF)
	{
		u32_data_idx -= DATA_SYM_ID_2SF;
	}
	else
	{
		u32_data_idx += 1;	
	}
	#endif	

	for(u32_ant_id = 0; u32_ant_id < RX_ANT_NUM; u32_ant_id++)
	{
		stp_temp = stp_input + (u32_delta_of_ants * u32_ant_id + s_s32_sync_pos_off[u32_ant_id]);
		if(g_u32_cca_detected & (1 << u32_ant_id))
		{
		    /*correct the freq offset*/

		    #if FUN_DC_REMOVE
		    _ks_cxv_add_cx(stp_temp + CP_LEN, SYMBOL_LEN, &s_st_dc[u32_ant_id],	&s_st_temp_buffer[0][0]);
			#if FREQ_OFFSET_COARSE
			dl_freq_offset_comp(&s_st_temp_buffer[0][0], &s_st_temp_buffer[0][0], &(s_st_cfo_seq.st_cfo_seq[CP_LEN >> 4]), SYMBOL_LEN, OAL_TRUE);
			#else
		    /*the second freq offset correction of pilot*/
			dl_freq_offset_comp(&s_st_temp_buffer[0][0], &s_st_temp_buffer[0][0], &(s_st_cfo_seq.st_cfo_seq[CP_LEN]), SYMBOL_LEN, OAL_FALSE);
			#endif
			#else
			#if FREQ_OFFSET_COARSE
			dl_freq_offset_comp(stp_temp + CP_LEN, &s_st_temp_buffer[0][0], &(s_st_cfo_seq.st_cfo_seq[CP_LEN >> 4]), SYMBOL_LEN, OAL_TRUE);
			#else
			dl_freq_offset_comp(stp_temp + CP_LEN, &s_st_temp_buffer[0][0], &(s_st_cfo_seq.st_cfo_seq[CP_LEN]), SYMBOL_LEN, OAL_FALSE);
			#endif
			#endif
		    //ks_cxv_mul_cxv(stp_temp + CP_LEN, &s_st_cfo_seq[0], SYMBOL_LEN, 0, s_st_temp_buffer[0]);
		    _ks_cxv_mul_cx(s_st_temp_buffer[0], SYMBOL_LEN, &s_st_cfo_seq.st_slot_phase[u32_cfo_idx], 0, s_st_temp_buffer[1]);
			#if RSSI_BIT_SHIFT_SAME
			_ks_shift_s16v((SINT16_PTR)&s_st_temp_buffer[1][0], (SYMBOL_LEN<<1), s_s16_shift_value_min - RX_SHIFT_VAL, (SINT16_PTR)s_st_fft_in);//OFDM PAPR 较高，shift_value进行减1操作，即再/2的操作，后面的1bit对应fft
			#else
			if(s_s16_shift_value_max - s_s16_shift_value_min < 2)
			{
				_ks_shift_s16v((SINT16_PTR)&s_st_temp_buffer[1][0], (SYMBOL_LEN<<1), s_s16_shift_value[u32_ant_id] - RX_SHIFT_VAL, (SINT16_PTR)s_st_fft_in);//OFDM PAPR 较高，shift_value进行减1操作，即再/2的操作，后面的1bit对应fft
			}
			else
			{
				_ks_shift_s16v((SINT16_PTR)&s_st_temp_buffer[1][0], (SYMBOL_LEN<<1), s_s16_shift_value_min - RX_SHIFT_VAL, (SINT16_PTR)s_st_fft_in);//OFDM PAPR 较高，shift_value进行减1操作，即再/2的操作，后面的1bit对应fft
			}
			#endif
		    /*transform into freq domain*/
		    st_fft_para.s16cp_indata_addr   = s_st_fft_in;
		    st_fft_para.s16cp_outdata_addr  = s_st_fft_out;
		    st_fft_para.s16cp_twiddle_addr  = (complex_s16_t*)ceva_fft_lib_fft_2048_twi_table;
		    st_fft_para.s32p_shift_amt_addr = s_s32_shift_amt_fft;
		    st_fft_para.s16cp_temp_addr     = (complex_s16_t*)(s_st_temp_buffer[0]);

		    ks_fft2048(&st_fft_para);

		    /*collect the 1200 subcarrier*/
		    

		    #if 1//def SCFDMA_SWITCH
			ks_oal_mem_copy(&s_st_data_iq[u32_ant_id][0 + SC_NUM * u32_data_idx], &s_st_fft_out[SYMBOL_LEN - HALF_SC_NUM], (HALF_SC_NUM<<2));
			ks_oal_mem_copy(&s_st_data_iq[u32_ant_id][HALF_SC_NUM + SC_NUM * u32_data_idx], &s_st_fft_out[1], (HALF_SC_NUM<<2));
			#else
			ks_oal_mem_copy(&s_st_data_iq[u32_ant_id][0 + SC_NUM * u32_data_idx], &s_st_fft_out[1], (HALF_SC_NUM<<2));
			ks_oal_mem_copy(&s_st_data_iq[u32_ant_id][HALF_SC_NUM + SC_NUM * u32_data_idx], &s_st_fft_out[SYMBOL_LEN - HALF_SC_NUM], (HALF_SC_NUM<<2));

		    #endif
    	}
    }
	g_u32_ics_sym_bmp |= (1 << (u32_sym_idx + SYMBOL_NUM_BEFORE_1STDATA));

	s_u32_data_sym_pre_cnt++;

	if(OAL_CRC_FAILURE EQ g_u32_ics_head_crc)
	{
		KS_LOG(0, 0xA11D, 0xDEAD);
		s_u32_data_sym_pre_cnt = 0;
		g_u32_ics_sym_bmp = 0;
	}
	else
	{
		#if 1
		if(((u32_sym_idx < PILOT1_SYM_ID) && (g_u32_ics_sym_bmp & (1 << PILOT_1_BMP))) || ((u32_sym_idx > PILOT1_SYM_ID) && (g_u32_ics_sym_bmp & (1 << PILOT_2_BMP))))
		{
			dl_data_dec_sf(u32_sym_idx, 1);
		}
		#else
		if((u32_sym_idx EQ 3) && (g_u32_ics_sym_bmp & (1 << PILOT_1_BMP)))
		{
			dl_data_dec_sf(0, 4);		
		}
		else if((u32_sym_idx EQ 9) && (g_u32_ics_sym_bmp & (1 << PILOT_2_BMP)))
		{
			dl_data_dec_sf(5, 5);			
		}
		#endif
	
	}
	return;

}

OAL_ITCM_CODE_SECTION
VOID dl_data_dec_pre(complex_s16_t* stp_input, UINT32 u32_sym_idx_first, UINT32 u32_sym_idx_last)
{
	/*
	u32_sym_idx(first or last) : 0 ~ 3, first 4 data symbols
	u32_sym_idx(first or last) : 4, 2nd pilot symbol
	u32_sym_idx(first or last) : 5 ~ 9, last 5 data symbol
	u32_sym_idx(first or last) : 10, last pilot symbol
	eg: u32_sym_idx_first = 1,  u32_sym_idx_last = 5, means symbols need to proc here is data symbol 1 ~ 3, pilot 1(symbol 4) , data symbol 4(symbol 5)
	*/
	UINT32 u32_sym_num = u32_sym_idx_last - u32_sym_idx_first + 1;
	UINT32 u32_i;

	KS_LOG(OAL_TRACE_LEVEL_0 , 0xA122 , (UINT16)((u32_sym_idx_last << 8) | (u32_sym_idx_first)));
	OAL_ASSERT(u32_sym_idx_last >= u32_sym_idx_first, "");
	
	if(u32_sym_idx_first < PILOT1_SYM_ID)
	{
		if(u32_sym_idx_last < PILOT1_SYM_ID)
		{
			for(u32_i = 0; u32_i < u32_sym_num; u32_i++)
			{
				dl_data_dec_data(stp_input + SLOT_LEN * u32_i, u32_sym_idx_first + u32_i);	
			}
		}
		else if(u32_sym_idx_last EQ PILOT1_SYM_ID)
		{
			dl_data_dec_pilot(stp_input + SLOT_LEN * (u32_sym_num - 1), 1);			
			for(u32_i = 0; u32_i < (u32_sym_num - 1); u32_i++)
			{
				dl_data_dec_data(stp_input + SLOT_LEN * u32_i, u32_sym_idx_first + u32_i);	
			}
			//dl_data_dec_pilot(stp_input + SLOT_LEN * u32_i, 1);			
		}
		else if(u32_sym_idx_last < PILOT2_SYM_ID)
		{
			dl_data_dec_pilot(stp_input + SLOT_LEN * (PILOT1_SYM_ID - u32_sym_idx_first), 1);
			for(u32_i = 0; u32_i < (PILOT1_SYM_ID - u32_sym_idx_first); u32_i++) 
			{
				dl_data_dec_data(stp_input + SLOT_LEN * u32_i, u32_sym_idx_first + u32_i);	
			}
			
			//dl_data_dec_pilot(stp_input + SLOT_LEN * u32_i, 1);
			
			for(u32_i = (PILOT1_SYM_ID - u32_sym_idx_first + 1); u32_i < u32_sym_num; u32_i++)
			{
				dl_data_dec_data(stp_input + SLOT_LEN * u32_i, u32_sym_idx_first + u32_i);	
			}
		}
		else if(u32_sym_idx_last EQ PILOT2_SYM_ID)
		{
			dl_data_dec_pilot(stp_input + SLOT_LEN * (PILOT1_SYM_ID - u32_sym_idx_first), 1);
			for(u32_i = 0; u32_i < (PILOT1_SYM_ID - u32_sym_idx_first); u32_i++)
			{
				dl_data_dec_data(stp_input + SLOT_LEN * u32_i, u32_sym_idx_first + u32_i);	
			}
			//dl_data_dec_pilot(stp_input + SLOT_LEN * u32_i, 1);
			dl_data_dec_pilot(stp_input + SLOT_LEN * (u32_sym_num - 1), 2);
			for(u32_i = (PILOT1_SYM_ID - u32_sym_idx_first + 1); u32_i < (u32_sym_num - 1); u32_i++)
			{
				dl_data_dec_data(stp_input + SLOT_LEN * u32_i, u32_sym_idx_first + u32_i);	
			}
			
			//dl_data_dec_pilot(stp_input + SLOT_LEN * u32_i, 2);
		}
		else
		{
			OAL_ASSERT(0, "");
		}
	}
	else if(u32_sym_idx_first EQ PILOT1_SYM_ID)
	{
		if(u32_sym_idx_last EQ PILOT1_SYM_ID)
		{
			dl_data_dec_pilot(stp_input, 1);			
		}
		else if(u32_sym_idx_last < PILOT2_SYM_ID)
		{
			dl_data_dec_pilot(stp_input, 1);
			
			for(u32_i = 1; u32_i < u32_sym_num; u32_i++)
			{
				dl_data_dec_data(stp_input + SLOT_LEN * u32_i, u32_sym_idx_first + u32_i);	
			}
		}
		else if(u32_sym_idx_last EQ PILOT2_SYM_ID)
		{
			dl_data_dec_pilot(stp_input, 1);
			dl_data_dec_pilot(stp_input + SLOT_LEN * (u32_sym_num - 1), 2);
			
			for(u32_i = 1; u32_i < (u32_sym_num - 1); u32_i++)
			{
				dl_data_dec_data(stp_input + SLOT_LEN * u32_i, u32_sym_idx_first + u32_i);	
			}
			
			//dl_data_dec_pilot(stp_input + SLOT_LEN * u32_i, 2);
		}
		else
		{
			OAL_ASSERT(0, "");
		}		

	}
	else if(u32_sym_idx_first < PILOT2_SYM_ID)
	{
		if(u32_sym_idx_last < PILOT2_SYM_ID)
		{
			for(u32_i = 0; u32_i < u32_sym_num; u32_i++)
			{
				dl_data_dec_data(stp_input + SLOT_LEN * u32_i, u32_sym_idx_first + u32_i);	
			}
		}
		else if(u32_sym_idx_last EQ PILOT2_SYM_ID)
		{
			dl_data_dec_pilot(stp_input + SLOT_LEN * (u32_sym_num - 1), 2);
			for(u32_i = 0; u32_i < (u32_sym_num - 1); u32_i++)
			{
				dl_data_dec_data(stp_input + SLOT_LEN * u32_i, u32_sym_idx_first + u32_i);	
			}
			
			//dl_data_dec_pilot(stp_input + SLOT_LEN * u32_i, 2);
		}
		else
		{
			OAL_ASSERT(0, "");
		}		

	}
	else if(u32_sym_idx_first EQ PILOT2_SYM_ID)
	{
		dl_data_dec_pilot(stp_input, 2);
		//to be done! zz
	}
	else
	{
		OAL_ASSERT(0, ""); 
	}	

}

OAL_ITCM_CODE_SECTION
__attribute__((noinline)) BOOLEAN dl_cca_recheck(complex_s16_t* seq_in, complex_s16_t* seq_conj_in,SINT32 s32_length, IN UINT32 u32_ant_id)//length 必须设计成16的倍数
{
    SINT32 s32_shift = 0;
    SINT32 s32_shift_remain=0;
    SINT32 s32_iloop;
    SINT32 s32_mac_conj[2];
    SINT32 s32_mac[2];
	UINT64 u64_cca_mac = 0;
	UINT64 u64_cca_mac_conj = 0;
	BOOLEAN b_ret = OAL_FALSE;


    vec_t v32_1;
    vec_t v32_2;
    vec_t v32_3;
    vec_t v32_4;
    vec_t v32_5;

    vec40_t v40_1;
    vec40_t v40_2;

    vec40_t v40_3;
    vec40_t v40_4;


    vec40_t v40_shift;
    coef_t vcoef_shift;
    coef_t vcoef_shift_remain;

	#if 1
    s32_shift = (2 * s_s16_shift_value[u32_ant_id] - 10);
    s32_shift_remain = 0;

    if(s32_shift > 7)
    {
        s32_shift_remain = s32_shift - 7;
        s32_shift = 7;
    }
    else if(s32_shift < -8)
    {
        s32_shift_remain = s32_shift + 8;
        s32_shift = -8;
    }
    #else
	s32_shift = -8;
	s32_shift_remain = 0;
    #endif

    __asm__("vpush {dw} modv0");
    __asm__("nop");
    __asm__("nop");
    __asm__("push {dw} a8");
    __asm__("mov #0x00000E06, a8");
    __asm__("nop");
    __asm__("nop");
    __asm__("vmova a8, modv0");
    __asm__("pop {dw} a8");
    __asm__("nop");
    __asm__("nop");

    vcoef_shift = vlddw_c32_clone_1dw (&s32_shift);
    v40_shift = vfillw_c32_v40 (8, vcoef_shift);

    vcoef_shift_remain = vlddw_c32_clone_1dw (&s32_shift_remain);

    v40_1 = vclr_v32 (8);
    v40_2 = vclr_v32 (8);
    v40_3 = vclr_v32 (8);
    v40_4 = vclr_v32 (8);

    for(s32_iloop = 0; s32_iloop < s32_length; )
    {
        v32_1 = vlddw_v32 (8, seq_in+s32_iloop, 0);
        v32_2 = vlddw_v32 (8, seq_conj_in+s32_iloop, 0);

        v40_2 = vmacx_v32_v32_v40_v40_conj (8, v32_1, 0, v32_2, 0, v40_2, v40_1);
        v40_4 = vmacx_v32_v32_v40_v40 (8, v32_1, 0, v32_2, 0, v40_4, v40_3);

        s32_iloop += 16;
    }

    v40_1 = vadd40_v40_v40_v40_out (8, v40_1, v40_2);
    v40_3 = vadd40_v40_v40_v40_out (8, v40_3, v40_4);

    v32_1 = vshiftx40_v40_v40_v32_w_s (4, v40_1, v40_shift, 0);
    v32_5 = vshiftx40_v40_v40_v32_w_s (4, v40_3, v40_shift, 0);

    v32_2 = vshift_v32_c32_v32_ar(8, v32_1, vcoef_shift_remain);

    v32_3 = vaddintx_v32_v32 (4, v32_2);
    v32_2 = vvmov_v40_v32_vuX (1, v32_3);

    v32_4 = vadd32_v32_v32_v32 (2, v32_2, v32_3);
	//v32_4 = vshift_v32_c32_v32_ar(2, v32_4, vcoef_shift_remain);

    vstdw_v32_vuX (2, 0, v32_4, 0, s32_mac_conj);

    u64_cca_mac_conj = (UINT64)(((SINT64)s32_mac_conj[0])*((SINT64)s32_mac_conj[0])+((SINT64)s32_mac_conj[1])*((SINT64)s32_mac_conj[1]));

    v32_2 = vshift_v32_c32_v32_ar(8, v32_5, vcoef_shift_remain);

    v32_3 = vaddintx_v32_v32 (4, v32_2);
    v32_2 = vvmov_v40_v32_vuX (1, v32_3);

    v32_4 = vadd32_v32_v32_v32 (2, v32_2, v32_3);
	//v32_4 = vshift_v32_c32_v32_ar(2, v32_4, vcoef_shift_remain);

    vstdw_v32_vuX (2, 0, v32_4, 0, s32_mac);

    __asm__("nop");
    __asm__("nop");
    __asm__("vpop {dw} modv0");
    __asm__("nop");
    __asm__("nop");

    u64_cca_mac = (UINT64)(((SINT64)s32_mac[0])*((SINT64)s32_mac[0])+((SINT64)s32_mac[1])*((SINT64)s32_mac[1]));
    //u64_cca_mac=((UINT64)u32_mac[0])*((UINT64)u32_mac[0])+((UINT64)u32_mac[1])*((UINT64)u32_mac[1]);


	if(u64_cca_mac_conj >= (u64_cca_mac))
	{
		b_ret = OAL_TRUE;
	}
	else
	{
		KS_LOG(0, 0xA214, (UINT16)(s32_mac_conj[0]));
		KS_LOG(0, 0xA214, (UINT16)(s32_mac_conj[0] >> 16));
		KS_LOG(0, 0xA214, (UINT16)(s32_mac_conj[1]));
		KS_LOG(0, 0xA214, (UINT16)(s32_mac_conj[1] >> 16));

		KS_LOG(0, 0xA215, (UINT16)(s32_mac[0]));
		KS_LOG(0, 0xA215, (UINT16)(s32_mac[0] >> 16));
		KS_LOG(0, 0xA215, (UINT16)(s32_mac[1]));
		KS_LOG(0, 0xA215, (UINT16)(s32_mac[1] >> 16));
	}
	

    return b_ret;

}



OAL_ITCM_CODE_SECTION
UINT32 dl_cca_detect(SINT32_PTR s32p_rx_buf, UINT32 u32_len, IN UINT32 u32_offset, IN UINT32 u32_fft_cnt, IN UINT32 u32_ant_id, IN SINT8 s8_pwr_thres)
{
    SINT32 s32_iloop, s32_shift;
    SINT32 s32_peak_value[2], s32_peak_index, s32_peak_max, s32_peak_min;
    SINT32 s32_noise_value;
    SINT32_PTR s32p_temp = NULL_PTR;
    UINT32 u32_start_pos = 0;
    UINT32 u32_first_path = 0;    
    ks_fft_para_t st_fft_para;
    UINT32 u32_temp[2] = {0,0};
	SINT32 s32_shift_amt_fft[5]  = {(UINT32)MODV_SV2, (UINT32)MODV_SV2, (UINT32)MODV_SV2, (UINT32)MODV_SV2, (UINT32)MODV_SV0};
	SINT32 s32_shift_amt_ifft[5]  = {(UINT32)MODV_SV1, (UINT32)MODV_SV1, (UINT32)MODV_SV1, (UINT32)MODV_SV1, (UINT32)MODV_SV0};
	UINT32 u32_thres = 0;
	SINT32 s32_noise_pos = 0;

    vec_t v32_0;
    vec_t v32_1;
    vec_t v32_2;
    vec_t v32_3;

    vec40_t v40_0;
    vec40_t v40_1;
    vec40_t v40_2;
    vec40_t v40_3;
    vec40_t v40_4;
    vec40_t v40_shift;
    coef_t  vcoef_shift;
    coef_t  vcoef_value;
    vprex_t v_prex;

    /*CCA做2048点相关检测*/
    //_ks_shift_s16v((SINT16_PTR)&s_st_rx_buffer[s_u32_pingpong_idx][AGC_LENGTH], SYMBOL_LEN, s_s16_shift_value - 8, (SINT16_PTR)s_st_fft_in);
	#if 1//FUN_DC_REMOVE
    _ks_cxv_add_cx(s32p_rx_buf, u32_len, &s_st_dc[u32_ant_id], (SINT32_PTR)s_st_fft_in);	
	_ks_shift_s16v((SINT16_PTR)s_st_fft_in, (u32_len << 1), s_s16_shift_value[u32_ant_id] - 3, (SINT16_PTR)s_st_fft_in);
	#else
	_ks_shift_s16v((SINT16_PTR)s32p_rx_buf, (u32_len << 1), s_s16_shift_value[u32_ant_id] - 4, (SINT16_PTR)s_st_fft_in);
	#endif
	if(u32_len < SYMBOL_LEN)
	{
		ks_oal_mem_set((VOID_PTR)(s_st_fft_in + u32_len), 0x0, ((SYMBOL_LEN - u32_len) << 2));
	}
    /*transform into freq domain*/
    st_fft_para.s16cp_indata_addr   = s_st_fft_in;
    st_fft_para.s16cp_outdata_addr  = s_st_fft_out;
    st_fft_para.s16cp_twiddle_addr  = (complex_s16_t*)ceva_fft_lib_fft_2048_twi_table;
    st_fft_para.s32p_shift_amt_addr = s32_shift_amt_fft;
    st_fft_para.s16cp_temp_addr     = (complex_s16_t*)(s_st_temp_buffer[0]);

    ks_fft2048(&st_fft_para);
	//while(READ_REG32(0x1724ffe0) NEQ 0);

    ks_cxv_mul_cxv(s_st_fft_out, s_st_fd_cca_seq, SYMBOL_LEN, 0, s_st_fft_in);
	//KS_LOG_TMRL(0xE110);

    st_fft_para.s16cp_indata_addr   = s_st_fft_in;    
    st_fft_para.s32p_shift_amt_addr = s32_shift_amt_ifft;//s_s32_shift_amt_ifft;

    ks_ifft2048(&st_fft_para);
	//while(READ_REG32(0x1724ffe0) NEQ 1);
	//KS_LOG_TMRL(0xE111);

	u32_start_pos = 0;
	//u32_start_pos = CCA_LEN - 1;
	s32p_temp = (SINT32_PTR)s_st_fft_out + u32_start_pos;
	#if 1
    for(s32_iloop = 0; s32_iloop < CCA_LEN; )
    {
        v32_0 = vlddw_v32 (8, &s32p_temp[s32_iloop], 0);
        v40_0 = vsqax_v32_v32 (8, v32_0, 0);

        v32_1 = vlddw_v32 (8, &s32p_temp[s32_iloop + (CCA_LEN)], 0);
        v40_1 = vsqax_v32_v32 (8, v32_1, 0);

        /*combine Region#0*/
        v40_2 = vadd40_v40_v40_v40_out (8, v40_0, v40_1);
        vstdw_v32_concat (8, v40_2, 0, &s_s32_comb_squa[s32_iloop]);

        v32_0 = vlddw_v32 (8, &s32p_temp[s32_iloop + (CCA_LEN << 1)], 0);
        v40_0 = vsqax_v32_v32 (8, v32_0, 0);

        /*combine Region#1*/
        v40_2 = vadd40_v40_v40_v40_out (8, v40_0, v40_1);
        vstdw_v32_concat (8, v40_2, 0, &s_s32_comb_squa[s32_iloop + CCA_LEN]);

        v32_1 = vlddw_v32 (8, &s32p_temp[s32_iloop + (CCA_LEN * 3)], 0);
        v40_1 = vsqax_v32_v32 (8, v32_1, 0);

        /*combine Region#2*/
        v40_2 = vadd40_v40_v40_v40_out (8, v40_0, v40_1);
        vstdw_v32_concat (8, v40_2, 0, &s_s32_comb_squa[s32_iloop + (CCA_LEN<<1)]);

        v32_0 = vlddw_v32 (8, &s32p_temp[s32_iloop + (CCA_LEN << 2)], 0);
        v40_0 = vsqax_v32_v32 (8, v32_0, 0);

        /*combine Region#3*/
        v40_2 = vadd40_v40_v40_v40_out (8, v40_0, v40_1);
        vstdw_v32_concat (8, v40_2, 0, &s_s32_comb_squa[s32_iloop + (CCA_LEN*3)]);

        v32_1 = vlddw_v32 (8, &s32p_temp[s32_iloop + (CCA_LEN * 5)], 0);
        v40_1 = vsqax_v32_v32 (8, v32_1, 0);

        /*combine Region#4*/
        v40_2 = vadd40_v40_v40_v40_out (8, v40_0, v40_1);
        vstdw_v32_concat (8, v40_2, 0, &s_s32_comb_squa[s32_iloop + (CCA_LEN<<2)]);

        v32_0 = vlddw_v32 (8, &s32p_temp[s32_iloop + (CCA_LEN * 6)], 0);
        v40_0 = vsqax_v32_v32 (8, v32_0, 0);

        /*combine Region#5*/
        v40_2 = vadd40_v40_v40_v40_out (8, v40_0, v40_1);
        vstdw_v32_concat (8, v40_2, 0, &s_s32_comb_squa[s32_iloop + (CCA_LEN*5)]);

        s32_iloop += 16;
    }
    #else
    for(s32_iloop = 0; s32_iloop < CCA_LEN; )
    {
        v32_0 = vlddw_v32 (8, &s_st_fft_out[s32_iloop], 0);
        v40_0 = vsqax_v32_v32 (8, v32_0, 0);
        /*combine Region#0*/
        vstdw_v32_concat (8, v40_0, 0, &s_s32_comb_squa[s32_iloop]);

    
        v32_1 = vlddw_v32 (8, &s_st_fft_out[s32_iloop + CCA_LEN], 0);
        v40_1 = vsqax_v32_v32 (8, v32_1, 0);
        /*combine Region#1*/
        vstdw_v32_concat (8, v40_1, 0, &s_s32_comb_squa[s32_iloop + CCA_LEN]);


        v32_2 = vlddw_v32 (8, &s_st_fft_out[s32_iloop + (CCA_LEN<<1)], 0);
        v40_2 = vsqax_v32_v32 (8, v32_2, 0);
        /*combine Region#2*/
        vstdw_v32_concat (8, v40_2, 0, &s_s32_comb_squa[s32_iloop + (CCA_LEN<<1)]);




        v32_3 = vlddw_v32 (8, &s_st_fft_out[s32_iloop + (CCA_LEN*3)], 0);
        v40_3 = vsqax_v32_v32 (8, v32_3, 0);
        /*combine Region#3*/
        vstdw_v32_concat (8, v40_3, 0, &s_s32_comb_squa[s32_iloop + (CCA_LEN * 3)]);



        v32_0 = vlddw_v32 (8, &s_st_fft_out[s32_iloop + (CCA_LEN<<2)], 0);
        v40_0 = vsqax_v32_v32 (8, v32_0, 0);
        /*combine Region#4*/
        vstdw_v32_concat (8, v40_0, 0, &s_s32_comb_squa[s32_iloop + (CCA_LEN<<2)]);


        v32_1 = vlddw_v32 (8, &s_st_fft_out[s32_iloop + (CCA_LEN*5)], 0);
        v40_1 = vsqax_v32_v32 (8, v32_1, 0);
        /*combine Region#5*/
        vstdw_v32_concat (8, v40_1, 0, &s_s32_comb_squa[s32_iloop + (CCA_LEN*5)]);

        s32_iloop += 16;
    }

    #endif

    _ks_max_s32v_idx(&s_s32_comb_squa[0], &s32_peak_index, (CCA_LEN*6));

	s32_peak_index += u32_start_pos;

	KS_LOG(0, 0xA113, (UINT16)s32_peak_index);

    v32_0 = vlddw_v32 (1, &s_st_fft_out[s32_peak_index], 0);
    v32_1 = vsqax_v32_v32 (1, v32_0, 0);
    //vstdw_v32_concat (1, v32_1, 0, &s32_peak_value[0]);
    vstdw_v32_vuX (1, 0, v32_1, 0, &s32_peak_value[0]);

    v32_0 = vlddw_v32 (1, &s_st_fft_out[s32_peak_index + CCA_LEN], 0);
    v32_1 = vsqax_v32_v32 (1, v32_0, 0);
    //vstdw_v32_concat (1, v32_1, 0, &s32_peak_value[1]);
    vstdw_v32_vuX (1, 0, v32_1, 0, &s32_peak_value[1]);
    
    s32_peak_max = (s32_peak_value[0] >= s32_peak_value[1]) ? s32_peak_value[0] : s32_peak_value[1];
    s32_peak_min = (s32_peak_value[0] + s32_peak_value[1]) - s32_peak_max;

#if 1
	KS_LOG(0, 0xA114, (UINT16)(s32_peak_max >> 12));
	//KS_LOG(0, 0xA114, (UINT16)(s32_peak_max));
	KS_LOG(0, 0xA214, (UINT16)(s32_peak_min >> 12));
	//KS_LOG(0, 0xA214, (UINT16)(s32_peak_min)); 

	//KS_LOG(0, 0xA114, (UINT16)(s_st_fft_out[s32_peak_index].s16_re));
	//KS_LOG(0, 0xA114, (UINT16)(s_st_fft_out[s32_peak_index].s16_im));
	//KS_LOG(0, 0xA114, (UINT16)(s_st_fft_out[s32_peak_index + CCA_LEN].s16_re));
	//KS_LOG(0, 0xA114, (UINT16)(s_st_fft_out[s32_peak_index + CCA_LEN].s16_im));
		//OAL_ASSERT((s32_noise_value NEQ 0), "");
#endif

    //s32_peak_div = (s32_peak_value[0] > s32_peak_value[1]) ? (s32_peak_value[0]/s32_peak_value[1]) : (s32_peak_value[1]/s32_peak_value[0]);
	//ks_log(0, 0xa112, (UINT16)s32_peak_div);
	//while(READ_REG32(0x1724ffe0) NEQ 2);

    //if(((s32_peak_min * 6) < s32_peak_max) || (s32_peak_min < s_s32_cca_power_thres[s_s16_shift_value[u32_ant_id]]))
    //u32_thres = READ_REG(0x1724fff4);
    if(((s32_peak_min * 6) < s32_peak_max) || (s32_peak_min < (s8_pwr_thres << 16)))
    {
        return OAL_FALSE;
    }

    /*calc noise*/
    s32_noise_pos = s32_peak_index + 64 - u32_start_pos;
    if(s32_noise_pos + (CCA_LEN>>1) >= ICS_CORR_LEN)
    {
    	s32_noise_pos -= CCA_LEN;
    }
    s32p_temp = (SINT32_PTR)&s_s32_comb_squa[s32_noise_pos];
    s32_shift = -7;
    vcoef_shift = vlddw_c32_clone_1dw (&s32_shift);
    v40_shift = vfillw_c32_v40 (8, vcoef_shift);
    v40_0 = vclr_v32 (8);

    /*calc 128 samples bwtween with peaks*/
    for(s32_iloop = 0; s32_iloop < (CCA_LEN>>1); )
    {
        v32_0 = vlddw_v32 (8, s32p_temp + s32_iloop, 0);
        v40_0 = vadd40_v40_v32_v40 (8, v40_0, v32_0);
        s32_iloop += 16;
    }

    v40_1 = vaddint_v32_v40 (8, v40_0);
    v32_0 = vshiftx40_v40_v40_v32_w_us (1, v40_1, v40_shift, 0);
    v32_1 = vvmov_v40_v32_vuX (1, v32_0);
    v32_2 = vadd32_v32_v32_v32 (1, v32_1, v32_0);

    vstdw_v32_vuX (1, 0, v32_2, 0, &s32_noise_value);
	KS_LOG(0, 0xA115, (UINT16)(s32_noise_value >> 12));


    //if((s32_peak_min > (s32_noise_value * 6)) || ((s_s16_shift_value[u32_ant_id] >= 10) && (s32_peak_min > (s32_noise_value << 3))))
    if(s32_peak_min > (s32_noise_value * s8_pwr_thres))
    {
		#if 0
		if(s32_noise_value < 16)
		{
			if(s32_peak_min < 1500)
			{
				return OAL_FALSE;
			}
		}
		#endif
        /*cca verify*/
		s32p_temp = (SINT32_PTR)(s32p_rx_buf + s32_peak_index);
        #if 0
        {
        	#if 1
            v40_0 = vclr_v32 (8);
            v40_1 = vclr_v32 (8);
            v40_2 = vclr_v32 (8);
            v40_3 = vclr_v32 (8);

            s32_shift = -8;
            vcoef_shift = vlddw_c32_clone_1dw (&s32_shift);
            v40_shift = vfillw_c32_v40 (8, vcoef_shift);

            //s32p_temp = (SINT32_PTR)&s_st_rx_buffer[s_u32_pingpong_idx][AGC_LENGTH + s32_peak_index];
			

            for(s32_iloop = 0; s32_iloop < CCA_LEN; )
            {
                v32_0 = vlddw_v32 (8, s32p_temp + s32_iloop, 0);
                v32_1 = vlddw_v32 (8, s32p_temp + s32_iloop + CCA_LEN, 0);

                v40_0 = vmacx_v32_v32_v40_v40_conj (8, v32_0, 0, v32_1, 0, v40_0, v40_1);
                v40_2 = vmacx_v32_v32_v40_v40 (8, v32_0, 0, v32_1, 0, v40_2, v40_3);

                s32_iloop += 16;
            }

            v40_0 = vadd40_v40_v40_v40_out (8, v40_0, v40_1);
            v40_2 = vadd40_v40_v40_v40_out (8, v40_2, v40_3);

            v32_0 = vshiftx40_v40_v40_v32_w_s (4, v40_0, v40_shift, 0);
            v32_2 = vshiftx40_v40_v40_v32_w_s (4, v40_2, v40_shift, 0);

            v32_1 = vaddintx_v32_v32 (4, v32_0);
            v32_3 = vaddintx_v32_v32 (4, v32_2);

            v32_0 = vvmov_v40_v32_vuX (1, v32_1);
            v32_2 = vvmov_v40_v32_vuX (1, v32_3);

            v32_0 = vadd32_v32_v32_v32 (2, v32_0, v32_1);
            v32_2 = vadd32_v32_v32_v32 (2, v32_2, v32_3);
            //v32_0 = vlddw_v32 (1, &s_st_fft_out[s32_peak_index], 0);
            //v32_1 = vsqax_v32_v32 (1, v32_0, 0);
            //vstdw_v32_concat (1, v32_1, 0, &s32_peak_value[0]);
			vstdw_v32_vuX (2, 0, v32_0, 0, &s32_mac[0]);
			
			s32_peak_value[0]=((SINT64)s32_mac[0])*((SINT64)s32_mac[0])+((SINT64)s32_mac[1])*((SINT64)s32_mac[1]);



            //v32_0 = vlddw_v32 (1, &s_st_fft_out[s32_peak_index + CCA_LEN], 0);
            //v32_1 = vsqax_v32_v32 (1, v32_0, 0);
            //vstdw_v32_concat (1, v32_1, 0, &s32_peak_value[1]);
			vstdw_v32_vuX (2, 0, v32_2, 0, &s32_mac[0]);
			s32_peak_value[1]=((SINT64)s32_mac[0])*((SINT64)s32_mac[0])+((SINT64)s32_mac[1])*((SINT64)s32_mac[1]);
			#endif

        }

		//s32p_temp = (SINT32_PTR)s_st_fft_out + u32_start_pos + s32_peak_index;
		
		s32p_temp = (SINT32_PTR)(s32p_rx_buf + s32_peak_index);
        if(s32_peak_value[0] >= (s32_peak_value[1]<<1))
        #else
		//if(dl_cca_recheck(s32p_temp, s32p_temp + CCA_LEN , CCA_LEN, u32_ant_id))
        #endif
        {
			#if CCA_FIRST_PATH
			s32_peak_max >>= 2;
			vcoef_value = vlddw_c32_clone_1dw (&s32_peak_max);

			for(s32_iloop = s32_peak_index - MULTI_PATH_WINDOW_LENGTH + 1; s32_iloop < s32_peak_index;)
			{
				v32_1 = vlddw_v32 (8, &s_s32_comb_squa[s32_iloop], 0);
				#if 0
				v32_2 = vlddw_v32 (8, &s_s32_comb_squa[s32_iloop + M_SEQ_LENGHT], 0);

				v40_4 = vadd40_v40_v40_v40_out (8, v32_1, v32_2);
				v32_1 = vshift_v32_imm6_v32_ar (8, v40_4, -1);
				#endif
				v_prex = vcmp_v32_c32_vpr_gt (8, v32_1, vcoef_value);

				vstdw_vpr_dw_vuX (0, v_prex, &u32_temp[0]);
				vstdw_vpr_dw_vuX (1, v_prex, &u32_temp[1]);

				u32_temp[0] = u32_temp[0] | u32_temp[1]<<8;
				u32_temp[0] = bitrev_rN (u32_temp[0]);

				u32_first_path = exp_acW_acZ_zero (u32_temp[0]>>2);

				if(0 != u32_first_path)
				{
					if(((s_s32_comb_squa[s32_peak_index + MULTI_PATH_WINDOW_LENGTH - u32_first_path]/13) > s32_noise_value) && ((s_s32_comb_squa[s32_peak_index + M_SEQ_LENGHT + MULTI_PATH_WINDOW_LENGTH - u32_first_path]/13) > s32_noise_value))
					{
						break;
					}
				}
				s32_iloop += 16;
			}
			u32_first_path = MULTI_PATH_WINDOW_LENGTH - u32_first_path - 1;
			KS_LOG(0, 0xA215, (UINT16)(u32_first_path));
        	#endif
        	
			g_u32_ics_frm_cnt_cca = g_u32_ics_frm_cnt;
			s_s32_frame_num_cca = s_s32_frame_num;
    		s_s32_sync_position_prev = s_s32_sync_position[u32_ant_id];
			s_s32_sync_position[u32_ant_id] = s32_peak_index + u32_fft_cnt * ICS_CORR_LEN - u32_offset;
    		s_s32_sync_position[u32_ant_id] -= (ICS_RSV_LEN + TA_LEN + u32_first_path);
			//s_s32_sync_position[u32_ant_id] = 380;

			//recalc rssi shift value for downlink
			//s_s16_shift_value = dl_scale_calc(s32p_temp, (CCA_LEN << 1));
			
			//if(s_s32_sync_position[u32_ant_id] >= CCA_POS_FULL_FRAME)
			{
				//not rx full frame, need calc freq offset by cca
				//s_s32_angle_cca[u32_ant_id] = dl_freq_offset_est_cca((&s_st_fft_out[s32_peak_index]), (&s_st_fft_out[s32_peak_index + CCA_LEN]), CCA_LEN, OAL_TRUE);
				#if FUN_DC_REMOVE
    			_ks_cxv_add_cx(s32p_temp, (CCA_LEN << 1), &s_st_dc[u32_ant_id], (SINT32_PTR)&s_st_temp_buffer[0][0]);	
				//s_s32_angle_cca[u32_ant_id] = dl_freq_offset_est((SINT32_PTR)&s_st_temp_buffer[0][0], (SINT32_PTR)&s_st_temp_buffer[0][CCA_LEN], CCA_LEN, CCA_LEN, 0, u32_ant_id, (BOOLEAN)OAL_FALSE);
				s_s32_angle_cca[u32_ant_id] = dl_freq_offset_est((SINT32_PTR)&s_st_temp_buffer[0][CCA_DELTA_LEN], \
						(SINT32_PTR)&s_st_temp_buffer[0][CCA_LEN + CCA_DELTA_LEN], CCA_LEN - (CCA_DELTA_LEN), CCA_LEN, 0, u32_ant_id, (BOOLEAN)OAL_FALSE);
				#else
				s_s32_angle_cca[u32_ant_id] = dl_freq_offset_est(s32p_temp, s32p_temp + CCA_LEN, CCA_LEN, CCA_LEN, 0, u32_ant_id, (BOOLEAN)OAL_FALSE);
				#endif
				//s_s32_angle_cca[u32_ant_id] = dl_freq_offset_est(s32p_temp - 808, s32p_temp - 408, 400, \
				//		400, 0, u32_ant_id, (BOOLEAN)OAL_FALSE);
			}
			g_u32_rx_ant_num += 1;
			s_s32_peak_min[u32_ant_id] = s32_peak_min; //for debug
			return (1 << u32_ant_id);
        }

	}
	else
	{
		return OAL_FALSE;
	}
}

#define DC_THRES  (200)
OAL_ITCM_CODE_SECTION
VOID dl_dc_calc(IN SINT32_PTR s32p_temp, IN UINT32 u32_input_len, OUT complex_s16_t *stp_dc) 
{
	ks_mean_calc(s32p_temp , u32_input_len, stp_dc);
	stp_dc->s16_re = -stp_dc->s16_re;
	stp_dc->s16_im = -stp_dc->s16_im;

	KS_LOG(0, 0xA116, (UINT16)stp_dc->s16_re);
	KS_LOG(0, 0xA116, (UINT16)stp_dc->s16_im);
	#if 1
	if((stp_dc->s16_re < DC_THRES) && (stp_dc->s16_re > -DC_THRES))
	{
		stp_dc->s16_re = 0;
	}
	if((stp_dc->s16_im < DC_THRES) && (stp_dc->s16_im > -DC_THRES))
	{
		stp_dc->s16_im = 0;
	}	
	#endif

}
OAL_ITCM_CODE_SECTION
VOID dl_rssi_meas(IN SINT32_PTR s32p_in, IN UINT32 u32_input_len) 
{
    UINT32 u32_delta_of_ants = DELTA_OF_ANT_BUF;
    UINT32 u32_ant_id = 0;
    SINT16 s16_shift_value = 0;
	complex_s16_t st_dc_local;
    ks_fft_para_t st_fft_para;
	SINT32 s32_shift_amt_fft[5]  = {(UINT32)MODV_SV2, (UINT32)MODV_SV2, (UINT32)MODV_SV2, (UINT32)MODV_SV2, (UINT32)MODV_SV0};
	SINT32 s32_rssi[MAX_RX_ANT_NUM];
	SINT32_PTR s32p_temp = s32p_in;

	st_fft_para.s16cp_indata_addr   = s_st_fft_in;
	st_fft_para.s16cp_outdata_addr  = s_st_fft_out;
	st_fft_para.s16cp_twiddle_addr  = (complex_s16_t*)ceva_fft_lib_fft_2048_twi_table;
	st_fft_para.s32p_shift_amt_addr = s32_shift_amt_fft;
	st_fft_para.s16cp_temp_addr     = (complex_s16_t*)(s_st_temp_buffer[0]);
	
	for(u32_ant_id = 0; u32_ant_id < MAX_RX_ANT_NUM; u32_ant_id++)
	{
		s32p_temp = s32p_in + u32_delta_of_ants * u32_ant_id;
		ks_mean_calc(s32p_temp, (UINT16)u32_input_len, &st_dc_local);
		st_dc_local.s16_re = -st_dc_local.s16_re;
		st_dc_local.s16_im = -st_dc_local.s16_im;

		KS_LOG(0, 0xA216, (UINT16)st_dc_local.s16_re);
		KS_LOG(0, 0xA216, (UINT16)st_dc_local.s16_im);

	    _ks_cxv_add_cx(s32p_temp, u32_input_len, &st_dc_local, &s_st_temp_buffer[u32_ant_id][0]);	
		s16_shift_value = dl_scale_calc(&s_st_temp_buffer[u32_ant_id][0], u32_input_len);			
		KS_LOG(0, 0xA31C, (UINT16)s16_shift_value);


		s32_rssi[u32_ant_id] = ks_cxv_rssi_calc(&s_st_temp_buffer[u32_ant_id][0], u32_input_len);
		#ifdef KD001_RF8242
		//temp here, do nothing
		#else
		s32_rssi[u32_ant_id] = ((10*(ks_logx(10, s32_rssi[u32_ant_id])>>8))>>18) - 105 - (SINT8)(g_zzw_ch_status.gain);
		#endif
		//ks_fft2048(&st_fft_para);
		KS_LOG(0, 0xA31D, (UINT16)s32_rssi[u32_ant_id]);
	}


}

OAL_ITCM_CODE_SECTION
UINT32 dl_cca_proc_sub(IN SINT32_PTR s32p_temp, IN UINT32 u32_input_len, IN UINT32 u32_offset, IN UINT32 u32_fft_cnt) 
{
    UINT32 u32_j;
    UINT32 u32_delta_of_ants = DELTA_OF_ANT_BUF;
    UINT32 u32_cca_detected = 0; 
    SINT8 s8_pwr_thres = 5;//READ_REG32(0x1724fff4);

    complex_s16_t st_dc_temp = {0, 0};
	//complex_s16_t cpx_mean;
	node_info_t* stp_node_info;
	stp_node_info = (node_info_t*)node_tbl_find(g_u8_node_id);
	
	if(stp_node_info)
	{
		s8_pwr_thres += (stp_node_info->s8_snr >> 3);
	}
	
	for(u32_j = 0; u32_j < RX_ANT_NUM; u32_j++)
	//for(u32_j = 0; u32_j < 1; u32_j++)
	{

		if(u32_input_len > ICS_RSV_LEN)
		{
			//dc calc
			#if 0
			ks_mean_calc(s32p_temp + ICS_RSV_LEN + u32_delta_of_ants * u32_j, u32_input_len - ICS_RSV_LEN, &st_dc_temp);
			st_dc_temp.s16_re = -st_dc_temp.s16_re;
			st_dc_temp.s16_im = -st_dc_temp.s16_im;
			if(u32_fft_cnt EQ 0)
			{
				s_st_dc[u32_j].s16_re = st_dc_temp.s16_re;
				s_st_dc[u32_j].s16_im = st_dc_temp.s16_im;				
			}
			else
			{
				s_st_dc[u32_j].s16_re = (s_st_dc[u32_j].s16_re + st_dc_temp.s16_re) >> 1;
				s_st_dc[u32_j].s16_im = (s_st_dc[u32_j].s16_im + st_dc_temp.s16_im) >> 1;
				
			}
			KS_LOG(0, 0xA116, (UINT16)s_st_dc[u32_j].s16_re);
			KS_LOG(0, 0xA116, (UINT16)s_st_dc[u32_j].s16_im);
			#else
			dl_dc_calc(s32p_temp + ICS_RSV_LEN + u32_delta_of_ants * u32_j, u32_input_len - ICS_RSV_LEN, &s_st_dc[u32_j]);
			#endif
			//dc remove
			#if 1//FUN_DC_REMOVE
	        _ks_cxv_add_cx(s32p_temp + ICS_RSV_LEN + u32_delta_of_ants * u32_j, u32_input_len - ICS_RSV_LEN, &s_st_dc[u32_j],\
	        		&s_st_temp_buffer[0][0]);	
			s_s16_shift_value[u32_j] = dl_scale_calc(&s_st_temp_buffer[0][0], u32_input_len - ICS_RSV_LEN);			
	        #else
			s_s16_shift_value[u32_j] = dl_scale_calc(s32p_temp + ICS_RSV_LEN + u32_delta_of_ants * u32_j, u32_input_len - ICS_RSV_LEN);
			#endif
		}
		#if 0
		else
		{
			//dc remove, use last dc
	        _ks_cxv_add_cx(s32p_temp + u32_delta_of_ants * u32_j, u32_input_len, &s_st_dc[u32_j],\
	        		s32p_temp + u32_delta_of_ants * u32_j);			
		}
		#endif
		KS_LOG(0, 0xA11C, (UINT16)((s_s16_shift_value[u32_j] << 12) | (u32_j << 8) | (u32_fft_cnt)));
		//KS_LOG(0, 0xA22C, (UINT16)((s_s16_shift_value)));
		if(s_s16_shift_value[u32_j] <= 10)
		{
			//s_s16_shift_value[u32_j] = 0; 

			u32_cca_detected |= dl_cca_detect(s32p_temp + u32_delta_of_ants * u32_j, u32_input_len, u32_offset, u32_fft_cnt, u32_j, s8_pwr_thres);
		}
	}
	return u32_cca_detected;
}

#define CCA_SEARCH_WIN_LEN         (100)
OAL_ITCM_CODE_SECTION
UINT32 dl_cca_proc(SINT32_PTR s32p_in, UINT32 u32_offset, UINT32 u32_fft_last, UINT32 *u32p_fft_cnt) 
{
    UINT32 u32_i;
    UINT32 u32_j;    
    UINT32 u32_cca_detected = 0; 
    UINT32 u32_fft_cnt = *u32p_fft_cnt;
    UINT32 u32_input_len = SYMBOL_LEN;
    UINT32 u32_delta_of_ants = DELTA_OF_ANT_BUF;
    SINT32 s32_seg_start = 0;
	complex_s16_t cpx_mean;

    SINT32_PTR s32p_temp;// = s32p_in + ICS_CORR_LEN * u32_fft_cnt - ICS_RSV_LEN;
    //SINT32_PTR s32p_temp_org = (SINT32_PTR)&s_st_rx_buffer[s_u32_pingpong_idx][0];
	node_info_t* stp_node_info;

	
	g_u32_rx_ant_num = 0;
	s32p_in -= (ICS_RSV_LEN + u32_offset);
	s32p_temp = s32p_in + ICS_CORR_LEN * u32_fft_cnt;

	if(u32_fft_cnt > u32_fft_last)
	{
		*u32p_fft_cnt = 0;
		return OAL_FALSE;
	}
	else if(u32_fft_cnt EQ 0)
	{
		if(s_u8_zzw_state EQ ZZW_STATE_NET)
		{
			stp_node_info = (node_info_t*)node_tbl_find(g_u8_node_id);
			if(stp_node_info)
			{
				s32_seg_start = ((stp_node_info->s32_cca_pos + ICS_RSV_LEN - CCA_SEARCH_WIN_LEN) / ICS_CORR_LEN);
				if(s32_seg_start < u32_fft_last)
				{
					u32_fft_cnt = s32_seg_start;
					s32p_temp = s32p_in + ICS_CORR_LEN * u32_fft_cnt;
				}
			}
		}
		if(u32_fft_cnt EQ 0)
		{
			s32p_temp = (SINT32_PTR)&s_st_rx_buffer.st_ics_left[0];
			for(u32_i = 0; u32_i < RX_ANT_NUM; u32_i++)
			{
				ks_oal_mem_copy((VOID_PTR)(s_st_rx_buffer.st_ics_left + ICS_RSV_LEN + u32_offset + DELTA_OF_ANT_BUF * u32_i), \
							(VOID_PTR)(s32p_in + ICS_RSV_LEN + u32_offset + DELTA_OF_ANT_BUF * u32_i), \
							((ICS_FFT_LEN - (ICS_RSV_LEN + u32_offset)) << 2));
			}
		}
	}
	KS_LOG(0, 0xA112, (u32_fft_cnt << 8) | u32_fft_last); 


	for(u32_i = u32_fft_cnt; u32_i < (u32_fft_last); u32_i++)
	{
		#if 1
		if((u32_i EQ ICS_SEG_NUM))
		{
			u32_input_len = (ICS_RSV_LEN + u32_offset);
		}
		#endif

		#if 0
		for(u32_j = 0; u32_j < RX_ANT_NUM; u32_j++)
		{
			if(u32_input_len <= ICS_RSV_LEN)
			{
				s_s16_shift_value[u32_j] = s_s16_shift_value[u32_j];
			}
			else
			{
				s_s16_shift_value[u32_j] = dl_scale_calc(s32p_temp + ICS_RSV_LEN + u32_delta_of_ants * u32_j, u32_input_len - ICS_RSV_LEN);			
			}
			KS_LOG(0, 0xA11C, (UINT16)((s_s16_shift_value[u32_j] << 12) | (u32_j << 8) | (u32_i)));
			//KS_LOG(0, 0xA22C, (UINT16)((s_s16_shift_value)));
			if(s_s16_shift_value[u32_j] < 10)
			{
				//s_s16_shift_value[u32_j] = 0; 
				ks_mean_calc(s32p_temp + ICS_RSV_LEN + u32_delta_of_ants * u32_j, u32_input_len - ICS_RSV_LEN, &cpx_mean);
				KS_LOG(0, 0xA116, (UINT16)cpx_mean.s16_re);
				KS_LOG(0, 0xA116, (UINT16)cpx_mean.s16_im);
				u32_cca_detected |= dl_cca_detect(s32p_temp + u32_delta_of_ants * u32_j, u32_input_len, u32_offset, u32_i, u32_j);
			}
		}
		#else
		u32_cca_detected |= dl_cca_proc_sub(s32p_temp, u32_input_len, u32_offset, u32_i);
		#endif
		//u32_fft_cnt++;
		s32p_temp = s32p_in + ICS_CORR_LEN * (u32_i + 1);
		
		if(u32_cca_detected)
		{
			u32_i += 1;
			break;
		}
	}
	u32_fft_cnt = u32_i;
	#if 0	
	//last seg in one frame
	if(!u32_cca_detected)
	{
		u32_cca_detected = dl_cca_detect(s32p_temp, ICS_SEG_REM, OAL_TRUE);
		g_u32_ics_fft_cnt++;	

		if(!u32_cca_detected)
		{
			ks_oal_mem_copy((VOID_PTR)(&s_st_ics_left[0]), (VOID_PTR)(s32p_temp), (CCA_LEN<<2));
			KS_LOG(0, 0xA112, 0xDEAD); //NO CCA DETECTED IN CURR FRAME
			g_u32_ics_fft_cnt = 0;// reset fft cnt here
			
			//OAL_ASSERT(0, "");
		}
	}
	#else
	if(!u32_cca_detected)
	{
		OAL_ASSERT(u32_fft_cnt EQ u32_fft_last, "");
		//ks_oal_mem_copy((VOID_PTR)(&s_st_ics_left[0]), (VOID_PTR)(s32p_temp), (CCA_LEN<<2));
		KS_LOG(0, 0xA112, 0xDEAD); //NO CCA DETECTED IN CURR FRAME
		u32_fft_cnt = 0;// reset fft cnt here

		if(READ_REG32(ASSERT_4500_ADDR) EQ RX_DUMP_CCA_MIS)
		{
			OAL_ASSERT(0, "");
		}
	}
	if(s_u8_zzw_state EQ ZZW_STATE_SCN)
	{
		for(u32_i = 0; u32_i < RX_ANT_NUM; u32_i++)
		{
			ks_oal_mem_copy((VOID_PTR)(s_st_rx_buffer.st_ics_left + DELTA_OF_ANT_BUF * u32_i), \
						(VOID_PTR)(s32p_in + ICS_CORR_LEN * (ICS_SEG_NUM) + DELTA_OF_ANT_BUF * u32_i),\
						((ICS_RSV_LEN + u32_offset) << 2));
		}
	}
	else
	{
		for(u32_i = 0; u32_i < RX_ANT_NUM; u32_i++)
		{
			ks_oal_mem_set((VOID_PTR)(s_st_rx_buffer.st_ics_left + DELTA_OF_ANT_BUF * u32_i), 0, \
						((ICS_RSV_LEN + u32_offset) << 2));
		}
	}
	#endif
	*u32p_fft_cnt = u32_fft_cnt;
	return u32_cca_detected;
}

OAL_ITCM_CODE_SECTION
UINT32 dl_sync_proc_pre()
{
	UINT32 u32_ret = 0; //16 ~ 31 bits: fft cnt ; 0 ~ 15 bits: do CCA DETECTING or not( only used for ZZW_STATE_SCN state)
	
	if(s_u8_zzw_state EQ ZZW_STATE_NET)
	{
		if(g_u32_ics_flag EQ ICS_CCA_DETECT)
		{
			//mips optimization, for CCA_POS_IDEAL is (1220 + 120), in 2nd fft seg, so CCA DETECTING starts from 2nd seg to reduce mips consumption
			#if 0
			if(g_u32_ics_fft_cnt EQ 2)
			{
				g_u32_ics_fft_cnt -= 1;
			}
			else
			#endif
			{
				g_u32_ics_fft_cnt = 0;
			}
			
			u32_ret = ((5 << 16) | 1);
		}
	}
	else if(s_u8_zzw_state EQ ZZW_STATE_SCN)
	{
		if((g_u32_ics_frm_cnt & 0x7FF) EQ 0)
		{
			//every 2048 frames, add 1
			g_u32_ics_sfrm_cnt += 1;
		}
		#if 1//ICS_OPTIMIZE
		if(s_u32_ics_optimize)
		{
			//in every 2048 frames, 0 ~ 2047 frame, proc even frames; 2048 ~ 4095 frame, proc odd frames....
			if((((g_u32_ics_frm_cnt >> 6) & 0x1) && ((g_u32_ics_sfrm_cnt & 0x1))) || \
				((((g_u32_ics_frm_cnt >> 6) & 0x1) EQ 0) && (((g_u32_ics_sfrm_cnt & 0x1) EQ 0))))
			{
				//second 1024 frame, proc odd frames
				if(g_u32_ics_frm_cnt & 0x1)
				{
					if(g_u32_ics_flag EQ ICS_CCA_DETECT)
					{
						g_u32_ics_fft_cnt = 0;
					}
					u32_ret = (((ICS_SEG_NUM + 1) << 16) | 1);
					//KS_LOG(0, 0xA211, 3);
				}
				else
				{
					u32_ret = 0;
					//KS_LOG(0, 0xA211, 4);
				}
			}
			else
			{
				//first 1024 frame, proc even frames
				if((g_u32_ics_frm_cnt & 0x1) EQ 0)
				{
					if(g_u32_ics_flag EQ ICS_CCA_DETECT)
					{
						g_u32_ics_fft_cnt = 0;
					}
					u32_ret = (((ICS_SEG_NUM + 1) << 16) | 1);		
					
					//KS_LOG(0, 0xA211, 1);
				}
				else
				{
					u32_ret = 0;
					//KS_LOG(0, 0xA211, 2);
					
				}					
			}
		}
		//#else
		else
		{
			if(g_u32_ics_flag EQ ICS_CCA_DETECT)
			{
				g_u32_ics_fft_cnt = 0;
			}
			u32_ret = (((ICS_SEG_NUM) << 16) | 1);		

		}
		#endif
	}
	else
	{
		if(g_u32_ics_flag EQ ICS_CCA_DETECT)
		{
			g_u32_ics_fft_cnt = 0;
			u32_ret = (((ICS_SEG_NUM) << 16) | 1);		
		}
	}
	
	#if RTC_ADJ
	if(s_u8_node_id_local NEQ 1)
	{
		if(s_u8_zzw_state EQ ZZW_STATE_NET)
		{
			if((g_u32_ics_frm_cnt > 10) && (s_u32_rtc_adj_wait_cnt EQ 0))
			{
				//ADJ RTC by DSP
				s_u32_rtc_adj_flg = OAL_TRUE;
			}
		}
	}
	#endif
	return u32_ret;
}



OAL_ITCM_CODE_SECTION
UINT32 dl_sync_proc()
{
    UINT32 u32_pre_ret = 0;
    UINT32 u32_cca_detected = OAL_FALSE;
    UINT32 u32_i = 0;
	SINT16 s16_shift_value_min = 20;
	SINT16 s16_shift_value_max = 0;	
	UINT8  u8_ssap = 0;
	UINT32 u32_local_fn = 0;
	SINT32 s32_local_fn_delta = 0;
	SINT32 s32_len_left = 0;
	SINT32 s32_gp_start = 0;

	node_info_t* stp_node_info;
	
    //SINT32_PTR s32p_in = (SINT32_PTR)&s_st_rx_buffer[s_u32_pingpong_idx][0];
    SINT32_PTR s32p_in = (SINT32_PTR)(s_st_rx_buffer.st_rx_buffer[s_u32_pingpong_idx]);
	sram_1643_4500_t *stp_sarm_1643_4500 = (sram_1643_4500_t*)(SRAM_1643_4500_ADDR);	//g_st_iq_dump

    //OAL_IRQ_SAVE_AREA;

	//OAL_IRQ_DISABLE_ALL;

	#if 0
	if(g_u32_ics_frm_cnt EQ 0)
	{
	    ks_oal_mem_set((VOID_PTR)s_st_ics_left, 0x0, ((SLOT_LEN) << 2));
	}
	#endif

	u8_ssap = stp_sarm_1643_4500->u8_ssap;
	if(s_u8_zzw_state NEQ s_u8_zzw_state_pre)
	{
		//state changed, need clear node tbl
		if(s_u8_zzw_state EQ ZZW_STATE_SCN)
		{
			s_u32_dl_scn_cnt += 1;
			s_u32_dl_sfb_cnt = 0;
			node_tbl_clear();
		}
		#if 0
		if(s_u32_dl_scn_cnt > 5)
		{
			s_u32_ics_optimize	= OAL_TRUE;	
		}
		else
		{
			s_u32_ics_optimize	= OAL_FALSE; 
		}
		#endif
	}
	OAL_ASSERT(s_u8_zzw_state <= ZZW_STATE_NET, "");			
	#ifdef TX_TEST
	//s_u32_zzw_state = ZZW_STATE_SCN;
	#endif
	u32_local_fn = (g_u32_ics_frm_cnt + s_s32_frame_num);
	KS_LOG(0, 0xA120, (UINT16)((s_u8_zzw_state << 12) | (g_u32_ics_flag << 8) | (s_u8_rx_gain + s_u32_pingpong_idx)));
	KS_LOG(0, 0xA121, (UINT16)u32_local_fn); 
	//KS_LOG(0, 0xA121, (UINT16)(READ_REG32(0x1724FF08)));
	if(s_u8_zzw_state > ZZW_STATE_SCN)
	{
		g_u8_node_id = node_slot_alloc_query(u8_ssap, u32_local_fn);
		//KS_LOG(0, 0xA124, (UINT16)g_u8_node_id); 

		stp_node_info = (node_info_t*)node_tbl_find(g_u8_node_id);
		if(stp_node_info)
		{
			s32_local_fn_delta = (u32_local_fn - stp_node_info->u32_rx_fn) & 0x7FF;
			if(s32_local_fn_delta > 0x400)
			{
				s32_local_fn_delta += 0x800;
			}
			if((s32_local_fn_delta > 100))
			{
				//tmp here, 
				node_tbl_del(g_u8_node_id);
			}
		}
		
	}
	else
	{
		g_u8_node_id = 0;
	}

	u32_pre_ret = dl_sync_proc_pre();

	switch(g_u32_ics_flag)
	{
		case ICS_CCA_DETECT:
		{
			g_u32_ics_cca_cnt = 0;
			if(u32_pre_ret & 0x1)
			{
				g_u32_cca_detected = dl_cca_proc(s32p_in, 0, (u32_pre_ret >> 16), &g_u32_ics_fft_cnt);
				
				if(g_u32_cca_detected)
				{
					dl_head_dec_pre(s32p_in);
					//if(s_u8_zzw_state EQ ZZW_STATE_NET)
					{
						s32_len_left = FRAME_LEN - (s_s32_sync_pos_min + (CCA_LEN << 1) + TOTAL_SYM_NUM * SLOT_LEN) - 100;
						if(s32_len_left > 200) //temp here
						{
							s32_gp_start = FRAME_LEN - s32_len_left;
							//do measurement in gap
							if(s32_len_left > SYMBOL_LEN)
							{
								s32_len_left = SYMBOL_LEN;
							}
							dl_rssi_meas(s32p_in + s32_gp_start, s32_len_left);
						}
					}
				}
			}
			else
			{
				for(u32_i = 0; u32_i < RX_ANT_NUM; u32_i++)
				{
					ks_oal_mem_copy((VOID_PTR)(s_st_rx_buffer.st_ics_left + DELTA_OF_ANT_BUF * u32_i), \
								(VOID_PTR)(s32p_in + DELTA_OF_ANT_BUF * u32_i + RX_BUF_LEN - ICS_RSV_LEN ), ((ICS_RSV_LEN) << 2));
				}
			}
			break;
		}

		case ICS_HEAD_COLLECT:
		{
			//1st pilot proc in last frame
			g_u32_ics_flag = ICS_HEAD_DECODE;
			OAL_ASSERT((g_u32_ics_left_cnt < SLOT_LEN), "");
			for(u32_i = 0; u32_i < RX_ANT_NUM; u32_i++)
			{
				ks_oal_mem_copy(((UINT32_PTR)(s_st_rx_buffer.st_ics_left + DELTA_OF_ANT_BUF * u32_i) + g_u32_ics_left_cnt), \
							s32p_in + DELTA_OF_ANT_BUF * u32_i, (((SLOT_LEN) - g_u32_ics_left_cnt)<<2));
			}
			#ifdef MIMO2
			dl_head_decode(&s_st_rx_buffer.st_ics_left[0], 2);
			#else
			dl_head_decode(&s_st_rx_buffer.st_ics_left[0], 0);
			#endif
			g_u32_ics_left_cnt = RX_BUF_LEN - ((SLOT_LEN) - g_u32_ics_left_cnt);
			dl_head_dec_post(s32p_in, (UINT32)ICS_HEAD_COLLECT, (u32_pre_ret & 0x1));
			break;
		}
		case ICS_HEAD_COLLECT2:
		{
			//1st pilot proc in curr frame
			g_u32_ics_flag = ICS_HEAD_DECODE;
			OAL_ASSERT((g_u32_ics_left_cnt < SLOT_LEN), "");
			for(u32_i = 0; u32_i < RX_ANT_NUM; u32_i++)
			{
				ks_oal_mem_copy(((UINT32_PTR)(s_st_rx_buffer.st_ics_left + DELTA_OF_ANT_BUF * u32_i) + g_u32_ics_left_cnt), \
							s32p_in + DELTA_OF_ANT_BUF * u32_i, (((SLOT_LEN) - g_u32_ics_left_cnt)<<2));
			}
			#ifdef MIMO2
			dl_head_pilot_proc(s32p_in + (SLOT_LEN - g_u32_ics_left_cnt));
			dl_head_decode(&s_st_rx_buffer.st_ics_left[0], 0);
			#else
			dl_head_pilot_proc(&s_st_rx_buffer.st_ics_left[0]);
			dl_head_decode(s32p_in + (SLOT_LEN - g_u32_ics_left_cnt), 0);
			#endif
			//s32p_temp_org += (SLOT_LEN - g_u32_ics_left_cnt);
			g_u32_ics_left_cnt = RX_BUF_LEN - ((SLOT_LEN << 1) - g_u32_ics_left_cnt);
			dl_head_dec_post(s32p_in, (UINT32)ICS_HEAD_COLLECT2, (u32_pre_ret & 0x1));
			break;
		}
		case ICS_HEAD_COLLECT3:
		{
			//1st pilot proc in curr frame
			g_u32_ics_flag = ICS_HEAD_DECODE;
			dl_head_decode(s32p_in + g_u32_ics_left_cnt, 1);
			g_u32_ics_left_cnt = RX_BUF_LEN - ((SLOT_LEN << 1) + g_u32_ics_left_cnt);
			dl_head_dec_post(s32p_in, (UINT32)ICS_HEAD_COLLECT3, (u32_pre_ret & 0x1));
			break;
		}
		#if 0
		case ICS_HEAD_DECODE:
		{
			if(g_u32_ics_head_crc EQ OAL_SUCCESS)
			{
				g_u32_ics_flag = ICS_DATA_COLLECT;
			}
			else
			{
				g_u32_ics_flag = ICS_CCA_DETECT;				
			}
			break;
		}
		#endif

		case ICS_DATA_COLLECT:
		{
			OAL_ASSERT((g_u32_ics_left_cnt < SLOT_LEN), "");
			#if 0
			if(s_u8_zzw_state NEQ ZZW_STATE_SCN)
			{
				for(u32_i = 0; u32_i < RX_ANT_NUM; u32_i++)
				{
					//attention!!!!! dc re-calc to be done!!!!
					s_s16_shift_value_before[u32_i] = s_s16_shift_value[u32_i];
					s_s16_shift_value[u32_i] = dl_scale_calc(s32p_in + DELTA_OF_ANT_BUF * u32_i, ((SLOT_LEN) - g_u32_ics_left_cnt));
					KS_LOG(0, 0xA21C, (UINT16)(s_s16_shift_value[u32_i]));

					ks_oal_mem_copy(((UINT32_PTR)(s_st_rx_buffer.st_ics_left + DELTA_OF_ANT_BUF * u32_i) + g_u32_ics_left_cnt), \
								s32p_in + DELTA_OF_ANT_BUF * u32_i, (((SLOT_LEN) - g_u32_ics_left_cnt)<<2));

					if(s_s16_shift_value[u32_i] - s_s16_shift_value_before[u32_i])
					{
						_ks_shift_s16v((SINT16_PTR)((UINT32_PTR)(s_st_rx_buffer.st_ics_left + DELTA_OF_ANT_BUF * u32_i) + g_u32_ics_left_cnt), \
									(((SLOT_LEN) - g_u32_ics_left_cnt) << 1), (s_s16_shift_value[u32_i] - s_s16_shift_value_before[u32_i]), \
									(SINT16_PTR)((UINT32_PTR)(s_st_rx_buffer.st_ics_left + DELTA_OF_ANT_BUF * u32_i) + g_u32_ics_left_cnt));
					} 
								
				}
			}
			#else
			for(u32_i = 0; u32_i < RX_ANT_NUM; u32_i++)
			{
				ks_oal_mem_copy(((UINT32_PTR)(s_st_rx_buffer.st_ics_left + DELTA_OF_ANT_BUF * u32_i) + g_u32_ics_left_cnt), \
							s32p_in + DELTA_OF_ANT_BUF * u32_i, (((SLOT_LEN) - g_u32_ics_left_cnt)<<2));
			}
			#endif
			dl_data_dec_pre(&s_st_rx_buffer.st_ics_left[0], g_u32_ics_left_sym, g_u32_ics_left_sym);
			#if 1
			for(u32_i = 0; u32_i < RX_ANT_NUM; u32_i++)
			{
				if(g_u32_cca_detected & (1 << u32_i))
				{
					if(s_s16_shift_value[u32_i] < s16_shift_value_min)
					{
						s16_shift_value_min = s_s16_shift_value[u32_i];
					}
					if(s_s16_shift_value[u32_i] > s16_shift_value_max)
					{
						s16_shift_value_max = s_s16_shift_value[u32_i];
					}
				}
			}
			s_s16_shift_value_min = s16_shift_value_min;
			s_s16_shift_value_max = s16_shift_value_max;
			#endif
			if(g_u32_ics_left_sym < LAST_SYM_ID)
			{
				dl_data_dec_pre(s32p_in + (SLOT_LEN - g_u32_ics_left_cnt), g_u32_ics_left_sym + 1, LAST_SYM_ID);
			}

			//continue cca detect in curr frame
			//g_u32_ics_fft_cnt = ((SYM_ID_LAST - g_u32_ics_left_sym + 1) * SLOT_LEN - g_u32_ics_left_cnt + (CCA_LEN << 1)) / ICS_CORR_LEN;
			#if 1//ICS_OPTIMIZE
			if((s_u8_zzw_state NEQ ZZW_STATE_SCN) || ((s_u8_zzw_state EQ ZZW_STATE_SCN) && (u32_pre_ret & 0x1)))
			#endif
			{
				g_u32_ics_fft_cnt -= 1;
				g_u32_cca_detected = dl_cca_proc(s32p_in, 0, ICS_SEG_NUM, &g_u32_ics_fft_cnt);
				if(g_u32_cca_detected)
				{
					dl_head_dec_pre(s32p_in);
				}
			}
			break;
		}
	}

	#if 0
	if((s_u32_data_cb_start_cnt EQ s_st_plcp_info.u16_subframe_num) && (g_u32_ics_head_crc EQ OAL_SUCCESS))
	{
		s_u32_data_left_cnt = 0;
		s_u32_data_cb_start_cnt = 0;
		s_u32_data_sym_pre_cnt = 0;
		s_u32_data_sym_eq_cnt = 0;
		g_u32_ics_flag = ICS_CCA_DETECT;
		g_u32_ics_head_crc = OAL_CRC_FAILURE;
	}
	#endif

	g_u32_ics_frm_cnt++;

	//OAL_IRQ_RESTORE;

}



#if 0

OAL_ITCM_CODE_SECTION
VOID dl_proc_main()
{
    s_s16_shift_value = 0;
    s_s32_sync_position = OAL_FAILURE;
    s_u32_pingpong_idx ^= 1;

    dl_sync_detect();

    if(OAL_FAILURE == s_s32_sync_position)
    {
        dl_head_decode();

        dl_data_decode();
    }
}
#else

//VOID (*dl_proc_ptr[5])(void) = {NULL_PTR, dl_cca_det, NULL_PTR, dl_plcp_dec, dl_data_dec};

VOID dl_dec_info_calc()
{
    UINT32 u32_fill_bits = 0, u32_llr_shift = 0;
	UINT32 u32_i;
	UINT16 u16_cb_len_in = 0;
	UINT16 u16_cb_len_out = 0;

	for(u32_i = 0; u32_i < ZZW_MAX_MCS_NUM; u32_i++)
	{
		u16_cb_len_in = g_st_mcs_tbl[u32_i].u32_raw_len_cb;
		u16_cb_len_out = g_st_mcs_tbl[u32_i].u32_len_cb;
		
		u32_fill_bits = (32 - (((UINT32)u16_cb_len_in + 4) & 0x1F)) & 0x1F;
		
		if(u32_fill_bits > 16)
		{
			u32_llr_shift = ((((UINT32)u16_cb_len_in + 4 + 31)>>5)<<1) - 2;
		}
		else
		{
			u32_llr_shift = ((((UINT32)u16_cb_len_in + 4 + 31)>>5)<<1) - 1;
		}
		s_u32_llr_shift[u32_i] = u32_llr_shift;
		s_u32_turbo_in_len[u32_i] = MIN(((UINT32)u16_cb_len_out + u32_llr_shift), (3 * (UINT32)u16_cb_len_in + 12));
		s_u32_mod_len[u32_i] = (UINT32)u16_cb_len_out / g_st_mcs_tbl[u32_i].u16_mod_type;
	}

	//plcp fill bits calc
	u32_fill_bits = (32 - (((UINT32)ZZW_PLCP_BIT_LEN + 4) & 0x1F)) & 0x1F;
	if(u32_fill_bits > 16)
	{
		u32_llr_shift = ((((UINT32)ZZW_PLCP_BIT_LEN + 4 + 31)>>5)<<1) - 2;
	}
	else
	{
		u32_llr_shift = ((((UINT32)ZZW_PLCP_BIT_LEN + 4 + 31)>>5)<<1) - 1;
	}
	s_u32_llr_shift_plcp = u32_llr_shift;
	s_u32_turbo_in_len_plcp = MIN(((UINT32)PLCP_ENCODE_LEN + u32_llr_shift), (3 * (UINT32)ZZW_PLCP_BIT_LEN + 12));
}


VOID dl_hw_init()
{
	UINT32 u32_i;
	UINT32 u32_tmp;

	_init_fic();
	//test(test_data);
	
    #ifdef HS_ONLY_DL_TEST_ONBOARD
	//dl_init_onboard();
    //dl_rxdfe_dmac_config(RXDFE_PACK_LENGHT, RXDFE_PACK_AMT, (UINT32_PTR)(s_st_rx_buffer_onboard.st_iq_data), 0);
	
    //dl_rxdfe_dmac_config(RXDFE_PACK_LENGHT, RXDFE_PACK_AMT, (UINT32_PTR)(s_st_rx_buffer.st_iq_data), 0);
	#else
    //dl_rxdfe_dmac_config(RXDFE_PACK_LENGHT, RXDFE_PACK_AMT, (UINT32_PTR)(s_st_rx_buffer), 0);
    #if RX_STUB
    dl_rxdfe_dmac_config(1024, 60, (UINT32_PTR)(s_st_rx_buffer_ddr), 0);
    #else
    dl_rxdfe_dmac_config(1024, 60, (UINT32_PTR)(s_st_rx_buffer.st_rx_buffer), 0);
    dl_rxdfe_dmac_config(1024, 60, (UINT32_PTR)(s_st_rx_buffer.st_rx_buffer2), 1);
    #endif
    #endif
	dl_turbo_dec_init();
	dl_dec_info_calc();
	node_tbl_clear();

#if 0
	{
		for(u32_i = 0; u32_i < 924; u32_i++)
		{
			s_st_window_seq[u32_i + 50].s16_im = 0;
			s_st_window_seq[u32_i + 50].s16_re = 0;
		}
		for(u32_i = 0; u32_i < 50; u32_i++)
		{
			s_st_window_seq[u32_i].s16_im = 0;
			s_st_window_seq[u32_i].s16_re = 32767;
			s_st_window_seq[u32_i + 974].s16_im = 0;
			s_st_window_seq[u32_i + 974].s16_re = 32767;

		}
	}
#endif

//#if SCFDMA_SWITCH
#if 0
	for(u32_i = 0; u32_i < 300; u32_i++)
	{
		u32_tmp = *((UINT32_PTR)(s_st_fd_pilot_seq + u32_i));
		*((UINT32_PTR)(s_st_fd_pilot_seq + u32_i)) = *((UINT32_PTR)(s_st_fd_pilot_seq + u32_i + 300));
		*((UINT32_PTR)(s_st_fd_pilot_seq + u32_i + 300)) = u32_tmp;		
	}
#endif
    //dft_ceva_algo(HALF_SC_NUM, s32_shift_amt_1, 1, s_st_fft_in, s_st_fft_out, s_st_temp_buffer[1]);
	WRITE_REG32(0x1724fff4, 6);
}


OAL_ITCM_CODE_SECTION
VOID dl_proc_main(VOID)
{
    oal_msg_t *stp_msg;
    oal_primitive_id_t CONST st_wait_msg_list[2] = {(UINT32)1, (UINT32)OAL_ANY_MSG_ID};
    oal_primitive_id_t st_primitive_id;
    OAL_IRQ_SAVE_AREA;

    //dl_init();
    //dl_hw_init();
    while(1)
    {
        //OAL_TASK_SAFE_SET();
        #if 1
        /* Wait for one message forever */
        stp_msg = ks_oal_msg_receive(st_wait_msg_list, (UINT16)OAL_IPC_WAIT_FOREVER);
        //OAL_TASK_UNSAFE_SET();

        if(NULL_PTR != stp_msg)
        {
            st_primitive_id = OAL_GET_OAL_MSG_PRIMITIVE_ID(stp_msg);
			KS_LOG_TMRL(0xB110);
			//KS_LOG(0, 0xA110, (UINT16)(st_primitive_id));
			
			if(s_u32_rxdfe_restart)
			{
		        //s_s32_compete_timing_compensate = READ_REG32(0x1724FF0C);
		        dl_ics_init();
				KS_LOG(OAL_TRACE_LEVEL_0 , 0xD111 , (UINT16)g_u32_build_date); 			
				KS_LOG(OAL_TRACE_LEVEL_0 , 0xD111 , (UINT16)(g_u32_build_time >> 8));			
			}

            switch(st_primitive_id)
            {
                case MSG_DL_RX_PRIMITIVE_ID:
            		//if(g_u32_ics_flag NEQ (UINT32)ICS_SUCCESS)
					{
						//do sync
						dl_sync_proc();
					}
					s_u32_pingpong_idx ^= 1;			
					OAL_IRQ_DISABLE;
					//KS_LOG(0, 0xA111, s_u32_dma_cnt_out);
					s_u32_dma_cnt_out += 1;
					OAL_ASSERT(s_u32_dma_cnt_in - s_u32_dma_cnt_out < 2, "");
					OAL_IRQ_RESTORE;

                    break;

                default:
                    OAL_ASSERT(0, "");
                    break;
            }
			KS_LOG_TMRL(0xB111);
            /* Release the received message */
            ks_oal_msg_release(&stp_msg);
			//ks_oal_sp_print();

        }
        #else
		if(g_u32_ics_flag NEQ (UINT32)ICS_SUCCESS)
		{
			//do sync
			dl_sync_proc();
			s_u32_pingpong_idx ^= 1;			
		}
        #endif
    }
}
#endif


/*************************************************** END OF FILE ******************************************************/

