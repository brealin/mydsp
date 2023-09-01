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
#ifndef _OFDM_DL_PROC_H_
#define _OFDM_DL_PROC_H_

/***********************************************************************************************************************
* INCLUDE FILES
***********************************************************************************************************************/
#include "vec-c.h"
#include "ks_typedef.h"
#include "ks_macro.h"
#include "alg_func.h"
#include "zzw_common.h"

/***********************************************************************************************************************
* MACRO DEFINITION
***********************************************************************************************************************/


#define CP_SHIFT                                    (-4)
#define PILOT_INSERT_GAP                            (5)

#define HALF_SC_NUM                                 (SC_NUM>>1)
#define QUARTER_SC_NUM                              (SC_NUM>>2)

#define EDGE_SUBCARRIER_NUM                         (12)
#define EDGE_SUBCARRIER_OPT_NUM 					(24)

#define SYMBOL_NUM_BEFORE_1STDATA                   (2)//symbol num before 1st data symbol
#ifdef MIMO2
#define DIS_FROM_CCA2PILOT                          ((CCA_LEN << 1) + 0)
#define PILOT_DIS_SYM                               (4)    //pilots distance ,unit: symbol
#define ICS_1ST_SF_BMP    							(0x1F)   // 1st&2nd pilots, 0~4 data symbols
#define ICS_2ND_SF_BMP    							(0x1E0)   // 3rd pilots, 5~9 data symbols

#define PILOT1_SYM_ID    							(3)//2nd pilot symbol index(symbol idx starts from 1st data symbol)                  
#define PILOT2_SYM_ID    							(7)//3nd pilot symbol index(symbol idx starts from 1st data symbol)
#define LAST_SYM_ID                                 (PILOT2_SYM_ID)

#else
#define DIS_FROM_CCA2PILOT                          ((CCA_LEN << 1) + 0)
#define PILOT_DIS_SYM                               (6)
#define ICS_1ST_SF_BMP    							(0x7F)   // 1st&2nd pilots, 0~4 data symbols
#define ICS_2ND_SF_BMP    							(0x1F80)   // 3rd pilots, 5~9 data symbols

#define PILOT1_SYM_ID    							(4)//2nd pilot symbol index(symbol idx starts from 1st data symbol)
#define PILOT2_SYM_ID    							(10)//3nd pilot symbol index(symbol idx starts from 1st data symbol)
#define LAST_SYM_ID                                 (PILOT2_SYM_ID)

#endif
#define DATA_SYM_ID_2SF    							(PILOT1_SYM_ID + 1)//1ST DATA symbol index in 2nd subframe(symbol idx starts from 1st data symbol)
//#define PILOT_0_BMP 		(0)	
#define PILOT_1_BMP 								(PILOT1_SYM_ID + SYMBOL_NUM_BEFORE_1STDATA)
#define PILOT_2_BMP 								(PILOT2_SYM_ID + SYMBOL_NUM_BEFORE_1STDATA)


#define CCA_POS_IDEAL                               (GAP0_LEN + AGC_LEN + CCA_POS_DELTA)   //gap0 + agc
#define CCA_POS_FULL_FRAME                          (CCA_POS_IDEAL + GAP1_LEN)// - CCA_POS_DELTA) //gap0 + agc + gap1



#define RX_BUF_LEN_ORG 		(FRAME_LEN)
#define RX_BUF_LEN   		(RX_BUF_LEN_ORG)

#define L1CC_BB_SCFDE_DEC_DATA_BUF_START_ADDR  (0x17051000)
//#define L1CC_BB_SCFDE_DEC_DATA_BUF_START_ADDR  (&s_s8_decode_out)
#define L1CC_BB_SCFDE_DEC_DATA_BUF_END_ADDR    ((SINT8_PTR)(L1CC_BB_SCFDE_DEC_DATA_BUF_START_ADDR) + 0x2800)




#define ICS_CORR_LEN 								(SYMBOL_LEN - (CCA_LEN << 1))    		//1536
#define ICS_SEG_NUM  								(RX_BUF_LEN / ICS_CORR_LEN)        		//30720/1536 = 20
#define ICS_SEG_REM  								(RX_BUF_LEN - ICS_SEG_NUM * ICS_CORR_LEN + (CCA_LEN << 1)) //512
#define ICS_RSV_LEN  								((CCA_LEN << 1)) 
#define ICS_FFT_LEN  								((2048)) //512 

#define MODV_SV0      								(0x21180C22) //0x21000E22
#define MODV_SV1      								(0x21180CA2) //0x21000EA2
#define MODV_SV2      								(0x21180D22) //0x21000F22

#define PILOT_INSERT_GAP                            (5)
#define MAX_CODE_BLOCK_SYM                          (3)

#define M_SEQ_LENGHT                                (256)
#define SEGMENT_NUM                                 (3)
#define RXDFE_PACK_LENGHT                        	(SEGMENT_NUM*M_SEQ_LENGHT)/*unit samples, complex_s16_t*/


#ifdef HS_ONLY_DL_TEST
#define RXDFE_PACK_AMT                              (80)
#else
#define RXDFE_PACK_AMT                              (80)
#endif

#define RTX_1T1R 								    (0)
#define RTX_1T2R 								    (1)
#define RTX_2T1R 								    (2)
#define RTX_2T2R 								    (3)
#define RX_2_ANT(MIMO_TYPE)   						(MIMO_TYPE & 0x1)   //1: rx with 2 ants , 0 ：1 ant
#define TX_2_ANT(MIMO_TYPE)   						(MIMO_TYPE >> 1) 	  //1: tx with 2 ants , 0 ：1 ant
//#define RX_ANT_NUM  								((g_u32_mimo_type & 0x1) + 1)   //2: rx with 2 ants , 1 ：1 ant
//#define TX_ANT_NUM  								((g_u32_mimo_type >> 1) + 1) 	  //2: tx with 2 ants , 1 ：1 ant

#define RX_ANT_NUM  								(2)   //2: rx with 2 ants , 1 ：1 ant
#define TX_ANT_NUM  								(2) 	  //2: tx with 2 ants , 1 ：1 ant

#define DELTA_OF_ANT_BUF                            ((RX_BUF_LEN << 1) + SLOT_LEN)
#define RX_2ANT_EN                                  (1)
#if RX_2ANT_EN
#define MAX_RX_ANT_NUM           					(2)
#else
#define MAX_RX_ANT_NUM           					(1)
#endif
#define MAX_H_NUM                                   (MAX_RX_ANT_NUM * MAX_RX_ANT_NUM)
#define FINE_TIME_TBIS								(10)



#define MAX_DATA_LATTER_PILOT_NUM					(9)
#define MSG_DL_RX_PRIMITIVE_ID                      (0)

// temp define for soft test

typedef struct dl_rx_buf_tag
{
	complex_s16_t st_ics_left[SLOT_LEN];
	complex_s16_t st_rx_buffer[2][RX_BUF_LEN];
	#if RX_2ANT_EN
	complex_s16_t st_ics_left2[SLOT_LEN];
	complex_s16_t st_rx_buffer2[2][RX_BUF_LEN];
	#endif
}dl_rx_buf_t;


//end
   
#define MSG_CCA_REPORT_PRIMITIVE_ID                 (1)
#define MSG_SYNC_REPORT_PRIMITIVE_ID                (2)
#define MSG_PLCP_REPORT_PRIMITIVE_ID                (3)
    
#define EVE_SELECTED                                (0)
#define ODD_SELECTED                                (1)

#define SYNC_THRESHOLD                              (32)

#define OAL_L1CC_TASK_ID                            (1)      

#define PLCP_TYPE                                   (0)
#define DATA_TYPE                                   (1)



#define MAX(a,b)                                    (((a)<(b))?(b):(a))
#define MIN(a,b)                                    (((a)<(b))?(a):(b))

#define TURBO_DEC_INFO_AMT                          (8)
#define TURBO_DEC_LLR_BUFFER_SIZE                   (0x10000)

typedef enum 
{
    CODE_RATE_HALF = 0,
    CODE_RATE_TWO_THIRD,
    CODE_RATE_THREE_FOURTHS,
}DL_CODE_RATE;

typedef enum 
{
    INIT = 0,
    CCA_DETECT,
    PILOT_DETECT,
    PLCP_DECODE,
    DATA_DECODE
}DL_PROC_STATUS;


typedef enum 
{
    ICS_CCA_DETECT = 0,
    ICS_HEAD_COLLECT = 1, //1st pilot done
    ICS_HEAD_COLLECT2 = 2,//pilot & plcp not processed
    ICS_HEAD_COLLECT3 = 3,//pilot & plcp fully in next frame
    ICS_HEAD_DECODE = 4,
    ICS_DATA_COLLECT = 5,
    ICS_DATA_DECODE = 6,
    ICS_SUCCESS = 7
}ICS_STATUS;

typedef struct _RX_RING_BUFFER_T_
{
    UINT32 u32_read_cnt;
    UINT32 u32_write_cnt;
    UINT32 u32_reserved[2];

    complex_s16_t st_iq_data[RXDFE_PACK_AMT][RXDFE_PACK_LENGHT];
}RX_RING_BUFFER_T;



typedef struct decode_info_tag
{
    UINT32 u32_input_lenght;
    UINT32 u32_output_lenght;
}decode_info_t;

typedef struct plcp_info_tag
{
    UINT16 u16_is_valid;
    UINT16 u16_need_report;
    
    UINT16 u16_crc;
    UINT16 u16_mcs;
    
    UINT16 u16_code_rate;
    UINT16 u16_subframe_num;
    
    UINT16 u16_last_subframe_slot_num;
    UINT16 u16_code_blcok_sym;
    UINT16 u32_input_lenght[2];
    UINT16 u32_output_lenght[2];
    UINT16 u16_data_type;
    UINT16 u16_fn_local;

}plcp_info_t;

typedef struct dl_debug_info_tag
{
    UINT32 u32_rx_status;

    UINT32 u32_dmac_callback_cnt;
    UINT32 u32_cca_read_cnt;
    UINT32 u32_plcp_read_cnt;
    UINT32 u32_data_read_cnt;

    UINT32 u32_plcp_dec_success_cnt;
    UINT32 u32_plcp_dec_failure_cnt;

    UINT32 u32_data_dec_success_cnt;
    UINT32 u32_data_dec_failure_cnt; 

    UINT32 u32_block_dec_success_cnt;
    UINT32 u32_block_dec_failure_cnt; 

    SINT32 s32_cca_cfo_real;
    SINT32 s32_cca_cfo_imag;

    SINT32 s32_plcp_cfo_real;
    SINT32 s32_plcp_cfo_imag;

    SINT32 s32_cca_eve_peak_index;
    SINT32 s32_cca_odd_peak_index;

    VOLATILE_UINT32 u32_enable_iq_dump;

	SINT32 s32_rtc_time;	// 0x48
    SINT32 s32_frame_no;
    SINT32 s32_frame_no_ori;
	SINT32 s32_cca_recheck_div;

	SINT32 s32_cfo_value;  // 0x58
    SINT32 s32_time_adj_value;	// 0x5c
    SINT32 s32_shift_value;	// 0x60
	SINT32 s32_signal_pwr;		// 0x64
	SINT32 s32_noise_pwr;		// 0x68
	
    SINT32 s32_temp_buffer[2][512];
}dl_debug_info_t;

typedef struct turbo_decode_info_tag
{
    UINT32 u32_data_type;

    UINT32 u32_input_lenght;
    UINT32 u32_output_lenght;

    UINT32 u32_write_offset;
	UINT32 u32_crc_type;
}turbo_decode_info_t;

typedef struct turbo_decode_info_link_tag
{
    UINT32 u32_read_index;
    UINT32 u32_write_index;
    UINT32 u32_reserved[2];

    turbo_decode_info_t st_turbo_dec_info[TURBO_DEC_INFO_AMT];
    //SINT8 s8_llr_buffer[TURBO_DEC_LLR_BUFFER_SIZE]; /*起始地址必须16字节对齐*/
}turbo_decode_info_link_t;


/***********************************************************************************************************************
* GLOBAL FUNCTION DECLARATION
***********************************************************************************************************************/

/* 复数矢量乘实数矢量，即矢量中的每个复数元素的实部和虚部均乘以对应实数矢量的对应元素。
   乘数和乘积的实部/虚部定点格式均为Q(16,1)。当实部或虚部的乘积饱和时，
   根据乘积的正负类型分别输出MAX_POS_S16和MAX_NEG_S16。*/
extern VOID _ks_cxv_mul_s16v(IN complex_s16_t * const s16cp_indata_complex_vector,
                             IN CONST_SINT16_PTR s16p_indata_real_vector,
                             IN CONST_SINT16 s16_data_len,
                             OUT complex_s16_t* s16cp_result_vector);

/* 复数矢量加复数矢量，即矢量中对应复数相加。
   数据具体定标无要求，但对应相加的两个数据的定标必须一致。
   当实部或虚部的加和结果饱和时，根据结果的正负类型分别输出MAX_POS_S16和MAX_NEG_S16。 */
extern VOID _ks_cxv_add_cxv(IN const complex_s16_t* const s16cp_indata1_vector,
                            IN const complex_s16_t* const s16cp_indata2_vector,
                            IN CONST_SINT16 s16_data_len,
                            OUT complex_s16_t* const s16cp_result_vector);

/* 复数矢量减复数矢量，即矢量中对应复数相减。
   数据具体定标无要求，但对应相减的两个数据的定标必须一致。
   当实部或虚部的减结果饱和时，根据结果的正负类型分别输出MAX_POS_S16和MAX_NEG_S16。 */
extern VOID _ks_cxv_sub_cxv(IN const complex_s16_t* s16cp_indata1_vector,
                            IN const complex_s16_t* s16cp_indata2_vector,
                            IN CONST_SINT16 s16_data_len,
                            OUT complex_s16_t* s16cp_result_vector);

extern VOID _ks_s16v_mul_s16(IN CONST_SINT16_PTR s16p_multiplier_vector,
                             IN CONST_SINT16 s16_data_len,
                             IN CONST_SINT16 s16_single_multiplier,
                             OUT SINT16_PTR s16p_product_vector);

extern VOID ks_fft512(INOUT ks_fft_para_t* const stp_fft_para);

extern VOID ks_ifft512(INOUT ks_fft_para_t* const stp_fft_para);

extern SINT32 ks_cxv_rssi_calc(IN complex_s16_t * s16cp_src_addr, IN CONST_SINT16 s16_data_len);
extern VOID dl_proc_main(VOID);
extern VOID dl_hw_init();
extern UINT32 dl_cca_proc(SINT32_PTR s32p_in, UINT32 u32_offset, UINT32 u32_fft_last, UINT32 *u32p_fft_cnt);


#endif /* _DL_PROC_H_ */
/**************************************************** END OF FILE *****************************************************/

