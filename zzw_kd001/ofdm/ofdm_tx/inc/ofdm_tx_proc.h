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
#ifndef _ALGO_UL_PROC_H_
#define _ALGO_UL_PROC_H_

/***********************************************************************************************************************
* INCLUDE FILES
***********************************************************************************************************************/
#include "ks_typedef.h"
#include "ks_macro.h"
#include "zzw_common.h"
/***********************************************************************************************************************
* MACRO DEFINITION
***********************************************************************************************************************/
#define AGLO_CB_MAX_LEN                   (6144)
#define AGLO_CRC24_LEN                    (24)
#define AGLO_K_TABLE_LEN                  (188)
#define AGLO_C_TC_SUBBLOCK                (32)
#define AGLO_ENCODE_DATA_MAX_LEN          (765)//6block承载的数据大小，BYTE

#define MAX_TURBO_ENCODER_COUNTER          1
#define TX_BUFFER_SIZE                    (0x1B00)
#define PLCP_TYPE                         (0)
#define DATA_TYPE                         (1)

#define TX_ANTENNA_1X		1
#define TX_ANTENNA_2X		2		
#define TX_2X_ANTENNA_MODE_SFBC			1
#define TX_2X_ANTENNA_MODE_SM				2

#define TX_CTRL_INFO_CNT_MAX	3		// actually buf size is (3 - 1)
#define ALGO_CTRL_MOD_CNT_MAX	3		// actually buf size is (3 - 1)
#define ALGO_DATA_MOD_CNT_MAX	3		// actually buf size is (3 - 1)

typedef struct TX_BRP_PARA_TAG
{
    UINT32 u32_in_bit_length;
    UINT32 u32_out_bit_length;
    UINT32 u32_rv;
    UINT32 u32_NL;
    UINT32 u32_Qm;
    UINT32 u32_cp_stytle;
    UINT32 u32_sch_symb_num;
    UINT32 u32_ul_sch_ram_sel;
    UINT32 u32_endian_mode;
    UINT32 u32_rd_mode;
    UINT32 u32_cfg_ubp_frm_flag;
}algo_tx_brp_para;

typedef struct algo_ul_encoder_info_tag
{
    UINT32 turbo_encode_cnt;//编码的次数
    UINT32 tx_mod;
    UINT32 block_num;
	UINT32 mcs_idx;
    UINT32 modulation;
    UINT32 rd_mode;
	UINT32 data_size;//bit
    UINT32 ping_pong_addr[MAX_TURBO_ENCODER_COUNTER];
    UINT32 data_addr[MAX_TURBO_ENCODER_COUNTER];
    algo_tx_brp_para tx_brp_cfg[MAX_TURBO_ENCODER_COUNTER];
}algo_ul_encoder_info_t;

typedef struct algo_up_ifft_info_tag
{
    UINT32 u32_fft_ifft_sel; 
    UINT32 u32_iq_data_width;
    UINT32 u32_fft_num;
    UINT32 u32_sym_num;
    UINT32 u32_fft_idx;
    UINT32 u32_cp_idx;
    UINT32 cfg_out_iq_swap;
}algo_up_ifft_info_t;

#if 0
#define ALGO_UL_QUEUE_NUM (3)

typedef struct algo_ul_des_ele_tag
{
    UINT32 u32_ul_type; //0:plcp;1:data;
    //UINT32 u32_ul_buf_id; //0: buf 0; 1: buf 1 
    //UINT32 u32_ul_buf_inuse; //0: not used; 1: in use  
    UINT32 u32_mod_type; //ul mod type      
}algo_ul_des_ele_t;

typedef struct algo_ul_des_tag
{
    UINT32 u32_cnt_in; 
    UINT32 u32_cnt_out;
    algo_ul_des_ele_t st_ul_queue[ALGO_UL_QUEUE_NUM];
}algo_ul_des_t;
#endif

typedef enum data_type_enum
{
	DATA_TYPE_PLCP = 0,
	DATA_TYPE_DATA = 1	
}data_type_e;

typedef enum tx_queue_type_enum
{
	TX_QUEUE_TYPE_TURBO = 0,
	TX_QUEUE_TYPE_IFFT = 1	
}tx_queue_type_e;

/***********************************************************************************************************************
* GLOBAL FUNCTION DECLARATION
***********************************************************************************************************************/
VOID algo_tx_proc_init(VOID);
VOID algo_data_tx_process(UINT32 u32_buf_id);
VOID algo_ul_datapro_ctrl_task(VOID);
VOID ofdm_tx_debug_para_init();
VOID algo_ul_ifft_config_init(VOID);
VOID algo_ul_dft1200_config_init(void);
VOID algo_ul_data_symbproc(VOID);
VOID algo_data_tx_process_plcp(VOID);
VOID algo_ul_scram(IN UINT16 tb_size, IN UINT32* input_buf, OUT UINT32* output_buf);
VOID algo_ul_scram_data(IN UINT16 tb_size,IN UINT32 mcs, IN UINT8* input_buf, OUT UINT8* output_buf);
VOID algo_ul_tx_cancel_proc(VOID);

/***********************************************************************************************************************
* EXTERN VARIABLE DECLARATION
***********************************************************************************************************************/
extern UINT8 g_u8_tx_buffer[3][TX_BUFFER_SIZE + TX_BUFF_HEAD_SIZE];

/***********************************************************************************************************************
* EXTERN VARIABLE DECLARATION
***********************************************************************************************************************/
#endif /* _ALGO_UL_PROC_H_ */
/**************************************************** END OF FILE *****************************************************/

