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
#define THIS_FILE_NAME_ID   FRAMING_DRV_FILE_ID

/***********************************************************************************************************************
* INCLUDE FILES
***********************************************************************************************************************/
#include "ks_oal.h"
#include "ks_framing.h"
#include "ofdm_framing_proc.h"
#include "zzw_common.h"


FRAMING_DRV_DATA_SECTION UINT32 g_frame_dmac_ch_id = 0xFF;
#ifdef VECTOR_TX_TEST
#define SYM_LEN_TYPE (4)
FRAMING_DRV_DATA_SECTION UINT16 g_frame_sym_len[SYM_LEN_TYPE] = {448, 1328, 2192, 448};
FRAMING_DRV_DATA_SECTION UINT32 g_u32_sym_len_type_map[2] = {0x22222210, 0x32222222};
#else
#define SYM_LEN_TYPE (5)
//FRAMING_DRV_DATA_SECTION UINT16 g_frame_sym_len[SYM_LEN_TYPE] = {GAP0_LEN, (AGC_LEN + (CCA_LEN >> 1)), SLOT_LEN, SYMBOL_LEN, GAP1_LEN};
#ifdef MIMO2
//FRAMING_DRV_DATA_SECTION UINT32 g_u32_sym_len_type_map[2] = {0x23332310, 0x42333};	//sym len type	16symbol
#else
//FRAMING_DRV_DATA_SECTION UINT32 g_u32_sym_len_type_map[2] = {0x33333210, 0x42333332};	//sym len type	16symbol
#endif

#endif



/***********************************************************************************************************************
* EXTERN VARIABLES DECLARATION
***********************************************************************************************************************/
extern DMAC_LLI_ITEM g_ch_LLI_ifft[2][128];
extern UINT32 frame_dmac_info[2][TOTAL_FRAME_SEG_NUM * 3];

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
FRAMING_DRV_ITCM_CODE_SECTION
VOID ul_ofdm_framing_para_config(UINT8 u8_frm_ch_id)
{
#ifdef VECTOR_TX_TEST	
	UINT32 u32_cp_len_type_map = 0;
#else
    UINT32 u32_cp_len_type_map = 0;//0x05545540;
#endif
    UINT32 u32_nco_type_map = 0xeaaaaaa4;
	UINT32 u32_i = 0;
	UINT32 u32_seg_type = 0;
	UINT32 u32_map_idx = 0;
	WAVEFORM_OFDM_CFG *stp_waveform_cfg = &g_st_waveform_cfg;

    //Framing块数配置
    framing_sym_num_cfg(TOTAL_FRAME_SEG_NUM, u8_frm_ch_id);

	//Framing块长度指示，4bit为一个索引，对应一个Framing块，块长度在g_frame_sym_len数组中配置
	for(u32_i = 0; u32_i < ((stp_waveform_cfg->u8_total_frame_seg + 0x7) >> 3); u32_i++)
	{
    	framing_sym_len_type_cfg(u32_i, stp_waveform_cfg->u32_frame_seg_map[u32_i], u8_frm_ch_id);
    }

	//CP插入指示，2bit对应一个Framing块
    for(u32_i = 0; u32_i < TOTAL_FRAME_SEG_NUM; u32_i++ )
    {
    	u32_map_idx = (u32_i & 0x8) >> 3;
    	u32_seg_type = (((stp_waveform_cfg->u32_frame_seg_map[u32_map_idx]) >> (4 * (u32_i & 0x7))) & 0xF);
    	if((u32_seg_type EQ DATA_SYM_TYPE) || (u32_seg_type EQ PILOT_SYM_TYPE)) 
    	{
    		u32_cp_len_type_map |= (0x1 << (u32_i * 2));
    	}
    }
    framing_cp_len_type_cfg(0, u32_cp_len_type_map, u8_frm_ch_id);
	//nco~{V8J>#,~}2bit~{6TS&R;8v~}Framing~{?i~}, ~{T]J1N^P'~}
	//nco指示，2bit对应一个Framing块, 暂时无效
    //framing_nco_type_cfg(0, u32_nco_type_map, u8_frm_ch_id);
    
	for(u32_i = 0; u32_i < MAX_SEG_TYPE; u32_i++)
	{
	    framing_len_type_len_cfg(u32_i, (UINT32)stp_waveform_cfg->u16_frame_seg_len[u32_i], u8_frm_ch_id);
	}


    framing_cp_type_len_cfg(0, 0, u8_frm_ch_id);

#ifndef VECTOR_TX_TEST	
    framing_cp_type_len_cfg(1, CP_LEN, u8_frm_ch_id);
#else
	framing_cp_type_len_cfg(1, 0, u8_frm_ch_id);
#endif
	#if 0
	for(u32_i = 0; u32_i < MAX_SEG_TYPE; u32_i++)
	{
	    framing_nco_type_len1_cfg(u32_i, u8_frm_ch_id, (UINT32)stp_waveform_cfg->u16_frame_seg_len[u32_i]);
	}
	#endif

    //framing_last_is_cfg(1, u8_frm_ch_id);

    framing_dmad_dly_cfg(u8_frm_ch_id, 0x40);
}

FRAMING_DRV_ITCM_CODE_SECTION
void ul_ofdm_framing_dmac_para_config(UINT8 u8_frm_ch_id, UINT32 *u32p_dfe_data_addr)
{
    DMAC_LINK_INFO link_item;
    UINT32 u32_LLI_idx;
	UINT32 u32_len_idx, u32_i, u32_map_idx, u32_seg_num = 0;
	UINT32 u32_block_len[TOTAL_FRAME_SEG_NUM] = {0};
	WAVEFORM_OFDM_CFG *stp_waveform_cfg = &g_st_waveform_cfg;

	for(u32_map_idx = 0; u32_map_idx < ((stp_waveform_cfg->u8_total_frame_seg + 0x7) >> 3); u32_map_idx++)
	{
		for(u32_i = 0; u32_i < 8; u32_i++)
		{
			u32_len_idx = (stp_waveform_cfg->u32_frame_seg_map[u32_map_idx] >> (u32_i * 4)) & 0xF;
		    u32_block_len[u32_i + (u32_map_idx << 3)] = ((stp_waveform_cfg->u16_frame_seg_len[u32_len_idx] - 4) >> 2);
			if(u32_seg_num++ >= TOTAL_FRAME_SEG_NUM)
			{
				break;
			}
		}
	}
    for(u32_LLI_idx = 0; u32_LLI_idx < TOTAL_FRAME_SEG_NUM * 3; u32_LLI_idx++)
    {
        g_ch_LLI_ifft[u8_frm_ch_id][u32_LLI_idx].src         = u32p_dfe_data_addr[u32_LLI_idx];
        g_ch_LLI_ifft[u8_frm_ch_id][u32_LLI_idx].dst         = FRAME_DATAIN_ADD + u8_frm_ch_id * FRAME_DATAIN_ADDOFFSET;
        g_ch_LLI_ifft[u8_frm_ch_id][u32_LLI_idx].src_width   = DMAC_TR_WIDTH_128;
        g_ch_LLI_ifft[u8_frm_ch_id][u32_LLI_idx].dst_width   = DMAC_TR_WIDTH_128;
        g_ch_LLI_ifft[u8_frm_ch_id][u32_LLI_idx].block_ts =    u32_block_len[u32_LLI_idx%TOTAL_FRAME_SEG_NUM];
        g_ch_LLI_ifft[u8_frm_ch_id][u32_LLI_idx].llp         = (UINT32)(&g_ch_LLI_ifft[u8_frm_ch_id][u32_LLI_idx + 1]);
        g_ch_LLI_ifft[u8_frm_ch_id][u32_LLI_idx].src_sts     = DMAC_ADDR_INCREMENT;
        g_ch_LLI_ifft[u8_frm_ch_id][u32_LLI_idx].dst_sts     = DMAC_ADDR_INCREMENT;
        g_ch_LLI_ifft[u8_frm_ch_id][u32_LLI_idx].lli_last    = DMAC_LLI_NOT_LAST;
        g_ch_LLI_ifft[u8_frm_ch_id][u32_LLI_idx].lli_valid   = DMAC_LLI_VALID;
        g_ch_LLI_ifft[u8_frm_ch_id][u32_LLI_idx].ioc_blk     = DMAC_IOC_BLKTRF_DISABLE;
        g_ch_LLI_ifft[u8_frm_ch_id][u32_LLI_idx].RESEVED_src = 0x0;
        g_ch_LLI_ifft[u8_frm_ch_id][u32_LLI_idx].RESEVED_dst = 0x0;
        g_ch_LLI_ifft[u8_frm_ch_id][u32_LLI_idx].RESEVED     = 0x0;
        g_ch_LLI_ifft[u8_frm_ch_id][u32_LLI_idx].RESEVED_llp = 0x0;
        g_ch_LLI_ifft[u8_frm_ch_id][u32_LLI_idx].reseved1    = 0x0;
        g_ch_LLI_ifft[u8_frm_ch_id][u32_LLI_idx].reseved2    = 0x0;
        g_ch_LLI_ifft[u8_frm_ch_id][u32_LLI_idx].reseved3    = 0x0;
        g_ch_LLI_ifft[u8_frm_ch_id][u32_LLI_idx].src_msize   = DMAC_MSIZE(g_ch_LLI_ifft[u8_frm_ch_id][u32_LLI_idx].block_ts);// DMAC_MSIZE_1024;
        g_ch_LLI_ifft[u8_frm_ch_id][u32_LLI_idx].dst_msize   = DMAC_MSIZE(g_ch_LLI_ifft[u8_frm_ch_id][u32_LLI_idx].block_ts);// DMAC_MSIZE_1024;
    }

    g_ch_LLI_ifft[u8_frm_ch_id][TOTAL_FRAME_SEG_NUM * 3 - 1].llp = (UINT32)(&g_ch_LLI_ifft[u8_frm_ch_id][0]);
    link_item.prior        = DMAC_CH_PRIO7;
    link_item.lli_mode     = DMAC_LLI_LOOP; //DMAC_LLI_NORMAL;
    link_item.lli          = g_ch_LLI_ifft[u8_frm_ch_id];
    link_item.lli_count    = TOTAL_FRAME_SEG_NUM * 3;
    link_item.trf_type     = DMAC_TRF_PERI2PERI;
    link_item.src_trf_mode = DMAC_TRF_LINK;
    link_item.dst_trf_mode = DMAC_TRF_LINK;
    link_item.hs_type      = DMAC_HS_HARDWARE;
    link_item.intr         = DMAC_CH_INTR_INVALID; //DMAC_CH_BLOCK_TRF_DONE; // DMAC_CH_DMA_TRF_DONE;
#if defined(CP1_X1643)
    link_item.dst_hs_ch    = DMAC_CP1_CP_INS0_WREQ + u8_frm_ch_id;
    link_item.src_hs_ch    = DMAC_CP1_CP_INS0_WREQ + u8_frm_ch_id;
    g_frame_dmac_ch_id     = dmac_ch_request(KS_DMAC1_CP);
    link_item.ch_id        = g_frame_dmac_ch_id;

    OAL_ASSERT(link_item.ch_id != DMAC_CH_INVALID, "dma channel req failure");

    ks_dmac_link_config(KS_DMAC1_CP, &link_item);
    ks_dmac_register_handle(KS_DMAC1_CP, link_item.ch_id, NULL_PTR, NULL);
    ks_dmac_ch_start(KS_DMAC1_CP, g_frame_dmac_ch_id);
#else
    link_item.dst_hs_ch    = DMAC_CP0_CP_INS0_WREQ + u8_frm_ch_id;
    link_item.src_hs_ch    = DMAC_CP0_CP_INS0_WREQ + u8_frm_ch_id;
    g_frame_dmac_ch_id     = dmac_ch_request(KS_DMAC2_CP);
    link_item.ch_id        = g_frame_dmac_ch_id;

    OAL_ASSERT(link_item.ch_id != DMAC_CH_INVALID, "dma channel req failure");

    ks_dmac_link_config(KS_DMAC2_CP, &link_item);
    ks_dmac_register_handle(KS_DMAC2_CP, link_item.ch_id, NULL_PTR, NULL);
    ks_dmac_ch_start(KS_DMAC2_CP, g_frame_dmac_ch_id);
#endif
}

FRAMING_DRV_ITCM_CODE_SECTION
VOID framing_config_init(UINT8 u8_frm_ch_id)
{
    framing_clk_en(u8_frm_ch_id);
    framing_config_en(u8_frm_ch_id);

    framing_init(u8_frm_ch_id);
    ul_ofdm_framing_para_config(u8_frm_ch_id);
    
    frame_ram_init(u8_frm_ch_id);
    ul_ofdm_framing_dmac_para_config(u8_frm_ch_id, frame_dmac_info[u8_frm_ch_id]);
}

