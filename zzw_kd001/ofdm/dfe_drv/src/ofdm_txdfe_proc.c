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
#define THIS_FILE_NAME_ID    CSMA_TXDFE_PROC_FILE_ID

/***********************************************************************************************************************
* INCLUDE FILES
***********************************************************************************************************************/
#include "ks_oal.h"
#include "txdfe_intf_reg.h"
#include "ks_txdfe.h"
#include "txdfe_drv.h"
#include "ks_framing.h"
#include "txdfe_reg.h"
#include "zzw_common.h"

#define TXDFE_DEBUG_EN  (0)
/***********************************************************************************************************************
* GLOBAL VARIABLES DEFINITION
***********************************************************************************************************************/
DTCM_DATAPATH_TCE_DATA_SECTION KS_TXDFE_INIT_PARA g_txdfe_init_para = {{{(UINT32)0}}};

DTCM_DATAPATH_TCE_DATA_SECTION UINT16 u16_txdfe_fir0_coef_real[33] = {
    #include "txdfe_FIR_filter_real.txt"
};

DTCM_DATAPATH_TCE_DATA_SECTION UINT16 u16_txdfe_fir0_coef_imag[33] = {
    #include "txdfe_FIR_filter_imag.txt"
};

VOID ul_ofdm_framing_para_config(UINT8 u8_frm_ch_id);

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
TXDFE_DRV_ITCM_CODE_SECTION
VOID txdfe_para_init(KS_TXDFE_INIT_PARA *txdfe_init_para)
{
    txdfe_init_para->txdfe_common.txdfe_channel.b_txdfe_channel_sel  = 0;
    txdfe_init_para->txdfe_common.txdfe_channel.b_txdfe_channel_en   = 1;
    txdfe_init_para->txdfe_common.txdfe_channel.b_txdfe_channel_1_en = 0;

    txdfe_init_para->txdfe_common.txdfe_bif.e_txdfe_adiu_auto_init_mode = 1;
    txdfe_init_para->txdfe_common.txdfe_bif.u32_txdfe_adiu_renew_mode   = 2;
    txdfe_init_para->txdfe_common.txdfe_bif.b_txdfe_bif_if_mode = 0;
    txdfe_init_para->txdfe_common.txdfe_bif.b_txdfe_bif_if_tone_trig_mode = 1;

    txdfe_init_para->txdfe_common.txdfe_fifo_out.b_txdfe_fifo_out_iq_swap = 0;

	#if TXDFE_DEBUG_EN
    txdfe_init_para->txdfe_filter.txdfe_dbg.b_txdfe_dbg_en = 1;// 1
    #else
    txdfe_init_para->txdfe_filter.txdfe_dbg.b_txdfe_dbg_en = 0;// 1
    #endif
    txdfe_init_para->txdfe_filter.txdfe_dbg.b_txdfe_dbg_mode = 1;
    txdfe_init_para->txdfe_filter.txdfe_dbg.e_txdfe_dbg_point_sel = KS_TXDFE_DEBUG_POINT_SEL_FREQ_SHIFT_OUPUT;

    txdfe_init_para->txdfe_filter.txdfe_scaling.b_txdfe_gain_en = 1;
    txdfe_init_para->txdfe_filter.txdfe_scaling.u32_txdfe_gain_coef_i_dynamic = 0x20000000;
    txdfe_init_para->txdfe_filter.txdfe_scaling.u32_txdfe_gain_coef_q_dynamic = 0;

    txdfe_init_para->txdfe_filter.txdfe_cord.b_txdfe_cord_en = 0;
    txdfe_init_para->txdfe_filter.txdfe_cord.u32_txdfe_cord_step_dynamic = 0x8000000;
    txdfe_init_para->txdfe_filter.txdfe_cord.u32_txdfe_cord_init_phase_dynamic = 0;

    txdfe_init_para->txdfe_filter.txdfe_bb.b_txdfe_bb_en = 0;
    
    txdfe_init_para->txdfe_filter.txdfe_ov.b_txdfe_ov_stage0_en = 0;// 1
    txdfe_init_para->txdfe_filter.txdfe_ov.b_txdfe_ov_stage1_en = 0;

    txdfe_init_para->txdfe_filter.txdfe_fir0.b_txdfe_fir0_en = 0;// 1
    ks_oal_mem_copy(txdfe_init_para->txdfe_filter.txdfe_fir0.u16_txdfe_fir0_coef_real, u16_txdfe_fir0_coef_real, 2 * 33);
    ks_oal_mem_copy(txdfe_init_para->txdfe_filter.txdfe_fir0.u16_txdfe_fir0_coef_imag, u16_txdfe_fir0_coef_imag, 2 * 33);

    txdfe_init_para->txdfe_filter.txdfe_txsrc.b_txdfe_tx_src_lpf1_stage0_fir_en = 0;
    txdfe_init_para->txdfe_filter.txdfe_txsrc.b_txdfe_tx_src_lpf1_stage1_fir_en = 0;
    txdfe_init_para->txdfe_filter.txdfe_txsrc.b_txdfe_tx_src_lpf1_stage2_fir_en = 0;
    txdfe_init_para->txdfe_filter.txdfe_txsrc.u8_txdfe_tx_src_cic_ratio = 0;
    txdfe_init_para->txdfe_filter.txdfe_txsrc.b_txdfe_tx_src_lpf1_stage0_upsrc_en = 0;
    txdfe_init_para->txdfe_filter.txdfe_txsrc.b_txdfe_tx_src_lpf1_stage1_upsrc_en = 0;
    txdfe_init_para->txdfe_filter.txdfe_txsrc.b_txdfe_tx_src_lpf1_stage2_upsrc_en = 0;
    txdfe_init_para->txdfe_filter.txdfe_txsrc.b_txdfe_tx_src_cic_upsrc_en = 0;

    txdfe_init_para->txdfe_filter.txdfe_nco.b_txdfe_nco_en = 0;
    txdfe_init_para->txdfe_filter.txdfe_nco.u32_txdfe_nco_step_dynamic = 0;
    txdfe_init_para->txdfe_filter.txdfe_nco.u32_txdfe_nco_init_phase_dynamic = 0;

    txdfe_init_para->txdfe_filter.txdfe_prach.b_txdfe_prach_en = 0;
    txdfe_init_para->txdfe_filter.txdfe_prach.u32_txdfe_prach_step_dynamic = 0;
    txdfe_init_para->txdfe_filter.txdfe_prach.u32_txdfe_prach_init_phase_dynamic = 0;

    #ifdef BAND_WIDTH_10M
    txdfe_init_para->txdfe_filter.txdfe_hb.b_txdfe_hb_upsrc_en = 1;
    txdfe_init_para->txdfe_filter.txdfe_hb.b_txdfe_hb_fir_en = 1;
	#else
    txdfe_init_para->txdfe_filter.txdfe_hb.b_txdfe_hb_upsrc_en = 0;
    txdfe_init_para->txdfe_filter.txdfe_hb.b_txdfe_hb_fir_en = 0;
	#endif
    txdfe_init_para->txdfe_filter.txdfe_freq_shift.b_txdfe_freq_en = 0;
    txdfe_init_para->txdfe_filter.txdfe_freq_shift.u32_txdfe_freq_step_dynamic = 0;
    txdfe_init_para->txdfe_filter.txdfe_freq_shift.u32_txdfe_freq_init_phase_dynamic = 0;
}

TXDFE_DRV_ITCM_CODE_SECTION
VOID txdfe_up_sampling_rate_para_init(KS_TXDFE_UP_SAMPLING_RATE_PARA *txdfe_up_sampling_rate)
{
    txdfe_up_sampling_rate->e_clk_txdfe_clk_data_output_rate =
    		KS_TXDFE_CLK_OUTPUT_RATE_61440KHz_DATA_OUTPUT_RATE_30720KHz;
    #ifdef BAND_WIDTH_10M
    txdfe_up_sampling_rate->u32_clk_txdfe_pre_ov_out_sample_rate_times   = 1;
    txdfe_up_sampling_rate->u32_clk_txdfe_ov_stage0_sample_rate_times    = 1;
    txdfe_up_sampling_rate->u32_clk_txdfe_ov_stage1_sample_rate_times    = 1;
    txdfe_up_sampling_rate->u32_clk_txdfe_fir0_sample_rate_times         = 1;
    txdfe_up_sampling_rate->u32_clk_txdfe_lpf1_stage0_sample_rate_times  = 1;
    txdfe_up_sampling_rate->u32_clk_txdfe_lpf1_stage1_sample_rate_times  = 1;
    txdfe_up_sampling_rate->u32_clk_txdfe_lpf1_stage2_sample_rate_times  = 1;
    txdfe_up_sampling_rate->u32_clk_txdfe_post_cic_out_sample_rate_times = 1;
    txdfe_up_sampling_rate->u32_clk_txdfe_post_hb_out_sample_rate_times  = 2;
    txdfe_up_sampling_rate->u32_clk_txdfe_debug_sample_rate_times        = 2;

	#else
    txdfe_up_sampling_rate->u32_clk_txdfe_pre_ov_out_sample_rate_times   = 1;
    txdfe_up_sampling_rate->u32_clk_txdfe_ov_stage0_sample_rate_times    = 1;
    txdfe_up_sampling_rate->u32_clk_txdfe_ov_stage1_sample_rate_times    = 1;
    txdfe_up_sampling_rate->u32_clk_txdfe_fir0_sample_rate_times         = 1;
    txdfe_up_sampling_rate->u32_clk_txdfe_lpf1_stage0_sample_rate_times  = 1;
    txdfe_up_sampling_rate->u32_clk_txdfe_lpf1_stage1_sample_rate_times  = 1;
    txdfe_up_sampling_rate->u32_clk_txdfe_lpf1_stage2_sample_rate_times  = 1;
    txdfe_up_sampling_rate->u32_clk_txdfe_post_cic_out_sample_rate_times = 1;
    txdfe_up_sampling_rate->u32_clk_txdfe_post_hb_out_sample_rate_times  = 1;
    txdfe_up_sampling_rate->u32_clk_txdfe_debug_sample_rate_times        = 1;
	#endif

}
#if MIMO_2ANTS
TXDFE_ITCM_CODE_SECTION
void txdfe_clk_enable_reset_disable(UINT8 ch_id)
{
    framing_clk_en(ch_id);
	framing_clk_en(1);
    ks_oal_delay((UINT32)20);
    framing_config_en(ch_id);
	framing_config_en(1);
    txdfe_reset_disable(ch_id);
	txdfe_reset_disable(1);
    ks_oal_delay((UINT32)20);
    SET_CPU_TXDFE_CLK_SET(1, ch_id);
	SET_CPU_TXDFE_CLK_SET(1, 1);
}

TXDFE_ITCM_CODE_SECTION
void txdfe_clk_disable_reset_enable(UINT8 ch_id)
{
    SET_CPU_TXDFE_DBG_EN(0, ch_id);
    SET_CPU_TXDFE_CHANNEL_EN(0, ch_id);
    SET_CPU_TXDFE_CLK_SET(0, ch_id);
	SET_CPU_TXDFE_DBG_EN(0, 1);
    SET_CPU_TXDFE_CHANNEL_EN(0, 1);
    SET_CPU_TXDFE_CLK_SET(0, 1);

    txdfe_all_subsys_clk_disable();
    framing_clk_cls(ch_id);
	framing_clk_cls(1);
    ks_oal_delay((UINT32)20);
    framing_config_cls(ch_id);
	framing_config_cls(1);
    txdfe_reset_enable(ch_id);
	txdfe_reset_enable(1);
    ks_oal_delay(20);
}

TXDFE_ITCM_CODE_SECTION
eKS_TXDFE_STATUS txdfe_para_cfg_init(KS_TXDFE_INIT_PARA *txdfe_init_para, UINT8 ch_id)
{
    eKS_TXDFE_STATUS sts;

    KS_ASSERT(NULL != txdfe_init_para);
    KS_ASSERT((0 == ch_id)||(1 == ch_id));
	txdfe_bus_clk_disable(1);
    txdfe_clk_disable(0);
    sts = txdfe_up_sampling_rate_config(&txdfe_init_para->txdfe_filter.txdfe_up_sampling_rate);

    if(sts == KS_TXDFE_STATUS_FAIL)
    {
        return KS_TXDFE_STATUS_FAIL;
    }

    framing_clk_enable(ch_id);
    framing_reset_do(ch_id);
	framing_clk_enable(1);
    framing_reset_do(1);

    txdfe_bus_clk_enable(1);
    txdfe_clk_enable(ch_id);
    txdfe_reset_do(ch_id);
    txdfe_reset_do(1);
	
    txdfe_all_subsys_config(txdfe_init_para, ch_id);
	txdfe_all_subsys_config(txdfe_init_para, 1);
}


TXDFE_DRV_ITCM_CODE_SECTION
void txdfe_subsys_used_config(KS_TXDFE_INIT_PARA *txdfe_init_para, UINT8 u8_ch_id)
{
    txdfe_channel_config(&txdfe_init_para->txdfe_common.txdfe_channel, u8_ch_id);
    txdfe_clk_config(&txdfe_init_para->txdfe_common.txdfe_clk, u8_ch_id);
    txdfe_bif_config(&txdfe_init_para->txdfe_common.txdfe_bif, u8_ch_id);
    txdfe_fifo_out_config(&txdfe_init_para->txdfe_common.txdfe_fifo_out, u8_ch_id);

    txdfe_debug_config(&txdfe_init_para->txdfe_filter.txdfe_dbg, u8_ch_id);
    txdfe_scaling_config(&txdfe_init_para->txdfe_filter.txdfe_scaling, u8_ch_id);
    txdfe_ov_config(&txdfe_init_para->txdfe_filter.txdfe_ov, u8_ch_id);
    txdfe_fir0_config(&txdfe_init_para->txdfe_filter.txdfe_fir0, u8_ch_id);
}

TXDFE_DRV_ITCM_CODE_SECTION
void txdfe_start(SINT8 ch_id)
{
    txdfe_clk_disable_reset_enable(ch_id);
    txdfe_clk_enable_reset_disable(ch_id);

    framing_init(0);
	framing_init(1);
    ul_ofdm_framing_para_config(0);
	ul_ofdm_framing_para_config(1);

    txdfe_subsys_used_config(&g_txdfe_init_para, 0);
    txdfe_subsys_used_config(&g_txdfe_init_para, 1);
	
    txdfe_clk_enable_all(ch_id);
	txdfe_clk_enable_all(1);
}
#else
TXDFE_ITCM_CODE_SECTION
void txdfe_clk_enable_reset_disable(UINT8 ch_id)
{
    framing_clk_en(ch_id);
    ks_oal_delay((UINT32)20);
    framing_config_en(ch_id);
    txdfe_reset_disable(ch_id);
    ks_oal_delay((UINT32)20);
    SET_CPU_TXDFE_CLK_SET(1, ch_id);
}

TXDFE_ITCM_CODE_SECTION
void txdfe_clk_disable_reset_enable(UINT8 ch_id)
{
    SET_CPU_TXDFE_DBG_EN(0, ch_id);
    SET_CPU_TXDFE_CHANNEL_EN(0, ch_id);
    SET_CPU_TXDFE_CLK_SET(0, ch_id);

    txdfe_all_subsys_clk_disable();
    framing_clk_cls(ch_id);
    ks_oal_delay((UINT32)20);
    framing_config_cls(ch_id);
    txdfe_reset_enable(ch_id);
    ks_oal_delay(20);
}

TXDFE_ITCM_CODE_SECTION
eKS_TXDFE_STATUS txdfe_para_cfg_init(KS_TXDFE_INIT_PARA *txdfe_init_para, UINT8 ch_id)
{
    eKS_TXDFE_STATUS sts;

    KS_ASSERT(NULL != txdfe_init_para);
    KS_ASSERT((0 == ch_id)||(1 == ch_id));
	txdfe_bus_clk_disable(1);
    txdfe_clk_disable(0);
    sts = txdfe_up_sampling_rate_config(&txdfe_init_para->txdfe_filter.txdfe_up_sampling_rate);

    if(sts == KS_TXDFE_STATUS_FAIL)
    {
        return KS_TXDFE_STATUS_FAIL;
    }

    framing_clk_enable(ch_id);
    framing_reset_do(ch_id);
	//framing_clk_enable(1);
    //framing_reset_do(1);

    //txdfe_bus_clk_enable(1);
    txdfe_clk_enable(ch_id);
    txdfe_reset_do(ch_id);
    //txdfe_reset_do(1);
	
    txdfe_all_subsys_config(txdfe_init_para, ch_id);
	//txdfe_all_subsys_config(txdfe_init_para, 1);
}


TXDFE_DRV_ITCM_CODE_SECTION
void txdfe_subsys_used_config(KS_TXDFE_INIT_PARA *txdfe_init_para, UINT8 u8_ch_id)
{
    txdfe_channel_config(&txdfe_init_para->txdfe_common.txdfe_channel, u8_ch_id);
    txdfe_clk_config(&txdfe_init_para->txdfe_common.txdfe_clk, u8_ch_id);
    txdfe_bif_config(&txdfe_init_para->txdfe_common.txdfe_bif, u8_ch_id);
    txdfe_fifo_out_config(&txdfe_init_para->txdfe_common.txdfe_fifo_out, u8_ch_id);

    txdfe_debug_config(&txdfe_init_para->txdfe_filter.txdfe_dbg, u8_ch_id);
    txdfe_scaling_config(&txdfe_init_para->txdfe_filter.txdfe_scaling, u8_ch_id);
    txdfe_ov_config(&txdfe_init_para->txdfe_filter.txdfe_ov, u8_ch_id);
    txdfe_fir0_config(&txdfe_init_para->txdfe_filter.txdfe_fir0, u8_ch_id);
}

TXDFE_DRV_ITCM_CODE_SECTION
void txdfe_start(SINT8 ch_id)
{
    txdfe_clk_disable_reset_enable(ch_id);
    txdfe_clk_enable_reset_disable(ch_id);

    framing_init(ch_id);
    ul_ofdm_framing_para_config(ch_id);
    //framing_timing_update(96, 1, ch_id);

    //txdfe_all_subsys_config(&g_txdfe_init_para, ch_id);
    txdfe_subsys_used_config(&g_txdfe_init_para, ch_id);

    txdfe_clk_enable(ch_id);
}



#endif
TXDFE_DRV_ITCM_CODE_SECTION
void txdfe_init(SINT8 ch_id)
{
    KS_TXDFE_DMA_PARA txdfe0_dma_para;

    ks_txdfe_default_common_para_init(&g_txdfe_init_para.txdfe_common);
    ks_txdfe_default_filter_para_init(&g_txdfe_init_para.txdfe_filter);

    txdfe_up_sampling_rate_para_init(&g_txdfe_init_para.txdfe_filter.txdfe_up_sampling_rate);
    txdfe_para_init(&g_txdfe_init_para);

	#if MIMO_2ANTS
    //txdfe_up_sampling_rate_config(&g_txdfe_init_para.txdfe_filter.txdfe_up_sampling_rate);
    txdfe_para_cfg_init(&g_txdfe_init_para, ch_id);
    #else
    txdfe_up_sampling_rate_config(&g_txdfe_init_para.txdfe_filter.txdfe_up_sampling_rate);
    ks_txdfe_init(&g_txdfe_init_para, ch_id);
    #endif
    //txdfe_clk_enable(ch_id);

	#if TXDFE_DEBUG_EN
    txdfe0_dma_para.e_txdfe_dma_trans_mode             = KS_TXDFE_DMA_TRANS_LOOP; //KS_TXDFE_DMA_TRANS_LOOP; //KS_TXDFE_DMA_TRANS_SINGLE;
    txdfe0_dma_para.txdfe_dma_dbg_call_back_fun_handle = NULL; //txdfe0_dma_dbg_call_back;
    txdfe0_dma_para.txdfe_dma_dbg_call_back_fun_para   = NULL;
    txdfe0_dma_para.txdfe_dma_dbg_sample_buf           = (UINT32*)(0x11250000); //g_txdfe0_dma_dbg_sample_buf;
    txdfe0_dma_para.u32_txdfe_dma_dbg_sample_len       = 512 * 120; //unit samples

    ks_txdfe_dma_dbg_trans_register(&txdfe0_dma_para, 0);
	#endif
}


