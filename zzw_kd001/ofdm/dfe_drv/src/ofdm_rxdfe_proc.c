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
#define THIS_FILE_NAME_ID    CSMA_RXDFE_PROC_FILE_ID

/***********************************************************************************************************************
* INCLUDE FILES
***********************************************************************************************************************/
#include "ks_oal.h"
#include "ks_rxdfe.h"
#include "rxdfe_drv.h"
#include "rxdfe_reg.h"
#include "ks_rf_rffe.h"
#include "ks_rf_spi.h"
#include "ks_rf_ctrl.h"
#include "zzw_common.h"

/***********************************************************************************************************************
* GLOBAL VARIABLES DEFINITION
***********************************************************************************************************************/
RXDFE_DRV_DTCM_DATA_SECTION UINT32 u32_rst_dfe_delay = 100;
RXDFE_DRV_DTCM_DATA_SECTION UINT32 g_u32_rssi_cnt = 0;

RXDFE_DRV_DTCM_DATA_SECTION KS_RXDFE_INIT_PARA g_rxdfe_init_para = {{{(UINT32)0}}};

RXDFE_DRV_DTCM_DATA_SECTION UINT16 u16_rxdfe_rrc_fir0_ping_coef_real[33] = 
{
    #include "rxdfe_rrc_fir0_ping_coef_real.txt"
};

RXDFE_DRV_DTCM_DATA_SECTION UINT16 u16_rxdfe_rrc_fir0_ping_coef_imag[33] = 
{
    #include "rxdfe_rrc_fir0_ping_coef_imag.txt"
};

extern UINT16 g_zzw_freq_scan_flag;
extern UINT16 g_zzw_freq_scan_star;
extern KS_RXDFE_PATH0_PACK g_rxdfe_path0_pack[2];
extern KS_RXDFE_PATH1_PACK g_rxdfe_path1_pack[2];


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
RXDFE_DRV_ITCM_CODE_SECTION
VOID rxdfe_para_init(KS_RXDFE_INIT_PARA *rxdfe_init_para)
{
    rxdfe_init_para->rxdfe_common.rxdfe_adiu.u8_rxdfe_adiu_auto_init_mode = 0x53;
    rxdfe_init_para->rxdfe_common.rxdfe_adiu.b_rxdfe_adiu_mode = 1;

    rxdfe_init_para->rxdfe_common.rxdfe_channel.b_rxdfe_channel_en = 1;

    rxdfe_init_para->rxdfe_common.rxdfe_path0.rxdfe_path0_pack.u8_rxdfe_path0_pack_mode = 1;
    rxdfe_init_para->rxdfe_common.rxdfe_path0.rxdfe_path0_pack.u8_rxdfe_path0_pack_num  = 4;
    rxdfe_init_para->rxdfe_common.rxdfe_path0.rxdfe_path0_pack.u16_rxdfe_path0_pack_length = 0x100;

    rxdfe_init_para->rxdfe_common.rxdfe_path1.rxdfe_path1_pack.u8_rxdfe_path1_pack_mode = 1;
    rxdfe_init_para->rxdfe_common.rxdfe_path1.rxdfe_path1_pack.u8_rxdfe_path1_pack_num  = 4;
    rxdfe_init_para->rxdfe_common.rxdfe_path1.rxdfe_path1_pack.u16_rxdfe_path1_pack_length = 0x100;

    rxdfe_init_para->rxdfe_filter.rxdfe_dbg.e_rxdfe_dbg_point_sel = KS_RXDFE_DEBUG_POINT_SEL_RRC_FIR0_OUPUT;
    rxdfe_init_para->rxdfe_filter.rxdfe_dbg.b_rxdfe_dbg_mode = 0;
    rxdfe_init_para->rxdfe_filter.rxdfe_dbg.b_rxdfe_dbg_en = 1;

    rxdfe_init_para->rxdfe_int.u8_rxdfe_int_mask = 0xFF;

    rxdfe_init_para->rxdfe_filter.rxdfe_agc.b_rxdfe_agc_rssi_en = 1;
    rxdfe_init_para->rxdfe_filter.rxdfe_agc.u32_rxdfe_agc_rssi_dfe_dur = 24;
    rxdfe_init_para->rxdfe_filter.rxdfe_agc.u32_rxdfe_agc_rssi_dfe_delay = 4;
    rxdfe_init_para->rxdfe_filter.rxdfe_agc.u32_rxdfe_agc_rssi_dfe_length = 16;
    rxdfe_init_para->rxdfe_filter.rxdfe_agc.u32_rxdfe_agc_rssi_dfe_sample = 1;

    rxdfe_init_para->rxdfe_filter.rxdfe_iqmc.b_rxdfe_iqmc_en = 0;
    rxdfe_init_para->rxdfe_filter.rxdfe_iqmc.u16_rxdfe_iqmc_amp_dynamic = 0;
    rxdfe_init_para->rxdfe_filter.rxdfe_iqmc.u16_rxdfe_iqmc_alpha_dynamic = 0;

    rxdfe_init_para->rxdfe_filter.rxdfe_lpfa.b_rxdfe_lpfa_stage0_fir_en = 0;
    rxdfe_init_para->rxdfe_filter.rxdfe_lpfa.b_rxdfe_lpfa_stage1_fir_en = 0;
    rxdfe_init_para->rxdfe_filter.rxdfe_lpfa.b_rxdfe_lpfa_stage0_hbf_en = 0;
    rxdfe_init_para->rxdfe_filter.rxdfe_lpfa.b_rxdfe_lpfa_stage1_hbf_en = 0;

    rxdfe_init_para->rxdfe_filter.rxdfe_agc.b_rxdfe_agc_grp_en      = 0;
    rxdfe_init_para->rxdfe_filter.rxdfe_agc.b_rxdfe_agc_ping_mode   = 0;
    rxdfe_init_para->rxdfe_filter.rxdfe_agc.u8_rxdfe_agc_mode1_time = 0;

    rxdfe_init_para->rxdfe_filter.rxdfe_agc.u32_rxdfe_agc_mode0_delay  = 768;
    rxdfe_init_para->rxdfe_filter.rxdfe_agc.u16_rxdfe_agc_mode0_length = 1024;
    rxdfe_init_para->rxdfe_filter.rxdfe_agc.u32_rxdfe_agc_mode0_dur    = 7680;
    rxdfe_init_para->rxdfe_filter.rxdfe_agc.u8_rxdfe_agc_mode0_sample  = 4;
    rxdfe_init_para->rxdfe_filter.rxdfe_agc.u16_rxdfe_agc_backoff      = 2047;

    rxdfe_init_para->rxdfe_filter.rxdfe_agc.u8_rxdfe_agc_sat_count_thr = 255;
    rxdfe_init_para->rxdfe_filter.rxdfe_agc.u16_rxdfe_agc_sat_countmax = 2000;
    rxdfe_init_para->rxdfe_filter.rxdfe_agc.u16_rxdfe_agc_sat_thr      = 8191;

    rxdfe_init_para->rxdfe_filter.rxdfe_agc.u16_rxdfe_agc_mode1_dur     = 32768;
    rxdfe_init_para->rxdfe_filter.rxdfe_agc.u32_rxdfe_agc_mode1_delay   = 20;
    rxdfe_init_para->rxdfe_filter.rxdfe_agc.u16_rxdfe_agc_mode1_length1 = 16;
    rxdfe_init_para->rxdfe_filter.rxdfe_agc.u16_rxdfe_agc_mode1_length2 = 32;
    rxdfe_init_para->rxdfe_filter.rxdfe_agc.u16_rxdfe_agc_mode1_length3 = 64;
    rxdfe_init_para->rxdfe_filter.rxdfe_agc.u8_rxdfe_agc_mode1_dur1     = 96;
    rxdfe_init_para->rxdfe_filter.rxdfe_agc.u8_rxdfe_agc_mode1_dur2     = 180;
    rxdfe_init_para->rxdfe_filter.rxdfe_agc.u8_rxdfe_agc_mode1_dur3     = 200;
    rxdfe_init_para->rxdfe_filter.rxdfe_agc.u8_rxdfe_agc_mode1_sample   = 1;
    rxdfe_init_para->rxdfe_filter.rxdfe_agc.u16_rxdfe_agc_mode1_backoff = 8192;

    rxdfe_init_para->rxdfe_filter.rxdfe_agc.u8_rxdfe_agc_thld  = 2;
    rxdfe_init_para->rxdfe_filter.rxdfe_agc.u8_rxdfe_agc_fresh = 3;
    rxdfe_init_para->rxdfe_filter.rxdfe_agc.u8_rxdfe_agc_filt  = 4;

    rxdfe_init_para->rxdfe_filter.rxdfe_dcoc.b_rxdfe_dcoc_en       = 0;
    rxdfe_init_para->rxdfe_filter.rxdfe_dcoc.u16_rxdfe_dcoc_dur    = 32768;
    rxdfe_init_para->rxdfe_filter.rxdfe_dcoc.u8_rxdfe_dcoc_mode    = 1;
    rxdfe_init_para->rxdfe_filter.rxdfe_dcoc.u16_rxdfe_dcoc_delay  = 20;
    rxdfe_init_para->rxdfe_filter.rxdfe_dcoc.u16_rxdfe_dcoc_length = 128;

    rxdfe_init_para->rxdfe_filter.rxdfe_nco.b_rxdfe_nco_en = 0;
    rxdfe_init_para->rxdfe_filter.rxdfe_nco.u32_rxdfe_nco_step_dynamic = 0;
    rxdfe_init_para->rxdfe_filter.rxdfe_nco.u32_rxdfe_nco_init_phase_dynamic = 0;

    rxdfe_init_para->rxdfe_filter.rxdfe_lpfb.u8_rxdfe_cic_ratio = 0;
    rxdfe_init_para->rxdfe_filter.rxdfe_lpfb.b_rxdfe_lpfb_en = 0;

    rxdfe_init_para->rxdfe_filter.rxdfe_cord.b_rxdfe_cord_en = 0;
    rxdfe_init_para->rxdfe_filter.rxdfe_cord.u32_rxdfe_cord_step_dynamic = 0;
    rxdfe_init_para->rxdfe_filter.rxdfe_cord.u32_rxdfe_cord_init_phase_dynamic = 0;

    rxdfe_init_para->rxdfe_filter.rxdfe_lpfc.b_rxdfe_lpfc_stage0_fir_en = 0;
    rxdfe_init_para->rxdfe_filter.rxdfe_lpfc.b_rxdfe_lpfc_stage1_fir_en = 0;
    rxdfe_init_para->rxdfe_filter.rxdfe_lpfc.b_rxdfe_lpfc_stage2_fir_en = 0;
    rxdfe_init_para->rxdfe_filter.rxdfe_lpfc.b_rxdfe_lpfc_stage0_hbf_en = 0;
    rxdfe_init_para->rxdfe_filter.rxdfe_lpfc.b_rxdfe_lpfc_stage1_hbf_en = 0;
    rxdfe_init_para->rxdfe_filter.rxdfe_lpfc.b_rxdfe_lpfc_stage2_hbf_en = 0;
    rxdfe_init_para->rxdfe_filter.rxdfe_lpfc.b_rxdfe_lpfc_path1_sel     = 0;
    rxdfe_init_para->rxdfe_filter.rxdfe_lpfc.b_rxdfe_lpfc_path0_sel     = 0;

    rxdfe_init_para->rxdfe_filter.rxdfe_lpfd.b_rxdfe_lpfd_stage1_hbf_en = 0;
    rxdfe_init_para->rxdfe_filter.rxdfe_lpfd.b_rxdfe_lpfd_stage0_hbf_en = 0;
    rxdfe_init_para->rxdfe_filter.rxdfe_lpfd.b_rxdfe_lpfd_stage0_fir_en = 0;
    rxdfe_init_para->rxdfe_filter.rxdfe_lpfd.b_rxdfe_lpfd_stage1_fir_en = 0;

    rxdfe_init_para->rxdfe_filter.rxdfe_dagc2.b_rxdfe_dagc2_en        = 0;
    rxdfe_init_para->rxdfe_filter.rxdfe_dagc2.u32_rxdfe_dagc2_gain    = 0x8000;
    rxdfe_init_para->rxdfe_filter.rxdfe_dagc2.u32_rxdfe_dagc2_dur     = 1920;
    rxdfe_init_para->rxdfe_filter.rxdfe_dagc2.u8_rxdfe_dagc2_mode     = 1;
    rxdfe_init_para->rxdfe_filter.rxdfe_dagc2.u16_rxdfe_dagc2_delay   = 120;
    rxdfe_init_para->rxdfe_filter.rxdfe_dagc2.u16_rxdfe_dagc2_length  = 64;
    rxdfe_init_para->rxdfe_filter.rxdfe_dagc2.u16_rxdfe_dagc2_backoff = 1024;

    rxdfe_init_para->rxdfe_filter.rxdfe_rrc.b_rxdfe_rrc_fir0_en = 1;
    rxdfe_init_para->rxdfe_filter.rxdfe_rrc.b_rxdfe_rrc_fir1_en = 0;
    rxdfe_init_para->rxdfe_filter.rxdfe_rrc.b_rxdfe_rrc_fir0_coef_sel_dynamic = 0;
    rxdfe_init_para->rxdfe_filter.rxdfe_rrc.b_rxdfe_rrc_fir1_coef_sel_dynamic = 0;

    ks_oal_mem_copy(rxdfe_init_para->rxdfe_filter.rxdfe_rrc.u16_rxdfe_rrc_fir0_ping_coef_real, u16_rxdfe_rrc_fir0_ping_coef_real, 2 * 33);
    ks_oal_mem_copy(rxdfe_init_para->rxdfe_filter.rxdfe_rrc.u16_rxdfe_rrc_fir0_ping_coef_imag, u16_rxdfe_rrc_fir0_ping_coef_imag, 2 * 33);

    rxdfe_init_para->rxdfe_filter.rxdfe_dagc.b_rxdfe_dagc_en        = 0;
    rxdfe_init_para->rxdfe_filter.rxdfe_dagc.u32_rxdfe_dagc_gain    = 0x8000;
    rxdfe_init_para->rxdfe_filter.rxdfe_dagc.u32_rxdfe_dagc_dur     = 3840;
    rxdfe_init_para->rxdfe_filter.rxdfe_dagc.u8_rxdfe_dagc_mode     = 1;
    rxdfe_init_para->rxdfe_filter.rxdfe_dagc.u16_rxdfe_dagc_delay   = 200;
    rxdfe_init_para->rxdfe_filter.rxdfe_dagc.u16_rxdfe_dagc_length  = 128;
    rxdfe_init_para->rxdfe_filter.rxdfe_dagc.u16_rxdfe_dagc_backoff = 2000;
    #ifdef BAND_WIDTH_10M
    rxdfe_init_para->rxdfe_filter.rxdfe_hbfd.b_rxdfe_hbfd_stage0_hbf_en = 1;
    #else
    rxdfe_init_para->rxdfe_filter.rxdfe_hbfd.b_rxdfe_hbfd_stage0_hbf_en = 0;
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
RXDFE_DRV_ITCM_CODE_SECTION
VOID rxdfe_down_sample_rate_para_init(KS_RXDFE_DOWN_SAMPLE_RATE_PARA *rxdfe_down_sample_rate)
{
    rxdfe_down_sample_rate->e_clk_rxdfe_clk_data_input_rate
    = KS_RXDFE_CLK_INPUT_RATE_61440KHz_DATA_INPUT_RATE_30720KHz;

    #ifdef BAND_WIDTH_10M
    rxdfe_down_sample_rate->u32_clk_rxdfe_post_hbfd_sample_rate_times = 2;
    rxdfe_down_sample_rate->u32_clk_rxdfe_lpfa_stage0_sample_rate_times   = 2;
    rxdfe_down_sample_rate->u32_clk_rxdfe_lpfa_stage1_sample_rate_times   = 2;
    rxdfe_down_sample_rate->u32_clk_rxdfe_post_lpfa_out_sample_rate_times = 2;
    rxdfe_down_sample_rate->u32_clk_rxdfe_lpfb_sample_rate_times          = 2;
    rxdfe_down_sample_rate->u32_clk_rxdfe_post_lpfb_out_sample_rate_times = 2;
    rxdfe_down_sample_rate->u32_clk_rxdfe_lpfc_stage0_sample_rate_times   = 2;
    rxdfe_down_sample_rate->u32_clk_rxdfe_lpfc_stage1_sample_rate_times   = 2;
    rxdfe_down_sample_rate->u32_clk_rxdfe_lpfc_stage2_sample_rate_times   = 2;
    rxdfe_down_sample_rate->u32_clk_rxdfe_post_lpfc_out_sample_rate_times = 2;
    rxdfe_down_sample_rate->u32_clk_rxdfe_lpfd_0_sample_rate_times    = 2;
    rxdfe_down_sample_rate->u32_clk_rxdfe_hbfd_sample_rate_times      = 2;
    rxdfe_down_sample_rate->u32_clk_rxdfe_post_hbfd_sample_rate_times = 1;
	#else
    rxdfe_down_sample_rate->u32_clk_rxdfe_post_hbfd_sample_rate_times = 1;
    rxdfe_down_sample_rate->u32_clk_rxdfe_lpfa_stage0_sample_rate_times   = 1;
    rxdfe_down_sample_rate->u32_clk_rxdfe_lpfa_stage1_sample_rate_times   = 1;
    rxdfe_down_sample_rate->u32_clk_rxdfe_post_lpfa_out_sample_rate_times = 1;
    rxdfe_down_sample_rate->u32_clk_rxdfe_lpfb_sample_rate_times          = 1;
    rxdfe_down_sample_rate->u32_clk_rxdfe_post_lpfb_out_sample_rate_times = 1;
    rxdfe_down_sample_rate->u32_clk_rxdfe_lpfc_stage0_sample_rate_times   = 1;
    rxdfe_down_sample_rate->u32_clk_rxdfe_lpfc_stage1_sample_rate_times   = 1;
    rxdfe_down_sample_rate->u32_clk_rxdfe_lpfc_stage2_sample_rate_times   = 1;
    rxdfe_down_sample_rate->u32_clk_rxdfe_post_lpfc_out_sample_rate_times = 1;
    rxdfe_down_sample_rate->u32_clk_rxdfe_lpfd_0_sample_rate_times    = 1;
    rxdfe_down_sample_rate->u32_clk_rxdfe_hbfd_sample_rate_times      = 1;
    rxdfe_down_sample_rate->u32_clk_rxdfe_post_hbfd_sample_rate_times = 1;

	#endif

    rxdfe_down_sample_rate->u32_clk_rxdfe_lpfd_1_sample_rate_times = 1;
    rxdfe_down_sample_rate->u32_clk_rxdfe_dagc2_sample_rate_times  = 1;

    rxdfe_down_sample_rate->u32_clk_rxdfe_debug_sample_rate_times  = 1;
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
#if MIMO_2ANTS
RXDFE_ITCM_CODE_SECTION
void rxdfe_clk_rst(SINT8 ch_id)
{
    WRITE_REG32(0x103c0008, (READ_REG32(0x103c0008) & 0xF5FFFFFF));
    SET_CPU_RXDFE_AGC_CLK_GATE(0, ch_id);
	SET_CPU_RXDFE_AGC_CLK_GATE(0, 1);
    ks_oal_delay((UINT32)10);

    WRITE_REG32(0x10340010, (READ_REG32(0x10340010) | 0x00000003));
    ks_oal_delay(20);
    WRITE_REG32(0x10340010, (READ_REG32(0x10340010) & 0xFFFFFFFC));
    ks_oal_delay(20);
    WRITE_REG32(0x103c0008, (READ_REG32(0x103c0008) | 0xF000000));
    SET_CPU_RXDFE_AGC_CLK_GATE(1, ch_id);
	SET_CPU_RXDFE_AGC_CLK_GATE(1, 1);
    ks_oal_delay((UINT32)10);
}

#else
RXDFE_ITCM_CODE_SECTION
void rxdfe_clk_rst(SINT8 ch_id)
{
    WRITE_REG32(0x103c0008, (READ_REG32(0x103c0008) & 0xFDFFFFFF));
    SET_CPU_RXDFE_AGC_CLK_GATE(0, ch_id);
    ks_oal_delay((UINT32)10);

    WRITE_REG32(0x10340010, (READ_REG32(0x10340010) | 0x00000002));
    ks_oal_delay(20);
    WRITE_REG32(0x10340010, (READ_REG32(0x10340010) & 0xFFFFFFFD));
    ks_oal_delay(20);
    WRITE_REG32(0x103c0008, (READ_REG32(0x103c0008) | 0x3000000));
    SET_CPU_RXDFE_AGC_CLK_GATE(1, ch_id);
    ks_oal_delay((UINT32)10);
}
#endif
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
RXDFE_DRV_ITCM_CODE_SECTION
void rxdfe_start(SINT8 ch_id)
{
    rxdfe_clk_rst(ch_id);
    ks_oal_delay((UINT32)10);

    SET_CPU_RXDFE_CHANNEL_EN(0, ch_id);
    #if MIMO_2ANTS
	SET_CPU_RXDFE_CHANNEL_EN(0, 1);
	#endif
	if((1 == g_zzw_freq_scan_flag) && (0 == g_zzw_freq_scan_star))
	{
		g_rxdfe_init_para.rxdfe_filter.rxdfe_agc.u32_rxdfe_agc_rssi_dfe_dur = 7680;
        g_rxdfe_init_para.rxdfe_filter.rxdfe_agc.u32_rxdfe_agc_rssi_dfe_delay = 2048;
        g_rxdfe_init_para.rxdfe_filter.rxdfe_agc.u32_rxdfe_agc_rssi_dfe_length = 4096;
        g_rxdfe_init_para.rxdfe_filter.rxdfe_agc.u32_rxdfe_agc_rssi_dfe_sample = 1;

		g_u32_rssi_cnt = 0;
	}
	else
	{
        g_rxdfe_init_para.rxdfe_filter.rxdfe_agc.u32_rxdfe_agc_rssi_dfe_dur = 24;
        g_rxdfe_init_para.rxdfe_filter.rxdfe_agc.u32_rxdfe_agc_rssi_dfe_delay = 4;
        g_rxdfe_init_para.rxdfe_filter.rxdfe_agc.u32_rxdfe_agc_rssi_dfe_length = 16;
        g_rxdfe_init_para.rxdfe_filter.rxdfe_agc.u32_rxdfe_agc_rssi_dfe_sample = 1;    
	}
    rxdfe_all_subsys_config(&g_rxdfe_init_para, ch_id);
	rxdfe_all_subsys_config(&g_rxdfe_init_para, 1);
}

RXDFE_ITCM_CODE_SECTION
eKS_RXDFE_STATUS rxdfe_para_cfg_init(KS_RXDFE_INIT_PARA *rxdfe_init_para, UINT8 ch_id)
{
	eKS_RXDFE_STATUS sts;

	KS_ASSERT(NULL != rxdfe_init_para);
	KS_ASSERT((0 == ch_id)||(1 == ch_id));

	rxdfe_clk_disable_all(ch_id);
	sts = rxdfe_down_sampling_rate_config(&rxdfe_init_para->rxdfe_filter.rxdfe_down_sample_rate);
	if(sts == KS_RXDFE_STATUS_FAIL) {
		return KS_RXDFE_STATUS_FAIL;
	}

    rxdfe_clk_enable_all(ch_id);
	rxdfe_reset_do(ch_id);
	#if MIMO_2ANTS
	rxdfe_reset_do(1);
	#endif
	rxdfe_all_subsys_config(rxdfe_init_para, ch_id);
	#if MIMO_2ANTS
	rxdfe_all_subsys_config(rxdfe_init_para, 1);
	#endif
    ks_oal_mem_copy(&g_rxdfe_path0_pack[ch_id], &rxdfe_init_para->rxdfe_common.rxdfe_path0.rxdfe_path0_pack, sizeof(KS_RXDFE_PATH0_PACK));
    ks_oal_mem_copy(&g_rxdfe_path1_pack[ch_id], &rxdfe_init_para->rxdfe_common.rxdfe_path1.rxdfe_path1_pack, sizeof(KS_RXDFE_PATH1_PACK));
	ks_oal_mem_copy(&g_rxdfe_path0_pack[1], &rxdfe_init_para->rxdfe_common.rxdfe_path0.rxdfe_path0_pack, sizeof(KS_RXDFE_PATH0_PACK));
    ks_oal_mem_copy(&g_rxdfe_path1_pack[1], &rxdfe_init_para->rxdfe_common.rxdfe_path1.rxdfe_path1_pack, sizeof(KS_RXDFE_PATH1_PACK));
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
RXDFE_DRV_ITCM_CODE_SECTION
void rxdfe_init(SINT8 ch_id)
{
    ks_rxdfe_default_common_para_init(&g_rxdfe_init_para.rxdfe_common);
    ks_rxdfe_default_filter_para_init(&g_rxdfe_init_para.rxdfe_filter);

    rxdfe_para_init(&g_rxdfe_init_para);
    rxdfe_down_sample_rate_para_init(&g_rxdfe_init_para.rxdfe_filter.rxdfe_down_sample_rate);
    //rxdfe_down_sampling_rate_config(&g_rxdfe_init_para.rxdfe_filter.rxdfe_down_sample_rate);

    //rxdfe_clk_enable(ch_id);
	#if MIMO_2ANTS
    rxdfe_para_cfg_init(&g_rxdfe_init_para, ch_id);
    #else
    rxdfe_down_sampling_rate_config(&g_rxdfe_init_para.rxdfe_filter.rxdfe_down_sample_rate);

    //rxdfe_clk_enable(ch_id);
    ks_rxdfe_init(&g_rxdfe_init_para, ch_id);
    #endif
}
