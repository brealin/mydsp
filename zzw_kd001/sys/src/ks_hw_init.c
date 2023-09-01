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
* 1.0 2019-06-17 created by FSLI
***********************************************************************************************************************/
#undef THIS_FILE_NAME_ID
#define THIS_FILE_NAME_ID KS_HW_INIT_FILE_ID
        
/***********************************************************************************************************************
* INCLUDE FILES
***********************************************************************************************************************/
#include "ks_dmas.h"
#include "ks_hw_init.h"
#include "ks_oal.h"
#include "ks_it.h"
#include "ks_lib_variable.h"
#include "ks_pll.h"
#include "ks_spinlock.h"

#ifdef CORE_X1643
#include "ks_txdfe.h"
#include "ks_rxdfe.h"
#include "ks_framing.h"
#include "ks_rf_rffe.h"
#include "ks_rf_spi.h"
#include "ks_rf_ctrl.h"
#include "ks_turbo_encoder.h"
#include "Rxdfe_drv.h"
#include "Txdfe_drv.h"
#include "ks_gpio.h"
#else
#include "ks_turbo_decoder.h"
#endif

#include "zzw_common.h"
#include "ks_kd001_main.h"

/***********************************************************************************************************************
* EXTERN VARIABLES DECLARATION
***********************************************************************************************************************/
#define NOP      0
#define RX       1
#define TX       2
#define TMJ      3
DTCM_RF_DRV_DATA_SECTION UINT32 g_u32_frame_no = 0;
DMAC_LINK_INFO g_st_rxdfe_link_item = {0};

#ifdef CORE_X1643 
extern UINT8 g_zzw_slot_table[1024];
extern ZZW_DATA_PACKET_T g_zzw_sf_b;
extern ZZW_NODE_STATE_T g_zzw_node_status;
extern complex_s16_t g_mod_output[2][1920+96];
OAL_DTCM_DATA_SECTION oal_tmr_info_t g_u32_tmr_slp[2] = {{0}, {0}};
#endif
OAL_DTCM_DATA_SECTION UINT32 g_u32_imask_bak[4] = {0};
OAL_DTCM_DATA_SECTION UINT32 g_u32_slp_cnt_enter = 0;
OAL_DTCM_DATA_SECTION UINT32 g_u32_slp_cnt_exit = 0;

OAL_DTCM_DATA_SECTION UINT32 g_u32_osmhz_scaleus = 40;   //delay scale factor  for delayus
/***********************************************************************************************************************
* FUNCTION
*
* VOID oal_dmac_init(VOID)
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
VOID ks_oal_init_dmac(VOID)
{
#ifdef CORE_X1643
    WRITE_REG(0x103C0000, READ_REG(0x103c0000) | BIT11);//cgm_en_clk_dmac1
	ks_oal_delay(100);
    WRITE_REG32(0x1034000c, READ_REG32(0x1034000c)  & (~(0x1 << 1)));//clear dmac1 reset

    //WRITE_REG(0x103C0000, READ_REG(0x103c0000) | BIT10);//cgm_en_clk_dmac0
    //WRITE_REG32(0x1034000c, READ_REG32(0x1034000c)  & (~(0x1 << 0)));//clear dmac0 reset

    
    ks_dmac_init(KS_DMAC1_CP);
    ks_dmac_handle_ch_init(KS_DMAC1_CP);
    ks_oal_irq_register(OAL_IRQ_CP_DMAD_1, OAL_IRQ_LVL_1, ks_cp_dmac1_handler);
	
    WRITE_REG(0x103C0000, READ_REG(0x103c0000) | BIT10);//cgm_en_clk_dmac0
	ks_oal_delay(100);
    WRITE_REG32(0x1034000c, READ_REG32(0x1034000c)  & (~(0x1 << 0)));//clear dmac0 reset
    ks_dmac_init(KS_DMAC0_CP);

    ks_dmac_handle_ch_init(KS_DMAC0_CP);
    ks_oal_irq_register(OAL_IRQ_CP_DMAD_0, OAL_IRQ_LVL_1, ks_cp_dmac0_handler);
#ifndef CP1_X1643
    WRITE_REG(0x103C0000, READ_REG(0x103c0000) | BIT12);//cgm_en_clk_dmac2
	ks_oal_delay(100);
    WRITE_REG32(0x1034000c, READ_REG32(0x1034000c) & (~(0x1 << 2)) );//clear dmac2 reset
    ks_dmac_init(KS_DMAC2_CP);
    ks_dmac_handle_ch_init(KS_DMAC2_CP);
    ks_oal_irq_register(OAL_IRQ_CP_DMAD_2, OAL_IRQ_LVL_1, ks_cp_dmac2_handler);
#endif

#else
    #if 0
    WRITE_REG(0x103C0000, READ_REG(0x103c0000) | BIT10);//cgm_en_clk_dmac0
	ks_oal_delay(100);
    WRITE_REG32(0x1034000c, READ_REG32(0x1034000c)  & (~(0x1 << 0)));//clear dmac0 reset
    ks_dmac_init(KS_DMAC0_CP);
    #endif
    ks_dmac_handle_ch_init(KS_DMAC0_CP);
    ks_oal_irq_register(OAL_IRQ_CP_DMAD_0, OAL_IRQ_LVL_1, ks_cp_dmac0_handler);
#endif
}

OAL_ITCM_CODE_SECTION
VOID ks_oal_set_clk(UINT32 u32_clk)
{
    eKS_PLL_MOD_SRC clk_src_core = (eKS_PLL_MOD_SRC)0;
    eKS_PLL_MOD_SRC clk_src_axi = (eKS_PLL_MOD_SRC)0;
    eKS_PLL_MOD_SRC clk_src_apb = (eKS_PLL_MOD_SRC)0;
    eKS_PLL_MODULE  pll_mod_core = (eKS_PLL_MODULE)0;
    eKS_PLL_MODULE  pll_mod_axi =  (eKS_PLL_MODULE)0;
    eKS_PLL_MODULE  pll_mod_apb = (eKS_PLL_MODULE)0;
    
    UINT32 osclk_tm = u32_clk/MHZ;
	  g_u32_osmhz_scaleus = osclk_tm/12;   //osclk_tm/12==40;

    #if ((!defined(CP1_X1643)) && (!defined(CP1_XC4500)))
    pll_mod_core = KS_PLL_MOD_CP0_CORE;
    pll_mod_axi = KS_PLL_MOD_CP0_AXI;
    pll_mod_apb = KS_PLL_MOD_CP0_PERI_APB;        
    #else
    pll_mod_core = KS_PLL_MOD_CP1_CORE;
    pll_mod_axi = KS_PLL_MOD_CP1_AXI;
    pll_mod_apb = KS_PLL_MOD_CP1_PERI_APB;       
    #endif

    if(u32_clk <= 24 * MHZ)
    {
        #if ((!defined(CP1_X1643)) && (!defined(CP1_XC4500)))
        clk_src_core = KS_PLL_MOD_CP0_CORE_CLK_SRC_XTL;
        clk_src_axi = KS_PLL_MOD_CP0_AXI_CLK_SRC_XTL;
        clk_src_apb = KS_PLL_MOD_CP0_PERI_APB_CLK_SRC_XTL;        
        #else
        clk_src_core = KS_PLL_MOD_CP1_CORE_CLK_SRC_XTL;
        clk_src_axi = KS_PLL_MOD_CP1_AXI_CLK_SRC_XTL;
        clk_src_apb = KS_PLL_MOD_CP1_PERI_APB_CLK_SRC_XTL;        
        #endif
    }
    else
    {
        #if ((!defined(CP1_X1643)) && (!defined(CP1_XC4500)))
        clk_src_core = KS_PLL_MOD_CP0_CORE_CLK_SRC_CP0PLL0_PA_DIV2;
        clk_src_axi = KS_PLL_MOD_CP0_AXI_CLK_SRC_CP0PLL0_PA_DIV2;
        clk_src_apb = KS_PLL_MOD_CP0_PERI_APB_CLK_SRC_CP0PLL0_PA_DIV4;        
        #else
        clk_src_core = KS_PLL_MOD_CP1_CORE_CLK_SRC_CP1PLL0_PA_DIV2;
        clk_src_axi = KS_PLL_MOD_CP1_AXI_CLK_SRC_CP1PLL0_PA_DIV2;
        clk_src_apb = KS_PLL_MOD_CP1_PERI_APB_CLK_SRC_CP1PLL0_PA_DIV4;        
        #endif
    }
    // set cp0/cp1 to 480M, axi/schedule clock source 240M,apb default 60M should be set as 120M
    ks_pll_mod_sel_src(pll_mod_core, clk_src_core);       // cpsys0 core: 480M
    ks_pll_set_mod_clk(pll_mod_core, u32_clk);
    //OAL_ASSERT(u32_clk == ks_pll_get_mod_clk(pll_mod_core),"CORE clk not right");

    //ks_pll_mod_sel_src(pll_mod_axi, clk_src_axi);        // cpsys0 axi: 240M, default div = 1/2,default value = core clk/2;
    //ks_pll_set_mod_clk(pll_mod_axi, (u32_clk >> 1));
    //OAL_ASSERT( (u32_clk >> 1) == ks_pll_get_mod_clk(pll_mod_axi),"AXI clk not right");

    ks_pll_mod_sel_src(pll_mod_apb, clk_src_apb);   // cpsys0 apb clk src 240M, default div = 1/4 = 60M
    ks_pll_set_mod_clk(pll_mod_apb, (u32_clk >> 2));
    //OAL_ASSERT( (u32_clk >> 2) == ks_pll_get_mod_clk(pll_mod_apb) ,"APB clk not right");


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
VOID ks_oal_init_clk()
{
	/*init dsp pll*/
	ks_pll_init();

    ks_oal_set_clk(480 * MHZ);

#if 0
    #ifdef CORE_X1643
    WRITE_REG32(0x103c0000, 0x030000f2);//int 0、1、2、3,core
    #else
    WRITE_REG32(0x103c0000, 0x030000f1);//int 0、1,2,3,core
    #endif

#else
    WRITE_REG32(0x103c0000, 0x030000f3);//int 0、1,2,3,core
#endif

    WRITE_REG32(0x1034000c, 0xFFFFFE1F);//release int 0、1、2、3

    WRITE_REG32(0x10340000, 0xe000);



    //WRITE_REG32(0x10340008, 0x0);
    //WRITE_REG32(0x1034000c, 0xc0000000);
    //WRITE_REG32(0x10340010, 0x0);
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
VOID ks_oal_init_mailbox(eKS_CPU_ID mb_id,eKS_CPU_ID src_id,eKS_CPU_ID dst_id)
{
    ks_mailbox_init(mb_id);
    ks_mailbox_send_intr_enable(mb_id,dst_id,SEND_TO_CORE,KS_ACT_DISABLE);
    ks_mailbox_recv_intr_enable(mb_id,src_id,RECV_FIFO_NOT_EMPTY,KS_ACT_ENABLE);
}
OAL_ITCM_CODE_SECTION
VOID timer0_ticks()
{
    KS_LOG(0, 0xD030 , (UINT16)(ks_timer_get_counter(KS_CP0_TIMER0)));  
    ks_timer_clear_intr(KS_CP0_TIMER0);
    if(g_u32_extend_ini_flg)
    {
        ks_slp_exit();
    }
    //ks_timer_deinit(eTimer);
    //ks_timer_stop(KS_CP0_TIMER0);
    __asm__("dint");
    *(volatile unsigned int *)(ICTL0_IRQ_INTMASK_L) &= ~(unsigned int)(0x1 << OAL_IRQ_CP_TIMER_0); //unmask ictl int
    __asm__("eint");
}
#ifdef CORE_X1643

OAL_ITCM_CODE_SECTION
VOID timer0_rf_ctrl_restore()
{
    KS_LOG(0, 0xD030 , (UINT16)(ks_timer_get_counter(KS_CP0_TIMER0)));  
    ks_timer_clear_intr(KS_CP0_TIMER0);

    rf_ctrl_enable();
    
    __asm__("dint");
    *(volatile unsigned int *)(ICTL0_IRQ_INTMASK_L) &= ~(unsigned int)(0x1 << OAL_IRQ_CP_TIMER_0); //unmask ictl int
    __asm__("eint");
}
#endif

#ifdef CORE_X1643
#ifdef KD001_RF8242
extern void rf_drv_init8242();
#endif
#endif
OAL_ITCM_CODE_SECTION
VOID timer1_ticks()
{
    KS_LOG(0, 0xD031 , (UINT16)(ks_timer_get_counter(KS_CP0_TIMER1)));  
    ks_timer_clear_intr(KS_CP0_TIMER1);

#ifdef CORE_X1643
//#ifdef RF_8242
#ifdef KD001_RF8242
	rf_drv_init8242();
#endif
#endif

    __asm__("dint");
    *(volatile unsigned int *)(ICTL0_IRQ_INTMASK_L) &= ~(unsigned int)(0x1 << OAL_IRQ_CP_TIMER_1); //unmask ictl int
    __asm__("eint");
}
OAL_ITCM_CODE_SECTION
VOID timer2_ticks()
{
    KS_LOG(0, 0xD032 , (UINT16)(ks_timer_get_counter(KS_CP0_TIMER2)));  
    ks_timer_clear_intr(KS_CP0_TIMER2);

    __asm__("dint");
    *(volatile unsigned int *)(ICTL0_IRQ_INTMASK_L) &= ~(unsigned int)(0x1 << OAL_IRQ_CP_TIMER_2); //unmask ictl int
    __asm__("eint");
}
OAL_ITCM_CODE_SECTION
VOID timer3_ticks()
{
    KS_LOG(0, 0xD033 , (UINT16)(ks_timer_get_counter(KS_CP0_TIMER3)));  
    ks_timer_clear_intr(KS_CP0_TIMER3);

    __asm__("dint");
    *(volatile unsigned int *)(ICTL0_IRQ_INTMASK_L) &= ~(unsigned int)(0x1 << OAL_IRQ_CP_TIMER_3); //unmask ictl int
    __asm__("eint");
}


OAL_ITCM_CODE_SECTION
//VOID ks_oal_tmr_init( eKS_TIMER eTimer, UINT32 period, UINT16 mode, UINT16 int_mask,  VOID_PTR isr)
VOID ks_oal_tmr_init( eKS_TIMER eTimer, UINT32 period, UINT16 mode,  UINT16 int_mask, eKS_PLL_MOD_SRC tmr_clk, VOID_PTR isr)

{
    KS_TIMER_PARAM test_timer_param;
    OAL_IRQ_ID e_irq_id = OAL_IRQ_CP_TIMER_0;
    eKS_PLL_MODULE pll = KS_PLL_MOD_CP0_TIMER0;
    //UINT16 int_mask = (UINT16)((mode >> 16) & 0xFFFF); 

    test_timer_param.nPeriod = period;///*配置timer计时周期, 实际周期是period*clk*/
    test_timer_param.nMode = (UINT16)(mode & 0xFFFF);
    test_timer_param.nIntr = int_mask;

    pll += (eTimer - KS_CP0_TIMER0);

    ks_pll_mod_sel_src(pll , tmr_clk);//KS_PLL_MOD_CP0_TIMER1_CLK_SRC_RTC_32K

    if(!int_mask)
    {
        e_irq_id += (eTimer - KS_CP0_TIMER0); 
        ks_oal_irq_register(e_irq_id, OAL_IRQ_LVL_2, (OAL_IRQ_FP)isr);
    }
    // deinit timer 0-7
    ks_timer_deinit(eTimer);
    // init timer0-7
    ks_timer_init(eTimer, &test_timer_param);
    // enable timer0-7
    ks_timer_start(eTimer);
    
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
#ifdef KD001_RF8242
extern UINT16 hmc7043_cfg[][4];
#endif
OAL_ITCM_CODE_SECTION
VOID ks_oal_init_bsp()
{
	OAL_IRQ_SAVE_AREA;
#ifdef CORE_X1643
    KS_UART_PARAM kup;
    VOID_PTR handle;
	OAL_IRQ_DISABLE_ALL;
    // configure uart
    kup.nBaudrate = 115200;//1500000;
    kup.nDataBits = KS_UART_PRM_DATA_BITS_8;
    kup.nStopBits = KS_UART_PRM_STOP_BITS_1;
    kup.nParity   = KS_UART_PRM_PARITY_NONE;
    kup.nHWFlow   = KS_UART_PRM_HWFLOW_NONE;

    // Register Verify
    #if defined(CP1_X1643)
    ks_uart_init(KS_CP1_UART0);
    ks_uart_init(KS_CP1_UART1);

    handle = ks_uart_open(KS_CP1_UART0, &kup);
    handle = ks_uart_open(KS_CP1_UART1, &kup);

    ks_dmas_init(KS_DMAS_CP1);
    ks_oal_irq_register(OAL_IRQ_CP_DMAS, OAL_IRQ_LVL_2, ks_cp1_dmas_handler);
    ks_oal_irq_register(OAL_IRQ_IPI_X1643_7, OAL_IRQ_LVL_2, ks_oal_ipi_handle);
    ks_oal_init_mailbox(KS_CPU_CP1_X1643, KS_CPU_CP1_X1643, KS_CPU_CPCPU1);

    #else
    ks_uart_init(KS_CP0_UART0);
    ks_uart_init(KS_CP0_UART1);

    handle = ks_uart_open(KS_CP0_UART0, &kup);
    handle = ks_uart_open(KS_CP0_UART1, &kup);

    ks_dmas_init(KS_DMAS_CP0);
    ks_oal_irq_register(OAL_IRQ_CP_DMAS, OAL_IRQ_LVL_2, ks_cp0_dmas_handler);
    ks_oal_irq_register(OAL_IRQ_IPI_X1643_5, OAL_IRQ_LVL_2, ks_oal_ipi_handle);

    ks_oal_init_mailbox(KS_CPU_CP0_X1643, KS_CPU_CP0_X1643, KS_CPU_CPCPU0);

    #endif
    ks_oal_irq_register(OAL_IRQ_CP_MAILBOX_X1643_OUT, OAL_IRQ_LVL_2, ks_oal_mailbox_handle);
	//@Zhemin, not use mailbox interrupt @X1643, let mailbox silence.
	#if defined(CORE_X1643)
	//ks_oal_set_reg(0x17500000, 17, 17, 0);
	#endif

    //32k tmr init
    //ks_oal_tmr_init( KS_CP0_TIMER1, 0xFFFFFFFF, (UINT16)KS_TIMER_PRM_MODE_USER_DEFINED, (UINT16)KS_TIMER_PRM_INTR_MASK, NULL_PTR);
    //24m tmr init
    ks_oal_tmr_init( KS_CP0_TIMER2, 0xFFFFFFFF, (UINT16)KS_TIMER_PRM_MODE_USER_DEFINED, (UINT16)KS_TIMER_PRM_INTR_MASK,\
        KS_PLL_MOD_CP0_TIMER2_CLK_SRC_XTL, NULL_PTR);
    //heart beats tmr init , 1 second beats once
    ks_oal_tmr_init( KS_CP0_TIMER1, 24000*1000, (UINT16)KS_TIMER_PRM_MODE_USER_DEFINED, (UINT16)KS_TIMER_PRM_INTR_NOTMASK,\
         KS_PLL_MOD_CP0_TIMER1_CLK_SRC_XTL,timer1_ticks);

	WRITE_REG32(0x103C0010, READ_REG32(0x103C0010) | (BIT1));//spi_clk en
	WRITE_REG32(0x103C0010, READ_REG32(0x103C0010) | (BIT0));//spi_pclk en

	// init 7043
	#ifdef KD001_RF8242
	WRITE_REG32(0x1724FFF8 , 0);
	rf_spi_init(KS_SPI_NOR, 0, 3, KS_RF_CXPS8242);
	rf_spi_config(DD_SPI1, 162, KS_HMC7043, hmc7043_cfg);
	ks_oal_delay(1000*4000);
	rf_spi_tx_num_config(0x1);	
	#endif
	
	
#else
	OAL_IRQ_DISABLE_ALL;
    #if defined(CP1_XC4500)
    ks_oal_irq_register(OAL_IRQ_CP_MAILBOX_XC4500_OUT, OAL_IRQ_LVL_2, ks_oal_mailbox_handle);
    ks_oal_init_mailbox(KS_CPU_CP1_XC4500, KS_CPU_CP1_XC4500, KS_CPU_CPCPU1);
    ks_oal_irq_register(OAL_IRQ_IPI_XC4500_6, OAL_IRQ_LVL_2, ks_oal_ipi_handle);
    #else
    ks_oal_irq_register(OAL_IRQ_CP_MAILBOX_XC4500_OUT, OAL_IRQ_LVL_2, ks_oal_mailbox_handle);
    ks_oal_init_mailbox(KS_CPU_CP0_XC4500, KS_CPU_CP0_XC4500, KS_CPU_CPCPU0);
    ks_oal_irq_register(OAL_IRQ_IPI_XC4500_4, OAL_IRQ_LVL_2, ks_oal_ipi_handle);
    #endif

    //heart beats tmr init , 1 second beats once
    ks_oal_tmr_init( KS_CP0_TIMER3, 24000*1000, (UINT16)KS_TIMER_PRM_MODE_USER_DEFINED, (UINT16)KS_TIMER_PRM_INTR_NOTMASK, \
        KS_PLL_MOD_CP0_TIMER3_CLK_SRC_XTL, timer3_ticks);

#endif
    ks_ipi_init();
    ks_oal_init_dmac();
    ks_spinlock_init();
	OAL_IRQ_RESTORE;
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
VOID ks_oal_init_trace(VOID)
{
    oal_trace_log_ctrl.u32_unit = 1;
	ks_trace_log_lev_type_config(OAL_TRACE_LEVEL_3,OAL_LOG_ENABLE);
    if (g_trace_type_select == OAL_LOG_ENABLE)
    {
        oal_trace_log_ctrl.u32_unit = 4;
        g_trace_log_buffer->log_trace_buffer[0] = 0xFFFFFFFF;
        #if (KS_LOG_DMA&&(!PRINT_LOG_BYTE))
        g_trace_log_buf_internal[0][0] = 0xFFFFFFFF;
        ks_trace_dma_init();
		g_u32_log_en = 1;
        #endif
        oal_trace_log_ctrl.u32_write_real_offset = 1;
        oal_trace_log_ctrl.u32_write_real_count = 4;
    }
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
VOID ks_oal_init_var(VOID)
{	
#ifdef CORE_X1643
	#ifdef TX_TEST

    ks_oal_mem_set(g_zzw_slot_table, 0x22, sizeof(g_zzw_slot_table));

	#if 1
	g_zzw_slot_table[0] = 0x02;
	g_zzw_slot_table[256] = 0x02;
	g_zzw_slot_table[512] = 0x02;
	g_zzw_slot_table[768] = 0x02;
	#endif

    //g_zzw_slot_table[0] = 0x42;
	#else
    #if 0
    ks_oal_mem_set(g_zzw_slot_table, 0x25, sizeof(g_zzw_slot_table));
	g_zzw_slot_table[0] = 0x24;
    //ks_oal_mem_set(g_u32_mod_out_data, 0x00, sizeof(g_u32_mod_out_data));
    #else
    ks_oal_mem_set(g_zzw_slot_table, 0x00, sizeof(g_zzw_slot_table));
    #endif
    //ks_oal_mem_set(g_zzw_slot_table, 0x22, sizeof(g_zzw_slot_table));
    //ks_oal_mem_set(g_mod_output, 0x00, sizeof(g_mod_output));
    //ks_oal_mem_set(g_u8_power_bmp_status,0x00,sizeof(g_u8_power_bmp_status));
	ks_oal_mem_set((UINT8*)OAL_POWER_BMP_STATUS_ADDR,0x00,sizeof(g_u8_power_bmp_status));	
	#endif
    ks_oal_mem_set(&g_zzw_sf_b, 0xA5, sizeof(g_zzw_sf_b));
    ks_oal_mem_set(&g_zzw_node_status, 0x00, sizeof(g_zzw_node_status));
	ks_oal_mem_set((void*)&g_st_iq_dump, 0x0, 20);  // only initiate first 5 dwords.
	g_st_iq_dump.dmac_ch_id = DMA_CHAN_FOR_IQ_DUMP;
	g_st_iq_dump.dmac_ch_id_ant2 = DMA_CHAN_FOR_IQ_DUMP_ANT2;
	g_st_iq_dump.dmac_lli_len = IQ_DUMP_DMAC_LLI_LEN;
	g_st_iq_dump.dmac_frame_len = IQ_DUMP_FRAME;
	g_st_iq_dump.rx_debug_info_addr = RX_DEBUG_INFO_ADDR;
	ks_oal_mem_set((VOID_PTR)SRAM_1643_4500_ADDR, 0x00, sizeof(sram_1643_4500_t));
	
#else
	#ifdef DSP_JTAG_IDE_SUPPORT
	//ks_oal_mem_set(g_u8_power_bmp_status,0x00,UINT8*DSP_POWER_DOMAIN_NUM);
	ks_oal_mem_set((UINT8*)OAL_POWER_BMP_STATUS_ADDR,0x00,sizeof(UINT8)*DSP_POWER_DOMAIN_NUM);
	#endif
#endif

    ks_oal_irq_init();

    ks_oal_sema_init();

#if OAL_MSG_QUEUE
    ks_oal_msg_init();
#endif    

    ks_oal_init_stack();

    ks_oal_init_heap();

    ks_oal_init_trace();

    ks_oal_assert_dump_addr_init();    

    ks_oal_init_mb_info();

    g_u32_boot_flg = 1; //init here to avoid dsp boot from zero addr twice
    
	ks_oal_dsp_pwr_state_set(1, 0);//clear pwr status
}

OAL_ITCM_CODE_SECTION
VOID ks_oal_dsp_pwr_state_set(UINT32 u32_src, UINT32 u32_state)
{
	/*u32_src : 0, CM4; 1, AP*/
    VOLATILE_UINT32_PTR vu32_cpu_status_ptr;

#ifdef CORE_X1643
	if(cp1_x1643_def_flag == 1)
	{			 
		vu32_cpu_status_ptr = (VOLATILE_UINT32_PTR)(KS_SYS_STATUS_ADDR(KS_CPU_CP1_X1643) + 0x400000);
	}
	else
	{
		vu32_cpu_status_ptr = (VOLATILE_UINT32_PTR)(KS_SYS_STATUS_ADDR(KS_CPU_CP0_X1643));		  
	}
#else
	if(cp1_xc4500_def_flag == 1)
	{			 
		vu32_cpu_status_ptr = (VOLATILE_UINT32_PTR)(KS_SYS_STATUS_ADDR(KS_CPU_CP1_XC4500) + 0x400000); 	   
	}
	else
	{
		vu32_cpu_status_ptr = (VOLATILE_UINT32_PTR)(KS_SYS_STATUS_ADDR(KS_CPU_CP0_XC4500));    
	}
	#endif
	if(u32_src)
	{
		vu32_cpu_status_ptr += 8;
	}
	*vu32_cpu_status_ptr = u32_state;  
}

OAL_ITCM_CODE_SECTION
VOID ks_slp_exit(VOID)
{
	OAL_IRQ_SAVE_AREA;

	if((DSP_DEEPSLEEP != g_enum_dsp_state) && (DSP_INIT!= g_enum_dsp_state))
	{
		return;
	}

	WRITE_REG32(ICTL0_IRQ_INTEN_L, g_u32_imask_bak[0]); // leave mailbox for wakeup
	WRITE_REG32(ICTL0_IRQ_INTEN_H, g_u32_imask_bak[1]);
	WRITE_REG32(ICTL1_IRQ_INTEN_L, g_u32_imask_bak[2]);
	WRITE_REG32(ICTL1_IRQ_INTEN_H, g_u32_imask_bak[3]);	

	WRITE_REG32(ICTL0_IRQ_INTMASK_L, ~(g_u32_imask_bak[0])); // leave mailbox for wakeup
	WRITE_REG32(ICTL0_IRQ_INTMASK_H, ~(g_u32_imask_bak[1]));
	WRITE_REG32(ICTL1_IRQ_INTMASK_L, ~(g_u32_imask_bak[2]));
	WRITE_REG32(ICTL1_IRQ_INTMASK_H, ~(g_u32_imask_bak[3]));	

	OAL_IRQ_DISABLE_ALL;
	#ifdef CORE_X1643
    //ks_oal_set_clk(480 * MHZ);
	ks_oal_drv_init_1643();
    //heart beats tmr init , 1 second beats once
    ks_oal_tmr_init( KS_CP0_TIMER1, 24000*1000, (UINT16)KS_TIMER_PRM_MODE_USER_DEFINED, (UINT16)KS_TIMER_PRM_INTR_NOTMASK,\
        KS_PLL_MOD_CP0_TIMER1_CLK_SRC_XTL,timer1_ticks);	
	#else
    //heart beats tmr init , 1 second beats once
    ks_oal_tmr_init( KS_CP0_TIMER3, 24000*1000, (UINT16)KS_TIMER_PRM_MODE_USER_DEFINED, (UINT16)KS_TIMER_PRM_INTR_NOTMASK,\
        KS_PLL_MOD_CP0_TIMER3_CLK_SRC_XTL,timer3_ticks);
	#endif


	ks_oal_dsp_pwr_state_set(0, KS_SYS_STATUS_NORMAL);   

    g_u32_slp_cnt_exit += 1;

	OAL_IRQ_RESTORE;

}

OAL_ITCM_CODE_SECTION
VOID ks_oal_dsp_state_set(OAL_DSP_STATE_E e_new_state)
{
	OAL_DSP_STATE_E e_old_state;
    OAL_IRQ_SAVE_AREA;

	OAL_IRQ_DISABLE_ALL;
	e_old_state = g_enum_dsp_state;
	g_enum_dsp_state = e_new_state;
	
    KS_LOG(OAL_TRACE_LEVEL_0,0xD220,(UINT16)(((UINT16)e_new_state << 8)|((UINT16)e_old_state)));	
	

	OAL_IRQ_RESTORE;

}




#ifdef CORE_X1643
//extern void rf_drv_cx9261_init();
extern DMAC_LINK_INFO g_st_tce_link_item;

OAL_ITCM_CODE_SECTION
VOID ks_slp_time_store(UINT32 u32_idx)
{
    UINT32 u32_tmr_cnt_32k;
    UINT32 u32_tmr_cnt_24m;
    OAL_IRQ_SAVE_AREA;

	OAL_IRQ_DISABLE_ALL;

    u32_tmr_cnt_24m = ks_timer_get_counter(KS_CP0_TIMER2);
    u32_tmr_cnt_32k = ks_timer_get_counter(KS_CP0_TIMER0);
	
	g_u32_tmr_slp[u32_idx].u32_tmr_32k = u32_tmr_cnt_32k;
	g_u32_tmr_slp[u32_idx].u32_tmr_24m = u32_tmr_cnt_24m;

    KS_LOG(OAL_TRACE_LEVEL_0,0xD221,(UINT16)(u32_tmr_cnt_32k));	
    KS_LOG(OAL_TRACE_LEVEL_0,0xD221,(UINT16)(u32_tmr_cnt_32k>>16));
    KS_LOG(OAL_TRACE_LEVEL_0,0xD222,(UINT16)(u32_tmr_cnt_24m));	
    KS_LOG(OAL_TRACE_LEVEL_0,0xD222,(UINT16)(u32_tmr_cnt_24m>>16));
    
	OAL_IRQ_RESTORE;

}
extern UINT32 g_u32_frame_no;

#define DSP_SLOT_POSITION_RTC   ((15360 >> 3))
#define DSP_SLEEP_PROTECT_RTC   (DSP_SLOT_POSITION_RTC + 10000)



OAL_ITCM_CODE_SECTION
VOID ks_slp_time_get(VOID)
{
    UINT32 u32_tmr_32k_b;//before 32k
    UINT32 u32_tmr_24m_b;//before 24m
    UINT32 u32_tmr_32k_a;//after 32k
    UINT32 u32_tmr_24m_a;//after 24m
    //UINT32 u32_delta_32k = 0;
    UINT64 u64_delta_24m = 0;
    UINT32 u32_delta_24m_l = 0;
    UINT32 u32_delta_24m_h = 0;

    UINT32 u32_delta_fn = 0;
    UINT32 u32_delta_rtc = 0;    
    //UINT32 u32_ratio = ((24 * 1000 * 1000) >> 15); //24m : 32k = 732.421875
    OAL_IRQ_SAVE_AREA;

	OAL_IRQ_DISABLE_ALL;

    u32_tmr_32k_b = g_u32_tmr_slp[0].u32_tmr_32k;
    u32_tmr_24m_b = g_u32_tmr_slp[0].u32_tmr_24m;
    
    ks_slp_time_store(1); //store the first rf sch int tmr after sleep

    u32_tmr_32k_a = g_u32_tmr_slp[1].u32_tmr_32k;
    u32_tmr_24m_a = g_u32_tmr_slp[1].u32_tmr_24m;

    //assume 32k never return to 0
    //u32_delta_32k = (u32_tmr_32k_b - u32_tmr_32k_a);
    
    u64_delta_24m = (((UINT64)((((UINT64)u32_tmr_32k_b - (UINT64)u32_tmr_32k_a) + ((UINT64)1 << 32)) & 0xFFFFFFFF) * (24 * 1000 * 1000)) >> 15);
    u32_delta_24m_l = (UINT32)((u64_delta_24m) & 0xFFFFFFFF);
    u32_delta_24m_h = (UINT32)(u64_delta_24m >> 32);
    u64_delta_24m = ((u64_delta_24m >> 32) << 32);

    if(u32_delta_24m_h)
    {
        //32K may not accurate
        if(u32_delta_24m_l < (0x02000000 * u32_delta_24m_h))
        {
            if(u32_tmr_32k_b >= u32_tmr_32k_a)
            {
                if(u32_tmr_32k_b - u32_tmr_32k_a > 0x7FFFFFFF)
                {
                    u64_delta_24m -= ((UINT64)1 << 32);
                }
            }
            else
            {
                if(u32_tmr_32k_a - u32_tmr_32k_b > 0x7FFFFFFF)
                {
                    u64_delta_24m -= ((UINT64)1 << 32);
                }
            }
        }
        else if(u32_delta_24m_l > (0xFFFFFFFF - 0x02000000 * u32_delta_24m_h))
        {
            if(u32_tmr_32k_b >= u32_tmr_32k_a)
            {
                if(u32_tmr_32k_b - u32_tmr_32k_a < 0x7FFFFFFF)
                {
                    u64_delta_24m += ((UINT64)1 << 32);
                }
            }
            else
            {
                if(u32_tmr_32k_a - u32_tmr_32k_b < 0x7FFFFFFF)
                {
                    u64_delta_24m += ((UINT64)1 << 32);
                }
            }        
        }
    }
    u64_delta_24m += ((((UINT64)u32_tmr_24m_b - (UINT64)u32_tmr_24m_a) + ((UINT64)1 << 32)) & 0xFFFFFFFF);    

    u32_delta_rtc = (UINT32)((u64_delta_24m % 6000)); //6000 / 24m = 1fn = 250us
    //u32_delta_rtc = ((u32_delta_rtc * 25) >> 3); // sample rate is 7.68m, u32_delta_rtc/(24m/7.68m)
    u32_delta_rtc = 6000 - u32_delta_rtc;
    u32_delta_fn = (UINT32)(((u64_delta_24m / 6000)));

    while(u32_delta_rtc < DSP_SLEEP_PROTECT_RTC)
    {
        u32_delta_rtc += 6000;  
        u32_delta_fn += 1;
    }
    u32_delta_rtc -= DSP_SLOT_POSITION_RTC;
    //u32_delta_fn += 1;

    KS_LOG(OAL_TRACE_LEVEL_0,0xD225,(UINT16)u32_delta_fn);	
    KS_LOG(OAL_TRACE_LEVEL_0,0xD225,(UINT16)(u32_delta_fn>>16));	
    u32_delta_fn &= 0x7ff;
    g_u32_frame_no = (g_u32_frame_no + u32_delta_fn) & 0x7FF;


    ks_oal_tmr_init( KS_CP0_TIMER0, u32_delta_rtc + 240, (UINT16)KS_TIMER_PRM_MODE_FREE_RUNNING, (UINT16)KS_TIMER_PRM_INTR_NOTMASK, \
        KS_PLL_MOD_CP0_TIMER0_CLK_SRC_XTL, timer0_rf_ctrl_restore);

    KS_LOG(OAL_TRACE_LEVEL_0,0xD223,(UINT16)u32_delta_fn);
    KS_LOG(OAL_TRACE_LEVEL_0,0xD224,(UINT16)u32_delta_rtc);	

	OAL_IRQ_RESTORE;

}


OAL_ITCM_CODE_SECTION
VOID ks_slp_enter(VOID)
{
    OAL_IRQ_SAVE_AREA;

	OAL_IRQ_DISABLE_ALL;
	if(g_enum_dsp_state EQ DSP_DEEPSLEEP)
	{
		//already enter deepsleep, do nothing 
    	OAL_IRQ_RESTORE;
		return;
	}

	
	if(g_enum_dsp_state EQ DSP_INIT)
	{
		//zzw not started, ENTER DSP_DEEPSLEEP_PRE DIRECTLY 
		ks_oal_dsp_state_set(DSP_DEEPSLEEP_PRE);
	}
	else
	{
		g_s32_sleep_protect_slot = SLEEP_PROTECT_SLOT; 
	}

	OAL_IRQ_RESTORE;

}

OAL_ITCM_CODE_SECTION
VOID ks_slp_enter_final(VOID)
{
    OAL_IRQ_SAVE_AREA;

	OAL_IRQ_DISABLE_ALL;
	ks_oal_drv_deinit_1643();
	#if 1
	g_u32_imask_bak[0] = READ_REG32(ICTL0_IRQ_INTEN_L);
	g_u32_imask_bak[1] = READ_REG32(ICTL0_IRQ_INTEN_H);
	g_u32_imask_bak[2] = READ_REG32(ICTL1_IRQ_INTEN_L);
	g_u32_imask_bak[3] = READ_REG32(ICTL1_IRQ_INTEN_H);

	WRITE_REG32(ICTL0_IRQ_INTEN_L, (0x1<<17) | (0x1<<4)); // leave mailbox for wakeup
	WRITE_REG32(ICTL0_IRQ_INTEN_H, 0);
	WRITE_REG32(ICTL1_IRQ_INTEN_L, 0);
	WRITE_REG32(ICTL1_IRQ_INTEN_H, 0);	

	WRITE_REG32(ICTL0_IRQ_INTMASK_L, 0xFFFDFFFF); // leave mailbox for wakeup
	WRITE_REG32(ICTL0_IRQ_INTMASK_H, 0xFFFFFFFF);
	WRITE_REG32(ICTL1_IRQ_INTMASK_L, 0xFFFFFFFF);
	WRITE_REG32(ICTL1_IRQ_INTMASK_H, 0xFFFFFFFF);
	#endif
    ks_timer_stop( KS_CP0_TIMER1 );
    ks_oal_irq_unregister(OAL_IRQ_CP_TIMER_1);
    
    g_u32_slp_cnt_enter += 1;
    
    //ks_oal_set_clk((3 * MHZ) >> 1);

	OAL_IRQ_RESTORE;

}



OAL_ITCM_CODE_SECTION
VOID ks_oal_drv_init_1643(VOID)
{
	OAL_IRQ_SAVE_AREA;
		
	OAL_IRQ_DISABLE_ALL;

	KS_LOG_TMR(0xD300);


	if(g_s32_sleep_protect_slot)
	{
		OAL_IRQ_RESTORE;
		return;
	}
	WRITE_REG32(0x103C0010, READ_REG32(0x103C0010) | (BIT20));//djtag en
	WRITE_REG32(0x103C0010, READ_REG32(0x103C0010) | (BIT9));//uart0 en
	WRITE_REG32(0x103C0010, READ_REG32(0x103C0010) | (BIT10));//uart1 en
	WRITE_REG32(0x103C0010, READ_REG32(0x103C0010) | (BIT11));//uart2 en
	WRITE_REG32(0x103C0010, READ_REG32(0x103C0010) | (BIT12));//uart3 en
	WRITE_REG32(0x103C0000, READ_REG32(0x103C0000) | (BIT8));//dmas en
	WRITE_REG32(0x103C0000, READ_REG32(0x103C0000) | (BIT26));//spinlock en

	if(g_u32_extend_ini_flg)
	{
	
		//WRITE_REG32(0x103C0000, READ_REG32(0x103C0000) | (BIT24));//global rf clk en
		WRITE_REG32(0x103C0010, READ_REG32(0x103C0010) | (BIT18));//gpio clk en
		WRITE_REG32(0x103C0010, READ_REG32(0x103C0010) | (BIT17));//gpio_pclk en
		
		WRITE_REG32(0x103C0010, READ_REG32(0x103C0010) | (BIT1));//spi_clk en
		WRITE_REG32(0x103C0010, READ_REG32(0x103C0010) | (BIT0));//spi_pclk en

	    if((DSP_INIT == g_enum_dsp_state) || \
	    	(DSP_DEEPSLEEP == g_enum_dsp_state))
	    {	
	    	if(DSP_DEEPSLEEP == g_enum_dsp_state)
	    	{
		
				ks_oal_init_dmac();
				ks_oal_dsp_state_set(DSP_DEEPSLEEP_OUT);
				g_s32_sleep_protect_slot = SLEEP_PROTECT_SLOT;	
				#if (KS_LOG_DMA&&(!PRINT_LOG_BYTE))
				g_u32_log_en = 1;
				#endif

				//ks_gpio_direction_output(KS_GPIO26, 1);
				
				WRITE_REG32(0x103c0010, READ_REG32(0x103c0010) | (BIT19));//rffe clk en
				ks_oal_delay(100);
				WRITE_REG32(0x10340000, READ_REG32(0x10340000) & (~BIT13));//rffe reset
			}
			else
			{
				g_s32_sleep_protect_slot = SLEEP_PROTECT_SLOT; 
				//ks_oal_dsp_state_set(DSP_ZZW);
			}
	    	//ind 4500 init now
		    if(cp1_x1643_def_flag == 1)
		    {     
				ks_oal_mailbox_send_direct((UINT32)KS_CPU_CP1_XC4500, (UINT32)KS_CPU_CP1_X1643, (UINT32)OAL_1643_WAKE_4500, (UINT32)0);
			}
			else
			{
				ks_oal_mailbox_send_direct((UINT32)KS_CPU_CP0_XC4500, (UINT32)KS_CPU_CP0_X1643, (UINT32)OAL_1643_WAKE_4500, (UINT32)0);
			}

			//rf_drv_cx9261_init();

			//ks_oal_delay(1000);	

			
		    // scfde_tx_framing_init();
		    // framing_start_config(0);
	        //ul_init();
	        #ifndef KD001_RF8242
	        algo_tx_proc_init();
			KS_LOG_TMRL(0xD401);
    		framing_config_init(0);
    		#if MIMO_2ANTS
			framing_config_init(1);
			#endif
			KS_LOG_TMRL(0xD402);
		    rxdfe_init(0);
			KS_LOG_TMRL(0xD403);
		    txdfe_init(0);
			KS_LOG_TMRL(0xD404);
		    //turbo_encoder_init();
		    // turbo_encoder_set_callback_func(scfde_tx_turbo_enc_callback);
			rf_drv_init();
			KS_LOG_TMRL(0xD405);

            #if 1
	    	if(DSP_DEEPSLEEP_OUT == g_enum_dsp_state)
	    	{
                ks_slp_time_get();	    	
            }
            else
			#endif
            {
                ks_oal_delay(10000*4*4000);	
    			rf_ctrl_enable();
		    }
		    
			KS_LOG_TMRL(0xD406);
			#endif
		}
	}
	else
	{
		//acc_cpsys_on();
		//ks_oal_init_clk();
		//ind 4500 init now
		if(cp1_x1643_def_flag == 1)
		{	  
			ks_oal_mailbox_send_direct((UINT32)KS_CPU_CP1_XC4500, (UINT32)KS_CPU_CP1_X1643, (UINT32)OAL_DEEP_SLP_4500_EXIT, (UINT32)0);
		}
		else
		{
			ks_oal_mailbox_send_direct((UINT32)KS_CPU_CP0_XC4500, (UINT32)KS_CPU_CP0_X1643, (UINT32)OAL_DEEP_SLP_4500_EXIT, (UINT32)0);
		}
		
		ks_oal_dsp_state_set(DSP_INIT);
	}
	KS_LOG_TMR(0xD301);

	OAL_IRQ_RESTORE;
}

OAL_ITCM_CODE_SECTION
VOID ks_oal_drv_clk_off(VOID)
{
	WRITE_REG32(0x103C0010, READ_REG32(0x103C0010) & (~BIT18));//gpio clk close
	WRITE_REG32(0x103C0010, READ_REG32(0x103C0010) & (~BIT17));//gpio_pclk close
	WRITE_REG32(0x103C0010, READ_REG32(0x103C0010) & (~BIT20));//djtag close

	WRITE_REG32(0x103C0010, READ_REG32(0x103C0010) & (~BIT9));//uart0 close
	WRITE_REG32(0x103C0010, READ_REG32(0x103C0010) & (~BIT10));//uart1 close
	WRITE_REG32(0x103C0010, READ_REG32(0x103C0010) & (~BIT11));//uart2 close
	WRITE_REG32(0x103C0010, READ_REG32(0x103C0010) & (~BIT12));//uart3 close

	WRITE_REG32(0x103C0000, READ_REG32(0x103C0000) & (~BIT8));//dmas close
	WRITE_REG32(0x103C0010, READ_REG32(0x103C0010) & (~BIT1));//spi_clk close
	WRITE_REG32(0x103C0010, READ_REG32(0x103C0010) & (~BIT0));//spi_pclk close
	WRITE_REG32(0x103C0000, READ_REG32(0x103C0000) & (~BIT26));//spinlock close
	WRITE_REG32(0x103c0010, READ_REG32(0x103c0010) & (~BIT19));//rffe clk disable

}
OAL_ITCM_CODE_SECTION
UINT32 ks_slp_judge(UINT32 nxt_fn)
{
    UINT32 u32_i = 0;
    UINT32 u32_nop_slot_num = 0;
    

    for(u32_i = 0 ; u32_i < 1024; u32_i++)
    {
        if(g_zzw_slot_table[((nxt_fn >> 1) + u32_i) & 0x3FF] EQ NOP)
        {
            u32_nop_slot_num += 2;
        }
        else
        {
            break;
        }
    }


    KS_LOG(OAL_TRACE_LEVEL_0,0xD227,(UINT16)u32_nop_slot_num);

#if 0
    if(500 > u32_nop_slot_num )
    {
        u32_nop_slot_num = 0;
    }
    else
    {
        ks_slp_enter();
        ks_oal_tmr_init( KS_CP0_TIMER0, (((u32_nop_slot_num - 120) << 15) / 4000), (UINT16)KS_TIMER_PRM_MODE_FREE_RUNNING , (UINT16)KS_TIMER_PRM_INTR_NOTMASK , \
            KS_PLL_MOD_CP0_TIMER0_CLK_SRC_RTC_32K, timer0_ticks);
    }
#endif
    return u32_nop_slot_num;
}

OAL_ITCM_CODE_SECTION
VOID ks_oal_drv_deinit_1643(VOID)
{
	OAL_IRQ_SAVE_AREA;
		
	OAL_IRQ_DISABLE_ALL;
	
	KS_LOG_TMR(0xD302);

	if(g_u32_extend_ini_flg)
	{
		//rxdfe_clk_disable(0);
		//txdfe_clk_disable(0);
		#ifndef KD001_RF8242
		rf_drv_deinit();
		#endif
		KS_LOG_TMRL(0xD411);

        framing_config_cls(0);
        
		KS_LOG_TMRL(0xD412);
        framing_tce_config_dmad_stop();
        ks_oal_delay(100);

        
		KS_LOG_TMRL(0xD413);

        #if 1
        framing_config_cls(1);
        ks_oal_delay(100);
        #endif
        
		KS_LOG_TMRL(0xD414);
        framing_clk_cls(0);
        framing_clk_cls(1);
		KS_LOG_TMRL(0xD415);

        rxdfe_reset_disable(0);
        //WRITE_REG32(0x1034000C, READ_REG32(0x1034000C) | BIT18);//rxdfe0 clk
        ks_oal_delay(100);
        WRITE_REG32(0x10340010, READ_REG32(0x10340010) | BIT1);//rxdfe0 reset
        ks_oal_delay(100);
		KS_LOG_TMRL(0xD416);

        #if 1
        rxdfe_reset_disable(1);
        //WRITE_REG32(0x1034000C, READ_REG32(0x1034000C) | BIT18);//rffe reset
        ks_oal_delay(100);
        WRITE_REG32(0x10340010, READ_REG32(0x10340010) | BIT0);//rxdfe1 reset
        ks_oal_delay(100);
        #endif
		KS_LOG_TMRL(0xD417);
        rxdfe_clk_disable(0);
        rxdfe_clk_disable(1);
		KS_LOG_TMRL(0xD418);

        txdfe_reset_disable(0);
        ks_oal_delay(100);
        txdfe_reset_disable(1);
		KS_LOG_TMRL(0xD419);        
        ks_oal_delay(100);
        txdfe_clk_disable(0);
        txdfe_clk_disable(1);
		KS_LOG_TMRL(0xD41A);

        rf_ctrl_disable();
		KS_LOG_TMRL(0xD41B);        
        turbo_encoder_deinit();
        #if (KS_LOG_DMA&&(!PRINT_LOG_BYTE))
        g_u32_log_en = 0;
        #endif
        dma_clk_off();
        //acc_cpsys_off();

        WRITE_REG32(0x10340000, READ_REG32(0x10340000) | BIT13);//rffe reset

		//ks_gpio_direction_output(KS_GPIO26, 0);

		//WRITE_REG32(0x103C0000, READ_REG32(0x103C0000) & (~BIT24));//global rf clk close

	
		if(cp1_x1643_def_flag == 1)
		{
			ks_oal_mailbox_send_direct((UINT32)KS_CPU_CP1_XC4500, (UINT32)KS_CPU_CP1_X1643, (UINT32)OAL_DEEP_SLP_4500_ENTER, (UINT32)g_u32_shutdown_flg);
		}
		else
		{
			ks_oal_mailbox_send_direct((UINT32)KS_CPU_CP0_XC4500, (UINT32)KS_CPU_CP0_X1643, (UINT32)OAL_DEEP_SLP_4500_ENTER, (UINT32)g_u32_shutdown_flg);
		}


	}
	else
	{
		// rf not init, no need to do deinit, only notify 4500 enter sleep
		if(cp1_x1643_def_flag == 1)
		{
			ks_oal_mailbox_send_direct((UINT32)KS_CPU_CP1_XC4500, (UINT32)KS_CPU_CP1_X1643, (UINT32)OAL_DEEP_SLP_4500_ENTER_NOZZW, (UINT32)g_u32_shutdown_flg);
		}
		else
		{
			ks_oal_mailbox_send_direct((UINT32)KS_CPU_CP0_XC4500, (UINT32)KS_CPU_CP0_X1643, (UINT32)OAL_DEEP_SLP_4500_ENTER_NOZZW, (UINT32)g_u32_shutdown_flg);
		}
	}

    ks_oal_drv_clk_off();
	if(g_u32_shutdown_flg)
	{
		ks_oal_init_mb_info();
 		g_u32_boot_flg = 0;	
		ks_oal_dsp_pwr_state_set(1, KS_SYS_STATUS_SHUTDOWN);
	}
	else
	{
		ks_oal_dsp_pwr_state_set(0, KS_SYS_STATUS_DEEP_SLEEP & 0xF);
	}
	
	ks_oal_dsp_state_set(DSP_DEEPSLEEP);

	KS_LOG_TMR(0xD303);

	OAL_IRQ_RESTORE;
}
OAL_ITCM_CODE_SECTION
UINT16 ks_slp_proc(VOID)
{

	if(0 < g_s32_sleep_protect_slot)
	{
		//OAL_ASSERT( (DSP_DEEPSLEEP_PRE == g_enum_dsp_state) || (DSP_DEEPSLEEP_OUT == g_enum_dsp_state), "");
		if((DSP_DEEPSLEEP_OUT EQ g_enum_dsp_state) && (g_s32_sleep_protect_slot EQ SLEEP_PROTECT_SLOT))
		{
		    //ks_oal_dsp_store_slp_time(1); //store the first rf sch int tmr after sleep
		    //ks_oal_dsp_get_slp_time(); //retore timing
		    //ks_timer_stop(KS_CP0_TIMER0);
	    }
	    
		g_s32_sleep_protect_slot--;

		if(0 == g_s32_sleep_protect_slot)
		{
			if(DSP_ZZW EQ g_enum_dsp_state)
			{
			    ks_slp_time_store(0); //store the last rf sch int tmr before sleep
				ks_oal_dsp_state_set(DSP_DEEPSLEEP_PRE);
			}
			else if(DSP_DEEPSLEEP_OUT EQ g_enum_dsp_state)
			{
			    //ks_oal_dsp_get_tmr_cnt(1); //store the first rf sch int tmr after sleep
				ks_oal_dsp_state_set(DSP_ZZW);
			}
			else if(DSP_INIT EQ g_enum_dsp_state)
			{
				ks_oal_dsp_state_set(DSP_ZZW);
			}

		}
		return OAL_TRUE;	
	}
	else
	{
		if( g_enum_dsp_state EQ DSP_DEEPSLEEP)
		{
			return OAL_TRUE;	
		}
		else
		{
			return OAL_FALSE;
		}
	}

}


#else
extern DMAC_LINK_INFO g_st_rxdfe_link_item;

OAL_ITCM_CODE_SECTION
VOID ks_slp_enter(VOID)
{
    OAL_IRQ_SAVE_AREA;

	OAL_IRQ_DISABLE_ALL;
	if(DSP_INIT != g_enum_dsp_state)
	{
		ks_oal_dsp_state_set(DSP_DEEPSLEEP_PRE);
	}
	OAL_IRQ_RESTORE;

}

OAL_ITCM_CODE_SECTION
VOID ks_slp_enter_nozzw(VOID)
{
	OAL_IRQ_SAVE_AREA;

	OAL_IRQ_DISABLE_ALL;

#if 1
	g_u32_imask_bak[0] = READ_REG32(ICTL0_IRQ_INTEN_L);
	g_u32_imask_bak[1] = READ_REG32(ICTL0_IRQ_INTEN_H);
	g_u32_imask_bak[2] = READ_REG32(ICTL1_IRQ_INTEN_L);
	g_u32_imask_bak[3] = READ_REG32(ICTL1_IRQ_INTEN_H);

	WRITE_REG32(ICTL0_IRQ_INTEN_L, (0x1<<17)); // leave mailbox for wakeup
	WRITE_REG32(ICTL0_IRQ_INTEN_H, 0);
	WRITE_REG32(ICTL1_IRQ_INTEN_L, 0);
	WRITE_REG32(ICTL1_IRQ_INTEN_H, 0);	

	WRITE_REG32(ICTL0_IRQ_INTMASK_L, 0xFFFDFFFF); // leave mailbox for wakeup
	WRITE_REG32(ICTL0_IRQ_INTMASK_H, 0xFFFFFFFF);
	WRITE_REG32(ICTL1_IRQ_INTMASK_L, 0xFFFFFFFF);
	WRITE_REG32(ICTL1_IRQ_INTMASK_H, 0xFFFFFFFF);	
#endif

	if(g_u32_shutdown_flg)
	{
	    WRITE_REG32(OAL_ASSERT_ADDR_1643, 0);
		ks_oal_init_mb_info();	
		g_u32_boot_flg = 0;
		ks_oal_dsp_pwr_state_set(1, KS_SYS_STATUS_SHUTDOWN);
	}
	else
	{
		ks_oal_dsp_pwr_state_set(0, KS_SYS_STATUS_DEEP_SLEEP & 0xF);
	}
	
    ks_timer_stop(KS_CP0_TIMER3);
    g_u32_slp_cnt_enter += 1;
    //ks_oal_set_clk((3 * MHZ) >> 1);
	OAL_IRQ_RESTORE;

}

OAL_ITCM_CODE_SECTION
VOID ks_slp_enter_final(VOID)
{
	OAL_IRQ_SAVE_AREA;

	OAL_IRQ_DISABLE_ALL;
	ks_oal_drv_deinit_4500();

	//ks_oal_dsp_state_set(DSP_DEEPSLEEP);

	#if 1
	g_u32_imask_bak[0] = READ_REG32(ICTL0_IRQ_INTEN_L);
	g_u32_imask_bak[1] = READ_REG32(ICTL0_IRQ_INTEN_H);
	g_u32_imask_bak[2] = READ_REG32(ICTL1_IRQ_INTEN_L);
	g_u32_imask_bak[3] = READ_REG32(ICTL1_IRQ_INTEN_H);

	WRITE_REG32(ICTL0_IRQ_INTEN_L, (0x1<<17)); // leave mailbox for wakeup
	WRITE_REG32(ICTL0_IRQ_INTEN_H, 0);
	WRITE_REG32(ICTL1_IRQ_INTEN_L, 0);
	WRITE_REG32(ICTL1_IRQ_INTEN_H, 0);	

	WRITE_REG32(ICTL0_IRQ_INTMASK_L, 0xFFFDFFFF); // leave mailbox for wakeup
	WRITE_REG32(ICTL0_IRQ_INTMASK_H, 0xFFFFFFFF);
	WRITE_REG32(ICTL1_IRQ_INTMASK_L, 0xFFFFFFFF);
	WRITE_REG32(ICTL1_IRQ_INTMASK_H, 0xFFFFFFFF);	
	#endif

    ks_timer_stop( KS_CP0_TIMER3);
    g_u32_boot_flg = 2; 
    
    g_u32_slp_cnt_enter += 1;
    //ks_oal_set_clk((3 * MHZ) >> 1);    
	OAL_IRQ_RESTORE;

}
//extern VOID dl_csma_hw_init();
OAL_ITCM_CODE_SECTION
VOID ks_oal_drv_init_4500(VOID)
{
	OAL_IRQ_SAVE_AREA;
		
	OAL_IRQ_DISABLE_ALL;
	KS_LOG_TMR(0xD300);	
	//_init_fic();
	if(DSP_DEEPSLEEP == g_enum_dsp_state)
	{
		//acc_cpsys_on();
		ks_oal_init_dmac();
		ks_slp_exit();
		#if (KS_LOG_DMA&&(!PRINT_LOG_BYTE))
		g_u32_log_en = 1;
		#endif
	}

	ks_oal_dsp_state_set(DSP_ZZW);
    //dl_csma_hw_init();
    //turbo_decoder_init();
    // turbo_decoder_set_callback_func(scfde_rx_turbo_dec_callback);
    //turbo_decoder_deinit();
    // scfde_rx_rxdfe_dmad_config(1920, 8, (UINT32_PTR)0x11290000, 0);
	//dl_csma_hw_init();
	dl_hw_init();

	KS_LOG_TMR(0xD301);
	OAL_IRQ_RESTORE;
}

OAL_ITCM_CODE_SECTION
VOID ks_oal_drv_deinit_4500(VOID)
{
	OAL_IRQ_SAVE_AREA;
		
	OAL_IRQ_DISABLE_ALL;
	
	KS_LOG_TMR(0xD302);

	if(DSP_INIT != g_enum_dsp_state)
	{
		//ks_dmac_ch_disable(KS_DMAC0_CP, g_st_rxdfe_link_item.ch_id);
	    turbo_decoder_deinit();
	    #if (KS_LOG_DMA&&(!PRINT_LOG_BYTE))
		g_u32_log_en = 0;
		#endif
		dma_clk_off();
		_init_close_fic();
		//acc_cpsys_off();
		ks_oal_dsp_state_set(DSP_DEEPSLEEP);
	}
	//_close_psvm_reg();

	if(g_u32_shutdown_flg)
	{
		WRITE_REG32(OAL_DSP_HANDSHAKE_ADDR, 0);
		ks_oal_init_mb_info();
		g_u32_boot_flg = 0;	
		ks_oal_dsp_pwr_state_set(1, KS_SYS_STATUS_SHUTDOWN);
	}
	else
	{
		ks_oal_dsp_pwr_state_set(0, KS_SYS_STATUS_DEEP_SLEEP & 0xF);
	}
	
	KS_LOG_TMR(0xD303);

	OAL_IRQ_RESTORE;
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
OAL_ITCM_CODE_SECTION
VOID ks_oal_init_task(VOID)
{
    UINT32  u32_index;
    VOID_PTR vp_stack_start;
    CONST oal_task_desc_table_t *stp_task_desc_tbl;

    /* Initialize all the tasks to invalid, including SYS task */
    for (u32_index = 0; u32_index < g_u16_task_amt; u32_index++)
    {
        g_st_oal_task_tcb[u32_index].b_invalid = OAL_TRUE;
    }

    /* Initialize the pointer array, whose element points to a ready list each */
    for (u32_index = 0; u32_index < OAL_TASK_PRIORITY_AMT; u32_index++)
    {
        g_stp_task_ready_list[u32_index] = NULL_PTR;
    }

    /* Create all the task */
    stp_task_desc_tbl = g_stp_task_desc;
    for (u32_index = 0; u32_index < g_u16_task_amt; u32_index++, stp_task_desc_tbl++)
    {
        OAL_ASSERT((stp_task_desc_tbl->entry != NULL_PTR),"task has an empty entry!");
        OAL_ASSERT((stp_task_desc_tbl->u16_stack_size != (UINT16)0), "stack size is zero!");
        OAL_ASSERT(((UINT32)stp_task_desc_tbl->entry != (UINT32)stp_task_desc_tbl->init_func), "task entry and init function is same!");
        OAL_ASSERT(((UINT16)OAL_LOWEST_PRIORITY >= stp_task_desc_tbl->u16_priority), "task entry and init function is same!");

        g_u32_oal_task_num++;
        /* allocate stack memory for task */
        vp_stack_start = ks_oal_mem_stack_alloc(stp_task_desc_tbl->u16_stack_level, stp_task_desc_tbl->u16_stack_size);

        OAL_ASSERT(NULL_PTR != vp_stack_start, "ks_oal_mem_stack_alloc is fail");

        ks_oal_task_create(stp_task_desc_tbl->u16_task_id,
                        stp_task_desc_tbl->u16_priority,
                        stp_task_desc_tbl->u16_active_id,
                        stp_task_desc_tbl->entry,
                        stp_task_desc_tbl->init_func,
                        vp_stack_start,
                        stp_task_desc_tbl->u16_stack_size);
    }

	g_stp_current_running_task = g_stp_ready_to_run_task;
}

/*************************************************** END OF FILE ******************************************************/

