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
#define THIS_FILE_NAME_ID MAIN_FILE_ID

/***********************************************************************************************************************
* INCLUDE FILES
***********************************************************************************************************************/
#include "ks_dmas.h"
#include "l1cc.h"
#include "zzw_common.h"
#include "ks_it.h"
#include "ks_lib_variable.h"
#include "ks_hw_init.h"
#include "ks_phych_func.h"
#include "ks_phych_dispatch.h"

#ifdef CORE_X1643
#include "ks_txdfe.h"
#include "ks_rxdfe.h"
#include "ks_rf_rffe.h"
#include "ks_rf_spi.h"
#include "ks_rf_ctrl.h"
#include "ks_turbo_encoder.h"
#else
#include "ks_turbo_decoder.h"
#include "ofdm_rx_proc.h"
#endif

/***********************************************************************************************************************
* GLOBAL VARIABLES DEFINITION
***********************************************************************************************************************/
OAL_SRAM_DATA_SECTION
oal_task_desc_table_t g_st_oal_task_desc_table[] =
{
#ifdef CORE_X1643
    {(UINT16)OAL_SYS_TASK_ID+4,         (UINT16)OAL_SYS_TASK_PRIORITY-3, (UINT16)UNSPECIFIED_ACTIVE_ID, (UINT16)0, (oal_task_entry_t)l1cc_dispatch_main, NULL_PTR, (UINT16)0x200, (UINT16)OAL_STACK_DTCM_LEVEL},
    {(UINT16)OAL_SYS_TASK_ID,           (UINT16)OAL_SYS_TASK_PRIORITY,   (UINT16)UNSPECIFIED_ACTIVE_ID, (UINT16)0, (oal_task_entry_t)l1cc_sys_main,      NULL_PTR, (UINT16)0x200, (UINT16)OAL_STACK_DTCM_LEVEL},
    {(UINT16)OAL_SYS_TASK_ID+0x50,      (UINT16)OAL_SYS_TASK_PRIORITY-3, (UINT16)UNSPECIFIED_ACTIVE_ID, (UINT16)0, (oal_task_entry_t)l1cc_hl_proc_main,  NULL_PTR, (UINT16)0x200, (UINT16)OAL_STACK_DTCM_LEVEL},
    {(UINT16)L1CC_UL_PROC_MAIN_TASK_ID, (UINT16)OAL_SYS_TASK_PRIORITY-5, (UINT16)UNSPECIFIED_ACTIVE_ID, (UINT16)0, (oal_task_entry_t)l1cc_ul_proc_main,  NULL_PTR, (UINT16)0x400, (UINT16)OAL_STACK_DTCM_LEVEL},
    {(UINT16)L1CC_UL_MOD_MAIN_TASK_ID,  (UINT16)OAL_SYS_TASK_PRIORITY-4, (UINT16)UNSPECIFIED_ACTIVE_ID, (UINT16)0, (oal_task_entry_t)l1cc_ul_mod_main,   NULL_PTR, (UINT16)0x200, (UINT16)OAL_STACK_DTCM_LEVEL},

#else
    {(UINT16)OAL_SYS_TASK_ID+4,         (UINT16)OAL_SYS_TASK_PRIORITY-5, (UINT16)UNSPECIFIED_ACTIVE_ID, (UINT16)0, (oal_task_entry_t)l1cc_dispatch_main, NULL_PTR, (UINT16)0x400, (UINT16)OAL_STACK_DTCM_LEVEL},
    {(UINT16)OAL_SYS_TASK_ID,           (UINT16)OAL_SYS_TASK_PRIORITY,   (UINT16)UNSPECIFIED_ACTIVE_ID, (UINT16)0, (oal_task_entry_t)l1cc_sys_main,      NULL_PTR, (UINT16)0x600, (UINT16)OAL_STACK_DTCM_LEVEL},
    {(UINT16)OAL_SYS_TASK_ID+0x5,       (UINT16)OAL_SYS_TASK_PRIORITY-4, (UINT16)UNSPECIFIED_ACTIVE_ID, (UINT16)0, (oal_task_entry_t)dl_proc_main,       NULL_PTR, (UINT16)0x600, (UINT16)OAL_STACK_DTCM_LEVEL},
#endif
};

OAL_DTCM_DATA_SECTION oal_task_desc_table_t* g_stp_task_desc = &g_st_oal_task_desc_table[0];
OAL_DTCM_DATA_SECTION UINT16 g_u16_task_amt = sizeof(g_st_oal_task_desc_table)/sizeof(g_st_oal_task_desc_table[0]);

OAL_SRAM_DATA_SECTION oal_scb_desc_table_t  g_st_oal_sema_table[OAL_IPC_SEMA_MAXNUM] =
{
    {(UINT16)COMMON_SEMA_BASE_ID,         (UINT16)0, (UINT16)1},
    {(UINT16)OAL_L1_HLS_MB0_SEMA,         (UINT16)1, (UINT16)1},
    {(UINT16)OAL_L1_HLS_MB1_SEMA,         (UINT16)1, (UINT16)1},
    {(UINT16)OAL_L1_HLS_MB2_SEMA,         (UINT16)1, (UINT16)1},
    {(UINT16)OAL_L1_HLS_MB3_SEMA,         (UINT16)1, (UINT16)1},
    {(UINT16)OAL_L1_HLS_MB4_SEMA,         (UINT16)1, (UINT16)1},
    {(UINT16)OAL_L1_HLS_MB5_SEMA,         (UINT16)1, (UINT16)1},
    {(UINT16)OAL_X1643_XC4500_MB0_SEMA,   (UINT16)1, (UINT16)1},
    {(UINT16)AP_2_DSP_PHYCH_ID0_SEMA,     (UINT16)0, (UINT16)128},
    {(UINT16)AP_2_DSP_PHYCH_ID1_SEMA,     (UINT16)0, (UINT16)128},
    {(UINT16)DSP_INTER_PHYCH_ID2_SEMA,    (UINT16)0, (UINT16)128},
    {(UINT16)DSP_INTER_PHYCH_ID3_SEMA,    (UINT16)0, (UINT16)128},
	{(UINT16)OAL_L1CC_IPC_SEMA,           (UINT16)0, (UINT16)1},  //@Zhemin
};

OAL_DRAM_DATA_SECTION uint32 verinfo_addr[128];
OAL_DRAM_DATA_SECTION uint8 g_u8_build_time[] = __TIME__;
OAL_DRAM_DATA_SECTION uint8 g_u8_build_date[] = __DATE__;
OAL_DRAM_DATA_SECTION uint8 g_u8_build_file[] = __FILE__;

OAL_DTCM_DATA_SECTION UINT32 g_u32_build_time = 0;
OAL_DTCM_DATA_SECTION UINT32 g_u32_build_date = 0;


/***********************************************************************************************************************
* EXTERN VARIABLES DECLARATION
***********************************************************************************************************************/


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
int main(VOID)
{
	#ifdef DSP_JTAG_IDE_SUPPORT

	ks_oal_init_clk();
	ks_oal_init_var();
	ks_oal_init_bsp();
    #ifdef CORE_X1643
	*((VOLATILE_UINT32*)0x1724FFD0) = 0x05280002;
	*((VOLATILE_UINT32*)0x1724FF20) = 0;
	*((VOLATILE_UINT32*)0x17090010) = 0;
	*((VOLATILE_UINT32*)0x17090020) = 0;
	g_u32_extend_ini_flg = 1;
	ks_oal_drv_init_1643();
    #else
	ks_oal_drv_init_4500();
    #endif

	#else

    #ifdef CORE_X1643
    
    WRITE_REG32(OAL_ASSERT_ADDR_1643, 0);
    
    ks_oal_init_clk();
    WRITE_REG32(OAL_DSP_HANDSHAKE_ADDR, OAL_MEM_SPECIAL_TAG - 2);
    
    ks_oal_init_var();
    WRITE_REG32(OAL_DSP_HANDSHAKE_ADDR, OAL_MEM_SPECIAL_TAG - 1);
	
    ks_oal_init_bsp();
    WRITE_REG32(OAL_DSP_HANDSHAKE_ADDR, OAL_MEM_SPECIAL_TAG);
    //WRITE_REG32(0x103c0014, READ_REG32(0x103c0014)|(0x6));

    ks_hs_init();


    // rxdfe_dmac_config(128*2*3, 160, (UINT32_PTR)(0x11200000), 0);
    // rxdfe_init(0);
    // txdfe_init(0);
    // rf_drv_init();
	
    #else
    
	WRITE_REG32(OAL_ASSERT_ADDR_4500, 0);
    WRITE_REG32(OAL_DSP_HANDSHAKE_WAIT_ADDR, 0);
    while(READ_REG32(OAL_DSP_HANDSHAKE_ADDR) != OAL_MEM_SPECIAL_TAG)
    {
        //4500 wait for 1643 init ready
    	ks_oal_delay(1000);
        WRITE_REG32(OAL_DSP_HANDSHAKE_WAIT_ADDR, READ_REG32(OAL_DSP_HANDSHAKE_WAIT_ADDR) + 1);
    }
    WRITE_REG32(OAL_DSP_HANDSHAKE_ADDR, READ_REG32(OAL_DSP_HANDSHAKE_ADDR) + 1);
    
    ks_oal_init_var();
    ks_oal_init_bsp();

    ks_hs_init();
    
    WRITE_REG32(OAL_DSP_HANDSHAKE_ADDR, READ_REG32(OAL_DSP_HANDSHAKE_ADDR) + 1);
    #endif 

    #ifdef CORE_X1643
    *((VOLATILE_UINT32*)0x1724FFD0) = 0x05280002;
	*((VOLATILE_UINT32*)0x1724FF20) = 0;
	*((VOLATILE_UINT32*)0x17090010) = 0;
	*((VOLATILE_UINT32*)0x17090020) = 0;
	//g_u32_extend_ini_flg = 1;
	//ks_oal_drv_init_1643();
    #else
	//ks_oal_drv_init_4500();
    #endif

	#endif
    VERINFO(verinfo_addr); // print sdk version
	g_u32_build_time = ks_build_time2int(g_u8_build_time);
	g_u32_build_date = ks_build_date2int(g_u8_build_date);
	#ifdef CORE_X1643
	{
		#ifdef DVB
		#ifdef KD001_RF8242
		char *ext_str = "_dvb_RF8242";
		#else
		char *ext_str = "_dvb";
		#endif
		#else
		#ifdef KD001_RF8242
		char *ext_str = "_RF8242";
		#else
		char *ext_str = NULL;
		#endif
		#endif
		ks_build_version(g_u8_build_date, g_u8_build_time, g_u8_build_file, ext_str, (UINT8_PTR)VERSION_ADDR);
	}
	#endif
    ks_oal_init_task();

    *((VOLATILE_UINT32*)0x1724FFD0) = 0x12345678;

    
    _ks_os_task_sch();

    return 0;
}

/*************************************************** END OF FILE ******************************************************/

