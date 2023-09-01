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
#define THIS_FILE_NAME_ID KS_LIB_VARIABLE_FILE_ID
        
/***********************************************************************************************************************
* INCLUDE FILES
***********************************************************************************************************************/
#include "ks_lib_variable.h"
#include "ks_oal.h"


/***********************************************************************************************************************
* GLOBAL VARIABLES DEFINITION
***********************************************************************************************************************/
/* TCB of all the tasks */
OAL_DTCM_DATA_SECTION
UINT32 g_u32_oal_task_num = 0;
OAL_DTCM_DATA_SECTION
UINT32 g_u32_oal_assert_info = 0;
OAL_DTCM_DATA_SECTION
oal_task_tcb_t g_st_oal_task_tcb[OAL_TASK_AMT] = {{0}};
OAL_DTCM_DATA_SECTION oal_task_tcb_t* g_stp_task_tcb = &g_st_oal_task_tcb[0];

OAL_DTCM_DATA_SECTION oal_scb_t g_st_oal_ipc_scb[OAL_IPC_SEMA_MAXNUM] = {{0}};
OAL_DTCM_DATA_SECTION UINT16 g_u16_ipc_sema_index[OAL_IPC_SEMA_MAXNUM] = {0};
OAL_DTCM_DATA_SECTION SINT32 oal_ipc_sema_maxnum = OAL_IPC_SEMA_MAXNUM;

#if defined(CORE_X1643)
OAL_DTCM_DATA_SECTION VOLATILE_SINT32 cp1_x1643_def_flag = CP1_X1643_DEF_FLAG;
#else
OAL_DTCM_DATA_SECTION VOLATILE_SINT32 cp1_xc4500_def_flag = CP1_XC4500_DEF_FLAG;
#endif

#ifdef CORE_XC4500
OAL_DTCM_DATA_SECTION
OAL_TRACE_LOG_BUF_DESC_T* g_trace_log_buffer = (OAL_TRACE_LOG_BUF_DESC_T*)(CORE_XC4500_TRACE_BUF_DESC_ADDR);

OAL_DTCM_DATA_SECTION
OAL_TRACE_STR_BUF_DESC_T* g_trace_str_buffer = (OAL_TRACE_STR_BUF_DESC_T*)(CORE_XC4500_TRACE_BUF_DESC_ADDR);
#else
OAL_DTCM_DATA_SECTION
OAL_TRACE_LOG_BUF_DESC_T* g_trace_log_buffer = (OAL_TRACE_LOG_BUF_DESC_T*)(CORE_XC1643_TRACE_BUF_DESC_ADDR);

OAL_DTCM_DATA_SECTION
OAL_TRACE_STR_BUF_DESC_T* g_trace_str_buffer = (OAL_TRACE_STR_BUF_DESC_T*)(CORE_XC1643_TRACE_BUF_DESC_ADDR);
#endif

OAL_DTCM_STACK_SECTION STATIC UINT32 s_u32_oal_stack_dtcm[OAL_STACK_DTCM_SIZE] = {0};
OAL_SRAM_STACK_SECTION STATIC UINT32 s_u32_oal_stack_sram[OAL_STACK_SRAM_SIZE] = {0};
OAL_DRAM_STACK_SECTION STATIC UINT32 s_u32_oal_stack_dram[OAL_STACK_DRAM_SIZE] = {0};

OAL_SRAM_DATA_SECTION
oal_mem_pool_desc_info_t g_st_oal_stack_desc_table[] =
{
    {(UINT32)OAL_STACK_DTCM_ID, (UINT32)OAL_STACK_DTCM_SIZE, &s_u32_oal_stack_dtcm[0], &s_u32_oal_stack_dtcm[OAL_STACK_DTCM_SIZE - 1], NULL_PTR},
    {(UINT32)OAL_STACK_SRAM_ID, (UINT32)OAL_STACK_SRAM_SIZE, &s_u32_oal_stack_sram[0], &s_u32_oal_stack_sram[OAL_STACK_SRAM_SIZE - 1], NULL_PTR},
    {(UINT32)OAL_STACK_DRAM_ID, (UINT32)OAL_STACK_DRAM_SIZE, &s_u32_oal_stack_dram[0], &s_u32_oal_stack_dram[OAL_STACK_DRAM_SIZE - 1], NULL_PTR}
};

OAL_DTCM_DATA_SECTION oal_mem_pool_desc_info_t* g_stp_stack_desc = &g_st_oal_stack_desc_table[0];
OAL_DTCM_DATA_SECTION UINT16 g_u16_stack_amt = sizeof(g_st_oal_stack_desc_table)/sizeof(g_st_oal_stack_desc_table[0]);

OAL_DTCM_HEAP_SECTION OAL_CACHE_LINE_ALIGNED STATIC UINT32 s_u32_oal_heap_dtcm[OAL_CACHE_LINE_DWORD_CEIL(OAL_HEAP_DTCM_SIZE)] = {0};
OAL_SRAM_HEAP_SECTION OAL_CACHE_LINE_ALIGNED STATIC UINT32 s_u32_oal_heap_sram[OAL_CACHE_LINE_DWORD_CEIL(OAL_HEAP_SRAM_SIZE)] = {0};
OAL_DRAM_HEAP_SECTION OAL_CACHE_LINE_ALIGNED STATIC UINT32 s_u32_oal_heap_dram[OAL_CACHE_LINE_DWORD_CEIL(OAL_HEAP_DRAM_SIZE)] = {0};

OAL_DTCM_DATA_SECTION oal_mem_pcb_t g_st_mem_pcb[OAL_HEAP_AMT] = {{NULL_PTR}};
OAL_DTCM_DATA_SECTION oal_mem_pcb_t* g_stp_mem_pcb = &g_st_mem_pcb[0];

OAL_SRAM_DATA_SECTION
oal_mem_cfg_t g_st_oal_heap_desc_table[] =
{
    {&s_u32_oal_heap_dtcm[0], &s_u32_oal_heap_dtcm[OAL_HEAP_DTCM_SIZE - 1], (SINT32)OAL_HEAP_DTCM_SIZE, (UINT32)0 + (UINT32)OAL_MEM_BCB_SIZE, (UINT32)1280 + (UINT32)OAL_MEM_BCB_SIZE, OAL_FALSE},
    {&s_u32_oal_heap_sram[0], &s_u32_oal_heap_sram[OAL_HEAP_SRAM_SIZE - 1], (SINT32)OAL_HEAP_SRAM_SIZE, (UINT32)28 + (UINT32)OAL_MEM_BCB_SIZE, (UINT32)66560 + (UINT32)OAL_MEM_BCB_SIZE, OAL_TRUE},
    {&s_u32_oal_heap_dram[0], &s_u32_oal_heap_dram[OAL_HEAP_DRAM_SIZE - 1], (SINT32)OAL_HEAP_DRAM_SIZE, (UINT32)900 + (UINT32)OAL_MEM_BCB_SIZE, (UINT32)0x7fff0000 + (UINT32)OAL_MEM_BCB_SIZE, OAL_TRUE}
};

OAL_DTCM_DATA_SECTION oal_mem_cfg_t* g_stp_heap_desc = &g_st_oal_heap_desc_table[0];
OAL_DTCM_DATA_SECTION UINT16 g_u16_heap_amt = sizeof(g_st_oal_heap_desc_table)/sizeof(g_st_oal_heap_desc_table[0]);

#ifdef CORE_X1643
OAL_DTCM_DATA_SECTION
oal_mb_info_t g_st_oal_dsp2arm_mb_info[OAL_DSP2ARM_MB_MAX_NUM] =
{
    {(CONST_UINT16)OAL_L1_HLS_WR_MB0_ID, (CONST_UINT16)OAL_L1_HLS_MB0_SEMA, (UINT16_PTR_CONST)L1_HLS_MB0_WR_ADDR, (UINT16_PTR_CONST)((UINT32)L1_HLS_MB0_WR_ADDR + ((UINT32)MAILBOX_CTRL_HEAD_SIZE_IN_WORD<<1)),(CONST_UINT32)L1_HLS_MB0_WR_SIZE},
    {(CONST_UINT16)OAL_L1_HLS_WR_MB1_ID, (CONST_UINT16)OAL_L1_HLS_MB1_SEMA, (UINT16_PTR_CONST)L1_HLS_MB1_WR_ADDR, (UINT16_PTR_CONST)((UINT32)L1_HLS_MB1_WR_ADDR + ((UINT32)MAILBOX_CTRL_HEAD_SIZE_IN_WORD<<1)),(CONST_UINT32)L1_HLS_MB1_WR_SIZE},
    {(CONST_UINT16)OAL_L1_HLS_WR_MB2_ID, (CONST_UINT16)OAL_L1_HLS_MB2_SEMA, (UINT16_PTR_CONST)L1_HLS_MB2_WR_ADDR, (UINT16_PTR_CONST)((UINT32)L1_HLS_MB2_WR_ADDR + ((UINT32)MAILBOX_CTRL_HEAD_SIZE_IN_WORD<<1)),(CONST_UINT32)L1_HLS_MB2_WR_SIZE},
};

OAL_DTCM_DATA_SECTION
oal_arm2dsp_mb_info_t g_st_oal_arm2dsp_mb_info[OAL_ARM2DSP_MB_MAX_NUM] =
{
    {(CONST_UINT16)OAL_HLS_L1_RD_MB0_ID, 0, (CONST_UINT32)HLS_L1_MB0_RD_SIZE, (UINT16_PTR)HLS_L1_MB0_RD_ADDR, (UINT16_PTR)((UINT32)HLS_L1_MB0_RD_ADDR + ((UINT32)MAILBOX_CTRL_HEAD_SIZE_IN_WORD<<1))},
    {(CONST_UINT16)OAL_HLS_L1_RD_MB1_ID, 0, (CONST_UINT32)HLS_L1_MB1_RD_SIZE, (UINT16_PTR)HLS_L1_MB1_RD_ADDR, (UINT16_PTR)((UINT32)HLS_L1_MB1_RD_ADDR + ((UINT32)MAILBOX_CTRL_HEAD_SIZE_IN_WORD<<1))},
    {(CONST_UINT16)OAL_HLS_L1_RD_MB2_ID, 0, (CONST_UINT32)HLS_L1_MB2_RD_SIZE, (UINT16_PTR)HLS_L1_MB2_RD_ADDR, (UINT16_PTR)((UINT32)HLS_L1_MB2_RD_ADDR + ((UINT32)MAILBOX_CTRL_HEAD_SIZE_IN_WORD<<1))}
};

OAL_DTCM_DATA_SECTION
oal_inter_dsp_mb_info_t g_st_oal_inter_dsp_mb_wr_info =
{
    CORE_ID_X1643, 0, OAL_X1643_XC4500_WR_MB0_ID, OAL_X1643_XC4500_MB0_SEMA, (UINT16_PTR)X1643_XC4500_MB0_ADDR,(UINT16_PTR)((UINT32)X1643_XC4500_MB0_ADDR + ((UINT32)MAILBOX_CTRL_HEAD_SIZE_IN_WORD<<1)), X1643_XC4500_MB0_SIZE
};

OAL_DTCM_DATA_SECTION
oal_inter_dsp_mb_info_t g_st_oal_inter_dsp_mb_rd_info =
{
    CORE_ID_X1643, 0, OAL_X1643_XC4500_RD_MB1_ID, OAL_INVALID_SEMA_ID, (UINT16_PTR)XC4500_X1643_MB0_ADDR, (UINT16_PTR)((UINT32)XC4500_X1643_MB0_ADDR + ((UINT32)MAILBOX_CTRL_HEAD_SIZE_IN_WORD<<1)), XC4500_X1643_MB0_SIZE
};

#else
OAL_DTCM_DATA_SECTION
oal_mb_info_t g_st_oal_dsp2arm_mb_info[OAL_DSP2ARM_MB_MAX_NUM] =
{
    {(CONST_UINT16)OAL_L1_HLS_WR_MB3_ID, (CONST_UINT16)OAL_L1_HLS_MB3_SEMA, (UINT16_PTR_CONST)L1_HLS_MB3_WR_ADDR, (UINT16_PTR_CONST)((UINT32)L1_HLS_MB3_WR_ADDR + ((UINT32)MAILBOX_CTRL_HEAD_SIZE_IN_WORD<<1)),(CONST_UINT32)L1_HLS_MB3_WR_SIZE},
    {(CONST_UINT16)OAL_L1_HLS_WR_MB4_ID, (CONST_UINT16)OAL_L1_HLS_MB4_SEMA, (UINT16_PTR_CONST)L1_HLS_MB4_WR_ADDR, (UINT16_PTR_CONST)((UINT32)L1_HLS_MB4_WR_ADDR + ((UINT32)MAILBOX_CTRL_HEAD_SIZE_IN_WORD<<1)),(CONST_UINT32)L1_HLS_MB4_WR_SIZE},
    {(CONST_UINT16)OAL_L1_HLS_WR_MB5_ID, (CONST_UINT16)OAL_L1_HLS_MB5_SEMA, (UINT16_PTR_CONST)L1_HLS_MB5_WR_ADDR, (UINT16_PTR_CONST)((UINT32)L1_HLS_MB5_WR_ADDR + ((UINT32)MAILBOX_CTRL_HEAD_SIZE_IN_WORD<<1)),(CONST_UINT32)L1_HLS_MB5_WR_SIZE},
};

OAL_DTCM_DATA_SECTION
oal_arm2dsp_mb_info_t g_st_oal_arm2dsp_mb_info[OAL_ARM2DSP_MB_MAX_NUM] =
{
    {(CONST_UINT16)OAL_HLS_L1_RD_MB3_ID, 0, (CONST_UINT32)HLS_L1_MB3_RD_SIZE, (UINT16_PTR)HLS_L1_MB3_RD_ADDR, (UINT16_PTR)((UINT32)HLS_L1_MB3_RD_ADDR + ((UINT32)MAILBOX_CTRL_HEAD_SIZE_IN_WORD<<1))},
    {(CONST_UINT16)OAL_HLS_L1_RD_MB4_ID, 0, (CONST_UINT32)HLS_L1_MB4_RD_SIZE, (UINT16_PTR)HLS_L1_MB4_RD_ADDR, (UINT16_PTR)((UINT32)HLS_L1_MB4_RD_ADDR + ((UINT32)MAILBOX_CTRL_HEAD_SIZE_IN_WORD<<1))},
    {(CONST_UINT16)OAL_HLS_L1_RD_MB5_ID, 0, (CONST_UINT32)HLS_L1_MB5_RD_SIZE, (UINT16_PTR)HLS_L1_MB5_RD_ADDR, (UINT16_PTR)((UINT32)HLS_L1_MB5_RD_ADDR + ((UINT32)MAILBOX_CTRL_HEAD_SIZE_IN_WORD<<1))}
};

OAL_SRAM_DATA_SECTION
oal_inter_dsp_mb_info_t g_st_oal_inter_dsp_mb_wr_info =
{
    CORE_ID_XC4500, 0, OAL_XC4500_X1643_WR_MB1_ID, OAL_X1643_XC4500_MB0_SEMA, (UINT16_PTR)XC4500_X1643_MB0_ADDR, (UINT16_PTR)((UINT32)XC4500_X1643_MB0_ADDR + ((UINT32)MAILBOX_CTRL_HEAD_SIZE_IN_WORD<<1)), XC4500_X1643_MB0_SIZE
};

OAL_SRAM_DATA_SECTION
oal_inter_dsp_mb_info_t g_st_oal_inter_dsp_mb_rd_info =
{
    CORE_ID_XC4500, 0, OAL_XC4500_X1643_RD_MB0_ID, OAL_INVALID_SEMA_ID, (UINT16_PTR)X1643_XC4500_MB0_ADDR, (UINT16_PTR)((UINT32)X1643_XC4500_MB0_ADDR + ((UINT32)MAILBOX_CTRL_HEAD_SIZE_IN_WORD<<1)), X1643_XC4500_MB0_SIZE
};

#endif

/*************************************************** END OF FILE ******************************************************/

