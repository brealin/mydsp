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
.IFNDEF _SYS_INIT_
.EQU _SYS_INIT_
.ENDIF

/***********************************************************************************************************************
* TYPE DEFINITION
***********************************************************************************************************************/
.DSECT OAL_DTCM_DATA_SYS_STACK_SECTION
.DSECT OAL_DTCM_DATA_IRQ_STACK_SECTION
.DSECT OAL_DATA_BOOT_STACK_SECTION
/***********************************************************************************************************************
* MACRO DEFINITION
***********************************************************************************************************************/
.EQU DSP_EXT_MEM_BOOT                       (0xA1B2)

/***********************************************************************************************************************
* GLOBAL VARIABLES DEFINITION
***********************************************************************************************************************/
.EXTERN _g_u32_error_sp_save
.EXTERN _g_u32p_error_stack_bottom
.EXTERN _g_st_context_save
.EXTERN _g_u32_reg_save_area
.EXTERN _g_u32_arx_save_area
.EXTERN _g_u32_retreg_save_area
.EXTERN _g_st_oal_mss_reg

.EXTERN _g_u16_oal_irq_index
.EXTERN _g_fp_oal_irq_vector_tbl
.EXTERN _g_u16_oal_irq_read_offset
.EXTERN _g_u16_oal_irq_write_offset

.EXTERN _g_u32_sp_save
.EXTERN _g_u32_retreg_save
.EXTERN _g_u32_retregi_save

.EXTERN _g_stp_ready_to_run_task
.EXTERN _g_stp_current_running_task
.EXTERN _g_u16_l1os_int_nesting_cnt

.EXTERN _g_u32p_boot_stack_bottom
.EXTERN _g_u32_boot_addr_flag
.EXTERN _g_u32_boot_flg
/***********************************************************************************************************************
* EXTERN FUNCTION DECLARATION
***********************************************************************************************************************/
.EXTERN .bss
.EXTERN _main
.EXTERN _ks_os_error_handler_ext_c
.EXTERN _ks_print_os_error_handler

.EXTERN __init_sys_stack
.EXTERN __init_irq_stack
.EXTERN __init_modx_regs
.EXTERN __init_ccr_reg
.EXTERN __pdma_mem_copy

.EXTERN __init_fic
.EXTERN __init_psvm_reg

.EXTERN __call_saved_store_w_frame
.EXTERN __call_saved_restore_w_frame
.EXTERN _ks_oal_assert
/***********************************************************************************************************************
* GLOBAL FUNCTION DECLARATION
***********************************************************************************************************************/
.GLOBAL __int0_handler
.GLOBAL __int1_handler
.GLOBAL __trap0_handler
.GLOBAL __trap1_handler
.GLOBAL __trap2_handler

.GLOBAL __init_cache

.GLOBAL __init_bss_section
.GLOBAL _ks_oal_bootload

.GLOBAL __mem_copy_asm
.GLOBAL __mem_set_asm

.IFDEF CORE_XC4500
.GLOBAL __ddma_mem_copy
.GLOBAL __mem_copy_asm_ext
.GLOBAL __mem_set_asm_ext
.ENDIF

.GLOBAL __ks_os_task_sch
.GLOBAL __ks_os_task_context_save
.GLOBAL __ks_os_task_context_restore
.GLOBAL __ks_os_task_switch_to_kernel

.GLOBAL __ks_os_delay
.GLOBAL __ks_os_error_handler
.GLOBAL __ks_os_context_save
.GLOBAL __ks_os_mss_reg_save
.GLOBAL __dcache_flush
/**************************************************** END OF FILE *****************************************************/

