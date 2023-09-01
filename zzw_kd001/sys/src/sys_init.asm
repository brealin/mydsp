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
.IFDEF THIS_FILE_NAME_ID
.PURGE THIS_FILE_NAME_ID
.ENDIF
.EQU THIS_FILE_NAME_ID    (2) /* SYS_INIT_FILE_ID */
.EQU PROGRAM_ZERO_ADDR_PROTECT    (0x1000) /* PROGRAM_ZERO_ADDR_PROTECT_ERROR ID */
/***********************************************************************************************************************
* INCLUDE FILES
***********************************************************************************************************************/
.INCLUDE "sys_init.inc"

/***********************************************************************************************************************
* FUNCTION
*
*   __start
*
* DESCRIPTION
*
*   __start£¬which must be located at 0x0000
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
.CSECT OAL_ITCM_CODE_INTVECT_SECTION
__start:
    /* reset jumps to __init */
    brr {t} __init
    brr {t} __dsp_boot

    /* int0 jumps to __int0_handler */
    .ORG 0x080
    brr {t} __int0_handler

    /* int1 jumps to __int1_handler */
    .ORG 0x0C0
    brr {t} __int1_handler
    
    /* trap0 jumps to __trap0_handler */
    .ORG 0x200
    brr {t} __trap0_handler

    /* trap1 jumps to __trap1_handler */
    .ORG 0x240
    brr {t} __trap1_handler

    /* trap2 jumps to __trap2_handler */
    .ORG 0x280
    brr {t} __trap2_handler

/***********************************************************************************************************************
*
* FUNCTION
*
*   __dsp_boot
*
* DESCRIPTION
*
*   DSP BOOT from ext code
*
* NOTE
*
*   <the limitations to use this function or other comments>
*
* PROTOTYPE
*
*   __dsp_boot
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
***********************************************************************************************************************/
.CSECT OAL_DRAM_BOOT_CODE_SECTION
__dsp_boot:
.func_start __dsp_boot

    mov #0x1724FFE0, a0
    mov #0x87654322, a1
    nop
    nop
    mov a0, r0
    nop
    nop
    st{dw} a1, (r0)

	
    mov {u} #SIZEOF(OAL_DATA_BOOT_STACK_SECTION) - 4, a1
    mov #OAL_DATA_BOOT_STACK_SECTION.0, a2
    add a1, a2
    nop
    nop
    nop
    mov a2, sp

    ;callr {t} __init_boot_stack
    nop
    nop
    nop
    /* init modx regs */
    callr {t} __init_modx_regs

.IF CORE_X1643
    /* init ccr reg */
    callr {t} __init_ccr_reg

.ELIF CORE_XC4500
    /* init fic */
    ;;callr {t} __init_fic

    /* init psvm reg */
    callr {t} __init_psvm_reg
.ENDIF

	.IF CORE_X1643
	.IF CP1_X1643
	mov #1, a0
	.ELSE
	mov #0, a0
	.ENDIF
	.ELSE
	.IF CP1_XC4500
	mov #1, a0
	.ELSE
	mov #0, a0
	.ENDIF
	.ENDIF
	
    callr {t} _ks_oal_bootload
           
    brr __init

.func_end __dsp_boot

/***********************************************************************************************************************
* FUNCTION
*
*   __init
*
* DESCRIPTION
*
*   init system
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
.CSECT OAL_ITCM_CODE_SECTION
__init:
.func_start __init

    ld {dw,u} [#_g_u32_boot_flg],  a0
    nop
    cmp {dw,eq} a0, #1, prg0
    nop
    nop
    mov #THIS_FILE_NAME_ID, a0
    mov #PROGRAM_ZERO_ADDR_PROTECT , a1     
    call _ks_oal_assert, ?prg0
    nop
    nop
    
    mov {u} #SIZEOF(OAL_DTCM_DATA_SYS_STACK_SECTION) - 4, a1
    mov #OAL_DTCM_DATA_SYS_STACK_SECTION.0, a2
    add a1, a2
    nop
    nop
    nop
    mov a2, sp

    callr {t} __init_sys_stack

    callr {t} __init_irq_stack
    
    ld {dw,u} [#_g_u32_boot_addr_flag],  a0
    nop
    cmp {dw,eq} a0, #DSP_EXT_MEM_BOOT, prg0
    nop
    nop
    brr {t} DO_NOT_REF_INT, ?prg0
    nop
    nop
    /* init modx regs */
    callr {t} __init_modx_regs

.IF CORE_X1643
    /* init ccr reg */
    callr {t} __init_ccr_reg

.ELIF CORE_XC4500
    /* init fic */
    ;;callr {t} __init_fic

    /* init psvm reg */
    callr {t} __init_psvm_reg
.ENDIF

DO_NOT_REF_INT:
    /* init icache and dcache */
    callr {t} __init_cache

    /* call main */
    callr {t} _main

    /* exception, restart dsp */
    brr {t} __start

.func_end __init
/**************************************************** END OF FILE *****************************************************/

