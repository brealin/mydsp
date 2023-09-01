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
#define THIS_FILE_NAME_ID KS_IT_FILE_ID
    
/***********************************************************************************************************************
* INCLUDE FILES
***********************************************************************************************************************/
#include "ks_it.h"
#include "ks_oal.h"
#include "ks_hw_init.h"
#include "zzw_common.h"

#if defined(CORE_X1643)
extern volatile signed long int cp1_x1643_def_flag;
extern UINT8 g_zzw_slot_table[1024];
#else
extern volatile signed long int cp1_xc4500_def_flag;
#endif

#ifdef CORE_X1643
#define DSP_RTC_ADJ
#if 1//def DSP_RTC_ADJ
VOID dfe_rtc_adjust_fast(SINT32 rtc_value);
VOID dfe_rtc_adjust_fast2(SINT32 rtc_value);
#endif
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
extern VOID dfe_rtc_adjust_fast(SINT32 rtc_value);

/************************************************************************************************************************/
OAL_ITCM_CODE_SECTION
VOID ks_oal_mailbox_handle_cm4(KS_MAILBOX_PKG *recv_pkg)
{
	UINT32 u32_datal = recv_pkg->dataL;
	//UINT32 u32_msg_type = KS_SYS_GET_MSG_TYPE(recv_pkg->dataL);
	UINT32 u32_para = KS_SYS_GET_MSG_PARA(recv_pkg->dataL);

	switch (u32_datal)
	{
		case KS_SYS_STATUS_DEEP_SLEEP:
		{
			ks_slp_enter();
			break;
		}
		
		case KS_SYS_STATUS_NORMAL:
		{
			ks_slp_exit();
			#if 0
		    if(cp1_x1643_def_flag == 1)
			{
				ks_oal_mailbox_send_direct((UINT32)KS_CPU_CP1_XC4500, (UINT32)KS_CPU_CP1_X1643, (UINT32)OAL_DEEP_SLP_4500_EXIT, (UINT32)0);
			}
			else
			{
				ks_oal_mailbox_send_direct((UINT32)KS_CPU_CP0_XC4500, (UINT32)KS_CPU_CP0_X1643, (UINT32)OAL_DEEP_SLP_4500_EXIT, (UINT32)0);
			}
			#endif
			break;
		}
					
        default:
        {
            OAL_ASSERT(0, "not defined msg type");
            break;
        }
	}
}

OAL_ITCM_CODE_SECTION
VOID ks_oal_mailbox_handle_cp(KS_MAILBOX_PKG *recv_pkg)
{
	UINT32 u32_datal = recv_pkg->dataL;
	//UINT32 u32_msg_type = KS_SYS_GET_MSG_TYPE(recv_pkg->dataL);
	UINT32 u32_para = KS_SYS_GET_MSG_PARA(recv_pkg->dataL);
	VOLATILE_UINT32_PTR vu32_ptr;
	
	switch (u32_datal)
	{
		case KS_COER_STATUS_STANDBY:
		{
			//arm assert, dsp need to hold here
			g_u32_oal_assert_info = 0xDEADDEAD;
			#ifdef CORE_X1643
			if(cp1_x1643_def_flag == 1)
			{
				ks_oal_mailbox_send_direct((UINT32)KS_CPU_CP1_XC4500, (UINT32)KS_CPU_CP1_X1643, (UINT32)KS_COER_STATUS_STANDBY, 0);
			}
			else
			{
				ks_oal_mailbox_send_direct((UINT32)KS_CPU_CP0_XC4500, (UINT32)KS_CPU_CP0_X1643, (UINT32)KS_COER_STATUS_STANDBY, 0);				  
			}
			#endif
			__asm__("dint");
			#ifdef CORE_X1643
			*((VOLATILE_UINT32*)OAL_ASSERT_ADDR_1643) = 0xDEADDEAD;
			#else
			*((VOLATILE_UINT32*)OAL_ASSERT_ADDR_4500) = 0xDEADDEAD;
			#endif
			while(1)
			{
				KS_LOG(0, 0xD121 , (UINT16)0xDEAD);
				#ifdef CORE_X1643
				if(cp1_x1643_def_flag == 1)
				{			 
					vu32_ptr = (VOLATILE_UINT32_PTR)(HEARTBEAT_ADDR(KS_CPU_CP1_X1643) + 0x400000);
				}
				else
				{
					vu32_ptr = (VOLATILE_UINT32_PTR)(HEARTBEAT_ADDR(KS_CPU_CP0_X1643));
				}
				#else
				if(cp1_xc4500_def_flag == 1)
				{			 
					vu32_ptr = (VOLATILE_UINT32_PTR)(HEARTBEAT_ADDR(KS_CPU_CP1_XC4500) + 0x400000);
				}
				else
				{
					vu32_ptr = (VOLATILE_UINT32_PTR)(HEARTBEAT_ADDR(KS_CPU_CP0_XC4500));			
				}
				#endif
					
				ks_oal_delay((UINT32)20000);/*1ms*/
				*vu32_ptr += 1;
			};
		}
				
        default:
        {
            OAL_ASSERT(0, "not defined msg type");
            break;
        }
	}
}

OAL_ITCM_CODE_SECTION
VOID ks_oal_mailbox_handle_ap(KS_MAILBOX_PKG *recv_pkg)
{
	UINT32 u32_datal = recv_pkg->dataL;
	UINT32 u32_datah = recv_pkg->dataH;
	//UINT32 u32_msg_type = KS_SYS_GET_MSG_TYPE(recv_pkg->dataL);
	UINT32 u32_para = KS_SYS_GET_MSG_PARA(recv_pkg->dataL);
	
	switch (u32_datal)
	{
		#ifdef CORE_X1643
		case IQ_DUMP_STOP:
		{
			// ks_rxdfe_iq_dump_reset() is called in dma callback, after data report.
			g_st_iq_dump.start_en = OAL_FALSE;
			g_st_iq_dump.cmd = IQ_DUMP_STOP;
			break;
		}
		
		case IQ_DUMP_PLCP_DEC_FAILURE:
		{
			if(OAL_TRUE == g_st_iq_dump.start_en)	// if changing to another cmd, reset iq dump first.
			{
				ks_rxdfe_iq_dump_reset();
			}
			g_st_iq_dump.cmd = IQ_DUMP_PLCP_DEC_FAILURE;
			g_st_iq_dump.loop_en = OAL_TRUE;
			g_st_iq_dump.plcp_fail_cnt = READ_REG32(RX_DEBUG_INFO_ADDR + 0x18); // record current fail cnt
			ks_rxdfe_iq_dump_start(0,g_st_iq_dump.dmac_ch_id);
#if MIMO_2ANTS
			ks_rxdfe_iq_dump_start(1,g_st_iq_dump.dmac_ch_id_ant2);
#endif
			g_st_iq_dump.start_en = OAL_TRUE;
			break;			
		}
		case IQ_DUMP_DATA_DEC_FAILURE:
		{
			if(OAL_TRUE == g_st_iq_dump.start_en)	// if changing to another cmd, reset iq dump first.
			{
				ks_rxdfe_iq_dump_reset();
			}
			g_st_iq_dump.cmd = IQ_DUMP_DATA_DEC_FAILURE;
			g_st_iq_dump.loop_en = OAL_TRUE;			
			g_st_iq_dump.data_fail_cnt = READ_REG32(RX_DEBUG_INFO_ADDR + 0x20);	// record current fail cnt
			ks_rxdfe_iq_dump_start(0,g_st_iq_dump.dmac_ch_id);
#if MIMO_2ANTS
			ks_rxdfe_iq_dump_start(1,g_st_iq_dump.dmac_ch_id_ant2);
#endif		
			g_st_iq_dump.start_en = OAL_TRUE;
			break;
		}
		case IQ_DUMP_BUF_LOOP_EN_CMD:
		{			
			if(OAL_TRUE == g_st_iq_dump.start_en)	// if changing to another cmd, reset iq dump first.
			{
				ks_rxdfe_iq_dump_reset();
			}
			g_st_iq_dump.cmd = IQ_DUMP_BUF_LOOP_EN_CMD;
			g_st_iq_dump.loop_en = OAL_TRUE;
				
			ks_rxdfe_iq_dump_start(0,g_st_iq_dump.dmac_ch_id);
#if MIMO_2ANTS
			ks_rxdfe_iq_dump_start(1,g_st_iq_dump.dmac_ch_id_ant2);
#endif
			g_st_iq_dump.start_en = OAL_TRUE;
			break;
		}
		
		case IQ_DUMP_BUF_SINGLE:	
		{			
			if(OAL_TRUE == g_st_iq_dump.start_en)	// if changing to another cmd, reset iq dump first.
			{
				ks_rxdfe_iq_dump_reset();
			}		
			g_st_iq_dump.cmd = IQ_DUMP_BUF_SINGLE;
			g_st_iq_dump.loop_en = OAL_FALSE;					
			ks_rxdfe_iq_dump_start(0,g_st_iq_dump.dmac_ch_id);
#if MIMO_2ANTS
			ks_rxdfe_iq_dump_start(1,g_st_iq_dump.dmac_ch_id_ant2);
#endif			
			g_st_iq_dump.start_en = OAL_TRUE;
			break;
		}

		#endif
        case OAL_MAIL_HS_MSG_PHYCH0:
		{
			ks_oal_sema_put(AP_2_DSP_PHYCH_ID0_SEMA);
			break;
		}

        case OAL_MAIL_HS_MSG_PHYCH1:
		{
			ks_oal_sema_put(AP_2_DSP_PHYCH_ID1_SEMA);
			break;
		}
		
		case KS_SYS_STATUS_DEEP_SLEEP:
		{
			ks_slp_enter();
			break;
		}
		
		case KS_SYS_STATUS_NORMAL:
		{
			ks_slp_exit();
	#if 0
		    if(cp1_x1643_def_flag == 1)
			{
				ks_oal_mailbox_send_direct((UINT32)KS_CPU_CP1_XC4500, (UINT32)KS_CPU_CP1_X1643, (UINT32)OAL_DEEP_SLP_4500_EXIT, (UINT32)0);
			}
			else
			{
				ks_oal_mailbox_send_direct((UINT32)KS_CPU_CP0_XC4500, (UINT32)KS_CPU_CP0_X1643, (UINT32)OAL_DEEP_SLP_4500_EXIT, (UINT32)0);
			}
	#endif
			break;
		}
		case KS_SYS_STATUS_SHUTDOWN:
		{
			#ifdef CORE_X1643
			ks_oal_mem_set(g_zzw_slot_table, 0x0, sizeof(g_zzw_slot_table));
			#endif
			g_u32_boot_addr_flag = (UINT32)0;
			g_u32_shutdown_flg = OAL_TRUE;
			ks_slp_enter();
			break;
		}
		case AP_CRC_ERR_FLG_CMD:
		{
			KS_LOG(0, 0xD124 , (UINT16)(u32_datah));
			KS_LOG(0, 0xD124 , (UINT16)(u32_datah>> 16));
			WRITE_REG32(OAL_DSP_HANDSHAKE_ADDR, u32_datah);		
			break;
		}
        default:
        {
            OAL_ASSERT(0, "not defined msg type");
            break;
        }
	}
	
}

OAL_ITCM_CODE_SECTION
VOID ks_oal_mailbox_handle_dsp(KS_MAILBOX_PKG *recv_pkg)
{
	UINT32 u32_ret;
	UINT32 u32_datal = recv_pkg->dataL;

	switch (u32_datal)
	{
        case OAL_MAIL_MSG_ASSERT:
        {
            ks_oal_assert_handle();
            break;
        }
		#ifdef CORE_XC4500
        case OAL_1643_WAKE_4500:
		{
			ks_oal_drv_init_4500();
			break;
		}
		
        case OAL_DEEP_SLP_4500_ENTER:
		{
			g_u32_shutdown_flg = recv_pkg->dataH;
			ks_slp_enter();
			break;
		}

        case OAL_DEEP_SLP_4500_EXIT:
		{
			ks_slp_exit();
			break;
		}

		case OAL_DEEP_SLP_4500_ENTER_NOZZW:
		{
			g_u32_shutdown_flg = recv_pkg->dataH;
			ks_slp_enter_nozzw();
			break;
			
		}
		#endif				

        case OAL_MAIL_HS_MSG_PHYCH2:
		{
			ks_oal_sema_put(DSP_INTER_PHYCH_ID2_SEMA);
			break;
		}

        case OAL_MAIL_HS_MSG_PHYCH3:
		{
			ks_oal_sema_put(DSP_INTER_PHYCH_ID3_SEMA);
			break;
		}

		#ifdef CORE_X1643
		#if 1 //def DSP_RTC_ADJ
		case SCFDE_4500_SEND_RTC_1643:
		{
			dfe_rtc_adjust_fast(recv_pkg->dataH);
			break;
		}

		case SCFDE_4500_SEND_RTC2_1643:
		{
			dfe_rtc_adjust_fast2(recv_pkg->dataH);
			break;
		}		
		#endif
		#endif
        default:
        {
            OAL_ASSERT(0, "not defined msg type");
            break;
        }
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
VOID ks_oal_ipi_handle(VOID)
{
    OAL_IRQ_ID ipi_irq_id;
    eKS_CPU_ID src_cpu;

#ifdef CORE_X1643
#if defined(CP1_X1643)
        ipi_irq_id = OAL_IRQ_IPI_X1643_7;
        src_cpu = KS_CPU_CP1_XC4500;
#else
        ipi_irq_id = OAL_IRQ_IPI_X1643_5;
        src_cpu = KS_CPU_CP0_XC4500;
#endif
#else
#if defined(CP1_XC4500)
        ipi_irq_id = OAL_IRQ_IPI_XC4500_6;
        src_cpu = KS_CPU_CP1_X1643;
#else
        ipi_irq_id = OAL_IRQ_IPI_XC4500_4;
        src_cpu = KS_CPU_CP0_X1643;
#endif
#endif
    ks_ipi_clear_intr(src_cpu);
    __asm__("dint");
    *(unsigned int *)(ICTL0_IRQ_INTMASK_L) &= ~(unsigned int)(0x1 << ipi_irq_id); //unmask ictl int
    __asm__("eint");

}

/*************************************************** END OF FILE ******************************************************/

